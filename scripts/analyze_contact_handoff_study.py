"""Verify and report every episode of the frozen handoff investigation."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot
from freeze_contact_handoff_study import ROOT, MISSION, read, sha
from analyze_dynamic_contact_study import old_fields_equal, replay_equal


def replay_prefix_equal(first, second):
    """Exact prefix comparison respecting the replay's packed observation schema.

    sensed_points is concatenate(frame_points), indexed by sensed_counts; its
    leading dimension is a point count, not a frame count. No tolerance changes.
    """
    frame_keys={'times','phase','agent_positions','push_flags','contact_flags',
                'efforts','min_distance','map_coverage','sensed_counts'}
    cargo_series={'centers','angles','strict_coverage','contacts','penetration','speed','progress_estimate'}
    with np.load(first,allow_pickle=False) as a, np.load(second,allow_pickle=False) as b:
        if set(a.files)!=set(b.files): return {'schema':False}
        n,m=len(a['times']),len(b['times'])
        if m<n: return {'frame_count':False}
        ac,bc=a['sensed_counts'],b['sensed_counts']
        if len(ac)!=n or len(bc)!=m or np.any(ac<0) or np.any(bc<0):
            raise ValueError('Malformed observation count sequence')
        equal={}
        for key in a.files:
            x,y=a[key],b[key]
            if key=='sensed_points':
                if len(x)!=int(ac.sum()) or len(y)!=int(bc.sum()):
                    raise ValueError('Packed observation length differs from count sum')
                y=y[:int(bc[:n].sum())]
            elif key in frame_keys or (key.startswith('cargo/') and key.rsplit('/',1)[-1] in cargo_series):
                if not x.ndim or not y.ndim or len(x)!=n or len(y)!=m:
                    raise ValueError(f'Malformed frame array: {key}')
                y=y[:n]
            equal[key]=bool(np.array_equal(x,y,equal_nan=True) if x.dtype.kind in 'fci' else np.array_equal(x,y))
    return equal


def analyze(stage):
    batch = ROOT / f'runs/research/{MISSION}/{stage}'
    out = ROOT / f'artifacts/research/{MISSION}/{stage}'
    plan = read(ROOT / f'experiments/ccta2027/contact_handoff_v1_{stage}.json')
    manifest, summary = read(batch / 'manifest.json'), read(batch / 'summary.json')
    if summary.get('source_integrity') != 'PASS' or summary['completed_denominator'] != len(plan['cases']):
        raise RuntimeError('Incomplete batch/source mismatch; retain outputs without claiming completion')
    verify_source_snapshot(batch, manifest['source_files'])
    assert encode(manifest['source_files']) == plan['frozen_source_sha256']
    for name, digest in plan['historical_inputs'].items():
        if sha(ROOT / name) != digest:
            raise RuntimeError(f'Changed historical input: {name}')
    out.mkdir(parents=True, exist_ok=True)
    rows, compat, events, trajectories = [], {}, {}, {}
    confirmation_records, abort_records = [], []
    inputs = dict(plan['historical_inputs'])
    for case in plan['cases']:
        folder = batch / case['case_id']
        receipt = read(folder / 'case_result.json')
        assert receipt['identity'] == {**manifest['identity'], 'case_sha256':encode(case)}
        result = cached_result(folder, receipt['identity'])
        for name, digest in receipt['products'].items():
            inputs[(folder/name).relative_to(ROOT).as_posix()] = digest
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()] = sha(folder/'case_result.json')
        row = dict(condition=case['condition_id'], arm=case['arm'], seed=case['seed'],
                   execution_status=result['execution_status'], completed=bool(result.get('task_completed')),
                   g500=bool(result.get('g500_pass')), settled=bool(result.get('settled')),
                   endpoint_error_m=result.get('euclidean_endpoint_error'), wall_seconds=result['wall_seconds'])
        if result['execution_status'] != 'COMPLETED':
            row['outcome'] = 'EXECUTION_ERROR'
            rows.append(row)
            continue
        steps = read(folder / 'paired_metrics.json')['steps']
        gate = read(folder / 'summary.json')['cargoes']['cargo_0']['g500']
        contract = ClosedLoopContract(**gate['gates'])
        verdict = contract.evaluate(gate['metrics'])
        assert verdict.success == gate['success'] and verdict.reasons == gate['failure_reasons']
        m = gate['metrics']
        hard = []
        if m['min_inter_agent_distance'] < m['d_min'] - contract.d_min_tolerance: hard.append('separation')
        if m['min_signed_clearance'] < 0: hard.append('inside_object')
        if m['max_penetration'] > m['penetration_budget']: hard.append('penetration_budget')
        if m['solver_fallbacks'] or m['solver_infeasible']: hard.append('solver_failure')
        diagnostics = [d for s in steps for d in s['diagnostics']]
        reasons = Counter(d.get('handoff_event', 'not_enabled') for d in diagnostics)
        rollbacks = Counter(d.get('handoff_rollback_reason', '') for d in diagnostics if d.get('handoff_rollback_reason'))
        modes = Counter(d.get('handoff_state', 'not_enabled') for d in diagnostics)
        v1reasons = Counter(d.get('contact_redeploy_reason', 'not_enabled') for d in diagnostics)
        for index, step in enumerate(steps):
            # Control row k uses pre-physics state k-1. The previous physics row
            # is independent evaluation, never fed back into the handoff policy.
            prior = steps[index-1] if index else None
            forces = (dict(zip(prior['contact_ids'], prior['contact_forces'])) if prior else {})
            for d in step['diagnostics']:
                if d.get('handoff_rollback_reason') == 'patch_or_estimate_lost':
                    normal=np.asarray(d['contact_selected_normal'],float)
                    direction=np.asarray(d['contact_desired_direction'],float)
                    abort_records.append(dict(case=case['case_id'],frame=step['frame'],actor=d['agent_id'],
                        normal=normal.tolist(),desired_direction=direction.tolist(),
                        local_proxy=d['handoff_contact_proxy'],gap_m=d['handoff_gap'],
                        recorded_direction_fails_geometry_gate=bool(-normal[0]<0 or -normal@direction<=case['config']['controller'].get('push_share_floor',.15)),
                        prior_physical_contact=d['agent_id'] in forces))
                if d.get('handoff_event') not in ('locally_confirmed','trial_unload','release_complete'):
                    continue
                actor, partner = d['agent_id'],d['handoff_partner']
                confirmation_records.append(dict(case=case['case_id'],frame=step['frame'],
                    event=d['handoff_event'],actor=actor,partner=partner,
                    actor_prior_physical_contact=actor in forces,
                    partner_prior_physical_contact=partner in forces,
                    actor_prior_force_N=float(np.linalg.norm(forces.get(actor,[0.,0.]))),
                    partner_prior_force_N=float(np.linalg.norm(forces.get(partner,[0.,0.]))),
                    local_proxy=d['handoff_contact_proxy'],local_forward_velocity=d['handoff_local_forward_velocity']))
        seconds = result['termination']['frames_run'] * case['config']['dt']
        tail=steps[-max(1,round(5/case['config']['dt'])):]
        outcome = ('COMPLETED' if row['completed'] else 'SETTLED_OUTSIDE_2D_TOLERANCE' if row['settled']
                   else 'RIGHT_CENSORED_600S' if stage == 'extended' else 'BUDGET_EXHAUSTED_150S')
        row.update(frames=result['termination']['frames_run'], seconds=seconds,
                   completion_seconds=seconds if row['completed'] else None, watchdog=result['watchdog'],
                   outcome=outcome, final_phase=steps[-1]['phase'], true_progress_m=steps[-1]['true_progress'],
                   estimated_progress_m=result['final_estimated_progress'], max_cross_track_m=result['max_cross_track'],
                   hard_safety='|'.join(hard), min_clearance_m=m['min_signed_clearance'],
                   min_separation_m=m['min_inter_agent_distance'], max_penetration_m=m['max_penetration'],
                   barrier_scalings=m['barrier_scalings'], solver_fallbacks=m['solver_fallbacks'],
                   tail_window_seconds=len(tail)*case['config']['dt'],
                   tail_mean_positive_contacts=float(np.mean([s['positive_contacts'] for s in tail])),
                   tail_mean_push_flags=float(np.mean([sum(d['push_side'] for d in s['diagnostics']) for s in tail])),
                   tail_mean_forward_contact_force_N=float(np.mean([s['actual_net_force'][0] for s in tail])),
                   tail_mean_true_speed_m_s=float(np.mean([np.linalg.norm(s['true_velocity']) for s in tail])),
                   g500_reasons=' | '.join(verdict.reasons),
                   handoff_proposals=reasons['proposed'], handoff_confirmations=reasons['locally_confirmed'],
                   handoff_releases=reasons['release_complete'],
                   handoff_rollbacks=sum(rollbacks.values()),
                   relocation_starts=v1reasons['committed_new'])
        rows.append(row)
        events[case['case_id']] = {'handoff_events':dict(reasons), 'states_agent_frames':dict(modes),
                                   'rollback_reasons':dict(rollbacks),
                                   'v1_reasons_agent_frames':dict(v1reasons), 'phase_events':result['events']}
        with np.load(folder / 'replay.npz', allow_pickle=False) as r:
            trajectories[case['case_id']] = {'xy':r['cargo/cargo_0/centers']-r['task/cargo_0/start'],
                                            'goal':r['task/cargo_0/goal_point']-r['task/cargo_0/start']}
        reference = None
        if stage == 'ablation' and case['arm'] in ('feedback_fix', 'joint_v1'):
            oldarm = 'dynamic_contact' if case['arm'] == 'joint_v1' else 'feedback_fix'
            reference = ROOT / f"runs/research/2026-11-01-dynamic-contact-v1/run/{case['condition_id']}_{oldarm}"
        elif stage == 'main' and case['arm'] in ('legacy', 'spacing'):
            oldcase = case['factors']['source_case']
            if case['arm'] == 'spacing': oldcase = oldcase.replace('boundary_cvt', 'local_spacing')
            reference = ROOT / 'runs/research/2026-10-31-multishape-census-v1/run' / oldcase
        if reference is not None:
            old_receipt=read(reference/'case_result.json')
            cached_result(reference,old_receipt['identity'])
            for name,digest in old_receipt['products'].items():
                inputs[(reference/name).relative_to(ROOT).as_posix()]=digest
            exact, arrays = replay_equal(reference/'replay.npz', folder/'replay.npz')
            compat[case['case_id']] = {'replay_exact':exact, 'arrays':arrays,
                                     'old_step_fields_exact':old_fields_equal(read(reference/'paired_metrics.json')['steps'],steps)}
        if stage == 'extended':
            reference = ROOT / f"runs/research/{MISSION}/main/{case['case_id']}/replay.npz"
            equal = replay_prefix_equal(reference,folder/'replay.npz')
            compat[case['case_id']] = {'prefix_exact':all(equal.values()), 'arrays':equal}
        del steps, diagnostics
    counts = {a: {'completed':sum(r['completed'] for r in rows if r['arm']==a),
                  'denominator':sum(r['arm']==a for r in rows),
                  'g500':sum(r['g500'] for r in rows if r['arm']==a),
                  'errors':sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==a),
                  'censored':sum(r.get('watchdog',False) for r in rows if r['arm']==a)} for a in plan['arms']}
    compatible = all((v.get('replay_exact', True) and v.get('old_step_fields_exact', True)
                      and v.get('prefix_exact', True)) for v in compat.values())
    lookup = {(r['condition'],r['arm']):r for r in rows}
    names = list(dict.fromkeys(c['condition_id'] for c in plan['cases']))
    comparisons = {}
    if stage != 'ablation':
        for arm in plan['arms'][:-1]:
            gained,lost,unsafe = [],[],[]
            for name in names:
                a,b=lookup[name,arm],lookup[name,'handoff']
                if b['completed'] and not a['completed']: gained.append(name)
                if a['completed'] and not b['completed']: lost.append(name)
                if b.get('hard_safety') and not a.get('hard_safety'): unsafe.append(name)
            comparisons[arm] = dict(gained=gained,lost=lost,net=len(gained)-len(lost),new_hard_safety=unsafe)
    gate = (stage=='main' and compatible and counts['handoff']['errors']==0 and
            all(c['net']>=2 and len(c['lost'])<=1 and not c['new_hard_safety'] for c in comparisons.values()))
    report = dict(stage=stage, counts=counts, comparisons=comparisons, development_gate_pass=gate,
                  compatibility_pass=compatible, compatibility=compat, all_episodes=rows, events=events,
                  source_integrity=summary['source_integrity'], historical_integrity='PASS',
                  scope='Fixed development seed 2; paired conditions, no reliability inference',
                  confirmation_limit='Local gap/command/motion proxy, not physical contact/force or individual causal contribution proof')
    report['confirmation_evaluation_only'] = confirmation_records
    report['patch_abort_evaluation_only'] = abort_records
    dump(out/'analysis.json',report)
    dump(out/'provenance.json',dict(inputs=inputs, execution_source_sha256=plan['frozen_source_sha256'],
                                  analysis_script_sha256=sha(Path(__file__)),
                                  analysis_helpers_sha256={name:sha(ROOT/'scripts'/name) for name in
                                      ('analyze_dynamic_contact_study.py','freeze_contact_handoff_study.py')}))
    with (out/'all_episodes.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader();w.writerows(rows)
    lines=['| Condition | '+' | '.join(plan['arms'])+' |', '|---|'+'---|'*len(plan['arms'])]
    for name in names:
        cells=[]
        for arm in plan['arms']:
            r=lookup[name,arm]
            cells.append(f"{'PASS' if r['completed'] else r['outcome']} / {r['endpoint_error_m']:.4f} m"
                         if r['endpoint_error_m'] is not None else 'EXECUTION_ERROR / unavailable')
        lines.append('| '+name+' | '+' | '.join(cells)+' |')
    (out/'capability_table.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    plot(rows,names,plan['arms'],trajectories,out,stage)
    print(json.dumps({k:report[k] for k in ('counts','comparisons','compatibility_pass','development_gate_pass')},indent=2))


def plot(rows,names,arms,trajectories,out,stage):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
    colors=('#0072B2','#E69F00','#009E73','#CC79A7')
    # Method identity, rather than column position, determines style across budgets.
    style={'legacy':(colors[0],'o','-'),'spacing':(colors[1],'s','--'),
           'handoff':(colors[2],'^',':'),'feedback_fix':(colors[0],'o','-'),
           'allocation_only':(colors[1],'s','--'),'relocation_only':(colors[2],'^',':'),
           'joint_v1':(colors[3],'D','-.')}
    fig,ax=plt.subplots(figsize=(7.15,3.5))
    for j,arm in enumerate(arms):
        color,marker,_=style[arm]
        group={r['condition']:r for r in rows if r['arm']==arm}
        for i,name in enumerate(names):
            r=group[name]
            if r['endpoint_error_m'] is None: continue
            ax.scatter(i+(j-(len(arms)-1)/2)*.17,r['endpoint_error_m'],s=25,marker=marker,
                       facecolors=color if r['completed'] else 'none',edgecolors=color,label=arm if i==0 else None)
    ax.axhline(.12,color='0.4',ls='--',lw=.8,label='Original 0.12 m tolerance')
    ax.set_xticks(range(len(names)),[n.replace('_','\n') for n in names],fontsize=7)
    ax.set_ylabel('Final 2D endpoint error (m)');ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2)
    ax.legend(fontsize=7,ncol=2)
    ax.set_title(f'{stage}: fixed seed 2, {len(names)} paired conditions per arm\nFilled: completed; open: not completed at observed end')
    fig.tight_layout()
    for ext in ('pdf','svg','png'): fig.savefig(out/f'endpoint_comparison.{ext}',dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots((len(names)+2)//3,3,figsize=(7.15,2.2*((len(names)+2)//3)),squeeze=False)
    for ax,name in zip(axes.flat,names):
        goal=None
        for j,arm in enumerate(arms):
            color,_,line=style[arm]
            data=trajectories.get(f'{name}_{arm}')
            if data is None: continue
            xy=data['xy'];goal=data['goal']
            ax.plot(xy[:,0],xy[:,1],line,color=color,lw=1.,label=arm)
            ax.plot(*xy[-1],marker='x',color=color,ms=4)
        if goal is not None: ax.add_patch(Circle(goal,.12,fill=False,color='0.5',ls='--',lw=.7))
        else: ax.text(.5,.5,'No executed trajectory',transform=ax.transAxes,ha='center')
        ax.set_aspect('equal',adjustable='datalim');ax.set_title(name,fontsize=8)
        ax.set_xlabel('Along goal (m)');ax.set_ylabel('Lateral (m)');ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=6)
    fig.tight_layout()
    for ext in ('pdf','svg','png'): fig.savefig(out/f'complete_trajectories.{ext}',dpi=180)
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=('ablation','main','extended'))
    analyze(parser.parse_args().stage)
