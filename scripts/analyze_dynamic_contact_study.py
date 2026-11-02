"""Audit all frozen dynamic-contact episodes and retain adverse outcomes."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-01-dynamic-contact-v1'
ARMS = ('legacy', 'feedback_fix', 'dynamic_contact')
LABELS = ('Original', 'Feedback repair', 'Dynamic contact')


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def old_fields_equal(old, new):
    """Strict equality on every historical key; only new diagnostic keys ignored."""
    if isinstance(old, dict):
        return isinstance(new, dict) and all(k in new and old_fields_equal(v, new[k]) for k, v in old.items())
    if isinstance(old, list):
        return isinstance(new, list) and len(old) == len(new) and all(old_fields_equal(a, b) for a,b in zip(old,new))
    return old == new


def replay_equal(first, second):
    with np.load(first, allow_pickle=False) as a, np.load(second, allow_pickle=False) as b:
        if set(a.files) != set(b.files):
            return False, {}
        equal = {}
        for key in a.files:
            equal[key] = bool(np.array_equal(a[key], b[key], equal_nan=True)
                              if a[key].dtype.kind in 'fci' else np.array_equal(a[key], b[key]))
    return all(equal.values()), equal


def analyze(out):
    out = out.resolve()
    if not out.is_relative_to(ROOT / 'artifacts/research' / MISSION):
        raise ValueError('Analysis writes only inside its research artifact directory')
    batch = ROOT / 'runs/research' / MISSION / 'run'
    plan = read(ROOT / 'experiments/ccta2027/dynamic_contact_v1.json')
    manifest = read(batch / 'manifest.json')
    summary = read(batch / 'summary.json')
    if summary.get('source_integrity') != 'PASS' or summary['completed_denominator'] != 27:
        raise RuntimeError('Study is incomplete or its execution source changed; preserve partial outputs')
    verify_source_snapshot(batch, manifest['source_files'])
    if encode(manifest['source_files']) != plan['frozen_source_sha256']:
        raise RuntimeError('Frozen source mismatch')
    for name, digest in plan['historical_inputs'].items():
        if sha(ROOT / name) != digest:
            raise RuntimeError(f'Historical input changed: {name}')
    out.mkdir(parents=True, exist_ok=True)
    rows, compat, events, trajectories = [], {}, {}, {}
    input_hashes = dict(plan['historical_inputs'])
    for case in plan['cases']:
        folder = batch / case['case_id']
        receipt = read(folder / 'case_result.json')
        if receipt['identity'] != {**manifest['identity'], 'case_sha256':encode(case)}:
            raise RuntimeError('Case identity mismatch')
        result = cached_result(folder, receipt['identity'])
        input_hashes[(folder / 'case_result.json').relative_to(ROOT).as_posix()] = sha(folder / 'case_result.json')
        for name, digest in receipt['products'].items():
            input_hashes[(folder / name).relative_to(ROOT).as_posix()] = digest
        row = {'condition':case['condition_id'], 'arm':case['arm'], 'seed':case['seed'],
               'execution_status':result['execution_status'], 'completed':bool(result.get('task_completed')),
               'g500':bool(result.get('g500_pass')), 'settled':bool(result.get('settled')),
               'endpoint_error_m':result.get('euclidean_endpoint_error'),
               'wall_seconds':result['wall_seconds'], 'failure':result.get('error', '')}
        if result['execution_status'] != 'COMPLETED':
            rows.append(row)
            continue
        raw_summary = read(folder / 'summary.json')
        cargo = raw_summary['cargoes']['cargo_0']
        gate = cargo['g500']
        contract = ClosedLoopContract(**gate['gates'])
        verdict = contract.evaluate(gate['metrics'])
        if verdict.success != gate['success'] or verdict.reasons != gate['failure_reasons']:
            raise RuntimeError('Frozen evaluator mismatch')
        m = gate['metrics']
        hard = []
        if m['min_inter_agent_distance'] < m['d_min'] - contract.d_min_tolerance:
            hard.append('separation')
        if m['min_signed_clearance'] < 0:
            hard.append('inside_object')
        if m['max_penetration'] > m['penetration_budget']:
            hard.append('penetration_budget')
        if m['solver_fallbacks'] or m['solver_infeasible']:
            hard.append('solver_failure')
        steps = read(folder / 'paired_metrics.json')['steps']
        diagnostics = [d for s in steps for d in s['diagnostics']]
        role_counts = Counter(d['contact_role'] for d in diagnostics)
        reason_counts = Counter(d['contact_redeploy_reason'] for d in diagnostics)
        attempts = sum(max(d['contact_redeploy_attempts'] for d in diagnostics if d['agent_id'] == a)
                       for a in {d['agent_id'] for d in diagnostics})
        tail = steps[-max(1,round(5/case['config']['dt'])):]
        tail_push = [sum(d['push_side'] for d in s['diagnostics']) for s in tail]
        tail_force = np.asarray([s['actual_net_force'] for s in tail])
        tail_speed = [float(np.linalg.norm(s['true_velocity'])) for s in tail]
        cone_values = [d['contact_cone_residual'] for d in diagnostics
                       if d['contact_role'] in ('press','yield','redeploy') and d['contact_cone_residual'] is not None]
        # Exploratory, after-freeze audit of the existing anti-windup policy.
        # A magnitude-only QP modification is not necessarily a blocked press.
        blocked_press = nonoutward = no_previous_press = 0
        for before, now in zip(steps,steps[1:]):
            for index, d in enumerate(now['diagnostics']):
                if not (d['push_side'] and d['safety_feedback_blocked']):
                    continue
                blocked_press += 1
                prior = before['diagnostics'][index]
                normal = prior['contact_selected_normal']
                if not prior['push_side'] or None in normal:
                    no_previous_press += 1
                    continue
                difference = np.asarray(before['commands'][index])-np.asarray(prior['nominal_command'])
                nonoutward += int(float(difference @ np.asarray(normal)) <= 1e-3)
        seconds = result['termination']['frames_run'] * case['config']['dt']
        row.update(seconds=seconds, completion_seconds=seconds if row['completed'] else None,
                   watchdog=result['watchdog'], frames=result['termination']['frames_run'],
                   contact_ready_frame=result['contact_ready_frame'],
                   first_physical_contact_frame=next((s['frame'] for s in steps if s['contacts'] > 0), None),
                   first_positive_progress_frame=next((s['frame'] for s in steps if s['true_progress'] > 1e-9), None),
                   final_phase=steps[-1]['phase'], final_true_progress_m=steps[-1]['true_progress'],
                   final_estimated_progress_m=result['final_estimated_progress'],
                   max_cross_track_m=result['max_cross_track'], rotation_deg=result['rotation_deg'],
                   min_robot_distance_m=m['min_inter_agent_distance'], min_clearance_m=m['min_signed_clearance'],
                   max_penetration_m=m['max_penetration'], solver_fallbacks=m['solver_fallbacks'],
                   solver_infeasible=m['solver_infeasible'], barrier_scalings=m['barrier_scalings'],
                   hard_safety_violations='|'.join(hard), redeploy_attempts=attempts,
                   redeploy_arrivals=reason_counts['arrived'], redeploy_expired=reason_counts['lease_expired'],
                   redeploy_occupied=reason_counts['occupied'], redeploy_map_lost=reason_counts['map_lost'],
                   redeploy_without_named_end=attempts-sum(reason_counts[k] for k in ('arrived','lease_expired','occupied','map_lost')),
                   redeploy_agent_frames=role_counts['redeploy'],
                   no_target_agent_frames=reason_counts['no_local_target'],
                   tail_window_frames=len(tail), tail_mean_contacts=float(np.mean([s['contacts'] for s in tail])),
                   tail_mean_push_flags=float(np.mean(tail_push)),
                   tail_max_contact_net_force_N=float(np.linalg.norm(tail_force,axis=1).max()),
                   tail_mean_speed_m_s=float(np.mean(tail_speed)),
                   local_cone_residual_mean=float(np.mean(cone_values)) if cone_values else None,
                   local_cone_below_005_fraction=float(np.mean(np.asarray(cone_values)<=.05)) if cone_values else None,
                   safety_feedback_blocked_agent_frames=sum(d['safety_feedback_blocked'] for d in diagnostics),
                   local_holding_agent_frames=sum(d['local_holding'] for d in diagnostics),
                   exploratory_blocked_press_decisions=blocked_press,
                   exploratory_no_previous_press=no_previous_press,
                   exploratory_previous_modification_not_outward=nonoutward,
                   phase_brake_reason=result['phase_response'].get('brake_reason'),
                   phase_contact_recovery_failed=result['phase_response'].get('contact_recovery_failed'),
                   phase_estimate_stopped=result['phase_response'].get('estimate_stopped'),
                   g500_reasons=' | '.join(verdict.reasons),
                   quality_warnings=' | '.join(verdict.metrics['quality_reasons']))
        row['failure'] = ('COMPLETED' if row['completed'] else 'SETTLED_OUTSIDE_2D_TOLERANCE'
                          if row['settled'] else 'WATCHDOG_' + row['final_phase'])
        with np.load(folder / 'replay.npz', allow_pickle=False) as replay:
            centers = replay['cargo/cargo_0/centers']
            start = replay['task/cargo_0/start']
            goal = replay['task/cargo_0/goal_point']
            trajectories[case['case_id']] = {'xy':centers-start, 'goal':goal-start}
        # Log alignment: diagnostics in row k+1 are from control at state k.
        # Do not compare the local pre-control estimate to post-physics truth.
        mismatch = checked = 0
        for s in steps:
            k = s['frame'] - 1
            true_lateral = float(centers[k, 1] - start[1])  # all frozen directions +x
            if abs(true_lateral) <= .05:
                continue
            for d in s['diagnostics']:
                lateral = d.get('contact_lateral_estimate', [None,None])[1]
                if lateral is not None and d['contact_role'] in ('press', 'yield', 'redeploy'):
                    checked += 1
                    mismatch += int(float(lateral)*true_lateral < 0)
        row.update(lateral_sign_disagreement_agent_frames=mismatch, lateral_sign_checked_agent_frames=checked)
        rows.append(row)
        events[case['case_id']] = {'roles_agent_frames':dict(role_counts), 'reasons_agent_frames':dict(reason_counts),
                                  'events':result['events']}
        if case['arm'] == 'legacy':
            original = ROOT / 'runs/research/2026-10-31-multishape-census-v1/run' / case['factors']['source_case']
            exact, arrays = replay_equal(original / 'replay.npz', folder / 'replay.npz')
            old_steps = read(original / 'paired_metrics.json')['steps']
            compat[case['condition_id']] = {'replay_exact':exact, 'arrays':arrays,
                                           'old_step_fields_exact':old_fields_equal(old_steps, steps)}
        del steps, diagnostics
    counts = {arm:{'completed':sum(r['completed'] for r in rows if r['arm']==arm), 'denominator':9,
                   'errors':sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==arm),
                   'watchdogs':sum(r.get('watchdog',False) for r in rows if r['arm']==arm),
                   'g500':sum(r['g500'] for r in rows if r['arm']==arm)} for arm in ARMS}
    lookup = {(r['condition'],r['arm']):r for r in rows}
    comparisons = {}
    for baseline in ARMS[:2]:
        gained,lost,new_unsafe = [],[],[]
        for c in plan['conditions']:
            name = c['condition']
            a,b = lookup[name,baseline],lookup[name,'dynamic_contact']
            if b['completed'] and not a['completed']: gained.append(name)
            if a['completed'] and not b['completed']: lost.append(name)
            if b.get('hard_safety_violations') and not a.get('hard_safety_violations'): new_unsafe.append(name)
        comparisons[baseline] = {'gained':gained,'lost':lost,'net':len(gained)-len(lost),'new_recorded_hard_safety':new_unsafe}
    compatible = all(x['replay_exact'] and x['old_step_fields_exact'] for x in compat.values()) and len(compat)==9
    gate = (compatible and counts['dynamic_contact']['errors']==0
            and all(c['net'] >= 2 and len(c['lost']) <= 1 and not c['new_recorded_hard_safety'] for c in comparisons.values()))
    report = {'scope':plan['scope'], 'counts':counts,'comparisons':comparisons,
              'development_gate_pass':gate,'legacy_compatibility_pass':compatible,'legacy_compatibility':compat,
              'all_27_episodes':rows,'role_events':events,'historical_integrity':'PASS',
              'execution_source_integrity':summary['source_integrity'], 'analysis_script_sha256':sha(Path(__file__)),
              'timing_alignment':'control diagnostics at frame k+1 compared with replay physical state k; all directions +x',
              'denominator':'9 paired conditions per arm, one development seed; no independent success-rate inference',
              'evidence_limit':'normal cone ignores force capacity/torque and unobserved contacts; events are agent-frames, not independent experimental samples'}
    report['exploratory_analysis'] = 'Anti-windup direction decomposition added after batch freeze; hypothesis generation only, no parameter or execution change'
    dump(out / 'analysis.json', report)
    dump(out / 'provenance.json', {'inputs':input_hashes, 'analysis_script_sha256':sha(Path(__file__)),
                                  'execution_source_sha256':plan['frozen_source_sha256']})
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with (out/'all_27_episodes.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    lines=['| Condition | Original | Feedback repair | Dynamic contact |','|---|---|---|---|']
    for c in plan['conditions']:
        cells=[]
        for arm in ARMS:
            r=lookup[c['condition'],arm]
            cells.append((f"{'PASS' if r['completed'] else 'FAIL'} / {r['endpoint_error_m']:.4f} m")
                         if r['endpoint_error_m'] is not None else 'EXECUTION ERROR')
        lines.append('| '+c['condition']+' | '+' | '.join(cells)+' |')
    (out/'capability_table.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    plot(rows,plan,trajectories,out)
    print(json.dumps({'counts':counts,'comparisons':comparisons,'development_gate_pass':gate,
                      'legacy_compatibility_pass':compatible},indent=2))
    return report


def plot(rows,plan,trajectories,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
    colors=('#0072B2','#E69F00','#009E73'); styles=('-','--',':')
    tolerance = plan['cases'][0]['config']['task']['tolerance']
    names=[c['condition'] for c in plan['conditions']]
    fig,ax=plt.subplots(figsize=(7.15,3.3))
    for j,arm in enumerate(ARMS):
        group={r['condition']:r for r in rows if r['arm']==arm}
        for i,name in enumerate(names):
            r=group[name]
            if r['endpoint_error_m'] is None: continue
            ax.scatter(i+(j-1)*.2,r['endpoint_error_m'],s=28,marker=('o','s','^')[j],
                       facecolors=colors[j] if r['completed'] else 'none',edgecolors=colors[j],
                       label=LABELS[j] if i==0 else None)
    ax.axhline(tolerance,color='0.4',linestyle='--',linewidth=.8,label=f'Original tolerance ({tolerance:g} m)')
    ax.set_xticks(range(9),[n.replace('_','\n') for n in names],fontsize=7)
    ax.set_ylabel('Final 2D endpoint error (m)');ax.set_ylim(bottom=0)
    ax.legend(fontsize=7,ncol=2);ax.grid(axis='y',alpha=.2)
    ax.set_title('One fixed development seed; 9 paired conditions per arm\nFilled: task completed; open: task failed (all executions retained)',fontsize=8)
    fig.tight_layout()
    for ext in ('pdf','svg','png'):fig.savefig(out/f'endpoint_comparison.{ext}',dpi=180)
    plt.close(fig)
    fig,axes=plt.subplots(3,3,figsize=(7.15,6.5))
    for ax,name in zip(axes.flat,names):
        goal = None
        for j,arm in enumerate(ARMS):
            data=trajectories.get(f'{name}_{arm}')
            if data is None:continue
            xy=data['xy'];goal=data['goal']
            ax.plot(xy[:,0],xy[:,1],styles[j],color=colors[j],linewidth=1.,label=LABELS[j])
            ax.plot(*xy[-1],marker='x',color=colors[j],markersize=4)
        if goal is None:
            ax.text(.5,.5,'No executed trajectory',transform=ax.transAxes,ha='center')
            continue
        ax.add_patch(Circle(goal,tolerance,facecolor='0.9',edgecolor='0.5',linewidth=.5))
        ax.plot([0,goal[0]],[0,goal[1]],color='0.7',linewidth=.6)
        ax.scatter([0,goal[0]],[0,goal[1]],marker='+',color='black',s=25)
        ax.set_title(name.replace('_',' '),fontsize=8);ax.set_aspect('equal',adjustable='datalim')
        ax.grid(alpha=.2);ax.set_xlabel('Along task (m)');ax.set_ylabel('Cross track (m)')
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, fontsize=8, loc='upper center', ncol=3, frameon=False)
    fig.tight_layout(rect=(0,0,1,.96))
    for ext in ('pdf','svg','png'):fig.savefig(out/f'cargo_paths.{ext}',dpi=180)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=ROOT/'artifacts/research'/MISSION/'analysis')
    args=p.parse_args()
    analyze(args.out)
