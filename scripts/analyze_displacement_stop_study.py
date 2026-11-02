"""Audit the four-arm fixed-budget comparison, keeping every planned outcome."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot
from audit_displacement_replay import ROOT, MISSION, HISTORY, read, sha
from freeze_displacement_stop_study import ARMS
from analyze_dynamic_contact_study import old_fields_equal, replay_equal
from analyze_boundary_navigation_study import at_horizon


def analyze(out):
    out=out.resolve()
    if not out.is_relative_to((ROOT/'artifacts/research'/MISSION).resolve()):
        raise ValueError('Output outside mission')
    out.mkdir(parents=True,exist_ok=False)
    protocol=ROOT/'experiments/ccta2027/displacement_stop_v1.json'
    plan=read(protocol);batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json')
    summary=read(batch/'summary.json')
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256']
    assert summary['source_integrity']=='PASS'
    for name,digest in plan['historical_inputs'].items():
        if sha(ROOT/name)!=digest:raise RuntimeError(f'Historical input changed: {name}')
    rows=[];compatibility={};series={};inputs=dict(plan['historical_inputs'])
    for case in plan['cases']:
        folder=batch/case['case_id'];print(f"Audit {case['case_id']}",flush=True)
        receipt=read(folder/'case_result.json')
        assert receipt['identity']=={**manifest['identity'],'case_sha256':encode(case)}
        r=cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        row=dict(condition=case['condition_id'],arm=case['arm'],seed=case['seed'],
            execution_status=r['execution_status'],completed=bool(r.get('task_completed')),
            completed_150=False,g500=bool(r.get('g500_pass')),wall_seconds=r['wall_seconds'],error=r.get('error',''))
        if r['execution_status']!='COMPLETED':rows.append(row);continue
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            t,xy,start,goal,speed=[z[k].copy() for k in
                ('times','cargo/cargo_0/centers','task/cargo_0/start','task/cargo_0/goal_point','cargo/cargo_0/speed')]
        seconds=r['termination']['frames_run']*case['config']['dt']
        horizon=at_horizon(t,xy,goal,150.,seconds,row['completed'])
        final_error=float(np.linalg.norm(xy[-1]-goal))
        assert abs(final_error-r['euclidean_endpoint_error'])<1e-12
        assert bool(r['settled'] and final_error<=case['config']['task']['tolerance'])==row['completed']
        gate=read(folder/'summary.json')['cargoes']['cargo_0']['g500']
        contract=ClosedLoopContract(**gate['gates'])
        verdict=contract.evaluate(gate['metrics'])
        assert verdict.success==gate['success'] and verdict.reasons==gate['failure_reasons']
        metrics=gate['metrics'];hard=[]
        if metrics['min_inter_agent_distance']<metrics['d_min']-contract.d_min_tolerance:hard.append('separation')
        if metrics['min_signed_clearance']<0:hard.append('inside_object')
        if metrics['max_penetration']>metrics['penetration_budget']:hard.append('penetration_budget')
        if metrics['solver_fallbacks'] or metrics['solver_infeasible']:hard.append('solver_failure')
        ref_estimate=[];ref_times=[];ref_errors=[];ref_voters=[]
        for k,s in enumerate(steps):
            ds=[d['reference_displacement'] for d in s['diagnostics'] if d.get('reference_matches',0)>0]
            if ds:
                estimate=np.median(ds,axis=0)
                ref_times.append(t[k]);ref_estimate.append(estimate)
                ref_errors.append(float(np.linalg.norm(estimate-(xy[k]-start))))
                ref_voters.append(len(ds))
        estimated_remaining=np.array([case['config']['task']['distance_min']-s['phase_guards']['progress']
            if s['phase_guards']['estimate_quality']!='unavailable' else np.nan for s in steps])
        physical_ready=(np.linalg.norm(xy-goal,axis=1)<=case['config']['task']['tolerance'])&(speed<=r['termination']['settle_speed'])
        tail=steps[-min(200,len(steps)):]
        tail_start=len(steps)-len(tail)
        correction_forces=[]
        for k,s in enumerate(tail,tail_start):
            error=goal-xy[k]  # penalty report is at the start of the physics hold
            direction=error/max(float(np.linalg.norm(error)),1e-12)
            correction_forces.append(float(np.asarray(s['actual_net_force'])@direction))
        streak=0
        for flag in physical_ready[:0:-1]:
            if not flag:break
            streak+=1
        row.update(completed_150=horizon['completed'],endpoint_error_150_m=horizon['endpoint_error_m'],
            endpoint_error_m=final_error,seconds=seconds,completion_seconds=seconds if row['completed'] else None,
            settled=bool(r['settled']),safe_completed=bool(row['completed'] and not hard),
            outcome='COMPLETED' if row['completed'] else 'SETTLED_OFF_TARGET' if r['settled'] else 'CENSORED_600S',
            final_phase=steps[-1]['phase'],hard_safety='|'.join(hard),
            max_penetration_m=metrics['max_penetration'],penetration_budget_m=metrics['penetration_budget'],
            min_separation_m=metrics['min_inter_agent_distance'],min_clearance_m=metrics['min_signed_clearance'],
            barrier_scalings=metrics['barrier_scalings'],g500_reasons=' | '.join(verdict.reasons),
            reference_frames=len(ref_times),reference_shared_MAE_m=float(np.mean(ref_errors)) if ref_errors else None,
            reference_last_available_error_m=ref_errors[-1] if ref_errors else None,
            reference_last_available_time_s=float(ref_times[-1]) if ref_times else None,
            reference_final_error_m=ref_errors[-1] if ref_times and ref_times[-1]==t[-2] else None,
            reference_final_voters=ref_voters[-1] if ref_voters else None,
            physical_target_low_speed_tail_s=streak*case['config']['dt'],
            final_estimated_remaining_m=float(estimated_remaining[-1]),
            diagnostic_last10s_mean_pushers=float(np.mean([sum(d['push_side'] for d in s['diagnostics']) for s in tail])),
            diagnostic_last10s_mean_contacts=float(np.mean([s['contacts'] for s in tail])),
            diagnostic_last10s_mean_physical_speed_m_s=float(np.mean([np.linalg.norm(s['true_velocity']) for s in tail])),
            diagnostic_last10s_mean_contact_force_toward_true_goal_N=float(np.mean(correction_forces)))
        rows.append(row)
        series[case['case_id']]=dict(time=t.tolist(),xy=(xy-start).tolist(),
            estimated_remaining=estimated_remaining.tolist(),ref_time=list(map(float,ref_times)),
            ref_error=ref_errors,ref_estimate=np.asarray(ref_estimate).tolist())
        if case['arm']!='reference_cvt':
            old=HISTORY/case['case_id']
            equal,arrays=replay_equal(old/'replay.npz',folder/'replay.npz')
            old_steps=read(old/'paired_metrics.json')['steps']
            compatibility[case['case_id']]=dict(arrays=arrays,all_arrays_exact=equal,
                all_old_step_fields_exact=old_fields_equal(old_steps,steps))
            del old_steps
        del steps
    counts={arm:dict(denominator=9,completed_150=sum(r['completed_150'] for r in rows if r['arm']==arm),
        completed_600=sum(r['completed'] for r in rows if r['arm']==arm),
        safe_completed=sum(r.get('safe_completed',False) for r in rows if r['arm']==arm),
        g500=sum(r['g500'] for r in rows if r['arm']==arm),
        hard_safety_episodes=sum(bool(r.get('hard_safety')) for r in rows if r['arm']==arm),
        execution_errors=sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==arm)) for arm in ARMS}
    lookup={(r['condition'],r['arm']):r for r in rows}
    names=[c['condition_id'] for c in plan['cases'] if c['arm']=='cvt']
    pairs={}
    for ref in ('cvt','spacing','vector_cvt'):
        pairs[ref]=dict(gained=[n for n in names if lookup[n,'reference_cvt']['completed'] and not lookup[n,ref]['completed']],
            lost=[n for n in names if lookup[n,ref]['completed'] and not lookup[n,'reference_cvt']['completed']],
            newly_unsafe=[n for n in names if lookup[n,'reference_cvt'].get('hard_safety') and not lookup[n,ref].get('hard_safety')])
    compatible=all(r['all_arrays_exact'] and r['all_old_step_fields_exact'] for r in compatibility.values())
    report=dict(mission=MISSION,counts=counts,paired=pairs,compatibility=compatibility,
        compatibility_pass=compatible,episodes=rows,source_integrity=summary['source_integrity'],
        development_only='All nine conditions use the same reused development seed; no independent reliability inference',
        diagnostic_scope='Last-10-second contact summaries are descriptive failure analysis, not causal interventions or new primary endpoints. Force is the penalty contact report projected onto the evaluator goal direction, not net force after friction.',
        provenance=dict(protocol_sha256=sha(protocol),inputs=inputs,analysis_sha256=sha(Path(__file__)),
            helpers={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'scripts/analyze_dynamic_contact_study.py',ROOT/'scripts/analyze_boundary_navigation_study.py')}))
    dump(out/'analysis.json',report);dump(out/'timeseries.json',series)
    with (out/'all_episodes.csv').open('w',encoding='utf8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)));w.writeheader();w.writerows(rows)
    lines=['| Condition | '+' | '.join(ARMS)+' |','|---|'+'---|'*4]
    for n in names:
        lines.append('| '+n+' | '+' | '.join(f"{lookup[n,a].get('outcome','ERROR')} / {lookup[n,a].get('endpoint_error_m',float('nan')):.4f} m / safety={'FAIL' if lookup[n,a].get('hard_safety') else 'PASS'}" for a in ARMS)+' |')
    (out/'capability_table.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    plot(rows,names,series,out)
    print(json.dumps(dict(counts=counts,paired=pairs,compatibility_pass=compatible),indent=2))
    if not compatible:raise RuntimeError('Control replay changed; do not claim a fair comparison')


def plot(rows,names,series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':7,'pdf.fonttype':42,'svg.fonttype':'none'})
    colors=('#0072B2','#E69F00','#009E73','#CC79A7');marks=('o','s','^','D')
    lookup={(r['condition'],r['arm']):r for r in rows}
    fig,axes=plt.subplots(1,2,figsize=(7.16,3.45),sharey=True)
    for ax,horizon in zip(axes,(150,600)):
        for j,arm in enumerate(ARMS):
            for k,n in enumerate(names):
                r=lookup[n,arm];y=r.get('endpoint_error_150_m' if horizon==150 else 'endpoint_error_m')
                if y is None:continue
                done=r['completed_150' if horizon==150 else 'completed']
                ax.scatter(k+(j-1.5)*.16,y,s=22,marker=marks[j],edgecolors=colors[j],
                    facecolors=colors[j] if done else 'none',label=arm if k==0 else None)
                if r.get('hard_safety'):ax.plot(k+(j-1.5)*.16,y,'kx',ms=4)
        ax.axhline(.12,c='k',ls=':',lw=.8);ax.set_title(f'{horizon} s; filled = completed')
        ax.set_xticks(range(9),[n.replace('_',' ') for n in names],rotation=60,ha='right',fontsize=6);ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('2D endpoint error (m)')
    fig.legend(*axes[0].get_legend_handles_labels(),ncol=4,loc='upper center',frameon=False)
    fig.tight_layout(rect=(0,0,1,.88))
    for ext in ('pdf','svg','png'):fig.savefig(out/f'endpoint_comparison.{ext}',dpi=220)
    plt.close(fig)
    fig,axes=plt.subplots(3,3,figsize=(7.16,5.7))
    for ax,n in zip(axes.flat,names):
        for j,arm in enumerate(ARMS):
            item=series.get(n+'_'+arm)
            if not item:continue
            xy=np.array(item['xy']);ax.plot(*xy.T,color=colors[j],lw=.8,label=arm,ls=('-','--',':','-')[j])
            ax.scatter(*xy[-1],s=12,marker=marks[j],c=colors[j])
        ax.add_patch(Circle((1.2,0),.12,fill=False,ec='k',ls=':',lw=.7));ax.set_aspect('equal',adjustable='datalim')
        ax.set_title(n.replace('_',' '));ax.set_xlabel('Task-axis displacement (m)');ax.set_ylabel('Lateral displacement (m)');ax.grid(alpha=.2)
    fig.legend(*axes.flat[0].get_legend_handles_labels(),ncol=4,loc='upper center',frameon=False)
    fig.tight_layout(rect=(0,0,1,.95))
    for ext in ('pdf','svg','png'):fig.savefig(out/f'complete_trajectories.{ext}',dpi=220)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    analyze(p.parse_args().out)
