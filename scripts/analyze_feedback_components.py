"""Analyze a frozen feedback factorial and separately identified historical controls."""
from __future__ import annotations
import argparse
import csv
import gc
import json
from pathlib import Path
import numpy as np
from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result,verify_source_snapshot
from analyze_dynamic_contact_study import read,sha,replay_equal,old_fields_equal
from analyze_boundary_navigation_study import at_horizon
from diagnose_feedback_contact_loss import extract
from freeze_feedback_components import ROOT,MISSION,HISTORY,ARMS


def factorial(lookup,condition,field):
    values=[lookup[condition,a].get(field) for a in ARMS]
    if any(v is None for v in values):return {'status':'UNAVAILABLE'}
    y00,y10,y01,y11=map(float,values)
    return dict(position_at_old_velocity=y10-y00,position_at_reference_velocity=y11-y01,
        velocity_at_old_position=y01-y00,velocity_at_reference_position=y11-y10,
        interaction=y11-y10-y01+y00)


def csv_rows(path,rows):
    with path.open('w',encoding='utf8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader();w.writerows(rows)


def safety_summary(folder):
    gate=read(folder/'summary.json')['cargoes']['cargo_0']['g500']
    contract=ClosedLoopContract(**gate['gates']);m=gate['metrics'];verdict=contract.evaluate(m)
    assert verdict.success==gate['success'] and verdict.reasons==gate['failure_reasons']
    hard=[]
    if m['min_inter_agent_distance']<m['d_min']-contract.d_min_tolerance:hard.append('separation')
    if m['min_signed_clearance']<0:hard.append('inside_object')
    if m['max_penetration']>m['penetration_budget']:hard.append('penetration_budget')
    if m['solver_fallbacks'] or m['solver_infeasible']:hard.append('solver_failure')
    return m,hard,verdict


def verified_historical_rows(old):
    """Cross-check reused scalar results against the verified original products."""
    rows=[]
    for r in old['episodes']:
        if r['arm']=='reference_cvt':continue
        folder=HISTORY/f"{r['condition']}_{r['arm']}"
        original=read(folder/'case_result.json')['result'];case=read(folder/'resolved_case.json')
        m,hard,verdict=safety_summary(folder)
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            horizon=at_horizon(z['times'],z['cargo/cargo_0/centers'],z['task/cargo_0/goal_point'],150.,
                original['termination']['frames_run']*case['config']['dt'],original['task_completed'])
        assert r['execution_status']==original['execution_status']
        assert r['completed']==original['task_completed'] and r['g500']==original['g500_pass']==verdict.success
        assert r['settled']==original['settled'] and r['endpoint_error_m']==original['euclidean_endpoint_error']
        assert r['hard_safety']=='|'.join(hard) and r['barrier_scalings']==m['barrier_scalings']
        assert r['completed_150']==horizon['completed'] and r['endpoint_error_150_m']==horizon['endpoint_error_m']
        assert r['safe_completed']==bool(r['completed'] and not hard)
        rows.append({**r,'origin':'HISTORICAL_DISPLACEMENT_STOP_V1'})
    assert len(rows)==27
    return rows


def analyze(out):
    out=out.resolve()
    if not out.is_relative_to(ROOT/'artifacts/research'/MISSION):raise ValueError('Output outside mission')
    out.mkdir(parents=True,exist_ok=False)
    protocol=ROOT/'experiments/ccta2027/feedback_components_v1.json';plan=read(protocol)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');summary=read(batch/'summary.json')
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256'] and summary['source_integrity']=='PASS'
    for name,h in plan['historical_inputs'].items():
        if sha(ROOT/name)!=h:raise RuntimeError('Changed historical input: '+name)
    old_report_path=ROOT/'artifacts/research/2026-11-01-displacement-stop-v1/closed-loop-analysis/analysis.json'
    old=read(old_report_path);rows=[];series={};checks={};contact={};inputs=dict(plan['historical_inputs'])
    for case in plan['cases']:
        print('Audit '+case['case_id'],flush=True)
        folder=batch/case['case_id'];receipt=read(folder/'case_result.json')
        assert receipt['identity']=={**manifest['identity'],'case_sha256':encode(case)}
        result=cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        row=dict(condition=case['condition_id'],arm=case['arm'],seed=case['seed'],origin='NEW_CURRENT_BATCH',
            execution_status=result['execution_status'],completed=bool(result.get('task_completed')),completed_150=False,
            g500=bool(result.get('g500_pass')),wall_seconds=result['wall_seconds'],error=result.get('error',''))
        if row['execution_status']!='COMPLETED':rows.append(row);continue
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            t,xy,start,goal,speed=[z[k].copy() for k in ('times','cargo/cargo_0/centers','task/cargo_0/start','task/cargo_0/goal_point','cargo/cargo_0/speed')]
        assert len(t)==len(steps)+1
        endpoint=float(np.linalg.norm(xy[-1]-goal));duration=result['termination']['frames_run']*case['config']['dt']
        assert abs(endpoint-result['euclidean_endpoint_error'])<1e-12
        assert row['completed']==bool(result['settled'] and endpoint<=case['config']['task']['tolerance'])
        horizon=at_horizon(t,xy,goal,150.,duration,row['completed'])
        m,hard,verdict=safety_summary(folder)
        selected_error=[];velocity_error=[];navtime=[];navpositions=[];navspeeds=[]
        # A logged true_velocity belongs to the completed physics hold. Shift it
        # by one row before comparing against the next control input.
        previous_velocity=None;speed_alignment=0.;supervisor_alignment=0.;channel_frames=0
        reference_position=case['arm'] in ('position_cvt','reference_cvt')
        reference_velocity=case['arm'] in ('velocity_cvt','reference_cvt')
        for k,s in enumerate(steps):
            true_v=np.asarray(s['true_velocity'])
            speed_alignment=max(speed_alignment,abs(float(np.linalg.norm(true_v))-float(speed[k+1])))
            nav=s['navigation_supervisor']
            if any(d['reference_status']!='not_enabled' for d in s['diagnostics']):
                accepted=[d for d in s['diagnostics'] if d['reference_matches']>0]
                assert (nav is None)==(not accepted)
                if accepted:
                    assert nav['voters']==[d['agent_id'] for d in accepted]
                    expected_velocity=np.median([d['reference_velocity'] if reference_velocity else
                        s['estimated_velocities'][d['agent_id']] for d in accepted],axis=0)
                    assert np.array_equal(expected_velocity,nav['velocity'])
                    if reference_position:
                        assert np.array_equal(np.median([d['reference_displacement'] for d in accepted],axis=0),nav['displacement'])
                    channel_frames+=1
            if nav is not None:
                estimate=np.asarray(nav['displacement']);v=np.asarray(nav['velocity'])
                selected_error.append(float(np.linalg.norm(estimate-(xy[k]-start))))
                velocity_error.append(float(np.linalg.norm(v-previous_velocity)) if previous_velocity is not None else np.nan)
                navtime.append(float(t[k]));navpositions.append(estimate.tolist());navspeeds.append(float(np.linalg.norm(v)))
                guards=s['phase_guards']
                supervisor_alignment=max(supervisor_alignment,abs(float(np.linalg.norm(v))-guards['estimated_speed']),
                    abs((guards['target_distance']-nav['remaining'])-guards['progress']))
            previous_velocity=true_v
        if speed_alignment>1e-12 or supervisor_alignment>1e-12:raise RuntimeError('Feedback/time alignment failure')
        holds=[k for k,s in enumerate(steps) if s['phase']=='HOLD'];first_hold=holds[0] if holds else None
        tail=steps[-min(200,len(steps)):]
        final_nav=steps[-1]['navigation_supervisor']
        row.update(endpoint_error_m=endpoint,endpoint_error_150_m=horizon['endpoint_error_m'],completed_150=horizon['completed'],
            seconds=duration,completion_seconds=duration if row['completed'] else None,settled=bool(result['settled']),
            outcome='COMPLETED' if row['completed'] else 'SETTLED_OFF_TARGET' if result['settled'] else 'CENSORED_600S',
            safe_completed=bool(row['completed'] and not hard),hard_safety='|'.join(hard),g500_reasons=' | '.join(verdict.reasons),
            barrier_scalings=m['barrier_scalings'],max_penetration_m=m['max_penetration'],penetration_budget_m=m['penetration_budget'],
            min_separation_m=m['min_inter_agent_distance'],min_clearance_m=m['min_signed_clearance'],
            final_phase=steps[-1]['phase'],brake_reason=steps[-1]['phase_response']['brake_reason'],
            first_HOLD_time_s=float(t[first_hold]) if first_hold is not None else None,
            first_HOLD_physical_error_m=float(np.linalg.norm(xy[first_hold]-goal)) if first_hold is not None else None,
            actual_navigation_frames=len(navtime),actual_navigation_MAE_m=float(np.mean(selected_error)) if selected_error else None,
            actual_navigation_final_error_m=float(np.linalg.norm(np.asarray(final_nav['displacement'])-(xy[-2]-start))) if final_nav else None,
            actual_navigation_velocity_MAE_m_s=float(np.nanmean(velocity_error)) if velocity_error else None,
            final_estimated_remaining_m=final_nav['remaining'] if final_nav else None,
            diagnostic_last10s_mean_pushers=float(np.mean([sum(d['push_side'] for d in s['diagnostics']) for s in tail])),
            diagnostic_last10s_mean_contacts=float(np.mean([s['contacts'] for s in tail])),
            diagnostic_last10s_mean_speed_m_s=float(np.mean([np.linalg.norm(s['true_velocity']) for s in tail])))
        rows.append(row)
        item=dict(time=t.tolist(),xy=(xy-start).tolist(),goal_error=np.linalg.norm(xy-goal,axis=1).tolist(),
            navtime=navtime,selected_error=selected_error,navposition=navpositions,navspeed=navspeeds,
            pushers=[sum(d['push_side'] for d in s['diagnostics']) for s in steps],contacts=[s['contacts'] for s in steps])
        series[case['case_id']]=item
        checks[case['case_id']]=dict(physical_speed_time_alignment_max_residual=speed_alignment,
            actual_shared_feedback_max_residual=supervisor_alignment,
            common_admission_and_selected_channel_frames_verified=channel_frames)
        if case['condition_id']=='c_open_away':
            contact[case['arm']],_=extract(case,folder,receipt,steps=steps)
        if case['arm']=='reference_cvt':
            old_folder=HISTORY/case['case_id'];equal,arrays=replay_equal(old_folder/'replay.npz',folder/'replay.npz')
            old_steps=read(old_folder/'paired_metrics.json')['steps']
            checks[case['case_id']].update(all_replay_arrays_exact=equal,arrays=arrays,
                all_historical_step_fields_exact=old_fields_equal(old_steps,steps))
            del old_steps
        del steps;gc.collect()
    historical=verified_historical_rows(old)
    all_rows=historical+rows;lookup={(r['condition'],r['arm']):r for r in rows}
    names=[c['condition_id'] for c in plan['cases'] if c['arm']=='gate_cvt']
    comparisons={name:{field:factorial(lookup,name,field) for field in ('completed','completed_150','endpoint_error_m','g500','safe_completed')} for name in names}
    counts={}
    for arm in ('cvt','spacing','vector_cvt',*ARMS):
        group=[r for r in all_rows if r['arm']==arm]
        counts[arm]=dict(denominator=9,origin=group[0]['origin'],completed_150=sum(r['completed_150'] for r in group),
            completed_600=sum(r['completed'] for r in group),g500=sum(r['g500'] for r in group),
            safe_completed=sum(r.get('safe_completed',False) for r in group),
            off_target_settling=sum(r.get('outcome')=='SETTLED_OFF_TARGET' for r in group),
            hard_safety_episodes=sum(bool(r.get('hard_safety')) for r in group),
            execution_errors=sum(r['execution_status']!='COMPLETED' for r in group))
    repeated=[v for k,v in checks.items() if k.endswith('_reference_cvt')]
    repeat_pass=len(repeated)==9 and all(v['all_replay_arrays_exact'] and v['all_historical_step_fields_exact'] for v in repeated)
    report=dict(mission=MISSION,counts=counts,episodes=all_rows,factorial_effects=comparisons,repeat_compatibility=repeat_pass,
        execution_checks=checks,c_away_contact_events=contact,source_integrity=summary['source_integrity'],
        statistics='One reused development seed per nine conditions. Factorial effects are conditional closed-loop interventions, no independent reliability/seed-level uncertainty inference. Estimator MAE uses different trajectories and is not a same-input estimator comparison.',
        provenance=dict(protocol_sha256=sha(protocol),inputs=inputs,analysis_sha256=sha(Path(__file__)),
            historical_analysis_path=old_report_path.relative_to(ROOT).as_posix(),historical_analysis_sha256=sha(old_report_path),
            helpers={p.relative_to(ROOT).as_posix():sha(p) for p in [ROOT/'scripts/diagnose_feedback_contact_loss.py',ROOT/'scripts/analyze_boundary_navigation_study.py',ROOT/'scripts/analyze_dynamic_contact_study.py']}))
    dump(out/'analysis.json',report);dump(out/'timeseries.json',series);csv_rows(out/'all_63_episodes.csv',all_rows)
    csv_rows(out/'factorial_effects.csv',[dict(condition=n,outcome=field,**value) for n,fields in comparisons.items() for field,value in fields.items()])
    table=['| Condition | '+' | '.join(ARMS)+' |','|---|'+'---|'*len(ARMS)]
    for n in names:table.append('| '+n+' | '+' | '.join(f"{lookup[n,a].get('outcome','ERROR')}; {lookup[n,a].get('endpoint_error_m',float('nan')):.4f} m; G500={lookup[n,a]['g500']}" for a in ARMS)+' |')
    (out/'capability_table.md').write_text('\n'.join(table)+'\n',encoding='utf8')
    plot(rows,names,series,out)
    print(json.dumps(dict(counts=counts,repeat_compatibility=repeat_pass),indent=2))
    if not repeat_pass:raise RuntimeError('Repeated old candidate changed; no fair comparison claim')


def plot(rows,names,series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':7,'pdf.fonttype':42,'svg.fonttype':'none'})
    colors=['#666666','#0072B2','#E69F00','#CC79A7'];marks=['o','s','^','D'];styles=[':','--','-.','-']
    labels=['Gate only (00)','Position only (10)','Velocity only (01)','Both (11)']
    lookup={(r['condition'],r['arm']):r for r in rows}
    fig,axes=plt.subplots(1,2,figsize=(7.16,3.5),sharey=True)
    for ax,horizon in zip(axes,(150,600)):
        for j,arm in enumerate(ARMS):
            for k,n in enumerate(names):
                r=lookup[n,arm];y=r.get('endpoint_error_150_m' if horizon==150 else 'endpoint_error_m')
                if y is None:continue
                done=r['completed_150' if horizon==150 else 'completed']
                ax.scatter(k+(j-1.5)*.16,y,s=21,marker=marks[j],edgecolors=colors[j],facecolors=colors[j] if done else 'none',label=labels[j] if k==0 else None)
                if r.get('hard_safety'):ax.plot(k+(j-1.5)*.16,y,'kx',ms=4)
        ax.axhline(.12,c='k',ls=':',lw=.8);ax.set_title(f'{horizon} s; filled = task completed');ax.grid(axis='y',alpha=.2)
        ax.set_xticks(range(9),[n.replace('_',' ') for n in names],rotation=60,ha='right',fontsize=6)
    axes[0].set_ylabel('2D endpoint error (m)');fig.legend(*axes[0].get_legend_handles_labels(),ncol=4,loc='upper center',frameon=False)
    fig.tight_layout(rect=(0,0,1,.88));save(fig,out,'endpoint_factorial')
    fig,axes=plt.subplots(3,3,figsize=(7.16,5.8))
    for ax,n in zip(axes.flat,names):
        for j,arm in enumerate(ARMS):
            d=series.get(n+'_'+arm)
            if not d:continue
            xy=np.asarray(d['xy']);ax.plot(*xy.T,color=colors[j],ls=styles[j],lw=.8,label=labels[j]);ax.scatter(*xy[-1],s=13,color=colors[j],marker=marks[j])
        ax.add_patch(Circle((1.2,0),.12,fill=False,ec='k',ls=':',lw=.7));ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set_title(n.replace('_',' '));ax.set_xlabel('Task-axis displacement (m)');ax.set_ylabel('Lateral displacement (m)')
    fig.legend(*axes.flat[0].get_legend_handles_labels(),ncol=4,loc='upper center',frameon=False);fig.tight_layout(rect=(0,0,1,.94));save(fig,out,'complete_trajectories')
    for limit in (40,600):
        fig,axes=plt.subplots(3,1,figsize=(7.16,5.8),sharex=True,layout='constrained')
        for j,arm in enumerate(ARMS):
            d=series.get('c_open_away_'+arm)
            if not d:continue
            t=np.asarray(d['time']);keep=t<=limit
            axes[0].plot(t[keep],np.asarray(d['goal_error'])[keep],color=colors[j],ls=styles[j],lw=.8,label=labels[j])
            # Preserve unavailable feedback as gaps, rather than connecting two
            # accepted samples across a missing interval.
            nt=np.asarray(d['navtime']);full_error=np.full(len(t)-1,np.nan)
            full_error[np.searchsorted(t[:-1],nt)]=np.asarray(d['selected_error'])
            axes[1].plot(t[:-1][keep[:-1]],full_error[keep[:-1]],color=colors[j],ls=styles[j],lw=.8)
            axes[2].plot(t[:-1][keep[:-1]],np.asarray(d['pushers'])[keep[:-1]],color=colors[j],ls=styles[j],lw=.6)
        for ax,label in zip(axes,['Physical goal error (m)','Selected displacement error (m)','Designated pushers']):ax.set_ylabel(label);ax.grid(alpha=.2)
        axes[0].axhline(.12,c='k',ls=':',lw=.7);axes[0].legend(fontsize=7,ncol=2);axes[-1].set_xlim(0,limit);axes[-1].set_xlabel('Physical time (s)')
        fig.suptitle('C-away: different closed-loop trajectories, not same-input estimation',fontsize=8)
        save(fig,out,f'c_away_{limit}s')


def save(fig,out,stem):
    import matplotlib.pyplot as plt
    for ext in ('pdf','svg','png'):fig.savefig(out/f'{stem}.{ext}',dpi=220)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',required=True,type=Path)
    analyze(p.parse_args().out)
