"""Verify and analyze all nine frozen contact-reinforcement episodes, including failures."""
from __future__ import annotations
import argparse,collections,gc,json
from pathlib import Path
import numpy as np
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result,verify_source_snapshot
from analyze_dynamic_contact_study import read,sha,replay_equal,old_fields_equal
from analyze_feedback_components import safety_summary,csv_rows,save
from analyze_boundary_navigation_study import at_horizon
from freeze_contact_reinforcement import ROOT,MISSION,HISTORY,ARMS,CONDITIONS


def event_counts(steps):
    events=collections.Counter();by_robot={};states=collections.Counter()
    suspended=[];prospective=[]
    for s in steps:
        suspended.append(bool(s['phase_response']['push_suspended']))
        prospective.append(s['phase_guards'].get('recovery_ready'))
        physical={}
        for aid,force in zip(s['contact_ids'],s['contact_forces']):
            physical[aid]=physical.get(aid,np.zeros(2))+np.asarray(force)
        for d in s['diagnostics']:
            robot=d['agent_id'];states[d['reinforcement_state']]+=1
            item=by_robot.setdefault(robot,dict(attempts=0,max_recorded_path_m=0.,confirmed_frames=0,retained_frames=0,
                recruited_physical_contact_frames=0,recruited_max_force_N=0.))
            item['attempts']=max(item['attempts'],d['reinforcement_attempts'])
            item['max_recorded_path_m']=max(item['max_recorded_path_m'],d['reinforcement_path_length'])
            item['confirmed_frames']+=int(d['reinforcement_confirmed'])
            item['retained_frames']+=int(d['reinforcement_retain'])
            # Evaluator-only force evidence, never supplied to the policy.
            if d['reinforcement_attempts']>0 and d['reinforcement_state'] in ('joining','support') and robot in physical:
                item['recruited_physical_contact_frames']+=1
                item['recruited_max_force_N']=max(item['recruited_max_force_N'],float(np.linalg.norm(physical[robot])))
            for ev in d['reinforcement_event'].split('|'):
                if ev not in ('none','not_enabled'):events[ev]+=1
    episodes=[];start=None
    for k,on in enumerate(suspended+[False]):
        if on and start is None:start=k
        if not on and start is not None:
            resumed=k<len(steps) and steps[k]['phase']=='TRANSPORT'
            episodes.append(dict(start_frame=start,end_frame=k-1,resumed_transport=resumed,
                max_prospective=max((n for n in prospective[start:k] if n is not None),default=None)))
            start=None
    return dict(events=dict(events),robot_states=dict(states),robots=by_robot,pauses=episodes,
        paused_frames=sum(suspended),resumed_pauses=sum(e['resumed_transport'] for e in episodes),
        started_attempts=sum(v['attempts'] for v in by_robot.values()),
        label='Agent/frame counts are diagnostics, not independent trials; group response is not individual force confirmation.')


def analyze(out):
    out=out.resolve()
    if not out.is_relative_to(ROOT/'artifacts/research'/MISSION):raise ValueError('Output outside mission')
    out.mkdir(parents=True,exist_ok=False)
    protocol=ROOT/'experiments/ccta2027/contact_reinforcement_v1.json';plan=read(protocol)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');summary=read(batch/'summary.json')
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256'] and summary['source_integrity']=='PASS'
    inputs=dict(plan['historical_inputs'])
    for path,h in inputs.items():
        if sha(ROOT/path)!=h:raise RuntimeError('Historical input changed: '+path)
    rows=[];series={};mechanisms={};checks={}
    for case in plan['cases']:
        cid=case['case_id'];print('Audit '+cid,flush=True)
        folder=batch/cid;receipt=read(folder/'case_result.json')
        assert receipt['identity']=={**manifest['identity'],'case_sha256':encode(case)}
        result=cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        row=dict(condition=case['condition_id'],arm=case['arm'],seed=case['seed'],
            execution_status=result['execution_status'],completed=bool(result.get('task_completed')),
            g500=bool(result.get('g500_pass')),wall_seconds=result.get('wall_seconds'),error=result.get('error',''))
        if row['execution_status']!='COMPLETED':rows.append(row);continue
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            t,xy,start,goal,speed=[z[k].copy() for k in ('times','cargo/cargo_0/centers','task/cargo_0/start','task/cargo_0/goal_point','cargo/cargo_0/speed')]
        assert len(t)==len(steps)+1
        endpoint=float(np.linalg.norm(xy[-1]-goal));duration=result['termination']['frames_run']*case['config']['dt']
        assert abs(endpoint-result['euclidean_endpoint_error'])<1e-12 and duration<=600.
        assert row['completed']==bool(result['settled'] and endpoint<=case['config']['task']['tolerance'])
        horizon=at_horizon(t,xy,goal,150.,duration,row['completed'])
        m,hard,verdict=safety_summary(folder);mech=event_counts(steps);mechanisms[cid]=mech
        nav_error=[];estimated_remaining=[];nav_count=0;alignment=0.
        for k,s in enumerate(steps):
            nav=s['navigation_supervisor']
            nav_error.append(float(np.linalg.norm(np.asarray(nav['displacement'])-(xy[k]-start))) if nav else None)
            estimated_remaining.append(nav['remaining'] if nav else None)
            alignment=max(alignment,abs(float(np.linalg.norm(s['true_velocity']))-float(speed[k+1])))
            if any(d['reference_status']!='not_enabled' for d in s['diagnostics']):
                accepted=[d for d in s['diagnostics'] if d['reference_matches']>0]
                assert (nav is None)==(not accepted)
                if accepted:
                    assert nav['voters']==[d['agent_id'] for d in accepted]
                    assert np.array_equal(np.median([d['reference_velocity'] for d in accepted],axis=0),nav['velocity'])
                    assert np.array_equal(np.median([d['reference_displacement'] for d in accepted],axis=0),nav['displacement'])
                    nav_count+=1
        assert alignment<1e-12
        distance=np.linalg.norm(xy-goal,axis=1);closest=int(distance.argmin())
        brakes=[k for k,s in enumerate(steps) if s['phase']=='BRAKE'];first_brake=brakes[0] if brakes else None
        row.update(endpoint_error_m=endpoint,endpoint_error_150_m=horizon['endpoint_error_m'],completed_150=horizon['completed'],
            closest_goal_error_m=float(distance[closest]),closest_goal_time_s=float(t[closest]),
            first_BRAKE_time_s=float(t[first_brake]) if first_brake is not None else None,
            first_BRAKE_goal_error_m=float(distance[first_brake]) if first_brake is not None else None,
            seconds=duration,completion_seconds=duration if row['completed'] else None,settled=bool(result['settled']),
            outcome='COMPLETED' if row['completed'] else 'SETTLED_OFF_TARGET' if result['settled'] else 'CENSORED_600S',
            safe_completed=bool(row['completed'] and not hard),hard_safety='|'.join(hard),g500_reasons=' | '.join(verdict.reasons),
            max_penetration_m=m['max_penetration'],penetration_budget_m=m['penetration_budget'],
            min_separation_m=m['min_inter_agent_distance'],min_clearance_m=m['min_signed_clearance'],
            barrier_scalings=m['barrier_scalings'],final_phase=steps[-1]['phase'],brake_reason=steps[-1]['phase_response']['brake_reason'],
            final_selected_displacement_error_m=nav_error[-1],final_estimated_remaining_m=estimated_remaining[-1],
            paused_frames=mech['paused_frames'],resumed_pauses=mech['resumed_pauses'],recruitment_attempts=mech['started_attempts'],
            response_confirmations=mech['events'].get('group_response_confirmed',0),
            direction_role_exits=mech['events'].get('direction_no_longer_useful',0),
            no_route_checks=mech['events'].get('no_observed_free_route',0),
            rollback_events=sum(n for e,n in mech['events'].items() if e.startswith('rollback:')),
            last10s_mean_contacts=float(np.mean([s['contacts'] for s in steps[-200:]])),
            last10s_max_net_force_N=max(float(np.linalg.norm(s['actual_net_force'])) for s in steps[-200:]),
            last10s_max_speed_m_s=float(max(speed[-200:])))
        rows.append(row)
        series[cid]=dict(time=t.tolist(),xy=(xy-start).tolist(),goal=(goal-start).tolist(),
            goal_error=np.linalg.norm(xy-goal,axis=1).tolist(),estimate_error=nav_error,estimated_remaining=estimated_remaining,
            pushers=[sum(d['push_side'] for d in s['diagnostics']) for s in steps],
            contacts=[s['contacts'] for s in steps],suspended=[s['phase_response']['push_suspended'] for s in steps],
            threshold=case['config']['task']['tolerance'],outcome=row['outcome'])
        checks[cid]=dict(reference_channel_frames=nav_count,physical_time_alignment_residual=alignment)
        if case['arm']=='reference':
            old_folder=HISTORY/(case['condition_id']+'_reference_cvt')
            equal,arrays=replay_equal(old_folder/'replay.npz',folder/'replay.npz')
            old_steps=read(old_folder/'paired_metrics.json')['steps']
            checks[cid].update(all_replay_arrays_exact=equal,arrays=arrays,
                              all_historical_step_fields_exact=old_fields_equal(old_steps,steps))
            del old_steps
        elif case['arm']=='recovery':
            equal,arrays=replay_equal(batch/(case['condition_id']+'_reference')/'replay.npz',folder/'replay.npz')
            checks[cid].update(recovery_vs_reference_arrays_exact=equal,arrays=arrays)
        del steps;gc.collect()
    lookup={(r['condition'],r['arm']):r for r in rows}
    counts={a:dict(denominator=3,completed=sum(r['completed'] for r in rows if r['arm']==a),
        safe_completed=sum(r.get('safe_completed',False) for r in rows if r['arm']==a),
        g500=sum(r['g500'] for r in rows if r['arm']==a),
        hard_safety_episodes=sum(bool(r.get('hard_safety')) for r in rows if r['arm']==a),
        execution_errors=sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==a)) for a in ARMS}
    repeat=[checks.get(n+'_reference',{}) for n in CONDITIONS]
    compatible=all(v.get('all_replay_arrays_exact') and v.get('all_historical_step_fields_exact') for v in repeat)
    rescues=[n for n in CONDITIONS if lookup[n,'reinforcement'].get('safe_completed') and not lookup[n,'reference']['completed']]
    used=any(r.get('recruitment_attempts',0)>0 for r in rows if r['arm']=='reinforcement')
    gate=bool(compatible and rescues and not counts['reinforcement']['hard_safety_episodes'] and not counts['reinforcement']['execution_errors'] and used)
    comparisons=[dict(condition=n,arm=a,completion_delta=int(lookup[n,a]['completed'])-int(lookup[n,'reference']['completed']),
        endpoint_delta_m=lookup[n,a].get('endpoint_error_m',np.nan)-lookup[n,'reference'].get('endpoint_error_m',np.nan)) for n in CONDITIONS for a in ARMS[1:]]
    report=dict(mission=MISSION,counts=counts,episodes=rows,comparisons=comparisons,repeat_compatibility=compatible,
        checks=checks,mechanisms=mechanisms,rescued_conditions=rescues,development_gate=gate,
        decision='Further independent validation may be justified; keep experimental/default-off.' if gate else 'No promotion; stop this fixed candidate without retuning.',
        statistics='Three preselected failures, one reused development seed. No reliable success-rate estimate or generalization. Frame/event counts are not independent observations.',
        limitations='Retention/tangential protection and spare recruitment are bundled. A difference cannot isolate their causal contributions. Group response does not prove individual force transfer. Existing shared supervisor retained.',
        provenance=dict(protocol_sha256=sha(protocol),analysis_sha256=sha(Path(__file__)),inputs=inputs,
            helpers={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in [ROOT/'scripts/analyze_feedback_components.py',ROOT/'scripts/analyze_boundary_navigation_study.py',ROOT/'scripts/analyze_dynamic_contact_study.py']}))
    dump(out/'analysis.json',report);dump(out/'timeseries.json',series)
    csv_rows(out/'all_9_episodes.csv',rows);csv_rows(out/'paired_comparisons.csv',comparisons)
    plot(rows,series,out)
    print(json.dumps(dict(counts=counts,reference_compatibility=compatible,development_gate=gate),indent=2))
    if not compatible:raise RuntimeError('Reference compatibility failed; preserve results, no fair-comparison claim')


def plot(rows,series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':7,'pdf.fonttype':42,'svg.fonttype':'none'})
    colors=['#666666','#0072B2','#D55E00'];styles=[':','--','-'];labels=['Reference','Recovery only','Recovery + reinforcement']
    fig,axes=plt.subplots(1,3,figsize=(7.16,2.6),layout='constrained')
    for ax,n in zip(axes,CONDITIONS):
        for a,col,sty,label in zip(ARMS,colors,styles,labels):
            d=series.get(n+'_'+a)
            if not d:continue
            xy=np.asarray(d['xy']);ax.plot(*xy.T,c=col,ls=sty,lw=.85,label=label)
            ax.plot(*xy[-1],marker='o' if d['outcome']=='COMPLETED' else 'x',ms=4,c=col)
        if d:ax.add_patch(Circle(d['goal'],d['threshold'],fill=False,ec='k',ls=':',lw=.7))
        ax.set_title(n.replace('_',' '));ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
        ax.set_xlabel('Task-axis displacement (m)')
    axes[0].set_ylabel('Lateral displacement (m)')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='outside upper center',ncol=3,frameon=False)
    save(fig,out,'complete_trajectories')
    fig,axes=plt.subplots(3,3,figsize=(7.16,5.8),sharex=True,layout='constrained')
    for j,n in enumerate(CONDITIONS):
        for a,col,sty,label in zip(ARMS,colors,styles,labels):
            d=series.get(n+'_'+a)
            if not d:continue
            t=np.array(d['time']);axes[0,j].plot(t,d['goal_error'],c=col,ls=sty,lw=.8,label=label)
            axes[1,j].plot(t[:-1],d['estimate_error'],c=col,ls=sty,lw=.8)
            axes[2,j].plot(t[:-1],d['contacts'],c=col,ls=sty,lw=.65)
        axes[0,j].set_title(n.replace('_',' '));axes[0,j].axhline(.12,c='k',ls=':',lw=.7)
        axes[2,j].set_xlabel('Physical time (s)')
        for ax in axes[:,j]:ax.grid(alpha=.2);ax.set_xlim(0,600)
    for ax,label in zip(axes[:,0],['Physical goal error (m)','Selected displacement error (m)','Physical contact count']):ax.set_ylabel(label)
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc='outside upper center',ncol=3,frameon=False)
    save(fig,out,'task_and_contact_timeseries')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',required=True,type=Path)
    analyze(p.parse_args().out)
