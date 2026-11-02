"""Verify the complete geometry factorial, including adverse and missing outcomes."""
import argparse, collections, gc, json
from pathlib import Path
import numpy as np
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot
from analyze_dynamic_contact_study import read, sha, replay_equal, old_fields_equal
from analyze_feedback_components import csv_rows, safety_summary, save
from freeze_contact_geometry import ROOT, MISSION, HISTORY, ARMS, CONDITIONS


def analyze(out):
    assert out.resolve().is_relative_to(ROOT/'artifacts/research'/MISSION)
    out.mkdir(parents=True,exist_ok=False);limit_blas_threads(1)
    protocol=ROOT/'experiments/ccta2027/contact_geometry_v1.json';plan=read(protocol)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');summary=read(batch/'summary.json')
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256'] and summary['source_integrity']=='PASS'
    inputs=dict(plan['historical_inputs'])
    for p,h in inputs.items():assert sha(ROOT/p)==h,p
    rows=[];series={};checks={}
    for case in plan['cases']:
        cid=case['case_id'];folder=batch/cid;print('Audit '+cid,flush=True)
        receipt=read(folder/'case_result.json')
        assert receipt['identity']=={**manifest['identity'],'case_sha256':encode(case)}
        r=cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        row=dict(condition=case['condition_id'],arm=case['arm'],seed=case['seed'],
            execution_status=r['execution_status'],completed=bool(r.get('task_completed')),
            g500=bool(r.get('g500_pass')),wall_seconds=r.get('wall_seconds'),error=r.get('error',''))
        if r['execution_status']!='COMPLETED':rows.append(row);continue
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            t,xy,start,goal,poses,speed=[z[k].copy() for k in ('times','cargo/cargo_0/centers',
                'task/cargo_0/start','task/cargo_0/goal_point','agent_positions','cargo/cargo_0/speed')]
            yaw=z['cargo/cargo_0/angles'].copy()
        assert len(t)==len(steps)+1
        duration=r['termination']['frames_run']*case['config']['dt'];errors=np.linalg.norm(xy-goal,axis=1)
        assert abs(float(errors[-1])-r['euclidean_endpoint_error'])<1e-12 and duration<=600.
        assert row['completed']==bool(r['settled'] and errors[-1]<=case['config']['task']['tolerance'])
        m,hard,verdict=safety_summary(folder)
        env=SimulationEnvironment(case['config'],seed=case['seed']);cargo=env.cargoes[0]
        selected=[];radial=collections.Counter();patch=collections.Counter();gap_old=[];gap_new=[]
        phases=collections.Counter();first_brake=None;first_contact=None;true_in_band=0
        physical_quiet_streak=0;max_physical_quiet_streak=0;controller_arrived_frames=0
        gates=env.controller.phase_monitor.gates
        for k,s in enumerate(steps):
            phases[s['phase']]+=1
            if first_brake is None and s['phase']=='BRAKE':first_brake=k
            if first_contact is None and s['contacts']>0:first_contact=k
            nav=s['navigation_supervisor']
            selected.append(None if nav is None else float(np.linalg.norm(np.asarray(nav['displacement'])-(xy[k]-start))))
            if nav is not None:
                accepted=[d['reference_displacement'] for d in s['diagnostics'] if d['reference_matches']>0]
                # Before reference bootstrap, the unchanged controller uses
                # accumulated-map votes. Compare reference votes only when
                # they exist; do not assert that an empty reference set is used.
                if accepted:assert np.array_equal(np.median(accepted,axis=0),nav['displacement'])
            assert abs(np.linalg.norm(s['true_velocity'])-speed[k+1])<1e-12
            cargo.set_pose(xy[k],yaw[k]);signed,_,_=cargo.signed_distance(poses[k])
            for i,d in enumerate(s['diagnostics']):
                radial[d['reference_pivot_status']]+=1;patch[d['contact_distance_source']]+=1
                if d['contact_distance_source']=='accepted':
                    gap_old.append(abs(d['contact_map_gap']-signed[i]));gap_new.append(abs(d['contact_used_gap']-signed[i]))
            true_in_band+=int(errors[k+1]<=case['config']['task']['tolerance'])
            quiet_here=errors[k+1]<=case['config']['task']['tolerance'] and speed[k+1]<=.005
            physical_quiet_streak=physical_quiet_streak+1 if quiet_here else 0
            max_physical_quiet_streak=max(max_physical_quiet_streak,physical_quiet_streak)
            guards=s['phase_guards']
            controller_arrived_frames+=int(guards['progress']>=guards['target_distance']-gates.hold_tolerance)
        valid=[v for v in selected if v is not None]
        row.update(endpoint_error_m=float(errors[-1]),seconds=duration,
            completion_seconds=duration if row['completed'] else None,settled=bool(r['settled']),
            outcome='COMPLETED' if row['completed'] else 'SETTLED_OFF_TARGET' if r['settled'] else 'CENSORED_600S',
            safe_completed=bool(row['completed'] and not hard),hard_safety='|'.join(hard),
            g500_reasons=' | '.join(verdict.reasons),closest_error_m=float(errors.min()),
            true_in_goal_band_frames=true_in_band,selected_displacement_mae_m=float(np.mean(valid)) if valid else None,
            max_physical_goal_quiet_streak_frames=max_physical_quiet_streak,
            controller_arrived_frames=controller_arrived_frames,
            selected_displacement_max_m=max(valid,default=None),final_selected_error_m=selected[-1],
            first_BRAKE_time_s=None if first_brake is None else float(t[first_brake]),
            first_BRAKE_goal_error_m=None if first_brake is None else float(errors[first_brake]),
            first_contact_time_s=None if first_contact is None else float(t[first_contact+1]),
            final_phase=steps[-1]['phase'],brake_reason=steps[-1]['phase_response']['brake_reason'],
            radial_admitted_agent_frames=radial['accepted'],patch_admitted_agent_frames=patch['accepted'],
            patch_same_sample_map_mae_m=float(np.mean(gap_old)) if gap_old else None,
            patch_same_sample_used_mae_m=float(np.mean(gap_new)) if gap_new else None,
            mean_contacts=float(np.mean([s['contacts'] for s in steps])),
            last10s_mean_contacts=float(np.mean([s['contacts'] for s in steps[-200:]])),
            last10s_max_net_force_N=max(float(np.linalg.norm(s['actual_net_force'])) for s in steps[-200:]),
            max_penetration_m=m['max_penetration'],penetration_budget_m=m['penetration_budget'],
            min_separation_m=m['min_inter_agent_distance'],barrier_scalings=m['barrier_scalings'])
        checks[cid]=dict(phases=dict(phases),radial_status=dict(radial),patch_status=dict(patch))
        if case['arm']=='reference':
            old=HISTORY/(case['condition_id']+'_reference')
            equal,arrays=replay_equal(old/'replay.npz',folder/'replay.npz')
            previous=read(old/'paired_metrics.json')['steps']
            checks[cid].update(all_replay_arrays_exact=equal,arrays=arrays,
                all_historical_step_fields_exact=old_fields_equal(previous,steps));del previous
        rows.append(row)
        series[cid]=dict(time=t.tolist(),xy=(xy-start).tolist(),goal=(goal-start).tolist(),
            goal_error=errors.tolist(),estimate_error=selected,contacts=[s['contacts'] for s in steps],
            threshold=case['config']['task']['tolerance'],outcome=row['outcome'])
        del steps,env;gc.collect()
    lookup={(r['condition'],r['arm']):r for r in rows}
    for c in CONDITIONS:
        for candidate,base in [('radial','reference'),('combined','patch')]:
            row=lookup[c,candidate]
            if row['execution_status']=='COMPLETED' and row.get('radial_admitted_agent_frames')==0:
                equal,arrays=replay_equal(batch/(c+'_'+candidate)/'replay.npz',batch/(c+'_'+base)/'replay.npz')
                checks[c+'_'+candidate]['rejected_radial_model_preserves_replay']=equal
                checks[c+'_'+candidate]['fallback_arrays']=arrays
                assert equal,'Rejected radial model changed behavior'
    counts={a:dict(denominator=3,completed=sum(r['completed'] for r in rows if r['arm']==a),
        safe_completed=sum(r.get('safe_completed',False) for r in rows if r['arm']==a),
        hard_safety_episodes=sum(bool(r.get('hard_safety')) for r in rows if r['arm']==a),
        g500=sum(r['g500'] for r in rows if r['arm']==a),
        execution_errors=sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==a)) for a in ARMS}
    compatible=all(checks.get(c+'_reference',{}).get('all_replay_arrays_exact') and
        checks.get(c+'_reference',{}).get('all_historical_step_fields_exact') for c in CONDITIONS)
    comparisons=[];factorial={};gates={}
    for c in CONDITIONS:
        for a in ARMS[1:]:
            new,base=lookup[c,a],lookup[c,'reference']
            comparisons.append(dict(condition=c,arm=a,completion_delta=int(new['completed'])-int(base['completed']),
                endpoint_delta_m=new.get('endpoint_error_m',np.nan)-base.get('endpoint_error_m',np.nan)))
        y=[lookup[c,a].get('endpoint_error_m',np.nan) for a in ARMS]
        factorial[c]=dict(radial_at_original_distance=y[1]-y[0],radial_at_patch_distance=y[3]-y[2],
            patch_at_original_reference=y[2]-y[0],patch_at_radial_reference=y[3]-y[1],interaction=y[3]-y[2]-y[1]+y[0])
    for a in ARMS[1:]:
        rescues=[c for c in CONDITIONS if lookup[c,a].get('safe_completed') and not lookup[c,'reference']['completed']]
        used=any(lookup[c,a].get('radial_admitted_agent_frames',0) or lookup[c,a].get('patch_admitted_agent_frames',0) for c in CONDITIONS)
        gates[a]=dict(rescued_conditions=rescues,passed=bool(compatible and rescues and used and
            not counts[a]['hard_safety_episodes'] and not counts[a]['execution_errors']))
    report=dict(mission=MISSION,counts=counts,episodes=rows,comparisons=comparisons,factorial_endpoint=factorial,
        checks=checks,repeat_compatibility=compatible,development_gates=gates,
        statistics='Three selected failures, reused seed 2; descriptive paired contrasts only. No success-rate generalization or independent samples from frames.',
        limitations='Trajectory-level MAE across arms is not a same-input comparison. Fresh nominal distance does not replace the safety map. Radial model admission does not certify center of mass or shape family. No default promotion.',
        provenance=dict(protocol_sha256=sha(protocol),analysis_sha256=sha(Path(__file__)),inputs=inputs))
    dump(out/'analysis.json',report);dump(out/'timeseries.json',series)
    csv_rows(out/'all_12_episodes.csv',rows);csv_rows(out/'paired_comparisons.csv',comparisons)
    plot(series,out)
    print(json.dumps(dict(counts=counts,reference_compatibility=compatible,gates=gates),indent=2))
    if not compatible:raise RuntimeError('Reference reproducibility failed; no fair-comparison claim')


def plot(series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({'font.size':7,'pdf.fonttype':42,'svg.fonttype':'none','path.simplify':False})
    colors=['#555555','#0072B2','#D55E00','#009E73'];styles=[':','--','-.','-']
    labels=['Reference','Radial readout','Patch distance','Combined']
    fig,axes=plt.subplots(1,3,figsize=(7.16,2.6),layout='constrained')
    for ax,c in zip(axes,CONDITIONS):
        available=None
        for a,col,style,label in zip(ARMS,colors,styles,labels):
            d=series.get(c+'_'+a)
            if d is None:continue
            available=d
            xy=np.array(d['xy']);ax.plot(*xy.T,c=col,ls=style,lw=.8,label=label)
            ax.plot(*xy[-1],marker='o' if d['outcome']=='COMPLETED' else 'x',ms=4,c=col)
        if available:ax.add_patch(Circle(available['goal'],available['threshold'],fill=False,ec='k',ls=':',lw=.7))
        else:ax.text(.5,.5,'No completed execution',transform=ax.transAxes,ha='center')
        ax.set_title(c.replace('_',' '));ax.set_xlabel('Task-axis displacement (m)');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
    axes[0].set_ylabel('Lateral displacement (m)')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='outside upper center',ncol=4,frameon=False)
    save(fig,out,'complete_trajectories')
    fig,axes=plt.subplots(3,3,figsize=(7.16,5.8),sharex=True,layout='constrained')
    for j,c in enumerate(CONDITIONS):
        available=None
        for a,col,style,label in zip(ARMS,colors,styles,labels):
            d=series.get(c+'_'+a)
            if d is None:continue
            available=d
            t=np.array(d['time']);axes[0,j].plot(t,d['goal_error'],c=col,ls=style,lw=.8,label=label)
            axes[1,j].plot(t[:-1],d['estimate_error'],c=col,ls=style,lw=.8)
            axes[2,j].plot(t[1:],d['contacts'],c=col,ls=style,lw=.5)
        axes[0,j].set_title(c.replace('_',' '))
        if available:axes[0,j].axhline(available['threshold'],c='k',ls=':',lw=.7)
        axes[2,j].set_xlabel('Physical time (s)')
        for ax in axes[:,j]:ax.grid(alpha=.2);ax.set_xlim(0,600)
    for ax,label in zip(axes[:,0],['Physical goal error (m)','Displacement error (m)','Physical contact count']):ax.set_ylabel(label)
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc='outside upper center',ncol=4,frameon=False)
    save(fig,out,'task_and_contact_timeseries')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);analyze(p.parse_args().out)
