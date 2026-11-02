"""Verify every frozen episode and retain failures and safety tradeoffs."""
import argparse,collections,gc,json
from pathlib import Path
import numpy as np
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result,verify_source_snapshot
from analyze_dynamic_contact_study import replay_equal,old_fields_equal
from analyze_feedback_components import safety_summary,csv_rows,save
from audit_contact_consistency import ROOT,MISSION,OLD,read,sha
from freeze_contact_consistency import ARMS

def analyze(out):
    assert out.resolve().is_relative_to(ROOT/'artifacts/research'/MISSION)
    out.mkdir(parents=True,exist_ok=False);limit_blas_threads(1)
    protocol=ROOT/'experiments/ccta2027/contact_consistency_v1.json';plan=read(protocol)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');summary=read(batch/'summary.json')
    assert summary.get('source_integrity')=='PASS' and summary['completed_denominator']==18
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256']
    for p,h in plan['historical_inputs'].items():assert sha(ROOT/p)==h,p
    rows=[];series={};checks={};inputs={}
    for case in plan['cases']:
        name=case['case_id'];folder=batch/name;print('Audit '+name,flush=True)
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
            t,xy,start,goal,speed=[z[k].copy() for k in ('times','cargo/cargo_0/centers',
                'task/cargo_0/start','task/cargo_0/goal_point','cargo/cargo_0/speed')]
            yaw=z['cargo/cargo_0/angles'].copy()
        errors=np.linalg.norm(xy-goal,axis=1);seconds=len(steps)*case['config']['dt']
        assert len(t)==len(steps)+1 and seconds<=600
        # Reuse the established analysis tolerance for alternate NumPy norm
        # evaluation paths; this does not change the task's 0.12 m threshold.
        assert abs(float(errors[-1])-r['euclidean_endpoint_error'])<1e-12
        assert row['completed']==bool(r['settled'] and errors[-1]<=case['config']['task']['tolerance'])
        metrics,hard,verdict=safety_summary(folder)
        selected=[];contacts=[];phases=collections.Counter();inside=0;streak=longest=0
        for k,s in enumerate(steps):
            nav=s['navigation_supervisor'];phases[s['phase']]+=1;contacts.append(s['contacts'])
            selected.append(None if nav is None else float(np.linalg.norm(np.asarray(nav['displacement'])-(xy[k]-start))))
            assert abs(np.linalg.norm(s['true_velocity'])-speed[k+1])<1e-12
            quiet=errors[k+1]<=case['config']['task']['tolerance'] and speed[k+1]<=.005
            streak=streak+1 if quiet else 0;longest=max(longest,streak)
            inside+=int(errors[k+1]<=case['config']['task']['tolerance'])
        row.update(endpoint_error_m=float(errors[-1]),seconds=seconds,
            completion_seconds=seconds if row['completed'] else None,
            outcome='COMPLETED' if row['completed'] else 'SETTLED_OFF_TARGET' if r['settled'] else 'CENSORED_600S',
            settled=bool(r['settled']),safe_completed=bool(row['completed'] and not hard),hard_safety='|'.join(hard),
            g500_reasons=' | '.join(verdict.reasons),closest_error_m=float(errors.min()),
            true_goal_frames=inside,max_goal_quiet_streak=longest,
            selected_displacement_mae_m=float(np.mean([x for x in selected if x is not None])),
            final_phase=steps[-1]['phase'],brake_reason=steps[-1]['phase_response']['brake_reason'],
            mean_contacts=float(np.mean(contacts)),final_speed=r['final_speed'],
            max_penetration_m=metrics['max_penetration'],penetration_budget_m=metrics['penetration_budget'],
            min_separation_m=metrics['min_inter_agent_distance'],barrier_scalings=metrics['barrier_scalings'])
        checks[name]=dict(phases=dict(phases))
        if name=='c_open_away_consistent_aggregate':
            hold=next((k for k,s in enumerate(steps) if s['phase']=='HOLD'),None)
            wanted=set(range(0,len(steps),200))|{len(steps)-1}
            if hold is not None:wanted.add(hold)
            events=[]
            for k in sorted(wanted):
                s=steps[k];nav=s['navigation_supervisor']
                accepted=[d for d in s['diagnostics'] if d['reference_matches']>0]
                votes=np.asarray([d['reference_displacement'] for d in accepted])
                events.append(dict(frame=k,seconds=float(t[k]),phase=s['phase'],
                    truth_displacement_m=(xy[k]-start).tolist(),controller_feedback=nav,
                    truth_goal_error_m=float(errors[k]),
                    accepted_disagreement_m=None if not len(votes) else float(np.max(np.linalg.norm(votes-np.median(votes,axis=0),axis=1))),
                    accepted=[dict(agent=d['agent_id'],displacement=d['reference_displacement'],
                        residual_rms=d['reference_residual_rms'],angle=d['reference_angle']) for d in accepted]))
            dump(out/'c_away_stop_audit.json',dict(case=name,events=events,
                statuses=dict(collections.Counter(d['reference_status'] for s in steps for d in s['diagnostics'])),
                max_true_speed_mps=float(speed.max()),
                configured_registration_speed_bound_mps=case['config']['controller']['max_object_speed'],
                frames_true_speed_exceeds_registration_bound=int(np.sum(speed[1:]>case['config']['controller']['max_object_speed'])),
                final_yaw_change_rad=float(yaw[-1]-yaw[0]),first_hold_frame=hold,
                scope='Post-hoc log diagnosis, no new intervention. Low residual does not establish correct correspondence. Speed-bound exceedance is not causal proof; no bounds or stop gates changed.'))
        old=next((x for x in plan['historical_controls'] if x['condition']==case['condition_id']),None)
        if old and case['arm']=='legacy_aggregate':
            origin=ROOT/old['path'];equal,arrays=replay_equal(origin/'replay.npz',folder/'replay.npz')
            previous=read(origin/'paired_metrics.json')['steps'];fields=old_fields_equal(previous,steps);del previous
            checks[name].update(historical_replay_exact=equal,historical_fields_exact=fields,arrays=arrays)
            assert equal and fields,name
        rows.append(row)
        series[name]=dict(time=t.tolist(),xy=(xy-start).tolist(),goal=(goal-start).tolist(),
            error=errors.tolist(),contacts=contacts,estimate_error=selected)
        del steps;gc.collect()
    paired=[]
    for condition in [c['condition'] for c in plan['conditions']]:
        a,b=[next(r for r in rows if r['condition']==condition and r['arm']==arm) for arm in ARMS]
        paired.append(dict(condition=condition,legacy_complete=a['completed'],corrected_complete=b['completed'],
            legacy_endpoint_m=a.get('endpoint_error_m'),corrected_endpoint_m=b.get('endpoint_error_m'),
            endpoint_change_m=None if 'endpoint_error_m' not in a or 'endpoint_error_m' not in b else b['endpoint_error_m']-a['endpoint_error_m'],
            legacy_g500=a['g500'],corrected_g500=b['g500'],
            legacy_completion_seconds=a.get('completion_seconds'),corrected_completion_seconds=b.get('completion_seconds'),
            legacy_hard_safety=a.get('hard_safety','EXECUTION_ERROR'),corrected_hard_safety=b.get('hard_safety','EXECUTION_ERROR')))
    totals={arm:dict(episodes=9,execution_errors=sum(r['execution_status']!='COMPLETED' for r in rows if r['arm']==arm),
        completed=sum(r['completed'] for r in rows if r['arm']==arm),g500=sum(r['g500'] for r in rows if r['arm']==arm),
        hard_safety_cases=sum(bool(r.get('hard_safety','EXECUTION_ERROR')) for r in rows if r['arm']==arm),
        safe_completed=sum(r.get('safe_completed',False) for r in rows if r['arm']==arm)) for arm in ARMS}
    rescued=[r['condition'] for r in paired if not r['legacy_complete'] and r['corrected_complete']]
    lost=[r['condition'] for r in paired if r['legacy_complete'] and not r['corrected_complete']]
    gate=bool(rescued and totals[ARMS[1]]['completed']>totals[ARMS[0]]['completed'] and totals[ARMS[1]]['hard_safety_cases']==0)
    analysis=dict(mission=MISSION,totals=totals,rescued=rescued,lost=lost,predeclared_task_gate=gate,
        episodes=rows,paired=paired,verification=checks,protocol_sha256=sha(protocol),input_sha256=inputs,
        analysis_script_sha256=sha(Path(__file__)),execution_source=manifest['source_files'],
        scope='Nine reused development conditions, one seed; completion proportion is not a reliability probability. No pooling with historical algorithms.',
        plotting='Full saved trajectories; no interpolation, smoothing or extrapolation. Failed cutoff times are not completion times.')
    dump(out/'analysis.json',analysis);dump(out/'plot_data.json',series)
    csv_rows(out/'all_18_episodes.csv',rows);csv_rows(out/'paired_comparison.csv',paired)
    plot(plan,analysis,series,out)
    print(json.dumps(dict(totals=totals,rescued=rescued,lost=lost,gate=gate)),flush=True)

def plot(plan,analysis,series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none'})
    colors=['#0072B2','#D55E00'];labels=['Legacy aggregate','Consistent aggregate'];styles=['--','-']
    conditions=[c['condition'] for c in plan['conditions']]
    fig,axes=plt.subplots(3,3,figsize=(7.15,7.0),layout='constrained')
    for ax,condition in zip(axes.flat,conditions):
        goal=None
        for arm,color,style,label in zip(ARMS,colors,styles,labels):
            s=series.get(condition+'_'+arm)
            if s is None:continue
            xy=np.asarray(s['xy']);goal=np.asarray(s['goal'])
            ax.plot(xy[:,0],xy[:,1],color=color,ls=style,lw=1,label=label)
            ax.scatter(*xy[-1],color=color,s=15,marker='x')
        if goal is None:
            ax.text(.5,.5,'Execution data unavailable',ha='center',transform=ax.transAxes)
            continue
        ax.add_patch(plt.Circle(goal,.12,fill=False,color='0.3',lw=.7))
        ax.scatter(0,0,color='black',s=8);ax.scatter(*goal,marker='+',color='black',s=25)
        ax.set_title(condition.replace('_',' '));ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.15)
        ax.set_xlabel('x displacement (m)');ax.set_ylabel('y displacement (m)')
    handles,legend=axes.flat[0].get_legend_handles_labels();fig.legend(handles,legend,loc='outside lower center',ncol=2)
    fig.suptitle('Complete recorded trajectories: 9 paired development conditions (seed 2)',fontsize=10)
    save(fig,out,'complete_trajectories')
    fig,axes=plt.subplots(3,1,figsize=(7.15,6.1),layout='constrained',sharex=True)
    x=np.arange(9)
    for offset,arm,color,label,marker in zip([-.12,.12],ARMS,colors,labels,['s','o']):
        rows=[next(r for r in analysis['episodes'] if r['condition']==c and r['arm']==arm) for c in conditions]
        axes[0].scatter(x+offset,[r.get('endpoint_error_m',np.nan) for r in rows],c=color,marker=marker,label=label)
        for i,r in enumerate(rows):
            if not r['completed']:axes[0].annotate('F',(x[i]+offset,r.get('endpoint_error_m',0)),xytext=(0,5),textcoords='offset points',ha='center',color=color,fontsize=7)
        for i,r in enumerate(rows):
            axes[1].scatter(x[i]+offset,r.get('seconds',np.nan),c=color,marker=marker if r['completed'] else 'x',s=27)
        axes[2].scatter(x+offset,[r.get('barrier_scalings',np.nan) for r in rows],c=color,marker=marker)
    axes[0].axhline(.12,color='0.4',ls=':',lw=.8);axes[0].set_ylabel('Endpoint error (m)');axes[0].legend(ncol=2)
    axes[1].set_ylabel('Physical duration (s)');axes[1].set_title('x = not completed; duration is not a completion time',fontsize=8)
    axes[2].set_ylabel('Barrier scalings');axes[2].set_xticks(x,conditions,rotation=30,ha='right')
    for ax in axes:ax.grid(alpha=.15,axis='y')
    fig.suptitle('All 18 starts retained; paired development results, not population reliability',fontsize=10)
    save(fig,out,'paired_task_results')
    screen=read(ROOT/'artifacts/research'/MISSION/'screen.json')
    fig,axes=plt.subplots(1,2,figsize=(7.15,2.7),layout='constrained')
    for ax,c in zip(axes,screen['cases']):
        values=[c['old_plane_absolute_error_m']['mean']*1000,c['new_plane_absolute_error_m']['mean']*1000]
        ax.bar(labels,values,color=colors,width=.55)
        for i,y in enumerate(values):ax.text(i,y+2,f'{y:.1f}',ha='center')
        ax.set_ylim(0,max(values)*1.23);ax.set_ylabel('Mean absolute plane-gap error (mm)')
        ax.set_title(c['case'].replace('_',' ')+f"\n{c['records']} repeated agent/frame records",fontsize=8)
    fig.suptitle('Same-input terminal replay (final 10 s); records are not independent trials',fontsize=9)
    save(fig,out,'same_input_plane_error')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);analyze(p.parse_args().out)
