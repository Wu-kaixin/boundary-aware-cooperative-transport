"""Read frozen logs to locate feedback, pusher-loss and actual-contact events.

This is offline evaluation: physical poses/forces are never sent to a controller.
No simulation step, tuning, or rewriting of an original product occurs.
"""
from __future__ import annotations
import argparse
import gc
import json
from pathlib import Path
import numpy as np
from dbact.control.controller import DBACTParams
from dbact.control.phase import PhaseMonitor, PhaseSignals
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result, verify_source_snapshot
from analyze_dynamic_contact_study import sha, read

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-01-feedback-components-v1'
HISTORY=ROOT/'runs/research/2026-11-01-displacement-stop-v1/run'


def extract(case, folder, receipt, *, reconstruct_reference=False, steps=None):
    if steps is None:steps=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times=z['times'].copy();xy=z['cargo/cargo_0/centers'].copy()
        start=z['task/cargo_0/start'].copy();goal=z['task/cargo_0/goal_point'].copy()
    cfg=case['config'];params=DBACTParams(**cfg['controller']);monitor=PhaseMonitor(params.phase_gates())
    dt=cfg['dt'];last_pushers=0;previous=(None,False);states={};events=[];a=[]
    max_speed_residual=0.;verified_speed=0;phase_exact=True;lag_exact=True
    for k,s in enumerate(steps):
        t=float(times[k]);guards=s['phase_guards'];response=s['phase_response'];diags=s['diagnostics']
        # observe(frame) labels the completed hold 1..N; controller used 0..N-1.
        if s['frame'] != k+1:raise RuntimeError('Unexpected recorded hold index')
        monitor.update(PhaseSignals(**guards),s['frame']-1)
        phase_exact &= monitor.as_dict()==response
        lag_exact &= guards['transport_active']==last_pushers
        pushers=[d['agent_id'] for d in diags if d['push_side']]
        last_pushers=len(pushers)
        velocity={};displacement={}
        for d in diags:
            aid=d['agent_id']
            if reconstruct_reference and d['reference_status']=='bootstrap_only':
                states[aid]=(np.zeros(2),np.zeros(2),t)
            if reconstruct_reference and d['reference_matches']>0:
                old,v0,t0=states[aid]
                translation=np.asarray(d['reference_displacement'])-d['reference_bootstrap_offset']
                v=(1-params.object_velocity_filter)*v0+params.object_velocity_filter*(translation-old)/(t-t0)
                states[aid]=(translation,v,t)
                velocity[aid]=v;displacement[aid]=d['reference_displacement']
        if reconstruct_reference and velocity:
            speed=float(np.linalg.norm(np.median(list(velocity.values()),axis=0)))
            max_speed_residual=max(max_speed_residual,abs(speed-guards['estimated_speed']));verified_speed+=1
        else:
            velocity={aid:np.asarray(v) for aid,v in s['estimated_velocities'].items()}
        goal_error=goal-xy[k]
        correction=goal_error/max(float(np.linalg.norm(goal_error)),1e-12)
        forces=np.asarray(s['contact_forces'],dtype=float).reshape(-1,2)
        useful=int(np.count_nonzero(forces@correction>1e-9))
        opposing=int(np.count_nonzero(forces@correction< -1e-9))
        current=(s['phase'],response['push_suspended'])
        row=dict(time_s=t,phase=s['phase'],push_suspended=bool(response['push_suspended']),
            actual_contacts=s['contacts'],positive_task_contacts=s['positive_contacts'],
            positive_goal_correction_contacts=useful,opposing_goal_correction_contacts=opposing,
            selected_pushers=len(pushers),previous_pushers_signal=guards['transport_active'],
            ready_agents=sum(d['contact_ready'] for d in diags),
            reported_boundary_gap_mean=float(np.mean([d['boundary_distance'] for d in diags if d['boundary_distance'] is not None])) if any(d['boundary_distance'] is not None for d in diags) else None,
            estimated_speed_m_s=guards['estimated_speed'],physical_speed_after_hold_m_s=float(np.linalg.norm(s['true_velocity'])),
            physical_goal_error_before_hold_m=float(np.linalg.norm(goal_error)),
            estimated_remaining_before_hold_m=guards['target_distance']-guards['progress'],
            contact_force_toward_true_goal_N=float(forces.sum(axis=0)@correction) if len(forces) else 0.,
            active_effort_sum=float(sum(d['effort'] for d in diags if d['push_side'])),
            blocked_pushers=sum(d['push_side'] and d['safety_feedback_blocked'] for d in diags),
            fresh_reference_readouts=sum(d['reference_matches']>0 for d in diags),
            local_holding=sum(d['local_holding'] for d in diags))
        if current!=previous:
            events.append(dict(**row,brake_reason=response['brake_reason'],pushers=pushers,
                physical_contact_ids=s['contact_ids'],loss_streak=monitor.loss_streak,
                pause_frames=monitor.pause_frames,regain_streak=monitor.regain_streak))
            previous=current
        a.append(row)
    if not phase_exact or not lag_exact:raise RuntimeError(f'Phase/log time alignment mismatch: {case["case_id"]}')
    if reconstruct_reference and max_speed_residual>1e-10:
        raise RuntimeError(f'Reconstructed selected velocity does not match logged supervisor: {max_speed_residual}')
    # Fixed windows describe event neighborhoods; no new scientific success rule.
    windows={}
    keys=['selected_pushers','ready_agents','actual_contacts','positive_goal_correction_contacts',
          'estimated_speed_m_s','physical_speed_after_hold_m_s','active_effort_sum','blocked_pushers',
          'physical_goal_error_before_hold_m','estimated_remaining_before_hold_m','fresh_reference_readouts']
    boundaries=[('0_to_10',0,10),('10_to_20',10,20),('20_to_30',20,30),('last_10',max(0,times[-1]-10),times[-1])]
    for name,lo,hi in boundaries:
        subset=[r for r in a if lo<=r['time_s']<hi]
        windows[name]=dict(start_s=float(lo),end_s=float(hi),frames=len(subset),
            **{key:float(np.mean([r[key] for r in subset])) if subset else None for key in keys})
    sustained=[];streak=0
    for r in a:
        streak=streak+1 if r['positive_goal_correction_contacts']==0 and r['phase'] in ('TRANSPORT','BRAKE') else 0
        if streak==params.phase_gates().contact_loss_dwell:
            sustained.append(r['time_s']-(streak-1)*dt)
    result=receipt['result']
    summary=dict(case_id=case['case_id'],events=events,windows=windows,
        first_zero_useful_contact_run_s=sustained[0] if sustained else None,
        zero_useful_contact_run_definition='No positive projection of actual contact force toward evaluator goal for the unchanged contact_loss_dwell frames; descriptive, not a controller guard',
        phase_response_exact=phase_exact,transport_active_is_previous_diagnostics_exact=lag_exact,
        reference_velocity_reconstructed_frames=verified_speed,reference_velocity_norm_max_residual=max_speed_residual,
        task_completed=result.get('task_completed'),endpoint_error_m=result.get('euclidean_endpoint_error'),
        g500=result.get('g500_pass'),final_phase_response=steps[-1]['phase_response'],
        alignment='row time is pre-control; force is start-of-control-hold contact; true_velocity is post-physics; phase active count comes from previous diagnostics',
        limits='Designated pushers are not actual contacts. Force report is before ground friction. Time ordering is not causal attribution. Reconstructed reference velocity uses saved readouts and has floating-point roundoff; not a new sensor recording.')
    del steps
    return summary,a


def run(out):
    out=out.resolve()
    if not out.is_relative_to(ROOT/'artifacts/research'/MISSION):raise ValueError('Output outside mission')
    out.mkdir(parents=True,exist_ok=False)
    manifest=read(HISTORY/'manifest.json');verify_source_snapshot(HISTORY,manifest['source_files'])
    plan=read(HISTORY/'resolved_protocol.json')
    selected=[c for c in plan['cases'] if c['condition_id'] in ('c_open_away','circle','ellipse_parallel','ellipse_perpendicular') and c['arm'] in ('vector_cvt','reference_cvt')]
    reports=[];series={};inputs={}
    for case in selected:
        print('Read '+case['case_id'],flush=True);folder=HISTORY/case['case_id'];receipt=read(folder/'case_result.json')
        cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        report,rows=extract(case,folder,receipt,reconstruct_reference=case['arm']=='reference_cvt')
        reports.append(report);series[case['case_id']]=rows;gc.collect()
        print(json.dumps({'case':case['case_id'],'events':len(report['events']),
            'brake_reason':report['final_phase_response']['brake_reason'],
            'speed_residual':report['reference_velocity_norm_max_residual']}),flush=True)
    dump(out/'diagnosis.json',dict(mission=MISSION,cases=reports,inputs=inputs,script_sha256=sha(Path(__file__)),
        kind='OFFLINE_EXISTING_LOGS_NO_NEW_PHYSICS'))
    dump(out/'timeseries.json',series)
    plot(series,out)


def plot(series,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axs=plt.subplots(4,1,figsize=(7.16,7),sharex=True,layout='constrained')
    for suffix,color,style,label in [('vector_cvt','#009E73','--','Previous vector-CVT'),('reference_cvt','#CC79A7','-','Reference position + velocity')]:
        rows=series['c_open_away_'+suffix];time=np.array([r['time_s'] for r in rows]);keep=time<=40
        for ax,key in zip(axs,['estimated_speed_m_s','selected_pushers','actual_contacts','physical_goal_error_before_hold_m']):
            y=np.array([r[key] for r in rows]);ax.plot(time[keep],y[keep],color=color,ls=style,lw=.8,label=label)
        first=[r['time_s'] for r in rows if r['phase']=='BRAKE']
        if first and first[0]<=40:
            for ax in axs:ax.axvline(first[0],color=color,ls=':',lw=.7)
    for ax,label in zip(axs,['Estimated speed (m/s)','Designated pushers','Actual contacts','Physical goal error (m)']):ax.set_ylabel(label);ax.grid(alpha=.2)
    axs[0].legend(fontsize=7);axs[-1].set_xlabel('Physical time before control (s)');axs[-1].set_xlim(0,40)
    fig.suptitle('C-away contact-loss timing; first 40 s of saved episodes; dotted lines = BRAKE',fontsize=9)
    for ext in ('pdf','svg','png'):fig.savefig(out/f'contact_loss_timing.{ext}',dpi=220)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--plot-existing',type=Path)
    args=p.parse_args()
    if args.plot_existing:
        out=args.out.resolve();source=args.plot_existing.resolve()
        allowed=ROOT/'artifacts/research'/MISSION
        if not out.is_relative_to(allowed) or not source.is_relative_to(allowed):raise ValueError('Paths outside mission')
        out.mkdir(parents=True,exist_ok=False)
        plot(read(source),out)
        dump(out/'figure_provenance.json',dict(input=source.relative_to(ROOT).as_posix(),input_sha256=sha(source),
            script_sha256=sha(Path(__file__)),processing='Saved rows, first 40 physical seconds only, no smoothing or interpolation. Events outside the shown interval are not drawn. Full-episode analysis is unchanged.'))
    else:
        run(args.out)
