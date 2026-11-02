"""Bounded, opt-in reference correspondence experiment; immutable prior runs."""
from __future__ import annotations
import argparse
import copy
import json
import os
import gc
from collections import Counter
from pathlib import Path
import subprocess
import sys
import time
import benchmark_workstation as monitor
import numpy as np
from scipy.spatial import cKDTree
from validate_transport_prior import ROOT, guard, read, sha, initial_metadata
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result, verify_source_snapshot
from dbact_sim.environment import SimulationEnvironment
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from audit_reference_motion import local_terms

MISSION='2026-11-02-reference-support-v1'
AUDIT=ROOT/'artifacts/migrations'/MISSION
OUT=ROOT/'runs/maintenance'/MISSION
PUBLIC=ROOT/'docs/results/reference_support_v1'
OLD=ROOT/'experiments/ccta2027/reference_origin_v1.json'
PROTOCOL=ROOT/'experiments/ccta2027/reference_support_v1.json'


def event(action,**values):
    with (AUDIT/'operations.jsonl').open('a',encoding='utf-8') as f:
        f.write(json.dumps(dict(action=action,**values))+'\n')


def command(label,args,seconds=1200):
    if (OUT/(label+'-result.json')).exists():raise RuntimeError('Preserve existing command record')
    monitor.OUT,monitor.AUDIT=OUT,AUDIT
    r=monitor.run_command(label,args,seconds,os.environ.copy())
    dump(OUT/(label+'-result.json'),r);event('command',label=label,result=r)
    if r['exit_code']!=0 or r['status']!='COMPLETED':raise SystemExit(r['exit_code'] or 1)


def probes():
    guard();dest=AUDIT/'correspondence_geometry.json'
    if dest.exists():raise RuntimeError('Diagnosis already saved')
    plan=read(OLD);result=[]
    for case in plan['cases']:
        if case['arm']!='reference' or case['stratum']!='far':continue
        folder=ROOT/plan['run_dir']/case['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
        content=read(folder/'paired_metrics.json');steps=content['steps'];metas=content['summary']['reference_initializations']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            times,robots,xy,angles,start=[z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start')]
        env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller
        frame_reasons={}
        for k,step in enumerate(steps):
            d=step['diagnostics'][4];meta=metas[d['agent_id']]
            if d['reference_status']!='accepted':continue
            birth=int(round(meta['time']/case['config']['dt']))
            terms=local_terms(meta,d,angles[k],xy[k],xy[birth],angles[birth],start)
            for threshold in (.12,.5,1.):
                key=f'agent04_first_fit_error_above_{threshold}'
                if np.linalg.norm(terms['fit_error'])>threshold and not any(key in r for r in frame_reasons.values()):
                    frame_reasons.setdefault(k,[]).append(key)
        for k,label in [(next(i for i,s in enumerate(steps) if s['phase']=='HOLD'),'first_HOLD'),(len(steps)-1,'final')]:
            frame_reasons.setdefault(k,[]).append(label)
        for k,reasons in sorted(frame_reasons.items()):
            env.cargoes[0].set_pose(xy[k],angles[k])
            for i,agent in enumerate(env.agents):agent.position=robots[k,i].copy()
            scans=[c.sensor.sense_view(agent,env.cargoes,times[k]) for agent in env.agents];neighbors=c._neighbor_indices(env.agents)
            assert [len(x) for x in scans]==[d['own_scan_points'] for d in steps[k]['diagnostics']]
            for i,d in enumerate(steps[k]['diagnostics']):
                if d['reference_status']!='accepted':continue
                scan,_=_merge_scans(scans,[i]+neighbors[i]);meta=metas[d['agent_id']]
                tr=ObservedReferenceTracker(gate=c.params.registration_gate,normal_cosine=c.params.registration_normal_cosine)
                tr.anchor=np.array(meta['anchor']);tr.reference=np.array(meta['reference_points_relative']);tr.normals=np.array(meta['reference_normals']);tr.tree=cKDTree(tr.reference)
                theta=d['reference_angle'];r=rotation(theta)
                trans=np.array(d['reference_displacement'])-meta['offset']-(r-np.eye(2))@np.array(meta['pivot_shift'])
                distance,idx=tr.tree.query((scan.points-tr.anchor-trans)@r)
                normals=tr.normals[idx]@r.T;lever=tr.reference[idx]@r.T
                keep=(distance<=tr.gate)&(np.einsum('ij,ij->i',normals,scan.normals)>=tr.normal_cosine)
                res=np.einsum('ij,ij->i',normals[keep],scan.points[keep]-tr.anchor-trans-lever[keep])
                n,ll,rr=tr._rows(scan,trans,theta)
                def minor_rms(points):
                    centered=points-points.mean(axis=0)
                    return float(np.sqrt(max(np.linalg.eigvalsh(centered.T@centered/len(points))[0],0.)))
                birth=int(round(meta['time']/case['config']['dt']));terms=local_terms(meta,d,angles[k],xy[k],xy[birth],angles[birth],start)
                result.append(dict(case=case['case_id'],frame=k,reasons=reasons,agent=d['agent_id'],actual_voter=d['agent_id'] in (steps[k]['navigation_supervisor'] or {}).get('voters',[]),
                    scan_points=len(scan),gate_points=int(keep.sum()),retained=len(n),fit_error_m=float(np.linalg.norm(terms['fit_error'])),
                    raw_rms_m=float(np.sqrt(np.mean(res**2))),trimmed_rms_m=float(np.sqrt(np.mean(rr**2))),
                    retained_minor_rms_m=minor_rms(ll),gate_minor_rms_m=minor_rms(lever[keep]),reference_minor_rms_m=minor_rms(tr.reference),
                    gate_normal_condition=tr._conditioning(normals[keep]),retained_normal_condition=tr._conditioning(n),
                    gate_residual_quantiles=np.quantile(res,[0,.1,.5,.9,1]).tolist(),
                    nearest_distance_quantiles=np.quantile(distance,[0,.25,.5,.75,1]).tolist()))
    dump(dest,dict(scope='Post-hoc existing-input diagnosis only; selected thresholds/events identify probes, not admission rules.',source=encode(source_hashes()),records=result))
    for r in result:
        if r['agent']=='agent_04':print(json.dumps(r),flush=True)
    event('diagnose',sha256=sha(dest))


def freeze_offline():
    guard();path=AUDIT/'offline_freeze.json'
    if path.exists():raise RuntimeError('Already frozen')
    assert read(OUT/'targeted-result.json')['exit_code']==0
    plan=read(OLD);cases=[c for c in plan['cases'] if c['arm']=='reference']
    assert len(cases)==8
    batch=ROOT/plan['run_dir'];m=read(batch/'manifest.json');verify_source_snapshot(batch,m['source_files'])
    receipts={}
    for case in cases:
        folder=batch/case['case_id'];r=read(folder/'case_result.json');cached_result(folder,r['identity'])
        receipts[case['case_id']]=sha(folder/'case_result.json')
    record=dict(source_files=source_hashes(),environment=environment_versions(),script_sha256=sha(__file__),
        original_protocol_sha256=sha(OLD),design_sha256=sha(AUDIT/'design.json'),receipts=receipts,
        cases=cases,original_run_dir=plan['run_dir'])
    dump(path,record);event('freeze_offline',sha256=sha(path))
    state=read(AUDIT/'mission_state.json');state.update(stage='OFFLINE_FROZEN');dump(AUDIT/'mission_state.json',state)


def tracker(params,gate):
    return ObservedReferenceTracker(gate=params.registration_gate,normal_cosine=params.registration_normal_cosine,
        max_speed=params.max_object_speed,max_yaw_rate=params.max_object_yaw_rate,velocity_filter=params.object_velocity_filter,
        radial_readout=params.reference_radial_pivot,range_noise_std=params.range_noise_std,
        innovation_trim=params.reference_innovation_trim,support_gate=gate)


def offline_case(case,folder,dest,deadline):
    content=read(folder/'paired_metrics.json');steps=content['steps'];meta=content['summary']['reference_initializations']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,robots,xy,angles,start=[z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start')]
    env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller;p=c.params
    birth={aid:int(round(m['time']/case['config']['dt'])) for aid,m in meta.items() if m['initialized']}
    trackers=[{},{}];n=len(steps);na=len(env.agents)
    positions=np.full((2,n,na,2),np.nan);fitted_angles=np.full((2,n,na),np.nan)
    reasons=np.full((2,n,na),'before_bootstrap',dtype='<U40');support=np.full((n,na),np.nan)
    estimates=np.full((2,n,2),np.nan);velocities=estimates.copy()
    states=exact=count=0;ready=next(k for k,s in enumerate(steps) if s['phase']=='CONTACT_READY')
    for k,step in enumerate(steps):
        if time.perf_counter()>deadline:raise TimeoutError('Frozen offline budget exhausted')
        env.cargoes[0].set_pose(xy[k],angles[k])
        for i,agent in enumerate(env.agents):agent.position=robots[k,i].copy()
        scans=[c.sensor.sense_view(agent,env.cargoes,times[k]) for agent in env.agents];neighbors=c._neighbor_indices(env.agents)
        assert [len(x) for x in scans]==[d['own_scan_points'] for d in step['diagnostics']];count+=na
        votes=[[],[]];speeds=[[],[]]
        for i,agent in enumerate(env.agents):
            aid=agent.agent_id;d=step['diagnostics'][i]
            if aid not in birth or k<birth[aid]:continue
            scan,_=_merge_scans(scans,[i]+neighbors[i])
            for arm in (0,1):
                if k==birth[aid]:
                    tr=tracker(p,bool(arm));tr.initialize(scan,meta[aid]['offset'],times[k]);trackers[arm][aid]=tr;reading=None
                    assert tr.initialization_record()==meta[aid],(case['case_id'],k,aid,'template')
                else:tr=trackers[arm][aid];reading=tr.update(scan,times[k])
                reasons[arm,k,i]=tr.reason
                if arm==1:support[k,i]=tr.support_minor_rms
                if arm==0:
                    assert tr.reason==d['reference_status'],(case['case_id'],k,aid,tr.reason,d['reference_status']);states+=1
                    if reading is not None:
                        np.testing.assert_array_equal(reading.displacement,d['reference_displacement'])
                        np.testing.assert_array_equal(reading.velocity,d['reference_velocity'])
                        assert reading.angle==d['reference_angle'] and reading.matches==d['reference_matches']
                        assert reading.residual_rms==d['reference_residual_rms'];exact+=1
                if reading is not None:
                    positions[arm,k,i]=reading.displacement;fitted_angles[arm,k,i]=reading.angle
                    votes[arm].append(reading.displacement);speeds[arm].append(reading.velocity)
        for arm in (0,1):
            if votes[arm]:estimates[arm,k]=np.median(votes[arm],axis=0);velocities[arm,k]=np.median(speeds[arm],axis=0)
        if k%1000==0:print(f"{case['case_id']} {k}/{n}",flush=True)
    # Truth is used only for sensor reconstruction above, then evaluation here.
    truth=xy[:n]-start;window=np.arange(n)>ready
    available=np.isfinite(estimates).all(axis=2);paired=window&available.all(axis=0)
    base_frames=window&available[0];paired_count=int(paired.sum())
    error=np.linalg.norm(estimates-truth[None,:,:],axis=2)
    task=c.tasks['cargo_0'];target=np.asarray(task.direction)*task.distance
    hold=next(k for k,s in enumerate(steps) if s['phase']=='HOLD');band=p.cross_track_deadband
    stop=dict(frame=hold,time_s=float(times[hold]),true_goal_error_m=float(np.linalg.norm(truth[hold]-target)),
        original_estimated_goal_error_m=float(np.linalg.norm(estimates[0,hold]-target)),
        candidate_estimated_goal_error_m=float(np.linalg.norm(estimates[1,hold]-target)) if available[1,hold] else None,
        candidate_fresh=bool(available[1,hold]),position_band_m=band)
    bad=[0,0];accepted=[0,0]
    for i,agent in enumerate(env.agents):
        aid=agent.agent_id
        if aid not in birth:continue
        j=birth[aid]
        for arm in (0,1):
            for k in np.flatnonzero(window&np.isfinite(positions[arm,:,i]).all(axis=1)):
                d=dict(reference_displacement=positions[arm,k,i],reference_angle=fitted_angles[arm,k,i])
                terms=local_terms(meta[aid],d,angles[k],xy[k],xy[j],angles[j],start)
                accepted[arm]+=1;bad[arm]+=int(np.linalg.norm(terms['fit_error'])>case['config']['task']['tolerance'])
    result=dict(case=case['case_id'],condition=case['condition_id'],frames=n,paired_frames=paired_count,
        availability_relative_to_original=float(paired.sum()/max(base_frames.sum(),1)),
        original_same_input_mae_m=float(error[0,paired].mean()) if paired_count else None,
        candidate_same_input_mae_m=float(error[1,paired].mean()) if paired_count else None,
        original_own_available_mae_m=float(error[0,base_frames].mean()),
        candidate_own_available_mae_m=float(error[1,window&available[1]].mean()) if (window&available[1]).any() else None,
        first_hold=stop,original_local_bad_fit_count=bad[0],candidate_local_bad_fit_count=bad[1],accepted_local_readouts=accepted,
        original_statuses_exact=states,original_readouts_exact=exact,scan_counts_verified=count,
        candidate_status_counts=dict(Counter(reasons[1,window].ravel().tolist())))
    np.savez_compressed(dest.with_suffix('.npz'),time=times[:n],truth=truth,estimates=estimates,velocities=velocities,
        local_displacements=positions,local_angles=fitted_angles,reasons=reasons,support_minor_rms=support)
    result['series_sha256']=sha(dest.with_suffix('.npz'));dump(dest,result)
    print(json.dumps(result),flush=True);del content,steps,env;gc.collect();return result


def offline():
    guard();freeze=read(AUDIT/'offline_freeze.json');design=read(AUDIT/'design.json')
    assert sha(__file__)==freeze['script_sha256'] and source_hashes()==freeze['source_files']
    assert environment_versions()==freeze['environment'] and sha(OLD)==freeze['original_protocol_sha256']
    assert sha(AUDIT/'design.json')==freeze['design_sha256']
    if (AUDIT/'offline_analysis.json').exists():raise RuntimeError('Offline comparison completed already')
    cache=OUT/'offline';cache.mkdir(exist_ok=False);rows=[]
    deadline=time.perf_counter()+design['budget']['offline_wall_seconds'];started=time.perf_counter()
    for case in freeze['cases']:
        folder=ROOT/freeze['original_run_dir']/case['case_id'];r=read(folder/'case_result.json')
        assert sha(folder/'case_result.json')==freeze['receipts'][case['case_id']];cached_result(folder,r['identity'])
        rows.append(offline_case(case,folder,cache/(case['case_id']+'.json'),deadline))
    gates={}
    for row in rows:
        old,new=row['original_same_input_mae_m'],row['candidate_same_input_mae_m'];stop=row['first_hold']
        checks=dict(availability=row['availability_relative_to_original']>=.9,
            same_input_error=new is not None and (new<=.75*old if row['condition']=='far_skew_box_s0_r0_pos0' else new<=old+.01))
        if row['condition']=='far_skew_box_s0_r0_pos0':checks['reject_old_false_stop']=stop['candidate_fresh'] and stop['candidate_estimated_goal_error_m']>stop['position_band_m']
        gates[row['condition']]=checks
    passed=all(all(g.values()) for g in gates.values())
    report=dict(status='PASS' if passed else 'FAIL',cases=rows,gates=gates,new_physical_episodes=0,
        wall_seconds=time.perf_counter()-started,freeze_sha256=sha(AUDIT/'offline_freeze.json'),
        scope='Eight reused trajectories, paired identical sensor inputs but candidate-specific admission and internal estimator state. No new task successes or independent validation.')
    dump(AUDIT/'offline_analysis.json',report);event('offline',status=report['status'])
    state=read(AUDIT/'mission_state.json');state.update(stage='OFFLINE_'+report['status']);dump(AUDIT/'mission_state.json',state)
    print(json.dumps(dict(status=report['status'],gates=gates)),flush=True)


def freeze_closed():
    guard();f=read(AUDIT/'offline_freeze.json');a=read(AUDIT/'offline_analysis.json')
    assert a['status']=='PASS','No closed-loop budget after offline failure'
    assert read(OUT/'checks-result.json')['exit_code']==0 and source_hashes()==f['source_files']
    if PROTOCOL.exists():raise RuntimeError('No refreeze')
    cases=[]
    for old in f['cases']:
        initial=initial_metadata(old['config'],old['seed'])
        assert initial['agent_initial_sha256']==old['expected_initial_state_sha256']
        for arm in ('reference','support_gate'):
            case=copy.deepcopy(old);case['arm']=arm;case['case_id']=case['condition_id']+'_'+arm
            case['config']['controller']['reference_support_gate']=arm=='support_gate'
            case['factors'].update(cohort='reference_support_development',historical_source_case=old['case_id'])
            assert case['frames']==12000 and case['config']['dt']==.05
            assert initial_metadata(case['config'],case['seed'])==initial
            cases.append(case)
    import random
    random.Random(2026110204).shuffle(cases)
    plan=dict(version='reference-support-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,
        run_dir=f'runs/research/{MISSION}/comparison',design_sha256=sha(AUDIT/'design.json'),
        offline_analysis_sha256=sha(AUDIT/'offline_analysis.json'),frozen_source_sha256=encode(source_hashes()),environment=environment_versions())
    dump(PROTOCOL,plan)
    from dbact_sim.experiments.protocol import load_protocol
    load_protocol(PROTOCOL,'development',ROOT)
    dump(AUDIT/'closed_freeze.json',dict(protocol_sha256=sha(PROTOCOL),source_files=source_hashes(),environment=environment_versions()))
    event('freeze_closed',episodes=len(cases))


def run_closed(dry_run=False,resume=False):
    f=read(AUDIT/'closed_freeze.json');plan=read(PROTOCOL)
    assert sha(PROTOCOL)==f['protocol_sha256'] and source_hashes()==f['source_files'] and environment_versions()==f['environment']
    dest=ROOT/plan['run_dir']
    if dest.exists() and not(dry_run or resume):raise RuntimeError('Use reviewed same-identity resume')
    args=[sys.executable,'-m','dbact_sim','study','--protocol',str(PROTOCOL),'--split','development','--workers','4','--out',str(dest)]
    if dry_run:args+=['--dry-run']
    if resume:args+=['--resume']
    command('dry-run' if dry_run else 'resume' if resume else 'comparison',args,120 if dry_run else 7200)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['probes','freeze-offline','offline','freeze-closed','run'])
    p.add_argument('--dry-run',action='store_true');p.add_argument('--resume',action='store_true');args=p.parse_args()
    if args.action=='probes':probes()
    elif args.action=='freeze-offline':freeze_offline()
    elif args.action=='offline':offline()
    elif args.action=='freeze-closed':freeze_closed()
    else:run_closed(args.dry_run,args.resume)
