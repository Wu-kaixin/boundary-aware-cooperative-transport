"""Verify all frozen cases, retain failures, and compare deployment/search arms."""
import argparse
import collections
import gc
import json
from pathlib import Path
import numpy as np
from dbact.cpu_budget import limit_blas_threads
from dbact.cargo import Cargo
from dbact.geometry import signed_distance_to_polygon, rotate
from dbact.metrics import maximum_uncovered_boundary_arc
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result,verify_source_snapshot
from analyze_feedback_components import safety_summary,csv_rows
from analyze_dynamic_contact_study import replay_equal,old_fields_equal
from prepare_cargo_discovery import ROOT,MISSION,sha
from freeze_cargo_discovery import METHODS

def read(p):return json.loads(p.read_text(encoding='utf-8'))

def deployment_snapshot(positions,vertices,radius=.42):
    """Independent 360-sample diagnostic, no runtime control or new pass gate."""
    cargo=Cargo('probe',vertices)
    boundary,_=cargo.boundary_samples(360)
    outside=signed_distance_to_polygon(positions,vertices)>=0.
    covered=np.any(np.linalg.norm(boundary[:,None,:]-positions[None,outside,:],axis=2)<=radius,axis=1) if outside.any() else np.zeros(360,dtype=bool)
    return dict(coverage=float(covered.mean()),**maximum_uncovered_boundary_arc(covered,cargo.perimeter))

def one_case(case,folder,manifest):
    receipt=read(folder/'case_result.json')
    assert receipt['identity']=={**manifest['identity'],'case_sha256':encode(case)}
    r=cached_result(folder,receipt['identity'])
    row=dict(condition=case['condition_id'],shape=case['shape'],arm=case['arm'],method=case['variant'],
        cohort=case['factors']['cohort'],seed=case['seed'],execution_status=r['execution_status'],
        completed=bool(r.get('task_completed')),safe_completed=False,g500=bool(r.get('g500_pass')),
        wall_seconds=r.get('wall_seconds'),error=r.get('error',''))
    if r['execution_status']!='COMPLETED':return row,None,{}
    summary=read(folder/'summary.json'); cargo=summary['cargoes']['cargo_0']
    metrics,hard,verdict=safety_summary(folder)
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,robots,xy,start,goal,local,angles,speed=[z[k].copy() for k in
            ('times','agent_positions','cargo/cargo_0/centers','task/cargo_0/start',
             'task/cargo_0/goal_point','cargo/cargo_0/local_vertices','cargo/cargo_0/angles','cargo/cargo_0/speed')]
    endpoint=float(np.linalg.norm(xy[-1]-goal))
    assert abs(endpoint-r['euclidean_endpoint_error'])<1e-12
    assert r['task_completed']==bool(r['settled'] and endpoint<=case['config']['task']['tolerance'])
    duration=r['termination']['frames_run']*case['config']['dt']
    assert duration<=600. and len(times)==r['termination']['frames_run']+1
    steps=read(folder/'paired_metrics.json')['steps']
    assert len(steps)+1==len(times)
    first_detection=next((k for k,s in enumerate(steps) if any(d['own_scan_points']>0 for d in s['diagnostics'])),None)
    first_contact=next((k for k,s in enumerate(steps) if s['contacts']>0),None)
    first_motion=next((k for k,s in enumerate(steps) if s['true_progress']>1e-9),None)
    ready=r['contact_ready_frame']
    estimate=np.full((len(steps),2),np.nan)
    for k,s in enumerate(steps):
        if s['navigation_supervisor'] is not None:estimate[k]=s['navigation_supervisor']['displacement']
    usable=np.isfinite(estimate).all(axis=1)
    estimate_error=np.linalg.norm(estimate-(xy[:-1]-start),axis=1)
    snapshot_frame=None if ready is None else min(ready+1,len(times)-1)
    snapshot=None if snapshot_frame is None else deployment_snapshot(robots[snapshot_frame],rotate(local,angles[snapshot_frame])+xy[snapshot_frame])
    final_snapshot=deployment_snapshot(robots[-1],rotate(local,angles[-1])+xy[-1])
    if row['completed']:outcome='COMPLETED'
    elif first_detection is None:outcome='NO_DETECTION'
    elif ready is None:outcome='DEPLOYMENT_NOT_READY'
    elif first_contact is None:outcome='NO_ACTUAL_CONTACT'
    elif first_motion is None:outcome='NO_FORWARD_MOTION'
    elif r['settled']:outcome='SETTLED_OFF_TARGET'
    else:outcome='CENSORED_600S_'+steps[-1]['phase']
    row.update(safe_completed=bool(row['completed'] and not hard),endpoint_error_m=endpoint,
        seconds=duration,completion_seconds=duration if row['completed'] else None,
        settled=bool(r['settled']),outcome=outcome,hard_safety='|'.join(hard),
        first_detection_seconds=None if first_detection is None else first_detection*case['config']['dt'],
        first_contact_seconds=None if first_contact is None else first_contact*case['config']['dt'],
        first_forward_motion_seconds=None if first_motion is None else first_motion*case['config']['dt'],
        first_contact_ready_seconds=None if ready is None else ready*case['config']['dt'],
        physical_contact_quorum_frame=r['physical_contact_ready']['frame'],
        final_phase=steps[-1]['phase'],g500_reasons=' | '.join(verdict.reasons),
        max_coverage=cargo['max_strict_coverage'],final_coverage=cargo['final_strict_coverage'],
        ready_coverage=None if snapshot is None else snapshot['coverage'],
        ready_gap_upper_m=None if snapshot is None else snapshot['max_uncovered_arc_upper_m'],
        final_gap_upper_m=final_snapshot['max_uncovered_arc_upper_m'],
        mean_robot_path_m=float(np.linalg.norm(np.diff(robots,axis=0),axis=2).sum(axis=0).mean()),
        final_speed=r['final_speed'],max_penetration_m=metrics['max_penetration'],
        min_separation_m=metrics['min_inter_agent_distance'],barrier_scalings=metrics['barrier_scalings'],
        mean_contacts=r['mean_contacts'],max_cross_track_m=r['max_cross_track'],
        rotation_deg=r['rotation_deg'],navigation_availability=float(usable.mean()),
        navigation_MAE_m=float(estimate_error[usable].mean()) if usable.any() else None,
        final_navigation_error_m=float(estimate_error[-1]) if usable[-1] else None,
        case_path=folder.relative_to(ROOT).as_posix())
    if case['stratum']=='far':
        assert first_detection is None or first_detection>0
        assert all(d['own_scan_points']==0 for d in steps[0]['diagnostics'])
    checks={}
    if case['stratum']=='near' and case['factors']['cohort']=='historical_nine' and case['variant']=='boundary_cvt':
        old=ROOT/'runs/research/2026-11-02-contact-consistency-v1/run'/case['factors']['origin_case']
        old_receipt=read(old/'case_result.json');cached_result(old,old_receipt['identity'])
        equal,arrays=replay_equal(old/'replay.npz',folder/'replay.npz')
        fields=old_fields_equal(read(old/'paired_metrics.json')['steps'],steps)
        checks=dict(historical_replay_exact=equal,historical_fields_exact=fields,arrays=arrays,
                    historical_receipt_sha256=sha(old/'case_result.json'))
        assert equal and fields,case['case_id']
    series=dict(time=times.tolist(),xy=(xy-start).tolist(),goal=(goal-start).tolist(),
                error=np.linalg.norm(xy-goal,axis=1).tolist(),nav_time=times[:-1].tolist(),
                estimated_displacement=estimate.tolist(),navigation_error=estimate_error.tolist(),
                contacts=[s['contacts'] for s in steps])
    del steps;gc.collect()
    return row,series,checks

def analyze(stage,available_only=False):
    limit_blas_threads(1)
    protocol=ROOT/f'experiments/ccta2027/cargo_{stage}_v1.json';plan=read(protocol)
    batch=ROOT/plan['run_dir'];manifest=read(batch/'manifest.json');summary=read(batch/'summary.json')
    if not available_only:
        assert summary['completed_denominator']==len(plan['cases']) and summary['source_integrity']=='PASS'
    verify_source_snapshot(batch,manifest['source_files'])
    assert encode(manifest['source_files'])==plan['frozen_source_sha256']
    cache=ROOT/'artifacts/research'/MISSION/(stage+'-case-analysis')
    cache.mkdir(exist_ok=True)
    out=ROOT/'artifacts/research'/MISSION/(stage+'-analysis')
    if not available_only:out.mkdir(exist_ok=False)
    rows=[];series={};checks={};inputs={}
    for case in plan['cases']:
        folder=batch/case['case_id']
        if available_only and not (folder/'case_result.json').exists():continue
        cached=cache/(case['case_id']+'.json')
        if cached.exists():
            record=read(cached)
            assert record['analysis_script_sha256']==sha(Path(__file__))
            assert record['input_receipt_sha256']==sha(folder/'case_result.json')
            if available_only:continue
            # Final closure rechecks every raw product, not just cached metrics.
            receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
            row,s,check=record['row'],record['series'],record['checks']
        else:
            print('Verify '+case['case_id'],flush=True)
            row,s,check=one_case(case,folder,manifest)
            dump(cached,dict(row=row,series=s,checks=check,
                analysis_script_sha256=sha(Path(__file__)),input_receipt_sha256=sha(folder/'case_result.json')))
        rows.append(row);checks[case['case_id']]=check
        if s is not None:series[case['case_id']]=s
        receipt=read(folder/'case_result.json')
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/name).relative_to(ROOT).as_posix():h for name,h in receipt['products'].items()})
    if available_only:
        print(json.dumps(dict(stage=stage,cached_cases=len(list(cache.glob('*.json'))),planned=len(plan['cases']),
                              status='INTERIM_ANALYSIS_ONLY_NO_PROTOCOL_CHANGE')),flush=True)
        return
    arms=list(dict.fromkeys(c['arm'] for c in plan['cases']))
    totals={};paired=[]
    for cohort in ['all']+list(dict.fromkeys(r['cohort'] for r in rows)):
        totals[cohort]={}
        for arm in arms:
            picked=[r for r in rows if r['arm']==arm and (cohort=='all' or r['cohort']==cohort)]
            totals[cohort][arm]=dict(planned=len(picked),completed=sum(r['completed'] for r in picked),
                safe_completed=sum(r['safe_completed'] for r in picked),g500=sum(r['g500'] for r in picked),
                detected=sum(r.get('first_detection_seconds') is not None for r in picked),
                deployment_ready=sum(r.get('first_contact_ready_seconds') is not None for r in picked),
                actual_contact=sum(r.get('first_contact_seconds') is not None for r in picked),
                execution_errors=sum(r['execution_status']!='COMPLETED' for r in picked),
                hard_safety_cases=sum(bool(r.get('hard_safety','EXECUTION_ERROR')) for r in picked))
    reference='boundary_cvt' if stage=='near' else 'unknown_location'
    for arm in arms:
        if arm==reference:continue
        wins=[];losses=[];pairs=[]
        for meta in plan['conditions']:
            c=meta['condition']
            a=next(r for r in rows if r['condition']==c and r['arm']==reference)
            b=next(r for r in rows if r['condition']==c and r['arm']==arm)
            if a['safe_completed'] and not b['safe_completed']:wins.append(c)
            if b['safe_completed'] and not a['safe_completed']:losses.append(c)
            pair=dict(condition=c,reference_safe_complete=a['safe_completed'],comparison_safe_complete=b['safe_completed'],
                endpoint_difference_m=None if a.get('endpoint_error_m') is None or b.get('endpoint_error_m') is None else a['endpoint_error_m']-b['endpoint_error_m'])
            pairs.append(pair)
        paired.append(dict(comparison_arm=arm,reference_wins=wins,reference_losses=losses,net=len(wins)-len(losses),pairs=pairs))
    gate=(all(p['net']>=3 for p in paired) and totals['all'][reference]['hard_safety_cases']==0) if stage=='near' else None
    result=dict(stage=stage,totals=totals,episodes=rows,paired=paired,development_advantage_gate=gate,
        verification=checks,protocol_sha256=sha(protocol),source_sha256=manifest['identity']['source_sha256'],
        analysis_script_sha256=sha(Path(__file__)),input_sha256=inputs,
        scope='Fixed development conditions, one seed; no inference of arbitrary-shape reliability',
        coverage='360 equal arc samples, .42m observer radius, outside robot centers only; ready snapshot is post-hold frame ready+1, not a guarantee',
        timing='Navigation is pre-physics at t[k]; compare xy[k], not xy[k+1]. Contacts are original start-of-hold reports.',
        statistics='Designed paired cases; no per-frame confidence interval or reliability probability')
    dump(out/'analysis.json',result);dump(out/'plot_data.json',series)
    csv_rows(out/'all_episodes.csv',rows)
    print(json.dumps(dict(stage=stage,totals=totals,gate=gate)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['near','far'])
    p.add_argument('--available',action='store_true');args=p.parse_args();analyze(args.stage,args.available)
