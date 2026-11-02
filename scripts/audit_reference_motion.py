"""Fixed-input motion/stop diagnosis; never advances physics or controls a robot."""
from __future__ import annotations
import argparse
import gc
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
if __name__ == "__main__":
    for _name in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS", "DBACT_CVT_WORKERS"):
        os.environ[_name] = "1"
import numpy as np
from research_io import ROOT, guard, read, sha
from dbact.control.controller import DBACTParams, _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes, environment_versions, verify_source_snapshot

MISSION='2026-11-02-reference-motion-audit-v1'
AUDIT=ROOT/'artifacts/migrations'/MISSION
OUT=ROOT/'runs/maintenance'/MISSION
PUBLIC=ROOT/'docs/results/reference_motion_audit_v1'
PROTOCOL=ROOT/'experiments/ccta2027/reference_motion_audit_v1.json'
ORIGIN=ROOT/'experiments/ccta2027/reference_origin_v1.json'
ORIGIN_ANALYSIS=ROOT/'docs/results/reference_origin_v1/analysis.json'
CF=('recorded','perfect_registration','remove_bootstrap','remove_point_rotation','remove_both')


def event(action,**data):
    with (AUDIT/'operations.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(dict(action=action,**data))+'\n')


def command(label,args,seconds=1800):
    import benchmark_workstation as monitor
    monitor.OUT,monitor.AUDIT=OUT,AUDIT
    result=monitor.run_command(label,args,seconds,monitor.os.environ.copy())
    dump(OUT/(label+'-result.json'),result);event('command',label=label,result=result)
    if result['exit_code']!=0 or result['status']!='COMPLETED':raise SystemExit(result['exit_code'] or 1)


def position_band(case):
    """Resolve the executed config override, not merely the parameter default."""
    return float(case['config']['controller'].get('cross_track_deadband',DBACTParams().cross_track_deadband))


def prepare():
    guard()
    if AUDIT.exists() or PROTOCOL.exists():raise RuntimeError('Existing mission: inspect and continue')
    old=read(ORIGIN);assert encode(source_hashes())==old['frozen_source_sha256']
    assert environment_versions()==old['environment']
    assert shutil.disk_usage(ROOT).free>20*1024**3
    for p in (AUDIT,OUT,PUBLIC):p.mkdir(parents=True,exist_ok=False)
    protected={}
    for name in ['README.md','docs/DATA_INDEX.md','docs/results/README.md','experiments/ccta2027/README.md']:
        target=AUDIT/'backup'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,target);protected[name]=sha(target);assert sha(ROOT/name)==protected[name]
    for name,args in [('worktree.patch',['diff','--binary','--no-ext-diff','--no-textconv']),
                      ('index.patch',['diff','--cached','--binary','--no-ext-diff','--no-textconv']),
                      ('refs.txt',['show-ref']),('status.txt',['status','--porcelain=v1','-uall'])]:
        (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git','--no-optional-locks',*args]))
    batch=ROOT/old['run_dir'];manifest=read(batch/'manifest.json')
    verify_source_snapshot(batch,manifest['source_files'])
    receipts={}
    for case in old['cases']:
        folder=batch/case['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
        receipts[case['case_id']]=sha(folder/'case_result.json')
    bands={position_band(c) for c in old['cases']};assert len(bands)==1
    plan=dict(mission=MISSION,status='FROZEN_BEFORE_NEW_ANALYSIS',source_protocol=str(ORIGIN.relative_to(ROOT)),
        source_protocol_sha256=sha(ORIGIN),source_analysis_sha256=sha(ORIGIN_ANALYSIS),receipt_sha256=receipts,
        source_files=source_hashes(),environment=environment_versions(),
        cases=[c['case_id'] for c in old['cases']],replay_cases=[c['case_id'] for c in old['cases'] if c['stratum']=='far'],
        budget=dict(existing_trajectories=24,far_estimator_replays=6,new_physical_episodes=0,wall_seconds=1800,workers=1,internal_threads=1),
        hypotheses=['Perfect registration alone may leave center-displacement error from bootstrap and material-point rotation.',
                    'Small point-to-plane residual need not imply an accurate center displacement or rigid transform.'],
        counterfactuals=list(CF),
        evaluation='Same original time, accepted voters, trajectory and velocity. Counterfactual position gates only; not new successes or controller policies.',
        probes='First accepted post-ready frame, first BRAKE, first HOLD, last frame, and maximum team registration-remainder norm. No hand-picked replacement.',
        thresholds=dict(identity_atol_m=1e-12,position_band_m=bands.pop(),
                        task_tolerance_m=.12,low_residual='3 * original range_noise_std'),
        decision='Do not combine failed arms, change default, tune gain, or launch new closed-loop episodes in this audit. Design a deployable intervention only after resolving reference semantics.',
        script_sha256=sha(__file__))
    dump(PROTOCOL,plan)
    state=dict(mission=MISSION,status='IN_PROGRESS',stage='FROZEN',protocol_sha256=sha(PROTOCOL),protected_originals=protected,
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),index_sha256=sha(ROOT/'.git/index'),
        manuscript={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()})
    dump(AUDIT/'mission_state.json',state)
    (AUDIT/'CONTINUE.md').write_text('Frozen offline motion audit. No physics run permitted under this identity. Run tests, then analyze and report; preserve all prior results.\n',encoding='utf-8')
    event('prepare',protocol_sha256=sha(PROTOCOL),budget=plan['budget']);print(json.dumps(plan['budget']))


def local_terms(meta,reading,angle,center,birth_center,birth_angle,start):
    """Evaluator-only exact rigid-body accounting of a controller readout."""
    anchor=np.asarray(meta['anchor']);pivot=np.asarray(meta['pivot_shift']);offset=np.asarray(meta['offset'])
    true_angle=float(angle-birth_angle)
    truth=center-start;true_translation=center-birth_center+(rotation(true_angle)-np.eye(2))@(anchor-birth_center)
    point_rotation=(rotation(true_angle)-np.eye(2))@(anchor+pivot-birth_center)
    initial=offset-(birth_center-start)
    fitted_angle=float(reading['reference_angle'])
    fitted_translation=np.array(reading['reference_displacement'])-offset-(rotation(fitted_angle)-np.eye(2))@pivot
    translation_error=fitted_translation-true_translation
    pivot_angle_error=(rotation(fitted_angle)-rotation(true_angle))@pivot
    fit_error=translation_error+pivot_angle_error
    actual=np.array(reading['reference_displacement'])
    assert np.max(np.abs(actual-truth-initial-point_rotation-fit_error))<1e-12
    return dict(initial=initial,point_rotation=point_rotation,fit_error=fit_error,
                translation_error=translation_error,pivot_angle_error=pivot_angle_error,
                fitted_translation=fitted_translation,true_translation=true_translation,true_angle=true_angle,
                angle_error=float(np.arctan2(np.sin(fitted_angle-true_angle),np.cos(fitted_angle-true_angle))),actual=actual)


def counterfactual_readouts(terms,truth):
    actual=np.array([t['actual'] for t in terms]);b=np.array([t['initial'] for t in terms]);r=np.array([t['point_rotation'] for t in terms])
    return dict(recorded=np.median(actual,axis=0),perfect_registration=truth+np.median(b+r,axis=0),
                remove_bootstrap=np.median(actual-b,axis=0),remove_point_rotation=np.median(actual-r,axis=0),
                remove_both=np.median(actual-b-r,axis=0))


def recorded_analysis(case,folder,plan):
    content=read(folder/'paired_metrics.json');steps=content['steps'];meta=content['summary']['reference_initializations']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,xy,angles,start,goal=[z[k].copy() for k in ('times','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start','task/cargo_0/goal_point')]
    ready=read(folder/'case_result.json')['result']['contact_ready_frame']
    birth={aid:int(round(m['time']/case['config']['dt'])) for aid,m in meta.items() if m['initialized']}
    band=position_band(case);tol=plan['thresholds']['task_tolerance_m']
    assert tol==case['config']['task']['tolerance']
    series=[];local_count=low_residual_bad_fit=0;errors=[];angle_errors=[];median_residual=0.
    probes=set();first_hold=None;noise=case['config']['controller']['range_noise_std']
    for k,s in enumerate(steps):
        if s['phase'] in ('BRAKE','HOLD') and (k==0 or steps[k-1]['phase']!=s['phase']):probes.add(k)
        nav=s['navigation_supervisor']
        if ready is None or k<=ready or nav is None:continue
        by_id={d['agent_id']:d for d in s['diagnostics']};terms=[];truth=xy[k]-start
        for aid in nav['voters']:
            d=by_id[aid];assert d['reference_status']=='accepted'
            j=birth[aid];assert abs(times[j]-meta[aid]['time'])<1e-10
            t=local_terms(meta[aid],d,angles[k],xy[k],xy[j],angles[j],start);terms.append(t)
            e=float(np.linalg.norm(t['fit_error']));errors.append(e);angle_errors.append(abs(t['angle_error']))
            local_count+=1;low_residual_bad_fit+=int(d['reference_residual_rms']<=3*noise and e>tol)
        cf=counterfactual_readouts(terms,truth)
        median_residual=max(median_residual,float(np.max(np.abs(cf['recorded']-np.array(nav['displacement'])))))
        error={name:float(np.linalg.norm(v-truth)) for name,v in cf.items()}
        goal_error={name:float(np.linalg.norm(goal-start-v)) for name,v in cf.items()}
        row=dict(frame=k,time_s=float(times[k]),phase=s['phase'],voters=list(nav['voters']),
                 true_goal_error_m=float(np.linalg.norm(goal-xy[k])),error_m=error,goal_error_m=goal_error,
                 position_gate={name:value<=band for name,value in goal_error.items()},
                 signed_error_xy_m={name:(v-truth).tolist() for name,v in cf.items()},
                 registration_remainder_norm_m=float(np.linalg.norm(cf['recorded']-cf['perfect_registration'])))
        series.append(row)
        if first_hold is None and s['phase']=='HOLD':first_hold=row
    assert median_residual<1e-12
    assert series and first_hold is not None
    probes.update([series[0]['frame'],len(steps)-1,max(series,key=lambda s:s['registration_remainder_norm_m'])['frame']])
    stats={name:dict(same_input_MAE_m=float(np.mean([r['error_m'][name] for r in series])),
                     maximum_error_m=max(r['error_m'][name] for r in series),
                     false_position_gate_frames=sum(r['position_gate'][name] and r['true_goal_error_m']>tol for r in series)) for name in CF}
    result=dict(case=case['case_id'],condition=case['condition_id'],arm=case['arm'],frames=len(steps),
        admitted_frames=len(series),accepted_local_readouts=local_count,statistics=stats,first_hold=first_hold,
        low_residual_bad_fit_readouts=low_residual_bad_fit,mean_local_fit_error_m=float(np.mean(errors)),
        median_local_angle_error_deg=float(np.degrees(np.median(angle_errors))),
        maximum_local_fit_error_m=max(errors),maximum_median_residual_m=median_residual,probe_frames=sorted(probes),
        receipt_sha256=sha(folder/'case_result.json'),
        scope='Conditional same original voter/time/trajectory, evaluator-only substitutions; not new closed-loop outcomes.')
    dump(OUT/(case['case_id']+'-timeseries.json'),series)
    del content,steps;gc.collect()
    return result


def replay_inputs(case,folder,audit,deadline):
    """Reconstruct permitted scans; validate every original tracker update exactly."""
    content=read(folder/'paired_metrics.json');steps=content['steps'];meta=content['summary']['reference_initializations']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,robots,xy,angles,start=[z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start')]
    env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller;p=c.params
    assert p.perception_every==1 and p.communication_dropout_prob==0
    birth={aid:int(round(m['time']/case['config']['dt'])) for aid,m in meta.items() if m['initialized']}
    trackers={};accepted=states=counts=0;probes=[]
    for k,s in enumerate(steps):
        if time.perf_counter()>deadline:raise TimeoutError('Fixed offline wall budget reached; no new budget')
        env.cargoes[0].set_pose(xy[k],angles[k])
        for i,agent in enumerate(env.agents):agent.position=robots[k,i].copy()
        scans=[c.sensor.sense_view(agent,env.cargoes,times[k]) for agent in env.agents]
        neighbors=c._neighbor_indices(env.agents)
        for i,agent in enumerate(env.agents):
            d=s['diagnostics'][i];aid=agent.agent_id
            assert d['agent_id']==aid and len(scans[i])==d['own_scan_points'],(k,aid,'scan count')
            counts+=1
            if aid not in birth or k<birth[aid]:continue
            scan,_=_merge_scans(scans,[i]+neighbors[i])
            if k==birth[aid]:
                tr=ObservedReferenceTracker(gate=p.registration_gate,normal_cosine=p.registration_normal_cosine,
                    max_speed=p.max_object_speed,max_yaw_rate=p.max_object_yaw_rate,velocity_filter=p.object_velocity_filter,
                    radial_readout=p.reference_radial_pivot,range_noise_std=p.range_noise_std,innovation_trim=p.reference_innovation_trim)
                tr.initialize(scan,meta[aid]['offset'],times[k]);trackers[aid]=tr;reading=None
                assert tr.initialization_record()==meta[aid],(case['case_id'],k,aid,'actual template')
            else:tr=trackers[aid];reading=tr.update(scan,times[k])
            assert tr.reason==d['reference_status'],(case['case_id'],k,aid,tr.reason,d['reference_status']);states+=1
            if reading is None:continue
            np.testing.assert_array_equal(reading.displacement,d['reference_displacement'])
            np.testing.assert_array_equal(reading.velocity,d['reference_velocity'])
            assert reading.angle==d['reference_angle'] and reading.matches==d['reference_matches']
            assert reading.residual_rms==d['reference_residual_rms'];accepted+=1
            if k in audit['probe_frames']:
                j=birth[aid];t=local_terms(meta[aid],d,angles[k],xy[k],xy[j],angles[j],start)
                n,lever,res=tr._rows(scan,tr.translation,tr.angle)
                design=np.column_stack([n,n[:,1]*lever[:,0]-n[:,0]*lever[:,1]])
                singular=np.linalg.svd(design,compute_uv=False)
                nn,ll,rr=tr._rows(scan,t['true_translation'],t['true_angle'])
                probes.append(dict(frame=k,time_s=float(times[k]),agent=aid,phase=s['phase'],
                    actual_voter=aid in (s['navigation_supervisor'] or {}).get('voters',[]),matches=reading.matches,
                    fitted_rms_m=reading.residual_rms,fit_error_m=float(np.linalg.norm(t['fit_error'])),
                    angle_error_deg=float(np.degrees(t['angle_error'])),
                    true_relative_angle_deg=float(np.degrees(t['true_angle'])),fitted_angle_deg=float(np.degrees(tr.angle)),
                    singular_values_mixed_units=singular.tolist(),rank_at_original_rcond=int(np.count_nonzero(singular>singular[0]*1e-3)),
                    truth_pose_matches=len(nn),truth_pose_rms_m=float(np.sqrt(np.mean(rr**2))) if len(rr) else None,
                    warning='Truth pose is evaluation only. It changes correspondence/gate/trim sets; residual costs do not share identical matched rows. Full rank is not correspondence correctness.'))
        if k%500==0:print(f"Replay {case['case_id']}: {k}/{len(steps)}",flush=True)
    result=dict(case=case['case_id'],status='PASS_EXACT_ESTIMATOR_RECONSTRUCTION',frames=len(steps),scan_counts_checked=counts,
        tracker_statuses_exact=states,accepted_readouts_exact=accepted,actual_templates_exact=len(trackers),probes=probes,
        scope='Reconstructed scans, not original full raw-scan recordings. Original bootstrap schedule/offset replayed; original tracker states/readouts match exactly. No physics or controller step.')
    del env,content,steps;gc.collect()
    return result


def analyze():
    guard();state=read(AUDIT/'mission_state.json');plan=read(PROTOCOL)
    assert sha(PROTOCOL)==state['protocol_sha256']
    amendment=read(AUDIT/'analysis_amendment.json') if (AUDIT/'analysis_amendment.json').exists() else None
    assert sha(__file__)==(amendment['corrected_script_sha256'] if amendment else plan['script_sha256'])
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert sha(ORIGIN)==plan['source_protocol_sha256'] and sha(ORIGIN_ANALYSIS)==plan['source_analysis_sha256']
    if (PUBLIC/'analysis.json').exists():raise RuntimeError('Preserve completed analysis')
    old=read(ORIGIN);batch=ROOT/old['run_dir'];results=[];replays=[]
    deadline=time.perf_counter()+plan['budget']['wall_seconds'];cache=AUDIT/'case_analysis';cache.mkdir(exist_ok=True)
    for case in old['cases']:
        folder=batch/case['case_id'];receipt=read(folder/'case_result.json')
        assert sha(folder/'case_result.json')==plan['receipt_sha256'][case['case_id']]
        cached_result(folder,receipt['identity']);dest=cache/(case['case_id']+'.json')
        if dest.exists():record=read(dest);assert record['script_sha256']==sha(__file__)
        else:
            result=recorded_analysis(case,folder,plan)
            replay=replay_inputs(case,folder,result,deadline) if case['case_id'] in plan['replay_cases'] else None
            record=dict(script_sha256=sha(__file__),result=result,replay=replay);dump(dest,record)
        results.append(record['result'])
        if record['replay']:replays.append(record['replay'])
        print(json.dumps(dict(case=case['case_id'],hold=record['result']['first_hold']['goal_error_m'])),flush=True)
    assert len(results)==24 and len(replays)==6
    report=dict(mission=MISSION,status='COMPLETED_OFFLINE_FIXED_INPUT',new_physical_episodes=0,
        cases=results,replays=replays,protocol_sha256=sha(PROTOCOL),script_sha256=sha(__file__),source_sha256=encode(source_hashes()),
        limitations='24 previously executed trajectories on eight reused blocks. All counterfactuals use evaluator truth and fixed historical voters; none is a deployable candidate, new success, reliability trial or proof of closed-loop improvement.')
    dump(PUBLIC/'analysis.json',report);event('analyze',sha256=sha(PUBLIC/'analysis.json'))
    state.update(stage='ANALYZED');dump(AUDIT/'mission_state.json',state)
    print(json.dumps(dict(cases=24,replays=6,accepted_exact=sum(r['accepted_readouts_exact'] for r in replays))))


def correct_gate():
    """Correct derived flags only, preserving original analysis and all replay evidence."""
    guard();plan=read(PROTOCOL);old=read(ORIGIN);path=PUBLIC/'analysis.json';analysis=read(path)
    if 'reporting_correction' in analysis:raise RuntimeError('Correction already complete')
    assert sha(ORIGIN)==plan['source_protocol_sha256']
    assert source_hashes()==plan['source_files']
    backup=AUDIT/'analysis_v1_default_band';backup.mkdir(exist_ok=False)
    shutil.copy2(path,backup/'analysis.json')
    assert sha(path)==sha(backup/'analysis.json')
    amendment=dict(reason='Reporting bug: default 0.03m used instead of explicit original-case 0.02m band.',
        original_protocol_sha256=sha(PROTOCOL),original_analysis_sha256=sha(path),
        original_script_sha256=plan['script_sha256'],corrected_script_sha256=sha(__file__),
        changed_fields=['position_gate','statistics.*.false_position_gate_frames'],
        unchanged='Raw runs, physical success/safety, all numerical readouts/errors, exact replay validation, and original frozen protocol.',
        resolved_bands={c['case_id']:position_band(c) for c in old['cases']})
    for c in analysis['cases']:
        f=OUT/(c['case']+'-timeseries.json');shutil.copy2(f,backup/f.name);assert sha(f)==sha(backup/f.name)
        series=read(f);band=amendment['resolved_bands'][c['case']];tol=plan['thresholds']['task_tolerance_m']
        for row in series:row['position_gate']={name:value<=band for name,value in row['goal_error_m'].items()}
        c['first_hold']['position_gate']={name:value<=band for name,value in c['first_hold']['goal_error_m'].items()}
        for name in CF:c['statistics'][name]['false_position_gate_frames']=sum(r['position_gate'][name] and r['true_goal_error_m']>tol for r in series)
        # Every numerical estimate, MAE and probe is required to remain unchanged.
        before=read(backup/f.name)
        assert all({k:v for k,v in x.items() if k!='position_gate'}=={k:v for k,v in y.items() if k!='position_gate'} for x,y in zip(before,series))
        dump(f,series)
    assert analysis['replays']==read(backup/'analysis.json')['replays']
    dump(AUDIT/'analysis_amendment.json',amendment)
    analysis['reporting_correction']=amendment;dump(path,analysis)
    event('correct_gate',amendment_sha256=sha(AUDIT/'analysis_amendment.json'),analysis_sha256=sha(path))
    print(json.dumps(dict(status='CORRECTED_DERIVED_FLAGS_ONLY',band=.02,new_simulations=0,new_estimator_replays=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['prepare','analyze','correct-gate'])
    args=p.parse_args()
    if args.action=='prepare':prepare()
    elif args.action=='analyze':analyze()
    else:correct_gate()
