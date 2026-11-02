"""Offline six-case audit of contact geometry, role changes and reference-point bias.

No physics steps or control decisions are executed. Saved poses only feed the
historical sensor and the independent geometric evaluator. Fresh scans are
reconstructions, not original recordings. One-second scan audits are sampled;
all saved scientific log frames enter the remaining statistics.
"""
from __future__ import annotations
import argparse,collections,gc,json,sys
from pathlib import Path
import numpy as np
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import rotation
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.study import cached_result,source_hashes,verify_source_snapshot
from dbact_sim.experiments.deployment import dump,encode
from analyze_dynamic_contact_study import read,sha

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-02-contact-geometry-v1'
OLD=ROOT/'runs/research/2026-11-01-contact-reinforcement-v1/run'


def stats(values):
    x=np.asarray(values,float);x=x[np.isfinite(x)]
    return dict(n=int(len(x)),mean=float(x.mean()) if len(x) else None,
        median=float(np.median(x)) if len(x) else None,p95=float(np.percentile(x,95)) if len(x) else None,
        max=float(x.max()) if len(x) else None)


def angle(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if not np.isfinite(a).all() or not np.isfinite(b).all():return np.nan
    return float(np.degrees(np.arccos(np.clip(a@b/max(np.linalg.norm(a)*np.linalg.norm(b),1e-12),-1,1))))


def audit_case(case):
    name=case['case_id'];folder=OLD/name;print('Audit '+name,flush=True)
    receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
    steps=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        saved={k:z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start','task/cargo_0/goal_point')}
    env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller;cargo=env.cargoes[0]
    p=c.params;positions=saved['agent_positions'];centers=saved['cargo/cargo_0/centers'];yaw=saved['cargo/cargo_0/angles']
    start=saved['task/cargo_0/start'];goal=saved['task/cargo_0/goal_point'];t=saved['times'][:-1];n=len(steps)
    ids=[a.agent_id for a in env.agents];na=len(ids)
    boot={};anchors={};pivots={};boot_yaw={};pivot_records={}
    for k,s in enumerate(steps):
        for i,d in enumerate(s['diagnostics']):
            if d['reference_status']=='bootstrap_only' and i not in boot:boot[i]=k
    sample=set(range(0,n,20));sensor_frames=sample|set(boot.values())
    counts=collections.Counter();force=collections.Counter();changes=collections.Counter()
    fused_gap_error=[];fused_normal_error=[];fresh_gap_error=[];fresh_normal_error=[]
    fresh_compare=[];flip_rows=[];old_error=[];pivot_error=[];oracle_pivot_error=[]
    reference_yaw_error=[];frame_series=[];last_normal={};last_direction={}
    for k,s in enumerate(steps):
        cargo.set_pose(centers[k],yaw[k])
        signed,true_n,_=cargo.signed_distance(positions[k])
        true_dir=goal-centers[k];true_dir/=max(np.linalg.norm(true_dir),1e-12)
        desired_forces=[float(np.asarray(f)@true_dir) for f in s['contact_forces']]
        force['physical_contact_samples']+=len(desired_forces)
        force['force_positive_toward_goal_samples']+=sum(f>1e-9 for f in desired_forces)
        force['force_opposing_goal_samples']+=sum(f< -1e-9 for f in desired_forces)
        force['net_force_below_breakaway_frames']+=np.linalg.norm(s['actual_net_force']) < cargo.mass*case['config']['transport']['ground_friction']*case['config']['transport']['gravity']
        if k in sensor_frames:
            for i,a in enumerate(env.agents):a.position=positions[k,i].copy()
            scans=[c.sensor.sense_view(a,env.cargoes,float(t[k])) for a in env.agents]
            assert [len(v) for v in scans]==[d['own_scan_points'] for d in s['diagnostics']]
            counts['reconstructed_scan_count_frames_exact']+=1
            for i,b in boot.items():
                if b!=k:continue
                neighbours=[j for j in range(na) if j!=i and np.linalg.norm(positions[k,i]-positions[k,j])<=p.comm_range]
                merged,_=_merge_scans(scans,[i]+neighbours)
                a=merged.points.mean(axis=0);lever=merged.points-a
                turn=merged.normals[:,1]*lever[:,0]-merged.normals[:,0]*lever[:,1]
                q=np.linalg.lstsq(merged.normals,turn,rcond=1e-3)[0]
                pivot=a+np.array([q[1],-q[0]])
                residual=float(np.sqrt(np.mean((turn-merged.normals@q)**2)))
                anchors[i]=a;pivots[i]=pivot;boot_yaw[i]=yaw[k]
                pivot_records[ids[i]]=dict(frame=k,anchor=a.tolist(),normal_concurrence_pivot=pivot.tolist(),
                    concurrence_rms_m=residual,existing_sensor_residual_tolerance_m=p.residual_tolerance,
                    admitted_by_existing_noise_scale=bool(residual<=p.residual_tolerance),
                    oracle_center_offset_m=float(np.linalg.norm(pivot-centers[k])))
        original=[];corrected=[];oracle=[]
        physical_ids=set(s['contact_ids'])
        for i,d in enumerate(s['diagnostics']):
            aid=ids[i];normal=np.array(d['contact_selected_normal'],float)
            error=np.array(d['navigation_error'],float);direction=error/max(np.linalg.norm(error),1e-12)
            estimated_distance=d['boundary_distance']
            if d['push_side']:
                counts['designated_pusher_samples']+=1
                counts['designated_without_physical_contact']+=aid not in physical_ids
                counts['designated_geometrically_outside_contact']+=signed[i]>p.robot_radius
                if estimated_distance is not None:
                    fused_gap_error.append(estimated_distance-signed[i])
                    counts['map_implies_contact_but_true_gap_outside']+=estimated_distance<=p.robot_radius and signed[i]>p.robot_radius
                if np.isfinite(normal).all():
                    err=angle(normal,true_n[i]);fused_normal_error.append(err)
                    counts['designated_normal_error_gt90']+=err>90
                    counts['designated_normal_error_gt30']+=err>30
                gap=d['reinforcement_gap']
                if gap is not None:counts['press_floor_but_not_contact']+=gap<=c._press_floor+1e-6 and signed[i]>p.robot_radius
            if k in sample and len(scans[i]) and d['contact_ready']:
                view=scans[i];j=int(np.linalg.norm(view.points-positions[k,i],axis=1).argmin())
                own_normal=view.normals[j];distance=float(np.linalg.norm(view.points[j]-positions[k,i]))
                fresh_gap_error.append(distance-signed[i]);fresh_normal_error.append(angle(own_normal,true_n[i]))
                fresh_compare.append(dict(frame=k,agent=aid,designated=d['push_side'],
                    map_normal_error_deg=angle(normal,true_n[i]),fresh_normal_error_deg=angle(own_normal,true_n[i]),
                    map_distance_error_m=None if estimated_distance is None else estimated_distance-signed[i],
                    fresh_distance_error_m=distance-signed[i]))
            if 'direction_no_longer_useful' in d['reinforcement_event']:
                dn=angle(last_normal.get(aid,[np.nan]*2),normal);dd=angle(last_direction.get(aid,[np.nan]*2),direction)
                changes['events']+=1
                if np.isfinite(dn) and np.isfinite(dd):
                    changes['both_angles_available']+=1;changes['normal_change_larger']+=dn>dd
                    changes['direction_change_larger']+=dd>dn
                flip_rows.append(dict(frame=k,agent=aid,normal_change_deg=dn,direction_change_deg=dd,
                    current_normal_truth_error_deg=angle(normal,true_n[i]),physical_contact=aid in physical_ids))
            last_normal[aid]=normal;last_direction[aid]=direction
            if d['reference_matches']>0:
                assert i in anchors
                raw=np.array(d['reference_displacement']);r=rotation(d['reference_angle'])
                original.append(raw)
                delta=(r-np.eye(2))@(pivots[i]-anchors[i])
                corrected.append(raw+delta if pivot_records[aid]['admitted_by_existing_noise_scale'] else raw)
                oracle.append(raw+(r-np.eye(2))@(centers[boot[i]]-anchors[i]))
                reference_yaw_error.append(abs(float(np.angle(np.exp(1j*(d['reference_angle']-(yaw[k]-boot_yaw[i])))))))
        if original:
            truth=centers[k]-start
            values=[float(np.linalg.norm(np.median(v,axis=0)-truth)) for v in (original,corrected,oracle)]
            old_error.append(values[0]);pivot_error.append(values[1]);oracle_pivot_error.append(values[2])
            frame_series.append(dict(frame=k,time=float(t[k]),original_error_m=values[0],normal_pivot_readout_error_m=values[1],
                oracle_lever_arm_error_m=values[2]))
        if k%4000==0:print(name+' frame '+str(k),flush=True)
    # Paired fresh/map errors use exactly the same audited records with finite map normals.
    paired=[x for x in fresh_compare if np.isfinite(x['map_normal_error_deg'])]
    result=dict(case=name,frames=n,counts={k:int(v) for k,v in counts.items()},forces={k:int(v) for k,v in force.items()},
        role_exit_changes={k:int(v) for k,v in changes.items()},
        designated_map_distance_error_m=stats(fused_gap_error),designated_map_normal_error_deg=stats(fused_normal_error),
        sampled_fresh_distance_error_m=stats(fresh_gap_error),sampled_fresh_normal_error_deg=stats(fresh_normal_error),
        sampled_paired_map_normal_error_deg=stats([v['map_normal_error_deg'] for v in paired]),
        sampled_paired_fresh_normal_error_deg=stats([v['fresh_normal_error_deg'] for v in paired]),
        original_reference_error_m=stats(old_error),normal_pivot_readout_error_m=stats(pivot_error),
        oracle_lever_arm_error_m=stats(oracle_pivot_error),reference_yaw_error_rad=stats(reference_yaw_error),
        pivots=pivot_records,receipt_sha256=sha(folder/'case_result.json'),
        products=receipt['products'],physical_success=receipt['result']['task_completed'])
    del steps;gc.collect()
    return result,dict(fresh_scan_samples=fresh_compare,role_exits=flip_rows,readout_series=frame_series)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    limit_blas_threads(1)
    plan=read(OLD/'resolved_protocol.json');manifest=read(OLD/'manifest.json')
    verify_source_snapshot(OLD,manifest['source_files'])
    assert source_hashes()==manifest['source_files'],'Audit before changing runtime'
    out=args.out.resolve()
    if not out.is_relative_to(ROOT/'artifacts/research'/MISSION):raise ValueError('Output outside mission')
    out.mkdir(exist_ok=False)
    reports=[]
    for case in plan['cases']:
        if case['arm'] not in ('reference','reinforcement'):continue
        result,details=audit_case(case);reports.append(result)
        dump(out/(case['case_id']+'.json'),dict(summary=result,details=details))
        print(json.dumps({k:result[k] for k in ('case','counts','role_exit_changes','sampled_paired_map_normal_error_deg','sampled_paired_fresh_normal_error_deg','original_reference_error_m','normal_pivot_readout_error_m','oracle_lever_arm_error_m')}),flush=True)
    dump(out/'summary.json',dict(cases=reports,source_sha256=encode(source_hashes()),script_sha256=sha(Path(__file__)),
        scope='Full existing six-case logs and all agents; fresh-scan geometry sampled every 20 frames plus bootstrap construction. Sensor return counts verified at those frames. Not an exact replay of every raw scan.',
        limits='Independent evaluator uses truth; candidate readout correction uses observed normals only. The oracle lever arm is diagnostic and forbidden in control. Normal-pivot metrics are same-input readout analyses, not a new closed-loop outcome. No new physics.'))
