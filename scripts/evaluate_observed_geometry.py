"""Fixed offline screen of two geometry readouts, without new physics steps."""
import argparse, collections, gc, json, sys
from pathlib import Path
import numpy as np
from dbact.control.controller import _merge_scans
from dbact.control.observed_geometry import radial_pivot, patch_distance
from dbact.control.observed_reference import rotation
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import source_hashes
from diagnose_contact_geometry import ROOT, MISSION, OLD, read, sha, stats


def evaluate(case, out):
    name=case['case_id']; print(name,flush=True)
    steps=read(OLD/name/'paired_metrics.json')['steps']
    with np.load(OLD/name/'replay.npz',allow_pickle=False) as z:
        poses=z['agent_positions'];centers=z['cargo/cargo_0/centers'];yaw=z['cargo/cargo_0/angles']
        times=z['times'];start=z['task/cargo_0/start']
    env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller;p=c.params
    boot={};pivots={};records={};counts=collections.Counter();samples=[];series=[]
    for k,s in enumerate(steps):
        for i,d in enumerate(s['diagnostics']):
            if d['reference_status']=='bootstrap_only' and i not in boot:boot[i]=k
    frames=set(range(0,len(steps),20))|set(boot.values())
    for k,s in enumerate(steps):
        if k in frames:
            for i,a in enumerate(env.agents):a.position=poses[k,i].copy()
            env.cargoes[0].set_pose(centers[k],yaw[k])
            scans=[c.sensor.sense_view(a,env.cargoes,float(times[k])) for a in env.agents]
            assert [len(v) for v in scans]==[d['own_scan_points'] for d in s['diagnostics']]
            for i,b in boot.items():
                if b!=k:continue
                neighbours=[j for j in range(len(scans)) if i!=j and np.linalg.norm(poses[k,i]-poses[k,j])<=p.comm_range]
                merged,_=_merge_scans(scans,[i]+neighbours)
                pivot,status,rms=radial_pivot(merged.points,p.range_noise_std)
                anchor=merged.points.mean(axis=0)
                pivots[i]=np.zeros(2) if pivot is None else pivot-anchor
                records[str(i)]=dict(status=status,rms=rms,pivot=None if pivot is None else pivot.tolist(),
                    anchor=anchor.tolist(),truth_center_error=None if pivot is None else float(np.linalg.norm(pivot-centers[k])))
            if k%20==0:
                signed,_,closest=env.cargoes[0].signed_distance(poses[k])
                for i,d in enumerate(s['diagnostics']):
                    normal=np.asarray(d['contact_selected_normal']);scan=scans[i]
                    if not d['push_side'] or not np.isfinite(normal).all() or not len(scan):continue
                    # Original nearest map position was not saved. Use own nearest
                    # point to locate the patch in this screen; runtime uses map point.
                    j=int(np.linalg.norm(scan.points-poses[k,i],axis=1).argmin())
                    distance,status,n=patch_distance(scan,poses[k,i],scan.points[j],normal,'cargo_0',
                        2*p.voxel_size,p.residual_tolerance)
                    counts[status]+=1
                    if distance is not None:
                        samples.append(dict(frame=k,agent=i,patch_gap_error_m=distance-signed[i],
                            map_euclidean_gap_error_m=d['boundary_distance']-signed[i],
                            fresh_nearest_gap_error_m=np.linalg.norm(scan.points[j]-poses[k,i])-signed[i],points=n))
        original=[];corrected=[]
        for i,d in enumerate(s['diagnostics']):
            if d['reference_matches']>0:
                raw=np.asarray(d['reference_displacement']);original.append(raw)
                corrected.append(raw+(rotation(d['reference_angle'])-np.eye(2))@pivots[i])
        if original:
            truth=centers[k]-start
            series.append(dict(frame=k,original_m=float(np.linalg.norm(np.median(original,axis=0)-truth)),
                candidate_m=float(np.linalg.norm(np.median(corrected,axis=0)-truth))))
    summary=dict(case=name,pivots=records,patch_admission=dict(counts),
        original_error_m=stats([s['original_m'] for s in series]),candidate_error_m=stats([s['candidate_m'] for s in series]),
        patch_abs_error_m=stats([abs(s['patch_gap_error_m']) for s in samples]),
        map_abs_error_m=stats([abs(s['map_euclidean_gap_error_m']) for s in samples]),
        fresh_abs_error_m=stats([abs(s['fresh_nearest_gap_error_m']) for s in samples]))
    dump(out/(name+'.json'),dict(summary=summary,samples=samples,series=series));del steps;gc.collect()
    print(json.dumps({k:v for k,v in summary.items() if k!='pivots'}),flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True);a=parser.parse_args()
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    out=a.out.resolve();assert out.is_relative_to(ROOT/'artifacts/research'/MISSION);out.mkdir(exist_ok=False)
    limit_blas_threads(1);source=source_hashes()
    reports=[evaluate(c,out) for c in read(OLD/'resolved_protocol.json')['cases'] if c['arm'] in ('reference','reinforcement')]
    assert source_hashes()==source
    dump(out/'summary.json',dict(cases=reports,source=source,script_sha256=sha(Path(__file__)),
        scope='Readout only; no new physics. Full-frame radial correction on identical admitted readings; own scan patch sampled every 20 frames. Patch neighborhood uses fresh nearest point because historical map points were not saved. Runtime validation is still required. Truth is evaluation only.'))
