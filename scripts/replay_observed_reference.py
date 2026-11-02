"""One frozen observed-reference candidate on verified reconstructed inputs."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import numpy as np
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker
from dbact.cpu_budget import limit_blas_threads
from dbact.types import BoundaryView
from dbact_sim.experiments.deployment import dump
from audit_displacement_replay import ROOT, MISSION, HISTORY, read, sha


def replay(folder, out, recover_report=False):
    folder, out = folder.resolve(), out.resolve()
    if not folder.is_relative_to((ROOT/'artifacts/research'/MISSION).resolve()):
        raise ValueError('Mission input required')
    if not out.resolve().is_relative_to((ROOT/'artifacts/research'/MISSION).resolve()):
        raise ValueError('Mission output required')
    if recover_report:
        if not (out/'reference_readout.npz').is_file() or (out/'offline_gate.json').exists():
            raise RuntimeError('Recovery needs a completed saved replay and no existing report')
    else:
        out.mkdir(parents=True,exist_ok=False)
    audit=read(folder/'audit.json')
    if audit['identity']!='RECONSTRUCTED_INPUTS_VERIFIED_AGAINST_HISTORICAL_OUTPUTS':
        raise RuntimeError('Unverified reconstruction')
    case=read(HISTORY/audit['case']/'resolved_case.json')
    receipt=read(HISTORY/audit['case']/'case_result.json')['result']
    cfg=case['config']['controller']
    with np.load(folder/'reconstruction.npz',allow_pickle=False) as f:
        data={k:f[k].copy() for k in f.files}
    counts=data['scan_counts'];n,na=counts.shape
    offsets=np.r_[0,np.cumsum(counts.ravel())]
    trackers=[ObservedReferenceTracker(gate=cfg.get('registration_gate',.12),
        normal_cosine=cfg.get('registration_normal_cosine',.5),max_speed=cfg.get('max_object_speed',.6),
        max_yaw_rate=cfg.get('max_object_yaw_rate',.8),velocity_filter=cfg.get('object_velocity_filter',.35)) for _ in range(na)]
    positions=np.full((n,na,2),np.nan);velocity=positions.copy()
    rms=np.full((n,na),np.nan);reason=np.full((n,na),'before_contact_ready',dtype='<U32')
    shared=np.full((n,2),np.nan)
    bootstrap={};ready=receipt['events']['CONTACT_READY']['frame']
    started=time.perf_counter()
    if recover_report:
        with np.load(out/'reference_readout.npz',allow_pickle=False) as f:
            for key,expected in [('time',data['time']),('old',data['team_navigation']),('truth',data['truth'])]:
                np.testing.assert_array_equal(f[key],expected)
            shared,positions,velocity,rms,reason=[f[k].copy() for k in ('shared','positions','velocity','rms','reasons')]
        bootstrap={'status':'NOT_SAVED: reporting-path exception after full replay; saved readouts reused without rerunning the estimator'}
    for k in (range(0) if recover_report else range(ready+1,n)):
        scans=[]
        for i in range(na):
            block=data['scans'][offsets[k*na+i]:offsets[k*na+i+1]]
            scans.append(BoundaryView(points=block[:,:2],normals=block[:,2:4],
                confidence=block[:,4],arc_length=block[:,5],object_ids=np.full(len(block),'cargo_0',dtype='<U32')))
        for i,tracker in enumerate(trackers):
            # Same own-first ordering as the controller, not a sorted relay with
            # the owner interspersed. Floating point bootstrap order is preserved.
            ids=[i]+[int(j) for j in np.flatnonzero(data['neighbors'][k,i]) if j!=i]
            scan,_=_merge_scans(scans,ids)
            reading=None
            if not tracker.initialized:
                if data['quality'][k,i]==2:
                    tracker.initialize(scan,1.24*data['raw'][k,i],data['time'][k])
                    if tracker.initialized:bootstrap[str(i)]=dict(frame=k,offset=tracker.offset.tolist(),anchor=tracker.anchor.tolist())
            else:
                reading=tracker.update(scan,data['time'][k])
            reason[k,i]=tracker.reason
            if reading:
                positions[k,i],velocity[k,i],rms[k,i]=reading.displacement,reading.velocity,reading.residual_rms
        live=np.isfinite(positions[k]).all(axis=1)
        if live.any():shared[k]=np.median(positions[k,live],axis=0)
        if k%1000==0:print(f'Offline reference {k}/{n}, elapsed {time.perf_counter()-started:.1f}s',flush=True)
    # Ground truth is first consumed HERE, after the complete estimator replay.
    window=data['time']>=30.
    available=np.isfinite(shared).all(axis=1)
    both=window&available&np.isfinite(data['team_navigation']).all(axis=1)
    err=np.linalg.norm(shared-data['truth'],axis=1)
    old=np.linalg.norm(data['team_navigation']-data['truth'],axis=1)
    old_mae=float(old[both].mean());mae=float(err[both].mean())
    availability=float(available[window].mean())
    final=float(err[-1]) if available[-1] else None
    passed=bool(availability>=.9 and mae<=.75*old_mae and final is not None and final<=.12)
    if not recover_report:
        np.savez_compressed(out/'reference_readout.npz',time=data['time'],shared=shared,
            old=data['team_navigation'],truth=data['truth'],positions=positions,velocity=velocity,rms=rms,reasons=reason)
    report=dict(status='PASS' if passed else 'FAIL',frames=n,physical_trajectory='Historical C-away vector-CVT, unchanged',
        evaluation_window_s=[30,600],paired_frames=int(both.sum()),availability=availability,
        original_same_admitted_frame_mae_m=old_mae,candidate_mae_m=mae,
        fractional_MAE_reduction=1-mae/old_mae,final_vector_error_m=final,
        final_truth=data['truth'][-1].tolist(),final_reference_estimate=shared[-1].tolist(),
        final_goal_error_m=float(np.linalg.norm(shared[-1]-[1.2,0.])) if available[-1] else None,
        bootstrap=bootstrap,wall_seconds=None if recover_report else time.perf_counter()-started,
        reporting_recovery=recover_report,
        reasons={str(v):int(c) for v,c in zip(*np.unique(reason,return_counts=True))},
        gate='At least 25% same-input MAE reduction, final displacement error <= .12m, availability >= .9 after30s; frozen before implementation',
        limitations='Reused selected development trajectory; no closed-loop result; observed-anchor displacement approximates center displacement under rotation; bootstrap bias remains',
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in
            (Path(__file__),ROOT/'src/dbact/control/observed_reference.py',folder/'audit.json',folder/'reconstruction.npz')})
    dump(out/'offline_gate.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('bootstrap','source_sha256')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--recover-report',action='store_true')
    args=p.parse_args();limit_blas_threads(1);replay(args.folder,args.out,args.recover_report)
