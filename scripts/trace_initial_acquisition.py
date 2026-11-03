"""Exactly replay frozen acquisition, observing capture support and QP conflicts."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from replay_qp_conflict import recorded
from trace_boundary_map import plain


def support_metrics(scan, noise):
    if not len(scan):
        return dict(points=0, conditioning=0., minor_rms_m=0., max_normal_gap_rad=2*np.pi,
                    enough_points=False, conditioned=False, spatial=False, normal_wrap=False, supported=False)
    eig = np.linalg.eigvalsh(scan.normals.T @ scan.normals)
    conditioning = float(max(eig[0], 0.) / max(eig[-1], 1e-12))
    centered = scan.points - scan.points.mean(axis=0)
    minor = float(np.sqrt(max(0., np.linalg.eigvalsh(centered.T @ centered/len(scan))[0])))
    angles = np.sort(np.mod(np.arctan2(scan.normals[:, 1], scan.normals[:, 0]), 2*np.pi))
    gap = float(np.max(np.diff(np.r_[angles, angles[0]+2*np.pi])))
    checks = dict(enough_points=len(scan)>=32, conditioned=conditioning>1e-3,
                  spatial=minor>3*noise, normal_wrap=gap<np.pi)
    return dict(points=len(scan), conditioning=conditioning, minor_rms_m=minor,
                max_normal_gap_rad=gap, **checks, supported=all(checks.values()))


def trace(case_id, destination, end_frame):
    guard()
    if any(k=='dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules):
        raise RuntimeError('Use a fresh process for frozen-source imports')
    protocol=ROOT/'experiments/ccta2027/reference_map_v1.json'
    plan=read(protocol)
    batch=ROOT/plan['run_dir']
    snapshot=batch/'source_snapshot'
    for name, digest in plan['source_files'].items():
        assert sha(snapshot/name)==digest, name
    sys.path.insert(0, str(snapshot/'src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    from dbact.safety.qp2d import solve_min_norm_2d
    assert environment_versions()==plan['environment']
    manifest=read(batch/'manifest.json')
    assert manifest['identity']['protocol_sha256']==sha(protocol)
    assert manifest['source_files']==plan['source_files']
    case=next(c for c in plan['cases'] if c['case_id']==case_id)
    folder=batch/case_id
    receipt=cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert receipt['execution_status']=='COMPLETED'
    target=Path(destination).resolve()
    if not target.is_relative_to(ROOT/'runs'):
        raise ValueError('Use a fresh output directory below runs')
    target.mkdir(parents=True, exist_ok=False)
    helpers=('trace_initial_acquisition.py', 'research_io.py', 'replay_qp_conflict.py', 'trace_boundary_map.py')
    for name in helpers:
        shutil.copyfile(ROOT/'scripts'/name, target/name)
    identity=dict(case_id=case_id, end_frame=end_frame, protocol_sha256=sha(protocol),
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'],
        original_receipt_sha256=sha(folder/'case_result.json'), new_physical_episodes=0,
        scripts={name:sha(ROOT/'scripts'/name) for name in helpers})
    dump(target/'identity.json', identity)
    rows=read(folder/'paired_metrics.json')['steps']
    assert 0<end_frame<=len(rows)
    with np.load(folder/'replay.npz', allow_pickle=False) as source:
        saved={key:source[key].copy() for key in ('times','agent_positions','cargo/cargo_0/centers',
            'cargo/cargo_0/angles','sensed_counts','sensed_points')}
    offsets=np.r_[0,np.cumsum(saved['sensed_counts'])]
    env=SimulationEnvironment(case['config'], seed=case['seed'])
    c=env.controller
    c.trace_enabled=True
    c._ensure_state(env.agents)
    assert c.params.perception_every==c.params.planning_every==1
    safety, capture=c.safety, c._initial_capture
    original_add, original_neighbors, original_filter=capture.add, c._neighbor_indices, safety.filter_velocity
    current_frame, call_index, verified=0,0,0
    incoming, neighbors, pending={},[],[]
    events, witnesses=[],{}
    started=time.perf_counter()

    def add(aid, scan):
        cells=capture.cells.get(aid,{})
        keys={tuple(key) for key in np.floor(scan.points/(2*capture.noise)).astype(int)}
        novel=keys-set(cells)
        before=len(cells)
        original_add(aid, scan)
        after=len(capture.cells.get(aid,{}))
        incoming[aid]=dict(fresh=support_metrics(scan,capture.noise), capacity_before=before,
            capacity_after=after, novel_input_cells=len(novel), admitted_cells=after-before,
            capacity_rejected_cells=len(novel)-(after-before))

    def neighbor_indices(agents):
        nonlocal neighbors
        neighbors=original_neighbors(agents)
        return neighbors

    def feasible(A,b):
        return bool(solve_min_norm_2d(np.zeros(2), A, b, safety.params.max_speed).feasible)

    def filtered(position, nominal_velocity, neighbor_positions=(), **kwargs):
        nonlocal call_index
        aid=env.agents[call_index].agent_id
        call_index+=1
        result=original_filter(position,nominal_velocity,neighbor_positions,**kwargs)
        if result.status!='scaled_barrier':
            return result
        A, free=result.A_rows,result.b_no_margin
        obj=np.asarray(result.row_kinds)=='object'
        trace_data={}
        own=(position,kwargs['boundary_points'],kwargs['boundary_normals'],kwargs['object_velocity'])
        ob_A,_,ob_free,ob_h=safety._object_rows(*own,kwargs.get('boundary_point_velocities'),trace=trace_data)
        np.testing.assert_array_equal(ob_A,A[obj])
        np.testing.assert_array_equal(ob_free,free[obj])
        zero_points=None if kwargs.get('boundary_point_velocities') is None else np.zeros_like(kwargs['boundary_point_velocities'])
        zero_A,_,zero_free,zero_h=safety._object_rows(*own[:3],np.zeros(2),zero_points)
        np.testing.assert_array_equal(zero_A,ob_A)
        np.testing.assert_array_equal(zero_h,ob_h)
        zero_rhs=free.copy(); zero_rhs[obj]=zero_free
        # Observer-only truth: never passed back into controller/map/constraint code.
        distance,normal,foot=env.cargoes[0].signed_distance(np.asarray(position).reshape(1,2))
        entry=dict(frame=current_frame,agent_id=aid,phase=c.phase.name,status=c.initial_reference_status,
            scale=result.barrier_scale,minimum_h=float(np.min(result.h_object)) if len(result.h_object) else None,
            true_h=float(distance[0]-safety.params.r_safe),r_safe=safety.params.r_safe,
            feasible=dict(full=feasible(A,result.b_original),no_rho=feasible(A,free),
                hard_only=feasible(A[~obj],free[~obj]),object_only=feasible(A[obj],free[obj]),
                zero_estimated_motion=feasible(A,zero_rhs)),
            A=A,b_original=result.b_original,b_no_margin=free,b_zero_motion=zero_rhs,
            row_kinds=result.row_kinds,nominal=result.u_nominal,command=result.velocity,
            position=position,neighbor_positions=neighbor_positions)
        witness=dict(**entry,boundary_points=kwargs['boundary_points'],boundary_normals=kwargs['boundary_normals'],
            point_velocities=kwargs.get('boundary_point_velocities'),object_velocity=kwargs['object_velocity'],
            true_normal=normal[0],true_foot=foot[0],filter_params=asdict(safety.params),object_trace=trace_data)
        pending.append((plain(entry),plain(witness)))
        return result

    capture.add,c._neighbor_indices,safety.filter_velocity=add,neighbor_indices,filtered
    try:
        with gzip.open(target/'frames.jsonl.gz','wt',encoding='utf-8') as stream:
            for k in range(end_frame):
                current_frame,call_index=k+1,0
                incoming.clear()
                row=rows[k]
                env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k],saved['cargo/cargo_0/angles'][k])
                for i,agent in enumerate(env.agents):
                    agent.position=saved['agent_positions'][k,i].copy()
                    agent.velocity=np.zeros(2) if k==0 else np.array(rows[k-1]['commands'][i])
                commands=c.step(env.agents,env.cargoes,float(saved['times'][k]),env.dt)
                assert call_index==len(env.agents)
                np.testing.assert_array_equal([v.velocity for v in commands],row['commands'])
                assert recorded([vars(d) for d in c.diagnostics])==row['diagnostics'],('diagnostics',current_frame)
                assert recorded(safety.stats.as_dict())==row['solver_statistics'],('solver',current_frame)
                assert c.phase.name==row['phase'] and recorded(vars(c.phase_signals))==row['phase_guards']
                assert recorded(c.phase_monitor.as_dict())==row['phase_response']
                np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                stats={a.agent_id:support_metrics(capture.view(a.agent_id),capture.noise) for a in env.agents}
                for aid,s in stats.items():
                    assert s['supported']==capture.supported(aid),('support',current_frame,aid)
                record=dict(frame=current_frame,time=float(saved['times'][k]),phase=c.phase.name,
                    status=c.initial_reference_status,epoch=c.initial_reference_epoch,
                    positions=saved['agent_positions'][k],true_center=saved['cargo/cargo_0/centers'][k],
                    true_angle=saved['cargo/cargo_0/angles'][k],neighbors=neighbors,incoming=incoming,
                    captures=stats,own_scan_points={aid:len(scan) for aid,scan in c.last_scans.items()},
                    diagnostics=row['diagnostics'],phase_guards=row['phase_guards'])
                stream.write(json.dumps(plain(record),separators=(',',':'),allow_nan=False)+'\n')
                for entry,witness in pending:
                    events.append(entry)
                    witnesses.setdefault('first',witness)
                    if 'worst' not in witnesses or witness['scale']<witnesses['worst']['scale']:
                        witnesses['worst']=witness
                pending.clear()
                verified+=1
                if verified%200==0:
                    print(json.dumps(dict(case=case_id,verified=verified,supported=sum(s['supported'] for s in stats.values()),scaled=len(events))),flush=True)
        assert len(events)==safety.stats.as_dict()['barrier_scalings']
        dump(target/'events.json',dict(events=events,witnesses=witnesses))
        dump(target/'final_capture.json',{a.agent_id:dict(points=capture.view(a.agent_id).points.tolist(),
            normals=capture.view(a.agent_id).normals.tolist(),stats=stats[a.agent_id]) for a in env.agents})
        products={name:sha(target/name) for name in ('frames.jsonl.gz','events.json','final_capture.json')}
        outcome=dict(**identity,status='EXACT_RECONSTRUCTION',verified_frames=verified,
            elapsed_seconds=time.perf_counter()-started,scaled_robot_steps=len(events),products=products)
        dump(target/'outcome.json',outcome)
        print(json.dumps(dict(case=case_id,status=outcome['status'],frames=verified,scaled=len(events))),flush=True)
    except Exception as exc:
        dump(target/'failure.json',dict(**identity,status='INVALID_RECONSTRUCTION',verified_frames=verified,
            failing_frame=current_frame,error=repr(exc)))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--end-frame',type=int,required=True)
    args=parser.parse_args()
    trace(args.case,args.out,args.end_frame)
