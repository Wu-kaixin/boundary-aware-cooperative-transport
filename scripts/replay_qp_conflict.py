"""Recover original joint QP rows from exactly verified forced-pose replay."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np

from research_io import ROOT, guard, read, sha

OLD = ROOT/'experiments/ccta2027/acquisition_geometry_v1.json'


def recorded(value):
    """Match deployment.dump's tuple and undefined-number serialization exactly."""
    if isinstance(value, dict):
        return {k:recorded(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)):
        return [recorded(v) for v in value]
    if isinstance(value, (float,np.floating)) and not np.isfinite(value):
        return None
    return value


def replay(case_id, destination, end_frame, diagnose=True):
    guard()
    if any(k=='dbact' or k.startswith('dbact.') or k.startswith('dbact_sim') for k in sys.modules):
        raise RuntimeError('Use a fresh process for frozen-source replay')
    plan = read(OLD)
    batch = ROOT/plan['run_dir']
    snapshot = batch/'source_snapshot'
    for name, digest in plan['source_files'].items():
        assert sha(snapshot/name)==digest, name
    sys.path.insert(0, str(snapshot/'src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    from dbact.safety.qp2d import solve_min_norm_2d
    assert environment_versions()==plan['environment']
    manifest = read(batch/'manifest.json')
    assert manifest['identity']['protocol_sha256']==sha(OLD)
    assert manifest['source_files']==plan['source_files']
    case = next(c for c in plan['cases'] if c['case_id']==case_id)
    folder = batch/case_id
    receipt = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert receipt['execution_status']=='COMPLETED'
    target = Path(destination).resolve()
    if not target.is_relative_to(ROOT/'runs'):
        raise ValueError('Use a fresh output directory below runs')
    target.mkdir(parents=True, exist_ok=False)
    start = time.perf_counter()
    dump(target/'started.json', dict(case_id=case_id, end_frame=end_frame, new_physical_episodes=0,
        script_sha256=sha(__file__), source_sha256=plan['frozen_source_sha256'],
        receipt_sha256=sha(folder/'case_result.json')))
    rows = read(folder/'paired_metrics.json')['steps']
    assert 0<end_frame<=len(rows)
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        saved = {k:z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers',
            'cargo/cargo_0/angles','sensed_counts','sensed_points')}
    offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c, safety = env.controller, env.controller.safety
    c.trace_enabled = True
    assert c.params.perception_every==c.params.planning_every==1
    actual_filter = safety.filter_velocity
    verified, events, witnesses, pending = 0, [], {}, []
    current_frame, call_index = 0, 0

    def feasible(A, b):
        return bool(solve_min_norm_2d(np.zeros(2), A, b, safety.params.max_speed).feasible)

    def wrapped(position, nominal_velocity, neighbor_positions=(), **kwargs):
        nonlocal call_index
        aid = env.agents[call_index].agent_id
        call_index += 1
        result = actual_filter(position, nominal_velocity, neighbor_positions, **kwargs)
        if diagnose and result.status=='scaled_barrier':
            A, free = result.A_rows, result.b_no_margin
            kinds = np.asarray(result.row_kinds)
            hard, obj = kinds!='object', kinds=='object'
            trace = {}
            own = (position, kwargs['boundary_points'], kwargs['boundary_normals'], kwargs['object_velocity'])
            ob_A, ob_b, ob_free, ob_h = safety._object_rows(*own, kwargs.get('boundary_point_velocities'), trace=trace)
            np.testing.assert_array_equal(ob_A, A[obj])
            np.testing.assert_array_equal(ob_free, free[obj])
            zero_points = None if kwargs.get('boundary_point_velocities') is None else np.zeros_like(kwargs['boundary_point_velocities'])
            zero_A, zero_b, zero_free, zero_h = safety._object_rows(*own[:3], np.zeros(2), zero_points)
            np.testing.assert_array_equal(zero_A, ob_A)
            np.testing.assert_array_equal(zero_h, ob_h)
            zero_rhs = free.copy()
            zero_rhs[obj] = zero_free
            # Evaluation only, after the original filter has returned. None of
            # these truth values are supplied to the online/counterfactual rows.
            distance, true_normal, foot = env.cargoes[0].signed_distance(np.asarray(position).reshape(1,2))
            true_velocity = np.zeros(2) if current_frame==1 else np.asarray(rows[current_frame-2]['true_velocity'])
            entry = dict(frame=current_frame, agent_id=aid, phase=c.phase.name,
                acquisition_status=c.initial_reference_status, scale=result.barrier_scale,
                minimum_h=float(np.min(result.h_object)),
                object_velocity=kwargs['object_velocity'].tolist(),
                true_signed_distance_m=float(distance[0]), true_h_m=float(distance[0]-safety.params.r_safe),
                true_velocity=true_velocity.tolist(),
                min_normal_alignment=float(np.min(ob_A@true_normal[0])),
                feasibility=dict(full=feasible(A, result.b_original), no_rho=feasible(A, free),
                    hard_only=feasible(A[hard], free[hard]), object_only=feasible(A[obj], free[obj]),
                    zero_estimated_motion=feasible(A, zero_rhs)),
                row_counts=dict(hard=int(hard.sum()), object=int(obj.sum())))
            witness = dict(**entry, A=A.tolist(), b_original=result.b_original.tolist(),
                b_no_margin=free.tolist(), b_zero_motion=zero_rhs.tolist(), row_kinds=result.row_kinds,
                h_object=result.h_object.tolist(), u_nominal=result.u_nominal.tolist(),
                u_returned=result.velocity.tolist(), max_speed=safety.params.max_speed,
                position=np.asarray(position).tolist(), neighbor_positions=np.asarray(neighbor_positions).tolist(),
                boundary_points=kwargs['boundary_points'].tolist(), boundary_normals=kwargs['boundary_normals'].tolist(),
                true_nearest_normal=true_normal[0].tolist(), true_nearest_foot=foot[0].tolist(),
                point_velocities=None if kwargs.get('boundary_point_velocities') is None else kwargs['boundary_point_velocities'].tolist(),
                filter_params=asdict(safety.params), object_trace=trace)
            pending.append((entry,witness))
        return result

    safety.filter_velocity = wrapped
    try:
        for k in range(end_frame):
            current_frame, call_index = k+1, 0
            row = rows[k]
            env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
            for i, agent in enumerate(env.agents):
                agent.position = saved['agent_positions'][k,i].copy()
                agent.velocity = np.zeros(2) if k==0 else np.array(rows[k-1]['commands'][i])
            commands = c.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
            assert call_index==len(env.agents)
            np.testing.assert_array_equal([v.velocity for v in commands], row['commands'], err_msg=f'commands frame{current_frame}')
            diagnostic = recorded([vars(d) for d in c.diagnostics])
            if diagnostic!=row['diagnostics']:
                dump(target/'diagnostics_mismatch.json', dict(frame=current_frame,
                    actual=diagnostic, expected=row['diagnostics']))
                raise AssertionError(('diagnostics',current_frame))
            assert recorded(c.safety.stats.as_dict())==row['solver_statistics'], ('solver_statistics', current_frame)
            assert c.phase.name==row['phase'] and recorded(vars(c.phase_signals))==row['phase_guards'], ('phase',current_frame)
            assert recorded(c.phase_monitor.as_dict())==row['phase_response'], ('phase_response',current_frame)
            np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                saved['sensed_points'][offsets[k+1]:offsets[k+2]], err_msg=f'map frame{current_frame}')
            verified += 1
            for entry, witness in pending:
                assert next(d for d in row['diagnostics'] if d['agent_id']==entry['agent_id'])['solver_status']=='scaled_barrier'
                events.append(entry)
                phase = entry['phase']
                witnesses.setdefault(phase+'_first', witness)
                key = phase+'_worst'
                if key not in witnesses or entry['scale']<witnesses[key]['scale']:
                    witnesses[key] = witness
            pending.clear()
            if current_frame%200==0:
                print(json.dumps(dict(case=case_id, verified=verified, scaled_robot_steps=len(events))), flush=True)
        result = dict(status='EXACT_RECONSTRUCTION', case_id=case_id, verified_frames=verified,
            end_frame=end_frame, elapsed_seconds=time.perf_counter()-start, new_physical_episodes=0,
            events=events, witnesses=witnesses, receipt_sha256=sha(folder/'case_result.json'),
            source_sha256=plan['frozen_source_sha256'], script_sha256=sha(__file__),
            end_scaling_counter=c.safety.stats.as_dict()['barrier_scalings'])
        assert not diagnose or len(events)==result['end_scaling_counter']
        dump(target/'reconstruction.json', result)
        print(json.dumps(dict(case=case_id, status=result['status'], frames=verified,
            scaled_robot_steps=len(events))), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(status='INVALID_RECONSTRUCTION', case_id=case_id,
            verified_frames=verified, failing_frame=current_frame, error=repr(exc), new_physical_episodes=0))
        raise


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--end-frame', type=int, required=True)
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    replay(args.case, args.out, args.end_frame, not args.preflight)
