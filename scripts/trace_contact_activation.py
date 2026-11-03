"""Exactly replay frozen contact activation with read-only local witnesses."""
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


def trace(case_id, destination, end_frame):
    guard()
    if any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules):
        raise RuntimeError('Frozen-source imports need a fresh process')
    protocol = ROOT / 'experiments/ccta2027/acquisition_circulation_v1.json'
    plan = read(protocol)
    batch = ROOT / plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch / 'source_snapshot' / name) == digest
    sys.path.insert(0, str(batch / 'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    assert environment_versions() == plan['environment']
    manifest = read(batch / 'manifest.json')
    assert manifest['identity']['protocol_sha256'] == sha(protocol)
    case = next(c for c in plan['cases'] if c['case_id'] == case_id)
    folder = batch / case_id
    receipt = cached_result(folder, {**manifest['identity'], 'case_sha256': encode(case)})
    assert receipt['execution_status'] == 'COMPLETED'
    target = Path(destination).resolve()
    if not target.is_relative_to(ROOT / 'runs'):
        raise ValueError('Use a fresh runs directory')
    target.mkdir(parents=True, exist_ok=False)
    helpers = ('trace_contact_activation.py', 'research_io.py', 'replay_qp_conflict.py', 'trace_boundary_map.py')
    for name in helpers:
        shutil.copy2(ROOT / 'scripts' / name, target / name)
    design = ROOT / 'experiments/ccta2027/contact_activation_v1_design.md'
    shutil.copy2(design, target / 'design.md')
    identity = dict(case_id=case_id, end_frame=end_frame, protocol_sha256=sha(protocol),
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'],
        original_receipt_sha256=sha(folder / 'case_result.json'), design_sha256=sha(design),
        scripts={name:sha(ROOT / 'scripts' / name) for name in helpers}, new_physical_episodes=0)
    dump(target / 'identity.json', identity)
    rows = read(folder / 'paired_metrics.json')['steps']
    assert 0 < end_frame <= len(rows)
    with np.load(folder / 'replay.npz', allow_pickle=False) as source:
        saved = {key:source[key].copy() for key in ('times', 'agent_positions', 'cargo/cargo_0/centers',
            'cargo/cargo_0/angles', 'sensed_counts', 'sensed_points')}
    offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c = env.controller
    c.trace_enabled = True
    c._ensure_state(env.agents)
    assert c.params.perception_every == c.params.planning_every == 1
    original_nominal, original_transport = c._nominal_command, c._transport_command
    original_neighbors, original_filter = c._neighbor_indices, c.safety.filter_velocity
    observations, geometry, snapshots, conflicts = {}, {}, [], {}
    neighbors = []
    current_frame, call_index, verified, transport_calls = 0, 0, 0, 0
    previous_phase, previous_reference = '', ''
    started = time.perf_counter()

    def selected_frame():
        entered = c.phase_monitor.entered.get('CONTACT_READY')
        return (current_frame == end_frame
            or (c.initial_reference_status == 'frozen' and previous_reference != 'frozen')
            or (c.phase.name == 'CONTACT_READY' and previous_phase != 'CONTACT_READY')
            or (entered is not None and c._frame - entered == 200))

    def neighbor_indices(agents):
        nonlocal neighbors
        neighbors = original_neighbors(agents)
        return neighbors

    def local_geometry(i):
        agent = env.agents[i]
        view = c._views[agent.agent_id]
        scan = c.last_scans[agent.agent_id]
        return plain(dict(position=agent.position, view_points=view.points, view_normals=view.normals,
            view_confidence=view.confidence, own_scan_points=scan.points, own_scan_normals=scan.normals,
            neighbors=[env.agents[j].position for j in neighbors[i]]))

    def nominal(i, agents, neighbor_indices, view, contact_ready, dt):
        agent = agents[i]
        aid = agent.agent_id
        nearest = c._nearest_index(view, agent.position)
        nav = c._navigation.get(aid)
        normal = None if nearest is None else view.normals[nearest]
        point = None if nearest is None else view.points[nearest]
        observations[aid] = plain(dict(ready=contact_ready[i], neighbors=neighbor_indices,
            quorum=1+sum(contact_ready[j] for j in neighbor_indices), position=agent.position,
            map_points=len(view), nearest_point=point, nearest_normal=normal,
            nearest_distance=None if nearest is None else np.linalg.norm(agent.position-point),
            signed_map_gap=None if nearest is None else (agent.position-point) @ normal,
            navigation=None if nav is None else asdict(nav),
            map_reference=getattr(c, '_map_reference_state', {}).get(aid), transport=None))
        result = original_nominal(i, agents, neighbor_indices, view, contact_ready, dt)
        observations[aid].update(plain(dict(nominal=result[0], nominal_mode=result[1],
            nominal_push=result[3], nominal_effort=result[4],
            cvt_centroid=c._trace.get('cvt_centroid'), redeploy_target=c._trace.get('redeploy_target'),
            redeploy_reason=c._trace.get('redeploy_reason'), held_fraction=c._trace.get('cell_held_fraction'))))
        if selected_frame():
            geometry[aid] = local_geometry(i)
        return result

    def transport(i, agents, neighbor_indices, view, contact_ready, dt, shape):
        nonlocal transport_calls
        aid = agents[i].agent_id
        loop = c.transport_loops[aid]
        before = dict(integral=loop.integral, holding=loop.holding)
        result = original_transport(i, agents, neighbor_indices, view, contact_ready, dt, shape)
        assert observations[aid]['transport'] is None
        observations[aid]['transport'] = plain(dict(before=before,
            after=dict(integral=loop.integral, holding=loop.holding),
            command=result[0], push=result[1], effort=result[2], press_normal=result[3],
            patch_gap=c._trace.get('contact_used_gap'), patch_source=c._trace.get('contact_distance_source'),
            selected_normal=c._trace.get('contact_selected_normal'),
            desired_direction=c._trace.get('contact_desired_direction'), shape=asdict(shape)))
        transport_calls += 1
        return result

    def filtered(position, nominal_velocity, neighbor_positions=(), **kwargs):
        nonlocal call_index
        i = call_index
        call_index += 1
        aid = env.agents[i].agent_id
        result = original_filter(position, nominal_velocity, neighbor_positions, **kwargs)
        distance, true_normal, foot = env.cargoes[0].signed_distance(np.asarray(position).reshape(1, 2))
        observations[aid]['filter'] = plain(dict(nominal=nominal_velocity, command=result.velocity,
            status=result.status, scale=result.barrier_scale, agent_rows_active=result.agent_rows_active,
            object_rows_active=result.object_rows_active, zero_input_feasible=result.zero_input_feasible,
            true_boundary_gap=distance[0], true_normal=true_normal[0], true_foot=foot[0],
            r_safe=c.safety.params.r_safe))
        if selected_frame() or result.status == 'scaled_barrier':
            full = local_geometry(i)
            full['filter'] = plain(dict(A=result.A_rows, b_original=result.b_original,
                b_no_margin=result.b_no_margin, row_kinds=result.row_kinds,
                boundary_points=kwargs['boundary_points'], boundary_normals=kwargs['boundary_normals'],
                object_velocity=kwargs['object_velocity'], point_velocities=kwargs.get('boundary_point_velocities'),
                params=asdict(c.safety.params), nominal=nominal_velocity, command=result.velocity))
            if selected_frame():
                geometry[aid].update(full)
            if result.status == 'scaled_barrier':
                witness = dict(frame=current_frame, agent_id=aid, scale=result.barrier_scale,
                               phase=c.phase.name, observation=observations[aid], geometry=full)
                conflicts.setdefault('first', witness)
                if 'worst' not in conflicts or result.barrier_scale < conflicts['worst']['scale']:
                    conflicts['worst'] = witness
        return result

    c._neighbor_indices, c._nominal_command, c._transport_command = neighbor_indices, nominal, transport
    c.safety.filter_velocity = filtered
    try:
        with gzip.open(target / 'frames.jsonl.gz', 'wt', encoding='utf-8') as stream:
            for k in range(end_frame):
                current_frame, call_index = k+1, 0
                observations.clear()
                geometry.clear()
                previous_phase, previous_reference = c.phase.name, c.initial_reference_status
                row = rows[k]
                env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
                for i, agent in enumerate(env.agents):
                    agent.position = saved['agent_positions'][k, i].copy()
                    agent.velocity = np.zeros(2) if k == 0 else np.asarray(rows[k-1]['commands'][i])
                commands = c.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
                assert call_index == len(env.agents)
                np.testing.assert_array_equal([v.velocity for v in commands], row['commands'])
                assert recorded([vars(d) for d in c.diagnostics]) == row['diagnostics'], ('diagnostics', current_frame)
                assert recorded(c.safety.stats.as_dict()) == row['solver_statistics'], ('solver', current_frame)
                assert c.phase.name == row['phase'] and recorded(vars(c.phase_signals)) == row['phase_guards']
                assert recorded(c.phase_monitor.as_dict()) == row['phase_response']
                np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                frame = plain(dict(frame=current_frame, time=float(saved['times'][k]), phase=c.phase.name,
                    reference_status=c.initial_reference_status, epoch=c.initial_reference_epoch,
                    observations=observations, phase_guards=row['phase_guards'],
                    true_center=saved['cargo/cargo_0/centers'][k], true_angle=saved['cargo/cargo_0/angles'][k],
                    contacts_after_physics=dict(ids=row['contact_ids'], normals=row['contact_normals'],
                        forces=row['contact_forces'], net_force=row['actual_net_force'], torque=row['actual_net_torque'])))
                stream.write(json.dumps(frame, separators=(',', ':'), allow_nan=False)+'\n')
                if geometry:
                    snapshots.append(dict(**frame, geometry=plain(geometry)))
                verified += 1
                if verified % 200 == 0:
                    print(json.dumps(dict(case=case_id, verified=verified, phase=c.phase.name)), flush=True)
        dump(target / 'witnesses.json', dict(snapshots=snapshots, conflicts=conflicts))
        dump(target / 'outcome.json', dict(**identity, status='EXACT_RECONSTRUCTION', verified_frames=verified,
            transport_calls=transport_calls, elapsed_seconds=time.perf_counter()-started,
            products={name:sha(target / name) for name in ('frames.jsonl.gz', 'witnesses.json')}))
        print(json.dumps(dict(case=case_id, verified=verified, status='EXACT_RECONSTRUCTION')), flush=True)
    except Exception as exc:
        dump(target / 'failure.json', dict(**identity, verified_frames=verified, failing_frame=current_frame,
                                          status='INVALID_RECONSTRUCTION', error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--end-frame', type=int, required=True)
    args = parser.parse_args()
    trace(args.case, args.out, args.end_frame)
