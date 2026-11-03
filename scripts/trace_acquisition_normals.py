"""Trace all saved acquisition conflicts back through one agent's local map."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

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

PROTOCOL = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
DESIGN = ROOT/'experiments/ccta2027/acquisition_normal_audit_v1_design.md'


def trace(condition, destination, frames=None):
    guard()
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    plan = read(PROTOCOL)
    batch = ROOT/plan['run_dir']
    snapshot = batch/'source_snapshot'
    for name, digest in plan['source_files'].items():
        assert sha(snapshot/name) == digest, name
    sys.path.insert(0, str(snapshot/'src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    assert environment_versions() == plan['environment']
    case = next(c for c in plan['cases'] if c['condition_id'] == condition and c['arm'] == 'reference_reacquisition')
    manifest = read(batch/'manifest.json')
    assert manifest['identity']['protocol_sha256'] == sha(PROTOCOL)
    folder = batch/case['case_id']
    result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert result['execution_status'] == 'COMPLETED'
    published = read(ROOT/'docs/results/reference_reacquisition_physical_v1/analysis.json')
    prefix = published['acquisition_prefix_reproduction'][condition]
    assert prefix['exact']
    end_frame = prefix['freeze_frame'] if frames is None else frames
    assert 0 < end_frame <= prefix['freeze_frame']
    episode = next(r for r in published['episodes'] if r['case_id'] == case['case_id'])
    expected = [(s['frame'], aid) for s in episode['scaling']['events']
                if s['frame'] <= end_frame for aid in s['agents']]
    assert all(aid == 'agent_05' for _, aid in expected)
    out = Path(destination).resolve()
    assert out.is_relative_to(ROOT/'runs')
    out.mkdir(parents=True, exist_ok=False)
    helpers = ('trace_acquisition_normals.py', 'research_io.py', 'replay_qp_conflict.py', 'trace_boundary_map.py')
    for name in helpers:
        shutil.copy2(ROOT/'scripts'/name, out/name)
    shutil.copy2(DESIGN, out/'design.md')
    identity = dict(case_id=case['case_id'], end_frame=end_frame, selected_agent='agent_05',
        protocol_sha256=sha(PROTOCOL), source_sha256=plan['frozen_source_sha256'], environment=plan['environment'],
        original_receipt_sha256=sha(folder/'case_result.json'), design_sha256=sha(DESIGN),
        scripts={name:sha(ROOT/'scripts'/name) for name in helpers}, new_physical_episodes=0)
    dump(out/'identity.json', identity)
    verified, current_frame = 0, 0
    started = time.perf_counter()
    try:
        rows = read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz', allow_pickle=False) as z:
            saved = {k:z[k].copy() for k in ('times', 'agent_positions', 'cargo/cargo_0/centers',
                'cargo/cargo_0/angles', 'sensed_counts', 'sensed_points')}
        offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        controller = env.controller
        controller.trace_enabled = True
        controller._ensure_state(env.agents)
        assert controller.params.perception_every == controller.params.planning_every == 1
        agent_index = next(i for i,a in enumerate(env.agents) if a.agent_id == 'agent_05')
        local = controller.maps['agent_05']
        original_update, original_filter = local.update, controller.safety.filter_velocity
        frame_record, events = {}, []
        call_index = 0

        def state():
            return plain(dict(keys=local._keys.copy(), points=local._points.copy(), normals=local._normals.copy(),
                weights=local._weight.copy(), confidence=local._confidence.copy(), timestamps=local._timestamp.copy()))

        def update(observations, timestamp, agent_codes=None, dt=0., **kwargs):
            frame_record['before_update'] = state()
            frame_record['incoming'] = plain(dict(points=observations.points, normals=observations.normals,
                confidence=observations.confidence, object_ids=observations.object_ids,
                source_codes=agent_codes, keys=local._cell_keys(observations.object_ids, observations.points)))
            answer = original_update(observations, timestamp, agent_codes=agent_codes, dt=dt, **kwargs)
            frame_record['after_update'] = state()
            return answer

        def filtered(position, nominal_velocity, neighbor_positions=(), **kwargs):
            nonlocal call_index
            i = call_index
            call_index += 1
            answer = original_filter(position, nominal_velocity, neighbor_positions, **kwargs)
            if answer.status != 'scaled_barrier':
                return answer
            assert i == agent_index
            view = controller._views['agent_05']
            object_trace = {}
            obj_A, obj_b, obj_free, h = controller.safety._object_rows(position, kwargs['boundary_points'],
                kwargs['boundary_normals'], kwargs['object_velocity'], kwargs.get('boundary_point_velocities'), trace=object_trace)
            selected = np.asarray(answer.row_kinds) == 'object'
            for observed, actual in ((obj_A, answer.A_rows[selected]), (obj_b, answer.b_original[selected]),
                                    (obj_free, answer.b_no_margin[selected])):
                np.testing.assert_array_equal(observed, actual)
            signed, true_normals, feet = env.cargoes[0].signed_distance(view.points)
            gap, normal, foot = env.cargoes[0].signed_distance(np.asarray(position).reshape(1,2))
            events.append(plain(dict(frame=current_frame, agent_id='agent_05', scale=answer.barrier_scale,
                phase=controller.phase.name, capture_state=controller.initial_reference_status,
                position=position, view_points=view.points, view_normals=view.normals, view_confidence=view.confidence,
                true_map_signed_distance=signed, true_map_normals=true_normals, true_map_feet=feet,
                true_boundary_gap=gap[0], true_normal=normal[0], true_foot=foot[0],
                params=asdict(controller.safety.params), object_trace=object_trace,
                A=answer.A_rows, b=answer.b_original, b_no_margin=answer.b_no_margin, row_kinds=answer.row_kinds,
                nominal=nominal_velocity, command=answer.velocity)))
            return answer

        local.update, controller.safety.filter_velocity = update, filtered
        with gzip.open(out/'map_history.jsonl.gz', 'wt', encoding='utf-8') as stream:
            for k in range(end_frame):
                current_frame, call_index = k+1, 0
                row = rows[k]
                env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
                for i, agent in enumerate(env.agents):
                    agent.position = saved['agent_positions'][k,i].copy()
                    agent.velocity = np.zeros(2) if k == 0 else np.asarray(rows[k-1]['commands'][i])
                frame_record = dict(frame=current_frame, time=float(saved['times'][k]), positions=saved['agent_positions'][k].tolist())
                commands = controller.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
                assert call_index == len(env.agents)
                np.testing.assert_array_equal([v.velocity for v in commands], row['commands'])
                assert recorded([vars(d) for d in controller.diagnostics]) == row['diagnostics'], ('diagnostics', current_frame)
                assert recorded(controller.safety.stats.as_dict()) == row['solver_statistics'], ('solver', current_frame)
                assert controller.phase.name == row['phase'] and recorded(vars(controller.phase_signals)) == row['phase_guards']
                assert recorded(controller.phase_monitor.as_dict()) == row['phase_response']
                np.testing.assert_array_equal(controller.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                assert 'before_update' in frame_record and 'after_update' in frame_record
                frame_record.update(phase=controller.phase.name, capture_state=controller.initial_reference_status,
                    voxel_size=local.voxel_size, max_weight=local.max_weight)
                stream.write(json.dumps(frame_record, separators=(',', ':'), allow_nan=False)+'\n')
                verified += 1
                if verified % 100 == 0:
                    print(json.dumps(dict(case=case['case_id'], verified=verified, scaled_events=len(events))), flush=True)
        assert [(e['frame'], e['agent_id']) for e in events] == expected
        dump(out/'events.json', dict(events=events, new_physical_episodes=0))
        outcome = dict(**identity, status='EXACT_RECONSTRUCTION', verified_frames=verified, scaled_events=len(events),
            elapsed_seconds=time.perf_counter()-started,
            products={name:sha(out/name) for name in ('map_history.jsonl.gz', 'events.json')})
        dump(out/'outcome.json', outcome)
        print(json.dumps(dict(case=case['case_id'], status=outcome['status'], verified=verified, events=len(events))), flush=True)
    except Exception as exc:
        dump(out/'failure.json', dict(**identity, status='INVALID_RECONSTRUCTION', verified_frames=verified,
            failing_frame=current_frame, error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--condition', choices=('far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--frames', type=int)
    args = parser.parse_args()
    trace(args.condition, args.out, args.frames)
