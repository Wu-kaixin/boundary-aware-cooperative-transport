"""Exact no-physics replay at the first actual matched-pair command divergence."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from itertools import islice
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

from research_io import ROOT, guard, read, sha
from stream_step_receipts import iter_step_receipts
from trace_boundary_map import plain
from probe_post_support_contact_direction import maximize_direction
import post_acquisition_support_study as study
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump

INPUT = ROOT / 'runs/research/2026-11-03-post-support-candidate-onset-v2'
OUT = ROOT / 'runs/research/2026-11-03-post-support-initial-divergence-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_candidate_onset_v1_design.md'


def serial(value):
    if is_dataclass(value):
        return serial(asdict(value))
    if isinstance(value, dict):
        return {k: serial(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [serial(v) for v in value]
    return plain(value)


def replay(case, end):
    folder = study.OUT / case['case_id']
    with (folder / 'paired_metrics.json').open(encoding='utf-8') as stream:
        rows = list(islice(iter_step_receipts(stream), end))
    assert len(rows) == end
    with np.load(folder / 'replay.npz', allow_pickle=False) as archive:
        saved = {key: archive[key].copy() for key in ('times', 'agent_positions',
            'cargo/cargo_0/centers', 'cargo/cargo_0/angles', 'sensed_counts', 'sensed_points',
            'task/cargo_0/direction')}
    offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c = env.controller
    c.trace_enabled = True
    ids = [agent.agent_id for agent in env.agents]
    original = c.safety.prepare_velocity
    frame, call_index = 0, 0
    captures = {}

    def prepare(position, nominal_velocity, neighbor_positions=(), **inputs):
        nonlocal call_index
        i, call_index = call_index, call_index + 1
        result = original(position, nominal_velocity, neighbor_positions, **inputs)
        if frame == end:
            captures[ids[i]] = dict(position=np.asarray(position).copy(),
                nominal_argument=np.asarray(nominal_velocity).copy(), u_nom=result.u_nom.copy(),
                neighbor_positions=np.asarray(neighbor_positions).copy(), inputs=serial(inputs),
                A=result.A.copy(), b=result.b_original.copy(), kinds=list(result.row_kinds),
                h=result.h_object.copy(), params=asdict(result.parameters),
                post_support=serial(c._trace.get('post_acquisition_support')),
                map_reference=serial(c._map_reference_state[ids[i]]))
        return result

    c.safety.prepare_velocity = prepare
    try:
        for k, row in enumerate(rows):
            frame, call_index = k + 1, 0
            env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
            for i, agent in enumerate(env.agents):
                agent.position = saved['agent_positions'][k, i].copy()
                agent.velocity = np.zeros(2) if k == 0 else np.asarray(rows[k-1]['commands'][i])
            commands = c.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
            assert call_index == len(ids)
            np.testing.assert_array_equal([v.velocity for v in commands], row['commands'])
            assert plain([vars(d) for d in c.diagnostics]) == row['diagnostics'], ('diagnostics', frame)
            assert plain(c.safety.stats.as_dict()) == row['solver_statistics']
            assert c.phase.name == row['phase'] and plain(vars(c.phase_signals)) == row['phase_guards']
            assert plain(c.phase_monitor.as_dict()) == row['phase_response']
            np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                saved['sensed_points'][offsets[k+1]:offsets[k+2]])
            if frame % 200 == 0:
                print(json.dumps(dict(case=case['case_id'], verified_control_frames=frame)), flush=True)
    finally:
        c.safety.prepare_velocity = original
    assert len(captures) == len(ids)
    goal = saved['task/cargo_0/direction'].reshape(2)
    for i, aid in enumerate(ids):
        state = captures[aid]
        u = np.asarray(rows[-1]['commands'][i])
        state.update(agent_id=aid, goal=goal, command=u,
            nominal_goal_velocity=float(state['u_nom'] @ goal), executed_goal_velocity=float(u @ goal),
            mode=rows[-1]['diagnostics'][i]['mode'], declared_pusher=rows[-1]['diagnostics'][i]['push_side'],
            solver_status=rows[-1]['diagnostics'][i]['solver_status'],
            minimum_executed_residual=float(np.min(state['A'] @ u - state['b'])) if len(state['b']) else None)
        state['capability'] = maximize_direction(state['A'], state['b'], state['params']['max_speed'], goal)
        assert state['capability']['status'] != 'INCONCLUSIVE', (case['case_id'], aid)
    return plain(dict(case_id=case['case_id'], arm=case['arm'], condition=case['condition_id'],
        frame=end, verified_frames=end, precommand_time_s=float(saved['times'][end-1]),
        states=list(captures.values()), exact_commands_diagnostics_solver_phase_maps=True))


def compare(results):
    old, new = results
    states = []
    for left, right in zip(old['states'], new['states']):
        assert left['agent_id'] == right['agent_id']
        a, b = np.asarray(left['A']).reshape(-1, 2), np.asarray(right['A']).reshape(-1, 2)
        ia, ib = np.asarray(left['kinds']) != 'object', np.asarray(right['kinds']) != 'object'
        np.testing.assert_array_equal(a[ia], b[ib])
        np.testing.assert_array_equal(np.asarray(left['b'])[ia], np.asarray(right['b'])[ib])
        common = {key: left[key] == right[key] for key in ('position', 'nominal_argument', 'u_nom',
            'neighbor_positions', 'map_reference')}
        original_input_keys = set(left['inputs'])
        shared = {key: left['inputs'][key] == right['inputs'][key] for key in original_input_keys}
        left_u, right_u, goal = [np.asarray(x) for x in (left['command'], right['command'], left['goal'])]
        nominal = np.asarray(right['u_nom'])
        support = right['post_support'] or {}
        states.append(dict(agent_id=left['agent_id'], common_arguments_equal=common,
            original_filter_inputs_equal=shared, extra_candidate_inputs=sorted(set(right['inputs'])-original_input_keys),
            hard_rows_equal=True, old_rows=len(left['b']), new_rows=len(right['b']),
            old_object_rows=left['kinds'].count('object'), new_object_rows=right['kinds'].count('object'),
            support_applied=support.get('applied', False), support_reason=support.get('reason'),
            demand_speed=support.get('demand_speed'),
            minimum_finite_h=min(right['h']) if right['h'] else None,
            old_command_new_min_residual=float(np.min(b @ left_u - np.asarray(right['b']))) if len(b) else None,
            new_nominal_min_residual=float(np.min(b @ nominal - np.asarray(right['b']))) if len(b) else None,
            old_nominal_goal_velocity=left['nominal_goal_velocity'], new_nominal_goal_velocity=right['nominal_goal_velocity'],
            old_executed_goal_velocity=left['executed_goal_velocity'], new_executed_goal_velocity=right['executed_goal_velocity'],
            command_change_norm=float(np.linalg.norm(right_u-left_u)),
            goal_velocity_change=float((right_u-left_u) @ goal), mode=right['mode'], pusher=right['declared_pusher'],
            old_capability=left['capability']['classification'], new_capability=right['capability']['classification']))
    return dict(condition=old['condition'], frame=old['frame'], states=states,
        all_common_arguments_equal=all(all(r['common_arguments_equal'].values()) for r in states),
        all_original_filter_inputs_equal=all(all(r['original_filter_inputs_equal'].values()) for r in states),
        changed_commands=sum(r['command_change_norm'] > 0. for r in states),
        changed_commands_above_tolerance=sum(r['command_change_norm'] > 1e-7 for r in states),
        candidate_capability_counts=dict(Counter(r['new_capability'] for r in states)))


def main():
    guard()
    plan = study.check()
    outcome = read(INPUT / 'outcome.json')
    assert outcome['status'] == 'VERIFIED_ACTUAL_TRAJECTORY_ONSETS'
    assert outcome['analysis_sha256'] == sha(INPUT / 'analysis.json')
    analysis = read(INPUT / 'analysis.json')
    assert analysis['runtime_source_sha256'] == plan['frozen_source_sha256']
    inputs = read(INPUT / 'input_hashes.json')
    for case in plan['cases']:
        for name, expected in inputs[case['case_id']]['products'].items():
            assert sha(study.OUT / case['case_id'] / name) == expected
    sources = [Path(__file__), DESIGN, ROOT / 'scripts/stream_step_receipts.py',
               ROOT / 'scripts/probe_post_support_contact_direction.py', ROOT / 'scripts/research_io.py',
               ROOT / 'scripts/trace_boundary_map.py']
    identities = {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}
    OUT.mkdir(parents=True, exist_ok=False)
    for p in sources:
        shutil.copy2(p, OUT / p.name)
    identity = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), sources=identities,
        runtime_source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(study.PROTOCOL),
        onset_analysis_sha256=sha(INPUT / 'analysis.json'), new_physical_episodes=0)
    dump(OUT / 'started.json', identity)
    results, pairs = [], []
    try:
        for pair in analysis['pairs']:
            assert pair['first_divergence'] is not None
            cases = {c['arm']: c for c in plan['cases'] if c['condition_id'] == pair['condition']}
            matched = []
            for arm in study.ARMS:
                result = replay(cases[arm], pair['first_divergence'])
                dump(OUT / (result['case_id'] + '.json'), result)
                matched.append(result)
                results.append(result)
            comparison = compare(matched)
            pairs.append(comparison)
            print(json.dumps({key: value for key, value in comparison.items() if key != 'states'}), flush=True)
        study.check()
        assert all(sha(ROOT / name) == digest for name, digest in identities.items())
        report = dict(**identity, verified_frames=sum(r['verified_frames'] for r in results),
            systems=sum(len(r['states']) for r in results), pairs=pairs,
            scope='First actual command divergence; unchanged controller replay, complete original rows and fixed-state directional certificates; no physics, intervention or full-trajectory causal proof.')
        dump(OUT / 'analysis.json', report)
        dump(OUT / 'outcome.json', dict(**identity, status='VERIFIED_INITIAL_DIVERGENCE',
            analysis_sha256=sha(OUT / 'analysis.json'), verified_frames=report['verified_frames'], systems=report['systems']))
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, completed_cases=[r['case_id'] for r in results],
                                       traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
