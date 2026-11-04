"""Compare actual finite filtering with aggregate rows at first pusher loss."""
from __future__ import annotations

from collections import Counter
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from replay_post_support_initial_divergence import replay, INPUT
from probe_post_support_contact_direction import maximize_direction
import post_acquisition_support_study as study
from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact_sim.experiments.deployment import dump

OUT = ROOT / 'runs/research/2026-11-04-post-support-first-pusher-loss-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_first_pusher_loss_v1_design.md'


def counterfactual(state):
    inputs = {key: value for key, value in state['inputs'].items()
              if key not in ('post_support_packets', 'post_support_trace')}
    assert inputs.get('finite_support_packets') is None
    safety = SafetyFilter(SafetyFilterParams(**state['params']))
    old = safety.prepare_velocity(np.asarray(state['position']), np.asarray(state['nominal_argument']),
        np.asarray(state['neighbor_positions']).reshape(-1, 2), **inputs)
    mask = np.asarray(state['kinds']) != 'object'
    old_mask = np.asarray(old.row_kinds) != 'object'
    np.testing.assert_array_equal(np.asarray(state['A']).reshape(-1, 2)[mask], old.A[old_mask])
    np.testing.assert_array_equal(np.asarray(state['b'])[mask], old.b_original[old_mask])
    np.testing.assert_array_equal(state['u_nom'], old.u_nom)
    result = safety.finish_velocity(old)
    goal = np.asarray(state['goal'])
    capability = maximize_direction(old.A, old.b_original, old.parameters.max_speed, goal)
    assert capability['status'] != 'INCONCLUSIVE'
    finite_A = np.asarray(state['A']).reshape(-1, 2)
    finite_b = np.asarray(state['b'])
    return plain(dict(agent_id=state['agent_id'], A=old.A, b=old.b_original, kinds=old.row_kinds,
        h=old.h_object, u_nom=old.u_nom, command=result.velocity,
        solver_status=result.status, barrier_scale=result.barrier_scale, capability=capability,
        goal_velocity=float(result.velocity @ goal), hard_rows_equal=True,
        aggregate_command_min_finite_residual=float(np.min(finite_A @ result.velocity-finite_b)) if len(finite_b) else None))


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    OUT = parser.parse_args().out.resolve()
    if not OUT.is_relative_to(ROOT / 'runs'):
        raise ValueError('New analysis output must be under this repository runs/')
    guard()
    plan = study.check()
    outcome = read(INPUT / 'outcome.json')
    assert outcome['status'] == 'VERIFIED_ACTUAL_TRAJECTORY_ONSETS'
    assert outcome['analysis_sha256'] == sha(INPUT / 'analysis.json')
    analysis = read(INPUT / 'analysis.json')
    inputs = read(INPUT / 'input_hashes.json')
    schedule = []
    for condition in study.CONDITIONS:
        case = next(c for c in plan['cases'] if c['condition_id'] == condition and c['arm'] == 'post_support_candidate')
        episode = next(e for e in analysis['episodes'] if e['case_id'] == case['case_id'])
        frame = episode['events']['first_positive_nominal_pusher_nonpositive_command']
        assert type(frame) is int and frame > 0
        for name, digest in inputs[case['case_id']]['products'].items():
            assert sha(study.OUT / case['case_id'] / name) == digest
        schedule.append(dict(case=case, frame=frame))
    sources = [Path(__file__), DESIGN, ROOT / 'scripts/replay_post_support_initial_divergence.py',
        ROOT / 'scripts/probe_post_support_contact_direction.py', ROOT / 'scripts/stream_step_receipts.py',
        ROOT / 'scripts/research_io.py', ROOT / 'scripts/trace_boundary_map.py',
        ROOT / 'tests/test_first_pusher_loss.py']
    identities = {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}
    OUT.mkdir(parents=True, exist_ok=False)
    for p in sources:
        shutil.copy2(p, OUT / p.name)
    identity = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), sources=identities,
        runtime_source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(study.PROTOCOL),
        onset_analysis_sha256=sha(INPUT / 'analysis.json'), new_physical_episodes=0)
    dump(OUT / 'started.json', dict(**identity, schedule=schedule))
    results = []
    try:
        for item in schedule:
            actual = replay(item['case'], item['frame'])
            states = []
            for state in actual['states']:
                aggregate = counterfactual(state)
                selected = (state['declared_pusher'] and state['nominal_goal_velocity'] > 1e-7
                            and state['executed_goal_velocity'] <= 1e-7)
                states.append(dict(actual=state, aggregate=aggregate, selected_pusher=selected))
            assert any(s['selected_pusher'] for s in states)
            report = dict(condition=item['case']['condition_id'], frame=item['frame'],
                verified_frames=actual['verified_frames'], states=states,
                selected_pusher_ids=[s['actual']['agent_id'] for s in states if s['selected_pusher']])
            dump(OUT / (report['condition'] + '.json'), report)
            results.append(report)
            print(json.dumps(dict(condition=report['condition'], frame=report['frame'],
                selected=[dict(agent=s['actual']['agent_id'], nominal=s['actual']['nominal_goal_velocity'],
                    actual=s['actual']['executed_goal_velocity'], aggregate=s['aggregate']['goal_velocity'],
                    finite_capability=s['actual']['capability']['classification'],
                    aggregate_capability=s['aggregate']['capability']['classification'])
                    for s in states if s['selected_pusher']])), flush=True)
        selected = [s for r in results for s in r['states'] if s['selected_pusher']]
        study.check()
        assert all(sha(ROOT / name) == digest for name, digest in identities.items())
        report = dict(**identity, verified_frames=sum(r['verified_frames'] for r in results),
            states=sum(len(r['states']) for r in results), selected_pushers=len(selected),
            selected_finite_capabilities=dict(Counter(s['actual']['capability']['classification'] for s in selected)),
            selected_aggregate_capabilities=dict(Counter(s['aggregate']['capability']['classification'] for s in selected)),
            selected_positive_aggregate_commands=sum(s['aggregate']['goal_velocity'] > 1e-7 for s in selected),
            scope='Actual candidate first-pusher events; same-input isolated aggregate filtering only, not a new physical/controller trajectory.')
        dump(OUT / 'analysis.json', report)
        dump(OUT / 'outcome.json', dict(**identity, status='VERIFIED_FIRST_PUSHER_FILTER_COMPARISON',
            analysis_sha256=sha(OUT / 'analysis.json'), verified_frames=report['verified_frames'],
            states=report['states'], selected_pushers=len(selected)))
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, completed_conditions=[r['condition'] for r in results],
                                       traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
