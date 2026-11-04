"""Independent scalar certificate and NPZ/CSV verification for Phase1 diagnosis."""
from __future__ import annotations

from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math

import numpy as np
import cvxpy as cp

from research_io import ROOT, guard, read, sha
import post_acquisition_support_study as study

ONSET = ROOT / 'runs/research/2026-11-03-post-support-candidate-onset-v2'
FIRST = ROOT / 'runs/research/2026-11-03-post-support-initial-divergence-v1'
PUSH = ROOT / 'runs/research/2026-11-04-post-support-first-pusher-loss-v2'
OUT = ROOT / 'runs/maintenance/2026-11-03-candidate-onset-v1/independent-verification.json'


def certificate(A, b, goal, speed, result):
    assert len(A) == len(b) and all(len(row) == 2 for row in A)
    if result['status'] == 'VERIFIED_FEASIBLE':
        weights, u = result['dual_multipliers'], result['u']
        assert len(weights) == len(A) and all(w >= 0. for w in weights)
        residuals = [math.fsum(a*x for a, x in zip(row, u)) - rhs for row, rhs in zip(A, b)]
        assert min(residuals, default=0.) >= -1e-7 and math.hypot(*u) <= speed + 1e-7
        lower = math.fsum(g*x for g, x in zip(goal, u))
        c = [goal[j] + math.fsum(row[j]*w for row, w in zip(A, weights)) for j in range(2)]
        upper = speed*math.hypot(*c) - math.fsum(rhs*w for rhs, w in zip(b, weights))
        assert abs(upper - result['dual_upper_bound']) < 1e-12
        assert -1e-6 <= upper - lower <= 1e-6
        relaxed = upper + 1e-7*(math.hypot(*c) + math.fsum(weights))
        if result['classification'] == 'POSITIVE_CAPABILITY':
            assert lower > 1e-7
        elif result['classification'] == 'MUST_REVERSE':
            assert relaxed < -1e-7
        else:
            assert result['classification'] == 'BOUNDARY_OR_UNCERTAIN'
        return dict(classification=result['classification'], gap=upper-lower,
                    relaxed_upper=relaxed, minimum_residual=min(residuals, default=None))
    assert result['status'] == 'VERIFIED_INFEASIBLE'
    weights = result['separator_multipliers']
    assert len(weights) == len(A) and all(w >= 0. for w in weights)
    assert abs(math.fsum(weights) - 1.) < 1e-12
    c = [math.fsum(row[j]*w for row, w in zip(A, weights)) for j in range(2)]
    upper = speed*math.hypot(*c) - math.fsum(rhs*w for rhs, w in zip(b, weights))
    assert abs(upper - result['separator_upper_bound']) < 1e-12
    relaxed = upper + 1e-7*(math.hypot(*c) + math.fsum(weights))
    assert relaxed < -1e-7
    return dict(classification='INFEASIBLE', relaxed_upper=relaxed)


def check_objective(state):
    """Independently solve the recorded full-row closest-nominal objective."""
    A, b, nominal, command = [np.asarray(state[key]) for key in ('A', 'b', 'u_nom', 'command')]
    assert state['solver_status'] == 'optimal', state['solver_status']
    u = cp.Variable(2)
    problem = cp.Problem(cp.Minimize(cp.sum_squares(u-nominal)),
                         [A @ u >= b, cp.norm(u, 2) <= state['params']['max_speed']])
    problem.solve(solver=cp.CLARABEL, tol_gap_abs=1e-10, tol_feas=1e-10, tol_gap_rel=1e-10)
    assert problem.status == 'optimal'
    objective = float(np.sum((command-nominal)**2))
    assert abs(float(problem.value)-objective) < 1e-7
    assert np.min(A @ command-b) >= -1e-7
    forward_witness = np.asarray(state['capability'].get('u', [np.nan, np.nan]))
    return dict(agent_id=state['agent_id'], selected_objective=objective,
        independent_objective=float(problem.value), independent_command=np.asarray(u.value).tolist(),
        maximum_goal_witness_objective=float(np.sum((forward_witness-nominal)**2)),
        selected_goal_velocity=state['executed_goal_velocity'],
        maximum_goal_velocity=state['capability'].get('maximum_goal_velocity'))


def main():
    guard()
    plan = study.check()
    expected = ('VERIFIED_ACTUAL_TRAJECTORY_ONSETS', 'VERIFIED_INITIAL_DIVERGENCE',
                'VERIFIED_FIRST_PUSHER_FILTER_COMPARISON')
    identities = {}
    for folder, status in zip((ONSET, FIRST, PUSH), expected):
        outcome = read(folder / 'outcome.json')
        assert outcome['status'] == status and outcome['analysis_sha256'] == sha(folder / 'analysis.json')
        for name, digest in outcome['sources'].items():
            assert sha(ROOT / name) == digest and sha(folder / name.split('/')[-1]) == digest
        identities[folder.name] = outcome['analysis_sha256']
    onset = read(ONSET / 'analysis.json')
    case_lookup = {c['case_id']: c for c in plan['cases']}
    checked_frames = 0
    for episode in onset['episodes']:
        case_id = episode['case_id']
        folder = study.OUT / case_id
        with np.load(folder / 'replay.npz', allow_pickle=False) as z:
            centers, start, goal, pushes = [z[key].copy() for key in (
                'cargo/cargo_0/centers', 'task/cargo_0/start', 'task/cargo_0/direction', 'push_flags')]
        with np.load(ONSET / (case_id + '-commands.npz'), allow_pickle=False) as z:
            commands, nominals, admitted = [z[key].copy() for key in ('commands', 'nominal', 'support_admission')]
        with (ONSET / (case_id + '-frames.csv')).open(encoding='utf-8', newline='') as f:
            curves = list(csv.DictReader(f))
        assert len(curves) == episode['frames'] == len(commands) == len(centers)-1
        events, counts = {}, Counter()
        for i, row in enumerate(curves):
            assert int(row['frame']) == i+1
            progress = math.fsum((x-y)*g for x, y, g in zip(centers[i+1], start, goal))
            assert abs(progress-float(row['true_progress'])) < 1e-12
            frozen = row['reference_status'] == 'frozen'
            positive = nominals[i] @ goal > 1e-7
            nonpositive = commands[i] @ goal <= 1e-7
            blocked = frozen & pushes[i+1] & positive & nonpositive
            expected_counts = dict(agents=len(commands[i]), support_admitted=int(admitted[i].sum()),
                pushers=int(pushes[i+1].sum()), positive_nominal=int(positive.sum()),
                positive_nominal_nonpositive_command=int((positive & nonpositive).sum()),
                frozen_pusher_positive_nominal=int((frozen & pushes[i+1] & positive).sum()),
                frozen_pusher_positive_nominal_nonpositive_command=int(blocked.sum()),
                admitted_frozen_pusher_positive_nominal_nonpositive_command=int((blocked & admitted[i]).sum()))
            for key, value in expected_counts.items():
                assert int(row[key]) == value
            counts.update(expected_counts)
            flags = dict(first_frozen_reference=frozen, first_finite_support=bool(admitted[i].any()),
                first_positive_nominal_pusher_nonpositive_command=bool(blocked.any()),
                first_frozen_positive_force=frozen and float(row['positive_force_sum']) > 0.)
            for key, value in flags.items():
                if value:
                    events.setdefault(key, i+1)
        assert dict(counts) == episode['totals']
        assert {key: events.get(key) for key in episode['events']} == episode['events']
        checked_frames += len(curves)
    # A complete short original file also goes through the unmodified stdlib
    # whole-document decoder, independent of the new incremental reader.
    case_id = 'near_circle_map_bridge_control'
    with (study.OUT / case_id / 'paired_metrics.json').open(encoding='utf-8') as f:
        reference = json.load(f)
    with np.load(ONSET / (case_id + '-commands.npz')) as z:
        np.testing.assert_array_equal([r['commands'] for r in reference['steps']], z['commands'])
    systems, objectives = [], []
    for case in plan['cases']:
        data = read(FIRST / (case['case_id'] + '.json'))
        for state in data['states']:
            systems.append(certificate(state['A'], state['b'], state['goal'], state['params']['max_speed'], state['capability']))
    for condition in study.CONDITIONS:
        data = read(PUSH / (condition + '.json'))
        for item in data['states']:
            state, aggregate = item['actual'], item['aggregate']
            for model in (state, aggregate):
                systems.append(certificate(model['A'], model['b'], state['goal'], state['params']['max_speed'], model['capability']))
            if item['selected_pusher']:
                objectives.append(dict(condition=condition, frame=data['frame'], **check_objective(state)))
    assert checked_frames == 50677 and len(systems) == 256
    study.check()
    receipt = dict(status='VERIFIED_PHASE1_DIAGNOSTIC', checked_at_utc=datetime.now(timezone.utc).isoformat(),
        verifier_sha256=sha(__file__), input_analyses=identities, recorded_frames=checked_frames,
        systems=len(systems), classifications=dict(Counter(r['classification'] for r in systems)),
        maximum_primal_dual_gap=max(r.get('gap', 0.) for r in systems),
        standard_json_reference_frames=len(reference['steps']), selected_objective_checks=objectives,
        frozen_source_environment_config='UNCHANGED', new_physical_episodes=0)
    with OUT.open('x', encoding='utf-8') as f:
        json.dump(receipt, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(receipt), flush=True)


if __name__ == '__main__':
    main()
