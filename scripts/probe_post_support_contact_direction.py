"""Bound goal-directed velocity over the complete saved safety constraints."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import traceback

import cvxpy as cp
import clarabel
import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from dbact_sim.experiments.deployment import dump


CONTACT = ROOT / 'runs/research/2026-11-03-post-support-contact-compatibility-v2'
GEOMETRY = ROOT / 'runs/research/2026-11-03-post-support-contact-packet-geometry-v1'
OUT = ROOT / 'runs/research/2026-11-03-post-support-contact-direction-capability-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_contact_direction_capability_v1_design.md'


def geometric_maximum(A, b, speed, goal):
    """Enumerate vertices and speed-circle extrema of a planar convex set."""
    candidates = [np.zeros((1, 2)), (speed*goal/np.linalg.norm(goal)).reshape(1, 2)]
    norm2 = np.sum(A*A, axis=1)
    live = norm2 > 1e-24
    Al, bl, nl = A[live], b[live], norm2[live]
    if len(Al):
        feet = (bl/nl)[:, None]*Al
        candidates.append(feet)
        gap = speed*speed-np.sum(feet*feet, axis=1)
        cutting = gap >= 0.
        if np.any(cutting):
            tangent = np.column_stack([-Al[cutting, 1], Al[cutting, 0]])/np.sqrt(nl[cutting])[:, None]
            offset = np.sqrt(gap[cutting])[:, None]*tangent
            candidates.extend([feet[cutting]+offset, feet[cutting]-offset])
        i, j = np.triu_indices(len(Al), 1)
        det = Al[i, 0]*Al[j, 1]-Al[i, 1]*Al[j, 0]
        usable = np.abs(det) > 1e-12
        if np.any(usable):
            i, j, det = i[usable], j[usable], det[usable]
            candidates.append(np.column_stack([(bl[i]*Al[j, 1]-bl[j]*Al[i, 1])/det,
                (Al[i, 0]*bl[j]-Al[j, 0]*bl[i])/det]))
    points = np.vstack(candidates)
    feasible = (np.linalg.norm(points, axis=1) <= speed+1e-7)
    if len(b):
        feasible &= np.all(points @ A.T >= b[None, :]-1e-7, axis=1)
    if not np.any(feasible):
        return None
    points = points[feasible]
    return points[int(np.argmax(points @ goal))]


def maximize_direction(A, b, speed, goal):
    A, b, goal = np.asarray(A, dtype=float).reshape(-1, 2), np.asarray(b, dtype=float), np.asarray(goal, dtype=float)
    assert b.shape == (len(A),) and goal.shape == (2,)
    assert np.isfinite(A).all() and np.isfinite(b).all() and np.isfinite(goal).all()
    assert np.isfinite(speed) and speed > 0. and abs(np.linalg.norm(goal)-1.) < 1e-10
    geometric = geometric_maximum(A, b, speed, goal)
    if not len(b):
        return dict(status='VERIFIED_FEASIBLE', classification='POSITIVE_CAPABILITY',
            u=geometric.tolist(), maximum_goal_velocity=speed, dual_upper_bound=speed,
            dual_multipliers=[], speed=speed, minimum_residual=None, solver_status='EMPTY_ROWS')
    u = cp.Variable(2)
    rows = A @ u >= b
    problem = cp.Problem(cp.Maximize(goal @ u), [rows, cp.norm(u, 2) <= speed])
    problem.solve(solver=cp.CLARABEL)
    result = dict(status='INCONCLUSIVE', classification='INCONCLUSIVE', solver_status=problem.status,
        geometric_feasible=geometric is not None)
    if problem.status in ('optimal', 'optimal_inaccurate') and u.value is not None:
        value = np.asarray(u.value).reshape(2)
        raw_dual = np.asarray(rows.dual_value).reshape(-1)
        multipliers = np.maximum(raw_dual, 0.)
        upper = float(speed*np.linalg.norm(goal+A.T @ multipliers)-b @ multipliers)
        result.update(solver_u=value.tolist(), solver_goal_velocity=float(goal @ value),
            solver_minimum_residual=float(np.min(A @ value-b)), solver_speed=float(np.linalg.norm(value)),
            raw_dual_minimum=float(raw_dual.min()), dual_multipliers=multipliers.tolist(), dual_upper_bound=upper)
        if geometric is not None:
            lower = float(goal @ geometric)
            gap = upper-lower
            result.update(u=geometric.tolist(), maximum_goal_velocity=lower,
                minimum_residual=float(np.min(A @ geometric-b)), speed=float(np.linalg.norm(geometric)),
                primal_dual_gap=gap)
            valid = (result['solver_minimum_residual'] >= -1e-7 and result['solver_speed'] <= speed+1e-7
                and result['minimum_residual'] >= -1e-7 and result['speed'] <= speed+1e-7
                and abs(result['solver_goal_velocity']-lower) <= 1e-6 and -1e-6 <= gap <= 1e-6)
            if valid:
                assert not (lower > 1e-7 and upper < -1e-7)
                classification = ('POSITIVE_CAPABILITY' if lower > 1e-7 else
                    'MUST_REVERSE' if upper < -1e-7 else 'BOUNDARY_OR_UNCERTAIN')
                result.update(status='VERIFIED_FEASIBLE', classification=classification)
    elif problem.status in ('infeasible', 'infeasible_inaccurate') and geometric is None:
        lam = cp.Variable(len(b))
        separator = cp.Problem(cp.Minimize(speed*cp.norm(A.T @ lam, 2)-b @ lam),
            [lam >= 0., cp.sum(lam) == 1.])
        separator.solve(solver=cp.CLARABEL)
        result['separator_solver_status'] = separator.status
        if lam.value is not None:
            weights = np.maximum(np.asarray(lam.value).reshape(-1), 0.)
            if np.isfinite(weights).all() and weights.sum() > 0.:
                weights /= weights.sum()
                bound = float(speed*np.linalg.norm(A.T @ weights)-b @ weights)
                result.update(separator_multipliers=weights.tolist(), separator_upper_bound=bound)
                if bound < -1e-7:
                    result.update(status='VERIFIED_INFEASIBLE', classification='INFEASIBLE')
    return result


def main():
    import post_acquisition_support_study as study
    guard()
    plan = study.check()
    contact, geometry = read(CONTACT / 'analysis.json'), read(GEOMETRY / 'analysis.json')
    for folder, status in ((CONTACT, 'VERIFIED_CONTACT_COMPATIBILITY'), (GEOMETRY, 'VERIFIED_PACKET_GEOMETRY')):
        outcome = read(folder / 'outcome.json')
        assert outcome['status'] == status and outcome['analysis_sha256'] == sha(folder / 'analysis.json')
    assert geometry['contact_analysis_sha256'] == sha(CONTACT / 'analysis.json')
    geom = {(r['condition'], r['frame'], r['agent_id']): r for r in geometry['states']}
    goals = {}
    for item in contact['schedule']:
        case = item['case']
        folder = study.OUT / case['case_id']
        assert sha(folder / 'case_result.json') == item['receipt_sha256']
        with np.load(folder / 'replay.npz', allow_pickle=False) as saved:
            goal = saved['task/cargo_0/direction'].reshape(2).tolist()
        goals[case['condition_id']] = dict(direction=goal, replay_sha256=sha(folder / 'replay.npz'))
    paths = [Path(__file__), DESIGN, ROOT / 'tests/test_contact_direction_capability.py',
        ROOT / 'scripts/research_io.py', ROOT / 'scripts/trace_boundary_map.py']
    sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}
    identity = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), sources=sources,
        contact_analysis_sha256=sha(CONTACT / 'analysis.json'), packet_geometry_sha256=sha(GEOMETRY / 'analysis.json'),
        runtime_source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(study.PROTOCOL),
        goals=goals, solver_versions=dict(cvxpy=cp.__version__, clarabel=clarabel.__version__, numpy=np.__version__),
        new_physical_episodes=0)
    OUT.mkdir(parents=True, exist_ok=False)
    for path in paths:
        shutil.copy2(path, OUT / path.name)
    dump(OUT / 'started.json', identity)
    results = []
    try:
        for condition in contact['results']:
            cid = condition['condition']
            goal = goals[cid]['direction']
            captured = read(GEOMETRY / (cid+'-packets.json'))
            parameters_by_key = {(c['frame'], c['agent_id']): c['parameters'] for c in captured['captures']}
            for event in condition['events']:
                for state in event['states']:
                    key = (cid, event['saved_frame'], state['agent_id'])
                    current = geom[key]
                    result = dict(condition=cid, frame=key[1], agent_id=key[2],
                        admitted=current['admitted'], guard=current['guard'],
                        positive_force=current['positive_force'], declared_pusher=current['declared_pusher'],
                        original_goal_velocity=state['original_goal_velocity'],
                        counterfactual_goal_velocity=state['counterfactual_goal_velocity'], systems={})
                    if result['admitted']:
                        parameters = parameters_by_key[key[1], key[2]]
                        speed = parameters['max_speed']
                        for name, matrix in (('original_aggregate', state['original_rows']),
                                             ('finite_capped', state['counterfactual_rows']),
                                             ('finite_uncapped', state['counterfactual_rows'])):
                            A, b = np.asarray(matrix['A']), np.asarray(matrix['b']).copy()
                            if name == 'finite_uncapped':
                                mask = np.asarray(matrix['kinds']) == 'object'
                                b[mask] = state['support']['demand_speed']-parameters['gamma_obj']*np.asarray(matrix['h'])+parameters['rho']
                            solved = maximize_direction(A, b, speed, goal)
                            result['systems'][name] = dict(A=A, b=b, kinds=matrix['kinds'],
                                speed_limit=speed, goal=goal, **solved)
                    results.append(result)
            dump(OUT / (cid+'.json'), plain(dict(condition=cid, states=[r for r in results if r['condition'] == cid])))
            print(json.dumps(dict(condition=cid, states=sum(r['condition'] == cid for r in results))), flush=True)
        assert len(results) == 128 and sum(r['admitted'] for r in results) == 112
        counts = {}
        for name, rows in (('all_admitted', [r for r in results if r['admitted']]),
                           ('positive_force', [r for r in results if r['positive_force']]),
                           ('positive_force_pusher', [r for r in results if r['positive_force'] and r['declared_pusher']])):
            counts[name] = dict(states=len(rows), systems={model: dict(Counter(r['systems'][model]['classification'] for r in rows))
                for model in ('original_aggregate', 'finite_capped', 'finite_uncapped')})
        study.check()
        assert all(sha(ROOT / name) == digest for name, digest in sources.items())
        dump(OUT / 'analysis.json', plain(dict(**identity, counts=counts, states=results,
            scope='Local goal-velocity capability under fixed saved constraints. Positive speed is not a contact-force or closed-loop success certificate.')))
        dump(OUT / 'outcome.json', dict(**identity, status='COMPLETED_DIRECTIONAL_DIAGNOSTIC',
            analysis_sha256=sha(OUT / 'analysis.json'), states=128, systems=336,
            inconclusive=sum(s['status'] == 'INCONCLUSIVE' for r in results for s in r['systems'].values())))
        print(json.dumps(counts), flush=True)
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, completed_states=len(results), traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
