"""Offline pair-responsibility witnesses at two fixed acquisition states."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import gzip
import json
import shutil

import cvxpy as cp
import numpy as np
from scipy.optimize import LinearConstraint, minimize

from research_io import ROOT, guard, read, sha

BASE = ROOT/'runs/research/2026-11-03-local-pair-feasibility-v1'
DESIGN = ROOT/'experiments/ccta2027/local_pair_feasibility_v1_design.md'
PRIOR = ROOT/'runs/research/2026-11-03-residual-coordination-v1/attempt2'


def validate(value, A, b, speed):
    if value is None:
        return dict(valid=False)
    value = np.asarray(value, dtype=float)
    residual = A@value-b
    speeds = [float(np.linalg.norm(value[:2])), float(np.linalg.norm(value[2:]))]
    valid = bool(np.isfinite(value).all() and residual.min() >= -1e-7 and max(speeds) <= speed+1e-7)
    return dict(valid=valid, command=value.tolist(), minimum_residual=float(residual.min()),
        row_residuals=residual.tolist(), speeds=speeds)


def paired_solutions(A, b, nominal, speed):
    u = cp.Variable(4)
    problem = cp.Problem(cp.Minimize(cp.sum_squares(u-nominal)),
        [A@u >= b, cp.norm(u[:2], 2) <= speed, cp.norm(u[2:], 2) <= speed])
    problem.solve(solver=cp.CLARABEL)
    clarabel = dict(status=problem.status, **validate(u.value, A, b, speed))
    def speed_slacks(value):
        return np.array([speed**2-value[:2]@value[:2], speed**2-value[2:]@value[2:]])
    def speed_jacobian(value):
        return np.array([[-2*value[0], -2*value[1], 0., 0.],
                         [0., 0., -2*value[2], -2*value[3]]])
    alternate = minimize(lambda value:float(np.sum((value-nominal)**2)), np.zeros(4),
        jac=lambda value:2*(value-nominal), method='SLSQP',
        constraints=[LinearConstraint(A, b, np.inf),
            dict(type='ineq', fun=speed_slacks, jac=speed_jacobian)],
        options=dict(ftol=1e-12, maxiter=1000))
    slsqp = dict(status=str(alternate.message), success=bool(alternate.success),
        **validate(alternate.x, A, b, speed))
    return dict(clarabel=clarabel, slsqp=slsqp,
        two_valid_witnesses=clarabel['valid'] and slsqp['valid'])


def main():
    guard()
    import finite_acquisition_study_v2 as study
    from analyze_residual_coordination import bounded_check
    from dbact.safety.filter import SafetyFilter, SafetyFilterParams
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, source_hashes
    plan = study.check_freeze()
    prior_outcome = read(PRIOR/'outcome.json')
    assert prior_outcome['status'] == 'VERIFIED_DIAGNOSTIC'
    assert sha(PRIOR/'analysis.json') == prior_outcome['analysis_sha256']
    prior = read(PRIOR/'analysis.json')
    assert prior['source_sha256'] == plan['frozen_source_sha256']
    BASE.mkdir(parents=True, exist_ok=False)
    scripts = ['scripts/analyze_local_pair_feasibility.py', 'scripts/analyze_residual_coordination.py',
        'scripts/qp_conflict_study.py', 'scripts/research_io.py', 'scripts/finite_acquisition_study_v2.py']
    for path in scripts:
        shutil.copy2(ROOT/path, BASE/ROOT.joinpath(path).name)
    shutil.copy2(DESIGN, BASE/'design.md')
    identity = dict(source_sha256=plan['frozen_source_sha256'], canonical_sha256=plan['canonical_sha256'],
        physical_protocol_sha256=sha(study.PROTOCOL), prior_analysis_sha256=sha(PRIOR/'analysis.json'),
        scripts={p:sha(ROOT/p) for p in scripts}, design_sha256=sha(DESIGN), new_physical_episodes=0,
        scope='Four same-state paired optimization problems; no runtime negotiation or closed-loop claim')
    dump(BASE/'identity.json', identity)
    systems = []
    try:
        manifest = read(study.OUT/'manifest.json')
        for residual in prior['residuals']:
            case = next(c for c in plan['cases'] if c['case_id'] == residual['case_id'])
            folder = study.OUT/case['case_id']
            receipt = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
            assert receipt['execution_status'] == 'COMPLETED'
            frame = residual['frame']
            metrics = read(folder/'paired_metrics.json')['steps'][frame-1]
            assert metrics['initial_reference']['status'] == 'acquiring'
            with np.load(folder/'replay.npz', allow_pickle=False) as saved:
                positions = saved['agent_positions'][frame-1].copy()
            env = SimulationEnvironment(case['config'], seed=case['seed'])
            assert env.controller.params.communication_dropout_prob == 0.
            for agent, position in zip(env.agents, positions):
                agent.position = position.copy()
            ids = [a.agent_id for a in env.agents]
            selected = [ids.index('agent_05'), ids.index('agent_09')]
            neighbors = env.controller._neighbor_indices(env.agents)
            alias = 'c' if case['condition_id'] == 'far_c_open_away_pos0' else 'skew'
            audit = study.AUDIT/alias
            audit_outcome = read(audit/'outcome.json')
            geometry_file = audit/'geometry_calls.jsonl.gz'
            assert sha(geometry_file) == audit_outcome['products'][geometry_file.name]
            with gzip.open(geometry_file, 'rt', encoding='utf-8') as stream:
                observations = {r['agent_id']:r for line in stream
                    if (r := json.loads(line))['frame'] == frame and r['agent_id'] in ('agent_05', 'agent_09')}
            assert len(observations) == 2
            safety = SafetyFilter(SafetyFilterParams(**residual['params']))
            for mode in ('full_rho', 'rho_free'):
                rho = safety.params.rho if mode == 'full_rho' else 0.
                agents, independent_checks, nominal = [], [], []
                for member, index in enumerate(selected):
                    aid, peer = ids[index], selected[1-member]
                    assert peer in neighbors[index]
                    observation = observations[aid]
                    assert observation['new_ready']
                    diag = next(d for d in metrics['diagnostics'] if d['agent_id'] == aid)
                    assert diag['acquisition_support']['applied']
                    assert diag['acquisition_support']['motion_bound'] == 0.
                    obj_A, h = np.asarray(observation['new_A']), np.asarray(observation['new_h'])
                    assert len(obj_A) == diag['acquisition_support']['object_rows']
                    agent_A, agent_b = safety._agent_rows(positions[index], positions[neighbors[index]])
                    wall_A, wall_b = safety._wall_rows(positions[index])
                    object_b = safety._cap_to_reachable(obj_A, -safety.params.gamma_obj*h+rho)
                    A, b = np.vstack([agent_A, wall_A, obj_A]), np.r_[agent_b, wall_b, object_b]
                    kinds = ['agent']*len(agent_A)+['wall']*len(wall_A)+['object']*len(obj_A)
                    mutual = neighbors[index].index(peer)
                    current_nominal = np.asarray(diag['nominal_command'])
                    speed = np.linalg.norm(current_nominal)
                    if speed > safety.params.max_speed:
                        current_nominal *= safety.params.max_speed/speed
                    nominal.extend(current_nominal.tolist())
                    original = bounded_check(A, b, safety.params.max_speed)
                    independent_checks.append(dict(agent_id=aid, result=original))
                    if member == 0:
                        original_system = next(s for s in residual['systems'] if s['kind'] == mode)
                        np.testing.assert_array_equal(A, original_system['A'])
                        np.testing.assert_array_equal(b, original_system['b'])
                    agents.append(dict(agent_id=aid, A=A.tolist(), b=b.tolist(), kinds=kinds,
                        mutual_index=mutual, neighbors=[ids[j] for j in neighbors[index]]))
                A_i, A_j = [np.asarray(a['A']) for a in agents]
                b_i, b_j = [np.asarray(a['b']) for a in agents]
                ii, jj = [a['mutual_index'] for a in agents]
                np.testing.assert_array_equal(A_i[ii], -A_j[jj])
                assert b_i[ii] == b_j[jj]
                keep_i, keep_j = np.arange(len(b_i)) != ii, np.arange(len(b_j)) != jj
                joint = np.r_[A_i[ii], A_j[jj]]
                A = np.vstack([np.c_[A_i[keep_i], np.zeros((keep_i.sum(), 2))],
                    np.c_[np.zeros((keep_j.sum(), 2)), A_j[keep_j]], joint])
                b = np.r_[b_i[keep_i], b_j[keep_j], b_i[ii]+b_j[jj]]
                solved = paired_solutions(A, b, np.asarray(nominal), safety.params.max_speed)
                result = dict(case_id=case['case_id'], frame=frame, kind=mode, agents=agents,
                    independent=independent_checks, coupled_A=A.tolist(), coupled_b=b.tolist(),
                    nominal=nominal, max_speed_per_agent=safety.params.max_speed,
                    saved_geometry_sha256=sha(geometry_file), receipt_sha256=sha(folder/'case_result.json'), **solved)
                for key in ('clarabel', 'slsqp'):
                    if solved[key]['valid']:
                        u = np.asarray(solved[key]['command'])
                        result[key]['original_mutual_half_residuals'] = [float(A_i[ii]@u[:2]-b_i[ii]),
                            float(A_j[jj]@u[2:]-b_j[jj])]
                        result[key]['mutual_relative_residual'] = float(joint@u-b[-1])
                systems.append(result)
                print(json.dumps(dict(case=case['case_id'], frame=frame, kind=mode,
                    independent_feasible=[r['result']['feasible'] for r in independent_checks],
                    pair_two_valid=solved['two_valid_witnesses'],
                    speeds=solved['clarabel'].get('speeds'))), flush=True)
        assert len(systems) == 4 and source_hashes() == plan['source_files']
        assert {s:sha(ROOT/s) for s in scripts} == identity['scripts']
        assert sha(DESIGN) == identity['design_sha256']
        analysis = dict(**identity, systems=systems, jointly_feasible_systems=sum(s['two_valid_witnesses'] for s in systems),
            denominator=4, status='COMPLETED_SAME_STATE_DIAGNOSIS')
        dump(BASE/'analysis.json', analysis)
        dump(BASE/'outcome.json', dict(status=analysis['status'], analysis_sha256=sha(BASE/'analysis.json'),
            new_physical_episodes=0, denominator=4, two_solver_valid=analysis['jointly_feasible_systems']))
    except Exception as exc:
        dump(BASE/'failure.json', dict(status='FAILED_DIAGNOSTIC', error=repr(exc), completed_systems=len(systems)))
        raise


if __name__ == '__main__':
    main()
