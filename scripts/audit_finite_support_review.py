"""Exact historical control replay with corrected geometry as an observer only."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
from collections import Counter
from dataclasses import asdict
import gzip
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from replay_qp_conflict import recorded
from trace_boundary_map import plain

PROTOCOL = ROOT/'experiments/ccta2027/finite_acquisition_support_v1.json'
DESIGN = ROOT/'experiments/ccta2027/finite_support_review_validation_v1_design.md'
CORRECTION = ROOT/'docs/results/finite_acquisition_support_v1/post_review_checks.json'


class WitnessSelector:
    def __init__(self):
        self.changed, self.violated = set(), set()

    def select(self, condition, *, changed, ready, violates, residual):
        rules = ['original_residual'] if residual else []
        if ready and changed and condition not in self.changed:
            rules.append('first_changed_admitted')
            self.changed.add(condition)
        if ready and violates and condition not in self.violated:
            rules.append('first_old_command_violation')
            self.violated.add(condition)
        return rules


def trace(condition, destination, frames=None):
    guard()
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    plan, correction = read(PROTOCOL), read(CORRECTION)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name) == digest, name
    for name, digest in correction['source_files'].items():
        assert sha(ROOT/name) == digest, name
    assert plan['frozen_source_sha256'] == correction['original_physical_source_sha256']
    assert sha(PROTOCOL) == correction['original_protocol_sha256']
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions
    from dbact.sensing.finite_support import support_rows as old_geometry
    from dbact.safety.qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy
    assert environment_versions() == plan['environment'] == correction['environment']
    case = next(c for c in plan['cases'] if c['condition_id'] == condition and c['arm'] == 'finite_acquisition_support')
    manifest = read(batch/'manifest.json')
    assert manifest['identity']['protocol_sha256'] == sha(PROTOCOL)
    folder = batch/case['case_id']
    receipt = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert receipt['execution_status'] == 'COMPLETED'
    rows = read(folder/'paired_metrics.json')['steps']
    freeze_frame = next(r['frame'] for r in rows if r['initial_reference']['status'] == 'frozen')
    end_frame = freeze_frame if frames is None else frames
    assert 0 < end_frame <= freeze_frame
    published = read(ROOT/'docs/results/finite_acquisition_support_v1/analysis.json')
    episode = next(r for r in published['episodes'] if r['case_id'] == case['case_id'])
    assert episode['receipt_sha256'] == sha(folder/'case_result.json')
    expected = {(e['frame'], aid) for e in episode['scaling']['events'] if e['frame'] <= end_frame for aid in e['agents']}
    out = Path(destination).resolve()
    assert out.is_relative_to(ROOT/'runs')
    out.mkdir(parents=True, exist_ok=False)
    helpers = ('audit_finite_support_review.py', 'research_io.py', 'replay_qp_conflict.py', 'trace_boundary_map.py')
    for name in helpers:
        shutil.copy2(ROOT/'scripts'/name, out/name)
    shutil.copy2(DESIGN, out/'design.md')
    shutil.copy2(ROOT/'src/dbact/sensing/finite_support.py', out/'corrected_finite_support.py')
    spec = importlib.util.spec_from_file_location('corrected_finite_geometry', out/'corrected_finite_support.py')
    geometry = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = geometry
    spec.loader.exec_module(geometry)
    identity = dict(case_id=case['case_id'], condition=condition, end_frame=end_frame,
        freeze_frame=freeze_frame, protocol_sha256=sha(PROTOCOL),
        original_source_sha256=plan['frozen_source_sha256'], corrected_source_sha256=correction['corrected_runtime_source_sha256'],
        candidate_module_sha256=sha(out/'corrected_finite_support.py'), environment=plan['environment'],
        original_receipt_sha256=sha(folder/'case_result.json'), design_sha256=sha(DESIGN),
        scripts={name:sha(out/name) for name in helpers}, new_physical_episodes=0)
    dump(out/'identity.json', identity)
    verified, current_frame, call_index = 0, 0, 0
    counters, transitions, new_reasons = Counter(), Counter(), Counter()
    witnesses, observed_residuals = [], set()
    selector = WitnessSelector()
    started = time.perf_counter()
    try:
        with np.load(folder/'replay.npz', allow_pickle=False) as z:
            saved = {k:z[k].copy() for k in ('times', 'agent_positions', 'cargo/cargo_0/centers',
                'cargo/cargo_0/angles', 'sensed_counts', 'sensed_points')}
        offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        c = env.controller
        c.trace_enabled = True
        c._ensure_state(env.agents)
        assert c.params.perception_every == c.params.planning_every == 1
        original_filter = c.safety.filter_velocity

        with gzip.open(out/'geometry_calls.jsonl.gz', 'wt', encoding='utf-8') as stream:
            def filtered(position, nominal_velocity, neighbor_positions=(), **kwargs):
                nonlocal call_index
                aid = env.agents[call_index].agent_id
                call_index += 1
                counters['filter_calls'] += 1
                answer = original_filter(position, nominal_velocity, neighbor_positions, **kwargs)
                residual = answer.status == 'scaled_barrier'
                if residual:
                    observed_residuals.add((current_frame, aid))
                if 'finite_support_packets' not in kwargs:
                    counters['without_finite_packets'] += 1
                    assert not residual, 'Residual outside supplied support needs an explicit design amendment'
                    return answer
                assert c.initial_reference_status == 'acquiring'
                counters['supplied_geometry_calls'] += 1
                inputs = (position, kwargs['finite_support_packets'], c.safety.params,
                          kwargs['observation_time'], kwargs['finite_support_motion_bound'])
                old = old_geometry(*inputs, isolated_point_guards=True)
                new = geometry.support_rows(*inputs, isolated_point_guards=True)
                assert old.ready == kwargs['finite_support_trace']['applied']
                object_mask = np.asarray(answer.row_kinds, dtype=str) == 'object'
                if old.ready:
                    np.testing.assert_array_equal(old.A, answer.A_rows[object_mask])
                    np.testing.assert_array_equal(old.h, answer.h_object)
                changed = (old.ready != new.ready or not np.array_equal(old.A, new.A)
                           or not np.array_equal(old.h, new.h))
                counters['geometry_changed'] += int(changed)
                counters['original_admitted'] += int(old.ready)
                counters['corrected_admitted'] += int(new.ready)
                transitions[f'{int(old.ready)}->{int(new.ready)}'] += 1
                new_reasons[new.reason] += 1
                entry = dict(frame=current_frame, agent_id=aid, original_scaled=residual,
                    old_ready=old.ready, new_ready=new.ready, old_reason=old.reason, new_reason=new.reason,
                    old_rows=len(old.A), new_rows=len(new.A), new_point_guards=new.point_guard_rows,
                    changed=changed, solves=[], old_command=answer.velocity.tolist())
                violates = False
                matrices = []
                if new.ready:
                    motion = kwargs['finite_support_motion_bound']
                    hard_A, hard_b = answer.A_rows[~object_mask], answer.b_original[~object_mask]
                    nominal = np.asarray(nominal_velocity, dtype=float).reshape(2)
                    speed = np.linalg.norm(nominal)
                    if speed > c.safety.params.max_speed:
                        nominal = nominal*(c.safety.params.max_speed/speed)
                    for rho, label in ((c.safety.params.rho, 'full_rho'), (0., 'rho_free')):
                        demand = motion-c.safety.params.gamma_obj*new.h+rho
                        object_b = c.safety._cap_to_reachable(new.A, demand)
                        A, b = np.vstack([hard_A, new.A]), np.r_[hard_b, object_b]
                        result = solve_min_norm_2d(nominal, A, b, c.safety.params.max_speed)
                        margin = float(np.min(new.A@answer.velocity-object_b))
                        violates |= margin < -1e-7
                        entry['solves'].append(dict(kind=label, rho=rho, feasible=result.feasible,
                            cap_rows=int(np.count_nonzero(object_b < demand-1e-15)),
                            old_command_minimum_row_residual=margin, command=result.u.tolist()))
                        counters[f'{label}_feasible'] += int(result.feasible)
                        counters[f'{label}_old_command_violation'] += int(margin < -1e-7)
                        matrices.append((A, b, result.feasible))
                    entry.update(new_A=new.A.tolist(), new_h=new.h.tolist())
                rules = selector.select(condition, changed=changed, ready=new.ready, violates=violates, residual=residual)
                if rules:
                    witness = dict(**entry, selection_rules=rules, params=asdict(c.safety.params),
                        original_A=answer.A_rows.tolist(), original_b=answer.b_original.tolist(),
                        original_b_no_margin=answer.b_no_margin.tolist(), original_row_kinds=answer.row_kinds,
                        corrected_counterfactual=[])
                    for (A, b, expected_feasible), solved in zip(matrices, entry['solves']):
                        independent = solve_min_norm_2d_cvxpy(nominal, A, b, c.safety.params.max_speed)
                        assert independent.feasible == expected_feasible, (current_frame, aid, solved['kind'])
                        counters['independent_qp_comparisons'] += 1
                        witness['corrected_counterfactual'].append(dict(kind=solved['kind'], A=A.tolist(),
                            b=b.tolist(), nominal=nominal.tolist(), independent_status=independent.detail,
                            independent_feasible=independent.feasible))
                    witnesses.append(plain(witness))
                stream.write(json.dumps(plain(entry), separators=(',', ':'), allow_nan=False)+'\n')
                return answer

            c.safety.filter_velocity = filtered
            for k in range(end_frame):
                current_frame, call_index = k+1, 0
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
                verified += 1
                if verified % 100 == 0:
                    print(json.dumps(dict(case=case['case_id'], verified=verified, changed=counters['geometry_changed'])), flush=True)
        assert observed_residuals == expected, (observed_residuals, expected)
        assert counters['filter_calls'] == verified*len(env.agents)
        if frames is None:
            assert c.initial_reference_status == 'frozen'
            assert all(not d.acquisition_support['applied'] for d in c.diagnostics)
        dump(out/'witnesses.json', dict(witnesses=witnesses, new_physical_episodes=0))
        dump(out/'summary.json', dict(counters=dict(counters), admission_transitions=dict(transitions),
            corrected_reasons=dict(new_reasons), original_residuals=sorted(observed_residuals),
            scope='Original exact controller replay with corrected same-state geometry/QPs; no physics or adoption'))
        outcome = dict(**identity, status='EXACT_RECONSTRUCTION', verified_frames=verified,
            elapsed_seconds=time.perf_counter()-started,
            products={name:sha(out/name) for name in ('geometry_calls.jsonl.gz', 'witnesses.json', 'summary.json')})
        dump(out/'outcome.json', outcome)
        print(json.dumps(dict(case=case['case_id'], status=outcome['status'], verified=verified,
                             counters=dict(counters))), flush=True)
    except Exception as exc:
        dump(out/'failure.json', dict(**identity, status='INVALID_RECONSTRUCTION', verified_frames=verified,
            failing_frame=current_frame, error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--condition', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--frames', type=int)
    args = parser.parse_args()
    trace(args.condition, args.out, args.frames)
