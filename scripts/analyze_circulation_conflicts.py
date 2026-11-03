"""Independently assess two preserved pre-freeze circulation QP conflicts."""
from __future__ import annotations

import json
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha
from qp_conflict_study import independent

BASE = ROOT/'runs/research/2026-11-03-circulation-conflict-witness-v1'
PUBLIC = ROOT/'docs/results/circulation_conflict_witness_v1'
DESIGN = ROOT/'experiments/ccta2027/circulation_conflict_witness_v1_design.md'
PROTOCOL = ROOT/'experiments/ccta2027/acquisition_circulation_v1.json'
TRACE = ROOT/'runs/research/2026-11-03-contact-activation-v1'


def main():
    guard()
    assert not BASE.exists() and not PUBLIC.exists()
    assert not any(k=='dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    plan = read(PROTOCOL)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name)==digest
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact.safety.filter import SafetyFilter, SafetyFilterParams
    from dbact.safety.qp2d import solve_min_norm_2d
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result

    BASE.mkdir(parents=True, exist_ok=False)
    sources = ('scripts/analyze_circulation_conflicts.py', 'scripts/qp_conflict_study.py', 'scripts/research_io.py')
    for name in sources:
        shutil.copy2(ROOT/name, BASE/ROOT.joinpath(name).name)
    shutil.copy2(DESIGN, BASE/'design.md')
    identity = dict(source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(PROTOCOL),
        scripts={name:sha(ROOT/name) for name in sources}, design_sha256=sha(DESIGN),
        new_physical_episodes=0, selection='The previously saved first scaled state in each far circulation case, strictly before capture freeze; two states only')
    dump(BASE/'identity.json', identity)
    report = dict(**identity, witnesses={})
    manifest = read(batch/'manifest.json')
    for alias in ('c-candidate', 'skew-candidate'):
        folder = TRACE/alias
        outcome = read(folder/'outcome.json')
        assert outcome['status']=='EXACT_RECONSTRUCTION'
        assert outcome['source_sha256']==plan['frozen_source_sha256']
        assert outcome['protocol_sha256']==sha(PROTOCOL)
        witness_path = folder/'witnesses.json'
        assert sha(witness_path)==outcome['products']['witnesses.json']
        case = next(c for c in plan['cases'] if c['case_id']==outcome['case_id'])
        receipt = batch/case['case_id']/'case_result.json'
        assert sha(receipt)==outcome['original_receipt_sha256']
        old_result = cached_result(receipt.parent, {**manifest['identity'], 'case_sha256':encode(case)})
        acquisition = old_result['initial_reference']
        assert acquisition['zero_epoch_valid'] and acquisition['max_pre_epoch_translation']==0 and acquisition['max_pre_epoch_rotation']==0
        w = read(witness_path)['conflicts']['first']
        assert (w['frame']-1)*case['config']['dt'] < acquisition['epoch']
        assert w['observation']['map_reference']['status']=='stationary_acquisition'
        f = w['geometry']['filter']
        safety = SafetyFilter(SafetyFilterParams(**f['params']))
        A, full, free = [np.asarray(f[k]) for k in ('A', 'b_original', 'b_no_margin')]
        obj = np.asarray(f['row_kinds'])=='object'
        position = np.asarray(w['geometry']['position'])
        points, normals, velocity = [np.asarray(f[k]) for k in ('boundary_points', 'boundary_normals', 'object_velocity')]
        velocities = None if f['point_velocities'] is None else np.asarray(f['point_velocities'])
        trace = {}
        ob_A, ob_b, ob_free, h = safety._object_rows(position, points, normals, velocity, velocities, trace=trace)
        for actual, expected in ((ob_A, A[obj]), (ob_b, full[obj]), (ob_free, free[obj])):
            np.testing.assert_array_equal(actual, expected)
        zero_A, _, zero_free, zero_h = safety._object_rows(position, points, normals, np.zeros(2),
            None if velocities is None else np.zeros_like(velocities))
        np.testing.assert_array_equal(zero_A, ob_A)
        np.testing.assert_array_equal(zero_h, h)
        zero_rhs = free.copy()
        zero_rhs[obj] = zero_free
        checks = {}
        for label, a, b in (('full', A, full), ('no_rho', A, free),
                ('agent_only', A[~obj], free[~obj]), ('object_only', A[obj], free[obj]),
                ('zero_estimated_motion', A, zero_rhs)):
            exact = solve_min_norm_2d(np.zeros(2), a, b, f['params']['max_speed'])
            cross = independent(a, b, f['params']['max_speed'])
            assert cross['numerically_valid'] and cross['feasible']==bool(exact.feasible)
            checks[label] = dict(exact_feasible=bool(exact.feasible), independent=cross)
        no_cap = independent(A, free, None)
        assert no_cap['numerically_valid']
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        true_gap, true_normal, foot = env.cargoes[0].signed_distance(position.reshape(1, 2))
        np.testing.assert_allclose(true_gap[0], w['observation']['filter']['true_boundary_gap'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(true_normal[0], w['observation']['filter']['true_normal'])
        report['witnesses'][alias] = dict(case_id=case['case_id'], frame=w['frame'], agent_id=w['agent_id'],
            phase=w['phase'], recorded_scale=w['scale'], original_receipt_sha256=sha(receipt),
            outcome_sha256=sha(folder/'outcome.json'), witness_file_sha256=sha(witness_path),
            witness=w, object_trace=trace, h_object=h.tolist(),
            true_boundary_distance_m=float(true_gap[0]), true_safety_margin_m=float(true_gap[0]-f['params']['r_safe']),
            aggregate_normal_true_normal_alignment=(ob_A@true_normal[0]).tolist(),
            b_zero_estimated_motion=zero_rhs.tolist(), feasibility=checks, no_rho_without_speed_cap=no_cap)
        print(json.dumps(dict(alias=alias, frame=w['frame'], h=h.tolist(),
            true_margin=float(true_gap[0]-f['params']['r_safe']),
            feasible={k:v['exact_feasible'] for k,v in checks.items()}, no_cap=no_cap['feasible'])), flush=True)
    report['scope'] = 'Two preserved old acquisition states only; no additional controller replay or physical trial. Truth is observer-only. Any transfer to the new trial requires its separate exact acquisition-prefix verification.'
    dump(BASE/'analysis.json', report)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    shutil.copy2(BASE/'analysis.json', PUBLIC/'analysis.json')
    dump(PUBLIC/'publication.json', dict(analysis_sha256=sha(PUBLIC/'analysis.json'),
        script_sha256=sha(__file__), design_sha256=sha(DESIGN), new_physical_episodes=0))


if __name__ == '__main__':
    main()
