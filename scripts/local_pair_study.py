"""Freeze, run and analyze four original finite-support controls and four pairs."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import gc
import json
import random
import shutil
import traceback
import xml.etree.ElementTree as ET

import numpy as np

from research_io import ROOT, guard, read, sha
from local_pair_audit import ARMS, audit_episode, evaluate_gates
from task_translation_physical_study_v2 import episode, reproduction, package_inventory
from reference_map_study import identity, scaling
from finite_acquisition_study_v2 import support_audit
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import (
    cached_result, environment_versions, run_study, source_hashes, verify_source_snapshot)

MISSION = '2026-11-03-local-pair-safety-v1'
BASE = ROOT/'runs/research'/MISSION
OUT, FREEZE = BASE/'run', BASE/'freeze'
CHECKS = ROOT/'runs/maintenance'/MISSION
PROTOCOL = ROOT/'experiments/ccta2027/local_pair_safety_v1.json'
DESIGN = ROOT/'experiments/ccta2027/local_pair_safety_v1_physical_design.md'
IMPLEMENTATION = ROOT/'experiments/ccta2027/local_pair_safety_v1_design.md'
OLD = ROOT/'experiments/ccta2027/finite_acquisition_support_v2.json'
OLD_PUBLIC = ROOT/'docs/results/finite_acquisition_support_v2/analysis.json'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
SCRIPTS = ('scripts/local_pair_study.py', 'scripts/local_pair_audit.py', 'scripts/research_io.py',
    'scripts/task_translation_physical_study_v2.py', 'scripts/reference_map_study.py',
    'scripts/finite_acquisition_study_v2.py', 'scripts/finite_acquisition_study.py',
    'tests/test_safety_preparation.py', 'tests/test_local_pair_safety.py',
    'tests/test_local_pair_wiring.py', 'tests/test_local_pair_study.py',
    'tests/test_reference_surface.py', 'tests/test_task_translation_wiring.py')


def test_receipt(path):
    suites = ET.parse(path).getroot().findall('testsuite')
    assert suites and all(int(s.get('failures', 0)) == int(s.get('errors', 0)) == 0 for s in suites)
    values = {k: sum(int(s.get(k, 0)) for s in suites) for k in ('tests', 'failures', 'errors', 'skipped')}
    return dict(**values, passed=values['tests']-values['skipped'], sha256=sha(path))


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists():
        raise RuntimeError('Preserve existing study identity; never refreeze or overwrite')
    checks = {name:test_receipt(CHECKS/name) for name in ('core-final.xml', 'study-green2.xml')}
    old, protected = read(OLD), read(PROTECTED)
    assert sha(OLD) == read(OLD_PUBLIC)['protocol_sha256']
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    assert environment_versions() == old['environment']
    assert package_inventory() == old['installed_locked_packages']
    old_identity = identity(old, OLD)
    assert read(ROOT/old['run_dir']/'manifest.json')['identity'] == old_identity
    verify_source_snapshot(ROOT/old['run_dir'], old['source_files'])
    assert set(old['conditions']) == {'near_circle', 'near_l_reference',
        'far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0'}
    cases, originals, preflight = [], {}, []
    for condition in old['conditions']:
        original = next(c for c in old['cases'] if c['condition_id'] == condition and c['arm'] == ARMS[0])
        folder = ROOT/old['run_dir']/original['case_id']
        result = cached_result(folder, {**old_identity, 'case_sha256':encode(original)})
        old_row = episode(original, folder, result)
        assert old_row['safe_completed'] and old_row['zero_epoch_valid']
        baseline = SimulationEnvironment(copy.deepcopy(original['config']), seed=original['seed'])
        expected = asdict(baseline.controller.params)
        assert expected.pop('acquisition_pair_safety') is False
        prior = next(p for p in old['preflight'] if p['case_id'] == original['case_id'])
        assert encode(expected) == prior['effective_controller_sha256']
        assert baseline.controller.params.communication_dropout_prob == 0.
        originals[condition] = dict(case=original, receipt_sha256=sha(folder/'case_result.json'))
        for arm in ARMS:
            case = copy.deepcopy(original)
            case.update(case_id=condition+'_'+arm, arm=arm)
            case['factors']['cohort'] = 'local_pair_safety_development'
            if arm == ARMS[1]:
                case['config']['controller']['acquisition_pair_safety'] = True
            env = SimulationEnvironment(copy.deepcopy(case['config']), seed=case['seed'])
            effective = asdict(env.controller.params)
            assert effective.pop('acquisition_pair_safety') is (arm == ARMS[1])
            assert effective == expected
            assert env.contact_params == baseline.contact_params
            assert env.controller.phase_monitor.gates == baseline.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist()) == case['expected_initial_state_sha256']
            assert case['frames'] == 12000 and case['config']['dt'] == .05 and case['config']['task']['tolerance'] == .12
            preflight.append(dict(case_id=case['case_id'], prior_effective_controller_sha256=encode(effective),
                only_new_default_off_boolean_changes=True, physics_gates_budget_initial_state_exact=True))
            cases.append(case)
        del result, old_row
        gc.collect()
    random.Random(20261103).shuffle(cases)
    assert len(cases) == len({c['case_id'] for c in cases}) == 8
    source = source_hashes()
    plan = dict(mission=MISSION, status='FROZEN_BEFORE_NEW_PHYSICS', run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases, conditions=old['conditions'], arms=list(ARMS), workers=4, internal_threads=1,
        schedule_seed=20261103, budget=dict(new_physical_episodes=8, maximum_steps_per_episode=12000,
            maximum_total_steps=96000, seconds_per_episode=600., settling_frames=40, settling_speed=.005,
            wall_clock_cutoff=None), mechanism_changes=dict(acquisition_pair_safety=True),
        design_sha256=sha(DESIGN), implementation_design_sha256=sha(IMPLEMENTATION),
        source_files=source, frozen_source_sha256=encode(source), scripts={s:sha(ROOT/s) for s in SCRIPTS},
        environment=environment_versions(), installed_locked_packages=package_inventory(),
        canonical_sha256=protected['canonical_sha256'], protected_baseline_sha256=sha(PROTECTED),
        tests=checks, original_protocol_sha256=sha(OLD), original_analysis_sha256=sha(OLD_PUBLIC),
        original_run_dir=old['run_dir'], original_identity=old_identity, original_source_files=old['source_files'],
        originals=originals, preflight=preflight, candidate_adopted=False,
        mechanism_gate=dict(safe_complete=4, false_holds=0, hard_failures=0, valid_zero_epochs=4,
            acquisition_scalings=0, no_matched_post_acquisition_scaling_increase=True,
            old_controls_exact=4, preserve_baseline_g500=True, independent_execution_audits=True),
        full_chain_gate=dict(all_mechanism_gates=True, candidate_original_g500=4, candidate_formal_transport=4),
        scope='Eight new matched development episodes on four reused conditions; no general reliability, '
              'asynchronous/hardware safety, independent deployment benefit or default adoption claim')
    FREEZE.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        target = FREEZE/'source_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, FREEZE/'protocol.json')
    shutil.copy2(DESIGN, FREEZE/'design.md')
    shutil.copy2(IMPLEMENTATION, FREEZE/'implementation-design.md')
    for name in checks:
        shutil.copy2(CHECKS/name, FREEZE/name)
    loaded, resolved = load_protocol(PROTOCOL, 'development', ROOT)
    assert loaded == plan and resolved == cases and sum(c['frames'] for c in cases) == 96000
    print(json.dumps(dict(status=plan['status'], episodes=8, source_sha256=plan['frozen_source_sha256'],
        protocol_sha256=sha(PROTOCOL))), flush=True)


def check_freeze():
    plan = read(PROTOCOL)
    assert sha(PROTOCOL) == sha(FREEZE/'protocol.json')
    assert sha(DESIGN) == plan['design_sha256'] == sha(FREEZE/'design.md')
    assert sha(IMPLEMENTATION) == plan['implementation_design_sha256'] == sha(FREEZE/'implementation-design.md')
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert package_inventory() == plan['installed_locked_packages']
    assert {s:sha(ROOT/s) for s in SCRIPTS} == plan['scripts']
    assert sha(PROTECTED) == plan['protected_baseline_sha256']
    assert sha(ROOT/read(PROTECTED)['canonical_config']) == plan['canonical_sha256']
    assert sha(OLD) == plan['original_protocol_sha256'] and sha(OLD_PUBLIC) == plan['original_analysis_sha256']
    for name, receipt in plan['tests'].items():
        assert sha(CHECKS/name) == sha(FREEZE/name) == receipt['sha256']
    verify_source_snapshot(FREEZE, {**plan['source_files'], **plan['scripts']})
    return plan


def run():
    guard()
    plan = check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Do not duplicate or restart an existing physical attempt')
    dump(BASE/'started.json', dict(started_at_utc=datetime.now(timezone.utc).isoformat(), protocol_sha256=sha(PROTOCOL)))
    result = run_study(PROTOCOL, 'development', OUT, plan['workers'], False)
    check_freeze()
    outcome = dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'], execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'], protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json', outcome)
    print(json.dumps(outcome), flush=True)


def analyze():
    guard()
    plan = check_freeze()
    manifest, summary = read(OUT/'manifest.json'), read(OUT/'summary.json')
    assert summary['source_integrity'] == 'PASS' and manifest['identity'] == identity(plan, PROTOCOL)
    verify_source_snapshot(OUT, plan['source_files'])
    verify_source_snapshot(ROOT/plan['original_run_dir'], plan['original_source_files'])
    if (BASE/'analysis.json').exists() or (BASE/'analysis_failure.json').exists():
        raise RuntimeError('Preserve the first analysis and any failure')
    rows, reproductions = [], {}
    try:
        for case in plan['cases']:
            folder = OUT/case['case_id']
            result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
            row = episode(case, folder, result)
            complete = row['execution'] == 'COMPLETED'
            row['scaling'] = scaling(folder, row['barrier_scalings']) if complete else None
            row['support_audit'] = support_audit(folder, case) if complete else None
            row['pair_audit'] = audit_episode(folder, case) if complete else None
            if case['arm'] == ARMS[0]:
                original = plan['originals'][case['condition_id']]
                old_case = original['case']
                old_folder = ROOT/plan['original_run_dir']/old_case['case_id']
                assert sha(old_folder/'case_result.json') == original['receipt_sha256']
                old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256':encode(old_case)})
                reproductions[case['condition_id']] = reproduction(old_folder, folder, old_result, result) if complete else dict(exact=False)
            rows.append(row)
            print(json.dumps(dict(case=case['case_id'], safe_completed=row['safe_completed'],
                endpoint_m=row.get('endpoint_m'), g500=row['g500'],
                pair_count=len(row['pair_audit']['events']) if complete else None)), flush=True)
            gc.collect()
        mechanism, full = evaluate_gates(rows, reproductions, plan['conditions'])
        totals = {arm:dict(episodes=sum(r['arm']==arm for r in rows),
            safe_completed=sum(r['safe_completed'] for r in rows if r['arm']==arm),
            false_holds=sum(r['false_hold'] for r in rows if r['arm']==arm),
            hard_failures=sum(bool(r['hard_safety']) for r in rows if r['arm']==arm),
            g500=sum(r['g500'] for r in rows if r['arm']==arm),
            timeouts=sum(r.get('budget_terminated', False) for r in rows if r['arm']==arm)) for arm in ARMS}
        report = dict(scope=plan['scope'], new_physical_episodes=8, episodes=rows, totals=totals,
            original_reproduction=reproductions, mechanism_gates=mechanism, full_chain_gates=full,
            mechanism_gate='PASS' if all(mechanism.values()) else 'FAIL',
            full_chain_gate='PASS' if all(full.values()) else 'FAIL', candidate_adopted=False,
            source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(PROTOCOL))
        dump(BASE/'analysis.json', report)
        dump(BASE/'analysis_outcome.json', dict(status='VERIFIED', analysis_sha256=sha(BASE/'analysis.json'),
            new_physical_episodes=8, denominator=len(rows), mechanism=report['mechanism_gate'], full=report['full_chain_gate']))
        print(json.dumps(dict(totals=totals, mechanism=mechanism, full_chain=full), indent=2), flush=True)
    except Exception as exc:
        dump(BASE/'analysis_failure.json', dict(error=repr(exc), traceback=traceback.format_exc(),
            completed_analysis_rows=len(rows), protocol_sha256=sha(PROTOCOL)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'run', 'analyze'))
    globals()[parser.parse_args().action]()
