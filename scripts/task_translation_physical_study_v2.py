"""Freeze, run and evaluate eight prospectively paired physical episodes."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import gc
import importlib.metadata
import json
import random
import re
import shutil

import numpy as np
from packaging.requirements import Requirement

from research_io import ROOT, guard, read, sha
from dbact.contracts import ClosedLoopContract
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import (
    cached_result, environment_versions, run_study, source_hashes, verify_source_snapshot)

MISSION = '2026-11-03-task-translation-physical-v2'
BASE = ROOT/'runs/research'/MISSION
OUT = BASE/'run'
FREEZE = BASE/'freeze'
PROTOCOL = ROOT/'experiments/ccta2027/task_translation_physical_v2.json'
DESIGN = ROOT/'experiments/ccta2027/task_translation_physical_v2_design.md'
OLD = ROOT/'experiments/ccta2027/task_reference_v1.json'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
PUBLIC = ROOT/'docs/results/task_translation_physical_v2'
CONDITIONS = ('near_circle', 'near_l_reference', 'far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0')
ARMS = ('reference', 'translation_all_captured')
SELECTORS = dict(initial_reference_acquisition=True, initial_task_reference=True,
    initial_reference_motion_model='translation', initial_reference_use_all_captured=True)
SCRIPTS = ('scripts/task_translation_physical_study_v2.py', 'scripts/research_io.py')


def package_inventory():
    requirements = [Requirement(s.rstrip(chr(92)).strip())
        for s in (ROOT/'requirements.lock.txt').read_text().splitlines()
        if re.match(r'^[A-Za-z0-9][A-Za-z0-9_.-]*==', s)]
    return {r.name: dict(locked=str(r.specifier), installed=importlib.metadata.version(r.name))
        for r in requirements if r.marker is None or r.marker.evaluate()}


def episode(case, folder, result):
    row = dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
        execution=result['execution_status'], completed=bool(result.get('task_completed')),
        safe_completed=False, g500=False, false_hold=False, hard_safety=['execution'],
        zero_epoch_valid=False, receipt_sha256=sha(folder/'case_result.json'))
    if result['execution_status'] != 'COMPLETED':
        row['error'] = result.get('error')
        return row
    gate = read(folder/'summary.json')['cargoes']['cargo_0']['g500']
    metrics = gate['metrics']
    contract = ClosedLoopContract(**gate['gates'])
    verdict = contract.evaluate(metrics)
    assert verdict.success == gate['success'] == result['g500_pass']
    assert verdict.reasons == gate['failure_reasons']
    hard = []
    if metrics['min_inter_agent_distance'] < metrics['d_min']-contract.d_min_tolerance:
        hard.append('separation')
    if metrics['min_signed_clearance'] < 0: hard.append('inside_object')
    if metrics['max_penetration'] > metrics['penetration_budget']: hard.append('penetration_budget')
    if metrics['solver_fallbacks'] or metrics['solver_infeasible']: hard.append('solver_failure')
    steps = read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        xy = z['cargo/cargo_0/centers'].copy()
        angles = z['cargo/cargo_0/angles'].copy()
        start, goal = z['task/cargo_0/start'].copy(), z['task/cargo_0/goal_point'].copy()
    assert len(xy) == len(steps)+1 and [s['frame'] for s in steps] == list(range(1, len(xy)))
    ready = result['contact_ready_frame']
    errors, holds, last_phase, denominator, quorum = [], [], None, 0, 0
    attempts = {}
    for s in steps:
        nav = s['navigation_supervisor']
        before = xy[s['frame']-1]
        if s['phase'] == 'HOLD' and last_phase != 'HOLD':
            holds.append(dict(frame=s['frame'], physical_error_before_m=float(np.linalg.norm(before-goal)),
                physical_error_after_m=float(np.linalg.norm(xy[s['frame']]-goal)),
                nominal_error_m=None if nav is None else float(np.linalg.norm(nav['error'])),
                workspace_error_bound_m=s['initial_reference']['center_error_bound']))
        last_phase = s['phase']
        if ready is not None and s['frame'] >= ready:
            denominator += 1
            if nav is not None:
                errors.append(float(np.linalg.norm(np.asarray(nav['displacement'])-(before-start))))
                quorum += len(nav['voters']) >= case['config']['controller']['min_push_agents']
        for d in s['diagnostics']:
            aid = d['agent_id']
            attempts[aid] = max(attempts.get(aid, 0), d['contact_redeploy_attempts'])
    acquisition = result.get('initial_reference', {})
    row.update(safe_completed=bool(row['completed'] and not hard), g500=verdict.success,
        g500_failure_reasons=verdict.reasons, hard_safety=hard,
        false_hold=any(h['physical_error_before_m'] > case['config']['task']['tolerance'] for h in holds),
        first_hold=holds[0] if holds else None, hold_entries=holds,
        zero_epoch_valid=bool(acquisition.get('zero_epoch_valid')), acquisition=acquisition,
        endpoint_m=result['euclidean_endpoint_error'], seconds=len(steps)*case['config']['dt'],
        termination=result['termination'], final_phase=steps[-1]['phase'],
        budget_terminated=bool(len(steps) >= case['frames'] and not result['settled']),
        barrier_scalings=metrics['barrier_scalings'], safety_metrics={k: metrics[k] for k in (
            'min_inter_agent_distance','d_min','min_signed_clearance','max_penetration',
            'penetration_budget','solver_fallbacks','solver_infeasible')},
        estimation=dict(post_ready_frames=denominator, available_frames=len(errors), quorum_frames=int(quorum),
            missing_frames=denominator-len(errors), insufficient_quorum_frames=denominator-int(quorum),
            mae_m=float(np.mean(errors)) if errors else None,
            p95_m=float(np.quantile(errors,.95)) if errors else None,
            max_m=max(errors) if errors else None, scope='Available control readings versus pre-physics truth; within each episode, not paired frames across different paths'),
        maximum_absolute_rotation_deg=float(np.degrees(np.max(np.abs(angles-angles[0])))),
        contact_redeploy_attempts_by_agent=attempts, phase_response=result['phase_response'])
    return row


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the existing study; no refreeze')
    old, protected = read(OLD), read(PROTECTED)
    batch = ROOT/old['run_dir']
    identity = dict(schema='ccta-study-v2', protocol_sha256=sha(OLD), split='development',
        source_sha256=old['frozen_source_sha256'], environment=old['environment'])
    assert read(batch/'manifest.json')['identity'] == identity
    verify_source_snapshot(batch, old['source_files'])
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    published = read(ROOT/'docs/results/task_reference_v1/analysis.json')
    cases, originals, preflight = [], {}, []
    for condition in CONDITIONS:
        base = next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='reference')
        folder = batch/base['case_id']
        result = cached_result(folder, {**identity, 'case_sha256':encode(base)})
        checked = episode(base, folder, result)
        expected = next(r for r in published['episodes'] if r['condition']==condition and r['arm']=='reference')
        for key in ('execution','completed','safe_completed','g500','false_hold','hard_safety','endpoint_m','seconds','first_hold'):
            assert checked[key] == expected[key], (condition,key)
        originals[condition] = dict(case=base, receipt_sha256=sha(folder/'case_result.json'))
        baseline_env = SimulationEnvironment(copy.deepcopy(base['config']), seed=base['seed'])
        baseline = asdict(baseline_env.controller.params)
        reference = dict(baseline)
        assert reference.pop('reference_surface_refinement') is False
        assert reference.pop('initial_reference_motion_model') == 'se2'
        assert reference.pop('initial_reference_use_all_captured') is False
        assert reference == protected['effective_controller']
        for arm in ARMS:
            case = copy.deepcopy(base)
            case.update(case_id=condition+'_'+arm, arm=arm)
            case['factors']['cohort'] = 'task_translation_physical_development'
            if arm != 'reference': case['config']['controller'].update(SELECTORS)
            env = SimulationEnvironment(copy.deepcopy(case['config']), seed=case['seed'])
            effective, expected_params = asdict(env.controller.params), dict(baseline)
            if arm != 'reference': expected_params.update(SELECTORS)
            assert effective == expected_params
            assert env.contact_params == baseline_env.contact_params
            assert env.controller.phase_monitor.gates == baseline_env.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist()) == case['expected_initial_state_sha256']
            assert case['frames'] == 12000 and case['config']['dt'] == .05
            assert case['config']['task']['tolerance'] == .12
            cases.append(case)
            preflight.append(dict(case_id=case['case_id'], protected_parameters_equal_except_declared_selectors=True,
                physics_and_phase_gates_equal=True, initial_state_verified=True, effective_controller_sha256=encode(effective)))
        gc.collect()
    random.Random(20261103).shuffle(cases)
    source = source_hashes()
    inventory = package_inventory()
    plan = dict(mission=MISSION, status='FROZEN_BEFORE_NEW_PHYSICS', run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases, conditions=list(CONDITIONS), arms=list(ARMS), workers=4, internal_threads=1, schedule_seed=20261103,
        budget=dict(new_physical_episodes=8, maximum_steps_per_episode=12000, maximum_total_steps=96000,
            seconds_per_episode=600., settling_frames=40, settling_speed=.005, wall_clock_cutoff=None),
        mechanism_changes=SELECTORS, design_sha256=sha(DESIGN), scripts={n:sha(ROOT/n) for n in SCRIPTS},
        source_files=source, frozen_source_sha256=encode(source), environment=environment_versions(),
        installed_locked_packages=inventory, protected_baseline_sha256=sha(PROTECTED),
        canonical_sha256=protected['canonical_sha256'], original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'], original_identity=identity, original_source_files=old['source_files'],
        originals=originals, preflight=preflight,
        advance_gate=dict(all_eight_execution_complete=True, all_four_baselines_exact=True,
            candidate_safe_completed=4, candidate_false_holds=0, candidate_valid_zero_epochs=4,
            preserve_each_matched_baseline_g500_pass=True), candidate_adopted=False,
        scope='Prospective eight-episode pilot on reused development conditions; original replay FAIL retained; no new-shape/general reliability or CVT-superiority claim')
    FREEZE.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        target = FREEZE/'source_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, FREEZE/'protocol.json')
    shutil.copy2(DESIGN, FREEZE/'design.md')
    loaded, resolved = load_protocol(PROTOCOL, 'development', ROOT)
    assert loaded == plan and resolved == cases and len(cases) == 8
    print(json.dumps(dict(status=plan['status'], episodes=8, source_sha256=plan['frozen_source_sha256'],
        protocol_sha256=sha(PROTOCOL), old_evaluator_checks=4), indent=2), flush=True)


def check_freeze():
    plan = read(PROTOCOL)
    assert sha(PROTOCOL) == sha(FREEZE/'protocol.json')
    assert sha(DESIGN) == plan['design_sha256'] == sha(FREEZE/'design.md')
    assert {n:sha(ROOT/n) for n in SCRIPTS} == plan['scripts']
    assert source_hashes() == plan['source_files']
    assert environment_versions() == plan['environment']
    assert package_inventory() == plan['installed_locked_packages']
    verify_source_snapshot(FREEZE, {**plan['source_files'], **plan['scripts']})
    return plan


def run():
    guard()
    plan = check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Preserve existing attempt; do not duplicate or rerun partial physics')
    dump(BASE/'started.json', dict(started_at_utc=datetime.now(timezone.utc).isoformat(), protocol_sha256=sha(PROTOCOL)))
    result = run_study(PROTOCOL, 'development', OUT, plan['workers'], False)
    check_freeze()
    dump(BASE/'execution_outcome.json', dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED', completed_denominator=result['completed_denominator'],
        execution_errors=result['execution_errors'], source_integrity=result['source_integrity'], protocol_sha256=sha(PROTOCOL)))
    print(json.dumps(dict(completed=result['completed_denominator'], execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'])), flush=True)


def reproduction(old_folder, new_folder, old_result, new_result):
    arrays = {}
    with np.load(old_folder/'replay.npz', allow_pickle=False) as a, np.load(new_folder/'replay.npz', allow_pickle=False) as b:
        same_keys = set(a.files) == set(b.files)
        for key in a.files:
            arrays[key] = bool(key in b.files and np.array_equal(a[key], b[key], equal_nan=True)
                if a[key].dtype.kind in 'fci' else key in b.files and np.array_equal(a[key], b[key]))
    scientific, provenance = {}, {}
    for name in ('config.json', 'paired_metrics.json', 'physics_substeps.json', 'summary.json'):
        a, b = read(old_folder/name), read(new_folder/name)
        if name == 'summary.json':
            provenance['git_sha'] = [a['provenance'].pop('git_sha'), b['provenance'].pop('git_sha')]
        scientific[name] = a == b
    a, b = copy.deepcopy(old_result), copy.deepcopy(new_result)
    for name in ('case_id', 'factors', 'wall_seconds'):
        provenance[name] = [a.pop(name), b.pop(name)]
    scientific['case_result_scientific_fields'] = a == b
    return dict(exact=bool(same_keys and all(arrays.values()) and all(scientific.values())),
        replay_keys_equal=same_keys, arrays=arrays, scientific_fields_exact=scientific, execution_metadata=provenance)


def analyze():
    guard()
    plan = check_freeze()
    manifest, summary = read(OUT/'manifest.json'), read(OUT/'summary.json')
    assert summary['source_integrity'] == 'PASS' and summary['completed_denominator'] == 8
    assert manifest['source_files'] == plan['source_files']
    expected_identity = dict(schema='ccta-study-v2', protocol_sha256=sha(PROTOCOL), split='development',
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'])
    assert manifest['identity'] == expected_identity
    verify_source_snapshot(OUT, plan['source_files'])
    if PUBLIC.exists() or (BASE/'analysis.json').exists():
        raise RuntimeError('Preserve existing analysis')
    episodes, reproductions = [], {}
    assert sha(OLD) == plan['original_protocol_sha256']
    verify_source_snapshot(ROOT/plan['original_run_dir'], plan['original_source_files'])
    for case in plan['cases']:
        folder = OUT/case['case_id']
        result = cached_result(folder, {**expected_identity, 'case_sha256':encode(case)})
        row = episode(case, folder, result)
        if case['arm'] == 'reference':
            original = plan['originals'][case['condition_id']]
            old_case = original['case']
            old_folder = ROOT/plan['original_run_dir']/old_case['case_id']
            assert sha(old_folder/'case_result.json') == original['receipt_sha256']
            old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256':encode(old_case)})
            reproductions[case['condition_id']] = reproduction(old_folder, folder, old_result, result) if row['execution']=='COMPLETED' else dict(exact=False)
        episodes.append(row)
        print(case['case_id'], row['safe_completed'], row.get('endpoint_m'), flush=True)
        gc.collect()
    totals = {arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        completed=sum(r['completed'] for r in episodes if r['arm']==arm),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        false_holds=sum(r['false_hold'] for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r['hard_safety']) for r in episodes if r['arm']==arm),
        timeouts=sum(r.get('budget_terminated',False) for r in episodes if r['arm']==arm)) for arm in ARMS}
    candidate = [r for r in episodes if r['arm']==ARMS[1]]
    primary = all(r['safe_completed'] and not r['false_hold'] and r['zero_epoch_valid'] for r in candidate)
    nonregression = {c: not next(r for r in episodes if r['condition']==c and r['arm']=='reference')['g500']
        or next(r for r in candidate if r['condition']==c)['g500'] for c in CONDITIONS}
    checks = dict(all_eight_execution_complete=all(r['execution']=='COMPLETED' for r in episodes),
        all_four_baselines_exact=all(r['exact'] for r in reproductions.values()),
        primary_all_four_candidate_safe_complete_no_false_hold_valid_epoch=primary,
        preserve_each_matched_baseline_g500_pass=all(nonregression.values()))
    analysis = dict(scope=plan['scope'], episodes=episodes, totals=totals, baseline_reproduction=reproductions,
        gates=checks, g500_nonregression_by_condition=nonregression, advance_gate='PASS' if all(checks.values()) else 'FAIL',
        candidate_adopted=False, new_physical_episodes=8, protocol_sha256=sha(PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'], analyzer_sha256=sha(__file__))
    dump(BASE/'analysis.json', analysis)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    shutil.copy2(BASE/'analysis.json', PUBLIC/'analysis.json')
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    print(json.dumps(dict(totals=totals, gates=checks, advance_gate=analysis['advance_gate']), indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze','run','analyze'))
    globals()[parser.parse_args().action]()
