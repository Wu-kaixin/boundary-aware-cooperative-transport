"""Prospectively freeze and evaluate reference-driven map motion."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
from collections import Counter
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import gc
import json
import random
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from task_translation_physical_study_v2 import episode, reproduction, package_inventory, SELECTORS
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import (
    cached_result, environment_versions, run_study, source_hashes, verify_source_snapshot)

MISSION = '2026-11-03-reference-map-v1'
BASE = ROOT/'runs/research'/MISSION
OUT = BASE/'run'
FREEZE = BASE/'freeze'
PROTOCOL = ROOT/'experiments/ccta2027/reference_map_v1.json'
DESIGN = ROOT/'experiments/ccta2027/reference_map_v1_design.md'
OLD = ROOT/'experiments/ccta2027/task_translation_physical_v2.json'
PUBLIC = ROOT/'docs/results/reference_map_v1'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
ARMS = ('translation_all_captured', 'reference_map')
SCRIPTS = ('scripts/reference_map_study.py', 'scripts/research_io.py',
           'scripts/task_translation_physical_study_v2.py')
CHANGE = dict(initial_reference_map_motion=True)


def identity(plan, protocol):
    return dict(schema='ccta-study-v2', protocol_sha256=sha(protocol), split='development',
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'])


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the existing attempt; no refreeze')
    old, protected = read(OLD), read(PROTECTED)
    old_batch, old_identity = ROOT/old['run_dir'], identity(old, OLD)
    assert read(old_batch/'manifest.json')['identity'] == old_identity
    verify_source_snapshot(old_batch, old['source_files'])
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    published = read(ROOT/'docs/results/task_translation_physical_v2/analysis.json')
    cases, originals, preflight = [], {}, []
    for condition in old['conditions']:
        base = next(c for c in old['cases'] if c['condition_id']==condition and c['arm']==ARMS[0])
        folder = old_batch/base['case_id']
        result = cached_result(folder, {**old_identity, 'case_sha256':encode(base)})
        assert episode(base, folder, result) == next(r for r in published['episodes'] if r['case_id']==base['case_id'])
        originals[condition] = dict(case=base, receipt_sha256=sha(folder/'case_result.json'))
        baseline_env = SimulationEnvironment(copy.deepcopy(base['config']), seed=base['seed'])
        baseline = asdict(baseline_env.controller.params)
        original = dict(baseline)
        assert original.pop('initial_reference_map_motion') is False
        assert original.pop('initial_reference_safe_geometry') is False
        assert original.pop('reference_surface_refinement') is False
        assert original.pop('initial_reference_motion_model') == SELECTORS['initial_reference_motion_model']
        assert original.pop('initial_reference_use_all_captured') == SELECTORS['initial_reference_use_all_captured']
        expected = dict(protected['effective_controller'])
        expected.update(initial_reference_acquisition=True, initial_task_reference=True)
        assert original == expected
        for arm in ARMS:
            case = copy.deepcopy(base)
            case.update(case_id=condition+'_'+arm, arm=arm)
            case['factors']['cohort'] = 'reference_map_development'
            if arm == ARMS[1]: case['config']['controller'].update(CHANGE)
            env = SimulationEnvironment(copy.deepcopy(case['config']), seed=case['seed'])
            params, expected = asdict(env.controller.params), dict(baseline)
            if arm == ARMS[1]: expected.update(CHANGE)
            assert params == expected
            assert env.contact_params == baseline_env.contact_params
            assert env.controller.phase_monitor.gates == baseline_env.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist()) == case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05 and case['config']['task']['tolerance']==.12
            cases.append(case)
            preflight.append(dict(case_id=case['case_id'], effective_controller_sha256=encode(params),
                numerical_parameters_physics_phase_gates_and_initial_state_preserved=True))
        gc.collect()
    random.Random(20261103).shuffle(cases)
    source = source_hashes()
    plan = dict(mission=MISSION, status='FROZEN_BEFORE_NEW_PHYSICS', run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases, conditions=old['conditions'], arms=list(ARMS), workers=4, internal_threads=1,
        schedule_seed=20261103, budget=old['budget'], mechanism_changes=CHANGE,
        design_sha256=sha(DESIGN), scripts={n:sha(ROOT/n) for n in SCRIPTS},
        source_files=source, frozen_source_sha256=encode(source), environment=environment_versions(),
        installed_locked_packages=package_inventory(), protected_baseline_sha256=sha(PROTECTED),
        canonical_sha256=protected['canonical_sha256'], original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'], original_identity=old_identity, original_source_files=old['source_files'],
        originals=originals, preflight=preflight,
        advance_gate=dict(all_eight_execution_complete=True, all_four_baselines_exact=True,
            candidate_safe_completed=4, candidate_false_holds=0, candidate_valid_zero_epochs=4,
            preserve_each_matched_baseline_g500_pass=True, candidate_acquisition_scalings=0,
            no_matched_post_acquisition_scaling_increase=True), candidate_adopted=False,
        scope='Eight prospective episodes on four reused development conditions; reference-driven map geometry/velocity with explicit missing-reference handling; no general reliability or deployment superiority claim')
    FREEZE.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        target = FREEZE/'source_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, FREEZE/'protocol.json')
    shutil.copy2(DESIGN, FREEZE/'design.md')
    loaded, resolved = load_protocol(PROTOCOL, 'development', ROOT)
    assert loaded==plan and resolved==cases and len(cases)==8
    print(json.dumps(dict(status=plan['status'], episodes=8, source_sha256=plan['frozen_source_sha256'],
        protocol_sha256=sha(PROTOCOL), exact_old_evaluator_checks=4)), flush=True)


def check_freeze():
    plan = read(PROTOCOL)
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert {n:sha(ROOT/n) for n in SCRIPTS}==plan['scripts']
    assert source_hashes()==plan['source_files']
    assert environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert sha(PROTECTED)==plan['protected_baseline_sha256']
    verify_source_snapshot(FREEZE, {**plan['source_files'], **plan['scripts']})
    return plan


def run():
    guard()
    plan = check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Preserve existing attempt; do not duplicate or rerun physics')
    dump(BASE/'started.json', dict(started_at_utc=datetime.now(timezone.utc).isoformat(), protocol_sha256=sha(PROTOCOL)))
    result = run_study(PROTOCOL, 'development', OUT, plan['workers'], False)
    check_freeze()
    outcome = dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'], execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'], protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json', outcome)
    print(json.dumps(outcome), flush=True)


def scaling(folder, expected):
    rows = read(folder/'paired_metrics.json')['steps']
    previous, by_state, by_phase, events = 0, Counter(), Counter(), []
    for row in rows:
        total = row['solver_statistics']['barrier_scalings']
        agents = [d['agent_id'] for d in row['diagnostics'] if d['solver_status']=='scaled_barrier']
        assert total-previous == len(agents)
        previous = total
        if agents:
            state, phase = row['initial_reference']['status'], row['phase']
            by_state[state] += len(agents)
            by_phase[phase] += len(agents)
            events.append(dict(frame=row['frame'], capture_state=state, phase=phase, agents=agents))
    assert previous == expected == sum(by_state.values())
    return dict(scaled_robot_steps=previous, acquisition=by_state['acquiring'],
        post_acquisition=previous-by_state['acquiring'], by_capture_state=dict(by_state),
        by_phase=dict(by_phase), control_frames_with_scaling=len(events),
        min_scale=rows[-1]['solver_statistics']['min_barrier_scale'], events=events)


def analyze():
    guard()
    plan = check_freeze()
    manifest, summary = read(OUT/'manifest.json'), read(OUT/'summary.json')
    assert summary['source_integrity']=='PASS' and summary['completed_denominator']==8
    assert manifest['identity']==identity(plan, PROTOCOL) and manifest['source_files']==plan['source_files']
    verify_source_snapshot(OUT, plan['source_files'])
    if PUBLIC.exists() or (BASE/'analysis.json').exists():
        raise RuntimeError('Preserve existing analysis')
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(ROOT/plan['original_run_dir'], plan['original_source_files'])
    episodes, reproductions = [], {}
    for case in plan['cases']:
        folder = OUT/case['case_id']
        result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = episode(case, folder, result)
        row['scaling'] = scaling(folder, row['barrier_scalings']) if row['execution']=='COMPLETED' else None
        if case['arm'] == ARMS[1] and row['execution'] == 'COMPLETED':
            steps = read(folder/'paired_metrics.json')['steps']
            counts = Counter(d['map_reference_status'] for step in steps for d in step['diagnostics'])
            assert set(counts) <= {'stationary_acquisition', 'fresh_reference', 'unavailable'}
            assert sum(counts.values()) == sum(len(step['diagnostics']) for step in steps)
            missing = [d for step in steps for d in step['diagnostics'] if d['map_reference_status']=='unavailable']
            assert all(d['mode']=='map_reference_unavailable' and d['nominal_command']==[0.,0.] for d in missing)
            with np.load(folder/'replay.npz', allow_pickle=False) as replay:
                positions = replay['agent_positions']
                for k, step in enumerate(steps):
                    ids = [d['agent_id'] for d in step['diagnostics']]
                    distance = np.linalg.norm(positions[k,:,None,:]-positions[k,None,:,:],axis=2)
                    for i, diagnostic in enumerate(step['diagnostics']):
                        permitted = {ids[j] for j in np.flatnonzero(distance[i]<=case['config']['controller']['comm_range'])}
                        voters = diagnostic['map_reference_voters']
                        assert set(voters)<=permitted
                        assert all(step['diagnostics'][ids.index(aid)]['reference_status']=='accepted' for aid in voters)
            row['map_reference'] = dict(robot_steps_by_status=dict(counts), unavailable_nominal_zero_verified=True,
                                        every_voter_fresh_and_one_hop_verified=True)
        if case['arm']==ARMS[0]:
            original = plan['originals'][case['condition_id']]
            old_case = original['case']
            old_folder = ROOT/plan['original_run_dir']/old_case['case_id']
            assert sha(old_folder/'case_result.json')==original['receipt_sha256']
            old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256':encode(old_case)})
            reproductions[case['condition_id']] = reproduction(old_folder, folder, old_result, result) if row['execution']=='COMPLETED' else dict(exact=False)
        episodes.append(row)
        print(case['case_id'], row['safe_completed'], row.get('endpoint_m'), flush=True)
        gc.collect()
    totals = {arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        false_holds=sum(r['false_hold'] for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r['hard_safety']) for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        timeouts=sum(r.get('budget_terminated', False) for r in episodes if r['arm']==arm)) for arm in ARMS}
    candidate = [r for r in episodes if r['arm']==ARMS[1]]
    pairs = {c: {a:next(r for r in episodes if r['condition']==c and r['arm']==a) for a in ARMS} for c in plan['conditions']}
    checks = dict(all_eight_execution_complete=all(r['execution']=='COMPLETED' for r in episodes),
        all_four_baselines_exact=all(r['exact'] for r in reproductions.values()),
        all_four_candidate_safe_complete_no_false_hold_valid_epoch=all(r['safe_completed'] and not r['false_hold'] and r['zero_epoch_valid'] for r in candidate),
        preserve_each_matched_baseline_g500_pass=all(not p[ARMS[0]]['g500'] or p[ARMS[1]]['g500'] for p in pairs.values()),
        candidate_acquisition_scalings_zero=all(r['scaling'] is not None and r['scaling']['acquisition']==0 for r in candidate),
        no_matched_post_acquisition_scaling_increase=all(p[ARMS[0]]['scaling'] is not None and p[ARMS[1]]['scaling'] is not None and p[ARMS[1]]['scaling']['post_acquisition']<=p[ARMS[0]]['scaling']['post_acquisition'] for p in pairs.values()))
    analysis = dict(scope=plan['scope'], episodes=episodes, totals=totals, gates=checks,
        baseline_reproduction=reproductions, advance_gate='PASS' if all(checks.values()) else 'FAIL',
        candidate_adopted=False, new_physical_episodes=8, protocol_sha256=sha(PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'], analyzer_sha256=sha(__file__))
    dump(BASE/'analysis.json', analysis)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    shutil.copy2(BASE/'analysis.json', PUBLIC/'analysis.json')
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    shutil.copy2(BASE/'execution_outcome.json', PUBLIC/'execution_outcome.json')
    print(json.dumps(dict(totals=totals, gates=checks, advance_gate=analysis['advance_gate']), indent=2), flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'run', 'analyze'))
    globals()[parser.parse_args().action]()
