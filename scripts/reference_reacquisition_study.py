"""Freeze eight matched episodes of reference-loss observation recovery."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
from collections import Counter, defaultdict
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import gc
import json
import random
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from research_io import ROOT, guard, read, sha
from task_translation_physical_study_v2 import episode, reproduction, package_inventory, SELECTORS
from reference_map_study import identity, scaling
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result, environment_versions, run_study, source_hashes, verify_source_snapshot

MISSION = '2026-11-03-reference-reacquisition-v1'
BASE = ROOT/'runs/research'/MISSION
OUT, FREEZE = BASE/'run', BASE/'freeze'
CHECKS = ROOT/'runs/maintenance'/MISSION
PROTOCOL = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
DESIGN = ROOT/'experiments/ccta2027/reference_reacquisition_v1_design.md'
OLD = ROOT/'experiments/ccta2027/acquisition_circulation_v1.json'
PUBLIC = ROOT/'docs/results/reference_reacquisition_physical_v1'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
ARMS = ('acquisition_circulation', 'reference_reacquisition')
CHANGE = dict(reference_map_reacquisition=True)
SCRIPTS = ('scripts/reference_reacquisition_study.py', 'scripts/reference_map_study.py',
           'scripts/task_translation_physical_study_v2.py', 'scripts/research_io.py',
           'tests/test_reference_reacquisition_study.py')


def _prefix_arrays(replay, frame):
    count = frame + 1  # initial sample plus post-physics samples through freeze
    assert len(replay['times']) >= count
    arrays = {}
    for key in replay.files:
        value = replay[key]
        if key in ('agent_ids', 'domain', 'robot_radius') or key.startswith('task/') or key.endswith('/local_vertices'):
            arrays[key] = value
        elif key == 'sensed_points':
            arrays[key] = value[:int(replay['sensed_counts'][:count].sum())]
        else:
            assert len(value) == len(replay['times']), key
            arrays[key] = value[:count]
    return arrays


def compare_capture_prefix(old_folder, new_folder):
    old_steps = read(old_folder/'paired_metrics.json')['steps']
    new_steps = read(new_folder/'paired_metrics.json')['steps']
    old_frame = next((s['frame'] for s in old_steps if s['initial_reference']['status']=='frozen'), None)
    new_frame = next((s['frame'] for s in new_steps if s['initial_reference']['status']=='frozen'), None)
    if old_frame is None or new_frame != old_frame:
        return dict(exact=False, freeze_frame=old_frame, candidate_freeze_frame=new_frame,
                    reason='initial capture frame differs or is unavailable')
    assert [s['frame'] for s in old_steps[:old_frame]] == list(range(1, old_frame+1))
    assert [s['frame'] for s in new_steps[:new_frame]] == list(range(1, new_frame+1))
    arrays = {}
    with np.load(old_folder/'replay.npz', allow_pickle=False) as a, np.load(new_folder/'replay.npz', allow_pickle=False) as b:
        aa, bb = _prefix_arrays(a, old_frame), _prefix_arrays(b, new_frame)
        keys_equal = set(aa) == set(bb)
        for key, value in aa.items():
            arrays[key] = bool(key in bb and (np.array_equal(value, bb[key], equal_nan=True)
                if value.dtype.kind in 'fci' else np.array_equal(value, bb[key])))
    paired = old_steps[:old_frame] == new_steps[:new_frame]
    return dict(exact=bool(keys_equal and all(arrays.values()) and paired), freeze_frame=old_frame,
                candidate_freeze_frame=new_frame, replay_keys_equal=keys_equal, arrays=arrays,
                paired_steps_exact=paired, scope='All replay arrays and complete recorded control/physics steps through the first frozen frame, including that frame')


def evaluate_gates(episodes, reproductions, prefixes, conditions):
    pairs = {c:{a:next(r for r in episodes if r['condition']==c and r['arm']==a)
                for a in ARMS} for c in conditions}
    candidate = [p[ARMS[1]] for p in pairs.values()]
    mechanism = dict(
        all_eight_execution_complete=len(episodes)==8 and all(r['execution']=='COMPLETED' for r in episodes),
        all_four_old_arms_exact=set(reproductions)==set(conditions) and all(r['exact'] for r in reproductions.values()),
        all_four_capture_prefixes_exact=set(prefixes)==set(conditions) and all(r['exact'] for r in prefixes.values()),
        all_four_candidate_safe_complete_no_false_hold_valid_epoch=len(candidate)==4 and all(
            r['safe_completed'] and not r['false_hold'] and not r['hard_safety'] and r['zero_epoch_valid'] for r in candidate),
        preserve_each_matched_baseline_g500_pass=all(not p[ARMS[0]]['g500'] or p[ARMS[1]]['g500'] for p in pairs.values()),
        no_matched_post_acquisition_scaling_increase=all(p[ARMS[0]]['scaling'] is not None
            and p[ARMS[1]]['scaling'] is not None
            and p[ARMS[1]]['scaling']['post_acquisition']<=p[ARMS[0]]['scaling']['post_acquisition'] for p in pairs.values()))
    complete = dict(**mechanism, candidate_acquisition_scalings_zero=all(
        r['scaling'] is not None and r['scaling']['acquisition']==0 for r in candidate))
    return mechanism, complete


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve existing attempt; no refreeze')
    suites = ET.parse(CHECKS/'study-core.xml').getroot().findall('testsuite')
    assert suites and all(int(s.get('failures', 0))==int(s.get('errors', 0))==0 for s in suites)
    engineering = ROOT/'docs/results/reference_reacquisition_v1/final_review_checks.json'
    assert encode(source_hashes()) == read(engineering)['runtime_source_sha256']
    old, protected = read(OLD), read(PROTECTED)
    old_batch, old_identity = ROOT/old['run_dir'], identity(old, OLD)
    assert read(old_batch/'manifest.json')['identity'] == old_identity
    verify_source_snapshot(old_batch, old['source_files'])
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    published = read(ROOT/'docs/results/acquisition_circulation_v1/analysis.json')
    cases, originals, preflight = [], {}, []
    for condition in old['conditions']:
        original = next(c for c in old['cases'] if c['condition_id']==condition and c['arm']==ARMS[0])
        folder = old_batch/original['case_id']
        result = cached_result(folder, {**old_identity, 'case_sha256':encode(original)})
        row = episode(original, folder, result)
        expected_row = next(r for r in published['episodes'] if r['case_id']==original['case_id'])
        assert row == {k:expected_row[k] for k in row}
        prefix = compare_capture_prefix(folder, folder)
        assert prefix['exact'] and len(prefix['arrays']) == 25
        originals[condition] = dict(case=original, receipt_sha256=sha(folder/'case_result.json'),
                                    capture_frame=prefix['freeze_frame'])
        baseline_env = SimulationEnvironment(copy.deepcopy(original['config']), seed=original['seed'])
        baseline = asdict(baseline_env.controller.params)
        stripped = dict(baseline)
        for key, expected in (('reference_map_reacquisition', False), ('initial_acquisition_circulation', True),
                ('initial_reference_map_motion', True), ('initial_reference_safe_geometry', False),
                ('reference_surface_refinement', False)):
            assert stripped.pop(key) is expected
        for key in ('initial_reference_motion_model', 'initial_reference_use_all_captured'):
            assert stripped.pop(key) == SELECTORS[key]
        expected = dict(protected['effective_controller'])
        expected.update(initial_reference_acquisition=True, initial_task_reference=True)
        assert stripped == expected
        for arm in ARMS:
            case = copy.deepcopy(original)
            case.update(case_id=condition+'_'+arm, arm=arm)
            case['factors']['cohort'] = 'reference_reacquisition_development'
            expected = dict(baseline)
            if arm == ARMS[1]:
                case['config']['controller'].update(CHANGE)
                expected.update(CHANGE)
            env = SimulationEnvironment(copy.deepcopy(case['config']), seed=case['seed'])
            assert asdict(env.controller.params) == expected
            assert env.contact_params == baseline_env.contact_params
            assert env.controller.phase_monitor.gates == baseline_env.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist()) == case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05 and case['config']['task']['tolerance']==.12
            cases.append(case)
            preflight.append(dict(case_id=case['case_id'], effective_controller_sha256=encode(expected),
                numerical_parameters_physics_phase_gates_and_initial_state_preserved=True))
        del result, row, prefix
        gc.collect()
    random.Random(20261103).shuffle(cases)
    assert len(cases)==len({c['case_id'] for c in cases})==8
    source = source_hashes()
    plan = dict(mission=MISSION, status='FROZEN_BEFORE_NEW_PHYSICS', run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases, conditions=old['conditions'], arms=list(ARMS), workers=4, internal_threads=1, schedule_seed=20261103,
        budget=dict(new_physical_episodes=8, maximum_steps_per_episode=12000, maximum_total_steps=96000,
            seconds_per_episode=600., settling_frames=40, settling_speed=.005, wall_clock_cutoff=None),
        mechanism_changes=CHANGE, design_sha256=sha(DESIGN), scripts={s:sha(ROOT/s) for s in SCRIPTS},
        source_files=source, frozen_source_sha256=encode(source), environment=environment_versions(),
        installed_locked_packages=package_inventory(), protected_baseline_sha256=sha(PROTECTED),
        canonical_sha256=protected['canonical_sha256'], engineering_receipt_sha256=sha(engineering),
        core_xml_sha256=sha(CHECKS/'study-core.xml'), original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'], original_identity=old_identity, original_source_files=old['source_files'],
        originals=originals, preflight=preflight,
        original_budget_metadata_erratum=dict(declared_episodes=old['budget']['new_physical_episodes'],
            actual_resolved_cases=len(old['cases']), effect='Descriptive header error only; previous twelve-case execution and original files remain unchanged'),
        mechanism_gate=dict(all_eight_execution_complete=True, all_four_old_arms_exact=True,
            all_four_capture_prefixes_exact=True, candidate_safe_completed=4, candidate_false_holds=0,
            candidate_hard_failures=0, candidate_valid_zero_epochs=4, preserve_each_matched_baseline_g500_pass=True,
            no_matched_post_acquisition_scaling_increase=True),
        full_chain_gate=dict(all_mechanism_gates=True, candidate_acquisition_scalings=0), candidate_adopted=False,
        scope='Eight prospective episodes on four reused development conditions; original numerical parameters and safety/acceptance criteria retained. Mechanism feasibility and complete-chain gates reported separately; no general reliability or deployment-superiority claim')
    FREEZE.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        target = FREEZE/'source_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, FREEZE/'protocol.json')
    shutil.copy2(DESIGN, FREEZE/'design.md')
    shutil.copy2(CHECKS/'study-core.xml', FREEZE/'core-tests.xml')
    loaded, resolved = load_protocol(PROTOCOL, 'development', ROOT)
    assert loaded == plan and resolved == cases and len(cases)==plan['budget']['new_physical_episodes']
    assert sum(c['frames'] for c in cases)==plan['budget']['maximum_total_steps']
    print(json.dumps(dict(status=plan['status'], episodes=len(cases), source_sha256=plan['frozen_source_sha256'],
        protocol_sha256=sha(PROTOCOL), old_evaluator_and_prefix_checks=len(originals))), flush=True)


def check_freeze():
    plan = read(PROTOCOL)
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert {s:sha(ROOT/s) for s in SCRIPTS}==plan['scripts']
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert sha(PROTECTED)==plan['protected_baseline_sha256']
    assert sha(ROOT/read(PROTECTED)['canonical_config'])==plan['canonical_sha256']
    assert sha(CHECKS/'study-core.xml')==sha(FREEZE/'core-tests.xml')==plan['core_xml_sha256']
    verify_source_snapshot(FREEZE, {**plan['source_files'], **plan['scripts']})
    return plan


def run():
    guard()
    plan = check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Do not duplicate or restart the existing physical attempt')
    dump(BASE/'started.json', dict(started_at_utc=datetime.now(timezone.utc).isoformat(), protocol_sha256=sha(PROTOCOL)))
    result = run_study(PROTOCOL, 'development', OUT, plan['workers'], False)
    check_freeze()
    outcome = dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'], execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'], protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json', outcome)
    print(json.dumps(outcome), flush=True)


def trace_audit(folder, case):
    steps = read(folder/'paired_metrics.json')['steps']
    counts, modes, phases = Counter(), Counter(), Counter()
    previous, granted, first_recovered, per_robot = {}, set(), {}, defaultdict(Counter)
    with np.load(folder/'replay.npz', allow_pickle=False) as replay:
        positions = replay['agent_positions']
        for k, step in enumerate(steps):
            phases[step['phase']] += 1
            ids = [d['agent_id'] for d in step['diagnostics']]
            distances = np.linalg.norm(positions[k,:,None,:]-positions[k,None,:,:], axis=2)
            for i, d in enumerate(step['diagnostics']):
                aid, status, mode = d['agent_id'], d['map_reference_status'], d['mode']
                counts[status] += 1
                modes[mode] += 1
                permitted = {ids[j] for j in np.flatnonzero(distances[i]<=case['config']['controller']['comm_range'])}
                assert set(d['map_reference_voters']) <= permitted
                assert all(step['diagnostics'][ids.index(v)]['reference_status']=='accepted' for v in d['map_reference_voters'])
                recovery = mode.startswith('map_reference_reacquire_')
                if recovery:
                    assert case['arm']==ARMS[1] and status=='unavailable'
                    assert step['initial_reference']['status']=='frozen'
                    assert step['phase'] in ('SEARCH', 'DISCOVER', 'ENCLOSE', 'CONTACT_READY')
                    assert mode.removeprefix('map_reference_reacquire_') in ('search', 'recall', 'approach')
                    assert not d['push_side'] and d['effort']==0
                    granted.add(aid)
                    per_robot[aid]['recovery_grants'] += 1
                    per_robot[aid]['nonzero_recovery_commands'] += int(d['speed_command']>0)
                elif status=='unavailable':
                    assert mode=='map_reference_unavailable' and d['nominal_command']==[0., 0.]
                if previous.get(aid)=='unavailable' and status=='fresh_reference':
                    per_robot[aid]['reference_recoveries'] += 1
                    if aid in granted:
                        first_recovered.setdefault(aid, step['frame'])
                previous[aid] = status
    return dict(robot_steps_by_status=dict(counts), modes=dict(modes), phase_frames=dict(phases),
        every_voter_fresh_and_one_hop_verified=True, recovery_scope_verified=True,
        per_robot=dict(per_robot), first_fresh_reference_after_recovery_grant=first_recovered,
        recovery_count_scope='Observed availability after a permission grant, not proof that the individual motion caused recovery',
        maximum_transport_active=max(s['phase_guards']['transport_active'] for s in steps))


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
    episodes, reproductions, prefixes = [], {}, {}
    for case in plan['cases']:
        folder = OUT/case['case_id']
        result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = episode(case, folder, result)
        row['scaling'] = scaling(folder, row['barrier_scalings']) if row['execution']=='COMPLETED' else None
        if row['execution']=='COMPLETED':
            row['map_reference'] = trace_audit(folder, case)
        original = plan['originals'][case['condition_id']]
        old_case = original['case']
        old_folder = ROOT/plan['original_run_dir']/old_case['case_id']
        assert sha(old_folder/'case_result.json')==original['receipt_sha256']
        if case['arm']==ARMS[0]:
            old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256':encode(old_case)})
            reproductions[case['condition_id']] = reproduction(old_folder, folder, old_result, result) if row['execution']=='COMPLETED' else dict(exact=False)
        else:
            prefixes[case['condition_id']] = compare_capture_prefix(old_folder, folder) if row['execution']=='COMPLETED' else dict(exact=False)
        episodes.append(row)
        print(json.dumps(dict(case=case['case_id'], safe_completed=row['safe_completed'], endpoint_m=row.get('endpoint_m'))), flush=True)
        gc.collect()
    totals = {arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        false_holds=sum(r['false_hold'] for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r['hard_safety']) for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        timeouts=sum(r.get('budget_terminated', False) for r in episodes if r['arm']==arm)) for arm in ARMS}
    mechanism, complete = evaluate_gates(episodes, reproductions, prefixes, plan['conditions'])
    analysis = dict(scope=plan['scope'], episodes=episodes, totals=totals,
        mechanism_gates=mechanism, full_chain_gates=complete,
        mechanism_gate='PASS' if all(mechanism.values()) else 'FAIL',
        full_chain_gate='PASS' if all(complete.values()) else 'FAIL',
        baseline_reproduction=reproductions, acquisition_prefix_reproduction=prefixes,
        candidate_adopted=False, new_physical_episodes=8, protocol_sha256=sha(PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], environment=plan['environment'], analyzer_sha256=sha(__file__))
    dump(BASE/'analysis.json', analysis)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    for src, name in ((BASE/'analysis.json', 'analysis.json'), (PROTOCOL, 'protocol.json'),
                      (BASE/'execution_outcome.json', 'execution_outcome.json')):
        shutil.copy2(src, PUBLIC/name)
    print(json.dumps(dict(totals=totals, mechanism_gates=mechanism, full_chain_gates=complete), indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'run', 'analyze'))
    globals()[parser.parse_args().action]()
