"""Frozen eight-episode comparison of current finite support after acquisition."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'

import argparse
import ast
import copy
from dataclasses import asdict
from datetime import datetime,timezone
import gc
import json
import math
from pathlib import Path
import random
import shutil
import traceback

import numpy as np

from research_io import ROOT,guard,read,sha
from task_translation_physical_study_v2 import episode,reproduction,package_inventory
from reference_map_study import identity,scaling as episode_scaling
from local_pair_study import test_receipt
from local_pair_audit import audit_episode
from map_bridge_audit import audit as bridge_audit
from post_acquisition_support_audit import audit as support_audit
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result,environment_versions,run_study,source_hashes,verify_source_snapshot


ARMS = ('map_bridge_control', 'post_support_candidate')
CONDITIONS = ('near_circle', 'near_l_reference', 'far_c_open_away_pos0',
              'far_skew_box_s0_r0_pos0')
BASE=ROOT/'runs/research/2026-11-03-post-acquisition-support-v1'
OUT,FREEZE=BASE/'run',BASE/'freeze'
CHECKS=ROOT/'runs/maintenance/2026-11-03-post-acquisition-support-v1'
PROTOCOL=ROOT/'experiments/ccta2027/post_acquisition_finite_support_v1.json'
DESIGN=ROOT/'experiments/ccta2027/post_acquisition_finite_support_v1_physical_design.md'
OLD=ROOT/'experiments/ccta2027/map_reference_bridge_v1.json'
PROTECTED=ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
TESTS=('tests/test_post_support_study.py','tests/test_post_acquisition_finite_support.py',
       'tests/test_reference_surface.py','tests/test_task_translation_wiring.py')


def evaluate(rows, reproductions, conditions):
    """Fail closed on missing/duplicate episodes, then keep both gate levels."""
    conditions = tuple(conditions)
    expected = {(condition, arm) for condition in CONDITIONS for arm in ARMS}
    keys = [(row.get('condition'), row.get('arm')) for row in rows]
    complete = (len(conditions) == len(CONDITIONS)
                and set(conditions) == set(CONDITIONS)
                and len(keys) == len(expected) and set(keys) == expected
                and all(row.get('execution') == 'COMPLETED' for row in rows))
    lookup = dict(zip(keys, rows))
    candidates = [lookup.get((condition, ARMS[1]), {}) for condition in CONDITIONS]
    pairs = [(lookup.get((condition, ARMS[0]), {}), candidate)
             for condition, candidate in zip(CONDITIONS, candidates)]

    def scaling(row, stage):
        value = (row.get('scaling') or {}).get(stage)
        return value if type(value) is int and value >= 0 else None

    def not_increased(control, candidate, stage):
        before, after = scaling(control, stage), scaling(candidate, stage)
        return before is not None and after is not None and after <= before

    def audit(row):
        return row.get('post_support_audit') or {}

    skew_control, skew_candidate = pairs[-1]
    before = scaling(skew_control, 'post_acquisition')
    after = scaling(skew_candidate, 'post_acquisition')
    c_before, c_after = [r.get('maximum_absolute_rotation_deg') for r in pairs[2]]
    rotation_valid = all(isinstance(v, (int, float)) and math.isfinite(v)
                         and v >= 0 for v in (c_before, c_after))
    mechanism = dict(
        complete_eight=complete,
        original_controls_exact=complete and set(reproductions) == set(CONDITIONS)
            and all(reproductions[c].get('exact') is True for c in CONDITIONS),
        safe_completion=complete and all(r.get('safe_completed') is True for r in candidates),
        zero_epoch_valid=complete and all(r.get('zero_epoch_valid') is True for r in candidates),
        no_false_hold=complete and all(r.get('false_hold') is False for r in candidates),
        no_hard_safety_failure=complete and all(r.get('hard_safety') == [] for r in candidates),
        no_budget_cutoff=complete and all(r.get('budget_terminated') is False for r in candidates),
        matched_g500_preserved=complete and all(
            type(control.get('g500')) is bool and type(candidate.get('g500')) is bool
            and (not control['g500'] or candidate['g500']) for control, candidate in pairs),
        scaling_not_increased=complete and all(not_increased(control, candidate, stage)
            for control, candidate in pairs for stage in ('acquisition', 'post_acquisition')),
        skew_post_acquisition_improved=complete and before is not None and after is not None and after < before,
        c_rotation_not_increased=complete and rotation_valid and c_after <= c_before + 1e-12,
        input_and_row_audits_verified=complete and all(audit(r).get('verified') is True
            and audit(r).get('full_episode_verified') is True for r in candidates),
        pair_and_bridge_audits_verified=complete and all(
            (r.get(kind+'_audit') or {}).get('verified') is True
            for r in rows for kind in ('pair','bridge')),
        candidate_path_exercised=complete and any(audit(r).get('applied', 0) > 0 for r in candidates),
        admitted_hard_rows_satisfied=complete and all(audit(r).get('hard_violations') == 0 for r in candidates),
    )
    full = dict(
        mechanism_pass=all(mechanism.values()),
        zero_candidate_scaling=complete and all(scaling(r, stage) == 0
            for r in candidates for stage in ('acquisition', 'post_acquisition')),
        g500_all_four=complete and all(r.get('g500') is True for r in candidates),
        formal_transport_all_four=complete and all(
            (r.get('phase_response') or {}).get('transport_frame') is not None for r in candidates),
        uncapped_admitted_object_rows_satisfied=complete and all(
            audit(r).get('uncapped_object_violations') == 0 for r in candidates),
    )
    return mechanism, full


def build_cases(old):
    """Prove each resolved controller still matches its prior frozen contract."""
    assert len(old['conditions'])==4 and set(old['conditions'])==set(CONDITIONS)
    cases,preflight=[],[]
    for condition in CONDITIONS:
        originals=[case for case in old['cases'] if case['condition_id']==condition
                   and case['arm']=='map_bridge_candidate']
        assert len(originals)==1
        original=originals[0]
        baseline=SimulationEnvironment(copy.deepcopy(original['config']),seed=original['seed'])
        params=asdict(baseline.controller.params)
        assert params.pop('post_acquisition_finite_support') is False
        legacy=dict(params)
        assert legacy.pop('reference_map_bridge') is True
        old_preflight=next(p for p in old['preflight'] if p['case_id']==original['case_id'])
        assert encode(legacy)==old_preflight['prior_effective_controller_sha256']
        for arm in ARMS:
            case=copy.deepcopy(original)
            case.update(case_id=condition+'_'+arm,arm=arm)
            case['factors']['cohort']='post_acquisition_finite_support_development'
            if arm==ARMS[1]:
                case['config']['controller']['post_acquisition_finite_support']=True
            env=SimulationEnvironment(copy.deepcopy(case['config']),seed=case['seed'])
            effective=asdict(env.controller.params)
            assert effective.pop('post_acquisition_finite_support') is (arm==ARMS[1])
            assert effective==params and env.contact_params==baseline.contact_params
            assert env.controller.phase_monitor.gates==baseline.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist())==case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05
            preflight.append(dict(case_id=case['case_id'],prior_effective_controller_sha256=encode(params),
                numeric_physics_gates_initial_state_unchanged=True))
            cases.append(case)
        gc.collect()
    return cases,preflight


def helper_sources():
    """Freeze the transitive local script imports, including deferred imports."""
    pending=[Path(__file__).resolve()]
    seen=set()
    while pending:
        path=pending.pop()
        if path in seen:
            continue
        seen.add(path)
        tree=ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            names=[item.name for item in node.names] if isinstance(node,ast.Import) else (
                [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            for name in names:
                candidate=ROOT/'scripts'/(name.split('.')[0]+'.py')
                if candidate.is_file() and candidate not in seen:
                    pending.append(candidate)
    return {path.relative_to(ROOT).as_posix():sha(path) for path in sorted(seen)} | {name:sha(ROOT/name) for name in TESTS}


def freeze():
    guard()
    if FREEZE.exists() or OUT.exists() or PROTOCOL.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Preserve existing physical protocol or attempt; no refreeze')
    tests={name:test_receipt(CHECKS/name) for name in ('physical-core.xml','physical-gates-green.xml')}
    old,protected=read(OLD),read(PROTECTED)
    assert sha(OLD)==read(ROOT/'docs/results/map_reference_bridge_v1/analysis.json')['protocol_sha256']
    assert sha(ROOT/protected['canonical_config'])==protected['canonical_sha256']
    assert environment_versions()==old['environment'] and package_inventory()==old['installed_locked_packages']
    old_identity=identity(old,OLD)
    assert read(ROOT/old['run_dir']/'manifest.json')['identity']==old_identity
    verify_source_snapshot(ROOT/old['run_dir'],old['source_files'])
    cases,preflight=build_cases(old)
    originals={}
    for condition in CONDITIONS:
        original=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='map_bridge_candidate')
        folder=ROOT/old['run_dir']/original['case_id']
        result=cached_result(folder,{**old_identity,'case_sha256':encode(original)})
        assert result['execution_status']=='COMPLETED'
        originals[condition]=dict(case=original,receipt_sha256=sha(folder/'case_result.json'))
    random.Random(20261103).shuffle(cases)
    source,scripts=source_hashes(),helper_sources()
    assert encode(source)==read(BASE/'observer/outcome.json')['new_source_sha256']
    plan=dict(mission=BASE.name,status='FROZEN_BEFORE_NEW_PHYSICS',run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases,conditions=list(CONDITIONS),arms=list(ARMS),workers=4,internal_threads=1,schedule_seed=20261103,
        budget=old['budget'],design_sha256=sha(DESIGN),source_files=source,frozen_source_sha256=encode(source),
        scripts=scripts,environment=environment_versions(),installed_locked_packages=package_inventory(),
        canonical_sha256=protected['canonical_sha256'],protected_baseline_sha256=sha(PROTECTED),tests=tests,
        original_protocol_sha256=sha(OLD),original_identity=old_identity,original_source_files=old['source_files'],
        original_run_dir=old['run_dir'],originals=originals,preflight=preflight,
        mechanism_changes=dict(post_acquisition_finite_support=True),candidate_adopted=False,
        scope='Eight matched development episodes; no generalization, deployment benefit or hardware claim')
    FREEZE.mkdir(parents=True,exist_ok=False)
    for name in [*source,*scripts]:
        dest=FREEZE/'source_snapshot'/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,dest)
    dump(PROTOCOL,plan)
    shutil.copy2(PROTOCOL,FREEZE/'protocol.json')
    shutil.copy2(DESIGN,FREEZE/'design.md')
    for name in tests:
        shutil.copy2(CHECKS/name,FREEZE/name)
    loaded,resolved=load_protocol(PROTOCOL,'development',ROOT)
    assert loaded==plan and resolved==cases
    print(json.dumps(dict(status=plan['status'],episodes=8,source_sha256=plan['frozen_source_sha256'],
        frozen_helpers=len(scripts))),flush=True)


def check():
    plan=read(PROTOCOL)
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages'] and helper_sources()==plan['scripts']
    assert sha(PROTECTED)==plan['protected_baseline_sha256']
    assert sha(ROOT/read(PROTECTED)['canonical_config'])==plan['canonical_sha256']
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(FREEZE,{**plan['source_files'],**plan['scripts']})
    for name,receipt in plan['tests'].items():
        assert sha(CHECKS/name)==receipt['sha256']==sha(FREEZE/name)
    return plan


def run():
    guard()
    plan=check()
    if OUT.exists() or (BASE/'started.json').exists():
        raise RuntimeError('Preserve original physical attempt; do not restart')
    dump(BASE/'started.json',dict(started_at_utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(PROTOCOL)))
    result=run_study(PROTOCOL,'development',OUT,plan['workers'],False)
    check()
    dump(BASE/'execution_outcome.json',dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'],execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'],protocol_sha256=sha(PROTOCOL)))
    print(json.dumps(dict(completed=result['completed_denominator'],execution_errors=result['execution_errors'])),flush=True)


def analyze():
    guard()
    plan=check()
    if (BASE/'analysis.json').exists() or (BASE/'analysis_attempt').exists():
        raise RuntimeError('Preserve original analysis attempt')
    attempt=BASE/'analysis_attempt'
    attempt.mkdir()
    manifest,summary=read(OUT/'manifest.json'),read(OUT/'summary.json')
    assert manifest['identity']==identity(plan,PROTOCOL) and summary['source_integrity']=='PASS'
    verify_source_snapshot(OUT,plan['source_files'])
    rows,reproductions=[],{}
    try:
        for case in plan['cases']:
            folder=OUT/case['case_id']
            result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
            row=episode(case,folder,result)
            complete=row['execution']=='COMPLETED'
            row['scaling']=episode_scaling(folder,row['barrier_scalings']) if complete else None
            row['pair_audit']=audit_episode(folder,case) if complete else None
            row['bridge_audit']=bridge_audit(folder,case) if complete else None
            row['post_support_audit']=None
            if complete and case['arm']==ARMS[1]:
                prefix=support_audit(folder,case,limit_frames=20)
                dump(attempt/(case['case_id']+'-prefix.json'),prefix)
                row['post_support_audit']=support_audit(folder,case)
            if case['arm']==ARMS[0]:
                original=plan['originals'][case['condition_id']]
                old_case=original['case']
                old_folder=ROOT/plan['original_run_dir']/old_case['case_id']
                assert sha(old_folder/'case_result.json')==original['receipt_sha256']
                old_result=cached_result(old_folder,{**plan['original_identity'],'case_sha256':encode(old_case)})
                reproductions[case['condition_id']]=reproduction(old_folder,folder,old_result,result) if complete else dict(exact=False)
            dump(attempt/(case['case_id']+'.json'),row)
            rows.append(row)
            print(json.dumps(dict(case=case['case_id'],safe=row['safe_completed'],g500=row['g500'],scaling=row['scaling'])),flush=True)
            gc.collect()
        mechanism,full=evaluate(rows,reproductions,plan['conditions'])
        report=dict(new_physical_episodes=8,episodes=rows,original_reproduction=reproductions,
            mechanism_gates=mechanism,full_gates=full,mechanism_gate='PASS' if all(mechanism.values()) else 'FAIL',
            full_gate='PASS' if all(full.values()) else 'FAIL',candidate_adopted=False,
            protocol_sha256=sha(PROTOCOL),source_sha256=plan['frozen_source_sha256'],scope=plan['scope'])
        check()
        dump(BASE/'analysis.json',report)
        dump(BASE/'analysis_outcome.json',dict(status='VERIFIED',analysis_sha256=sha(BASE/'analysis.json'),
            mechanism=report['mechanism_gate'],full=report['full_gate'],episodes=len(rows)))
        print(json.dumps(dict(mechanism=mechanism,full=full),indent=2),flush=True)
    except Exception:
        dump(attempt/'failure.json',dict(status='ANALYSIS_FAILED',completed_rows=len(rows),
            case_id=case['case_id'],traceback=traceback.format_exc()))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('freeze','run','analyze','check'))
    globals()[parser.parse_args().action]()
