"""Fresh physical validation of corrected finite support; preserve original v1."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_name]='1'

import argparse
from collections import Counter
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import gc
import json
import random
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from research_io import ROOT,guard,read,sha
from task_translation_physical_study_v2 import episode,reproduction,package_inventory
from reference_map_study import identity,scaling
from finite_acquisition_study import evaluate_gates
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result,environment_versions,run_study,source_hashes,verify_source_snapshot

MISSION='2026-11-03-finite-acquisition-support-v2'
BASE=ROOT/'runs/research'/MISSION
OUT,FREEZE=BASE/'run',BASE/'freeze'
CHECKS=ROOT/'runs/maintenance'/MISSION
PROTOCOL=ROOT/'experiments/ccta2027/finite_acquisition_support_v2.json'
DESIGN=ROOT/'experiments/ccta2027/finite_acquisition_support_physical_v2_design.md'
OLD=ROOT/'experiments/ccta2027/finite_acquisition_support_v1.json'
PROTECTED=ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
CORRECTION=ROOT/'docs/results/finite_acquisition_support_v1/post_review_checks.json'
AUDIT=ROOT/'runs/research/2026-11-03-finite-support-review-validation-v1'
ARMS=('reference_reacquisition','finite_acquisition_support')
SCRIPTS=('scripts/finite_acquisition_study_v2.py','scripts/finite_acquisition_study.py',
    'scripts/task_translation_physical_study_v2.py',
    'scripts/reference_map_study.py','scripts/research_io.py','tests/test_finite_acquisition_study.py',
    'tests/test_finite_support.py','tests/test_acquisition_support.py',
    'tests/test_reference_surface.py','tests/test_task_translation_wiring.py',
    'tests/test_finite_support_review_fix.py','scripts/audit_finite_support_review.py',
    'tests/test_finite_support_validation_audit.py')


def check_audit():
    correction=read(CORRECTION)
    assert encode(source_hashes())==correction['corrected_runtime_source_sha256']
    outcomes={}
    for name in ('preflight-c','preflight-l','circle','l','c','skew'):
        folder=AUDIT/name
        outcome=read(folder/'outcome.json')
        assert outcome['status']=='EXACT_RECONSTRUCTION' and outcome['new_physical_episodes']==0
        assert outcome['corrected_source_sha256']==correction['corrected_runtime_source_sha256']
        assert outcome['design_sha256']==sha(ROOT/'experiments/ccta2027/finite_support_review_validation_v1_design.md')
        for script,digest in outcome['scripts'].items():
            assert sha(ROOT/'scripts'/script)==digest==sha(folder/script)
        for product,digest in outcome['products'].items():assert sha(folder/product)==digest
        if not name.startswith('preflight-'):assert outcome['verified_frames']==outcome['freeze_frame']
        outcomes[name]=dict(outcome_sha256=sha(folder/'outcome.json'),verified_frames=outcome['verified_frames'])
    assert sum(v['verified_frames'] for k,v in outcomes.items() if not k.startswith('preflight-'))==874
    return outcomes


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists():raise RuntimeError('Preserve any previous frozen attempt')
    audit=check_audit()
    suites=ET.parse(CHECKS/'core-final.xml').getroot().findall('testsuite')
    assert suites and all(int(s.get('failures',0))==int(s.get('errors',0))==0 for s in suites)
    old,protected=read(OLD),read(PROTECTED)
    assert sha(ROOT/protected['canonical_config'])==protected['canonical_sha256']
    old_identity=identity(old,OLD)
    verify_source_snapshot(ROOT/old['run_dir'],old['source_files'])
    cases,originals,preflight=[],{},[]
    for condition in old['conditions']:
        original=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']==ARMS[0])
        folder=ROOT/old['run_dir']/original['case_id']
        result=cached_result(folder,{**old_identity,'case_sha256':encode(original)})
        old_row=episode(original,folder,result)
        assert old_row['safe_completed'] and old_row['zero_epoch_valid']
        originals[condition]=dict(case=original,receipt_sha256=sha(folder/'case_result.json'))
        base_env=SimulationEnvironment(copy.deepcopy(original['config']),seed=original['seed'])
        values=asdict(base_env.controller.params)
        stripped=dict(values)
        for key,value in dict(reference_surface_refinement=False,initial_reference_motion_model='translation',
            initial_reference_use_all_captured=True,initial_reference_safe_geometry=False,
            initial_reference_map_motion=True,initial_acquisition_circulation=True,
            reference_map_reacquisition=True,acquisition_finite_support=False).items():
            assert stripped.pop(key)==value,(condition,key)
        expected=dict(protected['effective_controller'])
        expected.update(initial_reference_acquisition=True,initial_task_reference=True)
        assert stripped==expected
        for arm in ARMS:
            case=copy.deepcopy(original)
            case.update(case_id=condition+'_'+arm,arm=arm)
            case['factors']['cohort']='finite_acquisition_support_corrected_development'
            expected=dict(values)
            if arm==ARMS[1]:
                case['config']['controller']['acquisition_finite_support']=True
                expected['acquisition_finite_support']=True
            env=SimulationEnvironment(copy.deepcopy(case['config']),seed=case['seed'])
            assert asdict(env.controller.params)==expected
            assert env.contact_params==base_env.contact_params
            assert env.controller.phase_monitor.gates==base_env.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist())==case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05 and case['config']['task']['tolerance']==.12
            cases.append(case)
            preflight.append(dict(case_id=case['case_id'],effective_controller_sha256=encode(expected),
                protected_numerical_parameters_physics_gates_and_initial_state_preserved=True))
        del result,old_row
        gc.collect()
    random.Random(20261103).shuffle(cases)
    assert len(cases)==len({c['case_id'] for c in cases})==8
    source=source_hashes()
    plan=dict(mission=MISSION,status='FROZEN_BEFORE_NEW_PHYSICS',run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases,conditions=old['conditions'],arms=list(ARMS),workers=4,internal_threads=1,schedule_seed=20261103,
        budget=dict(new_physical_episodes=8,maximum_steps_per_episode=12000,maximum_total_steps=96000,
            seconds_per_episode=600.,settling_frames=40,settling_speed=.005,wall_clock_cutoff=None),
        mechanism_changes=dict(acquisition_finite_support=True),design_sha256=sha(DESIGN),
        scripts={s:sha(ROOT/s) for s in SCRIPTS},source_files=source,frozen_source_sha256=encode(source),
        environment=environment_versions(),installed_locked_packages=package_inventory(),
        canonical_sha256=protected['canonical_sha256'],protected_baseline_sha256=sha(PROTECTED),
        core_xml_sha256=sha(CHECKS/'core-final.xml'),original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'],original_identity=old_identity,original_source_files=old['source_files'],
        originals=originals,preflight=preflight,candidate_adopted=False,
        saved_state_audit=audit,correction_receipt_sha256=sha(CORRECTION),
        mechanism_gate=dict(safe_complete=4,false_holds=0,hard_failures=0,valid_zero_epochs=4,
            acquisition_scalings=0,no_matched_post_acquisition_scaling_increase=True,old_controls_exact=4,
            preserve_baseline_g500=True),
        full_chain_gate=dict(all_mechanism_gates=True,candidate_original_g500=4,candidate_formal_transport=4),
        scope='Eight new matched episodes on four reused conditions using corrected finite support; separate from original v1 physics, no general reliability, default adoption or deployment-superiority claim')
    FREEZE.mkdir(parents=True,exist_ok=False)
    for name in [*source,*SCRIPTS]:
        target=FREEZE/'source_snapshot'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,target)
    dump(PROTOCOL,plan)
    shutil.copy2(PROTOCOL,FREEZE/'protocol.json')
    shutil.copy2(DESIGN,FREEZE/'design.md')
    shutil.copy2(CHECKS/'core-final.xml',FREEZE/'core-tests.xml')
    loaded,resolved=load_protocol(PROTOCOL,'development',ROOT)
    assert loaded==plan and resolved==cases and sum(c['frames'] for c in cases)==96000
    print(json.dumps(dict(status=plan['status'],episodes=8,source_sha256=plan['frozen_source_sha256'],protocol_sha256=sha(PROTOCOL))),flush=True)


def check_freeze():
    plan=read(PROTOCOL)
    assert check_audit()==plan['saved_state_audit'] and sha(CORRECTION)==plan['correction_receipt_sha256']
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert {s:sha(ROOT/s) for s in SCRIPTS}==plan['scripts']
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert sha(PROTECTED)==plan['protected_baseline_sha256']
    assert sha(ROOT/read(PROTECTED)['canonical_config'])==plan['canonical_sha256']
    assert sha(CHECKS/'core-final.xml')==sha(FREEZE/'core-tests.xml')==plan['core_xml_sha256']
    verify_source_snapshot(FREEZE,{**plan['source_files'],**plan['scripts']})
    return plan


def run():
    guard()
    plan=check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():raise RuntimeError('Do not duplicate or restart an existing attempt')
    dump(BASE/'started.json',dict(started_at_utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(PROTOCOL)))
    result=run_study(PROTOCOL,'development',OUT,plan['workers'],False)
    check_freeze()
    outcome=dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'],execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'],protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json',outcome)
    print(json.dumps(outcome),flush=True)


def support_audit(folder,case):
    steps=read(folder/'paired_metrics.json')['steps']
    if case['arm']==ARMS[0]:
        assert all('acquisition_support' not in d for step in steps for d in step['diagnostics'])
        total=sum(len(step['diagnostics']) for step in steps)
        return dict(counts=dict(robot_steps=total,original_row_robot_steps=total,
            applied_robot_steps=0,point_guard_rows=0,reach_cap_rows=0),reasons={'disabled':total},
            locality_and_freeze_boundary_verified=True,
            count_semantics='Filter-path robot steps including steps with zero object rows')
    counts,causes=Counter(),Counter()
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        positions,times=z['agent_positions'],z['times']
        for k,step in enumerate(steps):
            ids=[d['agent_id'] for d in step['diagnostics']]
            distances=np.linalg.norm(positions[k,:,None,:]-positions[k,None,:,:],axis=2)
            for i,d in enumerate(step['diagnostics']):
                trace=d['acquisition_support']
                causes[trace['reason']]+=1
                counts['robot_steps']+=1
                permitted={ids[j] for j in np.flatnonzero(distances[i]<=case['config']['controller']['comm_range'])}
                assert set(trace['source_agents'])<=permitted
                assert all(abs(stamp-times[k])<=1e-9 for stamp in trace.get('source_timestamps',[]))
                if trace['applied']:
                    assert step['initial_reference']['status']=='acquiring'
                    assert trace['stage']=='stationary_acquisition' and trace['empirical_only']
                    assert d['agent_id'] in trace['source_agents'] and trace['motion_bound']==0.
                    assert trace['object_rows']>0 and trace['uncovered_returns']==0
                    counts['applied_robot_steps']+=1
                    counts['point_guard_rows']+=trace['point_guard_rows']
                    counts['reach_cap_rows']+=trace['cap_active']
                else:counts['original_row_robot_steps']+=1
    return dict(counts=dict(counts),reasons=dict(causes),locality_and_freeze_boundary_verified=True)


def analyze():
    guard()
    plan=check_freeze()
    manifest,summary=read(OUT/'manifest.json'),read(OUT/'summary.json')
    assert summary['source_integrity']=='PASS' and manifest['identity']==identity(plan,PROTOCOL)
    verify_source_snapshot(OUT,plan['source_files'])
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(ROOT/plan['original_run_dir'],plan['original_source_files'])
    if (BASE/'analysis.json').exists():raise RuntimeError('Preserve existing analysis')
    episodes,reproductions=[],{}
    for case in plan['cases']:
        folder=OUT/case['case_id']
        result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        row=episode(case,folder,result)
        row['scaling']=scaling(folder,row['barrier_scalings']) if row['execution']=='COMPLETED' else None
        row['support_audit']=support_audit(folder,case) if row['execution']=='COMPLETED' else None
        if case['arm']==ARMS[0]:
            original=plan['originals'][case['condition_id']]
            old_case=original['case']
            old_folder=ROOT/plan['original_run_dir']/old_case['case_id']
            assert sha(old_folder/'case_result.json')==original['receipt_sha256']
            old_result=cached_result(old_folder,{**plan['original_identity'],'case_sha256':encode(old_case)})
            reproductions[case['condition_id']]=reproduction(old_folder,folder,old_result,result) if row['execution']=='COMPLETED' else dict(exact=False)
        episodes.append(row)
        print(json.dumps(dict(case=case['case_id'],safe_completed=row['safe_completed'],endpoint_m=row.get('endpoint_m'),g500=row['g500'])),flush=True)
        gc.collect()
    mechanism,complete=evaluate_gates(episodes,reproductions,plan['conditions'])
    totals={arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        false_holds=sum(r['false_hold'] for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r['hard_safety']) for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        timeouts=sum(r.get('budget_terminated',False) for r in episodes if r['arm']==arm)) for arm in ARMS}
    report=dict(scope=plan['scope'],episodes=episodes,totals=totals,original_reproduction=reproductions,
        mechanism_gates=mechanism,full_chain_gates=complete,
        mechanism_gate='PASS' if all(mechanism.values()) else 'FAIL',
        full_chain_gate='PASS' if all(complete.values()) else 'FAIL',candidate_adopted=False,
        source_sha256=plan['frozen_source_sha256'],protocol_sha256=sha(PROTOCOL))
    dump(BASE/'analysis.json',report)
    print(json.dumps(dict(totals=totals,mechanism=mechanism,complete=complete),indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('freeze','run','analyze'))
    globals()[parser.parse_args().action]()
