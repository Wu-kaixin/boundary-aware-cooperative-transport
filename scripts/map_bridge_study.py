"""Frozen four-condition paired physical test of same-voter map motion."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'

import argparse
import copy
from dataclasses import asdict
from datetime import datetime,timezone
import gc
import json
import random
import shutil

import numpy as np

from research_io import ROOT,guard,read,sha
from task_translation_physical_study_v2 import episode,reproduction,package_inventory
from reference_map_study import identity,scaling
from local_pair_study import test_receipt
from local_pair_audit import audit_episode
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result,environment_versions,run_study,source_hashes,verify_source_snapshot

BASE=ROOT/'runs/research/2026-11-03-map-reference-bridge-v1'
OUT,FREEZE=BASE/'run',BASE/'freeze'
CHECKS=ROOT/'runs/maintenance/2026-11-03-map-ancestry-v1'
PROTOCOL=ROOT/'experiments/ccta2027/map_reference_bridge_v1.json'
DESIGN=ROOT/'experiments/ccta2027/map_reference_bridge_v1_design.md'
OLD=ROOT/'experiments/ccta2027/local_pair_safety_v1.json'
PROTECTED=ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
ARMS=('local_pair_candidate','map_bridge_candidate')
SCRIPTS=('scripts/map_bridge_study.py','scripts/map_bridge_audit.py',
    'scripts/task_translation_physical_study_v2.py','scripts/reference_map_study.py',
    'scripts/local_pair_study.py','scripts/local_pair_audit.py','scripts/research_io.py',
    'tests/test_reference_map_bridge.py','tests/test_map_bridge_study.py','tests/test_map_point_lineage.py',
    'tests/test_reference_surface.py','tests/test_task_translation_wiring.py')


def evaluate(rows,reproductions,conditions,skew='far_skew_box_s0_r0_pos0'):
    expected={(c,a) for c in conditions for a in ARMS}
    actual=[(r['condition'],r['arm']) for r in rows]
    complete=len(rows)==len(set(actual))==8 and len(conditions)==4 and set(actual)==expected
    control={r['condition']:r for r in rows if r['arm']==ARMS[0]}
    candidate={r['condition']:r for r in rows if r['arm']==ARMS[1]}
    mechanism=dict(schedule_complete=complete,
        all_execution_complete=complete and all(r['execution']=='COMPLETED' for r in rows),
        original_controls_exact=set(reproductions)==set(conditions) and all(x['exact'] for x in reproductions.values()),
        safe_complete_no_false_hold=complete and all(r['safe_completed'] and r['zero_epoch_valid'] and
            not r['false_hold'] and not r['hard_safety'] for r in candidate.values()),
        preserve_matched_g500=complete and all(not control[c]['g500'] or r['g500'] for c,r in candidate.items()),
        no_acquisition_scaling_increase=complete and all(r.get('scaling') is not None and
            control[c].get('scaling') is not None and r['scaling']['acquisition']<=control[c]['scaling']['acquisition']
            for c,r in candidate.items()),
        no_post_acquisition_scaling_increase=complete and all(r.get('scaling') is not None and
            control[c].get('scaling') is not None and r['scaling']['post_acquisition']<=control[c]['scaling']['post_acquisition']
            for c,r in candidate.items()),
        skew_post_acquisition_improved=complete and skew in candidate and
            candidate[skew].get('scaling') is not None and control[skew].get('scaling') is not None and
            candidate[skew]['scaling']['post_acquisition']<control[skew]['scaling']['post_acquisition'],
        bridge_audits=complete and all((r.get('bridge_audit') or {}).get('verified',False) for r in candidate.values()))
    full=dict(**mechanism,zero_candidate_scaling=complete and all(r.get('scaling') is not None and
        r['scaling']['acquisition']==r['scaling']['post_acquisition']==0 for r in candidate.values()),
        g500_all=complete and all(r['g500'] for r in candidate.values()),
        formal_transport_all=complete and all(r.get('phase_response',{}).get('transport_frame') is not None
            for r in candidate.values()))
    return mechanism,full


def freeze():
    guard()
    assert not BASE.exists() and not PROTOCOL.exists(), 'Never overwrite a study'
    tests={name:test_receipt(CHECKS/name) for name in ('core-final.xml','study-green.xml')}
    old,protected=read(OLD),read(PROTECTED)
    assert sha(ROOT/protected['canonical_config'])==protected['canonical_sha256']
    assert environment_versions()==old['environment'] and package_inventory()==old['installed_locked_packages']
    old_identity=identity(old,OLD)
    assert read(ROOT/old['run_dir']/'manifest.json')['identity']==old_identity
    verify_source_snapshot(ROOT/old['run_dir'],old['source_files'])
    cases,originals,preflight=[],{},[]
    for condition in old['conditions']:
        original=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']==ARMS[0])
        folder=ROOT/old['run_dir']/original['case_id']
        result=cached_result(folder,{**old_identity,'case_sha256':encode(original)})
        assert result['execution_status']=='COMPLETED'
        originals[condition]=dict(case=original,receipt_sha256=sha(folder/'case_result.json'))
        baseline=SimulationEnvironment(copy.deepcopy(original['config']),seed=original['seed'])
        params=asdict(baseline.controller.params)
        assert params.pop('reference_map_bridge') is False
        # The old receipt's effective-parameter digest excluded only its added
        # acquisition_pair_safety flag. Verify against that frozen contract.
        legacy=dict(params)
        assert legacy.pop('acquisition_pair_safety') is True
        old_preflight=next(p for p in old['preflight'] if p['case_id']==original['case_id'])
        assert encode(legacy)==old_preflight['prior_effective_controller_sha256']
        for arm in ARMS:
            case=copy.deepcopy(original)
            case.update(case_id=condition+'_'+arm,arm=arm)
            case['factors']['cohort']='map_reference_bridge_development'
            if arm==ARMS[1]:
                case['config']['controller']['reference_map_bridge']=True
            env=SimulationEnvironment(copy.deepcopy(case['config']),seed=case['seed'])
            effective=asdict(env.controller.params)
            assert effective.pop('reference_map_bridge') is (arm==ARMS[1])
            assert effective==params and env.contact_params==baseline.contact_params
            assert env.controller.phase_monitor.gates==baseline.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist())==case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05
            preflight.append(dict(case_id=case['case_id'],prior_effective_controller_sha256=encode(params),
                numeric_physics_gates_initial_state_unchanged=True))
            cases.append(case)
        gc.collect()
    random.Random(20261103).shuffle(cases)
    source=source_hashes()
    plan=dict(mission=BASE.name,status='FROZEN_BEFORE_NEW_PHYSICS',run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases,conditions=old['conditions'],arms=list(ARMS),workers=4,internal_threads=1,schedule_seed=20261103,
        budget=old['budget'],design_sha256=sha(DESIGN),source_files=source,frozen_source_sha256=encode(source),
        scripts={n:sha(ROOT/n) for n in SCRIPTS},environment=environment_versions(),
        installed_locked_packages=package_inventory(),canonical_sha256=protected['canonical_sha256'],
        protected_baseline_sha256=sha(PROTECTED),tests=tests,original_protocol_sha256=sha(OLD),
        original_identity=old_identity,original_source_files=old['source_files'],original_run_dir=old['run_dir'],
        originals=originals,preflight=preflight,mechanism_changes=dict(reference_map_bridge=True),
        candidate_adopted=False,scope='Eight matched development episodes; no generalization, deployment benefit or hardware claim')
    FREEZE.mkdir(parents=True,exist_ok=False)
    for name in [*source,*SCRIPTS]:
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
    print(json.dumps(dict(status=plan['status'],episodes=8,source_sha256=plan['frozen_source_sha256'])),flush=True)


def check():
    plan=read(PROTOCOL)
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert {n:sha(ROOT/n) for n in SCRIPTS}==plan['scripts']
    assert sha(ROOT/read(PROTECTED)['canonical_config'])==plan['canonical_sha256']
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(FREEZE,{**plan['source_files'],**plan['scripts']})
    for name,receipt in plan['tests'].items():
        assert sha(CHECKS/name)==receipt['sha256']==sha(FREEZE/name)
    return plan


def run():
    guard()
    plan=check()
    assert not OUT.exists() and not (BASE/'started.json').exists(), 'Preserve original attempt'
    dump(BASE/'started.json',dict(started_at_utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(PROTOCOL)))
    result=run_study(PROTOCOL,'development',OUT,plan['workers'],False)
    check()
    dump(BASE/'execution_outcome.json',dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'],execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity']))


def analyze():
    guard()
    from map_bridge_audit import audit
    plan=check()
    assert not (BASE/'analysis.json').exists()
    manifest,summary=read(OUT/'manifest.json'),read(OUT/'summary.json')
    assert manifest['identity']==identity(plan,PROTOCOL) and summary['source_integrity']=='PASS'
    rows,reproductions=[],{}
    for case in plan['cases']:
        folder=OUT/case['case_id']
        result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        row=episode(case,folder,result)
        complete=row['execution']=='COMPLETED'
        row['scaling']=scaling(folder,row['barrier_scalings']) if complete else None
        row['pair_audit']=audit_episode(folder,case) if complete else None
        row['bridge_audit']=audit(folder,case) if complete and case['arm']==ARMS[1] else None
        if case['arm']==ARMS[0]:
            original=plan['originals'][case['condition_id']]
            old_case=original['case']
            old_folder=ROOT/plan['original_run_dir']/old_case['case_id']
            assert sha(old_folder/'case_result.json')==original['receipt_sha256']
            old_result=cached_result(old_folder,{**plan['original_identity'],'case_sha256':encode(old_case)})
            reproductions[case['condition_id']]=reproduction(old_folder,folder,old_result,result) if complete else dict(exact=False)
        rows.append(row)
        print(json.dumps(dict(case=case['case_id'],safe=row['safe_completed'],g500=row['g500'],scaling=row['scaling'])),flush=True)
        gc.collect()
    mechanism,full=evaluate(rows,reproductions,plan['conditions'])
    report=dict(new_physical_episodes=8,episodes=rows,original_reproduction=reproductions,
        mechanism_gates=mechanism,full_gates=full,mechanism_gate='PASS' if all(mechanism.values()) else 'FAIL',
        full_gate='PASS' if all(full.values()) else 'FAIL',candidate_adopted=False,
        protocol_sha256=sha(PROTOCOL),source_sha256=plan['frozen_source_sha256'],scope=plan['scope'])
    dump(BASE/'analysis.json',report)
    dump(BASE/'analysis_outcome.json',dict(status='VERIFIED',analysis_sha256=sha(BASE/'analysis.json'),
        mechanism=report['mechanism_gate'],full=report['full_gate'],episodes=len(rows)))
    print(json.dumps(dict(mechanism=mechanism,full=full),indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('freeze','run','analyze'))
    globals()[parser.parse_args().action]()
