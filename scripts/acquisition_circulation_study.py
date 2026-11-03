"""Freeze and evaluate twelve episodes of active local acquisition."""
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

from research_io import ROOT, guard, read, sha
from task_translation_physical_study_v2 import episode, reproduction, package_inventory, SELECTORS
from reference_map_study import identity, scaling
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result, environment_versions, run_study, source_hashes, verify_source_snapshot

MISSION='2026-11-03-acquisition-circulation-v1'
BASE=ROOT/'runs/research'/MISSION
OUT=BASE/'run'
FREEZE=BASE/'freeze'
PROTOCOL=ROOT/'experiments/ccta2027/acquisition_circulation_v1.json'
DESIGN=ROOT/'experiments/ccta2027/acquisition_circulation_v1_design.md'
OLD=ROOT/'experiments/ccta2027/reference_map_v1.json'
PUBLIC=ROOT/'docs/results/acquisition_circulation_v1'
PROTECTED=ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
ARMS=('translation_all_captured','reference_map','acquisition_circulation')
SCRIPTS=('scripts/acquisition_circulation_study.py','scripts/reference_map_study.py',
         'scripts/task_translation_physical_study_v2.py','scripts/research_io.py')
CHANGE=dict(initial_acquisition_circulation=True)


def freeze():
    guard()
    if BASE.exists() or PROTOCOL.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve existing attempt; no refreeze')
    suites=ET.parse(ROOT/'runs/maintenance'/MISSION/'core-tests.xml').getroot().findall('testsuite')
    assert suites and all(int(s.get('failures',0))==int(s.get('errors',0))==0 for s in suites)
    old,protected=read(OLD),read(PROTECTED)
    old_batch,old_identity=ROOT/old['run_dir'],identity(old,OLD)
    assert read(old_batch/'manifest.json')['identity']==old_identity
    verify_source_snapshot(old_batch,old['source_files'])
    assert sha(ROOT/protected['canonical_config'])==protected['canonical_sha256']
    published=read(ROOT/'docs/results/reference_map_v1/analysis.json')
    cases,originals,preflight=[],{},[]
    for condition in old['conditions']:
        bases={arm:next(c for c in old['cases'] if c['condition_id']==condition and c['arm']==arm) for arm in ARMS[:2]}
        for original in bases.values():
            folder=old_batch/original['case_id']
            result=cached_result(folder,{**old_identity,'case_sha256':encode(original)})
            row=episode(original,folder,result)
            expected=next(r for r in published['episodes'] if r['case_id']==original['case_id'])
            assert row=={k:expected[k] for k in row}
            originals[original['case_id']]=dict(case=original,receipt_sha256=sha(folder/'case_result.json'))
        baseline_env=SimulationEnvironment(copy.deepcopy(bases[ARMS[0]]['config']),seed=bases[ARMS[0]]['seed'])
        baseline=asdict(baseline_env.controller.params)
        original=dict(baseline)
        for key in ('initial_acquisition_circulation','initial_reference_map_motion','initial_reference_safe_geometry','reference_surface_refinement'):
            assert original.pop(key) is False
        for key in ('initial_reference_motion_model','initial_reference_use_all_captured'):
            assert original.pop(key)==SELECTORS[key]
        expected=dict(protected['effective_controller']);expected.update(initial_reference_acquisition=True,initial_task_reference=True)
        assert original==expected
        for arm in ARMS:
            case=copy.deepcopy(bases[ARMS[0] if arm==ARMS[0] else ARMS[1]])
            case.update(case_id=condition+'_'+arm,arm=arm)
            case['factors']['cohort']='acquisition_circulation_development'
            if arm==ARMS[2]:case['config']['controller'].update(CHANGE)
            env=SimulationEnvironment(copy.deepcopy(case['config']),seed=case['seed'])
            expected=dict(baseline)
            if arm!=ARMS[0]:expected['initial_reference_map_motion']=True
            if arm==ARMS[2]:expected.update(CHANGE)
            assert asdict(env.controller.params)==expected
            assert env.contact_params==baseline_env.contact_params
            assert env.controller.phase_monitor.gates==baseline_env.controller.phase_monitor.gates
            assert encode(np.vstack([a.position for a in env.agents]).tolist())==case['expected_initial_state_sha256']
            assert case['frames']==12000 and case['config']['dt']==.05 and case['config']['task']['tolerance']==.12
            cases.append(case)
            preflight.append(dict(case_id=case['case_id'],effective_controller_sha256=encode(expected),
                numerical_parameters_physics_phase_gates_and_initial_state_preserved=True))
        gc.collect()
    random.Random(20261103).shuffle(cases)
    source=source_hashes()
    plan=dict(mission=MISSION,status='FROZEN_BEFORE_NEW_PHYSICS',run_dir=OUT.relative_to(ROOT).as_posix(),
        cases=cases,conditions=old['conditions'],arms=list(ARMS),workers=4,internal_threads=1,schedule_seed=20261103,
        budget=old['budget'],mechanism_changes=CHANGE,design_sha256=sha(DESIGN),scripts={s:sha(ROOT/s) for s in SCRIPTS},
        source_files=source,frozen_source_sha256=encode(source),environment=environment_versions(),
        installed_locked_packages=package_inventory(),protected_baseline_sha256=sha(PROTECTED),
        canonical_sha256=protected['canonical_sha256'],original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'],original_identity=old_identity,original_source_files=old['source_files'],
        originals=originals,preflight=preflight,
        advance_gate=dict(all_twelve_execution_complete=True,all_eight_old_arms_exact=True,
            candidate_safe_completed=4,candidate_false_holds=0,candidate_valid_zero_epochs=4,
            preserve_each_translation_baseline_g500_pass=True,candidate_acquisition_scalings=0,
            no_matched_post_acquisition_scaling_increase=True),candidate_adopted=False,
        scope='Twelve prospective episodes on four reused development conditions; active acquisition with unchanged numeric parameters, safety rows and acceptance gates; no general reliability or deployment-superiority claim')
    FREEZE.mkdir(parents=True,exist_ok=False)
    for name in [*source,*SCRIPTS]:
        target=FREEZE/'source_snapshot'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
    dump(PROTOCOL,plan)
    shutil.copy2(PROTOCOL,FREEZE/'protocol.json');shutil.copy2(DESIGN,FREEZE/'design.md')
    loaded,resolved=load_protocol(PROTOCOL,'development',ROOT)
    assert loaded==plan and resolved==cases and len(cases)==12
    print(json.dumps(dict(status=plan['status'],episodes=12,source_sha256=plan['frozen_source_sha256'],
        protocol_sha256=sha(PROTOCOL),old_evaluator_checks=len(originals))),flush=True)


def check_freeze():
    plan=read(PROTOCOL)
    assert sha(PROTOCOL)==sha(FREEZE/'protocol.json')
    assert sha(DESIGN)==plan['design_sha256']==sha(FREEZE/'design.md')
    assert {s:sha(ROOT/s) for s in SCRIPTS}==plan['scripts']
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert sha(PROTECTED)==plan['protected_baseline_sha256']
    protected=read(PROTECTED);assert sha(ROOT/protected['canonical_config'])==plan['canonical_sha256']
    verify_source_snapshot(FREEZE,{**plan['source_files'],**plan['scripts']})
    return plan


def run():
    guard();plan=check_freeze()
    if OUT.exists() or (BASE/'started.json').exists():raise RuntimeError('Do not duplicate or rerun the existing attempt')
    dump(BASE/'started.json',dict(started_at_utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(PROTOCOL)))
    result=run_study(PROTOCOL,'development',OUT,plan['workers'],False)
    check_freeze()
    outcome=dict(status='COMPLETED_WITH_ERRORS' if result['execution_errors'] else 'COMPLETED',
        completed_denominator=result['completed_denominator'],execution_errors=result['execution_errors'],
        source_integrity=result['source_integrity'],protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json',outcome);print(json.dumps(outcome),flush=True)


def trace_audit(folder,case):
    steps=read(folder/'paired_metrics.json')['steps']
    counts=Counter(d['map_reference_status'] for step in steps for d in step['diagnostics'])
    missing=[d for step in steps for d in step['diagnostics'] if d['map_reference_status']=='unavailable']
    assert all(d['mode']=='map_reference_unavailable' and d['nominal_command']==[0.,0.] for d in missing)
    circulation=Counter()
    with np.load(folder/'replay.npz',allow_pickle=False) as replay:
        positions=replay['agent_positions']
        for k,step in enumerate(steps):
            ids=[d['agent_id'] for d in step['diagnostics']]
            distances=np.linalg.norm(positions[k,:,None,:]-positions[k,None,:,:],axis=2)
            for i,d in enumerate(step['diagnostics']):
                permitted={ids[j] for j in np.flatnonzero(distances[i]<=case['config']['controller']['comm_range'])}
                assert set(d['map_reference_voters'])<=permitted
                assert all(step['diagnostics'][ids.index(aid)]['reference_status']=='accepted' for aid in d['map_reference_voters'])
                if d['mode']=='acquisition_circulation':
                    assert step['initial_reference']['status']=='acquiring'
                    assert case['arm']==ARMS[2] and not d['push_side'] and d['effort']==0
                    assert d['speed_nominal']<=case['config']['controller']['max_speed']+1e-12
                    circulation[step['phase']]+=1
    return dict(robot_steps_by_status=dict(counts),unavailable_nominal_zero_verified=True,
        every_voter_fresh_and_one_hop_verified=True,circulation_robot_steps_by_phase=dict(circulation),
        circulation_only_during_acquisition_verified=True)


def analyze():
    guard();plan=check_freeze()
    manifest,summary=read(OUT/'manifest.json'),read(OUT/'summary.json')
    assert summary['source_integrity']=='PASS' and summary['completed_denominator']==12
    assert manifest['identity']==identity(plan,PROTOCOL) and manifest['source_files']==plan['source_files']
    verify_source_snapshot(OUT,plan['source_files'])
    if PUBLIC.exists() or (BASE/'analysis.json').exists():raise RuntimeError('Preserve existing analysis')
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(ROOT/plan['original_run_dir'],plan['original_source_files'])
    episodes,reproductions=[],{}
    for case in plan['cases']:
        folder=OUT/case['case_id'];result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        row=episode(case,folder,result)
        row['scaling']=scaling(folder,row['barrier_scalings']) if row['execution']=='COMPLETED' else None
        if case['arm']!=ARMS[0] and row['execution']=='COMPLETED':row['map_reference']=trace_audit(folder,case)
        if case['arm']!=ARMS[2]:
            original=plan['originals'][case['case_id']];old_case=original['case']
            old_folder=ROOT/plan['original_run_dir']/old_case['case_id']
            assert sha(old_folder/'case_result.json')==original['receipt_sha256']
            old_result=cached_result(old_folder,{**plan['original_identity'],'case_sha256':encode(old_case)})
            reproductions[case['case_id']]=reproduction(old_folder,folder,old_result,result) if row['execution']=='COMPLETED' else dict(exact=False)
        episodes.append(row);print(case['case_id'],row['safe_completed'],row.get('endpoint_m'),flush=True);gc.collect()
    totals={arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        false_holds=sum(r['false_hold'] for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r['hard_safety']) for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        timeouts=sum(r.get('budget_terminated',False) for r in episodes if r['arm']==arm)) for arm in ARMS}
    candidate=[r for r in episodes if r['arm']==ARMS[2]]
    pairs={c:{a:next(r for r in episodes if r['condition']==c and r['arm']==a) for a in ARMS} for c in plan['conditions']}
    checks=dict(all_twelve_execution_complete=all(r['execution']=='COMPLETED' for r in episodes),
        all_eight_old_arms_exact=len(reproductions)==8 and all(r['exact'] for r in reproductions.values()),
        all_four_candidate_safe_complete_no_false_hold_valid_epoch=all(r['safe_completed'] and not r['false_hold'] and r['zero_epoch_valid'] for r in candidate),
        preserve_each_translation_baseline_g500_pass=all(not p[ARMS[0]]['g500'] or p[ARMS[2]]['g500'] for p in pairs.values()),
        candidate_acquisition_scalings_zero=all(r['scaling'] is not None and r['scaling']['acquisition']==0 for r in candidate),
        no_matched_post_acquisition_scaling_increase=all(p[ARMS[0]]['scaling'] is not None and p[ARMS[2]]['scaling'] is not None and p[ARMS[2]]['scaling']['post_acquisition']<=p[ARMS[0]]['scaling']['post_acquisition'] for p in pairs.values()))
    analysis=dict(scope=plan['scope'],episodes=episodes,totals=totals,gates=checks,baseline_reproduction=reproductions,
        advance_gate='PASS' if all(checks.values()) else 'FAIL',candidate_adopted=False,new_physical_episodes=12,
        protocol_sha256=sha(PROTOCOL),source_sha256=plan['frozen_source_sha256'],environment=plan['environment'],analyzer_sha256=sha(__file__))
    dump(BASE/'analysis.json',analysis);PUBLIC.mkdir(parents=True,exist_ok=False)
    for src,name in ((BASE/'analysis.json','analysis.json'),(PROTOCOL,'protocol.json'),(BASE/'execution_outcome.json','execution_outcome.json')):
        shutil.copy2(src,PUBLIC/name)
    print(json.dumps(dict(totals=totals,gates=checks,advance_gate=analysis['advance_gate']),indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('freeze','run','analyze'))
    globals()[parser.parse_args().action]()
