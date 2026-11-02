"""Freeze/run/analyze the bounded initial-task-reference development comparison."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
import copy
import gc
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, run_study, cached_result, verify_source_snapshot

MISSION = '2026-11-03-task-reference-v1'
PROTOCOL = ROOT/'experiments/ccta2027/task_reference_v1.json'
OUT = ROOT/'runs/research'/MISSION
PUBLIC = ROOT/'docs/results/task_reference_v1'
CONDITIONS = ('near_circle', 'near_l_reference', 'far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0')
ARMS = ('reference', 'acquisition_only', 'task_reference')


def freeze():
    guard()
    if PROTOCOL.exists() or OUT.exists():
        raise RuntimeError('Preserve the frozen development study')
    original = read(ROOT/'experiments/ccta2027/reference_origin_v1.json')
    cases = []
    for condition in CONDITIONS:
        base = next(c for c in original['cases'] if c['condition_id']==condition and c['arm']=='reference')
        for arm in ARMS:
            case = copy.deepcopy(base)
            case.update(case_id=condition+'_'+arm, arm=arm)
            case['factors']['cohort'] = 'initial_task_reference_development'
            case['config']['controller'].update(initial_reference_acquisition=arm!='reference',
                                                initial_task_reference=arm=='task_reference', initial_acquisition_seconds=60.)
            cases.append(case)
    plan = dict(mission=MISSION, run_dir=OUT.relative_to(ROOT).as_posix(), cases=cases,
        arms=list(ARMS), conditions=list(CONDITIONS), workers=4, internal_threads=1,
        budget=dict(physical_episodes=12, physical_seconds_per_episode=600., acquisition_seconds=60.,
                    settling_frames=40, settling_speed=.005),
        semantics='Common workspace point q; zero initial offset only after observed standoff acquisition. '
                  'Unknown-center error maximized over known workspace vertices; no new size/shape prior. '
                  'Yaw-unobservable matches abstain. Empirical registration admission is not an error certificate.',
        hypotheses=['Additional acquisition alone may change task outcomes and must be separated.',
                    'Initial common-point reference plus unknown-center guard may remove false HOLD, yet yaw ambiguity or conservatism may prevent completion.'],
        adoption='All four conditions safely completed, far-skew rescued, no false HOLD, all acquisition zero epochs valid, '
                 'no new hard-safety failure, and G500 at least reference. Only then extend to the other four development conditions.',
        failure_policy='Retain every failure; no same-identity retuning, no per-shape arm combinations; no independent contribution claim from these reused blocks.',
        source_files=source_hashes(), frozen_source_sha256=encode(source_hashes()), environment=environment_versions(),
        runner_sha256=sha(__file__), original_protocol_sha256=sha(ROOT/'experiments/ccta2027/reference_origin_v1.json'))
    dump(PROTOCOL, plan)
    print({'frozen':str(PROTOCOL), 'episodes':len(cases)})


def run():
    guard(); plan=read(PROTOCOL)
    assert plan['runner_sha256']==sha(__file__) and plan['environment']==environment_versions()
    result=run_study(PROTOCOL,'development',OUT,plan['workers'],False)
    shutil.copy2(__file__,OUT/'study_helper.py')
    print({'completed':result['completed_denominator'],'source_integrity':result['source_integrity']})


def analyze():
    guard(); plan=read(PROTOCOL); manifest=read(OUT/'manifest.json'); summary=read(OUT/'summary.json')
    assert summary['source_integrity']=='PASS' and summary['completed_denominator']==len(plan['cases'])
    assert manifest['identity']['protocol_sha256']==sha(PROTOCOL)
    assert manifest['identity']['source_sha256']==plan['frozen_source_sha256']
    assert manifest['source_files']==plan['source_files']
    verify_source_snapshot(OUT,manifest['source_files'])
    if PUBLIC.exists():
        raise RuntimeError('Preserve the published analysis')
    PUBLIC.mkdir(parents=True)
    episodes=[]; identities={}; reproductions={}
    original=read(ROOT/'experiments/ccta2027/reference_origin_v1.json')
    for case in plan['cases']:
        folder=OUT/case['case_id']; receipt=read(folder/'case_result.json')
        expected={**manifest['identity'],'case_sha256':encode(case)}
        r=cached_result(folder,expected); identities[case['case_id']]=sha(folder/'case_result.json')
        row=dict(condition=case['condition_id'],arm=case['arm'],execution=r['execution_status'],
                 completed=r.get('task_completed',False),safe_completed=False,g500=r.get('g500_pass',False))
        if r['execution_status']!='COMPLETED':
            row['error']=r.get('error');episodes.append(row);continue
        gate=read(folder/'summary.json')['cargoes']['cargo_0']['g500']; m=gate['metrics']
        contract=ClosedLoopContract(**gate['gates']); verdict=contract.evaluate(m)
        assert verdict.success==gate['success'] and verdict.reasons==gate['failure_reasons']
        hard=[]
        if m['min_inter_agent_distance']<m['d_min']-contract.d_min_tolerance: hard.append('separation')
        if m['min_signed_clearance']<0: hard.append('inside_object')
        if m['max_penetration']>m['penetration_budget']: hard.append('penetration_budget')
        if m['solver_fallbacks'] or m['solver_infeasible']: hard.append('solver_failure')
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            xy=z['cargo/cargo_0/centers'].copy(); goal=z['task/cargo_0/goal_point'].copy()
        first=next((s for s in steps if s['phase']=='HOLD'),None)
        hold=None if first is None else dict(frame=first['frame'],
            physical_error_before_m=float(np.linalg.norm(xy[first['frame']-1]-goal)),
            physical_error_after_m=float(np.linalg.norm(xy[first['frame']]-goal)),
            nominal_error_m=float(np.linalg.norm(first['navigation_supervisor']['error'])) if first['navigation_supervisor'] else None,
            workspace_error_bound_m=first['initial_reference']['center_error_bound'])
        false_hold=bool(hold and hold['physical_error_before_m']>case['config']['task']['tolerance'])
        row.update(safe_completed=bool(row['completed'] and not hard),hard_safety=hard,
            endpoint_m=r['euclidean_endpoint_error'],seconds=r['termination']['frames_run']*case['config']['dt'],
            final_phase=steps[-1]['phase'],false_hold=false_hold,first_hold=hold,
            acquisition=r.get('initial_reference'),barrier_scalings=m['barrier_scalings'],
            reference_admission={reason:sum(d['reference_status']==reason for s in steps for d in s['diagnostics'])
                                 for reason in sorted({d['reference_status'] for s in steps for d in s['diagnostics']})})
        if case['arm']=='reference':
            old=ROOT/original['run_dir']/case['case_id']/'replay.npz'
            with np.load(old,allow_pickle=False) as a,np.load(folder/'replay.npz',allow_pickle=False) as b:
                physical=('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','phase')
                reproductions[case['condition_id']]=all(np.array_equal(a[k],b[k],equal_nan=True) for k in physical)
            assert reproductions[case['condition_id']]
        episodes.append(row); print(case['case_id'],row['completed'],row['endpoint_m'],flush=True)
        del steps; gc.collect()
    totals={arm:dict(episodes=sum(r['arm']==arm for r in episodes),
        completed=sum(r['completed'] for r in episodes if r['arm']==arm),
        safe_completed=sum(r['safe_completed'] for r in episodes if r['arm']==arm),
        g500=sum(r['g500'] for r in episodes if r['arm']==arm),
        false_holds=sum(r.get('false_hold',False) for r in episodes if r['arm']==arm),
        hard_failures=sum(bool(r.get('hard_safety',['execution'])) for r in episodes if r['arm']==arm)) for arm in ARMS}
    selected=[r for r in episodes if r['arm']=='task_reference']
    adopted=all(r['safe_completed'] and not r.get('false_hold') and r.get('acquisition',{}).get('zero_epoch_valid') for r in selected)
    adopted=adopted and totals['task_reference']['g500']>=totals['reference']['g500']
    dump(PUBLIC/'analysis.json',dict(episodes=episodes,totals=totals,adoption='PASS' if adopted else 'FAIL',
        baseline_physical_reproduction=reproductions,receipt_sha256=identities,source_sha256=plan['frozen_source_sha256'],
        environment=manifest['identity']['environment'],protocol_sha256=sha(PROTOCOL),analysis_helper_sha256=sha(__file__)))
    print(totals,'adopted',adopted)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['freeze','run','analyze'])
    globals()[parser.parse_args().action]()
