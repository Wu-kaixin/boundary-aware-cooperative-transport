"""Freeze, execute and independently assess four exact safety-QP replays."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import shutil
import subprocess
import sys

from research_io import ROOT, guard, read, sha

MISSION = '2026-11-03-qp-conflict-v1'
BASE = ROOT/'runs/research'/MISSION
PROTOCOL = ROOT/'experiments/ccta2027/qp_conflict_v1.json'
DESIGN = ROOT/'experiments/ccta2027/qp_conflict_v1_design.md'
OLD = ROOT/'experiments/ccta2027/acquisition_geometry_v1.json'
PUBLIC = ROOT/'docs/results/qp_conflict_v1'
SCRIPTS = ('scripts/qp_conflict_study.py','scripts/replay_qp_conflict.py','scripts/research_io.py',
           'scripts/task_translation_physical_study_v2.py')
CASES = dict(far_c_open_away_pos0_translation_all_captured=579,
    far_c_open_away_pos0_safe_geometry=933,
    far_skew_box_s0_r0_pos0_translation_all_captured=784,
    far_skew_box_s0_r0_pos0_safe_geometry=1115)


def freeze():
    guard()
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result, verify_source_snapshot
    from task_translation_physical_study_v2 import package_inventory
    if BASE.exists() or PROTOCOL.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the existing freeze')
    old = read(OLD)
    assert source_hashes()==old['source_files']
    assert environment_versions()==old['environment']
    batch = ROOT/old['run_dir']
    manifest = read(batch/'manifest.json')
    verify_source_snapshot(batch, old['source_files'])
    analysis_path = ROOT/'docs/results/acquisition_geometry_v1/analysis.json'
    analysis = read(analysis_path)
    receipts, counts = {}, {}
    for key, end in CASES.items():
        case = next(c for c in old['cases'] if c['case_id']==key)
        result = cached_result(batch/key, {**manifest['identity'],'case_sha256':encode(case)})
        assert result['execution_status']=='COMPLETED'
        row = next(r for r in analysis['episodes'] if r['case_id']==key)
        assert row['scaling']['events'][-1]['frame']==end
        receipts[key] = sha(batch/key/'case_result.json')
        counts[key] = row['barrier_scalings']
    plan = dict(mission=MISSION, status='FROZEN_BEFORE_DIAGNOSTIC', new_physical_episodes=0,
        prefixes=CASES, total_control_frames=sum(CASES.values()), workers=4, internal_threads=1,
        expected_scaling_counts=counts, original_protocol_sha256=sha(OLD),
        original_run_dir=old['run_dir'], original_source_files=old['source_files'],
        original_identity=manifest['identity'], receipts=receipts,
        existing_analysis_sha256=sha(analysis_path), source_files=source_hashes(),
        environment=environment_versions(), installed_locked_packages=package_inventory(),
        scripts={s:sha(ROOT/s) for s in SCRIPTS}, design_sha256=sha(DESIGN),
        witness_selection='First and lowest-scale (earliest tie) event per phase within each fixed prefix',
        exactness='All commands, recorded diagnostics, solver statistics, phase guards/response and packed map snapshot for every frame',
        scope='Retrospective constraint diagnosis on existing development data, not controller adoption or physical validation')
    snapshot = BASE/'freeze/source_snapshot'
    for name in [*plan['source_files'], *SCRIPTS]:
        target = snapshot/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,target)
    dump(PROTOCOL,plan)
    shutil.copy2(PROTOCOL,BASE/'freeze/protocol.json')
    shutil.copy2(DESIGN,BASE/'freeze/design.md')
    print(json.dumps(dict(status=plan['status'], cases=4, frames=sum(CASES.values()),
        scaled_robot_steps=sum(counts.values()), protocol_sha256=sha(PROTOCOL))),flush=True)


def check():
    from dbact_sim.experiments.study import source_hashes, environment_versions, verify_source_snapshot
    from task_translation_physical_study_v2 import package_inventory
    plan = read(PROTOCOL)
    assert sha(PROTOCOL)==sha(BASE/'freeze/protocol.json')
    assert sha(DESIGN)==sha(BASE/'freeze/design.md')==plan['design_sha256']
    assert {s:sha(ROOT/s) for s in SCRIPTS}==plan['scripts']
    assert source_hashes()==plan['source_files'] and environment_versions()==plan['environment']
    assert package_inventory()==plan['installed_locked_packages']
    assert sha(OLD)==plan['original_protocol_sha256']
    verify_source_snapshot(BASE/'freeze',{**plan['source_files'],**plan['scripts']})
    return plan


def run():
    guard()
    from dbact_sim.experiments.deployment import dump
    plan = check()
    if (BASE/'started.json').exists():
        raise RuntimeError('Do not duplicate an existing diagnostic attempt')
    dump(BASE/'started.json',dict(started_at_utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(PROTOCOL)))
    def one(item):
        key,end = item
        command = [sys.executable,str(ROOT/'scripts/replay_qp_conflict.py'),'--case',key,
                   '--out',str(BASE/key),'--end-frame',str(end)]
        with (BASE/(key+'.log')).open('x',encoding='utf-8') as log:
            result = subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        print(json.dumps(dict(case=key,exit_code=result.returncode)),flush=True)
        return dict(case_id=key,end_frame=end,exit_code=result.returncode,command=command)
    with ThreadPoolExecutor(max_workers=plan['workers']) as pool:
        results = list(pool.map(one,plan['prefixes'].items()))
    check()
    outcome = dict(status='COMPLETED' if all(r['exit_code']==0 for r in results) else 'COMPLETED_WITH_ERRORS',
        cases=results,new_physical_episodes=0,protocol_sha256=sha(PROTOCOL))
    dump(BASE/'execution_outcome.json',outcome)
    print(json.dumps(outcome),flush=True)
    if outcome['status']!='COMPLETED':
        raise SystemExit(1)


def independent(A,b,max_speed):
    import cvxpy as cp
    import numpy as np
    A,b = np.asarray(A,dtype=float).reshape(-1,2),np.asarray(b,dtype=float)
    u = cp.Variable(2)
    constraints = [A@u>=b] if len(b) else []
    if max_speed is not None:
        constraints.append(cp.norm(u,2)<=max_speed)
    problem = cp.Problem(cp.Minimize(cp.sum_squares(u)),constraints)
    problem.solve(solver=cp.CLARABEL)
    out = dict(status=problem.status,feasible=problem.status in ('optimal','optimal_inaccurate'))
    if out['feasible']:
        value = np.asarray(u.value).reshape(2)
        residual = float(np.min(A@value-b)) if len(b) else 0.
        speed = float(np.linalg.norm(value))
        out.update(u=value.tolist(),minimum_speed=speed,min_residual=residual,
            numerically_valid=residual>=-1e-7 and (max_speed is None or speed<=max_speed+1e-7))
    else:
        out['numerically_valid'] = problem.status in ('infeasible','infeasible_inaccurate')
    return out


def analyze():
    guard()
    import numpy as np
    from collections import Counter
    from dbact_sim.experiments.deployment import dump
    plan = check()
    assert read(BASE/'execution_outcome.json')['status']=='COMPLETED'
    if PUBLIC.exists() or (BASE/'analysis.json').exists():
        raise RuntimeError('Preserve existing analysis')
    summaries,witnesses = [],{}
    for key,end in plan['prefixes'].items():
        replay = read(BASE/key/'reconstruction.json')
        assert replay['status']=='EXACT_RECONSTRUCTION' and replay['verified_frames']==end
        assert replay['receipt_sha256']==plan['receipts'][key]
        events = replay['events']
        assert len(events)==replay['end_scaling_counter']==plan['expected_scaling_counts'][key]
        counts = {k:sum(e['feasibility'][k] for e in events) for k in events[0]['feasibility']}
        summaries.append(dict(case_id=key,verified_frames=end,scaled_robot_steps=len(events),
            feasible_counts=counts,by_phase=dict(Counter(e['phase'] for e in events)),
            observed_h_negative=sum(e['minimum_h']<0 for e in events),
            true_h_negative=sum(e['true_h_m']<0 for e in events),
            true_clearance_range_m=[min(e['true_signed_distance_m'] for e in events),max(e['true_signed_distance_m'] for e in events)],
            estimate_velocity_norm_range=[min(np.linalg.norm(e['object_velocity']) for e in events),max(np.linalg.norm(e['object_velocity']) for e in events)],
            min_scale=min(e['scale'] for e in events),reconstruction_sha256=sha(BASE/key/'reconstruction.json')))
        for label,w in replay['witnesses'].items():
            A,full,free,zero = [np.asarray(w[n]) for n in ('A','b_original','b_no_margin','b_zero_motion')]
            kinds = np.asarray(w['row_kinds'])
            hard,obj = kinds!='object',kinds=='object'
            checks = {name:independent(a,b,w['max_speed']) for name,a,b in (
                ('full',A,full),('no_rho',A,free),('hard_only',A[hard],free[hard]),
                ('object_only',A[obj],free[obj]),('zero_estimated_motion',A,zero))}
            without_cap = independent(A,free,None)
            agreement = all(c['numerically_valid'] and c['feasible']==w['feasibility'][n] for n,c in checks.items())
            witnesses[key+'/'+label] = dict(**w,independent=checks,without_speed_cap=without_cap,
                independent_agreement=agreement)
    result = dict(scope=plan['scope'],cases=summaries,witnesses=witnesses,
        all_selected_independent_agree=all(w['independent_agreement'] for w in witnesses.values()),
        all_exact_frames=sum(s['verified_frames'] for s in summaries),
        scaled_robot_steps=sum(s['scaled_robot_steps'] for s in summaries),new_physical_episodes=0,
        protocol_sha256=sha(PROTOCOL),analyzer_sha256=sha(__file__))
    dump(BASE/'analysis.json',result)
    PUBLIC.mkdir(parents=True,exist_ok=False)
    for source,dest in ((BASE/'analysis.json','analysis.json'),(PROTOCOL,'protocol.json'),
                        (BASE/'execution_outcome.json','execution_outcome.json')):
        shutil.copy2(source,PUBLIC/dest)
    print(json.dumps(dict(cases=summaries,independent_agreement=result['all_selected_independent_agree']),indent=2),flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('freeze','run','analyze'))
    globals()[parser.parse_args().action]()
