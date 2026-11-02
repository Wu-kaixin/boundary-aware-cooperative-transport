"""Separately frozen resource continuation of the unchanged four-case replay."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import shutil
import time

from research_io import ROOT, guard, read, sha
from task_translation_study import offline_case
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import (
    cached_result, environment_versions, limit_worker_threads, source_hashes, verify_source_snapshot)

PRIOR = ROOT/'experiments/ccta2027/task_translation_v2.json'
DESIGN = ROOT/'experiments/ccta2027/task_translation_completion_v1_design.md'
PROTOCOL = ROOT/'experiments/ccta2027/task_translation_completion_v1.json'
OUT = ROOT/'runs/research/2026-11-03-task-translation-completion-v1'
PUBLIC = ROOT/'docs/results/task_translation_completion_v1'
WRAPPER = 'scripts/complete_task_translation_study.py'


def verify_prior():
    guard()
    prior = read(PRIOR)
    raw = ROOT/prior['run_dir']
    assert sha(PRIOR) == sha(raw/'protocol.json')
    outcome = read(raw/'execution_outcome.json')
    assert outcome['execution_status'] == 'WALL_BUDGET_EXHAUSTED'
    assert outcome['observed_exit_code'] == 1 and outcome['observed_exec_session'] == 3893
    assert outcome['protocol_sha256'] == sha(PRIOR) and not (raw/'analysis.json').exists()
    assert source_hashes() == prior['source_files'] and environment_versions() == prior['environment']
    assert {n: sha(ROOT/n) for n in prior['scripts']} == prior['scripts']
    assert sha(ROOT/'experiments/ccta2027/task_translation_v1_design.md') == prior['design_sha256']
    assert sha(ROOT/'experiments/ccta2027/task_reference_v1.json') == prior['original_protocol_sha256']
    assert sha(ROOT/'docs/results/reference_surface_v1/protected_baseline.json') == prior['protected_baseline_sha256']
    verify_source_snapshot(raw, {**prior['source_files'], **prior['scripts']})
    verify_source_snapshot(ROOT/prior['original_run_dir'], prior['original_source_files'])
    checkpoint = read(ROOT/'docs/results/task_translation_checkpoint_v1/analysis.json')
    reused, missing = {}, []
    for case in prior['cases']:
        original = ROOT/prior['original_run_dir']/case['case_id']
        assert sha(original/'case_result.json') == prior['receipts'][case['case_id']]
        cached_result(original, {**prior['original_identity'], 'case_sha256': encode(case)})
        folder = raw/case['case_id']
        if not (folder/'summary.json').exists():
            missing.append(case['case_id'])
            continue
        row = read(folder/'summary.json')
        assert row == next(r for r in checkpoint['cases'] if r['case_id'] == case['case_id'])
        assert sha(folder/'series.npz') == row['series_sha256']
        assert sha(folder/'templates.json') == row['templates_sha256']
        reused[case['case_id']] = dict(path=folder.relative_to(ROOT).as_posix(),
            products={n: sha(folder/n) for n in ('summary.json', 'series.npz', 'templates.json')})
    assert list(reused) == outcome['completed_cases'] and missing == outcome['incomplete_cases']
    assert len(reused) == 3 and missing == ['far_skew_box_s0_r0_pos0_task_reference']
    return prior, reused, missing


def freeze():
    if PROTOCOL.exists() or OUT.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the frozen continuation')
    prior, reused, missing = verify_prior()
    scripts = {**prior['scripts'], WRAPPER: sha(ROOT/WRAPPER)}
    plan = dict(version='task-translation-completion-v1', status='FROZEN_BEFORE_EXECUTION',
        prior_protocol_sha256=sha(PRIOR),
        prior_outcome_sha256=sha(ROOT/prior['run_dir']/'execution_outcome.json'),
        source_files=prior['source_files'], environment=prior['environment'], scripts=scripts,
        cases=prior['cases'], arms=prior['arms'], selected_candidate=prior['selected_candidate'],
        admission=prior['admission'], reused=reused, recompute_cases=missing,
        run_dir=OUT.relative_to(ROOT).as_posix(), workers=1, internal_threads=1,
        wall_budget_seconds=2700, prior_budget_seconds=prior['wall_budget_seconds'],
        design_sha256=sha(DESIGN), new_physical_episodes=0,
        scope='Three hash-verified v2 outputs reused; one missing case recomputed with unchanged v2 code and gates. All inputs are old physical trajectories.')
    OUT.mkdir(parents=True, exist_ok=False)
    for name in [*plan['source_files'], *scripts]:
        dest = OUT/'source_snapshot'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, dest)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, OUT/'protocol.json')
    shutil.copy2(DESIGN, OUT/'design.md')
    print(json.dumps(dict(status=plan['status'], reused=list(reused), recompute=missing,
                         budget_seconds=plan['wall_budget_seconds'])), flush=True)


def run():
    guard()
    limit_worker_threads()
    plan = read(PROTOCOL)
    if (OUT/'started.json').exists() or PUBLIC.exists():
        raise RuntimeError('Preserve every started continuation')
    prior, reused, missing = verify_prior()
    assert sha(PROTOCOL) == sha(OUT/'protocol.json')
    assert sha(PRIOR) == plan['prior_protocol_sha256']
    assert sha(ROOT/prior['run_dir']/'execution_outcome.json') == plan['prior_outcome_sha256']
    assert reused == plan['reused'] and missing == plan['recompute_cases']
    for key in ('cases', 'arms', 'selected_candidate', 'admission', 'source_files', 'environment'):
        assert plan[key] == prior[key], key
    assert {n: sha(ROOT/n) for n in plan['scripts']} == plan['scripts']
    assert sha(DESIGN) == plan['design_sha256']
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    dump(OUT/'started.json', dict(started_at_utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha(PROTOCOL), environment=environment_versions()))
    started = time.perf_counter()
    rows, origins = [], {}
    for case in plan['cases']:
        cid = case['case_id']
        if cid in reused:
            rows.append(read(ROOT/reused[cid]['path']/'summary.json'))
            origins[cid] = dict(kind='reused_complete_v2_output', **reused[cid])
            continue
        assert cid in missing
        dest = OUT/cid
        dest.mkdir(exist_ok=False)
        rows.append(offline_case(case, ROOT/prior['original_run_dir']/cid, dest,
                                 started+plan['wall_budget_seconds']))
        origins[cid] = dict(kind='recomputed_missing_case', path=dest.relative_to(ROOT).as_posix(),
            products={n: sha(dest/n) for n in ('summary.json', 'series.npz', 'templates.json')})
    assert [r['case_id'] for r in rows] == [c['case_id'] for c in prior['cases']]
    gates, admission = {}, plan['admission']
    for row in rows:
        candidate = row['arms'][plan['selected_candidate']]
        mae, old_mae = candidate['paired_error']['mae_m'], candidate['paired_original_error']['mae_m']
        gates[row['condition']] = dict(availability=candidate['paired_availability'] >= admission['paired_availability_min'],
            error=mae is not None and mae <= admission['paired_mae_max_m'] and mae <= old_mae+admission['paired_mae_increase_max_m'],
            false_goal_stop=candidate['false_goal_stop_frames'] <= admission['false_goal_stop_frames_max'],
            sustained_feedback=candidate['longest_valid_potential_stop_streak'] >= admission['consecutive_valid_potential_stop_frames_min'])
    checkpoint = read(ROOT/'docs/results/task_translation_checkpoint_v1/analysis.json')
    assert all(gates[k] == v for k, v in checkpoint['completed_case_gates'].items())
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert {n: sha(ROOT/n) for n in plan['scripts']} == plan['scripts']
    assert all(sha(ROOT/o['path']/n) == h for o in origins.values() for n, h in o['products'].items())
    result = dict(scope=plan['scope'], cases=rows, gates=gates, origins=origins,
        physical_trial_screen='PASS' if all(all(v.values()) for v in gates.values()) else 'FAIL',
        new_physical_episodes=0, candidate_adopted=False, source_integrity='PASS',
        source_sha256=encode(plan['source_files']), protocol_sha256=sha(PROTOCOL),
        prior_protocol_sha256=sha(PRIOR), environment=plan['environment'],
        wall_seconds=time.perf_counter()-started, prior_execution_status='WALL_BUDGET_EXHAUSTED')
    dump(OUT/'analysis.json', result)
    dump(OUT/'execution_outcome.json', dict(execution_status='COMPLETED',
        protocol_sha256=sha(PROTOCOL), scientific_screen=result['physical_trial_screen']))
    PUBLIC.mkdir(parents=True, exist_ok=False)
    dump(PUBLIC/'analysis.json', result)
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    print(json.dumps(dict(status=result['physical_trial_screen'], gates=gates)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'run'])
    globals()[parser.parse_args().action]()
