"""Recover aggregation only after the complete replay's recorded path-join error.

No estimator updates or physics. The original failed execution and every raw
product remain unchanged; this finalization has a separate manifest and output.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import shutil
import time

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, environment_versions, source_hashes, verify_source_snapshot

PROTOCOL = ROOT/'experiments/ccta2027/task_translation_completion_v1.json'
OUT = ROOT/'runs/research/2026-11-03-task-translation-finalization-v1'
PUBLIC = ROOT/'docs/results/task_translation_completion_v1'


def main():
    guard()
    if OUT.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve existing finalization and publication')
    started = time.perf_counter()
    plan = read(PROTOCOL)
    raw = ROOT/plan['run_dir']
    prior_path = ROOT/'experiments/ccta2027/task_translation_v2.json'
    prior = read(prior_path)
    failure = read(raw/'execution_outcome.json')
    assert failure['execution_status'] == 'POSTPROCESSING_FAILED'
    assert failure['observed_exec_session'] == 25427 and failure['observed_exit_code'] == 1
    assert failure['protocol_sha256'] == sha(PROTOCOL) and not (raw/'analysis.json').exists()
    assert sha(PROTOCOL) == sha(raw/'protocol.json')
    assert sha(prior_path) == plan['prior_protocol_sha256']
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    # Original captured wrapper retains the failing byte; only its final path
    # assertion was faulty. The estimator function and runtime remain identical.
    verify_source_snapshot(raw, {**plan['source_files'], **plan['scripts']})
    verify_source_snapshot(ROOT/prior['run_dir'], {**prior['source_files'], **prior['scripts']})
    assert {n: sha(ROOT/n) for n in prior['scripts']} == prior['scripts']
    for key in ('cases', 'arms', 'selected_candidate', 'admission', 'source_files', 'environment'):
        assert plan[key] == prior[key]
    checkpoint = read(ROOT/'docs/results/task_translation_checkpoint_v1/analysis.json')
    rows, origins = [], {}
    for case in plan['cases']:
        cid = case['case_id']
        old = ROOT/prior['original_run_dir']/cid
        assert sha(old/'case_result.json') == prior['receipts'][cid]
        cached_result(old, {**prior['original_identity'], 'case_sha256': encode(case)})
        if cid in plan['reused']:
            origin = dict(kind='reused_complete_v2_output', **plan['reused'][cid])
        else:
            assert cid in plan['recompute_cases']
            folder = raw/cid
            origin = dict(kind='recomputed_missing_case', path=folder.relative_to(ROOT).as_posix(),
                products={n: sha(folder/n) for n in ('summary.json', 'series.npz', 'templates.json')})
        folder = ROOT/origin['path']
        assert all(sha(folder/n) == h for n, h in origin['products'].items())
        row = read(folder/'summary.json')
        assert row['case_id'] == cid and row['condition'] == case['condition_id']
        assert sha(folder/'series.npz') == row['series_sha256']
        assert sha(folder/'templates.json') == row['templates_sha256']
        if cid in plan['reused']:
            assert row == next(r for r in checkpoint['cases'] if r['case_id'] == cid)
        else:
            assert sha(folder/'summary.json') == failure['summary_sha256']
            assert row['series_sha256'] == failure['series_sha256']
        rows.append(row)
        origins[cid] = origin
    assert len(rows) == 4 and len(plan['reused']) == 3 and len(plan['recompute_cases']) == 1
    OUT.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, OUT/'finalizer_source.py')
    manifest = dict(scope=__doc__, started_at_utc=datetime.now(timezone.utc).isoformat(),
        script_sha256=sha(__file__), study_protocol_sha256=sha(PROTOCOL),
        failed_execution_receipt_sha256=sha(raw/'execution_outcome.json'),
        original_failed_wrapper_sha256=plan['scripts']['scripts/complete_task_translation_study.py'],
        corrected_wrapper_sha256=sha(ROOT/'scripts/complete_task_translation_study.py'),
        origins=origins, new_physical_episodes=0, estimator_updates=0)
    dump(OUT/'manifest.json', manifest)
    gates, admission = {}, plan['admission']
    for row in rows:
        candidate = row['arms'][plan['selected_candidate']]
        mae, old_mae = candidate['paired_error']['mae_m'], candidate['paired_original_error']['mae_m']
        gates[row['condition']] = dict(availability=candidate['paired_availability'] >= admission['paired_availability_min'],
            error=mae is not None and mae <= admission['paired_mae_max_m'] and mae <= old_mae+admission['paired_mae_increase_max_m'],
            false_goal_stop=candidate['false_goal_stop_frames'] <= admission['false_goal_stop_frames_max'],
            sustained_feedback=candidate['longest_valid_potential_stop_streak'] >= admission['consecutive_valid_potential_stop_frames_min'])
    assert all(gates[k] == v for k, v in checkpoint['completed_case_gates'].items())
    assert all(sha(ROOT/o['path']/n) == h for o in origins.values() for n, h in o['products'].items())
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    result = dict(scope=plan['scope'], cases=rows, gates=gates, origins=origins,
        physical_trial_screen='PASS' if all(all(v.values()) for v in gates.values()) else 'FAIL',
        new_physical_episodes=0, candidate_adopted=False, source_integrity='PASS',
        source_sha256=encode(plan['source_files']), protocol_sha256=sha(PROTOCOL),
        prior_protocol_sha256=sha(prior_path), environment=plan['environment'],
        prior_execution_status='WALL_BUDGET_EXHAUSTED', replay_execution_status='POSTPROCESSING_FAILED',
        finalization_status='COMPLETED', finalization_manifest_sha256=sha(OUT/'manifest.json'),
        finalization_wall_seconds=time.perf_counter()-started)
    dump(OUT/'analysis.json', result)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    dump(PUBLIC/'analysis.json', result)
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    shutil.copy2(OUT/'manifest.json', PUBLIC/'finalization_manifest.json')
    shutil.copy2(raw/'execution_outcome.json', PUBLIC/'replay_execution_outcome.json')
    print(json.dumps(dict(status=result['physical_trial_screen'], gates=gates,
        finalization=result['finalization_status'], estimator_updates=0)), flush=True)


if __name__ == '__main__':
    main()
