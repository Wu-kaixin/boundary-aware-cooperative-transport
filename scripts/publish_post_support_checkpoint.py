"""Publish a dated checkpoint, never fabricate pending candidate outcomes."""
from __future__ import annotations

from datetime import datetime, timezone
import shutil

import post_acquisition_support_study as study
from analyze_post_acquisition_support_v1 import control_reproduction
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


PUBLIC = study.ROOT / 'docs/results/post_support_validation_checkpoint_v1'


def main():
    guard()
    plan = study.check()
    manifest = read(study.OUT / 'manifest.json')
    assert manifest['identity'] == study.identity(plan, study.PROTOCOL)
    rows, reproductions = [], {}
    for case in plan['cases']:
        folder = study.OUT / case['case_id']
        receipt = folder / 'case_result.json'
        row = dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
                   terminal_receipt_present=receipt.is_file())
        if receipt.is_file():
            result = cached_result(folder, {**manifest['identity'], 'case_sha256': encode(case)})
            row.update(execution_status=result['execution_status'], receipt_sha256=sha(receipt))
            if case['arm'] == 'map_bridge_control':
                original = plan['originals'][case['condition_id']]
                old_case = original['case']
                old_folder = study.ROOT / plan['original_run_dir'] / old_case['case_id']
                assert sha(old_folder / 'case_result.json') == original['receipt_sha256']
                old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256': encode(old_case)})
                reproductions[case['condition_id']] = control_reproduction(
                    old_folder, folder, old_result, result, old_case, case)
        rows.append(row)
    assert len(rows) == 8 and len(reproductions) == 4 and all(r['exact'] for r in reproductions.values())
    preflight = read(study.BASE / 'analysis_preflight_v1/outcome.json')
    assert preflight['status'] == 'VERIFIED_SYNTHETIC_REPLAY' and preflight['audit']['applied'] > 0
    assert preflight['helper_sha256']['analyze_post_acquisition_support_v1.py'] == sha(study.ROOT / 'scripts/analyze_post_acquisition_support_v1.py')
    tests = {name: study.test_receipt(study.CHECKS / name)
             for name in ('publication-core.xml', 'packet-comparison-green.xml')}
    PUBLIC.mkdir(parents=True, exist_ok=False)
    dump(PUBLIC / 'execution_snapshot.json', dict(
        observed_at_utc=datetime.now(timezone.utc).isoformat(), planned_physical_episodes=8,
        execution_outcome_present=(study.BASE / 'execution_outcome.json').is_file(), cases=rows,
        completed_receipts=sum(r['terminal_receipt_present'] for r in rows),
        candidate_task_verdict='NOT_YET_AUDITED', candidate_adopted=False,
        protocol_sha256=sha(study.PROTOCOL), source_sha256=plan['frozen_source_sha256'],
        scope='Dated receipt observation, not an eight-episode outcome or candidate acceptance.'))
    dump(PUBLIC / 'control_reproduction.json', dict(checks=reproductions,
        adapter_sha256=sha(study.ROOT / 'scripts/analyze_post_acquisition_support_v1.py'),
        protocol_sha256=sha(study.PROTOCOL)))
    shutil.copy2(study.CHECKS / 'control_reproduction_precheck_v1.json', PUBLIC / 'original_label_mismatch.json')
    compact = {k: v for k, v in preflight['audit'].items() if k != 'violations'}
    dump(PUBLIC / 'engineering_checks.json', dict(tests=tests,
        freeze_tests=plan['tests'], canonical_sha256=plan['canonical_sha256'],
        source_sha256=plan['frozen_source_sha256'], frozen_helper_count=len(plan['scripts']),
        synthetic_preflight=dict(status=preflight['status'], frames=preflight['frames'],
            new_physical_episodes=0, scope=preflight['scope'], audit=compact,
            omitted_residual_records=len(preflight['audit']['violations']),
            source_helpers=preflight['helper_sha256'],
            raw_outcome='runs/research/2026-11-03-post-acquisition-support-v1/analysis_preflight_v1/outcome.json',
            raw_outcome_sha256=sha(study.BASE / 'analysis_preflight_v1/outcome.json')),
        analysis_status='Full eight-episode candidate replay and gates remain pending.'))
    dump(PUBLIC / 'publication.json', dict(publisher_sha256=sha(__file__),
        products={p.name: sha(p) for p in sorted(PUBLIC.iterdir()) if p.is_file()},
        scope='Frozen study and completed engineering checkpoint; no candidate physical success claim.'))
    print(dict(published=str(PUBLIC), completed_receipts=sum(r['terminal_receipt_present'] for r in rows)), flush=True)


if __name__ == '__main__':
    main()
