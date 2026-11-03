"""Export terminal task receipts; full controller/constraint audits stay separate."""
from __future__ import annotations

import csv
import argparse
from datetime import datetime, timezone
import gc
from pathlib import Path
import shutil

import numpy as np

import post_acquisition_support_study as study
from publish_post_support_checkpoint import PUBLIC
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main():
    guard()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', action='store_true', help='Publish all eight scheduled rows, including explicitly pending cases.')
    args = parser.parse_args()
    plan = study.check()
    if not args.snapshot:
        execution = read(study.BASE / 'execution_outcome.json')
        assert execution['status'] in ('COMPLETED', 'COMPLETED_WITH_ERRORS')
    target = study.BASE / ('receipt_snapshot_v1' if args.snapshot else 'receipt_summary_v1')
    target.mkdir(exist_ok=False)
    shutil.copy2(__file__, target / Path(__file__).name)
    manifest = read(study.OUT / 'manifest.json')
    assert manifest['identity'] == study.identity(plan, study.PROTOCOL)
    rows, receipts = [], {}
    for case in plan['cases']:
        folder = study.OUT / case['case_id']
        if args.snapshot and not (folder / 'case_result.json').is_file():
            rows.append(dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
                execution='PENDING_TERMINAL_RECEIPT', hard_safety=None))
            continue
        result = cached_result(folder, {**manifest['identity'], 'case_sha256': encode(case)})
        row = study.episode(case, folder, result)
        if row['execution'] == 'COMPLETED':
            with np.load(folder / 'replay.npz', allow_pickle=False) as z:
                endpoint = float(np.linalg.norm(z['cargo/cargo_0/centers'][-1] - z['task/cargo_0/goal_point']))
            assert np.isclose(endpoint, row['endpoint_m'], rtol=0., atol=1e-12)
        rows.append(row)
        receipts[case['case_id']] = sha(folder / 'case_result.json')
        print(dict(case=case['case_id'], completed=row['safe_completed'],
                   cutoff=row.get('budget_terminated'), endpoint_m=row.get('endpoint_m')), flush=True)
        gc.collect()
    report = dict(observed_at_utc=datetime.now(timezone.utc).isoformat(), episodes=rows,
        scheduled_physical_episodes=8, terminal_episode_receipts=len(receipts),
        protocol_sha256=sha(study.PROTOCOL), source_sha256=plan['frozen_source_sha256'],
        receipt_sha256=receipts, exporter_sha256=sha(__file__),
        independent_constraint_replay='PENDING', formal_mechanism_and_full_gates='NOT_YET_AUDITED',
        candidate_adopted=False,
        scope='All eight scheduled cases; observed terminal outcomes use existing task/G500 checks and independent endpoint geometry. Any pending case remains explicit. Full pair/bridge/support command replay remains pending.')
    dump(target / 'summary.json', report)
    summary_name = 'partial_receipt_summary.json' if args.snapshot else 'terminal_receipt_summary.json'
    csv_name = 'all_eight_scheduled_snapshot.csv' if args.snapshot else 'all_eight_receipts.csv'
    manifest_name = 'partial_receipt_publication.json' if args.snapshot else 'receipt_publication.json'
    dump(PUBLIC / summary_name, report)
    keys = ('condition', 'arm', 'execution', 'safe_completed', 'g500', 'false_hold',
            'budget_terminated', 'seconds', 'endpoint_m', 'maximum_absolute_rotation_deg', 'barrier_scalings')
    with (PUBLIC / csv_name).open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=[*keys, 'hard_safety_count', 'execution_error'])
        writer.writeheader()
        for row in sorted(rows, key=lambda r: (plan['conditions'].index(r['condition']), study.ARMS.index(r['arm']))):
            writer.writerow({**{k: row.get(k) for k in keys},
                'hard_safety_count': None if row['hard_safety'] is None else len(row['hard_safety']),
                'execution_error': row.get('error')})
    names = [summary_name, csv_name]
    if not args.snapshot:
        shutil.copy2(study.BASE / 'execution_outcome.json', PUBLIC / 'execution_outcome.json')
        names.append('execution_outcome.json')
    dump(PUBLIC / manifest_name, dict(exporter_sha256=sha(__file__),
        raw_summary_sha256=sha(target / 'summary.json'), products={n: sha(PUBLIC / n) for n in names},
        scope=report['scope']))


if __name__ == '__main__':
    main()
