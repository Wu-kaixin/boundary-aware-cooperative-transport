"""Save a dated, explicitly incomplete checkpoint without touching the live run."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import shutil
import xml.etree.ElementTree as ET

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import environment_versions, source_hashes, verify_source_snapshot


def main():
    guard()
    protocol = ROOT/'experiments/ccta2027/task_translation_v2.json'
    plan = read(protocol)
    raw = ROOT/plan['run_dir']
    public = ROOT/'docs/results/task_translation_checkpoint_v1'
    assert not public.exists(), 'Preserve the published checkpoint'
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert {n: sha(ROOT/n) for n in plan['scripts']} == plan['scripts']
    assert sha(protocol) == sha(raw/'protocol.json')
    verify_source_snapshot(raw, {**plan['source_files'], **plan['scripts']})
    completed, pending, products = [], [], {}
    for case in plan['cases']:
        folder = raw/case['case_id']
        if not (folder/'summary.json').exists():
            pending.append(case['case_id'])
            continue
        row = read(folder/'summary.json')
        assert row['case_id'] == case['case_id'] and row['condition'] == case['condition_id']
        assert list(row['arms']) == plan['arms']
        assert sha(folder/'series.npz') == row['series_sha256']
        assert sha(folder/'templates.json') == row['templates_sha256']
        products[case['case_id']] = {n: sha(folder/n) for n in ('summary.json', 'series.npz', 'templates.json')}
        completed.append(row)
    assert completed and pending, 'This checkpoint is for a partially completed study only'
    captured_at = datetime.now(timezone.utc).isoformat()
    public.mkdir(parents=True, exist_ok=False)
    admission = plan['admission']
    gates = {}
    csv_rows = []
    for case in completed:
        r = case['arms'][plan['selected_candidate']]
        mae = r['paired_error']['mae_m']
        gates[case['condition']] = dict(
            availability=r['paired_availability'] >= admission['paired_availability_min'],
            error=mae is not None and mae <= admission['paired_mae_max_m']
                and mae <= r['paired_original_error']['mae_m']+admission['paired_mae_increase_max_m'],
            false_goal_stop=r['false_goal_stop_frames'] <= admission['false_goal_stop_frames_max'],
            sustained_feedback=r['longest_valid_potential_stop_streak'] >= admission['consecutive_valid_potential_stop_frames_min'])
        for arm in plan['arms']:
            r = case['arms'][arm]
            csv_rows.append(dict(condition=case['condition'], arm=arm,
                **{k: r[k] for k in ('initialized', 'post_ready_frames', 'available_frames',
                    'missing_team_frames', 'quorum_frames', 'insufficient_quorum_frames',
                    'paired_availability', 'potential_stop_frames', 'false_goal_stop_frames',
                    'longest_valid_potential_stop_streak')},
                **{'paired_'+k: v for k, v in r['paired_error'].items()}))
    dump(public/'analysis.json', dict(scope=__doc__, captured_at_utc=captured_at,
        expected_cases=len(plan['cases']), completed_cases=len(completed), pending_cases=pending,
        overall_screen='INCOMPLETE', completed_case_gates=gates,
        candidate_adopted=False, new_physical_episodes=0, cases=completed,
        protocol_sha256=sha(protocol), source_sha256=encode(plan['source_files']),
        source_integrity_at_checkpoint='PASS', scripts=plan['scripts'], raw_products=products))
    with (public/'all_completed_arms.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    shutil.copy2(protocol, public/'protocol.json')
    maintenance = ROOT/'runs/maintenance/2026-11-03-task-translation-v2'
    calibration = maintenance/'stop_screen_calibration.json'
    assert read(calibration)['study_protocol_sha256'] == sha(protocol)
    shutil.copy2(calibration, public/'stop_screen_calibration.json')
    xml = ROOT/'runs/maintenance/2026-11-03-task-translation-v1/core-tests.xml'
    suites = list(ET.parse(xml).getroot().iter('testsuite'))
    counts = {k: sum(int(s.attrib[k]) for s in suites) for k in ('tests', 'failures', 'errors', 'skipped')}
    assert counts['failures'] == counts['errors'] == 0
    counts['passed'] = counts['tests']-counts['skipped']
    dump(public/'engineering_checks.json', dict(core=counts, core_xml_sha256=sha(xml),
        core_log_sha256=sha(xml.with_suffix('.log')), runtime_source_sha256=encode(plan['source_files']),
        scope='Core regression on this runtime source before v2 replay, not physical success'))
    files = {p.name: sha(p) for p in public.iterdir() if p.is_file()}
    dump(public/'publication.json', dict(script_sha256=sha(__file__), captured_at_utc=captured_at,
        files=files, frozen_screen_modified=False, local_raw_dir=plan['run_dir']))
    print(dict(completed=len(completed), pending=pending, overall_screen='INCOMPLETE', core=counts))


if __name__ == '__main__':
    main()
