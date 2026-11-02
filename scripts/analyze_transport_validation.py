"""Verify and report the frozen transport-prior validation without retuning."""
from __future__ import annotations

import argparse
import collections
import copy
import gc
import json
from pathlib import Path

import validate_transport_prior as study
import numpy as np
from analyze_cargo_discovery import one_case
from analyze_feedback_components import csv_rows
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes, verify_source_snapshot


def aggregate(rows):
    good = [r for r in rows if r['execution_status'] == 'COMPLETED']
    seconds = sum(r.get('seconds', 0) for r in good)
    scalings = sum(r.get('barrier_scalings', 0) for r in good)
    return dict(cases=len(rows), program_completed=len(good),
        safe_completed=sum(r['safe_completed'] for r in rows),
        completed=sum(r['completed'] for r in rows), g500=sum(r['g500'] for r in rows),
        hard_safety_cases=sum(bool(r.get('hard_safety', 'EXECUTION_ERROR')) for r in rows),
        errors=len(rows)-len(good), barrier_scalings=scalings,
        physical_seconds=seconds, scalings_per_1000_robot_steps=scalings/(seconds/.05*16)*1000 if seconds else None,
        mean_endpoint_error_m=float(np.mean([r['endpoint_error_m'] for r in good])) if good else None,
        median_endpoint_error_m=float(np.median([r['endpoint_error_m'] for r in good])) if good else None,
        detected=sum(r.get('first_detection_seconds') is not None for r in rows),
        reached_ready=sum(r.get('first_contact_ready_seconds') is not None for r in rows),
        actual_contact=sum(r.get('first_contact_seconds') is not None for r in rows),
        outcomes=dict(collections.Counter(r.get('outcome', 'EXECUTION_ERROR') for r in rows)))


def paired(rows, left_arm='reference', right_arm='candidate', left_method=None, right_method=None):
    pairs = []
    for condition in dict.fromkeys(r['condition'] for r in rows):
        left = [r for r in rows if r['condition'] == condition and r['arm'] == left_arm
                and (left_method is None or r['method'] == left_method)]
        right = [r for r in rows if r['condition'] == condition and r['arm'] == right_arm
                 and (right_method is None or r['method'] == right_method)]
        assert len(left) == len(right) == 1
        a, b = left[0], right[0]
        pairs.append(dict(condition=condition,
            left_safe=a['safe_completed'], right_safe=b['safe_completed'],
            left_g500=a['g500'], right_g500=b['g500'],
            left_hard=a.get('hard_safety', 'EXECUTION_ERROR'), right_hard=b.get('hard_safety', 'EXECUTION_ERROR'),
            endpoint_right_minus_left_m=(b['endpoint_error_m']-a['endpoint_error_m'])
                if 'endpoint_error_m' in a and 'endpoint_error_m' in b else None,
            ready_gap_right_minus_left_m=(b['ready_gap_upper_m']-a['ready_gap_upper_m'])
                if a.get('ready_gap_upper_m') is not None and b.get('ready_gap_upper_m') is not None else None))
    wins = [p['condition'] for p in pairs if p['right_safe'] and not p['left_safe']]
    losses = [p['condition'] for p in pairs if p['left_safe'] and not p['right_safe']]
    return dict(pairs=pairs, right_wins=wins, right_losses=losses, right_net=len(wins)-len(losses),
                new_hard=[p['condition'] for p in pairs if p['right_hard'] and not p['left_hard']],
                g500_gained=[p['condition'] for p in pairs if p['right_g500'] and not p['left_g500']],
                g500_lost=[p['condition'] for p in pairs if p['left_g500'] and not p['right_g500']])


def analyze(stage):
    study.guard()
    receipt = study.read(study.AUDIT / 'freeze_receipt.json')[stage]
    path = study.ROOT / receipt['path']
    assert study.sha(path) == receipt['sha256']
    plan = study.read(path)
    batch = study.ROOT / plan['run_dir']
    manifest = study.read(batch / 'manifest.json')
    summary = study.read(batch / 'summary.json')
    assert summary['source_integrity'] == 'PASS'
    assert summary['completed_denominator'] == len(plan['cases'])
    verify_source_snapshot(batch, manifest['source_files'])
    assert source_hashes() == manifest['source_files']
    assert encode(manifest['source_files']) == plan['frozen_source_sha256']
    destination = study.ANALYSIS / stage
    if (destination / 'analysis.json').exists():
        raise RuntimeError('Existing final analysis; preserve it')
    destination.mkdir(exist_ok=True)
    cache = destination / 'case_cache'
    cache.mkdir(exist_ok=True)
    rows, inputs = [], {}
    for case in plan['cases']:
        folder = batch / case['case_id']
        print('Verify ' + case['case_id'], flush=True)
        record_path = cache / (case['case_id'] + '.json')
        input_sha = study.sha(folder / 'case_result.json')
        if record_path.exists():
            record = study.read(record_path)
            assert record['input_sha256'] == input_sha
            assert record['shared_analysis_sha256'] == study.sha(study.ROOT / 'scripts/analyze_cargo_discovery.py')
            raw = study.read(folder / 'case_result.json')
            cached_result(folder, raw['identity'])
            row = record['row']
        else:
            row, series, _ = one_case(case, folder, manifest)
            del series
            dump(record_path, dict(row=row, input_sha256=input_sha,
                shared_analysis_sha256=study.sha(study.ROOT / 'scripts/analyze_cargo_discovery.py')))
        row = copy.deepcopy(row)
        row.update(stratum=case['stratum'], result_origin='new_execution', source_sha256=manifest['identity']['source_sha256'])
        rows.append(row)
        inputs[(folder / 'case_result.json').relative_to(study.ROOT).as_posix()] = input_sha
        gc.collect()
    design = study.read(study.AUDIT / 'design.json')
    if stage == 'regression':
        for item in design['historical_comparisons']:
            folder = study.ROOT / item['folder']
            assert study.sha(folder / 'case_result.json') == item['receipt_sha256']
            assert study.sha(study.ROOT / item['analysis_file']) == item['analysis_sha256']
            cached_result(folder, item['identity'])
            row = copy.deepcopy(item['row'])
            row.update(condition='near_' + item['condition'], arm=item['setting'],
                stratum='near', result_origin='verified_historical_reuse',
                source_sha256=item['identity']['source_sha256'])
            rows.append(row)
            inputs[(folder / 'case_result.json').relative_to(study.ROOT).as_posix()] = item['receipt_sha256']
        assert len(rows) == 66
    else:
        assert len(rows) == 72
    assert len({(r['condition'], r['method'], r['arm']) for r in rows}) == len(rows)
    by_method = {method: {arm: aggregate([r for r in rows if r['method'] == method and r['arm'] == arm])
        for arm in study.SETTINGS} for method in dict.fromkeys(r['method'] for r in rows)}
    by_stratum = {stratum: {method: {arm: aggregate([r for r in rows if r['stratum'] == stratum
        and r['method'] == method and r['arm'] == arm]) for arm in study.SETTINGS} for method in by_method}
        for stratum in dict.fromkeys(r['stratum'] for r in rows)}
    parameter_pairs = {method: paired([r for r in rows if r['method'] == method]) for method in by_method}
    method_pairs = {method: paired(rows, 'candidate', 'candidate', method, 'boundary_cvt')
        for method in by_method if method != 'boundary_cvt'}
    ref, candidate = by_method['boundary_cvt']['reference'], by_method['boundary_cvt']['candidate']
    gate = (not candidate['hard_safety_cases'] and not candidate['errors'] and
            candidate['safe_completed'] >= ref['safe_completed'] + (1 if stage == 'new_starts' else 0)
            and candidate['g500'] >= ref['g500'])
    advantage = (all(p['right_net'] >= 2 for p in method_pairs.values()) and not candidate['hard_safety_cases']) if method_pairs else None
    result = dict(stage=stage, totals={arm: aggregate([r for r in rows if r['arm'] == arm]) for arm in study.SETTINGS},
        by_method=by_method, by_stratum=by_stratum, parameter_pairs=parameter_pairs, method_pairs=method_pairs,
        frozen_stage_gate=gate, method_advantage_gate=advantage, default_changed=False,
        episodes=rows, input_receipts_sha256=inputs, protocol_sha256=study.sha(path),
        source_sha256=encode(manifest['source_files']), analysis_script_sha256=study.sha(__file__),
        shared_analysis_sha256=study.sha(study.ROOT / 'scripts/analyze_cargo_discovery.py'),
        scientific_scope=design['statistical_scope'],
        timing='Navigation at control time t[k] compared to cargo xy[k]; ready geometry at post-control frame k+1. Gap at method-specific readiness is descriptive, not a common-time causal comparison.',
        sampling='All frozen conditions and failures. No tuning or new experiments generated by this analysis.')
    dump(destination / 'analysis.json', result)
    csv_rows(destination / 'all_episodes.csv', rows)
    study.event('analysis_complete', stage=stage, gate=gate, by_method=by_method)
    print(json.dumps(dict(stage=stage, by_method=by_method, gate=gate, advantage=advantage)), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage', choices=['regression', 'new_starts'])
    analyze(p.parse_args().stage)
