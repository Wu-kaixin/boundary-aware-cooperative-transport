"""Publish corrected physical outcomes and separate exact saved-state audit."""
from __future__ import annotations

import csv
import gzip
import json
import shutil
import xml.etree.ElementTree as ET

import finite_acquisition_study_v2 as study
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

PUBLIC = study.ROOT/'docs/results/finite_acquisition_support_v2'
OLD_PUBLIC = study.ROOT/'docs/results/finite_acquisition_support_v1'


def write_csv(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def audit_products(plan):
    result, witnesses, violations = {}, [], []
    prior_plan = read(study.OLD)
    for alias in ('preflight-c', 'preflight-l', 'circle', 'l', 'c', 'skew'):
        folder = study.AUDIT/alias
        outcome, summary = read(folder/'outcome.json'), read(folder/'summary.json')
        assert sha(folder/'outcome.json') == plan['saved_state_audit'][alias]['outcome_sha256']
        result[alias] = dict(identity=outcome, summary=summary,
            raw_directory=folder.relative_to(study.ROOT).as_posix())
        if alias.startswith('preflight-'):
            continue
        for witness in read(folder/'witnesses.json')['witnesses']:
            witnesses.append(dict(case_id=outcome['case_id'], **witness))
        selected = []
        with gzip.open(folder/'geometry_calls.jsonl.gz', 'rt', encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if any(s['old_command_minimum_row_residual'] < -1e-7 for s in row['solves']):
                    selected.append(row)
        if selected:
            original_folder = study.ROOT/prior_plan['run_dir']/outcome['case_id']
            steps = read(original_folder/'paired_metrics.json')['steps']
            for row in selected:
                original = next(d for d in steps[row['frame']-1]['diagnostics'] if d['agent_id'] == row['agent_id'])
                violations.append(dict(case_id=outcome['case_id'], original_solver_status=original['solver_status'], **row))
    assert sum(v['identity']['verified_frames'] for k, v in result.items() if not k.startswith('preflight-')) == 874
    assert len(witnesses) == 7 and sum(len(w['corrected_counterfactual']) for w in witnesses) == 14
    dump(PUBLIC/'saved_state_audit.json', dict(cases=result, original_command_violations=violations,
        scope='874 exact historical control frames plus40 separate preflight frames; zero new physical episodes; old relaxed/scaled commands are not automatically new omission violations'))
    dump(PUBLIC/'audit_witnesses.json', dict(witnesses=witnesses, independent_comparisons=14, new_physical_episodes=0))


def main():
    guard()
    plan = study.check_freeze()
    analysis, outcome = read(study.BASE/'analysis.json'), read(study.BASE/'execution_outcome.json')
    assert outcome['status'] == 'COMPLETED' and outcome['completed_denominator'] == 8 and outcome['execution_errors'] == 0
    assert analysis['source_sha256'] == plan['frozen_source_sha256']
    assert analysis['protocol_sha256'] == sha(study.PROTOCOL) == outcome['protocol_sha256']
    gates, full = study.evaluate_gates(analysis['episodes'], analysis['original_reproduction'], plan['conditions'])
    assert gates == analysis['mechanism_gates'] and full == analysis['full_chain_gates']
    assert analysis['mechanism_gate'] == ('PASS' if all(gates.values()) else 'FAIL')
    assert analysis['full_chain_gate'] == ('PASS' if all(full.values()) else 'FAIL')
    assert analysis['candidate_adopted'] is False
    prior_publication = read(OLD_PUBLIC/'publication.json')
    for name, digest in prior_publication['outputs'].items():
        assert sha(OLD_PUBLIC/name) == digest
    outputs = ('analysis.json', 'all_eight_episodes.csv', 'saved_state_audit.json', 'audit_witnesses.json',
        'historical_candidate_comparison.csv', 'historical_candidate_reproduction.json',
        'prior_v1_all_eight_usage_corrected.csv',
        'paired_outcomes.png', 'paired_outcomes.pdf', 'engineering_checks.json', 'publication.json')
    if any((PUBLIC/name).exists() for name in outputs):
        raise RuntimeError('Preserve an existing publication')
    PUBLIC.mkdir(parents=True, exist_ok=True)
    manifest = read(study.OUT/'manifest.json')
    data, records = {}, []
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = next(r for r in analysis['episodes'] if r['case_id'] == case['case_id'])
        assert row['receipt_sha256'] == sha(folder/'case_result.json')
        data[(case['condition_id'], case['arm'])] = row
    for condition in plan['conditions']:
        for arm in study.ARMS:
            r = data[(condition, arm)]
            support = r['support_audit']['counts']
            records.append(dict(condition=condition, arm=arm, execution=r['execution'],
                safe_completed=r['safe_completed'], false_hold=r['false_hold'], hard_failures='; '.join(r['hard_safety']),
                g500=r['g500'], endpoint_m=r['endpoint_m'], simulated_seconds=r['seconds'],
                budget_terminated=r['budget_terminated'], valid_zero_epoch=r['zero_epoch_valid'],
                acquisition_scaled_robot_steps=r['scaling']['acquisition'],
                post_acquisition_scaled_robot_steps=r['scaling']['post_acquisition'],
                formal_transport=r['phase_response'].get('transport_frame') is not None,
                maximum_absolute_rotation_deg=r['maximum_absolute_rotation_deg'],
                applied_support_robot_steps=support.get('applied_robot_steps', 0),
                original_path_robot_steps=support['original_row_robot_steps'],
                point_guard_rows=support.get('point_guard_rows', 0), reach_cap_rows=support.get('reach_cap_rows', 0),
                g500_failure_reasons='; '.join(r['g500_failure_reasons'])))
    shutil.copy2(study.BASE/'analysis.json', PUBLIC/'analysis.json')
    write_csv(PUBLIC/'all_eight_episodes.csv', records)
    audit_products(plan)

    old_analysis = read(OLD_PUBLIC/'analysis.json')
    prior_plan = read(study.OLD)
    old_manifest = read(study.ROOT/prior_plan['run_dir']/'manifest.json')
    comparisons = []
    candidate_reproductions = {}
    for condition in plan['conditions']:
        old = next(r for r in old_analysis['episodes'] if r['condition'] == condition and r['arm'] == study.ARMS[1])
        new = data[(condition, study.ARMS[1])]
        old_case = next(c for c in prior_plan['cases'] if c['case_id'] == old['case_id'])
        new_case = next(c for c in plan['cases'] if c['case_id'] == new['case_id'])
        old_folder = study.ROOT/prior_plan['run_dir']/old['case_id']
        new_folder = study.OUT/new['case_id']
        old_result = cached_result(old_folder, {**old_manifest['identity'], 'case_sha256':encode(old_case)})
        new_result = cached_result(new_folder, {**manifest['identity'], 'case_sha256':encode(new_case)})
        candidate_reproductions[condition] = study.reproduction(old_folder, new_folder, old_result, new_result)
        comparisons.append(dict(condition=condition,
            old_source_sha256=old_analysis['source_sha256'], new_source_sha256=analysis['source_sha256'],
            old_safe_completed=old['safe_completed'], new_safe_completed=new['safe_completed'],
            old_endpoint_m=old['endpoint_m'], new_endpoint_m=new['endpoint_m'],
            old_seconds=old['seconds'], new_seconds=new['seconds'],
            old_scaled_robot_steps=old['barrier_scalings'], new_scaled_robot_steps=new['barrier_scalings'],
            old_rotation_deg=old['maximum_absolute_rotation_deg'], new_rotation_deg=new['maximum_absolute_rotation_deg'],
            old_g500=old['g500'], new_g500=new['g500']))
    write_csv(PUBLIC/'historical_candidate_comparison.csv', comparisons)
    dump(PUBLIC/'historical_candidate_reproduction.json', dict(comparisons=candidate_reproductions,
        old_source_sha256=old_analysis['source_sha256'], new_source_sha256=analysis['source_sha256'],
        scope='Comparison against four historical candidate episodes; no extra physical episodes; diagnostic records may differ even if physical replay arrays agree'))
    with (OLD_PUBLIC/'all_eight_episodes.csv').open(encoding='utf-8', newline='') as stream:
        prior_rows = list(csv.DictReader(stream))
    for r in prior_rows:
        if r['arm'] == study.ARMS[0]:
            case = next(c for c in prior_plan['cases'] if c['condition_id'] == r['condition'] and c['arm'] == r['arm'])
            folder = study.ROOT/prior_plan['run_dir']/case['case_id']
            receipt = cached_result(folder, {**old_manifest['identity'], 'case_sha256':encode(case)})
            counts = study.support_audit(folder, case)['counts']
            assert counts['original_row_robot_steps'] == receipt['termination']['frames_run']*case['config']['agents']['count']
            r['original_row_robot_steps'] = counts['original_row_robot_steps']
    write_csv(PUBLIC/'prior_v1_all_eight_usage_corrected.csv', prior_rows)

    fig, axes = plt.subplots(2, 3, figsize=(13.8, 8.0))
    fig.subplots_adjust(left=.065, right=.98, bottom=.16, top=.89, wspace=.30, hspace=.32)
    x, width = np.arange(4), .34
    getters = (lambda r:r['endpoint_m']*1000, lambda r:r['seconds'],
        lambda r:r['scaling']['acquisition'], lambda r:r['scaling']['post_acquisition'],
        lambda r:r['maximum_absolute_rotation_deg'], lambda r:int(r['phase_response'].get('transport_frame') is not None))
    maxima = [120., 0., 0., 0., 0., 1.]
    for a, (arm, label, color) in enumerate(zip(study.ARMS,
            ('Reference reacquisition', 'Corrected finite support'), ('#777777', '#0072b2'))):
        rows = [data[(condition, arm)] for condition in plan['conditions']]
        for i, (ax, getter) in enumerate(zip(axes.flat, getters)):
            values = [getter(r) for r in rows]
            maxima[i] = max(maxima[i], max(values))
            bars = ax.bar(x+(a-.5)*width, values, width, color=color, label=label)
            for bar, row in zip(bars, rows):
                if not row['safe_completed']:
                    bar.set_hatch('///')
                    bar.set_edgecolor('#8b0000')
            ax.bar_label(bars, fmt='%.2f' if i in (0, 1, 4) else '%g', fontsize=8, padding=3+11*a)
    axes[0, 0].axhline(120, color='#b2182b', linestyle='--', linewidth=1, label='Task tolerance: 120 mm')
    for ax, label, upper in zip(axes.flat,
            ('Final true goal error (mm)', 'Simulated duration (s)', 'Acquisition scaled robot-steps',
             'Post-acquisition scaled robot-steps', 'Maximum absolute rotation (degrees)', 'Formal TRANSPORT entry (0 / 1)'), maxima):
        ax.set_ylabel(label)
        ax.set_ylim(0, max(1., upper*1.25))
        ax.set_xticks(x, ['Near circle', 'Near L', 'Far C', 'Far skew'])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    axes[1, 2].set_yticks([0, 1], ['No', 'Yes'])
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncols=3, frameon=False, fontsize=10, bbox_to_anchor=(.5, .075))
    fig.suptitle('Corrected finite support: eight new matched physical episodes', fontsize=14)
    fig.text(.5, .035, f"Mechanism gate: {analysis['mechanism_gate']}; full-chain gate: {analysis['full_chain_gate']}. Candidate remains off. Hatch: failed safe completion.", ha='center', fontsize=10)
    fig.text(.5, .010, 'Four reused conditions, one run per arm/condition. Original parameters and gates preserved; old episodes not recounted.', ha='center', fontsize=9)
    for suffix in ('png', 'pdf'):
        fig.savefig(PUBLIC/f'paired_outcomes.{suffix}', dpi=180)
    plt.close(fig)
    stats = ET.parse(study.CHECKS/'core-final.xml').getroot().find('testsuite').attrib
    core = {k:int(stats[k]) for k in ('tests', 'failures', 'errors', 'skipped')}
    core['passed'] = core['tests']-core['failures']-core['errors']-core['skipped']
    assert core['failures'] == core['errors'] == 0
    dump(PUBLIC/'engineering_checks.json', dict(core=core, core_xml_sha256=sha(study.CHECKS/'core-final.xml'),
        runtime_source_sha256=plan['frozen_source_sha256'], canonical_sha256=plan['canonical_sha256'],
        execution=outcome, environment=plan['environment'], original_publication_products_unchanged=True,
        original_controls_exact=all(r['exact'] for r in analysis['original_reproduction'].values()),
        full_saved_state_replay_frames=874, separate_preflight_frames=40, independent_qp_comparisons=14,
        scope='Corrected-source eight-episode development study plus separately counted historical replay; no general safety/deployment superiority claim'))
    dump(PUBLIC/'publication.json', dict(script_sha256=sha(__file__), protocol_sha256=sha(study.PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], outputs={name:sha(PUBLIC/name) for name in outputs if name != 'publication.json'},
        original_v1_csv_sha256=sha(OLD_PUBLIC/'all_eight_episodes.csv'),
        original_v1_usage_correction='Only baseline original_row_robot_steps replaced with verified filter-path counts; original CSV untouched; includes zero-object-row steps.',
        new_physical_episodes=8, saved_state_audit_new_physical_episodes=0, candidate_adopted=False))
    print(dict(rows=len(records), mechanism=analysis['mechanism_gate'], full_chain=analysis['full_chain_gate'], core=core), flush=True)


if __name__ == '__main__':
    main()
