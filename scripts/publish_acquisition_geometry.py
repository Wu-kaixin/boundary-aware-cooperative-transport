"""Publish all outcomes of the frozen acquisition geometry experiment."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import acquisition_geometry_study as study
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main():
    guard()
    plan = study.check_freeze()
    public = study.PUBLIC
    analysis = read(public/'analysis.json')
    assert sha(public/'analysis.json')==sha(study.BASE/'analysis.json')
    assert analysis['protocol_sha256']==sha(study.PROTOCOL)
    assert len(analysis['episodes'])==8 and analysis['candidate_adopted'] is False
    outputs = ('all_eight_episodes.csv', 'paired_outcomes.png', 'paired_outcomes.pdf', 'engineering_checks.json', 'publication.json')
    if any((public/n).exists() for n in outputs):
        raise RuntimeError('Preserve existing publication')
    manifest = read(study.OUT/'manifest.json')
    data, records = {}, []
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = next(r for r in analysis['episodes'] if r['case_id']==case['case_id'])
        assert row['receipt_sha256']==sha(folder/'case_result.json')
        data[(case['condition_id'], case['arm'])] = row
    for condition in plan['conditions']:
        for arm in study.ARMS:
            row = data[(condition, arm)]
            scale, acq, first, est = row.get('scaling') or {}, row.get('acquisition') or {}, row.get('first_hold') or {}, row.get('estimation') or {}
            records.append(dict(condition=condition, arm=arm, execution=row['execution'],
                safe_completed=row['safe_completed'], false_hold=row['false_hold'], g500=row['g500'],
                hard_safety_failures=';'.join(row['hard_safety']), endpoint_m=row.get('endpoint_m'),
                termination_seconds=row.get('seconds'), budget_terminated=row.get('budget_terminated'),
                acquisition_status=acq.get('status'), epoch_seconds=acq.get('epoch'),
                valid_zero_epoch=row['zero_epoch_valid'], acquisition_scaled_robot_steps=scale.get('acquisition'),
                post_acquisition_scaled_robot_steps=scale.get('post_acquisition'),
                scaled_robot_steps=row.get('barrier_scalings'), minimum_barrier_scale=scale.get('min_scale'),
                first_hold_true_error_m=first.get('physical_error_before_m'),
                maximum_absolute_rotation_deg=row.get('maximum_absolute_rotation_deg'),
                **{'estimation_'+k:est.get(k) for k in ('post_ready_frames', 'available_frames', 'quorum_frames', 'mae_m', 'p95_m', 'max_m')},
                g500_failure_reasons='; '.join(row.get('g500_failure_reasons', []))))
    with (public/'all_eight_episodes.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.3))
    fig.subplots_adjust(left=.065, right=.99, bottom=.25, top=.84, wspace=.32)
    x, width = np.arange(4), .36
    maxima = [120., 0., 0.]
    for a, (arm, label, color) in enumerate(zip(study.ARMS, ('Translation baseline', 'Acquisition geometry'), ('#777777', '#0072b2'))):
        rows = [data[(condition, arm)] for condition in plan['conditions']]
        for i, (ax, key) in enumerate(zip(axes, ('endpoint_m', 'acquisition', 'post_acquisition'))):
            values = [(r.get(key, np.nan)*1000 if i==0 else (r.get('scaling') or {}).get(key, np.nan)) for r in rows]
            maxima[i] = max(maxima[i], max((v for v in values if np.isfinite(v)), default=0))
            bars = ax.bar(x+(a-.5)*width, values, width, color=color, label=label)
            for bar, row in zip(bars, rows):
                if not row['safe_completed']:
                    bar.set_hatch('///')
                    bar.set_edgecolor('#8b0000')
            ax.bar_label(bars, fmt='%.1f' if i==0 else '%g', fontsize=7, padding=3)
    axes[0].axhline(120, color='#b2182b', linestyle='--', linewidth=1, label='Task tolerance: 120 mm')
    axes[0].set_ylabel('Final true goal error (mm)')
    axes[1].set_ylabel('Acquisition scaled robot-steps')
    axes[2].set_ylabel('Post-acquisition scaled robot-steps')
    for ax, upper in zip(axes, maxima):
        ax.set_ylim(0, max(1., upper*1.2))
        ax.set_xticks(x, ['Near circle', 'Near L', 'Far C', 'Far skew'])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncols=3, fontsize=9, frameon=False, bbox_to_anchor=(.5, .07))
    fig.text(.5, .025, 'Eight fresh episodes on four reused conditions. Hatched bar: failed safe completion. All numerical parameters preserved.', ha='center', fontsize=8)
    fig.suptitle('Acquisition geometry: complete paired development outcomes', fontsize=12)
    for suffix in ('png', 'pdf'):
        fig.savefig(public/f'paired_outcomes.{suffix}', dpi=180)
    plt.close(fig)
    checks = study.ROOT/'runs/maintenance'/study.MISSION
    stats = ET.parse(checks/'core-tests.xml').getroot().find('testsuite').attrib
    core = {k:int(stats[k]) for k in ('tests', 'failures', 'errors', 'skipped')}
    core['passed'] = core['tests']-core['failures']-core['errors']-core['skipped']
    assert core['failures']==core['errors']==0
    dump(public/'engineering_checks.json', dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime_source_sha256=plan['frozen_source_sha256'], core=core, core_seconds=float(stats['time']),
        core_xml_sha256=sha(checks/'core-tests.xml'), local_checks=checks.relative_to(study.ROOT).as_posix(),
        focused_passed=37, preflight_exact_old_evaluation_rows=4,
        canonical_sha256=plan['canonical_sha256'], execution=read(study.BASE/'execution_outcome.json'),
        baseline_reproductions_exact=all(r['exact'] for r in analysis['baseline_reproduction'].values()),
        scope='Engineering checks plus eight new physical episodes; not general reliability or locked-environment physical reproduction'))
    dump(public/'publication.json', dict(script_sha256=sha(__file__), analysis_sha256=sha(public/'analysis.json'),
        protocol_sha256=sha(study.PROTOCOL), outputs={n:sha(public/n) for n in outputs if n!='publication.json'},
        new_physical_episodes=8, candidate_adopted=False))
    print(dict(rows=len(records), advance_gate=analysis['advance_gate'], candidate_adopted=False), flush=True)


if __name__=='__main__':
    main()
