"""Present all eight frozen outcomes; never modify the frozen evaluators."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import reference_reacquisition_study as study
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main():
    guard()
    plan = study.check_freeze()
    public = study.PUBLIC
    analysis = read(public/'analysis.json')
    assert sha(public/'analysis.json') == sha(study.BASE/'analysis.json')
    assert analysis['protocol_sha256'] == sha(study.PROTOCOL)
    assert len(analysis['episodes']) == 8 and analysis['candidate_adopted'] is False
    outputs = ('all_eight_episodes.csv', 'paired_outcomes.png', 'paired_outcomes.pdf',
               'engineering_checks.json', 'publication.json')
    if any((public/name).exists() for name in outputs):
        raise RuntimeError('Preserve existing publication')
    manifest = read(study.OUT/'manifest.json')
    data, records = {}, []
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = next(r for r in analysis['episodes'] if r['case_id']==case['case_id'])
        assert row['receipt_sha256'] == sha(folder/'case_result.json')
        data[(case['condition_id'], case['arm'])] = row
    for condition in plan['conditions']:
        for arm in study.ARMS:
            row = data[(condition, arm)]
            scale, acq = row.get('scaling') or {}, row.get('acquisition') or {}
            reference = row.get('map_reference') or {}
            per_robot = reference.get('per_robot', {})
            records.append(dict(condition=condition, arm=arm, execution=row['execution'],
                safe_completed=row['safe_completed'], false_hold=row['false_hold'], g500=row['g500'],
                hard_safety_failures=';'.join(row['hard_safety']), endpoint_m=row.get('endpoint_m'),
                simulated_seconds=row.get('seconds'), budget_terminated=row.get('budget_terminated'),
                acquisition_status=acq.get('status'), epoch_seconds=acq.get('epoch'),
                valid_zero_epoch=row['zero_epoch_valid'],
                pre_epoch_contact_samples=acq.get('pre_epoch_contact_samples'),
                max_pre_epoch_translation_m=acq.get('max_pre_epoch_translation'),
                max_pre_epoch_rotation_rad=acq.get('max_pre_epoch_rotation'),
                acquisition_scaled_robot_steps=scale.get('acquisition'),
                post_acquisition_scaled_robot_steps=scale.get('post_acquisition'),
                scaled_robot_steps=row.get('barrier_scalings'),
                minimum_barrier_scale=scale.get('min_scale'),
                final_phase=row.get('final_phase'),
                maximum_transport_active=reference.get('maximum_transport_active'),
                maximum_absolute_rotation_deg=row.get('maximum_absolute_rotation_deg'),
                unavailable_reference_robot_steps=reference.get('robot_steps_by_status', {}).get('unavailable'),
                recovery_grants=sum(r.get('recovery_grants', 0) for r in per_robot.values()),
                nonzero_recovery_command_steps=sum(r.get('nonzero_recovery_commands', 0) for r in per_robot.values()),
                agents_with_fresh_reference_after_grant=len(reference.get('first_fresh_reference_after_recovery_grant', {})),
                g500_failure_reasons='; '.join(row.get('g500_failure_reasons', []))))
    with (public/'all_eight_episodes.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.0))
    fig.subplots_adjust(left=.09, right=.98, bottom=.16, top=.90, wspace=.23, hspace=.28)
    x, width = np.arange(4), .34
    metrics = ('endpoint_m', 'seconds', 'acquisition', 'post_acquisition')
    maxima = [120., 0., 0., 0.]
    for a, (arm, label, color) in enumerate(zip(study.ARMS,
            ('Original circulation', 'Reference reacquisition'), ('#777777', '#0072b2'))):
        rows = [data[(condition, arm)] for condition in plan['conditions']]
        for i, (ax, key) in enumerate(zip(axes.flat, metrics)):
            values = [((r.get(key, np.nan)*(1000 if i==0 else 1)) if i<2
                       else (r.get('scaling') or {}).get(key, np.nan)) for r in rows]
            maxima[i] = max(maxima[i], max((v for v in values if np.isfinite(v)), default=0.))
            bars = ax.bar(x+(a-.5)*width, values, width, color=color, label=label)
            for bar, row in zip(bars, rows):
                if not row['safe_completed']:
                    bar.set_hatch('///')
                    bar.set_edgecolor('#8b0000')
            ax.bar_label(bars, fmt='%.1f' if i<2 else '%g', fontsize=8, padding=3)
    axes[0,0].axhline(120, color='#b2182b', linestyle='--', linewidth=1,
                      label='Task tolerance: 120 mm')
    for ax, label, upper in zip(axes.flat,
            ('Final true goal error (mm)', 'Simulated duration (s)',
             'Acquisition scaled robot-steps', 'Post-acquisition scaled robot-steps'), maxima):
        ax.set_ylabel(label)
        ax.set_ylim(0, max(1., upper*1.25))
        ax.set_xticks(x, ['Near circle', 'Near L', 'Far C', 'Far skew'])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncols=3, frameon=False, fontsize=9,
               bbox_to_anchor=(.5, .065))
    fig.suptitle('Reference reacquisition: complete matched development outcomes', fontsize=13)
    fig.text(.5, .028, 'Eight new episodes on four reused conditions. Hatch: failed safe completion. All numerical parameters preserved.',
             ha='center', fontsize=9)
    for suffix in ('png', 'pdf'):
        fig.savefig(public/f'paired_outcomes.{suffix}', dpi=180)
    plt.close(fig)
    stats = ET.parse(study.CHECKS/'study-core.xml').getroot().find('testsuite').attrib
    core = {k:int(stats[k]) for k in ('tests', 'failures', 'errors', 'skipped')}
    core['passed'] = core['tests']-core['failures']-core['errors']-core['skipped']
    assert core['failures']==core['errors']==0
    dump(public/'engineering_checks.json', dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime_source_sha256=plan['frozen_source_sha256'], core=core,
        core_xml_sha256=sha(study.CHECKS/'study-core.xml'), local_checks=study.CHECKS.relative_to(study.ROOT).as_posix(),
        evaluator_regression_passed=4, preflight_exact_old_evaluations=4, preflight_capture_prefix_self_checks=4,
        canonical_sha256=plan['canonical_sha256'], execution=read(study.BASE/'execution_outcome.json'),
        original_arms_exact=all(r['exact'] for r in analysis['baseline_reproduction'].values()),
        candidate_capture_prefixes_exact=all(r['exact'] for r in analysis['acquisition_prefix_reproduction'].values()),
        scope='Engineering checks plus eight new physical episodes; not general reliability or a locked-environment physical rerun'))
    dump(public/'publication.json', dict(script_sha256=sha(__file__),
        analysis_sha256=sha(public/'analysis.json'), protocol_sha256=sha(study.PROTOCOL),
        outputs={name:sha(public/name) for name in outputs if name!='publication.json'},
        new_physical_episodes=8, candidate_adopted=False))
    print(dict(rows=len(records), mechanism_gate=analysis['mechanism_gate'],
               full_chain_gate=analysis['full_chain_gate'], candidate_adopted=False), flush=True)


if __name__ == '__main__':
    main()
