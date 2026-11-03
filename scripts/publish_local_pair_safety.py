"""Publish all eight local-pair results, including the failed mechanism gates."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

from research_io import ROOT, guard, read, sha
from local_pair_study import BASE, CHECKS, OUT, PROTOCOL, ARMS, check_freeze
from dbact_sim.experiments.deployment import dump

PUBLIC = ROOT/'docs/results/local_pair_safety_v1'


def csv_file(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def main(destination=PUBLIC):
    guard()
    destination = Path(destination).resolve()
    assert destination == PUBLIC or destination.is_relative_to(ROOT/'runs')
    assert not destination.exists()
    plan = check_freeze()
    outcome = read(BASE/'analysis_outcome.json')
    assert outcome['status'] == 'VERIFIED' and outcome['denominator'] == 8
    assert sha(BASE/'analysis.json') == outcome['analysis_sha256']
    report = read(BASE/'analysis.json')
    assert report['protocol_sha256'] == sha(PROTOCOL)
    assert len(report['episodes']) == 8 and len(report['original_reproduction']) == 4
    destination.mkdir(parents=True)
    shutil.copy2(BASE/'analysis.json', destination/'analysis.json')
    dump(destination/'original_reproduction.json', report['original_reproduction'])
    rows, pairs, excluded = [], [], []
    ordering = {name:i for i, name in enumerate(plan['conditions'])}
    ordered = sorted(report['episodes'], key=lambda r:(ordering[r['condition']], ARMS.index(r['arm'])))
    for row in ordered:
        assert row['execution'] == 'COMPLETED'
        audit = row['pair_audit']
        rows.append(dict(condition=row['condition'], arm=row['arm'], safe_completed=row['safe_completed'],
            false_hold=row['false_hold'], hard_safety_count=len(row['hard_safety']),
            budget_terminated=row['budget_terminated'], g500=row['g500'],
            g500_failure_reasons=' | '.join(row['g500_failure_reasons']),
            formal_transport=row['phase_response']['transport_frame'] is not None,
            seconds=row['seconds'], endpoint_mm=1000*row['endpoint_m'],
            max_true_rotation_deg=row['maximum_absolute_rotation_deg'],
            acquisition_scaled_robot_steps=row['scaling']['acquisition'],
            post_acquisition_scaled_robot_steps=row['scaling']['post_acquisition'],
            minimum_scale=row['scaling']['min_scale'], accepted_pair_steps=len(audit['events']),
            accepted_robot_steps=audit['counts'].get('paired_robot_steps', 0),
            eligible_independent_infeasible_proposals=audit['counts'].get('independent_infeasible_proposals', 0),
            peer_trials=audit['counts'].get('peer_trials', 0), execution_audit_verified=audit['verified']))
        for event in audit['events']:
            pairs.append(dict(condition=row['condition'], saved_frame=event['frame']+1,
                control_frame_zero_based=event['frame'], control_time_s=event['timestamp'],
                member_1=event['members'][0], member_2=event['members'][1],
                speed_1_m_s=event['speeds'][0], speed_2_m_s=event['speeds'][1],
                minimum_row_residual=event['minimum_residual'],
                minimum_hold_separation_m=event['minimum_hold_separation'], proposal_id=event['proposal_id']))
        if row['arm'] == ARMS[1] and row['scaling']['events']:
            steps = read(OUT/row['case_id']/'paired_metrics.json')['steps']
            for event in row['scaling']['events']:
                step = steps[event['frame']-1]
                for aid in event['agents']:
                    diag = next(d for d in step['diagnostics'] if d['agent_id'] == aid)
                    excluded.append(dict(condition=row['condition'], saved_frame=event['frame'],
                        agent_id=aid, phase=step['phase'], reference_status=step['initial_reference']['status'],
                        solver_status=diag['solver_status'], pair_admission=diag['pair_safety'],
                        finite_support=diag['acquisition_support']))
    assert len(rows) == 8 and len(pairs) == 7 and len(excluded) == 2
    csv_file(destination/'all_eight.csv', rows)
    csv_file(destination/'accepted_pairs.csv', pairs)
    dump(destination/'residual_events.json', excluded)
    dump(destination/'gates.json', dict(mechanism=report['mechanism_gates'], full_chain=report['full_chain_gates'],
        mechanism_verdict=report['mechanism_gate'], full_chain_verdict=report['full_chain_gate'], candidate_adopted=False))

    far = ['far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0']
    lookup = {(r['condition'], r['arm']):r for r in rows}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    colors = ('#777777', '#0072B2')
    x, width = np.arange(2), .32
    specs = [('acquisition_scaled_robot_steps', 'Executed scaled robot steps', 'Count'),
        ('max_true_rotation_deg', 'Maximum actual rotation', 'Degrees'),
        ('endpoint_mm', 'Final task error', 'mm'), ('seconds', 'Completion time', 'Seconds')]
    for ax, (key, title, unit) in zip(axes.flat, specs):
        largest = 0.
        for arm_index, arm in enumerate(ARMS):
            values = np.array([lookup[(c, arm)][key] for c in far])
            locations = x+(arm_index-.5)*width
            ax.bar(locations, values, width, color=colors[arm_index])
            totals = values.copy()
            if key == 'acquisition_scaled_robot_steps':
                after = np.array([lookup[(c, arm)]['post_acquisition_scaled_robot_steps'] for c in far])
                ax.bar(locations, after, width, bottom=values, color='#E69F00', hatch='///')
                totals += after
            largest = max(largest, float(totals.max()))
            for location, value in zip(locations, totals):
                ax.annotate(f'{value:.0f}' if key == 'acquisition_scaled_robot_steps' else f'{value:.3f}',
                    (location, value), xytext=(0, 5), textcoords='offset points', ha='center', fontsize=10)
        ax.set(xticks=x, xticklabels=['Far C', 'Far skew box'], title=title, ylabel=unit, ylim=(0., largest*1.23))
        if key == 'acquisition_scaled_robot_steps': ax.set_yticks([0, 1, 2])
        ax.set_axisbelow(True); ax.grid(axis='y', alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    fig.suptitle('Local pair acquisition: fixed parameters, mixed physical results', fontsize=16, y=.99)
    fig.legend(handles=[Patch(color=colors[0], label='Finite-support control'),
        Patch(color=colors[1], label='Local-pair candidate'),
        Patch(facecolor='#E69F00', hatch='///', label='Post-acquisition scaling')],
        loc='upper center', bbox_to_anchor=(.5, .955), ncol=3, frameon=False)
    fig.text(.5, .035, '8 new episodes: 4 reused development conditions x 2 arms. Near circle/L outcomes unchanged.\n'
        'Both arms: safe completion 4/4; G500 2/4. All 4 original controls reproduced exactly.\n'
        '7 accepted pair steps verified; mechanism and full-chain gates FAIL. Candidate remains default-off.',
        ha='center', fontsize=10)
    fig.subplots_adjust(top=.86, bottom=.16, hspace=.34, wspace=.25)
    fig.savefig(destination/'comparison.png', dpi=170)
    fig.savefig(destination/'comparison.pdf')
    plt.close(fig)
    dump(destination/'engineering_checks.json', dict(tests=plan['tests'],
        runtime_core_scope='1033 passed / 2 skipped before seven new study/audit tests',
        separate_study_tests=7, new_physical_episodes=8, exact_original_controls=4,
        accepted_pair_steps=7, accepted_robot_steps=14, all_execution_audits_verified=True,
        runtime_source_sha256=plan['frozen_source_sha256'], canonical_sha256=plan['canonical_sha256'],
        environment=plan['environment'], installed_locked_packages=plan['installed_locked_packages']))
    outputs = {p.name:sha(p) for p in destination.iterdir() if p.is_file()}
    dump(destination/'publication.json', dict(outputs=outputs, publisher_sha256=sha(__file__),
        analysis_sha256=sha(BASE/'analysis.json'), protocol_sha256=sha(PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], new_physical_episodes=8, candidate_adopted=False))
    print(f'Published {len(outputs)} products; complete denominator8, accepted pairs7; both gatesFAIL', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=PUBLIC)
    main(parser.parse_args().out)
