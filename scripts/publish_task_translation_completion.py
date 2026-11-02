"""Publish all four completed replay conditions with their distinct origins."""
from __future__ import annotations

import csv
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def finite(value):
    return float(value) if np.isfinite(value) else None


def main():
    guard()
    protocol = ROOT/'experiments/ccta2027/task_translation_completion_v1.json'
    plan = read(protocol)
    raw = ROOT/'runs/research/2026-11-03-task-translation-finalization-v1'
    public = ROOT/'docs/results/task_translation_completion_v1'
    result = read(raw/'analysis.json')
    prior = read(ROOT/'experiments/ccta2027/task_translation_v2.json')
    assert sha(raw/'analysis.json') == sha(public/'analysis.json')
    assert sha(protocol) == sha(public/'protocol.json') == result['protocol_sha256']
    assert result['source_integrity'] == 'PASS' and result['new_physical_episodes'] == 0
    assert [r['case_id'] for r in result['cases']] == [c['case_id'] for c in plan['cases']]
    verify_source_snapshot(ROOT/plan['run_dir'], {**plan['source_files'], **plan['scripts']})
    names = ['all_arms.csv', 'last_control_sample.json', 'comparison.png', 'comparison.pdf',
             'v2_budget_outcome.json', 'publication.json']
    assert all(not (public/n).exists() for n in names), 'Preserve existing publication'
    rows, endpoints = [], []
    for case in result['cases']:
        origin = result['origins'][case['case_id']]
        folder = ROOT/origin['path']
        assert all(sha(folder/n) == h for n, h in origin['products'].items())
        assert read(folder/'summary.json') == case
        old = ROOT/prior['original_run_dir']/case['case_id']
        assert sha(old/'case_result.json') == prior['receipts'][case['case_id']]
        # The study has validated all original raw products. Recheck the original
        # replay through its independent receipt before deriving endpoint context.
        old_case = next(c for c in prior['cases'] if c['case_id'] == case['case_id'])
        cached_result(old, {**prior['original_identity'], 'case_sha256': encode(old_case)})
        with np.load(old/'replay.npz', allow_pickle=False) as z:
            target = z['task/cargo_0/distance']*z['task/cargo_0/direction']
        with np.load(folder/'series.npz', allow_pickle=False) as z:
            endpoints.append(dict(condition=case['condition'], time_s=float(z['times'][-1]),
                true_goal_error_m=float(np.linalg.norm(target-z['truth'][-1])),
                arms={arm: dict(goal_error_m=finite(np.linalg.norm(target-z['estimates'][a, -1])),
                    model_bound_m=finite(z['bounds'][a, -1]), fresh_voters=int(z['voters'][a, -1]))
                    for a, arm in enumerate(plan['arms'])}))
        for arm in plan['arms']:
            r = case['arms'][arm]
            rows.append(dict(condition=case['condition'], arm=arm, output_origin=origin['kind'],
                **{k: r[k] for k in ('initialized', 'post_ready_frames', 'available_frames',
                    'missing_team_frames', 'quorum_frames', 'insufficient_quorum_frames',
                    'paired_availability', 'potential_stop_frames', 'false_goal_stop_frames',
                    'longest_valid_potential_stop_streak')},
                **{'paired_'+k: v for k, v in r['paired_error'].items()}))
    with (public/'all_arms.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    dump(public/'last_control_sample.json', dict(cases=endpoints,
        scope='Last pre-physics control samples of old trajectories, not new physical endpoints. Translation bounds assume translation.'))

    labels = ['Near circle', 'Near L', 'Far C', 'Far skew box']
    arm_labels = ['SE(2), original supporters', 'Translation, original supporters',
                  'SE(2), all captured', 'Translation, all captured (preselected)']
    colors = ['#4d4d4d', '#0072b2', '#cc79a7', '#009e73']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.subplots_adjust(bottom=.24, top=.88, wspace=.28)
    x, width = np.arange(4), .19
    for a, (arm, label, color) in enumerate(zip(plan['arms'], arm_labels, colors)):
        values = [case['arms'][arm] for case in result['cases']]
        axes[0].bar(x+(a-1.5)*width,
            [np.nan if r['paired_error']['mae_m'] is None else r['paired_error']['mae_m']*1000 for r in values],
            width=width, label=label, color=color)
        axes[1].bar(x+(a-1.5)*width, [100*r['quorum_frames']/r['post_ready_frames'] for r in values],
                    width=width, color=color)
    axes[0].set_ylabel('Paired-frame displacement MAE (mm)')
    axes[1].set_ylabel('Post-ready frames with at least 4 fresh voters (%)')
    axes[1].set_ylim(0, 105)
    for ax in axes:
        ax.set_xticks(x, labels)
        ax.set_axisbelow(True)
        ax.grid(axis='y', alpha=.25)
        ax.spines[['top', 'right']].set_visible(False)
    handles, legend_labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, legend_labels, loc='lower center', ncols=2, fontsize=8, frameon=False)
    fig.suptitle('Four old input trajectories; zero new closed-loop episodes', fontsize=11)
    for suffix in ('png', 'pdf'):
        fig.savefig(public/f'comparison.{suffix}', dpi=170)
    plt.close(fig)
    prior_outcome = ROOT/prior['run_dir']/'execution_outcome.json'
    assert sha(prior_outcome) == plan['prior_outcome_sha256']
    shutil.copy2(prior_outcome, public/'v2_budget_outcome.json')
    dump(public/'publication.json', dict(script_sha256=sha(__file__),
        analysis_sha256=sha(raw/'analysis.json'), protocol_sha256=sha(protocol),
        outputs={n: sha(public/n) for n in names if n != 'publication.json'},
        source_sha256=result['source_sha256'], new_physical_episodes=0, frozen_screen_modified=False))
    print(dict(rows=len(rows), screen=result['physical_trial_screen'], new_physical_episodes=0))


if __name__ == '__main__':
    main()
