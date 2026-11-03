"""Render legible endpoint labels without overwriting the first publication."""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    public = Path(__file__).resolve().parents[1] / 'docs/results/acquisition_circulation_v1'
    original = read(public / 'publication.json')
    for name, expected in original['outputs'].items():
        assert sha(public / name) == expected
    outputs = ('paired_outcomes_v2.png', 'paired_outcomes_v2.pdf', 'figure_revision_v2.json')
    if any((public / name).exists() for name in outputs):
        raise RuntimeError('Preserve existing figure revision')
    with (public / 'all_twelve_episodes.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12 and all(row['execution'] == 'COMPLETED' for row in rows)
    conditions = list(dict.fromkeys(row['condition'] for row in rows))
    arms = ('translation_all_captured', 'reference_map', 'acquisition_circulation')
    data = {(row['condition'], row['arm']): row for row in rows}
    assert len(conditions) == 4 and len(data) == 12
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8))
    fig.subplots_adjust(left=.065, right=.99, bottom=.26, top=.83, wspace=.34)
    maxima = [120., 0., 0.]
    keys = ('endpoint_m', 'acquisition_scaled_robot_steps', 'post_acquisition_scaled_robot_steps')
    for a, (arm, label, color) in enumerate(zip(arms, ('Translation baseline', 'Reference map', 'Active acquisition'), ('#777777', '#0072b2', '#218463'))):
        for i, (ax, key) in enumerate(zip(axes, keys)):
            selected = [data[condition, arm] for condition in conditions]
            values = [float(row[key]) * (1000 if i == 0 else 1) for row in selected]
            maxima[i] = max(maxima[i], max(values))
            bars = ax.bar(np.arange(4) + (a-1)*.24, values, .24, color=color, label=label)
            for bar, row in zip(bars, selected):
                if row['safe_completed'] == 'False':
                    bar.set_hatch('///')
                    bar.set_edgecolor('#8b0000')
            ax.bar_label(bars, fmt='%.1f' if i == 0 else '%g', fontsize=7,
                         padding=4, rotation=90 if i == 0 else 0)
    axes[0].axhline(120, color='#b2182b', linestyle='--', linewidth=1, label='Task tolerance: 120 mm')
    for ax, upper, label in zip(axes, maxima, ('Final true goal error (mm)', 'Acquisition scaled robot-steps', 'Post-acquisition scaled robot-steps')):
        ax.set_ylabel(label)
        ax.set_ylim(0, max(1., upper*1.3))
        ax.set_xticks(np.arange(4), ['Near circle', 'Near L', 'Far C', 'Far skew'])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncols=4, fontsize=8, frameon=False, bbox_to_anchor=(.5, .08))
    fig.text(.5, .025, 'Twelve fresh episodes on four reused conditions. Hatched bar: failed safe completion. Numerical parameters unchanged.', ha='center', fontsize=8)
    fig.suptitle('Active acquisition: complete three-arm development outcomes', fontsize=12)
    for name in outputs[:2]:
        fig.savefig(public / name, dpi=180)
    plt.close(fig)
    dump(public / outputs[2], dict(change='Layout-only revision: separate adjacent endpoint labels; original figure and receipt retained',
        script_sha256=sha(__file__), csv_sha256=sha(public / 'all_twelve_episodes.csv'),
        original_publication_sha256=sha(public / 'publication.json'),
        outputs={name: sha(public / name) for name in outputs[:2]}, new_physical_episodes=0))
    print('Figure v2 generated; original publication unchanged.', flush=True)


if __name__ == '__main__':
    main()
