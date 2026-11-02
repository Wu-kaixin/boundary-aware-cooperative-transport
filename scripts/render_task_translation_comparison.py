"""Compact figure layout; preserve the first render and its publication receipt."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    public = ROOT/'docs/results/task_translation_completion_v1'
    source = public/'analysis.json'
    result = read(source)
    plan = read(public/'protocol.json')
    names = ['comparison_v2.png', 'comparison_v2.pdf', 'figure_v2_receipt.json']
    assert len(result['cases']) == 4 and result['new_physical_episodes'] == 0
    assert all(not (public/n).exists() for n in names)
    labels = ['Near circle', 'Near L', 'Far C', 'Far skew box']
    legends = ['SE(2), original supporters', 'Translation, original supporters',
               'SE(2), all captured', 'Translation, all captured (preselected)']
    colors = ['#4d4d4d', '#0072b2', '#cc79a7', '#009e73']
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    fig.subplots_adjust(bottom=.24, top=.88, wspace=.28)
    x, width = np.arange(4), .19
    for a, (arm, label, color) in enumerate(zip(plan['arms'], legends, colors)):
        values = [case['arms'][arm] for case in result['cases']]
        axes[0].bar(x+(a-1.5)*width, [r['paired_error']['mae_m']*1000 for r in values], width, color=color, label=label)
        axes[1].bar(x+(a-1.5)*width, [100*r['quorum_frames']/r['post_ready_frames'] for r in values], width, color=color)
    axes[0].set_ylabel('Paired-frame displacement MAE (mm)')
    axes[1].set_ylabel('Fresh-voter quorum availability (%)')
    axes[1].set_ylim(0, 105)
    for ax in axes:
        ax.set_xticks(x, labels)
        ax.set_axisbelow(True)
        ax.grid(axis='y', alpha=.25)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncols=2, fontsize=8, frameon=False)
    fig.suptitle('Four old input trajectories; quorum = at least 4 fresh voters after readiness', fontsize=10)
    for suffix in ('png', 'pdf'):
        fig.savefig(public/f'comparison_v2.{suffix}', dpi=170)
    plt.close(fig)
    dump(public/'figure_v2_receipt.json', dict(script_sha256=sha(__file__), source_sha256=sha(source),
        outputs={n: sha(public/n) for n in names[:-1]}, changes='Shorter axis label only; same data, scales and colors. First render preserved.',
        new_physical_episodes=0))


if __name__ == '__main__':
    main()
