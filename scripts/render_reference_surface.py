"""Layout-reviewed figures from the complete published eight-block CSV."""
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from research_io import ROOT, guard, sha
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    folder = ROOT/'docs/results/reference_surface_v1'
    names = ['same_input_errors_reviewed.png', 'same_input_errors_reviewed.pdf', 'figure_manifest_reviewed.json']
    if any((folder/name).exists() for name in names):
        raise RuntimeError('Preserve existing figure version')
    with (folder/'all_eight_blocks.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 8
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4), layout='constrained',
                             gridspec_kw={'width_ratios': [2.5, 1.5]})
    for ax, prefix, title in zip(axes, ['near_', 'far_'], ['Six near blocks', 'Two far blocks']):
        group = [r for r in rows if r['condition'].startswith(prefix)]
        y = np.arange(len(group))
        ax.barh(y-.17, [float(r['original_MAE_mm']) for r in group], .32, color='#52687b', label='Protected baseline')
        ax.barh(y+.17, [float(r['fitted_MAE_mm']) for r in group], .32, color='#d78136', label='Fitted surface')
        ax.set_yticks(y, [r['condition'].removeprefix(prefix).replace('_pos0', '').replace('_s0_r0', '').replace('_', ' ') for r in group])
        ax.invert_yaxis()
        ax.set_xlabel('Vector displacement MAE (mm)', fontsize=10)
        ax.set_title(title)
        ax.grid(axis='x', alpha=.2)
        ax.set_axisbelow(True)
    axes[0].legend(loc='upper right', fontsize=9)
    fig.suptitle('Same-input replay: all eight reused development blocks\nTeam availability retained: 100%; candidate admission: FAIL')
    for name in names[:2]:
        fig.savefig(folder/name, dpi=180, bbox_inches='tight')
    plt.close(fig)
    dump(folder/names[2], dict(script_sha256=sha(__file__), csv_sha256=sha(folder/'all_eight_blocks.csv'),
        files={name: sha(folder/name) for name in names[:2]},
        scope='Layout revision only; all data unchanged. Initial figures and their original receipt are preserved. '
              'Separate axis scales for near/far blocks. No frame-level uncertainty or new physical outcome claim.'))


if __name__ == '__main__':
    main()
