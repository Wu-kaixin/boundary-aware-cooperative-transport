"""Verify archived inputs and export version-separated manuscript evidence."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read(p):
    return json.loads(p.read_text(encoding='utf-8'))


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def save(p, x):
    p.write_text(json.dumps(x, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def main(out):
    out = out.resolve()
    if not out.is_relative_to(ROOT) or out.exists():
        raise ValueError('Use a fresh output directory within this repository')
    out.mkdir(parents=True)
    inputs = {}
    paths = {
        'deployment': 'docs/results/deployment_contribution_v1/analysis.json',
        'starts': 'docs/results/transport_validation_v1/new_starts_analysis.json',
        'physical': 'docs/results/post_support_validation_checkpoint_v1/all_eight_receipts.csv',
        'systems': 'docs/results/post_support_contact_geometry_capability_v1/all_336_systems.csv',
        'verification': 'docs/results/post_support_contact_geometry_capability_v1/verification.json',
    }
    for path in paths.values():
        inputs[path] = sha(ROOT / path)
    dep, starts = [read(ROOT / paths[k]) for k in ('deployment', 'starts')]
    assert len(dep['matched_blocks']) == 12 and len(dep['episodes']) == 36
    old = ROOT / 'runs/research/2026-11-02-transport-validation-v1/new_starts'
    validated = 0
    for case_id, expected in dep['receipt_sha256'].items():
        folder = old / case_id
        assert sha(folder / 'case_result.json') == expected
        receipt = read(folder / 'case_result.json')
        for name, digest in receipt['products'].items():
            assert sha(folder / name) == digest, (case_id, name)
            validated += 1
        row = next(r for r in dep['episodes'] if r['condition'] == receipt['result']['condition_id']
                   and r['method'] == receipt['result']['variant'])
        assert row['task_completed'] == receipt['result']['task_completed']
        assert row['g500'] == receipt['result']['g500_pass']
    methods = ('boundary_cvt', 'local_spacing', 'nearest_observed_target')
    for method in methods:
        rows = [r for r in dep['episodes'] if r['method'] == method]
        assert len(rows) == 12
        assert sum(r['task_completed'] for r in rows) == dep['totals'][method]['completed']
        assert sum(r['g500'] for r in rows) == dep['totals'][method]['g500']
    blocks = sorted(dep['matched_blocks'])
    differences = {}
    for control in methods[1:]:
        differences[control] = []
        for condition in blocks:
            a = next(r for r in dep['episodes'] if r['condition'] == condition and r['method'] == 'boundary_cvt')
            b = next(r for r in dep['episodes'] if r['condition'] == condition and r['method'] == control)
            differences[control].append(a['mean_coverage'] - b['mean_coverage'])
        assert abs(np.mean(differences[control]) - dep['comparisons'][control]['mean_coverage_delta']) < 1e-12
    with (ROOT / paths['physical']).open(encoding='utf-8') as f:
        physical = list(csv.DictReader(f))
    with (ROOT / paths['systems']).open(encoding='utf-8') as f:
        systems = list(csv.DictReader(f))
    assert len(physical) == 8 and len(systems) == 336
    positive = [r for r in systems if r['positive_force'] == 'True']
    counts = {}
    for model in sorted({r['model'] for r in positive}):
        rows = [r for r in positive if r['model'] == model]
        assert len(rows) == 17
        counts[model] = {k: sum(r['classification'] == k for r in rows)
                         for k in sorted({r['classification'] for r in rows})}
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8, 'axes.titlesize': 9,
                         'pdf.fonttype': 42, 'ps.fonttype': 42, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.55), layout='constrained')
    for j, control in enumerate(methods[1:]):
        axes[0].plot(np.arange(12) + 1, np.array(differences[control]) * 100,
                     ['o', 's'][j], markersize=4, label='CVT - ' + ['spacing', 'nearest'][j])
    axes[0].axhline(0, c='.5', lw=.7)
    axes[0].set(xlabel='Matched condition ID (1-12)', ylabel='Coverage difference (percentage points)',
                xticks=np.arange(1, 13), title='(a) Common pre-readiness interval')
    axes[0].legend(fontsize=7)
    for j, method in enumerate(methods):
        vals = [next(r['endpoint_error_m'] for r in dep['episodes']
                     if r['condition'] == c and r['method'] == method) for c in blocks]
        axes[1].plot(np.arange(12) + 1 + (j-1)*.15, vals, ['o','s','^'][j], markersize=4,
                     fillstyle='none', label=['CVT','Spacing','Nearest'][j])
    axes[1].axhline(.12, c='.35', ls=':', label='Task tolerance')
    axes[1].set(xlabel='Same matched condition ID', ylabel='Final spatial error (m)',
                xticks=np.arange(1, 13), title='(b) Complete-task endpoint, 11/12 each')
    axes[1].legend(fontsize=6.5, ncol=2)
    for ext in ('png','pdf','svg'):
        fig.savefig(out / ('deployment_tradeoff.'+ext), dpi=230)
    plt.close(fig)
    conditions = list(dict.fromkeys(r['condition'] for r in physical))
    arms = list(dict.fromkeys(r['arm'] for r in physical))
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.5), layout='constrained')
    for j, arm in enumerate(arms):
        vals = [float(next(r['endpoint_m'] for r in physical if r['condition'] == c and r['arm'] == arm)) for c in conditions]
        axes[0].plot(range(4), vals, ['o','s'][j], ms=5, label=['Bridge control','Finite support'][j])
    axes[0].axhline(.12, c='.4', ls=':', label='Task tolerance')
    axes[0].set(xticks=range(4), xticklabels=['Circle','L','C (far)','Box (far)'],
                ylabel='Final spatial error (m)', title='(a) Frozen physical comparison (4 pairs)')
    axes[0].legend(fontsize=7)
    model_order = ['original_aggregate', 'finite_capped', 'finite_uncapped']
    # Names must match the recorded export; never silently drop an unknown model.
    if set(model_order) != set(counts):
        model_order = ['original_aggregate'] + sorted(set(counts) - {'original_aggregate'})
    labels = ['Original aggregate'] + [m.replace('_',' ') for m in model_order[1:]]
    classes = ['POSITIVE_CAPABILITY','MUST_REVERSE','INFEASIBLE']
    assert set().union(*(set(c) for c in counts.values())) <= set(classes), counts
    left = np.zeros(3)
    for cls, color, hatch, label in zip(classes, ['#0072B2','#D55E00','#888888'], ['', '//','xx'],
                                      ['Positive feasible','Forced reverse','Infeasible']):
        vals = np.array([counts[m].get(cls,0) for m in model_order])
        axes[1].barh(range(3), vals, left=left, color=color, hatch=hatch, label=label)
        for y, (v,l) in enumerate(zip(vals,left)):
            if v: axes[1].text(l+v/2, y, str(v), ha='center',va='center',color='white',fontsize=8)
        left += vals
    assert np.array_equal(left, [17,17,17])
    axes[1].set(yticks=range(3), yticklabels=labels, xticks=[0,5,10,17],
                xlabel='Saved positive-force states (17 total)', title='(b) Instantaneous velocity capability')
    axes[1].invert_yaxis()
    handles, labels = axes[1].get_legend_handles_labels()
    fig.set_layout_engine(None)
    fig.subplots_adjust(left=.08, right=.99, bottom=.29, top=.86, wspace=.64)
    fig.legend(handles, labels, fontsize=6.5, loc='lower center',
               bbox_to_anchor=(.72,.005), ncol=3)
    for ext in ('png','pdf','svg'):
        fig.savefig(out / ('contact_incompatibility.'+ext), dpi=230)
    plt.close(fig)
    result = dict(input_sha256=inputs, verified_deployment_raw_products=validated,
        condition_order=blocks, deployment_comparisons=dep['comparisons'], deployment_totals=dep['totals'],
        starts_by_method=starts['by_method'], physical_receipts=physical, positive_force_system_counts=counts,
        scientific_units='12 designed scene blocks; 4 reused physical pairs; 17 dependent saved states; do not pool',
        source_script_sha256=sha(Path(__file__)), artifact_sha256={p.name:sha(p) for p in out.iterdir() if p.is_file()},
        verification_scope='Raw deployment products hashed; physical/capability compact tables cross-counted. Existing detailed witnesses separately retained.')
    save(out / 'evidence.json', result)
    print(json.dumps(dict(output=str(out), verified_products=validated, counts=counts)), flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    main(ap.parse_args().out)
