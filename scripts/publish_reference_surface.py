"""Verify the protected baseline replay and publish the complete fixed comparison."""
from __future__ import annotations

import csv
from dataclasses import asdict
import json
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from analyze_dynamic_contact_study import old_fields_equal, replay_equal
from dbact.contracts import ClosedLoopContract
from dbact.provenance import config_hash
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot

OUT = ROOT/'runs/research/2026-11-03-reference-surface-v1'
PUBLIC = ROOT/'docs/results/reference_surface_v1'
PROTOCOL = ROOT/'experiments/ccta2027/reference_surface_v1.json'
BASELINE = ROOT/'experiments/ccta2027/protected_baseline_v1_reproduction.json'


def verified_case(path, case_id):
    plan = read(path)
    case = next(c for c in plan['cases'] if c['case_id'] == case_id)
    batch = ROOT/plan['run_dir']
    manifest = read(batch/'manifest.json')
    identity = dict(schema='ccta-study-v2', protocol_sha256=sha(path), split=case['split'],
                    source_sha256=plan['frozen_source_sha256'], environment=plan['environment'])
    assert identity == manifest['identity']
    assert encode(manifest['source_files']) == identity['source_sha256']
    verify_source_snapshot(batch, manifest['source_files'])
    assert read(batch/'summary.json')['source_integrity'] == 'PASS'
    folder = batch/case_id
    result = cached_result(folder, {**identity, 'case_sha256': encode(case)})
    return case, folder, result


def baseline_reproduction():
    plan = read(BASELINE)
    old_path = ROOT/plan['original_protocol']
    assert sha(old_path) == plan['original_protocol_sha256']
    old_case, old, _ = verified_case(old_path, plan['original_case_id'])
    case, new, result = verified_case(BASELINE, plan['cases'][0]['case_id'])
    before = SimulationEnvironment(old_case['config'], seed=old_case['seed'])
    after = SimulationEnvironment(case['config'], seed=case['seed'])
    assert asdict(before.controller.params) == asdict(after.controller.params)
    assert before.contact_params == after.contact_params
    assert before.controller.phase_monitor.gates == after.controller.phase_monitor.gates
    equal, arrays = replay_equal(old/'replay.npz', new/'replay.npz')
    assert equal
    checks, identities = {}, {}
    old_cfg, new_cfg = read(old/'config.json'), read(new/'config.json')
    for name in ('paired_metrics.json', 'physics_substeps.json', 'summary.json'):
        a, b = read(old/name), read(new/name)
        if name == 'paired_metrics.json':
            values = [a['summary'].pop('config_sha256'), b['summary'].pop('config_sha256')]
            assert values == [encode(old_cfg), encode(new_cfg)]
            identities['config_sha256'] = values
        if name == 'summary.json':
            values = [a['provenance'].pop('config_hash'), b['provenance'].pop('config_hash')]
            assert values == [config_hash(old_cfg), config_hash(new_cfg)]
            identities['config_hash'] = values
            identities['git_sha'] = [a['provenance'].pop('git_sha'), b['provenance'].pop('git_sha')]
        checks[name] = old_fields_equal(a, b)
        assert checks[name], name
    gate = read(new/'summary.json')['cargoes']['cargo_0']['g500']
    contract, metrics = ClosedLoopContract(**gate['gates']), gate['metrics']
    verdict = contract.evaluate(metrics)
    assert verdict.success == gate['success'] and verdict.reasons == gate['failure_reasons']
    hard = []
    if metrics['min_inter_agent_distance'] < metrics['d_min']-contract.d_min_tolerance:
        hard.append('separation')
    if metrics['min_signed_clearance'] < 0:
        hard.append('inside_object')
    if metrics['max_penetration'] > metrics['penetration_budget']:
        hard.append('penetration_budget')
    if metrics['solver_fallbacks'] or metrics['solver_infeasible']:
        hard.append('solver_failure')
    report = dict(status='PASS', case=case['case_id'], current_protocol_sha256=sha(BASELINE),
        original_case=old_case['case_id'], original_protocol_sha256=sha(old_path),
        original_receipt_sha256=sha(old/'case_result.json'), current_receipt_sha256=sha(new/'case_result.json'),
        resolved_parameters_equal=True, physics_and_phase_gates_equal=True, replay_exact=equal, arrays=arrays,
        old_scientific_fields_exact=checks, verified_identity_changes=identities,
        safe_completed=bool(result['task_completed'] and not hard), g500=verdict.success,
        hard_safety_failures=hard, endpoint_error_m=result['euclidean_endpoint_error'],
        completion_seconds=result['termination']['frames_run']*case['config']['dt'],
        safety_metrics={k: metrics[k] for k in ('min_inter_agent_distance', 'd_min', 'min_signed_clearance',
            'max_penetration', 'penetration_budget', 'solver_fallbacks', 'solver_infeasible', 'barrier_scalings')},
        scope='One fresh physical compatibility run on a reused condition; not a new independent reliability trial.')
    assert report['safe_completed'] and report['g500']
    return report


def publish():
    guard()
    if PUBLIC.exists():
        raise RuntimeError('Preserve existing published evidence')
    plan, analysis = read(PROTOCOL), read(OUT/'analysis.json')
    assert sha(PROTOCOL) == analysis['protocol_sha256'] == sha(OUT/'protocol.json')
    assert analysis['source_integrity'] == 'PASS' and len(analysis['cases']) == 8
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    for row in analysis['cases']:
        base = OUT/'offline'/row['case']
        assert sha(base.with_suffix('.npz')) == row['series_sha256']
        assert sha(base.with_name(base.name+'-templates.json')) == row['templates_sha256']
    baseline = baseline_reproduction()
    PUBLIC.mkdir(parents=True, exist_ok=False)
    for name in ('analysis.json', 'protected_baseline.json'):
        shutil.copy2(OUT/name, PUBLIC/name)
    dump(PUBLIC/'baseline_reproduction.json', baseline)
    rows = []
    for r in analysis['cases']:
        rows.append(dict(condition=r['condition'], original_MAE_mm=1000*r['original_same_input_mae_m'],
            fitted_MAE_mm=1000*r['candidate_same_input_mae_m'], availability=r['availability_relative_to_original'],
            paired_frames=r['paired_frames'], original_bad_local_fits=r['original_local_bad_fit_count'],
            fitted_bad_local_fits=r['candidate_local_bad_fit_count'], original_accepted_local=r['accepted_local_readouts'][0],
            fitted_accepted_local=r['accepted_local_readouts'][1],
            gate_pass=all(analysis['gates'][r['condition']].values())))
    with (PUBLIC/'all_eight_blocks.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    # Every predeclared block is displayed. No frame-level confidence intervals.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.8), gridspec_kw={'width_ratios': [3, 1.4]}, layout='constrained')
    for ax, group, title in zip(axes, ([r for r in rows if r['condition'].startswith('near')],
                                     [r for r in rows if r['condition'].startswith('far')]), ('Six near blocks', 'Two far blocks')):
        y = np.arange(len(group))
        ax.barh(y-.17, [r['original_MAE_mm'] for r in group], .32, color='#52687b', label='Protected baseline')
        ax.barh(y+.17, [r['fitted_MAE_mm'] for r in group], .32, color='#d78136', label='Fitted surface')
        ax.set_yticks(y, [r['condition'].replace('near_', '').replace('far_', '').replace('_pos0', '').replace('_s0_r0', '').replace('_', ' ') for r in group])
        ax.invert_yaxis()
        ax.set_xlabel('Paired vector displacement MAE (mm)')
        ax.set_title(title)
        ax.grid(axis='x', alpha=.2)
        ax.set_axisbelow(True)
    axes[0].legend(loc='lower right', fontsize=8)
    fig.suptitle('Same-input replay: all eight reused development blocks\nTeam availability retained: 100%; candidate admission: FAIL')
    fig.savefig(PUBLIC/'same_input_errors.png', dpi=180)
    fig.savefig(PUBLIC/'same_input_errors.pdf')
    plt.close(fig)
    dump(PUBLIC/'figure_manifest.json', dict(
        caption='Paired Euclidean displacement MAE after CONTACT_READY on original-available frames; '
                'candidate and baseline receive identical reconstructed scans. Separate axes for near/far blocks. '
                'No physical performance or population confidence interval is inferred.',
        csv_sha256=sha(PUBLIC/'all_eight_blocks.csv'), analysis_sha256=sha(PUBLIC/'analysis.json'),
        files={n: sha(PUBLIC/n) for n in ('same_input_errors.png', 'same_input_errors.pdf')}))
    dump(OUT/'publication_receipt.json', dict(script_sha256=sha(__file__),
        products={p.name: sha(p) for p in PUBLIC.iterdir() if p.is_file()}, baseline_reproduction=baseline['status']))
    print(json.dumps(dict(offline_status=analysis['status'], baseline_reproduction=baseline)))


if __name__ == '__main__':
    publish()
