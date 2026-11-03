"""Publish the frozen finite-support study without changing its evaluators."""
from __future__ import annotations

from datetime import datetime, timezone
import csv
import gc
import shutil
import xml.etree.ElementTree as ET

import finite_acquisition_study as study
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

PUBLIC = study.ROOT/'docs/results/finite_acquisition_support_v1'


def write_csv(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def probe_summaries():
    """Verify each original snapshot, including the superseded chord-only v1."""
    result = {}
    for name, expected_qps in (('finite-support-probe-v1', 312),
                               ('ray-chord-support-probe-v1', 0),
                               ('ray-chord-support-probe-v2', 104)):
        folder = study.ROOT/'runs/research'/f'2026-11-03-{name}'
        outcome, identity = read(folder/'outcome.json'), read(folder/'identity.json')
        analysis = read(folder/'analysis.json')
        assert outcome['status'] == 'COMPLETED' and outcome['new_physical_episodes'] == 0
        assert outcome['analysis_sha256'] == sha(folder/'analysis.json')
        assert all(outcome[k] == analysis[k] == value for k, value in identity.items())
        script = 'probe_finite_support.py' if name == 'finite-support-probe-v1' else 'probe_ray_chord_support.py'
        assert sha(folder/script) == identity['script_sha256']
        assert sha(folder/'design.md') == identity['design_sha256']
        if 'candidate_module_sha256' in identity:
            assert sha(folder/'finite_support.py') == identity['candidate_module_sha256']
        if 'helper_sha256' in identity:
            assert sha(folder/'research_io.py') == identity['helper_sha256']
        trace_receipts = analysis.get('trace_outcomes', analysis.get('source_receipts'))
        for alias, digest in trace_receipts.items():
            trace = study.ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1'/alias
            assert sha(trace/'outcome.json') == digest
            for product, product_hash in read(trace/'outcome.json')['products'].items():
                assert sha(trace/product) == product_hash
        records = analysis['records']
        assert sum(len(r['solves']) for r in records) == outcome['qp_comparisons'] == expected_qps
        summary = []
        if name == 'finite-support-probe-v1':
            for variant in ('own_scan_aggregate', 'own_segments', 'one_hop_segments'):
                rows = [r for r in records if r['variant'] == variant]
                assert len(rows) == 26
                for motion in (0., .2):
                    for rho in (.02, 0.):
                        solves = [s for r in rows for s in r['solves'] if s['motion_bound'] == motion and s['rho'] == rho]
                        summary.append(dict(variant=variant, states=26, motion_bound=motion, rho=rho,
                            empty_support=sum(r['support_empty'] for r in rows),
                            supported_feasible=sum(s['supported_feasible'] for s in solves),
                            capped_solves=sum(s['cap_active'] > 0 for s in solves),
                            distance_observations=sum(r.get('distance_overestimate_m') is not None for r in rows),
                            distance_overestimates=sum((r.get('distance_overestimate_m') or 0.) > 1e-9 for r in rows)))
        else:
            for motion in (0., .2):
                rows = [r for r in records if r['motion_bound'] == motion]
                assert len(rows) == 26
                for rho in (.02, 0.):
                    solves = [s for r in rows for s in r['solves'] if s['rho'] == rho]
                    observed = [r['distance_overestimate_m'] for r in rows if 'distance_overestimate_m' in r]
                    summary.append(dict(states=26, motion_bound=motion, rho=rho,
                        admitted=sum(r['ready'] for r in rows), feasible=sum(s['feasible'] for s in solves),
                        capped_solves=sum(s['cap_active'] > 0 for s in solves),
                        distance_observations=len(observed),
                        distance_overestimates=sum(v > 1e-9 for v in observed) if observed else None))
        result[name] = dict(raw_directory=folder.relative_to(study.ROOT).as_posix(),
            outcome=outcome, outcome_sha256=sha(folder/'outcome.json'), summary=summary,
            snapshot_hashes={p.name:sha(p) for p in folder.iterdir() if p.is_file()})
    return result


def main():
    guard()
    plan = study.check_freeze()
    analysis = read(study.BASE/'analysis.json')
    outcome = read(study.BASE/'execution_outcome.json')
    assert outcome['status'] == 'COMPLETED' and outcome['completed_denominator'] == 8
    assert outcome['execution_errors'] == 0 and outcome['source_integrity'] == 'PASS'
    assert analysis['protocol_sha256'] == outcome['protocol_sha256'] == sha(study.PROTOCOL)
    assert analysis['source_sha256'] == plan['frozen_source_sha256']
    assert analysis['candidate_adopted'] is False and len(analysis['episodes']) == 8
    gates, full = study.evaluate_gates(analysis['episodes'], analysis['original_reproduction'], plan['conditions'])
    assert gates == analysis['mechanism_gates'] and full == analysis['full_chain_gates']
    assert analysis['mechanism_gate'] == ('PASS' if all(gates.values()) else 'FAIL')
    assert analysis['full_chain_gate'] == ('PASS' if all(full.values()) else 'FAIL')
    old_public = study.ROOT/'docs/results/reference_reacquisition_physical_v1'
    old_publication = read(old_public/'publication.json')
    assert sha(old_public/'analysis.json') == old_publication['analysis_sha256']
    for name, digest in old_publication['outputs'].items():
        assert sha(old_public/name) == digest

    outputs = ('analysis.json', 'all_eight_episodes.csv', 'residual_scalings.json',
               'preliminary_probes.json', 'paired_outcomes.png', 'paired_outcomes.pdf',
               'engineering_checks.json', 'publication.json')
    if any((PUBLIC/name).exists() for name in outputs):
        raise RuntimeError('Preserve existing publication')
    PUBLIC.mkdir(parents=True, exist_ok=True)
    manifest = read(study.OUT/'manifest.json')
    data, records, residual = {}, [], []
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        row = next(r for r in analysis['episodes'] if r['case_id'] == case['case_id'])
        assert row['receipt_sha256'] == sha(folder/'case_result.json')
        data[(case['condition_id'], case['arm'])] = row
        if case['arm'] == study.ARMS[1] and row['scaling']['events']:
            steps = read(folder/'paired_metrics.json')['steps']
            for event in row['scaling']['events']:
                step = steps[event['frame']-1]
                assert step['frame'] == event['frame']
                for diagnostic in step['diagnostics']:
                    if diagnostic['agent_id'] not in event['agents']:
                        continue
                    assert diagnostic['solver_status'] == 'scaled_barrier'
                    residual.append(dict(case_id=case['case_id'], saved_frame=event['frame'],
                        pre_physics_time_s=(event['frame']-1)*case['config']['dt'],
                        agent_id=diagnostic['agent_id'], phase=event['phase'], capture_state=event['capture_state'],
                        support=diagnostic['acquisition_support'],
                        zero_input_feasible=diagnostic['zero_input_feasible'],
                        agent_rows=diagnostic['agent_rows'], object_rows=diagnostic['object_rows'],
                        scope='Saved diagnostics only; no new counterfactual QP or physical episode'))
            del steps
            gc.collect()
    for condition in plan['conditions']:
        for arm in study.ARMS:
            row = data[(condition, arm)]
            scale, acquisition = row['scaling'], row['acquisition']
            support = row['support_audit']['counts']
            records.append(dict(condition=condition, arm=arm, execution=row['execution'],
                safe_completed=row['safe_completed'], false_hold=row['false_hold'], g500=row['g500'],
                hard_safety_failures=';'.join(row['hard_safety']), endpoint_m=row['endpoint_m'],
                simulated_seconds=row['seconds'], budget_terminated=row['budget_terminated'],
                acquisition_status=acquisition['status'], epoch_seconds=acquisition['epoch'],
                valid_zero_epoch=row['zero_epoch_valid'],
                pre_epoch_contact_samples=acquisition['pre_epoch_contact_samples'],
                max_pre_epoch_translation_m=acquisition['max_pre_epoch_translation'],
                max_pre_epoch_rotation_rad=acquisition['max_pre_epoch_rotation'],
                acquisition_scaled_robot_steps=scale['acquisition'],
                post_acquisition_scaled_robot_steps=scale['post_acquisition'],
                scaled_robot_steps=row['barrier_scalings'], minimum_barrier_scale=scale['min_scale'],
                formal_transport=row['phase_response'].get('transport_frame') is not None,
                final_phase=row['final_phase'], maximum_absolute_rotation_deg=row['maximum_absolute_rotation_deg'],
                applied_support_robot_steps=support.get('applied_robot_steps', 0),
                original_row_robot_steps=support.get('original_row_robot_steps', 0),
                point_guard_rows=support.get('point_guard_rows', 0), reach_cap_rows=support.get('reach_cap_rows', 0),
                g500_failure_reasons='; '.join(row['g500_failure_reasons'])))
    probes = probe_summaries()
    shutil.copy2(study.BASE/'analysis.json', PUBLIC/'analysis.json')
    write_csv(PUBLIC/'all_eight_episodes.csv', records)
    dump(PUBLIC/'residual_scalings.json', dict(new_physical_episodes=0, records=residual,
        scope='Posthoc extraction from the eight episodes already counted; not additional validation'))
    dump(PUBLIC/'preliminary_probes.json', probes)

    fig, axes = plt.subplots(2, 3, figsize=(13.8, 8.0))
    fig.subplots_adjust(left=.065, right=.98, bottom=.16, top=.89, wspace=.30, hspace=.32)
    x, width = np.arange(4), .34
    getters = (lambda r:r['endpoint_m']*1000, lambda r:r['seconds'],
               lambda r:r['scaling']['acquisition'], lambda r:r['scaling']['post_acquisition'],
               lambda r:r['maximum_absolute_rotation_deg'],
               lambda r:int(r['phase_response'].get('transport_frame') is not None))
    maxima = [120., 0., 0., 0., 0., 1.]
    for a, (arm, label, color) in enumerate(zip(study.ARMS,
            ('Reference reacquisition', 'Finite acquisition support'), ('#777777', '#0072b2'))):
        rows = [data[(condition, arm)] for condition in plan['conditions']]
        for i, (ax, getter) in enumerate(zip(axes.flat, getters)):
            values = [getter(r) for r in rows]
            maxima[i] = max(maxima[i], max(values))
            bars = ax.bar(x+(a-.5)*width, values, width, color=color, label=label)
            # Stagger arm labels so equal or near-equal paired values stay legible.
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
    fig.suptitle('Finite acquisition support: all eight matched development outcomes', fontsize=14)
    fig.text(.5, .035, 'Both arms: safe completion 4/4, G500 2/4. Mechanism and full-chain gates FAIL; candidate remains off.', ha='center', fontsize=10)
    fig.text(.5, .010, 'Four reused conditions, one run per arm/condition. All original numerical parameters and gates preserved.', ha='center', fontsize=9)
    for suffix in ('png', 'pdf'):
        fig.savefig(PUBLIC/f'paired_outcomes.{suffix}', dpi=180)
    plt.close(fig)

    stats = ET.parse(study.CHECKS/'core-final.xml').getroot().find('testsuite').attrib
    core = {k:int(stats[k]) for k in ('tests', 'failures', 'errors', 'skipped')}
    core['passed'] = core['tests']-core['failures']-core['errors']-core['skipped']
    assert core['failures'] == core['errors'] == 0
    dump(PUBLIC/'engineering_checks.json', dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime_source_sha256=plan['frozen_source_sha256'], core=core,
        core_xml_sha256=sha(study.CHECKS/'core-final.xml'), local_checks=study.CHECKS.relative_to(study.ROOT).as_posix(),
        canonical_sha256=plan['canonical_sha256'], execution=outcome,
        original_arms_exact=all(r['exact'] for r in analysis['original_reproduction'].values()),
        local_information_and_freeze_boundary_audit=all(r['support_audit']['locality_and_freeze_boundary_verified'] for r in analysis['episodes']),
        original_publication_hashes_unchanged=True, environment=plan['environment'],
        scope='Engineering checks and eight new physical episodes; neither general reliability nor a locked-environment rerun'))
    products = {name:sha(PUBLIC/name) for name in outputs if name != 'publication.json'}
    if (PUBLIC/'README.md').exists():
        products['README.md'] = sha(PUBLIC/'README.md')
    dump(PUBLIC/'publication.json', dict(script_sha256=sha(__file__), protocol_sha256=sha(study.PROTOCOL),
        source_sha256=plan['frozen_source_sha256'], outputs=products,
        new_physical_episodes=8, preliminary_probes_new_physical_episodes=0, candidate_adopted=False))
    print(dict(rows=len(records), residual_scaled_robot_steps=len(residual),
        mechanism_gate=analysis['mechanism_gate'], full_chain_gate=analysis['full_chain_gate'],
        core_passed=core['passed'], candidate_adopted=False), flush=True)


if __name__ == '__main__':
    main()
