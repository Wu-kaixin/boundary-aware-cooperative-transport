"""Publish the complete residual/contact diagnosis without new physics."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT, guard, read, sha

RAW = ROOT/'runs/research/2026-11-03-residual-coordination-v1/attempt2'
PAIR = ROOT/'runs/research/2026-11-03-local-pair-feasibility-v1'
PUBLIC = ROOT/'docs/results/residual_coordination_v1'
CHECKS = ROOT/'runs/maintenance/2026-11-03-residual-coordination-v1'


def write_csv(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(destination=PUBLIC):
    PUBLIC = Path(destination).resolve()
    guard()
    assert PUBLIC == ROOT/'docs/results/residual_coordination_v1' or PUBLIC.is_relative_to(ROOT/'runs')
    import finite_acquisition_study_v2 as study
    from dbact_sim.experiments.deployment import dump
    plan = study.check_freeze()
    for folder in (RAW, PAIR):
        outcome = read(folder/'outcome.json')
        assert sha(folder/'analysis.json') == outcome['analysis_sha256']
    analysis, pair = read(RAW/'analysis.json'), read(PAIR/'analysis.json')
    assert analysis['status'] == 'VERIFIED_DIAGNOSTIC'
    assert analysis['phase_reconstruction_frames'] == 5361 and analysis['substep_samples'] == 21444
    assert analysis['source_sha256'] == pair['source_sha256'] == plan['frozen_source_sha256']
    assert pair['denominator'] == 4 and pair['jointly_feasible_systems'] == 4
    assert pair['prior_analysis_sha256'] == sha(RAW/'analysis.json')
    for name, digest in analysis['inputs'].items():
        assert sha(ROOT/name) == digest
    for report in (analysis, pair):
        for name, digest in report['scripts'].items():
            assert sha(ROOT/name) == digest
    assert sha(ROOT/'experiments/ccta2027/local_pair_feasibility_v1_design.md') == pair['design_sha256']
    assert not PUBLIC.exists()
    PUBLIC.mkdir(parents=True)
    shutil.copy2(RAW/'analysis.json', PUBLIC/'analysis.json')
    shutil.copy2(PAIR/'analysis.json', PUBLIC/'pair_feasibility.json')
    residual_rows = []
    for witness in analysis['residuals']:
        for system in witness['systems']:
            coupled = next(s for s in pair['systems'] if s['case_id'] == witness['case_id'] and s['kind'] == system['kind'])
            conflict = system['minimal_conflict']
            neighbor = next(witness['row_geometry'][i] for i in conflict['indices']
                            if witness['row_geometry'][i]['kind'] == 'agent')
            residual_rows.append(dict(case_id=witness['case_id'], frame=witness['frame'], kind=system['kind'],
                robot=witness['agent_id'], blocking_neighbor=neighbor['neighbor_id'],
                agent_distance_m=neighbor['distance_m'], distance_above_d_min_m=neighbor['distance_above_d_min_m'],
                true_object_margin_m=witness['true_margin_m'], original_speed_cap=witness['params']['max_speed'],
                agent_only_feasible=system['groups']['agent']['bounded']['feasible'],
                object_only_feasible=system['groups']['object']['bounded']['feasible'],
                independent_joint_feasible=system['groups']['joint']['bounded']['feasible'],
                uncapped_minimum_speed=system['groups']['joint']['unbounded']['minimum_speed'],
                retained_rows='|'.join(map(str, conflict['indices'])), cap_independent_conflict=conflict['cap_independent'],
                paired_two_valid_witnesses=coupled['two_valid_witnesses'],
                paired_robot_speed=coupled['clarabel']['speeds'][0], paired_neighbor_speed=coupled['clarabel']['speeds'][1],
                paired_minimum_residual=coupled['clarabel']['minimum_residual']))
    write_csv(PUBLIC/'residual_constraints.csv', residual_rows)
    phase_rows, contact_rows = [], []
    for episode in analysis['episodes']:
        before = episode['before_transitions']['BRAKE']
        ready = episode['phases'].get('CONTACT_READY', {})
        quorum = episode['first_quorums']
        phase_rows.append(dict(condition=episode['condition'], arm=episode['arm'], frames=episode['frames'],
            substep_samples=episode['substep_samples'], endpoint_m=episode['endpoint_m'],
            maximum_rotation_deg=episode['maximum_absolute_rotation_deg'],
            formal_transport=episode['final_response']['transport_frame'] is not None,
            brake_time_s=before['control_time_s'], rotation_before_brake_deg=before['rotation_before_deg'],
            forward_displacement_before_brake_m=before['true_displacement_before'][0],
            first_positive_physical_quorum_s=quorum.get('positive_physical_contacts', {}).get('control_time_s'),
            first_current_push_quorum_s=quorum.get('current_push_flags', {}).get('control_time_s'),
            first_active_signal_quorum_s=quorum.get('transport_active_signal', {}).get('control_time_s'),
            contact_ready_seconds=ready.get('duration_s', 0), contact_ready_max_push_flags=ready.get('current_push_flag_max', 0),
            contact_ready_max_positive_contacts=ready.get('physical_positive_contact_max', 0),
            positive_contact_samples=ready.get('positive_contact_robot_samples', 0),
            positive_samples_without_push_flag=ready.get('positive_samples_without_push_flag', 0),
            contact_ready_torque_impulse=ready.get('substep_net_contact_impulses', {}).get('torque_impulse', 0)))
        for phase, agents in episode['first_substep_contact_samples_by_phase_and_agent'].items():
            for aid, samples in agents.items():
                contact_rows.append(dict(condition=episode['condition'], arm=episode['arm'], phase=phase, agent=aid,
                    sampling='first physical substep; sums are samples, not impulses', **samples))
    write_csv(PUBLIC/'phase_summary.csv', phase_rows)
    write_csv(PUBLIC/'first_sample_contact_distribution.csv', contact_rows)
    fig, axes = plt.subplots(1, 3, figsize=(16.5, 4.9))
    fig.subplots_adjust(left=.055, right=.985, bottom=.24, top=.82, wspace=.29)
    rows = [r for r in residual_rows if r['kind'] == 'full_rho']
    x = np.arange(2)
    for offset, field, label, color in [(-.25, 'uncapped_minimum_speed', 'Uncoupled minimum (cap removed)', '#777777'),
            (0., 'paired_robot_speed', 'Paired feasible witness: robot 05', '#0072B2'),
            (.25, 'paired_neighbor_speed', 'Paired feasible witness: robot 09', '#E69F00')]:
        values = [r[field] for r in rows]
        axes[0].bar(x+offset, values, width=.23, color=color, label=label)
        for xx, value in zip(x+offset, values):
            axes[0].text(xx, value+.012, f'{value:.3f}', ha='center', fontsize=8)
    axes[0].axhline(.35, color='#C51B36', linestyle='--', label='Original per-robot cap: 0.35')
    axes[0].set(xticks=x, xticklabels=['Far C: frame 365', 'Far skew: frame 399'], ylim=(0, .77),
                ylabel='Speed (m/s)', title='Local pair responsibility: full original rho')
    axes[0].legend(loc='upper left', fontsize=7, frameon=False)
    for arm, label, color in [('reference_reacquisition', 'Reference reacquisition', '#777777'),
                              ('finite_acquisition_support', 'Finite support', '#0072B2')]:
        case = next(c for c in plan['cases'] if c['condition_id'] == 'far_c_open_away_pos0' and c['arm'] == arm)
        folder = study.OUT/case['case_id']
        episode = next(e for e in analysis['episodes'] if e['case_id'] == case['case_id'])
        with np.load(folder/'replay.npz', allow_pickle=False) as saved:
            angles = np.rad2deg(saved['cargo/cargo_0/angles'])
            times = saved['times']
        axes[1].plot(times, angles, color=color, label=label)
        brake = episode['before_transitions']['BRAKE']
        axes[1].scatter([brake['control_time_s']], [brake['rotation_before_deg']], color=color, s=25)
        axes[1].axvline(brake['control_time_s'], color=color, linestyle=':', alpha=.65)
        if arm == 'finite_acquisition_support':
            raw = read(folder/'paired_metrics.json')['steps']
            control_times = np.arange(len(raw))*case['config']['dt']
            axes[2].step(control_times, [r['phase_guards']['transport_active'] for r in raw],
                where='post', color='#0072B2', label='Previous push flags (phase signal)')
            axes[2].step(control_times, [r['positive_contacts'] for r in raw],
                where='post', color='#E69F00', alpha=.8, label='Positive first-substep contacts')
            axes[2].axvline(brake['control_time_s'], color='#777777', linestyle=':', label='BRAKE: 46.45 s')
    axes[1].set(xlabel='Time (s)', ylabel='True rotation (degrees)', title='C rotation occurs mostly before BRAKE')
    axes[1].legend(loc='upper left', fontsize=8, frameon=False)
    axes[2].axhline(4, color='#C51B36', linestyle='--', label='Unchanged quorum: 4')
    axes[2].set(xlabel='Control time (s)', ylabel='Robot count', title='C contact force and push flags differ',
        ylim=(-.2, max(4, max(max(r['positive_contacts'], r['phase_guards']['transport_active']) for r in raw))+.7))
    axes[2].legend(loc='upper left', fontsize=7, frameon=False)
    for ax in axes:
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle('Residual coordination diagnosis: fixed states and preserved physical trajectories', fontsize=15)
    fig.text(.5, .07, 'No new physical episodes. Pair witnesses retain all non-mutual rows, radii, margins and speed caps.\n'
        'Four reused conditions; contact samples and integrated total contact impulses are reported separately.', ha='center', fontsize=9)
    fig.savefig(PUBLIC/'diagnosis.png', dpi=180)
    fig.savefig(PUBLIC/'diagnosis.pdf')
    plt.close(fig)
    stats = ET.parse(CHECKS/'core.xml').getroot().find('testsuite').attrib
    core = {name:int(stats[name]) for name in ('tests', 'failures', 'errors', 'skipped')}
    core['passed'] = core['tests']-core['failures']-core['errors']-core['skipped']
    assert core['passed'] == 1000 and core['failures'] == core['errors'] == 0
    dump(PUBLIC/'engineering_checks.json', dict(core=core, core_xml_sha256=sha(CHECKS/'core.xml'),
        runtime_source_sha256=plan['frozen_source_sha256'], canonical_sha256=plan['canonical_sha256'],
        new_physical_episodes=0, phase_reconstruction_frames=5361, physics_substep_samples=21444,
        two_selected_states=2, paired_systems=4, two_solver_paired_witnesses=4,
        invalidated_attempt='attempt1 retained with incorrect sampling labels; attempt2 is authoritative'))
    outputs = {p.name:sha(p) for p in PUBLIC.iterdir() if p.is_file()}
    dump(PUBLIC/'publication.json', dict(script_sha256=sha(__file__), outputs=outputs,
        source_sha256=plan['frozen_source_sha256'], analysis_sha256=sha(RAW/'analysis.json'),
        pair_analysis_sha256=sha(PAIR/'analysis.json'), new_physical_episodes=0, candidate_implemented=False))
    print(json.dumps(dict(status='PUBLISHED_DIAGNOSTIC', outputs=len(outputs), core=core)), flush=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=PUBLIC)
    main(parser.parse_args().out)
