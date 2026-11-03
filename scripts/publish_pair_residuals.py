"""Publish complete denominators for the unchanged local-pair residual diagnosis."""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from diagnose_pair_residuals import BASE, CHECKS, PROTOCOL, check
from dbact_sim.experiments.deployment import dump, encode

PUBLIC = ROOT/'docs/results/pair_residual_v1'


def table(path, rows):
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader(); writer.writerows(rows)


def main():
    guard()
    _, frozen = check()
    outcome = read(BASE/'analysis/outcome.json')
    assert outcome['status'] == 'VERIFIED_DIAGNOSTIC'
    assert sha(BASE/'analysis/analysis.json') == outcome['analysis_sha256']
    report = read(BASE/'analysis/analysis.json')
    assert len(report['residuals']) == 2 and len(report['pairs']) == 36 and len(report['phases']) == 8
    assert report['protocol_sha256'] == sha(PROTOCOL)
    assert report['controller_replay_frames'] == 2063 and report['phase_frames'] == 5550
    PUBLIC.mkdir(parents=True, exist_ok=False)
    states, individual, pair_rows, phase_rows = [], [], [], []
    for w in report['residuals']:
        agent = w['agent']
        A = np.asarray(agent['A']); obj = np.asarray(agent['kinds']) == 'object'
        full = next(s for s in w['systems'] if s['mode'] == 'uncapped_full')
        lower = np.asarray(full['b'])[obj]/np.linalg.norm(A[obj], axis=1)
        states.append(dict(saved_frame=w['frame'], phase=w['phase'], agent_id=agent['agent_id'],
            true_clearance_m=agent['true_gap'], r_safe_m=agent['params']['r_safe'],
            true_margin_m=w['true_margin'], minimum_observed_h_m=min(agent['h']),
            maximum_uncapped_single_object_row_speed_lower_bound=float(max(lower)),
            original_max_speed=agent['params']['max_speed'], scaled_factor=agent['scale'],
            actual_neighbor_count=len(agent['neighbors']), finite_support_applied=agent['support'].get('applied', False),
            finite_support_cap_active=agent['support'].get('cap_active', ''),
            normal_motion_min=min(agent['normal_motion']), normal_motion_max=max(agent['normal_motion'])))
        for s in w['systems']:
            joint, obj_group, conflict = s['groups']['joint'], s['groups']['object'], s['minimal_conflict']
            individual.append(dict(saved_frame=w['frame'], mode=s['mode'],
                bounded_exact_feasible=joint['bounded']['exact_feasible'],
                bounded_independent_feasible=joint['bounded']['feasible'],
                bounded_independent_status=joint['bounded']['status'],
                unbounded_independent_feasible=joint['unbounded']['feasible'],
                unbounded_minimum_speed=joint['unbounded'].get('minimum_speed', ''),
                object_only_bounded_feasible=obj_group['bounded']['feasible'],
                object_only_minimum_speed=obj_group['unbounded'].get('minimum_speed', ''),
                minimal_conflict_indices=';'.join(map(str,conflict['indices'])),
                minimal_conflict_kinds=';'.join(conflict['kinds'])))
    for p in report['pairs']:
        row = dict(saved_frame=p['frame'], peer=p['peer'], mode=p['mode'],
            two_valid_witnesses=p['two_valid_witnesses'], system_sha256=encode(dict(A=p['A'], b=p['b'])))
        for name in ('clarabel', 'slsqp'):
            cert = p[name]
            row.update({name+'_status':cert['status'], name+'_valid':cert['valid'],
                name+'_application_valid':cert.get('application_valid', False),
                name+'_speed_1':cert.get('speeds', ['', ''])[0],
                name+'_speed_2':cert.get('speeds', ['', ''])[1],
                name+'_minimum_residual':cert.get('minimum_residual', ''),
                name+'_minimum_hold_separation':cert.get('minimum_hold_separation', '')})
        pair_rows.append(row)
    for p in report['phases']:
        contact = p['phases'].get('CONTACT_READY', {})
        brake = p['before_transitions']['BRAKE']
        first = p['first_quorums'].get('transport_active_signal', {})
        phase_rows.append(dict(case_id=p['case_id'], arm=p['arm'], frames=p['frames'],
            physics_substeps=p['substep_samples'], formal_transport=p['before_transitions']['TRANSPORT']['entered'],
            contact_ready_max_declared_pushers=contact.get('current_push_flag_max', 0),
            contact_ready_max_physical_positive_contacts=contact.get('physical_positive_contact_max', 0),
            before_brake_rotation_deg=brake['rotation_before_deg'], brake_time_s=brake['control_time_s'],
            first_transport_quorum_s=first.get('control_time_s', ''),
            quorum_minus_brake_s=(first['control_time_s']-brake['control_time_s']) if first and brake['entered'] else '',
            contact_ready_positive_first_samples=contact.get('positive_contact_robot_samples', 0),
            contact_ready_nonpush_positive_first_samples=contact.get('positive_samples_without_push_flag', 0)))
    table(PUBLIC/'residual_states.csv', states)
    table(PUBLIC/'individual_systems.csv', individual)
    table(PUBLIC/'pair_attempts.csv', pair_rows)
    table(PUBLIC/'all_eight_phases.csv', phase_rows)
    table(PUBLIC/'c_components.csv', [dict(phase=k, **v) for k,v in report['c_components'].items()])
    dump(PUBLIC/'residuals.json', report['residuals'])
    dump(PUBLIC/'phases.json', report['phases'])
    dump(PUBLIC/'selected_pair_witnesses.json', [p for p in report['pairs']
        if (p['frame'], p['peer']) in ((402, 'agent_09'), (989, 'agent_03'))])
    counts = {str(frame):{mode:dict(attempts=len(group := [p for p in report['pairs'] if p['frame']==frame and p['mode']==mode]),
        two_valid=sum(p['two_valid_witnesses'] for p in group),
        two_applied_valid=sum(all(p[s].get('application_valid', False) for s in ('clarabel', 'slsqp')) for p in group))
        for mode in ('capped_full', 'uncapped_full')} for frame in (402, 989)}
    dump(PUBLIC/'summary.json', dict(status=report['status'], new_physical_episodes=0,
        controller_replay_frames=2063, separate_preflight_frames=40, phase_frames=5550,
        physics_substeps=22200, phase_episodes=8, individual_systems=10, pair_problems=36,
        pair_counts=counts, component_scope=report['component_scope'], c_components=report['c_components'],
        runtime_source_sha256=frozen['runtime_source_sha256'], canonical_sha256=frozen['canonical_sha256'],
        candidate_adopted=False, physical_gate_scope='Prior eight physical episodes keep both FAIL verdicts; no new physical validation'))
    dump(PUBLIC/'engineering_checks.json', dict(tests=frozen['tests'], source_unchanged=True,
        canonical_sha256=frozen['canonical_sha256'], environment=frozen['environment'],
        replays={alias:read(BASE/'replay'/alias/'outcome.json') for alias in ('c', 'skew')},
        preflights={alias:read(BASE/'preflight'/alias/'outcome.json') for alias in ('c', 'skew')}))
    products = {p.name:sha(p) for p in PUBLIC.iterdir() if p.is_file()}
    dump(PUBLIC/'publication.json', dict(outputs=products, protocol_sha256=sha(PROTOCOL),
        raw_analysis_sha256=outcome['analysis_sha256'], publisher_sha256=sha(__file__),
        new_physical_episodes=0, pair_problems=36, phase_episodes=8))
    print(f'Published {len(products)} products; all36pair problems and all8phase episodes retained', flush=True)


if __name__ == '__main__': main()
