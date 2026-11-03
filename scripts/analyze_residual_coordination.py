"""Decompose preserved residual QPs and audit saved phase/contact evidence."""
from __future__ import annotations

import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from qp_conflict_study import independent

DESIGN = ROOT/'experiments/ccta2027/residual_coordination_v1_design.md'
WITNESSES = ROOT/'docs/results/finite_acquisition_support_v2/audit_witnesses.json'
PUBLICATION = WITNESSES.parent/'publication.json'


def bounded_check(A, b, speed):
    from dbact.safety.qp2d import solve_min_norm_2d
    exact = solve_min_norm_2d(np.zeros(2), A, b, speed)
    cross = independent(A, b, speed)
    assert cross['numerically_valid'] and cross['feasible'] == bool(exact.feasible)
    return dict(**cross, exact_feasible=bool(exact.feasible))


def decompose(A, b, kinds, speed):
    """Ascending deletion gives an inclusion-minimal bounded conflict."""
    from dbact.safety.qp2d import solve_min_norm_2d
    A, b = np.asarray(A, dtype=float).reshape(-1, 2), np.asarray(b, dtype=float)
    kinds = np.asarray(kinds, dtype=str)
    assert len(A) == len(b) == len(kinds)
    groups = {}
    for name in [*sorted(set(kinds)), 'joint']:
        ids = np.arange(len(b)) if name == 'joint' else np.flatnonzero(kinds == name)
        uncapped = independent(A[ids], b[ids], None)
        assert uncapped['numerically_valid']
        groups[name] = dict(indices=ids.tolist(), bounded=bounded_check(A[ids], b[ids], speed),
                            unbounded=uncapped)
    conflict = None
    if not groups['joint']['bounded']['feasible']:
        ids = list(range(len(b)))
        for index in range(len(b)):
            trial = [i for i in ids if i != index]
            if not solve_min_norm_2d(np.zeros(2), A[trial], b[trial], speed).feasible:
                ids = trial
        retained = bounded_check(A[ids], b[ids], speed)
        assert not retained['feasible']
        removals = []
        for index in ids:
            trial = [i for i in ids if i != index]
            result = bounded_check(A[trial], b[trial], speed)
            assert result['feasible']
            removals.append(dict(removed_index=index, **result))
        uncapped = independent(A[ids], b[ids], None)
        assert uncapped['numerically_valid']
        conflict = dict(indices=ids, kinds=kinds[ids].tolist(), bounded=retained,
            unbounded=uncapped, cap_independent=not uncapped['feasible'], one_row_removals=removals,
            selection='Ascending single-row deletion; inclusion-minimal, not minimum cardinality')
    return dict(groups=groups, minimal_conflict=conflict)


def substep_impulses(samples, rows, dt, substeps):
    """Integrate every saved net-force sample, assigned to its command phase."""
    assert substeps > 0 and len(samples) == len(rows)*substeps
    result = {}
    delta = dt/substeps
    for index, sample in enumerate(samples):
        row, within = rows[index//substeps], index % substeps
        assert sample['frame'] == row['frame'] and sample['substep'] == within
        assert sample['object_id'] == 'cargo_0'
        assert abs(sample['offset_seconds']-within*delta) < 1e-12
        group = result.setdefault(row['phase'], dict(samples=0, force_impulse=np.zeros(2),
            torque_impulse=0., absolute_torque_impulse=0.))
        group['samples'] += 1
        group['force_impulse'] += np.asarray(sample['force'])*delta
        group['torque_impulse'] += sample['torque']*delta
        group['absolute_torque_impulse'] += abs(sample['torque'])*delta
    for group in result.values():
        group['force_impulse'] = group['force_impulse'].tolist()
    return result


def check_first_contact_sample(row, samples):
    """The engine returns its pre-loop report, i.e. substep zero."""
    first = samples[0]
    assert first['frame'] == row['frame'] and first['substep'] == 0
    np.testing.assert_array_equal(row['actual_net_force'], first['force'])
    assert row['actual_net_torque'] == first['torque']


def residuals(plan, batch, episodes):
    from dbact_sim.environment import SimulationEnvironment
    report = []
    selected = [w for w in read(WITNESSES)['witnesses'] if w['original_scaled']]
    assert {(w['case_id'], w['frame']) for w in selected} == {
        ('far_c_open_away_pos0_finite_acquisition_support', 365),
        ('far_skew_box_s0_r0_pos0_finite_acquisition_support', 399)}
    for witness in selected:
        case = next(c for c in plan['cases'] if c['case_id'] == witness['case_id'])
        folder = batch/case['case_id']
        with np.load(folder/'replay.npz', allow_pickle=False) as saved:
            index = witness['frame']-1
            positions = saved['agent_positions'][index].copy()
            center = saved['cargo/cargo_0/centers'][index].copy()
            angle = float(saved['cargo/cargo_0/angles'][index])
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        env.cargoes[0].set_pose(center, angle)
        agent_ids = [a.agent_id for a in env.agents]
        self_index = agent_ids.index(witness['agent_id'])
        position = positions[self_index]
        clearance, normal, _ = env.cargoes[0].signed_distance(position.reshape(1, 2))
        params = witness['params']
        original_kinds = np.asarray(witness['original_row_kinds'])
        hard = original_kinds != 'object'
        hard_A = np.asarray(witness['original_A'])[hard]
        hard_b = np.asarray(witness['original_b'])[hard]
        kinds = [*original_kinds[hard].tolist(), *(['object']*len(witness['new_h']))]
        row_geometry = []
        for row_index, (a, kind) in enumerate(zip(hard_A, original_kinds[hard])):
            item = dict(index=row_index, kind=str(kind), A=a.tolist(), b=float(hard_b[row_index]))
            if kind == 'agent':
                candidate = position-a/2.
                distances = np.linalg.norm(positions-candidate, axis=1)
                neighbor = int(np.argmin(distances))
                assert neighbor != self_index and distances[neighbor] < 1e-12
                distance = float(np.linalg.norm(positions[neighbor]-position))
                expected = -.5*params['gamma_agent']*(distance**2-params['d_min']**2)
                assert abs(expected-item['b']) < 1e-12
                item.update(neighbor_id=agent_ids[neighbor], neighbor_position=positions[neighbor].tolist(),
                    distance_m=distance, distance_above_d_min_m=distance-params['d_min'])
            row_geometry.append(item)
        for index, (a, h) in enumerate(zip(witness['new_A'], witness['new_h'])):
            row_geometry.append(dict(index=len(hard_A)+index, kind='object', A=a, h=h))
        systems = []
        for counterfactual in witness['corrected_counterfactual']:
            A, b = np.asarray(counterfactual['A']), np.asarray(counterfactual['b'])
            np.testing.assert_array_equal(A[:len(hard_A)], hard_A)
            np.testing.assert_array_equal(b[:len(hard_A)], hard_b)
            np.testing.assert_array_equal(A[len(hard_A):], witness['new_A'])
            rho = params['rho'] if counterfactual['kind'] == 'full_rho' else 0.
            demand = -params['gamma_obj']*np.asarray(witness['new_h'])+rho
            np.testing.assert_array_equal(b[len(hard_A):], demand)
            result = decompose(A, b, kinds, params['max_speed'])
            systems.append(dict(kind=counterfactual['kind'], A=A.tolist(), b=b.tolist(), **result))
        report.append(dict(case_id=case['case_id'], frame=witness['frame'], agent_id=witness['agent_id'],
            control_time_s=(witness['frame']-1)*case['config']['dt'], params=params,
            saved_state_source='Historical corrected-geometry observer matrices; current physical pre-step pose',
            position=position.tolist(), true_clearance_m=float(clearance[0]),
            true_margin_m=float(clearance[0]-params['r_safe']), true_normal=normal[0].tolist(),
            row_geometry=row_geometry, systems=systems, new_physical_episodes=0))
        print(json.dumps(dict(case=case['case_id'], residual_frame=witness['frame'],
            conflicts=[dict(kind=s['kind'], indices=s['minimal_conflict']['indices'],
                cap_independent=s['minimal_conflict']['cap_independent'],
                joint_minimum_speed=s['groups']['joint']['unbounded'].get('minimum_speed')) for s in systems])), flush=True)
    return report


def phase_contact(case, folder, result):
    from dbact.control.phase import PhaseSignals
    from dbact_sim.environment import SimulationEnvironment
    rows = read(folder/'paired_metrics.json')['steps']
    samples = read(folder/'physics_substeps.json')
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    monitor, dt = env.controller.phase_monitor, env.dt
    assert result['termination']['frames_run'] == len(rows)
    substeps = case['config']['transport']['substeps']
    impulses = substep_impulses(samples, rows, dt, substeps)
    with np.load(folder/'replay.npz', allow_pickle=False) as saved:
        centers = saved['cargo/cargo_0/centers'].copy()
        angles = saved['cargo/cargo_0/angles'].copy()
    phases, transitions, first_quorums, previous = {}, [], {}, []
    last_phase = 'SEARCH'
    goal = np.asarray(result['goal_direction'])
    contact_samples = {}
    for k, row in enumerate(rows):
        assert row['frame'] == k+1
        signal = PhaseSignals(**row['phase_guards'])
        assert signal.transport_active == sum(bool(d['push_side']) for d in previous)
        previous = row['diagnostics']
        monitor.update(signal, k)
        assert monitor.phase.name == row['phase'] and monitor.as_dict() == row['phase_response']
        phase = row['phase']
        group = phases.setdefault(phase, dict(frames=0, signal_active_max=0, current_push_flag_max=0,
            physical_positive_contact_max=0, current_push_robot_steps=0, mode_counts=Counter(),
            positive_contact_robot_samples=0, positive_samples_with_push_flag=0,
            positive_samples_without_push_flag=0))
        pushing = sum(bool(d['push_side']) for d in previous)
        group['frames'] += 1
        group['signal_active_max'] = max(group['signal_active_max'], signal.transport_active)
        group['current_push_flag_max'] = max(group['current_push_flag_max'], pushing)
        group['physical_positive_contact_max'] = max(group['physical_positive_contact_max'], row['positive_contacts'])
        group['current_push_robot_steps'] += pushing
        group['positive_contact_robot_samples'] += row['positive_contacts']
        group['mode_counts'].update(d['mode'] for d in previous)
        for name, value, threshold in [('contact_ready_signal', signal.contact_ready, monitor.gates.contact_quorum),
                ('transport_active_signal', signal.transport_active, monitor.gates.transport_quorum),
                ('current_push_flags', pushing, monitor.gates.transport_quorum),
                ('positive_physical_contacts', row['positive_contacts'], monitor.gates.contact_quorum)]:
            if value >= threshold:
                first_quorums.setdefault(name, dict(frame=k+1, control_time_s=k*dt, phase=phase,
                    count=value, threshold=threshold))
        if phase != last_phase:
            transitions.append(dict(to_phase=phase, from_phase=last_phase, frame=k+1, control_time_s=k*dt,
                signals=asdict(signal), before_command_displacement=(centers[k]-centers[0]).tolist(),
                before_command_rotation_deg=float(np.rad2deg(angles[k])),
                after_step_true_displacement=row['true_center_displacement'],
                after_step_rotation_deg=float(np.rad2deg(row['rotation']))))
        last_phase = phase
        assert len(row['contact_ids']) == len(row['contact_forces']) == row['contacts']
        frame_samples = samples[k*substeps:(k+1)*substeps]
        check_first_contact_sample(row, frame_samples)
        forces = np.asarray(row['contact_forces']).reshape(-1, 2)
        np.testing.assert_allclose(forces.sum(axis=0), row['actual_net_force'], rtol=0, atol=1e-10)
        assert sum(float(force@goal) > 1e-9 for force in forces) == row['positive_contacts']
        cargo = env.cargoes[0]
        cargo.set_pose(frame_samples[0]['position'], frame_samples[0]['angle'])
        _, normals, feet = cargo.signed_distance(np.asarray(row['robot_positions']))
        agent_ids = [agent.agent_id for agent in env.agents]
        diagnostic_by_id = {d['agent_id']:d for d in previous}
        reconstructed_torque = 0.
        per_agent = contact_samples.setdefault(phase, {})
        for aid, force, stored_normal in zip(row['contact_ids'], forces, row['contact_normals']):
            agent_index = agent_ids.index(aid)
            np.testing.assert_allclose(normals[agent_index], stored_normal, rtol=0, atol=1e-10)
            relative = feet[agent_index]-cargo.position
            torque = float(relative[0]*force[1]-relative[1]*force[0])
            reconstructed_torque += torque
            totals = per_agent.setdefault(aid, dict(contact_samples=0, positive_samples=0,
                forward_force_sample_sum=0., opposing_force_sample_sum=0.,
                torque_sample_sum=0., absolute_torque_sample_sum=0.,
                positive_samples_with_push_flag=0, positive_samples_without_push_flag=0))
            forward = float(force@goal)
            totals['contact_samples'] += 1
            totals['positive_samples'] += int(forward > 1e-9)
            totals['forward_force_sample_sum'] += max(0., forward)
            totals['opposing_force_sample_sum'] += min(0., forward)
            totals['torque_sample_sum'] += torque
            totals['absolute_torque_sample_sum'] += abs(torque)
            if forward > 1e-9:
                key = ('positive_samples_with_push_flag' if diagnostic_by_id[aid]['push_side']
                       else 'positive_samples_without_push_flag')
                totals[key] += 1
                group[key] += 1
        assert abs(reconstructed_torque-row['actual_net_torque']) < 1e-8
        for sample in frame_samples:
            np.testing.assert_array_equal(sample['robot_positions'], row['robot_positions'])
    for phase, group in phases.items():
        group.update(duration_s=group['frames']*dt, substep_net_contact_impulses=impulses[phase])
        group['mode_counts'] = dict(group['mode_counts'])
    cutoffs = {}
    for target in ('TRANSPORT', 'BRAKE'):
        cut = next((k for k, row in enumerate(rows) if row['phase'] == target), len(rows))
        phase_set = {row['phase'] for row in rows[:cut]}
        cutoffs[target] = dict(entered=cut < len(rows), frame_count=cut,
            control_time_s=cut*dt if cut < len(rows) else None,
            true_displacement_before=(centers[cut]-centers[0]).tolist(),
            rotation_before_deg=float(np.rad2deg(angles[cut])),
            sampled_forward_impulse=sum(float(np.asarray(impulses[p]['force_impulse'])@goal) for p in phase_set),
            sampled_torque_impulse=sum(impulses[p]['torque_impulse'] for p in phase_set),
            sampled_absolute_torque_impulse=sum(impulses[p]['absolute_torque_impulse'] for p in phase_set))
    return dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
        receipt_sha256=sha(folder/'case_result.json'), frames=len(rows), substep_samples=len(samples),
        dt=dt, gates=asdict(monitor.gates), phases=phases, transitions=transitions,
        first_quorums=first_quorums, before_transitions=cutoffs,
        final_response=monitor.as_dict(), first_substep_contact_samples_by_phase_and_agent=contact_samples,
        endpoint_m=result['euclidean_endpoint_error'], final_displacement=rows[-1]['true_center_displacement'],
        maximum_absolute_rotation_deg=float(np.max(np.abs(np.rad2deg(angles)))),
        per_agent_scope='First-substep samples only; reconstructed lever-arm torque checked against stored total; sample sums are not integrated per-agent impulses',
        net_impulse_scope='All saved pre-integration contact samples; descriptive, not causal attribution of final rotation')


def run(destination):
    guard()
    import finite_acquisition_study_v2 as study
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, source_hashes
    plan = study.check_freeze()
    publication = read(PUBLICATION)
    assert publication['source_sha256'] == plan['frozen_source_sha256']
    assert sha(WITNESSES) == publication['outputs']['audit_witnesses.json']
    out = Path(destination).resolve()
    assert out.is_relative_to(ROOT/'runs')
    out.mkdir(parents=True, exist_ok=False)
    scripts = ['scripts/analyze_residual_coordination.py', 'scripts/qp_conflict_study.py',
        'scripts/research_io.py', 'scripts/finite_acquisition_study_v2.py', 'tests/test_residual_coordination_audit.py']
    inputs = {str(p.relative_to(ROOT)):sha(p) for p in (study.PROTOCOL, WITNESSES, PUBLICATION, DESIGN)}
    manifest = read(study.OUT/'manifest.json')
    episodes = {}
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        assert result['execution_status'] == 'COMPLETED'
        episodes[case['case_id']] = result
        inputs[str((folder/'case_result.json').relative_to(ROOT))] = sha(folder/'case_result.json')
    for name in [*plan['source_files'], *scripts]:
        target = out/'freeze'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    shutil.copy2(DESIGN, out/'freeze/design.md')
    identity = dict(source_sha256=plan['frozen_source_sha256'], source_files=plan['source_files'],
        canonical_sha256=plan['canonical_sha256'], environment=plan['environment'], inputs=inputs,
        scripts={s:sha(ROOT/s) for s in scripts}, new_physical_episodes=0,
        new_controller_replay_frames=0, scope='Saved-state QP decomposition and original phase-monitor reconstruction only')
    dump(out/'identity.json', identity)
    try:
        conflicts = residuals(plan, study.OUT, episodes)
        dump(out/'residuals.json', conflicts)
        phases = []
        for case in plan['cases']:
            result = phase_contact(case, study.OUT/case['case_id'], episodes[case['case_id']])
            phases.append(result)
            dump(out/(case['case_id']+'_phases.json'), result)
            print(json.dumps(dict(case=case['case_id'], phase_frames=result['frames'],
                first_quorums=result['first_quorums'], before_brake=result['before_transitions']['BRAKE'])), flush=True)
        assert len(phases) == 8
        assert source_hashes() == plan['source_files']
        assert {name:sha(ROOT/name) for name in inputs} == inputs
        assert {name:sha(ROOT/name) for name in scripts} == identity['scripts']
        analysis = dict(**identity, residuals=conflicts, episodes=phases,
            phase_reconstruction_frames=sum(r['frames'] for r in phases),
            substep_samples=sum(r['substep_samples'] for r in phases), status='VERIFIED_DIAGNOSTIC')
        dump(out/'analysis.json', analysis)
        dump(out/'outcome.json', dict(status=analysis['status'], new_physical_episodes=0,
            phase_reconstruction_frames=analysis['phase_reconstruction_frames'],
            substep_samples=analysis['substep_samples'], analysis_sha256=sha(out/'analysis.json'),
            identity_sha256=sha(out/'identity.json')))
    except Exception as exc:
        dump(out/'failure.json', dict(status='FAILED_DIAGNOSTIC', error=repr(exc), new_physical_episodes=0))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    run(parser.parse_args().out)
