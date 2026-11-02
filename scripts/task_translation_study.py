"""Frozen 2x2 same-input task-reference comparison, with zero new physics."""
from __future__ import annotations

import argparse
from collections import Counter
import gc
import json
import shutil
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from dbact.control.boundary_navigation import point_goal_feedback
from dbact.control.controller import _merge_scans, _REFERENCE_CHANNELS
from dbact.control.initial_task_reference import InitialReferenceCapture, TaskReferenceTracker, center_error_bound
from dbact.control.task_translation import TranslationTaskReferenceTracker
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import (
    cached_result, environment_versions, limit_worker_threads, source_hashes, verify_source_snapshot)

OLD = ROOT/'experiments/ccta2027/task_reference_v1.json'
DESIGN = ROOT/'experiments/ccta2027/task_translation_v1_design.md'
PROTOCOL = ROOT/'experiments/ccta2027/task_translation_v2.json'
OUT = ROOT/'runs/research/2026-11-03-task-translation-v2'
PUBLIC = ROOT/'docs/results/task_translation_v2'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
SCRIPTS = ['scripts/task_translation_study.py', 'scripts/research_io.py']
ARMS = ('se2_supported', 'translation_supported', 'se2_all_captured', 'translation_all_captured')


def freeze():
    guard()
    if PROTOCOL.exists() or OUT.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the frozen study; no refreeze')
    old = read(OLD)
    batch = ROOT/old['run_dir']
    manifest = read(batch/'manifest.json')
    identity = dict(schema='ccta-study-v2', protocol_sha256=sha(OLD), split='development',
        source_sha256=old['frozen_source_sha256'], environment=old['environment'])
    assert manifest['identity'] == identity and manifest['source_files'] == old['source_files']
    assert encode(old['source_files']) == identity['source_sha256']
    verify_source_snapshot(batch, old['source_files'])
    source = source_hashes()
    for name in ('src/dbact/sensing/sensor.py', 'src/dbact/geometry.py',
                 'src/dbact/provenance.py', 'src/dbact/cargo.py', 'src/dbact/types.py'):
        assert source[name] == old['source_files'][name], name
    protected = read(PROTECTED)
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    cases = [c for c in old['cases'] if c['arm'] == 'task_reference']
    assert len(cases) == 4
    receipts = {}
    for case in cases:
        folder = batch/case['case_id']
        result = cached_result(folder, {**identity, 'case_sha256': encode(case)})
        assert result['execution_status'] == 'COMPLETED' and result['initial_reference']['zero_epoch_valid']
        assert result['initial_reference']['pre_epoch_contact_samples'] == 0
        assert result['initial_reference']['max_pre_epoch_translation'] == result['initial_reference']['max_pre_epoch_rotation'] == 0
        receipts[case['case_id']] = sha(folder/'case_result.json')
    plan = dict(version='task-translation-v2', status='FROZEN_BEFORE_EXECUTION',
        reconstruction_correction='Preserve invalidated v1; use controller distance*direction, not goal_point-start rounding. Same arms/cases/gates/budget.',
        run_dir=OUT.relative_to(ROOT).as_posix(), original_run_dir=old['run_dir'], cases=cases,
        arms=list(ARMS), selected_candidate='translation_all_captured',
        original_identity=identity, original_protocol_sha256=sha(OLD), original_source_files=old['source_files'],
        receipts=receipts, source_files=source, environment=environment_versions(),
        scripts={n: sha(ROOT/n) for n in SCRIPTS}, design_sha256=sha(DESIGN),
        protected_baseline_sha256=sha(PROTECTED), workers=1, internal_threads=1, wall_budget_seconds=3600,
        admission=dict(paired_availability_min=.9, paired_mae_max_m=.03, paired_mae_increase_max_m=.01,
            false_goal_stop_frames_max=0, consecutive_valid_potential_stop_frames_min=40),
        scope='Four reused trajectories, four estimator arms; no physical episodes or controller adoption')
    OUT.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        dest = OUT/'source_snapshot'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, dest)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, OUT/'protocol.json')
    shutil.copy2(DESIGN, OUT/'design.md')
    print(json.dumps(dict(status=plan['status'], cases=4, arms=4, source_sha256=encode(source))))


def make_tracker(p, q, translation):
    cls = TranslationTaskReferenceTracker if translation else TaskReferenceTracker
    return cls(reference_point=q, gate=p.registration_gate, normal_cosine=p.registration_normal_cosine,
        max_speed=p.max_object_speed, max_yaw_rate=p.max_object_yaw_rate,
        velocity_filter=p.object_velocity_filter, range_noise_std=p.range_noise_std, support_gate=True)


def error_stats(values):
    if not len(values):
        return dict(frames=0, mae_m=None, rmse_m=None, p95_m=None, max_m=None)
    return dict(frames=len(values), mae_m=float(values.mean()), rmse_m=float(np.sqrt(np.mean(values**2))),
        p95_m=float(np.quantile(values, .95)), max_m=float(values.max()))


def longest_streak(mask):
    longest = current = 0
    for value in mask:
        current = current+1 if value else 0
        longest = max(longest, current)
    return longest


def offline_case(case, folder, dest, deadline):
    receipt = read(folder/'case_result.json')['result']
    steps = read(folder/'paired_metrics.json')['steps']
    initial, old_templates = receipt['initial_reference'], receipt['reference_initializations']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        times, robots, xy, angles, start, goal, ids, task_direction, task_distance = [z[k].copy() for k in (
            'times', 'agent_positions', 'cargo/cargo_0/centers', 'cargo/cargo_0/angles',
            'task/cargo_0/start', 'task/cargo_0/goal_point', 'agent_ids',
            'task/cargo_0/direction', 'task/cargo_0/distance')]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c, p = env.controller, env.controller.params
    assert p.communication_dropout_prob == 0 and p.perception_every == p.planning_every == 1
    assert _REFERENCE_CHANNELS[p.navigation_estimator] == (True, True)
    epoch = int(round(initial['epoch']/env.dt))
    assert next(k for k, s in enumerate(steps) if s['initial_reference']['status'] == 'frozen') == epoch
    assert len(times) == len(steps)+1
    np.testing.assert_allclose(times[:-1], np.arange(len(steps))*env.dt, atol=1e-8, rtol=0)
    np.testing.assert_array_equal(xy[:epoch+1], np.repeat(xy[:1], epoch+1, axis=0))
    np.testing.assert_array_equal(angles[:epoch+1], np.repeat(angles[:1], epoch+1))
    np.testing.assert_array_equal(start, xy[0])
    q, workspace = c._task_reference_point, c._center_vertices
    task = c.tasks['cargo_0']
    np.testing.assert_array_equal(task.direction, task_direction)
    assert task.distance == float(task_distance)
    # Match the controller's declared-input operation exactly. Subtracting
    # rounded world coordinates would introduce a different floating-point path.
    target = task.distance*task.direction
    capture = InitialReferenceCapture(noise=p.range_noise_std)
    trackers = [{} for _ in ARMS]
    n, na = len(steps), len(ids)
    positions = np.full((4, n, na, 2), np.nan)
    fitted_angles = np.full((4, n, na), np.nan)
    reasons = np.full((4, n, na), 'before_freeze', dtype='<U40')
    estimates = np.full((4, n, 2), np.nan)
    velocities = estimates.copy()
    bounds = np.full((4, n), np.nan)
    voters = np.zeros((4, n), dtype=int)
    potential = np.zeros((4, n), dtype=bool)
    ready = next(k for k, s in enumerate(steps) if s['phase'] == 'CONTACT_READY')
    exact_status = exact_reading = exact_navigation = count_checks = 0
    templates = {}
    for k, step in enumerate(steps):
        if time.perf_counter() > deadline:
            raise TimeoutError('Frozen offline wall budget exhausted')
        # Evaluator truth only reconstructs the same sensor. Tracker inputs are
        # measured scans and the already-declared common reference point.
        env.cargoes[0].set_pose(xy[k], angles[k])
        for i, agent in enumerate(env.agents):
            assert agent.agent_id == ids[i]
            agent.position = robots[k, i].copy()
        scans = [c.sensor.sense_view(a, env.cargoes, times[k]) for a in env.agents]
        assert [len(s) for s in scans] == [d['own_scan_points'] for d in step['diagnostics']]
        count_checks += na
        neighbors = c._neighbor_indices(env.agents)
        batches = {agent.agent_id: _merge_scans(scans, [i]+neighbors[i])[0] for i, agent in enumerate(env.agents)}
        if k <= epoch:
            for aid, batch in batches.items():
                capture.add(aid, batch)
            supported = [str(aid) for aid in ids if capture.supported(str(aid))]
            if k < epoch:
                assert len(supported) < p.min_push_agents
                continue
            assert supported == initial['supporters']
            capture.freeze()
            for a, arm in enumerate(ARMS):
                for aid in (supported if a < 2 else ids.tolist()):
                    tr = make_tracker(p, q, a%2 == 1)
                    tr.initialize_initial(capture.view(aid), times[k])
                    trackers[a][aid] = tr
                    if a == 0:
                        assert tr.initialization_record() == old_templates[aid], (case['case_id'], aid, 'template')
                templates[arm] = {aid: tr.initialization_record() for aid, tr in trackers[a].items()}
            dump(dest/'templates.json', templates)
        for a, arm in enumerate(ARMS):
            displacement, velocity, local_bounds = {}, {}, []
            for i, aid in enumerate(ids.tolist()):
                tr = trackers[a].get(aid)
                if tr is None:
                    reasons[a, k, i] = 'not_selected'
                    continue
                reading = tr.update(batches[aid], times[k]) if k > epoch and tr.initialized else None
                reasons[a, k, i] = tr.reason
                diagnostic = step['diagnostics'][i]
                if a == 0:
                    assert tr.reason == diagnostic['reference_status'], (case['case_id'], k, aid, tr.reason)
                    exact_status += 1
                    if reading is not None:
                        np.testing.assert_array_equal(reading.displacement, diagnostic['reference_displacement'])
                        np.testing.assert_array_equal(reading.velocity, diagnostic['reference_velocity'])
                        assert reading.angle == diagnostic['reference_angle']
                        assert reading.residual_rms == diagnostic['reference_residual_rms']
                        assert reading.matches == diagnostic['reference_matches']
                        exact_reading += 1
                if reading is not None:
                    positions[a, k, i], fitted_angles[a, k, i] = reading.displacement, reading.angle
                    displacement[aid], velocity[aid] = reading.displacement, reading.velocity
                    local_bounds.append(center_error_bound(reading.displacement, reading.angle, q, workspace, target))
            nav = point_goal_feedback(displacement, velocity, ids.tolist(), target,
                                      p.cross_track_deadband, rotation_invariant=True)
            if a == 0:
                old_nav = step['navigation_supervisor']
                assert (nav is None) == (old_nav is None), (case['case_id'], k, 'nav availability')
                if nav is not None:
                    for key in ('displacement', 'velocity', 'error'):
                        np.testing.assert_array_equal(getattr(nav, key), old_nav[key])
                    assert list(nav.voters) == old_nav['voters'] and nav.remaining == old_nav['remaining']
                    exact_navigation += 1
            if nav is not None:
                estimates[a, k], velocities[a, k], voters[a, k] = nav.displacement, nav.velocity, len(nav.voters)
                bounds[a, k] = max(local_bounds)
                if a == 0:
                    assert bounds[a, k] == step['initial_reference']['center_error_bound']
                potential[a, k] = (k > ready and nav.remaining <= 0
                    and np.linalg.norm(nav.velocity) <= c.phase_monitor.gates.hold_speed
                    and len(nav.voters) >= p.min_push_agents and bounds[a, k] <= case['config']['task']['tolerance'])
        if k and k%2000 == 0:
            print(case['case_id'], 'frames', k, '/', n, flush=True)
    truth, window = xy[:n]-start, np.arange(n) > ready
    true_error = np.linalg.norm(xy[:n]-goal, axis=1)
    available = np.isfinite(estimates).all(axis=2)
    error = np.linalg.norm(estimates-truth[None, :, :], axis=2)
    rows = {}
    for a, arm in enumerate(ARMS):
        own = window & available[a]
        paired = window & available[0] & available[a]
        valid = potential[a] & (true_error <= case['config']['task']['tolerance'])
        active = list(trackers[a])
        active_indices = [ids.tolist().index(aid) for aid in active]
        rows[arm] = dict(initialized=sum(t.initialized for t in trackers[a].values()),
            selected=len(active), post_ready_frames=int(window.sum()), available_frames=int(own.sum()),
            missing_team_frames=int((window & ~available[a]).sum()),
            quorum_frames=int((window & (voters[a] >= p.min_push_agents)).sum()),
            insufficient_quorum_frames=int((window & (voters[a] < p.min_push_agents)).sum()),
            paired_availability=float(paired.sum()/max(1, (window & available[0]).sum())),
            paired_error=error_stats(error[a, paired]), paired_original_error=error_stats(error[0, paired]),
            own_available_error=error_stats(error[a, own]),
            potential_stop_frames=int(potential[a].sum()), false_goal_stop_frames=int((potential[a] & ~valid).sum()),
            longest_valid_potential_stop_streak=longest_streak(valid),
            statuses_all_agents=dict(Counter(reasons[a, window].ravel().tolist())),
            statuses_selected_agents=dict(Counter(reasons[a, window][:, active_indices].ravel().tolist())))
    result = dict(condition=case['condition_id'], case_id=case['case_id'], arms=rows,
        acquisition_epoch_s=initial['epoch'], original_supporters=initial['supporters'],
        acquisition_history=dict(frames=epoch+1, captured_points={str(aid): len(capture.view(str(aid))) for aid in ids}),
        true_rotation_radians=dict(min=float((angles[:n]-angles[0]).min()), max=float((angles[:n]-angles[0]).max())),
        exact_original_statuses=exact_status, exact_original_readings=exact_reading,
        exact_original_navigation=exact_navigation, scan_counts_verified=count_checks,
        scope='Reused input/control trajectory; translation yaw is a hypothesis, not measured physical yaw')
    np.savez_compressed(dest/'series.npz', times=times[:n], truth=truth, true_angle=angles[:n]-angles[0],
        estimates=estimates, velocities=velocities, local_displacement=positions,
        fitted_angles=fitted_angles, reasons=reasons, voters=voters, bounds=bounds, potential_stop=potential)
    result['series_sha256'], result['templates_sha256'] = sha(dest/'series.npz'), sha(dest/'templates.json')
    dump(dest/'summary.json', result)
    print(case['case_id'], json.dumps({a: {k: r[k] for k in ('initialized', 'paired_error',
        'false_goal_stop_frames', 'longest_valid_potential_stop_streak')} for a, r in rows.items()}), flush=True)
    return result


def run():
    guard()
    limit_worker_threads()
    plan = read(PROTOCOL)
    assert sha(PROTOCOL) == sha(OUT/'protocol.json')
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert sha(OLD) == plan['original_protocol_sha256'] and sha(DESIGN) == plan['design_sha256']
    assert {n: sha(ROOT/n) for n in SCRIPTS} == plan['scripts']
    assert sha(PROTECTED) == plan['protected_baseline_sha256']
    protected = read(PROTECTED)
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    verify_source_snapshot(ROOT/plan['original_run_dir'], plan['original_source_files'])
    if (OUT/'started.json').exists() or PUBLIC.exists():
        raise RuntimeError('Preserve every started study')
    dump(OUT/'started.json', dict(protocol_sha256=sha(PROTOCOL), environment=environment_versions()))
    started = time.perf_counter()
    rows = []
    for case in plan['cases']:
        folder = ROOT/plan['original_run_dir']/case['case_id']
        assert sha(folder/'case_result.json') == plan['receipts'][case['case_id']]
        cached_result(folder, {**plan['original_identity'], 'case_sha256': encode(case)})
        dest = OUT/case['case_id']
        dest.mkdir(exist_ok=False)
        rows.append(offline_case(case, folder, dest, started+plan['wall_budget_seconds']))
        gc.collect()
    gates, admission = {}, plan['admission']
    for row in rows:
        candidate = row['arms'][plan['selected_candidate']]
        mae, old_mae = candidate['paired_error']['mae_m'], candidate['paired_original_error']['mae_m']
        gates[row['condition']] = dict(availability=candidate['paired_availability'] >= admission['paired_availability_min'],
            error=mae is not None and mae <= admission['paired_mae_max_m'] and mae <= old_mae+admission['paired_mae_increase_max_m'],
            false_goal_stop=candidate['false_goal_stop_frames'] <= admission['false_goal_stop_frames_max'],
            sustained_feedback=candidate['longest_valid_potential_stop_streak'] >= admission['consecutive_valid_potential_stop_frames_min'])
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert {n: sha(ROOT/n) for n in SCRIPTS} == plan['scripts']
    result = dict(scope=plan['scope'], new_physical_episodes=0,
        physical_trial_screen='PASS' if all(all(v.values()) for v in gates.values()) else 'FAIL',
        cases=rows, gates=gates, wall_seconds=time.perf_counter()-started, source_integrity='PASS',
        source_sha256=encode(plan['source_files']), protocol_sha256=sha(PROTOCOL), environment=plan['environment'],
        products={p.relative_to(OUT).as_posix(): sha(p) for p in OUT.glob('*/*') if p.is_file() and 'source_snapshot' not in p.parts})
    dump(OUT/'analysis.json', result)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    dump(PUBLIC/'analysis.json', result)
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    print(json.dumps(dict(status=result['physical_trial_screen'], gates=gates)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'run'])
    globals()[parser.parse_args().action]()
