"""Frozen same-input evaluation of reference surfaces; no physics is advanced."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import gc
import json
import shutil
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from audit_reference_motion import local_terms
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker
from dbact.control.reference_surface import SurfaceReferenceTracker
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import (cached_result, environment_versions,
    limit_worker_threads, source_hashes, verify_source_snapshot)

OLD = ROOT/'experiments/ccta2027/reference_origin_v1.json'
DESIGN = ROOT/'experiments/ccta2027/reference_surface_v1_design.md'
PROTOCOL = ROOT/'experiments/ccta2027/reference_surface_v1.json'
PROTECTED = ROOT/'runs/maintenance/2026-11-03-reference-surface-v1/protected_baseline.json'
OUT = ROOT/'runs/research/2026-11-03-reference-surface-v1'
SCRIPTS = ['scripts/reference_surface_study.py', 'scripts/research_io.py',
           'scripts/audit_reference_motion.py']


def original_identity(plan, manifest):
    expected = dict(schema='ccta-study-v2', protocol_sha256=sha(OLD), split='development',
                    source_sha256=encode(manifest['source_files']), environment=plan['environment'])
    assert expected == manifest['identity']
    return expected


def freeze():
    guard()
    if PROTOCOL.exists() or OUT.exists():
        raise RuntimeError('Preserve existing protocol/output; no refreeze')
    old, protected = read(OLD), read(PROTECTED)
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    cases = [c for c in old['cases'] if c['arm'] == 'reference']
    assert len(cases) == 8
    batch = ROOT/old['run_dir']
    manifest = read(batch/'manifest.json')
    identity = original_identity(old, manifest)
    verify_source_snapshot(batch, manifest['source_files'])
    receipts = {}
    for case in cases:
        params = asdict(SimulationEnvironment(case['config'], seed=case['seed']).controller.params)
        assert not params.pop('reference_surface_refinement')
        assert params == protected['effective_controller'], case['case_id']
        folder = batch/case['case_id']
        cached_result(folder, {**identity, 'case_sha256': encode(case)})
        receipts[case['case_id']] = sha(folder/'case_result.json')
    source = source_hashes()
    plan = dict(version='reference-surface-v1', status='FROZEN_BEFORE_EXECUTION',
        type='same-input offline estimator comparison; no new physical episodes',
        run_dir=OUT.relative_to(ROOT).as_posix(), cases=cases, original_run_dir=old['run_dir'],
        original_identity=identity, original_protocol_sha256=sha(OLD), receipts=receipts,
        source_files=source, environment=environment_versions(), design_sha256=sha(DESIGN),
        scripts={name: sha(ROOT/name) for name in SCRIPTS},
        protected_baseline_sha256=sha(PROTECTED), wall_budget_seconds=1800,
        admission=dict(availability_min=.9, far_skew_mae_ratio_max=.75,
                       other_mae_increase_max_m=.01, reject_old_false_hold=True))
    OUT.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        dest = OUT/'source_snapshot'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, dest)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, OUT/'protocol.json')
    shutil.copy2(PROTECTED, OUT/'protected_baseline.json')
    shutil.copy2(DESIGN, OUT/'design.md')
    print(json.dumps(dict(status=plan['status'], cases=len(cases), source_sha256=encode(source))))


def tracker(p, candidate):
    cls, extra = ObservedReferenceTracker, {}
    if candidate:
        cls, extra = SurfaceReferenceTracker, dict(surface_residual_tolerance=p.residual_tolerance)
    return cls(gate=p.registration_gate, normal_cosine=p.registration_normal_cosine,
        max_speed=p.max_object_speed, max_yaw_rate=p.max_object_yaw_rate,
        velocity_filter=p.object_velocity_filter, radial_readout=p.reference_radial_pivot,
        range_noise_std=p.range_noise_std, innovation_trim=p.reference_innovation_trim,
        support_gate=p.reference_support_gate, **extra)


def offline_case(case, folder, dest, deadline):
    content = read(folder/'paired_metrics.json')
    steps, meta = content['steps'], content['summary']['reference_initializations']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        times, robots, xy, angles, start = [z[k].copy() for k in (
            'times', 'agent_positions', 'cargo/cargo_0/centers',
            'cargo/cargo_0/angles', 'task/cargo_0/start')]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c, p = env.controller, env.controller.params
    birth = {aid: int(round(m['time']/env.dt)) for aid, m in meta.items() if m['initialized']}
    n, na = len(steps), len(env.agents)
    assert times[0] == 0 and len(times) == n+1
    np.testing.assert_allclose(times[:n], np.arange(n)*env.dt, atol=1e-8, rtol=0)
    trackers, templates = [{}, {}], {}
    positions = np.full((2, n, na, 2), np.nan)
    fitted_angles = np.full((2, n, na), np.nan)
    reasons = np.full((2, n, na), 'before_bootstrap', dtype='<U40')
    estimates = np.full((2, n, 2), np.nan)
    velocities = estimates.copy()
    states = exact = count = 0
    ready = next(k for k, s in enumerate(steps) if s['phase'] == 'CONTACT_READY')
    for k, step in enumerate(steps):
        if time.perf_counter() > deadline:
            raise TimeoutError('Frozen offline wall budget exhausted')
        # Evaluator truth reconstructs the *same* scans. Trackers receive only
        # those scans, original bootstrap offsets and observation timestamps.
        env.cargoes[0].set_pose(xy[k], angles[k])
        for i, agent in enumerate(env.agents):
            agent.position = robots[k, i].copy()
        scans = [c.sensor.sense_view(a, env.cargoes, times[k]) for a in env.agents]
        neighbors = c._neighbor_indices(env.agents)
        assert [len(s) for s in scans] == [d['own_scan_points'] for d in step['diagnostics']]
        count += na
        votes, speeds = [[], []], [[], []]
        for i, agent in enumerate(env.agents):
            aid, diagnostic = agent.agent_id, step['diagnostics'][i]
            if aid not in birth or k < birth[aid]:
                continue
            scan, _ = _merge_scans(scans, [i]+neighbors[i])
            for arm in (0, 1):
                if k == birth[aid]:
                    tr = tracker(p, bool(arm))
                    tr.initialize(scan, meta[aid]['offset'], times[k])
                    trackers[arm][aid], reading = tr, None
                    record = tr.initialization_record()
                    if arm == 0:
                        assert record == meta[aid], (case['case_id'], aid, 'template')
                    else:
                        for key in ('time', 'anchor', 'offset', 'pivot_shift', 'pivot_status'):
                            assert record[key] == meta[aid][key]
                        templates[aid] = record
                else:
                    tr = trackers[arm][aid]
                    reading = tr.update(scan, times[k])
                reasons[arm, k, i] = tr.reason
                if arm == 0:
                    assert tr.reason == diagnostic['reference_status'], (case['case_id'], k, aid)
                    states += 1
                    if reading is not None:
                        np.testing.assert_array_equal(reading.displacement, diagnostic['reference_displacement'])
                        np.testing.assert_array_equal(reading.velocity, diagnostic['reference_velocity'])
                        assert reading.angle == diagnostic['reference_angle']
                        assert reading.matches == diagnostic['reference_matches']
                        assert reading.residual_rms == diagnostic['reference_residual_rms']
                        exact += 1
                if reading is not None:
                    positions[arm, k, i], fitted_angles[arm, k, i] = reading.displacement, reading.angle
                    votes[arm].append(reading.displacement)
                    speeds[arm].append(reading.velocity)
        for arm in (0, 1):
            if votes[arm]:
                estimates[arm, k] = np.median(votes[arm], axis=0)
                velocities[arm, k] = np.median(speeds[arm], axis=0)
        if k % 500 == 0:
            print(f"{case['condition_id']}: {k}/{n}", flush=True)
    truth = xy[:n]-start
    window = np.arange(n) > ready
    available = np.isfinite(estimates).all(axis=2)
    paired = window & available.all(axis=0)
    base_frames, candidate_frames = window & available[0], window & available[1]
    error = np.linalg.norm(estimates-truth[None, :, :], axis=2)
    task = c.tasks['cargo_0']
    target = np.asarray(task.direction)*task.distance
    hold = next(k for k, s in enumerate(steps) if s['phase'] == 'HOLD')
    stop = dict(frame=hold, time_s=float(times[hold]), true_goal_error_m=float(np.linalg.norm(truth[hold]-target)),
        original_estimated_goal_error_m=float(np.linalg.norm(estimates[0, hold]-target)),
        candidate_estimated_goal_error_m=float(np.linalg.norm(estimates[1, hold]-target)) if available[1, hold] else None,
        candidate_fresh=bool(available[1, hold]), position_band_m=p.cross_track_deadband)
    bad, accepted = [0, 0], [0, 0]
    for i, agent in enumerate(env.agents):
        aid = agent.agent_id
        if aid not in birth:
            continue
        j = birth[aid]
        for arm in (0, 1):
            for k in np.flatnonzero(window & np.isfinite(positions[arm, :, i]).all(axis=1)):
                reading = dict(reference_displacement=positions[arm, k, i], reference_angle=fitted_angles[arm, k, i])
                terms = local_terms(meta[aid], reading, angles[k], xy[k], xy[j], angles[j], start)
                accepted[arm] += 1
                bad[arm] += int(np.linalg.norm(terms['fit_error']) > case['config']['task']['tolerance'])
    result = dict(case=case['case_id'], condition=case['condition_id'], frames=n, paired_frames=int(paired.sum()),
        availability_relative_to_original=float(paired.sum()/max(base_frames.sum(), 1)),
        original_same_input_mae_m=float(error[0, paired].mean()) if paired.any() else None,
        candidate_same_input_mae_m=float(error[1, paired].mean()) if paired.any() else None,
        original_own_available_mae_m=float(error[0, base_frames].mean()),
        candidate_own_available_mae_m=float(error[1, candidate_frames].mean()) if candidate_frames.any() else None,
        first_hold=stop, original_local_bad_fit_count=bad[0], candidate_local_bad_fit_count=bad[1],
        accepted_local_readouts=accepted, original_statuses_exact=states, original_readouts_exact=exact,
        scan_counts_verified=count, candidate_status_counts=dict(Counter(reasons[1, window].ravel().tolist())),
        reference_surface_diagnostics={aid: m['surface_refinement'] for aid, m in templates.items()})
    np.savez_compressed(dest.with_suffix('.npz'), time=times[:n], truth=truth, estimates=estimates,
        velocities=velocities, local_displacements=positions, local_angles=fitted_angles, reasons=reasons)
    dump(dest.with_name(dest.stem+'-templates.json'), templates)
    result['series_sha256'] = sha(dest.with_suffix('.npz'))
    result['templates_sha256'] = sha(dest.with_name(dest.stem+'-templates.json'))
    dump(dest, result)
    print(json.dumps({k: result[k] for k in ('condition', 'original_same_input_mae_m', 'candidate_same_input_mae_m',
                                            'availability_relative_to_original')}), flush=True)
    del content, steps, env
    gc.collect()
    return result


def run():
    guard()
    limit_worker_threads()
    plan = read(PROTOCOL)
    assert sha(PROTOCOL) == sha(OUT/'protocol.json')
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert sha(OLD) == plan['original_protocol_sha256'] and sha(DESIGN) == plan['design_sha256']
    assert sha(PROTECTED) == plan['protected_baseline_sha256']
    assert all(sha(ROOT/name) == value for name, value in plan['scripts'].items())
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    dest = OUT/'offline'
    dest.mkdir(exist_ok=False)
    started = time.perf_counter()
    rows = []
    for case in plan['cases']:
        folder = ROOT/plan['original_run_dir']/case['case_id']
        assert sha(folder/'case_result.json') == plan['receipts'][case['case_id']]
        cached_result(folder, {**plan['original_identity'], 'case_sha256': encode(case)})
        rows.append(offline_case(case, folder, dest/(case['case_id']+'.json'),
                                 started+plan['wall_budget_seconds']))
    gates = {}
    admission = plan['admission']
    for row in rows:
        old, new, stop = row['original_same_input_mae_m'], row['candidate_same_input_mae_m'], row['first_hold']
        far_skew = row['condition'] == 'far_skew_box_s0_r0_pos0'
        checks = dict(availability=row['availability_relative_to_original'] >= admission['availability_min'],
            same_input_error=new is not None and (new <= admission['far_skew_mae_ratio_max']*old
                if far_skew else new <= old+admission['other_mae_increase_max_m']))
        if far_skew:
            checks['reject_old_false_stop'] = stop['candidate_fresh'] and stop['candidate_estimated_goal_error_m'] > stop['position_band_m']
        gates[row['condition']] = checks
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert all(sha(ROOT/name) == value for name, value in plan['scripts'].items())
    report = dict(status='PASS' if all(all(g.values()) for g in gates.values()) else 'FAIL',
        cases=rows, gates=gates, new_physical_episodes=0, wall_seconds=time.perf_counter()-started,
        protocol_sha256=sha(PROTOCOL), source_integrity='PASS',
        scope='Eight reused trajectories; identical inputs and original baseline readings exactly reproduced. '
              'Candidate-specific estimator state/admission. No new successes or independent validation.')
    dump(OUT/'analysis.json', report)
    print(json.dumps(dict(status=report['status'], gates=gates)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'run'])
    args = parser.parse_args()
    freeze() if args.action == 'freeze' else run()
