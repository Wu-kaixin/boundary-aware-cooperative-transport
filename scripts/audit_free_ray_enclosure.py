"""Freeze and audit measured free-ray enclosures on four existing snapshots.

No physics is advanced. Truth reconstructs the original scanner and scores the
result, but is never passed to the enclosure component. All-team packets are
an evaluator-only information upper bound, not one-hop execution.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import gc
import json
import shutil
import time

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from research_io import ROOT, guard, read, sha
from dbact.control.initial_task_reference import center_error_bound, workspace_vertices
from dbact.sensing.free_space import free_ray_enclosure, measured_free_ray_packet
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import (
    cached_result, environment_versions, limit_worker_threads, source_hashes, verify_source_snapshot)

OLD = ROOT/'experiments/ccta2027/task_reference_v1.json'
DESIGN = ROOT/'experiments/ccta2027/free_ray_enclosure_v1_design.md'
PROTOCOL = ROOT/'experiments/ccta2027/free_ray_enclosure_v1.json'
OUT = ROOT/'runs/research/2026-11-03-free-ray-enclosure-v1'
PUBLIC = ROOT/'docs/results/free_ray_enclosure_v1'
PROTECTED = ROOT/'docs/results/reference_surface_v1/protected_baseline.json'
SCRIPTS = ['scripts/audit_free_ray_enclosure.py', 'scripts/research_io.py']
SENSOR_SOURCES = ['src/dbact/sensing/sensor.py', 'src/dbact/geometry.py',
    'src/dbact/provenance.py', 'src/dbact/cargo.py', 'src/dbact/types.py']


def freeze():
    guard()
    if PROTOCOL.exists() or OUT.exists() or PUBLIC.exists():
        raise RuntimeError('Preserve the frozen audit; use a new directory')
    old = read(OLD)
    batch = ROOT/old['run_dir']
    manifest = read(batch/'manifest.json')
    expected = dict(schema='ccta-study-v2', protocol_sha256=sha(OLD), split='development',
        source_sha256=old['frozen_source_sha256'], environment=old['environment'])
    assert manifest['identity'] == expected
    assert manifest['source_files'] == old['source_files']
    assert encode(old['source_files']) == expected['source_sha256']
    verify_source_snapshot(batch, old['source_files'])
    source = source_hashes()
    for name in SENSOR_SOURCES:
        assert source[name] == old['source_files'][name], name
    protected = read(PROTECTED)
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    cases = [c for c in old['cases'] if c['arm'] == 'task_reference']
    assert len(cases) == 4
    snapshots = {}
    for case in cases:
        folder = batch/case['case_id']
        result = cached_result(folder, {**expected, 'case_sha256': encode(case)})
        initial = result['initial_reference']
        assert result['execution_status'] == 'COMPLETED'
        assert initial['status'] == 'frozen' and initial['zero_epoch_valid']
        assert initial['pre_epoch_contact_samples'] == 0
        assert initial['max_pre_epoch_translation'] == initial['max_pre_epoch_rotation'] == 0.
        snapshots[case['case_id']] = dict(epoch=initial['epoch'],
            control_index=int(round(initial['epoch']/case['config']['dt'])),
            receipt_sha256=sha(folder/'case_result.json'), zero_epoch=initial)
    plan = dict(version='free-ray-enclosure-v1', status='FROZEN_BEFORE_EXECUTION',
        scope='Four reused acquisition snapshots; zero new physical episodes',
        run_dir=OUT.relative_to(ROOT).as_posix(), original_run_dir=old['run_dir'],
        cases=cases, snapshots=snapshots, original_identity=expected, original_protocol_sha256=sha(OLD),
        original_source_files=old['source_files'], source_files=source,
        environment=environment_versions(), scripts={n: sha(ROOT/n) for n in SCRIPTS},
        design_sha256=sha(DESIGN), protected_baseline_sha256=sha(PROTECTED),
        margin_source='Existing controller carve_margin (0.06 m); no tuning',
        information_arms=['own', 'one_hop', 'all_team_evaluator_only'],
        evaluation='All agents including missing/inconsistent/unbounded packets; full-object and center containment',
        stop_counterfactual='Every old basic-stop-eligible BRAKE frame plus final frame; same transforms, no success relabeling',
        workers=1, internal_threads=1, wall_budget_seconds=1800)
    OUT.mkdir(parents=True, exist_ok=False)
    for name in [*source, *SCRIPTS]:
        dest = OUT/'source_snapshot'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, dest)
    dump(PROTOCOL, plan)
    shutil.copy2(PROTOCOL, OUT/'protocol.json')
    shutil.copy2(DESIGN, OUT/'design.md')
    shutil.copy2(PROTECTED, OUT/'protected_baseline.json')
    print(json.dumps(dict(status=plan['status'], cases=4, source_sha256=encode(source))))


def diameter(vertices):
    return float(np.linalg.norm(vertices[:, None]-vertices[None, :], axis=2).max())


def audit_case(case, snapshot, folder, dest, deadline):
    content = read(folder/'paired_metrics.json')
    steps = content['steps']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        times, robots, xy, angles, start, goal, agent_ids = [z[k].copy() for k in (
            'times', 'agent_positions', 'cargo/cargo_0/centers', 'cargo/cargo_0/angles',
            'task/cargo_0/start', 'task/cargo_0/goal_point', 'agent_ids')]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    controller, p = env.controller, env.controller.params
    assert p.communication_dropout_prob == 0, 'This audit does not reconstruct communication dropout'
    assert p.carve_margin == .06
    k, epoch = snapshot['control_index'], snapshot['epoch']
    assert len(times) == len(steps)+1 and times[0] == 0
    np.testing.assert_allclose(times[:-1], np.arange(len(steps))*env.dt, atol=1e-8, rtol=0)
    assert abs(times[k]-epoch) < 1e-8 and steps[k]['frame'] == k+1
    assert next(j for j, s in enumerate(steps) if s['initial_reference']['status'] == 'frozen') == k
    np.testing.assert_array_equal(xy[:k+1], np.repeat(xy[:1], k+1, axis=0))
    np.testing.assert_array_equal(angles[:k+1], np.repeat(angles[:1], k+1))
    env.cargoes[0].set_pose(xy[k], angles[k])
    for i, agent in enumerate(env.agents):
        assert agent.agent_id == agent_ids[i]
        agent.position = robots[k, i].copy()
    scans = [controller.sensor.sense_view(a, env.cargoes, times[k]) for a in env.agents]
    counts = [len(s) for s in scans]
    assert counts == [d['own_scan_points'] for d in steps[k]['diagnostics']]
    neighbors = controller._neighbor_indices(env.agents)
    domain = case['config']['domain']
    workspace = workspace_vertices([domain[n] for n in ('xmin', 'xmax', 'ymin', 'ymax')])
    reference_point, target = workspace.mean(axis=0), goal-start
    true_object = Polygon(env.cargoes[0].vertices)  # Evaluator only, below the input boundary.
    packets, packet_receipts = [], []
    for agent in env.agents:
        raw = controller.sensor.raw_returns(agent, env.cargoes, times[k])
        packet = measured_free_ray_packet(agent.position, raw,
            ray_count=controller.sensor.params.ray_count, sensor_range=controller.sensor.params.sensor_range,
            margin=p.carve_margin, target_id='cargo_0')
        packets.append(packet)
        packet_receipts.append(dict(agent_id=agent.agent_id, raw_returns=len(raw),
            starts=packet[0].tolist(), ends=packet[1].tolist(), target_points=packet[2].tolist()))
    dump(dest/'sanitized_packets.json', dict(timestamp=epoch, packets=packet_receipts,
        schema='Measured origin / calibrated direction / range / object label only; no simulator normals'))
    records, sets = [], {arm: {} for arm in ('own', 'one_hop', 'all_team_evaluator_only')}
    groups = [(arm, i, indices) for i in range(len(env.agents))
        for arm, indices in [('own', [i]), ('one_hop', [i]+neighbors[i])]]
    groups.append(('all_team_evaluator_only', None, list(range(len(env.agents)))))
    for arm, i, indices in groups:
        if time.perf_counter() > deadline:
            raise TimeoutError('Frozen audit wall budget exhausted')
        starts, ends, points = [np.concatenate([packets[j][n] for j in indices]) for n in range(3)]
        enclosure = free_ray_enclosure(starts, ends, points, workspace, margin=p.carve_margin)
        hull = Polygon(enclosure.vertices)
        aid = None if i is None else env.agents[i].agent_id
        # These scores NEVER affect the returned set or the simulated control.
        object_contained = hull.buffer(1e-10).covers(true_object)
        center_contained = hull.buffer(1e-10).covers(Point(xy[k]))
        intersecting_rays = sum(LineString([a, b]).intersects(true_object)
            for a, b in zip(starts, ends) if not np.array_equal(a, b))
        record = dict(arm=arm, agent_id=aid, packet_agents=[env.agents[j].agent_id for j in indices],
            target_returns=len(points), free_rays=len(starts), **asdict(enclosure))
        record['vertices'] = enclosure.vertices.tolist()
        record.update(area_ratio=enclosure.center_area/Polygon(workspace).area,
            diameter_m=diameter(enclosure.vertices), diameter_ratio=diameter(enclosure.vertices)/diameter(workspace),
            true_object_contained=bool(object_contained), true_center_contained=bool(center_contained),
            evaluator_free_ray_intersections=int(intersecting_rays),
            usable_in_this_snapshot=bool(object_contained and center_contained and not intersecting_rays))
        records.append(record)
        if aid is None:
            sets[arm] = {a.agent_id: enclosure.vertices for a in env.agents}
        else:
            sets[arm][aid] = enclosure.vertices
    dump(dest/'enclosures.json', records)
    eligible = [s for s in steps if s['phase'] == 'BRAKE' and s['navigation_supervisor']
        and s['navigation_supervisor']['remaining'] <= 0
        and np.linalg.norm(s['navigation_supervisor']['velocity']) <= .02]

    def stop_bounds(step):
        nav = step['navigation_supervisor']
        voters = [] if nav is None else nav['voters']
        diagnostics = {d['agent_id']: d for d in step['diagnostics']}
        all_sets = {'workspace': {aid: workspace for aid in voters}, **sets}
        bounds = {}
        for arm, by_agent in all_sets.items():
            values = [center_error_bound(diagnostics[aid]['reference_displacement'],
                diagnostics[aid]['reference_angle'], reference_point, by_agent[aid], target) for aid in voters]
            bounds[arm] = max(values) if values else None
        old_bound = step['initial_reference']['center_error_bound']
        if bounds['workspace'] is not None:
            assert np.isclose(bounds['workspace'], old_bound, atol=1e-10, rtol=1e-10)
        return dict(frame=step['frame'], phase=step['phase'], voters=voters,
            quorum=len(voters) >= p.min_push_agents,
            true_error_m=float(np.linalg.norm(xy[step['frame']-1]-goal)), bounds_m=bounds)

    counters = {arm: dict(bound_pass_frames=0, bound_and_quorum_frames=0,
        bound_pass_outside_task_tolerance=0, minimum_bound_m=None) for arm in ('workspace', *sets)}
    first, first_pass = None, {arm: None for arm in counters}
    for step in eligible:
        if time.perf_counter() > deadline:
            raise TimeoutError('Frozen audit wall budget exhausted')
        row = stop_bounds(step)
        if first is None:
            first = row
        for arm, value in row['bounds_m'].items():
            if value is None:
                continue
            c = counters[arm]
            c['minimum_bound_m'] = value if c['minimum_bound_m'] is None else min(value, c['minimum_bound_m'])
            if value <= case['config']['task']['tolerance']:
                c['bound_pass_frames'] += 1
                c['bound_and_quorum_frames'] += int(row['quorum'])
                c['bound_pass_outside_task_tolerance'] += int(row['true_error_m'] > case['config']['task']['tolerance'])
                if row['quorum'] and first_pass[arm] is None:
                    first_pass[arm] = row
    summary = dict(case_id=case['case_id'], condition=case['condition_id'], epoch=epoch,
        scan_count_checks=len(counts), original_scan_counts=counts, zero_epoch=snapshot['zero_epoch'],
        sensor_parameters=asdict(controller.sensor.params), communication_range_m=p.comm_range,
        sanitized_packet_sha256=sha(dest/'sanitized_packets.json'),
        arms={arm: dict(packets=len(selected := [r for r in records if r['arm'] == arm]),
            status_counts=dict(Counter(r['status'] for r in selected)),
            object_exclusions=sum(not r['true_object_contained'] for r in selected),
            center_exclusions=sum(not r['true_center_contained'] for r in selected),
            intersecting_free_rays=sum(r['evaluator_free_ray_intersections'] for r in selected),
            area_ratio_min=float(min(r['area_ratio'] for r in selected)),
            area_ratio_max=float(max(r['area_ratio'] for r in selected))) for arm in sets},
        stop_counterfactual=dict(scope='Original controls/transforms only; these are NOT new successes',
            basic_eligible_frames=len(eligible), counters=counters, first_basic=first,
            first_bound_and_quorum_pass=first_pass, final=stop_bounds(steps[-1])))
    dump(dest/'summary.json', summary)
    return summary


def run():
    guard()
    limit_worker_threads()
    plan = read(PROTOCOL)
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert sha(DESIGN) == plan['design_sha256'] and sha(OLD) == plan['original_protocol_sha256']
    assert {n: sha(ROOT/n) for n in SCRIPTS} == plan['scripts']
    assert sha(PROTECTED) == plan['protected_baseline_sha256']
    protected = read(PROTECTED)
    assert sha(ROOT/protected['canonical_config']) == protected['canonical_sha256']
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    verify_source_snapshot(ROOT/plan['original_run_dir'], plan['original_source_files'])
    if (OUT/'started.json').exists() or PUBLIC.exists():
        raise RuntimeError('Do not overwrite any started audit')
    dump(OUT/'started.json', dict(protocol_sha256=sha(PROTOCOL), environment=environment_versions()))
    started = time.perf_counter()
    deadline, summaries = started+plan['wall_budget_seconds'], []
    for case in plan['cases']:
        folder = ROOT/plan['original_run_dir']/case['case_id']
        snapshot = plan['snapshots'][case['case_id']]
        assert sha(folder/'case_result.json') == snapshot['receipt_sha256']
        cached_result(folder, {**plan['original_identity'], 'case_sha256': encode(case)})
        dest = OUT/case['case_id']
        dest.mkdir(exist_ok=False)
        summaries.append(audit_case(case, snapshot, folder, dest, deadline))
        print(case['case_id'], json.dumps(summaries[-1]['arms']), flush=True)
        gc.collect()
    assert source_hashes() == plan['source_files'] and environment_versions() == plan['environment']
    assert {n: sha(ROOT/n) for n in SCRIPTS} == plan['scripts']
    verify_source_snapshot(OUT, {**plan['source_files'], **plan['scripts']})
    output = dict(scope=plan['scope'], protocol_sha256=sha(PROTOCOL), source_sha256=encode(plan['source_files']),
        source_integrity='PASS', environment=plan['environment'], physical_episodes=0,
        wall_seconds=time.perf_counter()-started, cases=summaries,
        products={p.relative_to(OUT).as_posix(): sha(p) for p in sorted(OUT.glob('*/*.json'))})
    dump(OUT/'summary.json', output)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    dump(PUBLIC/'analysis.json', output)
    shutil.copy2(PROTOCOL, PUBLIC/'protocol.json')
    for case in plan['cases']:
        shutil.copy2(OUT/case['case_id']/'enclosures.json', PUBLIC/(case['case_id']+'_enclosures.json'))
    print(json.dumps(dict(status='COMPLETE', snapshots=4, physical_episodes=0,
        wall_seconds=output['wall_seconds'], public=PUBLIC.relative_to(ROOT).as_posix())))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'run'])
    globals()[parser.parse_args().action]()
