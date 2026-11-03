"""Diagnose all 18 saved scaling states; truth substitutions never control robots."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
import gzip
import json
from pathlib import Path
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from diagnose_pair_residuals import systems
from analyze_residual_coordination import decompose
from dbact.safety.filter import SafetyFilter, SafetyFilterParams

BASE = ROOT / 'runs/research/2026-11-03-map-bridge-conflicts-v1'
REPLAY = BASE / 'replay-capture-v2'
DESIGN = ROOT / 'experiments/ccta2027/map_bridge_conflicts_v1_design.md'
GEOMETRIES = ('points_only', 'normals_only', 'points_and_normals', 'true_nearest')


def edge_geometry(position, vertices):
    """Evaluate every finite CCW edge and its unbounded supporting line."""
    p, vertices = np.asarray(position, dtype=float), np.asarray(vertices, dtype=float)
    result = []
    for index, (a, b) in enumerate(zip(vertices, np.roll(vertices, -1, axis=0))):
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length <= 1e-12:
            raise ValueError('Finite-edge diagnosis requires nondegenerate edges')
        t = float((p-a) @ delta / (length*length))
        foot = a + np.clip(t, 0., 1.) * delta
        normal = np.array([delta[1], -delta[0]]) / length
        result.append(dict(edge=index, start=a.tolist(), end=b.tolist(), normal=normal.tolist(),
            projection_t=t, projection_inside=0. <= t <= 1.,
            endpoint_excess=length*max(-t, t-1., 0.), foot=foot.tolist(),
            finite_distance=float(np.linalg.norm(p-foot)), signed_plane_distance=float(normal @ (p-a))))
    return result


def geometry_system(saved, mode):
    """Keep all hard rows and received velocities; change only stated geometry.

    For the single true-nearest row, the received per-point velocity envelope
    spans all input points. No unobserved velocity or true motion is supplied.
    Other substitutions preserve each velocity's input-point association while
    rerunning the original selection and aggregation (including any row drop).
    """
    if mode not in GEOMETRIES:
        raise ValueError(mode)
    params = SafetyFilterParams(**saved['params'])
    safety = SafetyFilter(params)
    trace = {}
    if mode == 'true_nearest':
        A = np.asarray(saved['true_normal']).reshape(1, 2)
        h = np.array([saved['true_gap']-params.r_safe])
        received = (saved['point_velocities'] if saved['point_velocities'] is not None
                    else [saved['object_velocity']])
        velocity = np.asarray(received, dtype=float).reshape(-1, 2)
        magnitude = np.linalg.norm(velocity, axis=1)
        factors = np.minimum(1., params.object_velocity_bound / np.maximum(magnitude, 1e-300))
        velocity = velocity * factors[:, None]
        motion = np.array([np.max(velocity @ A[0])])
        demand = motion - params.gamma_obj*h
        b = safety._cap_to_reachable(A, demand+params.rho)
        trace = dict(A=A.tolist(), h=h.tolist(), rhs_uncapped=(demand+params.rho).tolist(),
            received_velocity_policy='maximum projected received velocity after original per-vector bound',
            received_velocity_count=len(velocity))
    else:
        points = saved['map_true_feet'] if mode in ('points_only', 'points_and_normals') else saved['boundary_points']
        normals = saved['map_true_normals'] if mode in ('normals_only', 'points_and_normals') else saved['boundary_normals']
        A, b, _, h = safety._object_rows(saved['position'], points, normals,
            saved['object_velocity'], saved['point_velocities'], trace=trace)
        motion = np.asarray(trace.get('demand', [])) + params.gamma_obj*h
    old_A = np.asarray(saved['A'], dtype=float).reshape(-1, 2)
    hard = np.asarray(saved['kinds']) != 'object'
    return dict(A=np.vstack([old_A[hard], A]).tolist(),
        b=np.r_[np.asarray(saved['b'])[hard], b].tolist(),
        kinds=[*[kind for kind in saved['kinds'] if kind != 'object'], *(['object']*len(h))],
        h=h.tolist(), normal_motion=motion.tolist(), params=saved['params'],
        hard_original_indices=np.flatnonzero(hard).tolist(), trace=plain(trace),
        geometry_mode=mode, truth_assisted=True)


def selected_geometry(saved, vertices):
    """Nearest true edges annotate noisy map points, not presumed scan origins."""
    robot = edge_geometry(saved['position'], vertices)
    trace = saved['aggregate_trace']
    aggregate = trace.get('aggregate', {})
    face = []
    inputs = np.asarray(saved['boundary_points'])
    for point, normal, weight in zip(aggregate.get('face_points', []),
            aggregate.get('face_normals', []), aggregate.get('weights', [])):
        distances = np.linalg.norm(inputs-np.asarray(point), axis=1)
        index = int(np.argmin(distances))
        assert distances[index] < 1e-12
        edges = edge_geometry(point, vertices)
        nearest = min(e['finite_distance'] for e in edges)
        tied = [e['edge'] for e in edges if abs(e['finite_distance']-nearest) < 1e-10]
        angle = float(np.rad2deg(np.arccos(np.clip(np.asarray(normal) @ saved['map_true_normals'][index], -1., 1.))))
        face.append(dict(input_index=index, point=point, estimated_normal=normal,
            weight=weight, true_signed_gap=saved['map_true_gaps'][index],
            true_nearest_normal=saved['map_true_normals'][index], normal_error_deg=angle,
            nearest_true_edge_ids=tied, robot_relative_to_these_edges=[robot[j] for j in tied]))
    return dict(robot_edges=robot, face_points=face, aggregate=aggregate,
        inference_limit='Nearest true edge to a noisy map point is a diagnostic association, not exact ray ancestry')


def row_annotations(saved, all_agents):
    notes = []
    for index, (a, b, kind) in enumerate(zip(saved['A'], saved['b'], saved['kinds'])):
        entry = dict(index=index, kind=kind, A=a, b=b)
        if kind == 'agent':
            implied = np.asarray(saved['position']) - np.asarray(a)/2.
            matches = [key for key in saved['neighbors']
                if np.linalg.norm(implied-all_agents[key]['position']) < 1e-12]
            assert len(matches) == 1
            distance = float(np.linalg.norm(implied-saved['position']))
            expected = -.5*saved['params']['gamma_agent']*(distance**2-saved['params']['d_min']**2)
            assert abs(expected-b) < 1e-12
            entry.update(neighbor_id=matches[0], distance=distance,
                distance_above_d_min=distance-saved['params']['d_min'])
        notes.append(entry)
    return notes


def analyze(output):
    import map_bridge_study as study
    from dbact_sim.experiments.deployment import dump
    guard()
    plan = study.check()
    receipt = read(REPLAY/'outcome.json')
    assert receipt['status'] == 'EXACT_RECONSTRUCTION' and receipt['verified_frames'] == 517
    assert receipt['source_sha256'] == plan['frozen_source_sha256']
    assert sha(REPLAY/'states.jsonl.gz') == receipt['product_sha256']
    assert sha(REPLAY/'replay_script.py') == receipt['script_sha256']
    assert sha(REPLAY/'design.md') == receipt['design_sha256'] == sha(DESIGN)
    for name, expected in receipt['helpers'].items():
        assert sha(REPLAY/'source_snapshot'/name) == expected
    target = Path(output).resolve()
    if not target.is_relative_to((ROOT/'runs').resolve()):
        raise ValueError('Analysis output must be a fresh directory under runs/')
    target.mkdir(parents=True, exist_ok=False)
    helpers = {name:sha(ROOT/name) for name in ('scripts/analyze_map_bridge_conflicts.py',
        'scripts/analyze_residual_coordination.py', 'scripts/diagnose_pair_residuals.py',
        'scripts/qp_conflict_study.py', 'tests/test_map_bridge_row_analysis.py')}
    identity = dict(source_sha256=plan['frozen_source_sha256'], replay_receipt_sha256=sha(REPLAY/'outcome.json'),
        replay_product_sha256=receipt['product_sha256'], design_sha256=sha(DESIGN), helpers=helpers,
        new_physical_episodes=0, comparison_scope='18 selected states from one previously executed candidate; no new controller or physics')
    for name in helpers:
        path = target/'source_snapshot'/name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, path)
    dump(target/'started.json', identity)
    events = []
    try:
        with gzip.open(REPLAY/'states.jsonl.gz', 'rt', encoding='utf-8') as stream:
            states = [json.loads(line) for line in stream]
        assert [x['frame'] for x in states] == receipt['captured_frames']
        assert len(states) == 18 and sum(x['target']=='agent_14' for x in states) == 17
        for state in states:
            saved = state['agents'][state['target']]
            assert saved['scale'] < 1. and len(state['agents']) == 16
            cases = []
            for mode, b in systems(saved).items():
                cases.append(dict(geometry='original', mode=mode, A=saved['A'], b=b.tolist(),
                    kinds=saved['kinds'], **decompose(saved['A'], b, saved['kinds'], saved['params']['max_speed'])))
            geometry = None
            if state['target'] == 'agent_14':
                assert not saved['support'].get('applied')
                check_trace = {}
                check = SafetyFilter(SafetyFilterParams(**saved['params']))._object_rows(saved['position'],
                    saved['boundary_points'], saved['boundary_normals'], saved['object_velocity'],
                    saved['point_velocities'], trace=check_trace)
                mask = np.asarray(saved['kinds']) == 'object'
                np.testing.assert_array_equal(check[0], np.asarray(saved['A'])[mask])
                np.testing.assert_array_equal(check[1], np.asarray(saved['b'])[mask])
                np.testing.assert_array_equal(check[3], saved['h'])
                geometry = selected_geometry(saved, state['true_vertices'])
                for substitution in GEOMETRIES:
                    changed = geometry_system(saved, substitution)
                    for mode in ('capped_full', 'uncapped_full'):
                        b = systems(changed)[mode]
                        cases.append(dict(geometry=substitution, mode=mode,
                            A=changed['A'], b=b.tolist(), kinds=changed['kinds'], h=changed['h'],
                            normal_motion=changed['normal_motion'], trace=changed['trace'],
                            hard_original_indices=changed['hard_original_indices'],
                            **decompose(changed['A'], b, changed['kinds'], saved['params']['max_speed'])))
            events.append(dict(frame=state['frame'], phase=state['phase'], target=state['target'],
                position=saved['position'], true_gap=saved['true_gap'], true_margin=saved['true_gap']-saved['params']['r_safe'],
                params=saved['params'], h=saved['h'], normal_motion=saved['normal_motion'],
                actual=saved['actual'], original_status=saved['status'], scale=saved['scale'],
                object_velocity=saved['object_velocity'], point_velocities_present=saved['point_velocities'] is not None,
                row_annotations=row_annotations(saved, state['agents']), geometry=geometry, comparisons=cases))
            print(json.dumps(dict(frame=state['frame'], comparisons=len(cases),
                feasible=[x['geometry']+'/'+x['mode'] for x in cases if x['groups']['joint']['bounded']['feasible']])), flush=True)
        counts = {}
        for label, subset in (('all_18', events), ('post_acquisition_17', events[1:])):
            tally = {}
            for event in subset:
                for result in event['comparisons']:
                    key = result['geometry']+'/'+result['mode']
                    item = tally.setdefault(key, dict(total=0, feasible=0, object_feasible=0, cap_independent_conflicts=0))
                    item['total'] += 1
                    item['feasible'] += int(result['groups']['joint']['bounded']['feasible'])
                    item['object_feasible'] += int(result['groups'].get('object', {'bounded': {'feasible': True}})['bounded']['feasible'])
                    item['cap_independent_conflicts'] += int(bool((result['minimal_conflict'] or {}).get('cap_independent')))
            counts[label] = tally
        assert len(events) == 18 and sum(len(x['comparisons']) for x in events) == 226
        study.check()
        assert helpers == {name:sha(ROOT/name) for name in helpers}
        report = dict(**identity, verified_replay_frames=517, captured_robot_states=288,
            selected_states=18, comparisons=226, counts=counts, events=events,
            inference_limit='Saved-state truth-assisted feasibility cannot establish a deployable repair or closed-loop improvement')
        dump(target/'analysis.json', report)
        dump(target/'outcome.json', dict(**identity, status='VERIFIED_SAVED_STATE_DIAGNOSIS',
            states=18, comparisons=226, analysis_sha256=sha(target/'analysis.json')))
        print(json.dumps(counts, indent=2), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(**identity, status='ANALYSIS_INCOMPLETE', completed_states=len(events), error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE/'analysis')
    analyze(parser.parse_args().output)
