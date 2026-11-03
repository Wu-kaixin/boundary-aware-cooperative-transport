"""Attribute fixed finite-support contact rows to actual scan-packet features."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

import post_acquisition_support_study as study
import probe_post_support_contact_compatibility as predecessor
from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from dbact.control.post_support import filter_inputs
from dbact.geometry import signed_distance_and_gradient
from dbact.safety.filter import SafetyFilterParams
from dbact.safety.post_support import build_rows
from dbact.sensing.finite_support import ScanSupport
from dbact_sim.experiments.deployment import dump


CONTACT = ROOT / 'runs/research/2026-11-03-post-support-contact-compatibility-v2'
DECOMPOSITION = ROOT / 'runs/research/2026-11-03-post-support-contact-row-decomposition-v1'
ENVELOPE = ROOT / 'runs/research/2026-11-03-post-support-contact-motion-envelope-v1'
OUT = ROOT / 'runs/research/2026-11-03-post-support-contact-packet-geometry-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_contact_packet_geometry_v1_design.md'


def packet(value):
    return ScanSupport(np.asarray(value['points']).reshape(-1, 2),
        np.asarray(value['object_ids']), np.asarray(value['origin']),
        value['timestamp'], value['noise_allowance'],
        np.asarray(value['chords'], dtype=int).reshape(-1, 2), np.asarray(value['half_gaps']))


def features(position, sources, packets, params, timestamp):
    """Reconstruct ordered feature metadata; callers verify every selected row."""
    active = [(aid, p) for aid, p in zip(sources, packets)
              if abs(p.timestamp-timestamp) <= 1e-9 and len(p.points)]
    assert active
    points = np.vstack([p.points for _, p in active])
    object_ids = np.concatenate([p.object_ids for _, p in active])
    covered = np.concatenate([np.isin(np.arange(len(p.points)), p.chords.reshape(-1))
                              for _, p in active])
    origins = np.vstack([np.tile(p.origin, (len(p.points), 1)) for _, p in active])
    errors = np.concatenate([np.full(len(p.points), p.noise_allowance) for _, p in active])
    point_ids = [(aid, i) for aid, p in active for i in range(len(p.points))]
    records = []
    for aid, p in active:
        for chord, half_gap in zip(p.chords, p.half_gaps):
            records.append(dict(source_agent=aid, kind='chord', indices=chord.tolist(),
                start=p.points[chord[0]], end=p.points[chord[1]], origin=p.origin,
                structural=float(half_gap), noise=p.noise_allowance))
    for index in np.flatnonzero(~covered):
        gaps = np.linalg.norm(points-points[index], axis=1)
        eligible = (object_ids == object_ids[index]) & (gaps > 1e-8)
        radius = .5*float(gaps[eligible].min()) if np.any(eligible) else np.inf
        aid, point_index = point_ids[index]
        records.append(dict(source_agent=aid, kind='isolated_point', indices=[point_index],
            start=points[index], end=points[index], origin=origins[index],
            structural=radius, noise=float(errors[index])))
    assert records
    starts = np.asarray([r['start'] for r in records])
    ends = np.asarray([r['end'] for r in records])
    origins = np.asarray([r['origin'] for r in records])
    structural = np.asarray([r['structural'] for r in records])
    noise = np.asarray([r['noise'] for r in records])
    ab = ends-starts
    squared = np.einsum('ij,ij->i', ab, ab)
    t = np.clip(np.divide(np.einsum('ij,ij->i', position-starts, ab), squared,
        out=np.zeros(len(ab)), where=squared > 1e-16), 0., 1.)
    feet = starts+t[:, None]*ab
    delta = position-feet
    distances = np.linalg.norm(delta, axis=1)
    lower = distances-structural-noise
    reach = 2.*(params.max_speed+params.object_velocity_bound)*params.dt
    keep = ((lower <= np.min(distances+structural+noise)+reach)
            & (lower <= params.object_row_range))
    normals = delta/np.maximum(distances[:, None], 1e-12)
    for i in np.flatnonzero(distances <= 1e-12):
        if records[i]['kind'] == 'isolated_point':
            n = origins[i]-feet[i]
            length = float(np.linalg.norm(n))
            n = n/length if length > 1e-12 else np.array([1., 0.])
        else:
            tangent = ab[i]/np.linalg.norm(ab[i])
            n = np.array([-tangent[1], tangent[0]])
        if n @ (origins[i]-feet[i]) < 0.:
            n = -n
        normals[i] = n
    selected = []
    for i, record in enumerate(records):
        record.update(feature_index=i, projection=float(t[i]), foot=feet[i],
            distance=float(distances[i]), lower_distance=float(lower[i]),
            normal=normals[i], h=float(lower[i]-params.r_safe), kept=bool(keep[i]))
        if keep[i]:
            record['row_index'] = len(selected)
            selected.append(record)
    return records, selected


def capture(item, expected):
    """Read-only hook inside the predecessor's exact control replay."""
    case = item['case']
    folder = study.OUT / case['case_id']
    assert sha(folder / 'case_result.json') == item['receipt_sha256']
    selected = {int(k): value for k, value in item['selection'].items()}
    with np.load(folder / 'replay.npz', allow_pickle=False) as saved:
        times = saved['times'].copy()
    original_factory = predecessor.SimulationEnvironment
    captured = {}
    calls = 0

    def factory(*args, **kwargs):
        env = original_factory(*args, **kwargs)
        ids = [a.agent_id for a in env.agents]
        original_prepare = env.controller.safety.prepare_velocity

        def prepare(position, nominal_velocity, neighbor_positions=(), **inputs):
            nonlocal calls
            frame, index = divmod(calls, len(ids))
            calls += 1
            result = original_prepare(position, nominal_velocity, neighbor_positions, **inputs)
            if frame not in selected:
                return result
            aid = ids[index]
            neighbors = []
            for point in neighbor_positions:
                matches = [a.agent_id for a in env.agents if np.array_equal(a.position, point)]
                assert len(matches) == 1 and matches[0] != aid
                neighbors.append(matches[0])
            current, trace = filter_inputs(env.controller, aid, neighbors, float(times[frame]))
            assert len(env.cargoes) == 1 and env.cargoes[0].object_id == 'cargo_0'
            assert (frame+1, aid) not in captured
            captured[frame+1, aid] = plain(dict(frame=frame+1, agent_id=aid,
                timestamp=float(times[frame]), position=np.asarray(position).copy(),
                neighbors=neighbors, source_agents=trace['source_agents'],
                packets=[asdict(p) for p in current.get('post_support_packets', [])],
                admission_inputs=trace, parameters=asdict(result.parameters),
                object_velocity=inputs.get('object_velocity'),
                boundary_point_velocities=inputs.get('boundary_point_velocities'),
                evaluation_precommand_vertices=env.cargoes[0].vertices.copy(),
                evaluation_precommand_center=env.cargoes[0].position.copy(),
                evaluation_precommand_angle=env.cargoes[0].angle))
            return result

        env.controller.safety.prepare_velocity = prepare
        return env

    predecessor.SimulationEnvironment = factory
    try:
        reproduced = predecessor.replay(case, folder, selected)
        assert reproduced == expected, case['condition_id']
        assert len(captured) == 32 and calls == expected['verified_frames']*16
    except Exception:
        dump(OUT / (case['condition_id']+'-capture-failure.json'),
            dict(captured=list(captured.values()), calls=calls, traceback=traceback.format_exc()))
        raise
    finally:
        predecessor.SimulationEnvironment = original_factory
    return list(captured.values()), reproduced['verified_frames']


def analyze_state(captured, previous, decomposition, envelope):
    position = np.asarray(captured['position'])
    params = SafetyFilterParams(**captured['parameters'])
    result = dict(condition=decomposition['condition'], frame=captured['frame'],
        agent_id=captured['agent_id'], admitted=previous['support']['applied'],
        guard=previous['support']['reason'], positive_force=decomposition['positive_force'],
        declared_pusher=decomposition['declared_pusher'],
        previous_signed_ball_exclusion=envelope.get('original_command_rejected_for_all_bounded_motion'))
    # Simulator geometry is used only after the unchanged controller replay passed.
    signed, normals, feet = signed_distance_and_gradient(position.reshape(1, 2),
        np.asarray(captured['evaluation_precommand_vertices']))
    result.update(evaluation_signed_distance=float(signed[0]), evaluation_normal=normals[0],
        evaluation_foot=feet[0], evaluation_h=float(signed[0]-params.r_safe),
        evaluation_time='pre-command pose; contact force is from the ensuing saved interval')
    if not captured['packets']:
        assert not result['admitted']
        assert captured['admission_inputs']['reason'] == result['guard']
        return result
    packets = [packet(value) for value in captured['packets']]
    support, demand = build_rows(position, packets, params, captured['timestamp'],
        captured['object_velocity'], captured['boundary_point_velocities'])
    assert support.ready == result['admitted'] and support.reason == result['guard']
    assert captured['source_agents'] == previous['support']['source_agents']
    assert demand == previous['support']['demand_speed']
    if not support.ready:
        return result
    matrix = previous['counterfactual_rows']
    A = np.asarray(matrix['A'])[np.asarray(matrix['kinds']) == 'object']
    h = np.asarray(matrix['h'])
    b = np.asarray(matrix['b'])[np.asarray(matrix['kinds']) == 'object']
    np.testing.assert_array_equal(support.A, A)
    np.testing.assert_array_equal(support.h, h)
    all_features, kept = features(position, captured['source_agents'], packets, params, captured['timestamp'])
    np.testing.assert_allclose([r['normal'] for r in kept], A, rtol=0., atol=1e-12)
    np.testing.assert_allclose([r['h'] for r in kept], h, rtol=0., atol=1e-12)
    np.testing.assert_allclose([r['lower_distance'] for r in kept], support.lower_distance, rtol=0., atol=1e-12)
    u = np.asarray(previous['original_command'])
    residual = A @ u + params.gamma_obj*h-params.rho-demand
    np.testing.assert_allclose(residual.min(), previous['original_command_min_uncapped_object_residual'], rtol=0., atol=1e-12)
    np.testing.assert_allclose((A @ u-b).min(), previous['original_command_min_capped_object_residual'], rtol=0., atol=1e-12)
    limiting = int(np.argmin(residual))
    assert limiting == decomposition['limiting_row']
    for row in kept:
        i = row['row_index']
        row.update(own_source=row['source_agent'] == captured['agent_id'],
            command_term=float(A[i] @ u), distance_term=params.gamma_obj*(row['distance']-params.r_safe),
            structural_term=-params.gamma_obj*row['structural'], noise_term=-params.gamma_obj*row['noise'],
            margin_term=-params.rho, motion_term=-demand,
            uncapped_residual=float(residual[i]),
            normal_alignment_with_evaluation=float(A[i] @ normals[0]),
            lower_distance_minus_evaluation=float(row['lower_distance']-signed[0]))
        np.testing.assert_allclose(sum(row[k] for k in ('command_term', 'distance_term',
            'structural_term', 'noise_term', 'margin_term', 'motion_term')),
            residual[i], rtol=0., atol=1e-12)
    structural = np.asarray([r['structural'] for r in kept])
    noise = np.asarray([r['noise'] for r in kept])
    result.update(object_rows=len(kept), all_features=all_features, rows=kept,
        limiting_row=limiting, limiting_feature=kept[limiting],
        minimum_uncapped_residual=float(residual.min()),
        minimum_without_structural=float((residual+params.gamma_obj*structural).min()),
        minimum_without_noise=float((residual+params.gamma_obj*noise).min()),
        minimum_without_both=float((residual+params.gamma_obj*(structural+noise)).min()),
        original_command=u, r_safe=params.r_safe, gamma_obj=params.gamma_obj,
        rho=params.rho, demand_speed=demand)
    return result


def main():
    guard()
    plan = study.check()
    contact, decomposition, envelope = [read(path / 'analysis.json') for path in (CONTACT, DECOMPOSITION, ENVELOPE)]
    for path, status in ((CONTACT, 'VERIFIED_CONTACT_COMPATIBILITY'),
                         (DECOMPOSITION, 'VERIFIED_ROW_DECOMPOSITION'), (ENVELOPE, 'VERIFIED_MOTION_ENVELOPE')):
        outcome = read(path / 'outcome.json')
        assert outcome['status'] == status and outcome['analysis_sha256'] == sha(path / 'analysis.json')
    assert decomposition['input_sha256'] == sha(CONTACT / 'analysis.json')
    assert envelope['decomposition_sha256'] == sha(DECOMPOSITION / 'analysis.json')
    assert contact['runtime_source_sha256'] == plan['frozen_source_sha256']
    helpers = [Path(__file__), DESIGN, Path(predecessor.__file__),
        ROOT / 'scripts/trace_boundary_map.py', ROOT / 'scripts/research_io.py']
    identities = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in helpers}
    identity = dict(started_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime_source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(study.PROTOCOL),
        contact_analysis_sha256=sha(CONTACT / 'analysis.json'), decomposition_sha256=sha(DECOMPOSITION / 'analysis.json'),
        envelope_sha256=sha(ENVELOPE / 'analysis.json'), sources=identities, new_physical_episodes=0)
    OUT.mkdir(parents=True, exist_ok=False)
    for path in helpers:
        shutil.copy2(path, OUT / path.name)
    dump(OUT / 'started.json', identity)
    key = lambda row: (row['condition'], row['frame'], row['agent_id'])
    dec = {key(row): row for row in decomposition['rows']}
    env = {key(row): row for row in envelope['rows']}
    expected = {row['condition']: row for row in contact['results']}
    states = []
    verified = 0
    try:
        for item in contact['schedule']:
            condition = item['case']['condition_id']
            captures, frames = capture(item, expected[condition])
            verified += frames
            dump(OUT / (condition+'-packets.json'), dict(condition=condition, verified_frames=frames, captures=captures))
            old = {(event['saved_frame'], state['agent_id']): state
                for event in expected[condition]['events'] for state in event['states']}
            rows = [analyze_state(c, old[c['frame'], c['agent_id']], dec[condition, c['frame'], c['agent_id']],
                env[condition, c['frame'], c['agent_id']]) for c in captures]
            states.extend(rows)
            dump(OUT / (condition+'-geometry.json'), plain(dict(condition=condition, states=rows)))
            print(json.dumps(dict(condition=condition, packet_states=len(rows), verified_frames=frames)), flush=True)
        assert len(states) == 128 and len({key(row) for row in states}) == 128 and verified == 1749
        counts = {}
        for name, rows in (
            ('all', states), ('admitted', [r for r in states if r['admitted']]),
            ('positive_force', [r for r in states if r['positive_force']]),
            ('positive_force_pusher', [r for r in states if r['positive_force'] and r['declared_pusher']]),
        ):
            admitted = [r for r in rows if r['admitted']]
            counts[name] = dict(states=len(rows), admitted=len(admitted),
                limiting_sources=dict(Counter('own' if r['limiting_feature']['own_source'] else 'one_hop' for r in admitted)),
                limiting_features=dict(Counter(r['limiting_feature']['kind'] for r in admitted)),
                conflicts_without_structural=sum(r['minimum_without_structural'] < -1e-7 for r in admitted),
                conflicts_without_noise=sum(r['minimum_without_noise'] < -1e-7 for r in admitted),
                conflicts_without_both=sum(r['minimum_without_both'] < -1e-7 for r in admitted),
                evaluation_negative_h=sum(r['evaluation_h'] < 0. for r in admitted))
        assert counts['admitted']['states'] == 112 and counts['positive_force']['states'] == 17
        assert counts['positive_force_pusher']['states'] == 11
        study.check()
        assert all(sha(ROOT / name) == digest for name, digest in identities.items())
        report = plain(dict(**identity, verified_control_frames=verified, states=states, counts=counts,
            scope='Fixed-row algebra and evaluation-only pre-command geometry on the unchanged 128-state development cohort; no new physics or candidate adoption.'))
        dump(OUT / 'analysis.json', report)
        dump(OUT / 'outcome.json', dict(**identity, status='VERIFIED_PACKET_GEOMETRY',
            analysis_sha256=sha(OUT / 'analysis.json'), verified_frames=verified, states=len(states),
            object_rows=sum(r.get('object_rows', 0) for r in states)))
        print(json.dumps(counts), flush=True)
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, verified_frames=verified,
            analyzed_states=len(states), traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
