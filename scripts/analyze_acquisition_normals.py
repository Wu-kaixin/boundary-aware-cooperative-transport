"""Separate scan-normal error, fused-cell lineage and finite-edge extrapolation."""
from __future__ import annotations

from collections import Counter
import gzip
import json
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha

BASE = ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1'


def angle(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1., 1.))))


def main():
    guard()
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    protocol = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
    design = ROOT/'experiments/ccta2027/acquisition_normal_audit_v1_design.md'
    plan = read(protocol)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name) == digest
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    from dbact.geometry import closest_point_on_segment, outward_edge_normals
    out = BASE/'normal-analysis-v1'
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, out/'analyze_acquisition_normals.py')
    shutil.copy2(design, out/'design.md')
    shutil.copy2(ROOT/'scripts/research_io.py', out/'research_io.py')
    identity = dict(source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(protocol),
        script_sha256=sha(__file__), design_sha256=sha(design), new_physical_episodes=0,
        new_controller_replay_frames=0, numerical_lineage_check_atol=1e-10,
        scope='Posthoc analysis of all26 saved acquisition conflicts on two reused conditions; source-normal reconstruction and true finite-edge geometry are observer-only')
    dump(out/'identity.json', identity)
    report = dict(**identity, cases={})
    current = None
    try:
        for alias in ('candidate-c', 'candidate-skew'):
            current = alias
            trace = BASE/alias
            outcome = read(trace/'outcome.json')
            assert outcome['status'] == 'EXACT_RECONSTRUCTION' and outcome['source_sha256'] == plan['frozen_source_sha256']
            assert outcome['protocol_sha256'] == sha(protocol)
            for name, digest in outcome['products'].items():
                assert sha(trace/name) == digest
            case = next(c for c in plan['cases'] if c['case_id'] == outcome['case_id'])
            assert sha(batch/case['case_id']/'case_result.json') == outcome['original_receipt_sha256']
            events = read(trace/'events.json')['events']
            with gzip.open(trace/'map_history.jsonl.gz', 'rt', encoding='utf-8') as stream:
                history = [json.loads(line) for line in stream]
            assert len(history) == outcome['verified_frames']
            selected = {}
            wanted = set()
            for event in events:
                frame = history[event['frame']-1]
                points = np.asarray(frame['after_update']['points'])
                keys = []
                for point in event['object_trace']['face_points']:
                    index = int(np.argmin(np.linalg.norm(points-point, axis=1)))
                    np.testing.assert_array_equal(points[index], point)
                    key = frame['after_update']['keys'][index]
                    keys.append(key)
                    wanted.add(key)
                selected[event['frame']] = keys
            # Follow the actual normalization and capped historical weight.
            # Positive coefficients reconstruct the final unit normal; their sum
            # need not equal one. This is not a probability or independent sample.
            ancestry, selected_ancestry = {}, {}
            fusion_checks, residuals = 0, []
            for frame in history:
                before, incoming, after = frame['before_update'], frame['incoming'], frame['after_update']
                pre = {key:i for i,key in enumerate(before['keys'])}
                post = {key:i for i,key in enumerate(after['keys'])}
                ancestry = {key:value for key,value in ancestry.items() if key in pre}
                for key in wanted & set(post):
                    indices = [i for i,k in enumerate(incoming['keys']) if k == key]
                    prior = ancestry.get(key, [])
                    if key in pre:
                        assert prior, (alias, frame['frame'], key, 'missing ancestry')
                        reconstructed = sum((weight*np.asarray(history[f-1]['incoming']['normals'][j])
                            for f,j,weight in prior), start=np.zeros(2))
                        np.testing.assert_allclose(reconstructed, before['normals'][pre[key]], rtol=0, atol=1e-10)
                    if indices:
                        weights = np.maximum(np.asarray(incoming['confidence'])[indices], 1e-6)
                        added = np.asarray(incoming['normals'])[indices]
                        summed = np.sum(weights[:,None]*added, axis=0)
                        old_weight = before['weights'][pre[key]] if key in pre else 0.
                        combined = summed + (np.asarray(before['normals'][pre[key]])*old_weight if key in pre else 0.)
                        magnitude = float(np.linalg.norm(combined))
                        if magnitude > 1e-9:
                            prior = [(f,j,w*old_weight/magnitude) for f,j,w in prior]
                            prior.extend((frame['frame'],j,float(w/magnitude)) for j,w in zip(indices, weights))
                        elif key not in pre:
                            prior = []
                    ancestry[key] = prior
                    reconstructed = sum((weight*np.asarray(history[f-1]['incoming']['normals'][j])
                        for f,j,weight in prior), start=np.zeros(2))
                    residuals.append(float(np.max(np.abs(reconstructed-np.asarray(after['normals'][post[key]])))))
                    np.testing.assert_allclose(reconstructed, after['normals'][post[key]], rtol=0, atol=1e-10)
                    fusion_checks += 1
                if frame['frame'] in selected:
                    selected_ancestry[frame['frame']] = {key:list(ancestry[key]) for key in selected[frame['frame']]}

            env = SimulationEnvironment(case['config'], seed=case['seed'])
            sensor = env.controller.sensor
            with np.load(batch/case['case_id']/'replay.npz', allow_pickle=False) as z:
                centers, angles = z['cargo/cargo_0/centers'].copy(), z['cargo/cargo_0/angles'].copy()
            cache, origins = {}, {}
            used = {(f,j) for cells in selected_ancestry.values() for lineage in cells.values() for f,j,_ in lineage}
            for f,j in sorted(used):
                frame = history[f-1]
                incoming = frame['incoming']
                source = incoming['source_codes'][j]
                cache_key = f, source
                if cache_key not in cache:
                    env.cargoes[0].set_pose(centers[f-1], angles[f-1])
                    agent = env.agents[source]
                    agent.position = np.asarray(frame['positions'][source])
                    returns = sensor.raw_returns(agent, env.cargoes, frame['time'])
                    view = sensor._estimate_normals_view(agent, returns)
                    cache[cache_key] = returns, view
                returns, view = cache[cache_key]
                matches = np.flatnonzero(np.all(view.points == incoming['points'][j], axis=1))
                assert len(matches) == 1
                at = int(matches[0])
                np.testing.assert_array_equal(view.normals[at], incoming['normals'][j])
                assert view.confidence[at] == incoming['confidence'][j]
                assert view.confidence[at] >= sensor.params.min_confidence
                item = next(r for r in returns if np.array_equal(r['point'], incoming['points'][j]))
                same = [r for r in returns if r['object_id'] == item['object_id']]
                points = np.asarray([r['point'] for r in same])
                k = min(sensor.params.pca_neighbors, len(points))
                nearby = np.argsort(np.sum((points-np.asarray(item['point']))**2, axis=1), kind='stable')[:k]
                neighborhood = points[nearby]
                covariance = (neighborhood-neighborhood.mean(axis=0)).T@(neighborhood-neighborhood.mean(axis=0))/max(1,k-1)
                eigenvalues = np.linalg.eigvalsh(covariance)
                origins[f'{f}:{j}'] = dict(frame=f, source_agent=env.agents[source].agent_id,
                    ray=int(item['ray_index']), point=incoming['points'][j], normal=incoming['normals'][j],
                    confidence=incoming['confidence'][j], true_edge_normal=item['true_normal'].tolist(),
                    normal_error_deg=angle(view.normals[at], item['true_normal']),
                    pca_ray_indices=[int(same[n]['ray_index']) for n in nearby],
                    pca_true_normal_span_deg=max(angle(item['true_normal'], same[n]['true_normal']) for n in nearby),
                    pca_eigenvalues=eigenvalues.tolist())

            records, detailed = [], []
            vertices = env.cargoes[0].vertices
            edge_normals = outward_edge_normals(vertices)
            for event in events:
                env.cargoes[0].set_pose(centers[event['frame']-1], angles[event['frame']-1])
                vertices = env.cargoes[0].vertices
                edge_normals = outward_edge_normals(vertices)
                position = np.asarray(event['position'])
                face = event['object_trace']
                cells = []
                for key, point, normal in zip(selected[event['frame']], face['face_points'], face['face_normals']):
                    distances = []
                    for edge, start in enumerate(vertices):
                        foot, _ = closest_point_on_segment(np.asarray(point), start, vertices[(edge+1)%len(vertices)])
                        distances.append(np.linalg.norm(np.asarray(point)-foot))
                    edge = int(np.argmin(distances))
                    start, finish = vertices[edge], vertices[(edge+1)%len(vertices)]
                    direction = finish-start
                    t = float((position-start)@direction/(direction@direction))
                    true_plane_h = float(edge_normals[edge]@(position-start)-event['params']['r_safe'])
                    lineage = selected_ancestry[event['frame']][key]
                    coefficients = np.asarray([w for _,_,w in lineage])
                    errors = np.asarray([origins[f'{f}:{j}']['normal_error_deg'] for f,j,_ in lineage])
                    sources = Counter()
                    for f,j,w in lineage:
                        sources[origins[f'{f}:{j}']['source_agent']] += w
                    cells.append(dict(key=key, point=point, normal=normal,
                        nearest_true_edge=edge, true_edge_normal=edge_normals[edge].tolist(),
                        true_edge_start=start.tolist(), true_edge_end=finish.tolist(),
                        map_normal_error_to_nearest_true_edge_deg=angle(normal, edge_normals[edge]),
                        robot_projection_on_true_edge=t, projection_outside_edge=bool(t<0 or t>1),
                        true_supporting_plane_h=true_plane_h,
                        source_return_count=len(lineage), source_agent_count=len(sources),
                        source_normal_error_min_deg=float(errors.min()), source_normal_error_max_deg=float(errors.max()),
                        coefficient_weighted_source_normal_error_deg=float(coefficients@errors/coefficients.sum()),
                        normalized_source_coefficients={aid:float(w/coefficients.sum()) for aid,w in sources.items()},
                        lineage=[dict(frame=f,batch_index=j,coefficient=w) for f,j,w in lineage]))
                # Correcting normals while retaining these same incident infinite
                # planes is an observer diagnostic, never an online candidate.
                normals = np.asarray([c['true_edge_normal'] for c in cells])
                starts = np.asarray([c['true_edge_start'] for c in cells])
                weight = np.asarray(face['weights'])
                magnitude = np.linalg.norm(weight@normals)
                corrected_plane_h = float(np.sum(weight*np.einsum('ij,ij->i',normals,position-starts))/magnitude-event['params']['r_safe']) if magnitude>1e-9 else None
                record = dict(frame=event['frame'], agent_id=event['agent_id'], scale=event['scale'],
                    true_margin_m=event['true_boundary_gap']-event['params']['r_safe'], aggregate_h_m=face['h'][0],
                    selected_cells=len(cells), outside_true_segment_projection_cells=sum(c['projection_outside_edge'] for c in cells),
                    min_map_normal_error_deg=min(c['map_normal_error_to_nearest_true_edge_deg'] for c in cells),
                    max_map_normal_error_deg=max(c['map_normal_error_to_nearest_true_edge_deg'] for c in cells),
                    corrected_true_supporting_planes_h_m=corrected_plane_h)
                records.append(record)
                detailed.append(dict(**record,cells=cells))
            dump(out/f'{alias}-lineage.json', dict(origins=origins, events=detailed,
                coefficient_scope='Positive reconstruction coefficients of fused unit normals; not independent observations or causal effect estimates'))
            report['cases'][alias] = dict(case_id=case['case_id'], trace_outcome_sha256=sha(trace/'outcome.json'),
                verified_controller_frames=outcome['verified_frames'], conflict_events=len(events),
                unique_selected_cells=len(wanted), fusion_reconstruction_checks=fusion_checks,
                maximum_normal_reconstruction_residual=max(residuals), reconstructed_source_returns=len(origins),
                reconstructed_source_scans=len(cache), events=records,
                first_event_cells=[{k:v for k,v in c.items() if k!='lineage'} for c in detailed[0]['cells']],
                lineage_sha256=sha(out/f'{alias}-lineage.json'),
                all_true_margins_positive=all(r['true_margin_m']>0 for r in records),
                all_original_aggregate_h_negative=all(r['aggregate_h_m']<0 for r in records),
                negative_after_true_plane_correction=sum(r['corrected_true_supporting_planes_h_m'] is not None and r['corrected_true_supporting_planes_h_m']<0 for r in records))
            print(json.dumps({k:v for k,v in report['cases'][alias].items() if k not in ('events','first_event_cells')}), flush=True)
        dump(out/'analysis.json', report)
        dump(out/'outcome.json', dict(**identity, status='COMPLETED', analysis_sha256=sha(out/'analysis.json')))
    except Exception as exc:
        dump(out/'failure.json', dict(**identity, status='FAILED_ANALYSIS', case=current, error=repr(exc)))
        raise


if __name__ == '__main__':
    main()
