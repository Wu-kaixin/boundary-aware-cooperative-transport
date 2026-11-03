"""Compare fresh observed support on all26 saved acquisition conflicts."""
from __future__ import annotations

import gzip
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha

DESIGN = ROOT/'experiments/ccta2027/finite_support_probe_v1_design.md'
OUT = ROOT/'runs/research/2026-11-03-finite-support-probe-v1'
TRACE = ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1'


def main():
    guard()
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    protocol = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
    plan = read(protocol)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name) == digest
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    from dbact_sim.experiments.study import environment_versions
    from dbact.safety.filter import SafetyFilter, SafetyFilterParams
    from dbact.safety.qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy
    from dbact.sensing.features import scan_segments, segment_rows

    assert environment_versions() == plan['environment']
    OUT.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, OUT/'probe_finite_support.py')
    shutil.copy2(ROOT/'scripts/research_io.py', OUT/'research_io.py')
    shutil.copy2(DESIGN, OUT/'design.md')
    identity = dict(source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(protocol),
        script_sha256=sha(__file__), helper_sha256=sha(ROOT/'scripts/research_io.py'),
        design_sha256=sha(DESIGN), new_physical_episodes=0, environment=plan['environment'])
    dump(OUT/'identity.json', identity)
    rows, products, current = [], {}, None
    try:
        for alias in ('candidate-c', 'candidate-skew'):
            folder = TRACE/alias
            outcome = read(folder/'outcome.json')
            assert outcome['status'] == 'EXACT_RECONSTRUCTION'
            for name,digest in outcome['products'].items(): assert sha(folder/name) == digest
            events = read(folder/'events.json')['events']
            wanted = {e['frame'] for e in events}
            with gzip.open(folder/'map_history.jsonl.gz','rt',encoding='utf-8') as stream:
                history = {}
                for line in stream:
                    item = json.loads(line)
                    if item['frame'] in wanted: history[item['frame']] = item
            case = next(c for c in plan['cases'] if c['case_id'] == outcome['case_id'])
            env = SimulationEnvironment(case['config'], seed=case['seed'])
            index = next(i for i,a in enumerate(env.agents) if a.agent_id == 'agent_05')
            with np.load(batch/case['case_id']/'replay.npz',allow_pickle=False) as z:
                centers, angles = z['cargo/cargo_0/centers'].copy(), z['cargo/cargo_0/angles'].copy()
            for event in events:
                current = alias, event['frame']
                frame = history[event['frame']]
                for i,agent in enumerate(env.agents): agent.position = np.asarray(frame['positions'][i])
                env.cargoes[0].set_pose(centers[event['frame']-1], angles[event['frame']-1])
                p, now = np.asarray(event['position']), frame['time']
                neighbors = env.controller._neighbor_indices(env.agents)[index]
                scans, features, raw = {}, {}, {}
                for i in [index]+list(neighbors):
                    observed = env.controller.sensor.raw_returns(env.agents[i],env.cargoes,now)
                    # The candidate interface cannot inspect simulator edge/normal data.
                    clean = [dict(point=r['point'],object_id=r['object_id'],ray_index=r['ray_index']) for r in observed]
                    view = env.controller.sensor._estimate_normals_view(env.agents[i],clean)
                    scans[i] = view.select(view.confidence >= env.controller.sensor.params.min_confidence)
                    features[i] = scan_segments(clean,env.agents[i].position,now,
                        noise_std=env.controller.sensor.params.range_noise_std)
                    raw[i] = clean
                safety = SafetyFilter(SafetyFilterParams(**event['params']))
                trace = event['object_trace']
                projected_motion = np.asarray(trace['demand'])+safety.params.gamma_obj*np.asarray(trace['h'])
                np.testing.assert_allclose(projected_motion, 0., rtol=0, atol=1e-12)
                keep = np.asarray(event['row_kinds']) != 'object'
                hard_A, hard_b = np.asarray(event['A'])[keep], np.asarray(event['b'])[keep]
                original = solve_min_norm_2d(np.asarray(event['nominal']),np.asarray(event['A']),np.asarray(event['b']),safety.params.max_speed)
                assert not original.feasible
                own = scans[index]
                own_A, _, _, own_h = safety._object_rows(p,own.points,own.normals,np.zeros(2))
                variants = [('own_scan_aggregate',own_A,own_h,None)]
                for name, ids in (('own_segments',[index]),('one_hop_segments',[index]+list(neighbors))):
                    fs = [f for i in ids for f in features[i]]
                    A,h = segment_rows(p,fs,safety.params,now)
                    coverage = dict(features=len(fs), raw_returns=sum(len(raw[i]) for i in ids))
                    if fs:
                        starts, ends = np.asarray([f.start for f in fs]), np.asarray([f.end for f in fs])
                        ab = ends-starts
                        def lower_distance(points):
                            delta = np.asarray(points)[:,None,:]-starts[None,:,:]
                            t = np.clip(np.einsum('nki,ki->nk',delta,ab)/np.einsum('ki,ki->k',ab,ab),0,1)
                            gap = np.linalg.norm(delta-t[:,:,None]*ab,axis=2)
                            return np.min(gap-np.asarray([f.empirical_error for f in fs]),axis=1)
                        points = np.asarray([r['point'] for i in ids for r in raw[i]])
                        distances = np.linalg.norm(points-p,axis=1)
                        near = distances <= distances.min()+2*safety.params.max_speed*safety.params.dt
                        coverage['near_raw_returns'] = int(near.sum())
                        coverage['near_raw_not_within_empirical_segment_error'] = int(np.count_nonzero(lower_distance(points[near])>1e-12))
                        coverage['maximum_selected_cell_distance_beyond_segment_error_m'] = float(lower_distance(event['object_trace']['face_points']).max())
                    variants.append((name,A,h,coverage))
                for name,A,h,coverage in variants:
                    record = dict(case=alias,frame=event['frame'],variant=name,object_rows=len(A),
                        support_empty=not len(A),minimum_h_m=float(np.min(h)) if len(h) else None,
                        true_margin_m=event['true_boundary_gap']-safety.params.r_safe,
                        empirical_only=True,coverage=coverage,solves=[])
                    if len(h):
                        record['distance_overestimate_m'] = float(np.min(h)+safety.params.r_safe-event['true_boundary_gap'])
                    for motion in (0., safety.params.object_velocity_bound):
                        for rho in (safety.params.rho,0.):
                            uncapped = motion-safety.params.gamma_obj*h+rho
                            rhs = safety._cap_to_reachable(A,uncapped)
                            all_A, all_b = np.vstack([hard_A,A]), np.r_[hard_b,rhs]
                            result = solve_min_norm_2d(np.asarray(event['nominal']),all_A,all_b,safety.params.max_speed)
                            independent = solve_min_norm_2d_cvxpy(np.asarray(event['nominal']),all_A,all_b,safety.params.max_speed)
                            assert result.feasible == independent.feasible, current
                            record['solves'].append(dict(motion_bound=motion,rho=rho,feasible=result.feasible,
                                supported_feasible=bool(len(A) and result.feasible),solver=independent.detail,
                                cap_active=int(np.count_nonzero(rhs < uncapped-1e-15)),
                                command=result.u.tolist()))
                    rows.append(record)
            products[alias] = sha(folder/'outcome.json')
        assert len(rows) == 78
        report = dict(**identity,trace_outcomes=products,events=26,records=rows,
            scope='Posthoc same-state observational counterfactual; zero new physics; no safety certificate or adoption')
        dump(OUT/'analysis.json',report)
        dump(OUT/'outcome.json',dict(**identity,status='COMPLETED',events=26,variants=3,
            qp_comparisons=312,analysis_sha256=sha(OUT/'analysis.json')))
        for variant in ('own_scan_aggregate','own_segments','one_hop_segments'):
            selected=[r for r in rows if r['variant']==variant]
            print(json.dumps(dict(variant=variant,events=len(selected),empty=sum(r['support_empty'] for r in selected),
                zero_motion_full_rho_feasible=sum(r['solves'][0]['supported_feasible'] for r in selected),
                bounded_motion_full_rho_feasible=sum(r['solves'][2]['supported_feasible'] for r in selected),
                distance_overestimated=sum(r.get('distance_overestimate_m',-1)>1e-9 for r in selected))),flush=True)
    except Exception as exc:
        dump(OUT/'failure.json',dict(**identity,status='FAILED',at=current,error=repr(exc)))
        raise


if __name__ == '__main__':
    main()
