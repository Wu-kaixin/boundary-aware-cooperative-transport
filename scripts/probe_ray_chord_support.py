"""Frozen all26-state check of the empirical chord module, no controller wiring."""
from __future__ import annotations

import argparse
import gzip
import importlib.util
import json
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha


def main():
    guard()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version',type=int,choices=(1,2),default=1)
    version = parser.parse_args().version
    assert not any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules)
    protocol = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
    design = ROOT/('experiments/ccta2027/finite_acquisition_support_v1_design.md' if version==1
                   else 'experiments/ccta2027/finite_acquisition_point_guard_v2_design.md')
    plan = read(protocol)
    batch = ROOT/plan['run_dir']
    for name,digest in plan['source_files'].items(): assert sha(batch/'source_snapshot'/name) == digest
    out = ROOT/f'runs/research/2026-11-03-ray-chord-support-probe-v{version}'
    out.mkdir(parents=True,exist_ok=False)
    for source,name in ((ROOT/'src/dbact/sensing/finite_support.py','finite_support.py'),
                        (design,'design.md'),(ROOT/'scripts/research_io.py','research_io.py')):
        shutil.copy2(source,out/name)
    shutil.copy2(__file__,out/'probe_ray_chord_support.py')
    identity = dict(frozen_runtime_sha256=plan['frozen_source_sha256'],protocol_sha256=sha(protocol),
        candidate_module_sha256=sha(out/'finite_support.py'),script_sha256=sha(__file__),
        design_sha256=sha(design),new_physical_episodes=0,environment=plan['environment'],
        version=version,isolated_point_guards=version==2)
    sys.path.insert(0,str(batch/'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    from dbact_sim.experiments.study import environment_versions
    from dbact.safety.filter import SafetyFilter, SafetyFilterParams
    from dbact.safety.qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy
    assert environment_versions() == plan['environment']
    spec = importlib.util.spec_from_file_location('candidate_finite_support',out/'finite_support.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    dump(out/'identity.json',identity)
    records, source_receipts, current = [], {}, None
    try:
        for alias in ('candidate-c','candidate-skew'):
            trace = ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1'/alias
            outcome = read(trace/'outcome.json')
            for name,digest in outcome['products'].items(): assert sha(trace/name) == digest
            assert outcome['status'] == 'EXACT_RECONSTRUCTION'
            source_receipts[alias] = sha(trace/'outcome.json')
            events = read(trace/'events.json')['events']
            wanted = {e['frame'] for e in events}
            history = {}
            with gzip.open(trace/'map_history.jsonl.gz','rt',encoding='utf-8') as stream:
                for line in stream:
                    row = json.loads(line)
                    if row['frame'] in wanted: history[row['frame']] = row
            case = next(c for c in plan['cases'] if c['case_id'] == outcome['case_id'])
            env = SimulationEnvironment(case['config'],seed=case['seed'])
            at = next(i for i,a in enumerate(env.agents) if a.agent_id == 'agent_05')
            with np.load(batch/case['case_id']/'replay.npz',allow_pickle=False) as z:
                centers, angles = z['cargo/cargo_0/centers'].copy(), z['cargo/cargo_0/angles'].copy()
            for event in events:
                current = alias,event['frame']
                state = history[event['frame']]
                for i,a in enumerate(env.agents): a.position = np.asarray(state['positions'][i])
                env.cargoes[0].set_pose(centers[event['frame']-1],angles[event['frame']-1])
                indices = [at]+list(env.controller._neighbor_indices(env.agents)[at])
                packets = []
                for i in indices:
                    raw = env.controller.sensor.raw_returns(env.agents[i],env.cargoes,state['time'])
                    clean = [dict(point=r['point'],ray_index=r['ray_index'],object_id=r['object_id']) for r in raw]
                    packets.append(module.scan_support(clean,env.agents[i].position,state['time'],
                        env.controller.sensor.params.range_noise_std))
                safety = SafetyFilter(SafetyFilterParams(**event['params']))
                tr = event['object_trace']
                np.testing.assert_allclose(np.asarray(tr['demand'])+safety.params.gamma_obj*np.asarray(tr['h']),0.,atol=1e-12,rtol=0)
                hard = np.asarray(event['row_kinds']) != 'object'
                hard_A, hard_b = np.asarray(event['A'])[hard],np.asarray(event['b'])[hard]
                for motion in (0.,safety.params.object_velocity_bound):
                    rows = module.support_rows(event['position'],packets,safety.params,state['time'],motion,
                        isolated_point_guards=True) if version==2 else module.support_rows(
                            event['position'],packets,safety.params,state['time'],motion)
                    record = dict(case=alias,frame=event['frame'],motion_bound=motion,ready=rows.ready,
                        reason=rows.reason,near_returns=rows.near_returns,uncovered_returns=rows.uncovered_returns,
                        object_rows=len(rows.A),point_guard_rows=getattr(rows,'point_guard_rows',0),
                        h=rows.h.tolist(),A=rows.A.tolist(),
                        true_clearance_m=event['true_boundary_gap'],solves=[])
                    if rows.ready:
                        record['minimum_lower_distance_m'] = float(rows.lower_distance.min())
                        record['distance_overestimate_m'] = float(rows.lower_distance.min()-event['true_boundary_gap'])
                        for rho in (safety.params.rho,0.):
                            uncapped = motion-safety.params.gamma_obj*rows.h+rho
                            b = safety._cap_to_reachable(rows.A,uncapped)
                            A, rhs = np.vstack([hard_A,rows.A]),np.r_[hard_b,b]
                            result = solve_min_norm_2d(np.asarray(event['nominal']),A,rhs,safety.params.max_speed)
                            independent = solve_min_norm_2d_cvxpy(np.asarray(event['nominal']),A,rhs,safety.params.max_speed)
                            assert result.feasible == independent.feasible,current
                            record['solves'].append(dict(rho=rho,feasible=result.feasible,
                                cap_active=int(np.count_nonzero(b < uncapped-1e-15)),
                                command=result.u.tolist(),independent_status=independent.detail))
                    records.append(record)
        assert len(records)==52
        report = dict(**identity,source_receipts=source_receipts,records=records,
            scope='Empirical saved-state counterfactual; non-ready states retain original constraints, not counted feasible')
        dump(out/'analysis.json',report)
        dump(out/'outcome.json',dict(**identity,status='COMPLETED',events=26,
            qp_comparisons=sum(len(r['solves']) for r in records),analysis_sha256=sha(out/'analysis.json')))
        for motion in (0.,.2):
            group=[r for r in records if r['motion_bound']==motion]
            print(json.dumps(dict(motion=motion,events=len(group),ready=sum(r['ready'] for r in group),
                full_rho_feasible=sum(r['ready'] and r['solves'][0]['feasible'] for r in group),
                false_clearance=sum(r.get('distance_overestimate_m',-1)>1e-9 for r in group),
                unready=[dict(case=r['case'],frame=r['frame'],reason=r['reason'],uncovered=r['uncovered_returns']) for r in group if not r['ready']])),flush=True)
    except Exception as exc:
        dump(out/'failure.json',dict(**identity,status='FAILED',at=current,error=repr(exc)))
        raise


if __name__ == '__main__':
    main()
