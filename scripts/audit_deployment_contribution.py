"""Post-hoc matched deployment audit of existing frozen 0.45-prior episodes.

No new trajectories, seeds, success definitions or independent-validation claim.
"""
from __future__ import annotations
import os
for _name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_name]='1'
import gc
import shutil
import numpy as np
from research_io import ROOT, read, sha, guard
from dbact.cargo import Cargo
from dbact.geometry import signed_distance_to_polygon, rotate
from dbact.metrics import maximum_uncovered_boundary_arc
from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result,verify_source_snapshot

OUT=ROOT/'runs/maintenance/2026-11-03-deployment-contribution-audit'
PUBLIC=ROOT/'docs/results/deployment_contribution_v1'
SOURCE_PROTOCOL=ROOT/'experiments/ccta2027/transport_validation_v1_new_starts.json'
METHODS=('boundary_cvt','local_spacing','nearest_observed_target')


def main():
    guard();OUT.mkdir(exist_ok=False)
    plan=read(SOURCE_PROTOCOL);source=ROOT/plan['run_dir'];manifest=read(source/'manifest.json')
    assert manifest['identity']['protocol_sha256']==sha(SOURCE_PROTOCOL)
    verify_source_snapshot(source,manifest['source_files'])
    for name in ('src/dbact/cargo.py','src/dbact/geometry.py','src/dbact/metrics.py','src/dbact/contracts.py'):
        assert sha(ROOT/name)==manifest['source_files'][name]
    cases=[c for c in plan['cases'] if c['arm']=='candidate']
    assert len(cases)==36
    groups={condition:[c for c in cases if c['condition_id']==condition] for condition in sorted({c['condition_id'] for c in cases})}
    protocol=dict(kind='post_hoc_matched_mechanism_audit_not_new_validation',source_protocol_sha256=sha(SOURCE_PROTOCOL),
        cases=[c['case_id'] for c in cases],source_sha256=manifest['identity']['source_sha256'],
        method='For each 3-method block, common horizon is earliest CONTACT_READY control timestamp. '
               'Twenty equal intervals, rounded to recorded frame times; 360 independent boundary samples, '
               'existing evaluation contact radius. Report trapezoidal time-average covered fraction and maximum uncovered arc, '
               'plus all-frame robot path through same horizon. No transport control hold has started by that horizon.',
        endpoints='Retain existing full-task completion, G500 and hard-safety classifications separately.',
        inference='Paired descriptions across 12 reused fixed blocks; no population probability or independent holdout assertion.',
        new_physical_episodes=0,runner_sha256=sha(__file__))
    dump(OUT/'frozen_protocol.json',protocol);shutil.copy2(__file__,OUT/'runner.py')
    rows=[];matched={};receipts={}
    for condition,block in groups.items():
        assert sorted(c['variant'] for c in block)==sorted(METHODS)
        assert len({encode(c['config']) for c in block})==1
        assert len({c['seed'] for c in block})==1
        results={}
        for case in block:
            folder=source/case['case_id'];expected={**manifest['identity'],'case_sha256':encode(case)}
            r=cached_result(folder,expected);assert r['execution_status']=='COMPLETED'
            results[case['variant']]=r;receipts[case['case_id']]=sha(folder/'case_result.json')
        assert len({r['initial_state_sha256'] for r in results.values()})==1
        assert all(r['contact_ready_frame'] is not None for r in results.values())
        last=min(r['contact_ready_frame'] for r in results.values()); assert last>0
        indices=np.unique(np.round(np.linspace(0,last,21)).astype(int))
        matched[condition]=dict(config_sha256=encode(block[0]['config']),seed=block[0]['seed'],
            initial_state_sha256=next(iter(results.values()))['initial_state_sha256'],
            common_horizon_s=last*block[0]['config']['dt'],sample_frames=indices.tolist())
        for case in block:
            method=case['variant'];folder=source/case['case_id'];r=results[method]
            gate=read(folder/'summary.json')['cargoes']['cargo_0']['g500'];m=gate['metrics']
            contract=ClosedLoopContract(**gate['gates']);v=contract.evaluate(m)
            assert v.success==gate['success'] and v.reasons==gate['failure_reasons']
            hard=[]
            if m['min_inter_agent_distance']<m['d_min']-contract.d_min_tolerance:hard.append('separation')
            if m['min_signed_clearance']<0:hard.append('inside_object')
            if m['max_penetration']>m['penetration_budget']:hard.append('penetration_budget')
            if m['solver_fallbacks'] or m['solver_infeasible']:hard.append('solver_failure')
            with np.load(folder/'replay.npz',allow_pickle=False) as z:
                robots=z['agent_positions'][:last+1].copy();xy=z['cargo/cargo_0/centers'][indices].copy()
                angles=z['cargo/cargo_0/angles'][indices].copy();local=z['cargo/cargo_0/local_vertices'].copy()
                times=z['times'][indices].copy()
            samples=[]
            for k,pos,angle,t in zip(indices,xy,angles,times):
                vertices=rotate(local,angle)+pos;cargo=Cargo('probe',vertices);boundary,_=cargo.boundary_samples(360)
                outside=signed_distance_to_polygon(robots[k],vertices)>=0.
                distances=np.linalg.norm(boundary[:,None,:]-robots[k][None,outside,:],axis=2)
                covered=np.any(distances<=case['config']['evaluation']['contact_radius'],axis=1)
                arc=maximum_uncovered_boundary_arc(covered,cargo.perimeter)
                samples.append(dict(time=float(t),coverage=float(covered.mean()),gap_upper_m=arc['max_uncovered_arc_upper_m']))
            coverage=np.array([s['coverage'] for s in samples]);gaps=np.array([s['gap_upper_m'] for s in samples])
            row=dict(condition=condition,method=method,stratum=case['stratum'],horizon_s=float(times[-1]),
                mean_coverage=float(np.trapezoid(coverage,times)/times[-1]),
                mean_gap_upper_m=float(np.trapezoid(gaps,times)/times[-1]),
                mean_robot_path_m=float(np.linalg.norm(np.diff(robots,axis=0),axis=2).sum(axis=0).mean()),
                contact_ready_s=r['contact_ready_frame']*case['config']['dt'],
                task_completed=r['task_completed'],g500=r['g500_pass'],hard_safety=hard,
                barrier_scalings=m['barrier_scalings'],endpoint_error_m=r['euclidean_endpoint_error'],samples=samples)
            rows.append(row);gc.collect()
        print(condition,'verified',flush=True)
    comparisons={}
    for other in METHODS[1:]:
        pairs=[]
        for condition in groups:
            a=next(r for r in rows if r['condition']==condition and r['method']=='boundary_cvt')
            b=next(r for r in rows if r['condition']==condition and r['method']==other)
            pairs.append(dict(condition=condition,coverage_delta=a['mean_coverage']-b['mean_coverage'],
                gap_reduction_m=b['mean_gap_upper_m']-a['mean_gap_upper_m'],
                path_delta_m=a['mean_robot_path_m']-b['mean_robot_path_m'],
                readiness_delta_s=a['contact_ready_s']-b['contact_ready_s']))
        comparisons[other]=dict(pairs=pairs,mean_coverage_delta=float(np.mean([p['coverage_delta'] for p in pairs])),
            coverage_wins=sum(p['coverage_delta']>1e-9 for p in pairs),coverage_losses=sum(p['coverage_delta']<-1e-9 for p in pairs),
            mean_gap_reduction_m=float(np.mean([p['gap_reduction_m'] for p in pairs])),
            mean_path_delta_m=float(np.mean([p['path_delta_m'] for p in pairs])),
            mean_readiness_delta_s=float(np.mean([p['readiness_delta_s'] for p in pairs])))
    totals={method:dict(completed=sum(r['task_completed'] for r in rows if r['method']==method),
        g500=sum(r['g500'] for r in rows if r['method']==method),
        hard_failures=sum(bool(r['hard_safety']) for r in rows if r['method']==method),
        barrier_scalings=sum(r['barrier_scalings'] for r in rows if r['method']==method)) for method in METHODS}
    PUBLIC.mkdir(exist_ok=False)
    dump(PUBLIC/'analysis.json',dict(protocol=protocol,matched_blocks=matched,episodes=rows,comparisons=comparisons,
        totals=totals,receipt_sha256=receipts,analysis_sha256=sha(__file__)))
    shutil.copy2(OUT/'frozen_protocol.json',PUBLIC/'analysis_protocol.json')
    print(totals);print({k:{q:v for q,v in r.items() if q!='pairs'} for k,r in comparisons.items()})


if __name__=='__main__':main()
