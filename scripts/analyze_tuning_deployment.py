"""Separate safety through CONTACT_READY from later transport failures."""
import argparse
import gc
import json
import numpy as np
from prepare_transport_tuning import ROOT, MISSION, sha
from dbact.contracts import ClosedLoopContract
from dbact.geometry import signed_distance_to_polygon, rotate
from dbact.cpu_budget import limit_blas_threads
from analyze_feedback_components import csv_rows

def read(path):return json.loads(path.read_text(encoding='utf-8'))

def main(stage):
    limit_blas_threads(1)
    dest=ROOT/'artifacts/research'/MISSION/stage/'deployment_safety.json'
    if dest.exists():raise RuntimeError('Preserve existing analysis')
    plan=read(ROOT/f'experiments/ccta2027/transport_tuning_v1_{stage}.json')
    rows=[]
    for c in plan['cases']:
        folder=ROOT/plan['run_dir']/c['case_id']
        receipt=read(folder/'case_result.json')['result']
        r=dict(case_id=c['case_id'],condition=c['condition_id'],method=c['variant'],arm=c['arm'],
               ready=receipt.get('contact_ready_frame') is not None,safe_ready=False)
        if receipt['execution_status']!='COMPLETED' or not r['ready']:
            rows.append(r);continue
        last=receipt['contact_ready_frame']+1  # saved physical frames start at1
        gate=read(folder/'summary.json')['cargoes']['cargo_0']['g500'];m=gate['metrics']
        contract=ClosedLoopContract(**gate['gates'])
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            xy=z['agent_positions'][:last+1].copy()
            centers=z['cargo/cargo_0/centers'][:last+1].copy()
            angles=z['cargo/cargo_0/angles'][:last+1].copy()
            vertices=z['cargo/cargo_0/local_vertices'].copy()
        distance=np.linalg.norm(xy[:,:,None,:]-xy[:,None,:,:],axis=-1)
        distance[:,np.arange(xy.shape[1]),np.arange(xy.shape[1])]=np.inf
        nearest=float(np.min(distance))
        substeps=[s for s in read(folder/'physics_substeps.json') if s['frame']<=last]
        assert substeps
        post_clearance=min(float(np.min(signed_distance_to_polygon(xy[k],rotate(vertices,angles[k])+centers[k])))
                           for k in range(len(xy)))
        clearance=min(post_clearance,min(s['signed_clearance_min'] for s in substeps))
        penetration=max(max(0.,c['config']['controller']['robot_radius']-post_clearance),
                        max(s['penetration_max'] for s in substeps))
        steps=read(folder/'paired_metrics.json')['steps'];solver=steps[last-1]['solver_statistics']
        scaling_by_phase={};previous=0
        for s in steps:
            delta=s['solver_statistics']['barrier_scalings']-previous
            assert delta>=0
            if delta:scaling_by_phase[s['phase']]=scaling_by_phase.get(s['phase'],0)+delta
            previous=s['solver_statistics']['barrier_scalings']
        assert previous==m['barrier_scalings']
        r.update(ready_physical_seconds=last*c['config']['dt'],min_separation_m=nearest,
            min_substep_signed_clearance_m=clearance,max_substep_penetration_m=penetration,
            barrier_scalings_until_ready=solver['barrier_scalings'],
            solver_fallbacks_until_ready=solver['fallbacks'],solver_infeasible_until_ready=solver['infeasible'],
            safe_ready=bool(nearest>=m['d_min']-contract.d_min_tolerance and clearance>=0 and
                penetration<=m['penetration_budget'] and solver['fallbacks']==solver['infeasible']==0),
            whole_run_scalings_by_phase=scaling_by_phase)
        rows.append(r);del steps,substeps;gc.collect()
    totals={arm:dict(cases=sum(r['arm']==arm for r in rows),
        reached_ready=sum(r['ready'] for r in rows if r['arm']==arm),
        safe_through_ready=sum(r['safe_ready'] for r in rows if r['arm']==arm),
        scaling_events_through_ready=sum(r.get('barrier_scalings_until_ready',0) for r in rows if r['arm']==arm))
        for arm in dict.fromkeys(r['arm'] for r in rows)}
    result=dict(totals=totals,episodes=rows,script_sha256=sha(__file__),
        definition='CONTACT_READY exists, original hard-safety geometric and solver limits hold through its first physical frame. Barrier scaling reported separately, not redefined as hard failure.',
        sampling='All saved robot frames and existing physics pre-integration substeps through first readiness; not a continuous-time guarantee.')
    dest.write_text(json.dumps(result,indent=2),encoding='utf-8')
    csv_rows(dest.with_suffix('.csv'),rows)
    print(json.dumps(totals))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',choices=['screen','transfer'],default='screen')
    main(p.parse_args().stage)
