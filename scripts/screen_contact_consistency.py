"""Same-input algebraic correction on verified terminal rows, without physics."""
import json
from pathlib import Path
import numpy as np
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump
from dbact.cpu_budget import limit_blas_threads
from audit_contact_consistency import ROOT, MISSION, OLD, read, sha

def stats(x):
    x=np.asarray(x,float)
    return dict(n=len(x),mean=float(x.mean()),median=float(np.median(x)),
                p95=float(np.quantile(x,.95)),min=float(x.min()),max=float(x.max()))

def main():
    limit_blas_threads(1);report=[];dest=ROOT/'artifacts/research'/MISSION/'screen.json'
    if dest.exists():raise RuntimeError('Preserve previous screen')
    for name in ('c_open_away_patch','circle_combined'):
        base=ROOT/'artifacts/research'/MISSION/name
        rows=read(base/'rows.json');summary=read(base/'summary.json')
        case=read(OLD/name/'resolved_case.json');env=SimulationEnvironment(case['config'],seed=case['seed'])
        old=env.controller.safety
        from dataclasses import replace
        from dbact.safety.filter import SafetyFilter
        new=SafetyFilter(replace(old.params,aggregate_frame_consistent=True))
        with np.load(OLD/name/'replay.npz',allow_pickle=False) as z:positions=z['agent_positions'].copy()
        olderr=[];newerr=[];shifts=[];opening=0;removed=0;examples=[];translation_errors=[]
        for r in rows:
            p,pts,ns,v=[np.asarray(r[k],float) for k in ('position','map_points','map_normals','object_velocity')]
            ao,bo,fo,ho=old._object_rows(p,pts,ns,v)
            an,bn,fn,hn=new._object_rows(p,pts,ns,v)
            np.testing.assert_array_equal(ao,an)
            delta=np.array([-3.,-2.]);am,bm,fm,hm=new._object_rows(p+delta,pts+delta,ns,v)
            translation_errors.append(float(np.max(np.abs(hm-hn))))
            np.testing.assert_allclose(am,an,atol=1e-11,rtol=0)
            np.testing.assert_allclose(bm,bn,atol=1e-11,rtol=0)
            neighbors=[q for q in positions[r['frame']] if 1e-9<np.linalg.norm(q-p)<=env.controller.params.comm_range]
            u=new.filter_velocity(p,r['nominal'],neighbors,pts,ns,v).velocity
            n=np.array(r['contact_normal']);oldin=-float(np.array(r['filtered'])@n);newin=-float(u@n)
            blocked=r['true_gap']>env.controller.params.robot_radius and r['object_rows_active']>0 and r['agent_rows_active']==0
            removed+=int(blocked);opening+=int(blocked and oldin<=0 and newin>0)
            olderr.extend(abs(ho+old.params.r_safe-r['true_gap']));newerr.extend(abs(hn+new.params.r_safe-r['true_gap']))
            shifts.extend(hn-ho)
            if blocked and len(examples)<6:
                examples.append(dict(frame=r['frame'],agent=r['agent'],true_gap=r['true_gap'],
                    old_plane_gap=(ho+old.params.r_safe).tolist(),new_plane_gap=(hn+new.params.r_safe).tolist(),
                    old_inward=oldin,new_inward=newin,old_normal_velocity=r['object_velocity'],
                    nearest_map_age_s=float(np.asarray(r['map_age'])[np.linalg.norm(pts-p,axis=1).argmin()])))
        report.append(dict(case=name,records=len(rows),old_plane_absolute_error_m=stats(olderr),
            new_plane_absolute_error_m=stats(newerr),corrected_minus_old_h_m=stats(shifts),
            outside_with_object_active=removed,outside_retreat_becomes_inward=opening,
            max_translated_row_error=max(translation_errors),examples=examples,
            source_rows_sha256=sha(base/'rows.json'),verification=summary['verification']))
    dump(dest,dict(cases=report,scope='Identical saved inputs, all terminal nominal pushers. Ground truth only evaluates plane error. Filtered commands are counterfactual, not executed trajectories. No safety or completion certificate.',
        script_sha256=sha(Path(__file__)),safety_source_sha256=sha(ROOT/'src/dbact/safety/filter.py')))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
