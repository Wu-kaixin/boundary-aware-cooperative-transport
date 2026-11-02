"""Sampled, evaluator-only static wrench capacities on preserved trajectories.

No dynamics are advanced. A zero-side-force, zero-torque wrench is a diagnostic
target, not a necessary condition for eventual transport. Friction-cone choices
are optimistic: the physical slip law need not realize them simultaneously.
"""
import argparse, json, warnings
from pathlib import Path
import numpy as np
from scipy.optimize import linprog, OptimizeWarning
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump
from dbact.cpu_budget import limit_blas_threads
from analyze_dynamic_contact_study import read, sha
from freeze_contact_geometry import ROOT, MISSION, HISTORY, CONDITIONS


def capacity(points, normals, center, direction, normal_limit, friction):
    n=len(points)
    if not n:return 0.
    inward=-np.asarray(normals);tangent=np.c_[-normals[:,1],normals[:,0]]
    forces=np.stack((inward,tangent),axis=1).reshape(-1,2)
    lever=np.repeat(points-center,2,axis=0)
    side=np.array([-direction[1],direction[0]])
    equality=np.vstack((forces@side,lever[:,0]*forces[:,1]-lever[:,1]*forces[:,0]))
    cone=np.zeros((2*n,2*n))
    for i in range(n):
        cone[2*i,2*i:2*i+2]=[-friction,1.]
        cone[2*i+1,2*i:2*i+2]=[-friction,-1.]
    # SciPy forwards the backend thread option; one thread avoids competing
    # with the separately frozen eight simulation workers.
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore',category=OptimizeWarning,message='Unrecognized options detected.*')
        r=linprog(-forces@direction,A_ub=cone,b_ub=np.zeros(2*n),
            A_eq=equality,b_eq=np.zeros(2),bounds=[b for _ in range(n) for b in ((0.,normal_limit),(None,None))],
            method='highs',options={'threads':1})
    if not r.success:raise RuntimeError(r.message)
    assert np.max(np.abs(equality@r.x))<1e-7
    assert np.max(cone@r.x)<1e-7
    return max(0.,-float(r.fun))


def main(out):
    assert out.resolve().is_relative_to(ROOT/'artifacts/research'/MISSION)
    out.mkdir(exist_ok=False);limit_blas_threads(1)
    old=read(HISTORY/'resolved_protocol.json');rows=[];inputs={}
    for condition in CONDITIONS:
        c=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='reference')
        p=HISTORY/c['case_id']/'replay.npz';inputs[p.relative_to(ROOT).as_posix()]=sha(p)
        env=SimulationEnvironment(c['config'],seed=c['seed']);cargo=env.cargoes[0];params=env.contact_params
        with np.load(p,allow_pickle=False) as z:
            times=z['times'];poses=z['agent_positions'];centers=z['cargo/cargo_0/centers'];angles=z['cargo/cargo_0/angles']
            goal=z['task/cargo_0/goal_point']
            for time in (150.,300.,600.):
                k=int(np.argmin(np.abs(times-time)));assert abs(times[k]-time)<1e-9
                center=centers[k];cargo.set_pose(center,angles[k])
                signed,normals,points=cargo.signed_distance(poses[k]);touch=signed<params.robot_radius
                d=goal-center;d=d/max(np.linalg.norm(d),1e-12)
                for name,depth in [('nominal_press',params.robot_radius-env.controller._press_floor),
                                   ('hard_penetration_budget',env.controller.params.delta_max)]:
                    bound=params.stiffness*depth
                    rows.append(dict(condition=condition,time_s=time,geometric_contacts=int(touch.sum()),
                        depth_model=name,per_contact_force_limit_N=bound,
                        normal_only_balanced_capacity_N=capacity(points[touch],normals[touch],center,d,bound,0.),
                        friction_cone_balanced_capacity_N=capacity(points[touch],normals[touch],center,d,bound,params.friction),
                        ground_breakaway_N=cargo.mass*params.ground_friction*params.gravity))
    dump(out/'analysis.json',dict(rows=rows,inputs=inputs,script_sha256=sha(Path(__file__)),
        scope='Three reference trajectories, three predetermined timestamps, two existing depth budgets. Geometry truth is used by the independent evaluator only; no physics steps or controller input.',
        limits='A sampled static wrench feasibility diagnostic, not an overall transport impossibility proof. Does not establish feasible robot repositioning, slip realization, dynamic contact or safety. Nonzero torque can be held by ground friction or allowed temporarily by the actual task.'))
    print(json.dumps(rows,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);main(p.parse_args().out)
