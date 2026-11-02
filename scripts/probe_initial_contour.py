"""Fixed offline contour probe on all four completed acquisition trajectories.

No new physics or controller changes. Contour fit receives only own/one-hop
observations. True initial center is used only to score the returned fit.
"""
from __future__ import annotations
import os
for _name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_name]='1'
import json
import shutil
import numpy as np
from scipy.spatial import cKDTree
from shapely import MultiPoint, concave_hull
from research_io import ROOT, read, sha, guard
from dbact.control.controller import _merge_scans
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump

OUT=ROOT/'runs/maintenance/2026-11-03-initial-contour-probe-v2'
SOURCE=ROOT/'runs/research/2026-11-02-task-reference-acquisition-v1'


def contour(points,normals,noise):
    if len(points)<32:return dict(status='insufficient')
    hull=concave_hull(MultiPoint(points),ratio=0.,allow_holes=False)
    if hull.geom_type!='Polygon':return dict(status=hull.geom_type)
    vertices=np.asarray(hull.exterior.coords); area=float(hull.area)
    if not hull.exterior.is_ccw:vertices=vertices[::-1]
    a,b=vertices[:-1],vertices[1:]; length=np.linalg.norm(b-a,axis=1)
    edge_normals=np.c_[(b-a)[:,1],-(b-a)[:,0]]/np.maximum(length[:,None],1e-12)
    samples=[]; ns=[]
    for p,q,n,size in zip(a,b,edge_normals,length):
        count=max(1,int(np.ceil(size/(2*noise))))
        samples.extend(p+(q-p)*u for u in (np.arange(count)+.5)/count)
        ns.extend([n]*count)
    distance,index=cKDTree(points).query(np.array(samples))
    alignment=np.einsum('ij,ij->i',np.array(ns),normals[index])
    return dict(status='fitted',center=list(hull.centroid.coords)[0],area=area,
        perimeter=float(hull.length),vertices=len(vertices)-1,
        unsupported_fraction=float(np.mean((distance>3*noise)|(alignment<.5))),
        max_nearest_sample_gap=float(distance.max()))


def main():
    guard();limit_blas_threads(1);OUT.mkdir(exist_ok=False)
    source=read(ROOT/'experiments/ccta2027/task_reference_acquisition_v1.json')
    protocol=dict(source_protocol_sha256=sha(ROOT/'experiments/ccta2027/task_reference_acquisition_v1.json'),
        conditions=[c['condition_id'] for c in source['cases']],snapshots=['first_support','final'],
        method='Shapely concave_hull ratio=0, no holes; boundary samples every 2*noise; report unsupported fraction at distance>3*noise OR outward-normal dot<.5',
        decision='Diagnostic only. Fit centroid and low unsupported fraction do not certify complete hidden geometry. Never use contaminated final skew template as a zero reference.',
        runner_sha256=sha(__file__),new_physical_episodes=0)
    protocol['alignment_correction']='Direct-step acquisition replay starts after its first step; prepend constructor initial state. Require every reconstructed first-support voxel count to match the original receipt.'
    dump(OUT/'frozen_protocol.json',protocol)
    shutil.copy2(__file__,OUT/'runner.py')
    reports=[]
    for case in source['cases']:
        folder=SOURCE/(case['condition_id']+'_observed_standoff')
        result=read(folder/'acquisition_result.json'); epoch=result['first_support']['frame']
        env=SimulationEnvironment(case['config'],seed=case['seed']);p=env.controller.params
        cells=[{} for _ in env.agents]
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            robots=np.concatenate([np.array([[a.position for a in env.agents]]),z['agent_positions']])
            centers=np.vstack([env.cargoes[0].position,z['cargo/cargo_0/centers']])
            angles=np.r_[env.cargoes[0].angle,z['cargo/cargo_0/angles']];times=np.r_[0.,z['times']]
        assert np.all(centers[:epoch+1]==centers[0]) and np.all(angles[:epoch+1]==angles[0])
        for k in range(epoch):
            env.cargoes[0].set_pose(centers[k],angles[k])
            for i,agent in enumerate(env.agents):agent.position=robots[k,i].copy()
            scans=[env.controller.sensor.sense_view(a,env.cargoes,times[k]) for a in env.agents]
            neighbors=env.controller._neighbor_indices(env.agents)
            for i in range(len(env.agents)):
                batch,_=_merge_scans(scans,[i]+list(neighbors[i]))
                for point,normal in zip(batch.points,batch.normals):
                    cells[i].setdefault(tuple(np.floor(point/(2*p.range_noise_std)).astype(int)),np.r_[point,normal])
        initial={f'agent_{i:02d}':np.array(list(v.values())).reshape(-1,4) for i,v in enumerate(cells)}
        assert [len(v) for v in initial.values()]==[r['points'] for r in result['first_support']['local']], case['condition_id']
        np.savez_compressed(OUT/(case['condition_id']+'-first-support.npz'),**initial)
        with np.load(folder/'observed_templates.npz',allow_pickle=False) as z:
            final={key:z[key].copy() for key in z.files}
        for label,arrays in [('first_support',initial),('final',final)]:
            rows=[]
            for aid,points in arrays.items():
                fit=contour(points[:,:2],points[:,2:],p.range_noise_std)
                if 'center' in fit:fit['initial_center_error_m']=float(np.linalg.norm(np.array(fit['center'])-centers[0]))
                rows.append(dict(agent=aid,points=len(points),**fit))
            reports.append(dict(condition=case['condition_id'],snapshot=label,local=rows,
                uncontaminated=label=='first_support' or (result['final']['contact_samples']==0 and result['final']['max_rotation']==0 and result['final']['max_translation']==0)))
        print(case['condition_id'],'done',flush=True)
    dump(OUT/'analysis.json',dict(protocol=protocol,reports=reports))


if __name__=='__main__':main()
