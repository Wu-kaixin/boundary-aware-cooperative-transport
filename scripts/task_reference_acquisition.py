"""Frozen acquisition feasibility study, distinct from full transport success."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from dbact.control.controller import _merge_scans
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import source_hashes, environment_versions

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT/'experiments/ccta2027/task_reference_acquisition_v1.json'
OUT = ROOT/'runs/research/2026-11-02-task-reference-acquisition-v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze():
    if PROTOCOL.exists() or OUT.exists():
        raise RuntimeError('Preserve the existing acquisition study')
    old = json.loads((ROOT/'experiments/ccta2027/reference_origin_v1.json').read_text())
    names = {'near_circle', 'near_l_reference', 'far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0'}
    cases = [copy.deepcopy(c) for c in old['cases'] if c['arm']=='reference' and c['condition_id'] in names]
    assert len(cases)==4
    plan = dict(kind='acquisition_feasibility_not_transport_comparison', cases=cases,
        arms=['original_clearance', 'observed_standoff'], frames=1200, physical_seconds=60,
        interventions=dict(both='contact_quorum=n+1 disables CONTACT_READY; original deployment continues',
            observed_standoff='safety.r_safe=robot_radius+3*range_noise_std+2*max_speed*dt; no physical/evaluator changes'),
        support_proxy='At least 4 local initial templates with >=32 voxels, minor RMS>3*noise and normal angular gap<pi',
        epoch_evaluation='No contact and max true translation/rotation <=1e-9 before support; truth only evaluates, never controls',
        decision='Static support is necessary, not a center observability certificate or full-task success. Report all eight.',
        wall_seconds=1800, workers=4, source_files=source_hashes(), environment=environment_versions(),
        runner_sha256=digest(__file__))
    dump(PROTOCOL, plan)
    OUT.mkdir(parents=True)
    (OUT/'source_snapshot').mkdir()
    import shutil
    for name in plan['source_files']:
        dest=OUT/'source_snapshot'/name; dest.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(ROOT/name,dest)
    shutil.copy2(__file__, OUT/'runner.py')
    dump(OUT/'frozen_protocol.json',plan)


def support(rows, noise):
    if len(rows)<32:
        return dict(usable=False, points=len(rows), minor_rms=None, max_normal_gap=None)
    points=rows[:,:2]; normals=rows[:,2:]
    centered=points-points.mean(axis=0)
    minor=float(np.sqrt(max(0.,np.linalg.eigvalsh(centered.T@centered/len(points))[0])))
    angles=np.sort(np.mod(np.arctan2(normals[:,1],normals[:,0]),2*np.pi))
    gap=float(np.max(np.diff(np.r_[angles,angles[0]+2*np.pi])))
    return dict(usable=bool(minor>3*noise and gap<np.pi),points=len(rows),minor_rms=minor,max_normal_gap=gap)


def one(payload):
    case,arm,plan=payload
    limit_blas_threads(1)
    folder=OUT/(case['condition_id']+'_'+arm); folder.mkdir(exist_ok=False)
    env=SimulationEnvironment(case['config'],seed=case['seed']); c=env.controller;p=c.params
    c.trace_enabled=True
    c.phase_monitor.gates.contact_quorum=len(env.agents)+1
    if arm=='observed_standoff':
        c.safety.params.r_safe=p.robot_radius+3*p.range_noise_std+2*p.max_speed*env.dt
    cells=[{} for _ in env.agents]; rows=[]; initial=env.cargoes[0].position.copy(); theta0=env.cargoes[0].angle
    started=time.perf_counter(); first_supported=None; max_translation=max_rotation=0.; total_contacts=0
    for k in range(plan['frames']):
        if time.perf_counter()-started>plan['wall_seconds']:
            raise TimeoutError('Frozen acquisition wall budget')
        neighbors=c._neighbor_indices(env.agents)
        env.step()
        scans=[c.last_scans[a.agent_id] for a in env.agents]
        for i in range(len(env.agents)):
            batch,_=_merge_scans(scans,[i]+list(neighbors[i]))
            for point,normal in zip(batch.points,batch.normals):
                key=tuple(np.floor(point/(2*p.range_noise_std)).astype(int))
                cells[i].setdefault(key,np.r_[point,normal])
        cargo=env.cargoes[0]
        max_translation=max(max_translation,float(np.linalg.norm(cargo.position-initial)))
        max_rotation=max(max_rotation,abs(cargo.angle-theta0))
        total_contacts+=env.engine.last_reports[cargo.object_id].contact_count
        if k%10==0 or k+1==plan['frames']:
            info=[support(np.array(list(v.values())).reshape(-1,4),p.range_noise_std) for v in cells]
            count=sum(x['usable'] for x in info)
            record=dict(frame=k+1,time=(k+1)*env.dt,supporters=count,local=info,
                max_translation=max_translation,max_rotation=max_rotation,contact_samples=total_contacts)
            rows.append(record)
            if count>=4 and first_supported is None:first_supported=record
    env.save_outputs(folder)
    np.savez_compressed(folder/'observed_templates.npz',**{f'agent_{i:02d}':np.array(list(v.values())).reshape(-1,4) for i,v in enumerate(cells)})
    result=dict(condition=case['condition_id'],arm=arm,execution='COMPLETED',physical_seconds=plan['physical_seconds'],
        wall_seconds=time.perf_counter()-started,first_support=first_supported,
        static_support=bool(first_supported and first_supported['contact_samples']==0 and
            first_supported['max_translation']<=1e-9 and first_supported['max_rotation']<=1e-9),
        final=rows[-1],history=rows,source_files=plan['source_files'],runner_sha256=plan['runner_sha256'],
        actual_safety_parameters=vars(c.safety.params),actual_phase_gates=vars(c.phase_monitor.gates))
    dump(folder/'acquisition_result.json',result)
    print(case['condition_id'],arm,'static support',result['static_support'], 'motion',max_translation,flush=True)
    return result


def run():
    plan=json.loads(PROTOCOL.read_text())
    assert plan['runner_sha256']==digest(__file__) and plan['source_files']==source_hashes()
    assert plan['environment']==environment_versions()
    from concurrent.futures import ProcessPoolExecutor
    payloads=[(case,arm,plan) for case in plan['cases'] for arm in plan['arms']]
    with ProcessPoolExecutor(max_workers=plan['workers']) as pool:
        results=list(pool.map(one,payloads))
    dump(OUT/'analysis.json',dict(results=results,new_acquisition_episodes=len(results),new_transport_completions=0))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['freeze','run'])
    args=parser.parse_args()
    freeze() if args.action=='freeze' else run()
