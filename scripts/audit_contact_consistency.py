"""Reconstruct frozen maps and inspect terminal safety rows; no new physics.

Historical poses feed only the original sensor and the independent evaluator.
Every saved map snapshot is checked exactly. Terminal filtered commands are
recomputed from saved nominal commands; candidate methods are not executed.
"""
from pathlib import Path
import argparse, collections, hashlib, json, sys, time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-02-contact-consistency-v1'
OLD=ROOT/'runs/research/2026-11-02-contact-geometry-v1/run'

def read(p): return json.loads(p.read_text(encoding='utf-8'))
def sha(p):
    with p.open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()

def main(name):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    assert name in ('c_open_away_patch','circle_combined')
    out=ROOT/'artifacts/research'/MISSION/name
    out.mkdir(exist_ok=False)
    manifest=read(OLD/'manifest.json')
    for p,h in manifest['source_files'].items(): assert sha(OLD/'source_snapshot'/p)==h,p
    sys.path.insert(0,str(OLD/'source_snapshot/src'))
    from dbact.control.controller import _merge_scans
    from dbact.cpu_budget import limit_blas_threads
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.study import cached_result
    from dbact_sim.experiments.deployment import dump
    limit_blas_threads(1)
    folder=OLD/name;case=read(folder/'resolved_case.json');receipt=read(folder/'case_result.json')
    cached_result(folder,receipt['identity'])
    steps=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z: saved={k:z[k].copy() for k in z.files}
    env=SimulationEnvironment(case['config'],seed=case['seed']);c=env.controller
    assert c.params.perception_every==1 and not c.params.estimate_object_yaw
    c._ensure_state(env.agents);oid=env.cargoes[0].object_id
    offset=np.r_[0,np.cumsum(saved['sensed_counts'])]
    records=[];counts=collections.Counter();started=time.perf_counter();n=len(steps)
    for k,s in enumerate(steps):
        if time.perf_counter()-started>2400: raise TimeoutError('Fixed shadow replay time budget')
        c._frame=k;c._time=float(saved['times'][k]);cargo=env.cargoes[0]
        cargo.set_pose(saved[f'cargo/{oid}/centers'][k],saved[f'cargo/{oid}/angles'][k])
        for i,a in enumerate(env.agents): a.position=saved['agent_positions'][k,i].copy()
        neighbors=c._neighbor_indices(env.agents)
        scans=[c.sensor.sense_view(a,env.cargoes,c._time) for a in env.agents]
        for i,a in enumerate(env.agents):
            m=c.maps[a.agent_id];m.register(scans[i],env.dt)
            if m.carve_enabled:m.carve(a.position,scans[i])
            batch,codes=_merge_scans(scans,[i]+neighbors[i]);m.update(batch,c._time,agent_codes=codes,dt=env.dt)
            c._views[a.agent_id]=m.view(c._time)
            assert len(scans[i])==s['diagnostics'][i]['own_scan_points']
            assert len(c._views[a.agent_id])==s['diagnostics'][i]['map_points']
        np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
            saved['sensed_points'][offset[k+1]:offset[k+2]])
        counts['exact_map_snapshot_frames']+=1
        if k>=n-200:
            signed,true_normals,_=cargo.signed_distance(saved['agent_positions'][k])
            for i,a in enumerate(env.agents):
                d=s['diagnostics'][i];view=c._views[a.agent_id];m=c.maps[a.agent_id]
                points,normals,vel,pvel=c._object_rows_from_map(a.agent_id,a.position,view)
                trace={};A,b,free,h=c.safety._object_rows(a.position,points,normals,vel,pvel,trace=trace)
                result=c.safety.filter_velocity(a.position,np.asarray(d['nominal_command']),
                    [env.agents[j].position for j in neighbors[i]],boundary_points=points,
                    boundary_normals=normals,object_velocity=vel,boundary_point_velocities=pvel)
                np.testing.assert_array_equal(result.velocity,s['commands'][i])
                assert result.object_rows_active==d['object_rows_active']
                counts['exact_terminal_commands']+=1
                if not d['push_side']:continue
                residual=A@result.velocity-b
                scan=scans[i]
                records.append(dict(frame=k,agent=a.agent_id,position=a.position.tolist(),
                    true_gap=float(signed[i]),true_normal=true_normals[i].tolist(),
                    nominal=d['nominal_command'],filtered=result.velocity.tolist(),
                    contact_normal=d['contact_selected_normal'],used_gap=d['contact_used_gap'],
                    object_velocity=vel.tolist(),reference_velocity=d['reference_velocity'],
                    object_rows_active=d['object_rows_active'],agent_rows_active=d['agent_rows_active'],
                    residual=residual.tolist(),trace=trace,map_points=points.tolist(),
                    map_normals=normals.tolist(),map_age=(c._time-m._timestamp).tolist(),
                    scan_points=scan.points.tolist(),scan_normals=scan.normals.tolist(),
                    scan_confidence=scan.confidence.tolist(),scan_arc=scan.arc_length.tolist()))
        if k%1000==0:print(json.dumps(dict(case=name,frame=k,seconds=round(time.perf_counter()-started,1))),flush=True)
    errs=[];speeds=[];active=[]
    for r in records:
        if r['object_rows_active'] and not r['agent_rows_active']:
            active.append(r);errs.extend(np.asarray(r['trace']['h'])+c.safety.params.r_safe-r['true_gap'])
            speeds.extend(np.asarray(r['trace']['demand'])+c.safety.params.gamma_obj*np.asarray(r['trace']['h']))
    def stats(x):
        x=np.asarray(x,float);return dict(n=len(x),mean=float(x.mean()) if len(x) else None,
            min=float(x.min()) if len(x) else None,max=float(x.max()) if len(x) else None)
    summary=dict(case=name,verification=dict(counts),pusher_records=len(records),active_object_only_records=len(active),
        active_plane_minus_true_gap_m=stats(errs),active_normal_velocity_mps=stats(speeds),
        seconds=time.perf_counter()-started,receipt_sha256=sha(folder/'case_result.json'),
        frozen_source=manifest['source_files'],script_sha256=sha(Path(__file__)),
        scope='All map snapshots exact; all final 200-frame commands exact. Detailed rows only nominal pushers in final10s. Not whole-episode force proof.')
    dump(out/'rows.json',records);dump(out/'summary.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='frozen_source'}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True);main(p.parse_args().case)
