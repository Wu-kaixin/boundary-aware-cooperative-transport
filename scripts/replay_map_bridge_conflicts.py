"""Freeze and reconstruct every observed map-bridge scaling state exactly."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'

import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import shutil
import time

import numpy as np

from research_io import ROOT,guard,read,sha
from trace_boundary_map import plain
from replay_qp_conflict import recorded

BASE=ROOT/'runs/research/2026-11-03-map-bridge-conflicts-v1'
DESIGN=ROOT/'experiments/ccta2027/map_bridge_conflicts_v1_design.md'
CASE='far_skew_box_s0_r0_pos0_map_bridge_candidate'


def state_record(frame, phase, target, reference_status, agents, cargo):
    """Snapshot evaluation-only world geometry after all commands are fixed."""
    return plain(dict(frame=frame, phase=phase, target=target,
        reference_status=reference_status, agents=agents,
        true_center=cargo.position, true_angle=cargo.angle, true_vertices=cargo.vertices))


def replay(preflight=False, output=None):
    guard()
    import map_bridge_study as study
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump,encode
    from dbact_sim.experiments.study import cached_result
    plan=study.check()
    case=next(c for c in plan['cases'] if c['case_id']==CASE)
    folder=ROOT/plan['run_dir']/CASE
    manifest=read(ROOT/plan['run_dir']/'manifest.json')
    result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
    assert result['execution_status']=='COMPLETED'
    screen=ROOT/'docs/results/map_reference_bridge_v1/residual_screen.json'
    assert sha(screen)==read(screen.parent/'residual_screen_receipt.json')['screen_sha256']
    events=read(screen)['events']
    assert len(events)==18 and all(e['case_id']==CASE for e in events)
    selected={e['saved_frame']:e['agent_id'] for e in events}
    assert len(selected)==18 and max(selected)==517
    end=20 if preflight else 517
    target=(Path(output) if output is not None else BASE/('preflight' if preflight else 'replay')).resolve()
    if not target.is_relative_to((ROOT/'runs').resolve()):
        raise ValueError('Replay output must be a fresh directory under runs/')
    target.mkdir(parents=True,exist_ok=False)
    helpers={name:sha(ROOT/name) for name in ('scripts/research_io.py',
        'scripts/trace_boundary_map.py','scripts/replay_qp_conflict.py','tests/test_map_bridge_conflicts.py')}
    source_identity=dict(source_sha256=plan['frozen_source_sha256'],physical_protocol_sha256=sha(study.PROTOCOL),
        original_receipt_sha256=sha(folder/'case_result.json'),screen_sha256=sha(screen),
        script_sha256=sha(__file__),design_sha256=sha(DESIGN),helpers=helpers,environment=plan['environment'])
    identity=dict(**source_identity,case_id=CASE,end_frame=end,preflight=preflight,
        selected=selected,new_physical_episodes=0)
    shutil.copy2(__file__,target/'replay_script.py')
    shutil.copy2(DESIGN,target/'design.md')
    for name in helpers:
        path=target/'source_snapshot'/name
        path.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,path)
    dump(target/'started.json',identity)
    rows=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        saved={k:z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers',
            'cargo/cargo_0/angles','sensed_counts','sensed_points')}
    offsets=np.r_[0,np.cumsum(saved['sensed_counts'])]
    env=SimulationEnvironment(case['config'],seed=case['seed'])
    c,safety=env.controller,env.controller.safety
    c.trace_enabled=True
    assert c.params.perception_every==c.params.planning_every==1
    ids=[a.agent_id for a in env.agents]
    original_prepare,original_finish=safety.prepare_velocity,safety.finish_velocity
    original_neighbors=c._neighbor_indices
    neighbors=[]
    frame,call_index,verified=0,0,0
    captured,problem_ids={},{}

    def linked(agents):
        nonlocal neighbors
        neighbors=original_neighbors(agents)
        return neighbors

    def prepare(position,nominal_velocity,neighbor_positions=(),**kwargs):
        nonlocal call_index
        i,call_index=call_index,call_index+1
        aid=ids[i]
        problem=original_prepare(position,nominal_velocity,neighbor_positions,**kwargs)
        problem_ids[id(problem)]=aid
        if frame in selected:
            mask=np.asarray(problem.row_kinds)=='object'
            support=kwargs.get('finite_support_trace') or {}
            trace={}
            if support.get('applied'):
                motion=np.full(problem.object_count,kwargs['finite_support_motion_bound'])
            elif problem.object_count:
                A,b,free,h=safety._object_rows(position,kwargs['boundary_points'],kwargs['boundary_normals'],
                    kwargs['object_velocity'],kwargs.get('boundary_point_velocities'),trace=trace)
                np.testing.assert_array_equal(A,problem.A[mask])
                np.testing.assert_array_equal(b,problem.b_original[mask])
                np.testing.assert_array_equal(free,problem.b_no_margin[mask])
                np.testing.assert_array_equal(h,problem.h_object)
                motion=np.asarray(trace['demand'])+safety.params.gamma_obj*h
            else:
                motion=np.empty(0)
            permitted=[aid]+[ids[j] for j in neighbors[i]]
            packets=c.sensor.finite_support_packets
            captured[aid]=plain(dict(agent_id=aid,position=position,neighbors=permitted[1:],
                A=problem.A,b=problem.b_original,b_no_margin=problem.b_no_margin,
                kinds=problem.row_kinds,h=problem.h_object,normal_motion=motion,nominal=problem.u_nom,
                params=asdict(problem.parameters),domain=c.domain,support=support,aggregate_trace=trace,
                object_velocity=kwargs['object_velocity'],point_velocities=kwargs.get('boundary_point_velocities'),
                boundary_points=kwargs['boundary_points'],boundary_normals=kwargs['boundary_normals'],
                permitted_packets={a:asdict(packets[a]) for a in permitted if a in packets},
                own_scan=asdict(c.last_scans[aid])))
        return problem

    def finish(problem,**kwargs):
        aid=problem_ids.pop(id(problem))
        result=original_finish(problem,**kwargs)
        if aid in captured:
            captured[aid].update(status=result.status,scale=result.barrier_scale,actual=result.velocity.tolist())
        return result

    c._neighbor_indices=linked
    safety.prepare_velocity,safety.finish_velocity=prepare,finish
    started=time.perf_counter()
    captured_frames=[]
    try:
        with gzip.open(target/'states.jsonl.gz','wt',encoding='utf-8') as stream:
            for k in range(end):
                frame,call_index=k+1,0
                captured.clear();problem_ids.clear()
                row=rows[k]
                cargo=env.cargoes[0]
                cargo.set_pose(saved['cargo/cargo_0/centers'][k],saved['cargo/cargo_0/angles'][k])
                for i,agent in enumerate(env.agents):
                    agent.position=saved['agent_positions'][k,i].copy()
                    agent.velocity=np.zeros(2) if k==0 else np.asarray(rows[k-1]['commands'][i])
                commands=c.step(env.agents,env.cargoes,float(saved['times'][k]),env.dt)
                assert call_index==len(ids) and not problem_ids
                np.testing.assert_array_equal([v.velocity for v in commands],row['commands'])
                assert recorded([vars(d) for d in c.diagnostics])==row['diagnostics'],('diagnostics',frame)
                assert recorded(safety.stats.as_dict())==row['solver_statistics'],('statistics',frame)
                assert c.phase.name==row['phase'] and recorded(vars(c.phase_signals))==row['phase_guards']
                assert recorded(c.phase_monitor.as_dict())==row['phase_response']
                np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                if captured:
                    # Every truth query is strictly after all unchanged commands.
                    for i,aid in enumerate(ids):
                        item=captured[aid]
                        gap,normal,foot=cargo.signed_distance(env.agents[i].position[None,:])
                        item.update(true_gap=float(gap[0]),true_normal=normal[0].tolist(),true_foot=foot[0].tolist(),
                            diagnostic=row['diagnostics'][i])
                        pts=np.asarray(item['boundary_points']).reshape(-1,2)
                        if len(pts):
                            gaps,normals,feet=cargo.signed_distance(pts)
                            item.update(map_true_gaps=gaps.tolist(),map_true_normals=normals.tolist(),map_true_feet=feet.tolist())
                    stream.write(json.dumps(state_record(frame,c.phase.name,selected[frame],
                        c.initial_reference_status,captured,cargo),
                        allow_nan=False,separators=(',',':'))+'\n')
                    captured_frames.append(frame)
                verified+=1
                if verified%100==0:
                    print(json.dumps(dict(verified_frames=verified,captured_states=len(captured_frames))),flush=True)
        study.check()
        assert sha(__file__)==identity['script_sha256'] and sha(DESIGN)==identity['design_sha256']
        assert helpers=={name:sha(ROOT/name) for name in helpers}
        assert captured_frames==sorted(k for k in selected if k<=end)
        dump(target/'outcome.json',dict(**identity,status='EXACT_RECONSTRUCTION',verified_frames=verified,
            captured_frames=captured_frames,elapsed_seconds=time.perf_counter()-started,
            product_sha256=sha(target/'states.jsonl.gz')))
        print(json.dumps(dict(status='EXACT_RECONSTRUCTION',verified_frames=verified,captured_states=len(captured_frames))),flush=True)
    except Exception as exc:
        dump(target/'failure.json',dict(**identity,status='INVALID_RECONSTRUCTION',verified_frames=verified,
            failing_frame=frame,error=repr(exc)))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preflight',action='store_true')
    parser.add_argument('--output',type=Path,help='Fresh directory under runs/; existing evidence is never overwritten')
    args=parser.parse_args()
    replay(args.preflight,args.output)
