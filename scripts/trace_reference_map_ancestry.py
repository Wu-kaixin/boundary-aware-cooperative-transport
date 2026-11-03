"""Capture reference-driven map operations without changing frozen commands."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import shutil
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from replay_qp_conflict import recorded

PLAN = ROOT/'experiments/ccta2027/map_ancestry_v1_plan.md'
BASE = ROOT/'runs/research/2026-11-03-map-ancestry-v1'


def trace(end, name):
    guard()
    import diagnose_pair_residuals as prior
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result
    plan, frozen = prior.check()
    case_id = prior.CASES['skew'][0]
    assert 0 < end <= 989
    case = next(x for x in plan['cases'] if x['case_id'] == case_id)
    folder = ROOT/plan['run_dir']/case_id
    manifest = read(ROOT/plan['run_dir']/'manifest.json')
    result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert result['execution_status'] == 'COMPLETED'
    target = BASE/name
    target.mkdir(parents=True, exist_ok=False)
    script_hash, plan_hash = sha(__file__), sha(PLAN)
    shutil.copy2(__file__, target/'trace_script.py')
    shutil.copy2(PLAN, target/'plan.md')
    identity = dict(case_id=case_id, agent_id='agent_05', end_frame=end,
        runtime_source_sha256=plan['frozen_source_sha256'],
        physical_protocol_sha256=sha(ROOT/'experiments/ccta2027/local_pair_safety_v1.json'),
        prior_diagnostic_protocol_sha256=sha(prior.PROTOCOL),
        original_receipt_sha256=sha(folder/'case_result.json'),
        script_sha256=script_hash, plan_sha256=plan_hash, environment=plan['environment'],
        new_physical_episodes=0)
    dump(target/'started.json', identity)
    rows = read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        saved = {k:z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers',
            'cargo/cargo_0/angles','sensed_counts','sensed_points')}
    offsets = np.r_[0,np.cumsum(saved['sensed_counts'])]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c = env.controller
    c.trace_enabled = True
    c._ensure_state(env.agents)
    assert c.params.perception_every == c.params.planning_every == 1
    aid = 'agent_05'
    local = c.maps[aid]
    index = next(i for i,a in enumerate(env.agents) if a.agent_id == aid)
    frame_record = {}

    def state():
        return {name:getattr(local,'_'+attribute).copy() for name,attribute in
            [('keys','keys'),('points','points'),('normals','normals'),('weights','weight'),
             ('timestamps','timestamp'),('confidence','confidence'),('arc','arc'),('objects','objects')]}

    def scan_state(scan):
        return {name:getattr(scan,name).copy() for name in
                ('points','normals','confidence','arc_length','object_ids')}

    original_align, original_register = local.align_translation, local.register
    original_carve, original_update = local.carve, local.update

    def align(object_id, displacement, velocity):
        frame_record['before_align'] = state()
        frame_record['prior_displacement'] = local.object_displacement(object_id)
        frame_record['alignment'] = dict(object_id=object_id,
            displacement=displacement.copy(), velocity=velocity.copy())
        result = original_align(object_id, displacement, velocity)
        frame_record['after_align'] = state()
        return result

    def register(scan, dt, **kwargs):
        frame_record['own_scan'] = scan_state(scan)
        result = original_register(scan, dt, **kwargs)
        frame_record['register_kwargs'] = kwargs
        frame_record['registrations'] = {k:asdict(v) for k,v in result.items()}
        frame_record['after_register'] = state()
        return result

    def carve(origin, scan):
        frame_record['carve_origin'] = origin.copy()
        result = original_carve(origin, scan)
        frame_record['carved'] = result
        frame_record['after_carve'] = state()
        return result

    def update(observations, timestamp, agent_codes=None, dt=0., **kwargs):
        frame_record['before_update'] = state()
        frame_record['one_hop_scan'] = scan_state(observations)
        frame_record['source_codes'] = agent_codes.copy()
        result = original_update(observations, timestamp, agent_codes=agent_codes, dt=dt, **kwargs)
        frame_record['update_kwargs'] = kwargs
        frame_record['after_update'] = state()
        return result

    local.align_translation, local.register, local.carve, local.update = align, register, carve, update
    verified, frame = 0, 0
    started = time.perf_counter()
    try:
        with gzip.open(target/'map_stages.jsonl.gz','wt',encoding='utf-8') as stream:
            for k in range(end):
                frame = k+1
                row = rows[k]
                env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k],saved['cargo/cargo_0/angles'][k])
                for i,agent in enumerate(env.agents):
                    agent.position = saved['agent_positions'][k,i].copy()
                    agent.velocity = np.zeros(2) if k==0 else np.asarray(rows[k-1]['commands'][i])
                frame_record = dict(frame=frame,time=float(saved['times'][k]),
                    robot_position=env.agents[index].position.copy(),
                    true_center=saved['cargo/cargo_0/centers'][k],
                    true_angle=saved['cargo/cargo_0/angles'][k], at_start=state())
                commands = c.step(env.agents,env.cargoes,float(saved['times'][k]),env.dt)
                np.testing.assert_array_equal([x.velocity for x in commands],row['commands'])
                assert recorded([vars(d) for d in c.diagnostics]) == row['diagnostics'], ('diagnostics',frame)
                assert recorded(c.safety.stats.as_dict()) == row['solver_statistics'], ('statistics',frame)
                assert c.phase.name == row['phase'] and recorded(vars(c.phase_signals)) == row['phase_guards']
                assert recorded(c.phase_monitor.as_dict()) == row['phase_response']
                np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                frame_record.update(at_end=state(),phase=c.phase.name,
                    acquisition_status=c.initial_reference_status,
                    reference_state=c._map_reference_state[aid],
                    fresh_readings={key:asdict(value) for key,value in c._reference_readings.items()},
                    diagnostics=row['diagnostics'],voxel_size=local.voxel_size,
                    object_index=local._object_index.copy(),max_weight=local.max_weight,
                    carve_enabled=local.carve_enabled)
                # Truth is queried only after commands and all exact checks.
                frame_record['stage_true_gaps'] = {key:env.cargoes[0].signed_distance(
                    np.asarray(value['points']).reshape(-1,2))[0] for key,value in frame_record.items()
                    if isinstance(value,dict) and 'points' in value and len(value['points'])}
                stream.write(json.dumps(plain(frame_record),allow_nan=False,separators=(',',':'))+'\n')
                verified += 1
                if verified % 100 == 0:
                    print(json.dumps(dict(verified_frames=verified,phase=c.phase.name)),flush=True)
        prior.check()
        assert sha(__file__)==script_hash and sha(PLAN)==plan_hash
        dump(target/'outcome.json',dict(**identity,status='EXACT_RECONSTRUCTION',verified_frames=verified,
            elapsed_seconds=time.perf_counter()-started,product_sha256=sha(target/'map_stages.jsonl.gz')))
        print(json.dumps(dict(status='EXACT_RECONSTRUCTION',verified_frames=verified)),flush=True)
    except Exception as exc:
        dump(target/'failure.json',dict(**identity,status='INVALID_RECONSTRUCTION',verified_frames=verified,
            failing_frame=frame,error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--end-frame',type=int,default=989)
    parser.add_argument('--name',required=True)
    args = parser.parse_args()
    assert Path(args.name).name == args.name and args.name not in ('.','..')
    trace(args.end_frame,args.name)
