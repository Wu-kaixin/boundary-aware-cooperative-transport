"""Compare integrated rows/admission against every previously frozen robot state."""
from __future__ import annotations

import gzip
import json
import shutil
from types import SimpleNamespace

import numpy as np

from research_io import ROOT, guard, read, sha
from probe_map_bridge_finite_support import packet_from_saved
from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.control.post_support import filter_inputs
from dbact_sim.experiments.study import source_hashes, environment_versions
from dbact_sim.experiments.deployment import dump, encode

OLD = ROOT/'runs/research/2026-11-03-map-bridge-conflicts-v1'
OUT = ROOT/'runs/research/2026-11-03-post-acquisition-support-v1/observer'


def run():
    guard()
    receipt=read(OLD/'replay-capture-v2/outcome.json')
    screen_receipt=read(OLD/'neighborhood-observer/outcome.json')
    assert sha(OLD/'replay-capture-v2/states.jsonl.gz')==receipt['product_sha256']
    assert sha(OLD/'neighborhood-observer/screen.json')==screen_receipt['screen_sha256']
    source=source_hashes()
    identity=dict(new_source_sha256=encode(source),source_files=source,
        original_source_sha256=receipt['source_sha256'],replay_sha256=receipt['product_sha256'],
        observer_sha256=screen_receipt['screen_sha256'],script_sha256=sha(__file__),
        environment=environment_versions(),new_physical_episodes=0)
    OUT.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__,OUT/'audit_script.py')
    for name in source:
        path=OUT/'source_snapshot'/name
        path.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,path)
    dump(OUT/'started.json',identity)
    checks=[]
    try:
        expected={(x['frame'],x['agent_id']):next(p['system'] for p in x['probes']
            if p['model']=='received_speed') for x in read(OLD/'neighborhood-observer/screen.json')['records']}
        with gzip.open(OLD/'replay-capture-v2/states.jsonl.gz','rt',encoding='utf-8') as stream:
            states=[json.loads(line) for line in stream]
        for state in states:
            for aid,saved in state['agents'].items():
                timestamp=(state['frame']-1)*saved['params']['dt']
                packets={key:packet_from_saved(value) for key,value in saved['permitted_packets'].items()}
                controller=SimpleNamespace(initial_reference_status=state['reference_status'],
                    _map_reference_state={aid:{'status':saved['diagnostic']['map_reference_status']}},
                    sensor=SimpleNamespace(finite_support_packets=packets))
                kwargs,trace=filter_inputs(controller,aid,[aid,*saved['neighbors']],timestamp)
                common=dict(position=np.asarray(saved['position']),nominal_velocity=np.asarray(saved['nominal']),
                    neighbor_positions=[state['agents'][key]['position'] for key in saved['neighbors']],
                    boundary_points=np.asarray(saved['boundary_points']).reshape(-1,2),
                    boundary_normals=np.asarray(saved['boundary_normals']).reshape(-1,2),
                    object_velocity=np.asarray(saved['object_velocity']),
                    boundary_point_velocities=saved['point_velocities'])
                support=saved['support']
                if support.get('source_agents'):
                    common.update(finite_support_packets=[packets[key] for key in support['source_agents']],
                        observation_time=timestamp,finite_support_motion_bound=support['motion_bound'])
                safety=SafetyFilter(SafetyFilterParams(**saved['params']))
                original=safety.prepare_velocity(**common)
                np.testing.assert_array_equal(original.A,np.asarray(saved['A']).reshape(-1,2))
                np.testing.assert_array_equal(original.b_original,saved['b'])
                np.testing.assert_array_equal(original.h_object,saved['h'])
                current=safety.prepare_velocity(**common,**kwargs)
                if trace.get('applied'):
                    wanted=expected[(state['frame'],aid)]
                    assert wanted['ready']
                    np.testing.assert_array_equal(current.A,wanted['A'])
                    np.testing.assert_array_equal(current.b_original,wanted['b'])
                    np.testing.assert_array_equal(current.h_object,wanted['h'])
                    assert trace['cover_motion_bound']==wanted['cover_motion_bound']==.2
                    assert trace['demand_speed']==wanted['normal_motion'][0]
                    assert trace['source_agents']==wanted['source_agents']
                else:
                    np.testing.assert_array_equal(current.A,original.A)
                    np.testing.assert_array_equal(current.b_original,original.b_original)
                    np.testing.assert_array_equal(current.h_object,original.h_object)
                    if state['frame']!=402:
                        assert not expected[(state['frame'],aid)]['ready']
                        assert trace['reason']==expected[(state['frame'],aid)]['reason']
                checks.append(dict(frame=state['frame'],agent_id=aid,applied=trace.get('applied',False),
                    reason=trace['reason'],original_rows_exact=True,integrated_rows_exact=True))
        assert len(checks)==288 and sum(x['applied'] for x in checks)==181
        assert source_hashes()==source and sha(__file__)==identity['script_sha256']
        dump(OUT/'checks.json',dict(**identity,checks=checks))
        dump(OUT/'outcome.json',dict(**identity,status='EXACT_INTEGRATED_OBSERVER',robot_states=288,
            admitted=181,product_sha256=sha(OUT/'checks.json')))
        print(json.dumps(dict(status='EXACT_INTEGRATED_OBSERVER',robot_states=288,admitted=181)),flush=True)
    except Exception as exc:
        dump(OUT/'failure.json',dict(**identity,status='INCOMPLETE',verified=len(checks),error=repr(exc)))
        raise


if __name__=='__main__':
    run()
