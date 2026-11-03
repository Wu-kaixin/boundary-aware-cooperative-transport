"""Replay candidate commands and audit current local support and executed rows."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import json

import numpy as np

from research_io import read
from replay_qp_conflict import recorded
from probe_map_bridge_finite_support import observed_system, systems
from dbact_sim.environment import SimulationEnvironment


def audit(folder, case, *, limit_frames=None):
    rows=read(folder/'paired_metrics.json')['steps']
    total_frames=len(rows)
    if limit_frames is not None:
        assert type(limit_frames) is int and 0 < limit_frames <= total_frames
        rows=rows[:limit_frames]
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        saved={key:z[key].copy() for key in ('times','agent_positions',
            'cargo/cargo_0/centers','cargo/cargo_0/angles','sensed_counts','sensed_points')}
    offsets=np.r_[0,np.cumsum(saved['sensed_counts'])]
    env=SimulationEnvironment(case['config'],seed=case['seed'])
    c,safety=env.controller,env.controller.safety
    c.trace_enabled=True
    ids=[a.agent_id for a in env.agents]
    original=safety.prepare_velocity
    counts=Counter()
    frame,call_index=0,0
    captured={}
    residuals=[]

    def prepare(position, nominal_velocity, neighbor_positions=(), **kwargs):
        nonlocal call_index
        i,call_index=call_index,call_index+1
        aid=ids[i]
        problem=original(position,nominal_velocity,neighbor_positions,**kwargs)
        trace=c._trace['post_acquisition_support']
        counts[trace['reason']]+=1
        neighbor_ids=[]
        for point in neighbor_positions:
            matches=[a.agent_id for a in env.agents if np.array_equal(a.position,point)]
            assert len(matches)==1 and matches[0]!=aid
            neighbor_ids.append(matches[0])
        packets=c.sensor.finite_support_packets
        has_motion=c.initial_reference_status=='frozen' and c._map_reference_state[aid]['status']=='fresh_reference'
        own=packets.get(aid)
        fresh_own=own is not None and len(own.points)>0 and abs(own.timestamp-float(saved['times'][frame-1]))<=1e-9
        if not has_motion or not fresh_own:
            assert 'post_support_packets' not in kwargs and not trace['applied']
        else:
            assert 'post_support_packets' in kwargs
            allowed=[aid,*neighbor_ids]
            expected_ids=[key for key in dict.fromkeys(allowed) if key in packets
                and abs(packets[key].timestamp-kwargs['observation_time'])<=1e-9]
            assert trace['source_agents']==expected_ids
            assert len(expected_ids)==len(kwargs['post_support_packets'])
            for key,packet in zip(expected_ids,kwargs['post_support_packets']):
                assert recorded(asdict(packet))==recorded(asdict(packets[key]))
            observer=observed_system(dict(agent_id=aid,position=position,neighbors=neighbor_ids,
                A=problem.A,b=problem.b_original,kinds=problem.row_kinds,params=asdict(problem.parameters),
                object_velocity=kwargs['object_velocity'],point_velocities=kwargs.get('boundary_point_velocities'),
                permitted_packets={key:asdict(packets[key]) for key in allowed if key in packets}),
                kwargs['observation_time'],'received_speed')
            assert trace['applied']==observer['ready'] and trace['reason']==observer['reason']
            if observer['ready']:
                np.testing.assert_array_equal(problem.A,observer['A'])
                np.testing.assert_array_equal(problem.b_original,observer['b'])
                np.testing.assert_array_equal(problem.h_object,observer['h'])
                assert trace['demand_speed']==observer['normal_motion'][0]
                assert trace['cover_motion_bound']==observer['cover_motion_bound']
                assert trace['cap_active']==observer['cap_active'] and trace['noise_in_radius'] is False
                mask=np.asarray(problem.row_kinds)=='object'
                captured[aid]=dict(A=problem.A.copy(),b=problem.b_original.copy(),
                    uncapped=systems(observer)['uncapped_full'],object_mask=mask,
                    speed=problem.parameters.max_speed,trace=dict(trace))
        # Independently reconstruct every hard neighbor row from current positions.
        q=np.asarray(neighbor_positions).reshape(-1,2)
        delta=np.asarray(position)-q
        np.testing.assert_array_equal(problem.A[:len(q)],2.*delta)
        expected=-.5*problem.parameters.gamma_agent*(np.sum(delta*delta,axis=1)-problem.parameters.d_min**2)
        np.testing.assert_array_equal(problem.b_original[:len(q)],expected)
        return problem

    safety.prepare_velocity=prepare
    for k,row in enumerate(rows):
        frame,call_index=k+1,0
        captured.clear()
        env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k],saved['cargo/cargo_0/angles'][k])
        for i,agent in enumerate(env.agents):
            agent.position=saved['agent_positions'][k,i].copy()
            agent.velocity=np.zeros(2) if k==0 else np.asarray(rows[k-1]['commands'][i])
        commands=c.step(env.agents,env.cargoes,float(saved['times'][k]),env.dt)
        assert call_index==len(ids)
        np.testing.assert_array_equal([v.velocity for v in commands],row['commands'])
        assert recorded([vars(d) for d in c.diagnostics])==row['diagnostics'],('diagnostics',frame)
        assert recorded(safety.stats.as_dict())==row['solver_statistics']
        assert c.phase.name==row['phase'] and recorded(vars(c.phase_signals))==row['phase_guards']
        assert recorded(c.phase_monitor.as_dict())==row['phase_response']
        np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
            saved['sensed_points'][offsets[k+1]:offsets[k+2]])
        for aid,item in captured.items():
            u=np.asarray(row['commands'][ids.index(aid)])
            mask=item['object_mask']
            hard=item['A'][~mask]@u-item['b'][~mask]
            uncap=item['A'][mask]@u-item['uncapped'][mask]
            cap=item['A'][mask]@u-item['b'][mask]
            minimum_hard=float(hard.min()) if len(hard) else 0.
            assert np.linalg.norm(u)<=item['speed']+1e-7
            residuals.append(dict(frame=frame,agent_id=aid,minimum_hard=minimum_hard,
                minimum_capped_object=float(cap.min()),minimum_uncapped_object=float(uncap.min())))
        if frame%200==0:
            print(json.dumps(dict(audit_case=case['case_id'],verified_frames=frame)),flush=True)
    assert sum(counts.values())==len(rows)*len(ids)
    return dict(verified=True,frames=len(rows),total_physical_frames=total_frames,
        full_episode_verified=len(rows)==total_frames,agent_steps=sum(counts.values()),applied=len(residuals),
        guard_counts=dict(counts),hard_violations=sum(r['minimum_hard'] < -1e-7 for r in residuals),
        capped_object_violations=sum(r['minimum_capped_object'] < -1e-7 for r in residuals),
        uncapped_object_violations=sum(r['minimum_uncapped_object'] < -1e-7 for r in residuals),
        minimum_uncapped_object_residual=min((r['minimum_uncapped_object'] for r in residuals),default=None),
        violations=[r for r in residuals if min(r['minimum_hard'],r['minimum_uncapped_object']) < -1e-7],
        scope='Exact controller replay of the stated frame range; every current packet/row and admitted executed command checked. Geometry/motion assumptions remain empirical.')
