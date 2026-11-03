"""Current finite support must retain locality, noise, and hard constraints."""
from dataclasses import asdict
import importlib
import inspect
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.sensing.finite_support import ScanSupport, support_rows


def packet(x=0., noise=.03, time=3., half=.01):
    return ScanSupport(np.array([[x, -half], [x, half]]), np.array(['box','box']),
        np.array([.2,0.]), time, noise, np.array([[0,1]]), np.array([half]))


def params():
    return SafetyFilterParams(r_safe=.065, gamma_obj=8., rho=.02,
        max_speed=.35, object_velocity_bound=.2, dt=.05)


def module(name):
    assert importlib.util.find_spec(name) is not None, 'Missing current-scan support implementation'
    return importlib.import_module(name)


def test_noise_conventions_preserve_acquisition_and_transport_floors():
    assert 'noise_in_radius' in inspect.signature(support_rows).parameters
    p = params()
    before = asdict(p)
    acquisition = support_rows([.2,0.],[packet()],p,3.,.2,isolated_point_guards=True)
    transport = support_rows([.2,0.],[packet()],p,3.,.2,
        isolated_point_guards=True,noise_in_radius=False)
    np.testing.assert_allclose(acquisition.h,[.125])
    np.testing.assert_allclose(transport.h,[.095])
    np.testing.assert_allclose(transport.lower_distance,[.16])
    assert asdict(p) == before
    with pytest.raises(ValueError):
        support_rows([.2,0.],[packet(noise=.08)],p,3.,.2)
    wide = support_rows([.2,0.],[packet(noise=.08)],p,3.,.2,noise_in_radius=False)
    np.testing.assert_allclose(wide.h,[.045])


def test_measured_demand_does_not_shrink_feature_switch_cover():
    post = module('dbact.safety.post_support')
    rows, speed = post.build_rows([.2,0.],[packet(noise=0.,half=.005),
        packet(x=-.06,noise=0.,half=.005)],params(),3.,np.zeros(2))
    assert rows.ready and len(rows.A) == 2 and speed == 0.
    np.testing.assert_allclose(sorted(rows.h),[.13,.19])
    rows, speed = post.build_rows([.2,0.],[packet()],params(),3.,[.03,.04],
        point_velocities=np.array([[0.,.1],[0.,.8]]))
    assert speed == .2
    with pytest.raises(ValueError,match='finite'):
        post.build_rows([.2,0.],[packet()],params(),3.,[np.nan,0.])


def test_original_hard_rows_and_fallback_survive_runtime_override():
    assert 'post_support_packets' in inspect.signature(SafetyFilter.prepare_velocity).parameters
    p=params()
    safety=SafetyFilter(p)
    common=dict(position=np.array([.2,0.]),nominal_velocity=np.array([-.1,0.]),
        neighbor_positions=[np.array([.2,.4])],boundary_points=np.array([[0.,0.]]),
        boundary_normals=np.array([[1.,0.]]),object_velocity=np.array([.03,.04]))
    original=safety.prepare_velocity(**common)
    trace={}
    current=safety.prepare_velocity(**common,post_support_packets=[packet()],
        observation_time=3.,post_support_trace=trace)
    np.testing.assert_array_equal(current.A[:1],original.A[:1])
    np.testing.assert_array_equal(current.b_original[:1],original.b_original[:1])
    np.testing.assert_allclose(current.h_object,[.095])
    np.testing.assert_allclose(current.b_original[-1],-.69)
    assert trace['applied'] and trace['cover_motion_bound']==.2
    assert trace['demand_speed']==pytest.approx(.05) and trace['noise_in_radius'] is False
    sparse=ScanSupport(np.array([[0.,0.]]),np.array(['box']),np.array([.2,0.]),3.,.03,
        np.empty((0,2),dtype=int),np.empty(0))
    rejected={}
    fallback=safety.prepare_velocity(**common,post_support_packets=[sparse],
        observation_time=3.,post_support_trace=rejected)
    assert not rejected['applied'] and rejected['reason']=='isolated_gap_unknown'
    np.testing.assert_array_equal(fallback.A,original.A)
    np.testing.assert_array_equal(fallback.b_original,original.b_original)
    with pytest.raises(ValueError,match='simultaneous'):
        safety.prepare_velocity(**common,post_support_packets=[packet()],finite_support_packets=[packet()])


def test_admission_uses_frozen_current_local_motion_and_own_scan():
    post=module('dbact.control.post_support')
    c=SimpleNamespace(initial_reference_status='frozen',
        _map_reference_state={'self':{'status':'fresh_reference'}},
        sensor=SimpleNamespace(finite_support_packets={'self':packet(),'peer':packet(),
            'stale':packet(time=2.),'remote':packet()}))
    inputs,trace=post.filter_inputs(c,'self',['self','peer','stale'],3.)
    assert trace['source_agents']==['self','peer']
    assert len(inputs['post_support_packets'])==2
    c.initial_reference_status='acquiring'
    assert post.filter_inputs(c,'self',['self','peer'],3.)[0]=={}
    c.initial_reference_status='frozen'
    c._map_reference_state['self']['status']='unavailable'
    assert post.filter_inputs(c,'self',['self','peer'],3.)[0]=={}
    c._map_reference_state['self']['status']='fresh_reference'
    c.sensor.finite_support_packets.pop('self')
    assert post.filter_inputs(c,'self',['self','peer'],3.)[1]['reason']=='no_current_own_returns'


def test_option_is_explicit_default_off_and_controller_calls_local_gate(monkeypatch):
    from dbact.control.controller import DBACTParams
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    assert 'post_acquisition_finite_support' in DBACTParams.__dataclass_fields__
    assert DBACTParams().post_acquisition_finite_support is False
    cfg=load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller']['post_acquisition_finite_support']=True
    with pytest.raises(ValueError,match='Post-acquisition'):
        SimulationEnvironment(cfg,seed=1)
    cfg['controller'].update(initial_reference_acquisition=True,initial_task_reference=True,
        initial_reference_motion_model='translation',initial_reference_use_all_captured=True,
        initial_reference_map_motion=True,reference_map_bridge=True,acquisition_finite_support=True)
    post=module('dbact.control.post_support')
    original=post.filter_inputs
    calls=[]
    def observe(c,aid,permitted,time):
        calls.append(aid)
        return original(c,aid,permitted,time)
    monkeypatch.setattr(post,'filter_inputs',observe)
    env=SimulationEnvironment(cfg,seed=1)
    env.controller.trace_enabled=True
    env.controller.step(env.agents,env.cargoes,0.,env.dt)
    assert calls==[a.agent_id for a in env.agents]
    assert all(d.post_acquisition_support['reason']=='outside_frozen_reference'
        for d in env.controller.diagnostics)
