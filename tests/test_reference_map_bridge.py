"""A changed set of biased absolute voters must not teleport map history."""
import importlib

import numpy as np
import pytest


def bridge_type():
    try:
        m=importlib.import_module('dbact.control.map_reference_bridge')
    except ModuleNotFoundError:
        pytest.fail('Missing same-voter map bridge')
    return m.ReferenceMapBridge


def call(bridge,readings,permitted=None,stationary=False):
    d={k:np.asarray(v,dtype=float) for k,v in readings.items()}
    v={k:np.array([.2,0.]) for k in d}
    return bridge.update(d,v,list(d) if permitted is None else permitted,stationary=stationary)


def test_observed_989_roster_change_uses_same_peer_increment():
    bridge=bridge_type()()
    first=call(bridge,{'09':[1.0994852377941047,-.06014094111871942]})
    result=call(bridge,{'06':[.18381631869282308,-.06073796986455334],
                        '09':[1.1119627463861494,-.054837227396275647]})
    np.testing.assert_allclose(result['displacement'],[1.1119627463861494,-.054837227396275647])
    np.testing.assert_allclose(result['delta'],[.01247750859204477,.005303713722443773])
    assert result['common_voters']==['09'] and not result['reset_history']
    assert first['status']=='anchored'


def test_offsets_rotation_and_permitted_membership():
    theta=.71
    R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    outputs=[]
    for rotation in (np.eye(2),R):
        bridge=bridge_type()()
        initial={'a':np.array([.2,-.1]),'b':np.array([1.,2.])}
        call(bridge,{k:rotation@v for k,v in initial.items()},['a','a'])
        result=call(bridge,{k:rotation@(v+[.02,.01]) for k,v in initial.items()},['a','a','b'])
        assert result['common_voters']==['a']
        outputs.append(result)
        result=call(bridge,{k:rotation@(v+[.04,.02]) for k,v in initial.items()},['a','b'])
        np.testing.assert_allclose(result['displacement'],rotation@np.array([.24,-.08]),atol=1e-12)
    np.testing.assert_allclose(outputs[1]['delta'],R@outputs[0]['delta'],atol=1e-12)


def test_disjoint_and_missing_data_reset_history_but_stationary_does_not():
    bridge=bridge_type()()
    result=call(bridge,{},stationary=True)
    assert result['status']=='stationary' and not result['reset_history']
    call(bridge,{'a':[.1,0.]})
    result=call(bridge,{'b':[5.,0.]})
    assert result['status']=='reanchored' and result['reset_history']
    assert call(bridge,{}) is None
    result=call(bridge,{'b':[5.1,0.]})
    assert result['reset_history']
    result=call(bridge,{'b':[5.2,0.]})
    assert not result['reset_history']
    np.testing.assert_allclose(result['delta'],[.1,0.])


def test_invalid_contract_and_default_off():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/research/best_known_transport_v1.yaml')
    env=SimulationEnvironment(cfg,seed=1)
    assert getattr(env.controller.params,'reference_map_bridge',None) is False
    cfg['controller']['reference_map_bridge']=True
    with pytest.raises(ValueError,match='Map bridge'):
        SimulationEnvironment(cfg,seed=1)


def test_history_reset_is_object_specific_and_new_scan_is_unshifted():
    from dbact.sensing.boundary_map import LocalBoundaryMap
    from dbact.types import BoundaryView
    local=LocalBoundaryMap()
    local.update(BoundaryView(np.array([[0.,0.],[2.,0.]]),np.array([[0.,1.],[0.,1.]]),
        np.ones(2),np.ones(2),np.array(['cargo_0','other'])),0.)
    assert hasattr(local,'discard_object_history'), 'Missing object-specific reset'
    assert local.discard_object_history('cargo_0')==1
    local.align_translation('cargo_0',np.array([10.,0.]),np.zeros(2))
    np.testing.assert_array_equal(local._points,[[2.,0.]])
    np.testing.assert_array_equal(local._objects,['other'])


def test_controller_bridges_current_neighbors_then_discards_unbridged_history(monkeypatch):
    from types import SimpleNamespace
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    from dbact.control.reference_map import update_reference_maps
    from dbact.types import BoundaryView
    cfg=load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True,initial_task_reference=True,
        initial_reference_motion_model='translation',initial_reference_use_all_captured=True,
        initial_reference_map_motion=True,reference_map_bridge=True)
    # Use real initialized simulator agents and map lifecycle.
    env=SimulationEnvironment(cfg,seed=1)
    c=env.controller
    agents=env.agents[:3]
    c._ensure_state(agents)
    c.initial_reference_status='frozen'
    aid,peer,newcomer=[a.agent_id for a in agents]
    readings={}
    monkeypatch.setattr(c,'_update_reference',lambda *args:setattr(c,'_reference_readings',readings))
    def observation(point):
        return BoundaryView(np.array([point]),np.array([[0.,1.]]),np.ones(1),
            np.ones(1),np.array(['cargo_0']))
    readings[peer]=SimpleNamespace(displacement=np.array([1.1,0.]),velocity=np.zeros(2))
    update_reference_maps(c,agents,[[1,2],[0],[0]],
        [observation([3.2,4.]),BoundaryView.empty(),BoundaryView.empty()],1.,.05)
    readings[peer]=SimpleNamespace(displacement=np.array([1.112,0.]),velocity=np.zeros(2))
    readings[newcomer]=SimpleNamespace(displacement=np.array([.18,0.]),velocity=np.zeros(2))
    update_reference_maps(c,agents,[[1,2],[0],[0]],[BoundaryView.empty()]*3,1.05,.05)
    np.testing.assert_allclose(c.maps[aid]._points,[[3.212,4.]],atol=1e-12)
    readings.clear()
    update_reference_maps(c,agents,[[1,2],[0],[0]],[BoundaryView.empty()]*3,1.1,.05)
    readings[newcomer]=SimpleNamespace(displacement=np.array([.2,0.]),velocity=np.zeros(2))
    update_reference_maps(c,agents,[[2],[0],[0]],
        [observation([3.6,4.]),BoundaryView.empty(),BoundaryView.empty()],1.15,.05)
    np.testing.assert_allclose(c.maps[aid]._points,[[3.6,4.]],atol=1e-12)
    assert c._map_reference_state[aid]['bridge']['discarded_cells']==1
