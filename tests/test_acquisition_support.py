"""Finite observational rows must preserve safety, scope and disabled behavior."""
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.sensing.finite_support import scan_support
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def packet(points,origin,stamp=0.):
    return scan_support([dict(point=p,ray_index=i,object_id='cargo_0') for i,p in enumerate(points)],origin,stamp,.01)


def configuration(enabled=True):
    cfg=load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True,initial_task_reference=True,
        initial_reference_motion_model='translation',initial_reference_use_all_captured=True,
        initial_reference_map_motion=True,reference_map_reacquisition=True,
        initial_acquisition_circulation=True,acquisition_finite_support=enabled)
    return cfg


def test_finite_endpoint_rows_resolve_a_spurious_plane_neighbor_conflict():
    p=np.array([.05,-.26])
    par=SafetyFilterParams(d_min=.28,r_safe=.195,gamma_obj=8.,rho=.02,max_speed=.35,
        aggregate_frame_consistent=True)
    inputs=dict(boundary_points=np.array([[0,0],[0,.04]]),boundary_normals=np.array([[1,0],[1,0]]))
    old=SafetyFilter(par).filter_velocity(p,[0,0],[p+[.28,0]],**inputs)
    trace={}
    new=SafetyFilter(par).filter_velocity(p,[0,0],[p+[.28,0]],**inputs,
        finite_support_packets=[packet([[0,0],[0,.04]],p)],observation_time=0.,
        finite_support_motion_bound=0.,finite_support_trace=trace)
    assert old.status=='scaled_barrier'
    assert new.status!='scaled_barrier' and new.barrier_scale==1.
    assert new.object_rows>0 and trace['applied']
    np.testing.assert_allclose(new.velocity,[0,0],atol=1e-12)


def test_isotropic_motion_bound_survives_rotated_radial_normals():
    par=SafetyFilterParams(r_safe=.195,gamma_obj=1.,rho=0.,max_speed=.5)
    result=SafetyFilter(par).filter_velocity([0,.3],[0,0],
        finite_support_packets=[packet([[-.02,0],[.02,0]],[0,.3])],
        finite_support_motion_bound=.2,observation_time=0.)
    np.testing.assert_allclose(result.velocity,[0,.115],atol=1e-7)
    assert result.object_rows==1 and result.barrier_scale==1.


@pytest.mark.parametrize('packets', [[],[packet([[0,0]],[0,.3])],[packet([[0,0],[.02,0]],[0,.3],-1.)]])
def test_unavailable_support_keeps_original_constraints_and_output(packets):
    par=SafetyFilterParams(r_safe=.195)
    inputs=dict(boundary_points=np.array([[-.02,0],[.02,0]]),boundary_normals=np.array([[0,1],[0,1]]))
    before=SafetyFilter(par).filter_velocity([0,.3],[0,-.2],**inputs)
    trace={}
    after=SafetyFilter(par).filter_velocity([0,.3],[0,-.2],**inputs,
        finite_support_packets=packets,finite_support_motion_bound=0.,finite_support_trace=trace)
    for field in ('A_rows','b_original','b_effective','h_object','velocity'):
        np.testing.assert_array_equal(getattr(before,field),getattr(after,field))
    assert before.status==after.status and not trace['applied']


def test_finite_rows_reject_missing_motion_instead_of_assuming_zero():
    with pytest.raises(ValueError):
        SafetyFilter(SafetyFilterParams()).filter_velocity([0,.3],[0,0],
            finite_support_packets=[packet([[0,0],[.02,0]],[0,.3])])


@pytest.mark.parametrize('state',['fresh_reference','unavailable','unknown'])
def test_only_stationary_acquisition_supplies_the_new_rows(state):
    from dbact.control.acquisition_support import filter_inputs
    controller=SimpleNamespace(initial_reference_status='acquiring',_map_reference_state={'a':{'status':state}},
        sensor=SimpleNamespace(finite_support_packets={'a':packet([[0,0],[.02,0]],[0,.3])}))
    inputs,trace=filter_inputs(controller,'a',['a'],0.)
    assert inputs=={} and not trace['applied']


def test_current_own_and_one_hop_packets_are_the_only_inputs():
    from dbact.control.acquisition_support import filter_inputs
    packets={aid:packet([[x,0],[x+.02,0]],[x,.3]) for aid,x in (('a',0),('peer',.02),('outsider',.04))}
    controller=SimpleNamespace(initial_reference_status='acquiring',_map_reference_state={'a':{'status':'stationary_acquisition'}},
        sensor=SimpleNamespace(finite_support_packets=packets))
    inputs,trace=filter_inputs(controller,'a',['a','peer'],0.)
    assert len(inputs['finite_support_packets'])==2
    assert inputs['finite_support_packets'][0] is packets['a']
    assert inputs['finite_support_packets'][1] is packets['peer']
    assert trace['source_agents']==['a','peer'] and inputs['finite_support_motion_bound']==0.
    stale,trace=filter_inputs(controller,'a',['a','peer'],1.)
    assert stale=={} and not trace['applied']


@pytest.mark.parametrize('change',[{'acquisition_finite_support':'false'},
    {'initial_reference_map_motion':False,'reference_map_reacquisition':False,'initial_acquisition_circulation':False}])
def test_incompatible_controller_selection_is_rejected(change):
    cfg=configuration()
    cfg['controller'].update(change)
    with pytest.raises(ValueError):SimulationEnvironment(cfg,seed=1)


def test_candidate_preserves_existing_numeric_configuration_and_uses_real_current_scans():
    base=SimulationEnvironment(configuration(False),seed=1)
    env=SimulationEnvironment(configuration(True),seed=1)
    values=asdict(env.controller.params)
    assert values.pop('acquisition_finite_support') is True
    expected=asdict(base.controller.params)
    assert expected.pop('acquisition_finite_support') is False
    assert values==expected and env.contact_params==base.contact_params
    env.agents=env.agents[:3]
    for agent,p in zip(env.agents,([4.3,3.0],[4.0,3.0],[.5,.5])):agent.position=np.array(p)
    env.controller.trace_enabled=True
    env.controller.step(env.agents,env.cargoes,0.,env.dt)
    diagnostics=env.controller.diagnostics
    allowed={env.agents[0].agent_id,env.agents[1].agent_id}
    assert diagnostics[0].acquisition_support['applied']
    assert set(diagnostics[0].acquisition_support['source_agents'])==allowed
    assert diagnostics[0].object_rows>0


def test_omitted_and_disabled_modes_have_identical_short_physical_trajectories():
    cfg=configuration(False)
    missing=configuration(False)
    missing['controller'].pop('acquisition_finite_support')
    first,second=SimulationEnvironment(cfg,seed=1),SimulationEnvironment(missing,seed=1)
    assert first.controller.params.acquisition_finite_support is False
    for _ in range(6):
        first.step();second.step()
        np.testing.assert_array_equal([a.position for a in first.agents],[a.position for a in second.agents])
        np.testing.assert_array_equal(first.cargoes[0].center,second.cargoes[0].center)


def test_freeze_frame_returns_to_original_rows_even_when_map_model_is_stationary():
    from dbact.control.acquisition_support import filter_inputs
    controller=SimpleNamespace(initial_reference_status='frozen',
        _map_reference_state={'a':{'status':'stationary_acquisition'}},
        sensor=SimpleNamespace(finite_support_packets={'a':packet([[0,0],[.02,0]],[0,.3])}))
    inputs,trace=filter_inputs(controller,'a',['a'],0.)
    assert inputs=={} and not trace['applied']


def test_reach_cap_is_reported_without_claiming_uncapped_barrier_satisfaction():
    trace={}
    result=SafetyFilter(SafetyFilterParams(r_safe=.195,gamma_obj=1.,rho=0.,max_speed=.2)).filter_velocity(
        [0,.05],[0,0],finite_support_packets=[packet([[-.02,0],[.02,0]],[0,.3])],
        finite_support_motion_bound=0.,finite_support_trace=trace)
    np.testing.assert_allclose(result.velocity,[0,.12],atol=1e-7)
    assert trace['cap_active']==1
