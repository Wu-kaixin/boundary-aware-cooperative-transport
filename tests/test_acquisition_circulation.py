"""Active acquisition must move along observed boundary without changing later control."""
from dataclasses import asdict, replace

import numpy as np
import pytest

from dbact.types import BoundaryView
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def config(enabled=True):
    cfg=load_yaml('configs/research/best_known_transport_v1.yaml')
    cfg['controller'].update(initial_reference_acquisition=True,initial_task_reference=True,
        initial_reference_motion_model='translation',initial_reference_use_all_captured=True,
        initial_reference_map_motion=True,initial_acquisition_circulation=enabled)
    return cfg


def view(point,normal):
    return BoundaryView(np.array([point],dtype=float),np.array([normal],dtype=float),
        np.ones(1),np.ones(1),np.full(1,'cargo_0'))


def test_circulation_progresses_tangentially_and_corrects_outward_at_existing_speed():
    from dbact.control.acquisition_navigation import circulation_velocity
    boundary=view([0.,0.],[0.,1.])
    np.testing.assert_allclose(circulation_velocity([0.,.26],boundary,.26,.9,.35),[-.35,0.],atol=1e-12)
    close=circulation_velocity([0.,.13],boundary,.26,.9,.35)
    assert close[0]<0 and close[1]>0
    assert np.linalg.norm(close)==pytest.approx(.35)
    far=circulation_velocity([0.,.8],boundary,.26,.9,.35)
    assert far[0]<0 and far[1]<0 and np.linalg.norm(far)<=.35+1e-12


def test_circulation_is_rotation_and_translation_covariant():
    from dbact.control.acquisition_navigation import circulation_velocity
    angle=.73;rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    shift=np.array([8.,-3.])
    boundary=view([1.,2.],[0.,1.]);position=np.array([1.05,2.18])
    original=circulation_velocity(position,boundary,.26,.9,.35)
    moved=view(boundary.points[0]@rotation.T+shift,boundary.normals[0]@rotation.T)
    np.testing.assert_allclose(circulation_velocity(position@rotation.T+shift,moved,.26,.9,.35),
        original@rotation.T,atol=1e-12)


def test_selector_preserves_all_existing_parameters_and_defaults_off():
    old=SimulationEnvironment(config(False),seed=1);new=SimulationEnvironment(config(),seed=1)
    values=asdict(new.controller.params);values['initial_acquisition_circulation']=False
    assert values==asdict(old.controller.params)
    assert old.contact_params==new.contact_params
    assert not SimulationEnvironment(load_yaml('configs/research/best_known_transport_v1.yaml'),seed=1).controller.params.initial_acquisition_circulation


def test_acquisition_uses_larger_existing_offset_and_retains_empty_view_search():
    env=SimulationEnvironment(config(),seed=1);c=env.controller;c._ensure_state(env.agents)
    c.density_params=replace(c.density_params,cage_offset=.6,lead_offset=.7)
    c.initial_reference_status='acquiring'
    p=env.agents[0].position
    boundary=view(p-[0.,.7],[0.,1.])
    u,mode,_,pushing,effort=c._nominal_command(0,env.agents,[],boundary,[False]*len(env.agents),env.dt)
    assert mode=='acquisition_circulation' and not pushing and effort==0
    np.testing.assert_allclose(u,[-c.params.max_speed,0.],atol=1e-12)
    _,mode,*_=c._nominal_command(0,env.agents,[],BoundaryView.empty(),[False]*len(env.agents),env.dt)
    assert mode=='search'


@pytest.mark.parametrize('status',['frozen','unavailable'])
def test_circulation_releases_the_original_nominal_path_after_acquisition(status):
    results=[]
    for enabled in (False,True):
        env=SimulationEnvironment(config(enabled),seed=1);c=env.controller;c._ensure_state(env.agents)
        c.initial_reference_status=status
        p=env.agents[0].position
        results.append(c._nominal_command(0,env.agents,[],view(p-[0.,.3],[0.,1.]),[False]*len(env.agents),env.dt))
    np.testing.assert_array_equal(results[0][0],results[1][0])
    assert results[0][1:]==results[1][1:]


@pytest.mark.parametrize('updates',[
    {'initial_acquisition_circulation':'false'},
    {'initial_reference_map_motion':False},
])
def test_invalid_acquisition_circulation_selector_rejected(updates):
    cfg=config();cfg['controller'].update(updates)
    with pytest.raises(ValueError,match='Acquisition circulation'):
        SimulationEnvironment(cfg,seed=1)
