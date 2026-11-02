"""Synthetic geometry/permission checks, not transport success experiments."""
import numpy as np
import pytest
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact.types import BoundaryView


def view(points,normals):
    return BoundaryView(points=np.asarray(points),normals=np.asarray(normals),
        confidence=np.ones(len(points)),arc_length=np.ones(len(points))*.03,
        object_ids=np.full(len(points),'cargo_0',dtype='<U32'))


def rectangle():
    u=np.linspace(-.47,.47,30)
    return view(np.vstack([np.c_[u, np.full(30,-.5)],np.c_[np.full(30,.5),u],
        np.c_[u,np.full(30,.5)],np.c_[np.full(30,-.5),u]]),
        np.repeat([[0.,-1.],[1.,0.],[0.,1.],[-1.,0.]],30,axis=0))


def moved(scan,delta,angle=0.):
    r=rotation(angle)
    return view(scan.points@r.T+delta,scan.normals@r.T)


def test_persistent_reference_tracks_metric_translation_without_gain_drift():
    scan=rectangle();tracker=ObservedReferenceTracker()
    tracker.initialize(scan,[.04,-.01],0.)
    frozen=tracker.reference.copy()
    for k in range(1,101):
        delta=np.array([k*.005,k*.002])
        result=tracker.update(moved(scan,delta),k*.05)
        assert result is not None
        np.testing.assert_allclose(result.displacement,[.04,-.01]+delta,atol=1e-7)
    np.testing.assert_array_equal(tracker.reference,frozen)
    np.testing.assert_allclose(result.velocity,[.1,.04],atol=1e-7)


def test_rotation_is_estimated_about_observed_anchor_not_simulator_center():
    scan=rectangle();scan=view(scan.points+[3.,4.],scan.normals)
    tracker=ObservedReferenceTracker();tracker.initialize(scan,[0.,0.],0.)
    for k in range(1,21):
        theta=k*.002;delta=np.array([k*.004,0.])
        current=view((scan.points-[3.,4.])@rotation(theta).T+[3.,4.]+delta,
                     scan.normals@rotation(theta).T)
        result=tracker.update(current,k*.05)
        assert result is not None
        np.testing.assert_allclose(result.displacement,delta,atol=2e-5)
        assert result.angle==pytest.approx(theta,abs=2e-5)


def test_symmetric_circle_still_observes_translation_without_yaw_claim():
    angles=np.linspace(0,2*np.pi,128,endpoint=False)
    n=np.c_[np.cos(angles),np.sin(angles)];scan=view(.5*n,n)
    tracker=ObservedReferenceTracker();tracker.initialize(scan,[0.,0.],0.)
    result=tracker.update(moved(scan,[.01,-.005]),.05)
    assert result is not None
    np.testing.assert_allclose(result.displacement,[.01,-.005],atol=1e-6)


def test_flat_face_cannot_initialize_two_dimensional_stop_feedback():
    scan=view(np.c_[np.linspace(-.5,.5,30),np.zeros(30)],np.tile([0.,1.],(30,1)))
    tracker=ObservedReferenceTracker()
    assert tracker.initialize(scan,[0.,0.],0.) is None
    assert not tracker.initialized


def test_missing_scan_abstains_and_does_not_create_fresh_zero_velocity_vote():
    scan=rectangle();tracker=ObservedReferenceTracker();tracker.initialize(scan,[0.,0.],0.)
    assert tracker.last_reading is None
    tracker.update(moved(scan,[.01,0.]),.05)
    assert tracker.update(BoundaryView.empty(),.10) is None
    assert tracker.last_reading is None
    result=tracker.update(moved(scan,[.03,0.]),.15)
    assert result is not None
    np.testing.assert_allclose(result.displacement,[.03,0.],atol=1e-7)


def test_rejected_motion_does_not_commit_or_replace_reference():
    scan=rectangle();tracker=ObservedReferenceTracker();tracker.initialize(scan,[0.,0.],0.)
    assert tracker.update(moved(scan,[.1,0.]),.05) is None
    assert tracker.reason=='motion_bound_exceeded'
    np.testing.assert_array_equal(tracker.translation,np.zeros(2))
    assert tracker.last_time==0.
    with pytest.raises(RuntimeError,match='silently replaced'):
        tracker.initialize(scan,[1.,1.],.1)


def test_same_time_and_nonfinite_parameters_rejected():
    with pytest.raises(ValueError):ObservedReferenceTracker(gate=float('nan'))
    tracker=ObservedReferenceTracker();tracker.initialize(rectangle(),[0.,0.],0.)
    with pytest.raises(ValueError):tracker.update(rectangle(),0.)


def reference_environment():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(boundary_navigation='vector_goal',navigation_estimator='observed_reference')
    return SimulationEnvironment(cfg,seed=2)


def test_reference_requires_vector_mode_and_preserves_physics_and_stop_gates():
    from dbact.control.controller import DBACTParams
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    baseline=SimulationEnvironment(cfg,seed=2)
    candidate=reference_environment()
    assert candidate.contact_params==baseline.contact_params
    assert candidate.controller.phase_monitor.gates==baseline.controller.phase_monitor.gates
    assert candidate.controller.params.contact_contract()==baseline.controller.params.contact_contract()
    assert DBACTParams().navigation_estimator=='accumulated'
    cfg['controller']['navigation_estimator']='observed_reference'
    with pytest.raises(ValueError,match='requires the experimental vector_goal'):
        SimulationEnvironment(cfg,seed=2)


def test_missing_reference_does_not_fall_back_to_biased_accumulated_votes():
    from dbact.control.phase import Phase
    env=reference_environment();c=env.controller;c._ensure_state(env.agents)
    c.phase_monitor.phase=Phase.CONTACT_READY
    c._reference_readings={}
    c._update_navigation(env.agents,[[] for _ in env.agents])
    assert c.team_navigation is None
    assert all(v is None for v in c._navigation.values())


def test_reference_consumes_observations_without_mutating_safety_map():
    from types import SimpleNamespace
    from dbact.control.phase import Phase
    env=reference_environment();c=env.controller;c._ensure_state(env.agents)
    oid=next(iter(c.tasks));aid=env.agents[0].agent_id;m=c.maps[aid]
    m.motion_quality[oid]='credible';m.displacement[oid]=np.array([.03,0.])
    m.velocity[oid]=np.array([.1,.2]);before=m.displacement[oid].copy()
    c.phase_monitor.phase=Phase.CONTACT_READY
    # Agent identity only: no simulator object pose or even agent position needed.
    agents=[SimpleNamespace(agent_id=aid)]
    scan=rectangle();c._update_reference(agents,{aid:scan},0.)
    assert not c._reference_readings
    c._update_reference(agents,{aid:moved(scan,[.01,0.])},.05)
    result=c._reference_readings[aid]
    np.testing.assert_allclose(result.displacement,1.24*before+[.01,0.],atol=1e-7)
    np.testing.assert_array_equal(m.displacement[oid],before)
    np.testing.assert_array_equal(m.velocity[oid],[.1,.2])


def test_reference_messages_remain_one_hop():
    from dbact.control.phase import Phase
    from dbact.control.observed_reference import ReferenceReading
    env=reference_environment();c=env.controller;c.phase_monitor.phase=Phase.CONTACT_READY
    a,b=env.agents[:2]
    c._reference_readings={b.agent_id:ReferenceReading(np.array([9.,9.]),np.zeros(2),10,0.,1.,0.)}
    c._update_navigation(env.agents,[[] for _ in env.agents])
    assert c._navigation[a.agent_id] is None
    assert c._navigation[b.agent_id].voters==(b.agent_id,)
    assert c.team_navigation.voters==(b.agent_id,) # explicitly shared supervisor only
