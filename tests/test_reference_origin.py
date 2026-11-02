"""Reference epoch/gain ablations cannot change information or physical contracts."""
import copy
import numpy as np
import pytest
from dbact.control.controller import DBACTParams
from dbact.control.observed_reference import ObservedReferenceTracker
from dbact.control.phase import Phase
from dbact.types import BoundaryView
from test_observed_reference import rectangle, moved


def environment(**params):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(boundary_navigation='vector_goal',navigation_estimator='observed_reference',**params)
    return SimulationEnvironment(cfg,seed=2)


def inputs(c,agents,oid):
    aid=agents[0].agent_id
    c.maps[aid].motion_quality[oid]='credible'
    c.maps[aid].displacement[oid]=np.array([.1,.02])
    batches={a.agent_id:BoundaryView.empty() for a in agents}
    batches[aid]=rectangle()
    return aid,batches


def test_early_reference_uses_existing_bias_and_never_claims_initial_velocity():
    env=environment(reference_early_anchor=True);c=env.controller;c._ensure_state(env.agents)
    oid=next(iter(c.tasks));aid,batches=inputs(c,env.agents,oid)
    before=c.maps[aid].object_displacement(oid).copy()
    c.phase_monitor.phase=Phase.DISCOVER
    c._update_reference(env.agents,batches,1.)
    tracker=c._reference_trackers[aid]
    assert tracker.initialized and tracker.initial_time==1.
    np.testing.assert_array_equal(tracker.offset,1.24*before)
    assert aid not in c._reference_readings
    np.testing.assert_array_equal(c.maps[aid].object_displacement(oid),before)
    anchor=tracker.anchor.copy()
    batches[aid]=moved(rectangle(),[.01,0.])
    c._update_reference(env.agents,batches,1.05)
    assert aid in c._reference_readings
    c.phase_monitor.phase=Phase.CONTACT_READY
    c._update_reference(env.agents,batches,1.10)
    assert c._reference_trackers[aid] is tracker and tracker.initial_time==1.
    np.testing.assert_array_equal(tracker.anchor,anchor)


def test_default_still_waits_for_contact_ready_and_unit_gain_changes_only_offset():
    env=environment(reference_bootstrap_gain=1.);c=env.controller;c._ensure_state(env.agents)
    oid=next(iter(c.tasks));aid,batches=inputs(c,env.agents,oid)
    c._update_reference(env.agents,batches,0.)
    assert not c._reference_trackers
    c.phase_monitor.phase=Phase.CONTACT_READY
    c._update_reference(env.agents,batches,1.)
    np.testing.assert_array_equal(c._reference_trackers[aid].offset,[.1,.02])
    assert c.params.progress_estimate_gain==1.24
    assert c.phase_monitor.gates==environment().controller.phase_monitor.gates
    assert env.contact_params==environment().contact_params


def test_bootstrap_scale_does_not_change_registration_fit_admission_or_velocity():
    a,b=ObservedReferenceTracker(),ObservedReferenceTracker()
    a.initialize(rectangle(),[.124,.0248],0.);b.initialize(rectangle(),[.1,.02],0.)
    for k in range(1,20):
        scan=moved(rectangle(),[k*.004,k*.001],k*.001)
        x,y=a.update(scan,k*.05),b.update(scan,k*.05)
        assert x is not None and y is not None
        np.testing.assert_array_equal(a.translation,b.translation)
        np.testing.assert_array_equal(x.velocity,y.velocity)
        assert x.matches==y.matches and x.angle==y.angle and x.residual_rms==y.residual_rms
        np.testing.assert_allclose(x.displacement-y.displacement,[.024,.0048],atol=1e-15)


def test_initialization_record_is_immutable_snapshot_not_last_update():
    t=ObservedReferenceTracker()
    assert t.initialization_record()=={'initialized':False}
    t.initialize(rectangle(),[.02,0.],1.)
    record=t.initialization_record()
    t.update(moved(rectangle(),[.01,0.]),1.05)
    assert t.initialization_record()==record and record['time']==1.
    record['anchor'][0]=99.
    assert t.anchor[0]!=99.
    assert not any(k in record for k in ('center_of_mass','true_position','true_displacement'))


@pytest.mark.parametrize('gain',[0.,-1.,float('nan'),float('inf'),True])
def test_invalid_gain_rejected(gain):
    with pytest.raises(ValueError,match='finite and positive'):environment(reference_bootstrap_gain=gain)


def test_options_are_disabled_by_default_and_require_reference_mode():
    assert DBACTParams().reference_early_anchor is False
    assert DBACTParams().reference_bootstrap_gain is None
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller']['reference_early_anchor']=True
    with pytest.raises(ValueError,match='require observed_reference'):SimulationEnvironment(cfg,seed=2)


def test_early_observer_has_no_effect_on_pre_contact_physics_or_controls():
    a,b=environment(),environment(reference_early_anchor=True)
    for _ in range(25):
        a.step();b.step()
        assert a.controller.phase==b.controller.phase
        assert a.controller.phase<Phase.CONTACT_READY
        np.testing.assert_array_equal([x.position for x in a.agents],[x.position for x in b.agents])
        np.testing.assert_array_equal([x.velocity for x in a.agents],[x.velocity for x in b.agents])
        np.testing.assert_array_equal(a.cargoes[0].position,b.cargoes[0].position)
