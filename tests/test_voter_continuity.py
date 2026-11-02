"""Causal information limits, roster invariance and irreversible-stop admission."""
import copy

import numpy as np
import pytest

from dbact.control.boundary_navigation import VoterContinuity, point_goal_feedback
from dbact.control.controller import DBACTParams
from dbact.control.phase import Phase, PhaseMonitor, PhaseSignals


def step(bridge, values, permitted=None):
    ds = {i: np.asarray(x, dtype=float) for i, x in values.items()}
    vs = {i: np.zeros(2) for i in values}
    raw = point_goal_feedback(ds, vs, list(ds) if permitted is None else permitted, [1.2, 0.], .02)
    return raw, bridge.update(raw, ds, [1.2, 0.], .02)


def test_dropping_biased_voter_cannot_make_stationary_output_arrive():
    bridge = VoterContinuity()
    _, first = step(bridge, {'a':[1.19,0.], 'b':[1.29,0.]})
    raw, second = step(bridge, {'a':[1.19,0.]})
    assert raw.remaining == 0.
    np.testing.assert_array_equal(second.displacement, first.displacement)
    assert second.remaining > 0.
    np.testing.assert_allclose(bridge.trace['roster_delta'], [-.05,0.], atol=1e-15)
    np.testing.assert_array_equal(bridge.trace['common_delta'], [0.,0.])


def test_constant_individual_bias_and_true_translation_have_distinct_effects():
    bridge = VoterContinuity()
    _, first = step(bridge, {'a':[.1,.2], 'b':[.5,.2], 'c':[.9,.2]})
    _, changed = step(bridge, {'b':[.53,.24], 'c':[.93,.24]})
    np.testing.assert_allclose(changed.displacement-first.displacement, [.03,.04], atol=1e-15)
    # The starting 0.5m aggregate bias is retained, never certified away.
    np.testing.assert_allclose(changed.displacement, [.53,.24])


def test_fixed_roster_is_exact_and_remote_readings_are_irrelevant():
    bridge = VoterContinuity()
    for k in range(7):
        raw, value = step(bridge, {'own':[.0123*k,-.0007*k], 'remote':[900+k,-999]}, ['own'])
        for name in ('displacement','velocity','error','direction'):
            np.testing.assert_array_equal(getattr(raw,name),getattr(value,name))
        assert value.voters == ('own',)
        assert 'remote' not in bridge.trace['common_voters']


def test_missing_or_disjoint_messages_do_not_become_stale_votes():
    bridge = VoterContinuity()
    step(bridge, {'old':[.4,0.]})
    _, lost = step(bridge, {'new':[1.2,0.]})
    assert lost is None and bridge.trace['status']=='reanchored'
    _, resumed = step(bridge, {'new':[1.2,0.]})
    assert resumed is not None and resumed.voters==('new',)
    assert step(bridge,{})==(None,None)
    assert step(bridge,{'new':[1.2,0.]})[1] is None
    assert step(bridge,{'new':[1.2,0.]})[1] is not None


def test_input_arrays_are_not_kept_as_mutable_history():
    bridge=VoterContinuity(); a=np.array([.3,0.])
    step(bridge,{'a':a}); a[0]=.4
    _,value=step(bridge,{'a':a})
    np.testing.assert_array_equal(value.displacement,a)
    np.testing.assert_allclose(bridge.trace['common_delta'],[.1,0.],atol=1e-15)


def test_stop_confirmation_rejects_transient_and_missing_readout_but_keeps_braking():
    monitor=PhaseMonitor(gates=DBACTParams(boundary_navigation='vector_goal', navigation_hold_dwell=20).phase_gates(),phase=Phase.BRAKE)
    arrived=PhaseSignals(progress=1.2,target_distance=1.2,estimated_speed=0.)
    for frame in range(19): monitor.update(arrived,frame)
    assert monitor.phase==Phase.BRAKE and monitor.settle_streak==19
    monitor.update(PhaseSignals(progress=1.2,target_distance=1.2,estimate_quality='unavailable'),19)
    assert monitor.settle_streak==0
    for frame in range(20,39): monitor.update(arrived,frame)
    assert monitor.phase==Phase.BRAKE
    monitor.update(arrived,39)
    assert monitor.phase==Phase.HOLD


@pytest.mark.parametrize('dwell',[0,-1,1.5,True])
def test_invalid_confirmation_configuration_is_rejected(dwell):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(navigation_hold_dwell=dwell)
    with pytest.raises(ValueError,match='positive integer'): SimulationEnvironment(cfg,seed=2)


def test_optional_path_preserves_local_permissions_and_physics():
    from types import SimpleNamespace
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    original=SimulationEnvironment(copy.deepcopy(cfg),seed=2)
    cfg['controller'].update(boundary_navigation='vector_goal',navigation_voter_continuity=True,navigation_hold_dwell=20)
    env=SimulationEnvironment(cfg,seed=2); c=env.controller
    assert env.contact_params==original.contact_params
    assert c.params.contact_contract()==original.controller.params.contact_contract()
    c._ensure_state(env.agents); oid=next(iter(c.tasks))
    for k,a in enumerate(env.agents):
        m=c.maps[a.agent_id]
        m.displacement[oid]=np.array([k*.01,0.]); m.velocity[oid]=np.zeros(2)
        m.motion_quality[oid]='credible';m.last_registration[oid]=SimpleNamespace(matches=5)
    neighbors=[[] for _ in env.agents]
    c._update_navigation(env.agents,neighbors)
    own=env.agents[0].agent_id
    before=c._navigation[own].displacement.copy()
    c.maps[env.agents[-1].agent_id].displacement[oid]=np.array([999.,999.])
    c._update_navigation(env.agents,neighbors)
    np.testing.assert_array_equal(c._navigation[own].displacement,before)
    assert c._navigation[own].voters==(own,)
    assert c.params.phase_gates().hold_dwell==20


def test_default_confirmation_does_not_change_phase_behavior():
    assert DBACTParams().navigation_voter_continuity is False
    assert DBACTParams().phase_gates().hold_dwell==1
    monitor=PhaseMonitor(gates=DBACTParams().phase_gates(),phase=Phase.BRAKE)
    monitor.update(PhaseSignals(progress=1.,target_distance=1.,estimated_speed=0.),1)
    assert monitor.phase==Phase.HOLD
