"""Opt-in recovery and local recruitment contracts; no task success assertions."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from dbact.control.controller import DBACTController, DBACTParams
from dbact.control.phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals
from dbact.control.contact_reinforcement import (
    ContactReinforcement, ReinforcementMessage, ReinforcementState, observed_route)


def signals(**kw):
    return replace(PhaseSignals(agent_count=16, contact_ready=8, progress=.2,
        target_distance=1., transport_active=0, recovery_ready=4), **kw)


def paused():
    return PhaseMonitor(gates=PhaseGates(transport_quorum=4),
                        phase=Phase.TRANSPORT, push_suspended=True)


def test_recovery_uses_prospective_quorum_with_full_original_dwell():
    m=paused()
    for k in range(m.gates.contact_regain_dwell-1):
        assert m.update(signals(),k)==Phase.TRANSPORT
        assert m.push_suspended
    m.update(signals(),19)
    assert not m.push_suspended and m.pause_frames==0


@pytest.mark.parametrize('kw',[{'recovery_ready':None},{'recovery_ready':3},
                              {'estimate_quality':'unavailable'}])
def test_disabled_insufficient_or_unknown_recovery_cannot_resume(kw):
    m=paused()
    for k in range(80):m.update(signals(**kw),k)
    assert m.phase==Phase.BRAKE
    assert m.brake_reason in ('contact_recovery_failed','estimate_unavailable')


def test_recovery_transients_do_not_accumulate_or_bypass_brake_hold():
    m=paused()
    for k in range(19):m.update(signals(),k)
    m.update(signals(recovery_ready=3),19)
    m.update(signals(),20)
    assert m.push_suspended and m.regain_streak==1
    m.update(signals(progress=1.),21)
    assert m.phase==Phase.BRAKE
    m.update(signals(progress=1.),22)
    assert m.phase==Phase.HOLD
    m.update(signals(progress=.2,transport_active=4),23)
    assert m.phase==Phase.HOLD


def step(policy,t,*,robot='b',inbox=(),position=(-.16,0.),velocity=(0.,0.),
         active=True,ready=True,press=False,suspended=False):
    return policy.decide(robot,[[0.,0.]], [[-1.,0.]], position,[],[1.,0.],velocity,
        1.,inbox,timestamp=t,dt=.05,active=active,ready=ready,
        previous_press=press,suspended=suspended)


def test_recruitment_without_donor_and_joining_does_not_block_next_spare():
    p=ContactReinforcement(DBACTParams())
    events=[]
    for k in range(22):
        # Existing helper has arrived, but may need another helper to move cargo.
        msg=ReinforcementMessage('a',k*.05,'joining',True)
        _,_,trace=step(p,k*.05,inbox=[msg]);events.extend(trace['reinforcement_event'].split('|'))
    assert 'recruit_request' in events and 'recruit_started' in events
    assert p.states['b'].attempts==1


def test_only_fresh_local_competitor_defers_request():
    for timestamp,expected in [(0.,'joining'),(1.,'idle')]:
        p=ContactReinforcement(DBACTParams())
        p.states['b']=ReinforcementState(state='request',route=[np.array([-.3,0.])])
        step(p,1.,inbox=[ReinforcementMessage('a',timestamp,'request')])
        assert p.states['b'].state==expected


def test_observed_route_rejects_disconnected_or_occupied_patch():
    p=DBACTParams()
    # The useful patch exists but is not joined to the currently observed patch.
    assert observed_route(np.array([[0.,0.],[10.,0.]]),np.array([[0.,1.],[-1.,0.]]),
                          [0.,.2],np.empty((0,2)),[1.,0.],p)==[]
    assert observed_route(np.array([[0.,0.]]),np.array([[-1.,0.]]),[-.2,0.],
                          np.array([[-p.hold_offset,0.]]),[1.,0.],p)==[]


def test_no_response_cannot_confirm_and_recruitment_is_bounded():
    p=ContactReinforcement(DBACTParams())
    events=[]
    for k in range(1000):
        command,_,tr=step(p,k*.05,position=(-.2,0.))
        events.extend(tr['reinforcement_event'].split('|'))
        assert not tr['reinforcement_confirmed']
        if command is not None:assert np.linalg.norm(command)<=p.params.max_speed+1e-12
    assert p.states['b'].attempts==3
    assert 'rollback:response_timeout' in events


def test_group_motion_confirmation_requires_dwell_and_is_revocable():
    p=ContactReinforcement(DBACTParams())
    p.states['b']=ReinforcementState(state='joining')
    for k in range(19):
        _,_,tr=step(p,k*.05,position=(-.12,0.),velocity=(.02,0.),press=True)
        assert not tr['reinforcement_confirmed']
    _,_,tr=step(p,.95,position=(-.12,0.),velocity=(.02,0.),press=True)
    assert tr['reinforcement_event']=='group_response_confirmed'
    _,_,tr=step(p,1.,position=(-.12,0.),velocity=(0.,0.),press=True)
    assert not tr['reinforcement_confirmed']


def test_incumbent_never_asked_to_relocate_and_pause_approach_has_no_penetration_setpoint():
    p=ContactReinforcement(DBACTParams())
    for k in range(40):
        command,retain,tr=step(p,k*.05,position=(-.12,0.),press=True)
        assert command is None and retain and p.states['b'].attempts==0
    command,retain,tr=step(p,2.,position=(-.12,0.),press=True,suspended=True)
    assert not retain and command[0]<0  # outward to original readiness band
    command,retain,tr=step(p,2.05,active=False,press=True)
    assert command is None and not retain and tr['reinforcement_state']=='idle'


def test_prospective_readiness_counts_only_declared_neighbors_and_needs_local_estimate():
    from dbact.types import BoundaryView
    c=object.__new__(DBACTController);c.params=DBACTParams(contact_recovery_fix=True,min_push_agents=4)
    c.phase_monitor=PhaseMonitor(phase=Phase.TRANSPORT,push_suspended=True)
    agents=[SimpleNamespace(agent_id=str(i),position=np.array([-.12,0.])) for i in range(6)]
    c._views={'0':BoundaryView(points=np.array([[0.,0.]]),normals=np.array([[-1.,0.]]),
        confidence=np.ones(1),arc_length=np.ones(1),object_ids=np.array(['cargo_0']))}
    c._navigation={'0':SimpleNamespace(remaining=1.,direction=np.array([1.,0.]))}
    assert not c._prospective_push_ready(0,agents,[1,2],[True]*6)
    assert c._prospective_push_ready(0,agents,[1,2,3],[True]*6)
    c._navigation['0']=None
    assert not c._prospective_push_ready(0,agents,[1,2,3],[True]*6)


def test_retained_pusher_keeps_original_pressure_and_hold_permission(monkeypatch):
    from test_boundary_navigation import environment
    from dbact.types import BoundaryView
    outputs=[]
    for enabled in (False,True):
        env=environment('vector_goal');c=env.controller;c._ensure_state(env.agents)
        c.params.contact_recovery_fix=enabled;c.params.contact_reinforcement=enabled
        oid=next(iter(c.tasks));goal=c.goal_directions[oid];normal=-goal
        point=np.array([4.,4.]);env.agents[0].position=point+(c._press_floor+.02)*normal
        view=BoundaryView(points=point[None,:],normals=normal[None,:],confidence=np.ones(1),
            arc_length=np.ones(1),object_ids=np.array([oid]))
        c.phase_monitor.phase=Phase.CONTACT_READY
        c._navigation[env.agents[0].agent_id]=SimpleNamespace(remaining=.5,direction=goal,velocity=np.zeros(2))
        c._reinforcement_retain={env.agents[0].agent_id} if enabled else set()
        outputs.append(c._transport_command(0,env.agents,list(range(1,5)),view,[True]*16,.05,c.density_params))
        c.phase_monitor.phase=Phase.HOLD
        assert not c._transport_command(0,env.agents,list(range(1,5)),view,[True]*16,.05,c.density_params)[1]
    assert outputs[0][1] and outputs[1][1]
    np.testing.assert_array_equal(outputs[0][0],outputs[1][0])
    assert outputs[0][2]==outputs[1][2]


def test_defaults_and_incompatible_modes_are_explicit():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    assert not DBACTParams().contact_recovery_fix and not DBACTParams().contact_reinforcement
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(contact_reinforcement=True)
    with pytest.raises(ValueError):SimulationEnvironment(cfg,seed=2)


def test_controller_sends_only_actual_one_hop_messages_and_stops_at_hold(monkeypatch):
    from test_boundary_navigation import environment
    env=environment('vector_goal');c=env.controller
    c.params.contact_recovery_fix=True;c.params.contact_reinforcement=True;c.trace_enabled=True
    for _ in range(40):env.step()
    for i,a in enumerate(env.agents):
        c._reinforcement_messages[a.agent_id]=ReinforcementMessage(a.agent_id,c._time,'joining')
    seen=[]
    original=c.contact_reinforcement.decide
    def capture(*args,**kwargs):
        seen.extend(m.robot for m in args[8])
        return original(*args,**kwargs)
    monkeypatch.setattr(c.contact_reinforcement,'decide',capture)
    a=env.agents[0];neighbour=env.agents[1]
    c._trace={}
    c._reinforcement_command(a,c._views[a.agent_id],[neighbour],True,.05)
    assert seen==[neighbour.agent_id]
    c.phase_monitor.phase=Phase.HOLD
    env.step()
    assert not any(d.push_side or d.reinforcement_retain for d in c.diagnostics)
