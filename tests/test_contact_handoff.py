"""Contact handoff experiment permissions and isolated mechanism regressions."""
import numpy as np
from dataclasses import replace
import pytest

from dbact.control.contact_handoff import ConfirmedContactHandoff, HandoffMessage, HandoffState
from dbact.control.controller import DBACTParams


def test_allocation_only_never_spends_relocation_budget():
    from dbact.control.controller import DBACTParams
    from dbact.control.contact_roles import DynamicContactPolicy
    p = DBACTParams(dynamic_contact_roles=True, dynamic_contact_relocation=False)
    policy = DynamicContactPolicy(p)
    theta = np.array([0., .6])
    points = np.c_[np.cos(theta), np.sin(theta)]
    for _ in range(100):
        _, command, trace = policy.decide(
            'r', points, points, np.array([1.13, 0.]), [], np.array([0., -1.]),
            np.array([1., 0.]), np.array([0., .2]), np.zeros(2), .05, active=True)
        assert command is None
        assert trace['contact_redeploy_reason'] == 'allocation_only'
    assert policy.states['r'].attempts == 0


def test_relocation_only_does_not_override_pressure_membership():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg = load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(dynamic_contact_roles=True, dynamic_contact_allocation=False)
    env = SimulationEnvironment(cfg, seed=2)
    for _ in range(40):
        env.step()
        assert not env.controller._contact_allocations


def donor_step(policy, t, inbox, *, velocity=(.05, -.005), active=True):
    return policy.decide('donor', [[0.,0.]], [[-1.,0.]], [-.12,0.], [],
                         np.array([1.,-1.])/np.sqrt(2), np.array([1.,0.]),
                         np.array([0.,.2]), np.asarray(velocity), .05, t, inbox,
                         active=active,usable=True,previous_press=True)


def warm_donor(policy):
    for k in range(20): donor_step(policy,k*.05,[])


def test_unsolicited_ready_message_cannot_release_original_pusher():
    policy=ConfirmedContactHandoff(DBACTParams())
    msg=HandoffMessage('new',0.,state='ready',partner='donor',normal=(0.,1.),
                       target_normal=(0.,1.),press=True,contact_proxy=True,usable=True)
    for k in range(40):
        allocation,command,trace=donor_step(policy,k*.05,[replace(msg,timestamp=k*.05)])
        assert allocation is None and command is None
        assert trace['handoff_release_fraction']==0


def test_donor_keeps_pressure_before_confirmation_and_rolls_back_stale_link():
    policy=ConfirmedContactHandoff(DBACTParams())
    warm_donor(policy)
    s=policy.states['donor'];s.donor_state='holding';s.donor_partner='new'
    msg=HandoffMessage('new',1.,state='engage',partner='donor',press=True,usable=True)
    for k in range(20):
        t=1+k*.05
        allocation,_,_=donor_step(policy,t,[replace(msg,timestamp=t)])
        assert allocation is None
    _,_,trace=donor_step(policy,3.,[msg])
    assert trace['handoff_event']=='rollback:replacement_or_link_lost'
    assert s.donor_partner=='' and s.fraction==0


def test_confirmed_replacement_gets_gradual_unload_then_response_loss_restores():
    policy=ConfirmedContactHandoff(DBACTParams())
    warm_donor(policy)
    s=policy.states['donor'];s.donor_state='holding';s.donor_partner='new'
    msg=HandoffMessage('new',1.,state='ready',partner='donor',press=True,
                       contact_proxy=True,usable=True)
    fractions=[]
    for k in range(25):
        t=1+k*.05
        allocation,command,trace=donor_step(policy,t,[replace(msg,timestamp=t)])
        fractions.append(trace['handoff_release_fraction'])
        assert allocation is not None and 0 <= allocation <= 1 and command is None
    assert 0<fractions[0]<1 and fractions[-1]==1
    assert s.donor_state=='retired'
    events=[]
    for k in range(45):
        t=2.25+k*.05
        allocation,_,trace=donor_step(policy,t,[replace(msg,timestamp=t)],velocity=(0.,.02))
        events.append(trace['handoff_event'])
    assert 'rollback:unload_response_lost' in events
    assert allocation is None and s.donor_state=='idle'


def test_offer_round_trip_does_not_cancel_acknowledgement():
    policy=ConfirmedContactHandoff(DBACTParams())
    # A backward-and-wrong-lateral normal makes a better incoming patch possible.
    n=np.array([-.6,-.8])
    def step(t,msg):
        return policy.decide('donor',[[0.,0.]],[n],.12*n,[],np.array([1.,-1.])/np.sqrt(2),
            np.array([1.,0.]),np.array([0.,.2]),np.array([.05,-.005]),.05,t,[msg],
            active=True,usable=True,previous_press=True)
    msg=HandoffMessage('new',0.,state='offer',partner='donor',target_normal=(-.6,.8),usable=True)
    step(0.,msg)
    assert policy.states['donor'].donor_partner=='new'
    step(.05,replace(msg,timestamp=.05))
    assert policy.states['donor'].donor_state=='holding'


def incoming_step(policy,t,inbox,*,position=(0.,.12),velocity=(.05,-.005),press=True):
    return policy.decide('new',[[0.,0.]],[[0.,1.]],np.array(position),[],
        np.array([1.,-1.])/np.sqrt(2),np.array([1.,0.]),np.array([0.,.2]),
        np.array(velocity),.05,t,inbox,active=True,usable=True,previous_press=press)


@pytest.mark.parametrize('gap,velocity,press',[(.2,(.05,-.005),True),(.12,(0.,0.),True),(.12,(.05,-.005),False)])
def test_arrival_without_contact_command_and_motion_cannot_confirm(gap,velocity,press):
    policy=ConfirmedContactHandoff(DBACTParams())
    policy.states['new']=HandoffState(state='engage',partner='donor',origin=np.array([0.,.2]),target=np.array([0.,.12]))
    msg=HandoffMessage('donor',0.,donor_state='holding',donor_partner='new',press=True,usable=True)
    for k in range(50):
        incoming_step(policy,k*.05,[replace(msg,timestamp=k*.05)],position=(0.,gap),velocity=velocity,press=press)
    assert policy.states['new'].state=='engage'
    assert policy.states['new'].confirm==0


def test_incoming_requires_sustained_proxy_before_ready():
    policy=ConfirmedContactHandoff(DBACTParams())
    policy.states['new']=HandoffState(state='engage',partner='donor',origin=np.array([0.,.2]),target=np.array([0.,.12]))
    msg=HandoffMessage('donor',0.,donor_state='holding',donor_partner='new',press=True,usable=True)
    events=[]
    for k in range(45):
        _,_,trace=incoming_step(policy,k*.05,[replace(msg,timestamp=k*.05)])
        events.append(trace['handoff_event'])
    assert events.count('locally_confirmed')==1
    assert events.index('locally_confirmed')>=38  # full motion window then one dwell


def test_only_spare_robot_moves_and_missing_ack_has_bounded_attempts():
    policy=ConfirmedContactHandoff(DBACTParams())
    msg=HandoffMessage('donor',0.,normal=(-.6,-.8),press=True,contact_proxy=True,usable=True)
    events=[]
    rollbacks=[]
    for k in range(200):
        t=k*.05
        _,command,trace=incoming_step(policy,t,[replace(msg,timestamp=t)],press=False)
        events.append(trace['handoff_event'])
        rollbacks.append(trace['handoff_rollback_reason'])
        assert command is None  # it remains at the original patch without acceptance
    assert events.count('proposed')==3
    assert policy.states['new'].attempts==3
    assert 'offer_unacknowledged' in rollbacks


def test_handoff_integrates_with_controller_and_hold_cancels_permissions():
    from dbact.control.phase import Phase
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller']['confirmed_contact_handoff']=True
    env=SimulationEnvironment(cfg,seed=2)
    env.controller.trace_enabled=True
    for _ in range(40): env.step()
    assert env.controller._handoff_messages
    assert all(np.isfinite(a.position).all() for a in env.agents)
    env.controller.phase_monitor.phase=Phase.HOLD
    env.step()
    assert not any(d.push_side for d in env.controller.diagnostics)
    assert all(s.state==s.donor_state=='idle' for s in env.controller.contact_handoff.states.values())
    assert not env.controller._contact_allocations


def test_old_joint_and_new_handoff_cannot_be_mixed_silently():
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(confirmed_contact_handoff=True,dynamic_contact_roles=True)
    with pytest.raises(ValueError,match='dynamic_contact_roles=False'):
        SimulationEnvironment(cfg,seed=2)
