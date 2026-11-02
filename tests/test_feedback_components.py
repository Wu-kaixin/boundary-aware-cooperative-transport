"""Channel isolation and permitted-voter regression tests, not physics evidence."""
from types import SimpleNamespace
import numpy as np
import pytest
from dbact.control.controller import DBACTController, DBACTParams
from dbact.control.phase import Phase, PhaseMonitor
from dbact.control.observed_reference import ReferenceReading

MODES=[('observed_reference_gate',False,False),
       ('observed_reference_position',True,False),
       ('observed_reference_velocity',False,True),
       ('observed_reference',True,True)]


def controller(mode,phase=Phase.CONTACT_READY):
    c=object.__new__(DBACTController)
    c.params=DBACTParams(boundary_navigation='vector_goal',navigation_estimator=mode)
    c.phase_monitor=PhaseMonitor();c.phase_monitor.phase=phase
    c.tasks={'cargo_0':SimpleNamespace(distance=1.2,direction=np.array([1.,0.]))}
    # Different values prevent an accidentally swapped channel from passing.
    c.maps={}
    for i,aid in enumerate(('a','b','unpermitted')):
        c.maps[aid]=SimpleNamespace(
            motion_quality={'cargo_0':'credible'},last_registration={'cargo_0':SimpleNamespace(matches=10)},
            object_displacement=lambda oid,i=i:np.array([.1+i*.1,.03]),
            object_velocity=lambda oid,i=i:np.array([.2+i*.1,-.04]))
    c._reference_readings={aid:ReferenceReading(np.array([.7+i*.1,-.02]),
        np.array([.01+i*.01,.005]),10,.001,1.,0.) for i,aid in enumerate(('a','b'))}
    return c,[SimpleNamespace(agent_id=aid) for aid in ('a','b','unpermitted')]


@pytest.mark.parametrize('mode,position,velocity',MODES)
def test_only_declared_channels_change_and_voters_are_common(mode,position,velocity):
    c,agents=controller(mode)
    # Gate-only explicitly uses old values on the new reference admission, even
    # when the old instantaneous map gate disagrees. It is a distinct control.
    c.maps['a'].motion_quality['cargo_0']='unavailable'
    c._update_navigation(agents,[[1],[],[]])
    assert c._navigation['a'].voters==('a','b')
    assert c._navigation['b'].voters==('b',)
    assert c._navigation['unpermitted'] is None
    assert c.team_navigation.voters==('a','b')
    np.testing.assert_allclose(c._navigation['a'].displacement,[.75,-.02] if position else np.array([.15,.03])*1.24)
    np.testing.assert_allclose(c._navigation['a'].velocity,[.015,.005] if velocity else [.25,-.04])


@pytest.mark.parametrize('mode,position,velocity',MODES)
def test_missing_reference_abstains_in_every_factorial_arm(mode,position,velocity):
    c,agents=controller(mode);c._reference_readings={}
    c._update_navigation(agents,[[],[],[]])
    assert c.team_navigation is None
    assert all(v is None for v in c._navigation.values())


@pytest.mark.parametrize('mode,position,velocity',MODES)
def test_pre_contact_deployment_feedback_is_not_changed(mode,position,velocity):
    c,agents=controller(mode,Phase.ENCLOSE)
    c._update_navigation(agents,[[],[],[]])
    np.testing.assert_array_equal(c._navigation['a'].displacement,np.array([.1,.03])*1.24)
    np.testing.assert_array_equal(c._navigation['a'].velocity,[.2,-.04])
    assert c.team_navigation.voters==('a','b','unpermitted')


def test_original_accumulated_retains_its_original_admission():
    c,agents=controller('accumulated')
    c.maps['a'].motion_quality['cargo_0']='unavailable'
    c._update_navigation(agents,[[],[],[]])
    assert c._navigation['a'] is None
    assert c.team_navigation.voters==('b','unpermitted')
    np.testing.assert_array_equal(c._navigation['b'].displacement,np.array([.2,.03])*1.24)
    assert DBACTParams().navigation_estimator=='accumulated'
