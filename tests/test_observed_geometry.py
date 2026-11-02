"""Synthetic readout checks; no claims about transport completion."""
import numpy as np
import pytest
from dbact.control.observed_geometry import radial_pivot, patch_distance
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact.types import BoundaryView


def view(points, normals):
    return BoundaryView(points=np.asarray(points),normals=np.asarray(normals),
        confidence=np.ones(len(points)),arc_length=np.full(len(points),.03),
        object_ids=np.full(len(points),'cargo_0',dtype='<U32'))


def arc():
    angles=np.linspace(-1.9,1.3,100)
    n=np.c_[np.cos(angles),np.sin(angles)]
    return view([3.,4.]+.7*n,n)


def test_observed_radial_pivot_is_translation_and_rotation_equivariant():
    scan=arc();pivot,status,rms=radial_pivot(scan.points,.01)
    assert status=='accepted' and rms<1e-12
    np.testing.assert_allclose(pivot,[3.,4.],atol=1e-12)
    r=rotation(.7);shift=np.array([-5.,3.])
    other,_,_=radial_pivot(scan.points@r.T+shift,.01)
    np.testing.assert_allclose(other,r@pivot+shift,atol=1e-12)


def test_model_rejects_flat_and_noncircular_boundaries():
    x=np.linspace(-1.,1.,80)
    assert radial_pivot(np.c_[x,x*0],.01)[0] is None
    a=np.linspace(0,2*np.pi,100,endpoint=False)
    assert radial_pivot(np.c_[2*np.cos(a),np.sin(a)],.01)[0] is None
    assert radial_pivot(np.ones((3,2)),.01)[0] is None


def test_readout_tracks_center_of_registered_geometry_and_its_velocity():
    scan=arc();tracker=ObservedReferenceTracker(radial_readout=True)
    tracker.initialize(scan,[0.,0.],0.)
    assert tracker.pivot_status=='accepted'
    center=np.array([3.,4.])
    previous=np.zeros(2);velocity=np.zeros(2)
    for k in range(1,41):
        delta=np.array([.002*k,-.001*k]);r=rotation(.003*k)
        moved=view((scan.points-center)@r.T+center+delta,scan.normals@r.T)
        reading=tracker.update(moved,k*.05)
        assert reading is not None
        # The finite-iteration ICP fit is approximate. Independently refit the
        # transformed reference geometry; this checks which point the readout
        # follows without falsely demanding exact physical truth from ICP.
        registered=tracker.anchor+tracker.translation+tracker.reference@rotation(tracker.angle).T
        fitted,_,_=radial_pivot(registered,.01)
        expected=fitted-center
        np.testing.assert_allclose(reading.displacement,expected,atol=2e-5)
        velocity=.65*velocity+.35*(expected-previous)/.05
        np.testing.assert_allclose(reading.velocity,velocity,atol=2e-5)
        previous=expected


def test_rejected_radial_model_has_exact_original_readout_and_admission():
    from test_observed_reference import rectangle,moved
    scan=rectangle();old=ObservedReferenceTracker();new=ObservedReferenceTracker(radial_readout=True)
    for t in (old,new):t.initialize(scan,[.03,.01],0.)
    assert new.pivot_status!='accepted'
    for k in range(1,21):
        a=old.update(moved(scan,[k*.002,0.],k*.001),k*.05)
        b=new.update(moved(scan,[k*.002,0.],k*.001),k*.05)
        np.testing.assert_array_equal(a.displacement,b.displacement)
        np.testing.assert_array_equal(a.velocity,b.velocity)
        assert a.angle==b.angle and a.matches==b.matches


def test_patch_interpolates_curvature_without_nearest_outlier_bias():
    x=np.linspace(-.08,.08,21);y=.5*x*x
    scan=view(np.c_[x,y],np.tile([0.,1.],(len(x),1)))
    gap,status,n=patch_distance(scan,np.array([0.,.13]),np.array([0.,0.]),
        np.array([0.,1.]),'cargo_0',.12,.03)
    assert status=='accepted' and n==21 and gap==pytest.approx(.13)
    assert np.array_equal(scan.points,np.c_[x,y])


def test_patch_abstains_on_occlusion_wrong_object_and_inconsistent_normals():
    x=np.linspace(.01,.1,10);n=np.tile([0.,1.],(10,1));scan=view(np.c_[x,x*0],n)
    args=(np.array([0.,.13]),np.zeros(2),np.array([0.,1.]),'cargo_0',.12,.03)
    assert patch_distance(scan,*args)[1]=='no_interpolation'
    scan=view(np.c_[np.linspace(-.1,.1,10),x*0],-n)
    assert patch_distance(scan,*args)[0] is None


def test_geometry_factors_preserve_physics_phase_and_safety_rows():
    from test_observed_reference import reference_environment
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    old=reference_environment()
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(boundary_navigation='vector_goal',navigation_estimator='observed_reference',
        reference_radial_pivot=True,contact_patch_distance=True)
    new=SimulationEnvironment(cfg,seed=2)
    assert new.contact_params==old.contact_params
    assert new.controller.phase_monitor.gates==old.controller.phase_monitor.gates
    assert new.controller.safety.params==old.controller.safety.params
    from test_observed_reference import rectangle
    scan=rectangle();rows=[]
    for env in (old,new):
        c=env.controller;c._ensure_state(env.agents);a=env.agents[0]
        rows.append(c._object_rows_from_map(a.agent_id,a.position,scan))
    for a,b in zip(*rows):
        if a is None:assert b is None
        else:np.testing.assert_array_equal(a,b)


def test_geometry_flags_are_opt_in_and_reject_wrong_information_path():
    from dbact.control.controller import DBACTParams
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    assert not DBACTParams().reference_radial_pivot
    assert not DBACTParams().contact_patch_distance
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller']['reference_radial_pivot']=True
    with pytest.raises(ValueError,match='Radial pivot requires'):
        SimulationEnvironment(cfg,seed=2)
