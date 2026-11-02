"""Geometric support regressions; these are not transport success samples."""
import numpy as np
import pytest
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from test_observed_reference import view, rectangle, moved


def noisy_line():
    rng=np.random.default_rng(7)
    points=np.c_[np.linspace(-.5,.5,80),rng.normal(0,.003,80)]
    normals=np.c_[rng.normal(0,.15,80),np.ones(80)]
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    return view(points,normals)


def test_noisy_normal_rank_does_not_make_a_line_a_two_dimensional_motion_reference():
    scan=noisy_line();old=ObservedReferenceTracker();candidate=ObservedReferenceTracker()
    # Setting the future opt-in flag also demonstrates the old implementation's
    # failure before the option exists: it silently accepts this same fit.
    candidate.support_gate=True
    for tr in (old,candidate):tr.initialize(scan,[.03,.02],0.)
    assert old._conditioning(scan.normals)>1e-3
    assert old.update(scan,.05) is not None
    assert candidate.update(scan,.05) is None
    assert candidate.reason=='support_nearly_linear'
    assert candidate.last_reading is None and candidate.last_time==0.
    np.testing.assert_array_equal(candidate.translation,[0.,0.])


def test_supported_rigid_translation_is_unchanged_by_gate():
    scan=rectangle();a=ObservedReferenceTracker();b=ObservedReferenceTracker()
    b.support_gate=True
    for tr in (a,b):tr.initialize(scan,[.04,-.01],0.)
    for k in range(1,21):
        current=moved(scan,[.005*k,.002*k]);ra=a.update(current,.05*k);rb=b.update(current,.05*k)
        assert ra is not None and rb is not None
        np.testing.assert_array_equal(ra.displacement,rb.displacement)
        np.testing.assert_array_equal(ra.velocity,rb.velocity)
        assert ra.matches==rb.matches and ra.residual_rms==rb.residual_rms


def test_support_gate_is_invariant_to_global_translation_and_rotation():
    scan=noisy_line()
    for shift,angle in [([0,0],0.),([12,-7],.73)]:
        tr=ObservedReferenceTracker();tr.support_gate=True
        transformed=moved(scan,shift,angle);tr.initialize(transformed,[0,0],0.)
        assert tr.update(transformed,.05) is None
        assert tr.reason=='support_nearly_linear'


def test_opt_in_api_noise_validation_and_empty_scan_abstention():
    from dbact.types import BoundaryView
    assert not ObservedReferenceTracker().support_gate
    for noise in (-1.,float('nan')):
        with pytest.raises(ValueError,match='noise scale'):
            ObservedReferenceTracker(support_gate=True,range_noise_std=noise)
    tr=ObservedReferenceTracker(support_gate=True)
    tr.initialize(rectangle(),[0.,0.],0.)
    assert tr.update(rectangle(),.05) is not None
    assert tr.support_minor_rms>.03
    assert tr.update(BoundaryView.empty(),.1) is None
    assert tr.last_reading is None and np.isnan(tr.support_minor_rms)


def test_controller_support_option_keeps_safety_physics_and_default_information_path():
    from dbact.control.controller import DBACTParams
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml
    from test_observed_reference import reference_environment
    assert not DBACTParams().reference_support_gate
    cfg=load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller']['reference_support_gate']=True
    with pytest.raises(ValueError,match='Reference support gate'):
        SimulationEnvironment(cfg,seed=2)
    cfg['controller'].update(boundary_navigation='vector_goal',navigation_estimator='observed_reference')
    new=SimulationEnvironment(cfg,seed=2);old=reference_environment()
    assert new.contact_params==old.contact_params
    assert new.controller.phase_monitor.gates==old.controller.phase_monitor.gates
    assert new.controller.safety.params==old.controller.safety.params
