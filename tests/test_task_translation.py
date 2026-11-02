"""Translation-model mechanisms, not a certificate for rotating objects."""
import numpy as np
import pytest

from dbact.control.observed_reference import rotation
from dbact.control.task_translation import TranslationTaskReferenceTracker
from dbact.types import BoundaryView


def view(points, normals):
    return BoundaryView(np.asarray(points), np.asarray(normals), np.ones(len(points)),
                        np.full(len(points), .01), np.full(len(points), 'cargo_0', dtype='<U32'))


def circle():
    angle = np.linspace(0., 2*np.pi, 160, endpoint=False)
    normals = np.c_[np.cos(angle), np.sin(angle)]
    return view([3., 4.]+.8*normals, normals)


def test_circle_translation_does_not_require_material_yaw_observability():
    scan = circle()
    tr = TranslationTaskReferenceTracker(reference_point=[4., 4.], support_gate=True)
    tr.initialize_initial(scan, 0.)
    frozen = tr.initialization_record()
    for k in range(1, 61):
        shift = np.array([k*.004, k*-.002])
        reading = tr.update(view(scan.points+shift, scan.normals), k*.05)
        assert reading is not None
        np.testing.assert_allclose(reading.displacement, shift, atol=1e-10)
        assert reading.angle == 0.
    assert tr.initialization_record() == frozen
    assert frozen['rotation_model'] == 'translation_hypothesis_not_measured_zero_yaw'
    assert frozen['registration_certified'] is False


def test_pure_translation_is_covariant_under_workspace_coordinate_rotation():
    scan, q, delta = circle(), np.array([4., 4.]), np.array([.012, -.007])
    r, shift = rotation(.6), np.array([-8., 13.])
    a = TranslationTaskReferenceTracker(reference_point=q)
    b = TranslationTaskReferenceTracker(reference_point=q@r.T+shift)
    a.initialize_initial(scan, 0.)
    b.initialize_initial(view(scan.points@r.T+shift, scan.normals@r.T), 0.)
    ra = a.update(view(scan.points+delta, scan.normals), .05)
    rb = b.update(view((scan.points+delta)@r.T+shift, scan.normals@r.T), .05)
    np.testing.assert_allclose(rb.displacement, ra.displacement@r.T, atol=1e-10)


def test_nearly_flat_noisy_normals_do_not_forge_two_dimensional_support():
    x = np.linspace(-.5, .5, 100)
    angle = .2*np.where(np.arange(len(x))%2, 1., -1.)
    scan = view(np.c_[x, np.zeros(len(x))], np.c_[np.sin(angle), np.cos(angle)])
    tr = TranslationTaskReferenceTracker(reference_point=[0., 0.], support_gate=True)
    tr.initialize_initial(scan, 0.)
    assert tr.initialized
    assert tr.update(scan, .05) is None
    assert tr.reason == 'support_nearly_linear'
    assert tr.last_time == 0.


def test_missing_returns_and_motion_limit_abstain_without_stale_votes():
    scan = circle()
    tr = TranslationTaskReferenceTracker(reference_point=[4., 4.], max_speed=.45)
    tr.initialize_initial(scan, 0.)
    assert tr.update(view(scan.points+[.004, 0.], scan.normals), .05) is not None
    assert tr.update(BoundaryView.empty(), .1) is None
    assert tr.last_reading is None
    assert tr.last_time == .05
    assert tr.update(view(scan.points+[.1, 0.], scan.normals), .1) is None
    assert tr.reason == 'motion_bound_exceeded'
    assert tr.last_time == .05
    with pytest.raises(ValueError, match='increasing'):
        tr.update(scan, .05)
    with pytest.raises(RuntimeError, match='replaced'):
        tr.initialize_initial(scan, 2.)
