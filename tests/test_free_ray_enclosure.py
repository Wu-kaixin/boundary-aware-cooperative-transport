"""Conditional topology tests; no unconditional noisy-sensor certificate."""
import numpy as np
import pytest
from shapely.geometry import Polygon

from dbact.sensing.free_space import free_ray_enclosure


def test_raw_packet_uses_only_measured_ranges_ids_and_calibrated_rays():
    from dbact.sensing.free_space import measured_free_ray_packet
    # Truth fields and a supplied point/direction are deliberately poisoned.
    raw = [dict(ray_index=0, range=.5, object_id='target', true_normal='forbidden',
                point='forbidden', direction='forbidden'),
           dict(ray_index=1, range=1.3, object_id='occluder')]
    starts, ends, points = measured_free_ray_packet([2., 3.], raw, ray_count=4,
        sensor_range=1.2, margin=.06, target_id='target')
    np.testing.assert_allclose(starts, [[2., 3.]]*4)
    np.testing.assert_allclose(ends, [[2.44, 3.], [2., 4.14], [.86, 3.], [2., 1.86]])
    np.testing.assert_allclose(points, [[2.5, 3.]])
    assert raw[0]['true_normal'] == 'forbidden'


def test_empty_raw_scan_still_has_measured_no_hit_segments_but_no_target_points():
    from dbact.sensing.free_space import measured_free_ray_packet
    starts, ends, points = measured_free_ray_packet([0., 0.], [], ray_count=4,
        sensor_range=1.2, margin=.06, target_id='target')
    assert points.shape == (0, 2)
    np.testing.assert_allclose(np.linalg.norm(ends-starts, axis=1), 1.14)


@pytest.mark.parametrize('raw', [
    [dict(ray_index=0, range=.5, object_id='x')]*2,
    [dict(ray_index=4, range=.5, object_id='x')],
    [dict(ray_index=0, range=float('nan'), object_id='x')],
    [dict(ray_index=0, range=-.5, object_id='x')]])
def test_invalid_or_duplicate_raw_returns_are_rejected(raw):
    from dbact.sensing.free_space import measured_free_ray_packet
    with pytest.raises(ValueError):
        measured_free_ray_packet([0., 0.], raw, ray_count=4, sensor_range=1.2,
                                margin=.06, target_id='x')


WORKSPACE = np.array([[-5., -5.], [5., -5.], [5., 5.], [-5., 5.]])


def free_loop():
    # Crossing free segments close a box; no interpolation between laser rays.
    return np.array([[-3., -2.], [2., -3.], [3., 2.], [-2., 3.]]), np.array([
        [3., -2.], [2., 3.], [-3., 2.], [-2., -3.]])


def test_closed_free_ray_loop_contains_the_whole_connected_object():
    starts, ends = free_loop()
    observed = np.array([[-1., 0.], [1., 0.], [0., -1.], [0., 1.]])
    result = free_ray_enclosure(starts, ends, observed, WORKSPACE, margin=0.)
    assert result.status == 'BOUNDED_BY_FREE_RAYS'
    assert result.feasible_faces == 1
    assert result.center_area == pytest.approx(16.)
    assert Polygon(result.vertices).covers(Polygon([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]))
    # Input points and segments are unchanged.
    np.testing.assert_array_equal(starts, free_loop()[0])
    np.testing.assert_array_equal(observed, [[-1., 0.], [1., 0.], [0., -1.], [0., 1.]])


def test_dangling_free_ray_does_not_invent_an_enclosure():
    result = free_ray_enclosure([[-4., 0.]], [[-1.5, 0.]], [[0., 0.]], WORKSPACE, margin=.06)
    assert result.status == 'WORKSPACE_ONLY'
    assert result.center_area == pytest.approx(100.)


def test_missing_or_contradictory_returns_keep_the_workspace_bound():
    starts, ends = free_loop()
    for points, reason in [([], 'NO_TARGET_RETURNS'), ([[0., 0.], [3., 0.]], 'INCONSISTENT_RETURNS')]:
        result = free_ray_enclosure(starts, ends, points, WORKSPACE, margin=0.)
        assert result.status == reason
        assert Polygon(result.vertices).equals(Polygon(WORKSPACE))


def test_every_noise_consistent_face_is_retained_not_just_the_smallest():
    # Unequal cells: the observation uncertainty touches both sides.
    result = free_ray_enclosure([[-3., -5.]], [[-3., 5.]], [[-3.01, 0.]], WORKSPACE, margin=.06)
    assert result.feasible_faces == 2
    assert result.center_area == pytest.approx(100.)
    assert result.status == 'WORKSPACE_ONLY'


def test_nonconvex_face_requires_a_convex_centroid_set():
    loop = np.array([[-2., -2.], [2., -2.], [2., -1.], [-1., -1.], [-1., 2.], [-2., 2.]])
    face = Polygon(loop)
    assert not face.covers(face.centroid)
    result = free_ray_enclosure(loop, np.roll(loop, -1, axis=0), [[0., -1.5], [-1.5, 0.]], WORKSPACE, margin=0.)
    assert result.feasible_faces == 1
    assert Polygon(result.vertices).equals(face.convex_hull)
    assert Polygon(result.vertices).covers(face.centroid)


def test_no_rays_or_zero_length_rays_do_not_shrink_the_set():
    for starts, ends in [([], []), ([[0., 0.]], [[0., 0.]])]:
        result = free_ray_enclosure(starts, ends, [[1., 1.]], WORKSPACE, margin=.06)
        assert result.status == 'WORKSPACE_ONLY'
        assert result.center_area == pytest.approx(100.)


def test_rigid_coordinate_change_preserves_enclosure():
    starts, ends = free_loop()
    points = np.array([[-1., 0.], [1., 0.]])
    result = free_ray_enclosure(starts, ends, points, WORKSPACE, margin=.06)
    a = .72
    rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    shift = np.array([13., -8.])
    transform = lambda x: x@rotation.T+shift
    changed = free_ray_enclosure(transform(starts), transform(ends), transform(points), transform(WORKSPACE), margin=.06)
    assert changed.status == result.status
    assert Polygon(changed.vertices).hausdorff_distance(Polygon(transform(result.vertices))) < 1e-10


@pytest.mark.parametrize('which', ['margin', 'nan_ray', 'length', 'workspace', 'point'])
def test_invalid_inputs_are_not_claimed_as_valid_geometry(which):
    starts, ends = free_loop()
    domain, points, margin = WORKSPACE.copy(), [[0., 0.]], .06
    if which == 'margin': margin = -1.
    if which == 'nan_ray': starts[0, 0] = np.nan
    if which == 'length': ends = ends[:-1]
    if which == 'workspace': domain = np.zeros((4, 2))
    if which == 'point': points = [[np.inf, 0.]]
    with pytest.raises(ValueError):
        free_ray_enclosure(starts, ends, points, domain, margin=margin)
