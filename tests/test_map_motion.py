"""S3 motion compensation: a map of a moving body has to move, and has to forget."""

import numpy as np
import pytest

from dbact.boundary_map import LocalBoundaryMap
from dbact.types import BoundaryView


def wall_scan(offset: np.ndarray = np.zeros(2), count: int = 25, spacing: float = 0.05) -> BoundaryView:
    """A flat face along x with outward normal +y, translated by ``offset``."""
    xs = (np.arange(count) - count // 2) * spacing
    points = np.column_stack([xs, np.zeros(count)]) + np.asarray(offset, dtype=float)[None, :]
    return BoundaryView(
        points=points,
        normals=np.tile([0.0, 1.0], (count, 1)),
        confidence=np.full(count, 0.9),
        arc_length=np.full(count, spacing),
        object_ids=np.full(count, "obj", dtype="<U32"),
    )


def corner_scan(offset: np.ndarray = np.zeros(2), count: int = 13, spacing: float = 0.05) -> BoundaryView:
    """Two faces at right angles, so translation is observable in both axes."""
    a, b = wall_scan(offset, count, spacing), wall_scan(offset, count, spacing)
    side = np.column_stack([np.full(count, 0.4), (np.arange(count) - count // 2) * spacing])
    b = BoundaryView(
        points=side + np.asarray(offset, dtype=float)[None, :],
        normals=np.tile([1.0, 0.0], (count, 1)),
        confidence=b.confidence,
        arc_length=b.arc_length,
        object_ids=b.object_ids,
    )
    return BoundaryView(
        points=np.vstack([a.points, b.points]),
        normals=np.vstack([a.normals, b.normals]),
        confidence=np.concatenate([a.confidence, b.confidence]),
        arc_length=np.concatenate([a.arc_length, b.arc_length]),
        object_ids=np.concatenate([a.object_ids, b.object_ids]),
    )


def fresh_map(**kwargs) -> LocalBoundaryMap:
    defaults = dict(voxel_size=0.06, age_decay=0.0, max_weight=4.0, carve_enabled=False)
    defaults.update(kwargs)
    return LocalBoundaryMap(**defaults)


def test_translation_is_recovered_from_two_scans_of_a_corner():
    m = fresh_map()
    m.update(corner_scan(), 0.0, dt=0.05)
    shift = np.array([0.02, 0.015])
    # register/update is the order the controller uses: the estimate is committed
    # when the scan is fused, so that the fusion's own pull on the cells it lands
    # in is counted as observed motion rather than lost.
    m.register(corner_scan(shift), dt=0.05)
    m.update(corner_scan(shift), 0.05, dt=0.05)
    assert np.allclose(m.object_displacement("obj"), shift, atol=4e-3)


def test_the_map_itself_moves_so_the_cage_does_not_chase_a_ghost():
    m = fresh_map()
    m.update(corner_scan(), 0.0, dt=0.05)
    before = m.view(0.0).points.max(axis=0)
    shift = np.array([0.03, 0.0])
    m.register(corner_scan(shift), dt=0.05)
    after = m.view(0.0).points.max(axis=0)
    # The extent is the thing to compare, not the mean: rekeying after the shift
    # merges cells that have moved into one another, which moves the mean for a
    # reason that has nothing to do with the estimate.
    assert np.allclose(after - before, shift, atol=4e-3)
    assert np.allclose(m.last_registration["obj"].translation, shift, atol=4e-3)


def test_a_single_flat_face_reports_motion_along_its_normal_and_nothing_along_itself():
    """Point-to-plane, not point-to-point: a range scan slides freely along a
    surface, and point-to-point would report tangential motion that did not
    happen."""
    m = fresh_map()
    m.update(wall_scan(), 0.0, dt=0.05)
    m.register(wall_scan(np.array([0.04, 0.02])), dt=0.05)
    m.update(wall_scan(np.array([0.04, 0.02])), 0.05, dt=0.05)
    estimate = m.object_displacement("obj")
    assert estimate[1] == pytest.approx(0.02, abs=3e-3)
    assert abs(estimate[0]) < 5e-3
    assert not m.last_registration["obj"].observable


def test_velocity_is_the_translation_over_the_step():
    m = fresh_map(velocity_filter=1.0)
    m.update(corner_scan(), 0.0, dt=0.05)
    m.register(corner_scan(np.array([0.01, 0.0])), dt=0.05)
    m.update(corner_scan(np.array([0.01, 0.0])), 0.05, dt=0.05)
    assert m.object_velocity("obj")[0] == pytest.approx(0.2, rel=0.35)


def test_the_estimate_is_clamped_to_a_physically_possible_step():
    m = fresh_map(max_object_speed=0.2)
    m.update(corner_scan(), 0.0, dt=0.05)
    m.register(corner_scan(np.array([5.0, 0.0])), dt=0.05)
    assert float(np.linalg.norm(m.object_displacement("obj"))) <= 0.2 * 0.05 + 1e-9


def test_cells_are_rekeyed_after_a_shift_so_the_map_does_not_double_up():
    """Without rekeying the next scan lands in the cell the record moved into,
    finds it empty, and creates a second record for the same boundary; the map
    doubles along the direction of travel and the estimate collapses."""
    m = fresh_map()
    m.update(corner_scan(), 0.0, dt=0.05)
    size = len(m)
    for step in range(1, 12):
        shift = np.array([0.0, 0.01 * step])
        m.register(corner_scan(shift), dt=0.05)
        m.update(corner_scan(shift), 0.05 * step, dt=0.05)
    assert len(m) <= size + 6


def test_fusion_weight_saturates_so_an_old_cell_is_an_estimate_not_an_archive():
    m = fresh_map(max_weight=4.0)
    for step in range(200):
        m.update(wall_scan(), 0.05 * step, dt=0.05)
    assert float(np.max(m._weight)) <= 4.0 + 1e-9
    # A fresh return still moves the cell appreciably after two hundred steps.
    before = m.view(10.0).points[:, 1].mean()
    m.update(wall_scan(np.array([0.0, 0.02])), 10.0, dt=0.05)
    after = m.view(10.0).points[:, 1].mean()
    assert after - before > 0.002


def test_carving_removes_the_trail_the_object_has_left_behind():
    """The new returns land in a different voxel, so the stale cell survives age
    decay and remains the nearest boundary point the robot has -- which is what
    made the pushing arc press against a surface that was no longer there."""
    m = fresh_map(carve_enabled=True)
    m.update(wall_scan(), 0.0, dt=0.05)
    observer = np.array([0.0, -1.0])  # below the wall, looking up at it
    moved = wall_scan(np.array([0.0, 0.30]))
    m.update(moved, 0.05, dt=0.05)
    assert len(m) > len(moved.points)  # both the stale and the fresh rows are held
    stale_before = int(np.sum(m.view(0.05).points[:, 1] < 0.15))
    removed = m.carve(observer, moved)
    stale_after = int(np.sum(m.view(0.05).points[:, 1] < 0.15))
    assert removed > 0
    # Only the cells a ray actually passes through are dropped. The two at each
    # end of the old face lie outside the moved face's narrower angular extent, so
    # nothing sees through them and they are left to age out -- carving removes
    # what the scan contradicts, not what it merely fails to confirm.
    assert stale_after < 0.25 * stale_before


def test_carving_never_removes_the_surface_the_scan_just_measured():
    m = fresh_map(carve_enabled=True)
    scan = wall_scan()
    m.update(scan, 0.0, dt=0.05)
    size = len(m)
    assert m.carve(np.array([0.0, -1.0]), scan) == 0
    assert len(m) == size


def test_motion_compensation_can_be_switched_off_for_the_ablation():
    m = fresh_map(motion_compensation=False)
    m.update(corner_scan(), 0.0, dt=0.05)
    before = m.view(0.0).points.copy()
    m.register(corner_scan(np.array([0.05, 0.05])), dt=0.05)
    assert np.allclose(m.view(0.0).points, before)
    assert np.allclose(m.object_displacement("obj"), 0.0)
