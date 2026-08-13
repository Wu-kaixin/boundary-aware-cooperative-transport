"""The voxel map: repetition and relay must not amplify density mass."""

import numpy as np
import pytest

from dbact.boundary_map import LocalBoundaryMap
from dbact.types import BoundaryObservation


def wall_observations(agent_id: str = "a0", timestamp: float = 0.0, count: int = 21, spacing: float = 0.06):
    xs = (np.arange(count) - count // 2) * spacing
    return [
        BoundaryObservation(
            object_id="obj",
            agent_id=agent_id,
            point=np.array([x, 0.0]),
            normal=np.array([0.0, 1.0]),
            timestamp=timestamp,
            confidence=0.8,
            arc_length=spacing,
        )
        for x in xs
    ]


def test_one_update_creates_one_cell_per_spatial_unit():
    observations = wall_observations()
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    m.update(observations, 0.0)
    assert len(m) == len(observations)


def test_repeating_the_same_packet_adds_no_cells_and_no_mass():
    """The pre-refactor map appended every packet, so a point observed repeatedly
    contributed repeatedly and the coverage law was pulled towards whichever piece
    of boundary was talked about most."""
    observations = wall_observations()
    once = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    once.update(observations, 0.0)
    many = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    for _ in range(50):
        many.update(observations, 0.0)
    assert len(many) == len(once)
    assert many.total_arc_length() == pytest.approx(once.total_arc_length())


def test_relay_from_many_neighbours_adds_no_mass():
    observations = wall_observations()
    single = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    single.update(observations, 0.0)

    relayed = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    batch = []
    for agent in ("a0", "a1", "a2", "a3"):
        batch.extend(wall_observations(agent_id=agent))
    relayed.update(batch, 0.0)

    assert len(relayed) == len(single)
    assert relayed.total_arc_length() == pytest.approx(single.total_arc_length())


def test_the_same_packet_arriving_several_times_in_one_update_counts_once():
    """Regression: a packet relayed over several paths arrives as several identical
    copies inside a single update. Summing arc length over them inflated each cell up
    to the diagonal cap -- total arc length went from 0.939 to 1.273 on an 8x relay,
    which is exactly the amplification the voxel map exists to prevent."""
    observations = wall_observations()
    once = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    once.update(observations, 0.0)

    duplicated = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    duplicated.update(observations * 8, 0.0)

    assert len(duplicated) == len(once)
    assert duplicated.total_arc_length() == pytest.approx(once.total_arc_length(), rel=1e-12)


def test_two_distinct_returns_in_the_same_cell_still_both_count():
    """The deduplication must key on the source observation, not on the cell, or a
    genuinely denser scan would stop contributing."""
    m = LocalBoundaryMap(voxel_size=0.20, age_decay=0.0)
    a = BoundaryObservation("obj", "a0", np.array([0.00, 0.0]), np.array([0.0, 1.0]), 0.0, 0.9, arc_length=0.02)
    b = BoundaryObservation("obj", "a0", np.array([0.03, 0.0]), np.array([0.0, 1.0]), 0.0, 0.9, arc_length=0.02)
    m.update([a, b], 0.0)
    assert len(m) == 1
    assert m.total_arc_length() == pytest.approx(0.04)


def test_arc_length_per_cell_is_capped_by_the_cell_diagonal():
    """A cell of side v cannot represent more than v*sqrt(2) of boundary however
    many robots report it, which is what keeps total mass proportional to
    perimeter rather than to scan rate."""
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    dense = [
        BoundaryObservation("obj", "a0", np.array([0.001 * k, 0.0]), np.array([0.0, 1.0]), 0.0, 0.9, arc_length=0.05)
        for k in range(20)
    ]
    m.update(dense, 0.0)
    assert len(m) == 1
    assert m.total_arc_length() <= m.voxel_diagonal + 1e-12


def test_total_arc_length_tracks_the_scanned_boundary_length():
    observations = wall_observations(count=21, spacing=0.06)
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    m.update(observations, 0.0)
    expected = sum(o.arc_length for o in observations)
    assert m.total_arc_length() == pytest.approx(expected, rel=0.35)


def test_confidence_is_fused_by_maximum_not_by_sum():
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    low = BoundaryObservation("obj", "a0", np.zeros(2), np.array([0.0, 1.0]), 0.0, confidence=0.3, arc_length=0.06)
    high = BoundaryObservation("obj", "a1", np.zeros(2), np.array([0.0, 1.0]), 0.0, confidence=0.9, arc_length=0.06)
    m.update([low, high], 0.0)
    assert m.all_observations(0.0)[0].confidence == pytest.approx(0.9)


def test_position_and_normal_are_confidence_weighted_averages():
    m = LocalBoundaryMap(voxel_size=0.20, age_decay=0.0)
    a = BoundaryObservation("obj", "a0", np.array([0.00, 0.0]), np.array([0.0, 1.0]), 0.0, confidence=0.9, arc_length=0.05)
    b = BoundaryObservation("obj", "a1", np.array([0.04, 0.0]), np.array([0.0, 1.0]), 0.0, confidence=0.1, arc_length=0.05)
    m.update([a, b], 0.0)
    fused = m.all_observations(0.0)[0]
    assert 0.0 < fused.point[0] < 0.02  # pulled towards the confident observation
    assert np.linalg.norm(fused.normal) == pytest.approx(1.0)


def test_age_decay_fades_confidence_at_read_time():
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.5, min_weight=1e-9)
    m.update(wall_observations(), 0.0)
    fresh = np.mean([o.confidence for o in m.all_observations(0.0)])
    later = np.mean([o.confidence for o in m.all_observations(4.0)])
    assert later < fresh
    assert later == pytest.approx(fresh * np.exp(-0.5 * 4.0), rel=1e-6)


def test_faded_cells_are_eventually_dropped():
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=1.0, min_weight=1e-3)
    m.update(wall_observations(), 0.0)
    assert len(m) > 0
    m.prune(30.0)
    assert len(m) == 0


def test_capacity_applies_to_cells_not_packets():
    """Capping raw packets meant the buffer filled in a fraction of a second at
    realistic scan rates and started discarding fresh geometry."""
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0, max_voxels_per_object=5)
    m.update(wall_observations(count=41), 0.0)
    assert len(m) == 5
    for _ in range(20):
        m.update(wall_observations(count=41), 0.0)
    assert len(m) == 5


def test_object_ids_are_tracked_separately():
    m = LocalBoundaryMap(voxel_size=0.06, age_decay=0.0)
    first = wall_observations()
    second = [
        BoundaryObservation("other", "a0", o.point + np.array([5.0, 0.0]), o.normal, 0.0, 0.8, arc_length=0.06)
        for o in first
    ]
    m.update(first + second, 0.0)
    assert m.object_ids() == ["obj", "other"]
    assert m.total_arc_length("obj") == pytest.approx(m.total_arc_length("other"))


def test_point_to_plane_motion_compensation_moves_the_map_once_per_scan():
    """A translating cargo must not leave a world-frame density trail behind it."""
    m = LocalBoundaryMap(
        voxel_size=0.04,
        age_decay=0.0,
        motion_match_radius=0.20,
        motion_min_matches=5,
        max_translation_per_update=0.08,
    )
    horizontal = wall_observations(timestamp=0.0, count=11, spacing=0.08)
    vertical = [
        BoundaryObservation(
            "obj",
            "a1",
            np.array([0.0, (k - 5) * 0.08]),
            np.array([1.0, 0.0]),
            0.0,
            0.9,
            arc_length=0.08,
        )
        for k in range(11)
    ]
    m.update(horizontal + vertical, 0.0)
    before = np.mean(np.vstack([o.point for o in m.all_observations(0.0)]), axis=0)

    shift = np.array([0.03, -0.02])
    moved = [
        BoundaryObservation(
            o.object_id,
            o.agent_id,
            o.point + shift,
            o.normal,
            1.0,
            o.confidence,
            arc_length=o.arc_length,
        )
        for o in horizontal + vertical
    ]
    m.update(moved, 1.0)
    after = np.mean(np.vstack([o.point for o in m.all_observations(1.0)]), axis=0)
    assert m.last_motion["obj"] == pytest.approx(shift, abs=2e-3)
    assert after - before == pytest.approx(shift, abs=1e-2)

    # Same-frame relay is a duplicate, not a second body motion.
    m.update(moved * 4, 1.0)
    assert np.mean(np.vstack([o.point for o in m.all_observations(1.0)]), axis=0) == pytest.approx(after)


def test_se2_registration_separates_translation_from_rotation_for_progress():
    m = LocalBoundaryMap(
        voxel_size=0.025,
        age_decay=0.0,
        motion_match_radius=0.20,
        motion_min_matches=5,
        max_translation_per_update=0.08,
        max_rotation_per_update=0.12,
    )
    horizontal = wall_observations(timestamp=0.0, count=17, spacing=0.05)
    vertical = [
        BoundaryObservation(
            "obj",
            "a1",
            np.array([0.0, (k - 8) * 0.05]),
            np.array([1.0, 0.0]),
            0.0,
            0.9,
            arc_length=0.05,
        )
        for k in range(17)
    ]
    original = horizontal + vertical
    m.update(original, 0.0)

    theta = 0.035
    shift = np.array([0.025, -0.015])
    c, s = np.cos(theta), np.sin(theta)
    rotation = np.array([[c, -s], [s, c]])
    moved = [
        BoundaryObservation(
            o.object_id,
            o.agent_id,
            o.point @ rotation.T + shift,
            o.normal @ rotation.T,
            1.0,
            o.confidence,
            arc_length=o.arc_length,
        )
        for o in original
    ]
    m.update(moved, 1.0)

    assert m.last_motion["obj"] == pytest.approx(shift, abs=4e-3)
    assert m.last_rotation["obj"] == pytest.approx(theta, abs=5e-3)
