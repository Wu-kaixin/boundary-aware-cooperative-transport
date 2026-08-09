"""D10-DIAG - the segmentation has to be a function of state, not of taste."""

from __future__ import annotations

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.diagnosis import (
    SEGMENTS,
    FrameRecord,
    SegmentRules,
    arc_of_run,
    backside_samples,
    classify_frame,
    longest_false_run,
    observed_boundary_mask,
    occupied_boundary_mask,
    segment,
)


PERIMETER = 8.0
RULES = SegmentRules(quorum=4, arrival_radius=0.8, unobserved_arc_fraction=0.20)


def record(**kwargs) -> FrameRecord:
    base = dict(frame=0, phase=2, agents=16)
    base.update(kwargs)
    return FrameRecord(**base)


# --------------------------------------------------------------------------- #
# the cyclic run, which is the part a straightforward scan gets wrong
# --------------------------------------------------------------------------- #


def test_longest_false_run_wraps_around_the_seam():
    mask = np.array([False, True, True, True, False, False])
    # Three unobserved samples straddle index 0; a non-cyclic scan reports two.
    assert longest_false_run(mask) == 3


def test_longest_false_run_saturates_at_the_full_circle():
    assert longest_false_run(np.zeros(10, dtype=bool)) == 10
    assert longest_false_run(np.ones(10, dtype=bool)) == 0


def test_arc_of_run_is_in_metres():
    assert arc_of_run(40, 8.0, 160) == pytest.approx(2.0)


# --------------------------------------------------------------------------- #
# coverage measured on the true boundary
# --------------------------------------------------------------------------- #


def square() -> Cargo:
    vertices = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    return Cargo(object_id="c", vertices=vertices)


def test_observed_mask_is_empty_without_map_points():
    mask = observed_boundary_mask(square(), np.empty((0, 2)), tolerance=0.09)
    assert not mask.any()


def test_observing_one_face_leaves_the_rest_unobserved():
    cargo = square()
    face = np.column_stack([np.linspace(-1.0, 1.0, 60), np.full(60, -1.0)])
    mask = observed_boundary_mask(cargo, face, tolerance=0.05)
    assert 0.20 <= mask.mean() <= 0.30  # one of four faces
    # The unseen three quarters are contiguous, so the gap is most of the perimeter.
    assert longest_false_run(mask) > 0.6 * len(mask)


def test_occupied_mask_follows_the_robots_not_the_map():
    cargo = square()
    positions = np.array([[0.0, -1.3]])
    mask = occupied_boundary_mask(cargo, positions, contact_radius=0.42)
    assert mask.any()
    assert mask.mean() < 0.25


def test_backside_is_the_half_away_from_the_observer():
    cargo = square()
    mask = backside_samples(cargo, np.array([0.0, -5.0]))
    boundary, _ = cargo.boundary_samples(160)
    # Every far-side sample is on the +y side of the centre and no near-side one is.
    assert np.all(boundary[mask][:, 1] > cargo.position[1] - 1e-9)
    assert 0.4 <= mask.mean() <= 0.6


# --------------------------------------------------------------------------- #
# the cascade
# --------------------------------------------------------------------------- #


def test_news_not_yet_spread_is_token_recall():
    assert classify_frame(record(informed=3, arrived=0), RULES, PERIMETER) == "A"


def test_team_informed_but_nobody_there_is_first_arrival():
    assert classify_frame(record(informed=16, arrived=0), RULES, PERIMETER) == "B"


def test_some_arrived_but_no_quorum_is_local_mapping():
    assert classify_frame(record(informed=16, arrived=3), RULES, PERIMETER) == "C"


def test_quorum_with_the_far_side_unmapped_is_backside_discovery():
    frame = record(informed=16, arrived=8, largest_unobserved_arc=0.5 * PERIMETER)
    assert classify_frame(frame, RULES, PERIMETER) == "E"


def test_quorum_with_the_boundary_mapped_is_enclosure_convergence():
    frame = record(informed=16, arrived=8, largest_unobserved_arc=0.05 * PERIMETER)
    assert classify_frame(frame, RULES, PERIMETER) == "F"


def test_an_active_redeploy_takes_precedence_over_the_arc():
    frame = record(informed=16, arrived=8, redeploy_active=2,
                   largest_unobserved_arc=0.5 * PERIMETER)
    assert classify_frame(frame, RULES, PERIMETER) == "D"


def test_the_contact_quorum_wins_over_everything_below_it():
    frame = record(informed=16, arrived=8, contact_ready=4, redeploy_active=5,
                   largest_unobserved_arc=0.9 * PERIMETER)
    assert classify_frame(frame, RULES, PERIMETER) == "G"


def test_labels_partition_the_interval():
    records = [
        record(frame=k, informed=16, arrived=8, largest_unobserved_arc=0.5 * PERIMETER)
        for k in range(10, 30)
    ]
    counts = segment(records, RULES, PERIMETER, start=10, end=30)
    assert sum(counts.values()) == 20
    assert counts["E"] == 20
    assert set(counts) == {key for key, _ in SEGMENTS}


def test_frames_outside_the_window_are_not_counted():
    records = [record(frame=k, informed=16, arrived=8) for k in range(0, 40)]
    counts = segment(records, RULES, PERIMETER, start=10, end=20)
    assert sum(counts.values()) == 10
