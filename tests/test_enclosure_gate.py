"""D10-ENC - what an enclosure certificate is allowed to depend on."""

from __future__ import annotations

import numpy as np
import pytest

from dbact.enclosure_gate import (
    BINS,
    BitmapConsensus,
    GateInputs,
    bearing_bins,
    direction_bins,
    g0_current,
    g1_known,
    g2_operational,
    g3_hybrid,
    gap_degrees,
    largest_gap,
    occupancy,
)


def unit(degrees: list[float]) -> np.ndarray:
    radians = np.radians(degrees)
    return np.column_stack([np.cos(radians), np.sin(radians)])


# --------------------------------------------------------------------------- #
# the binning
# --------------------------------------------------------------------------- #


def test_direction_bins_ignore_magnitude_and_position():
    """A direction is intrinsic; that is the whole reason for preferring it."""
    one = direction_bins(unit([0.0, 90.0, 180.0, 270.0]))
    scaled = direction_bins(3.7 * unit([0.0, 90.0, 180.0, 270.0]))
    assert np.array_equal(one, scaled)
    assert np.count_nonzero(one) == 4


def test_four_orthogonal_normals_leave_a_ninety_degree_gap():
    """The L's four face directions are the interesting case: this is the best a
    fully enclosed axis-aligned object can score, so a threshold below it can
    never fire on this shape."""
    mask = direction_bins(unit([0.0, 90.0, 180.0, 270.0]))
    # One bin of slack: the bin edges fall on these exact angles, so which side of
    # an edge a direction lands on is a floating-point detail, not a measurement.
    assert 80.0 <= gap_degrees(mask) <= 100.0


def test_one_face_leaves_almost_the_whole_circle_open():
    mask = direction_bins(unit([0.0, 5.0, -5.0]))
    assert gap_degrees(mask) > 300.0


def test_largest_gap_is_cyclic():
    mask = np.array([False, True, True, True, False, False])
    assert largest_gap(mask) == 3


def test_gap_of_an_empty_set_is_the_full_circle():
    assert gap_degrees(np.zeros(BINS, dtype=bool)) == pytest.approx(360.0)
    assert occupancy(np.zeros(BINS, dtype=bool)) == 0.0


def test_bearing_bins_depend_on_the_chosen_origin():
    """The defect that disqualified the bearing family, stated as a test.

    One 0.15 m patch of noisy returns -- a robot looking at a single corner --
    covers three times as much of the bearing circle about its own centroid as it
    does about an origin five metres away. Neither number is about how much of the
    object has been seen, and a gate reading it is reading where the origin was
    put. That is why the certificate below uses directions, which have no origin.
    """
    patch = np.array([[0.0, 0.0], [0.08, 0.03], [-0.06, 0.05],
                      [0.02, -0.07], [-0.03, -0.04], [0.05, 0.06]])
    far = bearing_bins(patch, np.array([0.0, -5.0]))
    on_the_set = bearing_bins(patch, patch.mean(axis=0))
    assert occupancy(far) < 0.10
    assert occupancy(on_the_set) >= 3 * occupancy(far)


# --------------------------------------------------------------------------- #
# consensus
# --------------------------------------------------------------------------- #


def line_graph(names: list[str]) -> dict[str, list[str]]:
    return {
        name: [n for n in (names[i - 1] if i else None, names[i + 1] if i + 1 < len(names) else None) if n]
        for i, name in enumerate(names)
    }


def test_consensus_reaches_every_robot_over_a_chain():
    names = [f"a{i}" for i in range(5)]
    graph = line_graph(names)
    consensus = BitmapConsensus(bins=BINS, ttl=None)
    own = {name: np.zeros(BINS, dtype=bool) for name in names}
    own["a0"][3] = True
    for step in range(4):
        consensus.step(own, graph, float(step))
        own = {name: np.zeros(BINS, dtype=bool) for name in names}
    assert all(consensus.view(name, 4.0)[3] for name in names)


def test_consensus_takes_one_hop_per_step():
    names = [f"a{i}" for i in range(5)]
    graph = line_graph(names)
    consensus = BitmapConsensus(bins=BINS, ttl=None)
    own = {name: np.zeros(BINS, dtype=bool) for name in names}
    own["a0"][3] = True
    consensus.step(own, graph, 0.0)
    assert consensus.view("a1", 0.0)[3]
    assert not consensus.view("a2", 0.0)[3]


def test_an_observation_latches_but_an_occupancy_expires():
    graph = {"a": [], "b": []}
    monotone = BitmapConsensus(bins=BINS, ttl=None)
    expiring = BitmapConsensus(bins=BINS, ttl=2.0)
    mask = np.zeros(BINS, dtype=bool)
    mask[7] = True
    monotone.step({"a": mask, "b": mask}, graph, 0.0)
    expiring.step({"a": mask, "b": mask}, graph, 0.0)
    assert monotone.view("a", 100.0)[7]
    assert expiring.view("a", 1.0)[7]
    assert not expiring.view("a", 5.0)[7]


# --------------------------------------------------------------------------- #
# the candidates
# --------------------------------------------------------------------------- #


def inputs(**overrides) -> GateInputs:
    base = dict(
        informed=16, agents=16, best_own_coverage=0.0,
        known_normal_gap_deg=360.0, held_normal_gap_deg=360.0,
        held_normal_bins=0, contact_ready=0,
    )
    base.update(overrides)
    return GateInputs(**base)


def test_current_gate_needs_both_a_quorum_informed_and_one_good_map():
    assert g0_current(inputs(best_own_coverage=0.75))
    assert not g0_current(inputs(best_own_coverage=0.69))
    assert not g0_current(inputs(informed=3, best_own_coverage=0.99))


def test_known_gate_reads_the_team_not_one_robot():
    assert g1_known(inputs(known_normal_gap_deg=90.0), gap_max_deg=120.0)
    assert not g1_known(inputs(known_normal_gap_deg=180.0), gap_max_deg=120.0)
    assert not g1_known(inputs(informed=2, known_normal_gap_deg=0.0), gap_max_deg=120.0)


def test_operational_gate_needs_robots_on_distinct_faces():
    assert g2_operational(inputs(held_normal_bins=4, held_normal_gap_deg=80.0), quorum=4)
    # Four robots all on the same face is not an enclosure however many there are.
    assert not g2_operational(inputs(held_normal_bins=1, held_normal_gap_deg=350.0), quorum=4)
    assert not g2_operational(inputs(held_normal_bins=3, held_normal_gap_deg=80.0), quorum=4)


def test_hybrid_requires_both_halves():
    knowing = dict(known_normal_gap_deg=80.0)
    standing = dict(held_normal_bins=4, held_normal_gap_deg=80.0)
    assert g3_hybrid(inputs(**knowing, **standing))
    assert not g3_hybrid(inputs(**knowing, held_normal_bins=1, held_normal_gap_deg=350.0))
    assert not g3_hybrid(inputs(known_normal_gap_deg=270.0, **standing))


def test_no_candidate_gate_can_read_the_truth():
    """The type is the guarantee: there is no field here that comes from the
    simulator, so a gate cannot acquire one without changing this signature."""
    fields = set(GateInputs.__dataclass_fields__)
    assert not fields & {"strict_coverage", "cargo_vertices", "contact_count", "truth"}
