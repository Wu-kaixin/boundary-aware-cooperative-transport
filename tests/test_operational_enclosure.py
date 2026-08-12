"""What "the team has enclosed the object" is allowed to mean, as an audit.

Ported from the CODEX branch's ``tests/test_operational_enclosure.py``. The two
functions under test are truth-reading audits in :mod:`dbact.metrics`, not gates:
:mod:`dbact.enclosure_gate` is what the running system uses, and it is deliberately
origin-free and truth-free. The distinction matters for reading these tests -- a
failure here means the *measurement* is wrong, not that a run failed.

The gap this closes is that mean coverage cannot separate the two configurations
that matter. Fifteen of sixteen robots evenly spread and fifteen piled on one side
report nearly the same :func:`strict_boundary_coverage`, and only the second has an
arc wide enough for the object to leave through.
"""

from __future__ import annotations

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.metrics import maximum_uncovered_boundary_arc, operational_enclosure_certificate
from dbact.types import AgentState


def ring_agents(count: int = 8, radius: float = 0.75) -> list[AgentState]:
    return [
        AgentState(
            f"a{index}",
            radius
            * np.array([np.cos(2.0 * np.pi * index / count), np.sin(2.0 * np.pi * index / count)]),
        )
        for index in range(count)
    ]


def certificate(agents: list[AgentState], **overrides) -> dict:
    kwargs = dict(
        contact_radius=0.45,
        strict_coverage_min=0.99,
        max_uncovered_arc_m=0.10,
        d_min=0.32,
        cage_offset=0.20,
        min_engaged_agents=6,
        engaged_radius=0.30,
        samples=720,
    )
    kwargs.update(overrides)
    return operational_enclosure_certificate(Cargo.circle("obj", [0.0, 0.0], 0.5), agents, **kwargs)


# --------------------------------------------------------------------------- #
# the certificate
# --------------------------------------------------------------------------- #


def test_operational_enclosure_passes_complete_safe_exterior_ring():
    result = certificate(ring_agents())

    assert result["passed"] is True
    assert result["formal_caging"] is False
    assert result["strict_boundary_coverage"] == pytest.approx(1.0)
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(0.0)
    assert result["engaged_agents"] == 8
    assert all(result["checks"].values())
    assert result["failure_reasons"] == []


def test_operational_enclosure_rejects_large_uncovered_arc():
    result = certificate(ring_agents()[1:])

    assert result["passed"] is False
    assert result["checks"]["maximum_uncovered_boundary_arc"] is False
    assert result["max_uncovered_arc_upper_m"] > 0.10
    assert "maximum_uncovered_boundary_arc" in result["failure_reasons"]


def test_operational_enclosure_rejects_robot_centre_inside_object():
    """A robot that has passed through the boundary must not count as covering it.

    This is the failure the legacy mean-coverage metric actively rewarded: a centre
    inside the object is close to samples on both sides at once, so including it
    would close the very arc whose width is being measured.
    """
    agents = ring_agents()
    agents[0] = AgentState("a0", np.zeros(2))
    result = certificate(agents)

    assert result["passed"] is False
    assert result["checks"]["all_robot_centres_outside"] is False
    assert result["min_signed_clearance_m"] < 0.0
    # The inside robot is excluded from the covering set, so the arc it used to
    # "cover" is now correctly reported as open.
    assert result["checks"]["maximum_uncovered_boundary_arc"] is False


def test_operational_enclosure_rejects_inter_agent_violation():
    """Separation is part of the certificate, not a separate concern.

    A configuration that covers the boundary by standing two robots on top of each
    other is not one the barrier filter can maintain.
    """
    agents = ring_agents()
    agents[1] = AgentState("a1", agents[0].position + np.array([0.05, 0.0]))
    result = certificate(agents)

    assert result["passed"] is False
    assert result["checks"]["inter_agent_safety"] is False
    assert result["min_inter_agent_distance_m"] < 0.32


def test_operational_enclosure_rejects_a_thin_quorum_that_still_covers():
    """Coverage and quorum are different premises and must fail independently.

    Widening ``contact_radius`` lets four robots cover the whole boundary while the
    engaged count is below the pushing quorum. If the quorum check were implied by
    coverage there would be no reason to state it.
    """
    result = certificate(ring_agents(count=4), contact_radius=0.95, min_engaged_agents=6)

    assert result["checks"]["strict_boundary_coverage"] is True
    assert result["checks"]["engaged_quorum"] is False
    assert result["engaged_agents"] < 6
    assert result["passed"] is False


def test_operational_enclosure_never_claims_formal_caging():
    """Constant ``False``, including on the configuration that passes everything.

    An exterior ring with complete coverage is the best case this audit can see,
    and it is still not a configuration-space escape proof. The non-claim is carried
    in the payload so a reader of the JSON cannot miss it.
    """
    good = certificate(ring_agents())
    bad = certificate([])
    assert good["passed"] is True
    assert good["formal_caging"] is False
    assert bad["formal_caging"] is False
    assert "escape proof" in good["formal_caging_nonclaim"]


def test_operational_enclosure_fails_closed_with_no_agents():
    """An empty team is total failure, not a vacuous pass."""
    result = certificate([])
    assert result["passed"] is False
    assert result["strict_boundary_coverage"] == 0.0
    assert result["engaged_agents"] == 0
    assert result["min_signed_clearance_m"] == float("inf")
    assert result["checks"]["maximum_uncovered_boundary_arc"] is False


def test_convex_object_has_no_facing_cage_constraint():
    """A circle has no concavity, so the offset-curve premise is vacuously satisfied.

    Reported as ``None`` rather than as a number, because "there is no facing edge
    pair" and "the facing edge pair is 0 m apart" are opposite findings.
    """
    result = certificate(ring_agents())
    assert result["facing_cage_clearance_m"] is None
    assert result["checks"]["cage_offset_feasible"] is True


# --------------------------------------------------------------------------- #
# the arc bound
# --------------------------------------------------------------------------- #


def test_uncovered_arc_wraps_across_boundary_sample_zero():
    """Cyclic, because the boundary is closed.

    An open arc straddling sample 0 is one arc. Counting it as two understates
    exactly the piled-on-one-side configuration this measure exists to catch.
    """
    result = maximum_uncovered_boundary_arc(np.array([False, False, True, True, False]), perimeter=5.0)

    assert result["longest_uncovered_samples"] == 3
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(4.0)


def test_uncovered_arc_adds_one_sampling_interval():
    """``(longest + 1) * resolution``, which is the conservative direction.

    The true covered/uncovered transition lies somewhere inside each of the two
    bounding sampling intervals, so the continuous arc can exceed
    ``longest * resolution`` by up to two half intervals. Reporting the exact
    sampled product would be optimistic by precisely the quantity being bounded.
    """
    mask = np.array([True, False, False, True, True, True, True, True])
    result = maximum_uncovered_boundary_arc(mask, perimeter=8.0)
    assert result["sample_resolution_m"] == pytest.approx(1.0)
    assert result["longest_uncovered_samples"] == 2
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(3.0)


def test_uncovered_arc_is_clamped_to_the_perimeter():
    """The ``+1`` term must never push the bound past the whole boundary."""
    mask = np.array([True, False, False, False])
    result = maximum_uncovered_boundary_arc(mask, perimeter=4.0)
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(4.0)


def test_uncovered_arc_extremes():
    full = maximum_uncovered_boundary_arc(np.ones(16, dtype=bool), perimeter=4.0)
    assert full["longest_uncovered_samples"] == 0
    assert full["max_uncovered_arc_upper_m"] == 0.0

    none = maximum_uncovered_boundary_arc(np.zeros(16, dtype=bool), perimeter=4.0)
    assert none["longest_uncovered_samples"] == 16
    assert none["max_uncovered_arc_upper_m"] == pytest.approx(4.0)

    empty = maximum_uncovered_boundary_arc(np.empty(0, dtype=bool), perimeter=4.0)
    assert empty["max_uncovered_arc_upper_m"] == float("inf")


def test_equal_mean_coverage_can_hide_very_different_worst_arcs():
    """The reason this measure was added, stated as a test.

    Both masks below leave a quarter of the boundary uncovered, so
    :func:`strict_boundary_coverage` scores them identically at 0.75. One leaves a
    single contiguous quarter -- an opening the object can leave through -- and the
    other leaves four scattered slivers, which no escape uses. Mean coverage cannot
    tell them apart by construction; the arc bound differs by a factor of four.
    """
    one_gap = np.array([False] * 4 + [True] * 12)
    slivers = np.array([False, True, True, True] * 4)
    assert one_gap.mean() == slivers.mean() == 0.75

    wide = maximum_uncovered_boundary_arc(one_gap, perimeter=16.0)
    narrow = maximum_uncovered_boundary_arc(slivers, perimeter=16.0)
    assert wide["max_uncovered_arc_upper_m"] == pytest.approx(5.0)
    assert narrow["max_uncovered_arc_upper_m"] == pytest.approx(2.0)


def test_a_piled_team_is_rejected_for_its_arc_not_its_average():
    """The same ten robots pass when spread and fail when piled on one side.

    Ten robots on an exterior ring cover the whole boundary. Crowding the same ten
    into a 198 degree span leaves one opening of about 90 degrees, and it is the arc
    check that fires -- the configuration a fixed-quorum-and-average criterion would
    have accepted.
    """
    cargo = Cargo.circle("obj", [0.0, 0.0], 0.5)
    spread = ring_agents(count=10, radius=0.75)
    piled = [
        AgentState(f"p{i}", 0.75 * np.array([np.cos(a), np.sin(a)]))
        for i, a in enumerate(np.linspace(-0.55 * np.pi, 0.55 * np.pi, 10))
    ]
    kwargs = dict(
        contact_radius=0.45,
        strict_coverage_min=0.0,
        max_uncovered_arc_m=0.10,
        d_min=0.0,
        cage_offset=0.20,
        min_engaged_agents=0,
        engaged_radius=0.30,
        samples=720,
    )
    a = operational_enclosure_certificate(cargo, spread, **kwargs)
    b = operational_enclosure_certificate(cargo, piled, **kwargs)

    assert a["max_uncovered_arc_upper_m"] == pytest.approx(0.0)
    assert a["passed"] is True
    assert b["passed"] is False
    assert b["failure_reasons"] == ["maximum_uncovered_boundary_arc"]
    # One opening, not scattered slivers: the whole deficit is in the worst arc.
    deficit = (1.0 - b["strict_boundary_coverage"]) * cargo.perimeter
    assert b["max_uncovered_arc_upper_m"] >= deficit
