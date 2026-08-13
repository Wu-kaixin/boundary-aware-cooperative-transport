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
            * np.array(
                [np.cos(2.0 * np.pi * index / count), np.sin(2.0 * np.pi * index / count)]
            ),
        )
        for index in range(count)
    ]


def certificate(agents: list[AgentState]) -> dict:
    return operational_enclosure_certificate(
        Cargo.circle("obj", [0.0, 0.0], 0.5),
        agents,
        contact_radius=0.45,
        strict_coverage_min=0.99,
        max_uncovered_arc_m=0.10,
        d_min=0.32,
        cage_offset=0.20,
        min_engaged_agents=6,
        engaged_radius=0.30,
        samples=720,
    )


def test_operational_enclosure_passes_complete_safe_exterior_ring():
    result = certificate(ring_agents())

    assert result["passed"] is True
    assert result["formal_caging"] is False
    assert result["strict_boundary_coverage"] == pytest.approx(1.0)
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(0.0)
    assert result["engaged_agents"] == 8
    assert all(result["checks"].values())


def test_operational_enclosure_rejects_large_uncovered_arc():
    result = certificate(ring_agents()[1:])

    assert result["passed"] is False
    assert result["checks"]["maximum_uncovered_boundary_arc"] is False
    assert result["max_uncovered_arc_upper_m"] > 0.10


def test_operational_enclosure_rejects_robot_centre_inside_object():
    agents = ring_agents()
    agents[0] = AgentState("a0", np.zeros(2))
    result = certificate(agents)

    assert result["passed"] is False
    assert result["checks"]["all_robot_centres_outside"] is False


def test_uncovered_arc_wraps_across_boundary_sample_zero():
    result = maximum_uncovered_boundary_arc(
        np.array([False, False, True, True, False]),
        perimeter=5.0,
    )

    assert result["longest_uncovered_samples"] == 3
    assert result["max_uncovered_arc_upper_m"] == pytest.approx(4.0)
