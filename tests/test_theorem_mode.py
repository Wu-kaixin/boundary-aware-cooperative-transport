"""Unit tests for sampled theorem_mode execution interface."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dbact.controller import DBACTController, DBACTParams
from dbact.safety_filter import SafetyFilter, SafetyFilterParams
from dbact.theorem_mode import (
    TheoremModeAbort,
    hold_segment_min_distance,
    hold_segment_object_clearance,
    omitted_neighbor_certificate,
    omitted_pair_radius,
    point_in_domain,
    wall_halfplanes,
)
from dbact.types import AgentState, ControlCommand


def test_wall_rows_keep_unclipped_update_inside_domain():
    domain = (0.0, 1.0, 0.0, 1.0)
    dt = 0.05
    p = np.array([0.01, 0.5])
    A, b = wall_halfplanes(p, domain, dt)
    # Strong leftward command would exit without wall rows.
    u_nom = np.array([-10.0, 0.0])
    filt = SafetyFilter(
        SafetyFilterParams(
            d_min=0.2,
            max_speed=0.35,
            enable_object_rows=False,
            enable_wall_rows=True,
            domain=domain,
            dt=dt,
            forbid_fallback=True,
            backend="qp",
        )
    )
    result = filt.filter_velocity(p, u_nom, [])
    nxt = p + dt * result.velocity
    assert point_in_domain(nxt, domain)
    assert result.wall_rows == 4
    assert result.feasible


def test_theorem_mode_apply_commands_aborts_instead_of_clipping():
    domain = (0.0, 8.0, 0.0, 8.0)
    params = DBACTParams(
        theorem_mode=True,
        theorem_disable_clipping=True,
        theorem_forbid_fallback=True,
        task_mode="caging",
        lead_offset=None,
        gap_gain=0.0,
        explore_gain=0.0,
        backend="qp",
        use_object_barrier=False,
        dt=0.05,
        d_min=0.28,
        robot_radius=0.13,
        delta_max=0.065,
        cage_offset=0.105,
        max_speed=0.35,
    )
    controller = DBACTController(params, domain, seed=0)
    a = params.d_min / math.sqrt(2)
    agents = [AgentState("a", [0.0, a]), AgentState("b", [a, 0.0])]
    u = np.array([-1.0, -1.0]) * params.max_speed / math.sqrt(2)
    with pytest.raises(TheoremModeAbort, match="outside_domain"):
        controller.apply_commands(
            agents,
            [ControlCommand("a", u), ControlCommand("b", np.zeros(2))],
            0.05,
        )


def test_legacy_clipping_still_exists_when_theorem_mode_off():
    domain = (0.0, 8.0, 0.0, 8.0)
    params = DBACTParams(
        theorem_mode=False,
        d_min=0.28,
        robot_radius=0.13,
        delta_max=0.065,
        cage_offset=0.105,
        max_speed=0.35,
        backend="qp",
        dt=0.05,
    )
    controller = DBACTController(params, domain, seed=0)
    a = params.d_min / math.sqrt(2)
    agents = [AgentState("a", [0.0, a]), AgentState("b", [a, 0.0])]
    u = np.array([-1.0, -1.0]) * params.max_speed / math.sqrt(2)
    controller.apply_commands(
        agents,
        [ControlCommand("a", u), ControlCommand("b", np.zeros(2))],
        0.05,
    )
    assert point_in_domain(agents[0].position, domain)
    assert float(np.linalg.norm(agents[0].position - agents[1].position)) < params.d_min


def test_hold_segment_min_distance_matches_linear_geometry():
    p1 = np.array([0.0, 0.0])
    p2 = np.array([1.0, 0.0])
    u1 = np.array([0.0, 0.0])
    u2 = np.array([-1.0, 0.0])
    assert hold_segment_min_distance(p1, p2, u1, u2, 0.5) == pytest.approx(0.5)


def test_omitted_neighbor_certificate():
    positions = np.array([[0.0, 0.0], [1.0, 0.0], [3.0, 0.0]])
    neighbors = [[1], [0], []]
    ok, omitted_min, violations = omitted_neighbor_certificate(
        positions, neighbors, d_min=0.28, u_max=0.35, dt=0.05
    )
    assert ok
    assert omitted_min == pytest.approx(2.0)
    radius = omitted_pair_radius(0.28, 0.35, 0.05)
    assert radius == pytest.approx(0.28 + 2 * 0.35 * 0.05)


def test_theorem_mode_rejects_transport_and_lead_offset():
    with pytest.raises(ValueError, match="excludes transport"):
        DBACTController(
            DBACTParams(
                theorem_mode=True,
                task_mode="transport",
                lead_offset=None,
                gap_gain=0.0,
                explore_gain=0.0,
                backend="qp",
            ),
            (0.0, 8.0, 0.0, 8.0),
            seed=0,
        )
    with pytest.raises(ValueError, match="lead_offset"):
        DBACTController(
            DBACTParams(
                theorem_mode=True,
                task_mode="caging",
                lead_offset=0.22,
                gap_gain=0.0,
                explore_gain=0.0,
                backend="qp",
            ),
            (0.0, 8.0, 0.0, 8.0),
            seed=0,
        )


def test_hold_object_clearance_lipschitz_lower_bound():
    """Lower bound is at most sampled min; Lipschitz margin is subtracted."""
    dt = 0.05
    n_samples = 21
    h = dt / (n_samples - 1)
    positions = np.array([[1.0, 0.0], [2.0, 0.0]])
    velocities = np.array([[0.0, 0.35], [0.0, -0.20]])
    # Square cargo centered at origin, side 1.0 -> clearance at (1,0) is 0.5 outside.
    square = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
    lb, sampled, margin, details = hold_segment_object_clearance(
        positions, velocities, [square], dt, n_samples=n_samples
    )
    assert lb <= sampled + 1e-12
    assert margin == pytest.approx(float(np.max(np.linalg.norm(velocities, axis=1))) * h)
    per_robot_lb = details["per_robot_lower_bound"]
    per_robot_sampled = details["per_robot_sampled_min"]
    for s, speed, robot_lb in zip(per_robot_sampled, np.linalg.norm(velocities, axis=1), per_robot_lb):
        assert robot_lb == pytest.approx(s - speed * h)
    assert lb == pytest.approx(min(per_robot_lb))


def test_forbid_fallback_returns_infeasible_not_projection():
    filt = SafetyFilter(
        SafetyFilterParams(
            d_min=0.5,
            max_speed=0.1,
            enable_object_rows=True,
            forbid_fallback=True,
            allow_object_barrier_scaling=False,
            backend="qp",
            r_safe=0.1,
            gamma_obj=8.0,
            rho=0.05,
            dt=0.05,
            object_row_inner_limit=0.16,
        )
    )
    # Two opposing close faces: no admissible retreat under a tiny speed ball.
    points = np.array([[0.0, -0.12], [0.0, 0.12]])
    normals = np.array([[0.0, 1.0], [0.0, -1.0]])
    result = filt.filter_velocity(
        np.zeros(2),
        np.array([0.0, 0.0]),
        [np.array([0.4, 0.0])],
        boundary_points=points,
        boundary_normals=normals,
    )
    assert result.status in {"infeasible", "scaled_barrier", "relaxed_margin", "optimal"}
    if not result.feasible:
        assert result.status == "infeasible"
