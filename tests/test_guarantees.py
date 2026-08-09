"""Executable checks for the conditional arbitrary-shape theorem boundary."""

from __future__ import annotations

import copy

import numpy as np
import pytest

from dbact.geometry import certified_inscribed_radius, is_simple_polygon
from dbact.guarantees import boundary_map_gap_upper_bound, guaranteed_detection_radius
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import build_agents, build_cargoes, load_yaml


CONFIG = "configs/sim/v3/arbitrary_shape_full_workspace_500.yaml"


def test_simple_polygon_predicate_rejects_a_bow_tie():
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    bow_tie = np.array([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
    assert is_simple_polygon(square)
    assert not is_simple_polygon(bow_tie)


def test_inscribed_radius_is_a_constructive_lower_bound():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    # Ear clipping witnesses a triangle incircle. It is conservative relative to
    # the square's true inradius, but strictly contained and therefore provable.
    assert 0.5 < certified_inscribed_radius(square) <= 1.0


def test_finite_ray_detection_radius_decreases_when_more_returns_are_required():
    one = guaranteed_detection_radius(1.2, 0.1, 96, required_returns=1)
    three = guaranteed_detection_radius(1.2, 0.1, 96, required_returns=3)
    assert 0.0 < three < one <= 1.2


def test_boundary_map_gap_adds_a_continuous_sampling_upper_bound():
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    map_points = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    witness = boundary_map_gap_upper_bound(square, map_points, sample_count=100)
    assert witness["sampling_resolution_bound"] == pytest.approx(4.0 / 200.0)
    assert witness["max_boundary_gap"] == pytest.approx(
        witness["sampled_max_boundary_gap"] + witness["sampling_resolution_bound"]
    )


def test_seeded_random_simple_polygons_are_reproducible_and_not_whitelisted():
    cfg = load_yaml(CONFIG)
    agents0 = build_agents(cfg, seed=0)
    first = build_cargoes(cfg, seed=0, agents=agents0)[0]
    replay = build_cargoes(cfg, seed=0, agents=agents0)[0]
    other = build_cargoes(cfg, seed=1, agents=build_agents(cfg, seed=1))[0]
    assert first.vertices == pytest.approx(replay.vertices)
    assert not np.allclose(first.local_vertices, other.local_vertices)
    assert is_simple_polygon(first.vertices)
    assert is_simple_polygon(other.vertices)


def test_paired_sweep_layout_builds_two_lane_chains():
    cfg = load_yaml(CONFIG)
    agents = build_agents(cfg, seed=0)
    points = np.vstack([agent.position for agent in agents])
    half = len(agents) // 2
    assert np.allclose(points[:half, 0], 0.20)
    assert np.allclose(points[half:, 0], 7.80)
    assert np.allclose(points[:half, 1], points[half:, 1])
    assert np.diff(points[:half, 1]) == pytest.approx(np.full(half - 1, 8.0 / half))


def test_reference_full_workspace_instance_has_a_complete_certificate():
    env = SimulationEnvironment(load_yaml(CONFIG), seed=0)
    cert = env.guarantee_certificates["cargo_0"]
    assert cert["eligible"] is True
    assert cert["search"]["sweep_bound_frames"] <= 204
    assert cert["search"]["release_bound_frames"] == 268
    assert all(check["passed"] for check in cert["checks"].values())


def test_too_thin_shape_is_simulatable_but_not_theorem_eligible():
    cfg = load_yaml(CONFIG)
    cfg = copy.deepcopy(cfg)
    cfg["cargoes"][0] = {
        "id": "cargo_0",
        "shape": "rectangle",
        "center": [4.0, 4.0],
        "width": 1.0,
        "height": 0.05,
        "surface_density": 1.0,
    }
    cfg["task"]["random_goal"] = {"enabled": False, "target_distance": 0.08}
    cfg["task"]["goal_directions"] = {"cargo_0": [1.0, 0.0]}
    env = SimulationEnvironment(cfg, seed=0)
    cert = env.guarantee_certificates["cargo_0"]
    assert cert["eligible"] is False
    assert "feature_witness" in cert["failure_reasons"]


def test_narrow_u_shape_is_rejected_by_cage_self_clearance_certificate():
    cfg = copy.deepcopy(load_yaml(CONFIG))
    cfg["cargoes"][0] = {
        "id": "cargo_0",
        "shape": "polygon",
        "vertices_frame": "local",
        "center": [4.0, 4.0],
        "surface_density": 1.5,
        "random_center": {"enabled": False},
        "vertices": [
            [-0.60, -0.48],
            [0.60, -0.48],
            [0.60, 0.48],
            [0.18, 0.48],
            [0.18, -0.08],
            [-0.18, -0.08],
            [-0.18, 0.48],
            [-0.60, 0.48],
        ],
    }
    cfg["task"]["random_goal"] = {"enabled": False, "target_distance": 0.05}
    cfg["task"]["goal_directions"] = {"cargo_0": [1.0, 0.0]}

    env = SimulationEnvironment(cfg, seed=0)
    cert = env.guarantee_certificates["cargo_0"]
    assert cert["eligible"] is False
    assert "cage_offset_self_clearance" in cert["failure_reasons"]
    assert cert["checks"]["cage_offset_self_clearance"]["value"] < cfg["controller"]["d_min"]
