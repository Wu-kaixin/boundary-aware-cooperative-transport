"""Fail-closed scenario loading and the initial-state check."""

import copy

import numpy as np
import pytest

from dbact.contracts import ContractViolation
from dbact_sim.scenarios import (
    assert_initial_state_valid,
    build_agents,
    build_cargoes,
    goal_directions_from_config,
    load_yaml,
    validate_config,
)

PAPER = "configs/sim/v2/l_shape_v2.yaml"


def base_config() -> dict:
    return copy.deepcopy(load_yaml(PAPER))


# --------------------------------------------------------------------------- #
# required fields
# --------------------------------------------------------------------------- #


def test_paper_config_loads_and_validates():
    cfg = base_config()
    validate_config(cfg)
    assert cfg["paper"] is True


def test_missing_transport_engine_is_rejected():
    """It used to default to the scripted engine, so a scenario that had never
    simulated contact reported transport."""
    cfg = base_config()
    del cfg["transport"]["engine"]
    with pytest.raises(ContractViolation, match="transport.engine is required"):
        validate_config(cfg)


def test_missing_backend_is_rejected():
    """It used to default to 'auto', so a missing solver became a silent projection
    fallback while the write-up claimed a hard QP."""
    cfg = base_config()
    del cfg["controller"]["backend"]
    with pytest.raises(ContractViolation, match="controller.backend is required"):
        validate_config(cfg)


def test_auto_backend_is_not_a_valid_value():
    cfg = base_config()
    cfg["controller"]["backend"] = "auto"
    with pytest.raises(ContractViolation, match="controller.backend must be one of"):
        validate_config(cfg)


def test_scripted_engine_is_banned_from_paper_configs():
    cfg = base_config()
    cfg["transport"]["engine"] = "scripted"
    with pytest.raises(ContractViolation, match="restates its input"):
        validate_config(cfg)


def test_projection_backend_is_banned_from_paper_configs():
    cfg = base_config()
    cfg["controller"]["backend"] = "projection"
    with pytest.raises(ContractViolation, match="inexact filter"):
        validate_config(cfg)


def test_scripted_engine_is_allowed_outside_paper_configs():
    """The B0 baseline has to remain runnable; it is simply not a paper result."""
    cfg = load_yaml("configs/sim/baseline_scripted_b0.yaml")
    validate_config(cfg)
    assert cfg.get("paper", False) is False
    assert cfg["transport"]["engine"] == "scripted"


def test_v2_directory_is_marked_as_paper_by_its_path():
    cfg = load_yaml(PAPER)
    assert cfg.get("paper") is True


# --------------------------------------------------------------------------- #
# goal direction ownership
# --------------------------------------------------------------------------- #


def test_goal_direction_is_read_into_the_task_not_onto_the_body():
    cfg = base_config()
    goals = goal_directions_from_config(cfg)
    assert set(goals) == {"cargo_0"}
    assert np.linalg.norm(goals["cargo_0"]) == pytest.approx(1.0)
    cargo = build_cargoes(cfg)[0]
    assert not hasattr(cargo, "transport_direction")


def test_task_block_can_override_the_goal_direction():
    cfg = base_config()
    cfg["task"] = {"goal_directions": {"cargo_0": [0.0, -1.0]}}
    assert goal_directions_from_config(cfg)["cargo_0"] == pytest.approx([0.0, -1.0])


def test_random_goal_is_seeded_bounded_and_kept_inside_the_controlled_domain():
    cfg = base_config()
    del cfg["cargoes"][0]["transport_direction"]
    cfg["task"] = {
        "random_goal": {
            "enabled": True,
            "angle_min_deg": -20.0,
            "angle_max_deg": 50.0,
            "target_distance": 0.30,
            "wall_margin": 1.0,
        }
    }
    g1 = goal_directions_from_config(cfg, seed=7)["cargo_0"]
    g2 = goal_directions_from_config(cfg, seed=7)["cargo_0"]
    g3 = goal_directions_from_config(cfg, seed=8)["cargo_0"]
    assert g1 == pytest.approx(g2)
    assert not np.allclose(g1, g3)
    angle = np.degrees(np.arctan2(g1[1], g1[0]))
    assert -20.0 <= angle <= 50.0
    target = np.asarray(cfg["cargoes"][0]["center"], dtype=float) + 0.30 * g1
    assert 1.0 <= target[0] <= 7.0
    assert 1.0 <= target[1] <= 7.0


def test_random_goal_fails_when_no_direction_can_fit_the_margin():
    cfg = base_config()
    del cfg["cargoes"][0]["transport_direction"]
    cfg["task"] = {
        "random_goal": {
            "enabled": True,
            "angle_min_deg": 0.0,
            "angle_max_deg": 1.0,
            "target_distance": 20.0,
            "wall_margin": 1.0,
            "max_attempts": 4,
        }
    }
    with pytest.raises(ContractViolation, match="could not place"):
        goal_directions_from_config(cfg, seed=0)


# --------------------------------------------------------------------------- #
# layouts
# --------------------------------------------------------------------------- #


def test_scatter_layout_respects_the_minimum_separation():
    cfg = base_config()
    agents = build_agents(cfg, seed=3)
    assert len(agents) == cfg["agents"]["count"]
    pts = np.vstack([a.position for a in agents])
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    assert float(np.min(d)) >= cfg["agents"]["min_separation"] - 1e-9


def test_scatter_layout_is_reproducible_for_a_given_seed():
    cfg = base_config()
    a = np.vstack([x.position for x in build_agents(cfg, seed=5)])
    b = np.vstack([x.position for x in build_agents(cfg, seed=5)])
    assert np.allclose(a, b)
    c = np.vstack([x.position for x in build_agents(cfg, seed=6)])
    assert not np.allclose(a, c)


def test_scatter_layout_fails_loudly_when_the_annulus_is_too_small():
    cfg = base_config()
    cfg["agents"] = dict(cfg["agents"], count=200, radius_min=1.0, radius_max=1.05)
    with pytest.raises(ContractViolation, match="rather than accepting an overlapping start"):
        build_agents(cfg, seed=0)


def test_ring_layout_is_evenly_spaced():
    cfg = base_config()
    cfg["agents"] = {"count": 8, "layout": "ring", "center": [4.0, 4.0], "radius": 2.0}
    pts = np.vstack([a.position for a in build_agents(cfg, seed=0)])
    radii = np.linalg.norm(pts - np.array([4.0, 4.0]), axis=1)
    assert np.allclose(radii, 2.0)


def test_unknown_layout_is_rejected():
    cfg = base_config()
    cfg["agents"] = {"count": 4, "layout": "spiral", "center": [4.0, 4.0]}
    with pytest.raises(ContractViolation, match="unknown agents.layout"):
        build_agents(cfg, seed=0)


# --------------------------------------------------------------------------- #
# initial state
# --------------------------------------------------------------------------- #


def test_initial_state_check_passes_for_the_paper_config():
    cfg = base_config()
    assert_initial_state_valid(build_agents(cfg, seed=0), build_cargoes(cfg), 0.34, 0.16)


def test_initial_state_check_rejects_overlapping_robots():
    """A barrier certificate is about maintaining safety from a safe initial set, so
    a run that begins outside it cannot demonstrate anything about the filter."""
    from dbact.types import AgentState

    agents = [AgentState("a0", np.array([0.0, 0.0])), AgentState("a1", np.array([0.1, 0.0]))]
    with pytest.raises(ContractViolation, match="initial inter-robot distance"):
        assert_initial_state_valid(agents, [], 0.34, 0.16)


def test_initial_state_check_rejects_a_robot_starting_on_the_cargo():
    from dbact.types import AgentState

    cfg = base_config()
    cargo = build_cargoes(cfg)[0]
    agents = [AgentState("a0", cargo.position.copy())]
    with pytest.raises(ContractViolation, match="within the robot radius of cargo"):
        assert_initial_state_valid(agents, [cargo], 0.34, 0.16)
