"""End-to-end smoke tests, including the invariants a short run can already check."""

import json

import numpy as np
import pytest

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml

PAPER = "configs/sim/v2/l_shape_v2.yaml"


def test_migrated_legacy_scenario_still_runs():
    cfg = load_yaml("configs/sim/circle.yaml")
    env = SimulationEnvironment(cfg)
    env.run(steps=5)
    assert len(env.agents) == cfg["agents"]["count"]
    assert env.log.times[-1] > 0.0


def test_paper_scenario_runs_and_records_every_invariant():
    env = SimulationEnvironment(load_yaml(PAPER), seed=0)
    env.run(steps=20)
    summary = env.summary()

    # Safety is recorded every step, not sampled at the end: a final-frame snapshot
    # cannot see a robot that passed through the cargo and came back out.
    assert len(env.log.min_clearance["cargo_0"]) == len(env.log.times)
    assert len(env.log.max_penetration["cargo_0"]) == len(env.log.times)
    assert len(env.log.contact_counts["cargo_0"]) == len(env.log.times)

    entry = summary["cargoes"]["cargo_0"]
    assert entry["max_agents_inside"] == 0
    assert entry["min_signed_clearance"] > 0.0
    assert summary["min_inter_agent_distance"] >= env.controller.params.d_min - 1e-6


def test_summary_is_json_serialisable_and_carries_provenance():
    env = SimulationEnvironment(load_yaml(PAPER), seed=3)
    env.run(steps=5)
    summary = env.summary()
    json.dumps(summary)  # must not raise

    provenance = summary["provenance"]
    assert set(provenance) == {"git_sha", "config_hash", "seed", "backend"}
    assert provenance["seed"] == 3
    assert provenance["backend"] == "qp"
    assert summary["engine"] == "penalty"


def test_config_hash_distinguishes_configurations():
    from dbact.provenance import config_hash

    base = load_yaml(PAPER)
    changed = load_yaml(PAPER)
    changed["controller"]["cage_offset"] = 0.14
    assert config_hash(base) != config_hash(changed)


def test_the_same_seed_reproduces_the_same_trajectory():
    """Frame randomness goes through BLAKE2, so a run is reproducible across
    processes; Python's built-in hash is salted and would not be."""
    a = SimulationEnvironment(load_yaml(PAPER), seed=11)
    a.run(steps=15)
    b = SimulationEnvironment(load_yaml(PAPER), seed=11)
    b.run(steps=15)
    for agent_id, history in a.log.agent_positions.items():
        assert np.allclose(np.vstack(history), np.vstack(b.log.agent_positions[agent_id]))
    assert a.cargoes[0].position == pytest.approx(b.cargoes[0].position)


def test_different_seeds_give_different_runs():
    a = SimulationEnvironment(load_yaml(PAPER), seed=1)
    a.run(steps=15)
    b = SimulationEnvironment(load_yaml(PAPER), seed=2)
    b.run(steps=15)
    assert not np.allclose(
        np.vstack([agent.position for agent in a.agents]),
        np.vstack([agent.position for agent in b.agents]),
    )


def test_enclosure_scenario_does_not_evaluate_transport_success():
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_enclosure_v2.yaml"), seed=0)
    env.run(steps=10)
    entry = env.summary()["cargoes"]["cargo_0"]
    assert entry["success"] is None
    assert "no goal direction" in entry["failure_reasons"][0]


def test_distance_field_ablation_runs_from_its_own_config():
    env = SimulationEnvironment(load_yaml("configs/sim/v2/l_shape_distance_field_v2.yaml"), seed=0)
    env.run(steps=10)
    assert env.summary()["density_mode"] == "distance_field"


def test_b0_baseline_runs_and_is_rejected_by_the_validator():
    """It is run in order to be rejected, and the rejection is the result."""
    import importlib.util
    import sys
    from pathlib import Path

    spec = importlib.util.spec_from_file_location(
        "validate_run_mod", Path(__file__).resolve().parents[1] / "scripts" / "validate_run.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["validate_run_mod"] = module
    spec.loader.exec_module(module)

    env = SimulationEnvironment(load_yaml("configs/sim/baseline_scripted_b0.yaml"), seed=0)
    env.run(steps=10)
    reasons = module.validate(env.summary())
    assert any("scripted" in r for r in reasons)
    assert any("projection" in r for r in reasons)


def test_frame_callback_receives_step_then_environment():
    env = SimulationEnvironment(load_yaml(PAPER), seed=0)
    seen: list[tuple[int, int]] = []
    env.run(steps=2, on_frame=lambda i, e: seen.append((i, len(e.log.times))))
    assert seen == [(0, 1), (1, 2), (2, 3)]
