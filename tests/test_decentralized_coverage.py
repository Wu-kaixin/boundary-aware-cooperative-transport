import numpy as np
import pytest

from dbact.controller import DBACTController, DBACTParams
from dbact.types import AgentState
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def coverage_params(**overrides) -> DBACTParams:
    kwargs = dict(
        task_mode="coverage",
        comm_range=2.0,
        local_radius=1.0,
        grid_resolution=21,
        target_center=[2.0, 2.0],
        target_radius=0.6,
        target_sensor_range=3.0,
        target_samples=12,
        backend="qp",
        d_min=0.34,
        robot_radius=0.16,
    )
    kwargs.update(overrides)
    return DBACTParams(**kwargs)


def test_coverage_mode_generates_local_cvt_commands():
    controller = DBACTController(coverage_params(), (0.0, 4.0, 0.0, 4.0))
    agents = [
        AgentState("a0", np.array([1.5, 1.0])),
        AgentState("a1", np.array([2.0, 1.0])),
        AgentState("a2", np.array([2.5, 1.0])),
    ]

    commands = controller.step(agents, [], timestamp=0.0, dt=0.05)

    assert {command.mode for command in commands} == {"region_coverage"}
    assert any(np.linalg.norm(command.velocity) > 1e-6 for command in commands)
    assert all(np.linalg.norm(command.velocity) <= coverage_params().max_speed + 1e-9 for command in commands)


def test_coverage_mode_does_not_assert_the_contact_contract():
    """There is no object in region-coverage mode, so C1 has nothing to constrain."""
    controller = DBACTController(coverage_params(cage_offset=0.9), (0.0, 4.0, 0.0, 4.0))
    assert controller.contract is None if hasattr(controller, "contract") else True


def test_safety_filter_pushes_away_from_an_unsafe_neighbour():
    controller = DBACTController(coverage_params(d_min=0.40), (0.0, 4.0, 0.0, 4.0))
    result = controller.safety.filter_velocity(
        np.array([0.0, 0.0]), np.array([0.20, 0.0]), [np.array([0.20, 0.0])]
    )
    assert result.velocity[0] < 0.0
    assert np.linalg.norm(result.velocity) <= controller.params.max_speed + 1e-9


def test_decentralized_coverage_simulation_moves_toward_the_target_region():
    cfg = load_yaml("configs/sim/decentralized_cvt_coverage.yaml")
    env = SimulationEnvironment(cfg)
    target = np.asarray(cfg["controller"]["target_center"], dtype=float)
    before = np.mean([np.linalg.norm(agent.position - target) for agent in env.agents])

    env.run(steps=60)

    after = np.mean([np.linalg.norm(agent.position - target) for agent in env.agents])
    assert after < before
    assert min(env.log.min_distances) >= cfg["controller"]["d_min"] - 1e-6


def test_simulation_frame_callback_receives_step_then_environment():
    cfg = load_yaml("configs/sim/decentralized_cvt_coverage.yaml")
    env = SimulationEnvironment(cfg)
    seen: list[tuple[int, int]] = []

    env.run(steps=2, on_frame=lambda step_index, frame_env: seen.append((step_index, len(frame_env.log.times))))

    assert seen == [(0, 1), (1, 2), (2, 3)]


def test_unknown_controller_parameter_is_rejected():
    """A silently ignored parameter is a configuration that does not describe the
    run, which is how `map_ttl` survived long after the map stopped using a TTL."""
    with pytest.raises(ValueError, match="unknown controller parameters"):
        DBACTParams.from_dict({"cage_offset": 0.135, "map_ttl": 4.0, "cbf_gamma": 6.0})
