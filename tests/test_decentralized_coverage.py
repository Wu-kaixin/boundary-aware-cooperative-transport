import numpy as np

from dbact.controller import DBACTController, DBACTParams
from dbact.local_cbf_qp import LocalCBFQP
from dbact.types import AgentState
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def test_coverage_mode_generates_local_cvt_commands():
    params = DBACTParams(
        task_mode="coverage",
        comm_range=1.0,
        target_center=[2.0, 2.0],
        target_radius=0.6,
        target_sensor_range=3.0,
        target_samples=12,
        cbf_use_qp=False,
    )
    controller = DBACTController(params, (0.0, 4.0, 0.0, 4.0))
    agents = [
        AgentState("a0", np.array([1.5, 1.0])),
        AgentState("a1", np.array([2.0, 1.0])),
        AgentState("a2", np.array([2.5, 1.0])),
    ]

    commands = controller.step(agents, [], timestamp=0.0, dt=0.05)

    assert {command.mode for command in commands} == {"dbact_coverage"}
    assert any(np.linalg.norm(command.velocity) > 1e-6 for command in commands)
    assert all(np.linalg.norm(command.velocity) <= params.max_speed + 1e-9 for command in commands)


def test_local_cbf_qp_pushes_away_from_unsafe_neighbor():
    cbf = LocalCBFQP(d_min=0.40, gamma=6.0, max_speed=0.30, use_qp=False)

    velocity = cbf.filter_velocity(
        position=np.array([0.0, 0.0]),
        nominal_velocity=np.array([0.20, 0.0]),
        neighbor_positions=[np.array([0.20, 0.0])],
        neighbor_velocities=[np.zeros(2)],
    )

    assert velocity[0] < 0.0
    assert np.linalg.norm(velocity) <= cbf.max_speed + 1e-9


def test_decentralized_cvt_coverage_simulation_moves_toward_target_region():
    cfg = load_yaml("configs/sim/decentralized_cvt_coverage.yaml")
    cfg["controller"]["cbf_use_qp"] = False
    env = SimulationEnvironment(cfg)
    target = np.asarray(cfg["controller"]["target_center"], dtype=float)
    before = np.mean([np.linalg.norm(agent.position - target) for agent in env.agents])

    env.run(steps=80)

    after = np.mean([np.linalg.norm(agent.position - target) for agent in env.agents])
    assert after < before
    assert min(env.log.min_distances) >= cfg["controller"]["d_min"] - 0.05


def test_simulation_frame_callback_receives_step_then_environment():
    cfg = load_yaml("configs/sim/decentralized_cvt_coverage.yaml")
    cfg["controller"]["cbf_use_qp"] = False
    env = SimulationEnvironment(cfg)
    seen = []

    env.run(steps=2, on_frame=lambda step_index, frame_env: seen.append((step_index, len(frame_env.log.times))))

    assert seen == [(0, 1), (1, 2), (2, 3)]
