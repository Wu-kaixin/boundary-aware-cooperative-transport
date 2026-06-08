import math

import numpy as np

from dbact.agent_control import (
    AgentController,
    AgentVelocityCommand,
    BodyVelocityCommand,
    CentralizedCVTParams,
    CentralizedCVTPolicy,
    DecentralizedDBACTPolicy,
    IntegratingStateProvider,
    MockRobotBackend,
    StaticStateProvider,
    TrackedAgentState,
    VelocityLimits,
    WorldSnapshot,
    world_to_body_command,
)
from dbact.controller import DBACTParams


def test_centralized_cvt_single_agent_moves_toward_domain_center():
    policy = CentralizedCVTPolicy(
        ["robot_1"],
        CentralizedCVTParams(domain=(-1.0, 1.0, -1.0, 1.0), kp_xy=1.0, max_speed=1.0),
    )
    snapshot = WorldSnapshot(
        timestamp=0.0,
        agents={"robot_1": TrackedAgentState("robot_1", x=-0.8, y=-0.6)},
    )

    command = policy.compute(snapshot)["robot_1"]

    assert command.mode == "centralized_cvt"
    assert command.vx_world > 0.0
    assert command.vy_world > 0.0


def test_world_to_body_command_uses_robot_yaw():
    state = TrackedAgentState("robot_1", x=0.0, y=0.0, yaw=math.pi / 2.0)
    command = AgentVelocityCommand("robot_1", vx_world=1.0, vy_world=0.0, wz=0.2, mode="test")

    body = world_to_body_command(state, command)

    assert abs(body.vx) < 1e-9
    assert np.isclose(body.vy, -1.0)
    assert body.wz == 0.2
    assert body.mode == "test"


def test_agent_controller_sends_limited_body_commands_to_backend():
    provider = StaticStateProvider({"robot_1": TrackedAgentState("robot_1", x=-1.0, y=-1.0)})
    policy = CentralizedCVTPolicy(
        ["robot_1"],
        CentralizedCVTParams(domain=(-1.0, 1.0, -1.0, 1.0), kp_xy=10.0, max_speed=10.0),
    )
    backend = MockRobotBackend()
    controller = AgentController(
        ["robot_1"],
        provider,
        policy,
        backend,
        limits=VelocityLimits(max_vx=0.05, max_vy=0.04, max_wz=0.03),
    )

    controller.connect()
    sent = controller.step()

    assert backend.connected_ids == ["robot_1"]
    assert sent == backend.sent_commands
    assert abs(sent[0].vx) <= 0.05 + 1e-9
    assert abs(sent[0].vy) <= 0.04 + 1e-9
    assert abs(sent[0].wz) <= 0.03 + 1e-9


def test_untracked_agent_gets_zero_command():
    provider = StaticStateProvider({"robot_1": TrackedAgentState("robot_1", x=0.0, y=0.0, tracked=False)})
    policy = CentralizedCVTPolicy(
        ["robot_1"],
        CentralizedCVTParams(domain=(-1.0, 1.0, -1.0, 1.0)),
    )
    backend = MockRobotBackend()
    controller = AgentController(["robot_1"], provider, policy, backend)

    command = controller.step()[0]

    assert command.vx == 0.0
    assert command.vy == 0.0
    assert command.wz == 0.0
    assert command.mode == "untracked"


def test_integrating_state_provider_advances_body_commands():
    provider = IntegratingStateProvider({"robot_1": TrackedAgentState("robot_1", x=0.0, y=0.0)})

    provider.advance_body_commands([BodyVelocityCommand("robot_1", vx=0.1, vy=0.0, wz=0.0)], dt=2.0)
    state = provider.snapshot().agents["robot_1"]

    assert np.isclose(state.x, 0.2)
    assert np.isclose(state.y, 0.0)
    assert np.isclose(state.vx, 0.1)


def test_dbact_policy_can_replace_centralized_cvt_policy():
    policy = DecentralizedDBACTPolicy(
        ["robot_1", "robot_2"],
        domain=(-1.0, 1.0, -1.0, 1.0),
        params=DBACTParams(task_mode="coverage", target_center=[0.0, 0.0], cbf_use_qp=False),
    )
    snapshot = WorldSnapshot(
        timestamp=0.0,
        agents={
            "robot_1": TrackedAgentState("robot_1", x=-0.8, y=0.0),
            "robot_2": TrackedAgentState("robot_2", x=0.8, y=0.0),
        },
    )

    commands = policy.compute(snapshot)

    assert set(commands) == {"robot_1", "robot_2"}
    assert all(command.mode.startswith("dbact_") for command in commands.values())
