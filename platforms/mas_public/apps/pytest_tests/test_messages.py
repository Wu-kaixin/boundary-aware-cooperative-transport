from src.common.messages import ControlCommand, RobotCommand, RobotState, WorldState, message_to_dict


def test_world_state_roundtrip():
    state = WorldState(1.0, 1, [RobotState("robot_1", True, 1, 2, 0, 0.0, 0.0, 0.1, 0, 0, 0, 1.0)])
    data = message_to_dict(state)
    assert "roll" in data["robots"][0]
    assert "pitch" in data["robots"][0]
    assert "yaw" in data["robots"][0]
    assert "theta" not in data["robots"][0]
    restored = WorldState.from_dict(data)
    assert restored.robots[0].robot_id == "robot_1"
    assert restored.robots[0].tracked is True
    assert restored.robots[0].yaw == 0.1


def test_control_command_roundtrip():
    command = ControlCommand(
        1.0,
        "chassis_lead",
        [RobotCommand("robot_1", 0.1, 0.0, 0.0, 0.0, 0.0, "test")],
    )
    restored = ControlCommand.from_dict(message_to_dict(command))
    assert restored.robot_mode == "chassis_lead"
    assert restored.commands[0].chassis_vx == 0.1

