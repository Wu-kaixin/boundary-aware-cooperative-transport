from src.common.messages import ControlCommand, RobotCommand
from src.controller.controller_module import ControllerModule


def command(mode: str) -> ControlCommand:
    return ControlCommand(
        1.0,
        mode,
        [RobotCommand("r1", 1.0, 2.0, 3.0, 4.0, 5.0, "test")],
    )


def test_chassis_lead_normalization_omits_gimbal_data():
    normalized = ControllerModule._normalize_command_for_mode(command("chassis_lead"))
    robot_command = normalized.commands[0]

    assert robot_command.chassis_vx == 1.0
    assert robot_command.chassis_vy == 2.0
    assert robot_command.chassis_wz == 3.0
    assert robot_command.gimbal_yaw_speed is None
    assert robot_command.gimbal_pitch_speed is None


def test_gimbal_lead_normalization_omits_chassis_data():
    normalized = ControllerModule._normalize_command_for_mode(command("gimbal_lead"))
    robot_command = normalized.commands[0]

    assert robot_command.chassis_vx is None
    assert robot_command.chassis_vy is None
    assert robot_command.chassis_wz is None
    assert robot_command.gimbal_yaw_speed == 4.0
    assert robot_command.gimbal_pitch_speed == 5.0


def test_free_normalization_keeps_all_data():
    original = command("free")
    normalized = ControllerModule._normalize_command_for_mode(original)

    assert normalized is original

