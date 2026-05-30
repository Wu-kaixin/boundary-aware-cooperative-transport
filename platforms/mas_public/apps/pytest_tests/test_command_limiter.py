from src.common.messages import RobotCommand
from src.robot.command_limiter import CommandLimiter


def test_command_limiter_clamps_values():
    limiter = CommandLimiter(
        {
            "chassis": {"max_vx": 0.2, "max_vy": 0.3, "max_wz": 0.4},
            "gimbal": {"max_yaw_speed": 10, "max_pitch_speed": 20},
        }
    )
    limited = limiter.limit(RobotCommand("r1", 1, -1, 2, 99, -99, "test"))
    assert limited.chassis_vx == 0.2
    assert limited.chassis_vy == -0.3
    assert limited.chassis_wz == 0.4
    assert limited.gimbal_yaw_speed == 10
    assert limited.gimbal_pitch_speed == -20


def test_command_limiter_preserves_inactive_channels():
    limiter = CommandLimiter(
        {
            "chassis": {"max_vx": 0.2, "max_vy": 0.3, "max_wz": 0.4},
            "gimbal": {"max_yaw_speed": 10, "max_pitch_speed": 20},
        }
    )
    limited = limiter.limit(RobotCommand("r1", None, None, None, 99, None, "test"))
    assert limited.chassis_vx is None
    assert limited.chassis_vy is None
    assert limited.chassis_wz is None
    assert limited.gimbal_yaw_speed == 10
    assert limited.gimbal_pitch_speed is None

