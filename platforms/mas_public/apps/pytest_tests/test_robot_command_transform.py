import pytest

from src.common.messages import RobotCommand
from src.robot.robot_command_transform import RobotCommandTransform


def command() -> RobotCommand:
    return RobotCommand("r1", 0.1, 0.2, 0.3, 4.0, 5.0, "test")


def test_planar_transform_maps_to_robomaster_direction():
    transform = RobotCommandTransform(
        {
            "enabled": True,
            "linear": {
                "dimension": 2,
                "matrix": [[1.0, 0.0], [0.0, -1.0]],
            },
            "angular": {
                "dimension": 1,
                "scale": -1.0,
            },
        }
    )

    transformed = transform.apply(command())

    assert transformed.chassis_vx == pytest.approx(0.1)
    assert transformed.chassis_vy == pytest.approx(-0.2)
    assert transformed.chassis_wz == pytest.approx(-0.3)
    assert transformed.gimbal_yaw_speed == pytest.approx(4.0)
    assert transformed.gimbal_pitch_speed == pytest.approx(5.0)
    assert transformed.controller_mode == "test"


def test_disabled_transform_returns_same_command():
    original = command()
    transformed = RobotCommandTransform({"enabled": False}).apply(original)
    assert transformed is original


def test_transform_preserves_inactive_chassis_channels():
    transform = RobotCommandTransform({"enabled": True})
    transformed = transform.apply(RobotCommand("r1", None, None, None, 4.0, 5.0, "test"))
    assert transformed.chassis_vx is None
    assert transformed.chassis_vy is None
    assert transformed.chassis_wz is None
    assert transformed.gimbal_yaw_speed == 4.0


def test_transform_maps_gimbal_yaw_scale_in_robot_layer():
    transform = RobotCommandTransform({"enabled": True, "gimbal": {"yaw_scale": -1.0}})

    transformed = transform.apply(RobotCommand("r1", 0.0, 0.0, 0.3, 4.0, 0.0, "test"))

    assert transformed.chassis_wz == pytest.approx(0.3)
    assert transformed.gimbal_yaw_speed == pytest.approx(-4.0)


def test_invalid_linear_matrix_is_rejected():
    with pytest.raises(ValueError, match="2x2"):
        RobotCommandTransform(
            {
                "enabled": True,
                "linear": {"dimension": 2, "matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]},
                "angular": {"dimension": 1, "scale": 1.0},
            }
        )

