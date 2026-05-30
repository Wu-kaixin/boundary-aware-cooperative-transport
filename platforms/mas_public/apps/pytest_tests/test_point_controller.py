import math

import pytest

from src.common.messages import RobotCommand, RobotState, WorldState
from src.common.messages import RobotStatus
from src.common.time_utils import monotonic_s, now_s
from src.controller.controller_module import ControllerModule
from src.controller.coordinate_transform import CoordinateTransformer
from src.controller.point_controller import PointController


def point_config():
    return {
        "controller": {"type": "point", "robot_mode": "free"},
        "controller_params": {
            "point": {
                "kp_x": 0.4,
                "kd_x": 0.0,
                "max_vx": 0.15,
                "kp_y": 0.4,
                "kd_y": 0.0,
                "max_vy": 0.15,
                "kp_yaw": 0.4,
                "kd_yaw": 0.0,
                "max_wz": 0.15,
                "position_tolerance_m": 0.05,
                "yaw_tolerance_rad": 0.05,
                "targets": {"robot_1": {"x": 1.0, "y": 0.0, "yaw": 0.0}},
            }
        },
    }


def worldstate_smoothing_config(**smoothing):
    return {
        "enabled": True,
        "method": "ema",
        "alpha_xy": 0.5,
        "alpha_yaw": 0.5,
        "near_target_only": False,
        "near_target_distance_m": 0.3,
        **smoothing,
    }


def point_config_with_yaw_params(**yaw_params):
    config = point_config()
    config["controller_params"]["point"].update(
        {
            "kp_yaw": 0.4,
            "kd_yaw": 0.0,
            "max_wz": 0.15,
            **yaw_params,
        }
    )
    return config


def gimbal_control_with_yaw_follow(**yaw_follow):
    return {
        "gimbal_control": {
            "yaw_follow": {
                "enabled": True,
                "feedforward_enabled": True,
                **yaw_follow,
            }
        }
    }


def gimbal_control_with_yaw_feedback(**yaw_follow):
    return gimbal_control_with_yaw_follow(
        feedback_enabled=True,
        feedback_kp=2.0,
        feedback_deadband_deg=1.0,
        feedback_max_speed_deg_s=35.0,
        feedback_timeout_s=1.0e10,
        **yaw_follow,
    )


def gimbal_control_with_pitch_hold(**pitch_hold):
    config = gimbal_control_with_yaw_feedback()
    config["gimbal_control"]["pitch_hold"] = {
        "enabled": True,
        "target_deg": 0.0,
        "kp": -2.0,
        "deadband_deg": 1.0,
        "max_speed_deg_s": 35.0,
        "feedback_timeout_s": 1.0e10,
        **pitch_hold,
    }
    return config


def world_state(
    x: float,
    y: float,
    yaw: float = 0.0,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    tracked: bool = True,
) -> WorldState:
    return WorldState(
        1.0,
        1,
        [RobotState("robot_1", tracked, x, y, 0.0, 0.0, 0.0, yaw, vx, vy, wz, 1.0)],
    )


def matmul(first, second):
    return [
        [sum(first[row][index] * second[index][column] for index in range(3)) for column in range(3)]
        for row in range(3)
    ]


def rx(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rpy_from_matrix(matrix):
    pitch = math.asin(-matrix[2][0])
    roll = math.atan2(matrix[2][1], matrix[2][2])
    yaw = math.atan2(matrix[1][0], matrix[0][0])
    return roll, pitch, yaw


def test_controller_module_builds_point_controller():
    module = ControllerModule.__new__(ControllerModule)
    module.controller_config = point_config()
    module.robots_config = {}
    module.robot_ids = ["robot_1"]
    assert isinstance(module._build_controller(), PointController)


def test_controller_module_rejects_old_pid_type():
    module = ControllerModule.__new__(ControllerModule)
    module.controller_config = {"controller": {"type": "pid", "robot_mode": "chassis_lead"}}
    module.robot_ids = ["robot_1"]
    with pytest.raises(ValueError, match="Unsupported controller type: pid"):
        module._build_controller()


def test_controller_module_reports_tracking_lost_after_untracked_timeout():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"experiment": {"auto_stop_on_untracked": True, "untracked_timeout_s": 1.0}}
    module.robot_ids = ["robot_1", "robot_2"]
    module.last_world_state = WorldState(
        1.0,
        1,
        [
            RobotState("robot_1", True, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            RobotState("robot_2", False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        ],
    )
    module.untracked_since_by_id = {"robot_2": monotonic_s() - 2.0}

    message = module._tracking_lost_message(world_state_fresh=True)

    assert message == "tracking_lost: untracked robots=['robot_2']"


def test_controller_module_resets_untracked_timer_when_robot_recovers():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"experiment": {"auto_stop_on_untracked": True, "untracked_timeout_s": 1.0}}
    module.robot_ids = ["robot_1"]
    module.last_world_state = world_state(0.0, 0.0, tracked=True)
    module.untracked_since_by_id = {"robot_1": monotonic_s() - 2.0}

    message = module._tracking_lost_message(world_state_fresh=True)

    assert message is None
    assert module.untracked_since_by_id == {}


def test_point_controller_outputs_point_command():
    controller = PointController(point_config(), ["robot_1"])
    state = world_state(0.0, 0.0)
    command = controller.compute(state).commands[0]
    assert command.controller_mode == "point"
    assert command.chassis_vx == 0.15
    assert command.chassis_vy == 0.0


def test_point_controller_stops_when_target_is_reached():
    controller = PointController(point_config(), ["robot_1"])
    state = world_state(0.98, 0.0, 0.03)
    command = controller.compute(state).commands[0]
    assert command.controller_mode == "point_completed"
    assert command.chassis_vx == 0.0
    assert command.chassis_vy == 0.0
    assert command.chassis_wz == 0.0


def test_point_controller_requires_yaw_at_target():
    controller = PointController(point_config(), ["robot_1"])
    state = world_state(0.98, 0.0, 0.5)
    command = controller.compute(state).commands[0]
    assert command.controller_mode == "point"
    assert command.chassis_wz != 0.0
    assert command.chassis_vx == 0.0
    assert command.chassis_vy == 0.0


def test_point_controller_pd_damps_world_velocity():
    config = point_config()
    config["controller_params"]["point"]["kd_x"] = 0.2
    controller = PointController(config, ["robot_1"])

    command = controller.compute(world_state(0.0, 0.0, vx=2.0)).commands[0]

    assert command.chassis_vx == pytest.approx(0.0)
    assert command.chassis_vy == 0.0
    assert command.chassis_wz == 0.0


def test_point_controller_uses_separate_x_y_pd_params():
    config = point_config()
    config["controller_params"]["point"].update(
        {
            "kp_x": 0.1,
            "kd_x": 0.0,
            "max_vx": 0.5,
            "kp_y": 0.4,
            "kd_y": 0.0,
            "max_vy": 0.5,
            "targets": {"robot_1": {"x": 1.0, "y": 1.0, "yaw": 0.0}},
        }
    )
    controller = PointController(config, ["robot_1"])

    command = controller.compute(world_state(0.0, 0.0)).commands[0]

    assert command.chassis_vx == pytest.approx(0.1)
    assert command.chassis_vy == pytest.approx(0.4)


def test_point_smoothing_ema_uses_smoothed_pose():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"worldstate_smoothing": worldstate_smoothing_config()}
    module.controller_config = point_config()
    module.smoothed_pose_by_id = {}
    module._smooth_world_state(world_state(0.0, 0.0))

    smoothed = module._smooth_world_state(world_state(0.8, 0.0)).robots[0]

    assert smoothed.x == pytest.approx(0.4)
    assert module.smoothed_pose_by_id["robot_1"].x == pytest.approx(0.4)


def test_point_smoothing_near_target_only_uses_raw_state_when_far():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {
        "worldstate_smoothing": worldstate_smoothing_config(near_target_only=True, near_target_distance_m=0.3)
    }
    module.controller_config = point_config()
    module.smoothed_pose_by_id = {}

    module._smooth_world_state(world_state(0.0, 0.0))
    smoothed = module._smooth_world_state(world_state(0.4, 0.0)).robots[0]

    assert smoothed.x == 0.4
    assert "robot_1" not in module.smoothed_pose_by_id


def test_control_frame_smooths_raw_state_before_z_up_transform():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"worldstate_smoothing": worldstate_smoothing_config(alpha_xy=0.5)}
    module.controller_config = point_config()
    module.smoothed_pose_by_id = {}
    module.coordinate_transformer = CoordinateTransformer(90.0)

    module._world_state_for_control_frame(world_state(0.0, 1.0), update_yaw_rate=False)
    control_frame = module._world_state_for_control_frame(world_state(0.0, 3.0), update_yaw_rate=False)

    assert control_frame.robots[0].z == pytest.approx(2.0)


def test_point_yaw_control_tracks_target_yaw():
    controller = PointController(point_config_with_yaw_params(max_wz=1.0), ["robot_1"])

    command = controller.compute(world_state(0.0, 0.0, yaw=0.4)).commands[0]

    assert command.chassis_wz == pytest.approx(-0.16)


def test_point_yaw_control_clamps_wz():
    controller = PointController(point_config_with_yaw_params(kp_yaw=1.0, max_wz=0.1), ["robot_1"])
    controller.compute(world_state(0.0, 0.0, yaw=0.0))

    command = controller.compute(world_state(0.0, 0.0, yaw=1.0)).commands[0]

    assert command.chassis_wz == pytest.approx(-0.1)


def test_point_yaw_control_uses_wz_damping():
    controller = PointController(point_config_with_yaw_params(kd_yaw=0.2), ["robot_1"])
    controller.compute(world_state(0.0, 0.0, yaw=0.0))

    command = controller.compute(world_state(0.0, 0.0, yaw=0.0, wz=0.3)).commands[0]

    assert command.chassis_wz == pytest.approx(-0.06)


def test_point_yaw_control_keeps_target_after_untracked():
    controller = PointController(point_config_with_yaw_params(), ["robot_1"])
    controller.compute(world_state(0.0, 0.0, yaw=0.0))
    controller.compute(world_state(0.0, 0.0, yaw=1.0, tracked=False))

    command = controller.compute(world_state(0.0, 0.0, yaw=1.0)).commands[0]

    assert command.chassis_wz == pytest.approx(-0.15)


def test_point_completion_requires_yaw_tolerance():
    controller = PointController(point_config_with_yaw_params(), ["robot_1"])

    command = controller.compute(world_state(1.0, 0.0, yaw=0.2)).commands[0]

    assert command.controller_mode == "point"
    assert command.chassis_vx == 0.0
    assert command.chassis_vy == 0.0
    assert command.chassis_wz != 0.0


def test_point_completed_when_position_and_yaw_are_within_tolerance():
    controller = PointController(point_config_with_yaw_params(), ["robot_1"])

    command = controller.compute(world_state(1.0, 0.0, yaw=0.03)).commands[0]

    assert command.controller_mode == "point_completed"
    assert command.chassis_vx == 0.0
    assert command.chassis_vy == 0.0
    assert command.chassis_wz == 0.0
    assert command.gimbal_yaw_speed == 0.0


def test_point_completed_latches_zero_command_after_reaching_target():
    controller = PointController(point_config_with_yaw_params(), ["robot_1"], {})

    reached = controller.compute(world_state(1.0, 0.0, yaw=0.03)).commands[0]
    completed = controller.compute(world_state(0.0, 0.0, yaw=1.0)).commands[0]

    assert reached.controller_mode == "point_completed"
    assert completed.controller_mode == "point_completed"
    assert completed.chassis_vx == 0.0
    assert completed.chassis_vy == 0.0
    assert completed.chassis_wz == 0.0
    assert completed.gimbal_yaw_speed == 0.0
    assert completed.gimbal_pitch_speed == 0.0
    assert controller.task_completed() is True


def test_point_hold_after_reached_delays_task_completion():
    config = point_config_with_yaw_params()
    config["controller_params"]["point"].update(
        {
            "hold_enabled": True,
            "hold_kp_x": 0.5,
            "hold_kd_x": 0.0,
            "hold_kp_y": 0.5,
            "hold_kd_y": 0.0,
            "hold_kp_yaw": 0.2,
            "hold_kd_yaw": 0.0,
            "hold_max_vx": 0.1,
            "hold_max_vy": 0.1,
            "hold_max_wz": 0.1,
            "hold_duration_s": 1.0,
        }
    )
    controller = PointController(config, ["robot_1"], {})

    first = controller.compute(WorldState(1.0, 1, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0)])).commands[0]
    second = controller.compute(WorldState(1.5, 2, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 1.5)])).commands[0]
    task_completed_before_hold_duration = controller.task_completed()
    completed = controller.compute(WorldState(2.1, 3, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 2.1)])).commands[0]

    assert first.controller_mode == "point_completed"
    assert first.chassis_vx != 0.0
    assert second.controller_mode == "point_completed"
    assert second.chassis_vx != 0.0
    assert task_completed_before_hold_duration is False
    assert completed.controller_mode == "point_completed"
    assert completed.chassis_vx == 0.0
    assert controller.task_completed() is True


def test_point_hold_after_reached_does_not_reset_when_error_leaves_tolerance():
    config = point_config_with_yaw_params()
    config["controller_params"]["point"].update({"hold_enabled": True, "hold_duration_s": 1.0})
    controller = PointController(config, ["robot_1"], {})

    controller.compute(WorldState(1.0, 1, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0)]))
    noisy = controller.compute(WorldState(1.5, 2, [RobotState("robot_1", True, 0.5, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.5)])).commands[0]
    completed = controller.compute(WorldState(2.1, 3, [RobotState("robot_1", True, 0.5, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 2.1)])).commands[0]

    assert noisy.controller_mode == "point_completed"
    assert completed.controller_mode == "point_completed"
    assert controller.task_completed() is True


def test_point_hold_completed_state_is_latched():
    config = point_config_with_yaw_params()
    config["controller_params"]["point"].update({"hold_enabled": True, "hold_duration_s": 1.0})
    controller = PointController(config, ["robot_1"], {})

    controller.compute(WorldState(1.0, 1, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 1.0)]))
    completed = controller.compute(WorldState(2.1, 2, [RobotState("robot_1", True, 0.98, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 2.1)])).commands[0]
    after_noise = controller.compute(WorldState(2.2, 3, [RobotState("robot_1", True, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.2)])).commands[0]

    assert completed.controller_mode == "point_completed"
    assert after_noise.controller_mode == "point_completed"
    assert after_noise.chassis_vx == 0.0
    assert after_noise.chassis_vy == 0.0
    assert after_noise.chassis_wz == 0.0


def test_gimbal_yaw_follow_uses_controller_wz_direction_and_deg_units():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"gimbal_control": {"yaw_follow": {"enabled": True, "feedforward_enabled": True}}}
    module.latest_robot_status_by_id = {}
    command = RobotCommand("robot_1", 0.0, 0.0, -1.0, 0.0, 0.0, "point")

    augmented = module._apply_gimbal_control_to_robot(command, module.system_config["gimbal_control"])

    assert augmented.chassis_wz == pytest.approx(-1.0)
    assert augmented.gimbal_yaw_speed == pytest.approx(math.degrees(-1.0))


def test_gimbal_yaw_follow_adds_gimbal_angle_feedback():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = gimbal_control_with_yaw_feedback()
    module.latest_robot_status_by_id = {
        "robot_1": RobotStatus("robot_1", "gimbal_angle", now_s(), yaw_angle=10.0)
    }
    command = RobotCommand("robot_1", 0.0, 0.0, -1.0, 0.0, 0.0, "point")

    augmented = module._apply_gimbal_control_to_robot(command, module.system_config["gimbal_control"])

    assert augmented.chassis_wz == pytest.approx(-1.0)
    assert augmented.gimbal_yaw_speed == pytest.approx(math.degrees(-1.0) + 20.0)


def test_gimbal_pitch_hold_uses_pitch_angle_feedback():
    module = ControllerModule.__new__(ControllerModule)
    module.system_config = {"gimbal_control": {"pitch_hold": gimbal_control_with_pitch_hold()["gimbal_control"]["pitch_hold"]}}
    module.latest_robot_status_by_id = {
        "robot_1": RobotStatus("robot_1", "gimbal_angle", now_s(), pitch_angle=10.0, yaw_angle=0.0)
    }
    command = RobotCommand("robot_1", 0.0, 0.0, 0.0, 0.0, 0.0, "point")

    augmented = module._apply_gimbal_control_to_robot(command, module.system_config["gimbal_control"])

    assert augmented.gimbal_pitch_speed == pytest.approx(-20.0)


def test_point_coordinate_transform_extracts_yaw_after_rx_correction():
    transformer = CoordinateTransformer(-90.0)
    expected_yaw = 0.7
    motive_matrix = matmul(rx(math.pi / 2.0), rz(expected_yaw))
    roll, pitch, yaw = rpy_from_matrix(motive_matrix)

    corrected_yaw = transformer.robot_state(
        RobotState("", True, 0.0, 0.0, 0.0, roll, pitch, yaw, 0.0, 0.0, 0.0, 0.0)
    ).yaw

    assert corrected_yaw == pytest.approx(expected_yaw)

