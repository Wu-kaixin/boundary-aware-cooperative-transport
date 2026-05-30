from src.common.messages import RobotState, WorldState
from src.controller.controller_module import ControllerModule
from src.controller.cvt_controller import CVTController


def cvt_config():
    return {
        "controller": {"type": "cvt", "robot_mode": "chassis_lead"},
        "controller_params": {
            "cvt": {
                "kp_x": 0.5,
                "kd_x": 0.0,
                "kp_y": 0.5,
                "kd_y": 0.0,
                "kp_yaw": 0.8,
                "kd_yaw": 0.0,
                "max_wz": 0.3,
                "grid_resolution": 30,
                "yaw_mode": "face_velocity",
                "centroid_tolerance_m": 0.0,
            }
        },
    }


def limits_config(max_vx: float = 0.2, max_vy: float = 0.2, max_wz: float = 0.3):
    return {"chassis": {"max_vx": max_vx, "max_vy": max_vy, "max_wz": max_wz}}


def world_config():
    return {
        "x_min": -1.0,
        "x_max": 1.0,
        "y_min": -1.5,
        "y_max": 1.5,
        "z_min": -0.1,
        "z_max": 1.0,
        "stop_on_out_of_bounds": True,
    }


def robot_state(robot_id: str, tracked: bool, x: float, y: float, yaw: float = 0.0) -> RobotState:
    return RobotState(robot_id, tracked, x, y, 0.0, 0.0, 0.0, yaw, 0.0, 0.0, 0.0, 1.0)


def command_by_id(commands):
    return {command.robot_id: command for command in commands}


def test_controller_module_builds_cvt_controller():
    module = ControllerModule.__new__(ControllerModule)
    module.controller_config = cvt_config()
    module.robot_ids = ["robot_1", "robot_2"]
    module.system_config = {"world": world_config()}
    assert isinstance(module._build_controller(), CVTController)


def test_cvt_zero_when_world_state_missing():
    controller = CVTController(cvt_config(), ["robot_1", "robot_2"], world_config())
    commands = controller.compute(None).commands
    assert all(command.controller_mode == "cvt_zero" for command in commands)
    assert all(command.chassis_vx == 0.0 and command.chassis_vy == 0.0 for command in commands)


def test_cvt_uses_tracked_robots_dynamically_and_untracked_zero():
    controller = CVTController(cvt_config(), ["robot_1", "robot_2", "robot_3"], world_config())
    state = WorldState(
        1.0,
        1,
        [
            robot_state("robot_1", True, -0.8, 0.0),
            robot_state("robot_2", True, 1.8, 0.0),
            robot_state("robot_3", False, 0.0, 0.0),
        ],
    )
    commands = command_by_id(controller.compute(state).commands)
    assert commands["robot_1"].controller_mode == "cvt"
    assert commands["robot_2"].controller_mode == "cvt"
    assert commands["robot_3"].controller_mode == "cvt_untracked"
    assert commands["robot_3"].chassis_vx == 0.0
    assert commands["robot_1"].chassis_vx != commands["robot_2"].chassis_vx


def test_cvt_single_tracked_robot_moves_toward_world_center():
    controller = CVTController(cvt_config(), ["robot_1"], world_config(), limits_config(1.0, 1.0))
    state = WorldState(1.0, 1, [robot_state("robot_1", True, -1.0, -1.0)])
    command = controller.compute(state).commands[0]
    assert command.controller_mode == "cvt"
    assert command.chassis_vx > 0.0
    assert command.chassis_vy > 0.0


def test_cvt_speed_is_limited_by_system_chassis_limits():
    max_vx = 0.05
    max_vy = 0.04
    controller = CVTController(cvt_config(), ["robot_1"], world_config(), limits_config(max_vx, max_vy))
    state = WorldState(1.0, 1, [robot_state("robot_1", True, -1.0, -1.0)])
    command = controller.compute(state).commands[0]
    assert abs(command.chassis_vx) <= max_vx + 1e-9
    assert abs(command.chassis_vy) <= max_vy + 1e-9


def test_cvt_uses_xy_pd_velocity_terms():
    config = cvt_config()
    config["controller_params"]["cvt"].update({"kp_x": 0.5, "kd_x": 0.2, "kp_y": 0.5, "kd_y": 0.0})
    controller = CVTController(config, ["robot_1"], world_config(), limits_config(1.0, 1.0))
    moving = RobotState("robot_1", True, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0)

    command = controller.compute(WorldState(1.0, 1, [moving])).commands[0]

    assert command.chassis_vx < 1.0
    assert command.chassis_vy > 0.0


def test_cvt_fixed_yaw_tracks_target_yaw():
    config = cvt_config()
    config["controller_params"]["cvt"].update({"yaw_mode": "fixed", "target_yaw": 0.0, "kp_yaw": 0.5})
    controller = CVTController(config, ["robot_1"], world_config(), limits_config(1.0, 1.0, 1.0))

    command = controller.compute(WorldState(1.0, 1, [robot_state("robot_1", True, -1.0, -1.0, yaw=0.5)])).commands[0]

    assert command.chassis_wz < 0.0


def test_cvt_global_completion_requires_all_robots_at_centroids():
    config = cvt_config()
    config["controller_params"]["cvt"].update(
        {"centroid_tolerance_m": 10.0, "hold_enabled": False}
    )
    controller = CVTController(config, ["robot_1", "robot_2"], world_config())
    state = WorldState(
        1.0,
        1,
        [
            robot_state("robot_1", True, -0.5, 0.0),
            robot_state("robot_2", False, 0.5, 0.0),
        ],
    )

    commands = command_by_id(controller.compute(state).commands)

    assert controller.task_completed() is False
    assert commands["robot_1"].controller_mode == "cvt"
    assert commands["robot_2"].controller_mode == "cvt_untracked"


def test_cvt_completed_without_hold_when_all_robots_at_centroids():
    config = cvt_config()
    config["controller_params"]["cvt"].update(
        {"centroid_tolerance_m": 10.0, "hold_enabled": False}
    )
    controller = CVTController(config, ["robot_1"], world_config())

    command = controller.compute(WorldState(1.0, 1, [robot_state("robot_1", True, 0.0, 0.0)])).commands[0]

    assert command.controller_mode == "cvt_completed"
    assert command.chassis_vx == 0.0
    assert command.chassis_vy == 0.0
    assert command.chassis_wz == 0.0
    assert controller.task_completed() is True


def test_cvt_hold_after_global_completion_delays_task_completion():
    config = cvt_config()
    config["controller_params"]["cvt"].update(
        {
            "centroid_tolerance_m": 10.0,
            "hold_enabled": True,
            "hold_duration_s": 0.5,
            "hold_kp_x": 0.1,
            "hold_kp_y": 0.1,
            "hold_max_vx": 0.05,
            "hold_max_vy": 0.05,
        }
    )
    controller = CVTController(config, ["robot_1"], world_config())

    first = controller.compute(WorldState(1.0, 1, [robot_state("robot_1", True, -0.5, 0.0)])).commands[0]
    task_completed_before_hold_duration = controller.task_completed()
    done = controller.compute(WorldState(1.6, 2, [robot_state("robot_1", True, -0.5, 0.0)])).commands[0]

    assert first.controller_mode == "cvt_completed"
    assert first.chassis_vx != 0.0
    assert task_completed_before_hold_duration is False
    assert done.controller_mode == "cvt_completed"
    assert done.chassis_vx == 0.0
    assert done.chassis_vy == 0.0
    assert controller.task_completed() is True

