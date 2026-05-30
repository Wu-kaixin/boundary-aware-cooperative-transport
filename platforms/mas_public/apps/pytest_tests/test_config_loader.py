import copy

import pytest

from src.common.config_loader import load_all_configs, load_config, validate_configs
from src.common.exceptions import ConfigError


def test_load_config():
    system = load_config("system.yaml")
    assert system["network"]["backend"] == "zmq"
    assert system["world"]["stop_on_out_of_bounds"] is True
    assert system["experiment"]["auto_stop_on_task_completed"] is True
    assert system["experiment"]["auto_stop_on_untracked"] is True
    assert system["experiment"]["untracked_timeout_s"] > 0


def test_load_all_configs():
    configs = load_all_configs()
    assert "controller" in configs
    assert "robots" in configs


def test_natnet_python_client_path_is_project_relative():
    optitrack = load_config("optitrack.yaml")
    assert optitrack["natnet"]["python_client_path"] == "third_party/natnet_client"


def test_robot_expected_count_must_match_robot_list_length():
    configs = load_all_configs()
    broken = copy.deepcopy(configs)
    broken["robots"]["robots"]["expected_count"] = len(broken["robots"]["robots"]["list"]) + 1

    with pytest.raises(ConfigError, match="expected_count"):
        validate_configs(broken)


def test_duplicate_robot_hardware_mapping_is_rejected():
    configs = load_all_configs()
    broken = copy.deepcopy(configs)
    robot_list = broken["robots"]["robots"]["list"]
    robot_list[1]["rigid_body_id"] = robot_list[0]["rigid_body_id"]

    with pytest.raises(ConfigError, match="duplicate rigid_body_id"):
        validate_configs(broken)


def test_invalid_cvt_yaw_mode_is_rejected():
    broken = {
        "system": {
            "experiment": {
                "auto_stop_on_task_completed": True,
                "auto_stop_on_untracked": True,
                "untracked_timeout_s": 5.0,
            },
            "z_up_transform": {"enabled": True},
            "worldstate_smoothing": {"enabled": False, "near_target_only": False},
            "gimbal_control": {
                "yaw_follow": {"enabled": False, "feedforward_enabled": True, "feedback_enabled": True},
                "pitch_hold": {"enabled": False},
            },
            "robot_command_transform": {"enabled": True},
            "network": {"backend": "zmq"},
            "frequency": {
                "optitrack_publish_hz": 100,
                "controller_hz": 20,
                "robot_command_hz": 20,
            },
            "world": {"stop_on_out_of_bounds": True, "out_of_bounds_fail_delay_s": 5.0},
        },
        "controller": {
            "controller": {"type": "cvt", "robot_mode": "free"},
            "input": {"state_timeout_ms": 500, "require_all_tracked_for_valid_state": False},
            "recording": {"enable": True},
            "plot": {
                "enable_after_experiment": True,
                "plot_trajectory": True,
                "plot_pose_time_series": True,
                "plot_command_time_series": True,
            },
            "controller_params": {
                "cvt": {
                    "hold_enabled": True,
                    "yaw": {
                        "mode": "face_veloci",
                        "target": 0.0,
                    },
                },
            }
        },
        "supervisor": {
            "use_optitrack": True,
            "use_robot": True,
            "use_controller": True,
            "shutdown_timeout_s": 4.0,
        },
        "optitrack": {
            "natnet": {"connect_check_timeout_s": 2.0, "stream_type": "d"},
            "tracking_validation": {
                "enabled": False,
                "reject_position_jump": False,
                "tracking_timeout_enabled": False,
            },
            "publish": {"publish_untracked": True},
            "state_estimation": {"enable_velocity_estimation": True},
            "diagnostics": {"log_rigid_bodies": True},
        },
        "robots": {
            "robots": {
                "list": [
                    {
                        "robot_id": "robot_1",
                        "sn": "sn1",
                        "rigid_body_name": "Rigid Body 001",
                        "chassis_enabled": True,
                        "gimbal_enabled": True,
                    }
                ]
            },
            "connection": {"require_sn": True},
            "chassis": {"stop_on_exit": True},
            "gimbal": {"angle_status": {"enabled": True}, "init_zero_on_connect": {"enabled": True}},
            "watchdog": {"stop_on_timeout": True},
        },
        "logging": {"log_to_console": True, "log_to_file": True},
    }

    with pytest.raises(ConfigError, match="controller_params.cvt.yaw.mode"):
        validate_configs(broken)


def test_string_bool_values_are_rejected():
    configs = load_all_configs()
    broken = copy.deepcopy(configs)
    broken["robots"]["robots"]["list"][0]["gimbal_enabled"] = "false"

    with pytest.raises(ConfigError, match=r"robots\.list\[0\]\.gimbal_enabled"):
        validate_configs(broken)


def test_controller_plot_bool_values_are_rejected():
    configs = load_all_configs()
    broken = copy.deepcopy(configs)
    broken["controller"]["plot"]["plot_trajectory"] = "true"

    with pytest.raises(ConfigError, match="plot.plot_trajectory"):
        validate_configs(broken)

