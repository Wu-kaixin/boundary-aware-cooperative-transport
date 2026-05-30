from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from src.common.exceptions import ConfigError


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "configs"
CONTROLLER_TYPES = {"manual", "point", "cvt", "dtransport"}
ROBOT_MODES = {"chassis_lead", "free", "gimbal_lead"}
CVT_YAW_MODES = {"face_velocity", "fixed"}


def load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    if not config_path.is_absolute():
        config_path = PROJECT_ROOT / config_path
    if not config_path.exists():
        raise ConfigError(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"Config must be a mapping: {config_path}")
    return data


def load_config(name: str) -> dict[str, Any]:
    return load_yaml(CONFIG_DIR / name)


def load_controller_config() -> dict[str, Any]:
    """Internal module."""
    config = load_config("controller.yaml")
    controller = _lookup(config, "controller")
    _forbid_field(controller, "controller.params_file")
    controller_type = _require_enum(controller, "controller.type", CONTROLLER_TYPES)
    params = load_yaml(CONFIG_DIR / "controllers" / f"{controller_type}.yaml")
    config["controller_params"] = {controller_type: params}
    return config


def load_all_configs() -> dict[str, dict[str, Any]]:
    """Internal module."""
    configs = {
        "system": load_config("system.yaml"),
        "optitrack": load_config("optitrack.yaml"),
        "robots": load_config("robots.yaml"),
        "controller": load_controller_config(),
        "supervisor": load_config("supervisor.yaml"),
        "logging": load_config("logging.yaml"),
    }
    validate_configs(configs)
    return configs


def validate_configs(configs: dict[str, dict[str, Any]]) -> None:
    """Internal module."""
    system = configs["system"]
    experiment = _lookup(system, "experiment")
    _require_bool(experiment, "experiment.auto_stop_on_task_completed")
    _require_bool(experiment, "experiment.auto_stop_on_untracked")
    _require_positive(experiment, "experiment.untracked_timeout_s")
    _require_bool(_lookup(system, "z_up_transform"), "z_up_transform.enabled")
    worldstate_smoothing = _lookup(system, "worldstate_smoothing")
    _require_bool(worldstate_smoothing, "worldstate_smoothing.enabled")
    _require_bool(worldstate_smoothing, "worldstate_smoothing.near_target_only")
    gimbal_control = _lookup(system, "gimbal_control")
    yaw_follow = _lookup(gimbal_control, "yaw_follow")
    _require_bool(yaw_follow, "gimbal_control.yaw_follow.enabled")
    _require_bool(yaw_follow, "gimbal_control.yaw_follow.feedforward_enabled")
    _require_bool(yaw_follow, "gimbal_control.yaw_follow.feedback_enabled")
    _require_bool(_lookup(gimbal_control, "pitch_hold"), "gimbal_control.pitch_hold.enabled")
    _require_bool(_lookup(system, "robot_command_transform"), "robot_command_transform.enabled")
    _require_enum(_lookup(system, "network"), "network.backend", {"zmq"})
    frequency = _lookup(system, "frequency")
    _require_positive(frequency, "frequency.optitrack_publish_hz")
    _require_positive(frequency, "frequency.controller_hz")
    _require_positive(frequency, "frequency.robot_command_hz")
    world = _lookup(system, "world")
    _require_bool(world, "world.stop_on_out_of_bounds")
    _require_positive(world, "world.out_of_bounds_fail_delay_s")

    controller_config = configs["controller"]
    controller = _lookup(controller_config, "controller")
    _forbid_field(controller, "controller.params_file")
    controller_type = _require_enum(controller, "controller.type", CONTROLLER_TYPES)
    _require_enum(controller, "controller.robot_mode", ROBOT_MODES)
    _require_positive(_lookup(controller_config, "input"), "input.state_timeout_ms")
    _require_bool(_lookup(controller_config, "input"), "input.require_all_tracked_for_valid_state")
    _require_bool(_lookup(controller_config, "recording"), "recording.enable")
    plot = _lookup(controller_config, "plot")
    _require_bool(plot, "plot.enable_after_experiment")
    _require_bool(plot, "plot.plot_trajectory")
    _require_bool(plot, "plot.plot_pose_time_series")
    _require_bool(plot, "plot.plot_command_time_series")
    _require_current_controller_params(controller_config, controller_type)
    _validate_controller_param_bools(controller_config["controller_params"][controller_type], controller_type)
    if controller_type == "cvt":
        _validate_cvt_params(controller_config["controller_params"]["cvt"])

    supervisor = configs["supervisor"]
    _require_bool(supervisor, "use_optitrack")
    _require_bool(supervisor, "use_robot")
    _require_bool(supervisor, "use_controller")
    _require_positive(supervisor, "shutdown_timeout_s")

    optitrack = configs["optitrack"]
    _require_positive(_lookup(optitrack, "natnet"), "natnet.connect_check_timeout_s")
    _require_enum(_lookup(optitrack, "natnet"), "natnet.stream_type", {"d", "c"})
    tracking_validation = _lookup(optitrack, "tracking_validation")
    _require_bool(tracking_validation, "tracking_validation.enabled")
    _require_bool(tracking_validation, "tracking_validation.reject_position_jump")
    _require_bool(tracking_validation, "tracking_validation.tracking_timeout_enabled")
    _require_bool(_lookup(optitrack, "publish"), "publish.publish_untracked")
    _require_bool(_lookup(optitrack, "state_estimation"), "state_estimation.enable_velocity_estimation")
    _require_bool(_lookup(optitrack, "diagnostics"), "diagnostics.log_rigid_bodies")

    _require_robot_ids(configs["robots"])
    _validate_robot_bools(configs["robots"])
    _validate_logging_bools(configs["logging"])


def get_project_root() -> Path:
    return PROJECT_ROOT


def _lookup(config: dict[str, Any], dotted_key: str) -> dict[str, Any]:
    value: Any = config
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            raise ConfigError(f"Missing required config section: {dotted_key}")
        value = value[part]
    if not isinstance(value, dict):
        raise ConfigError(f"Config section must be a mapping: {dotted_key}")
    return value


def _forbid_field(config: dict[str, Any], dotted_key: str) -> None:
    key = dotted_key.split(".")[-1]
    if key in config:
        raise ConfigError(
            f"{dotted_key} is no longer supported; select controller.type instead"
        )


def _require_bool(config: dict[str, Any], dotted_key: str) -> bool:
    key = dotted_key.split(".")[-1]
    value = config.get(key)
    if not isinstance(value, bool):
        raise ConfigError(f"{dotted_key} must be a boolean")
    return value


def _require_positive(config: dict[str, Any], dotted_key: str) -> float:
    key = dotted_key.split(".")[-1]
    value = config.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ConfigError(f"{dotted_key} must be a positive number")
    return float(value)


def _require_enum(config: dict[str, Any], dotted_key: str, allowed: set[str]) -> str:
    key = dotted_key.split(".")[-1]
    value = config.get(key)
    if not isinstance(value, str) or value not in allowed:
        allowed_text = ", ".join(sorted(allowed))
        raise ConfigError(f"{dotted_key} must be one of: {allowed_text}")
    return value


def _require_current_controller_params(config: dict[str, Any], controller_type: str) -> None:
    params = config.get("controller_params")
    if not isinstance(params, dict) or controller_type not in params:
        raise ConfigError(f"Missing controller_params for current controller type: {controller_type}")
    if set(params) != {controller_type}:
        raise ConfigError("controller_params must contain only the selected controller type")


def _validate_cvt_params(params: dict[str, Any]) -> None:
    yaw_config = params.get("yaw", {})
    if "yaw_mode" in params:
        _require_enum(params, "controller_params.cvt.yaw_mode", CVT_YAW_MODES)
    if yaw_config:
        if not isinstance(yaw_config, dict):
            raise ConfigError("controller_params.cvt.yaw must be a mapping")
        _require_enum(yaw_config, "controller_params.cvt.yaw.mode", CVT_YAW_MODES)


def _validate_controller_param_bools(params: dict[str, Any], controller_type: str) -> None:
    if controller_type in {"point", "cvt"}:
        _require_bool(params, f"controller_params.{controller_type}.hold_enabled")


def _validate_robot_bools(config: dict[str, Any]) -> None:
    robots = _lookup(config, "robots")
    robot_list = robots.get("list")
    if isinstance(robot_list, list):
        for index, item in enumerate(robot_list):
            if isinstance(item, dict):
                _require_bool(item, f"robots.list[{index}].chassis_enabled")
                _require_bool(item, f"robots.list[{index}].gimbal_enabled")
    _require_bool(_lookup(config, "connection"), "connection.require_sn")
    _require_bool(_lookup(config, "chassis"), "chassis.stop_on_exit")
    gimbal = _lookup(config, "gimbal")
    _require_bool(_lookup(gimbal, "angle_status"), "gimbal.angle_status.enabled")
    _require_bool(_lookup(gimbal, "init_zero_on_connect"), "gimbal.init_zero_on_connect.enabled")
    _require_bool(_lookup(config, "watchdog"), "watchdog.stop_on_timeout")


def _validate_logging_bools(config: dict[str, Any]) -> None:
    _require_bool(config, "logging.log_to_console")
    _require_bool(config, "logging.log_to_file")


def _require_robot_ids(config: dict[str, Any]) -> None:
    robots = _lookup(config, "robots")
    robot_list = robots.get("list")
    if not isinstance(robot_list, list) or not robot_list:
        raise ConfigError("robots.list must be a non-empty list")
    expected_count = robots.get("expected_count")
    if expected_count is not None:
        if not isinstance(expected_count, int) or isinstance(expected_count, bool) or expected_count <= 0:
            raise ConfigError("robots.expected_count must be a positive integer")
        if expected_count != len(robot_list):
            raise ConfigError("robots.expected_count must match robots.list length")
    robot_ids = []
    sns = []
    rigid_body_names = []
    rigid_body_ids = []
    for index, item in enumerate(robot_list):
        if not isinstance(item, dict) or not isinstance(item.get("robot_id"), str) or not item["robot_id"]:
            raise ConfigError(f"robots.list[{index}].robot_id must be a non-empty string")
        robot_ids.append(item["robot_id"])
        for key, values in {
            "sn": sns,
            "rigid_body_name": rigid_body_names,
        }.items():
            value = item.get(key)
            if not isinstance(value, str) or not value:
                raise ConfigError(f"robots.list[{index}].{key} must be a non-empty string")
            values.append(value)
        rigid_body_id = item.get("rigid_body_id")
        if rigid_body_id is not None:
            if not isinstance(rigid_body_id, int) or isinstance(rigid_body_id, bool):
                raise ConfigError(f"robots.list[{index}].rigid_body_id must be an integer")
            rigid_body_ids.append(rigid_body_id)
    if len(set(robot_ids)) != len(robot_ids):
        raise ConfigError("robots.list contains duplicate robot_id values")
    if len(set(sns)) != len(sns):
        raise ConfigError("robots.list contains duplicate sn values")
    if len(set(rigid_body_names)) != len(rigid_body_names):
        raise ConfigError("robots.list contains duplicate rigid_body_name values")
    if len(set(rigid_body_ids)) != len(rigid_body_ids):
        raise ConfigError("robots.list contains duplicate rigid_body_id values")

