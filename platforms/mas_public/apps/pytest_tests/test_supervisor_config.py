import csv

import pytest

from src.common.exceptions import ConfigError
from src.common.config_loader import load_config, validate_configs
from src.common.messages import ModuleStatus
from src.supervisor.supervisor import Supervisor
from src.messaging.topics import MODULE_STATUS, WORLD_STATE


def test_supervisor_config_defaults():
    config = load_config("supervisor.yaml")
    assert config["use_optitrack"] is True
    assert config["use_robot"] is True
    assert config["use_controller"] is True
    assert config["shutdown_timeout_s"] == 3.0


def test_supervisor_preflight_rejects_invalid_robot_mode(monkeypatch):
    broken_configs = {
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
            "network": {"backend": "zmq", "ports": {}},
            "frequency": {
                "optitrack_publish_hz": 100,
                "controller_hz": 20,
                "robot_command_hz": 20,
            },
            "world": {"stop_on_out_of_bounds": True, "out_of_bounds_fail_delay_s": 5.0},
        },
        "controller": {
            "controller": {"type": "manual", "robot_mode": False},
            "controller_params": {"manual": {}},
            "input": {"state_timeout_ms": 500, "require_all_tracked_for_valid_state": False},
            "recording": {"enable": True},
            "plot": {
                "enable_after_experiment": True,
                "plot_trajectory": True,
                "plot_pose_time_series": True,
                "plot_command_time_series": True,
            },
        },
        "supervisor": {
            "use_optitrack": True,
            "use_robot": True,
            "use_controller": True,
            "shutdown_timeout_s": 3.0,
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

    def fake_load_all_configs():
        validate_configs(broken_configs)
        return broken_configs

    monkeypatch.setattr("src.supervisor.supervisor.get_project_root", lambda: None)
    monkeypatch.setattr("src.supervisor.supervisor.load_all_configs", fake_load_all_configs)

    with pytest.raises(ConfigError, match="controller.robot_mode"):
        Supervisor()


def test_supervisor_start_and_shutdown_order():
    assert [spec.name for spec in Supervisor.START_ORDER] == ["optitrack", "robot", "controller"]
    assert Supervisor.SHUTDOWN_ORDER == ["controller", "robot", "optitrack"]
    assert Supervisor.TASK_COMPLETED_SHUTDOWN_ORDER == ["robot", "optitrack"]


class FakeSubscriber:
    def __init__(self, messages):
        self.messages = list(messages)

    def receive(self, timeout_ms=0):
        if not self.messages:
            return None
        return self.messages.pop(0)


def test_supervisor_auto_stops_on_controller_failed_when_untracked_stop_enabled():
    supervisor = Supervisor.__new__(Supervisor)
    supervisor.system_config = {
        "experiment": {
            "auto_stop_on_task_completed": False,
            "auto_stop_on_untracked": True,
        }
    }
    supervisor.status_subscribers = {
        "control_command": FakeSubscriber(
            [
                (
                    MODULE_STATUS,
                    ModuleStatus("controller", "failed", "tracking_lost: untracked robots=['robot_2']", 1.0).__dict__,
                )
            ]
        )
    }

    assert supervisor._auto_stop_requested() is True


def test_supervisor_caches_optitrack_running_when_ready_via_world_state(monkeypatch):
    supervisor = Supervisor.__new__(Supervisor)
    supervisor.supervisor_config = {"optitrack_ready_timeout_s": 1.0}
    supervisor.status_subscribers = {"world_state": FakeSubscriber([(WORLD_STATE, {"frame_id": 1})])}
    supervisor._pending_startup_statuses = []
    supervisor.logger = type("Logger", (), {"info": lambda *args, **kwargs: None})()
    supervisor.manager = type(
        "Manager",
        (),
        {
            "is_running": lambda self, name: True,
            "returncode": lambda self, name: None,
        },
    )()
    monkeypatch.setattr("src.supervisor.supervisor.now_s", lambda: 123.0)

    assert supervisor._wait_until_ready(Supervisor.START_ORDER[0]) is True
    assert len(supervisor._pending_startup_statuses) == 1
    status = supervisor._pending_startup_statuses[0]
    assert status.module_name == "optitrack"
    assert status.status == "running"
    assert status.timestamp == 123.0


def test_supervisor_records_shutdown_status_to_latest_experiment(tmp_path):
    output_dir = tmp_path / "experiments"
    old_dir = output_dir / "20260515_100000_demo"
    latest_dir = output_dir / "20260515_110000_demo"
    old_dir.mkdir(parents=True)
    latest_dir.mkdir(parents=True)
    for directory in [old_dir, latest_dir]:
        with (directory / "system_status.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["timestamp", "module_name", "status", "message"])
            writer.writeheader()

    supervisor = Supervisor.__new__(Supervisor)
    supervisor.root = tmp_path
    supervisor.system_config = {"experiment_name": "demo"}
    supervisor.controller_config = {"recording": {"output_dir": str(output_dir)}}
    supervisor._controller_started = True
    supervisor._pending_startup_statuses = []
    supervisor._pending_shutdown_statuses = [supervisor._shutdown_status("robot", 0)]

    supervisor._record_supervisor_statuses()

    with (latest_dir / "system_status.csv").open(newline="", encoding="utf-8") as file:
        latest_rows = list(csv.DictReader(file))
    with (old_dir / "system_status.csv").open(newline="", encoding="utf-8") as file:
        old_rows = list(csv.DictReader(file))

    assert old_rows == []
    assert latest_rows[-1]["module_name"] == "robot"
    assert latest_rows[-1]["status"] == "stopped"

