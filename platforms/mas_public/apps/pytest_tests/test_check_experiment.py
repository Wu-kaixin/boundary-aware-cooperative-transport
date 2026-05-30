import csv

from apps.check_experiment import check_experiment


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_final_experiment(tmp_path):
    exp = tmp_path / "20260519_120000_manual"
    snapshot = exp / "config_snapshot"
    controllers = snapshot / "controllers"
    plots = exp / "plots"
    controllers.mkdir(parents=True)
    plots.mkdir(parents=True)

    world_fields = [
        "timestamp",
        "frame_id",
        "robot_id",
        "tracked",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "vx",
        "vy",
        "vz",
        "wz",
    ]
    world_row = {
        "timestamp": "1.0",
        "frame_id": "1",
        "robot_id": "robot_1",
        "tracked": "True",
        "x": "0.0",
        "y": "0.0",
        "z": "0.0",
        "roll": "0.0",
        "pitch": "0.0",
        "yaw": "0.0",
        "vx": "0.0",
        "vy": "0.0",
        "vz": "0.0",
        "wz": "0.0",
    }
    write_csv(exp / "world_state.csv", world_fields, [world_row])
    write_csv(exp / "world_state_zup.csv", world_fields, [world_row])
    write_csv(
        exp / "control_command.csv",
        [
            "timestamp",
            "robot_mode",
            "robot_id",
            "chassis_vx",
            "chassis_vy",
            "chassis_wz",
            "gimbal_yaw_speed",
            "gimbal_pitch_speed",
            "controller_mode",
        ],
        [
            {
                "timestamp": "1.0",
                "robot_mode": "free",
                "robot_id": "robot_1",
                "chassis_vx": "0.0",
                "chassis_vy": "0.0",
                "chassis_wz": "0.0",
                "gimbal_yaw_speed": "0.0",
                "gimbal_pitch_speed": "0.0",
                "controller_mode": "manual",
            }
        ],
    )
    write_csv(
        exp / "system_status.csv",
        ["timestamp", "module_name", "status", "message"],
        [
            {"timestamp": "1.0", "module_name": "controller", "status": "running", "message": "started"},
            {"timestamp": "2.0", "module_name": "controller", "status": "stopped", "message": "stopped"},
        ],
    )
    write_csv(
        exp / "robot_status.csv",
        [
            "timestamp",
            "robot_id",
            "status_type",
            "pitch_angle",
            "yaw_angle",
            "pitch_ground_angle",
            "yaw_ground_angle",
            "requested_mode",
            "actual_mode",
        ],
        [],
    )
    for name in ["trajectory_xy.png", "pose_time_series.png", "command_time_series.png"]:
        (plots / name).write_bytes(b"png")
    (snapshot / "system.yaml").write_text(
        "network:\n  backend: zmq\nworld:\n  stop_on_out_of_bounds: true\n", encoding="utf-8"
    )
    (snapshot / "controller.yaml").write_text(
        "controller:\n  type: manual\n  robot_mode: free\n", encoding="utf-8"
    )
    for name in ["robots.yaml", "optitrack.yaml", "supervisor.yaml", "logging.yaml"]:
        (snapshot / name).write_text("{}\n", encoding="utf-8")
    (controllers / "manual.yaml").write_text("chassis_vx: 0.0\n", encoding="utf-8")
    return exp


def test_check_experiment_accepts_final_format(tmp_path):
    report = check_experiment(make_final_experiment(tmp_path))

    assert report.ok
    assert report.warnings == []
    assert report.info["controller_type"] == "manual"
    assert report.info["robot_mode"] == "free"


def test_check_experiment_rejects_legacy_command_columns(tmp_path):
    exp = make_final_experiment(tmp_path)
    write_csv(
        exp / "control_command.csv",
        [
            "timestamp",
            "coordination_mode",
            "robot_id",
            "chassis_vx",
            "chassis_vy",
            "chassis_wz",
            "gimbal_yaw_speed",
            "gimbal_pitch_speed",
            "command_mode",
        ],
        [],
    )

    report = check_experiment(exp)

    assert not report.ok
    assert any("control_command.csv columns" in error for error in report.errors)
    assert any("legacy field coordination_mode" in warning for warning in report.warnings)


def test_check_experiment_ignores_shutdown_commands_for_row_count(tmp_path):
    exp = make_final_experiment(tmp_path)
    write_csv(
        exp / "control_command.csv",
        [
            "timestamp",
            "robot_mode",
            "robot_id",
            "chassis_vx",
            "chassis_vy",
            "chassis_wz",
            "gimbal_yaw_speed",
            "gimbal_pitch_speed",
            "controller_mode",
        ],
        [
            {
                "timestamp": "1.0",
                "robot_mode": "free",
                "robot_id": "robot_1",
                "chassis_vx": "0.0",
                "chassis_vy": "0.0",
                "chassis_wz": "0.0",
                "gimbal_yaw_speed": "0.0",
                "gimbal_pitch_speed": "0.0",
                "controller_mode": "manual",
            },
            {
                "timestamp": "2.0",
                "robot_mode": "free",
                "robot_id": "robot_1",
                "chassis_vx": "0.0",
                "chassis_vy": "0.0",
                "chassis_wz": "0.0",
                "gimbal_yaw_speed": "0.0",
                "gimbal_pitch_speed": "0.0",
                "controller_mode": "shutdown",
            },
        ],
    )

    report = check_experiment(exp)

    assert report.ok
    assert "world_state.csv and control_command.csv row counts differ" not in report.warnings
