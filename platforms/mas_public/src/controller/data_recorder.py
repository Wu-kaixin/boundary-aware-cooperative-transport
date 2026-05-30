from __future__ import annotations

import csv
from pathlib import Path

from src.common.messages import ControlCommand, ModuleStatus, RobotStatus, WorldState


class DataRecorder:
    """Internal module."""

    def __init__(self, experiment_dir: str | Path):
        self.experiment_dir = Path(experiment_dir)
        self.experiment_dir.mkdir(parents=True, exist_ok=True)
        self._files = []

        self.world_file = (self.experiment_dir / "world_state.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.world_zup_file = (self.experiment_dir / "world_state_zup.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.command_file = (self.experiment_dir / "control_command.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.status_file = (self.experiment_dir / "system_status.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.robot_status_file = (self.experiment_dir / "robot_status.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self._files.extend(
            [self.world_file, self.world_zup_file, self.command_file, self.status_file, self.robot_status_file]
        )

        self.world_writer = csv.DictWriter(
            self.world_file,
            fieldnames=[
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
            ],
        )
        self.world_zup_writer = csv.DictWriter(self.world_zup_file, fieldnames=self.world_writer.fieldnames)
        self.command_writer = csv.DictWriter(
            self.command_file,
            fieldnames=[
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
        )
        self.status_writer = csv.DictWriter(
            self.status_file, fieldnames=["timestamp", "module_name", "status", "message"]
        )
        self.robot_status_writer = csv.DictWriter(
            self.robot_status_file,
            fieldnames=[
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
        )
        self.world_writer.writeheader()
        self.world_zup_writer.writeheader()
        self.command_writer.writeheader()
        self.status_writer.writeheader()
        self.robot_status_writer.writeheader()

    def record_world_state(self, state: WorldState) -> None:
        for robot in state.robots:
            self.world_writer.writerow(
                {
                    "timestamp": state.timestamp,
                    "frame_id": state.frame_id,
                    "robot_id": robot.robot_id,
                    "tracked": robot.tracked,
                    "x": robot.x,
                    "y": robot.y,
                    "z": robot.z,
                    "roll": robot.roll,
                    "pitch": robot.pitch,
                    "yaw": robot.yaw,
                    "vx": robot.vx,
                    "vy": robot.vy,
                    "vz": robot.vz,
                    "wz": robot.wz,
                }
            )

    def record_world_state_zup(self, state: WorldState) -> None:
        for robot in state.robots:
            self.world_zup_writer.writerow(
                {
                    "timestamp": state.timestamp,
                    "frame_id": state.frame_id,
                    "robot_id": robot.robot_id,
                    "tracked": robot.tracked,
                    "x": robot.x,
                    "y": robot.y,
                    "z": robot.z,
                    "roll": robot.roll,
                    "pitch": robot.pitch,
                    "yaw": robot.yaw,
                    "vx": robot.vx,
                    "vy": robot.vy,
                    "vz": robot.vz,
                    "wz": robot.wz,
                }
            )

    def record_control_command(self, command: ControlCommand) -> None:
        for robot_cmd in command.commands:
            row = {key: ("" if value is None else value) for key, value in robot_cmd.__dict__.items()}
            self.command_writer.writerow(
                {
                    "timestamp": command.timestamp,
                    "robot_mode": command.robot_mode,
                    **row,
                }
            )

    def record_module_status(self, status: ModuleStatus) -> None:
        self.status_writer.writerow(status.__dict__)
        self.status_file.flush()

    def record_robot_status(self, status: RobotStatus) -> None:
        self.robot_status_writer.writerow(status.__dict__)

    def flush(self) -> None:
        for file in self._files:
            file.flush()

    def close(self) -> None:
        self.flush()
        for file in self._files:
            file.close()


