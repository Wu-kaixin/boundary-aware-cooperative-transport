from __future__ import annotations

from typing import Any

from src.common.messages import ControlCommand, RobotCommand, WorldState
from src.common.time_utils import now_s
from src.controller.base_controller import BaseController


class ManualController(BaseController):
    """Internal module."""

    def __init__(self, config: dict[str, Any], robot_ids: list[str]):
        super().__init__(config, robot_ids)
        params: dict[str, Any] = config.get("controller_params", {}).get("manual", {})
        self.vx = float(params.get("chassis_vx", 0.0))
        self.vy = float(params.get("chassis_vy", 0.0))
        self.wz = float(params.get("chassis_wz", 0.0))
        self.gimbal_yaw_speed = float(params.get("gimbal_yaw_speed", 0.0))
        self.gimbal_pitch_speed = float(params.get("gimbal_pitch_speed", 0.0))

    def compute(self, world_state: WorldState | None) -> ControlCommand:
        commands = [
            RobotCommand(
                robot_id=robot_id,
                chassis_vx=self.vx,
                chassis_vy=self.vy,
                chassis_wz=self.wz,
                gimbal_yaw_speed=self.gimbal_yaw_speed,
                gimbal_pitch_speed=self.gimbal_pitch_speed,
                controller_mode="manual",
            )
            for robot_id in self.robot_ids
        ]
        return ControlCommand(
            timestamp=now_s(),
            robot_mode=self.robot_mode,
            commands=commands,
        )

