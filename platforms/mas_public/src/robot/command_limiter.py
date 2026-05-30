from __future__ import annotations

from src.common.math_utils import clamp
from src.common.messages import RobotCommand


class CommandLimiter:
    """Internal module."""

    def __init__(self, limits_config: dict):
        chassis_limits = limits_config.get("chassis", {})
        gimbal_limits = limits_config.get("gimbal", {})
        self.max_vx = float(chassis_limits.get("max_vx", 0.5))
        self.max_vy = float(chassis_limits.get("max_vy", 0.5))
        self.max_wz = float(chassis_limits.get("max_wz", 1.0))
        self.max_yaw_speed = float(gimbal_limits.get("max_yaw_speed", 120.0))
        self.max_pitch_speed = float(gimbal_limits.get("max_pitch_speed", 120.0))

    def limit(self, command: RobotCommand) -> RobotCommand:
        return RobotCommand(
            robot_id=command.robot_id,
            chassis_vx=self._limit_optional(command.chassis_vx, -self.max_vx, self.max_vx),
            chassis_vy=self._limit_optional(command.chassis_vy, -self.max_vy, self.max_vy),
            chassis_wz=self._limit_optional(command.chassis_wz, -self.max_wz, self.max_wz),
            gimbal_yaw_speed=self._limit_optional(command.gimbal_yaw_speed, -self.max_yaw_speed, self.max_yaw_speed),
            gimbal_pitch_speed=self._limit_optional(
                command.gimbal_pitch_speed, -self.max_pitch_speed, self.max_pitch_speed
            ),
            controller_mode=command.controller_mode,
        )

    @staticmethod
    def _limit_optional(value: float | None, min_value: float, max_value: float) -> float | None:
        if value is None:
            return None
        return clamp(value, min_value, max_value)

