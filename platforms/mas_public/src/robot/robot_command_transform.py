from __future__ import annotations

from src.common.messages import RobotCommand


class RobotCommandTransform:
    """Map internal robot commands to the hardware command frame."""

    def __init__(self, config: dict | None = None):
        self.config = config or {}
        self.enabled = bool(self.config.get("enabled", False))
        linear_config = self.config.get("linear", {})
        angular_config = self.config.get("angular", {})
        gimbal_transform_config = self.config.get("gimbal", {})
        self.linear_dimension = int(linear_config.get("dimension", 2))
        self.linear_matrix = linear_config.get("matrix", [[1.0, 0.0], [0.0, 1.0]])
        self.angular_dimension = int(angular_config.get("dimension", 1))
        self.angular_scale = float(angular_config.get("scale", 1.0))
        self.gimbal_yaw_scale = float(gimbal_transform_config.get("yaw_scale", 1.0))
        self._validate()

    def apply(self, command: RobotCommand) -> RobotCommand:
        if not self.enabled:
            return command
        vx, vy = self._transform_linear(command.chassis_vx, command.chassis_vy)
        wz = None if command.chassis_wz is None else self.angular_scale * command.chassis_wz
        gimbal_yaw_speed = (
            None if command.gimbal_yaw_speed is None else self.gimbal_yaw_scale * command.gimbal_yaw_speed
        )
        return RobotCommand(
            robot_id=command.robot_id,
            chassis_vx=vx,
            chassis_vy=vy,
            chassis_wz=wz,
            gimbal_yaw_speed=gimbal_yaw_speed,
            gimbal_pitch_speed=command.gimbal_pitch_speed,
            controller_mode=command.controller_mode,
        )

    def _transform_linear(self, vx: float | None, vy: float | None) -> tuple[float | None, float | None]:
        if vx is None and vy is None:
            return None, None
        matrix = self.linear_matrix
        vx_value = 0.0 if vx is None else vx
        vy_value = 0.0 if vy is None else vy
        return (
            float(matrix[0][0]) * vx_value + float(matrix[0][1]) * vy_value,
            float(matrix[1][0]) * vx_value + float(matrix[1][1]) * vy_value,
        )

    def _validate(self) -> None:
        if self.linear_dimension != 2:
            raise ValueError("robot_command_transform.linear.dimension currently only supports 2")
        if len(self.linear_matrix) != 2 or any(len(row) != 2 for row in self.linear_matrix):
            raise ValueError("robot_command_transform.linear.matrix must be a 2x2 matrix")
        if self.angular_dimension != 1:
            raise ValueError("robot_command_transform.angular.dimension currently only supports 1")

