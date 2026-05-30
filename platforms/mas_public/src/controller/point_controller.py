from __future__ import annotations

import math
from typing import Any

from src.common.math_utils import clamp, wrap_angle_rad
from src.common.messages import ControlCommand, RobotCommand, RobotState, WorldState
from src.common.time_utils import now_s
from src.controller.base_controller import BaseController


class PointController(BaseController):
    """Target-point xy PD controller."""

    def __init__(self, config: dict[str, Any], robot_ids: list[str], limits_config: dict[str, Any] | None = None):
        super().__init__(config, robot_ids)
        params: dict[str, Any] = config.get("controller_params", {}).get("point", {})
        chassis_limits = (limits_config or {}).get("chassis", {})
        self.targets = params.get("targets", {})
        self.kp_x = float(params.get("kp_x", 0.5))
        self.kd_x = float(params.get("kd_x", 0.0))
        self.max_vx = float(chassis_limits.get("max_vx", params.get("max_vx", 0.4)))
        self.kp_y = float(params.get("kp_y", 0.5))
        self.kd_y = float(params.get("kd_y", 0.0))
        self.max_vy = float(chassis_limits.get("max_vy", params.get("max_vy", 0.4)))
        self.kp_yaw = float(params.get("kp_yaw", 0.0))
        self.kd_yaw = float(params.get("kd_yaw", 0.0))
        self.max_wz = float(chassis_limits.get("max_wz", params.get("max_wz", 0.0)))
        self.position_tolerance_m = float(params.get("position_tolerance_m", 0.0))
        self.yaw_tolerance_rad = float(params.get("yaw_tolerance_rad", 0.0))
        self.hold_enabled = bool(params.get("hold_enabled", False))
        self.hold_kp_x = float(params.get("hold_kp_x", self.kp_x))
        self.hold_kd_x = float(params.get("hold_kd_x", self.kd_x))
        self.hold_kp_y = float(params.get("hold_kp_y", self.kp_y))
        self.hold_kd_y = float(params.get("hold_kd_y", self.kd_y))
        self.hold_kp_yaw = float(params.get("hold_kp_yaw", self.kp_yaw))
        self.hold_kd_yaw = float(params.get("hold_kd_yaw", self.kd_yaw))
        self.hold_max_vx = float(params.get("hold_max_vx", self.max_vx))
        self.hold_max_vy = float(params.get("hold_max_vy", self.max_vy))
        self.hold_max_wz = float(params.get("hold_max_wz", self.max_wz))
        self.hold_duration_s = float(params.get("hold_duration_s", 0.5))
        self.completed_robot_ids: set[str] = set()
        self.hold_since_by_id: dict[str, float] = {}
        self.hold_finished_robot_ids: set[str] = set()

    def task_completed(self) -> bool:
        completed = set(self.robot_ids).issubset(self.completed_robot_ids)
        if not self.hold_enabled:
            return completed
        return completed and set(self.robot_ids).issubset(self.hold_finished_robot_ids)

    def compute(self, world_state: WorldState | None) -> ControlCommand:
        if world_state is None:
            return self._zero_command()

        state_by_id: dict[str, RobotState] = {robot.robot_id: robot for robot in world_state.robots}
        commands: list[RobotCommand] = []
        for robot_id in self.robot_ids:
            state = state_by_id.get(robot_id)
            target = self.targets.get(robot_id, {"x": 0.0, "y": 0.0, "yaw": 0.0})
            if robot_id in self.completed_robot_ids:
                commands.append(self._completed_command(robot_id, target, state))
                continue
            if state is None or not state.tracked:
                self.hold_since_by_id.pop(robot_id, None)
                commands.append(self._robot_zero(robot_id, "point_untracked"))
                continue

            control_state = state
            dx = float(target.get("x", 0.0)) - control_state.x
            dy = float(target.get("y", 0.0)) - control_state.y
            distance = math.hypot(dx, dy)

            if distance <= self.position_tolerance_m:
                vx_body = 0.0
                vy_body = 0.0
            else:
                ux_world = self.kp_x * dx - self.kd_x * control_state.vx
                uy_world = self.kp_y * dy - self.kd_y * control_state.vy
                vx_body, vy_body = self._world_velocity_to_body(ux_world, uy_world, control_state.yaw)
                vx_body = clamp(vx_body, -self.max_vx, self.max_vx)
                vy_body = clamp(vy_body, -self.max_vy, self.max_vy)

            yaw_error = self._yaw_error(target, control_state)
            wz = self._yaw_control_wz(yaw_error, control_state)
            yaw_reached = abs(yaw_error) <= self.yaw_tolerance_rad
            position_reached = distance <= self.position_tolerance_m
            controller_mode = "point_completed" if position_reached and yaw_reached else "point"
            if controller_mode == "point_completed":
                self.completed_robot_ids.add(robot_id)
                self.hold_since_by_id[robot_id] = control_state.timestamp
                vx_body = 0.0
                vy_body = 0.0
                wz = 0.0
                if self.hold_enabled:
                    vx_body, vy_body, wz = self._hold_command(dx, dy, yaw_error, control_state)
                else:
                    self.hold_finished_robot_ids.add(robot_id)
            commands.append(
                RobotCommand(
                    robot_id=robot_id,
                    chassis_vx=vx_body,
                    chassis_vy=vy_body,
                    chassis_wz=wz,
                    gimbal_yaw_speed=0.0,
                    gimbal_pitch_speed=0.0,
                    controller_mode=controller_mode,
                )
            )
        return ControlCommand(
            timestamp=now_s(),
            robot_mode=self.robot_mode,
            commands=commands,
        )

    def _yaw_error(self, target: dict[str, Any], state: RobotState) -> float:
        target_yaw = float(target.get("yaw", 0.0))
        return wrap_angle_rad(target_yaw - state.yaw)

    def _yaw_control_wz(self, yaw_error: float, state: RobotState) -> float:
        wz = self.kp_yaw * yaw_error - self.kd_yaw * state.wz
        return clamp(wz, -self.max_wz, self.max_wz)

    def _hold_command(
        self, dx: float, dy: float, yaw_error: float, state: RobotState
    ) -> tuple[float, float, float]:
        ux_world = self.hold_kp_x * dx - self.hold_kd_x * state.vx
        uy_world = self.hold_kp_y * dy - self.hold_kd_y * state.vy
        vx_body, vy_body = self._world_velocity_to_body(ux_world, uy_world, state.yaw)
        wz = self.hold_kp_yaw * yaw_error - self.hold_kd_yaw * state.wz
        return (
            clamp(vx_body, -self.hold_max_vx, self.hold_max_vx),
            clamp(vy_body, -self.hold_max_vy, self.hold_max_vy),
            clamp(wz, -self.hold_max_wz, self.hold_max_wz),
        )

    def _completed_command(
        self, robot_id: str, target: dict[str, Any], state: RobotState | None
    ) -> RobotCommand:
        if not self.hold_enabled or robot_id in self.hold_finished_robot_ids or state is None or not state.tracked:
            self.hold_finished_robot_ids.add(robot_id)
            return self._robot_zero(robot_id, "point_completed")
        hold_since = self.hold_since_by_id.setdefault(robot_id, state.timestamp)
        if state.timestamp - hold_since >= self.hold_duration_s:
            self.hold_finished_robot_ids.add(robot_id)
            return self._robot_zero(robot_id, "point_completed")
        dx = float(target.get("x", 0.0)) - state.x
        dy = float(target.get("y", 0.0)) - state.y
        yaw_error = self._yaw_error(target, state)
        vx_body, vy_body, wz = self._hold_command(dx, dy, yaw_error, state)
        return RobotCommand(robot_id, vx_body, vy_body, wz, 0.0, 0.0, "point_completed")

    @staticmethod
    def _world_velocity_to_body(ux_world: float, uy_world: float, yaw: float) -> tuple[float, float]:
        return (
            math.cos(yaw) * ux_world + math.sin(yaw) * uy_world,
            -math.sin(yaw) * ux_world + math.cos(yaw) * uy_world,
        )

    def _zero_command(self) -> ControlCommand:
        return ControlCommand(
            timestamp=now_s(),
            robot_mode=self.robot_mode,
            commands=[self._robot_zero(robot_id, "point_zero") for robot_id in self.robot_ids],
        )

    @staticmethod
    def _robot_zero(robot_id: str, mode: str) -> RobotCommand:
        return RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, mode)

