from __future__ import annotations

import math
from typing import Any

import numpy as np

from src.common.math_utils import clamp, wrap_angle_rad
from src.common.messages import ControlCommand, RobotCommand, RobotState, WorldState
from src.common.time_utils import now_s
from src.controller.base_controller import BaseController
from src.controller.cvt_utils import compute_grid_cvt


class CVTController(BaseController):
    """Grid-approximated Centroidal Voronoi coverage controller."""

    def __init__(
        self,
        config: dict[str, Any],
        robot_ids: list[str],
        world_config: dict[str, Any],
        limits_config: dict[str, Any] | None = None,
    ):
        super().__init__(config, robot_ids)
        params: dict[str, Any] = config.get("controller_params", {}).get("cvt", {})
        chassis_limits = (limits_config or {}).get("chassis", {})
        self.world_config = world_config
        self.kp_x = float(params.get("kp_x", 0.4))
        self.kd_x = float(params.get("kd_x", 0.0))
        self.kp_y = float(params.get("kp_y", 0.4))
        self.kd_y = float(params.get("kd_y", 0.0))
        self.kp_yaw = float(params.get("kp_yaw", 0.8))
        self.kd_yaw = float(params.get("kd_yaw", 0.0))
        self.max_vx = float(chassis_limits.get("max_vx", params.get("max_v", 0.15)))
        self.max_vy = float(chassis_limits.get("max_vy", params.get("max_v", 0.15)))
        self.max_wz = float(chassis_limits.get("max_wz", params.get("max_wz", 0.3)))
        self.grid_resolution = int(params.get("grid_resolution", 80))
        yaw_config = params.get("yaw", {})
        self.yaw_mode = str(yaw_config.get("mode", params.get("yaw_mode", "face_velocity")))
        self.target_yaw = float(yaw_config.get("target", params.get("target_yaw", 0.0)))
        self.centroid_tolerance_m = float(params.get("centroid_tolerance_m", 0.03))
        self.hold_enabled = bool(params.get("hold_enabled", False))
        self.hold_duration_s = float(params.get("hold_duration_s", 0.5))
        self.hold_kp_x = float(params.get("hold_kp_x", self.kp_x))
        self.hold_kd_x = float(params.get("hold_kd_x", self.kd_x))
        self.hold_kp_y = float(params.get("hold_kp_y", self.kp_y))
        self.hold_kd_y = float(params.get("hold_kd_y", self.kd_y))
        self.hold_kp_yaw = float(params.get("hold_kp_yaw", self.kp_yaw))
        self.hold_kd_yaw = float(params.get("hold_kd_yaw", self.kd_yaw))
        self.hold_max_vx = float(params.get("hold_max_vx", self.max_vx))
        self.hold_max_vy = float(params.get("hold_max_vy", self.max_vy))
        self.hold_max_wz = float(params.get("hold_max_wz", self.max_wz))
        self.completed = False
        self.hold_started_at: float | None = None
        self.hold_finished = False

    def task_completed(self) -> bool:
        if not self.completed:
            return False
        if not self.hold_enabled:
            return True
        return self.hold_finished

    def compute(self, world_state: WorldState | None) -> ControlCommand:
        if world_state is None:
            return self._zero_command("cvt_zero")

        state_by_id: dict[str, RobotState] = {robot.robot_id: robot for robot in world_state.robots}
        tracked_states = [
            state_by_id[robot_id]
            for robot_id in self.robot_ids
            if robot_id in state_by_id and state_by_id[robot_id].tracked
        ]
        if not tracked_states:
            return self._zero_command("cvt_zero")

        centroid_by_id = self._centroids_for_states(tracked_states)
        if not self.completed and self._all_tracked_states_at_centroids(tracked_states, centroid_by_id):
            self.completed = True
            self.hold_started_at = world_state.timestamp
            if not self.hold_enabled:
                self.hold_finished = True

        if self.completed and self.hold_enabled and not self.hold_finished:
            hold_started_at = self.hold_started_at if self.hold_started_at is not None else world_state.timestamp
            if world_state.timestamp - hold_started_at >= self.hold_duration_s:
                self.hold_finished = True

        commands: list[RobotCommand] = []
        for robot_id in self.robot_ids:
            state = state_by_id.get(robot_id)
            centroid = centroid_by_id.get(robot_id)
            if state is None or not state.tracked or centroid is None:
                commands.append(self._robot_zero(robot_id, "cvt_untracked"))
                continue
            if self.completed and self.hold_finished:
                commands.append(self._robot_zero(robot_id, "cvt_completed"))
                continue
            if self.completed:
                commands.append(self._command_to_centroid(state, centroid, hold=True, controller_mode="cvt_completed"))
                continue
            commands.append(self._command_to_centroid(state, centroid, hold=False, controller_mode="cvt"))
        return ControlCommand(
            timestamp=now_s(),
            robot_mode=self.robot_mode,
            commands=commands,
        )

    def _centroids_for_states(self, tracked_states: list[RobotState]) -> dict[str, np.ndarray]:
        points = np.array([[state.x, state.y] for state in tracked_states], dtype=float)
        cvt = compute_grid_cvt(points, self.world_config, self.grid_resolution)
        return {state.robot_id: cvt.centroids[index] for index, state in enumerate(tracked_states)}

    def _all_tracked_states_at_centroids(
        self, tracked_states: list[RobotState], centroid_by_id: dict[str, np.ndarray]
    ) -> bool:
        if len(tracked_states) != len(self.robot_ids):
            return False
        return all(
            self._distance_to_centroid(state, centroid_by_id[state.robot_id]) <= self.centroid_tolerance_m
            for state in tracked_states
        )

    @staticmethod
    def _distance_to_centroid(state: RobotState, centroid: np.ndarray) -> float:
        return math.hypot(float(centroid[0]) - state.x, float(centroid[1]) - state.y)

    def _command_to_centroid(
        self, state: RobotState, centroid: np.ndarray, hold: bool, controller_mode: str
    ) -> RobotCommand:
        dx = float(centroid[0]) - state.x
        dy = float(centroid[1]) - state.y
        kp_x = self.hold_kp_x if hold else self.kp_x
        kd_x = self.hold_kd_x if hold else self.kd_x
        kp_y = self.hold_kp_y if hold else self.kp_y
        kd_y = self.hold_kd_y if hold else self.kd_y
        max_vx = self.hold_max_vx if hold else self.max_vx
        max_vy = self.hold_max_vy if hold else self.max_vy
        max_wz = self.hold_max_wz if hold else self.max_wz
        vx_world = kp_x * dx - kd_x * state.vx
        vy_world = kp_y * dy - kd_y * state.vy

        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world

        wz = 0.0
        if self.yaw_mode == "fixed":
            kp_yaw = self.hold_kp_yaw if hold else self.kp_yaw
            kd_yaw = self.hold_kd_yaw if hold else self.kd_yaw
            wz = kp_yaw * wrap_angle_rad(self.target_yaw - state.yaw) - kd_yaw * state.wz
        elif self.yaw_mode == "face_velocity" and math.hypot(vx_world, vy_world) > 1e-6:
            target_yaw = math.atan2(vy_world, vx_world)
            kp_yaw = self.hold_kp_yaw if hold else self.kp_yaw
            kd_yaw = self.hold_kd_yaw if hold else self.kd_yaw
            wz = kp_yaw * wrap_angle_rad(target_yaw - state.yaw) - kd_yaw * state.wz

        return RobotCommand(
            robot_id=state.robot_id,
            chassis_vx=clamp(vx_body, -max_vx, max_vx),
            chassis_vy=clamp(vy_body, -max_vy, max_vy),
            chassis_wz=clamp(wz, -max_wz, max_wz),
            gimbal_yaw_speed=0.0,
            gimbal_pitch_speed=0.0,
            controller_mode=controller_mode,
        )

    def _zero_command(self, mode: str) -> ControlCommand:
        return ControlCommand(
            timestamp=now_s(),
            robot_mode=self.robot_mode,
            commands=[self._robot_zero(robot_id, mode) for robot_id in self.robot_ids],
        )

    @staticmethod
    def _robot_zero(robot_id: str, mode: str) -> RobotCommand:
        return RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, mode)

