from __future__ import annotations

"""MAS-public controller adapter for DBACT.

Copy this file into MAS-public/src/controller/ or import it from this package.
It follows the style of MAS-public CVTController:

    compute(WorldState | None) -> ControlCommand

The controller currently uses ObjectObserver. Until perception is implemented,
set virtual_object.enabled=true in dtransport.yaml for lab debugging.
"""

import math
from typing import Any

import numpy as np

from dbact.controller import DBACTController, DBACTParams
from dbact.types import AgentState
from mas_adapter.object_observer import ObjectObserver

try:  # MAS-public imports, available only inside MAS project.
    from src.common.math_utils import clamp, wrap_angle_rad
    from src.common.messages import ControlCommand, RobotCommand, RobotState, WorldState
    from src.common.time_utils import now_s
    from src.controller.base_controller import BaseController
except Exception:  # pragma: no cover - lets this module be imported outside MAS.
    BaseController = object  # type: ignore
    ControlCommand = RobotCommand = RobotState = WorldState = None  # type: ignore

    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    def wrap_angle_rad(angle: float) -> float:
        while angle >= math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def now_s() -> float:
        import time
        return time.monotonic()


class DecentralizedTransportController(BaseController):  # type: ignore[misc]
    """DBACT controller wrapper for MAS-public."""

    def __init__(
        self,
        config: dict[str, Any],
        robot_ids: list[str],
        world_config: dict[str, Any],
        limits_config: dict[str, Any] | None = None,
    ):
        if hasattr(super(), "__init__"):
            try:
                super().__init__(config, robot_ids)
            except TypeError:
                pass
        self.config = config
        self.robot_ids = robot_ids
        self.robot_mode = str(config.get("controller", {}).get("robot_mode", "free"))
        params: dict[str, Any] = config.get("controller_params", {}).get("dtransport", config.get("controller_params", {}))
        chassis_limits = (limits_config or {}).get("chassis", {})
        self.max_vx = float(chassis_limits.get("max_vx", params.get("max_speed", 0.18)))
        self.max_vy = float(chassis_limits.get("max_vy", params.get("max_speed", 0.18)))
        self.max_wz = float(chassis_limits.get("max_wz", params.get("max_wz", 0.6)))
        self.kp_yaw = float(params.get("kp_yaw", 0.8))
        self.target_yaw = float(params.get("target_yaw", 0.0))
        self.yaw_mode = str(params.get("yaw_mode", "face_velocity"))

        domain = self._domain_from_world_config(world_config)
        self.dbact = DBACTController(DBACTParams.from_dict(params), domain)
        self.object_observer = ObjectObserver(params)
    
    def compute_planar_velocities(self, world_state: Any | None) -> dict[str, np.ndarray]:
        """Compute world-frame planar velocities without MAS message classes.

        This method is used for unit tests and mock integration before running
        inside MAS-public. It verifies the pipeline:

            mock WorldState -> DBACTController -> planar velocity commands

        The returned velocities are in the world frame.
        """
        if world_state is None:
            return {rid: np.zeros(2, dtype=float) for rid in self.robot_ids}

        state_by_id = {robot.robot_id: robot for robot in world_state.robots}

        tracked = [
            state_by_id[rid]
            for rid in self.robot_ids
            if rid in state_by_id and state_by_id[rid].tracked
        ]

        if not tracked:
            return {rid: np.zeros(2, dtype=float) for rid in self.robot_ids}

        agents = [
            AgentState(
                agent_id=s.robot_id,
                position=np.array([s.x, s.y], dtype=float),
                velocity=np.array([s.vx, s.vy], dtype=float),
                yaw=s.yaw,
            )
            for s in tracked
        ]

        cargoes = self.object_observer.observe()

        commands = self.dbact.step(
            agents,
            cargoes,
            float(world_state.timestamp),
            dt=0.05,
        )

        velocity_by_id = {
            cmd.agent_id: cmd.velocity
            for cmd in commands
        }

        return {
            rid: velocity_by_id.get(rid, np.zeros(2, dtype=float))
            for rid in self.robot_ids
        }

    def compute(self, world_state: Any | None) -> Any:
        if ControlCommand is None or RobotCommand is None:
            raise RuntimeError("This adapter must be run inside MAS-public to produce MAS messages.")
        if world_state is None:
            return self._zero_command("dtransport_zero")

        state_by_id = {robot.robot_id: robot for robot in world_state.robots}
        tracked = [state_by_id[rid] for rid in self.robot_ids if rid in state_by_id and state_by_id[rid].tracked]
        if not tracked:
            return self._zero_command("dtransport_untracked")

        agents = [AgentState(s.robot_id, np.array([s.x, s.y]), np.array([s.vx, s.vy]), s.yaw) for s in tracked]
        cargoes = self.object_observer.observe()
        commands = self.dbact.step(agents, cargoes, float(world_state.timestamp), dt=0.05)
        by_id = {cmd.agent_id: cmd for cmd in commands}

        robot_commands = []
        for rid in self.robot_ids:
            state = state_by_id.get(rid)
            cmd = by_id.get(rid)
            if state is None or not state.tracked or cmd is None:
                robot_commands.append(self._robot_zero(rid, "dtransport_untracked"))
                continue
            robot_commands.append(self._to_mas_command(state, cmd.velocity, cmd.mode))
        return ControlCommand(timestamp=now_s(), robot_mode=self.robot_mode, commands=robot_commands)

    def _to_mas_command(self, state: Any, velocity_world: np.ndarray, mode: str) -> Any:
        vx_world, vy_world = float(velocity_world[0]), float(velocity_world[1])
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
        wz = 0.0
        if self.yaw_mode == "fixed":
            wz = self.kp_yaw * wrap_angle_rad(self.target_yaw - state.yaw)
        elif self.yaw_mode == "face_velocity" and math.hypot(vx_world, vy_world) > 1e-6:
            target_yaw = math.atan2(vy_world, vx_world)
            wz = self.kp_yaw * wrap_angle_rad(target_yaw - state.yaw)
        return RobotCommand(
            robot_id=state.robot_id,
            chassis_vx=clamp(vx_body, -self.max_vx, self.max_vx),
            chassis_vy=clamp(vy_body, -self.max_vy, self.max_vy),
            chassis_wz=clamp(wz, -self.max_wz, self.max_wz),
            gimbal_yaw_speed=0.0,
            gimbal_pitch_speed=0.0,
            controller_mode=mode,
        )

    def _zero_command(self, mode: str) -> Any:
        return ControlCommand(timestamp=now_s(), robot_mode=self.robot_mode, commands=[self._robot_zero(rid, mode) for rid in self.robot_ids])

    @staticmethod
    def _robot_zero(robot_id: str, mode: str) -> Any:
        return RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, mode)

    @staticmethod
    def _domain_from_world_config(world_config: dict[str, Any]) -> tuple[float, float, float, float]:
        # MAS world config names may differ between versions; support common keys.
        xmin = float(world_config.get("xmin", world_config.get("x_min", -2.0)))
        xmax = float(world_config.get("xmax", world_config.get("x_max", 2.0)))
        ymin = float(world_config.get("ymin", world_config.get("y_min", -2.0)))
        ymax = float(world_config.get("ymax", world_config.get("y_max", 2.0)))
        bounds = world_config.get("bounds")
        if isinstance(bounds, dict):
            xmin = float(bounds.get("xmin", bounds.get("x_min", xmin)))
            xmax = float(bounds.get("xmax", bounds.get("x_max", xmax)))
            ymin = float(bounds.get("ymin", bounds.get("y_min", ymin)))
            ymax = float(bounds.get("ymax", bounds.get("y_max", ymax)))
        return xmin, xmax, ymin, ymax
