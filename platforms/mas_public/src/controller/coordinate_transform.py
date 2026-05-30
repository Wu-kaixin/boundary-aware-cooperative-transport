from __future__ import annotations

import math
from dataclasses import replace

from src.common.messages import RobotState, WorldState


class CoordinateTransformer:
    """Apply a fixed left-multiplied Rx transform to positions, velocities, and poses."""

    def __init__(self, rx_deg: float):
        self.rx_rad = math.radians(rx_deg)
        self.rx = self._rx(self.rx_rad)

    def robot_state(self, state: RobotState) -> RobotState:
        x, y, z = self._matvec(self.rx, [state.x, state.y, state.z])
        vx, vy, vz = self._matvec(self.rx, [state.vx, state.vy, state.vz])
        rotation = self._matmul(self.rx, self._rpy_to_matrix(state.roll, state.pitch, state.yaw))
        roll, pitch, yaw = self._rpy_from_matrix(rotation)
        return replace(
            state,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            vx=vx,
            vy=vy,
            vz=vz,
        )

    def world_state(self, state: WorldState | None) -> WorldState | None:
        if state is None:
            return None
        return WorldState(
            timestamp=state.timestamp,
            frame_id=state.frame_id,
            robots=[self.robot_state(robot) for robot in state.robots],
        )

    @staticmethod
    def _matvec(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float]:
        return (
            sum(matrix[0][index] * vector[index] for index in range(3)),
            sum(matrix[1][index] * vector[index] for index in range(3)),
            sum(matrix[2][index] * vector[index] for index in range(3)),
        )

    @staticmethod
    def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
        ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
        rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
        return CoordinateTransformer._matmul(CoordinateTransformer._matmul(rz, ry), rx)

    @staticmethod
    def _rpy_from_matrix(matrix: list[list[float]]) -> tuple[float, float, float]:
        pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
        return roll, pitch, yaw

    @staticmethod
    def _rx(angle: float) -> list[list[float]]:
        c, s = math.cos(angle), math.sin(angle)
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]

    @staticmethod
    def _matmul(first: list[list[float]], second: list[list[float]]) -> list[list[float]]:
        return [
            [sum(first[row][index] * second[index][column] for index in range(3)) for column in range(3)]
            for row in range(3)
        ]
