from __future__ import annotations

from dataclasses import dataclass

from src.common.math_utils import wrap_angle_rad


@dataclass
class PreviousPose:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float


class StateEstimator:
    """根据相邻帧位姿估计速度；缺帧或时间异常时返回零速度。 / Estimate velocity from adjacent poses."""

    def __init__(self):
        self.previous: dict[str, PreviousPose] = {}

    def estimate(
        self, robot_id: str, x: float, y: float, z: float, yaw: float, timestamp: float
    ) -> tuple[float, float, float, float]:
        prev = self.previous.get(robot_id)
        self.previous[robot_id] = PreviousPose(x, y, z, yaw, timestamp)
        if prev is None:
            return 0.0, 0.0, 0.0, 0.0
        dt = timestamp - prev.timestamp
        if dt <= 1e-6 or dt > 1.0:
            return 0.0, 0.0, 0.0, 0.0
        return (
            (x - prev.x) / dt,
            (y - prev.y) / dt,
            (z - prev.z) / dt,
            wrap_angle_rad(yaw - prev.yaw) / dt,
        )
