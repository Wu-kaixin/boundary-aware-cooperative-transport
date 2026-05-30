from __future__ import annotations

import math


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def wrap_angle_rad(angle: float) -> float:
    """将角度约束到 [-pi, pi)，便于 yaw 误差计算。 / Wrap an angle to [-pi, pi) for yaw-error math."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取 yaw，输入顺序为 x, y, z, w。 / Extract yaw from a quaternion ordered as x, y, z, w."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """从四元数提取 roll/pitch/yaw，输入顺序为 x, y, z, w。 / Extract roll, pitch, and yaw."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    yaw = quaternion_to_yaw(x, y, z, w)
    return roll, pitch, yaw
