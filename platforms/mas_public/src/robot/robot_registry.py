from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RobotInfo:
    robot_id: str
    sn: str
    group: str
    rigid_body_name: str
    chassis_enabled: bool
    gimbal_enabled: bool
    ip: str | None = None
    rigid_body_id: int | None = None


class RobotRegistry:
    """集中管理机器人配置映射，避免硬编码 robot_id/SN/IP。 / Central registry for robot config mappings."""

    def __init__(self, robots_config: dict):
        self.robots = [RobotInfo(**item) for item in robots_config["robots"]["list"]]
        self.by_id = {robot.robot_id: robot for robot in self.robots}

    def get(self, robot_id: str) -> RobotInfo:
        return self.by_id[robot_id]

    def ids(self) -> list[str]:
        return list(self.by_id.keys())
