from __future__ import annotations

from src.common.messages import WorldState


def out_of_bounds_robot_ids(world_state: WorldState | None, world_config: dict) -> list[str]:
    """检查 tracked 机器人是否超出实验场地范围。 / Return tracked robots outside the configured world bounds."""
    if world_state is None:
        return []
    bounds = (
        float(world_config["x_min"]),
        float(world_config["x_max"]),
        float(world_config["y_min"]),
        float(world_config["y_max"]),
        float(world_config["z_min"]),
        float(world_config["z_max"]),
    )
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    robot_ids: list[str] = []
    for robot in world_state.robots:
        if not robot.tracked:
            continue
        if not (x_min <= robot.x <= x_max and y_min <= robot.y <= y_max and z_min <= robot.z <= z_max):
            robot_ids.append(robot.robot_id)
    return robot_ids
