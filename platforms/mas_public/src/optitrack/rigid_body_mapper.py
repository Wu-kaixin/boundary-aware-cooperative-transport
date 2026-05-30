from __future__ import annotations


class RigidBodyMapper:
    """Map Motive rigid-body names or ids to project robot ids."""

    def __init__(self, robots_config: dict):
        self.rigid_to_robot = {
            item["rigid_body_name"]: item["robot_id"] for item in robots_config["robots"]["list"]
        }
        self.id_to_robot = {
            int(item["rigid_body_id"]): item["robot_id"]
            for item in robots_config["robots"]["list"]
            if item.get("rigid_body_id") is not None
        }

    def robot_id_for(self, rigid_body_name: str, rigid_body_id: int | None = None) -> str | None:
        if rigid_body_id is not None and rigid_body_id in self.id_to_robot:
            return self.id_to_robot[rigid_body_id]
        return self.rigid_to_robot.get(rigid_body_name)

    def expected_names(self) -> list[str]:
        return list(self.rigid_to_robot.keys())
