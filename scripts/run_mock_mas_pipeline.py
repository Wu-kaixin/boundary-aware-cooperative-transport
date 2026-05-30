from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from mas_adapter.decentralized_transport_controller import DecentralizedTransportController


@dataclass
class MockRobotState:
    robot_id: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    tracked: bool = True


@dataclass
class MockWorldState:
    timestamp: float
    robots: list[MockRobotState]


def make_controller() -> DecentralizedTransportController:
    config = {
        "controller": {
            "type": "dtransport",
            "robot_mode": "free",
        },
        "controller_params": {
            "dtransport": {
                "sensor_range": 1.20,
                "comm_range": 2.40,
                "cage_offset": 0.28,
                "sigma": 0.34,
                "d_min": 0.30,
                "max_speed": 0.30,
                "kp_explore": 0.20,
                "kp_cage": 1.20,
                "kp_transport": 0.0,
                "grid_resolution": 24,
                "map_ttl": 8.0,
                "cbf_gamma": 6.0,
                "virtual_object": {
                    "enabled": True,
                    "id": "cargo_0",
                    "vertices": [
                        [3.10, 4.55],
                        [4.45, 4.30],
                        [5.10, 4.90],
                        [4.80, 5.75],
                        [3.70, 6.05],
                        [3.05, 5.30],
                    ],
                    "transport_direction": [0.0, 1.0],
                },
            }
        },
    }

    robot_ids = [
        "agent_00",
        "agent_01",
        "agent_02",
        "agent_03",
    ]

    world_config = {
        "xmin": 0.0,
        "xmax": 8.0,
        "ymin": 0.0,
        "ymax": 8.0,
    }

    limits_config = {
        "chassis": {
            "max_vx": 0.30,
            "max_vy": 0.30,
            "max_wz": 0.60,
        }
    }

    return DecentralizedTransportController(
        config=config,
        robot_ids=robot_ids,
        world_config=world_config,
        limits_config=limits_config,
    )


def make_world_state() -> MockWorldState:
    return MockWorldState(
        timestamp=0.0,
        robots=[
            MockRobotState("agent_00", 3.4, 4.0),
            MockRobotState("agent_01", 4.0, 4.0),
            MockRobotState("agent_02", 4.6, 4.0),
            MockRobotState("agent_03", 5.2, 4.4),
        ],
    )


def main() -> None:
    controller = make_controller()
    world_state = make_world_state()

    velocities = controller.compute_planar_velocities(world_state)

    print("Mock MAS pipeline:")
    print("WorldState -> DecentralizedTransportController -> planar velocities")
    print()

    for robot_id, velocity in velocities.items():
        speed = float(np.linalg.norm(velocity))
        print(
            f"{robot_id}: "
            f"vx={velocity[0]: .4f}, "
            f"vy={velocity[1]: .4f}, "
            f"speed={speed: .4f}"
        )

    print()
    print("Done.")


if __name__ == "__main__":
    main()