from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

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


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def make_controller(
    controller_config_path: str | Path,
    dtransport_config_path: str | Path,
) -> DecentralizedTransportController:
    controller_cfg = load_yaml(controller_config_path)
    dtransport_cfg = load_yaml(dtransport_config_path)

    config = {
        "controller": controller_cfg["controller"],
        "controller_params": {
            "dtransport": dtransport_cfg,
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
            "max_vx": float(dtransport_cfg.get("max_speed", 0.30)),
            "max_vy": float(dtransport_cfg.get("max_speed", 0.30)),
            "max_wz": float(dtransport_cfg.get("max_wz", 0.60)),
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
    parser = argparse.ArgumentParser(
        description="Run a mock MAS pipeline with DBACT adapter."
    )
    parser.add_argument(
        "--controller-config",
        default="configs/mas/controller.yaml",
        help="Path to MAS-style controller.yaml.",
    )
    parser.add_argument(
        "--dtransport-config",
        default="configs/mas/dtransport_mock.yaml",
        help="Path to mock dtransport config.",
    )
    args = parser.parse_args()

    controller = make_controller(
        controller_config_path=args.controller_config,
        dtransport_config_path=args.dtransport_config,
    )
    world_state = make_world_state()

    velocities = controller.compute_planar_velocities(world_state)

    print("Mock MAS pipeline:")
    print("WorldState -> DecentralizedTransportController -> planar velocities")
    print()
    print(f"controller_config: {args.controller_config}")
    print(f"dtransport_config: {args.dtransport_config}")
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