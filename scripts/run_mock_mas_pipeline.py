from __future__ import annotations

import argparse
import csv
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


def print_velocities(step: int, world_state: MockWorldState, velocities: dict[str, np.ndarray]) -> None:
    print(f"step={step}, t={world_state.timestamp:.2f}s")
    for robot in world_state.robots:
        velocity = velocities[robot.robot_id]
        speed = float(np.linalg.norm(velocity))
        print(
            f"  {robot.robot_id}: "
            f"pos=({robot.x: .3f}, {robot.y: .3f}), "
            f"vx={velocity[0]: .4f}, "
            f"vy={velocity[1]: .4f}, "
            f"speed={speed: .4f}"
        )


def integrate_world_state(
    world_state: MockWorldState,
    velocities: dict[str, np.ndarray],
    dt: float,
) -> None:
    for robot in world_state.robots:
        velocity = velocities.get(robot.robot_id, np.zeros(2, dtype=float))

        robot.vx = float(velocity[0])
        robot.vy = float(velocity[1])

        if robot.tracked:
            robot.x += robot.vx * dt
            robot.y += robot.vy * dt

    world_state.timestamp += dt
def append_state_rows(
    rows: list[dict],
    step: int,
    world_state: MockWorldState,
) -> None:
    for robot in world_state.robots:
        rows.append(
            {
                "step": step,
                "time": world_state.timestamp,
                "robot_id": robot.robot_id,
                "x": robot.x,
                "y": robot.y,
                "vx": robot.vx,
                "vy": robot.vy,
                "yaw": robot.yaw,
                "tracked": robot.tracked,
            }
        )


def append_command_rows(
    rows: list[dict],
    step: int,
    world_state: MockWorldState,
    velocities: dict[str, np.ndarray],
) -> None:
    for robot_id, velocity in velocities.items():
        rows.append(
            {
                "step": step,
                "time": world_state.timestamp,
                "robot_id": robot_id,
                "cmd_vx": float(velocity[0]),
                "cmd_vy": float(velocity[1]),
                "cmd_speed": float(np.linalg.norm(velocity)),
            }
        )


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a multi-step mock MAS pipeline with DBACT adapter."
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
    parser.add_argument(
        "--steps",
        type=int,
        default=20,
        help="Number of mock integration steps.",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.05,
        help="Mock integration time step.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=5,
        help="Print one command summary every N steps.",
    )
    parser.add_argument(
        "--output",
        default="runs/mock_mas_pipeline",
        help="Output directory for mock states and commands CSV files.",
    )
    args = parser.parse_args()

    controller = make_controller(
        controller_config_path=args.controller_config,
        dtransport_config_path=args.dtransport_config,
    )
    world_state = make_world_state()

    print("Mock MAS multi-step pipeline:")
    print("WorldState -> DecentralizedTransportController -> planar velocities -> integrated WorldState")
    print()
    print(f"controller_config: {args.controller_config}")
    print(f"dtransport_config: {args.dtransport_config}")
    print(f"steps: {args.steps}")
    print(f"dt: {args.dt}")
    print()

    last_velocities: dict[str, np.ndarray] = {}
    state_rows: list[dict] = []
    command_rows: list[dict] = []

    for step in range(args.steps):
        velocities = controller.compute_planar_velocities(world_state)
        last_velocities = velocities
        append_state_rows(state_rows, step, world_state)
        append_command_rows(command_rows, step, world_state, velocities)

        if step % args.print_every == 0 or step == args.steps - 1:
            print_velocities(step, world_state, velocities)
            print()

        integrate_world_state(world_state, velocities, args.dt)

    print("Final mock robot states:")
    for robot in world_state.robots:
        print(
            f"  {robot.robot_id}: "
            f"pos=({robot.x: .3f}, {robot.y: .3f}), "
            f"vel=({robot.vx: .4f}, {robot.vy: .4f})"
        )

    max_speed = max(float(np.linalg.norm(v)) for v in last_velocities.values())
    print()
    output_dir = Path(args.output)
    states_path = output_dir / "states.csv"
    commands_path = output_dir / "commands.csv"

    write_csv(states_path, state_rows)
    write_csv(commands_path, command_rows)

    print()
    print(f"states_csv={states_path}")
    print(f"commands_csv={commands_path}")
    print(f"max_last_step_speed={max_speed:.4f}")
    print("Done.")


if __name__ == "__main__":
    main()