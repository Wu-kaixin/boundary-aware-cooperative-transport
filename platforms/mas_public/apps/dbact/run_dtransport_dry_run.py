from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


MAS_ROOT = Path(__file__).resolve().parents[2]
if str(MAS_ROOT) not in sys.path:
    sys.path.insert(0, str(MAS_ROOT))

from src.common.config_loader import load_all_configs
from src.common.messages import RobotState, WorldState
from src.controller.decentralized_transport_controller import DecentralizedTransportController


def make_world_state(
    step: int,
    timestamp: float,
    robot_states: dict[str, dict[str, float]],
) -> WorldState:
    robots = []
    for robot_id, state in robot_states.items():
        robots.append(
            RobotState(
                robot_id=robot_id,
                tracked=True,
                x=float(state["x"]),
                y=float(state["y"]),
                z=0.0,
                roll=0.0,
                pitch=0.0,
                yaw=float(state.get("yaw", 0.0)),
                vx=float(state.get("vx", 0.0)),
                vy=float(state.get("vy", 0.0)),
                wz=0.0,
                timestamp=timestamp,
            )
        )
    return WorldState(timestamp=timestamp, frame_id=step, robots=robots)


def integrate_robot_states(
    robot_states: dict[str, dict[str, float]],
    control_command,
    dt: float,
) -> None:
    for command in control_command.commands:
        state = robot_states[command.robot_id]
        vx = float(command.chassis_vx or 0.0)
        vy = float(command.chassis_vy or 0.0)
        wz = float(command.chassis_wz or 0.0)

        state["x"] += vx * dt
        state["y"] += vy * dt
        state["yaw"] = wrap_angle(state.get("yaw", 0.0) + wz * dt)
        state["vx"] = vx
        state["vy"] = vy


def wrap_angle(angle: float) -> float:
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="DBACT dtransport MAS dry-run without hardware.")
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--print-every", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("data/dry_runs/dtransport"))
    args = parser.parse_args()

    configs = load_all_configs()
    controller_config = configs["controller"]
    system_config = configs["system"]
    robots_config = configs["robots"]

    robot_ids = [item["robot_id"] for item in robots_config["robots"]["list"]]

    controller = DecentralizedTransportController(
        controller_config,
        robot_ids,
        system_config["world"],
        system_config.get("limits", {}),
    )

    robot_states = {
        robot_ids[0]: {"x": -0.45, "y": -0.45, "yaw": 0.0, "vx": 0.0, "vy": 0.0},
        robot_ids[1]: {"x": 0.00, "y": -0.45, "yaw": 0.0, "vx": 0.0, "vy": 0.0},
        robot_ids[2]: {"x": 0.45, "y": -0.45, "yaw": 0.0, "vx": 0.0, "vy": 0.0},
    }

    state_rows: list[dict] = []
    command_rows: list[dict] = []

    print("MAS dtransport dry-run")
    print("No OptiTrack. No RoboMaster. No network.")
    print(f"controller.type={controller_config['controller']['type']}")
    print(f"robot_ids={robot_ids}")
    print(f"steps={args.steps}, dt={args.dt}")

    for step in range(args.steps):
        timestamp = step * args.dt
        world_state = make_world_state(step, timestamp, robot_states)
        control_command = controller.compute(world_state)

        for robot_id, state in robot_states.items():
            state_rows.append(
                {
                    "step": step,
                    "time": timestamp,
                    "robot_id": robot_id,
                    "x": state["x"],
                    "y": state["y"],
                    "yaw": state["yaw"],
                    "vx": state["vx"],
                    "vy": state["vy"],
                }
            )

        for command in control_command.commands:
            vx = float(command.chassis_vx or 0.0)
            vy = float(command.chassis_vy or 0.0)
            wz = float(command.chassis_wz or 0.0)
            command_rows.append(
                {
                    "step": step,
                    "time": timestamp,
                    "robot_id": command.robot_id,
                    "chassis_vx": vx,
                    "chassis_vy": vy,
                    "chassis_wz": wz,
                    "speed": math.hypot(vx, vy),
                    "controller_mode": command.controller_mode,
                }
            )

        if step % args.print_every == 0 or step == args.steps - 1:
            print(f"\nstep={step}, t={timestamp:.2f}s, robot_mode={control_command.robot_mode}")
            for command in control_command.commands:
                vx = float(command.chassis_vx or 0.0)
                vy = float(command.chassis_vy or 0.0)
                wz = float(command.chassis_wz or 0.0)
                print(
                    f"  {command.robot_id}: "
                    f"vx={vx: .4f}, vy={vy: .4f}, wz={wz: .4f}, "
                    f"speed={math.hypot(vx, vy): .4f}, mode={command.controller_mode}"
                )

        integrate_robot_states(robot_states, control_command, args.dt)

    states_csv = args.output / "states.csv"
    commands_csv = args.output / "commands.csv"
    write_csv(states_csv, state_rows)
    write_csv(commands_csv, command_rows)

    print("\nFinal dry-run robot states:")
    for robot_id, state in robot_states.items():
        print(
            f"  {robot_id}: "
            f"pos=({state['x']: .3f}, {state['y']: .3f}), "
            f"yaw={state['yaw']: .3f}, "
            f"vel=({state['vx']: .4f}, {state['vy']: .4f})"
        )

    print(f"\nstates_csv={states_csv}")
    print(f"commands_csv={commands_csv}")
    print("Done.")


if __name__ == "__main__":
    main()