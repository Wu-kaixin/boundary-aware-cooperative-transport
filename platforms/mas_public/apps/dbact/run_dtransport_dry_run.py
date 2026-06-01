from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


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



def out_of_bounds_robot_ids(
    robot_states: dict[str, dict[str, float]],
    world_config: dict,
) -> list[str]:
    x_min = float(world_config.get("x_min", -float("inf")))
    x_max = float(world_config.get("x_max", float("inf")))
    y_min = float(world_config.get("y_min", -float("inf")))
    y_max = float(world_config.get("y_max", float("inf")))

    out = []
    for robot_id, state in robot_states.items():
        x = float(state["x"])
        y = float(state["y"])
        if x < x_min or x > x_max or y < y_min or y > y_max:
            out.append(robot_id)
    return out


def world_bounds_text(world_config: dict) -> str:
    return (
        f"x=[{world_config.get('x_min')}, {world_config.get('x_max')}], "
        f"y=[{world_config.get('y_min')}, {world_config.get('y_max')}]"
    )

def build_initial_robot_states(
    robot_ids: list[str],
    controller_config: dict,
    system_config: dict,
) -> dict[str, dict[str, float]]:
    world = system_config["world"]
    x_min = float(world.get("x_min", -1.0))
    x_max = float(world.get("x_max", 1.0))
    y_min = float(world.get("y_min", -1.5))
    y_max = float(world.get("y_max", 1.5))

    params = controller_config.get("controller_params", {}).get("dtransport", {})
    virtual_object = params.get("virtual_object", {})
    cage_offset = float(params.get("cage_offset", 0.28))

    if virtual_object.get("enabled", False) and "vertices" in virtual_object:
        vertices = np.array(virtual_object["vertices"], dtype=float)
        cargo_min_x = float(np.min(vertices[:, 0]))
        cargo_max_x = float(np.max(vertices[:, 0]))
        cargo_min_y = float(np.min(vertices[:, 1]))
        cargo_center_x = float(np.mean(vertices[:, 0]))
    else:
        cargo_min_x = -0.2
        cargo_max_x = 0.2
        cargo_min_y = 0.2
        cargo_center_x = 0.0

    n = len(robot_ids)

    span = max(cargo_max_x - cargo_min_x, 0.6)
    left = cargo_center_x - 0.5 * span
    right = cargo_center_x + 0.5 * span

    # Keep the automatically generated row safely inside MAS world bounds.
    margin_x = max(0.18, 0.10 * (x_max - x_min))
    left = max(left, x_min + margin_x)
    right = min(right, x_max - margin_x)

    if right < left:
        center = min(max(cargo_center_x, x_min + margin_x), x_max - margin_x)
        left = right = center

    if n == 1:
        x_positions = [0.5 * (left + right)]
    else:
        x_positions = np.linspace(left, right, n).tolist()

    y_row = cargo_min_y - (cage_offset + 0.22)
    y_row = max(y_row, y_min + 0.12)
    y_row = min(y_row, y_max - 0.12)

    robot_states: dict[str, dict[str, float]] = {}
    for robot_id, x in zip(robot_ids, x_positions):
        robot_states[robot_id] = {
            "x": float(x),
            "y": float(y_row),
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
    return robot_states


def plot_trajectory(
    path: Path,
    state_rows: list[dict],
    controller_config: dict,
    system_config: dict,
    event_rows: list[dict],
) -> None:
    if not state_rows:
        return

    path.parent.mkdir(parents=True, exist_ok=True)

    robot_ids = sorted({row["robot_id"] for row in state_rows})
    world = system_config["world"]

    plt.figure(figsize=(7, 7))

    for robot_id in robot_ids:
        rows = [row for row in state_rows if row["robot_id"] == robot_id]
        xs = [float(row["x"]) for row in rows]
        ys = [float(row["y"]) for row in rows]

        plt.plot(xs, ys, linewidth=1.5, label=robot_id)
        plt.scatter(xs[0], ys[0], marker="o", s=30)
        plt.scatter(xs[-1], ys[-1], marker="x", s=45)

    params = controller_config.get("controller_params", {}).get("dtransport", {})
    virtual_object = params.get("virtual_object", {})
    if virtual_object.get("enabled", False):
        vertices = np.array(virtual_object["vertices"], dtype=float)
        closed = np.vstack([vertices, vertices[0]])
        plt.plot(closed[:, 0], closed[:, 1], linewidth=2.0, label="virtual cargo")

    x_min = float(world.get("x_min", -1.0))
    x_max = float(world.get("x_max", 1.0))
    y_min = float(world.get("y_min", -1.5))
    y_max = float(world.get("y_max", 1.5))

    bounds_x = [x_min, x_max, x_max, x_min, x_min]
    bounds_y = [y_min, y_min, y_max, y_max, y_min]
    plt.plot(bounds_x, bounds_y, linestyle="--", linewidth=1.5, label="world bounds")

    if event_rows:
        first_event = event_rows[0]
        event_step = int(first_event["step"])
        event_robot_ids = str(first_event["robot_ids"]).split(";")
        for robot_id in event_robot_ids:
            matches = [
                row for row in state_rows
                if int(row["step"]) == event_step and row["robot_id"] == robot_id
            ]
            for row in matches:
                plt.scatter(
                    float(row["x"]),
                    float(row["y"]),
                    marker="*",
                    s=120,
                    label="first out-of-bounds",
                )

    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("MAS dtransport dry-run trajectory")
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="DBACT dtransport MAS dry-run without hardware.")
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--print-every", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("data/dry_runs/dtransport"))
    parser.add_argument(
    "--stop-on-out-of-bounds",
    action="store_true",
    help="Stop dry-run when any robot leaves system.world bounds.",
)
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

    robot_states = build_initial_robot_states(
        robot_ids,
        controller_config,
        system_config,
    )

    state_rows: list[dict] = []
    command_rows: list[dict] = []
    event_rows: list[dict] = []
    final_status = "completed"

    print("MAS dtransport dry-run")
    print("No OptiTrack. No RoboMaster. No network.")
    print(f"controller.type={controller_config['controller']['type']}")
    print(f"robot_ids={robot_ids}")
    print(f"steps={args.steps}, dt={args.dt}")
    print(f"world_bounds={world_bounds_text(system_config['world'])}")
    print(f"stop_on_out_of_bounds={args.stop_on_out_of_bounds}")
    print("initial_robot_states:")
    for robot_id, state in robot_states.items():
        print(
            f"  {robot_id}: "
            f"pos=({state['x']: .3f}, {state['y']: .3f}), yaw={state['yaw']: .3f}"
        )

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
        out_ids = out_of_bounds_robot_ids(robot_states, system_config["world"])
        if out_ids:
            event = {
                "step": step,
                "time": timestamp,
                "event": "out_of_bounds",
                "robot_ids": ";".join(out_ids),
            }
            event_rows.append(event)
            print(
                f"WARNING: out_of_bounds at step={step}, "
                f"t={timestamp:.2f}s, robot_ids={out_ids}"
            )
            if args.stop_on_out_of_bounds:
                final_status = "stopped_out_of_bounds"
                break

    states_csv = args.output / "states.csv"
    commands_csv = args.output / "commands.csv"
    events_csv = args.output / "events.csv"
    trajectory_png = args.output / "trajectory.png"

    write_csv(states_csv, state_rows)
    write_csv(commands_csv, command_rows)
    write_csv(events_csv, event_rows)
    plot_trajectory(trajectory_png, state_rows, controller_config, system_config, event_rows)

    print(f"\nFinal dry-run status: {final_status}")

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
    print(f"events_csv={events_csv}")
    print(f"trajectory_png={trajectory_png}")
    print(f"final_status={final_status}")
    print("Done.")


if __name__ == "__main__":
    main()