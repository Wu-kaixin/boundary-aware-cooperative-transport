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
from src.common.time_utils import monotonic_s
from src.controller.controller_module import ControllerModule
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
                wz=float(state.get("wz", 0.0)),
                timestamp=timestamp,
            )
        )
    return WorldState(timestamp=timestamp, frame_id=step, robots=robots)


def build_initial_robot_states(robot_ids: list[str]) -> dict[str, dict[str, float]]:
    # Keep this identical to the current dtransport dry-run initial layout.
    base = [
        (-0.517, -0.400),
        (-0.042, -0.400),
        (0.433, -0.400),
    ]
    states: dict[str, dict[str, float]] = {}
    for index, robot_id in enumerate(robot_ids):
        x, y = base[index] if index < len(base) else (0.0, -0.4)
        states[robot_id] = {"x": x, "y": y, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0}
    return states


def integrate_robot_states(
    robot_states: dict[str, dict[str, float]],
    command,
    dt: float,
) -> None:
    for robot_command in command.commands:
        state = robot_states[robot_command.robot_id]
        vx = float(robot_command.chassis_vx or 0.0)
        vy = float(robot_command.chassis_vy or 0.0)
        wz = float(robot_command.chassis_wz or 0.0)

        state["x"] += vx * dt
        state["y"] += vy * dt
        state["yaw"] = wrap_angle(float(state.get("yaw", 0.0)) + wz * dt)
        state["vx"] = vx
        state["vy"] = vy
        state["wz"] = wz


def wrap_angle(angle: float) -> float:
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def write_csv(path: Path, rows: list[dict], fieldnames: list[str] | None = None) -> None:
    if not rows and fieldnames is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = fieldnames or list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        if rows:
            writer.writerows(rows)


def make_dry_run_module() -> ControllerModule:
    configs = load_all_configs()

    module = ControllerModule.__new__(ControllerModule)
    module.logger = type(
        "Logger",
        (),
        {
            "info": lambda *args, **kwargs: None,
            "warning": lambda *args, **kwargs: None,
            "error": lambda *args, **kwargs: None,
            "exception": lambda *args, **kwargs: None,
        },
    )()
    module.configs = configs
    module.system_config = configs["system"]
    module.controller_config = configs["controller"]
    module.robots_config = configs["robots"]

    # This is a synthetic dry-run. The input WorldState is already in the
    # controller/MAS world frame, so do not apply OptiTrack-specific z-up
    # transforms or OptiTrack-gated world-bound safety logic here.
    module.configs.setdefault("supervisor", {})["use_optitrack"] = False
    module.system_config.setdefault("z_up_transform", {})["enabled"] = False
    module.robot_ids = [item["robot_id"] for item in module.robots_config["robots"]["list"]]

    module.coordinate_transformer = module._build_coordinate_transformer()
    module.controller = module._build_controller()

    module.last_world_state = None
    module.last_state_monotonic = None
    module.previous_control_frame_yaw_by_id = {}
    module.smoothed_pose_by_id = {}
    module.latest_robot_status_by_id = {}
    module.latest_module_status_by_name = {}
    module.untracked_since_by_id = {}
    module.out_of_bounds_since = None
    module.recorder = None
    module.task_completed = False
    module.task_failed = False
    module.failure_message = ""
    module.safety_event_messages = []
    module.interrupted = False
    module.running = True

    return module


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ControllerModule-level DBACT dtransport dry-run without network or hardware."
    )
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--print-every", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("data/dry_runs/controller_module_dtransport"))
    parser.add_argument("--stop-on-out-of-bounds", action="store_true")
    args = parser.parse_args()

    module = make_dry_run_module()

    assert isinstance(module.controller, DecentralizedTransportController), type(module.controller)

    robot_states = build_initial_robot_states(module.robot_ids)
    timeout_ms = int(module.controller_config["input"].get("state_timeout_ms", 500))
    world_config = module.system_config["world"]

    state_rows: list[dict] = []
    command_rows: list[dict] = []
    event_rows: list[dict] = []

    final_status = "completed"

    print("ControllerModule-level MAS dtransport dry-run")
    print("No OptiTrack. No RoboMaster. No ZMQ run loop.")
    print(f"controller.type={module.controller_config['controller']['type']}")
    print(f"controller_class={type(module.controller).__name__}")
    print(f"robot_ids={module.robot_ids}")
    print(f"steps={args.steps}, dt={args.dt}")
    print(f"stop_on_out_of_bounds={args.stop_on_out_of_bounds}")

    for step in range(args.steps):
        timestamp = step * args.dt
        world_state = make_world_state(step, timestamp, robot_states)

        module.last_world_state = world_state
        module.last_state_monotonic = monotonic_s()

        world_state_fresh = module._world_state_is_fresh(timeout_ms)
        state_valid = module._state_is_valid(timeout_ms)
        control_frame_world_state = (
            module._world_state_for_control_frame(module.last_world_state, update_yaw_rate=True)
            if module.last_world_state and state_valid
            else None
        )

        if module._bounds_check_enabled(world_state_fresh) and module._is_out_of_bounds(
            control_frame_world_state, world_config
        ):
            command = module._zero_command("world_out_of_bounds")
            event_rows.append(
                {
                    "step": step,
                    "time": timestamp,
                    "event": "world_out_of_bounds",
                    "robot_ids": ";".join(module.safety_event_messages[-1:]),
                }
            )
            if args.stop_on_out_of_bounds:
                final_status = "stopped_out_of_bounds"
                break
        else:
            command = module.controller.compute(control_frame_world_state)

        command = module._apply_gimbal_control(command)
        command = module._normalize_command_for_mode(command)

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
                    "wz": state["wz"],
                }
            )

        for robot_command in command.commands:
            vx = float(robot_command.chassis_vx or 0.0)
            vy = float(robot_command.chassis_vy or 0.0)
            wz = float(robot_command.chassis_wz or 0.0)
            command_rows.append(
                {
                    "step": step,
                    "time": timestamp,
                    "robot_id": robot_command.robot_id,
                    "chassis_vx": vx,
                    "chassis_vy": vy,
                    "chassis_wz": wz,
                    "speed": math.hypot(vx, vy),
                    "controller_mode": robot_command.controller_mode,
                    "robot_mode": command.robot_mode,
                }
            )

        if step % args.print_every == 0 or step == args.steps - 1:
            print(f"\nstep={step}, t={timestamp:.2f}s, robot_mode={command.robot_mode}")
            for robot_command in command.commands:
                vx = float(robot_command.chassis_vx or 0.0)
                vy = float(robot_command.chassis_vy or 0.0)
                wz = float(robot_command.chassis_wz or 0.0)
                print(
                    f"  {robot_command.robot_id}: "
                    f"vx={vx: .4f}, vy={vy: .4f}, wz={wz: .4f}, "
                    f"speed={math.hypot(vx, vy): .4f}, mode={robot_command.controller_mode}"
                )

        integrate_robot_states(robot_states, command, args.dt)

    states_csv = args.output / "states.csv"
    commands_csv = args.output / "commands.csv"
    events_csv = args.output / "events.csv"

    write_csv(states_csv, state_rows)
    write_csv(commands_csv, command_rows)
    write_csv(events_csv, event_rows, ["step", "time", "event", "robot_ids"])

    print("\nFinal ControllerModule dry-run robot states:")
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
    print(f"final_status={final_status}")
    print("Done.")


if __name__ == "__main__":
    main()