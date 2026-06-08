from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def ensure_robomaster_importable() -> None:
    try:
        import robomaster  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    candidates = [
        WORKSPACE_ROOT / "Wormhole" / "s1 hack" / "Test" / "test.venv" / "Lib" / "site-packages",
        WORKSPACE_ROOT / "Wormhole" / "s1 hack" / "Test" / "fork.venv" / "Lib" / "site-packages",
        WORKSPACE_ROOT / "Wormhole" / "s1 hack" / "Test" / "rm_env" / "Lib" / "site-packages",
    ]
    for candidate in candidates:
        if (candidate / "robomaster").exists() and str(candidate) not in sys.path:
            sys.path.append(str(candidate))
            try:
                import robomaster  # noqa: F401

                return
            except ModuleNotFoundError:
                continue
    raise ModuleNotFoundError(
        "No module named 'robomaster'. Install it in the active Python environment "
        "or keep the bundled Test/test.venv site-packages directory available."
    )

from dbact.agent_control import (  # noqa: E402
    AgentVelocityCommand,
    AgentController,
    CentralizedCVTParams,
    CentralizedCVTPolicy,
    IntegratingStateProvider,
    MockRobotBackend,
    S1RoboMasterBackend,
    TrackedAgentState,
    VelocityLimits,
)
from dbact.cargo import Cargo  # noqa: E402
from dbact.controller import DBACTController, DBACTParams  # noqa: E402


AGENT_IDS = [
    "robot_1",
    "robot_2",
    "robot_3",
    "robot_4",
    "robot_5",
    "robot_6",
    "robot_7",
]

SN_BY_AGENT_ID = {
    "robot_1": "159CG7R0040AN0",
    "robot_2": "159CG8J0040QA3",
    "robot_3": "159CG8J0040Q8K",
    "robot_4": "159CG9F00506T2",
    "robot_5": "159CG930040ZLQ",
    "robot_6": "159CG9F005065N",
    "robot_7": "159CG9B00503VJ",
}


class ReversePolicy:
    """Invert another policy's world-frame velocity commands."""

    def __init__(self, inner_policy):
        self.inner_policy = inner_policy

    def compute(self, snapshot):
        commands = self.inner_policy.compute(snapshot)
        return {
            agent_id: AgentVelocityCommand(
                agent_id=command.agent_id,
                vx_world=-command.vx_world,
                vy_world=-command.vy_world,
                wz=-command.wz,
                mode=f"reverse_{command.mode}",
            )
            for agent_id, command in commands.items()
        }


class VirtualBoxDBACTPolicy:
    """DBACT caging policy with a manually placed virtual box.

    Use this only when OptiTrack is unavailable. The physical box must be placed
    where the virtual box is configured, and robot poses are dead-reckoned from
    the assumed initial line.
    """

    def __init__(self, agent_ids: list[str], args: argparse.Namespace):
        self.agent_ids = list(agent_ids)
        self.controller = DBACTController(
            DBACTParams(
                task_mode="caging",
                sensor_range=args.sensor_range,
                comm_range=args.comm_range,
                cage_offset=args.cage_offset,
                sigma=args.sigma,
                d_min=args.d_min,
                max_speed=args.max_speed,
                kp_explore=args.kp_explore,
                kp_cage=args.kp_cage,
                grid_resolution=args.grid_resolution,
                cbf_use_qp=False,
            ),
            (args.world_x_min, args.world_x_max, args.world_y_min, args.world_y_max),
        )
        self.cargo = Cargo.rectangle(
            "virtual_box",
            center=[args.box_x, args.box_y],
            width=args.box_width,
            height=args.box_height,
            yaw=args.box_yaw,
            transport_direction=[0.0, 1.0],
        )
        self._last_timestamp: float | None = None

    def compute(self, snapshot):
        tracked = [
            snapshot.agents[agent_id].to_agent_state()
            for agent_id in self.agent_ids
            if agent_id in snapshot.agents and snapshot.agents[agent_id].tracked
        ]
        if not tracked:
            return {agent_id: AgentVelocityCommand(agent_id=agent_id, mode="dbact_box_untracked") for agent_id in self.agent_ids}
        commands = self.controller.step(tracked, [self.cargo], snapshot.timestamp, self._dt(snapshot.timestamp))
        by_id = {command.agent_id: command for command in commands}
        result = {}
        for agent_id in self.agent_ids:
            command = by_id.get(agent_id)
            if command is None:
                result[agent_id] = AgentVelocityCommand(agent_id=agent_id, mode="dbact_box_untracked")
                continue
            result[agent_id] = AgentVelocityCommand(
                agent_id=agent_id,
                vx_world=float(command.velocity[0]),
                vy_world=float(command.velocity[1]),
                wz=0.0,
                mode=f"virtual_box_{command.mode}",
            )
        return result

    def _dt(self, timestamp: float) -> float:
        if self._last_timestamp is None:
            self._last_timestamp = timestamp
            return 0.05
        dt = max(1e-3, min(0.2, timestamp - self._last_timestamp))
        self._last_timestamp = timestamp
        return dt


def build_initial_agents(spacing_m: float) -> dict[str, TrackedAgentState]:
    """Assumed start poses for the no-OptiTrack smoke test."""
    center = (len(AGENT_IDS) - 1) / 2.0
    return {
        agent_id: TrackedAgentState(
            agent_id=agent_id,
            x=(index - center) * spacing_m,
            y=-0.65,
            yaw=0.0,
        )
        for index, agent_id in enumerate(AGENT_IDS)
    }


def build_backend(args: argparse.Namespace):
    if args.real:
        return S1RoboMasterBackend(
            SN_BY_AGENT_ID,
            conn_type="sta",
            proto_type="udp",
            drive_timeout_s=args.drive_timeout,
        )
    return MockRobotBackend()


def run(args: argparse.Namespace) -> None:
    if args.real:
        ensure_robomaster_importable()
    dt = 1.0 / args.hz
    provider = IntegratingStateProvider(build_initial_agents(args.initial_spacing))
    if args.controller == "dbact-box":
        policy = VirtualBoxDBACTPolicy(AGENT_IDS, args)
    else:
        policy = CentralizedCVTPolicy(
            AGENT_IDS,
            CentralizedCVTParams(
                domain=(args.world_x_min, args.world_x_max, args.world_y_min, args.world_y_max),
                kp_xy=args.kp_xy,
                kd_xy=0.05,
                kp_yaw=0.0,
                yaw_mode="fixed",
                target_yaw=0.0,
                grid_resolution=args.grid_resolution,
                max_speed=args.max_speed,
            ),
        )
    if args.reverse_return:
        policy = ReversePolicy(policy)
    backend = build_backend(args)
    controller = AgentController(
        AGENT_IDS,
        provider,
        policy,
        backend,
        limits=VelocityLimits(max_vx=args.max_vx, max_vy=args.max_vy, max_wz=0.0),
    )

    controller.connect()
    if args.connect_only:
        print("Connected robots only; sending stop_all and closing.")
        backend.stop_all()
        backend.close()
        return

    started_at = time.monotonic()
    next_print = 0.0
    try:
        while time.monotonic() - started_at < args.duration:
            loop_start = time.monotonic()
            commands = controller.step()
            provider.advance_body_commands(commands, dt)
            elapsed = time.monotonic() - started_at
            if elapsed >= next_print:
                summary = ", ".join(
                    f"{cmd.agent_id}:vx={cmd.vx:+.3f},vy={cmd.vy:+.3f}" for cmd in commands
                )
                print(f"t={elapsed:05.2f}s {summary}")
                next_print = elapsed + args.print_period
            sleep_s = dt - (time.monotonic() - loop_start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    finally:
        print("Stopping all robots.")
        backend.stop_all()
        backend.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Seven RoboMaster S1 centralized-CVT smoke test. Default is mock; use --real to move robots."
    )
    parser.add_argument("--real", action="store_true", help="Connect to the seven real STA S1 robots.")
    parser.add_argument("--controller", choices=["cvt", "dbact-box"], default="cvt", help="Controller to run.")
    parser.add_argument("--connect-only", action="store_true", help="Only connect, send zero speed, and close.")
    parser.add_argument("--duration", type=float, default=12.0, help="Control duration in seconds.")
    parser.add_argument("--hz", type=float, default=10.0, help="Control loop frequency.")
    parser.add_argument("--max-speed", type=float, default=0.05, help="Policy world-frame speed norm limit.")
    parser.add_argument("--max-vx", type=float, default=0.05, help="Body-frame x speed limit in m/s.")
    parser.add_argument("--max-vy", type=float, default=0.05, help="Body-frame y speed limit in m/s.")
    parser.add_argument("--kp-xy", type=float, default=0.25, help="CVT proportional gain.")
    parser.add_argument("--initial-spacing", type=float, default=0.22, help="Assumed initial line spacing in meters.")
    parser.add_argument("--drive-timeout", type=float, default=0.20, help="RoboMaster SDK drive_speed timeout.")
    parser.add_argument("--print-period", type=float, default=1.0, help="Command print period in seconds.")
    parser.add_argument(
        "--reverse-return",
        action="store_true",
        help="Invert the CVT velocity commands to drive back along the approximate previous direction.",
    )
    parser.add_argument("--world-x-min", type=float, default=-1.2, help="Assumed world minimum x.")
    parser.add_argument("--world-x-max", type=float, default=1.2, help="Assumed world maximum x.")
    parser.add_argument("--world-y-min", type=float, default=-1.0, help="Assumed world minimum y.")
    parser.add_argument("--world-y-max", type=float, default=1.0, help="Assumed world maximum y.")
    parser.add_argument("--box-x", type=float, default=0.0, help="Virtual box center x.")
    parser.add_argument("--box-y", type=float, default=0.0, help="Virtual box center y.")
    parser.add_argument("--box-width", type=float, default=0.50, help="Virtual box width in meters.")
    parser.add_argument("--box-height", type=float, default=0.35, help="Virtual box height in meters.")
    parser.add_argument("--box-yaw", type=float, default=0.0, help="Virtual box yaw in radians.")
    parser.add_argument("--sensor-range", type=float, default=0.90, help="DBACT virtual local boundary sensor range.")
    parser.add_argument("--comm-range", type=float, default=0.75, help="DBACT neighbor communication range.")
    parser.add_argument("--cage-offset", type=float, default=0.28, help="Desired offset from box boundary.")
    parser.add_argument("--sigma", type=float, default=0.25, help="Boundary density smoothing.")
    parser.add_argument("--d-min", type=float, default=0.25, help="Minimum robot-robot distance for local safety.")
    parser.add_argument("--kp-cage", type=float, default=0.25, help="DBACT cage gain.")
    parser.add_argument("--kp-explore", type=float, default=0.05, help="DBACT explore gain.")
    parser.add_argument("--grid-resolution", type=int, default=28, help="CVT grid resolution.")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
