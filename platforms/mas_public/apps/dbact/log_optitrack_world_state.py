from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path


MAS_ROOT = Path(__file__).resolve().parents[2]
if str(MAS_ROOT) not in sys.path:
    sys.path.insert(0, str(MAS_ROOT))

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.math_utils import quaternion_to_rpy
from src.common.messages import RobotState, WorldState
from src.common.time_utils import now_s, sleep_until_next
from src.optitrack.natnet_adapter import MockNatNetAdapter, NatNetAdapter, NatNetRigidBody
from src.optitrack.rigid_body_mapper import RigidBodyMapper
from src.optitrack.state_estimator import StateEstimator
from src.optitrack.tracking_validator import TrackingValidator


class ReadOnlyWorldStateBuilder:
    """Build WorldState from NatNet rigid bodies without publishing commands."""

    def __init__(self, use_mock: bool):
        self.configs = load_all_configs()
        self.optitrack_config = self.configs["optitrack"]
        self.robots_config = self.configs["robots"]
        self.logger = setup_logging("optitrack_readonly")
        self.mapper = RigidBodyMapper(self.robots_config)
        self.estimator = StateEstimator()
        validation_config = dict(self.optitrack_config.get("tracking_validation", {}))
        validation_config["publish_untracked"] = self.optitrack_config.get("publish", {}).get(
            "publish_untracked",
            validation_config.get("publish_untracked", True),
        )
        self.tracking_validator = TrackingValidator(validation_config, self.mapper.expected_names())
        self.enable_velocity_estimation = bool(
            self.optitrack_config.get("state_estimation", {}).get("enable_velocity_estimation", True)
        )
        if use_mock:
            self.adapter = MockNatNetAdapter(self.mapper.expected_names(), self.logger)
        else:
            self.adapter = NatNetAdapter(self.optitrack_config["natnet"], self.logger)
        self.frame_id = 0

    def start(self) -> None:
        if isinstance(self.adapter, NatNetAdapter):
            if not self.adapter.sdk_available:
                raise RuntimeError(
                    "NatNet SDK unavailable. Use --mock or copy the NatNet Python client into "
                    "third_party/natnet_client."
                )
            self.adapter.start()

    def stop(self) -> None:
        self.adapter.stop()

    def next_world_state(self) -> WorldState:
        return self.build_world_state(self.adapter.next_frame())

    def build_world_state(self, bodies: list[NatNetRigidBody]) -> WorldState:
        self.frame_id += 1
        robots: list[RobotState] = []
        for body in self.tracking_validator.apply(bodies):
            robot_id = self.mapper.robot_id_for(body.name, body.rigid_body_id)
            if robot_id is None:
                continue
            x, y, z = float(body.position[0]), float(body.position[1]), float(body.position[2])
            roll, pitch, yaw = quaternion_to_rpy(*body.quaternion)
            if self.enable_velocity_estimation and body.tracked:
                vx, vy, vz, wz = self.estimator.estimate(robot_id, x, y, z, yaw, body.timestamp)
            else:
                vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            robots.append(
                RobotState(
                    robot_id=robot_id,
                    tracked=body.tracked,
                    x=x,
                    y=y,
                    z=z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    timestamp=body.timestamp,
                    vz=vz,
                )
            )
        return WorldState(timestamp=now_s(), frame_id=self.frame_id, robots=robots)


def world_state_rows(world_state: WorldState) -> list[dict]:
    rows = []
    for robot in world_state.robots:
        rows.append(
            {
                "time": world_state.timestamp,
                "frame_id": world_state.frame_id,
                "robot_id": robot.robot_id,
                "tracked": robot.tracked,
                "x": robot.x,
                "y": robot.y,
                "z": robot.z,
                "roll": robot.roll,
                "pitch": robot.pitch,
                "yaw": robot.yaw,
                "vx": robot.vx,
                "vy": robot.vy,
                "vz": robot.vz,
                "wz": robot.wz,
                "robot_timestamp": robot.timestamp,
            }
        )
    return rows


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "time",
        "frame_id",
        "robot_id",
        "tracked",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "vx",
        "vy",
        "vz",
        "wz",
        "robot_timestamp",
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_world_state(index: int, world_state: WorldState) -> None:
    print(f"\nframe={index}, world_frame_id={world_state.frame_id}, robots={len(world_state.robots)}")
    for robot in world_state.robots:
        print(
            f"  {robot.robot_id}: "
            f"tracked={robot.tracked}, "
            f"pos=({robot.x: .3f}, {robot.y: .3f}, {robot.z: .3f}), "
            f"yaw={robot.yaw: .3f}, "
            f"vel=({robot.vx: .3f}, {robot.vy: .3f}, {robot.vz: .3f}), "
            f"wz={robot.wz: .3f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Log OptiTrack/NatNet WorldState to CSV without ControllerModule or RoboMaster output."
    )
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--mock", action="store_true", help="Use MockNatNetAdapter instead of real NatNet.")
    parser.add_argument("--print-every", type=int, default=20)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/optitrack_readonly/world_states.csv"),
    )
    args = parser.parse_args()

    if args.frames <= 0:
        raise ValueError("--frames must be positive")
    if args.hz <= 0.0:
        raise ValueError("--hz must be positive")

    builder = ReadOnlyWorldStateBuilder(use_mock=args.mock)
    period = 1.0 / args.hz
    rows: list[dict] = []

    print("OptiTrack WorldState read-only logger")
    print("No ControllerModule. No RoboMaster. No ControlCommand.")
    print(f"adapter={type(builder.adapter).__name__}")
    print(f"expected_rigid_bodies={builder.mapper.expected_names()}")
    print(f"frames={args.frames}, hz={args.hz}")
    print(f"output={args.output}")

    try:
        builder.start()
        for index in range(args.frames):
            loop_start = time.monotonic()
            world_state = builder.next_world_state()
            rows.extend(world_state_rows(world_state))
            if index % args.print_every == 0 or index == args.frames - 1:
                print_world_state(index, world_state)
            sleep_until_next(loop_start, period)
    finally:
        builder.stop()
        write_rows(args.output, rows)

    print(f"\nrows={len(rows)}")
    print(f"csv={args.output}")
    print("Done.")


if __name__ == "__main__":
    main()
