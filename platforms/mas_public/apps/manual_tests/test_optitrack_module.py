from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config_loader import load_all_configs
from src.common.messages import WorldState
from src.messaging.factory import TransportFactory
from src.messaging.topics import WORLD_STATE


def main() -> None:
    configs = load_all_configs()
    sub = TransportFactory(configs["system"]["network"]).create_subscriber("world_state", [WORLD_STATE])
    print("Waiting for WorldState. If no hardware is available, run apps/tests/mock_optitrack.py.")
    start = time.monotonic()
    count = 0
    try:
        while time.monotonic() - start < 5.0:
            received = sub.receive(timeout_ms=500)
            if received is None:
                continue
            _, payload = received
            state = WorldState.from_dict(payload)
            count += 1
            for robot in state.robots:
                print(
                    f"frame={state.frame_id} {robot.robot_id}: "
                    f"x={robot.x:.3f}, y={robot.y:.3f}, z={robot.z:.3f}, "
                    f"roll={robot.roll:.3f}, pitch={robot.pitch:.3f}, yaw={robot.yaw:.3f}, "
                    f"tracked={robot.tracked}"
                )
        fps = count / max(time.monotonic() - start, 1e-6)
        print(f"received frames={count}, fps={fps:.1f}")
    finally:
        sub.close()


if __name__ == "__main__":
    main()

