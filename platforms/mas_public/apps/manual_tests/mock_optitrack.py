from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.messages import RobotState, WorldState
from src.common.time_utils import now_s
from src.messaging.factory import TransportFactory
from src.messaging.topics import WORLD_STATE


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock OptiTrack WorldState publisher.")
    parser.add_argument("--hz", type=float, default=100.0)
    args = parser.parse_args()
    configs = load_all_configs()
    logger = setup_logging("mock_optitrack")
    publisher = TransportFactory(configs["system"]["network"], logger).create_publisher("world_state")
    robot_ids = [item["robot_id"] for item in configs["robots"]["robots"]["list"]]
    frame_id = 0
    period = 1.0 / args.hz
    logger.info("mock optitrack publishing at %.1f Hz", args.hz)
    try:
        while True:
            timestamp = now_s()
            frame_id += 1
            robots = []
            for index, robot_id in enumerate(robot_ids):
                phase = timestamp * 0.3 + index
                x = index * 0.8 + 0.5 * math.cos(phase)
                y = 0.5 * math.sin(phase)
                yaw = phase % (2.0 * math.pi)
                robots.append(RobotState(robot_id, True, x, y, 0.0, 0.0, 0.0, yaw, 0.0, 0.0, 0.0, timestamp))
            publisher.publish(WORLD_STATE, WorldState(timestamp, frame_id, robots))
            time.sleep(period)
    except KeyboardInterrupt:
        logger.info("mock optitrack stopped")
    finally:
        publisher.close()


if __name__ == "__main__":
    main()

