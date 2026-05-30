from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config_loader import load_all_configs
from src.common.messages import ControlCommand, RobotCommand, WorldState
from src.messaging.factory import TransportFactory
from src.messaging.topics import CONTROL_COMMAND, WORLD_STATE


def main() -> None:
    configs = load_all_configs()
    transport = TransportFactory(configs["system"]["network"])
    state_sub = transport.create_subscriber("world_state", [WORLD_STATE])
    cmd_pub = transport.create_publisher("control_command")
    target_x = 1.0
    print("Closed-loop IO tester started. Run mock_optitrack and mock_robot in separate terminals.")
    try:
        while True:
            received = state_sub.receive(timeout_ms=500)
            if received is None:
                print("waiting for WorldState...")
                continue
            _, payload = received
            state = WorldState.from_dict(payload)
            commands = []
            for robot in state.robots:
                vx = 0.1 if robot.x < target_x else 0.0
                commands.append(RobotCommand(robot.robot_id, vx, 0.0, 0.0, 0.0, 0.0, "io_test"))
            cmd_pub.publish(CONTROL_COMMAND, ControlCommand(time.time(), "chassis_lead", commands))
    except KeyboardInterrupt:
        print("closed-loop IO tester stopped")
    finally:
        state_sub.close()
        cmd_pub.close()


if __name__ == "__main__":
    main()

