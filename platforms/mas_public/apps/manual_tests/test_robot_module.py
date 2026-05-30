from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config_loader import load_all_configs
from src.common.messages import ControlCommand, RobotCommand
from src.robot.command_limiter import CommandLimiter
from src.robot.robomaster_adapter import RoboMasterAdapter
from src.robot.robot_registry import RobotRegistry
from src.robot.watchdog import CommandWatchdog
from src.common.logger import setup_logging


def main() -> None:
    configs = load_all_configs()
    logger = setup_logging("test_robot_module")
    registry = RobotRegistry(configs["robots"])
    limiter = CommandLimiter(configs["robots"]["command"], configs["robots"]["gimbal"])
    watchdog = CommandWatchdog(configs["robots"]["watchdog"]["command_timeout_ms"])
    adapter = RoboMasterAdapter(logger)
    adapter.connect_all(registry.robots, configs["robots"]["connection"]["conn_type"])
    try:
        low_speed = ControlCommand(
            time.time(),
            "chassis_lead",
            [RobotCommand(robot_id, 0.1, 0.0, 0.0, 0.0, 0.0, "low_speed_test") for robot_id in registry.ids()],
        )
        adapter.set_robot_mode(low_speed.robot_mode)
        for command in low_speed.commands:
            adapter.send_command(limiter.limit(command))
        watchdog.mark_command()
        time.sleep(1.0)
        print(f"watchdog_expired={watchdog.expired()}")
    finally:
        adapter.stop_all()
        adapter.close()
        print("stop_all called")


if __name__ == "__main__":
    main()

