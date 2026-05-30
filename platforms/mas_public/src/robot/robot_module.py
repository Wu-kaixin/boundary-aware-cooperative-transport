from __future__ import annotations

import signal
from typing import Any

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.messages import ControlCommand, ModuleStatus
from src.common.time_utils import now_s
from src.robot.command_limiter import CommandLimiter
from src.robot.robot_command_transform import RobotCommandTransform
from src.robot.robomaster_adapter import RoboMasterAdapter
from src.robot.robot_registry import RobotRegistry
from src.robot.watchdog import CommandWatchdog
from src.messaging.factory import TransportFactory
from src.messaging.topics import CONTROL_COMMAND, MODULE_STATUS, ROBOT_STATUS


class RobotModule:
    """Internal module."""

    def __init__(self):
        self.configs = load_all_configs()
        self.system_config = self.configs["system"]
        self.robots_config = self.configs["robots"]
        self.logger = setup_logging("robot")
        transport = TransportFactory(self.system_config["network"], self.logger)
        self.subscriber = transport.create_subscriber("control_command", [CONTROL_COMMAND])
        self.publisher = transport.create_publisher("module_status")
        self.registry = RobotRegistry(self.robots_config)
        self.limiter = CommandLimiter(self.system_config.get("limits", {}))
        self.transform = RobotCommandTransform(
            self.system_config.get("robot_command_transform", {}),
        )
        self.watchdog = CommandWatchdog(int(self.robots_config["watchdog"]["command_timeout_ms"]))
        self.adapter = RoboMasterAdapter(
            self.logger,
            drive_timeout_s=float(self.robots_config["chassis"]["sdk_drive_timeout_s"]),
            angular_unit=self.robots_config["chassis"]["angular_unit"],
            sdk_z_unit=self.robots_config["chassis"]["sdk_z_unit"],
            gimbal_config=self.robots_config["gimbal"],
        )
        self.running = False
        self.stopping = False
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
        if hasattr(signal, "SIGBREAK"):
            signal.signal(signal.SIGBREAK, self._handle_signal)

    def run(self) -> None:
        self.running = True
        self._publish_status("starting", "robot module starting")
        try:
            connection = self.robots_config["connection"]
            self.adapter.connect_all(
                self.registry.robots,
                conn_type=connection["conn_type"],
                proto_type=connection["proto_type"],
                retry_count=int(connection["retry_count"]),
                retry_delay_s=float(connection.get("retry_delay_s", 1.0)),
                require_sn=bool(connection["require_sn"]),
            )
            self._publish_status("running", "robot module running")
            while self.running:
                command_hz = float(self.system_config["frequency"].get("robot_command_hz", 50))
                poll_timeout_ms = int(1000.0 / command_hz)
                received = self.subscriber.receive(timeout_ms=poll_timeout_ms)
                if received is not None:
                    _, payload = received
                    command = ControlCommand.from_dict(payload)
                    self._apply_command(command)
                    self.watchdog.mark_command()
                if self.robots_config["watchdog"].get("stop_on_timeout", True) and self.watchdog.expired():
                    self.logger.warning("Command timeout, stopping all robots")
                    self.adapter.stop_all()
                    self.watchdog.reset()
                self._publish_robot_statuses()
        except Exception as exc:
            self.logger.exception("Robot module error: %s", exc)
            self._publish_status("error", str(exc))
            raise
        finally:
            self.shutdown()

    def _apply_command(self, command: ControlCommand) -> None:
        if self.stopping:
            self.logger.debug("Ignoring control command while robot module is stopping")
            return
        self.adapter.set_robot_mode(command.robot_mode)
        for robot_command in command.commands:
            limited = self.limiter.limit(robot_command)
            transformed = self.transform.apply(limited)
            self.adapter.send_command(transformed)

    def _publish_status(self, status: str, message: str) -> None:
        self.publisher.publish(MODULE_STATUS, ModuleStatus("robot", status, message, now_s()))

    def _publish_robot_statuses(self) -> None:
        for status in self.adapter.get_robot_statuses():
            self.publisher.publish(ROBOT_STATUS, status)

    def _handle_signal(self, signum: int, frame: Any) -> None:
        self.logger.info("Received signal %s, stopping robot module", signum)
        self.stopping = True
        self.running = False

    def shutdown(self) -> None:
        self.running = False
        self.stopping = True
        shutdown_start = now_s()
        self._publish_status("stopping", "robot module stopping")
        if self.robots_config["chassis"].get("stop_on_exit", True):
            stop_start = now_s()
            self.logger.info("Robot shutdown stop_all begin")
            self.adapter.stop_all()
            self.logger.info("Robot shutdown stop_all end elapsed=%.3fs", now_s() - stop_start)
        close_start = now_s()
        self.logger.info("Robot shutdown adapter close begin")
        self.adapter.close()
        self.logger.info("Robot shutdown adapter close end elapsed=%.3fs", now_s() - close_start)
        self._publish_status("stopped", "robot module stopped")
        self.publisher.close()
        self.subscriber.close()
        self.logger.info("Robot module shutdown complete elapsed=%.3fs", now_s() - shutdown_start)

