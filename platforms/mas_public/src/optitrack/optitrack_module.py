from __future__ import annotations

import signal
from typing import Any

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.math_utils import quaternion_to_rpy
from src.common.messages import ModuleStatus, RobotState, SystemCommand, WorldState
from src.common.time_utils import monotonic_s, now_s, sleep_until_next
from src.messaging.factory import TransportFactory
from src.messaging.topics import MODULE_STATUS, OPTITRACK_COMMAND, WORLD_STATE
from src.optitrack.natnet_adapter import MockNatNetAdapter, NatNetAdapter, NatNetRigidBody
from src.optitrack.rigid_body_mapper import RigidBodyMapper
from src.optitrack.state_estimator import StateEstimator
from src.optitrack.tracking_validator import TrackingValidator


class OptiTrackModule:
    """OptiTrack module: converts NatNet rigid bodies into WorldState messages."""

    def __init__(self, use_mock: bool | None = None):
        self.configs = load_all_configs()
        self.system_config = self.configs["system"]
        self.optitrack_config = self.configs["optitrack"]
        self.robots_config = self.configs["robots"]
        self.logger = setup_logging("optitrack")
        transport = TransportFactory(self.system_config["network"], self.logger)
        self.publisher = transport.create_publisher("world_state")
        self.command_server = transport.create_command_server("system_command")
        self.mapper = RigidBodyMapper(self.robots_config)
        self.estimator = StateEstimator()
        validation_config = dict(self.optitrack_config.get("tracking_validation", {}))
        validation_config["publish_untracked"] = self.optitrack_config.get("publish", {}).get(
            "publish_untracked", validation_config.get("publish_untracked", True)
        )
        self.tracking_validator = TrackingValidator(validation_config, self.mapper.expected_names())
        state_estimation_config = self.optitrack_config.get("state_estimation", {})
        self.enable_velocity_estimation = bool(
            state_estimation_config.get("enable_velocity_estimation", True)
        )
        diagnostics_config = self.optitrack_config.get("diagnostics", {})
        self.log_rigid_bodies = bool(diagnostics_config.get("log_rigid_bodies", False))
        self.rigid_body_log_interval_s = float(diagnostics_config.get("log_interval_s", 1.0))
        self.last_rigid_body_log_s = 0.0
        self.running = False
        self.publishing = True
        self.frame_id = 0
        self.use_mock = False if use_mock is None else use_mock
        self.adapter = self._build_adapter()
        self.publish_period_s = self._publish_period_s()
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
        if hasattr(signal, "SIGBREAK"):
            signal.signal(signal.SIGBREAK, self._handle_signal)

    def _build_adapter(self) -> NatNetAdapter | MockNatNetAdapter:
        if self.use_mock:
            self.logger.warning("OptiTrack module running with MockNatNetAdapter")
            return MockNatNetAdapter(self.mapper.expected_names(), self.logger)
        adapter = NatNetAdapter(self.optitrack_config["natnet"], self.logger)
        if not adapter.sdk_available:
            self.logger.warning("Falling back to MockNatNetAdapter because NatNet SDK is unavailable")
            return MockNatNetAdapter(self.mapper.expected_names(), self.logger)
        return adapter

    def run(self) -> None:
        self.running = True
        if isinstance(self.adapter, NatNetAdapter):
            self.adapter.start()
        self._publish_status("running", "optitrack module started")
        try:
            while self.running:
                loop_start = monotonic_s()
                self._handle_commands()
                if self.publishing:
                    bodies = self._read_bodies()
                    self._log_rigid_body_diagnostics(bodies)
                    world_state = self._build_world_state(bodies)
                    self.publisher.publish(WORLD_STATE, world_state)
                sleep_until_next(loop_start, self.publish_period_s)
        except Exception as exc:
            self.logger.exception("OptiTrack error: %s", exc)
            self._publish_status("error", str(exc))
            raise
        finally:
            self.shutdown()

    def _read_bodies(self) -> list[NatNetRigidBody]:
        return self.adapter.next_frame()

    def _build_world_state(self, bodies: list[NatNetRigidBody]) -> WorldState:
        self.frame_id += 1
        robots: list[RobotState] = []
        for body in self.tracking_validator.apply(bodies):
            robot_id = self.mapper.robot_id_for(body.name, body.rigid_body_id)
            if robot_id is None:
                continue
            # Keep raw Motive/NatNet position here; controller-side transform handles control coordinates.
            x, y, z = float(body.position[0]), float(body.position[1]), float(body.position[2])
            roll, pitch, yaw = quaternion_to_rpy(*body.quaternion)
            if self.enable_velocity_estimation and body.tracked:
                vx, vy, vz, wz = self.estimator.estimate(robot_id, x, y, z, yaw, body.timestamp)
            else:
                vx, vy, vz, wz = 0.0, 0.0, 0.0, 0.0
            robots.append(
                RobotState(robot_id, body.tracked, x, y, z, roll, pitch, yaw, vx, vy, wz, body.timestamp, vz=vz)
            )
        return WorldState(timestamp=now_s(), frame_id=self.frame_id, robots=robots)

    def _log_rigid_body_diagnostics(self, bodies: list[NatNetRigidBody]) -> None:
        if not self.log_rigid_bodies:
            return
        current_time = monotonic_s()
        if current_time - self.last_rigid_body_log_s < self.rigid_body_log_interval_s:
            return
        self.last_rigid_body_log_s = current_time
        body_ids = [body.rigid_body_id for body in bodies]
        body_names = [body.name for body in bodies]
        mapped = {}
        unmapped = []
        for body in bodies:
            robot_id = self.mapper.robot_id_for(body.name, body.rigid_body_id)
            if robot_id is None:
                unmapped.append({"id": body.rigid_body_id, "name": body.name, "tracked": body.tracked})
            else:
                mapped[str(body.rigid_body_id if body.rigid_body_id is not None else body.name)] = robot_id
        self.logger.info(
            "NatNet rigid bodies ids=%s names=%s mapped=%s unmapped=%s",
            body_ids,
            body_names,
            mapped,
            unmapped,
        )

    def _handle_commands(self) -> None:
        received = self.command_server.receive_command(timeout_ms=0)
        if received is None:
            return
        topic, payload = received
        if topic != OPTITRACK_COMMAND:
            return
        command = SystemCommand.from_dict(payload)
        if command.command_type == "start_publish":
            self.publishing = True
        elif command.command_type == "stop_publish":
            self.publishing = False
        elif command.command_type == "set_frequency":
            self.system_config["frequency"]["optitrack_publish_hz"] = float(command.payload["frequency_hz"])
            self.publish_period_s = self._publish_period_s()
        self.logger.info("Handled OptiTrack command: %s", command.command_type)

    def _publish_period_s(self) -> float:
        hz = float(self.system_config["frequency"].get("optitrack_publish_hz", 100))
        return 1.0 / hz

    def _publish_status(self, status: str, message: str) -> None:
        self.publisher.publish(MODULE_STATUS, ModuleStatus("optitrack", status, message, now_s()))

    def _handle_signal(self, signum: int, frame: Any) -> None:
        self.logger.info("Received signal %s, stopping optitrack module", signum)
        self.running = False

    def shutdown(self) -> None:
        self.running = False
        self._publish_status("stopped", "optitrack module stopped")
        self.adapter.stop()
        self.publisher.close()
        self.command_server.close()
