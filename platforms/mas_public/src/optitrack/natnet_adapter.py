from __future__ import annotations

import logging
import math
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from src.common.config_loader import get_project_root
from src.common.time_utils import now_s


@dataclass
class NatNetRigidBody:
    name: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    tracked: bool
    timestamp: float
    rigid_body_id: int | None = None


class NatNetAdapter:
    """NatNet receiver wrapper that converts SDK frames into NatNetRigidBody objects."""

    def __init__(self, config: dict, logger: logging.Logger):
        self.config = config
        self.logger = logger
        self.frame_callback: Callable[[list[NatNetRigidBody]], None] | None = None
        self.client = None
        self.latest_bodies: list[NatNetRigidBody] = []
        self.lock = threading.Lock()
        sdk_path = config.get("python_client_path")
        if sdk_path:
            resolved_sdk_path = Path(sdk_path)
            if not resolved_sdk_path.is_absolute():
                resolved_sdk_path = get_project_root() / resolved_sdk_path
            if str(resolved_sdk_path) not in sys.path:
                sys.path.insert(0, str(resolved_sdk_path))
        try:
            from NatNetClient import NatNetClient  # type: ignore

            self.client_cls = NatNetClient

            self.sdk_available = True
        except Exception as exc:
            self.client_cls = None
            self.sdk_available = False
            self.logger.warning("NatNet SDK unavailable: %s", exc)

    def set_frame_callback(self, callback: Callable[[list[NatNetRigidBody]], None]) -> None:
        self.frame_callback = callback

    def start(self) -> None:
        if not self.sdk_available:
            raise RuntimeError("NatNet SDK unavailable. Use MockNatNetAdapter or apps/manual_tests/mock_optitrack.py")
        self.client = self.client_cls()
        self.client.set_client_address(self.config["client_ip"])
        self.client.set_server_address(self.config["server_ip"])
        self.client.command_port = int(self.config["command_port"])
        self.client.data_port = int(self.config["data_port"])
        use_multicast = str(self.config.get("connection_type", "Unicast")).lower() == "multicast"
        self.client.set_use_multicast(use_multicast)
        self.client.set_print_level(0)
        self.client.new_frame_with_data_listener = self._receive_frame_with_data
        stream_type = str(self.config.get("stream_type", "d")).lower()
        if stream_type not in {"d", "c"}:
            raise ValueError("natnet.stream_type must be 'd' for datastream or 'c' for command stream")

        # Official PythonSample.py uses run(stream_type): d for datastream, c for command stream.
        is_running = self.client.run(stream_type)
        if not is_running:
            raise RuntimeError("Could not start NatNet streaming client")
        timeout_s = float(self.config.get("connect_check_timeout_s", 1.0))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.client.connected():
            time.sleep(0.05)
        if not self.client.connected():
            self.client.shutdown()
            self.client = None
            raise RuntimeError("Could not connect to Motive. Check that Motive streaming is enabled.")
        self.logger.info(
            "NatNet started server=%s client=%s connection=%s stream_type=%s data_port=%s command_port=%s",
            self.config["server_ip"],
            self.config["client_ip"],
            self.config["connection_type"],
            stream_type,
            self.config["data_port"],
            self.config["command_port"],
        )

    def stop(self) -> None:
        if self.client is not None:
            self.client.shutdown()
            self.client = None
        self.logger.info("NatNet adapter stopped")

    def next_frame(self) -> list[NatNetRigidBody]:
        with self.lock:
            return list(self.latest_bodies)

    def connected(self) -> bool:
        return bool(self.client is not None and self.client.connected())

    def _receive_frame_with_data(self, data_dict: dict) -> None:
        mocap_data = data_dict.get("mocap_data")
        rigid_body_data = getattr(mocap_data, "rigid_body_data", None)
        rigid_body_list = getattr(rigid_body_data, "rigid_body_list", []) if rigid_body_data else []
        timestamp = float(data_dict.get("timestamp") or now_s())
        bodies = []
        for rigid_body in rigid_body_list:
            rigid_body_id = int(getattr(rigid_body, "id_num"))
            bodies.append(
                NatNetRigidBody(
                    name=str(rigid_body_id),
                    position=tuple(getattr(rigid_body, "pos")),
                    quaternion=tuple(getattr(rigid_body, "rot")),
                    tracked=bool(getattr(rigid_body, "tracking_valid", True)),
                    timestamp=timestamp,
                    rigid_body_id=rigid_body_id,
                )
            )
        with self.lock:
            self.latest_bodies = bodies
        if self.frame_callback:
            self.frame_callback(bodies)


class MockNatNetAdapter:
    """No-hardware mock data source for OptiTrack module tests and manual checks."""

    def __init__(self, rigid_body_names: list[str], logger: logging.Logger):
        self.rigid_body_names = rigid_body_names
        self.logger = logger
        self.frame_id = 0

    def next_frame(self) -> list[NatNetRigidBody]:
        timestamp = now_s()
        self.frame_id += 1
        bodies = []
        for index, name in enumerate(self.rigid_body_names):
            phase = timestamp * 0.2 + index
            x = 0.5 * math.cos(phase) + index * 0.8
            y = 0.5 * math.sin(phase)
            yaw = phase % (2.0 * math.pi)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            bodies.append(NatNetRigidBody(name, (x, y, 0.0), (0.0, 0.0, qz, qw), True, timestamp))
        return bodies

    def stop(self) -> None:
        self.logger.info("Mock NatNet adapter stopped")
