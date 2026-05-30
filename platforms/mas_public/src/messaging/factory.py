from __future__ import annotations

import logging

from src.messaging.base_transport import CommandClient, CommandServer, Publisher, Subscriber
from src.messaging.zmq_transport import ZmqCommandClient, ZmqCommandServer, ZmqPublisher, ZmqSubscriber


def _address(host: str, port: int, bind: bool) -> str:
    target = "*" if bind else host
    return f"tcp://{target}:{port}"


class TransportFactory:
    """Create messaging objects from configs/system.yaml network settings."""

    def __init__(self, network_config: dict, logger: logging.Logger | None = None):
        self.network_config = network_config
        self.transport = network_config.get("backend", "zmq")
        self.host = network_config.get("host", "127.0.0.1")
        self.ports = network_config.get("ports", {})
        self.logger = logger

    def create_publisher(self, channel: str) -> Publisher:
        port = self._port(channel)
        if self.transport == "zmq":
            return ZmqPublisher(_address(self.host, port, bind=True), self.logger)
        raise ValueError(f"Unsupported transport: {self.transport}")

    def create_subscriber(self, channel: str, topics: list[str]) -> Subscriber:
        port = self._port(channel)
        if self.transport == "zmq":
            return ZmqSubscriber(_address(self.host, port, bind=False), topics, self.logger)
        raise ValueError(f"Unsupported transport: {self.transport}")

    def create_command_client(self, channel: str) -> CommandClient:
        port = self._port(channel)
        if self.transport == "zmq":
            return ZmqCommandClient(_address(self.host, port, bind=False), self.logger)
        raise ValueError(f"Unsupported transport: {self.transport}")

    def create_command_server(self, channel: str) -> CommandServer:
        port = self._port(channel)
        if self.transport == "zmq":
            return ZmqCommandServer(_address(self.host, port, bind=True), self.logger)
        raise ValueError(f"Unsupported transport: {self.transport}")

    def _port(self, channel: str) -> int:
        if channel not in self.ports:
            raise KeyError(f"Missing network.ports.{channel} in configs/system.yaml")
        return int(self.ports[channel])
