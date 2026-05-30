from __future__ import annotations

import json
import logging
from dataclasses import is_dataclass
from typing import Any

import zmq

from src.common.messages import message_to_dict
from src.messaging.base_transport import CommandClient, CommandServer, Publisher, Subscriber


def _encode_message(message: Any) -> str:
    payload = message_to_dict(message) if is_dataclass(message) else message
    return json.dumps(payload, ensure_ascii=False)


def _decode_message(raw: bytes) -> dict:
    return json.loads(raw.decode("utf-8"))


class ZmqPublisher(Publisher):
    """ZeroMQ PUB publisher behind the common Publisher interface."""

    def __init__(self, bind_address: str, logger: logging.Logger | None = None):
        self.logger = logger or logging.getLogger(__name__)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(bind_address)
        self.logger.info("ZMQ publisher bound at %s", bind_address)

    def publish(self, topic: str, message: Any) -> None:
        self.socket.send_multipart([topic.encode("utf-8"), _encode_message(message).encode("utf-8")])

    def close(self) -> None:
        self.socket.close(linger=0)


class ZmqSubscriber(Subscriber):
    """ZeroMQ SUB subscriber that can subscribe to multiple topics."""

    def __init__(self, connect_address: str, topics: list[str], logger: logging.Logger | None = None):
        self.logger = logger or logging.getLogger(__name__)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(connect_address)
        for topic in topics:
            self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.logger.info("ZMQ subscriber connected to %s topics=%s", connect_address, topics)

    def receive(self, timeout_ms: int | None = None) -> tuple[str, dict] | None:
        events = dict(self.poller.poll(timeout_ms))
        if self.socket not in events:
            return None
        topic_raw, payload_raw = self.socket.recv_multipart()
        return topic_raw.decode("utf-8"), _decode_message(payload_raw)

    def close(self) -> None:
        self.socket.close(linger=0)


class ZmqCommandClient(CommandClient):
    """Command client implemented with ZeroMQ PUSH."""

    def __init__(self, connect_address: str, logger: logging.Logger | None = None):
        self.logger = logger or logging.getLogger(__name__)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.connect(connect_address)
        self.logger.info("ZMQ command client connected to %s", connect_address)

    def send_command(self, topic: str, message: Any) -> None:
        self.socket.send_multipart([topic.encode("utf-8"), _encode_message(message).encode("utf-8")])

    def close(self) -> None:
        self.socket.close(linger=0)


class ZmqCommandServer(CommandServer):
    """Command server implemented with ZeroMQ PULL."""

    def __init__(self, bind_address: str, logger: logging.Logger | None = None):
        self.logger = logger or logging.getLogger(__name__)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.bind(bind_address)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.logger.info("ZMQ command server bound at %s", bind_address)

    def receive_command(self, timeout_ms: int | None = None) -> tuple[str, dict] | None:
        events = dict(self.poller.poll(timeout_ms))
        if self.socket not in events:
            return None
        topic_raw, payload_raw = self.socket.recv_multipart()
        return topic_raw.decode("utf-8"), _decode_message(payload_raw)

    def close(self) -> None:
        self.socket.close(linger=0)


def pub_address(host: str, port: int, bind: bool = False) -> str:
    target = "*" if bind else host
    return f"tcp://{target}:{port}"


def command_address(host: str, port: int, bind: bool = False) -> str:
    target = "*" if bind else host
    return f"tcp://{target}:{port}"
