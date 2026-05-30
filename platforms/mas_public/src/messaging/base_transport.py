from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class Publisher(ABC):
    @abstractmethod
    def publish(self, topic: str, message: Any) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class Subscriber(ABC):
    @abstractmethod
    def receive(self, timeout_ms: int | None = None) -> tuple[str, dict] | None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class CommandClient(ABC):
    @abstractmethod
    def send_command(self, topic: str, message: Any) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class CommandServer(ABC):
    @abstractmethod
    def receive_command(self, timeout_ms: int | None = None) -> tuple[str, dict] | None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

