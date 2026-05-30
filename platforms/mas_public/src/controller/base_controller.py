from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from src.common.messages import ControlCommand, WorldState


class BaseController(ABC):
    """Internal module."""

    def __init__(self, config: dict[str, Any], robot_ids: list[str]):
        self.config = config
        self.robot_ids = robot_ids
        self.robot_mode = str(config["controller"]["robot_mode"])

    @abstractmethod
    def compute(self, world_state: WorldState | None) -> ControlCommand:
        raise NotImplementedError

