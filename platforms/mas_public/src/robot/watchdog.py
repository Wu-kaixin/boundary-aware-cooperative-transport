from __future__ import annotations

from src.common.time_utils import monotonic_s


class CommandWatchdog:
    """Detect command timeout; RobotModule sends the stop command when expired."""

    def __init__(self, timeout_ms: int):
        self.timeout_s = timeout_ms / 1000.0
        self.last_command_time: float | None = None

    def mark_command(self) -> None:
        self.last_command_time = monotonic_s()

    def expired(self) -> bool:
        if self.last_command_time is None:
            return False
        return (monotonic_s() - self.last_command_time) > self.timeout_s

    def reset(self) -> None:
        self.last_command_time = None
