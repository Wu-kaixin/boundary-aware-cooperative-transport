from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any, Type, TypeVar


T = TypeVar("T")


@dataclass
class RobotState:
    robot_id: str
    tracked: bool
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float
    vy: float
    wz: float
    timestamp: float
    vz: float = 0.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotState":
        return cls(**data)


@dataclass
class WorldState:
    timestamp: float
    frame_id: int
    robots: list[RobotState] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorldState":
        robots = [RobotState.from_dict(item) for item in data.get("robots", [])]
        return cls(timestamp=data["timestamp"], frame_id=data["frame_id"], robots=robots)


@dataclass
class RobotCommand:
    robot_id: str
    chassis_vx: float | None
    chassis_vy: float | None
    chassis_wz: float | None
    gimbal_yaw_speed: float | None
    gimbal_pitch_speed: float | None
    controller_mode: str

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotCommand":
        return cls(**data)


@dataclass
class ControlCommand:
    timestamp: float
    robot_mode: str
    commands: list[RobotCommand] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ControlCommand":
        commands = [RobotCommand.from_dict(item) for item in data.get("commands", [])]
        return cls(
            timestamp=data["timestamp"],
            robot_mode=data["robot_mode"],
            commands=commands,
        )


@dataclass
class ModuleStatus:
    module_name: str
    status: str
    message: str
    timestamp: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ModuleStatus":
        return cls(**data)


@dataclass
class RobotStatus:
    robot_id: str
    status_type: str
    timestamp: float
    pitch_angle: float | None = None
    yaw_angle: float | None = None
    pitch_ground_angle: float | None = None
    yaw_ground_angle: float | None = None
    requested_mode: str | None = None
    actual_mode: str | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotStatus":
        return cls(**data)


@dataclass
class SystemCommand:
    command_type: str
    target_module: str
    payload: dict[str, Any]
    timestamp: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SystemCommand":
        return cls(**data)


def message_to_dict(message: Any) -> dict[str, Any]:
    """Convert a dataclass message to a JSON-serializable dictionary."""
    if not is_dataclass(message):
        raise TypeError(f"Expected dataclass message, got {type(message)!r}")
    return asdict(message)


def message_from_dict(message_type: Type[T], data: dict[str, Any]) -> T:
    """Restore a message object from a dictionary."""
    if not hasattr(message_type, "from_dict"):
        return message_type(**data)
    return message_type.from_dict(data)  # type: ignore[return-value]


