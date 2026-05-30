from __future__ import annotations

import math
from dataclasses import dataclass

from src.common.time_utils import now_s
from src.optitrack.natnet_adapter import NatNetRigidBody


@dataclass
class TrackedBodyMemory:
    body: NatNetRigidBody
    last_seen_s: float


class TrackingValidator:
    """Optional OptiTrack tracking validation controlled by independent config switches."""

    def __init__(self, config: dict, expected_names: list[str] | None = None):
        self.config = config
        self.expected_names = expected_names or []
        self.enabled = bool(config.get("enabled", False))
        self.reject_position_jump = bool(config.get("reject_position_jump", False))
        self.max_position_jump_m = float(config.get("max_position_jump_m", 1.0))
        self.tracking_timeout_enabled = bool(config.get("tracking_timeout_enabled", False))
        self.tracking_timeout_s = float(config.get("tracking_timeout_ms", 200)) / 1000.0
        self.publish_untracked = bool(config.get("publish_untracked", True))
        self.memory: dict[str, TrackedBodyMemory] = {}

    def apply(self, bodies: list[NatNetRigidBody]) -> list[NatNetRigidBody]:
        if not self.enabled:
            return bodies

        output: list[NatNetRigidBody] = []
        seen_keys: set[str] = set()
        for body in bodies:
            key = self._key(body)
            seen_keys.add(key)
            validated = self._validate_body(key, body)
            if validated.tracked:
                self.memory[key] = TrackedBodyMemory(validated, validated.timestamp)
            if validated.tracked or self.publish_untracked:
                output.append(validated)

        if self.tracking_timeout_enabled and self.publish_untracked:
            output.extend(self._timed_out_bodies(seen_keys))
        return output

    def _validate_body(self, key: str, body: NatNetRigidBody) -> NatNetRigidBody:
        if not body.tracked:
            return body
        previous = self.memory.get(key)
        if previous is None or not self.reject_position_jump:
            return body
        if self._position_distance(previous.body, body) <= self.max_position_jump_m:
            return body
        return NatNetRigidBody(
            name=body.name,
            position=previous.body.position,
            quaternion=previous.body.quaternion,
            tracked=False,
            timestamp=body.timestamp,
            rigid_body_id=body.rigid_body_id,
        )

    def _timed_out_bodies(self, seen_keys: set[str]) -> list[NatNetRigidBody]:
        current_time = now_s()
        timed_out: list[NatNetRigidBody] = []
        for key, memory in self.memory.items():
            if key in seen_keys:
                continue
            if current_time - memory.last_seen_s < self.tracking_timeout_s:
                continue
            body = memory.body
            timed_out.append(
                NatNetRigidBody(
                    name=body.name,
                    position=body.position,
                    quaternion=body.quaternion,
                    tracked=False,
                    timestamp=current_time,
                    rigid_body_id=body.rigid_body_id,
                )
            )
        return timed_out

    @staticmethod
    def _position_distance(first: NatNetRigidBody, second: NatNetRigidBody) -> float:
        return math.dist(first.position, second.position)

    @staticmethod
    def _key(body: NatNetRigidBody) -> str:
        if body.rigid_body_id is not None:
            return f"id:{body.rigid_body_id}"
        return f"name:{body.name}"
