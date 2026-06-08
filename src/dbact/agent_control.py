from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .controller import DBACTController, DBACTParams
from .geometry import clip_to_domain
from .types import AgentState


@dataclass(frozen=True)
class TrackedAgentState:
    """Minimal state expected from OptiTrack, simulation, or a mock provider."""

    agent_id: str
    x: float
    y: float
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    tracked: bool = True

    def to_agent_state(self) -> AgentState:
        return AgentState(
            agent_id=self.agent_id,
            position=np.array([self.x, self.y], dtype=float),
            velocity=np.array([self.vx, self.vy], dtype=float),
            yaw=float(self.yaw),
        )


@dataclass(frozen=True)
class WorldSnapshot:
    timestamp: float
    agents: dict[str, TrackedAgentState]


@dataclass(frozen=True)
class AgentVelocityCommand:
    agent_id: str
    vx_world: float = 0.0
    vy_world: float = 0.0
    wz: float = 0.0
    mode: str = "idle"


@dataclass(frozen=True)
class BodyVelocityCommand:
    agent_id: str
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    mode: str = "idle"


class StateProvider(Protocol):
    def snapshot(self) -> WorldSnapshot:
        """Return the latest world-frame robot states."""


class ControlPolicy(Protocol):
    def compute(self, snapshot: WorldSnapshot) -> dict[str, AgentVelocityCommand]:
        """Return world-frame velocity commands keyed by agent id."""


class RobotBackend(Protocol):
    def connect(self, agent_ids: list[str]) -> None:
        """Prepare all robots for command streaming."""

    def send(self, command: BodyVelocityCommand) -> None:
        """Send one body-frame command to a robot."""

    def stop_all(self) -> None:
        """Send zero commands to all connected robots."""

    def close(self) -> None:
        """Release robot resources."""


@dataclass(frozen=True)
class VelocityLimits:
    max_vx: float = 0.20
    max_vy: float = 0.20
    max_wz: float = 0.60

    def limit_body(self, command: BodyVelocityCommand) -> BodyVelocityCommand:
        return BodyVelocityCommand(
            agent_id=command.agent_id,
            vx=clamp(command.vx, -self.max_vx, self.max_vx),
            vy=clamp(command.vy, -self.max_vy, self.max_vy),
            wz=clamp(command.wz, -self.max_wz, self.max_wz),
            mode=command.mode,
        )


@dataclass
class CentralizedCVTParams:
    domain: tuple[float, float, float, float]
    kp_xy: float = 0.45
    kd_xy: float = 0.0
    kp_yaw: float = 0.8
    target_yaw: float = 0.0
    yaw_mode: str = "face_velocity"
    grid_resolution: int = 40
    max_speed: float = 0.20


class CentralizedCVTPolicy:
    """Global CVT coverage policy used as the first real-robot baseline."""

    def __init__(self, agent_ids: list[str], params: CentralizedCVTParams):
        self.agent_ids = list(agent_ids)
        self.params = params

    def compute(self, snapshot: WorldSnapshot) -> dict[str, AgentVelocityCommand]:
        tracked = [
            snapshot.agents[agent_id]
            for agent_id in self.agent_ids
            if agent_id in snapshot.agents and snapshot.agents[agent_id].tracked
        ]
        if not tracked:
            return self._zero_commands("cvt_untracked")

        positions = np.array([[agent.x, agent.y] for agent in tracked], dtype=float)
        centroids = compute_grid_centroids(positions, self.params.domain, self.params.grid_resolution)
        commands = self._zero_commands("cvt_untracked")
        for agent, centroid in zip(tracked, centroids):
            vx_world = self.params.kp_xy * (float(centroid[0]) - agent.x) - self.params.kd_xy * agent.vx
            vy_world = self.params.kp_xy * (float(centroid[1]) - agent.y) - self.params.kd_xy * agent.vy
            vx_world, vy_world = limit_vector(vx_world, vy_world, self.params.max_speed)
            wz = self._yaw_rate(agent, vx_world, vy_world)
            commands[agent.agent_id] = AgentVelocityCommand(
                agent_id=agent.agent_id,
                vx_world=vx_world,
                vy_world=vy_world,
                wz=wz,
                mode="centralized_cvt",
            )
        return commands

    def _yaw_rate(self, agent: TrackedAgentState, vx_world: float, vy_world: float) -> float:
        if self.params.yaw_mode == "fixed":
            return self.params.kp_yaw * wrap_angle_rad(self.params.target_yaw - agent.yaw)
        if self.params.yaw_mode == "face_velocity" and math.hypot(vx_world, vy_world) > 1e-6:
            target_yaw = math.atan2(vy_world, vx_world)
            return self.params.kp_yaw * wrap_angle_rad(target_yaw - agent.yaw)
        return 0.0

    def _zero_commands(self, mode: str) -> dict[str, AgentVelocityCommand]:
        return {agent_id: AgentVelocityCommand(agent_id=agent_id, mode=mode) for agent_id in self.agent_ids}


class DecentralizedDBACTPolicy:
    """Drop-in policy for the later decentralized boundary-aware CVT stage."""

    def __init__(
        self,
        agent_ids: list[str],
        domain: tuple[float, float, float, float],
        params: DBACTParams | None = None,
    ):
        self.agent_ids = list(agent_ids)
        self.controller = DBACTController(params or DBACTParams(task_mode="coverage"), domain)
        self._last_timestamp: float | None = None

    def compute(self, snapshot: WorldSnapshot) -> dict[str, AgentVelocityCommand]:
        tracked = [
            snapshot.agents[agent_id].to_agent_state()
            for agent_id in self.agent_ids
            if agent_id in snapshot.agents and snapshot.agents[agent_id].tracked
        ]
        if not tracked:
            return {agent_id: AgentVelocityCommand(agent_id=agent_id, mode="dbact_untracked") for agent_id in self.agent_ids}

        dt = self._dt(snapshot.timestamp)
        raw_commands = self.controller.step(tracked, [], snapshot.timestamp, dt)
        by_id = {cmd.agent_id: cmd for cmd in raw_commands}
        commands: dict[str, AgentVelocityCommand] = {}
        for agent_id in self.agent_ids:
            cmd = by_id.get(agent_id)
            if cmd is None:
                commands[agent_id] = AgentVelocityCommand(agent_id=agent_id, mode="dbact_untracked")
                continue
            commands[agent_id] = AgentVelocityCommand(
                agent_id=agent_id,
                vx_world=float(cmd.velocity[0]),
                vy_world=float(cmd.velocity[1]),
                wz=0.0,
                mode=cmd.mode,
            )
        return commands

    def _dt(self, timestamp: float) -> float:
        if self._last_timestamp is None:
            self._last_timestamp = timestamp
            return 0.05
        dt = max(1e-3, min(0.2, timestamp - self._last_timestamp))
        self._last_timestamp = timestamp
        return dt


class AgentController:
    """Small orchestration layer: state -> policy -> body-frame robot backend."""

    def __init__(
        self,
        agent_ids: list[str],
        state_provider: StateProvider,
        policy: ControlPolicy,
        backend: RobotBackend,
        limits: VelocityLimits | None = None,
    ):
        self.agent_ids = list(agent_ids)
        self.state_provider = state_provider
        self.policy = policy
        self.backend = backend
        self.limits = limits or VelocityLimits()

    def connect(self) -> None:
        self.backend.connect(self.agent_ids)

    def step(self) -> list[BodyVelocityCommand]:
        snapshot = self.state_provider.snapshot()
        world_commands = self.policy.compute(snapshot)
        body_commands: list[BodyVelocityCommand] = []
        for agent_id in self.agent_ids:
            state = snapshot.agents.get(agent_id)
            world_command = world_commands.get(agent_id, AgentVelocityCommand(agent_id=agent_id, mode="missing_command"))
            if state is None or not state.tracked:
                body_command = BodyVelocityCommand(agent_id=agent_id, mode="untracked")
            else:
                body_command = world_to_body_command(state, world_command)
            limited = self.limits.limit_body(body_command)
            self.backend.send(limited)
            body_commands.append(limited)
        return body_commands

    def run(self, hz: float = 20.0, duration_s: float | None = None) -> None:
        period = 1.0 / float(hz)
        started_at = time.monotonic()
        try:
            while duration_s is None or time.monotonic() - started_at < duration_s:
                loop_start = time.monotonic()
                self.step()
                sleep_s = period - (time.monotonic() - loop_start)
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            self.backend.stop_all()

    def close(self) -> None:
        self.backend.close()


class StaticStateProvider:
    """Test and dry-run provider; update `agents` from OptiTrack later."""

    def __init__(self, agents: dict[str, TrackedAgentState], timestamp: float = 0.0):
        self.agents = dict(agents)
        self.timestamp = timestamp

    def snapshot(self) -> WorldSnapshot:
        return WorldSnapshot(timestamp=self.timestamp, agents=dict(self.agents))


class IntegratingStateProvider:
    """Dead-reckoning provider for low-speed hardware smoke tests.

    This is not a substitute for OptiTrack feedback. It only keeps the policy
    running when the first test goal is to verify multi-S1 command streaming.
    """

    def __init__(self, agents: dict[str, TrackedAgentState], timestamp: float = 0.0):
        self.agents = dict(agents)
        self.timestamp = timestamp

    def snapshot(self) -> WorldSnapshot:
        return WorldSnapshot(timestamp=self.timestamp, agents=dict(self.agents))

    def advance_body_commands(self, commands: list[BodyVelocityCommand], dt: float) -> None:
        for command in commands:
            state = self.agents.get(command.agent_id)
            if state is None or not state.tracked:
                continue
            cos_yaw = math.cos(state.yaw)
            sin_yaw = math.sin(state.yaw)
            vx_world = cos_yaw * command.vx - sin_yaw * command.vy
            vy_world = sin_yaw * command.vx + cos_yaw * command.vy
            yaw = wrap_angle_rad(state.yaw + command.wz * dt)
            self.agents[command.agent_id] = TrackedAgentState(
                agent_id=state.agent_id,
                x=state.x + vx_world * dt,
                y=state.y + vy_world * dt,
                yaw=yaw,
                vx=vx_world,
                vy=vy_world,
                wz=command.wz,
                tracked=state.tracked,
            )
        self.timestamp += dt


@dataclass
class MockRobotBackend:
    connected_ids: list[str] = field(default_factory=list)
    sent_commands: list[BodyVelocityCommand] = field(default_factory=list)

    def connect(self, agent_ids: list[str]) -> None:
        self.connected_ids = list(agent_ids)

    def send(self, command: BodyVelocityCommand) -> None:
        self.sent_commands.append(command)

    def stop_all(self) -> None:
        for agent_id in self.connected_ids:
            self.send(BodyVelocityCommand(agent_id=agent_id, mode="stop"))

    def close(self) -> None:
        self.stop_all()


class S1RoboMasterBackend:
    """Thin RoboMaster SDK backend for already-hacked STA-connected S1 robots."""

    def __init__(
        self,
        sn_by_agent_id: dict[str, str],
        conn_type: str = "sta",
        proto_type: str = "udp",
        drive_timeout_s: float = 0.10,
        z_unit: str = "deg_per_s",
    ):
        self.sn_by_agent_id = dict(sn_by_agent_id)
        self.conn_type = conn_type
        self.proto_type = proto_type
        self.drive_timeout_s = drive_timeout_s
        self.z_unit = z_unit
        self.instances: dict[str, object] = {}
        self._robot_sdk = None

    def connect(self, agent_ids: list[str]) -> None:
        from robomaster import robot

        self._robot_sdk = robot
        for agent_id in agent_ids:
            sn = self.sn_by_agent_id.get(agent_id)
            if not sn:
                raise ValueError(f"Missing SN for {agent_id}")
            ep_robot = robot.Robot()
            ep_robot.initialize(conn_type=self.conn_type, proto_type=self.proto_type, sn=sn)
            ep_robot.set_robot_mode(mode=robot.FREE)
            self.instances[agent_id] = ep_robot

    def send(self, command: BodyVelocityCommand) -> None:
        ep_robot = self.instances.get(command.agent_id)
        if ep_robot is None:
            return
        z = math.degrees(command.wz) if self.z_unit == "deg_per_s" else command.wz
        ep_robot.chassis.drive_speed(x=command.vx, y=command.vy, z=z, timeout=self.drive_timeout_s)

    def stop_all(self) -> None:
        for agent_id in list(self.instances):
            self.send(BodyVelocityCommand(agent_id=agent_id, mode="stop"))

    def close(self) -> None:
        self.stop_all()
        for ep_robot in self.instances.values():
            ep_robot.close()
        self.instances.clear()


def world_to_body_command(state: TrackedAgentState, command: AgentVelocityCommand) -> BodyVelocityCommand:
    cos_yaw = math.cos(state.yaw)
    sin_yaw = math.sin(state.yaw)
    vx_body = cos_yaw * command.vx_world + sin_yaw * command.vy_world
    vy_body = -sin_yaw * command.vx_world + cos_yaw * command.vy_world
    return BodyVelocityCommand(
        agent_id=command.agent_id,
        vx=vx_body,
        vy=vy_body,
        wz=command.wz,
        mode=command.mode,
    )


def compute_grid_centroids(
    positions: np.ndarray,
    domain: tuple[float, float, float, float],
    grid_resolution: int,
) -> np.ndarray:
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError("positions must have shape (n, 2)")
    if len(positions) == 0:
        return positions.copy()
    xmin, xmax, ymin, ymax = domain
    resolution = max(2, int(grid_resolution))
    xs = np.linspace(xmin, xmax, resolution)
    ys = np.linspace(ymin, ymax, resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    samples = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    distances = np.linalg.norm(samples[:, None, :] - positions[None, :, :], axis=2)
    owners = np.argmin(distances, axis=1)
    centroids = np.empty_like(positions, dtype=float)
    for index, position in enumerate(positions):
        owned_samples = samples[owners == index]
        centroid = owned_samples.mean(axis=0) if len(owned_samples) else position
        centroids[index] = clip_to_domain(centroid, domain)
    return centroids


def limit_vector(vx: float, vy: float, max_norm: float) -> tuple[float, float]:
    norm = math.hypot(vx, vy)
    if norm <= max_norm or norm <= 1e-12:
        return vx, vy
    scale = max_norm / norm
    return vx * scale, vy * scale


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def wrap_angle_rad(angle: float) -> float:
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
