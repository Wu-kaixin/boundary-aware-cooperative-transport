"""Compact, renderer-neutral simulation traces.

The simulation owns state evolution.  This module only copies already-produced
state and diagnostics into a serialisable record.  Renderers consume that record
offline and therefore cannot feed simulator truth back into the controller.
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - imported only by static type checkers
    from .environment import SimulationEnvironment


TRACE_SCHEMA_VERSION = 1


def _points(value: object) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.size == 0:
        return np.empty((0, 2), dtype=float)
    return array.reshape(-1, 2)


@dataclass(frozen=True)
class VisualSnapshot:
    """Display-only perception/map data captured at one simulation frame."""

    frame: int
    sensor_origins: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    detected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    mapped_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    mapped_normals: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    cage_targets: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame", int(self.frame))
        for name in (
            "sensor_origins",
            "detected_points",
            "mapped_points",
            "mapped_normals",
            "cage_targets",
        ):
            object.__setattr__(self, name, _points(getattr(self, name)).copy())


class VisualizationRecorder:
    """Read-only callback that records sparse sensor/map overlays.

    It deliberately has no mutation path back to ``SimulationEnvironment``.
    Expensive display data is sampled at ``stride`` while safety and controller
    updates continue at every physics step.
    """

    def __init__(
        self,
        stride: int = 5,
        sensor_ray_stride: int = 3,
        max_map_points: int = 600,
    ) -> None:
        self.stride = max(1, int(stride))
        self.sensor_ray_stride = max(1, int(sensor_ray_stride))
        self.max_map_points = max(1, int(max_map_points))
        self.snapshots: list[VisualSnapshot] = []

    def capture(
        self,
        step_index: int,
        env: "SimulationEnvironment",
        force: bool = False,
    ) -> None:
        if not force and step_index % self.stride != 0:
            return
        if self.snapshots and self.snapshots[-1].frame == int(step_index):
            return

        positions = {agent.agent_id: np.asarray(agent.position, dtype=float) for agent in env.agents}
        observations = [
            obs
            for agent_id in sorted(env.controller.last_sensed_observations)
            for obs in env.controller.last_sensed_observations[agent_id]
        ][:: self.sensor_ray_stride]
        detected = _points([obs.point for obs in observations])
        origins = _points([positions[obs.agent_id] for obs in observations])

        # Each local map may contain a relayed copy of the same voxel.  Select a
        # single highest-confidence record per voxel for a stable fused display.
        best: dict[tuple[str, int, int], object] = {}
        for agent_id in sorted(env.controller.maps):
            boundary_map = env.controller.maps[agent_id]
            for key in sorted(boundary_map.records):
                record = boundary_map.records[key]
                previous = best.get(key)
                if previous is None or (record.confidence, record.timestamp) > (
                    previous.confidence,
                    previous.timestamp,
                ):
                    best[key] = record
        records = [best[key] for key in sorted(best)]
        if len(records) > self.max_map_points:
            indices = np.linspace(0, len(records) - 1, self.max_map_points, dtype=int)
            records = [records[index] for index in indices]
        mapped = _points([record.point for record in records])
        normals = _points([record.normal for record in records])
        cage_targets = (
            mapped + float(env.controller.params.cage_offset) * normals
            if len(mapped)
            else np.empty((0, 2), dtype=float)
        )
        self.snapshots.append(
            VisualSnapshot(
                frame=step_index,
                sensor_origins=origins,
                detected_points=detected,
                mapped_points=mapped,
                mapped_normals=normals,
                cage_targets=cage_targets,
            )
        )


@dataclass(frozen=True)
class SimulationTrace:
    """All data required by demo, debug, and paper renderers."""

    domain: tuple[float, float, float, float]
    dt: float
    times: np.ndarray
    agent_ids: tuple[str, ...]
    cargo_ids: tuple[str, ...]
    agent_positions: np.ndarray
    agent_modes: tuple[tuple[str, ...], ...]
    cargo_centers: dict[str, np.ndarray]
    cargo_angles: dict[str, np.ndarray]
    cargo_vertices: dict[str, np.ndarray]
    goal_directions: dict[str, np.ndarray]
    goal_targets: dict[str, np.ndarray]
    strict_coverage: dict[str, np.ndarray]
    max_uncovered_gap: dict[str, np.ndarray]
    max_penetration: dict[str, np.ndarray]
    contact_counts: dict[str, np.ndarray]
    net_force: dict[str, np.ndarray]
    net_torque: dict[str, np.ndarray]
    detection_counts: dict[str, np.ndarray]
    directional_progress: dict[str, np.ndarray]
    target_distance: dict[str, float]
    progress_ratio: dict[str, np.ndarray]
    direction_efficiency: dict[str, np.ndarray]
    cross_track_error: dict[str, np.ndarray]
    cargo_rotation_deg: dict[str, np.ndarray]
    min_distances: np.ndarray
    phase_labels: tuple[str, ...]
    mode_counts: tuple[dict[str, int], ...]
    contact_ready_agents: tuple[tuple[str, ...], ...]
    push_agents: tuple[tuple[str, ...], ...]
    qp_status_counts: tuple[dict[str, int], ...]
    solver_fallbacks: np.ndarray
    solver_infeasible: np.ndarray
    settings: dict[str, float]
    visual_snapshots: tuple[VisualSnapshot, ...] = ()
    schema_version: int = TRACE_SCHEMA_VERSION

    @property
    def frame_count(self) -> int:
        return int(len(self.times))

    @classmethod
    def from_environment(
        cls,
        env: "SimulationEnvironment",
        recorder: VisualizationRecorder | None = None,
        simulation_fps: float | None = None,
    ) -> "SimulationTrace":
        log = env.log
        if not log.times:
            raise ValueError("cannot build a trace before the simulation records a frame")
        frames = len(log.times)
        agent_ids = tuple(log.agent_positions)
        cargo_ids = tuple(log.cargo_centers)
        agent_positions = np.stack(
            [np.vstack(log.agent_positions[agent_id]) for agent_id in agent_ids],
            axis=1,
        )
        agent_modes = tuple(
            tuple(log.agent_modes[agent_id][frame] for agent_id in agent_ids)
            for frame in range(frames)
        )
        contact_ready = _string_frames(log.contact_ready_agents, frames)
        push_agents = _string_frames(log.push_agents, frames)
        phase_labels = _phase_labels(env, contact_ready)

        centers = {cargo_id: np.vstack(log.cargo_centers[cargo_id]) for cargo_id in cargo_ids}
        angles = {
            cargo_id: np.asarray(log.cargo_angles[cargo_id], dtype=float)
            for cargo_id in cargo_ids
        }
        vertices = {
            cargo_id: np.stack(log.cargo_vertices[cargo_id], axis=0)
            for cargo_id in cargo_ids
        }
        goal_directions = {
            cargo_id: np.asarray(env.goal_directions.get(cargo_id, np.zeros(2)), dtype=float)
            for cargo_id in cargo_ids
        }
        goal_targets = {
            cargo_id: np.asarray(env.goal_targets[cargo_id], dtype=float)
            for cargo_id in cargo_ids
            if cargo_id in env.goal_targets
        }
        max_gap = {
            cargo_id: np.asarray(
                [
                    float(item.get("max_uncovered_arc_upper_m", np.nan))
                    for item in log.operational_enclosure[cargo_id]
                ],
                dtype=float,
            )
            for cargo_id in cargo_ids
        }

        progress: dict[str, np.ndarray] = {}
        target_distance: dict[str, float] = {}
        ratios: dict[str, np.ndarray] = {}
        efficiencies: dict[str, np.ndarray] = {}
        cross_track: dict[str, np.ndarray] = {}
        rotation: dict[str, np.ndarray] = {}
        first_transport = next(
            (index for index, phase in enumerate(phase_labels) if phase == "TRANSPORT"),
            0,
        )
        configured_distance = float(env.controller.params.transport_distance)
        for cargo_id in cargo_ids:
            goal = goal_directions[cargo_id]
            norm = float(np.linalg.norm(goal))
            goal = goal / norm if norm > 1e-12 else np.zeros(2)
            origin = centers[cargo_id][first_transport]
            displacement = centers[cargo_id] - origin
            values = displacement @ goal
            values[:first_transport] = 0.0
            displacement_norm = np.linalg.norm(displacement, axis=1)
            efficiency = np.divide(
                values,
                displacement_norm,
                out=np.zeros_like(values),
                where=displacement_norm > 1e-12,
            )
            cross = np.abs(goal[0] * displacement[:, 1] - goal[1] * displacement[:, 0])
            cross[:first_transport] = 0.0
            length = configured_distance
            if length <= 0.0 and cargo_id in goal_targets:
                length = float(np.linalg.norm(goal_targets[cargo_id] - origin))
            progress[cargo_id] = values
            target_distance[cargo_id] = float(length)
            ratios[cargo_id] = values / max(float(length), 1e-12)
            efficiencies[cargo_id] = efficiency
            cross_track[cargo_id] = cross
            rotation[cargo_id] = np.degrees(angles[cargo_id] - angles[cargo_id][0])

        snapshots = tuple(recorder.snapshots) if recorder is not None else ()
        if recorder is not None and (not snapshots or snapshots[-1].frame != frames - 1):
            recorder.capture(frames - 1, env, force=True)
            snapshots = tuple(recorder.snapshots)

        return cls(
            domain=tuple(float(value) for value in env.domain),
            dt=float(env.dt),
            times=np.asarray(log.times, dtype=float),
            agent_ids=agent_ids,
            cargo_ids=cargo_ids,
            agent_positions=agent_positions,
            agent_modes=agent_modes,
            cargo_centers=centers,
            cargo_angles=angles,
            cargo_vertices=vertices,
            goal_directions=goal_directions,
            goal_targets=goal_targets,
            strict_coverage={
                cargo_id: np.asarray(log.strict_coverage[cargo_id], dtype=float)
                for cargo_id in cargo_ids
            },
            max_uncovered_gap=max_gap,
            max_penetration={
                cargo_id: np.asarray(log.max_penetration[cargo_id], dtype=float)
                for cargo_id in cargo_ids
            },
            contact_counts={
                cargo_id: np.asarray(log.contact_counts[cargo_id], dtype=int)
                for cargo_id in cargo_ids
            },
            net_force={
                cargo_id: np.vstack(log.net_force[cargo_id]) for cargo_id in cargo_ids
            },
            net_torque={
                cargo_id: np.asarray(log.net_torque[cargo_id], dtype=float)
                for cargo_id in cargo_ids
            },
            detection_counts={
                cargo_id: np.asarray(log.detection_counts[cargo_id], dtype=int)
                for cargo_id in cargo_ids
            },
            directional_progress=progress,
            target_distance=target_distance,
            progress_ratio=ratios,
            direction_efficiency=efficiencies,
            cross_track_error=cross_track,
            cargo_rotation_deg=rotation,
            min_distances=np.asarray(log.min_distances, dtype=float),
            phase_labels=phase_labels,
            mode_counts=tuple(dict(item) for item in log.mode_counts),
            contact_ready_agents=contact_ready,
            push_agents=push_agents,
            qp_status_counts=tuple(dict(item) for item in log.qp_status_counts),
            solver_fallbacks=np.asarray(log.solver_fallbacks, dtype=int),
            solver_infeasible=np.asarray(log.solver_infeasible, dtype=int),
            settings={
                "robot_radius": float(env.controller.params.robot_radius),
                "cage_offset": float(env.controller.params.cage_offset),
                "d_min": float(env.controller.params.d_min),
                "comm_range": float(env.controller.params.comm_range),
                "simulation_fps": (
                    float(simulation_fps) if simulation_fps is not None else float("nan")
                ),
            },
            visual_snapshots=snapshots,
        )

    def visual_snapshot(self, frame: int) -> VisualSnapshot:
        """Return the newest sparse overlay snapshot available at ``frame``."""
        if not self.visual_snapshots:
            return VisualSnapshot(frame=int(frame))
        frames = [snapshot.frame for snapshot in self.visual_snapshots]
        index = max(0, bisect.bisect_right(frames, int(frame)) - 1)
        return self.visual_snapshots[index]

    def save(self, directory: str | Path) -> Path:
        output = Path(directory)
        output.mkdir(parents=True, exist_ok=True)
        arrays: dict[str, np.ndarray] = {
            "times": self.times,
            "agent_positions": self.agent_positions,
            "min_distances": self.min_distances,
            "solver_fallbacks": self.solver_fallbacks,
            "solver_infeasible": self.solver_infeasible,
        }
        for index, cargo_id in enumerate(self.cargo_ids):
            prefix = f"cargo_{index}"
            for name, source in (
                ("centers", self.cargo_centers),
                ("angles", self.cargo_angles),
                ("vertices", self.cargo_vertices),
                ("goal_direction", self.goal_directions),
                ("strict_coverage", self.strict_coverage),
                ("max_uncovered_gap", self.max_uncovered_gap),
                ("max_penetration", self.max_penetration),
                ("contact_counts", self.contact_counts),
                ("net_force", self.net_force),
                ("net_torque", self.net_torque),
                ("detection_counts", self.detection_counts),
                ("directional_progress", self.directional_progress),
                ("progress_ratio", self.progress_ratio),
                ("direction_efficiency", self.direction_efficiency),
                ("cross_track_error", self.cross_track_error),
                ("cargo_rotation_deg", self.cargo_rotation_deg),
            ):
                arrays[f"{prefix}_{name}"] = np.asarray(source[cargo_id])
            if cargo_id in self.goal_targets:
                arrays[f"{prefix}_goal_target"] = self.goal_targets[cargo_id]
        for index, snapshot in enumerate(self.visual_snapshots):
            prefix = f"overlay_{index}"
            arrays[f"{prefix}_sensor_origins"] = snapshot.sensor_origins
            arrays[f"{prefix}_detected_points"] = snapshot.detected_points
            arrays[f"{prefix}_mapped_points"] = snapshot.mapped_points
            arrays[f"{prefix}_mapped_normals"] = snapshot.mapped_normals
            arrays[f"{prefix}_cage_targets"] = snapshot.cage_targets
        np.savez_compressed(output / "arrays.npz", **arrays)

        metadata = {
            "schema_version": int(self.schema_version),
            "domain": list(self.domain),
            "dt": float(self.dt),
            "agent_ids": list(self.agent_ids),
            "cargo_ids": list(self.cargo_ids),
            "agent_modes": [list(items) for items in self.agent_modes],
            "phase_labels": list(self.phase_labels),
            "mode_counts": list(self.mode_counts),
            "contact_ready_agents": [list(items) for items in self.contact_ready_agents],
            "push_agents": [list(items) for items in self.push_agents],
            "qp_status_counts": list(self.qp_status_counts),
            "target_distance": self.target_distance,
            "settings": self.settings,
            "visual_snapshot_frames": [item.frame for item in self.visual_snapshots],
        }
        (output / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        return output

    @classmethod
    def load(cls, directory: str | Path) -> "SimulationTrace":
        source = Path(directory)
        metadata = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
        version = int(metadata.get("schema_version", 0))
        if version != TRACE_SCHEMA_VERSION:
            raise ValueError(f"unsupported trace schema {version}; expected {TRACE_SCHEMA_VERSION}")
        with np.load(source / "arrays.npz", allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        cargo_ids = tuple(str(item) for item in metadata["cargo_ids"])

        def cargo_dict(suffix: str) -> dict[str, np.ndarray]:
            return {
                cargo_id: data[f"cargo_{index}_{suffix}"]
                for index, cargo_id in enumerate(cargo_ids)
            }

        goal_targets = {
            cargo_id: data[f"cargo_{index}_goal_target"]
            for index, cargo_id in enumerate(cargo_ids)
            if f"cargo_{index}_goal_target" in data
        }
        snapshots = tuple(
            VisualSnapshot(
                frame=frame,
                sensor_origins=data[f"overlay_{index}_sensor_origins"],
                detected_points=data[f"overlay_{index}_detected_points"],
                mapped_points=data[f"overlay_{index}_mapped_points"],
                mapped_normals=data[f"overlay_{index}_mapped_normals"],
                cage_targets=data[f"overlay_{index}_cage_targets"],
            )
            for index, frame in enumerate(metadata["visual_snapshot_frames"])
        )
        return cls(
            domain=tuple(float(value) for value in metadata["domain"]),
            dt=float(metadata["dt"]),
            times=data["times"],
            agent_ids=tuple(str(item) for item in metadata["agent_ids"]),
            cargo_ids=cargo_ids,
            agent_positions=data["agent_positions"],
            agent_modes=tuple(tuple(items) for items in metadata["agent_modes"]),
            cargo_centers=cargo_dict("centers"),
            cargo_angles=cargo_dict("angles"),
            cargo_vertices=cargo_dict("vertices"),
            goal_directions=cargo_dict("goal_direction"),
            goal_targets=goal_targets,
            strict_coverage=cargo_dict("strict_coverage"),
            max_uncovered_gap=cargo_dict("max_uncovered_gap"),
            max_penetration=cargo_dict("max_penetration"),
            contact_counts=cargo_dict("contact_counts"),
            net_force=cargo_dict("net_force"),
            net_torque=cargo_dict("net_torque"),
            detection_counts=cargo_dict("detection_counts"),
            directional_progress=cargo_dict("directional_progress"),
            target_distance={key: float(value) for key, value in metadata["target_distance"].items()},
            progress_ratio=cargo_dict("progress_ratio"),
            direction_efficiency=cargo_dict("direction_efficiency"),
            cross_track_error=cargo_dict("cross_track_error"),
            cargo_rotation_deg=cargo_dict("cargo_rotation_deg"),
            min_distances=data["min_distances"],
            phase_labels=tuple(metadata["phase_labels"]),
            mode_counts=tuple(dict(item) for item in metadata["mode_counts"]),
            contact_ready_agents=tuple(tuple(items) for items in metadata["contact_ready_agents"]),
            push_agents=tuple(tuple(items) for items in metadata["push_agents"]),
            qp_status_counts=tuple(dict(item) for item in metadata["qp_status_counts"]),
            solver_fallbacks=data["solver_fallbacks"],
            solver_infeasible=data["solver_infeasible"],
            settings={key: float(value) for key, value in metadata["settings"].items()},
            visual_snapshots=snapshots,
            schema_version=version,
        )


def _string_frames(values: list[list[str]], frames: int) -> tuple[tuple[str, ...], ...]:
    padded = list(values) + [[] for _ in range(max(0, frames - len(values)))]
    return tuple(tuple(str(item) for item in frame) for frame in padded[:frames])


def _phase_labels(
    env: "SimulationEnvironment",
    contact_ready: tuple[tuple[str, ...], ...],
) -> tuple[str, ...]:
    labels: list[str] = []
    for frame, modes in enumerate(env.log.mode_counts):
        supervisor = (
            env.log.transport_phase_counts[frame]
            if frame < len(env.log.transport_phase_counts)
            else {}
        )
        detections = sum(
            values[frame]
            for values in env.log.detection_counts.values()
            if frame < len(values)
        )
        if supervisor.get("hold", 0) > 0 or modes.get("hold", 0) > 0:
            label = "HOLD"
        elif supervisor.get("brake", 0) > 0 or modes.get("brake", 0) > 0:
            label = "BRAKE"
        elif supervisor.get("transport", 0) > 0 or modes.get("push", 0) + modes.get("convoy", 0) > 0:
            label = "TRANSPORT"
        elif frame < len(contact_ready) and contact_ready[frame]:
            label = "CONTACT_READY"
        elif any(modes.get(name, 0) > 0 for name in ("cage", "approach", "redeploy")):
            label = "ENCLOSE"
        elif detections > 0 or any(modes.get(name, 0) > 0 for name in ("map_boundary", "relay")):
            label = "MAP"
        else:
            label = "SEARCH"
        labels.append(label)
    return tuple(labels)


__all__ = [
    "TRACE_SCHEMA_VERSION",
    "SimulationTrace",
    "VisualSnapshot",
    "VisualizationRecorder",
]
