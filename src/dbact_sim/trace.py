"""Immutable, renderer-neutral traces for the Claude v2 research core.

The simulation owns state evolution.  This module only copies quantities already
recorded by ``SimulationEnvironment`` or its replay file.  It never writes to the
controller, physics engine, maps, tasks, or safety filter.
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dbact.phase import Phase

if TYPE_CHECKING:  # pragma: no cover
    from .environment import SimulationEnvironment


TRACE_SCHEMA_VERSION = 2


def _points(value: object) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.size == 0:
        return np.empty((0, 2), dtype=float)
    return array.reshape(-1, 2)


def _padded_rows(values: list, frames: int, width: int, dtype=bool) -> np.ndarray:
    result = np.zeros((frames, width), dtype=dtype)
    for frame, value in enumerate(values[:frames]):
        row = np.asarray(value, dtype=dtype).reshape(-1)
        result[frame, : min(width, len(row))] = row[:width]
    return result


def _phase_labels(values: list[int], frames: int) -> tuple[str, ...]:
    raw = list(values[:frames])
    raw.extend([raw[-1] if raw else int(Phase.SEARCH)] * max(0, frames - len(raw)))
    labels: list[str] = []
    for value in raw:
        try:
            label = Phase(int(value)).label
        except (ValueError, TypeError):
            label = "SEARCH"
        labels.append("MAP" if label == "DISCOVER" else label)
    return tuple(labels)


def _mode_for_phase(phase: str) -> str:
    return {
        "SEARCH": "search",
        "MAP": "map_boundary",
        "ENCLOSE": "cage",
        "CONTACT_READY": "cage",
        "TRANSPORT": "push",
        "BRAKE": "brake",
        "HOLD": "hold",
    }.get(phase, "search")


@dataclass(frozen=True)
class VisualSnapshot:
    """Sparse display-only perception and map data for one frame."""

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


@dataclass(frozen=True)
class FrameDiagnostics:
    """Lightweight per-frame diagnostics captured without changing simulation."""

    frame: int
    agent_modes: tuple[str, ...]
    qp_status_counts: dict[str, int]
    solver_fallbacks: int
    solver_infeasible: int
    observation_counts: dict[str, int]


class VisualizationRecorder:
    """Read-only callback for exact diagnostics and sparse visual overlays."""

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
        self.diagnostics: list[FrameDiagnostics] = []

    def capture(
        self,
        step_index: int,
        env: "SimulationEnvironment",
        force: bool = False,
    ) -> None:
        frame = int(step_index)
        self._capture_diagnostics(frame, env)
        if not force and frame % self.stride != 0:
            return
        if self.snapshots and self.snapshots[-1].frame == frame:
            return

        positions = {
            agent.agent_id: np.asarray(agent.position, dtype=float)
            for agent in env.agents
        }
        detected_parts: list[np.ndarray] = []
        origin_parts: list[np.ndarray] = []
        scans = getattr(env.controller, "last_scans", {})
        for agent_id in sorted(scans):
            scan = scans[agent_id]
            points = _points(getattr(scan, "points", np.empty((0, 2))))
            points = points[:: self.sensor_ray_stride]
            if len(points):
                detected_parts.append(points)
                origin_parts.append(np.repeat(positions[agent_id][None, :], len(points), axis=0))
        detected = np.vstack(detected_parts) if detected_parts else np.empty((0, 2))
        origins = np.vstack(origin_parts) if origin_parts else np.empty((0, 2))

        view = env.controller.map_snapshot(env.map_snapshot_agent)
        mapped = _points(getattr(view, "points", np.empty((0, 2))))
        normals = _points(getattr(view, "normals", np.empty((0, 2))))
        if len(mapped) > self.max_map_points:
            indices = np.linspace(0, len(mapped) - 1, self.max_map_points, dtype=int)
            mapped = mapped[indices]
            normals = normals[indices] if len(normals) == len(getattr(view, "points", [])) else np.empty((0, 2))
        cage = (
            mapped + float(env.controller.params.cage_offset) * normals
            if len(mapped) and len(normals) == len(mapped)
            else np.empty((0, 2))
        )
        self.snapshots.append(
            VisualSnapshot(
                frame=frame,
                sensor_origins=origins,
                detected_points=detected,
                mapped_points=mapped,
                mapped_normals=normals,
                cage_targets=cage,
            )
        )

    def _capture_diagnostics(self, frame: int, env: "SimulationEnvironment") -> None:
        if self.diagnostics and self.diagnostics[-1].frame == frame:
            return
        modes = {item.agent_id: item.mode for item in env.controller.diagnostics}
        statuses: dict[str, int] = {}
        for item in env.controller.diagnostics:
            statuses[item.solver_status] = statuses.get(item.solver_status, 0) + 1
        stats = env.controller.safety.stats
        view = env.controller.map_snapshot(env.map_snapshot_agent)
        object_ids = np.asarray(getattr(view, "object_ids", []), dtype=str)
        counts = {
            cargo.object_id: int(np.count_nonzero(object_ids == cargo.object_id))
            for cargo in env.cargoes
        }
        self.diagnostics.append(
            FrameDiagnostics(
                frame=frame,
                agent_modes=tuple(modes.get(agent.agent_id, "") for agent in env.agents),
                qp_status_counts=statuses,
                solver_fallbacks=int(getattr(stats, "fallbacks", 0)),
                solver_infeasible=int(getattr(stats, "infeasible", 0)),
                observation_counts=counts,
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
    settings: dict[str, Any]
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
        agent_ids = tuple(agent.agent_id for agent in env.agents)
        cargo_ids = tuple(cargo.object_id for cargo in env.cargoes)
        positions = np.stack(
            [np.vstack(log.agent_positions[agent_id]) for agent_id in agent_ids],
            axis=1,
        )
        phases = _phase_labels(log.phase, frames)
        contact_flags = _padded_rows(log.contact_flags, frames, len(agent_ids))
        push_flags = _padded_rows(log.push_flags, frames, len(agent_ids))
        diagnostics = {item.frame: item for item in recorder.diagnostics} if recorder else {}

        agent_modes: list[tuple[str, ...]] = []
        qp_status: list[dict[str, int]] = []
        fallbacks = np.full(frames, -1, dtype=int)
        infeasible = np.full(frames, -1, dtype=int)
        observations = {cargo_id: np.zeros(frames, dtype=int) for cargo_id in cargo_ids}
        for frame in range(frames):
            item = diagnostics.get(frame)
            modes = []
            for index in range(len(agent_ids)):
                recorded = item.agent_modes[index] if item and index < len(item.agent_modes) else ""
                if recorded:
                    mode = recorded
                elif push_flags[frame, index]:
                    mode = "push"
                elif contact_flags[frame, index]:
                    mode = "cage"
                else:
                    mode = _mode_for_phase(phases[frame])
                modes.append(mode)
            agent_modes.append(tuple(modes))
            qp_status.append(dict(item.qp_status_counts) if item else {})
            if item:
                fallbacks[frame] = item.solver_fallbacks
                infeasible[frame] = item.solver_infeasible
                for cargo_id in cargo_ids:
                    observations[cargo_id][frame] = item.observation_counts.get(cargo_id, 0)
            elif frame < len(log.sensed_points) and cargo_ids:
                observations[cargo_ids[0]][frame] = len(log.sensed_points[frame])

        if not diagnostics:
            stats = env.controller.safety.stats
            fallbacks[-1] = int(getattr(stats, "fallbacks", 0))
            infeasible[-1] = int(getattr(stats, "infeasible", 0))

        centers = {cargo_id: np.vstack(log.cargo_centers[cargo_id]) for cargo_id in cargo_ids}
        angles = {
            cargo_id: np.asarray(log.cargo_angles[cargo_id], dtype=float)
            for cargo_id in cargo_ids
        }
        vertices = {
            cargo_id: np.stack(log.cargo_vertices[cargo_id], axis=0)
            for cargo_id in cargo_ids
        }
        directions = {
            cargo_id: np.asarray(env.goal_directions.get(cargo_id, np.zeros(2)), dtype=float)
            for cargo_id in cargo_ids
        }
        targets = {
            cargo_id: np.asarray(env.tasks[cargo_id].goal_point, dtype=float)
            for cargo_id in cargo_ids
            if cargo_id in env.tasks
        }

        progress: dict[str, np.ndarray] = {}
        target_distance: dict[str, float] = {}
        ratios: dict[str, np.ndarray] = {}
        efficiencies: dict[str, np.ndarray] = {}
        cross_track: dict[str, np.ndarray] = {}
        rotation: dict[str, np.ndarray] = {}
        for cargo_id in cargo_ids:
            goal = directions[cargo_id]
            norm = float(np.linalg.norm(goal))
            goal = goal / norm if norm > 1e-12 else np.zeros(2)
            task = env.tasks.get(cargo_id)
            origin = np.asarray(task.start, dtype=float) if task is not None else centers[cargo_id][0]
            displacement = centers[cargo_id] - origin
            values = displacement @ goal
            displacement_norm = np.linalg.norm(displacement, axis=1)
            efficiency = np.divide(
                values,
                displacement_norm,
                out=np.zeros_like(values),
                where=displacement_norm > 1e-12,
            )
            cross = np.abs(goal[0] * displacement[:, 1] - goal[1] * displacement[:, 0])
            length = float(task.distance) if task is not None else float("nan")
            progress[cargo_id] = values
            target_distance[cargo_id] = length
            ratios[cargo_id] = (
                values / length
                if np.isfinite(length) and length > 1e-12
                else np.full(frames, np.nan)
            )
            efficiencies[cargo_id] = efficiency
            cross_track[cargo_id] = cross
            rotation[cargo_id] = np.degrees(angles[cargo_id] - angles[cargo_id][0])

        snapshots = tuple(recorder.snapshots) if recorder is not None else ()
        if recorder is not None and (not snapshots or snapshots[-1].frame != frames - 1):
            recorder.capture(frames - 1, env, force=True)
            snapshots = tuple(recorder.snapshots)
        elif recorder is None:
            snapshots = tuple(
                VisualSnapshot(frame=frame, mapped_points=log.sensed_points[frame])
                for frame in range(min(frames, len(log.sensed_points)))
            )

        return cls(
            domain=tuple(float(value) for value in env.domain),
            dt=float(env.dt),
            times=np.asarray(log.times, dtype=float),
            agent_ids=agent_ids,
            cargo_ids=cargo_ids,
            agent_positions=positions,
            agent_modes=tuple(agent_modes),
            cargo_centers=centers,
            cargo_angles=angles,
            cargo_vertices=vertices,
            goal_directions=directions,
            goal_targets=targets,
            strict_coverage={
                cargo_id: np.asarray(log.strict_coverage[cargo_id], dtype=float)
                for cargo_id in cargo_ids
            },
            max_uncovered_gap={
                cargo_id: np.full(frames, np.nan, dtype=float) for cargo_id in cargo_ids
            },
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
            detection_counts=observations,
            directional_progress=progress,
            target_distance=target_distance,
            progress_ratio=ratios,
            direction_efficiency=efficiencies,
            cross_track_error=cross_track,
            cargo_rotation_deg=rotation,
            min_distances=np.asarray(log.min_distances, dtype=float),
            phase_labels=phases,
            mode_counts=tuple(dict(item) for item in log.mode_counts),
            contact_ready_agents=tuple(
                tuple(agent_ids[index] for index in np.flatnonzero(contact_flags[frame]))
                for frame in range(frames)
            ),
            push_agents=tuple(
                tuple(agent_ids[index] for index in np.flatnonzero(push_flags[frame]))
                for frame in range(frames)
            ),
            qp_status_counts=tuple(qp_status),
            solver_fallbacks=fallbacks,
            solver_infeasible=infeasible,
            settings={
                "robot_radius": float(env.controller.params.robot_radius),
                "cage_offset": float(env.controller.params.cage_offset),
                "d_min": float(env.controller.params.d_min),
                "comm_range": float(env.controller.params.comm_range),
                "simulation_fps": (
                    float(simulation_fps) if simulation_fps is not None else float("nan")
                ),
                "source": "claude_v2_environment",
                "observation_metric": "local mapped boundary points",
                "max_uncovered_gap_available": False,
                "ground_truth_scope": "evaluation_and_rendering_only",
            },
            visual_snapshots=snapshots,
        )

    @classmethod
    def from_replay(cls, path: str | Path) -> "SimulationTrace":
        """Adapt Claude v2 ``replay.npz`` without re-running the simulation."""
        source = Path(path)
        with np.load(source, allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        times = np.asarray(data["times"], dtype=float)
        frames = len(times)
        agent_ids = tuple(str(item) for item in data["agent_ids"])
        cargo_ids = tuple(
            sorted(
                key.split("/")[1]
                for key in data
                if key.startswith("cargo/") and key.endswith("/centers")
            )
        )
        phases = _phase_labels(data["phase"].tolist(), frames)
        contact_flags = np.asarray(data["contact_flags"], dtype=bool)
        push_flags = np.asarray(data["push_flags"], dtype=bool)
        agent_modes = tuple(
            tuple(
                "push"
                if push_flags[frame, index]
                else "cage"
                if contact_flags[frame, index]
                else _mode_for_phase(phases[frame])
                for index in range(len(agent_ids))
            )
            for frame in range(frames)
        )

        centers = {
            cargo_id: np.asarray(data[f"cargo/{cargo_id}/centers"], dtype=float)
            for cargo_id in cargo_ids
        }
        angles = {
            cargo_id: np.asarray(data[f"cargo/{cargo_id}/angles"], dtype=float)
            for cargo_id in cargo_ids
        }
        vertices: dict[str, np.ndarray] = {}
        for cargo_id in cargo_ids:
            local = np.asarray(data[f"cargo/{cargo_id}/local_vertices"], dtype=float)
            history = []
            for center, angle in zip(centers[cargo_id], angles[cargo_id]):
                cosine, sine = np.cos(angle), np.sin(angle)
                rotation_matrix = np.array([[cosine, -sine], [sine, cosine]])
                history.append(local @ rotation_matrix.T + center)
            vertices[cargo_id] = np.stack(history)

        directions = {
            cargo_id: np.asarray(
                data.get(f"task/{cargo_id}/direction", np.zeros(2)), dtype=float
            )
            for cargo_id in cargo_ids
        }
        targets = {
            cargo_id: np.asarray(data[f"task/{cargo_id}/goal_point"], dtype=float)
            for cargo_id in cargo_ids
            if f"task/{cargo_id}/goal_point" in data
        }
        progress: dict[str, np.ndarray] = {}
        target_distance: dict[str, float] = {}
        ratios: dict[str, np.ndarray] = {}
        efficiencies: dict[str, np.ndarray] = {}
        cross_track: dict[str, np.ndarray] = {}
        rotation: dict[str, np.ndarray] = {}
        for cargo_id in cargo_ids:
            goal = directions[cargo_id]
            norm = float(np.linalg.norm(goal))
            goal = goal / norm if norm > 1e-12 else np.zeros(2)
            origin = np.asarray(
                data.get(f"task/{cargo_id}/start", centers[cargo_id][0]), dtype=float
            )
            displacement = centers[cargo_id] - origin
            values = displacement @ goal
            displacement_norm = np.linalg.norm(displacement, axis=1)
            progress[cargo_id] = values
            length = float(data[f"task/{cargo_id}/distance"]) if f"task/{cargo_id}/distance" in data else float("nan")
            target_distance[cargo_id] = length
            ratios[cargo_id] = values / length if np.isfinite(length) and length > 0 else np.full(frames, np.nan)
            efficiencies[cargo_id] = np.divide(
                values,
                displacement_norm,
                out=np.zeros_like(values),
                where=displacement_norm > 1e-12,
            )
            cross_track[cargo_id] = np.abs(
                goal[0] * displacement[:, 1] - goal[1] * displacement[:, 0]
            )
            rotation[cargo_id] = np.degrees(angles[cargo_id] - angles[cargo_id][0])

        counts = np.asarray(data.get("sensed_counts", np.zeros(frames)), dtype=int)
        flat = _points(data.get("sensed_points", np.empty((0, 2))))
        snapshots: list[VisualSnapshot] = []
        offset = 0
        for frame, count in enumerate(counts):
            points = flat[offset : offset + count]
            offset += int(count)
            snapshots.append(VisualSnapshot(frame=frame, mapped_points=points))
        observation_counts = {
            cargo_id: (counts.copy() if index == 0 else np.zeros(frames, dtype=int))
            for index, cargo_id in enumerate(cargo_ids)
        }
        unavailable_vector = {
            cargo_id: np.full((frames, 2), np.nan) for cargo_id in cargo_ids
        }
        unavailable_scalar = {
            cargo_id: np.full(frames, np.nan) for cargo_id in cargo_ids
        }
        mode_counts = []
        for modes in agent_modes:
            row: dict[str, int] = {}
            for mode in modes:
                row[mode] = row.get(mode, 0) + 1
            mode_counts.append(row)

        dt = float(np.median(np.diff(times))) if frames > 1 else 0.05
        return cls(
            domain=tuple(float(value) for value in data["domain"]),
            dt=dt,
            times=times,
            agent_ids=agent_ids,
            cargo_ids=cargo_ids,
            agent_positions=np.asarray(data["agent_positions"], dtype=float),
            agent_modes=agent_modes,
            cargo_centers=centers,
            cargo_angles=angles,
            cargo_vertices=vertices,
            goal_directions=directions,
            goal_targets=targets,
            strict_coverage={
                cargo_id: np.asarray(data[f"cargo/{cargo_id}/strict_coverage"], dtype=float)
                for cargo_id in cargo_ids
            },
            max_uncovered_gap={key: value.copy() for key, value in unavailable_scalar.items()},
            max_penetration={
                cargo_id: np.asarray(data[f"cargo/{cargo_id}/penetration"], dtype=float)
                for cargo_id in cargo_ids
            },
            contact_counts={
                cargo_id: np.asarray(data[f"cargo/{cargo_id}/contacts"], dtype=int)
                for cargo_id in cargo_ids
            },
            net_force=unavailable_vector,
            net_torque={key: value.copy() for key, value in unavailable_scalar.items()},
            detection_counts=observation_counts,
            directional_progress=progress,
            target_distance=target_distance,
            progress_ratio=ratios,
            direction_efficiency=efficiencies,
            cross_track_error=cross_track,
            cargo_rotation_deg=rotation,
            min_distances=np.asarray(data["min_distance"], dtype=float),
            phase_labels=phases,
            mode_counts=tuple(mode_counts),
            contact_ready_agents=tuple(
                tuple(agent_ids[index] for index in np.flatnonzero(contact_flags[frame]))
                for frame in range(frames)
            ),
            push_agents=tuple(
                tuple(agent_ids[index] for index in np.flatnonzero(push_flags[frame]))
                for frame in range(frames)
            ),
            qp_status_counts=tuple({} for _ in range(frames)),
            solver_fallbacks=np.full(frames, -1, dtype=int),
            solver_infeasible=np.full(frames, -1, dtype=int),
            settings={
                "robot_radius": float(data.get("robot_radius", 0.12)),
                "cage_offset": float("nan"),
                "d_min": float("nan"),
                "comm_range": float("nan"),
                "simulation_fps": float("nan"),
                "source": "claude_v2_replay",
                "observation_metric": "local mapped boundary points",
                "max_uncovered_gap_available": False,
                "solver_timeseries_available": False,
                "net_wrench_available": False,
                "ground_truth_scope": "evaluation_and_rendering_only",
            },
            visual_snapshots=tuple(snapshots),
        )

    def visual_snapshot(self, frame: int) -> VisualSnapshot:
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
            raise ValueError(
                f"unsupported trace schema {version}; expected {TRACE_SCHEMA_VERSION}"
            )
        with np.load(source / "arrays.npz", allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        cargo_ids = tuple(str(item) for item in metadata["cargo_ids"])

        def cargo_dict(suffix: str) -> dict[str, np.ndarray]:
            return {
                cargo_id: data[f"cargo_{index}_{suffix}"]
                for index, cargo_id in enumerate(cargo_ids)
            }

        targets = {
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
            goal_targets=targets,
            strict_coverage=cargo_dict("strict_coverage"),
            max_uncovered_gap=cargo_dict("max_uncovered_gap"),
            max_penetration=cargo_dict("max_penetration"),
            contact_counts=cargo_dict("contact_counts"),
            net_force=cargo_dict("net_force"),
            net_torque=cargo_dict("net_torque"),
            detection_counts=cargo_dict("detection_counts"),
            directional_progress=cargo_dict("directional_progress"),
            target_distance={
                key: float(value) for key, value in metadata["target_distance"].items()
            },
            progress_ratio=cargo_dict("progress_ratio"),
            direction_efficiency=cargo_dict("direction_efficiency"),
            cross_track_error=cargo_dict("cross_track_error"),
            cargo_rotation_deg=cargo_dict("cargo_rotation_deg"),
            min_distances=data["min_distances"],
            phase_labels=tuple(metadata["phase_labels"]),
            mode_counts=tuple(dict(item) for item in metadata["mode_counts"]),
            contact_ready_agents=tuple(
                tuple(items) for items in metadata["contact_ready_agents"]
            ),
            push_agents=tuple(tuple(items) for items in metadata["push_agents"]),
            qp_status_counts=tuple(
                dict(item) for item in metadata["qp_status_counts"]
            ),
            solver_fallbacks=data["solver_fallbacks"],
            solver_infeasible=data["solver_infeasible"],
            settings=dict(metadata["settings"]),
            visual_snapshots=snapshots,
            schema_version=version,
        )


__all__ = [
    "TRACE_SCHEMA_VERSION",
    "FrameDiagnostics",
    "SimulationTrace",
    "VisualSnapshot",
    "VisualizationRecorder",
]
