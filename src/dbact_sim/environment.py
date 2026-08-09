"""Simulation environment.

Safety is recorded every step, not sampled at the end: ``min_t`` and ``max_t``
are the quantities the invariants are stated over, and a final-frame snapshot
cannot see a robot that passed through the cargo and came back out.

The environment produces a ``summary.json`` carrying its own provenance and its
own success verdict, so a run can be judged without re-running it and without
trusting whoever reports it.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from dbact.contracts import ClosedLoopContract, DirectionalProgressContract
from dbact.controller import DBACTController
from dbact.phase import Phase
from dbact.metrics import (
    boundary_coverage,
    directional_progress,
    min_inter_agent_distance,
    path_lengths,
    penetration_report,
    recruited_agents_count,
    strict_boundary_coverage,
)
from dbact.provenance import run_provenance
from dbact.transport_dynamics import build_engine

from .scenarios import (
    assert_initial_state_valid,
    build_agents,
    build_cargoes,
    contact_params_from_config,
    controller_params_from_config,
    domain_from_config,
    goal_directions_from_config,
    scripted_params_from_config,
    tasks_from_config,
    validate_config,
)


@dataclass
class SimulationLog:
    times: list[float] = field(default_factory=list)
    agent_positions: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_centers: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_angles: dict[str, list[float]] = field(default_factory=dict)
    cargo_vertices: dict[str, list[np.ndarray]] = field(default_factory=dict)
    min_distances: list[float] = field(default_factory=list)
    coverage: dict[str, list[float]] = field(default_factory=dict)
    strict_coverage: dict[str, list[float]] = field(default_factory=dict)
    min_clearance: dict[str, list[float]] = field(default_factory=dict)
    max_penetration: dict[str, list[float]] = field(default_factory=dict)
    agents_inside: dict[str, list[int]] = field(default_factory=dict)
    contact_counts: dict[str, list[int]] = field(default_factory=dict)
    net_force: dict[str, list[np.ndarray]] = field(default_factory=dict)
    net_torque: dict[str, list[float]] = field(default_factory=dict)
    cargo_speed: dict[str, list[float]] = field(default_factory=dict)
    mode_counts: list[dict[str, int]] = field(default_factory=list)
    # Closed-loop record. Written every frame so that the animation is a replay of
    # the run rather than a second simulation of it.
    phase: list[int] = field(default_factory=list)
    push_flags: list[np.ndarray] = field(default_factory=list)
    contact_flags: list[np.ndarray] = field(default_factory=list)
    efforts: list[np.ndarray] = field(default_factory=list)
    progress_estimate: dict[str, list[float]] = field(default_factory=dict)
    map_coverage: list[float] = field(default_factory=list)
    sensed_points: list[np.ndarray] = field(default_factory=list)


class SimulationEnvironment:
    def __init__(self, config: dict, seed: int = 0):
        validate_config(config)
        self.config = config
        self.seed = int(seed)
        self.dt = float(config.get("dt", 0.05))
        self.domain = domain_from_config(config)
        self.agents = build_agents(config, seed=self.seed)
        self.cargoes = build_cargoes(config)
        self.goal_directions = goal_directions_from_config(config)

        params = controller_params_from_config(config)
        assert_initial_state_valid(self.agents, self.cargoes, params.d_min, params.robot_radius)
        self.tasks = tasks_from_config(config, self.cargoes, seed=self.seed)
        for object_id, task in self.tasks.items():
            self.goal_directions[object_id] = task.direction
        self.controller = DBACTController(
            params, self.domain, self.goal_directions, seed=self.seed, tasks=self.tasks
        )
        self.map_snapshot_agent = str(config.get("render", {}).get("map_agent", "agent_00"))
        self.contact_params = contact_params_from_config(config)
        self.engine_name = str(config["transport"]["engine"])
        self.engine = build_engine(
            self.engine_name,
            self.contact_params,
            scripted_params_from_config(config) if self.engine_name == "scripted" else None,
        )

        self.evaluation_contact_radius = float(config.get("evaluation", {}).get("contact_radius", 0.42))
        self.success_contract = DirectionalProgressContract(
            j_min=float(config.get("evaluation", {}).get("j_min", 0.15)),
            efficiency_min=float(config.get("evaluation", {}).get("efficiency_min", 0.7)),
            displacement_gate=float(config.get("evaluation", {}).get("displacement_gate", 0.1)),
            # Filled in at summary time from the measured cargo speed; see
            # `_discrete_overshoot`.
            discrete_overshoot=0.0,
        )
        g500 = config.get("g500", {}) or {}
        unknown = sorted(set(g500) - set(ClosedLoopContract.__dataclass_fields__))
        if unknown:
            raise ValueError(f"unknown g500 gate(s) {unknown}")
        self.closed_loop_contract = ClosedLoopContract(**g500)
        self._reached_frame: dict[str, int] = {}

        self.t = 0.0
        self.log = SimulationLog()
        for a in self.agents:
            self.log.agent_positions[a.agent_id] = []
        for c in self.cargoes:
            for store in (
                self.log.cargo_centers,
                self.log.cargo_angles,
                self.log.cargo_vertices,
                self.log.coverage,
                self.log.strict_coverage,
                self.log.min_clearance,
                self.log.max_penetration,
                self.log.agents_inside,
                self.log.contact_counts,
                self.log.net_force,
                self.log.net_torque,
                self.log.cargo_speed,
                self.log.progress_estimate,
            ):
                store[c.object_id] = []
        self._last_statuses = {}

    # ------------------------------------------------------------------ #

    def step(self) -> None:
        commands = self.controller.step(self.agents, self.cargoes, self.t, self.dt)
        self.controller.apply_commands(self.agents, commands, self.dt)
        statuses = self.engine.step(self.cargoes, self.agents, self.dt)
        self._last_statuses = {s.object_id: s for s in statuses}
        self.t += self.dt
        self._record()

    def run(self, steps: int, on_frame: Callable[[int, "SimulationEnvironment"], None] | None = None) -> SimulationLog:
        self._record()
        if on_frame is not None:
            on_frame(0, self)
        for step_index in range(1, steps + 1):
            self.step()
            if on_frame is not None:
                on_frame(step_index, self)
        return self.log

    def _record(self) -> None:
        self.log.times.append(self.t)
        for a in self.agents:
            self.log.agent_positions[a.agent_id].append(a.position.copy())
        self.log.min_distances.append(min_inter_agent_distance(self.agents))
        self.log.mode_counts.append(self.controller.mode_counts())

        diagnostics = {d.agent_id: d for d in self.controller.diagnostics}
        self.log.phase.append(int(self.controller.phase))
        self.log.map_coverage.append(float(self.controller.phase_signals.map_coverage))
        self.log.push_flags.append(
            np.asarray([bool(diagnostics[a.agent_id].push_side) if a.agent_id in diagnostics else False
                        for a in self.agents])
        )
        self.log.contact_flags.append(
            np.asarray([bool(diagnostics[a.agent_id].contact_ready) if a.agent_id in diagnostics else False
                        for a in self.agents])
        )
        self.log.efforts.append(
            np.asarray([float(diagnostics[a.agent_id].effort) if a.agent_id in diagnostics else 0.0
                        for a in self.agents])
        )
        snapshot = self.controller.map_snapshot(self.map_snapshot_agent)
        self.log.sensed_points.append(snapshot.points.copy() if len(snapshot) else np.empty((0, 2)))

        frame_index = len(self.log.times) - 1
        for object_id, task in self.tasks.items():
            self.log.progress_estimate[object_id].append(
                float(self.controller.team_progress.get(object_id, 0.0))
            )
            if object_id not in self._reached_frame:
                cargo = next((c for c in self.cargoes if c.object_id == object_id), None)
                if cargo is not None and task.progress(cargo.position) >= task.distance:
                    self._reached_frame[object_id] = frame_index

        robot_radius = self.contact_params.robot_radius
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id].append(c.center.copy())
            self.log.cargo_angles[c.object_id].append(float(c.angle))
            self.log.cargo_vertices[c.object_id].append(c.vertices.copy())
            self.log.coverage[c.object_id].append(
                boundary_coverage(c, self.agents, contact_radius=self.evaluation_contact_radius)
            )
            self.log.strict_coverage[c.object_id].append(
                strict_boundary_coverage(c, self.agents, contact_radius=self.evaluation_contact_radius)
            )
            report = penetration_report(c, self.agents, robot_radius)
            self.log.min_clearance[c.object_id].append(report["min_signed_clearance"])
            self.log.max_penetration[c.object_id].append(report["max_penetration"])
            self.log.agents_inside[c.object_id].append(report["agents_inside"])

            status = self._last_statuses.get(c.object_id)
            self.log.contact_counts[c.object_id].append(status.contact_count if status else 0)
            self.log.net_force[c.object_id].append(status.net_force.copy() if status else np.zeros(2))
            self.log.net_torque[c.object_id].append(status.net_torque if status else 0.0)
            self.log.cargo_speed[c.object_id].append(float(np.linalg.norm(c.linear_velocity)))

    # ------------------------------------------------------------------ #

    def _discrete_overshoot(self) -> float:
        """Bound on how far a fixed-step integrator can leave the safe set.

        The barrier condition holds in continuous time, so between two evaluations
        the robot and the cargo can close by at most one step of relative motion.
        The robot term is its speed limit; the cargo term is the speed actually
        observed in this run rather than the engine's clamp, which is two orders of
        magnitude larger and would make the bound vacuous.
        """
        observed = max(
            (max(speeds, default=0.0) for speeds in self.log.cargo_speed.values()),
            default=0.0,
        )
        return self.dt * (self.controller.params.max_speed + observed)

    def _g500_report(
        self,
        task,
        centers: list[np.ndarray],
        entry: dict,
        solver_stats: dict,
        min_distance: float,
        params,
    ) -> dict:
        """Everything C4 needs, measured, in one flat record."""
        cid = task.object_id
        phases = self.controller.phase_monitor.as_dict()
        track = np.vstack(centers)
        progress = (track - track[0]) @ task.direction
        cross = np.abs(
            (track[:, 0] - track[0, 0]) * task.direction[1]
            - (track[:, 1] - track[0, 1]) * task.direction[0]
        )
        displacement = float(np.linalg.norm(track[-1] - track[0]))
        j = float(progress[-1])
        return {
            "engine": self.engine_name,
            "target_distance": task.distance,
            "goal_angle_deg": task.angle_deg,
            "J": j,
            "displacement": displacement,
            "efficiency": j / displacement if displacement > 1e-12 else 0.0,
            "direction_error_deg": entry.get("angle_deg"),
            "max_cross_track": float(np.max(cross)),
            "max_strict_coverage": entry["max_strict_coverage"],
            "final_strict_coverage": entry["final_strict_coverage"],
            "rotation_deg": entry["rotation_deg"],
            "final_cargo_speed": self.log.cargo_speed[cid][-1] if self.log.cargo_speed[cid] else None,
            "holding": self.controller.phase_monitor.reached(Phase.HOLD),
            "final_phase": phases["final_phase"],
            "first_detection_frame": phases["first_detection_frame"],
            "enclosure_frame": phases["enclosure_frame"],
            "contact_ready_frame": phases["contact_ready_frame"],
            "transport_frame": phases["transport_frame"],
            "brake_frame": phases["brake_frame"],
            "hold_frame": phases["hold_frame"],
            "reached_frame": self._reached_frame.get(cid),
            "progress_estimate_final": self.log.progress_estimate[cid][-1]
            if self.log.progress_estimate.get(cid)
            else None,
            # Only meaningful once the cargo has actually travelled: dividing the
            # estimate by a J of a few millimetres reports a ratio of -13 and
            # poisons any average taken over the seeds.
            "progress_estimate_ratio": (
                self.log.progress_estimate[cid][-1] / j
                if self.log.progress_estimate.get(cid) and j > 0.1
                else None
            ),
            "solver_fallbacks": solver_stats["fallbacks"],
            "solver_infeasible": solver_stats["infeasible"],
            "margin_relaxations": solver_stats["margin_relaxations"],
            "barrier_scalings": solver_stats["barrier_scalings"],
            "min_barrier_scale": solver_stats["min_barrier_scale"],
            "min_inter_agent_distance": min_distance,
            "d_min": params.d_min,
            "min_signed_clearance": entry["min_signed_clearance"],
            "max_penetration": entry["max_penetration"],
            "penetration_budget": params.delta_max + self.success_contract.discrete_overshoot,
        }

    def summary(self) -> dict:
        params = self.controller.params
        self.success_contract = replace(self.success_contract, discrete_overshoot=self._discrete_overshoot())
        solver_stats = self.controller.safety.stats.as_dict()
        min_distance = min(self.log.min_distances) if self.log.min_distances else float("inf")

        cargo_summaries: dict[str, dict] = {}
        for cargo in self.cargoes:
            cid = cargo.object_id
            centers = self.log.cargo_centers[cid]
            goal = self.goal_directions.get(cid)
            min_clearance = min(self.log.min_clearance[cid]) if self.log.min_clearance[cid] else float("inf")
            max_penetration = max(self.log.max_penetration[cid]) if self.log.max_penetration[cid] else 0.0
            contacts = self.log.contact_counts[cid]
            forces = np.vstack(self.log.net_force[cid]) if self.log.net_force[cid] else np.zeros((1, 2))

            entry = {
                "displacement_vector": (centers[-1] - centers[0]).tolist() if len(centers) >= 2 else [0.0, 0.0],
                "displacement": float(np.linalg.norm(centers[-1] - centers[0])) if len(centers) >= 2 else 0.0,
                "rotation_deg": float(np.degrees(self.log.cargo_angles[cid][-1] - self.log.cargo_angles[cid][0]))
                if len(self.log.cargo_angles[cid]) >= 2
                else 0.0,
                "final_coverage_legacy": self.log.coverage[cid][-1] if self.log.coverage[cid] else 0.0,
                "final_strict_coverage": self.log.strict_coverage[cid][-1] if self.log.strict_coverage[cid] else 0.0,
                "max_strict_coverage": max(self.log.strict_coverage[cid], default=0.0),
                "min_signed_clearance": min_clearance,
                "max_penetration": max_penetration,
                "max_agents_inside": max(self.log.agents_inside[cid], default=0),
                "mean_contacts": float(np.mean(contacts)) if contacts else 0.0,
                "max_cargo_speed": max(self.log.cargo_speed[cid], default=0.0),
                "max_contacts": int(np.max(contacts)) if contacts else 0,
                "peak_net_force": float(np.max(np.linalg.norm(forces, axis=1))),
                "recruited_agents": recruited_agents_count(cargo, self.agents, self.evaluation_contact_radius),
            }
            if goal is not None and len(centers) >= 2:
                entry.update(directional_progress(centers[0], centers[-1], goal))
                verdict = self.success_contract.evaluate(
                    centers[0],
                    centers[-1],
                    goal,
                    min_signed_clearance=min_clearance,
                    max_penetration=max_penetration,
                    delta_max=params.delta_max,
                    solver_fallbacks=solver_stats["fallbacks"],
                    min_inter_agent_distance=min_distance,
                    d_min=params.d_min,
                )
                entry["success"] = verdict.success
                entry["failure_reasons"] = verdict.reasons
            else:
                entry["success"] = None
                entry["failure_reasons"] = ["no goal direction configured for this cargo"]

            task = self.tasks.get(cid)
            if task is not None:
                report = self._g500_report(task, centers, entry, solver_stats, min_distance, params)
                g500 = self.closed_loop_contract.evaluate(report)
                entry["task"] = task.as_dict()
                entry["g500"] = {
                    "success": g500.success,
                    "failure_reasons": g500.reasons,
                    "metrics": report,
                    "gates": self.closed_loop_contract.as_dict(),
                }
                # C4 subsumes C3: a run that satisfies the directional criterion but
                # missed a deadline, drifted after the target, or leaned on a solver
                # fallback is not a closed loop, and the verdict has to say so.
                entry["success"] = bool(entry["success"]) and g500.success
                entry["failure_reasons"] = list(entry["failure_reasons"]) + list(g500.reasons)
            cargo_summaries[cid] = entry

        lengths = path_lengths(self.log.agent_positions)
        return {
            "provenance": run_provenance(self.config, self.seed, params.backend),
            "engine": self.engine_name,
            "task_mode": params.task_mode,
            "density_mode": params.density_mode,
            "steps": len(self.log.times) - 1,
            "final_time": self.log.times[-1] if self.log.times else 0.0,
            "contracts": {
                "C1": params.contact_contract().as_dict() if params.task_mode != "coverage" else None,
                "coverage": {"local_radius": params.local_radius, "comm_range": params.comm_range},
                "d_min": params.d_min,
                "delta_max": params.delta_max,
                "discrete_overshoot": self.success_contract.discrete_overshoot,
            },
            "solver": solver_stats,
            "phases": self.controller.phase_monitor.as_dict(),
            "tasks": {cid: task.as_dict() for cid, task in self.tasks.items()},
            "min_inter_agent_distance": min_distance,
            "mean_path_length": float(np.mean(list(lengths.values()))) if lengths else 0.0,
            "cargoes": cargo_summaries,
        }

    # ------------------------------------------------------------------ #

    def save_outputs(self, output_dir: str | Path, replay: bool = True) -> dict:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        summary = self.summary()
        (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self._save_trajectories(out / "trajectories.csv")
        self._save_safety_timeseries(out / "safety_timeseries.csv")
        if replay:
            self.save_replay(out / "replay.npz")
        return summary

    def save_replay(self, path: str | Path) -> Path:
        """Everything the renderer needs, and nothing it does not.

        Rendering during the run charges figure export to the simulation clock and
        makes the reported frame rate a statement about matplotlib. The run writes
        this file; the animation is produced from it afterwards, as many times as
        the figures need revising, without re-running the physics.

        The per-frame boundary points are one robot's *own* map, not the true
        outline. A density surface reconstructed from ground truth looks better and
        answers none of the questions worth asking about it.
        """
        path = Path(path)
        payload: dict[str, np.ndarray] = {
            "times": np.asarray(self.log.times),
            "phase": np.asarray(self.log.phase, dtype=np.int16),
            "min_distance": np.asarray(self.log.min_distances),
            "map_coverage": np.asarray(self.log.map_coverage),
            "push_flags": np.asarray(self.log.push_flags),
            "contact_flags": np.asarray(self.log.contact_flags),
            "efforts": np.asarray(self.log.efforts),
            "agent_ids": np.asarray([a.agent_id for a in self.agents], dtype="<U16"),
            "agent_positions": np.stack(
                [np.vstack(self.log.agent_positions[a.agent_id]) for a in self.agents], axis=1
            ),
            "domain": np.asarray(self.domain),
            "robot_radius": np.asarray(self.contact_params.robot_radius),
        }
        # Ragged per-frame map snapshots are stored flat with an offset index, so
        # the file stays a plain .npz rather than a pickle.
        counts = np.asarray([len(p) for p in self.log.sensed_points], dtype=np.int64)
        payload["sensed_counts"] = counts
        payload["sensed_points"] = (
            np.vstack([p for p in self.log.sensed_points if len(p)]) if counts.sum() else np.empty((0, 2))
        )
        for cargo in self.cargoes:
            cid = cargo.object_id
            payload[f"cargo/{cid}/centers"] = np.vstack(self.log.cargo_centers[cid])
            payload[f"cargo/{cid}/angles"] = np.asarray(self.log.cargo_angles[cid])
            payload[f"cargo/{cid}/local_vertices"] = cargo.local_vertices
            payload[f"cargo/{cid}/strict_coverage"] = np.asarray(self.log.strict_coverage[cid])
            payload[f"cargo/{cid}/contacts"] = np.asarray(self.log.contact_counts[cid])
            payload[f"cargo/{cid}/penetration"] = np.asarray(self.log.max_penetration[cid])
            payload[f"cargo/{cid}/speed"] = np.asarray(self.log.cargo_speed[cid])
            if self.log.progress_estimate.get(cid):
                payload[f"cargo/{cid}/progress_estimate"] = np.asarray(self.log.progress_estimate[cid])
            task = self.tasks.get(cid)
            if task is not None:
                payload[f"task/{cid}/direction"] = task.direction
                payload[f"task/{cid}/start"] = task.start
                payload[f"task/{cid}/goal_point"] = task.goal_point
                payload[f"task/{cid}/distance"] = np.asarray(task.distance)
        np.savez_compressed(path, **payload)
        return path

    def _save_trajectories(self, path: Path) -> None:
        lines = ["time,kind,id,x,y"]
        for ti, t in enumerate(self.log.times):
            for agent_id, hist in self.log.agent_positions.items():
                p = hist[ti]
                lines.append(f"{t:.4f},agent,{agent_id},{p[0]:.6f},{p[1]:.6f}")
            for cargo_id, hist in self.log.cargo_centers.items():
                p = hist[ti]
                lines.append(f"{t:.4f},cargo,{cargo_id},{p[0]:.6f},{p[1]:.6f}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _save_safety_timeseries(self, path: Path) -> None:
        lines = [
            "iteration,time,cargo_id,strict_coverage,legacy_coverage,min_signed_clearance,"
            "max_penetration,agents_inside,contacts,net_force_x,net_force_y,net_torque"
        ]
        for ti, t in enumerate(self.log.times):
            for cargo_id in self.log.cargo_centers:
                f = self.log.net_force[cargo_id][ti]
                lines.append(
                    f"{ti},{t:.4f},{cargo_id},"
                    f"{self.log.strict_coverage[cargo_id][ti]:.6f},"
                    f"{self.log.coverage[cargo_id][ti]:.6f},"
                    f"{self.log.min_clearance[cargo_id][ti]:.6f},"
                    f"{self.log.max_penetration[cargo_id][ti]:.6f},"
                    f"{self.log.agents_inside[cargo_id][ti]},"
                    f"{self.log.contact_counts[cargo_id][ti]},"
                    f"{f[0]:.6f},{f[1]:.6f},{self.log.net_torque[cargo_id][ti]:.6f}"
                )
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


__all__ = ["SimulationEnvironment", "SimulationLog"]
