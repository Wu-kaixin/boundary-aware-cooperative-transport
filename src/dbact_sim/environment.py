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

from dbact.contracts import DirectionalProgressContract
from dbact.controller import DBACTController
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
    goal_targets_from_config,
    scripted_params_from_config,
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
    agent_modes: dict[str, list[str]] = field(default_factory=dict)


class SimulationEnvironment:
    def __init__(self, config: dict, seed: int = 0):
        validate_config(config)
        self.config = config
        self.seed = int(seed)
        self.dt = float(config.get("dt", 0.05))
        self.domain = domain_from_config(config)
        self.agents = build_agents(config, seed=self.seed)
        self.cargoes = build_cargoes(config, seed=self.seed, agents=self.agents)
        self.goal_directions = goal_directions_from_config(config, seed=self.seed, cargoes=self.cargoes)
        self.goal_targets = goal_targets_from_config(config, self.cargoes, self.goal_directions)

        params = controller_params_from_config(config)
        assert_initial_state_valid(self.agents, self.cargoes, params.d_min, params.robot_radius)
        self.controller = DBACTController(params, self.domain, self.goal_directions, seed=self.seed)
        self.contact_params = contact_params_from_config(config)
        self.engine_name = str(config["transport"]["engine"])
        self.engine = build_engine(
            self.engine_name,
            self.contact_params,
            scripted_params_from_config(config, seed=self.seed, cargoes=self.cargoes)
            if self.engine_name == "scripted"
            else None,
        )

        self.evaluation_contact_radius = float(config.get("evaluation", {}).get("contact_radius", 0.42))
        self.success_contract = DirectionalProgressContract(
            j_min=float(config.get("evaluation", {}).get("j_min", 0.15)),
            j_max=float(config.get("evaluation", {}).get("j_max", float("inf"))),
            efficiency_min=float(config.get("evaluation", {}).get("efficiency_min", 0.7)),
            displacement_gate=float(config.get("evaluation", {}).get("displacement_gate", 0.1)),
            coverage_min=float(config.get("evaluation", {}).get("coverage_min", 0.0)),
            max_rotation_deg=float(config.get("evaluation", {}).get("max_rotation_deg", float("inf"))),
            # Filled in at summary time from the measured cargo speed; see
            # `_discrete_overshoot`.
            discrete_overshoot=0.0,
        )
        evaluation = config.get("evaluation", {})
        self.require_initially_unobserved = bool(evaluation.get("require_initially_unobserved", False))
        self.frame_budget = evaluation.get("frame_budget")
        self.phase_deadlines = {
            "first_detection": evaluation.get("detection_deadline"),
            "first_enclosure": evaluation.get("enclosure_deadline"),
            "first_transport": evaluation.get("transport_deadline"),
            "first_hold": evaluation.get("hold_deadline"),
        }
        self.initial_detection_counts = {cargo.object_id: 0 for cargo in self.cargoes}
        for agent in self.agents:
            for observation in self.controller.sensor.sense(agent, self.cargoes, 0.0):
                self.initial_detection_counts[observation.object_id] += 1

        self.t = 0.0
        self.log = SimulationLog()
        for a in self.agents:
            self.log.agent_positions[a.agent_id] = []
            self.log.agent_modes[a.agent_id] = []
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
        mode_by_agent = {diag.agent_id: diag.mode for diag in self.controller.diagnostics}
        for a in self.agents:
            initial_mode = "explore" if self.controller.params.task_mode != "coverage" else "search"
            self.log.agent_modes[a.agent_id].append(mode_by_agent.get(a.agent_id, initial_mode))
        self.log.min_distances.append(min_inter_agent_distance(self.agents))
        self.log.mode_counts.append(self.controller.mode_counts())

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
            rotation_deg = (
                float(np.degrees(self.log.cargo_angles[cid][-1] - self.log.cargo_angles[cid][0]))
                if len(self.log.cargo_angles[cid]) >= 2
                else 0.0
            )

            entry = {
                "displacement_vector": (centers[-1] - centers[0]).tolist() if len(centers) >= 2 else [0.0, 0.0],
                "displacement": float(np.linalg.norm(centers[-1] - centers[0])) if len(centers) >= 2 else 0.0,
                "rotation_deg": rotation_deg,
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
                "initial_detection_count": self.initial_detection_counts.get(cid, 0),
            }
            first_detection = next(
                (
                    k
                    for k, modes in enumerate(self.log.mode_counts)
                    if any(name not in {"explore", "search"} and count > 0 for name, count in modes.items())
                ),
                None,
            )
            first_enclosure = next(
                (k for k, value in enumerate(self.log.strict_coverage[cid]) if value >= self.success_contract.coverage_min),
                None,
            )
            first_transport = next(
                (
                    k
                    for k, modes in enumerate(self.log.mode_counts)
                    if modes.get("push", 0) + modes.get("convoy", 0) > 0
                ),
                None,
            )
            first_hold = next(
                (k for k, modes in enumerate(self.log.mode_counts) if modes.get("hold", 0) > 0),
                None,
            )
            entry["phase_frames"] = {
                "first_detection": first_detection,
                "first_enclosure": first_enclosure,
                "first_transport": first_transport,
                "first_hold": first_hold,
            }
            if goal is not None and len(centers) >= 2:
                entry["goal_direction"] = np.asarray(goal, dtype=float).tolist()
                entry["goal_angle_deg"] = float(np.degrees(np.arctan2(goal[1], goal[0])))
                if cid in self.goal_targets:
                    entry["goal_target"] = self.goal_targets[cid].tolist()
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
                    final_strict_coverage=entry["final_strict_coverage"],
                    rotation_deg=rotation_deg,
                )
                phase_reasons: list[str] = []
                if self.require_initially_unobserved and self.initial_detection_counts.get(cid, 0) > 0:
                    phase_reasons.append(
                        f"phase gate: {self.initial_detection_counts[cid]} boundary returns existed at frame 0; "
                        "the episode did not start from an unknown object"
                    )
                if first_detection is None:
                    phase_reasons.append("phase gate: cargo was never detected")
                if first_enclosure is None:
                    phase_reasons.append("phase gate: enclosure threshold was never reached")
                if first_transport is None:
                    phase_reasons.append("phase gate: transport phase never activated")
                if (
                    first_enclosure is not None
                    and first_transport is not None
                    and first_transport < first_enclosure
                ):
                    phase_reasons.append(
                        f"phase gate: transport started at frame {first_transport} before enclosure at frame {first_enclosure}"
                    )
                for phase_name, frame in entry["phase_frames"].items():
                    deadline = self.phase_deadlines.get(phase_name)
                    if deadline is None:
                        continue
                    if frame is None:
                        phase_reasons.append(f"phase deadline: {phase_name} was not reached by frame {int(deadline)}")
                    elif frame > int(deadline):
                        phase_reasons.append(
                            f"phase deadline: {phase_name}={frame} exceeded frame {int(deadline)}"
                        )
                entry["success"] = verdict.success and not phase_reasons
                entry["failure_reasons"] = verdict.reasons + phase_reasons
            else:
                entry["success"] = None
                entry["failure_reasons"] = ["no goal direction configured for this cargo"]
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
                "require_initially_unobserved": self.require_initially_unobserved,
                "phase_deadlines": self.phase_deadlines,
                "frame_budget": self.frame_budget,
            },
            "solver": solver_stats,
            "transport_progress_estimates": self.controller.transport_progress_summary(),
            "goal_targets": {key: value.tolist() for key, value in self.goal_targets.items()},
            "multi_rate": {
                "perception_every": params.perception_every,
                "planning_every": params.planning_every,
                "safety_every": 1,
            },
            "min_inter_agent_distance": min_distance,
            "mean_path_length": float(np.mean(list(lengths.values()))) if lengths else 0.0,
            "cargoes": cargo_summaries,
        }

    # ------------------------------------------------------------------ #

    def save_outputs(self, output_dir: str | Path) -> dict:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        summary = self.summary()
        (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self._save_trajectories(out / "trajectories.csv")
        self._save_safety_timeseries(out / "safety_timeseries.csv")
        return summary

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
