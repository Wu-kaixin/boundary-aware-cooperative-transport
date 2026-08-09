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
import copy
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np

from dbact.contracts import DirectionalProgressContract
from dbact.controller import DBACTController
from dbact.guarantees import (
    boundary_map_gap_upper_bound,
    build_admissibility_certificate,
    minimum_facing_cage_clearance,
)
from dbact.metrics import (
    boundary_coverage,
    directional_progress,
    min_inter_agent_distance,
    operational_enclosure_certificate,
    path_lengths,
    penetration_report,
    recruited_agents_count,
)
from dbact.perception import normal_errors_deg
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
    detection_counts: dict[str, list[int]] = field(default_factory=dict)
    mode_counts: list[dict[str, int]] = field(default_factory=list)
    agent_modes: dict[str, list[str]] = field(default_factory=dict)
    normal_error_deg: dict[str, list[float]] = field(default_factory=dict)
    normal_error_norm: dict[str, list[float]] = field(default_factory=dict)
    boundary_point_error: dict[str, list[float]] = field(default_factory=dict)
    map_point_error: dict[str, list[float]] = field(default_factory=dict)
    object_velocity_error: dict[str, list[float]] = field(default_factory=dict)
    object_velocity_projection_error: dict[str, list[float]] = field(default_factory=dict)
    boundary_velocity_error: dict[str, list[float]] = field(default_factory=dict)
    cbf_velocity_projection_error: dict[str, list[float]] = field(default_factory=dict)
    relaxation_events: list[dict] = field(default_factory=list)
    operational_enclosure: dict[str, list[dict]] = field(default_factory=dict)


@dataclass(frozen=True)
class EpisodeTermination:
    status: str
    frame: int
    time: float
    success: bool
    detail: str


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
        self.guarantee_certificates = {
            cargo.object_id: build_admissibility_certificate(
                cargo=cargo,
                agents=self.agents,
                domain=self.domain,
                goal_direction=self.goal_directions.get(cargo.object_id),
                config=config,
                controller=params,
                contact=self.contact_params,
                dt=self.dt,
            )
            for cargo in self.cargoes
        }
        self.boundary_map_witnesses: dict[str, dict] = {}
        self.guarantee_release_time = max(
            (cert.get("search", {}).get("release_bound_frames", 0) for cert in self.guarantee_certificates.values()),
            default=0,
        ) * self.dt
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
        enclosure = evaluation.get("operational_enclosure", {}) or {}
        self.enclosure_samples = int(enclosure.get("samples", 360))
        self.enclosure_strict_coverage_min = float(
            enclosure.get("strict_coverage_min", 0.99)
        )
        self.enclosure_max_uncovered_arc = float(
            enclosure.get("max_uncovered_arc_m", 0.10)
        )
        self.enclosure_min_engaged_agents = int(
            enclosure.get("min_engaged_agents", params.min_push_agents)
        )
        self.enclosure_engaged_radius = float(
            enclosure.get("engaged_radius_m", self.evaluation_contact_radius)
        )
        self.enclosure_facing_clearance = {
            cargo.object_id: minimum_facing_cage_clearance(
                cargo.local_vertices,
                params.cage_offset,
            )
            for cargo in self.cargoes
        }
        self.require_initially_unobserved = bool(evaluation.get("require_initially_unobserved", False))
        self.require_guarantee_certificate = bool(evaluation.get("require_guarantee_certificate", False))
        self.require_measured_error_bounds = bool(evaluation.get("require_measured_error_bounds", False))
        self.measured_error_bounds = dict(evaluation.get("measured_error_bounds", {}) or {})
        self.online_truth_audit = bool(evaluation.get("online_truth_audit", False))
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
                self.log.detection_counts,
                self.log.normal_error_deg,
                self.log.normal_error_norm,
                self.log.boundary_point_error,
                self.log.map_point_error,
                self.log.object_velocity_error,
                self.log.object_velocity_projection_error,
                self.log.boundary_velocity_error,
                self.log.cbf_velocity_projection_error,
                self.log.operational_enclosure,
            ):
                store[c.object_id] = []
        self._last_statuses = {}
        self.last_termination: EpisodeTermination | None = None

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

    def run_until(
        self,
        max_steps: int,
        on_frame: Callable[[int, "SimulationEnvironment"], None] | None = None,
    ) -> EpisodeTermination:
        """Run until closed-loop HOLD, a hard failure, or an explicit timeout.

        ``max_steps`` is a safety timeout, not a success deadline. Historical
        fixed-horizon experiments continue to use :meth:`run` unchanged.
        """
        if max_steps < 1:
            raise ValueError("max_steps must be positive")
        if not self.log.times:
            self._record()
            if on_frame is not None:
                on_frame(0, self)
        for frame in range(1, int(max_steps) + 1):
            self.step()
            if on_frame is not None:
                on_frame(frame, self)
            stats = self.controller.safety.stats
            if stats.fallbacks or stats.infeasible:
                return self._terminate(
                    "SOLVER_FAILURE",
                    frame,
                    False,
                    f"fallbacks={stats.fallbacks}, infeasible={stats.infeasible}",
                )
            phases = self.controller._transport_phase
            feedback_hold = (
                self.controller.params.progress_feedback
                and len(phases) == len(self.agents)
                and all(value == "hold" for value in phases.values())
            )
            legacy_hold = (
                not self.controller.params.progress_feedback
                and self.controller.mode_counts().get("hold", 0) == len(self.agents)
            )
            if feedback_hold or legacy_hold:
                return self._terminate(
                    "SUCCESS_HOLD",
                    frame,
                    True,
                    "all local progress supervisors completed BRAKE and entered HOLD",
                )
        return self._terminate(
            "TIMEOUT",
            int(max_steps),
            False,
            "episode did not reach a terminal success/failure condition before max_steps",
        )

    def _terminate(self, status: str, frame: int, success: bool, detail: str) -> EpisodeTermination:
        result = EpisodeTermination(status, int(frame), float(self.t), bool(success), detail)
        self.last_termination = result
        return result

    def _record(self) -> None:
        self.log.times.append(self.t)
        frame = len(self.log.times) - 1
        for a in self.agents:
            self.log.agent_positions[a.agent_id].append(a.position.copy())
        mode_by_agent = {diag.agent_id: diag.mode for diag in self.controller.diagnostics}
        for a in self.agents:
            initial_mode = "explore" if self.controller.params.task_mode != "coverage" else "search"
            self.log.agent_modes[a.agent_id].append(mode_by_agent.get(a.agent_id, initial_mode))
        self.log.min_distances.append(min_inter_agent_distance(self.agents))
        self.log.mode_counts.append(self.controller.mode_counts())
        for diag in self.controller.diagnostics:
            if diag.solver_status == "relaxed_margin":
                agent = next(a for a in self.agents if a.agent_id == diag.agent_id)
                true_clearance = min(
                    (
                        float(c.signed_distance(agent.position[None, :])[0][0])
                        for c in self.cargoes
                    ),
                    default=float("inf"),
                )
                self.log.relaxation_events.append(
                    {
                        "frame": frame,
                        "time": float(self.t),
                        "agent_id": diag.agent_id,
                        "mode": diag.mode,
                        "max_full_margin_deficit": diag.max_full_margin_deficit,
                        "max_barrier_deficit": diag.max_barrier_deficit,
                        "max_object_margin_deficit": diag.max_object_margin_deficit,
                        "min_object_h": diag.min_object_h,
                        "max_object_velocity_projection": diag.max_object_velocity_projection,
                        "true_surface_clearance": true_clearance,
                    }
                )

        robot_radius = self.contact_params.robot_radius
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id].append(c.center.copy())
            self.log.cargo_angles[c.object_id].append(float(c.angle))
            self.log.cargo_vertices[c.object_id].append(c.vertices.copy())
            self.log.coverage[c.object_id].append(
                boundary_coverage(c, self.agents, contact_radius=self.evaluation_contact_radius)
            )
            enclosure = operational_enclosure_certificate(
                c,
                self.agents,
                contact_radius=self.evaluation_contact_radius,
                strict_coverage_min=self.enclosure_strict_coverage_min,
                max_uncovered_arc_m=self.enclosure_max_uncovered_arc,
                d_min=self.controller.params.d_min,
                cage_offset=self.controller.params.cage_offset,
                min_engaged_agents=self.enclosure_min_engaged_agents,
                engaged_radius=self.enclosure_engaged_radius,
                facing_clearance_m=self.enclosure_facing_clearance[c.object_id],
                samples=self.enclosure_samples,
            )
            self.log.operational_enclosure[c.object_id].append(enclosure)
            self.log.strict_coverage[c.object_id].append(enclosure["strict_boundary_coverage"])
            report = penetration_report(c, self.agents, robot_radius)
            self.log.min_clearance[c.object_id].append(report["min_signed_clearance"])
            self.log.max_penetration[c.object_id].append(report["max_penetration"])
            self.log.agents_inside[c.object_id].append(report["agents_inside"])

            status = self._last_statuses.get(c.object_id)
            self.log.contact_counts[c.object_id].append(status.contact_count if status else 0)
            self.log.net_force[c.object_id].append(status.net_force.copy() if status else np.zeros(2))
            self.log.net_torque[c.object_id].append(status.net_torque if status else 0.0)
            self.log.cargo_speed[c.object_id].append(float(np.linalg.norm(c.linear_velocity)))
            detections = (
                self.initial_detection_counts.get(c.object_id, 0)
                if abs(self.t) <= 1e-12
                else self.controller.last_detection_counts.get(c.object_id, 0)
            )
            self.log.detection_counts[c.object_id].append(int(detections))
            fresh_perception = (
                self.controller.last_perception_timestamp is not None
                and abs(self.controller.last_perception_timestamp - (self.t - self.dt)) <= 1e-9
            )
            if fresh_perception and self.online_truth_audit:
                observations = [
                    obs
                    for items in self.controller.last_sensed_observations.values()
                    for obs in items
                    if obs.object_id == c.object_id
                ]
                if observations:
                    normal_angles = normal_errors_deg(observations, [c])
                    self.log.normal_error_deg[c.object_id].extend(normal_angles.tolist())
                    self.log.normal_error_norm[c.object_id].extend(
                        (2.0 * np.sin(0.5 * np.radians(normal_angles))).tolist()
                    )
                    points = np.vstack([obs.point for obs in observations])
                    signed, _, _ = c.signed_distance(points)
                    self.log.boundary_point_error[c.object_id].extend(np.abs(signed).tolist())
                map_points = [
                    record.point
                    for boundary_map in self.controller.maps.values()
                    for record in boundary_map.records.values()
                    if record.object_id == c.object_id
                ]
                if map_points:
                    signed_map, _, _ = c.signed_distance(np.vstack(map_points))
                    self.log.map_point_error[c.object_id].extend(np.abs(signed_map).tolist())
            if self.online_truth_audit:
                for agent_id, estimates in self.controller.object_velocity.items():
                    if c.object_id in estimates:
                        velocity_error = np.asarray(estimates[c.object_id]) - c.linear_velocity
                        self.log.object_velocity_error[c.object_id].append(
                            float(np.linalg.norm(velocity_error))
                        )
                        normals = [
                            obs.normal
                            for obs in self.controller._safety_observation_cache.get(agent_id, [])
                            if obs.object_id == c.object_id
                        ]
                        if normals:
                            projections = np.abs(np.vstack(normals) @ velocity_error)
                            self.log.object_velocity_projection_error[c.object_id].extend(
                                projections.tolist()
                            )
                        agent = next(a for a in self.agents if a.agent_id == agent_id)
                        safety_observations = [
                            obs
                            for obs in self.controller._safety_observation_cache.get(agent_id, [])
                            if obs.object_id == c.object_id
                        ]
                        for obs in safety_observations:
                            true_point_velocity = c.point_velocity(np.asarray(obs.point, dtype=float))
                            point_velocity_error = (
                                np.asarray(estimates[c.object_id], dtype=float) - true_point_velocity
                            )
                            self.log.boundary_velocity_error[c.object_id].append(
                                float(np.linalg.norm(point_velocity_error))
                            )
                            radial = np.asarray(agent.position, dtype=float) - np.asarray(
                                obs.point, dtype=float
                            )
                            radial_norm = float(np.linalg.norm(radial))
                            if radial_norm > 1e-12:
                                self.log.cbf_velocity_projection_error[c.object_id].append(
                                    abs(float(np.dot(radial / radial_norm, point_velocity_error)))
                                )
        if self.t + 1e-12 >= self.guarantee_release_time and not self.boundary_map_witnesses:
            self._capture_boundary_map_witnesses()

    def _capture_boundary_map_witnesses(self) -> None:
        """Measure the theorem's map-completeness premise at rendezvous release.

        Ground-truth boundary samples are used only by this independent witness;
        neither the controller nor the map receives them.
        """
        all_observations = [
            obs
            for boundary_map in self.controller.maps.values()
            for obs in boundary_map.all_observations(self.t)
        ]
        for cargo in self.cargoes:
            mapped = [obs.point for obs in all_observations if obs.object_id == cargo.object_id]
            if not mapped:
                self.boundary_map_witnesses[cargo.object_id] = {
                    "frame": int(round(self.t / self.dt)),
                    "points": 0,
                    "max_boundary_gap": float("inf"),
                    "p95_boundary_gap": float("inf"),
                }
                continue
            gap_witness = boundary_map_gap_upper_bound(cargo.vertices, np.vstack(mapped), sample_count=1024)
            self.boundary_map_witnesses[cargo.object_id] = {
                "frame": int(round(self.t / self.dt)),
                "points": int(len(mapped)),
                **gap_witness,
            }

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
        error_audit: dict[str, dict] = {}
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
            max_abs_rotation_deg = (
                float(
                    np.max(
                        np.abs(
                            np.degrees(
                                np.asarray(self.log.cargo_angles[cid], dtype=float)
                                - self.log.cargo_angles[cid][0]
                            )
                        )
                    )
                )
                if self.log.cargo_angles[cid]
                else 0.0
            )

            entry = {
                "displacement_vector": (centers[-1] - centers[0]).tolist() if len(centers) >= 2 else [0.0, 0.0],
                "displacement": float(np.linalg.norm(centers[-1] - centers[0])) if len(centers) >= 2 else 0.0,
                "rotation_deg": rotation_deg,
                "max_abs_rotation_deg": max_abs_rotation_deg,
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
                "peak_abs_net_torque": max(
                    (abs(value) for value in self.log.net_torque[cid]),
                    default=0.0,
                ),
                "recruited_agents": recruited_agents_count(cargo, self.agents, self.evaluation_contact_radius),
                "initial_detection_count": self.initial_detection_counts.get(cid, 0),
                "final_operational_enclosure": (
                    self.log.operational_enclosure[cid][-1]
                    if self.log.operational_enclosure[cid]
                    else None
                ),
            }
            first_detection = next(
                (k for k, count in enumerate(self.log.detection_counts[cid]) if count > 0),
                None,
            )
            error_audit[cid] = {
                "normal_error_deg": self._distribution(self.log.normal_error_deg[cid]),
                "normal_error_norm": self._distribution(self.log.normal_error_norm[cid]),
                "boundary_point_error_m": self._distribution(self.log.boundary_point_error[cid]),
                "map_point_error_m": self._distribution(self.log.map_point_error[cid]),
                "object_velocity_error_mps": self._distribution(self.log.object_velocity_error[cid]),
                "object_velocity_projection_error_mps": self._distribution(
                    self.log.object_velocity_projection_error[cid]
                ),
                "boundary_velocity_error_mps": self._distribution(
                    self.log.boundary_velocity_error[cid]
                ),
                "cbf_velocity_projection_error_mps": self._distribution(
                    self.log.cbf_velocity_projection_error[cid]
                ),
            }
            first_enclosure = next(
                (
                    k
                    for k, certificate in enumerate(self.log.operational_enclosure[cid])
                    if certificate["passed"]
                ),
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
            first_contact = next(
                (k for k, count in enumerate(self.log.contact_counts[cid]) if count >= params.min_push_agents),
                None,
            )
            first_brake = next(
                (k for k, modes in enumerate(self.log.mode_counts) if modes.get("brake", 0) > 0),
                None,
            )
            first_hold = next(
                (k for k, modes in enumerate(self.log.mode_counts) if modes.get("hold", 0) > 0),
                None,
            )
            entry["phase_frames"] = {
                "first_detection": first_detection,
                "first_map_complete": (self.boundary_map_witnesses.get(cid) or {}).get("frame"),
                "first_enclosure": first_enclosure,
                "first_contact": first_contact,
                "first_transport": first_transport,
                "first_brake": first_brake,
                "first_hold": first_hold,
            }
            if first_transport is not None:
                entry["min_strict_coverage_during_transport"] = min(
                    self.log.strict_coverage[cid][first_transport:],
                    default=0.0,
                )
                entry["min_contact_count_during_transport"] = min(
                    self.log.contact_counts[cid][first_transport:],
                    default=0,
                )
                entry["max_uncovered_arc_during_transport_m"] = max(
                    (
                        certificate["max_uncovered_arc_upper_m"]
                        for certificate in self.log.operational_enclosure[cid][first_transport:]
                    ),
                    default=float("inf"),
                )
                entry["operational_enclosure_maintained_during_transport"] = all(
                    certificate["passed"]
                    for certificate in self.log.operational_enclosure[cid][first_transport:]
                )
            else:
                entry["min_strict_coverage_during_transport"] = None
                entry["min_contact_count_during_transport"] = None
                entry["max_uncovered_arc_during_transport_m"] = None
                entry["operational_enclosure_maintained_during_transport"] = None
            if goal is not None and len(centers) >= 2:
                entry["goal_direction"] = np.asarray(goal, dtype=float).tolist()
                entry["goal_angle_deg"] = float(np.degrees(np.arctan2(goal[1], goal[0])))
                if cid in self.goal_targets:
                    entry["goal_target"] = self.goal_targets[cid].tolist()
                episode_progress = directional_progress(centers[0], centers[-1], goal)
                entry["episode_total_J"] = episode_progress["J"]
                entry["episode_total_displacement"] = episode_progress["displacement"]
                entry["episode_total_efficiency"] = episode_progress["efficiency"]
                # Task progress is defined from controller activation, not from
                # the beginning of SEARCH.  Search/enclosure contacts can move
                # the cargo passively; counting that displacement toward the
                # requested transport length makes the estimator, HOLD latch and
                # success contract refer to different physical tasks.
                transport_origin_index = first_transport if first_transport is not None else 0
                transport_origin = np.asarray(centers[transport_origin_index], dtype=float)
                entry["transport_activation_center"] = transport_origin.tolist()
                entry["transport_goal_target"] = (
                    transport_origin + params.transport_distance * np.asarray(goal, dtype=float)
                ).tolist()
                entry.update(directional_progress(transport_origin, centers[-1], goal))
                center_array = np.vstack(centers)
                displacement_history = center_array[transport_origin_index:] - transport_origin
                unit_goal = np.asarray(goal, dtype=float)
                unit_goal = unit_goal / max(float(np.linalg.norm(unit_goal)), 1e-12)
                cross_track = np.abs(
                    unit_goal[0] * displacement_history[:, 1]
                    - unit_goal[1] * displacement_history[:, 0]
                )
                entry["final_cross_track_error"] = float(cross_track[-1])
                entry["max_cross_track_error"] = float(np.max(cross_track))
                verdict = self.success_contract.evaluate(
                    transport_origin,
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
                certificate = self.guarantee_certificates.get(cid)
                if self.require_guarantee_certificate and not bool((certificate or {}).get("eligible")):
                    failed = ", ".join((certificate or {}).get("failure_reasons", [])) or "missing certificate"
                    phase_reasons.append(f"guarantee gate: admissibility certificate failed ({failed})")
                elif self.require_guarantee_certificate:
                    witness = self.boundary_map_witnesses.get(cid)
                    required_gap = ((certificate or {}).get("mapping") or {}).get("required_max_boundary_gap")
                    if (
                        not isinstance(witness, dict)
                        or required_gap is None
                        or witness.get("max_boundary_gap") is None
                        or witness["max_boundary_gap"] > required_gap
                    ):
                        measured = (witness or {}).get("max_boundary_gap", float("inf"))
                        phase_reasons.append(
                            f"guarantee gate: boundary map max gap {measured:.4f} m exceeds "
                            f"epsilon {float(required_gap or 0.0):.4f} m"
                        )
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
                if first_hold is not None and first_transport is not None and first_hold < first_transport:
                    phase_reasons.append(
                        f"phase gate: HOLD started at frame {first_hold} before transport at frame {first_transport}"
                    )
                if params.progress_feedback:
                    if first_brake is None:
                        phase_reasons.append("phase gate: feedback transport never entered BRAKE")
                    elif first_transport is not None and first_brake < first_transport:
                        phase_reasons.append(
                            f"phase gate: BRAKE started at frame {first_brake} before transport at frame {first_transport}"
                        )
                    if first_hold is not None and first_brake is not None and first_hold < first_brake:
                        phase_reasons.append(
                            f"phase gate: HOLD started at frame {first_hold} before BRAKE at frame {first_brake}"
                        )
                if self.require_measured_error_bounds:
                    observed = error_audit[cid]
                    for key, audit_key in (
                        ("normal_error_deg", "normal_error_deg"),
                        ("normal_error_norm", "normal_error_norm"),
                        ("boundary_point_error_m", "boundary_point_error_m"),
                        ("map_point_error_m", "map_point_error_m"),
                        ("object_velocity_error_mps", "object_velocity_error_mps"),
                        (
                            "object_velocity_projection_error_mps",
                            "object_velocity_projection_error_mps",
                        ),
                        ("boundary_velocity_error_mps", "boundary_velocity_error_mps"),
                        (
                            "cbf_velocity_projection_error_mps",
                            "cbf_velocity_projection_error_mps",
                        ),
                    ):
                        bound = self.measured_error_bounds.get(key)
                        if bound is None:
                            continue
                        maximum = (observed.get(audit_key) or {}).get("max")
                        if maximum is None:
                            phase_reasons.append(f"error-bound gate: {key} bound/measurement is missing")
                        elif maximum > float(bound):
                            phase_reasons.append(
                                f"error-bound gate: measured {key}={maximum:.6g} exceeds {float(bound):.6g}"
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
            certificate = copy.deepcopy(self.guarantee_certificates.get(cid))
            if isinstance(certificate, dict):
                witness = self.boundary_map_witnesses.get(cid)
                certificate["runtime_map_witness"] = witness
                required_gap = (certificate.get("mapping") or {}).get("required_max_boundary_gap")
                map_ok = bool(
                    isinstance(witness, dict)
                    and required_gap is not None
                    and witness.get("max_boundary_gap") is not None
                    and witness["max_boundary_gap"] <= required_gap
                )
                certificate["runtime_eligible"] = bool(certificate.get("eligible")) and map_ok
                certificate["runtime_domain_eligible"] = bool(
                    certificate.get("domain_eligible")
                ) and map_ok
                runtime_failures = list(certificate.get("failure_reasons") or [])
                runtime_domain_failures = list(
                    certificate.get("domain_failure_reasons") or []
                )
                if not map_ok:
                    runtime_failures.append("boundary_map_epsilon")
                    runtime_domain_failures.append("boundary_map_epsilon")
                certificate["runtime_failure_reasons"] = runtime_failures
                certificate["runtime_domain_failure_reasons"] = runtime_domain_failures
            cargo_summaries[cid] = entry
            entry["guarantee_certificate"] = certificate

        lengths = path_lengths(self.log.agent_positions)
        return {
            "provenance": run_provenance(self.config, self.seed, params.backend),
            "engine": self.engine_name,
            "task_mode": params.task_mode,
            "density_mode": params.density_mode,
            "steps": len(self.log.times) - 1,
            "final_time": self.log.times[-1] if self.log.times else 0.0,
            "termination": asdict(self.last_termination) if self.last_termination is not None else None,
            "contracts": {
                "C1": params.contact_contract().as_dict() if params.task_mode != "coverage" else None,
                "coverage": {"local_radius": params.local_radius, "comm_range": params.comm_range},
                "d_min": params.d_min,
                "delta_max": params.delta_max,
                "discrete_overshoot": self.success_contract.discrete_overshoot,
                "require_initially_unobserved": self.require_initially_unobserved,
                "phase_deadlines": self.phase_deadlines,
                "frame_budget": self.frame_budget,
                "require_guarantee_certificate": self.require_guarantee_certificate,
                "online_truth_audit": self.online_truth_audit,
                "measured_error_bounds": self.measured_error_bounds,
                "operational_enclosure": {
                    "certificate_type": "operational_boundary_enclosure",
                    "formal_caging": False,
                    "samples": self.enclosure_samples,
                    "strict_coverage_min": self.enclosure_strict_coverage_min,
                    "max_uncovered_arc_m": self.enclosure_max_uncovered_arc,
                    "min_engaged_agents": self.enclosure_min_engaged_agents,
                    "engaged_radius_m": self.enclosure_engaged_radius,
                },
            },
            "solver": solver_stats,
            "relaxation_events": self.log.relaxation_events,
            "measured_error_audit": error_audit,
            "transport_progress_estimates": self.controller.transport_progress_summary(),
            "transport_feedback": self.controller.transport_feedback_summary(),
            "goal_targets": {key: value.tolist() for key, value in self.goal_targets.items()},
            "multi_rate": {
                "perception_every": params.perception_every,
                "planning_every": params.planning_every,
                "map_gossip_every": params.map_gossip_every,
                "safety_every": 1,
            },
            "communication": {
                "dropout_probability": params.communication_dropout_prob,
                "measured_delivery_rate": self.controller.communication_delivery_rate,
            },
            "min_inter_agent_distance": min_distance,
            "mean_path_length": float(np.mean(list(lengths.values()))) if lengths else 0.0,
            "cargoes": cargo_summaries,
        }

    @staticmethod
    def _distribution(values: list[float]) -> dict:
        data = np.asarray(values, dtype=float)
        data = data[np.isfinite(data)]
        if len(data) == 0:
            return {"n": 0, "mean": None, "p50": None, "p95": None, "p99": None, "max": None}
        return {
            "n": int(len(data)),
            "mean": float(np.mean(data)),
            "p50": float(np.quantile(data, 0.50)),
            "p95": float(np.quantile(data, 0.95)),
            "p99": float(np.quantile(data, 0.99)),
            "max": float(np.max(data)),
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


__all__ = ["SimulationEnvironment", "SimulationLog", "EpisodeTermination"]
