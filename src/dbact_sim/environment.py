from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
import json

import numpy as np

from dbact.controller import DBACTController
from dbact.geometry import clip_to_domain
from dbact.metrics import (
    boundary_coverage,
    enclosure_time,
    min_agent_boundary_distance,
    min_inter_agent_distance,
    path_lengths,
    recruited_agents_count,
    success_flag,
)
from dbact.transport_dynamics import build_transport
from dbact.types import AgentState

from .scenarios import build_agents, build_cargoes, controller_params_from_config, domain_from_config, transport_params_from_config


@dataclass
class SimulationLog:
    times: list[float] = field(default_factory=list)
    agent_positions: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_centers: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_vertices: dict[str, list[np.ndarray]] = field(default_factory=dict)
    min_distances: list[float] = field(default_factory=list)
    min_boundary_distances: dict[str, list[float]] = field(default_factory=dict)
    cargo_coverages: dict[str, list[float]] = field(default_factory=dict)


class SimulationEnvironment:
    def __init__(self, config: dict):
        self.config = config
        self.dt = float(config.get("dt", 0.05))
        self.domain = domain_from_config(config)
        self.agents = build_agents(config)
        self.cargoes = build_cargoes(config)
        params = controller_params_from_config(config)
        # Task velocity is an external instruction; default from first cargo direction.
        if params.enable_transport_bias and float(np.linalg.norm(params.task_velocity)) < 1e-9 and self.cargoes:
            params.task_velocity = self.cargoes[0].transport_direction.tolist()
        self.controller = DBACTController(params, self.domain)
        transport_params = transport_params_from_config(config)
        if transport_params.robot_radius <= 0:
            transport_params.robot_radius = params.robot_radius
        self.transport = build_transport(transport_params, self.cargoes, self.agents)
        self.t = 0.0
        self.log = SimulationLog()
        for a in self.agents:
            self.log.agent_positions[a.agent_id] = []
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id] = []
            self.log.cargo_vertices[c.object_id] = []
            self.log.cargo_coverages[c.object_id] = []
            self.log.min_boundary_distances[c.object_id] = []

    def step(self) -> None:
        commands = self.controller.step(self.agents, self.cargoes, self.t, self.dt)
        physics_advances_agents = bool(getattr(self.transport, "advances_agents", False))
        self.controller.apply_commands(
            self.agents,
            commands,
            self.dt,
            advance_positions=not physics_advances_agents,
        )
        self.transport.step(self.cargoes, self.agents, self.dt)
        if physics_advances_agents:
            for agent in self.agents:
                agent.position = clip_to_domain(agent.position, self.domain)
        self._record()
        self.t += self.dt

    def run(
        self,
        steps: int,
        on_frame: Callable[[int, "SimulationEnvironment"], None] | None = None,
    ) -> SimulationLog:
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
        contact_radius = float(self.config.get("transport", {}).get("contact_radius", 0.42))
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id].append(c.center.copy())
            self.log.cargo_vertices[c.object_id].append(c.vertices.copy())
            self.log.cargo_coverages[c.object_id].append(
                boundary_coverage(c, self.agents, contact_radius=contact_radius)
            )
            self.log.min_boundary_distances[c.object_id].append(
                min_agent_boundary_distance(c, self.agents)
            )
        self.log.min_distances.append(min_inter_agent_distance(self.agents))

    def compute_metrics(self) -> dict:
        lengths = path_lengths(self.log.agent_positions)
        evaluation_contact_radius = float(self.config.get("evaluation", {}).get("contact_radius", 0.50))
        enclosure_threshold = float(self.config.get("evaluation", {}).get("enclosure_threshold", 0.5))
        require_transport = bool(self.config.get("evaluation", {}).get("require_transport", False))
        min_displacement = float(self.config.get("evaluation", {}).get("min_displacement", 0.2))

        recruited_agents = {
            cargo.object_id: recruited_agents_count(
                cargo,
                self.agents,
                contact_radius=evaluation_contact_radius,
            )
            for cargo in self.cargoes
        }
        final_coverage = {k: v[-1] if v else 0.0 for k, v in self.log.cargo_coverages.items()}
        cargo_displacement = {
            cargo_id: float(np.linalg.norm(hist[-1] - hist[0])) if len(hist) >= 2 else 0.0
            for cargo_id, hist in self.log.cargo_centers.items()
        }
        t_enclosure = {
            cargo_id: enclosure_time(hist, self.log.times, threshold=enclosure_threshold)
            for cargo_id, hist in self.log.cargo_coverages.items()
        }
        d_min_obs = {
            cargo_id: float(min(hist)) if hist else None
            for cargo_id, hist in self.log.min_boundary_distances.items()
        }
        success = {
            cargo_id: success_flag(
                final_coverage.get(cargo_id, 0.0),
                cargo_displacement.get(cargo_id, 0.0),
                coverage_threshold=enclosure_threshold,
                min_displacement=min_displacement,
                require_transport=require_transport,
            )
            for cargo_id in final_coverage
        }
        stats = self.controller.stats
        return {
            "final_time": self.log.times[-1] if self.log.times else 0.0,
            "method": self.controller.params.method,
            "transport_backend": self.config.get("transport", {}).get("backend", "scripted"),
            "min_inter_agent_distance": min(self.log.min_distances) if self.log.min_distances else None,
            "d_min_obs": d_min_obs,
            "mean_path_length": float(np.mean(list(lengths.values()))) if lengths else 0.0,
            "path_lengths": lengths,
            "final_coverage": final_coverage,
            "cargo_displacement": cargo_displacement,
            "recruited_agents": recruited_agents,
            "T_enclosure": t_enclosure,
            "R_CBF": stats.intervention_rate,
            "T_solve": stats.mean_solve_time_s,
            "cbf_calls": stats.cbf_calls,
            "cbf_infeasible_calls": stats.infeasible_calls,
            "success": success,
            "P_success": float(np.mean(list(success.values()))) if success else 0.0,
        }

    def save_outputs(self, output_dir: str | Path) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._save_trajectories(out / "trajectories.csv")
        self._save_agent_positions(out / "agent_positions.csv")
        self._save_coverage_rates(out / "coverage_rates.csv")
        metrics = self.compute_metrics()
        (out / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

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

    def _save_agent_positions(self, path: Path) -> None:
        lines = ["iteration,time,agent_id,x,y"]
        for ti, t in enumerate(self.log.times):
            for agent_id, hist in self.log.agent_positions.items():
                p = hist[ti]
                lines.append(f"{ti},{t:.4f},{agent_id},{p[0]:.6f},{p[1]:.6f}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _save_coverage_rates(self, path: Path) -> None:
        lines = ["iteration,time,cargo_id,coverage_rate"]
        for ti, t in enumerate(self.log.times):
            for cargo_id, hist in self.log.cargo_coverages.items():
                lines.append(f"{ti},{t:.4f},{cargo_id},{hist[ti]:.6f}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
