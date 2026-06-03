from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
import json

import numpy as np

from dbact.controller import DBACTController
from dbact.metrics import (
    boundary_coverage,
    min_inter_agent_distance,
    path_lengths,
    recruited_agents_count,
)
from dbact.transport_dynamics import SimpleCagingTransportDynamics
from dbact.types import AgentState

from .scenarios import build_agents, build_cargoes, controller_params_from_config, domain_from_config, transport_params_from_config


@dataclass
class SimulationLog:
    times: list[float] = field(default_factory=list)
    agent_positions: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_centers: dict[str, list[np.ndarray]] = field(default_factory=dict)
    cargo_vertices: dict[str, list[np.ndarray]] = field(default_factory=dict)
    min_distances: list[float] = field(default_factory=list)
    cargo_coverages: dict[str, list[float]] = field(default_factory=dict)


class SimulationEnvironment:
    def __init__(self, config: dict):
        self.config = config
        self.dt = float(config.get("dt", 0.05))
        self.domain = domain_from_config(config)
        self.agents = build_agents(config)
        self.cargoes = build_cargoes(config)
        self.controller = DBACTController(controller_params_from_config(config), self.domain)
        self.transport = SimpleCagingTransportDynamics(transport_params_from_config(config))
        self.t = 0.0
        self.log = SimulationLog()
        for a in self.agents:
            self.log.agent_positions[a.agent_id] = []
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id] = []
            self.log.cargo_vertices[c.object_id] = []
            self.log.cargo_coverages[c.object_id] = []

    def step(self) -> None:
        commands = self.controller.step(self.agents, self.cargoes, self.t, self.dt)
        self.controller.apply_commands(self.agents, commands, self.dt)
        self.transport.step(self.cargoes, self.agents, self.dt)
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
        for c in self.cargoes:
            self.log.cargo_centers[c.object_id].append(c.center.copy())
            self.log.cargo_vertices[c.object_id].append(c.vertices.copy())
            contact_radius = float(self.config.get("transport", {}).get("contact_radius", 0.42))
            self.log.cargo_coverages[c.object_id].append(boundary_coverage(c, self.agents, contact_radius=contact_radius))
        self.log.min_distances.append(min_inter_agent_distance(self.agents))

    def save_outputs(self, output_dir: str | Path) -> None:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        self._save_trajectories(out / "trajectories.csv")
        self._save_agent_positions(out / "agent_positions.csv")
        self._save_coverage_rates(out / "coverage_rates.csv")
        lengths = path_lengths(self.log.agent_positions)
        evaluation_contact_radius = 0.50
        recruited_agents = {
            cargo.object_id: recruited_agents_count(
                cargo,
                self.agents,
                contact_radius=evaluation_contact_radius,
            )
            for cargo in self.cargoes
        }
        metrics = {
            "final_time": self.log.times[-1] if self.log.times else 0.0,
            "min_inter_agent_distance": min(self.log.min_distances) if self.log.min_distances else None,
            "mean_path_length": float(np.mean(list(lengths.values()))) if lengths else 0.0,
            "path_lengths": lengths,
            "final_coverage": {k: v[-1] if v else 0.0 for k, v in self.log.cargo_coverages.items()},
            "cargo_displacement": {
                cargo_id: float(np.linalg.norm(hist[-1] - hist[0])) if len(hist) >= 2 else 0.0
                for cargo_id, hist in self.log.cargo_centers.items()
            },
            "recruited_agents": recruited_agents,
        }
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
