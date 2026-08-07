from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from dbact.cargo import Cargo
from dbact.controller import DBACTParams
from dbact.transport_dynamics import TransportParams
from dbact.types import AgentState


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def domain_from_config(cfg: dict) -> tuple[float, float, float, float]:
    d = cfg.get("domain", {})
    return (float(d.get("xmin", 0.0)), float(d.get("xmax", 8.0)), float(d.get("ymin", 0.0)), float(d.get("ymax", 8.0)))


def build_agents(cfg: dict) -> list[AgentState]:
    a = cfg.get("agents", {})
    count = int(a.get("count", 12))
    center = np.asarray(a.get("center", [4.0, 4.0]), dtype=float)
    spacing = float(a.get("spacing", 0.35))
    layout = str(a.get("layout", "grid"))
    seed = int(a.get("seed", cfg.get("seed", 1)))
    jitter = float(a.get("jitter", 0.0))
    rng = np.random.default_rng(seed)
    agents: list[AgentState] = []
    if layout == "grid":
        cols = int(np.ceil(np.sqrt(count)))
        rows = int(np.ceil(count / cols))
        idx = 0
        for r in range(rows):
            for c in range(cols):
                if idx >= count:
                    break
                offset = np.array([(c - (cols - 1) / 2) * spacing, (r - (rows - 1) / 2) * spacing])
                pos = center + offset
                if jitter > 0.0:
                    pos = pos + rng.normal(scale=jitter, size=2)
                agents.append(AgentState(agent_id=f"agent_{idx:02d}", position=pos))
                idx += 1
    else:
        for idx in range(count):
            agents.append(AgentState(agent_id=f"agent_{idx:02d}", position=center + rng.normal(scale=spacing, size=2)))
    return agents


def build_cargoes(cfg: dict) -> list[Cargo]:
    return [Cargo.from_config(item) for item in cfg.get("cargoes", [])]


def controller_params_from_config(cfg: dict) -> DBACTParams:
    return DBACTParams.from_dict(cfg.get("controller", {}))


def transport_params_from_config(cfg: dict) -> TransportParams:
    return TransportParams(**{k: v for k, v in cfg.get("transport", {}).items() if k in TransportParams.__dataclass_fields__})
