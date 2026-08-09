"""Scenario loading, fail-closed.

Every field that decides what a run *means* has to be written down. Two in
particular have no default here:

``transport.engine``    which engine moved the cargo
``controller.backend``  which solver produced the safety input

Both used to have defaults, and both defaults were the weak option. A missing
``engine`` silently gave the scripted engine, so a scenario that had never
simulated contact reported transport. A ``backend`` of ``auto`` silently gave the
projection fallback when cvxpy was absent, so runs described as "hard QP" were
not. Requiring them makes the configuration a record of the experiment rather
than a partial one.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from dbact.cargo import Cargo
from dbact.contact_dynamics import ContactParams
from dbact.contracts import VALID_BACKENDS, ContractViolation
from dbact.controller import DBACTParams
from dbact.geometry import normalize, signed_distance_to_polygon
from dbact.provenance import frame_rng
from dbact.task import TaskSampler, TransportTask
from dbact.transport_dynamics import ScriptedParams
from dbact.types import AgentState

TASK_MODES = ("fixed", "random_constrained")

PAPER_CONFIG_MARKERS = ("configs/sim/v2", "configs\\sim\\v2", "configs/sim/d", "configs\\sim\\d")


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    cfg.setdefault("_source", str(path))
    if any(marker in str(path).replace("\\", "/") for marker in ("configs/sim/v2", "configs/sim/d")):
        cfg.setdefault("paper", True)
    return cfg


def is_paper_config(cfg: dict) -> bool:
    return bool(cfg.get("paper", False))


def validate_config(cfg: dict) -> None:
    """Reject configurations that cannot describe an experiment unambiguously."""
    problems: list[str] = []

    transport = cfg.get("transport", {})
    engine = transport.get("engine")
    if engine is None:
        problems.append(
            "transport.engine is required and has no default; write 'penalty', 'pymunk' or 'scripted'"
        )
    elif engine not in ("penalty", "pymunk", "scripted"):
        problems.append(f"transport.engine must be 'penalty', 'pymunk' or 'scripted', got {engine!r}")
    elif engine == "scripted" and is_paper_config(cfg):
        problems.append(
            "transport.engine='scripted' is not permitted in a paper configuration: it moves the cargo "
            "along a direction no robot measured, so its output restates its input"
        )

    controller = cfg.get("controller", {})
    backend = controller.get("backend")
    if backend is None:
        problems.append(
            f"controller.backend is required and has no default; write one of {list(VALID_BACKENDS)}"
        )
    elif backend not in VALID_BACKENDS:
        problems.append(f"controller.backend must be one of {list(VALID_BACKENDS)}, got {backend!r}")
    elif backend == "projection" and is_paper_config(cfg):
        problems.append(
            "controller.backend='projection' is an inexact filter and is not permitted in a paper "
            "configuration; use 'qp'"
        )

    if problems:
        raise ContractViolation("configuration rejected:\n  - " + "\n  - ".join(problems))


def domain_from_config(cfg: dict) -> tuple[float, float, float, float]:
    d = cfg.get("domain", {})
    return (
        float(d.get("xmin", 0.0)),
        float(d.get("xmax", 8.0)),
        float(d.get("ymin", 0.0)),
        float(d.get("ymax", 8.0)),
    )


def build_agents(cfg: dict, seed: int = 0) -> list[AgentState]:
    """Initial robot placement.

    ``grid``     a compact block, all robots arriving from one side
    ``ring``     evenly spaced on a circle around the work area
    ``scatter``  rejection-sampled in an annulus with a guaranteed separation

    ``scatter`` uses rejection sampling rather than plain Gaussian jitter because
    an unconstrained draw regularly places two robots closer than ``d_min``, and a
    run that starts in violation of its own inter-robot constraint has no valid
    barrier to maintain: the QP begins in the recovery regime and the reported
    minimum separation is set by the initial condition rather than by the filter.
    """
    a = cfg.get("agents", {})
    count = int(a.get("count", 12))
    center = np.asarray(a.get("center", [4.0, 4.0]), dtype=float)
    spacing = float(a.get("spacing", 0.35))
    layout = str(a.get("layout", "grid"))
    jitter = float(a.get("jitter", 0.0))
    rng = frame_rng("agent_layout", base=seed)
    domain = domain_from_config(cfg)

    positions: list[np.ndarray] = []
    if layout == "grid":
        cols = int(np.ceil(np.sqrt(count)))
        rows = int(np.ceil(count / cols))
        for idx in range(count):
            r, c = divmod(idx, cols)
            offset = np.array([(c - (cols - 1) / 2) * spacing, (r - (rows - 1) / 2) * spacing])
            if jitter > 0.0:
                offset = offset + rng.normal(scale=jitter, size=2)
            positions.append(center + offset)
    elif layout == "ring":
        radius = float(a.get("radius", 1.5))
        phase = float(a.get("phase", 0.0))
        for idx in range(count):
            angle = phase + 2.0 * np.pi * idx / max(count, 1)
            positions.append(center + radius * np.array([np.cos(angle), np.sin(angle)]))
    elif layout == "scatter":
        radius_min = float(a.get("radius_min", 0.9))
        radius_max = float(a.get("radius_max", 2.2))
        separation = float(a.get("min_separation", cfg.get("controller", {}).get("d_min", 0.34) * 1.15))
        attempts = 0
        while len(positions) < count and attempts < 20000:
            attempts += 1
            angle = rng.uniform(0.0, 2.0 * np.pi)
            radius = np.sqrt(rng.uniform(radius_min ** 2, radius_max ** 2))
            candidate = center + radius * np.array([np.cos(angle), np.sin(angle)])
            if not (domain[0] <= candidate[0] <= domain[1] and domain[2] <= candidate[1] <= domain[3]):
                continue
            if positions and np.min(np.linalg.norm(np.vstack(positions) - candidate[None, :], axis=1)) < separation:
                continue
            positions.append(candidate)
        if len(positions) < count:
            raise ContractViolation(
                f"scatter layout could not place {count} robots with separation >= {separation:.4f} in the "
                f"annulus [{radius_min}, {radius_max}] after {attempts} attempts; widen the annulus or "
                "reduce the robot count rather than accepting an overlapping start"
            )
    else:
        raise ContractViolation(f"unknown agents.layout {layout!r}; expected 'grid', 'ring' or 'scatter'")

    return [AgentState(agent_id=f"agent_{i:02d}", position=p) for i, p in enumerate(positions)]


def assert_initial_state_valid(agents: list[AgentState], cargoes: list, d_min: float, robot_radius: float) -> None:
    """Reject a scenario whose initial state already violates its own constraints.

    Both checks matter for the same reason: the barrier certificate is about
    *maintaining* safety from a safe initial set, so a run that begins outside it
    cannot demonstrate anything about the filter.
    """
    problems: list[str] = []
    if len(agents) > 1:
        pts = np.vstack([a.position for a in agents])
        d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        worst = float(np.min(d))
        if worst < d_min:
            i, j = np.unravel_index(int(np.argmin(d)), d.shape)
            problems.append(
                f"initial inter-robot distance {worst:.4f} < d_min {d_min:.4f} "
                f"({agents[i].agent_id} and {agents[j].agent_id})"
            )
    for cargo in cargoes:
        clearances = signed_distance_to_polygon(np.vstack([a.position for a in agents]), cargo.vertices)
        if len(clearances) and float(np.min(clearances)) < robot_radius:
            problems.append(
                f"a robot starts within the robot radius of cargo {cargo.object_id!r} "
                f"(min signed clearance {float(np.min(clearances)):.4f} < r_robot {robot_radius:.4f})"
            )
    if problems:
        raise ContractViolation("invalid initial state:\n  - " + "\n  - ".join(problems))


def build_cargoes(cfg: dict) -> list[Cargo]:
    return [Cargo.from_config(item) for item in cfg.get("cargoes", [])]


def goal_directions_from_config(cfg: dict) -> dict[str, np.ndarray]:
    """Task goal directions, keyed by cargo id.

    Held by the task, never by the body. ``transport_direction`` is still read
    from the cargo block for continuity with older scenario files, but it reaches
    only the controller and the success criterion -- never a transport engine
    other than ``scripted``.
    """
    goals: dict[str, np.ndarray] = {}
    for item in cfg.get("cargoes", []):
        direction = item.get("transport_direction")
        if direction is not None:
            goals[str(item.get("id", "cargo"))] = normalize(np.asarray(direction, dtype=float))
    for object_id, direction in (cfg.get("task", {}).get("goal_directions", {}) or {}).items():
        goals[str(object_id)] = normalize(np.asarray(direction, dtype=float))
    return goals


def task_sampler_from_config(cfg: dict) -> TaskSampler | None:
    """Build the episode task sampler, or ``None`` for a fixed-direction scenario."""
    task = cfg.get("task", {}) or {}
    mode = str(task.get("mode", "fixed"))
    if mode not in TASK_MODES:
        raise ContractViolation(f"task.mode must be one of {list(TASK_MODES)}, got {mode!r}")
    if mode == "fixed":
        return None
    known = {k: v for k, v in task.items() if k in TaskSampler.__dataclass_fields__}
    unknown = sorted(set(task) - set(known) - {"mode", "goal_directions"})
    if unknown:
        raise ContractViolation(
            f"unknown task parameters {unknown}; a silently ignored parameter is a configuration "
            "that does not describe the experiment"
        )
    return TaskSampler(**known)


def tasks_from_config(cfg: dict, cargoes: list[Cargo], seed: int = 0) -> dict[str, TransportTask]:
    """Sample one transport task per cargo, reproducibly from the run seed.

    The direction reaches the controller and the success criterion. It does not
    reach ``build_engine``: no contact-dynamics dataclass has a field it could be
    written into, so a run cannot restate its configuration as its result.
    """
    sampler = task_sampler_from_config(cfg)
    if sampler is None:
        return {}
    params = controller_params_from_config(cfg)
    domain = domain_from_config(cfg)
    tasks: dict[str, TransportTask] = {}
    for cargo in cargoes:
        rng = frame_rng("transport_task", cargo.object_id, base=seed)
        radius = float(np.max(np.linalg.norm(cargo.local_vertices, axis=1)))
        tasks[cargo.object_id] = sampler.sample(
            rng,
            object_id=cargo.object_id,
            start=cargo.position,
            object_radius=radius,
            cage_offset=params.cage_offset,
            robot_radius=params.robot_radius,
            domain=domain,
        )
    return tasks


def controller_params_from_config(cfg: dict) -> DBACTParams:
    controller = dict(cfg.get("controller", {}))
    # The safety filter needs the control period to check discrete-time
    # admissibility of the object barrier, and dt is a top-level scenario field.
    controller.setdefault("dt", float(cfg.get("dt", 0.05)))
    return DBACTParams.from_dict(controller)


def contact_params_from_config(cfg: dict) -> ContactParams:
    transport = cfg.get("transport", {})
    controller = cfg.get("controller", {})
    fields = {k: v for k, v in transport.items() if k in ContactParams.__dataclass_fields__}
    # The contact model and the safety filter must agree on the robot radius:
    # C1 relates the two, so they cannot be configured independently.
    fields["robot_radius"] = float(controller.get("robot_radius", ContactParams.robot_radius))
    return ContactParams(**fields)


def scripted_params_from_config(cfg: dict) -> ScriptedParams:
    transport = cfg.get("transport", {})
    fields = {k: v for k, v in transport.items() if k in ScriptedParams.__dataclass_fields__ and k != "goal_directions"}
    return ScriptedParams(goal_directions=goal_directions_from_config(cfg), **fields)


__all__ = [
    "load_yaml",
    "validate_config",
    "is_paper_config",
    "domain_from_config",
    "build_agents",
    "assert_initial_state_valid",
    "build_cargoes",
    "goal_directions_from_config",
    "task_sampler_from_config",
    "tasks_from_config",
    "controller_params_from_config",
    "contact_params_from_config",
    "scripted_params_from_config",
]
