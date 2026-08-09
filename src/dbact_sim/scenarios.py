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
from dbact.transport_dynamics import ScriptedParams
from dbact.types import AgentState

PAPER_CONFIG_MARKERS = ("configs/sim/v2", "configs/sim/v3", "configs\\sim\\v2", "configs\\sim\\v3")


def load_yaml(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    cfg.setdefault("_source", str(path))
    if any(marker in str(path).replace("\\", "/") for marker in ("configs/sim/v2", "configs/sim/v3")):
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


def build_cargoes(
    cfg: dict,
    seed: int = 0,
    agents: list[AgentState] | None = None,
) -> list[Cargo]:
    """Build cargoes, optionally sampling a reproducible unknown start pose.

    ``random_center`` is deliberately attached to the scenario rather than the
    controller.  The sampler may use ground-truth geometry to construct a valid
    episode, but the controller receives only ray observations once the episode
    starts.  ``initial_sensor_gap`` makes the discovery claim executable: every
    robot must begin outside the object's sensing horizon.
    """
    domain = domain_from_config(cfg)
    positions = np.vstack([agent.position for agent in agents]) if agents else np.empty((0, 2))
    cargoes: list[Cargo] = []
    for item in cfg.get("cargoes", []):
        item_cfg = dict(item)
        random_cfg = item_cfg.get("random_center", {}) or {}
        if not bool(random_cfg.get("enabled", False)):
            cargoes.append(Cargo.from_config(item_cfg))
            continue

        object_id = str(item_cfg.get("id", "cargo"))
        margin = float(random_cfg.get("domain_margin", 0.0))
        xmin = float(random_cfg.get("xmin", domain[0] + margin))
        xmax = float(random_cfg.get("xmax", domain[1] - margin))
        ymin = float(random_cfg.get("ymin", domain[2] + margin))
        ymax = float(random_cfg.get("ymax", domain[3] - margin))
        if xmax <= xmin or ymax <= ymin:
            raise ContractViolation(f"cargo {object_id!r} random_center bounds are empty")
        initial_sensor_gap = float(random_cfg.get("initial_sensor_gap", 0.0))
        max_attempts = int(random_cfg.get("max_attempts", 512))
        yaw_min = float(random_cfg.get("yaw_min_deg", np.degrees(float(item_cfg.get("yaw", 0.0)))))
        yaw_max = float(random_cfg.get("yaw_max_deg", yaw_min))
        if yaw_max < yaw_min:
            raise ContractViolation(f"cargo {object_id!r} random_center yaw_max_deg < yaw_min_deg")

        rng = frame_rng("random_cargo_center", object_id, base=seed)
        accepted: Cargo | None = None
        for _ in range(max_attempts):
            candidate_cfg = dict(item_cfg)
            candidate_cfg["center"] = [float(rng.uniform(xmin, xmax)), float(rng.uniform(ymin, ymax))]
            candidate_cfg["yaw"] = float(np.deg2rad(rng.uniform(yaw_min, yaw_max)))
            cargo = Cargo.from_config(candidate_cfg)
            vertices = cargo.vertices
            inside = (
                np.min(vertices[:, 0]) >= domain[0] + margin
                and np.max(vertices[:, 0]) <= domain[1] - margin
                and np.min(vertices[:, 1]) >= domain[2] + margin
                and np.max(vertices[:, 1]) <= domain[3] - margin
            )
            if not inside:
                continue
            if len(positions) and initial_sensor_gap > 0.0:
                clearances = signed_distance_to_polygon(positions, vertices)
                if float(np.min(clearances)) <= initial_sensor_gap:
                    continue
            accepted = cargo
            break
        if accepted is None:
            raise ContractViolation(
                f"cargo {object_id!r} random_center could not satisfy the domain and initial_sensor_gap "
                f"contracts after {max_attempts} attempts"
            )
        cargoes.append(accepted)
    return cargoes


def goal_directions_from_config(
    cfg: dict,
    seed: int = 0,
    cargoes: list[Cargo] | None = None,
) -> dict[str, np.ndarray]:
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

    random_cfg = (cfg.get("task", {}).get("random_goal", {}) or {})
    if bool(random_cfg.get("enabled", False)):
        angle_min = float(random_cfg.get("angle_min_deg", 0.0))
        angle_max = float(random_cfg.get("angle_max_deg", 360.0))
        if angle_max <= angle_min:
            raise ContractViolation("task.random_goal requires angle_max_deg > angle_min_deg")
        distance = float(random_cfg.get("target_distance", 0.30))
        wall_margin = float(random_cfg.get("wall_margin", 0.50))
        max_attempts = int(random_cfg.get("max_attempts", 256))
        xmin, xmax, ymin, ymax = domain_from_config(cfg)
        cargo_by_id = {cargo.object_id: cargo for cargo in (cargoes or [])}
        for item in cfg.get("cargoes", []):
            object_id = str(item.get("id", "cargo"))
            # An explicit task goal remains authoritative.  The random mode fills
            # only missing goals so a baseline can override one cargo without
            # silently randomising the others.
            if object_id in goals:
                continue
            cargo = cargo_by_id.get(object_id)
            center = cargo.center.copy() if cargo is not None else np.asarray(item.get("center", [0.0, 0.0]), dtype=float).reshape(2)
            rng = frame_rng("random_goal", object_id, base=seed)
            accepted = None
            for _ in range(max_attempts):
                angle = np.deg2rad(rng.uniform(angle_min, angle_max))
                direction = np.array([np.cos(angle), np.sin(angle)], dtype=float)
                target = center + distance * direction
                center_inside = (
                    xmin + wall_margin <= target[0] <= xmax - wall_margin
                    and ymin + wall_margin <= target[1] <= ymax - wall_margin
                )
                footprint_margin = random_cfg.get("footprint_margin")
                footprint_inside = True
                if cargo is not None and footprint_margin is not None:
                    shifted = cargo.vertices + distance * direction
                    fm = float(footprint_margin)
                    footprint_inside = bool(
                        np.min(shifted[:, 0]) >= xmin + fm
                        and np.max(shifted[:, 0]) <= xmax - fm
                        and np.min(shifted[:, 1]) >= ymin + fm
                        and np.max(shifted[:, 1]) <= ymax - fm
                    )
                if center_inside and footprint_inside:
                    accepted = direction
                    break
            if accepted is None:
                raise ContractViolation(
                    f"task.random_goal could not place the {distance:.3f} m target for {object_id!r} "
                    f"inside the domain margin after {max_attempts} attempts"
                )
            goals[object_id] = accepted
    return goals


def goal_targets_from_config(
    cfg: dict,
    cargoes: list[Cargo],
    goal_directions: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Return the bounded target point used for provenance and rendering."""
    random_cfg = cfg.get("task", {}).get("random_goal", {}) or {}
    distance = float(random_cfg.get("target_distance", cfg.get("task", {}).get("target_distance", 0.0)))
    explicit = cfg.get("task", {}).get("goal_positions", {}) or {}
    targets: dict[str, np.ndarray] = {}
    for cargo in cargoes:
        if cargo.object_id in explicit:
            targets[cargo.object_id] = np.asarray(explicit[cargo.object_id], dtype=float).reshape(2)
        elif cargo.object_id in goal_directions and distance > 0.0:
            targets[cargo.object_id] = cargo.center + distance * goal_directions[cargo.object_id]
    return targets


def controller_params_from_config(cfg: dict) -> DBACTParams:
    return DBACTParams.from_dict(cfg.get("controller", {}))


def contact_params_from_config(cfg: dict) -> ContactParams:
    transport = cfg.get("transport", {})
    controller = cfg.get("controller", {})
    fields = {k: v for k, v in transport.items() if k in ContactParams.__dataclass_fields__}
    # The contact model and the safety filter must agree on the robot radius:
    # C1 relates the two, so they cannot be configured independently.
    fields["robot_radius"] = float(controller.get("robot_radius", ContactParams.robot_radius))
    return ContactParams(**fields)


def scripted_params_from_config(
    cfg: dict,
    seed: int = 0,
    cargoes: list[Cargo] | None = None,
) -> ScriptedParams:
    transport = cfg.get("transport", {})
    fields = {k: v for k, v in transport.items() if k in ScriptedParams.__dataclass_fields__ and k != "goal_directions"}
    return ScriptedParams(goal_directions=goal_directions_from_config(cfg, seed=seed, cargoes=cargoes), **fields)


__all__ = [
    "load_yaml",
    "validate_config",
    "is_paper_config",
    "domain_from_config",
    "build_agents",
    "assert_initial_state_valid",
    "build_cargoes",
    "goal_directions_from_config",
    "goal_targets_from_config",
    "controller_params_from_config",
    "contact_params_from_config",
    "scripted_params_from_config",
]
