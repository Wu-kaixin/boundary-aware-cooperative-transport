#!/usr/bin/env python
"""v2 - the decisive experiment: v1's controller against an unscreened shape matrix.

    python scripts/run_arbitrary_shape_monte_carlo.py \
        --seeds 0..4 --alpha 0.1,0.4,0.8 --max-steps 3000 --output runs/v2_shape_matrix

CODEX ran twelve shape families and reported a transport displacement of
0.076-0.119 m against objects 1.05-1.87 m across -- four to ten percent of the
object's own size. v1 reported 1.474 m against one object 1.8 m across, which is
eighty-two percent, but it reported it for one shape at one scale. Neither number
answers the question the paper needs answered, and the two are not comparable
because CODEX's task distance was a fixed 0.10 m regardless of how big the object
was.

This harness puts v1's controller on CODEX's twelve families and makes the task
distance scale with the object:

    L = alpha * diameter,    alpha in {0.1, 0.4, 0.8}

so that the reported quantity is J / diameter -- how far the team moved the
object in units of the object -- and the shapes are scaled to v1's magnitude
rather than CODEX's, anchored on the one family both branches share.

No screening
------------
Every case enters the denominator. A case whose admissibility certificate rejects
it is *still run* and still reported; it is counted in P(eligible) rather than
deleted, and its failure class is recorded. A case that crashes the scenario
builder is recorded as a construction failure rather than skipped. The three
probabilities are reported separately with Wilson intervals, because
P(success | eligible) alone is the number a survivor filter produces.

Nothing here tunes the controller. The configuration is the baseline's, and the
only per-case writes are the object, the annulus the robots start in, and the
task distance.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.cargo import Cargo  # noqa: E402
from dbact.geometry import polygon_area, polygon_diameter, rotate  # noqa: E402
from dbact.guarantees import (  # noqa: E402
    build_admissibility_certificate,
    evaluate_runtime_map_completeness,
)
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import (  # noqa: E402
    contact_params_from_config,
    controller_params_from_config,
    load_yaml,
)

SHAPE_NAMES = (
    "circle",
    "rectangle",
    "ellipse24",
    "l_shape",
    "u_shape",
    "c_shape",
    "star10",
    "convex_random",
    "concave_random7",
    "concave_random15",
    "high_aspect",
    "polygon32",
)

#: CODEX drew its outlines at its own scale (circle radius 0.58, l_shape scale
#: 0.95); v1's cargo is an l_shape at scale 1.50. Multiplying every CODEX family
#: by 1.50 / 0.95 puts the shared family exactly on v1's object and carries the
#: other eleven to the same magnitude, so "scale" is one declared factor rather
#: than an accident of which branch wrote the shape down. Diameters land between
#: roughly 1.8 m and 2.7 m, which is what makes the diameter regression in the
#: analysis a real sweep rather than a scatter about one point.
SHAPE_SCALE = 1.50 / 0.95


def parse_seeds(spec: str) -> list[int]:
    values: list[int] = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if ".." in token:
            start, end = (int(part) for part in token.split("..", 1))
            step = 1 if end >= start else -1
            values.extend(range(start, end + step, step))
        else:
            values.append(int(token))
    return sorted(set(values))


def parse_alphas(spec: str) -> list[float]:
    return [float(token) for token in spec.split(",") if token.strip()]


def radial_polygon(count: int, outer: float, inner: float, rng: np.random.Generator) -> np.ndarray:
    step = 2.0 * np.pi / count
    angles = step * np.arange(count) + rng.uniform(-0.18 * step, 0.18 * step, count)
    angles.sort()
    radii = rng.uniform(inner, outer, count)
    if count >= 6:
        radii[1::3] *= 0.55
    return np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])


def local_outline(name: str, rng: np.random.Generator) -> np.ndarray:
    """Body-frame outline of one family, at CODEX's dimensions, before scaling."""
    if name == "circle":
        angles = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
        return np.column_stack([0.58 * np.cos(angles), 0.58 * np.sin(angles)])
    if name == "rectangle":
        w, h = 1.15 / 2.0, 0.72 / 2.0
        return np.array([[-w, -h], [w, -h], [w, h], [-w, h]], dtype=float)
    if name == "high_aspect":
        w, h = 1.60 / 2.0, 0.25 / 2.0
        return np.array([[-w, -h], [w, -h], [w, h], [-w, h]], dtype=float)
    if name == "l_shape":
        return np.array(
            [
                [-0.60, -0.60], [0.60, -0.60], [0.60, -0.15],
                [-0.10, -0.15], [-0.10, 0.60], [-0.60, 0.60],
            ],
            dtype=float,
        ) * 0.95
    if name == "ellipse24":
        angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        return np.column_stack([0.72 * np.cos(angles), 0.42 * np.sin(angles)])
    if name == "u_shape":
        return np.array(
            [
                [-0.65, -0.55], [0.65, -0.55], [0.65, 0.55], [0.32, 0.55],
                [0.32, -0.20], [-0.32, -0.20], [-0.32, 0.55], [-0.65, 0.55],
            ],
            dtype=float,
        )
    if name == "c_shape":
        return np.array(
            [
                [-0.65, -0.55], [0.65, -0.55], [0.65, -0.28], [-0.25, -0.28],
                [-0.25, 0.28], [0.65, 0.28], [0.65, 0.55], [-0.65, 0.55],
            ],
            dtype=float,
        )
    if name == "star10":
        angles = np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False)
        radii = np.where(np.arange(10) % 2 == 0, 0.70, 0.34)
        return np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    if name == "convex_random":
        points = rng.uniform(-0.70, 0.70, size=(28, 2))
        return points[ConvexHull(points).vertices]
    if name == "concave_random7":
        return radial_polygon(7, 0.72, 0.42, rng)
    if name == "concave_random15":
        return radial_polygon(15, 0.72, 0.42, rng)
    if name == "polygon32":
        angles = np.linspace(0.0, 2.0 * np.pi, 32, endpoint=False)
        radii = 0.58 + rng.uniform(-0.08, 0.08, len(angles))
        return np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    raise ValueError(f"unknown shape {name!r}")


def concavity_ratio(vertices: np.ndarray) -> float:
    """1 - area / area(convex hull). Zero for a convex outline.

    Reported per case so that "does concavity hurt" is answered against a
    measured quantity rather than against which names sound concave.
    """
    v = np.asarray(vertices, dtype=float)
    if len(v) < 3:
        return 0.0
    area = abs(polygon_area(v))
    try:
        hull_area = float(ConvexHull(v).volume)
    except Exception:
        return 0.0
    return float(max(0.0, 1.0 - area / hull_area)) if hull_area > 1e-12 else 0.0


def build_case_config(base: dict, shape: str, seed: int, alpha: float) -> tuple[dict, dict]:
    """The baseline configuration with the object, the annulus and L written in."""
    rng = np.random.default_rng(np.random.SeedSequence([seed, SHAPE_NAMES.index(shape), 20260812]))
    yaw = float(rng.uniform(0.0, 2.0 * np.pi))

    outline = local_outline(shape, rng) * SHAPE_SCALE
    # Rotate in the body frame and place the centroid at the workspace centre.
    # Cargo.__init__ re-centres on the centroid, so the object's centre of area
    # is exactly the workspace centre and the displacement measurement has no
    # offset built into it.
    domain = base["domain"]
    centre = np.array(
        [0.5 * (domain["xmin"] + domain["xmax"]), 0.5 * (domain["ymin"] + domain["ymax"])]
    )
    world = rotate(outline, yaw) + centre[None, :]

    probe = Cargo("probe", world)
    diameter = float(polygon_diameter(probe.vertices))
    reach = float(np.max(np.linalg.norm(probe.local_vertices, axis=1)))
    distance = float(alpha * diameter)

    config = copy.deepcopy(base)
    config["cargoes"] = [
        {
            "id": "cargo_0",
            "shape": "polygon",
            "vertices": world.tolist(),
            "surface_density": 2.0,
        }
    ]
    # The robots start clear of the object rather than the object being shrunk to
    # fit a fixed annulus. 0.35 m of slack past the object's own reach keeps
    # assert_initial_state_valid satisfied for every family in the matrix.
    config["agents"] = dict(config["agents"])
    config["agents"]["center"] = centre.tolist()
    config["agents"]["radius_min"] = round(reach + 0.35, 4)
    config["agents"]["radius_max"] = round(reach + 1.15, 4)
    config["task"] = dict(config["task"])
    config["task"]["distance_min"] = distance
    config["task"]["distance_max"] = distance

    metadata = {
        "shape": shape,
        "seed": seed,
        "alpha": alpha,
        "yaw_deg": float(np.degrees(yaw)),
        "diameter_m": diameter,
        "perimeter_m": float(probe.perimeter),
        "area_m2": float(probe.area),
        "mass_kg": float(probe.mass),
        "object_reach_m": reach,
        "concavity_ratio": concavity_ratio(probe.vertices),
        "vertex_count": int(len(probe.vertices)),
        "target_distance_m": distance,
        "shape_scale_factor": SHAPE_SCALE,
        "annulus_min_m": config["agents"]["radius_min"],
        "annulus_max_m": config["agents"]["radius_max"],
    }
    return config, metadata


def classify(record: dict) -> str:
    """Failure taxonomy, most structural cause first.

    Ordered so that a case rejected before the run is never labelled by whatever
    the run then did: an object 16 robots cannot surround does not get to be a
    'transport stall'.
    """
    checks = record.get("certificate_checks") or {}

    def failed(name: str) -> bool:
        entry = checks.get(name)
        return entry is not None and not entry

    if record.get("construction_error"):
        return "CONSTRUCTION_FAILURE"
    if record.get("solver_fallbacks", 0) or record.get("solver_infeasible", 0):
        return "SOLVER_FAILURE"
    if record.get("min_inter_agent_distance") is not None and record.get("d_min") is not None:
        if record["min_inter_agent_distance"] < record["d_min"] - 1e-6:
            return "SAFETY_VIOLATION"
    if record.get("max_penetration") is not None and record.get("penetration_budget") is not None:
        if record["max_penetration"] > record["penetration_budget"] + 1e-6:
            return "SAFETY_VIOLATION"
    if failed("boundary_covering_number"):
        return "COVER_INFEASIBLE"
    if failed("cage_offset_self_clearance"):
        return "CAGE_INFEASIBLE"
    if failed("swept_cargo_corridor") or failed("swept_cage_corridor"):
        return "CORRIDOR_INFEASIBLE"
    if failed("goal_wrench_feasibility") or failed("contact_force_capacity"):
        return "WRENCH_INFEASIBLE"
    if failed("simple_polygon") or failed("feature_witness") or failed("perimeter_bound") or failed("diameter_bound"):
        return "SHAPE_INADMISSIBLE"
    if not record.get("map_complete", True):
        return "MAP_INCOMPLETE"
    if record.get("first_detection_frame") is None:
        return "SEARCH_TIMEOUT"
    if record.get("contact_ready_frame") is None:
        return "ENCLOSURE_TIMEOUT"
    if record.get("transport_frame") is None:
        return "TRANSPORT_NEVER_ARMED"
    if record.get("hold_frame") is None:
        return "TRANSPORT_STALL"
    if record.get("terminated_by") == "watchdog":
        return "WATCHDOG"
    if record.get("success"):
        return "SUCCESS"
    return "CONTRACT_FAILURE"


def wilson(successes: int, trials: int, z: float = 1.959963984540054) -> list[float] | None:
    if trials <= 0:
        return None
    p = successes / trials
    denominator = 1.0 + z * z / trials
    centre = (p + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / denominator
    return [max(0.0, centre - half), min(1.0, centre + half)]


def describe(values: list[float]) -> dict:
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not clean:
        return {"n": 0}
    a = np.asarray(clean, dtype=float)
    n = len(a)
    mean = float(a.mean())
    # Sample standard deviation, and a normal-approximation interval on the mean.
    # Reported as a mean interval, not as a claim that the samples are normal.
    sd = float(a.std(ddof=1)) if n > 1 else 0.0
    half = 1.959963984540054 * sd / math.sqrt(n) if n > 1 else 0.0
    return {
        "n": n,
        "mean": mean,
        "sd": sd,
        "ci95_mean": [mean - half, mean + half],
        "min": float(a.min()),
        "p25": float(np.quantile(a, 0.25)),
        "median": float(np.median(a)),
        "p75": float(np.quantile(a, 0.75)),
        "max": float(a.max()),
    }


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parents[1], text=True
        ).strip()
    except Exception:
        return "unknown"


def git_dirty() -> bool:
    try:
        out = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=Path(__file__).resolve().parents[1], text=True
        )
        return bool(out.strip())
    except Exception:
        return True


def run_case(config: dict, metadata: dict, seed: int, max_steps: int) -> dict:
    """One episode, plus its certificate. Exceptions become records, not gaps."""
    record: dict = dict(metadata)
    record["case_id"] = f"{metadata['shape']}__a{metadata['alpha']:.2f}__seed{seed:03d}"
    started = time.perf_counter()
    try:
        env = SimulationEnvironment(config, seed=seed)
    except Exception as exc:  # noqa: BLE001 - a rejected scenario is data
        record.update(
            construction_error=f"{type(exc).__name__}: {exc}",
            construction_traceback=traceback.format_exc(limit=3),
            domain_eligible=False,
            success=False,
            wall_seconds=time.perf_counter() - started,
        )
        record["failure_class"] = classify(record)
        return record

    cargo = env.cargoes[0]
    task = env.tasks[cargo.object_id]
    controller = controller_params_from_config(config)
    contact = contact_params_from_config(config)

    # The certificate is built on the initial state, before a single frame runs.
    # Building it afterwards would let the episode's own outcome reach the
    # premises, which is the failure mode the whole certificate exists to avoid.
    certificate = build_admissibility_certificate(
        cargo=cargo,
        agents=env.agents,
        domain=env.domain,
        goal_direction=task.direction,
        target_distance=task.distance,
        config=config,
        controller=controller,
        contact=contact,
        dt=env.dt,
    )

    termination = env.run_until_settled(max_frames=max_steps)
    wall = time.perf_counter() - started
    summary = env.summary()
    entry = summary["cargoes"][cargo.object_id]
    g500 = entry["g500"]
    m = g500["metrics"]

    # The team's own map at the end of the run, pooled over the robots. The true
    # outline is read only to score this witness; nothing here reaches control.
    pooled = [env.controller.map_snapshot(a.agent_id).points for a in env.agents]
    pooled = [p for p in pooled if len(p)]
    map_points = np.vstack(pooled) if pooled else np.empty((0, 2))
    runtime_map = evaluate_runtime_map_completeness(
        certificate=certificate, vertices=cargo.vertices, map_points=map_points
    )

    diameter = float(metadata["diameter_m"])
    j = float(m["J"])
    record.update(
        # -- identity / provenance
        sampler_attempts=int(task.attempts),
        goal_angle_deg=float(m["goal_angle_deg"]),
        # -- primary result
        J=j,
        J_over_diameter=j / diameter if diameter > 1e-9 else None,
        target_over_diameter=float(task.distance) / diameter if diameter > 1e-9 else None,
        J_over_target=j / task.distance if task.distance > 1e-9 else None,
        efficiency=float(m["efficiency"]),
        direction_error_deg=m["direction_error_deg"],
        max_cross_track=float(m["max_cross_track"]),
        max_cross_track_over_diameter=float(m["max_cross_track"]) / diameter if diameter > 1e-9 else None,
        rotation_deg=float(m["rotation_deg"]),
        # -- enclosure quality
        max_strict_coverage=float(m["max_strict_coverage"]),
        final_strict_coverage=float(m["final_strict_coverage"]),
        # -- safety
        min_inter_agent_distance=float(m["min_inter_agent_distance"]),
        d_min=float(m["d_min"]),
        min_signed_clearance=float(m["min_signed_clearance"]),
        max_penetration=float(m["max_penetration"]),
        penetration_budget=float(m["penetration_budget"]),
        # -- solver
        solver_fallbacks=int(summary["solver"]["fallbacks"]),
        solver_infeasible=int(summary["solver"]["infeasible"]),
        barrier_scalings=int(summary["solver"]["barrier_scalings"]),
        min_barrier_scale=float(summary["solver"]["min_barrier_scale"]),
        margin_relaxations=int(summary["solver"]["margin_relaxations"]),
        solves=int(summary["solver"]["solves"]),
        # -- phases and time
        first_detection_frame=m["first_detection_frame"],
        contact_ready_frame=m["contact_ready_frame"],
        transport_frame=m["transport_frame"],
        brake_frame=m["brake_frame"],
        hold_frame=m["hold_frame"],
        final_phase=m["final_phase"],
        frames_run=int(termination["frames_run"]),
        terminated_by=str(termination["terminated_by"]),
        settled=bool(termination["settled"]),
        completion_time_s=float(termination["frames_run"]) * env.dt,
        # -- runtime
        wall_seconds=wall,
        fps=float(termination["frames_run"]) / max(wall, 1e-12),
        # -- verdicts
        success=bool(g500["success"]),
        failure_reasons=list(g500["failure_reasons"]),
        # -- certificate
        domain_eligible=bool(certificate["domain_eligible"]),
        finite_time_eligible=bool(certificate["finite_time_eligible"]),
        formal_caging=bool(certificate["formal_caging"]),
        certificate_failures=list(certificate["domain_failure_reasons"]),
        certificate_checks={k: bool(v["passed"]) for k, v in certificate["checks"].items()},
        map_complete=bool(runtime_map["passed"]),
        runtime_map_gap_m=float(runtime_map["max_boundary_gap"]),
        runtime_map_gap_required_m=float(runtime_map["required_max_boundary_gap"]),
        runtime_domain_eligible=bool(runtime_map["runtime_domain_eligible"]),
        certified_inscribed_radius=float(certificate["shape"]["certified_inscribed_radius"]),
        required_boundary_agents=certificate["shape"]["required_boundary_agents"],
    )
    record["failure_class"] = classify(record)
    return record


def summarise(records: list[dict]) -> dict:
    total = len(records)
    eligible = [r for r in records if r.get("runtime_domain_eligible")]
    pre_eligible = [r for r in records if r.get("domain_eligible")]
    successes = [r for r in records if r.get("success")]
    eligible_successes = [r for r in eligible if r.get("success")]

    def block(rows: list[dict]) -> dict:
        return {
            "n": len(rows),
            "successes": sum(1 for r in rows if r.get("success")),
            "J": describe([r.get("J") for r in rows]),
            "J_over_diameter": describe([r.get("J_over_diameter") for r in rows]),
            "J_over_target": describe([r.get("J_over_target") for r in rows]),
            "efficiency": describe([r.get("efficiency") for r in rows]),
            "direction_error_deg": describe([r.get("direction_error_deg") for r in rows]),
            "max_cross_track": describe([r.get("max_cross_track") for r in rows]),
            "max_cross_track_over_diameter": describe(
                [r.get("max_cross_track_over_diameter") for r in rows]
            ),
            "max_strict_coverage": describe([r.get("max_strict_coverage") for r in rows]),
            "final_strict_coverage": describe([r.get("final_strict_coverage") for r in rows]),
            "min_inter_agent_distance": describe([r.get("min_inter_agent_distance") for r in rows]),
            "barrier_scalings": describe([r.get("barrier_scalings") for r in rows]),
            "completion_time_s": describe([r.get("completion_time_s") for r in rows]),
            "frames_run": describe([r.get("frames_run") for r in rows]),
            "fps": describe([r.get("fps") for r in rows]),
            "diameter_m": describe([r.get("diameter_m") for r in rows]),
            "failure_composition": dict(Counter(r.get("failure_class") for r in rows)),
            "watchdog": sum(1 for r in rows if r.get("terminated_by") == "watchdog"),
            "hold_reached": sum(1 for r in rows if r.get("hold_frame") is not None),
            "solver_fallbacks": sum(int(r.get("solver_fallbacks") or 0) for r in rows),
            "solver_infeasible": sum(int(r.get("solver_infeasible") or 0) for r in rows),
            "barrier_scaling_total": sum(int(r.get("barrier_scalings") or 0) for r in rows),
        }

    shapes = sorted({r["shape"] for r in records})
    alphas = sorted({r["alpha"] for r in records})

    # The five worst cases by normalised displacement, so the table cannot be
    # read as a highlight reel. Construction failures sort first: no J at all is
    # worse than a small one.
    def sort_key(r: dict):
        value = r.get("J_over_diameter")
        return (0.0 if value is None else float(value), r["case_id"])

    worst = sorted(records, key=sort_key)[:5]

    return {
        "episodes": total,
        "construction_failures": sum(1 for r in records if r.get("construction_error")),
        "P_eligible_pre_run": len(pre_eligible) / total if total else None,
        "P_eligible_pre_run_wilson95": wilson(len(pre_eligible), total),
        "P_eligible": len(eligible) / total if total else None,
        "P_eligible_wilson95": wilson(len(eligible), total),
        "P_success_given_eligible": len(eligible_successes) / len(eligible) if eligible else None,
        "P_success_given_eligible_wilson95": wilson(len(eligible_successes), len(eligible)),
        "P_success": len(successes) / total if total else None,
        "P_success_wilson95": wilson(len(successes), total),
        "rejected_pre_run": total - len(pre_eligible),
        "rejected_runtime": len(pre_eligible) - len(eligible),
        "rejection_composition": dict(
            Counter(reason for r in records for reason in (r.get("certificate_failures") or []))
        ),
        "overall": block(records),
        "eligible_only": block(eligible),
        "per_shape": {s: block([r for r in records if r["shape"] == s]) for s in shapes},
        "per_alpha": {f"{a:.2f}": block([r for r in records if r["alpha"] == a]) for a in alphas},
        "per_shape_alpha": {
            f"{s}|{a:.2f}": block([r for r in records if r["shape"] == s and r["alpha"] == a])
            for s in shapes
            for a in alphas
        },
        "worst_five": [
            {
                "case_id": r["case_id"],
                "shape": r["shape"],
                "alpha": r["alpha"],
                "seed": r["seed"],
                "diameter_m": r.get("diameter_m"),
                "J": r.get("J"),
                "J_over_diameter": r.get("J_over_diameter"),
                "efficiency": r.get("efficiency"),
                "max_cross_track": r.get("max_cross_track"),
                "final_phase": r.get("final_phase"),
                "failure_class": r.get("failure_class"),
                "failure_reasons": r.get("failure_reasons"),
                "certificate_failures": r.get("certificate_failures"),
            }
            for r in worst
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/v2/shape_matrix.yaml")
    parser.add_argument("--seeds", default="0..4")
    parser.add_argument("--alpha", default="0.1,0.4,0.8")
    parser.add_argument("--shapes", nargs="+", choices=SHAPE_NAMES, default=list(SHAPE_NAMES))
    parser.add_argument("--max-steps", type=int, default=3000)
    parser.add_argument("--output", default="runs/v2_shape_matrix")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    alphas = parse_alphas(args.alpha)
    base = load_yaml(args.config)
    config_text = Path(args.config).read_text(encoding="utf-8")
    config_hash = hashlib.sha256(config_text.encode("utf-8")).hexdigest()

    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    checkpoint = root / "checkpoint.json"
    records: list[dict] = []
    if args.resume and checkpoint.exists():
        records = json.loads(checkpoint.read_text(encoding="utf-8"))
    done = {r["case_id"] for r in records}

    manifest = {
        "schema_version": 2,
        "experiment": "v2 decisive matrix: v1 controller x 12 shape families x scale-relative distance",
        "git_sha": git_sha(),
        "git_dirty": git_dirty(),
        "config_path": args.config,
        "config_sha256": config_hash,
        "seeds": seeds,
        "alphas": alphas,
        "shapes": list(args.shapes),
        "shape_scale_factor": SHAPE_SCALE,
        "max_steps_watchdog": args.max_steps,
        "task_distance_rule": "L = alpha * polygon_diameter(cargo)",
        "screening": "none; every case enters the denominator including rejected and crashed ones",
        "held_fixed": {
            "surface_density": 2.0,
            "ground_friction": 0.60,
            "contact_friction": 0.60,
            "agent_count": 16,
            "placement": "cargo centroid at workspace centre for every case",
        },
        "declared_but_unverified_premises": [
            "guarantee.bounded_errors.normal_error_deg",
            "guarantee.bounded_errors.velocity_error",
            "guarantee.finite_time.enclosure_contraction_rate_hz",
            "guarantee.finite_time.transport_progress_rate_mps",
            "guarantee.finite_time.brake_contraction_rate_hz",
        ],
        "cases": [],
    }

    total_cases = len(args.shapes) * len(alphas) * len(seeds)
    index = 0
    for shape in args.shapes:
        for alpha in alphas:
            for seed in seeds:
                index += 1
                case_id = f"{shape}__a{alpha:.2f}__seed{seed:03d}"
                if case_id in done:
                    continue
                config, metadata = build_case_config(base, shape, seed, alpha)
                manifest["cases"].append({"case_id": case_id, **metadata})
                record = run_case(config, metadata, seed, args.max_steps)
                records.append(record)
                checkpoint.write_text(json.dumps(records, indent=2, default=str), encoding="utf-8")
                jd = record.get("J_over_diameter")
                print(
                    f"[{index:3d}/{total_cases}] {case_id:38s} "
                    f"d={record.get('diameter_m', 0):.2f} "
                    f"L={record.get('target_distance_m', 0):.2f} "
                    f"J={record.get('J', float('nan')):6.3f} "
                    f"J/d={jd if jd is not None else float('nan'):5.3f} "
                    f"elig={int(bool(record.get('runtime_domain_eligible')))} "
                    f"ok={int(bool(record.get('success')))} "
                    f"{record.get('failure_class')}",
                    flush=True,
                )

    statistics = summarise(records)
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str), encoding="utf-8")
    (root / "monte_carlo.json").write_text(
        json.dumps({"manifest": manifest, "statistics": statistics, "records": records}, indent=2, default=str),
        encoding="utf-8",
    )
    if records:
        scalar_fields: list[str] = []
        for record in records:
            for key, value in record.items():
                if key not in scalar_fields and not isinstance(value, (dict, list)):
                    scalar_fields.append(key)
        with (root / "episodes.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=scalar_fields)
            writer.writeheader()
            for record in records:
                writer.writerow({key: record.get(key) for key in scalar_fields})
    print(json.dumps({k: v for k, v in statistics.items() if k != "per_shape_alpha"}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
