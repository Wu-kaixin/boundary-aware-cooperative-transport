#!/usr/bin/env python3
"""Run an unscreened arbitrary-simple-shape closed-loop Monte Carlo matrix."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402


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


def radial_polygon(count: int, outer: float, inner: float, rng: np.random.Generator) -> np.ndarray:
    step = 2.0 * np.pi / count
    angles = step * np.arange(count) + rng.uniform(-0.18 * step, 0.18 * step, count)
    angles.sort()
    radii = rng.uniform(inner, outer, count)
    if count >= 6:
        radii[1::3] *= 0.55
    return np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])


def shape_config(name: str, rng: np.random.Generator) -> dict:
    common = {"id": "cargo_0", "surface_density": 1.5, "random_center": {"enabled": False}}
    if name == "circle":
        return {**common, "shape": "circle", "radius": 0.58}
    if name == "rectangle":
        return {**common, "shape": "rectangle", "width": 1.15, "height": 0.72}
    if name == "l_shape":
        return {**common, "shape": "l_shape", "scale": 0.95}
    if name == "high_aspect":
        return {**common, "shape": "rectangle", "width": 1.60, "height": 0.25}
    if name == "ellipse24":
        angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        vertices = np.column_stack([0.72 * np.cos(angles), 0.42 * np.sin(angles)])
    elif name == "u_shape":
        vertices = np.array(
            [
                [-0.65, -0.55], [0.65, -0.55], [0.65, 0.55], [0.32, 0.55],
                [0.32, -0.20], [-0.32, -0.20], [-0.32, 0.55], [-0.65, 0.55],
            ]
        )
    elif name == "c_shape":
        vertices = np.array(
            [
                [-0.65, -0.55], [0.65, -0.55], [0.65, -0.28], [-0.25, -0.28],
                [-0.25, 0.28], [0.65, 0.28], [0.65, 0.55], [-0.65, 0.55],
            ]
        )
    elif name == "star10":
        angles = np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False)
        radii = np.where(np.arange(10) % 2 == 0, 0.70, 0.34)
        vertices = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    elif name == "convex_random":
        points = rng.uniform(-0.70, 0.70, size=(28, 2))
        hull = ConvexHull(points)
        vertices = points[hull.vertices]
    elif name == "concave_random7":
        vertices = radial_polygon(7, 0.72, 0.42, rng)
    elif name == "concave_random15":
        vertices = radial_polygon(15, 0.72, 0.42, rng)
    elif name == "polygon32":
        angles = np.linspace(0.0, 2.0 * np.pi, 32, endpoint=False)
        radii = 0.58 + rng.uniform(-0.08, 0.08, len(angles))
        vertices = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    else:
        raise ValueError(f"unknown shape {name!r}")
    return {**common, "shape": "polygon", "vertices_frame": "local", "vertices": vertices.tolist()}


def place_shape(
    item: dict,
    position_class: str,
    yaw: float,
    rng: np.random.Generator,
) -> dict:
    placed = copy.deepcopy(item)
    if placed["shape"] == "circle":
        radius = float(placed["radius"])
    elif placed["shape"] == "rectangle":
        radius = 0.5 * math.hypot(float(placed["width"]), float(placed["height"]))
    elif placed["shape"] == "l_shape":
        radius = float(placed["scale"])
    else:
        radius = float(np.max(np.linalg.norm(np.asarray(placed["vertices"]), axis=1)))
    margin = radius + 0.55
    if position_class == "center":
        center = np.array([4.0, 4.0])
    elif position_class == "edge":
        center = np.array([margin, rng.uniform(margin, 8.0 - margin)])
    elif position_class == "corner":
        center = np.array([margin, margin])
    elif position_class == "random":
        center = rng.uniform(margin, 8.0 - margin, size=2)
    else:
        raise ValueError(position_class)
    placed["center"] = center.tolist()
    placed["yaw"] = float(yaw)
    return placed


def configure_case(base: dict, shape_name: str, seed: int, index: int, distance: float) -> tuple[dict, dict]:
    rng = np.random.default_rng(np.random.SeedSequence([seed, index, 20260809]))
    position_class = ("center", "edge", "corner", "random")[(seed + index) % 4]
    yaw = float(rng.uniform(0.0, 2.0 * np.pi))
    item = place_shape(shape_config(shape_name, rng), position_class, yaw, rng)
    density = float((0.8, 1.5, 2.2)[(seed + 2 * index) % 3])
    ground_friction = float((0.15, 0.30, 0.45)[(2 * seed + index) % 3])
    contact_friction = float((0.40, 0.60, 0.80)[(seed + index) % 3])
    item["surface_density"] = density

    config = copy.deepcopy(base)
    config["cargoes"] = [item]
    config["task"]["random_goal"]["target_distance"] = float(distance)
    config["controller"]["transport_distance"] = float(distance + 0.001)
    config["transport"]["ground_friction"] = ground_friction
    config["transport"]["friction"] = contact_friction
    config["evaluation"]["j_min"] = max(0.0, float(distance - 0.025))
    config["evaluation"]["j_max"] = float(distance + max(0.15, 0.3 * distance))
    config["evaluation"]["require_guarantee_certificate"] = False
    config["guarantee"].update(
        {
            "enabled": True,
            "transport_target_reserve_m": 0.001,
            "min_feature_radius": 0.04,
            "max_perimeter": 8.0,
            "max_diameter": 2.5,
            "boundary_map_epsilon": 0.10,
            "force_margin": 1.05,
            "bounded_errors": {"normal_error_deg": 89.99, "velocity_error": 0.35},
            "search": {"required_returns": 1},
        }
    )
    metadata = {
        "shape": shape_name,
        "seed": seed,
        "position_class": position_class,
        "center": item["center"],
        "yaw_deg": float(np.degrees(yaw)),
        "surface_density": density,
        "ground_friction": ground_friction,
        "contact_friction": contact_friction,
        "vertex_count": (
            len(item.get("vertices", []))
            if item["shape"] == "polygon"
            else (96 if item["shape"] == "circle" else None)
        ),
    }
    return config, metadata


def classify_failure(summary: dict, cargo: dict, certificate: dict) -> str:
    termination = (summary.get("termination") or {}).get("status")
    phases = cargo.get("phase_frames") or {}
    checks = certificate.get("checks") or {}
    if termination == "SOLVER_FAILURE":
        return "SOLVER_FAILURE"
    if summary.get("min_inter_agent_distance", float("inf")) < summary["contracts"]["d_min"] - 1e-9:
        return "SAFETY_VIOLATION"
    if phases.get("first_detection") is None:
        return "SEARCH_TIMEOUT"
    if "boundary_map_epsilon" in (certificate.get("runtime_failure_reasons") or []):
        return "MAP_INCOMPLETE"
    if not checks.get("cage_offset_self_clearance", {}).get("passed", False):
        return "CAGE_INFEASIBLE"
    if not checks.get("swept_cargo_corridor", {}).get("passed", False) or not checks.get(
        "swept_cage_corridor", {}
    ).get("passed", False):
        return "GOAL_CORRIDOR_INFEASIBLE"
    if not checks.get("goal_wrench_feasibility", {}).get("passed", False) or not checks.get(
        "contact_force_capacity", {}
    ).get("passed", False):
        return "WRENCH_INFEASIBLE"
    if phases.get("first_enclosure") is None:
        return "ENCLOSURE_TIMEOUT"
    if phases.get("first_transport") is not None and phases.get("first_hold") is None:
        return "TRANSPORT_STALL"
    if termination == "TIMEOUT":
        return "TRANSPORT_STALL"
    return "SUCCESS" if cargo.get("success") is True and termination == "SUCCESS_HOLD" else "CONTRACT_FAILURE"


def empirical_completion_bound(success_frames: list[int], eligible_failures: int, alpha: float = 0.05) -> dict:
    if eligible_failures or not success_frames:
        return {
            "available": False,
            "reason": "eligible failures are right-censored; no finite completion-time bound reported",
            "confidence": 1.0 - alpha,
        }
    count = len(success_frames)
    return {
        "available": True,
        "classification": "empirical_iid_max_tolerance_bound",
        "analytic_theorem": False,
        "observed_max_frames": int(max(success_frames)),
        "sample_count": count,
        "confidence": 1.0 - alpha,
        "population_coverage_lower": float(alpha ** (1.0 / count)),
        "assumptions": "episodes are iid from the manifest sampling distribution",
    }


def wilson_interval(successes: int, trials: int, z: float = 1.959963984540054) -> list[float] | None:
    if trials <= 0:
        return None
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = (proportion + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(
        proportion * (1.0 - proportion) / trials + z * z / (4.0 * trials * trials)
    ) / denominator
    return [max(0.0, center - half), min(1.0, center + half)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/sim/research/adaptive_progress_closed_loop.yaml",
    )
    parser.add_argument("--seeds", default="0..4")
    parser.add_argument("--shapes", nargs="+", choices=SHAPE_NAMES, default=list(SHAPE_NAMES))
    parser.add_argument("--distance", type=float, default=0.10)
    parser.add_argument("--max-steps", type=int, default=1500)
    parser.add_argument("--truth-audit", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", default="runs/arbitrary_shape_monte_carlo")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    if not seeds:
        raise ValueError("at least one seed is required")
    base = load_yaml(args.config)
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    checkpoint_path = root / "checkpoint.json"
    if args.resume and checkpoint_path.exists():
        checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
        records = list(checkpoint.get("records", []))
        manifest_cases = list(checkpoint.get("cases", []))
    else:
        records = []
        manifest_cases = []
    for record in records:
        if "contract_failure_reasons" in record:
            continue
        summary_path = root / "episodes" / record["case_id"] / "summary.json"
        if summary_path.exists():
            saved_summary = json.loads(summary_path.read_text(encoding="utf-8"))
            saved_cargo = saved_summary.get("cargoes", {}).get("cargo_0", {})
            record["contract_failure_reasons"] = saved_cargo.get("failure_reasons", [])
    existing_ids = {record["case_id"] for record in records}
    for seed in seeds:
        for shape_name in args.shapes:
            # A filtered run must reproduce the exact case from the full matrix.
            # Enumeration over the CLI subset changed position/friction/yaw
            # strata and made performance ablations incomparable.
            index = SHAPE_NAMES.index(shape_name)
            config, metadata = configure_case(base, shape_name, seed, index, args.distance)
            config["evaluation"]["online_truth_audit"] = bool(args.truth_audit)
            config["evaluation"]["require_measured_error_bounds"] = bool(args.truth_audit)
            case_id = f"{shape_name}__seed_{seed:03d}"
            if case_id in existing_ids:
                continue
            manifest_cases.append({"case_id": case_id, **metadata})
            started = time.perf_counter()
            env = SimulationEnvironment(config, seed=seed)
            termination = env.run_until(args.max_steps)
            wall = time.perf_counter() - started
            summary = env.save_outputs(root / "episodes" / case_id)
            cargo = summary["cargoes"]["cargo_0"]
            certificate = cargo["guarantee_certificate"]
            domain_eligible = bool(certificate.get("runtime_domain_eligible"))
            task_success = bool(termination.success and cargo.get("success") is True)
            failure = classify_failure(summary, cargo, certificate)
            record = {
                "case_id": case_id,
                **metadata,
                "goal_angle_deg": cargo.get("goal_angle_deg"),
                "diameter_m": (certificate.get("shape") or {}).get("diameter"),
                "perimeter_m": (certificate.get("shape") or {}).get("perimeter"),
                "domain_eligible": domain_eligible,
                "finite_time_eligible": bool(certificate.get("finite_time_eligible")),
                "theorem_eligible": bool(certificate.get("runtime_eligible")),
                "rejection_reasons": certificate.get("runtime_domain_failure_reasons", []),
                "termination": termination.status,
                "task_success": task_success,
                "success_given_eligible": bool(domain_eligible and task_success),
                "failure_class": failure,
                "contract_failure_reasons": cargo.get("failure_reasons", []),
                "frame": termination.frame,
                "wall_seconds": wall,
                "fps": termination.frame / max(wall, 1e-12),
                "J_m": cargo.get("J"),
                "efficiency": cargo.get("efficiency"),
                "max_cross_track_error_m": cargo.get("max_cross_track_error"),
                "max_abs_rotation_deg": cargo.get("max_abs_rotation_deg"),
                "max_penetration_m": cargo.get("max_penetration"),
                "operational_enclosure_maintained": cargo.get(
                    "operational_enclosure_maintained_during_transport"
                ),
                "fallbacks": summary["solver"]["fallbacks"],
                "infeasible": summary["solver"]["infeasible"],
                "rho_relaxations": summary["solver"]["margin_relaxations"],
                "phase_frames": cargo.get("phase_frames"),
            }
            records.append(record)
            existing_ids.add(case_id)
            checkpoint_path.write_text(
                json.dumps({"records": records, "cases": manifest_cases}, indent=2),
                encoding="utf-8",
            )
            print(
                json.dumps(
                    {
                        "case": case_id,
                        "eligible": domain_eligible,
                        "success": task_success,
                        "failure": failure,
                        "frame": termination.frame,
                        "fps": record["fps"],
                    }
                ),
                flush=True,
            )

    total = len(records)
    eligible = [record for record in records if record["domain_eligible"]]
    eligible_successes = [record for record in eligible if record["task_success"]]
    rejected = [record for record in records if not record["domain_eligible"]]
    eligible_failures = len(eligible) - len(eligible_successes)
    statistics = {
        "episodes": total,
        "eligible": len(eligible),
        "eligible_successes": len(eligible_successes),
        "eligible_failures": eligible_failures,
        "rejected": len(rejected),
        "P_eligible": len(eligible) / total if total else None,
        "P_eligible_wilson95": wilson_interval(len(eligible), total),
        "P_success_given_eligible": (
            len(eligible_successes) / len(eligible) if eligible else None
        ),
        "P_success_given_eligible_wilson95": wilson_interval(
            len(eligible_successes),
            len(eligible),
        ),
        "P_rejected": len(rejected) / total if total else None,
        "P_rejected_wilson95": wilson_interval(len(rejected), total),
        "task_success_all": sum(bool(record["task_success"]) for record in records),
        "solver": {
            "fallback_free_episodes": sum(record["fallbacks"] == 0 for record in records),
            "infeasible_free_episodes": sum(record["infeasible"] == 0 for record in records),
            "rho_relaxation_free_episodes": sum(
                record["rho_relaxations"] == 0 for record in records
            ),
        },
        "runtime_fps": {
            "min": min((record["fps"] for record in records), default=None),
            "mean": (
                sum(record["fps"] for record in records) / total if total else None
            ),
            "median": (
                float(np.median([record["fps"] for record in records])) if total else None
            ),
            "max": max((record["fps"] for record in records), default=None),
        },
        "failure_composition": dict(Counter(record["failure_class"] for record in records)),
        "rejection_composition": dict(
            Counter(reason for record in rejected for reason in record["rejection_reasons"])
        ),
        "completion_time_bound": empirical_completion_bound(
            [int(record["frame"]) for record in eligible_successes],
            eligible_failures,
        ),
    }
    manifest = {
        "schema_version": 1,
        "config": args.config,
        "seeds": seeds,
        "shapes": args.shapes,
        "distance_m": args.distance,
        "max_steps_timeout": args.max_steps,
        "truth_audit": bool(args.truth_audit),
        "sampling": {
            "position_classes": ["center", "edge", "corner", "random"],
            "yaw": "uniform [0, 2*pi)",
            "goal_direction": "uniform feasible direction from scenario sampler",
            "surface_density": [0.8, 1.5, 2.2],
            "ground_friction": [0.15, 0.30, 0.45],
            "contact_friction": [0.40, 0.60, 0.80],
        },
        "cases": manifest_cases,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (root / "monte_carlo.json").write_text(
        json.dumps({"manifest": manifest, "statistics": statistics, "records": records}, indent=2),
        encoding="utf-8",
    )
    if records:
        with (root / "episodes.csv").open("w", newline="", encoding="utf-8") as handle:
            scalar_fields = [key for key, value in records[0].items() if not isinstance(value, (dict, list))]
            writer = csv.DictWriter(handle, fieldnames=scalar_fields)
            writer.writeheader()
            for record in records:
                writer.writerow({key: record.get(key) for key in scalar_fields})
    print(json.dumps(statistics, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
