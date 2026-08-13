#!/usr/bin/env python
"""T6 - task distance, swept scale-relative over 12 seeds.

    python scripts/run_distance_ablation.py --seeds 0..11 --out docs/results/t6

Ported from CODEX's ``run_distance_ablation.py`` and re-stated in this branch's own terms.

What changed in the port, and why
--------------------------------
CODEX swept **fixed metric distances** -- 0.10, 0.25, 0.50 m -- and set
``controller.transport_distance``, ``evaluation.j_min`` and ``evaluation.j_max`` per arm.
None of those fields exist in v1: the distance lives once, on the sampled ``TransportTask``,
and the team closes its own loop on its own registration estimate rather than reading a
configured distance. There is nothing to reconcile, which is also why
``guarantees.py`` has no ``transport_distance_consistency`` predicate.

More importantly, a fixed metric distance is the thing the decisive matrix was built to
avoid. CODEX's 0.10 m on a 1.8 m object made its reported ``J/diameter`` a statement about
the number in the config rather than about the object, which is how it came out at 4-10%.
This sweep is therefore over **alpha = L / diameter**, the same scale-relative quantity the
matrix used, and it extends the matrix's three alpha levels to five on a single shape with
twelve seeds instead of five -- so the alpha trend the matrix found on 12 families at 5
seeds can be checked on 1 family at 12 seeds.

``alpha = 1.0`` is included deliberately, past the matrix's 0.8: it asks the object to
travel its own diameter, and the point of a sweep is to find where the method stops rather
than to stop where it still works.

Every arm rewrites only ``task.distance_min`` and ``task.distance_max``, both to the same
value, so the sampler cannot widen the target. Nothing else moves -- no gate, no threshold,
no controller parameter.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.geometry import polygon_diameter  # noqa: E402
from dbact.provenance import config_hash, git_sha  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import build_cargoes, load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"
ALPHAS = (0.2, 0.4, 0.6, 0.8, 1.0)
CROSS_TRACK_GATE = 0.15


def parse_seeds(spec: str) -> list[int]:
    if ".." in spec:
        low, high = (int(p) for p in spec.split("..", 1))
        return list(range(low, high + 1))
    return [int(p) for p in spec.split(",") if p.strip()]


def wilson(successes: int, trials: int, z: float = 1.959963984540054) -> list[float] | None:
    if trials <= 0:
        return None
    p = successes / trials
    denominator = 1.0 + z * z / trials
    centre = (p + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / denominator
    return [max(0.0, centre - half), min(1.0, centre + half)]


def describe(values) -> dict:
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not clean:
        return {"n": 0}
    a = np.asarray(clean, dtype=float)
    return {
        "n": len(a),
        "mean": float(a.mean()),
        "sd": float(a.std(ddof=1)) if len(a) > 1 else 0.0,
        "min": float(a.min()),
        "median": float(np.median(a)),
        "max": float(a.max()),
    }


def baseline_diameter(base: dict) -> float:
    """The baseline cargo's diameter, so alpha can be turned into metres."""
    cargo = build_cargoes(base, seed=0)[0]
    return float(polygon_diameter(cargo.vertices))


def run_seed(config: dict, seed: int, alpha: float, diameter: float, max_frames: int) -> dict:
    env = SimulationEnvironment(config, seed=seed)
    started = time.perf_counter()
    termination = env.run_until_settled(max_frames=max_frames)
    wall = time.perf_counter() - started
    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    g500 = entry["g500"]
    m = g500["metrics"]
    task = next(iter(env.tasks.values()))
    j = float(m["J"])
    distance = float(task.distance)

    return {
        "alpha": alpha,
        "seed": seed,
        "diameter_m": diameter,
        "target_distance_m": distance,
        "sampler_attempts": int(task.attempts),
        "goal_angle_deg": float(m["goal_angle_deg"]),
        "success": bool(g500["success"]),
        "failure_reasons": "|".join(g500["failure_reasons"]),
        "J": j,
        "J_over_diameter": j / diameter,
        "J_over_target": j / distance if distance > 1e-9 else None,
        "efficiency": float(m["efficiency"]),
        "direction_error_deg": m["direction_error_deg"],
        "max_cross_track": float(m["max_cross_track"]),
        "max_cross_track_over_diameter": float(m["max_cross_track"]) / diameter,
        "implied_direction_gate_deg": (
            math.degrees(math.asin(min(1.0, CROSS_TRACK_GATE / j))) if j > CROSS_TRACK_GATE else 90.0
        ),
        "rotation_deg": float(m["rotation_deg"]),
        "max_strict_coverage": float(m["max_strict_coverage"]),
        "final_strict_coverage": float(m["final_strict_coverage"]),
        "min_inter_agent_distance": float(m["min_inter_agent_distance"]),
        "d_min": float(m["d_min"]),
        "min_signed_clearance": float(m["min_signed_clearance"]),
        "max_penetration": float(m["max_penetration"]),
        "penetration_budget": float(m["penetration_budget"]),
        "barrier_scalings": int(summary["solver"]["barrier_scalings"]),
        "margin_relaxations": int(summary["solver"]["margin_relaxations"]),
        "solver_fallbacks": int(summary["solver"]["fallbacks"]),
        "solver_infeasible": int(summary["solver"]["infeasible"]),
        "solves": int(summary["solver"]["solves"]),
        "first_detection_frame": m["first_detection_frame"],
        "contact_ready_frame": m["contact_ready_frame"],
        "transport_frame": m["transport_frame"],
        "brake_frame": m["brake_frame"],
        "hold_frame": m["hold_frame"],
        "final_phase": m["final_phase"],
        "frames_run": int(termination["frames_run"]),
        "terminated_by": str(termination["terminated_by"]),
        "settled": bool(termination["settled"]),
        "completion_time_s": float(termination["frames_run"]) * env.dt,
        "fps": float(termination["frames_run"]) / max(wall, 1e-12),
    }


NUMERIC = [
    "J", "J_over_diameter", "J_over_target", "efficiency", "direction_error_deg",
    "max_cross_track", "max_cross_track_over_diameter", "implied_direction_gate_deg",
    "rotation_deg", "max_strict_coverage", "final_strict_coverage",
    "min_signed_clearance", "max_penetration", "barrier_scalings", "frames_run",
    "completion_time_s", "fps", "target_distance_m",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--alphas", default=",".join(str(a) for a in ALPHAS))
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="docs/results/t6")
    args = parser.parse_args()

    base = load_yaml(args.config)
    seeds = parse_seeds(args.seeds)
    alphas = [float(a) for a in args.alphas.split(",") if a.strip()]
    diameter = baseline_diameter(base)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    print(f"baseline cargo diameter {diameter:.4f} m", flush=True)

    rows: list[dict] = []
    for alpha in alphas:
        distance = alpha * diameter
        config = copy.deepcopy(base)
        config["task"] = dict(config["task"])
        config["task"]["distance_min"] = distance
        config["task"]["distance_max"] = distance
        for seed in seeds:
            row = run_seed(config, seed, alpha, diameter, args.max_frames)
            rows.append(row)
            print(
                f"[a={alpha:.1f}] seed {seed:2d}  {'PASS' if row['success'] else 'FAIL'}  "
                f"L={row['target_distance_m']:.3f}  J={row['J']:.3f}  "
                f"J/d={row['J_over_diameter']:.3f}  J/L={row['J_over_target']:.3f}  "
                f"cross/d={row['max_cross_track_over_diameter']:.4f}  "
                f"dir={row['direction_error_deg']:5.2f}deg  scal={row['barrier_scalings']:3d}",
                flush=True,
            )
        print(flush=True)

    per_alpha = {}
    for alpha in alphas:
        sub = [r for r in rows if r["alpha"] == alpha]
        successes = sum(1 for r in sub if r["success"])
        per_alpha[f"{alpha:.1f}"] = {
            "n": len(sub),
            "successes": successes,
            "success_rate": successes / len(sub) if sub else None,
            "success_wilson95": wilson(successes, len(sub)),
            "target_distance_m": alpha * diameter,
            "separation_held": all(
                r["min_inter_agent_distance"] >= r["d_min"] - 1e-6 for r in sub
            ),
            "solver_fallbacks": sum(r["solver_fallbacks"] for r in sub),
            "solver_infeasible": sum(r["solver_infeasible"] for r in sub),
            "barrier_scalings": sum(r["barrier_scalings"] for r in sub),
            "over_cross_track_gate": sum(1 for r in sub if r["max_cross_track"] > CROSS_TRACK_GATE),
            **{field: describe([r.get(field) for r in sub]) for field in NUMERIC},
        }

    # The alpha trend, as a correlation over episodes rather than over arm means: 60
    # points, not 5, so the interval is honest about the spread inside each arm.
    alpha_values = np.array([r["alpha"] for r in rows])
    report = {
        "experiment": "T6 scale-relative task-distance ablation",
        "config": args.config,
        "config_hash": config_hash(base),
        "git_sha": git_sha(ROOT),
        "baseline_diameter_m": diameter,
        "alphas": alphas,
        "seeds": seeds,
        "episodes": len(rows),
        "cross_track_gate_m": CROSS_TRACK_GATE,
        "correlations_over_episodes": {
            "alpha_vs_J_over_diameter": float(
                np.corrcoef(alpha_values, [r["J_over_diameter"] for r in rows])[0, 1]
            ),
            "alpha_vs_cross_track_over_diameter": float(
                np.corrcoef(alpha_values, [r["max_cross_track_over_diameter"] for r in rows])[0, 1]
            ),
            "alpha_vs_J_over_target": float(
                np.corrcoef(alpha_values, [r["J_over_target"] for r in rows])[0, 1]
            ),
            "alpha_vs_max_strict_coverage": float(
                np.corrcoef(alpha_values, [r["max_strict_coverage"] for r in rows])[0, 1]
            ),
        },
        "per_alpha": per_alpha,
        "runs": rows,
    }
    (out / "distance_ablation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    fields = list(rows[0].keys())
    with (out / "distance_episodes.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print("=" * 104)
    print(f"{'alpha':>6} {'L (m)':>7} {'pass':>7} {'J/d':>7} {'J/L':>7} {'cross/d':>8} "
          f"{'dir':>7} {'gate':>7} {'peakcov':>8} {'scal':>5} {'sep':>4}")
    for alpha in alphas:
        s = per_alpha[f"{alpha:.1f}"]
        print(
            f"{alpha:>6.1f} {s['target_distance_m']:>7.3f} "
            f"{s['successes']:>3}/{s['n']:<3} {s['J_over_diameter']['mean']:>7.3f} "
            f"{s['J_over_target']['mean']:>7.3f} "
            f"{s['max_cross_track_over_diameter']['mean']:>8.4f} "
            f"{s['direction_error_deg']['mean']:>7.2f} "
            f"{s['implied_direction_gate_deg']['mean']:>7.2f} "
            f"{s['max_strict_coverage']['mean']:>8.3f} {s['barrier_scalings']:>5} "
            f"{'ok' if s['separation_held'] else 'BAD':>4}"
        )
    print("=" * 104)
    for name, value in report["correlations_over_episodes"].items():
        print(f"  corr {name:44s} {value:+.4f}")
    print(f"wrote {out / 'distance_ablation.json'} and {out / 'distance_episodes.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
