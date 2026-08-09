#!/usr/bin/env python
"""D8 - multi-seed G500 evaluation, reported per gate.

    python scripts/evaluate_closed_loop.py --seeds 0..11 --out runs/d_sweep

A single success rate hides which gate is failing, and a mean over a table that
silently drops the failures is worse than no table. This script reports the rate,
a Wilson interval for it, the distribution of every scored quantity across *all*
seeds including the failures, and a count of how often each individual gate was
the one that failed -- which is the number that says what to work on next.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from collections import Counter
from pathlib import Path

import numpy as np

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml

GATE_PATTERNS = (
    ("detection deadline", r"object detection"),
    ("enclosure deadline", r"enclosure / contact-ready"),
    ("transport deadline", r"transport activation"),
    ("target not reached", r"target reached|< target L"),
    ("overshoot", r"did not stop"),
    ("efficiency", r"efficiency"),
    ("direction error", r"direction error"),
    ("cross-track", r"cross-track"),
    ("coverage", r"never enclosed"),
    ("yaw", r"rotated"),
    ("not holding", r"did not end in HOLD"),
    ("still drifting", r"still drifting"),
    ("solver fallback", r"solver fallback"),
    ("qp infeasible", r"QP infeasibility"),
    ("scaled barrier", r"scaled-barrier"),
    ("inter-agent distance", r"min inter-agent distance"),
    ("clearance", r"entered the cargo"),
    ("penetration", r"max penetration"),
)


def parse_seeds(text: str) -> list[int]:
    if ".." in text:
        low, high = text.split("..")
        return list(range(int(low), int(high) + 1))
    return [int(part) for part in text.split(",") if part.strip()]


def classify(reason: str) -> str:
    for name, pattern in GATE_PATTERNS:
        if re.search(pattern, reason):
            return name
    return "other"


def wilson(successes: int, total: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval. A proportion out of twelve needs its interval."""
    if total == 0:
        return (0.0, 1.0)
    p = successes / total
    denominator = 1.0 + z * z / total
    centre = (p + z * z / (2 * total)) / denominator
    spread = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denominator
    return (max(0.0, centre - spread), min(1.0, centre + spread))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_closed_loop.yaml")
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--frames", type=int, default=500,
                        help="Fixed frame budget. Ignored when --until-settled is given.")
    parser.add_argument("--until-settled", action="store_true",
                        help="Run each episode to completion instead of to a budget, so that the "
                             "enclosure and transport times are measured rather than assumed.")
    parser.add_argument("--max-frames", type=int, default=3000, help="Watchdog for --until-settled.")
    parser.add_argument("--settle-frames", type=int, default=40)
    parser.add_argument("--out", default="runs/d_sweep")
    parser.add_argument("--save-replays", action="store_true", help="Keep replay.npz for every seed.")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    failures: Counter[str] = Counter()
    for seed in seeds:
        env = SimulationEnvironment(load_yaml(args.config), seed=seed)
        started = time.perf_counter()
        if args.until_settled:
            termination = env.run_until_settled(max_frames=args.max_frames, settle_frames=args.settle_frames)
        else:
            env.run(args.frames)
            termination = {"frames_run": args.frames, "terminated_by": "budget", "settled": None}
        wall = time.perf_counter() - started
        frames_run = termination["frames_run"]
        summary = env.summary()
        if args.save_replays:
            env.save_outputs(out / f"seed{seed}")

        entry = next(iter(summary["cargoes"].values()))
        g500 = entry["g500"]
        metrics = dict(g500["metrics"])
        metrics.update(
            seed=seed,
            success=g500["success"],
            reasons=g500["failure_reasons"],
            frames_run=frames_run,
            terminated_by=termination["terminated_by"],
            frames_per_second=frames_run / wall if wall > 0 else float("inf"),
            barrier_scalings=summary["solver"]["barrier_scalings"],
            min_barrier_scale=summary["solver"]["min_barrier_scale"],
            margin_relaxations=summary["solver"]["margin_relaxations"],
            solves=summary["solver"]["solves"],
        )
        rows.append(metrics)
        for reason in g500["failure_reasons"]:
            failures[classify(reason)] += 1

        print(f"seed {seed:2d}  {'PASS' if g500['success'] else 'FAIL'}  "
              f"goal {metrics['goal_angle_deg']:6.1f} deg  L={metrics['target_distance']:.3f}  "
              f"J={metrics['J']:.4f}  eff={metrics['efficiency']:.3f}  "
              f"cross={metrics['max_cross_track']:.4f}  "
              f"transport@{metrics['transport_frame']}  hold@{metrics['hold_frame']}  "
              f"end@{frames_run}({termination['terminated_by'][:4]})  "
              f"{metrics['frames_per_second']:.1f} fps")

    passed = sum(1 for row in rows if row["success"])
    low, high = wilson(passed, len(rows))

    def stats(field: str) -> dict:
        values = [float(row[field]) for row in rows if row.get(field) is not None]
        if not values:
            return {}
        array = np.asarray(values)
        return {
            "n": len(values),
            "mean": float(array.mean()),
            "sd": float(array.std()),
            "min": float(array.min()),
            "max": float(array.max()),
        }

    report = {
        "config": args.config,
        "frames": args.frames,
        "until_settled": bool(args.until_settled),
        "max_frames": args.max_frames,
        "seeds": seeds,
        "g500_pass": passed,
        "g500_total": len(rows),
        "g500_rate": passed / len(rows) if rows else 0.0,
        "g500_wilson_95": [low, high],
        # Every seed is in here, including the failures. A mean over the survivors
        # is a different quantity from a mean over the experiment.
        "distributions": {
            field: stats(field)
            for field in (
                "J", "target_distance", "efficiency", "max_cross_track", "direction_error_deg",
                "rotation_deg", "max_strict_coverage", "min_inter_agent_distance",
                "min_signed_clearance", "max_penetration", "first_detection_frame",
                "contact_ready_frame", "transport_frame", "hold_frame",
                "progress_estimate_ratio", "barrier_scalings", "frames_per_second",
                "frames_run", "margin_relaxations",
            )
        },
        "gate_failure_counts": dict(sorted(failures.items(), key=lambda kv: -kv[1])),
        "runs": rows,
    }
    (out / "g500_sweep.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print()
    print(f"G500 {passed}/{len(rows)}  (Wilson 95%: {low:.2f}-{high:.2f})")
    print("gate failures, most common first:")
    for gate, count in report["gate_failure_counts"].items():
        print(f"  {count:3d}  {gate}")
    print(f"wrote {out / 'g500_sweep.json'}")


if __name__ == "__main__":
    main()
