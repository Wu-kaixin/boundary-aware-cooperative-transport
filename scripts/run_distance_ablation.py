#!/usr/bin/env python3
"""Run closed-loop transport at 0.10, 0.25 and 0.50 metre task distances."""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/sim/research/adaptive_progress_closed_loop.yaml",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--distances", nargs="+", type=float, default=[0.10, 0.25, 0.50])
    parser.add_argument(
        "--target-reserve",
        type=float,
        default=0.001,
        help=(
            "explicit controller reserve beyond the activation-relative task length; "
            "the default 1 mm covers integration/estimation discretisation without changing j_min"
        ),
    )
    parser.add_argument("--position-tolerance", type=float, default=0.025)
    parser.add_argument("--max-steps", type=int, default=2500)
    parser.add_argument("--output", default="runs/distance_ablation_stage3")
    parser.add_argument("--truth-audit", action="store_true")
    args = parser.parse_args()

    base = load_yaml(args.config)
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    results = []
    for distance in args.distances:
        config = copy.deepcopy(base)
        config["task"]["random_goal"]["target_distance"] = float(distance)
        config["controller"]["transport_distance"] = float(distance + args.target_reserve)
        config["evaluation"]["j_min"] = max(0.0, float(distance - args.position_tolerance))
        config["evaluation"]["j_max"] = float(distance + max(0.15, 0.3 * distance))
        config["evaluation"]["online_truth_audit"] = bool(args.truth_audit)
        config["evaluation"]["require_measured_error_bounds"] = bool(args.truth_audit)
        env = SimulationEnvironment(config, seed=args.seed)
        started = time.perf_counter()
        termination = env.run_until(args.max_steps)
        wall = time.perf_counter() - started
        label = f"distance_{distance:.2f}m".replace(".", "p")
        summary = env.save_outputs(root / label)
        cargo_success = all(entry.get("success") is True for entry in summary["cargoes"].values())
        result = {
            "distance_m": float(distance),
            "controller_target_m": float(distance + args.target_reserve),
            "termination": termination.status,
            "frame": termination.frame,
            "success": bool(termination.success and cargo_success),
            "wall_seconds": wall,
            "fps": termination.frame / max(wall, 1e-12),
            "solver": summary["solver"],
            "min_inter_agent_distance": summary["min_inter_agent_distance"],
            "cargoes": {
                cargo_id: {
                    **{
                        key: entry.get(key)
                        for key in (
                            "J",
                            "episode_total_J",
                            "efficiency",
                            "final_cross_track_error",
                            "max_cross_track_error",
                            "rotation_deg",
                            "max_abs_rotation_deg",
                            "max_penetration",
                            "final_strict_coverage",
                            "min_strict_coverage_during_transport",
                            "max_uncovered_arc_during_transport_m",
                            "operational_enclosure_maintained_during_transport",
                            "final_operational_enclosure",
                            "min_contact_count_during_transport",
                            "peak_net_force",
                            "peak_abs_net_torque",
                            "success",
                            "failure_reasons",
                            "phase_frames",
                        )
                    },
                    "progress_estimate": (
                        summary.get("transport_progress_estimates", {})
                        .get(cargo_id, {})
                        .get("mean")
                    ),
                    "progress_estimation_error": (
                        summary.get("transport_progress_estimates", {})
                        .get(cargo_id, {})
                        .get("mean", 0.0)
                        - float(entry.get("J", 0.0))
                    ),
                }
                for cargo_id, entry in summary["cargoes"].items()
            },
        }
        results.append(result)
        print(
            json.dumps(
                {
                    "distance_m": distance,
                    "termination": termination.status,
                    "frame": termination.frame,
                    "success": result["success"],
                    "relaxations": summary["solver"]["margin_relaxations"],
                    "fps": result["fps"],
                }
            ),
            flush=True,
        )
    payload = {
        "config": args.config,
        "seed": args.seed,
        "max_steps_timeout": args.max_steps,
        "truth_audit": bool(args.truth_audit),
        "results": results,
    }
    (root / "distance_ablation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0 if all(item["success"] for item in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
