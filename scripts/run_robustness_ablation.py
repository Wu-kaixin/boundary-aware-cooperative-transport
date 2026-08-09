#!/usr/bin/env python3
"""Run actual sensing/update/communication perturbations with truth audits."""

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


VARIANTS = {
    "nominal": {},
    "range_noise_005": {"range_noise_std": 0.005},
    "range_noise_010": {"range_noise_std": 0.010},
    "slow_updates_5": {"perception_every": 5, "planning_every": 5},
    "comm_dropout_10": {"communication_dropout_prob": 0.10},
    "combined": {
        "range_noise_std": 0.005,
        "perception_every": 4,
        "planning_every": 4,
        "communication_dropout_prob": 0.10,
    },
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/sim/research/adaptive_progress_closed_loop.yaml",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--output", default="runs/robustness_ablation_stage2")
    parser.add_argument("--variants", nargs="*", choices=sorted(VARIANTS), default=list(VARIANTS))
    args = parser.parse_args()

    base = load_yaml(args.config)
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    results: list[dict] = []
    for name in args.variants:
        config = copy.deepcopy(base)
        config.setdefault("controller", {}).update(VARIANTS[name])
        config.setdefault("evaluation", {})["online_truth_audit"] = True
        env = SimulationEnvironment(config, seed=args.seed)
        started = time.perf_counter()
        termination = env.run_until(args.max_steps)
        wall = time.perf_counter() - started
        summary = env.save_outputs(root / name)
        cargo_success = all(item.get("success") is True for item in summary["cargoes"].values())
        result = {
            "variant": name,
            "overrides": VARIANTS[name],
            "termination": termination.status,
            "frame": termination.frame,
            "success": bool(termination.success and cargo_success),
            "wall_seconds": wall,
            "fps": termination.frame / max(wall, 1e-12),
            "solver": summary["solver"],
            "communication": summary["communication"],
            "measured_error_audit": summary["measured_error_audit"],
            "cargoes": {
                cargo_id: {
                    key: entry.get(key)
                    for key in (
                        "J",
                        "efficiency",
                        "rotation_deg",
                        "max_penetration",
                        "final_strict_coverage",
                        "success",
                        "failure_reasons",
                    )
                }
                for cargo_id, entry in summary["cargoes"].items()
            },
        }
        results.append(result)
        print(
            json.dumps(
                {
                    "variant": name,
                    "termination": termination.status,
                    "frame": termination.frame,
                    "success": result["success"],
                    "relaxations": summary["solver"]["margin_relaxations"],
                    "fallbacks": summary["solver"]["fallbacks"],
                    "fps": result["fps"],
                }
            ),
            flush=True,
        )
    payload = {
        "config": args.config,
        "seed": args.seed,
        "max_steps_timeout": args.max_steps,
        "variants": results,
    }
    (root / "robustness_ablation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0 if all(item["success"] for item in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
