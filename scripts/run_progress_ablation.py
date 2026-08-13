#!/usr/bin/env python3
"""Ablate fixed feed-forward, PI pressure, release, and wrench allocation."""

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
    "fixed_feedforward": {
        "progress_feedback": False,
        "transport_progress_estimator": "centroid",
        "transport_speed": 0.20,
        "wrench_allocation": False,
        "contact_release_enabled": False,
    },
    "pi_only": {
        "progress_feedback": True,
        "wrench_allocation": False,
        "contact_release_enabled": False,
    },
    "pi_release": {
        "progress_feedback": True,
        "wrench_allocation": False,
        "contact_release_enabled": True,
    },
    "pi_wrench_release": {
        "progress_feedback": True,
        "wrench_allocation": True,
        "contact_release_enabled": True,
    },
}


def parse_seeds(text: str) -> list[int]:
    values: list[int] = []
    for chunk in text.split(","):
        if ".." in chunk:
            lo, hi = (int(value) for value in chunk.split("..", 1))
            values.extend(range(lo, hi + 1))
        elif chunk.strip():
            values.append(int(chunk))
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sim/research/adaptive_progress_closed_loop.yaml")
    parser.add_argument("--variants", default=",".join(VARIANTS))
    parser.add_argument("--seeds", default="0")
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--out", default="runs/progress_ablation")
    args = parser.parse_args()

    base = load_yaml(args.config)
    names = [name.strip() for name in args.variants.split(",") if name.strip()]
    unknown = sorted(set(names) - set(VARIANTS))
    if unknown:
        raise ValueError(f"unknown variants {unknown}; choose from {sorted(VARIANTS)}")
    seeds = parse_seeds(args.seeds)
    max_steps = int(args.max_steps or (base.get("episode", {}) or {}).get("max_steps", 1500))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    records: list[dict] = []
    for seed in seeds:
        for name in names:
            cfg = copy.deepcopy(base)
            cfg["controller"].update(VARIANTS[name])
            started = time.perf_counter()
            env = SimulationEnvironment(cfg, seed=seed)
            termination = env.run_until(max_steps)
            wall = time.perf_counter() - started
            run_dir = out / f"{name}_seed{seed}"
            summary = env.save_outputs(run_dir)
            cargo = next(iter(summary["cargoes"].values()))
            record = {
                "variant": name,
                "seed": seed,
                "termination": termination.status,
                "executed_steps": termination.frame,
                "success": bool(termination.success and cargo.get("success") is True),
                "wall_seconds": wall,
                "control_fps": termination.frame / max(wall, 1e-12),
                "J": cargo.get("J"),
                "efficiency": cargo.get("efficiency"),
                "rotation_deg": cargo.get("rotation_deg"),
                "coverage": cargo.get("final_strict_coverage"),
                "phase_frames": cargo.get("phase_frames"),
                "failure_reasons": cargo.get("failure_reasons", []),
                "solver": summary["solver"],
            }
            records.append(record)
            print(
                f"[{termination.status}] {name:20s} seed={seed} steps={termination.frame} "
                f"J={cargo.get('J', float('nan')):+.4f} eff={cargo.get('efficiency', float('nan')):+.3f}",
                flush=True,
            )

    report = {
        "config": args.config,
        "max_steps_timeout": max_steps,
        "records": records,
        "variant_summary": {
            name: {
                "runs": sum(record["variant"] == name for record in records),
                "successes": sum(record["variant"] == name and record["success"] for record in records),
            }
            for name in names
        },
    }
    report_path = out / "progress_ablation_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"report: {report_path}")
    reference = [record for record in records if record["variant"] == "pi_wrench_release"]
    return 0 if reference and all(record["success"] for record in reference) else 2


if __name__ == "__main__":
    raise SystemExit(main())
