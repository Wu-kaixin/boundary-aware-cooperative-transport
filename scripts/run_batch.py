#!/usr/bin/env python
"""Gate 4 batch runner: multiple seeds over a config directory.

Only runs that pass ``scripts/validate_run.py`` enter the statistics, and the
number of *rejected* runs is reported alongside. A high rejection rate is itself
a finding -- reporting the mean of the survivors without it would be selection on
the outcome.

    python scripts/run_batch.py --configs configs/sim/v2 --seeds 0..19 --out runs/main_exp
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402
from validate_run import validate  # noqa: E402


def parse_seeds(spec: str) -> list[int]:
    seeds: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ".." in chunk:
            lo, hi = chunk.split("..")
            seeds.extend(range(int(lo), int(hi) + 1))
        else:
            seeds.append(int(chunk))
    return seeds


def summarise(values: list[float]) -> dict:
    if not values:
        return {"n": 0}
    arr = np.asarray(values, dtype=float)
    return {
        "n": int(len(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
        "median": float(np.median(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a seed sweep over a config directory.")
    parser.add_argument("--configs", required=True, help="Directory of scenario YAML files, or a single file.")
    parser.add_argument("--seeds", default="0..9", help="Seed spec, e.g. '0..19' or '0,1,5'.")
    parser.add_argument("--steps", type=int, default=1400)
    parser.add_argument("--out", required=True, help="Output directory.")
    args = parser.parse_args()

    config_path = Path(args.configs)
    configs = sorted(config_path.glob("*.yaml")) if config_path.is_dir() else [config_path]
    seeds = parse_seeds(args.seeds)
    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)

    records: list[dict] = []
    for config_file in configs:
        cfg = load_yaml(config_file)
        for seed in seeds:
            started = time.time()
            run_dir = out_root / f"{config_file.stem}_seed{seed}"
            env = SimulationEnvironment(cfg, seed=seed)
            env.run(args.steps)
            summary = env.save_outputs(run_dir)
            reasons = validate(summary)
            certificates = [
                entry.get("guarantee_certificate")
                for entry in summary["cargoes"].values()
                if entry.get("guarantee_certificate") is not None
            ]
            guarantee_eligible = (
                all(cert.get("runtime_eligible") is True for cert in certificates)
                if certificates
                else None
            )
            record = {
                "config": config_file.name,
                "seed": seed,
                "run_dir": str(run_dir),
                "valid": not reasons,
                "guarantee_eligible": guarantee_eligible,
                "reasons": reasons,
                "wall_seconds": time.time() - started,
                "cargoes": summary["cargoes"],
            }
            records.append(record)
            flag = "PASS" if record["valid"] else "REJECT"
            first = summary["cargoes"][next(iter(summary["cargoes"]))]
            print(
                f"[{flag}] {config_file.name} seed={seed}  J={first.get('J', float('nan')):+.4f}  "
                f"strict_cov={first['final_strict_coverage']:.3f}  "
                f"inside={first['max_agents_inside']}  ({record['wall_seconds']:.0f}s)",
                flush=True,
            )

    per_config: dict[str, dict] = {}
    for config_file in configs:
        subset = [r for r in records if r["config"] == config_file.name]
        valid = [r for r in subset if r["valid"]]
        eligible = [r for r in subset if r.get("guarantee_eligible") is True]
        valid_eligible = [r for r in eligible if r["valid"]]
        metrics: dict[str, list[float]] = {}
        for record in valid:
            for entry in record["cargoes"].values():
                for key in ("J", "displacement", "progress_efficiency", "final_strict_coverage",
                            "rotation_deg", "min_signed_clearance", "max_penetration", "mean_contacts"):
                    value = entry.get(key)
                    if isinstance(value, (int, float)):
                        metrics.setdefault(key, []).append(float(value))
        per_config[config_file.name] = {
            "runs": len(subset),
            "valid": len(valid),
            "rejected": len(subset) - len(valid),
            "rejection_rate": (len(subset) - len(valid)) / len(subset) if subset else None,
            "guarantee_eligible": len(eligible),
            "valid_of_eligible": len(valid_eligible),
            "eligible_success_rate": len(valid_eligible) / len(eligible) if eligible else None,
            "metrics": {k: summarise(v) for k, v in metrics.items()},
            "rejection_reasons": sorted({reason for r in subset for reason in r["reasons"]}),
        }

    report = {"steps": args.steps, "seeds": seeds, "per_config": per_config, "runs": records}
    (out_root / "batch_report.json").write_text(json.dumps(report, indent=2, default=float), encoding="utf-8")

    print("\n=== batch summary ===")
    for name, stats in per_config.items():
        print(f"{name}: {stats['valid']}/{stats['runs']} valid, {stats['rejected']} rejected "
              f"({(stats['rejection_rate'] or 0):.1%})")
        if stats["guarantee_eligible"]:
            print(
                f"    conditional domain: {stats['valid_of_eligible']}/{stats['guarantee_eligible']} valid "
                f"({stats['eligible_success_rate']:.1%})"
            )
        for key, s in stats["metrics"].items():
            if s.get("n"):
                print(f"    {key:24s} mean={s['mean']:+.4f}  std={s['std']:.4f}  n={s['n']}")
        if stats["valid"] < 10:
            print("    NOTE: fewer than 10 valid seeds -- no stability or robustness claim is supported")
    print(f"\nreport written to {out_root / 'batch_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
