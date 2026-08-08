"""Multi-seed paper experiment matrix runner.

Examples:
  conda run -n dbact python scripts/run_paper_matrix.py --quick
  conda run -n dbact python scripts/run_paper_matrix.py --configs configs/sim/paper/b3_dbact.yaml --seeds 3 --steps 300
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.metrics import summarize_seeds
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml

DEFAULT_CONFIGS = [
    "configs/sim/paper/b0_arm.yaml",
    "configs/sim/paper/b1_oracle.yaml",
    "configs/sim/paper/b2_no_cbf.yaml",
    "configs/sim/paper/b3_dbact.yaml",
    "configs/sim/paper/ablation_dropout30.yaml",
    "configs/sim/paper/ablation_normal15.yaml",
    "configs/sim/paper/pymunk_push.yaml",
    "configs/sim/paper/pymunk_l_shape_transport.yaml",
]

QUICK_CONFIGS = [
    "configs/sim/paper/b3_dbact.yaml",
    "configs/sim/paper/b0_arm.yaml",
    "configs/sim/paper/pymunk_push.yaml",
    "configs/sim/paper/pymunk_l_shape_transport.yaml",
]


def _scalarize(metrics: dict) -> dict:
    """Flatten nested metrics for seed aggregation."""
    row: dict = {
        "method": metrics.get("method"),
        "transport_backend": metrics.get("transport_backend"),
        "final_time": metrics.get("final_time"),
        "mean_path_length": metrics.get("mean_path_length"),
        "min_inter_agent_distance": metrics.get("min_inter_agent_distance"),
        "R_CBF": metrics.get("R_CBF"),
        "T_solve": metrics.get("T_solve"),
        "P_success": metrics.get("P_success"),
        "cbf_calls": metrics.get("cbf_calls"),
    }
    final_coverage = metrics.get("final_coverage") or {}
    displacement = metrics.get("cargo_displacement") or {}
    t_enc = metrics.get("T_enclosure") or {}
    recruited = metrics.get("recruited_agents") or {}
    d_min_obs = metrics.get("d_min_obs") or {}
    if final_coverage:
        key = next(iter(final_coverage))
        row["final_coverage"] = final_coverage[key]
        row["cargo_displacement"] = displacement.get(key, 0.0)
        row["T_enclosure"] = t_enc.get(key)
        row["recruited_agents"] = recruited.get(key, 0)
        row["d_min_obs"] = d_min_obs.get(key)
        succ = (metrics.get("success") or {}).get(key, False)
        row["success"] = 1.0 if succ else 0.0
    return row


def run_one(config_path: Path, seed: int, steps: int, output_root: Path) -> dict:
    cfg = load_yaml(config_path)
    cfg = copy.deepcopy(cfg)
    cfg["seed"] = seed
    cfg.setdefault("agents", {})["seed"] = seed
    env = SimulationEnvironment(cfg)
    env.run(steps=steps)
    out_dir = output_root / config_path.stem / f"seed_{seed:03d}"
    env.save_outputs(out_dir)
    metrics = env.compute_metrics()
    row = _scalarize(metrics)
    row["config"] = config_path.name
    row["seed"] = seed
    row["output_dir"] = str(out_dir)
    return row


def main() -> None:
    parser = argparse.ArgumentParser(description="Run DBACT paper experiment matrix.")
    parser.add_argument("--configs", nargs="*", default=None, help="YAML configs to run")
    parser.add_argument("--quick", action="store_true", help="Small subset for smoke validation")
    parser.add_argument("--seeds", type=int, default=3, help="Number of random seeds")
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--output", type=str, default="runs/paper_matrix")
    args = parser.parse_args()

    if args.configs:
        config_paths = [Path(p) for p in args.configs]
    elif args.quick:
        config_paths = [Path(p) for p in QUICK_CONFIGS]
        args.seeds = min(args.seeds, 2)
        args.steps = min(args.steps, 120)
    else:
        config_paths = [Path(p) for p in DEFAULT_CONFIGS]

    output_root = Path(args.output)
    output_root.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict] = []
    summaries: dict[str, dict] = {}

    for config_path in config_paths:
        rows: list[dict] = []
        print(f"=== {config_path} ===")
        for offset in range(args.seeds):
            seed = args.seed_start + offset
            print(f"  seed={seed} steps={args.steps}")
            row = run_one(config_path, seed, args.steps, output_root)
            rows.append(row)
            all_rows.append(row)
            cov = row.get("final_coverage")
            print(f"    coverage={cov} success={row.get('success')} R_CBF={row.get('R_CBF'):.3f}")
        summaries[config_path.name] = summarize_seeds(rows)
        (output_root / f"summary_{config_path.stem}.json").write_text(
            json.dumps(summaries[config_path.name], indent=2),
            encoding="utf-8",
        )

    (output_root / "all_runs.json").write_text(json.dumps(all_rows, indent=2), encoding="utf-8")
    (output_root / "summaries.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")

    if all_rows:
        fieldnames = sorted({key for row in all_rows for key in row})
        with (output_root / "all_runs.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_rows)

    manifest = {
        "python": sys.version,
        "platform": platform.platform(),
        "configs": [str(path) for path in config_paths],
        "seed_start": args.seed_start,
        "seeds": args.seeds,
        "steps": args.steps,
        "grid_contract": "fixed physical spacing from controller.grid_spacing",
    }
    (output_root / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )

    # Optional CSV if pandas is available.
    try:
        import pandas as pd

        frame = pd.DataFrame(all_rows)
        frame.to_parquet(output_root / "all_runs.parquet", index=False)
    except Exception:
        pass

    print(f"Wrote matrix outputs to {output_root}")


if __name__ == "__main__":
    main()
