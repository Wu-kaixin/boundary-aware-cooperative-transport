#!/usr/bin/env python3
"""Run the same 500-frame full-workspace controller over diverse simple shapes.

This is an empirical regression matrix, not the theorem.  Every instance first
receives the same independent admissibility certificate; ineligible and failed
runs remain in the report and make the command exit non-zero.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402
from validate_run import validate  # noqa: E402


DEFAULT_CONFIG = "configs/sim/v3/arbitrary_shape_full_workspace_500.yaml"


SHAPES: dict[str, dict] = {
    "circle": {"shape": "circle", "radius": 0.55},
    "rectangle": {"shape": "rectangle", "width": 1.15, "height": 0.72},
    "l_shape": {"shape": "l_shape", "scale": 0.78},
    "nonconvex": {"shape": "nonconvex", "scale": 0.72},
    "star": {
        "shape": "polygon",
        "vertices_frame": "local",
        "vertices": [
            [0.68, 0.00], [0.28, 0.20], [0.21, 0.65], [-0.12, 0.34],
            [-0.55, 0.40], [-0.36, 0.00], [-0.55, -0.40], [-0.12, -0.34],
            [0.21, -0.65], [0.28, -0.20],
        ],
    },
    "u_shape": {
        "shape": "polygon",
        "vertices_frame": "local",
        "vertices": [
            [-0.600, -0.480], [0.600, -0.480], [0.600, 0.480], [0.325, 0.480],
            [0.325, -0.080], [-0.325, -0.080], [-0.325, 0.480], [-0.600, 0.480],
        ],
    },
    "random": {"shape": "random_simple_polygon"},
}


def scenario_for(base: dict, name: str) -> dict:
    if name not in SHAPES:
        raise ValueError(f"unknown shape {name!r}; choose from {sorted(SHAPES)}")
    cfg = copy.deepcopy(base)
    common = cfg["cargoes"][0]
    selected = copy.deepcopy(SHAPES[name])
    item = {
        "id": common.get("id", "cargo_0"),
        "center": common.get("center", [4.0, 4.0]),
        "surface_density": common.get("surface_density", 1.5),
        "random_center": copy.deepcopy(common.get("random_center", {})),
        **selected,
    }
    if selected.get("shape") == "random_simple_polygon":
        for key in ("vertex_count", "radius_min", "radius_max", "angle_jitter"):
            item[key] = common[key]
    cfg["cargoes"] = [item]
    cfg["_source"] = f"{base.get('_source', DEFAULT_CONFIG)}::{name}"
    return cfg


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--shapes", default=",".join(SHAPES))
    parser.add_argument("--seed-base", type=int, default=20)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--out", default="runs/full_workspace_shape_matrix")
    args = parser.parse_args()

    names = [name.strip() for name in args.shapes.split(",") if name.strip()]
    base = load_yaml(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    records: list[dict] = []
    for index, name in enumerate(names):
        seed = args.seed_base + index
        started = time.perf_counter()
        env = SimulationEnvironment(scenario_for(base, name), seed=seed)
        env.run(args.steps)
        run_dir = out / f"{name}_seed{seed}"
        summary = env.save_outputs(run_dir)
        reasons = validate(summary)
        cargo = next(iter(summary["cargoes"].values()))
        record = {
            "shape": name,
            "seed": seed,
            "valid": not reasons,
            "reasons": reasons,
            "wall_seconds": time.perf_counter() - started,
            "J": cargo.get("J"),
            "coverage": cargo.get("final_strict_coverage"),
            "rotation_deg": cargo.get("rotation_deg"),
            "phase_frames": cargo.get("phase_frames"),
            "guarantee_eligible": (cargo.get("guarantee_certificate") or {}).get("eligible"),
        }
        records.append(record)
        print(
            f"[{'PASS' if not reasons else 'REJECT'}] {name:10s} seed={seed} "
            f"J={cargo.get('J', float('nan')):+.4f} coverage={cargo.get('final_strict_coverage', 0.0):.3f}",
            flush=True,
        )

    report = {
        "config": args.config,
        "steps": args.steps,
        "runs": len(records),
        "valid": sum(int(record["valid"]) for record in records),
        "rejected": sum(int(not record["valid"]) for record in records),
        "records": records,
    }
    (out / "shape_matrix_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"report: {out / 'shape_matrix_report.json'}")
    return 0 if report["rejected"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
