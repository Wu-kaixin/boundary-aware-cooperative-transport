#!/usr/bin/env python
"""What correcting the ~24% progress-estimate gain error would buy, and what it costs.

    python scripts/run_progress_gain_ablation.py --seeds 0..11 --out docs/results/progress

The distance sweep measured ``J/L`` at 1.23-1.28 for every alpha from 0.2 to 1.0 with
``corr(alpha, J/L) = +0.029``: the team travels about 24% further than asked, and the bias
does not vary with the distance. A distance-independent bias is a **gain** error on the
on-board estimate, and ``LocalBoundaryMap._commit_motion`` already names the mechanism --
registration moves the map rigidly, fusion pulls each cell toward the latest return, and the
fused share is motion the estimate never sees as a shift. That docstring records the
integrated estimate reading 79% of the true displacement at a fusion cap of 4, and
``1 / 0.79 = 1.266``.

So the arms are chosen from the mechanism rather than fitted to the outcome:

``1.00``   the estimate as it is -- v1, and an exact no-op
``1.266``  ``1 / 0.79``, the fusion-absorbed share from _commit_motion's own measurement
``1.24``   the mean ``J/L`` measured across the whole distance sweep

The two corrections are within 2% of each other, which is the point: the number derived from
the fusion model and the number measured from 60 episodes agree, so this is a mechanism with
a measurement behind it rather than a constant tuned until the gate passed.

What to read
------------
``J/L`` moving towards 1.0 is the whole objective. Watch three other columns for the cost:
the contract pass rate cannot be the headline (the gate is cross-track-dominated and most
episodes fail it for reasons unrelated to distance), while **separation, fallbacks and
barrier scalings are gate-independent** and are where a change of this kind does damage if it
does any. Stopping earlier also means less time in TRANSPORT, so ``frames_run`` should fall.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"
GAINS = (1.00, 1.24, 1.266)


def parse_seeds(spec: str) -> list[int]:
    if ".." in spec:
        low, high = (int(p) for p in spec.split("..", 1))
        return list(range(low, high + 1))
    return [int(p) for p in spec.split(",") if p.strip()]


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
        "max": float(a.max()),
    }


FIELDS = ["J", "J_over_target", "efficiency", "max_cross_track", "direction_error_deg",
          "max_strict_coverage", "barrier_scalings", "frames_run", "overshoot_m"]


def run_arm(gain: float, base: dict, seeds: list[int], max_frames: int) -> dict:
    config = copy.deepcopy(base)
    config["controller"] = dict(config.get("controller", {}))
    config["controller"]["progress_estimate_gain"] = float(gain)

    rows = []
    for seed in seeds:
        env = SimulationEnvironment(config, seed=seed)
        termination = env.run_until_settled(max_frames=max_frames)
        summary = env.summary()
        entry = next(iter(summary["cargoes"].values()))
        g500 = entry["g500"]
        m = g500["metrics"]
        j = float(m["J"])
        target = float(m["target_distance"])
        rows.append({
            "seed": seed,
            "success": bool(g500["success"]),
            "J": j,
            "target_distance": target,
            "J_over_target": j / target if target > 1e-9 else None,
            "overshoot_m": j - target,
            "efficiency": float(m["efficiency"]),
            "max_cross_track": float(m["max_cross_track"]),
            "direction_error_deg": m["direction_error_deg"],
            "max_strict_coverage": float(m["max_strict_coverage"]),
            "min_inter_agent_distance": float(m["min_inter_agent_distance"]),
            "d_min": float(m["d_min"]),
            "max_penetration": float(m["max_penetration"]),
            "penetration_budget": float(m["penetration_budget"]),
            "barrier_scalings": int(summary["solver"]["barrier_scalings"]),
            "solver_fallbacks": int(summary["solver"]["fallbacks"]),
            "solver_infeasible": int(summary["solver"]["infeasible"]),
            "frames_run": int(termination["frames_run"]),
            "terminated_by": str(termination["terminated_by"]),
        })
        print(f"[gain={gain:.3f}] seed {seed:2d}  {'PASS' if rows[-1]['success'] else 'FAIL'}  "
              f"J={j:.3f}  L={target:.3f}  J/L={rows[-1]['J_over_target']:.3f}  "
              f"over={rows[-1]['overshoot_m']:+.3f}m  "
              f"cross={rows[-1]['max_cross_track']:.4f}  "
              f"scal={rows[-1]['barrier_scalings']:3d}  fr={rows[-1]['frames_run']}", flush=True)

    return {
        "progress_estimate_gain": gain,
        "pass": sum(1 for r in rows if r["success"]),
        "total": len(rows),
        "barrier_scalings_total": sum(r["barrier_scalings"] for r in rows),
        "solver_fallbacks_total": sum(r["solver_fallbacks"] for r in rows),
        "solver_infeasible_total": sum(r["solver_infeasible"] for r in rows),
        "separation_held": all(r["min_inter_agent_distance"] >= r["d_min"] - 1e-6 for r in rows),
        "penetration_within_budget": all(
            r["max_penetration"] <= r["penetration_budget"] + 1e-6 for r in rows),
        "distributions": {f: describe([r.get(f) for r in rows]) for f in FIELDS},
        "runs": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--gains", default=",".join(str(g) for g in GAINS))
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="docs/results/progress")
    args = parser.parse_args()

    base = load_yaml(args.config)
    seeds = parse_seeds(args.seeds)
    gains = [float(g) for g in args.gains.split(",") if g.strip()]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    arms = [run_arm(g, base, seeds, args.max_frames) for g in gains]
    for _ in arms:
        print(flush=True)

    report = {
        "experiment": "progress-estimate gain calibration",
        "config": args.config,
        "seeds": seeds,
        "gains": gains,
        "rationale": {
            "measured_J_over_L_distance_sweep": 1.24,
            "fusion_model_1_over_0_79": 1.2658,
            "note": (
                "The two corrections agree to within 2%: one is derived from the fusion "
                "share _commit_motion already measured, the other from 60 episodes of the "
                "distance sweep. Neither was tuned until a gate passed."
            ),
        },
        "arms": arms,
    }
    (out / "progress_gain.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("=" * 96)
    print(f"{'gain':>6} {'pass':>7} {'J/L':>7} {'overshoot':>10} {'cross':>8} {'peakcov':>8} "
          f"{'scal':>5} {'fb':>4} {'frames':>7} {'sep':>4}")
    for a in arms:
        d = a["distributions"]
        print(f"{a['progress_estimate_gain']:>6.3f} {a['pass']:>3}/{a['total']:<3} "
              f"{d['J_over_target']['mean']:>7.3f} {d['overshoot_m']['mean']:>+10.4f} "
              f"{d['max_cross_track']['mean']:>8.4f} {d['max_strict_coverage']['mean']:>8.3f} "
              f"{a['barrier_scalings_total']:>5} {a['solver_fallbacks_total']:>4} "
              f"{d['frames_run']['mean']:>7.0f} {'ok' if a['separation_held'] else 'BAD':>4}")
    print("=" * 96)
    print(f"wrote {out / 'progress_gain.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
