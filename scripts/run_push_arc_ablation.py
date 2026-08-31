#!/usr/bin/env python
"""T3 - is the direction error limited by authority, or by arc dispersion?

    python scripts/run_push_arc_ablation.py --seeds 0..11 --out docs/results/t3

`scripts/analyse_lateral_authority.py` measured the reachable normal cone over the
twelve baseline seeds and found the opposite of the expected result:

    corr(direction error, reachable cone half-width)      = +0.909
    corr(direction error, goal-outside-cone fraction)     = -0.777
    reachable half-width, mean                            = 27.4 deg
    direction the 0.15 m cross-track gate demands, mean   =  5.94 deg

So the aim is *not* limited by the available authority -- the cone is 4.6 times wider
than the gate needs -- and the seeds whose goal direction most often lies outside the
reachable cone are the ones that aim *best*. The reading that fits is that the cone half
width is a measure of how *dispersed* the push arc is, and a dispersed arc aims badly:
the net force is a nonnegative combination of press directions spanning up to +-47
degrees, so the sensitivity of the resulting direction to an allocation error scales with
the span.

That is a correlation over twelve seeds, which is not a mechanism. This script runs the
controlled version. ``push_side_threshold`` is the membership test
``n_k . d_goal <= -threshold``; raising it admits only robots whose own normal opposes the
goal more directly, which narrows the arc's angular span without touching any gain.

The prediction, if dispersion is the cause:

* direction error and cross-track fall as the threshold rises;
* the push set shrinks, so at some threshold the quorum fails and transport stops
  arming at all -- which is the failure mode polygon32 seed 2 already exhibits at the
  default threshold, with 8 robots meeting the alignment test and 3 pushing.

If instead the authority reading were right, raising the threshold would make the aim
*worse*, because it narrows the reachable cone.

Nothing here is a proposed default. The point is to find out which of the two readings
survives, and both outcomes are reportable.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"
THRESHOLDS = (0.35, 0.55, 0.75)


def parse_seeds(spec: str) -> list[int]:
    if ".." in spec:
        low, high = (int(p) for p in spec.split("..", 1))
        return list(range(low, high + 1))
    return [int(p) for p in spec.split(",") if p.strip()]


def describe(values: list[float]) -> dict:
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


def run_arm(threshold: float, base: dict, seeds: list[int], max_frames: int) -> dict:
    config = copy.deepcopy(base)
    config["controller"] = dict(config.get("controller", {}))
    config["controller"]["push_side_threshold"] = float(threshold)

    rows = []
    for seed in seeds:
        env = SimulationEnvironment(config, seed=seed)
        termination = env.run_until_settled(max_frames=max_frames)
        summary = env.summary()
        entry = next(iter(summary["cargoes"].values()))
        g500 = entry["g500"]
        m = g500["metrics"]
        rows.append(
            {
                "seed": seed,
                "success": bool(g500["success"]),
                "J": float(m["J"]),
                "efficiency": float(m["efficiency"]),
                "direction_error_deg": m["direction_error_deg"],
                "max_cross_track": float(m["max_cross_track"]),
                "rotation_deg": float(m["rotation_deg"]),
                "min_inter_agent_distance": float(m["min_inter_agent_distance"]),
                "d_min": float(m["d_min"]),
                "transport_frame": m["transport_frame"],
                "hold_frame": m["hold_frame"],
                "final_phase": m["final_phase"],
                "transport_armed": m["transport_frame"] is not None,
                "barrier_scalings": int(summary["solver"]["barrier_scalings"]),
                "solver_fallbacks": int(summary["solver"]["fallbacks"]),
                "solver_infeasible": int(summary["solver"]["infeasible"]),
                "frames_run": int(termination["frames_run"]),
                "terminated_by": str(termination["terminated_by"]),
            }
        )
        print(
            f"[tau={threshold:.2f}] seed {seed:2d}  "
            f"{'PASS' if rows[-1]['success'] else 'FAIL'}  "
            f"J={rows[-1]['J']:.3f}  dir={rows[-1]['direction_error_deg']:5.2f}deg  "
            f"cross={rows[-1]['max_cross_track']:.4f}  "
            f"armed={'y' if rows[-1]['transport_armed'] else 'N'}  "
            f"scal={rows[-1]['barrier_scalings']:3d}",
            flush=True,
        )

    fields = ["J", "efficiency", "direction_error_deg", "max_cross_track", "rotation_deg",
              "barrier_scalings", "frames_run"]
    return {
        "push_side_threshold": threshold,
        "membership_half_cone_deg": math.degrees(math.acos(threshold)),
        "pass": sum(1 for r in rows if r["success"]),
        "total": len(rows),
        "transport_armed": sum(1 for r in rows if r["transport_armed"]),
        "over_cross_track_gate": sum(1 for r in rows if r["max_cross_track"] > 0.15),
        "barrier_scalings_total": sum(r["barrier_scalings"] for r in rows),
        "solver_fallbacks_total": sum(r["solver_fallbacks"] for r in rows),
        "solver_infeasible_total": sum(r["solver_infeasible"] for r in rows),
        "separation_held": all(r["min_inter_agent_distance"] >= r["d_min"] - 1e-6 for r in rows),
        "distributions": {f: describe([r.get(f) for r in rows]) for f in fields},
        "runs": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--thresholds", default=",".join(str(t) for t in THRESHOLDS))
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="docs/results/t3")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    base = load_yaml(args.config)
    thresholds = [float(t) for t in args.thresholds.split(",") if t.strip()]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    arms = []
    for threshold in thresholds:
        arms.append(run_arm(threshold, base, seeds, args.max_frames))
        print(flush=True)

    direction = [a["distributions"]["direction_error_deg"]["mean"] for a in arms]
    cross = [a["distributions"]["max_cross_track"]["mean"] for a in arms]
    dispersion_reading = direction[-1] < direction[0] and cross[-1] < cross[0]
    authority_reading = direction[-1] > direction[0]

    report = {
        "experiment": "T3 push-arc dispersion vs lateral authority",
        "config": args.config,
        "seeds": seeds,
        "thresholds": thresholds,
        "arms": arms,
        "verdict": {
            "dispersion_reading_supported": bool(dispersion_reading),
            "authority_reading_supported": bool(authority_reading),
            "direction_error_by_threshold": dict(zip(map(str, thresholds), direction)),
            "cross_track_by_threshold": dict(zip(map(str, thresholds), cross)),
            "transport_armed_by_threshold": {
                str(t): a["transport_armed"] for t, a in zip(thresholds, arms)
            },
        },
    }
    (out / "push_arc_ablation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("=" * 84)
    print(f"{'tau':>5} {'halfcone':>9} {'pass':>6} {'armed':>6} {'dir_err':>8} "
          f"{'cross':>7} {'>gate':>6} {'J':>7} {'scal':>5} {'fb':>4}")
    for a in arms:
        d = a["distributions"]
        print(
            f"{a['push_side_threshold']:>5.2f} {a['membership_half_cone_deg']:>8.1f}d "
            f"{a['pass']:>3}/{a['total']:<2} {a['transport_armed']:>4}/{a['total']:<2} "
            f"{d['direction_error_deg']['mean']:>8.2f} {d['max_cross_track']['mean']:>7.4f} "
            f"{a['over_cross_track_gate']:>4}/{a['total']:<2} {d['J']['mean']:>7.3f} "
            f"{a['barrier_scalings_total']:>5} {a['solver_fallbacks_total']:>4}"
        )
    print("=" * 84)
    print(f"dispersion reading supported: {dispersion_reading}")
    print(f"authority reading supported:  {authority_reading}")
    print(f"wrote {out / 'push_arc_ablation.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
