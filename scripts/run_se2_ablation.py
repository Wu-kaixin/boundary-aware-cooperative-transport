#!/usr/bin/env python
"""T2 - the SE(2) boundary-point velocity, measured against v1 on the same 12 seeds.

    python scripts/run_se2_ablation.py --seeds 0..11 --out docs/results/se2

Two arms over the baseline configuration, differing in exactly one controller field:

``off``  ``estimate_object_yaw: false``  -- v1. The registration keeps two unknowns,
         no stored normal is rotated, and every barrier row is built from the single
         translational velocity estimate.
``on``   ``estimate_object_yaw: true``   -- the yaw rate is estimated as a third
         unknown about the map's own centroid, and each row carries the velocity of
         *its own* boundary point, ``v_c + omega R90 (b_k - c)``.

Both arms run with the six-term error audit enabled, which is the point of the
exercise as much as the gate is: the audit turns
``guarantee.bounded_errors.normal_error_deg`` and ``velocity_error`` from declared
premises into measured numbers.

A caveat that has to be read with the result
--------------------------------------------
The baseline cargo barely rotates. Measured over the 12 seeds, ``rotation_deg`` has a
mean of -0.06 and a maximum of 0.086 -- less than a tenth of a degree. The SE(2) term
exists to capture the boundary-point velocity that rotation induces, so on this
scenario there is almost nothing for it to capture, and the acceptance gate
consequently tests that the change does no *harm* rather than that it does good. That
is a real limit on what this arm can establish, and it is why the gate below is a
regression gate and the shape matrix is where the term would have to earn its keep.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"

#: The audit needs premises to check. These are the values declared in
#: configs/sim/v2/shape_matrix.yaml, carried over unchanged so that the number this
#: script measures is comparable with the premise the decisive matrix was run under.
DECLARED_ERRORS = {"normal_error_deg": 30.0, "velocity_error": 0.02}

ARMS = {
    "off": {"estimate_object_yaw": False},
    "on": {"estimate_object_yaw": True},
}


def parse_seeds(spec: str) -> list[int]:
    values: list[int] = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if ".." in token:
            low, high = (int(part) for part in token.split("..", 1))
            step = 1 if high >= low else -1
            values.extend(range(low, high + step, step))
        else:
            values.append(int(token))
    return sorted(set(values))


def arm_config(base: dict, overrides: dict) -> dict:
    config = copy.deepcopy(base)
    config["controller"] = dict(config.get("controller", {}))
    config["controller"].update(overrides)
    # Enable the audit without touching the certificate: this block is read by
    # SimulationEnvironment for the error premises and by nothing in the control path.
    guarantee = dict(config.get("guarantee", {}) or {})
    guarantee["bounded_errors"] = dict(DECLARED_ERRORS)
    config["guarantee"] = guarantee
    config["audit_errors"] = True
    return config


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


def run_arm(name: str, config: dict, seeds: list[int], max_frames: int) -> dict:
    rows: list[dict] = []
    for seed in seeds:
        env = SimulationEnvironment(config, seed=seed)
        started = time.perf_counter()
        termination = env.run_until_settled(max_frames=max_frames)
        wall = time.perf_counter() - started
        summary = env.summary()
        entry = next(iter(summary["cargoes"].values()))
        g500 = entry["g500"]
        m = g500["metrics"]
        audit = summary["error_audit"] or {}
        terms = audit.get("terms", {})

        rows.append(
            {
                "seed": seed,
                "success": bool(g500["success"]),
                "failure_reasons": list(g500["failure_reasons"]),
                "J": float(m["J"]),
                "efficiency": float(m["efficiency"]),
                "max_cross_track": float(m["max_cross_track"]),
                "direction_error_deg": m["direction_error_deg"],
                "rotation_deg": float(m["rotation_deg"]),
                "min_inter_agent_distance": float(m["min_inter_agent_distance"]),
                "d_min": float(m["d_min"]),
                "min_signed_clearance": float(m["min_signed_clearance"]),
                "max_penetration": float(m["max_penetration"]),
                "barrier_scalings": int(summary["solver"]["barrier_scalings"]),
                "margin_relaxations": int(summary["solver"]["margin_relaxations"]),
                "solver_fallbacks": int(summary["solver"]["fallbacks"]),
                "solver_infeasible": int(summary["solver"]["infeasible"]),
                "frames_run": int(termination["frames_run"]),
                "fps": float(termination["frames_run"]) / max(wall, 1e-12),
                "within_declared_bounds": audit.get("within_declared_bounds"),
                "fail_closed_reasons": audit.get("fail_closed_reasons", []),
                "measured_normal_error_deg": audit.get("measured_bounds", {}).get("normal_error_deg"),
                "measured_velocity_error": audit.get("measured_bounds", {}).get("velocity_error"),
                "normal_breach_fraction": audit.get("breach_fractions", {}).get("normal_error_deg"),
                "velocity_breach_fraction": audit.get("breach_fractions", {}).get("velocity_error"),
                "normal_p999": audit.get("p999_not_a_bound", {}).get("normal_error_deg"),
                "velocity_p999": audit.get("p999_not_a_bound", {}).get("velocity_error"),
                # Every statistic of every term, flattened. Built from the keys the
                # audit actually produced rather than from a hand-written list, so a
                # renamed statistic shows up as a missing column instead of a KeyError
                # in the report three hundred episodes later.
                **{
                    f"{term}_{stat}": value
                    for term, stats in terms.items()
                    for stat, value in (stats or {}).items()
                },
            }
        )
        print(
            f"[{name}] seed {seed:2d}  {'PASS' if rows[-1]['success'] else 'FAIL'}  "
            f"J={rows[-1]['J']:.4f}  eff={rows[-1]['efficiency']:.3f}  "
            f"cross={rows[-1]['max_cross_track']:.4f}  "
            f"yaw={rows[-1]['rotation_deg']:+.3f}deg  "
            f"scal={rows[-1]['barrier_scalings']:3d}  "
            f"bounds={rows[-1]['within_declared_bounds']}  "
            f"{rows[-1]['fps']:.1f} fps",
            flush=True,
        )

    fields = [
        "J", "efficiency", "max_cross_track", "direction_error_deg", "rotation_deg",
        "min_inter_agent_distance", "min_signed_clearance", "max_penetration",
        "barrier_scalings", "frames_run", "fps",
        "measured_normal_error_deg", "measured_velocity_error",
        "normal_breach_fraction", "velocity_breach_fraction", "normal_p999", "velocity_p999",
        "normal_error_deg_mean", "normal_error_deg_p95", "normal_error_deg_p99",
        "normal_projection_error_mps_mean", "normal_projection_error_mps_p95",
        "normal_error_deg_max", "boundary_point_error_m_max", "map_gap_m_max",
        "object_velocity_error_mps_max", "point_velocity_error_mps_max",
        "normal_projection_error_mps_max",
    ]
    return {
        "arm": name,
        "overrides": ARMS[name],
        "seeds": seeds,
        "pass": sum(1 for r in rows if r["success"]),
        "total": len(rows),
        "barrier_scalings_total": sum(r["barrier_scalings"] for r in rows),
        "margin_relaxations_total": sum(r["margin_relaxations"] for r in rows),
        "solver_fallbacks_total": sum(r["solver_fallbacks"] for r in rows),
        "solver_infeasible_total": sum(r["solver_infeasible"] for r in rows),
        "within_declared_bounds_all": all(
            r["within_declared_bounds"] is True for r in rows
        ),
        "fail_closed_seeds": [r["seed"] for r in rows if r["within_declared_bounds"] is False],
        "distributions": {f: describe([r.get(f) for r in rows]) for f in fields},
        "runs": rows,
    }


def gate(off: dict, on: dict) -> dict:
    """The T2 acceptance gate, stated on cross-track rather than on J.

    The decisive matrix showed the binding constraint at high alpha is the normalised
    cross-track error, not transport authority -- enclosure timeouts were identically
    zero -- so the gate is written on ``max_cross_track`` and J only has to not regress.
    """
    d_off, d_on = off["distributions"], on["distributions"]
    checks = {
        "cross_track_not_worse": d_on["max_cross_track"]["mean"]
        <= d_off["max_cross_track"]["mean"] + 1e-9,
        "J_mean_at_least_1_40": d_on["J"]["mean"] >= 1.40,
        "efficiency_at_least_0_98": d_on["efficiency"]["mean"] >= 0.98,
        "zero_fallbacks": on["solver_fallbacks_total"] == 0,
        "zero_infeasible": on["solver_infeasible_total"] == 0,
        "barrier_scalings_not_worse": on["barrier_scalings_total"]
        <= off["barrier_scalings_total"],
        "inter_agent_separation_held": all(
            r["min_inter_agent_distance"] >= r["d_min"] - 1e-6 for r in on["runs"]
        ),
    }
    return {
        "checks": checks,
        "passed": all(checks.values()),
        "failed": [name for name, ok in checks.items() if not ok],
        "reference": {
            "off_cross_track_mean": d_off["max_cross_track"]["mean"],
            "off_J_mean": d_off["J"]["mean"],
            "off_barrier_scalings_total": off["barrier_scalings_total"],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="docs/results/se2")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    base = load_yaml(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    arms = {}
    for name, overrides in ARMS.items():
        arms[name] = run_arm(name, arm_config(base, overrides), seeds, args.max_frames)
        print(flush=True)

    verdict = gate(arms["off"], arms["on"])
    report = {
        "experiment": "T2 SE(2) boundary-point velocity in the object-boundary CBF",
        "base_config": args.config,
        "declared_errors": DECLARED_ERRORS,
        "seeds": seeds,
        "arms": arms,
        "gate": verdict,
        "caveat": (
            "The baseline cargo rotates by less than 0.1 degree over the whole episode, "
            "so this arm can only establish that the SE(2) term does no harm. It cannot "
            "establish that it helps; the shape matrix is where rotation is present."
        ),
    }
    (out / "se2_ablation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("=" * 78)
    for name in ("off", "on"):
        a = arms[name]
        # ``describe`` returns {"n": 0} with no other keys when a field was never
        # populated, so the report reads through a default rather than raising after the
        # episodes have already been paid for.
        d = {k: (v if "mean" in v else dict(v, mean=float("nan"), max=float("nan")))
             for k, v in a["distributions"].items()}
        print(
            f"{name:3s}  {a['pass']}/{a['total']}  J={d['J']['mean']:.4f}+-{d['J']['sd']:.4f}  "
            f"eff={d['efficiency']['mean']:.4f}  cross={d['max_cross_track']['mean']:.4f}  "
            f"scal={a['barrier_scalings_total']:3d}  fb={a['solver_fallbacks_total']}  "
            f"inf={a['solver_infeasible_total']}"
        )
        print(
            f"     normal_error_deg   max={d['measured_normal_error_deg']['max']:7.3f}  "
            f"p99={d['normal_error_deg_p99']['mean']:7.3f}  p999={d['normal_p999']['mean']:7.3f}  "
            f"mean={d['normal_error_deg_mean']['mean']:7.3f}  "
            f"breach={d['normal_breach_fraction']['mean']:.5f}  (premise {DECLARED_ERRORS['normal_error_deg']})"
        )
        print(
            f"     velocity_error     max={d['measured_velocity_error']['max']:7.4f}  "
            f"p95={d['normal_projection_error_mps_p95']['mean']:7.4f}  "
            f"p999={d['velocity_p999']['mean']:7.4f}  "
            f"mean={d['normal_projection_error_mps_mean']['mean']:7.4f}  "
            f"breach={d['velocity_breach_fraction']['mean']:.5f}  (premise {DECLARED_ERRORS['velocity_error']})"
        )
        print(
            f"     within declared bounds: {a['within_declared_bounds_all']}"
            + (f"  breached on seeds {a['fail_closed_seeds']}" if a["fail_closed_seeds"] else "")
        )
    print("=" * 78)
    print(f"GATE {'PASS' if verdict['passed'] else 'FAIL'}")
    for name, ok in verdict["checks"].items():
        print(f"  {'ok  ' if ok else 'FAIL'}  {name}")
    print(f"wrote {out / 'se2_ablation.json'}")
    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
