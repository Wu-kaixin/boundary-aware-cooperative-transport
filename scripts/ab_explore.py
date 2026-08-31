#!/usr/bin/env python
"""D10 - A/B for the unobserved-boundary exploration term.

    PYTHONPATH=src python scripts/ab_explore.py --seeds 0..7 --gains 0,6 --out runs/d10_ab

Baseline is ``explore_gain = 0``, which leaves the density bit-identical to the
committed controller, so this is an ablation of one term rather than a comparison
of two controllers. Every other parameter, the scenario, and the seeds are shared.

The candidate is kept only if it satisfies all of:

* no loss of T1 safety   -- min inter-agent distance no worse, no robot inside the cargo
* watchdog timeouts do not increase
* far-side discovery is significantly faster
* contact-ready is clearly earlier
* coverage improves

Anything less and it is reverted, and the numbers are recorded either way.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from diagnose_redeployment import SeedTrace, parse_seeds, stats  # noqa: E402

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

# The inter-agent barrier is exactly binding by design -- the ring sits on d_min
# for most of a transport run -- so a float comparison reports the last bit of the
# QP's arithmetic as a collision. This tolerance is sized against the arithmetic,
# not chosen: an earlier version of this branch reported four "safety failures"
# whose measured deficits were 1e-16 to 3e-8 m.
D_MIN_TOLERANCE = 1e-6


def run_arm(config: dict, seed: int, gain, max_frames: int, settle_frames: int,
            param: str = "explore_gain") -> dict:
    cfg = json.loads(json.dumps(config))  # deep copy; the config is plain data
    cfg["controller"][param] = gain
    env = SimulationEnvironment(cfg, seed=seed)
    env.controller.trace_enabled = True
    trace = SeedTrace(env)
    started = time.perf_counter()
    termination = env.run_until_settled(
        max_frames=max_frames, settle_frames=settle_frames, on_frame=trace.on_frame
    )
    wall = time.perf_counter() - started
    phases = env.controller.phase_monitor.as_dict()
    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    solver = summary["solver"]
    d_min = float(env.controller.params.d_min)
    observed_min = min(env.log.min_distances)
    peak_union = max((r.union_map_coverage for r in trace.records), default=0.0)
    final_union = trace.records[-1].union_map_coverage if trace.records else 0.0
    detect = trace.first_detection
    backside = trace.backside_first
    return {
        "seed": seed,
        "param": param,
        "value": gain,
        "T_detect": detect,
        "T_backside_discovery": (backside - detect) if (backside is not None and detect is not None) else None,
        "T_contact_ready": phases.get("contact_ready_frame"),
        "T_transport": phases.get("transport_frame"),
        "T_hold": phases.get("hold_frame"),
        "frames_run": termination["frames_run"],
        "terminated_by": termination["terminated_by"],
        "watchdog": termination["terminated_by"] == "watchdog",
        "peak_strict_coverage": entry["max_strict_coverage"],
        "final_strict_coverage": entry["final_strict_coverage"],
        "peak_union_map_coverage": peak_union,
        "final_union_map_coverage": final_union,
        "min_inter_agent": observed_min,
        "d_min": d_min,
        "d_min_breach": bool(observed_min < d_min - D_MIN_TOLERANCE),
        "agents_inside": int(max(env.log.agents_inside[trace.object_id])),
        "min_signed_clearance": float(min(env.log.min_clearance[trace.object_id])),
        "fallbacks": solver["fallbacks"],
        "infeasible": solver.get("infeasible", 0),
        "barrier_scalings": solver["barrier_scalings"],
        "margin_relaxations": solver["margin_relaxations"],
        "solves": solver["solves"],
        "g500": entry["g500"]["success"],
        "wall": wall,
        "fps": termination["frames_run"] / wall if wall else 0.0,
    }


COMPARED = (
    ("T_detect", "lower"),
    ("T_backside_discovery", "lower"),
    ("T_contact_ready", "lower"),
    ("T_transport", "lower"),
    ("T_hold", "lower"),
    ("peak_strict_coverage", "higher"),
    ("final_strict_coverage", "higher"),
    ("peak_union_map_coverage", "higher"),
    ("min_inter_agent", "higher"),
    ("fps", "higher"),
)


def table(arms: dict[float, list[dict]]) -> str:
    gains = sorted(arms)
    lines = ["| quantity | " + " | ".join(f"gain {g:g}" for g in gains) + " |",
             "| --- |" + " --- |" * len(gains)]
    for name, _ in COMPARED:
        cells = []
        for g in gains:
            s = stats([r[name] for r in arms[g]])
            cells.append("-" if s["mean"] is None else f"{s['mean']:.3g} ± {s['sd']:.3g}")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    for name, reduce in (("watchdog", sum), ("d_min_breach", sum), ("agents_inside", max),
                         ("fallbacks", sum), ("infeasible", sum), ("barrier_scalings", sum),
                         ("g500", sum)):
        cells = [str(reduce(r[name] for r in arms[g])) for g in gains]
        lines.append(f"| {name} (total) | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _safety_checks(baseline: list[dict], candidate: list[dict]) -> dict:
    """The gates every candidate has to clear, whatever it was trying to improve."""
    return {
        "safety not reduced": (
            sum(r["d_min_breach"] for r in candidate) <= sum(r["d_min_breach"] for r in baseline)
            and max(r["agents_inside"] for r in candidate) <= max(r["agents_inside"] for r in baseline)
            and min(r["min_signed_clearance"] for r in candidate) >= 0.0
        ),
        "watchdog not increased": (
            sum(r["watchdog"] for r in candidate) <= sum(r["watchdog"] for r in baseline)
        ),
        "solver not degraded": (
            sum(r["fallbacks"] for r in candidate) <= sum(r["fallbacks"] for r in baseline)
            and sum(r["infeasible"] for r in candidate) <= sum(r["infeasible"] for r in baseline)
        ),
    }


def verdict(baseline: list[dict], candidate: list[dict], criteria: str = "explore") -> dict:
    """The keep/revert decision, stated before the numbers were in.

    The safety gates are common. What differs is what the change was *for*, and a
    candidate is not allowed to be scored against a target it was not aimed at:
    the exploration term is judged on far-side discovery and coverage, the uniform
    enclosure ring on contact-ready and on not losing the transport it feeds.
    """
    def mean(rows, key):
        return stats([r[key] for r in rows])["mean"]

    checks = _safety_checks(baseline, candidate)
    if criteria == "explore":
        checks["far-side discovery faster"] = _better(baseline, candidate, "T_backside_discovery", lower=True)
        checks["contact-ready earlier"] = _better(baseline, candidate, "T_contact_ready", lower=True)
        checks["coverage improved"] = _better(baseline, candidate, "peak_strict_coverage", lower=False)
    elif criteria == "enclose":
        checks["contact-ready earlier"] = _better(baseline, candidate, "T_contact_ready", lower=True)
        # The lead offset exists so the leading arc does not resist the press.
        # Removing it during ENCLOSE must not cost the transport that follows:
        # every seed that reached HOLD before must still reach it, and the
        # enclosure it certifies must not be worse.
        checks["transport completion not degraded"] = (
            sum(1 for r in candidate if r["T_hold"] is not None)
            >= sum(1 for r in baseline if r["T_hold"] is not None)
        )
        checks["coverage not degraded"] = (
            (mean(candidate, "peak_strict_coverage") or 0.0)
            >= (mean(baseline, "peak_strict_coverage") or 0.0) - 0.02
        )
        checks["scaled barrier not worse"] = (
            sum(r["barrier_scalings"] for r in candidate)
            <= sum(r["barrier_scalings"] for r in baseline)
        )
    else:
        raise ValueError(f"unknown criteria set {criteria!r}")
    return {
        "criteria": criteria,
        "checks": checks,
        "keep": all(checks.values()),
        "baseline_contact_ready": mean(baseline, "T_contact_ready"),
        "candidate_contact_ready": mean(candidate, "T_contact_ready"),
    }


def _better(baseline: list[dict], candidate: list[dict], key: str, lower: bool) -> bool:
    """Paired comparison on the seeds where both arms produced the quantity.

    Paired because the seeds are the same episodes: comparing two means over eight
    draws of a quantity whose spread is as large as its mean says almost nothing,
    and the per-seed difference says a great deal.
    """
    pairs = [
        (b[key], c[key]) for b, c in zip(baseline, candidate)
        if b[key] is not None and c[key] is not None
    ]
    if not pairs:
        return False
    delta = np.asarray([c - b for b, c in pairs], dtype=float)
    improved = (delta < 0).sum() if lower else (delta > 0).sum()
    mean_delta = float(delta.mean())
    return improved >= len(delta) / 2 and (mean_delta < 0 if lower else mean_delta > 0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_search.yaml")
    parser.add_argument("--seeds", default="0..7")
    parser.add_argument("--param", default="explore_gain",
                        help="Controller parameter to sweep. The first value is the baseline.")
    parser.add_argument("--gains", default="0,6",
                        help="Comma-separated values for --param. The first is the committed baseline.")
    parser.add_argument("--criteria", default="explore", choices=("explore", "enclose"),
                        help="Which pre-stated keep criteria to apply.")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--settle-frames", type=int, default=40)
    parser.add_argument("--out", default="runs/d10_ab")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = parse_seeds(args.seeds)
    gains = [float(g) for g in args.gains.split(",")]
    config = load_yaml(args.config)

    arms: dict[float, list[dict]] = {g: [] for g in gains}
    for gain in gains:
        for seed in seeds:
            row = run_arm(config, seed, gain, args.max_frames, args.settle_frames, args.param)
            arms[gain].append(row)
            print(f"{args.param}={gain:g}  seed {seed:2d}  detect@{row['T_detect']}  "
                  f"far-side+{row['T_backside_discovery']}  CR@{row['T_contact_ready']}  "
                  f"cov {row['peak_strict_coverage']:.3f}  union {row['peak_union_map_coverage']:.3f}  "
                  f"d_min {row['min_inter_agent']:.4f}  end@{row['frames_run']}"
                  f"({row['terminated_by'][:4]})  {row['fps']:.1f} fps")

    report = table(arms)
    print("\n" + report)

    payload = {"config": args.config, "seeds": seeds, "param": args.param, "gains": gains,
               "criteria": args.criteria, "arms": {str(g): arms[g] for g in gains}}
    if len(gains) == 2:
        payload["verdict"] = verdict(arms[gains[0]], arms[gains[1]], args.criteria)
        print("\nverdict:")
        for name, ok in payload["verdict"]["checks"].items():
            print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        print(f"  => {'KEEP' if payload['verdict']['keep'] else 'REVERT'}")
    (out / "ab.json").write_text(json.dumps(payload, indent=2, default=float), encoding="utf-8")
    (out / "ab.md").write_text(report + "\n", encoding="utf-8")
    print(f"\nwritten to {out}")


if __name__ == "__main__":
    main()
