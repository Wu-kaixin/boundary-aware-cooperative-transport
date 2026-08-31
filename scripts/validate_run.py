#!/usr/bin/env python
"""Fail-closed validation of a finished run.

Takes ``summary.json`` and prints PASS or the list of reasons it is not a valid
run. Default is failure: a criterion that cannot be evaluated because a field is
missing rejects the run rather than passing it, because an unattributable number
cannot be defended.

    python scripts/validate_run.py runs/l_shape_v2_seed0/summary.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REQUIRED_PROVENANCE = ("git_sha", "config_hash", "seed", "backend")


def validate(summary: dict) -> list[str]:
    reasons: list[str] = []

    provenance = summary.get("provenance")
    if not isinstance(provenance, dict):
        return ["missing 'provenance' block; run cannot be attributed"]
    for field in REQUIRED_PROVENANCE:
        if provenance.get(field) in (None, "", "unknown"):
            reasons.append(f"provenance.{field} is missing or unknown")

    backend = provenance.get("backend")
    if backend == "projection":
        reasons.append(
            "backend='projection': the safety input came from an inexact iterative filter, "
            "so no hard-QP claim is supported by this run"
        )
    elif backend not in ("qp", "cvxpy"):
        reasons.append(f"backend={backend!r} is not a recognised backend")

    engine = summary.get("engine")
    if engine is None:
        reasons.append("missing 'engine'; which dynamics moved the cargo is unrecorded")
    elif engine == "scripted":
        reasons.append(
            "engine='scripted': the cargo was translated along a configured direction, "
            "so this run says nothing about transport"
        )

    solver = summary.get("solver")
    if not isinstance(solver, dict):
        reasons.append("missing 'solver' block; solver provenance is unrecorded")
    else:
        if solver.get("fallbacks", 1) != 0:
            reasons.append(f"{solver.get('fallbacks')} solver fallback(s) occurred; C2 requires zero")
        if solver.get("infeasible", 1) != 0:
            reasons.append(f"{solver.get('infeasible')} QP infeasibility event(s); the constraint set was unsatisfiable")
        if solver.get("max_slack", 1.0) not in (0, 0.0):
            reasons.append(f"non-zero slack {solver.get('max_slack')}; the filter was not a hard QP")

    contracts = summary.get("contracts") or {}
    d_min = contracts.get("d_min")
    delta_max = contracts.get("delta_max")
    # Stated discretisation allowance on the barrier, not a fudge factor: the
    # continuous-time condition can be overshot by one step of relative motion.
    overshoot = contracts.get("discrete_overshoot") or 0.0
    c1 = contracts.get("C1")
    if c1 is None and summary.get("task_mode") != "coverage":
        reasons.append("missing C1 contract record")
    elif isinstance(c1, dict):
        r_safe, d_c, r_robot = c1.get("r_safe"), c1.get("cage_offset"), c1.get("robot_radius")
        if None in (r_safe, d_c, r_robot):
            reasons.append("C1 record incomplete")
        elif not (r_safe < d_c < r_robot):
            reasons.append(f"C1 violated in the recorded contract: r_safe={r_safe} < d_c={d_c} < r_robot={r_robot} is false")
        if (c1.get("barrier_margin") or -1.0) <= 0.0:
            reasons.append(f"C1 barrier margin {c1.get('barrier_margin')} <= 0")

    min_distance = summary.get("min_inter_agent_distance")
    if min_distance is None:
        reasons.append("missing 'min_inter_agent_distance'")
    elif d_min is not None and min_distance < d_min:
        reasons.append(f"inter-robot safety violated: min distance {min_distance:.4f} < d_min {d_min:.4f}")

    cargoes = summary.get("cargoes")
    if not cargoes:
        reasons.append("no cargo results recorded")
        return reasons

    for cargo_id, entry in cargoes.items():
        prefix = f"cargo {cargo_id}"
        clearance = entry.get("min_signed_clearance")
        if clearance is None:
            reasons.append(f"{prefix}: min_signed_clearance not recorded")
        elif clearance < 0.0:
            reasons.append(f"{prefix}: min signed clearance {clearance:.4f} < 0 (a robot entered the cargo)")
        penetration = entry.get("max_penetration")
        if penetration is None:
            reasons.append(f"{prefix}: max_penetration not recorded")
        elif delta_max is not None and penetration > delta_max + overshoot:
            reasons.append(
                f"{prefix}: max penetration {penetration:.4f} > delta_max {delta_max:.4f} "
                f"+ discrete overshoot {overshoot:.4f}"
            )
        if entry.get("success") is not True:
            for reason in entry.get("failure_reasons") or ["success flag absent"]:
                reasons.append(f"{prefix}: {reason}")

    return reasons


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate one or more run summaries, fail-closed.")
    parser.add_argument("summary", nargs="+", help="Path(s) to summary.json, or directories containing them.")
    parser.add_argument("--quiet", action="store_true", help="Print only the final tally.")
    args = parser.parse_args()

    paths: list[Path] = []
    for item in args.summary:
        p = Path(item)
        if p.is_dir():
            paths.extend(sorted(p.rglob("summary.json")))
        else:
            paths.append(p)

    passed = 0
    rejected = 0
    for path in paths:
        if not path.exists():
            rejected += 1
            print(f"[FAIL] {path}: file does not exist")
            continue
        try:
            summary = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            rejected += 1
            print(f"[FAIL] {path}: unreadable ({exc})")
            continue
        reasons = validate(summary)
        if reasons:
            rejected += 1
            print(f"[FAIL] {path}")
            if not args.quiet:
                for reason in reasons:
                    print(f"         - {reason}")
        else:
            passed += 1
            print(f"[PASS] {path}")

    total = passed + rejected
    print(f"\n{passed}/{total} runs valid, {rejected} rejected")
    if rejected and total:
        print(f"rejection rate {rejected / total:.1%} -- a high rejection rate is itself a result and must be reported")
    return 0 if rejected == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
