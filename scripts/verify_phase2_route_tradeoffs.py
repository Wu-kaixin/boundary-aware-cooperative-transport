"""Verify fixed-state route diagnostics without control replay or physics."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from research_io import ROOT, guard, read, sha
from threadpoolctl import threadpool_limits
from verify_post_support_onsets import certificate

from dbact.safety.qp2d import solve_min_norm_2d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    folder, output = args.analysis_dir.resolve(), args.output.resolve()
    if not output.is_relative_to(ROOT / "runs") or output.exists():
        raise ValueError("Verification needs a new receipt under runs/")
    guard()
    started = time.perf_counter()
    data, outcome = read(folder / "analysis.json"), read(folder / "outcome.json")
    assert outcome["status"] == "COMPLETED_FIXED_STATE_DIAGNOSTIC"
    assert sha(folder / "analysis.json") == outcome["analysis_sha256"]
    for name, expected in data["inputs"].items():
        assert sha(ROOT / name) == expected
        assert sha(folder / "inputs_and_sources" / Path(name).name) == expected
    checks, costs, errors, minima = [], [], [], []
    with threadpool_limits(limits=1):
        for row in data["rows"]:
            s = row["input_system"]
            A, b, g, mask = s["A"], s["b"], s["goal"], s["object_mask"]
            speed, nominal = s["speed"], np.asarray(s["nominal"])
            checks.append(certificate(A, b, g, speed, row["baseline"]))
            augmented_A, augmented_b = A + [g], b + [0.0]
            nr = row["nonreverse"]
            checks.append(
                certificate(
                    augmented_A, augmented_b, g, speed, nr["feasibility_certificate"]
                )
            )
            # Analytic planar enumeration is separate from the CLARABEL projection.
            exact = solve_min_norm_2d(
                nominal, np.asarray(augmented_A), np.asarray(augmented_b), speed
            )
            assert exact.feasible == (nr["status"] == "VERIFIED_FEASIBLE")
            if exact.feasible:
                u = nr["u"]
                residuals = [
                    math.fsum(a * x for a, x in zip(n, u)) - r
                    for n, r in zip(augmented_A, augmented_b)
                ]
                assert min(residuals) >= -1e-7 and math.hypot(*u) <= speed + 1e-7
                cost = math.fsum((x - y) ** 2 for x, y in zip(u, nominal))
                other_cost = math.fsum((x - y) ** 2 for x, y in zip(exact.u, nominal))
                assert abs(cost - nr["objective"]) < 1e-12
                assert abs(cost - other_cost) < 1e-7
                costs.append(abs(cost - other_cost))
            rel = row["minimum_object_rhs_change"]
            assert rel["status"] == "VERIFIED_FEASIBLE"
            u, epsilon = rel["u"], rel["epsilon"]
            weights, mu = rel["dual_multipliers"], rel["nonreverse_multiplier"]
            assert epsilon >= 0.0 and mu >= 0.0 and all(w >= 0.0 for w in weights)
            assert (
                math.fsum(w for w, is_object in zip(weights, mask) if is_object)
                <= 1.0 + 1e-12
            )
            residuals = [
                math.fsum(a * x for a, x in zip(n, u)) + is_object * epsilon - r
                for n, is_object, r in zip(A, mask, b)
            ]
            assert min(residuals, default=0.0) >= -1e-7
            assert (
                math.hypot(*u) <= speed + 1e-7
                and math.fsum(a * x for a, x in zip(g, u)) >= -1e-7
            )
            c = [
                math.fsum(n[j] * w for n, w in zip(A, weights)) + mu * g[j]
                for j in range(2)
            ]
            lower = math.fsum(r * w for r, w in zip(b, weights)) - speed * math.hypot(
                *c
            )
            assert abs(lower - rel["dual_lower_bound"]) < 1e-12
            assert -1e-7 <= epsilon - lower <= 1e-6
            errors.append(epsilon - lower)
            minima.append(min(residuals, default=0.0))
            zero = row["zero_motion_demand"]
            if zero is not None:
                assert all(
                    x == y
                    for x, y, is_object in zip(b, zero["b"], mask)
                    if not is_object
                )
                assert all(x >= y - 1e-12 for x, y in zip(b, zero["b"]))
                checks.append(certificate(A, zero["b"], g, speed, zero["capability"]))
    rows = data["rows"]
    assert len(rows) == 64 and sum(r["selected_pusher"] for r in rows) == 4
    assert len({(r["condition"], r["frame"], r["agent_id"]) for r in rows}) == 64
    for label, subset in [
        ("all", rows),
        ("selected_pushers", [r for r in rows if r["selected_pusher"]]),
        ("support_applied", [r for r in rows if r["support_applied"]]),
    ]:
        expected = data["groups"][label]
        assert len(subset) == expected["states"]
        assert (
            dict(Counter(r["baseline"]["classification"] for r in subset))
            == expected["baseline"]
        )
        assert (
            dict(Counter(r["nonreverse"]["status"] for r in subset))
            == expected["nonreverse"]
        )
        assert (
            dict(Counter(r["minimum_object_rhs_change"]["status"] for r in subset))
            == expected["object_relaxation"]
        )
        assert (
            dict(
                Counter(
                    r["zero_motion_demand"]["capability"]["classification"]
                    for r in subset
                    if r["zero_motion_demand"] is not None
                )
            )
            == expected["zero_motion_demand"]
        )
    example = read(folder / "cap_claim_counterexample.json")
    checks.append(
        certificate(
            example["normals"],
            example["capped_rhs"],
            [1.0, 0.0],
            example["max_speed"],
            example["certificate"],
        )
    )
    result = {
        "status": "VERIFIED_FIXED_STATE_TRADEOFFS",
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "verifier_sha256": sha(__file__),
        "scalar_certificate_helper_sha256": sha(
            ROOT / "scripts/verify_post_support_onsets.py"
        ),
        "planar_solver_sha256": sha(ROOT / "src/dbact/safety/qp2d.py"),
        "analysis_sha256": sha(folder / "analysis.json"),
        "states": len(rows),
        "scalar_direction_certificates": len(checks),
        "nonreverse_projection_comparisons": len(costs),
        "object_rhs_primal_dual_checks": len(errors),
        "maximum_projection_objective_difference": max(costs),
        "maximum_object_rhs_primal_dual_gap": max(errors),
        "minimum_object_rhs_primal_residual": min(minima),
        "wall_seconds": time.perf_counter() - started,
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
        "scope": "Independent numerical paths, not an independent dataset or a formal interval proof",
    }
    with output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
