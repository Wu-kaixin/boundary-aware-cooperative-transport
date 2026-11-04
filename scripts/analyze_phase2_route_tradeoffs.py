"""Fixed-state Phase 2 route diagnostics, never a deployed control policy."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import time
import traceback
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cvxpy as cp
import numpy as np
import post_acquisition_support_study as study
from probe_post_support_contact_direction import maximize_direction
from research_io import ROOT, guard, read, sha
from threadpoolctl import threadpool_limits

from dbact.safety.filter import SafetyFilter, SafetyFilterParams

INPUT = ROOT / "runs/research/2026-11-04-phase1-first-pusher-loss-v3"
DESIGN = ROOT / "experiments/ccta2027/phase2_route_evaluation_v1_design.md"
TOL = 1e-7


def solve(problem):
    problem.solve(
        solver=cp.CLARABEL, tol_gap_abs=1e-10, tol_feas=1e-10, tol_gap_rel=1e-10
    )


def arrays(A, b, goal):
    A, b, goal = (
        np.asarray(A, dtype=float).reshape(-1, 2),
        np.asarray(b, dtype=float),
        np.asarray(goal, dtype=float),
    )
    assert b.shape == (len(A),) and goal.shape == (2,)
    assert np.isfinite(A).all() and np.isfinite(b).all() and np.isfinite(goal).all()
    assert abs(np.linalg.norm(goal) - 1.0) < 1e-10
    return A, b, goal


def nonreverse_projection(A, b, speed, goal, nominal):
    """Least change with g.u>=0, with independent full-set feasibility checking."""
    A, b, goal = arrays(A, b, goal)
    nominal = np.asarray(nominal, dtype=float)
    augmented_A, augmented_b = np.vstack([A, goal]), np.append(b, 0.0)
    capability = maximize_direction(augmented_A, augmented_b, speed, goal)
    result = {"status": capability["status"], "feasibility_certificate": capability}
    if capability["status"] != "VERIFIED_FEASIBLE":
        return result
    u = cp.Variable(2)
    constraints = [augmented_A @ u >= augmented_b, cp.norm(u, 2) <= speed]
    problem = cp.Problem(cp.Minimize(cp.sum_squares(u - nominal)), constraints)
    solve(problem)
    if problem.status != "optimal" or u.value is None:
        return {**result, "status": "INCONCLUSIVE", "solver_status": problem.status}
    value = np.asarray(u.value).reshape(2)
    residual = float(np.min(augmented_A @ value - augmented_b))
    speed_excess = float(np.linalg.norm(value) - speed)
    if residual < -TOL or speed_excess > TOL:
        return {
            **result,
            "status": "INCONCLUSIVE",
            "minimum_residual": residual,
            "speed_excess": speed_excess,
        }
    return {
        **result,
        "u": value.tolist(),
        "objective": float(np.sum((value - nominal) ** 2)),
        "goal_velocity": float(goal @ value),
        "minimum_residual": residual,
        "speed_excess": speed_excess,
        "solver_status": problem.status,
    }


def minimum_object_relaxation(A, b, object_mask, speed, goal):
    """Diagnostic RHS change and dual lower bound. Never relax deployed rows."""
    A, b, goal = arrays(A, b, goal)
    mask = np.asarray(object_mask, dtype=bool)
    assert mask.shape == b.shape
    hard = nonreverse_projection(A[~mask], b[~mask], speed, goal, np.zeros(2))
    if hard["status"] == "VERIFIED_INFEASIBLE":
        return {"status": "NO_FINITE_OBJECT_RELAXATION", "hard_feasibility": hard}
    if hard["status"] != "VERIFIED_FEASIBLE":
        return {"status": "INCONCLUSIVE", "hard_feasibility": hard}
    u, epsilon = cp.Variable(2), cp.Variable(nonneg=True)
    forward = goal @ u >= 0.0
    constraints = [forward, cp.norm(u, 2) <= speed]
    rows = None
    if len(b):
        rows = A @ u + mask.astype(float) * epsilon >= b
        constraints.append(rows)
    problem = cp.Problem(cp.Minimize(epsilon), constraints)
    solve(problem)
    if problem.status != "optimal" or u.value is None:
        return {"status": "INCONCLUSIVE", "solver_status": problem.status}
    value, eps = np.asarray(u.value).reshape(2), max(0.0, float(epsilon.value))
    weights = (
        np.maximum(np.asarray(rows.dual_value).reshape(-1), 0.0)
        if rows is not None
        else np.empty(0)
    )
    mu = max(0.0, float(forward.dual_value))
    # Feasible dual: lambda>=0, mu>=0, sum(object lambda)<=1. Rescaling
    # produces a valid lower bound even with floating-point dual residuals.
    scale = max(1.0, float(np.sum(weights[mask])))
    weights, mu = weights / scale, mu / scale
    lower = float(b @ weights - speed * np.linalg.norm(A.T @ weights + mu * goal))
    residual = float(np.min(A @ value + mask * eps - b)) if len(b) else 0.0
    valid = (
        residual >= -TOL
        and np.linalg.norm(value) <= speed + TOL
        and goal @ value >= -TOL
        and -TOL <= eps - lower <= 1e-6
    )
    return {
        "status": "VERIFIED_FEASIBLE" if valid else "INCONCLUSIVE",
        "epsilon": eps,
        "dual_lower_bound": lower,
        "primal_dual_gap": eps - lower,
        "u": value.tolist(),
        "dual_multipliers": weights.tolist(),
        "nonreverse_multiplier": mu,
        "minimum_residual": residual,
        "goal_velocity": float(goal @ value),
        "speed_excess": float(np.linalg.norm(value) - speed),
        "solver_status": problem.status,
    }


def save(path, value):
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def analyze_state(condition, frame, item):
    state = item["actual"]
    A, b, goal = arrays(state["A"], state["b"], state["goal"])
    speed = state["params"]["max_speed"]
    mask = np.asarray(state["kinds"]) == "object"
    baseline = maximize_direction(A, b, speed, goal)
    assert baseline["classification"] == state["capability"]["classification"]
    if "maximum_goal_velocity" in baseline:
        assert (
            abs(
                baseline["maximum_goal_velocity"]
                - state["capability"]["maximum_goal_velocity"]
            )
            <= 1e-6
        )
    command, nominal = np.asarray(state["command"]), np.asarray(state["u_nom"])
    projected = nonreverse_projection(A, b, speed, goal, nominal)
    relaxed = minimum_object_relaxation(A, b, mask, speed, goal)
    zero = None
    support = state["post_support"]
    if support["applied"]:
        safety = SafetyFilter(SafetyFilterParams(**state["params"]))
        h = np.asarray(state["h"])
        assert len(h) == int(mask.sum())
        reconstructed = safety._cap_to_reachable(
            A[mask],
            support["demand_speed"]
            - state["params"]["gamma_obj"] * h
            + state["params"]["rho"],
        )
        np.testing.assert_allclose(reconstructed, b[mask], rtol=0.0, atol=1e-12)
        zero_b = b.copy()
        zero_b[mask] = safety._cap_to_reachable(
            A[mask], -state["params"]["gamma_obj"] * h + state["params"]["rho"]
        )
        np.testing.assert_array_equal(zero_b[~mask], b[~mask])
        zero = {
            "b": zero_b.tolist(),
            "maximum_rhs_drop": float(np.max(b - zero_b)) if len(b) else 0.0,
            "capability": maximize_direction(A, zero_b, speed, goal),
            "is_safety_validated": False,
        }
    selected = bool(item["selected_pusher"])
    row = {
        "condition": condition,
        "frame": frame,
        "agent_id": state["agent_id"],
        "selected_pusher": selected,
        "declared_pusher": bool(state["declared_pusher"]),
        "support_applied": bool(support["applied"]),
        "support_reason": support["reason"],
        "original_solver_status": state["solver_status"],
        "object_rows": int(mask.sum()),
        "nominal_goal_velocity": float(goal @ nominal),
        "executed_goal_velocity": float(goal @ command),
        "original_objective": float(np.sum((command - nominal) ** 2)),
        "baseline": baseline,
        "nonreverse": projected,
        "minimum_object_rhs_change": relaxed,
        "zero_motion_demand": zero,
        "input_system": {
            "A": A.tolist(),
            "b": b.tolist(),
            "object_mask": mask.tolist(),
            "goal": goal.tolist(),
            "nominal": nominal.tolist(),
            "speed": speed,
        },
    }
    if projected["status"] == "VERIFIED_FEASIBLE":
        row["objective_increase"] = projected["objective"] - row["original_objective"]
        row["command_change_norm"] = float(
            np.linalg.norm(np.asarray(projected["u"]) - command)
        )
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(ROOT / "runs"):
        raise ValueError("Fresh output must stay under repository runs/")
    guard()
    plan = study.check()
    receipt = read(INPUT / "outcome.json")
    assert receipt["status"] == "VERIFIED_FIRST_PUSHER_FILTER_COMPARISON"
    assert receipt["analysis_sha256"] == sha(INPUT / "analysis.json")
    paths = [INPUT / (name + ".json") for name in study.CONDITIONS]
    paths += [
        INPUT / "outcome.json",
        INPUT / "analysis.json",
        DESIGN,
        Path(__file__),
        ROOT / "tests/test_phase2_route_tradeoffs.py",
        ROOT / "scripts/probe_post_support_contact_direction.py",
        ROOT / "src/dbact/safety/filter.py",
    ]
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}
    out.mkdir(parents=True, exist_ok=False)
    snap = out / "inputs_and_sources"
    snap.mkdir()
    for path in paths:
        shutil.copy2(path, snap / path.name)
    identity = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": hashes,
        "runtime_source_sha256": plan["frozen_source_sha256"],
        "protocol_sha256": sha(study.PROTOCOL),
        "environment": plan["environment"],
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
        "compute_budget_seconds": 7200,
    }
    save(out / "started.json", identity)
    started = time.perf_counter()
    rows = []
    try:
        with threadpool_limits(limits=1):
            for condition in study.CONDITIONS:
                data = read(INPUT / (condition + ".json"))
                assert len(data["states"]) == 16
                for item in data["states"]:
                    if time.perf_counter() - started >= 7200:
                        raise TimeoutError("Offline budget exhausted")
                    rows.append(analyze_state(condition, data["frame"], item))
                print(
                    json.dumps({"condition": condition, "states": len(rows)}),
                    flush=True,
                )
        assert len(rows) == 64 and sum(r["selected_pusher"] for r in rows) == 4
        # Independent review's algebraic counterexample to a generic cap claim.
        params = SafetyFilterParams()
        normals = np.array([[1.0, 0.0], [1.0, 0.0], [-1.0, 0.0]])
        capped = SafetyFilter(params)._cap_to_reachable(normals, np.full(3, 100.0))
        cap_example = {
            "normals": normals.tolist(),
            "capped_rhs": capped.tolist(),
            "max_speed": params.max_speed,
            "certificate": maximize_direction(
                normals, capped, params.max_speed, np.array([1.0, 0.0])
            ),
            "scope": "Synthetic algebraic example, not a recorded physical state",
        }
        assert cap_example["certificate"]["classification"] == "INFEASIBLE"
        save(out / "cap_claim_counterexample.json", cap_example)
        groups = {}
        for label, subgroup in [
            ("all", rows),
            ("selected_pushers", [r for r in rows if r["selected_pusher"]]),
            ("support_applied", [r for r in rows if r["support_applied"]]),
        ]:
            groups[label] = {
                "states": len(subgroup),
                "baseline": dict(
                    Counter(r["baseline"]["classification"] for r in subgroup)
                ),
                "nonreverse": dict(
                    Counter(r["nonreverse"]["status"] for r in subgroup)
                ),
                "object_relaxation": dict(
                    Counter(r["minimum_object_rhs_change"]["status"] for r in subgroup)
                ),
                "zero_motion_demand": dict(
                    Counter(
                        r["zero_motion_demand"]["capability"]["classification"]
                        for r in subgroup
                        if r["zero_motion_demand"] is not None
                    )
                ),
            }
        elapsed = time.perf_counter() - started
        report = {
            **identity,
            "groups": groups,
            "rows": rows,
            "wall_seconds": elapsed,
            "scope": "Reused fixed-state diagnostics; no trajectory improvement or safety guarantee",
        }
        study.check()
        assert all(sha(ROOT / name) == expected for name, expected in hashes.items())
        save(out / "analysis.json", report)
        with (out / "all_64_states.csv").open(
            "x", encoding="utf-8", newline=""
        ) as handle:
            flat = []
            for row in rows:
                flat.append(
                    {
                        key: row[key]
                        for key in (
                            "condition",
                            "frame",
                            "agent_id",
                            "selected_pusher",
                            "declared_pusher",
                            "support_applied",
                            "support_reason",
                            "nominal_goal_velocity",
                            "executed_goal_velocity",
                        )
                    }
                )
                flat[-1].update(
                    baseline=row["baseline"]["classification"],
                    maximum_goal_velocity=row["baseline"].get("maximum_goal_velocity"),
                    nonreverse_status=row["nonreverse"]["status"],
                    nonreverse_goal_velocity=row["nonreverse"].get("goal_velocity"),
                    objective_increase=row.get("objective_increase"),
                    command_change_norm=row.get("command_change_norm"),
                    minimum_object_rhs_change=row["minimum_object_rhs_change"].get(
                        "epsilon"
                    ),
                    zero_demand_classification=(
                        row["zero_motion_demand"]["capability"]["classification"]
                        if row["zero_motion_demand"]
                        else "NOT_APPLIED"
                    ),
                )
            writer = csv.DictWriter(handle, fieldnames=list(flat[0]))
            writer.writeheader()
            writer.writerows(flat)
        save(
            out / "outcome.json",
            {
                **identity,
                "status": "COMPLETED_FIXED_STATE_DIAGNOSTIC",
                "analysis_sha256": sha(out / "analysis.json"),
                "groups": groups,
                "wall_seconds": elapsed,
            },
        )
        print(json.dumps({"groups": groups, "wall_seconds": elapsed}), flush=True)
    except Exception:
        save(
            out / "failure.json",
            {
                **identity,
                "completed_states": len(rows),
                "wall_seconds": time.perf_counter() - started,
                "traceback": traceback.format_exc(),
            },
        )
        raise


if __name__ == "__main__":
    main()
