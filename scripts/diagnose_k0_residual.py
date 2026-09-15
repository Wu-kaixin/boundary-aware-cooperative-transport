"""Residual term when 0 is not in F_rho (route B sketch + numeric diagnostic).

Projection optimality on F = {u : A u <= b} gives, for every v in F,
    (v - u)^T (u - u_sat) >= 0.
If z = Pi_F(0) (min-norm feasible point), then
    (z - u)^T (u - u_sat) >= 0.
With u_sat = lambda (chat - p) and g* involving (p - c*), the usual cascade
picks up an extra inner product involving z.

This module records a *computable* residual majorant on historical frames:
    R_k = sum_i 2 m_i* ||e_i|| ||z_i||   (loose)
and asks whether a uniform a priori B_R exists.  Trajectory-measured R_k is
post_hoc and must not be written into B_E / B_J priors.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def min_norm_point_halfspaces(A: np.ndarray, b: np.ndarray, umax: float) -> np.ndarray:
    """Approx Pi_F(0) by solving min ||u||^2 s.t. A u <= b, ||u||<=umax (projection QP).

    Falls back to zeros if no rows.  Used only for diagnostics.
    """
    A = np.asarray(A, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)
    if len(A) == 0:
        return np.zeros(2)
    # Single most-violated row at 0: if b_min >= 0 then 0 is feasible.
    if float(np.min(b)) >= -1e-12:
        return np.zeros(2)
    # For one halfspace a·u <= β with β<0, min-norm point is (β/||a||^2) a.
    # Multi-row: take the worst single-row candidate then verify; else scale.
    i = int(np.argmin(b))
    a = A[i]
    na2 = float(a @ a)
    if na2 <= 1e-18:
        return np.zeros(2)
    z = (float(b[i]) / na2) * a
    # If z violates other rows, fall back to radial scaling toward a feasible direction.
    if np.all(A @ z <= b + 1e-9):
        n = float(np.linalg.norm(z))
        if n > umax:
            z = z * (umax / n)
        return z
    # Least-squares against active equalities a_i·u = b_i for negative b_i.
    active = b < 0.0
    Aa, ba = A[active], b[active]
    if len(Aa) == 0:
        return np.zeros(2)
    z, *_ = np.linalg.lstsq(Aa, ba, rcond=None)
    n = float(np.linalg.norm(z))
    if n > umax > 0:
        z = z * (umax / n)
    return np.asarray(z, dtype=float)


def residual_majorant_from_step(
    mass_star: np.ndarray,
    e_norm: np.ndarray,
    z_norm: np.ndarray,
) -> float:
    """Loose residual majorant sum 2 m* ||e|| ||z||."""
    m = np.asarray(mass_star, dtype=float).reshape(-1)
    e = np.asarray(e_norm, dtype=float).reshape(-1)
    z = np.asarray(z_norm, dtype=float).reshape(-1)
    return float(2.0 * np.sum(m * e * z))


def diagnose_k0_gap(step_records_path: Path, umax: float = 0.35) -> dict:
    """Post-hoc scan of K0 failures; does not produce a prior constant."""
    recs = json.loads(Path(step_records_path).read_text(encoding="utf-8"))
    fails = []
    for r in recs:
        if r.get("in_K0", True):
            continue
        bad = r.get("k0_bad_agents") or []
        fails.append(
            {
                "frame": r.get("frame"),
                "time": r.get("time"),
                "bad_agents": bad,
                "zero_input_feasible": [
                    bool(x) for i, x in enumerate(r.get("zero_input_feasible", [])) if i in bad
                ]
                if bad
                else [],
                "zero_input_feasible_with_rho": [
                    bool(x)
                    for i, x in enumerate(r.get("zero_input_feasible_with_rho", []))
                    if i in bad
                ]
                if bad
                else [],
                "solver_status": [
                    r.get("solver_status", [])[i] for i in bad if i < len(r.get("solver_status", []))
                ],
                "J_k": r.get("J_k"),
                "E_k": r.get("E_k"),
                "note": "A,b not stored in step_records; z=Pi(0) needs online logging for route B",
            }
        )
    return {
        "status": "post_hoc_diagnosis",
        "n_fail_frames": len(fails),
        "fails": fails,
        "prior_constant_allowed": False,
        "reason": (
            "Without an a priori bound on ||Pi_F(0)|| from design/initial data alone, "
            "any residual built from these frames is post_hoc and cannot enter B_J_prior."
        ),
    }


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("step_records", type=Path)
    args = ap.parse_args()
    print(json.dumps(diagnose_k0_gap(args.step_records), indent=2))
