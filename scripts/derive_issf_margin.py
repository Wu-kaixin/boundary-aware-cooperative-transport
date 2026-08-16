#!/usr/bin/env python
"""Re-derive the ISSf margin ``rho`` against a *moving* boundary.

    python scripts/derive_issf_margin.py configs/sim/d/l_shape_closed_loop.yaml

The debt
--------
`docs/CLOSED_LOOP_V2.md` §3.1 rewrote the S1 certificate *criterion* and said plainly that
it did not re-derive the ISSf constant. §6.3 and §7b.2 then measured the premise that
constant rests on and found it violated by 60.4% of cells — and by 36.7% with a **noiseless**
sensor, so it is not a sensor problem. This is the re-derivation.

The derivation
--------------
The object row is built on ``h_k = n_k^T (p_i - b_k) - r_safe``. Differentiating exactly:

    dh_k/dt  =  n_k^T (u_i - v_{b_k})  +  (dn_k/dt)^T (p_i - b_k)
                \\_______ kept _______/    \\____ dropped, priced at rho ____/

`safety_filter`'s own docstring says rho "is the price of dropping the
``d/dt(n_k)^T (p_i - b_k)`` term". Bounding that term:

**1. Rigid-body rotation.** For a rigid body the surface normal rotates with it, so
``dn_k/dt = omega R90 n_k`` and

    (dn_k/dt)^T (p_i - b_k)  =  omega * (R90 n_k)^T (p_i - b_k)  =  omega * t_k

where ``t_k`` is the robot's *tangential* offset from its boundary point. That offset is
bounded by construction: the tangential window ``W`` is exactly the filter that admits a row
only while ``|t_k| <= W``. Hence

    | dropped term |  <=  omega_max * W                                          (1)

This is why the window is "part of the barrier construction rather than an implementation
detail" — it is what makes the dropped term boundable at all.

**2. Estimator normal drift.** ``n_k`` is also a fused estimate that moves as cells are
added and removed. `_aggregate_face` was introduced precisely to make that motion
``O(g_k / sum g)`` per step instead of a switch between samples, so it contributes a further
``|dn/dt|_est * W``. It is not separately instrumented here and is reported as an unquantified
addition rather than folded into a number.

The structural finding: rho is double-booked
--------------------------------------------
``guarantees.build_admissibility_certificate`` checks

    bounded_perception_and_motion_error:  velocity_error <= rho

but ``velocity_error`` is the error in the **kept** term ``n_k^T v_{b_k}`` — a different
disturbance from the **dropped** term rho was budgeted for. One budget is being asked to
cover two independent errors, and the check reads as though rho covers the velocity error
when the safety filter's own docstring says it covers the normal-rate term.

The honest requirement is their sum:

    rho  >=  omega_max * W  +  e_v                                               (2)

with ``e_v`` the barrier-visible velocity error measured by `dbact.error_audit` term 6.

And rho is capped from above by what a robot can deliver
--------------------------------------------------------
``_cap_to_reachable`` limits every object row to ``recovery_fraction * max_speed`` along the
common retreat direction. A margin larger than that is not a stronger guarantee, it is an
infeasible problem. So a *satisfiable* ISSf statement needs

    omega_max * W  +  e_v   <=   rho   <=   recovery_fraction * max_speed        (3)

and when the left side exceeds the right, no value of rho works: the actuator cannot deliver
the margin the analysis requires. That is a statement about the robot, not about tuning.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact_sim.scenarios import controller_params_from_config, load_yaml  # noqa: E402

RESULTS = ROOT / "docs" / "results"


def measured_velocity_error(source: Path) -> dict:
    """Barrier-visible velocity error from a committed six-term audit, if present."""
    if not source.exists():
        return {}
    payload = json.loads(source.read_text(encoding="utf-8"))
    arms = payload.get("arms", {})
    out = {}
    for name in ("nominal", "range_noise_000", "off"):
        arm = arms.get(name)
        if not arm:
            continue
        d = arm["distributions"]
        out[name] = {
            "max": (d.get("measured_velocity_error") or {}).get("max"),
            "mean": (d.get("normal_projection_error_mps_mean") or {}).get("mean"),
            "p99": (d.get("normal_projection_error_mps_p99") or {}).get("mean"),
            "breach_fraction": (d.get("velocity_breach_fraction") or {}).get("mean"),
        }
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", nargs="?", default="configs/sim/d/l_shape_closed_loop.yaml")
    parser.add_argument("--omega-max", type=float, default=None,
                        help="Yaw-rate bound in rad/s. Defaults to the controller's "
                             "max_object_yaw_rate, which is the bound the SE(2) estimator "
                             "admits and therefore the bound the barrier must survive.")
    parser.add_argument("--out", default="docs/results/issf")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    config = load_yaml(args.config)
    p = controller_params_from_config(config)
    W = float(p.object_row_window)
    rho = float(p.rho)
    omega_max = float(args.omega_max if args.omega_max is not None else p.max_object_yaw_rate)
    cap = float(p.recovery_fraction * p.max_speed)

    rotation_term = omega_max * W
    audit = measured_velocity_error(RESULTS / "t5" / "robustness_ablation.json")
    e_v_clean = (audit.get("range_noise_000") or {}).get("max")
    e_v_nominal = (audit.get("nominal") or {}).get("max")

    required_rotation_only = rotation_term
    required_with_ev = {
        name: rotation_term + (value["max"] or 0.0) for name, value in audit.items()
    }

    # The regimes in which the statement is satisfiable, inverted from (2) and (3).
    omega_for_configured_rho = rho / W
    omega_for_cap = cap / W
    ev_budget_at_cap = max(0.0, cap - rotation_term)

    report = {
        "config": args.config,
        "constants": {
            "rho_configured": rho,
            "tangential_window_W": W,
            "omega_max_rad_s": omega_max,
            "recovery_fraction": float(p.recovery_fraction),
            "max_speed": float(p.max_speed),
            "reachable_cap": cap,
        },
        "derivation": {
            "rotation_term_omega_max_times_W": rotation_term,
            "required_rho_rotation_only": required_rotation_only,
            "required_rho_with_measured_velocity_error": required_with_ev,
            "satisfiable_rotation_only": bool(rotation_term <= cap),
        },
        "measured_velocity_error": audit,
        "regimes": {
            "omega_max_covered_by_configured_rho_rad_s": omega_for_configured_rho,
            "omega_max_covered_by_configured_rho_deg_s": omega_for_configured_rho * 180.0 / 3.141592653589793,
            "omega_max_at_the_reachable_cap_rad_s": omega_for_cap,
            "velocity_error_budget_left_at_the_cap": ev_budget_at_cap,
        },
        "structural_finding": (
            "guarantees.build_admissibility_certificate checks velocity_error <= rho, but "
            "velocity_error is the error in the KEPT term n^T v while rho is the price of the "
            "DROPPED d/dt(n) term. One budget, two independent disturbances."
        ),
    }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "issf_margin.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    if args.json:
        print(json.dumps(report, indent=2))
        return 0

    print(f"config                         {args.config}")
    print(f"rho as configured              {rho:.4f} m/s")
    print(f"tangential window W            {W:.4f} m")
    print(f"omega_max (yaw-rate bound)     {omega_max:.4f} rad/s")
    print(f"reachable cap f*v_max          {cap:.4f} m/s")
    print()
    print("(1) rotation term  omega_max * W")
    print(f"      = {omega_max:.4f} * {W:.4f} = {rotation_term:.4f} m/s")
    print(f"      vs rho    {rotation_term / rho:8.1f}x the configured margin")
    print(f"      vs cap    {rotation_term / cap:8.2f}x what the actuator can deliver")
    print()
    if audit:
        print("(2) plus the measured barrier-visible velocity error e_v")
        for name, value in audit.items():
            total = rotation_term + (value["max"] or 0.0)
            print(f"      {name:18s} e_v_max = {value['max']:.4f}   "
                  f"rho_required = {total:.4f}   "
                  f"breach at rho = {value['breach_fraction']:.3f}")
        print()
    print("(3) satisfiability   omega_max*W + e_v  <=  rho  <=  f*v_max")
    if rotation_term > cap:
        print(f"      UNSATISFIABLE at this yaw bound: the rotation term alone "
              f"({rotation_term:.4f}) exceeds")
        print(f"      the reachable cap ({cap:.4f}). No value of rho works; this is a statement")
        print("      about the actuator, not about tuning.")
    else:
        print(f"      rotation term fits under the cap with {cap - rotation_term:.4f} m/s left "
              "for e_v.")
    print()
    print("regimes")
    print(f"      rho = {rho:.4f} covers rotation up to "
          f"{omega_for_configured_rho:.4f} rad/s = {omega_for_configured_rho * 57.2958:.2f} deg/s")
    print(f"      the cap covers rotation up to     {omega_for_cap:.4f} rad/s = "
          f"{omega_for_cap * 57.2958:.2f} deg/s")
    print(f"      e_v budget left at the cap        {ev_budget_at_cap:.4f} m/s")
    print()
    print("structural finding")
    print("      velocity_error <= rho double-books one budget across two independent")
    print("      disturbances: the KEPT term's estimation error and the DROPPED normal-rate")
    print("      term. See guarantees.issf_margin_budget.")
    print(f"\nwrote {out / 'issf_margin.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
