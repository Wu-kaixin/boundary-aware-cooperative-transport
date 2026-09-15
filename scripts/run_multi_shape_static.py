"""Task C: multi-shape static-oracle theory baseline (9 cases).

Runs the sampled theorem_mode static caging cover on three cargo shapes
(L-shape, axis-aligned rectangle, non-convex C/U-channel) x three seeds (2,5,8)
x 30 s (600 frames) with the independent UniformOffsetObserver every step.

The L-shape case is *reused* from the existing
``n16_discrete_dissipation/seed_*`` artifacts (already produced); rectangle and
C-shape are simulated fresh into ``n16_multi_shape/<shape>/seed_*``.

Each case reports: complete_30s / abort reason, zero_input_with_rho failure
count, H0, H_final, J_bar, E_bar, post-hoc J bound and geometric J bound, and
whether the hypothetical-next-state discrete inequality slack stays within
tolerance for the final command.

This is explicitly a ``static_oracle_theory_baseline`` -- NOT a certificate for
unknown-object transport.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from audit_discrete_dissipation import projection_verdict, run_case  # noqa: E402
from audit_JE_and_bounds import (  # noqa: E402
    build_observers,
    compute_bounds,
    dump,
    je_series,
    three_command_hyp,
)
from audit_n16_static_performance import theoretical_constants  # noqa: E402

SLACK_TOL = 1e-6
ARTIFACT_ROOT = Path(
    r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11"
)
L_DISSIPATION_DIR = ARTIFACT_ROOT / "n16_discrete_dissipation"


def cargo_vertices(cfg: dict) -> np.ndarray:
    from dbact_sim.scenarios import build_cargoes

    return build_cargoes(cfg, seed=0)[0].vertices.copy()


def case_metrics(step_records, vertices, cfg, consts, observer, fine, kc, dt, N) -> dict:
    series = je_series(step_records)
    bounds = compute_bounds(series, consts, kc, dt, N)
    hyp = three_command_hyp(step_records, vertices, observer, fine, dt)
    cmd = hyp["stages"]["u_command"]
    return {"bounds": bounds, "hyp": hyp, "u_command_hyp": cmd}


def run_fresh_shape(shape_key, config_path, out_shape_dir, seeds, frames, ntheta, nradial) -> list[dict]:
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    from dbact_sim.scenarios import controller_params_from_config

    params = controller_params_from_config(cfg)
    kc = float(params.kp_cage)
    rho = float(params.rho)
    dt = float(cfg["dt"])
    N = int(cfg["agents"]["count"])
    vertices = cargo_vertices(cfg)
    consts = theoretical_constants(cfg, vertices)
    dump(out_shape_dir / "constants.json", consts)
    observer, fine = build_observers(cfg, ntheta, nradial)

    rows = []
    for seed in seeds:
        seed_dir = out_shape_dir / f"seed_{seed}"
        payload = run_case(config_path, seed_dir, seed, frames, ntheta=ntheta, nradial=nradial)
        summary = payload["summary"]
        proj = projection_verdict(payload["projection_rows"], rho)
        metrics = case_metrics(
            payload["step_rows"], vertices, cfg, consts, observer, fine, kc, dt, N
        )
        rows.append(
            build_case_row(shape_key, seed, summary, proj, metrics, consts, complete_source=summary)
        )
        print(f"[{shape_key} seed {seed}] " + json.dumps(rows[-1], ensure_ascii=False), flush=True)
    return rows


def reuse_l_shape(config_path, seeds, ntheta, nradial) -> list[dict]:
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    from dbact_sim.scenarios import controller_params_from_config

    params = controller_params_from_config(cfg)
    kc = float(params.kp_cage)
    dt = float(cfg["dt"])
    N = int(cfg["agents"]["count"])
    vertices = cargo_vertices(cfg)
    consts = theoretical_constants(cfg, vertices)
    observer, fine = build_observers(cfg, ntheta, nradial)

    proj_conditions = json.loads(
        (L_DISSIPATION_DIR / "projection_conditions.json").read_text(encoding="utf-8")
    )
    per_seed_rho = {}
    for entry in proj_conditions.get("margin_band_audit", {}).get("per_seed", []):
        per_seed_rho[int(entry["seed"])] = entry

    rows = []
    for seed in seeds:
        seed_dir = L_DISSIPATION_DIR / f"seed_{seed}"
        step_records = json.loads((seed_dir / "step_records.json").read_text(encoding="utf-8"))
        summary = json.loads((seed_dir / "summary.json").read_text(encoding="utf-8"))
        metrics = case_metrics(step_records, vertices, cfg, consts, observer, fine, kc, dt, N)
        rho_entry = per_seed_rho.get(seed, {})
        proj = {
            "zero_input_infeasible_with_rho_agent_steps": rho_entry.get("zero_with_rho_false", 0),
            "inside_margin_band_agent_steps": rho_entry.get("inside_margin_band", 0),
        }
        rows.append(
            build_case_row("l_shape", seed, summary, proj, metrics, consts, complete_source=summary)
        )
        print(f"[l_shape seed {seed} reuse] " + json.dumps(rows[-1], ensure_ascii=False), flush=True)
    return rows


def build_case_row(shape_key, seed, summary, proj, metrics, consts, complete_source) -> dict:
    b = metrics["bounds"]
    cmd = metrics["u_command_hyp"]
    refined = cmd["max_slack_refined"]
    return {
        "shape": shape_key,
        "seed": seed,
        "complete_30s": bool(complete_source.get("complete_30s")),
        "frames_completed": complete_source.get("frames_completed"),
        "abort": complete_source.get("abort"),
        "zero_input_with_rho_failures": int(
            proj.get("zero_input_infeasible_with_rho_agent_steps", 0)
        ),
        "inside_margin_band_agent_steps": int(proj.get("inside_margin_band_agent_steps", 0)),
        "H0": b["H0"],
        "H_final": complete_source.get("H_final"),
        "J_bar": b["J_bar"],
        "E_bar": b["E_bar"],
        "E_bar_estimate_type": "post_hoc_trajectory",
        "posthoc_J_bound": b["posthoc_J_bound"],
        "J_geom_bound_cellwise": b["J_geom_bound_cellwise_Nmplus_umax2"],
        "posthoc_J_bound_below_geom": b["posthoc_J_bound_below_geom_cellwise"],
        "u_command_hyp_max_slack_refined": refined,
        "inequality_slack_ok": bool(refined is not None and refined <= SLACK_TOL),
        "numerical_consistency": cmd["numerical_consistency"],
        "strict_numerical_certificate": cmd["strict_numerical_certificate"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=ARTIFACT_ROOT / "n16_multi_shape")
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cfg_dir = ROOT / "configs/sim/theorem"
    all_rows = []

    # 1) L-shape: reuse existing dissipation artifacts.
    all_rows += reuse_l_shape(cfg_dir / "static_l_shape_n16_oracle.yaml", args.seeds, args.ntheta, args.nradial)

    # 2) Rectangle (fresh).
    all_rows += run_fresh_shape(
        "rectangle",
        cfg_dir / "static_rectangle_n16_oracle.yaml",
        args.out / "rectangle",
        args.seeds,
        args.frames,
        args.ntheta,
        args.nradial,
    )

    # 3) Non-convex C/U-channel (fresh).
    all_rows += run_fresh_shape(
        "c_shape",
        cfg_dir / "static_c_shape_n16_oracle.yaml",
        args.out / "c_shape",
        args.seeds,
        args.frames,
        args.ntheta,
        args.nradial,
    )

    table = {
        "experiment_class": "static_oracle_theory_baseline",
        "not_a_certificate_for": "unknown-object transport",
        "seeds": args.seeds,
        "frames": args.frames,
        "duration_s": args.frames * 0.05,
        "observer_quadrature": {"ntheta": args.ntheta, "nradial": args.nradial},
        "cases": all_rows,
        "roll_up": {
            "complete_30s": sum(1 for r in all_rows if r["complete_30s"]),
            "aborted": sum(1 for r in all_rows if r["abort"] is not None),
            "zero_input_with_rho_failures_total": sum(
                r["zero_input_with_rho_failures"] for r in all_rows
            ),
            "inside_margin_band_total": sum(r["inside_margin_band_agent_steps"] for r in all_rows),
            "all_inequality_slack_ok": all(r["inequality_slack_ok"] for r in all_rows),
        },
    }
    dump(args.out / "multi_shape_table.json", table)
    print(json.dumps(table, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
