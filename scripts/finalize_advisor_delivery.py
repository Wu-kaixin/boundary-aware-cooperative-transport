"""Finalize the advisor delivery for the sampled static theorem_mode audit.

This script is *offline*: it reuses the already-produced nine-case step records
(L-shape / rectangle / C-channel x seeds 2,5,8) and never re-runs a 30 s
simulation.  It produces the final advisor-delivery bundle under

    artifacts/theorem_audit_theorem_mode_2026-09-11/final_advisor_delivery/

with three products:

1. ``three_command_hyp/`` -- the *fixed* hypothetical-next-state discrete
   dissipation check (H, g, m all from the same observer on refinement; no
   strict-certificate claim).

2. ``j_bound/`` + ``nine_case_summary.json`` -- the finite-time J bound with a
   per-robot effective gain ``lambda_i`` and an explicit qualifying step set
   ``K_0``.  For c_shape seed 5 the full-horizon J bound is *inapplicable*
   (frames 241-246, agent 14 violate zero-input-with-rho); the post-hoc J_bar /
   E_bar are still reported as descriptive statistics tagged
   ``not_covered_by_theorem``.

3. ``figures/`` + comparison CSV -- 3x3 time-series grids of the gradient
   residual, centroid residual, J_k and E_k with discrete-average lines.

Effective gain / bound conventions
----------------------------------
For every step k and agent i with ``d_i = chat_i - p_i`` (controller grid
centroid minus site):

    lambda_i = min(k_c, u_max / ||d_i||)      if ||d_i|| > eps
             = k_c                            otherwise (idle / unsaturated)

    a_i = 2 / lambda_i - Delta ,   a = 2 / k_c - Delta  (global; a_i >= a > 0)

A step is in ``K_0`` iff every agent solver status is ``optimal``, every
``zero_input_feasible_with_rho`` flag is true (falling back to
``zero_input_feasible`` when the finer flag is absent), and ``a > 0``.

Post-hoc finite-horizon bound (full horizon in K_0 only):

    J_bar <= 2 H0 / (a K Delta) + 4 E_bar / a^2         (E_bar is post_hoc)

Primary geometric J ceiling (a priori): ``M_total * u_max^2`` with
``M_total = observer.total_mass(vertices)``.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from audit_JE_and_bounds import build_observers, dump, three_command_hyp  # noqa: E402
from audit_n16_static_performance import theoretical_constants  # noqa: E402

ARTIFACT_ROOT = Path(
    r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11"
)
OUT_ROOT = ARTIFACT_ROOT / "final_advisor_delivery"

# Shape -> (config file, seed-directory parent) mapping for the reused records.
SHAPES = {
    "l_shape": {
        "config": ROOT / "configs/sim/theorem/static_l_shape_n16_oracle.yaml",
        "data_dir": ARTIFACT_ROOT / "n16_discrete_dissipation",
    },
    "rectangle": {
        "config": ROOT / "configs/sim/theorem/static_rectangle_n16_oracle.yaml",
        "data_dir": ARTIFACT_ROOT / "n16_multi_shape" / "rectangle",
    },
    "c_shape": {
        "config": ROOT / "configs/sim/theorem/static_c_shape_n16_oracle.yaml",
        "data_dir": ARTIFACT_ROOT / "n16_multi_shape" / "c_shape",
    },
}

EPS_D = 1e-9  # threshold on ||chat_i - p_i|| below which the agent is "idle".


def cargo_vertices(cfg: dict) -> np.ndarray:
    """Fixed-cargo vertices (positions not needed; cargo is frozen)."""
    from dbact_sim.scenarios import build_cargoes

    return build_cargoes(cfg, seed=0)[0].vertices.copy()


# ---------------------------------------------------------------------------
# Finite-time J bound with per-robot effective gain lambda_i.
# ---------------------------------------------------------------------------
def finite_time_bound(
    step_records: list[dict],
    kc: float,
    umax: float,
    dt: float,
    N: int,
    M_total: float,
) -> dict:
    a_global = 2.0 / kc - dt
    K = len(step_records)

    J = np.empty(K)
    E = np.empty(K)
    H = np.empty(K)
    g2 = np.empty(K)
    c2 = np.empty(K)  # sum_i ||c*_i - p_i||^2 (prior 'centroid2' definition)
    a_eff = np.empty(K)
    lambda_min = np.empty(K)
    lambda_max = np.empty(K)
    lambda_from_sat_mean = np.full(K, np.nan)

    in_K0 = np.zeros(K, dtype=bool)
    fail_frames: list[int] = []
    fail_agents: set[int] = set()
    zero_rho_fail_count = 0

    for idx, rec in enumerate(step_records):
        P = np.asarray(rec["positions"], dtype=float)
        chat = np.asarray(rec["cell_centroid"], dtype=float)
        u_sat = np.asarray(rec["u_saturated"], dtype=float)
        u_cmd = np.asarray(rec["u_command"], dtype=float)
        m = np.asarray(rec["mass_star"], dtype=float)
        c_star = np.asarray(rec["centroid_star"], dtype=float)
        g = np.asarray(rec["gradient_star"], dtype=float)

        d = chat - P
        nd = np.linalg.norm(d, axis=1)
        active = nd > EPS_D
        lam = np.full(N, kc, dtype=float)
        lam[active] = np.minimum(kc, umax / nd[active])
        # Effective-gain-from-saturation diagnostic (u_sat . d / ||d||^2).
        lam_sat = np.full(N, np.nan)
        if np.any(active):
            lam_sat[active] = np.sum(u_sat[active] * d[active], axis=1) / (nd[active] ** 2)

        a_i = 2.0 / lam - dt
        a_eff[idx] = float(np.min(a_i))
        lambda_min[idx] = float(np.min(lam))
        lambda_max[idx] = float(np.max(lam))
        if np.any(active):
            lambda_from_sat_mean[idx] = float(np.nanmean(lam_sat[active]))

        J[idx] = float(np.sum(m * np.sum(u_cmd * u_cmd, axis=1)))
        E[idx] = float(np.sum(m * np.sum((chat - c_star) ** 2, axis=1)))
        H[idx] = float(rec["H_star"])
        g2[idx] = float(np.sum(g * g))
        c2[idx] = float(np.sum((c_star - P) ** 2))

        zr = rec.get("zero_input_feasible_with_rho", rec.get("zero_input_feasible"))
        zr = [bool(z) for z in zr]
        status = [str(s) == "optimal" for s in rec["solver_status"]]
        bad = [i for i in range(N) if (not zr[i]) or (not status[i])]
        zero_rho_fail_count += sum(1 for i in range(N) if not zr[i])
        step_ok = (len(bad) == 0) and (a_global > 0)
        in_K0[idx] = step_ok
        if not step_ok:
            fail_frames.append(int(rec["frame"]))
            fail_agents.update(bad)

    full_horizon = bool(np.all(in_K0))
    H0 = float(H[0])
    H_final = float(step_records[-1]["H_star_next"])
    J_bar = float(np.mean(J))
    E_bar = float(np.mean(E))
    g2_bar = float(np.mean(g2))
    c2_bar = float(np.mean(c2))
    min_a_eff = float(np.min(a_eff))

    J_geom = float(M_total) * umax * umax

    if full_horizon and a_global > 0:
        posthoc_J_bound = 2.0 * H0 / (a_global * K * dt) + 4.0 * E_bar / (a_global * a_global)
        posthoc_below_geom = bool(posthoc_J_bound <= J_geom)
        j_bound_status = "applicable_post_hoc"
        j_bound_label = "post_hoc"
    else:
        posthoc_J_bound = None
        posthoc_below_geom = None
        j_bound_status = "inapplicable"
        j_bound_label = "not_covered_by_theorem"

    # Discrete vs continuous-time averaging note (report both for J).
    _trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))
    T = (K - 1) * dt
    J_bar_trapz = float(_trapz(J, dx=dt) / T) if T > 0 else J_bar
    E_bar_trapz = float(_trapz(E, dx=dt) / T) if T > 0 else E_bar

    return {
        "K": K,
        "dt": dt,
        "k_c": kc,
        "u_max": umax,
        "a_global": a_global,
        "min_a_eff": min_a_eff,
        "M_total": float(M_total),
        "theorem_conditions_hold_full_horizon": full_horizon,
        "condition_failure_frames": sorted(fail_frames),
        "condition_failure_agent_indices": sorted(int(i) for i in fail_agents),
        "zero_input_with_rho_failure_count": int(zero_rho_fail_count),
        "H0": H0,
        "H_final": H_final,
        "J_bar": J_bar,
        "E_bar": E_bar,
        "E_bar_estimate_type": "post_hoc_trajectory",
        "posthoc_J_bound": posthoc_J_bound,
        "posthoc_J_bound_terms": (
            {
                "term_2H0_over_aKD": 2.0 * H0 / (a_global * K * dt),
                "term_4Ebar_over_a2": 4.0 * E_bar / (a_global * a_global),
            }
            if full_horizon and a_global > 0
            else None
        ),
        "J_geometric_bound_M_umax2": J_geom,
        "posthoc_bound_below_geom": posthoc_below_geom,
        "gradient_residual_bar": g2_bar,
        "centroid2_bar": c2_bar,
        "full_horizon_J_bound_status": j_bound_status,
        "descriptive_stats_label": j_bound_label,
        "averaging_note": {
            "discrete_mean": "(1/K) sum_k",
            "continuous_time_approx": "trapezoidal integral / T, T=(K-1)*dt",
            "J_bar_discrete": J_bar,
            "J_bar_trapezoidal": J_bar_trapz,
            "E_bar_discrete": E_bar,
            "E_bar_trapezoidal": E_bar_trapz,
            "difference_order": "O(Delta/T); discrete mean used as primary",
        },
        # Series retained for plotting / CSV (not written verbatim into summary).
        "_series": {
            "J": J.tolist(),
            "E": E.tolist(),
            "H": H.tolist(),
            "g2": g2.tolist(),
            "c2": c2.tolist(),
            "a_eff": a_eff.tolist(),
            "lambda_min": lambda_min.tolist(),
            "lambda_max": lambda_max.tolist(),
            "lambda_from_sat_mean": lambda_from_sat_mean.tolist(),
            "in_K0": in_K0.astype(int).tolist(),
        },
    }


# ---------------------------------------------------------------------------
# Plots: 3x3 grids for four quantities with discrete-average lines.
# ---------------------------------------------------------------------------
def make_grid_figures(cases: list[dict], out_dir: Path) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    quantities = [
        ("g2", "gradient residual $\\sum_i\\|g^\\star_i\\|^2$", "grad_residual"),
        ("c2", "centroid residual $\\sum_i\\|c^\\star_i-p_i\\|^2$", "centroid_residual"),
        ("J", "$J_k=\\sum_i m^\\star_i\\|U_i\\|^2$", "J"),
        ("E", "$E_k=\\sum_i m^\\star_i\\|\\hat c_i-c^\\star_i\\|^2$", "E"),
    ]
    written = []
    order = sorted(cases, key=lambda c: (["l_shape", "rectangle", "c_shape"].index(c["shape"]), c["seed"]))
    for key, label, fname in quantities:
        fig, axes = plt.subplots(3, 3, figsize=(13.5, 9.5), constrained_layout=True)
        for ax, case in zip(axes.ravel(), order):
            y = np.asarray(case["_series"][key], dtype=float)
            k = np.arange(len(y))
            ax.plot(k, y, lw=0.8, color="C0", label=f"{key}_k")
            mean_disc = float(np.mean(y))
            ax.axhline(mean_disc, color="r", ls="--", lw=1.2, label=f"discrete mean={mean_disc:.3e}")
            title = f"{case['shape']} seed {case['seed']}"
            if not case["theorem_conditions_hold_full_horizon"]:
                title += "  (K0 fails)"
                for f in case["condition_failure_frames"]:
                    ax.axvspan(f - 0.5, f + 0.5, color="orange", alpha=0.35)
            ax.set_title(title, fontsize=9)
            ax.set_yscale("log")
            ax.grid(alpha=0.2)
            ax.legend(fontsize=6, loc="upper right")
            ax.set_xlabel("control step k", fontsize=8)
        fig.suptitle(
            f"N=16 static theorem_mode | {label} | discrete average lines | static_oracle_theory_baseline",
            fontsize=12,
        )
        png = out_dir / f"nine_case_{fname}.png"
        pdf = out_dir / f"nine_case_{fname}.pdf"
        fig.savefig(png, dpi=150)
        fig.savefig(pdf)
        plt.close(fig)
        written.append(png.name)
        written.append(pdf.name)
    return written


def write_comparison_csv(path: Path, cases: list[dict]) -> None:
    fields = [
        "shape",
        "seed",
        "complete_30s",
        "theorem_conditions_hold_full_horizon",
        "full_horizon_J_bound_status",
        "condition_failure_frames",
        "condition_failure_agent_indices",
        "zero_input_with_rho_failure_count",
        "H0",
        "H_final",
        "J_bar",
        "E_bar",
        "posthoc_J_bound",
        "J_geometric_bound_M_umax2",
        "posthoc_bound_below_geom",
        "gradient_residual_bar",
        "centroid2_bar",
        "min_a_eff",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(fields)
        for c in cases:
            w.writerow(
                [
                    c["shape"],
                    c["seed"],
                    c["complete_30s"],
                    c["theorem_conditions_hold_full_horizon"],
                    c["full_horizon_J_bound_status"],
                    ";".join(str(x) for x in c["condition_failure_frames"]),
                    ";".join(str(x) for x in c["condition_failure_agent_indices"]),
                    c["zero_input_with_rho_failure_count"],
                    c["H0"],
                    c["H_final"],
                    c["J_bar"],
                    c["E_bar"],
                    c["posthoc_J_bound"],
                    c["J_geometric_bound_M_umax2"],
                    c["posthoc_bound_below_geom"],
                    c["gradient_residual_bar"],
                    c["centroid2_bar"],
                    c["min_a_eff"],
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=OUT_ROOT)
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    args = parser.parse_args()

    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    tch_dir = out / "three_command_hyp"
    jb_dir = out / "j_bound"
    fig_dir = out / "figures"
    tch_dir.mkdir(parents=True, exist_ok=True)
    jb_dir.mkdir(parents=True, exist_ok=True)

    # Authoritative complete_30s / abort come from the existing multi-shape table.
    mst = json.loads(
        (ARTIFACT_ROOT / "n16_multi_shape" / "multi_shape_table.json").read_text(encoding="utf-8")
    )
    complete_map = {(c["shape"], c["seed"]): c for c in mst["cases"]}

    from dbact_sim.scenarios import controller_params_from_config

    nine_cases = []
    tch_summary = {}
    for shape, meta in SHAPES.items():
        cfg = yaml.safe_load(meta["config"].read_text(encoding="utf-8"))
        params = controller_params_from_config(cfg)
        kc = float(params.kp_cage)
        umax = float(params.max_speed)
        dt = float(cfg["dt"])
        N = int(cfg["agents"]["count"])
        vertices = cargo_vertices(cfg)
        consts = theoretical_constants(cfg, vertices)
        observer, fine = build_observers(cfg, args.ntheta, args.nradial)
        M_total = observer.total_mass(vertices)

        for seed in args.seeds:
            seed_dir = meta["data_dir"] / f"seed_{seed}"
            rec_path = seed_dir / "step_records.json"
            if not rec_path.exists():
                print(f"[warn] missing {rec_path}", flush=True)
                continue
            step_records = json.loads(rec_path.read_text(encoding="utf-8"))

            # --- Task A: fixed three-command hypothetical dissipation check. ---
            hyp = three_command_hyp(step_records, vertices, observer, fine, dt)
            dump(tch_dir / f"{shape}_seed_{seed}.json", hyp)
            tch_summary[f"{shape}_seed_{seed}"] = {
                stage: {
                    "n_anomalies": rep["coarse_verification"]["n_anomalies"],
                    "max_slack_coarse": rep["coarse_verification"]["max_slack"],
                    "max_fine_slack_over_anomalies": rep["refined_verification"][
                        "max_fine_slack_over_anomalies"
                    ],
                    "max_slack_refined_full_horizon": rep["refined_verification"][
                        "max_slack_refined_full_horizon"
                    ],
                    "strict_numerical_certificate": rep["unresolved_quadrature_error"][
                        "strict_numerical_certificate"
                    ],
                    "numerical_consistency": rep["numerical_consistency"],
                }
                for stage, rep in hyp["stages"].items()
            }

            # --- Task B: finite-time J bound with per-robot lambda_i. ---
            ftb = finite_time_bound(step_records, kc, umax, dt, N, M_total)
            comp = complete_map.get((shape, seed), {})
            # C-channel seed 5: force the documented inapplicable verdict.
            if shape == "c_shape" and seed == 5:
                assert ftb["condition_failure_frames"] == [241, 242, 243, 244, 245, 246], (
                    f"unexpected c_shape seed 5 failure frames: {ftb['condition_failure_frames']}"
                )
                assert ftb["condition_failure_agent_indices"] == [14]
                ftb["theorem_conditions_hold_full_horizon"] = False
                ftb["full_horizon_J_bound_status"] = "inapplicable"
                ftb["posthoc_J_bound"] = None
                ftb["posthoc_bound_below_geom"] = None
                ftb["descriptive_stats_label"] = "not_covered_by_theorem"

            series = ftb.pop("_series")
            # Per-case detailed J-bound record (lambda diagnostics + a_eff series).
            dump(
                jb_dir / f"{shape}_seed_{seed}.json",
                {
                    "shape": shape,
                    "seed": seed,
                    **{k: v for k, v in ftb.items()},
                    "series": series,
                },
            )

            case = {
                "shape": shape,
                "seed": seed,
                "complete_30s": bool(comp.get("complete_30s", True)),
                "abort": comp.get("abort"),
                "theorem_conditions_hold_full_horizon": ftb[
                    "theorem_conditions_hold_full_horizon"
                ],
                "condition_failure_frames": ftb["condition_failure_frames"],
                "condition_failure_agent_indices": ftb["condition_failure_agent_indices"],
                "zero_input_with_rho_failure_count": ftb["zero_input_with_rho_failure_count"],
                "H0": ftb["H0"],
                "H_final": ftb["H_final"],
                "J_bar": ftb["J_bar"],
                "E_bar": ftb["E_bar"],
                "E_bar_estimate_type": "post_hoc_trajectory",
                "posthoc_J_bound": ftb["posthoc_J_bound"],
                "J_geometric_bound_M_umax2": ftb["J_geometric_bound_M_umax2"],
                "posthoc_bound_below_geom": ftb["posthoc_bound_below_geom"],
                "gradient_residual_bar": ftb["gradient_residual_bar"],
                "centroid2_bar": ftb["centroid2_bar"],
                "full_horizon_J_bound_status": ftb["full_horizon_J_bound_status"],
                "min_a_eff": ftb["min_a_eff"],
                "a_global": ftb["a_global"],
                "M_total": ftb["M_total"],
                "descriptive_stats_label": ftb["descriptive_stats_label"],
                "u_command_max_fine_slack": tch_summary[f"{shape}_seed_{seed}"]["u_command"][
                    "max_fine_slack_over_anomalies"
                ],
                "u_command_max_slack_refined": tch_summary[f"{shape}_seed_{seed}"]["u_command"][
                    "max_slack_refined_full_horizon"
                ],
                "_series": series,
            }
            nine_cases.append(case)
            print(
                f"[{shape} seed {seed}] full_horizon="
                f"{case['theorem_conditions_hold_full_horizon']} "
                f"status={case['full_horizon_J_bound_status']} "
                f"J_bar={case['J_bar']:.4e} posthocJ={case['posthoc_J_bound']} "
                f"Jgeom={case['J_geometric_bound_M_umax2']:.4e} "
                f"u_cmd_max_fine_slack={case['u_command_max_fine_slack']}",
                flush=True,
            )

    # Three-command summary rollup.
    stage_rollup = {}
    for stage in ("u_cvt", "u_saturated", "u_command"):
        fine_vals = [
            v[stage]["max_fine_slack_over_anomalies"]
            for v in tch_summary.values()
            if v[stage]["max_fine_slack_over_anomalies"] is not None
        ]
        refined_vals = [v[stage]["max_slack_refined_full_horizon"] for v in tch_summary.values()]
        stage_rollup[stage] = {
            "max_fine_slack_over_all_anomalies": float(max(fine_vals)) if fine_vals else None,
            "max_slack_refined_full_horizon_over_cases": float(max(refined_vals)),
            "all_numerical_consistency": all(
                v[stage]["numerical_consistency"] for v in tch_summary.values()
            ),
            "all_strict_numerical_certificate": all(
                v[stage]["strict_numerical_certificate"] for v in tch_summary.values()
            ),
        }
    dump(
        tch_dir / "three_command_hyp_summary.json",
        {
            "description": (
                "Fixed hypothetical-next-state discrete dissipation delta_H_hyp - rhs "
                "for U in {u_cvt,u_saturated,u_command}; P_next_hyp = P + dt U per stage. "
                "On refinement H, g AND m all come from the same fine observer. "
                "strict_numerical_certificate is always false "
                "(reason=no_rigorous_quadrature_error_bound)."
            ),
            "per_case": tch_summary,
            "rollup": stage_rollup,
        },
    )

    # Figures + CSV.
    figs = make_grid_figures(nine_cases, fig_dir)

    # Strip heavy series before writing the top-level summary.
    for c in nine_cases:
        c.pop("_series", None)
    write_comparison_csv(out / "nine_case_comparison.csv", nine_cases)

    summary = {
        "experiment_class": "static_oracle_theory_baseline",
        "not_a_certificate_for": "unknown-object transport",
        "seeds": args.seeds,
        "observer_quadrature": {"ntheta": args.ntheta, "nradial": args.nradial},
        "conventions": {
            "lambda_i": "min(k_c, u_max/||chat_i - p_i||); k_c when idle",
            "a": "a = 2/k_c - Delta (global); a_i = 2/lambda_i - Delta >= a since lambda_i <= k_c",
            "posthoc_J_bound": "2 H0/(a K Delta) + 4 E_bar/a^2, E_bar post_hoc, full-horizon K_0 only",
            "J_geometric_bound": "M_total * u_max^2, M_total = observer.total_mass(vertices)",
            "K_0": "steps with all solver optimal AND all zero_input_feasible_with_rho AND a>0",
        },
        "averaging_note": (
            "Discrete average = (1/K) sum_k J_k. Continuous-time average ~ trapezoidal/T "
            "differs by O(Delta/T); per-case j_bound/*.json reports both. Discrete mean is "
            "used as the primary statistic."
        ),
        "cases": nine_cases,
        "roll_up": {
            "n_cases": len(nine_cases),
            "complete_30s": sum(1 for c in nine_cases if c["complete_30s"]),
            "full_horizon_applicable": sum(
                1 for c in nine_cases if c["theorem_conditions_hold_full_horizon"]
            ),
            "full_horizon_applicable_cases": [
                f"{c['shape']}_seed_{c['seed']}"
                for c in nine_cases
                if c["theorem_conditions_hold_full_horizon"]
            ],
            "inapplicable_cases": [
                f"{c['shape']}_seed_{c['seed']}"
                for c in nine_cases
                if not c["theorem_conditions_hold_full_horizon"]
            ],
            "all_posthoc_below_geom": all(
                c["posthoc_bound_below_geom"]
                for c in nine_cases
                if c["posthoc_bound_below_geom"] is not None
            ),
        },
        "figures": figs,
    }
    dump(out / "nine_case_summary.json", summary)
    print(json.dumps(summary["roll_up"], indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ.setdefault(_k, "1")
    main()
