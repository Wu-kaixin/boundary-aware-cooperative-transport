"""Offline J/E verification and three-bound comparison for static theorem_mode.

Reprocesses the existing N=16 static discrete-dissipation artifacts
(``n16_discrete_dissipation/seed_*/{step_records.json,trajectory.npz}``) without
re-simulating, and adds two things the earlier audit was missing:

Task A -- three-command hypothetical-next-state dissipation check
==================================================================
For every control step and every command stage ``U in {u_cvt, u_saturated,
u_command}`` we evaluate the *hypothetical* one-step map

    P_next_hyp = P + Delta * U          (NOT the applied next state unless U == u_command)
    delta_H_hyp = H*(P_next_hyp) - H*(P)
    rhs = Delta g*^T U + Delta^2 sum_i m*_i ||U_i||^2
    slack = delta_H_hyp - rhs

The earlier ``audit_discrete_dissipation.py`` compared rhs(U_cvt) and
rhs(U_saturated) against the *applied* delta_H (the u_command trajectory), which
is not the correct discrete inequality for those stages.  Here every stage is
propagated through its own hold map with the independent UniformOffsetObserver.
Anomalous frames (slack > tol) are refined with a fine (1024/32) observer, and
we separate ``proof_status`` / ``numerical_consistency`` /
``strict_numerical_certificate``.

Task B -- J/E metrics and three comparison blocks
==================================================
Mass-weighted, with U = final command:

    J_k = sum_i m*_i,k ||U_i,k||^2
    E_k = sum_i m*_i,k ||chat_i,k - c*_i,k||^2      (chat = controller grid centroid)
    a   = 2/k_c - Delta                              (require a > 0)
    J_bar = (1/K) sum_k J_k ,  E_bar = (1/K) sum_k E_k

Candidate post-hoc bound (E_bar taken from the trajectory, so NOT a priori):

    J_bar <= 2 H*(P_0) / (a K Delta) + 4 E_bar / a^2

Young derivation sketch (comments only): for the unsaturated exact-direction
case U_i = k_c (chat_i - p_i), the discrete descent inequality carries an error
cross term 2 Delta sqrt(E J); Young's inequality 2 sqrt(EJ) <= (a/2) J + (2/a) E
absorbs it and, after summing and dividing by K Delta, yields the displayed
bound.  For the final QP command U we only *evaluate* the descent inequality
numerically on steps where the qualifying conditions hold (optimal solver +
zero_input_feasible_with_rho); the closed-form bound is reported separately as
an unconditional post-hoc number.  This is NOT the original gradient-residual
time average ||g*||^2.

Nothing here loosens a failure threshold or turns a measured quantity into an a
priori guarantee.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from audit_n16_static_performance import (  # noqa: E402
    UniformOffsetObserver,
    environment_snapshot,
    theoretical_constants,
)

# Numerical tolerances for the discrete inequality (shared with dissipation audit).
DISSIPATION_TOL_ABS = 1e-6
DISSIPATION_TOL_REL = 1e-6
STRICT_CERT_ABS = 1e-9


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def build_observers(cfg: dict, ntheta: int, nradial: int):
    """Coarse (audit) and fine (refinement) observers matching the controller model."""
    from dbact_sim.scenarios import controller_params_from_config

    p = controller_params_from_config(cfg)
    domain = [
        cfg["domain"]["xmin"],
        cfg["domain"]["xmax"],
        cfg["domain"]["ymin"],
        cfg["domain"]["ymax"],
    ]
    coarse = UniformOffsetObserver(
        domain, p.local_radius, p.sigma, p.base_density, p.cage_offset, ntheta, nradial
    )
    fine = UniformOffsetObserver(
        domain, p.local_radius, p.sigma, p.base_density, p.cage_offset, 1024, 32
    )
    return coarse, fine


def dissipation_rhs(dt: float, gradient: np.ndarray, mass: np.ndarray, U: np.ndarray) -> float:
    g_dot = float(np.sum(gradient * U))
    mass_term = float(dt * dt * np.sum(mass * np.linalg.norm(U, axis=1) ** 2))
    return float(dt * g_dot + mass_term)


# ---------------------------------------------------------------------------
# Loading of existing offline artifacts
# ---------------------------------------------------------------------------
def load_case(seed_dir: Path) -> tuple[list[dict], np.ndarray]:
    step_records = json.loads((seed_dir / "step_records.json").read_text(encoding="utf-8"))
    traj = np.load(seed_dir / "trajectory.npz")
    vertices = traj["vertices"]
    return step_records, vertices


# ---------------------------------------------------------------------------
# Task A -- three-command hypothetical next-state dissipation
# ---------------------------------------------------------------------------
def three_command_hyp(
    step_records: list[dict],
    vertices: np.ndarray,
    observer: UniformOffsetObserver,
    fine: UniformOffsetObserver,
    dt: float,
) -> dict:
    """Evaluate delta_H_hyp - rhs for each command stage on its own hold map."""
    stages = ("u_cvt", "u_saturated", "u_command")
    rows: list[dict] = []
    command_map_consistency = []  # |H*(P+dt U_cmd) - stored H*_next|

    for rec in step_records:
        P = np.asarray(rec["positions"], dtype=float)
        g = np.asarray(rec["gradient_star"], dtype=float)
        m = np.asarray(rec["mass_star"], dtype=float)
        H_P = float(rec["H_star"])
        entry = {"frame": rec["frame"], "time": rec["time"], "H_star": H_P}
        for stage in stages:
            U = np.asarray(rec[stage], dtype=float)
            P_next_hyp = P + dt * U
            H_hyp = float(observer.evaluate(P_next_hyp, vertices)["H"])
            delta_H_hyp = H_hyp - H_P
            rhs = dissipation_rhs(dt, g, m, U)
            slack = delta_H_hyp - rhs
            entry[stage] = {
                "delta_H_hyp": delta_H_hyp,
                "rhs": rhs,
                "slack": slack,
            }
            if stage == "u_command":
                command_map_consistency.append(abs(H_hyp - float(rec["H_star_next"])))
        rows.append(entry)

    # Per-stage aggregate + anomaly refinement with the fine observer.
    stage_reports = {}
    for stage in stages:
        slacks = np.asarray([r[stage]["slack"] for r in rows], dtype=float)
        rhs_arr = np.asarray([r[stage]["rhs"] for r in rows], dtype=float)
        dH_arr = np.asarray([r[stage]["delta_H_hyp"] for r in rows], dtype=float)
        tol = np.maximum(
            DISSIPATION_TOL_ABS,
            DISSIPATION_TOL_REL * np.maximum.reduce([np.abs(rhs_arr), np.abs(dH_arr)]),
        )
        anomalies = [int(r["frame"]) for r, t, s in zip(rows, tol, slacks) if s > t]

        refinements = []
        for frame in anomalies:
            rec = step_records[frame]
            P = np.asarray(rec["positions"], dtype=float)
            U = np.asarray(rec[stage], dtype=float)
            P_next_hyp = P + dt * U
            # Same-observer refinement: H, g AND m are ALL taken from a single fine
            # observer evaluation.  The earlier code reused the coarse-record mass m
            # while taking H and g from the fine observer, mixing two quadratures in
            # one inequality (fixed here).
            fk = fine.evaluate(P, vertices)
            fn = fine.evaluate(P_next_hyp, vertices)
            dH_fine = float(fn["H"] - fk["H"])
            rhs_fine = dissipation_rhs(dt, fk["gradient"], fk["mass"], U)
            refinements.append(
                {
                    "frame": frame,
                    "coarse_slack": float(rows[frame][stage]["slack"]),
                    "fine_delta_H_hyp": dH_fine,
                    "fine_rhs": rhs_fine,
                    "fine_slack": float(dH_fine - rhs_fine),
                    "observer": "fine (H,g,m all from same fine observer)",
                }
            )
        # Refined worst-case slack: replace coarse anomalies by their fine value.
        refined_slacks = slacks.copy()
        for ref in refinements:
            refined_slacks[ref["frame"]] = ref["fine_slack"]
        refined_max_slack = float(np.max(refined_slacks)) if len(refined_slacks) else None
        max_fine_slack = (
            float(max(ref["fine_slack"] for ref in refinements)) if refinements else None
        )

        numerical_consistency = bool(np.all(slacks <= tol))
        # Three explicit verification layers (no fixed-residual certificate claim).
        stage_reports[stage] = {
            "n_steps": int(len(slacks)),
            "coarse_verification": {
                "observer": "coarse (H, g, m as stored / coarse observer)",
                "max_slack": float(np.max(slacks)) if len(slacks) else None,
                "mean_slack": float(np.mean(slacks)) if len(slacks) else None,
                "fraction_positive": float(np.mean(slacks > 0)) if len(slacks) else None,
                "n_anomalies": len(anomalies),
            },
            "refined_verification": {
                "observer": "fine (H, g and m ALL re-evaluated on the same fine observer)",
                "note": (
                    "Anomalous frames (coarse slack > tol) are re-evaluated with one fine "
                    "observer so that H, g and m come from the same quadrature."
                ),
                "n_refined": len(refinements),
                "max_fine_slack_over_anomalies": max_fine_slack,
                "max_slack_refined_full_horizon": refined_max_slack,
                "refinements": refinements,
            },
            "unresolved_quadrature_error": {
                "strict_numerical_certificate": False,
                "reason": "no_rigorous_quadrature_error_bound",
                "note": (
                    "Without a rigorous integration (quadrature) error bound on H*, a "
                    "positive or near-zero refined slack cannot be promoted to a strict "
                    "numerical certificate of the discrete inequality."
                ),
            },
            "proof_status": (
                "not_established: numerical evaluation of the discrete descent "
                "inequality on the hypothetical hold map; assumptions for Theorem 1 "
                "(Lipschitz feedback, fine-grid mass certificate) remain unmet"
            ),
            "numerical_consistency": numerical_consistency,
            # Back-compat flat fields for existing consumers (multi-shape table, rollup).
            "max_slack_coarse": float(np.max(slacks)) if len(slacks) else None,
            "mean_slack_coarse": float(np.mean(slacks)) if len(slacks) else None,
            "fraction_positive": float(np.mean(slacks > 0)) if len(slacks) else None,
            "n_anomalies": len(anomalies),
            "max_slack_refined": refined_max_slack,
            "refinements": refinements,
            # Never promoted from a fixed residual threshold.
            "strict_numerical_certificate": False,
            "strict_numerical_certificate_reason": "no_rigorous_quadrature_error_bound",
        }

    return {
        "dt": dt,
        "observer_quadrature": {
            "ntheta": len(observer.unit),
            "nradial": len(observer.radial_nodes),
        },
        "stages": stage_reports,
        "command_hold_map_consistency": {
            "max_abs_H_diff_vs_stored_next": float(np.max(command_map_consistency))
            if command_map_consistency
            else None,
            "note": (
                "H*(P + dt u_command) compared with the stored applied H*_next; a small "
                "value confirms the applied hold map equals P + dt U (no clipping)."
            ),
        },
        "tolerances": {
            "abs": DISSIPATION_TOL_ABS,
            "rel": DISSIPATION_TOL_REL,
            "strict_certificate_abs": STRICT_CERT_ABS,
            "strict_certificate_note": (
                "strict_certificate_abs is retained only as a legacy anomaly-detection "
                "scale; it is NOT used to grant a strict numerical certificate. "
                "strict_numerical_certificate is always false "
                "(reason=no_rigorous_quadrature_error_bound)."
            ),
        },
    }


# ---------------------------------------------------------------------------
# Task B -- J / E series and bounds
# ---------------------------------------------------------------------------
def je_series(step_records: list[dict]) -> dict:
    """Per-step mass-weighted J_k, E_k, H*_k and gradient residual ||g*||^2."""
    J, E, H, g2 = [], [], [], []
    for rec in step_records:
        m = np.asarray(rec["mass_star"], dtype=float)
        U = np.asarray(rec["u_command"], dtype=float)
        c_hat = np.asarray(rec["cell_centroid"], dtype=float)
        c_star = np.asarray(rec["centroid_star"], dtype=float)
        g = np.asarray(rec["gradient_star"], dtype=float)
        J.append(float(np.sum(m * np.sum(U * U, axis=1))))
        E.append(float(np.sum(m * np.sum((c_hat - c_star) ** 2, axis=1))))
        H.append(float(rec["H_star"]))
        g2.append(float(np.sum(g * g)))
    return {
        "J": np.asarray(J),
        "E": np.asarray(E),
        "H": np.asarray(H),
        "g2": np.asarray(g2),
    }


def cumulative_mean(x: np.ndarray) -> np.ndarray:
    return np.cumsum(x) / np.arange(1, len(x) + 1)


def compute_bounds(series: dict, consts: dict, kc: float, dt: float, N: int) -> dict:
    J = series["J"]
    E = series["E"]
    H = series["H"]
    g2 = series["g2"]
    K = int(len(J))
    a = 2.0 / kc - dt
    if a <= 0:
        raise ValueError(f"a = 2/k_c - Delta = {a} must be positive")

    H0 = float(H[0]) if K else float("nan")
    J_bar = float(np.mean(J)) if K else float("nan")
    E_bar = float(np.mean(E)) if K else float("nan")
    g2_bar = float(np.mean(g2)) if K else float("nan")

    # Candidate analytic bound (post-hoc: uses E_bar measured from trajectory).
    posthoc_J_bound = 2.0 * H0 / (a * K * dt) + 4.0 * E_bar / (a * a)

    # Crude geometric J upper bound: J_k = sum m*_i ||U_i||^2 <= (sum m*_i) u_max^2.
    # We over-bound sum m*_i by N * m_plus (per-cell geometric mass ceiling) so the
    # number is a priori (no measured mass).  Also report a total-mass variant.
    u_max = consts["umax"]
    m_plus = consts["m_plus"]
    M_total = consts["M_total_upper"]
    J_geom_cellwise = N * m_plus * u_max * u_max
    J_geom_totalmass = M_total * u_max * u_max

    # E threshold at which the analytic J bound meets the geometric J bound.
    # 2H0/(aKD) + 4 E*/a^2 = J_geom  ->  E* = (J_geom - 2H0/(aKD)) * a^2 / 4
    def e_star(j_geom: float) -> float:
        return (j_geom - 2.0 * H0 / (a * K * dt)) * a * a / 4.0

    E_star_cellwise = e_star(J_geom_cellwise)
    E_star_totalmass = e_star(J_geom_totalmass)

    # Only a priori E bound derivable from the model: both chat and c* are centroids
    # inside the reach region (radius <= R) of agent i, so ||chat-c*|| <= 2R, and
    # sum m*_i <= M_total, giving E <= 4 R^2 M_total.  This is a diameter bound,
    # independent of the 20x20 grid resolution; it is NOT a sampling/discretization
    # certificate.
    R = consts["R"]
    E_trivial_diameter = 4.0 * R * R * M_total

    return {
        "K": K,
        "dt": dt,
        "k_c": kc,
        "a": a,
        "H0": H0,
        "J_bar": J_bar,
        "E_bar": E_bar,
        "gradient_residual_bar_sum": g2_bar,
        "gradient_residual_bar_per_agent": g2_bar / N,
        "posthoc_J_bound": posthoc_J_bound,
        "posthoc_J_bound_terms": {
            "term_2H0_over_aKD": 2.0 * H0 / (a * K * dt),
            "term_4Ebar_over_a2": 4.0 * E_bar / (a * a),
        },
        "J_geom_bound_cellwise_Nmplus_umax2": J_geom_cellwise,
        "J_geom_bound_totalmass_umax2": J_geom_totalmass,
        "E_star_threshold_cellwise": E_star_cellwise,
        "E_star_threshold_totalmass": E_star_totalmass,
        "E_trivial_diameter_bound_4R2M": E_trivial_diameter,
        "posthoc_J_bound_below_geom_cellwise": bool(posthoc_J_bound <= J_geom_cellwise),
        "J_bar_below_geom_cellwise": bool(J_bar <= J_geom_cellwise),
    }


def a_priori_conclusion(bounds: dict, consts: dict) -> dict:
    """Explicit NEGATIVE conclusion on an a priori centroid-error certificate.

    Forbidden shortcut (trajectory max E as a uniform bound) is not used.  The
    only model-derivable bound is the reach-diameter bound E <= 4 R^2 M, which is
    independent of grid resolution and therefore does not certify sampling
    accuracy.
    """
    E_star = bounds["E_star_threshold_cellwise"]
    E_triv = bounds["E_trivial_diameter_bound_4R2M"]
    return {
        "a_priori_error_bound_sufficient": False,
        "E_star_threshold_cellwise": E_star,
        "only_model_derivable_E_bound": {
            "value": E_triv,
            "derivation": "chat and c* both lie in the reach disk (radius <= R) of "
            "agent i, so ||chat-c*|| <= 2R; sum m*_i <= M_total; hence E <= 4 R^2 M.",
            "resolution_dependent": False,
            "numerically_below_E_star": bool(E_triv <= E_star),
        },
        "reason": (
            "No resolution-dependent a priori bound on the grid-truncation centroid "
            "error ||chat_grid - c*_continuous|| was derived from the 20x20 grid, the "
            "Gaussian offset kernel, or R. The only model-derivable bound is the "
            "trivial reach-diameter bound E <= 4 R^2 M, which (i) is independent of the "
            "grid resolution and so certifies nothing about the oracle/CVT sampling "
            "accuracy, and (ii) reuses the same loose geometric mass ceiling. It is "
            "therefore not accepted as a genuine a priori centroid-error certificate. "
            "Trajectory-measured E is explicitly NOT used as a uniform a priori bound."
        ),
        "estimate_type_of_E_bar": "post_hoc_trajectory",
    }


def build_comparisons(per_seed: list[dict], consts: dict, prior_bound_comparison: list | None) -> dict:
    """Three comparison blocks: gradient residual, J, centroid residual."""
    prior_map = {}
    if prior_bound_comparison:
        for row in prior_bound_comparison:
            prior_map[int(row["seed"])] = row

    gradient_block = {
        "definition": "||g*||^2 = sum over agents of gradient_star . gradient_star "
        "(observer truncated-cell gradient); matches prior n16_static_audit.",
        "theorem1_static_formula": consts["B_gradient_static"],
        "geometric_bound": consts["B_gradient_geometry_total"],
        "geometric_bound_note": "4 R^2 M^2 (total-mass gradient geometry ceiling).",
        "measured_time_average_per_seed": [
            {
                "seed": s["seed"],
                "gradient_residual_bar_sum": s["bounds"]["gradient_residual_bar_sum"],
                "gradient_residual_bar_per_agent": s["bounds"]["gradient_residual_bar_per_agent"],
                "prior_trapezoid_time_average": prior_map.get(s["seed"], {}).get(
                    "observed_gradient2_time_average"
                ),
            }
            for s in per_seed
        ],
        "formula_is_certificate": False,
    }

    J_block = {
        "definition": "J_k = sum_i m*_i ||U_i||^2 with U = final QP command; "
        "J_bar = (1/K) sum_k J_k.",
        "analytic_bound_formula": "2 H0 / (a K Delta) + 4 E_bar / a^2",
        "analytic_bound_uses": "E_bar (post_hoc_trajectory estimate)",
        "geometric_bound_cellwise": {
            "formula": "N * m_plus * u_max^2",
            "value": per_seed[0]["bounds"]["J_geom_bound_cellwise_Nmplus_umax2"],
        },
        "geometric_bound_totalmass": {
            "formula": "M_total * u_max^2",
            "value": per_seed[0]["bounds"]["J_geom_bound_totalmass_umax2"],
        },
        "per_seed": [
            {
                "seed": s["seed"],
                "H0": s["bounds"]["H0"],
                "J_bar_measured": s["bounds"]["J_bar"],
                "posthoc_J_bound": s["bounds"]["posthoc_J_bound"],
                "posthoc_J_bound_terms": s["bounds"]["posthoc_J_bound_terms"],
                "E_bar_estimate_type": "post_hoc_trajectory",
                "posthoc_J_bound_below_geom_cellwise": s["bounds"][
                    "posthoc_J_bound_below_geom_cellwise"
                ],
            }
            for s in per_seed
        ],
    }

    centroid_block = {
        "definition": "E_k = sum_i m*_i ||chat_i - c*_i||^2 (controller grid centroid "
        "vs observer centroid); E_bar = (1/K) sum_k E_k.",
        "estimate_type": "post_hoc_trajectory",
        "per_seed": [
            {"seed": s["seed"], "E_bar": s["bounds"]["E_bar"]} for s in per_seed
        ],
    }

    return {
        "gradient_residual_block": gradient_block,
        "J_block": J_block,
        "centroid_residual_block": centroid_block,
    }


def write_series_csv(path: Path, series: dict) -> None:
    J, E, H, g2 = series["J"], series["E"], series["H"], series["g2"]
    Jc, Ec, g2c = cumulative_mean(J), cumulative_mean(E), cumulative_mean(g2)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(
            [
                "frame",
                "H_star",
                "J_k",
                "E_k",
                "grad_residual_k",
                "J_cumavg",
                "E_cumavg",
                "grad_residual_cumavg",
            ]
        )
        for k in range(len(J)):
            w.writerow([k, H[k], J[k], E[k], g2[k], Jc[k], Ec[k], g2c[k]])


def make_plot(out: Path, all_series: dict, bounds_by_seed: dict, consts: dict) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), constrained_layout=True)
    for seed, series in all_series.items():
        J, E, g2 = series["J"], series["E"], series["g2"]
        k = np.arange(1, len(J) + 1)
        axes[0].plot(k, cumulative_mean(J), label=f"seed {seed}")
        axes[1].plot(k, cumulative_mean(E), label=f"seed {seed}")
        axes[2].plot(k, cumulative_mean(g2), label=f"seed {seed}")
    b0 = next(iter(bounds_by_seed.values()))
    axes[0].axhline(b0["J_geom_bound_cellwise_Nmplus_umax2"], color="k", ls=":", label="geom N m+ u^2")
    for seed, b in bounds_by_seed.items():
        axes[0].axhline(b["posthoc_J_bound"], color="r", ls="--", alpha=0.4)
    axes[0].set_title("J cumulative mean (+ bounds)")
    axes[0].set_yscale("log")
    axes[1].set_title("E cumulative mean (post-hoc)")
    axes[1].set_yscale("log")
    axes[2].axhline(consts["B_gradient_geometry_total"], color="k", ls=":", label="geom")
    axes[2].set_title("||g*||^2 cumulative mean")
    axes[2].set_yscale("log")
    for ax in axes:
        ax.set_xlabel("control step k")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=7)
    fig.suptitle("N=16 static theorem_mode: J / E / gradient residual (post-hoc E)")
    fig.savefig(out / "JE_bounds.png", dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def process_seed(
    seed: int,
    seed_dir: Path,
    vertices: np.ndarray,
    consts: dict,
    observer: UniformOffsetObserver,
    fine: UniformOffsetObserver,
    kc: float,
    dt: float,
    N: int,
    run_task_a: bool,
) -> dict:
    step_records, vtx = load_case(seed_dir)
    if vertices is None:
        vertices = vtx
    series = je_series(step_records)
    bounds = compute_bounds(series, consts, kc, dt, N)
    result = {"seed": seed, "series": series, "bounds": bounds, "step_records": step_records}
    if run_task_a:
        result["three_command_hyp"] = three_command_hyp(
            step_records, vertices, observer, fine, dt
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/sim/theorem/static_l_shape_n16_oracle.yaml",
    )
    parser.add_argument(
        "--dissipation-dir",
        type=Path,
        default=Path(
            r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\n16_discrete_dissipation"
        ),
    )
    parser.add_argument(
        "--static-audit-dir",
        type=Path,
        default=Path(
            r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\n16_static_audit"
        ),
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(
            r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\n16_JE_bounds"
        ),
    )
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    parser.add_argument("--skip-task-a", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    from dbact_sim.scenarios import build_cargoes, controller_params_from_config

    cargo = build_cargoes(cfg, seed=0)[0]
    consts = theoretical_constants(cfg, cargo.vertices)
    params = controller_params_from_config(cfg)
    kc = float(params.kp_cage)
    dt = float(cfg["dt"])
    N = int(cfg["agents"]["count"])

    observer, fine = build_observers(cfg, args.ntheta, args.nradial)

    prior_bc = None
    prior_bc_path = args.static_audit_dir / "bound_comparison.json"
    if prior_bc_path.exists():
        prior_bc = json.loads(prior_bc_path.read_text(encoding="utf-8"))

    dump(args.out / "constants.json", consts)
    dump(args.out / "environment_snapshot.json", environment_snapshot(ROOT))

    per_seed = []
    all_series = {}
    bounds_by_seed = {}
    task_a_dir = args.dissipation_dir / "three_command_hyp"
    task_a_dir.mkdir(parents=True, exist_ok=True)
    task_a_aggregate = {}

    for seed in args.seeds:
        seed_dir = args.dissipation_dir / f"seed_{seed}"
        if not (seed_dir / "step_records.json").exists():
            print(f"[warn] missing step_records for seed {seed} at {seed_dir}", flush=True)
            continue
        result = process_seed(
            seed, seed_dir, None, consts, observer, fine, kc, dt, N, not args.skip_task_a
        )
        per_seed.append(result)
        all_series[seed] = result["series"]
        bounds_by_seed[seed] = result["bounds"]
        write_series_csv(args.out / f"je_series_seed_{seed}.csv", result["series"])
        if "three_command_hyp" in result:
            dump(task_a_dir / f"seed_{seed}.json", result["three_command_hyp"])
            task_a_aggregate[str(seed)] = {
                stage: {
                    "max_slack_coarse": rep["max_slack_coarse"],
                    "max_slack_refined": rep["max_slack_refined"],
                    "n_anomalies": rep["n_anomalies"],
                    "numerical_consistency": rep["numerical_consistency"],
                    "strict_numerical_certificate": rep["strict_numerical_certificate"],
                }
                for stage, rep in result["three_command_hyp"]["stages"].items()
            }
        b = result["bounds"]
        print(
            f"seed {seed}: H0={b['H0']:.5f} J_bar={b['J_bar']:.4e} E_bar={b['E_bar']:.4e} "
            f"posthocJ={b['posthoc_J_bound']:.4e} Jgeom={b['J_geom_bound_cellwise_Nmplus_umax2']:.4e} "
            f"E*={b['E_star_threshold_cellwise']:.4e}",
            flush=True,
        )

    if not per_seed:
        raise SystemExit("no seeds processed")

    # Task A aggregate across seeds.
    if task_a_aggregate:
        stage_rollup = {}
        for stage in ("u_cvt", "u_saturated", "u_command"):
            refined = [v[stage]["max_slack_refined"] for v in task_a_aggregate.values()]
            stage_rollup[stage] = {
                "max_slack_refined_over_seeds": float(max(refined)),
                "all_numerical_consistency": all(
                    v[stage]["numerical_consistency"] for v in task_a_aggregate.values()
                ),
                "all_strict_numerical_certificate": all(
                    v[stage]["strict_numerical_certificate"] for v in task_a_aggregate.values()
                ),
            }
        dump(
            task_a_dir / "three_command_hyp_summary.json",
            {
                "description": (
                    "Hypothetical-next-state discrete dissipation delta_H_hyp - rhs for "
                    "U in {u_cvt,u_saturated,u_command}; P_next_hyp = P + dt U per stage."
                ),
                "per_seed": task_a_aggregate,
                "rollup": stage_rollup,
            },
        )

    # Task B outputs.
    comparisons = build_comparisons(per_seed, consts, prior_bc)
    apc = a_priori_conclusion(per_seed[0]["bounds"], consts)

    bounds_summary = {
        "config": str(args.config),
        "K": per_seed[0]["bounds"]["K"],
        "dt": dt,
        "k_c": kc,
        "a": per_seed[0]["bounds"]["a"],
        "N": N,
        "per_seed_bounds": [
            {k: v for k, v in s["bounds"].items()} for s in per_seed
        ],
        "a_priori_conclusion": apc,
        "labels": {
            "experiment_class": "static_oracle_theory_baseline",
            "E_bar_estimate_type": "post_hoc_trajectory",
            "posthoc_J_bound_estimate_type": "post_hoc_trajectory",
            "warning": "post-hoc bound; NOT a certificate for unknown-object transport.",
        },
    }

    dump(args.out / "comparisons.json", comparisons)
    dump(args.out / "bounds_summary.json", bounds_summary)
    dump(args.out / "a_priori_conclusion.json", apc)
    make_plot(args.out, all_series, bounds_by_seed, consts)

    print(json.dumps(bounds_summary, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
