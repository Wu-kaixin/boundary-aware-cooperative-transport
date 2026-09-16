"""Full-step N=16 static audit with offline discrete dissipation verification.

Runs every control step (600), records all command stages, evaluates the
independent UniformOffsetObserver at every step, and checks
    delta_H - (Delta g*^T U + Delta^2 sum m* ||U||^2) <= tol
for U in {u_cvt, u_saturated, u_command}. Also audits QP projection tiers.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import subprocess
import sys
import time
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

DISSIPATION_TOL_ABS = 1e-6
DISSIPATION_TOL_REL = 1e-6
# Polar-observer slack of size ~1e-5 is a known numerical envelope, not a
# theorem counterexample and not a reason to dump multi-megabyte traces.
OBSERVER_SLACK_DUMP = 1e-3
PROJECTION_RESIDUAL_TOL = 1e-6


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def dissipation_rhs(dt: float, gradient: np.ndarray, mass: np.ndarray, U: np.ndarray) -> float:
    g_dot = float(np.sum(gradient * U))
    mass_term = float(dt * dt * np.sum(mass * np.linalg.norm(U, axis=1) ** 2))
    return float(dt * g_dot + mass_term)


def decomposition_terms(
    positions: np.ndarray,
    centroid_hat: np.ndarray,
    centroid_star: np.ndarray,
    mass_star: np.ndarray,
    u_cvt: np.ndarray,
    u_saturated: np.ndarray,
    u_command: np.ndarray,
) -> dict:
    e = centroid_hat - centroid_star
    g_star_dot = 2.0 * mass_star[:, None] * (positions - centroid_star)
    g_hat_dot = 2.0 * mass_star[:, None] * (positions - centroid_hat)
    return {
        "centroid_error_norm2": float(np.sum(np.sum(e * e, axis=1))),
        "unsaturated_norm": float(np.linalg.norm(u_cvt)),
        "saturation_delta_norm": float(np.linalg.norm(u_saturated - u_cvt)),
        "safety_delta_norm": float(np.linalg.norm(u_command - u_saturated)),
        "g_star_dot_u_command": float(np.sum(g_star_dot * u_command)),
        "g_hat_dot_u_command": float(np.sum(g_hat_dot * u_command)),
        "centroid_error_contribution": float(np.sum(2.0 * mass_star * np.sum(e * u_command, axis=1))),
    }


def check_projection_step(
    rec,
    filter_results,
    umax: float,
    rho: float,
) -> dict:
    """Offline projection diagnostics for one control instant."""
    statuses = list(rec.solver_status)
    n_agents = len(statuses)
    speed_ok = bool(np.all(rec.speed <= umax + 1e-9))
    agent_ok = bool(np.all(rec.agent_residual_min >= -PROJECTION_RESIDUAL_TOL))
    wall_ok = bool(np.all(rec.wall_residual_min >= -PROJECTION_RESIDUAL_TOL))
    object_ok = bool(np.all(rec.object_residual_min >= -PROJECTION_RESIDUAL_TOL))
    zero_flags = [bool(r.zero_input_feasible) for r in filter_results]
    zero_with_rho = [bool(getattr(r, "zero_input_feasible_with_rho", True)) for r in filter_results]
    inside_band = [bool(getattr(r, "inside_margin_band", False)) for r in filter_results]
    mods = [float(r.modification) for r in filter_results]
    return {
        "statuses": statuses,
        "speed_within_umax": speed_ok,
        "agent_residuals_ok": agent_ok,
        "wall_residuals_ok": wall_ok,
        "object_residuals_ok": object_ok,
        "zero_input_feasible": zero_flags,
        "zero_input_all_feasible": all(zero_flags),
        "zero_input_feasible_with_rho": zero_with_rho,
        "zero_input_with_rho_all_feasible": all(zero_with_rho),
        "inside_margin_band": inside_band,
        "inside_margin_band_count": int(sum(1 for z in inside_band if z)),
        "modification_max": float(max(mods)) if mods else 0.0,
        "optimal_count": int(sum(1 for s in statuses if s == "optimal")),
        "relaxed_margin_count": int(sum(1 for s in statuses if s == "relaxed_margin")),
        "scaled_barrier_count": int(sum(1 for s in statuses if s == "scaled_barrier")),
        "n_agents": n_agents,
        "rho": rho,
    }


def run_case(
    config_path: Path,
    out_dir: Path,
    seed: int,
    frames: int,
    ntheta: int = 128,
    nradial: int = 12,
) -> dict:
    from dbact.theorem_mode import TheoremModeAbort
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import controller_params_from_config

    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = controller_params_from_config(cfg)
    dt = float(cfg["dt"])
    umax = float(params.max_speed)
    rho = float(params.rho)

    started = time.perf_counter()
    env = SimulationEnvironment(cfg, seed=seed)
    vertices0 = env.cargoes[0].vertices.copy()
    observer = UniformOffsetObserver(
        domain=env.domain,
        radius=params.local_radius,
        sigma=params.sigma,
        floor=params.base_density,
        cage_offset=params.cage_offset,
        ntheta=ntheta,
        nradial=nradial,
    )
    refined = UniformOffsetObserver(
        domain=env.domain,
        radius=params.local_radius,
        sigma=params.sigma,
        floor=params.base_density,
        cage_offset=params.cage_offset,
        ntheta=256,
        nradial=16,
    )
    fine = UniformOffsetObserver(
        domain=env.domain,
        radius=params.local_radius,
        sigma=params.sigma,
        floor=params.base_density,
        cage_offset=params.cage_offset,
        ntheta=1024,
        nradial=32,
    )

    positions_trace: list[np.ndarray] = []
    step_rows: list[dict] = []
    projection_rows: list[dict] = []
    abort = None
    completed_frames = 0

    P0 = np.vstack([a.position for a in env.agents])
    obs0 = observer.evaluate(P0, vertices0)
    positions_trace.append(P0.copy())
    H_trace = [float(obs0["H"])]

    try:
        for k in range(frames):
            P = np.vstack([a.position for a in env.agents])
            obs_k = observer.evaluate(P, vertices0)
            cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
            rec = env.controller.last_theorem_record
            filter_results = list(env.controller._last_filter_results)
            U_cmd = rec.u_command.copy()
            U_sat = rec.u_saturated.copy()
            U_cvt = rec.u_cvt.copy()

            env.controller.apply_commands(env.agents, cmds, env.dt)
            env.engine.step(env.cargoes, env.agents, env.dt)
            env.t += env.dt
            env._record()
            completed_frames = k + 1

            P_next = np.vstack([a.position for a in env.agents])
            positions_trace.append(P_next.copy())
            obs_next = observer.evaluate(P_next, vertices0)
            H_next = float(obs_next["H"])
            H_trace.append(H_next)

            delta_H = H_next - float(obs_k["H"])
            g = obs_k["gradient"]
            m_star = obs_k["mass"]
            rhs = {}
            slack = {}
            for label, U in (
                ("u_cvt", U_cvt),
                ("u_saturated", U_sat),
                ("u_command", U_cmd),
            ):
                r = dissipation_rhs(dt, g, m_star, U)
                rhs[label] = r
                slack[label] = float(delta_H - r)

            decomp = decomposition_terms(
                P, rec.cell_centroid, obs_k["centroid"], m_star, U_cvt, U_sat, U_cmd
            )
            proj = check_projection_step(rec, filter_results, umax, rho)
            projection_rows.append({"frame": k, "time": float(k * dt), **proj})

            step_rows.append(
                {
                    "frame": k,
                    "time": float(k * dt),
                    "positions": P.tolist(),
                    "positions_next": P_next.tolist(),
                    "u_cvt": U_cvt.tolist(),
                    "u_saturated": U_sat.tolist(),
                    "u_command": U_cmd.tolist(),
                    "cell_mass": rec.cell_mass.tolist(),
                    "cell_centroid": rec.cell_centroid.tolist(),
                    "solver_status": rec.solver_status,
                    "barrier_scale": rec.barrier_scale.tolist(),
                    "agent_residual_min": rec.agent_residual_min.tolist(),
                    "wall_residual_min": rec.wall_residual_min.tolist(),
                    "object_residual_min": rec.object_residual_min.tolist(),
                    "zero_input_feasible": proj["zero_input_feasible"],
                    "zero_input_feasible_with_rho": proj["zero_input_feasible_with_rho"],
                    "inside_margin_band": proj["inside_margin_band"],
                    "hold_min_pair": rec.hold_min_pair_distance,
                    "hold_min_object_clearance": rec.hold_min_object_clearance,
                    "hold_object_clearance_sampled": rec.hold_object_clearance_sampled,
                    "hold_object_clearance_margin": rec.hold_object_clearance_margin,
                    "hold_clearance_n_samples": rec.hold_clearance_n_samples,
                    "hold_clearance_h": rec.hold_clearance_h,
                    "H_star": float(obs_k["H"]),
                    "H_star_next": H_next,
                    "delta_H": delta_H,
                    "mass_star": m_star.tolist(),
                    "centroid_star": obs_k["centroid"].tolist(),
                    "gradient_star": g.tolist(),
                    "rhs": rhs,
                    "dissipation_slack": slack,
                    "decomposition": decomp,
                }
            )
    except TheoremModeAbort as exc:
        abort = exc.as_dict()
        env.theorem_abort = abort

    # Refine anomalous dissipation steps.
    anomalies = []
    for row in step_rows:
        s = row["dissipation_slack"]["u_command"]
        rhs = row["rhs"]["u_command"]
        tol = max(DISSIPATION_TOL_ABS, DISSIPATION_TOL_REL * max(abs(rhs), abs(row["delta_H"]), 1e-30))
        if s > tol:
            anomalies.append(row["frame"])

    refinements = []
    for frame in anomalies:
        row = step_rows[frame]
        P = np.asarray(row["positions"], dtype=float)
        Pn = np.asarray(row["positions_next"], dtype=float)
        coarse_k = observer.evaluate(P, vertices0)
        coarse_n = observer.evaluate(Pn, vertices0)
        fine_k = fine.evaluate(P, vertices0)
        fine_n = fine.evaluate(Pn, vertices0)
        U_cmd = np.asarray(row["u_command"], dtype=float)
        for tag, ok, on in (("coarse", coarse_k, coarse_n), ("fine", fine_k, fine_n)):
            dH = float(on["H"] - ok["H"])
            r = dissipation_rhs(dt, ok["gradient"], ok["mass"], U_cmd)
            refinements.append(
                {
                    "frame": frame,
                    "observer": tag,
                    "delta_H": dH,
                    "rhs": r,
                    "slack": float(dH - r),
                }
            )

    positions_arr = np.stack(positions_trace) if positions_trace else np.zeros((0, 0, 2))
    n_solves = sum(len(r["solver_status"]) for r in step_rows)
    status_counts: dict[str, int] = {}
    barrier_scalings = 0
    min_barrier_scale = 1.0
    zero_input_false = 0
    for row in step_rows:
        for st in row["solver_status"]:
            status_counts[st] = status_counts.get(st, 0) + 1
        for s in row["barrier_scale"]:
            if s < 1.0 - 1e-12:
                barrier_scalings += 1
            min_barrier_scale = min(min_barrier_scale, float(s))
        zero_input_false += sum(1 for z in row["zero_input_feasible"] if not z)

    slacks_cmd = [r["dissipation_slack"]["u_command"] for r in step_rows]
    violations = [
        s
        for s, row in zip(slacks_cmd, step_rows)
        if s
        > max(
            DISSIPATION_TOL_ABS,
            DISSIPATION_TOL_REL
            * max(abs(row["rhs"]["u_command"]), abs(row["delta_H"]), 1e-30),
        )
    ]

    summary = {
        "seed": seed,
        "frames_requested": frames,
        "frames_completed": completed_frames,
        "physical_seconds_completed": completed_frames * dt,
        "complete_30s": completed_frames >= frames and abort is None,
        "abort": abort,
        "wall_seconds": time.perf_counter() - started,
        "n_positions": len(positions_trace),
        "n_control_steps": len(step_rows),
        "n_qp_solves": n_solves,
        "expected_qp_solves": int(len(env.agents) * frames),
        "solver_status_counts": status_counts,
        "barrier_scalings": barrier_scalings,
        "min_barrier_scale": float(min_barrier_scale),
        "hold_min_pair": float(min((r["hold_min_pair"] for r in step_rows), default=float("inf"))),
        "hold_min_object_clearance_lb": float(
            min((r["hold_min_object_clearance"] for r in step_rows), default=float("inf"))
        ),
        "hold_min_object_clearance_margin": float(
            min((r["hold_object_clearance_margin"] for r in step_rows), default=float("inf"))
        ),
        "H0": float(obs0["H"]),
        "H_final": float(H_trace[-1]) if H_trace else None,
        "dissipation_u_command": {
            "max_slack": float(max(slacks_cmd)) if slacks_cmd else None,
            "mean_slack": float(np.mean(slacks_cmd)) if slacks_cmd else None,
            "fraction_positive": float(np.mean(np.asarray(slacks_cmd) > 0)) if slacks_cmd else None,
            "n_violations": len(violations),
            "n_anomalies_refined": len(anomalies),
        },
        "dissipation_u_saturated_max_slack": float(
            max(r["dissipation_slack"]["u_saturated"] for r in step_rows)
        )
        if step_rows
        else None,
        "dissipation_u_cvt_max_slack": float(max(r["dissipation_slack"]["u_cvt"] for r in step_rows))
        if step_rows
        else None,
        "observer_quadrature": {"ntheta": ntheta, "nradial": nradial},
        "zero_input_infeasible_count": zero_input_false,
    }

    if step_rows:
        np.savez_compressed(
            out_dir / "trajectory.npz",
            positions=positions_arr,
            H_star=np.asarray(H_trace, dtype=float),
            u_cvt=np.stack([np.asarray(r["u_cvt"]) for r in step_rows]),
            u_saturated=np.stack([np.asarray(r["u_saturated"]) for r in step_rows]),
            u_command=np.stack([np.asarray(r["u_command"]) for r in step_rows]),
            vertices=vertices0,
        )
    dump(out_dir / "step_records.json", step_rows)
    dump(out_dir / "dissipation_refinements.json", refinements)
    dump(out_dir / "projection_rows.json", projection_rows)
    dump(out_dir / "summary.json", summary)

    with (out_dir / "dissipation_slack.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=[
                "frame",
                "time",
                "delta_H",
                "rhs_u_command",
                "slack_u_command",
                "rhs_u_saturated",
                "slack_u_saturated",
                "rhs_u_cvt",
                "slack_u_cvt",
            ],
        )
        writer.writeheader()
        for r in step_rows:
            writer.writerow(
                {
                    "frame": r["frame"],
                    "time": r["time"],
                    "delta_H": r["delta_H"],
                    "rhs_u_command": r["rhs"]["u_command"],
                    "slack_u_command": r["dissipation_slack"]["u_command"],
                    "rhs_u_saturated": r["rhs"]["u_saturated"],
                    "slack_u_saturated": r["dissipation_slack"]["u_saturated"],
                    "rhs_u_cvt": r["rhs"]["u_cvt"],
                    "slack_u_cvt": r["dissipation_slack"]["u_cvt"],
                }
            )

    return {
        "summary": summary,
        "step_rows": step_rows,
        "projection_rows": projection_rows,
        "refinements": refinements,
    }


def projection_verdict(all_projection_rows: list[dict], rho: float) -> dict:
    optimal = sum(r["optimal_count"] for r in all_projection_rows)
    relaxed = sum(r["relaxed_margin_count"] for r in all_projection_rows)
    scaled = sum(r["scaled_barrier_count"] for r in all_projection_rows)
    total = sum(r["n_agents"] for r in all_projection_rows)
    zero_false = sum(
        r["n_agents"] - sum(1 for z in r["zero_input_feasible"] if z) for r in all_projection_rows
    )
    zero_rho_false = sum(
        r["n_agents"] - sum(1 for z in r.get("zero_input_feasible_with_rho", []) if z)
        for r in all_projection_rows
    )
    inside_band = sum(int(r.get("inside_margin_band_count", 0)) for r in all_projection_rows)
    speed_fail = sum(1 for r in all_projection_rows if not r["speed_within_umax"])
    agent_fail = sum(1 for r in all_projection_rows if not r["agent_residuals_ok"])
    wall_fail = sum(1 for r in all_projection_rows if not r["wall_residuals_ok"])
    object_fail = sum(1 for r in all_projection_rows if not r["object_residuals_ok"])
    only_optimal = relaxed == 0 and scaled == 0
    residuals_ok = speed_fail == 0 and agent_fail == 0 and wall_fail == 0 and object_fail == 0
    if only_optimal and residuals_ok and zero_rho_false == 0:
        verdict = (
            "U = Pi_F(u_nom) on full intended set including rho; u=0 feasible for all final rows"
        )
    elif only_optimal and residuals_ok and inside_band > 0:
        verdict = (
            "U = Pi_F(u_nom) on full intended set (all optimal); "
            "u=0 often fails object rows with rho (inside ISSf margin band) — "
            "projection optimality vs 0 is not available on those steps"
        )
    elif only_optimal and residuals_ok:
        verdict = "U = Pi_F(u_nom) on intended set (all optimal, residuals ok)"
    elif scaled > 0 or relaxed > 0:
        verdict = "mixed: optimal path with occasional relaxed_margin/scaled_barrier tiers"
    else:
        verdict = "optimal tier only but check residual/speed flags"
    return {
        "total_agent_steps": total,
        "status_optimal": optimal,
        "status_relaxed_margin": relaxed,
        "status_scaled_barrier": scaled,
        "fraction_optimal": float(optimal / total) if total else None,
        "fraction_relaxed_margin": float(relaxed / total) if total else None,
        "fraction_scaled_barrier": float(scaled / total) if total else None,
        "zero_input_infeasible_agent_steps": zero_false,
        "zero_input_infeasible_with_rho_agent_steps": zero_rho_false,
        "inside_margin_band_agent_steps": inside_band,
        "rho_margin_parameter": rho,
        "speed_violations_steps": speed_fail,
        "agent_residual_violations_steps": agent_fail,
        "wall_residual_violations_steps": wall_fail,
        "object_residual_violations_steps": object_fail,
        "verdict": verdict,
        "zero_input_vs_rho": (
            "zero_input_feasible = barrier (no rho); "
            "zero_input_feasible_with_rho = full final QP including object rho rows; "
            f"inside_margin_band_agent_steps={inside_band}"
        ),
        "projection_optimality_obstacle": (
            None
            if zero_rho_false == 0
            else (
                "On inside-margin-band steps, 0 is not in the final feasible set F_rho, "
                "so the comparison <u_nom - U, -U> <= 0 from projection optimality does not apply. "
                "Do not delete rho rows; treat those steps as blocked for the 0-feasible dissipation path."
            )
        ),
    }


def make_plots(out: Path, case_results: dict) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    for seed, payload in case_results.items():
        rows = payload["step_rows"]
        if not rows:
            continue
        t = [r["time"] for r in rows]
        dH = [r["delta_H"] for r in rows]
        rhs = [r["rhs"]["u_command"] for r in rows]
        slack = [r["dissipation_slack"]["u_command"] for r in rows]
        axes[0].plot(t, dH, alpha=0.7, label=f"seed {seed} ΔH*")
        axes[0].plot(t, rhs, linestyle="--", alpha=0.7, label=f"seed {seed} rhs(U_cmd)")
        axes[1].plot(t, slack, label=f"seed {seed} ΔH-rhs")
    axes[0].set_ylabel("ΔH* vs rhs")
    axes[0].set_title("Discrete dissipation: ΔH* and rhs(U_command)")
    axes[0].grid(alpha=0.2)
    axes[0].legend(fontsize=7)
    axes[1].axhline(DISSIPATION_TOL_ABS, color="k", ls=":", alpha=0.5, label="tol")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("ΔH* - rhs")
    axes[1].set_title("Dissipation slack (should be ≤ 0 + tol)")
    axes[1].grid(alpha=0.2)
    axes[1].legend(fontsize=7)
    fig.savefig(out / "delta_H_vs_rhs.png", dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/sim/theorem/static_l_shape_n16_oracle.yaml",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(
            r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\n16_discrete_dissipation"
        ),
    )
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    from dbact_sim.scenarios import build_cargoes, controller_params_from_config

    cargo = build_cargoes(cfg, seed=0)[0]
    consts = theoretical_constants(cfg, cargo.vertices)
    params = controller_params_from_config(cfg)
    dump(args.out / "constants.json", consts)
    dump(args.out / "environment_snapshot.json", environment_snapshot(ROOT))

    case_results = {}
    summaries = []
    all_projection = []
    for seed in args.seeds:
        seed_dir = args.out / f"seed_{seed}"
        payload = run_case(
            args.config,
            seed_dir,
            seed,
            args.frames,
            ntheta=args.ntheta,
            nradial=args.nradial,
        )
        summaries.append(payload["summary"])
        case_results[seed] = payload
        all_projection.extend(payload["projection_rows"])
        s = payload["summary"]
        print(
            f"seed {seed}: complete={s['complete_30s']} solves={s['n_qp_solves']} "
            f"barrier_scalings={s['barrier_scalings']} "
            f"min_pair={s['hold_min_pair']:.4f} "
            f"min_clear_lb={s['hold_min_object_clearance_lb']:.4f} "
            f"max_slack={s['dissipation_u_command']['max_slack']:.3e} "
            f"frac_pos={s['dissipation_u_command']['fraction_positive']:.3f}",
            flush=True,
        )

    projection = projection_verdict(all_projection, float(params.rho))
    dump(args.out / "projection_conditions.json", projection)
    dump(args.out / "run_summaries.json", summaries)
    dump(
        args.out / "full_stats_vs_observer.json",
        {
            "n_control_steps_per_seed": args.frames,
            "n_agents": int(cfg["agents"]["count"]),
            "expected_qp_solves_per_seed": int(cfg["agents"]["count"]) * args.frames,
            "observer_evaluations_per_seed": args.frames + 1 + args.frames,
            "note": (
                "9600 QP solves = 16 agents × 600 steps when complete; "
                "observer runs at every position (601) and every pre-step control instant (600)."
            ),
            "summaries": summaries,
            "projection": projection,
        },
    )
    make_plots(args.out, case_results)
    print(json.dumps({"summaries": summaries, "projection": projection}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
