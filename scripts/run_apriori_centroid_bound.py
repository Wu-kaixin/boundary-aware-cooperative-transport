"""Fresh 27-case a priori centroid-bound experiment (static sampled theorem_mode).

Does not read previous theorem_audit artifacts. Priors are written before each
closed-loop run and are never fitted to trajectory E.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from audit_discrete_dissipation import (  # noqa: E402
    DISSIPATION_TOL_ABS,
    DISSIPATION_TOL_REL,
    check_projection_step,
    dissipation_rhs,
)
from audit_n16_static_performance import UniformOffsetObserver  # noqa: E402
from dbact.apriori_centroid_bound import (  # noqa: E402
    assemble_prior,
    config_fingerprint,
    overlay_grid_resolution,
    step_in_k0,
)
from dbact.theorem_mode import TheoremModeAbort  # noqa: E402

SHAPES = {
    "l_shape": ROOT / "configs/sim/theorem/static_l_shape_n16_oracle.yaml",
    "rectangle": ROOT / "configs/sim/theorem/static_rectangle_n16_oracle.yaml",
    "c_shape": ROOT / "configs/sim/theorem/static_c_shape_n16_oracle.yaml",
}


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=_json_default), encoding="utf-8")


def _json_default(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    raise TypeError(type(obj))


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_overlay_config(
    base: dict,
    grid: int,
    dest: Path,
    integration_method: str = "endpoint_grid",
    edge_n_gon: int = 256,
    edge_panels: int = 48,
    edge_h_max: float = 0.004,
    edge_cull_sigmas: float = 6.0,
    clamp_margin: bool = False,
) -> dict:
    cfg = overlay_grid_resolution(base, grid)
    cfg.setdefault("controller", {})
    cfg["controller"]["integration_method"] = str(integration_method)
    cfg["controller"]["edge_n_gon"] = int(edge_n_gon)
    cfg["controller"]["edge_panels"] = int(edge_panels)
    cfg["controller"]["edge_h_max"] = float(edge_h_max)
    cfg["controller"]["edge_cull_sigmas"] = float(edge_cull_sigmas)
    cfg["controller"]["theorem_clamp_margin_to_keep_zero"] = bool(clamp_margin)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    return cfg


def make_observer(cfg: dict, ntheta: int, nradial: int) -> UniformOffsetObserver:
    from dbact_sim.scenarios import controller_params_from_config

    p = controller_params_from_config(cfg)
    domain = [cfg["domain"]["xmin"], cfg["domain"]["xmax"], cfg["domain"]["ymin"], cfg["domain"]["ymax"]]
    return UniformOffsetObserver(domain, p.local_radius, p.sigma, p.base_density, p.cage_offset, ntheta, nradial)


def run_fresh_case(
    cfg: dict,
    out_dir: Path,
    seed: int,
    frames: int,
    observer: UniformOffsetObserver,
    prior: dict,
    code_sha: str,
) -> dict:
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import controller_params_from_config

    out_dir.mkdir(parents=True, exist_ok=True)
    dump(out_dir / "prior.json", prior)
    dump(out_dir / "config_used.json", cfg)

    params = controller_params_from_config(cfg)
    dt = float(cfg["dt"])
    umax = float(params.max_speed)
    rho = float(params.rho)
    n_agents = int(cfg["agents"]["count"])
    a = prior["parameters"]["a"]
    kc = float(params.kp_cage)

    started = time.perf_counter()
    env = SimulationEnvironment(cfg, seed=seed)
    vertices = env.cargoes[0].vertices.copy()
    p0 = np.vstack([ag.position for ag in env.agents])
    if not np.allclose(p0, np.asarray(prior["P0"]), atol=1e-12):
        raise RuntimeError("P0 used for the prior does not match the simulation initial state")

    positions_trace = [p0.copy()]
    step_rows: list[dict] = []
    abort = None
    completed = 0
    obs_prev = observer.evaluate(p0, vertices)
    h_trace = [float(obs_prev["H"])]

    try:
        for k in range(frames):
            p = np.vstack([ag.position for ag in env.agents])
            cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
            rec = env.controller.last_theorem_record
            filter_results = list(env.controller._last_filter_results)
            env.controller.apply_commands(env.agents, cmds, env.dt)
            env.engine.step(env.cargoes, env.agents, env.dt)
            env.t += env.dt
            env._record()
            completed = k + 1
            p_next = np.vstack([ag.position for ag in env.agents])
            positions_trace.append(p_next.copy())
            obs_k = obs_prev
            obs_next = observer.evaluate(p_next, vertices)
            h_next = float(obs_next["H"])
            h_trace.append(h_next)
            u_cmd = rec.u_command.copy()
            g = obs_k["gradient"]
            m_star = obs_k["mass"]
            delta_h = h_next - float(obs_k["H"])
            rhs = dissipation_rhs(dt, g, m_star, u_cmd)
            slack = float(delta_h - rhs)
            proj = check_projection_step(rec, filter_results, umax, rho)
            k0_rec = {
                "solver_status": rec.solver_status,
                "zero_input_feasible_with_rho": proj["zero_input_feasible_with_rho"],
            }
            in_k0, bad = step_in_k0(k0_rec, n_agents, a)
            chat = rec.cell_centroid
            c_star = obs_k["centroid"]
            j_k = float(np.sum(m_star * np.sum(u_cmd * u_cmd, axis=1)))
            e_k = float(np.sum(m_star * np.sum((chat - c_star) ** 2, axis=1)))
            g2 = float(np.sum(g * g))
            c2 = float(np.sum((c_star - p) ** 2))
            row = {
                "frame": k,
                    "time": float(k * dt),
                    "positions": p.tolist(),
                    "positions_next": p_next.tolist(),
                    "u_cvt": rec.u_cvt.tolist(),
                    "u_saturated": rec.u_saturated.tolist(),
                    "u_command": u_cmd.tolist(),
                    "cell_mass": rec.cell_mass.tolist(),
                    "cell_centroid": chat.tolist(),
                    "solver_status": rec.solver_status,
                    "qp_all_optimal": all(str(s) == "optimal" for s in rec.solver_status),
                    "zero_input_feasible": proj["zero_input_feasible"],
                    "zero_input_feasible_with_rho": proj["zero_input_feasible_with_rho"],
                    "inside_margin_band": proj["inside_margin_band"],
                    "H_star": float(obs_k["H"]),
                    "H_star_next": h_next,
                    "delta_H": delta_h,
                    "rhs_u_command": rhs,
                    "dissipation_slack_u_command": slack,
                    "mass_star": m_star.tolist(),
                    "centroid_star": c_star.tolist(),
                    "gradient_star": g.tolist(),
                    "J_k": j_k,
                    "E_k": e_k,
                    "grad_residual_k": g2,
                    "centroid2_k": c2,
                    "in_K0": bool(in_k0),
                    "k0_bad_agents": bad,
                    "hold_min_pair": rec.hold_min_pair_distance,
                    "hold_min_object_clearance": rec.hold_min_object_clearance,
                    "hold_object_clearance_margin": rec.hold_object_clearance_margin,
            }
            step_rows.append(row)
            if (not in_k0) or slack > max(
                DISSIPATION_TOL_ABS,
                DISSIPATION_TOL_REL * max(abs(rhs), abs(delta_h), 1e-30),
            ):
                _dump_filter_traces(
                    out_dir / "constraint_dumps",
                    k,
                    p,
                    rec,
                    filter_results,
                    row,
                )
            obs_prev = obs_next
    except TheoremModeAbort as exc:
        abort = exc.as_dict()
        env.theorem_abort = abort

    wall = time.perf_counter() - started
    fail_frames = [r["frame"] for r in step_rows if not r["in_K0"]]
    fail_agents = sorted({i for r in step_rows for i in r["k0_bad_agents"]})
    full_horizon = bool(step_rows) and all(r["in_K0"] for r in step_rows) and abort is None
    j_series = np.asarray([r["J_k"] for r in step_rows], dtype=float)
    e_series = np.asarray([r["E_k"] for r in step_rows], dtype=float)
    h_series = np.asarray([r["H_star"] for r in step_rows], dtype=float)
    g_series = np.asarray([r["grad_residual_k"] for r in step_rows], dtype=float)
    c_series = np.asarray([r["centroid2_k"] for r in step_rows], dtype=float)
    k_done = len(step_rows)
    j_bar = float(np.mean(j_series)) if k_done else float("nan")
    e_bar = float(np.mean(e_series)) if k_done else float("nan")
    h0 = float(h_series[0]) if k_done else float("nan")
    h_final = float(h_trace[-1]) if h_trace else None

    posthoc = None
    posthoc_below = None
    status = "inapplicable"
    label = "not_covered_by_theorem"
    if full_horizon and a > 0 and k_done:
        posthoc = 2.0 * h0 / (a * k_done * dt) + 4.0 * e_bar / (a * a)
        posthoc_below = bool(posthoc <= prior["B_J"]["geometric"]["B_J_geom"])
        status = "applicable_post_hoc"
        label = "post_hoc"

    slacks = [r["dissipation_slack_u_command"] for r in step_rows]
    anomalies = []
    for r in step_rows:
        tol = max(
            DISSIPATION_TOL_ABS,
            DISSIPATION_TOL_REL * max(abs(r["rhs_u_command"]), abs(r["delta_H"]), 1e-30),
        )
        if r["dissipation_slack_u_command"] > tol:
            anomalies.append(r["frame"])

    geom = prior["B_J"]["geometric"]["B_J_geom"]
    summary = {
        "seed": seed,
        "grid_resolution": int(cfg["controller"]["grid_resolution"]),
        "config_sha256": config_fingerprint(cfg),
        "code_sha": code_sha,
        "frames_requested": frames,
        "frames_completed": completed,
        "complete_30s": completed >= frames and abort is None,
        "abort": abort,
        "wall_seconds": wall,
        "n_qp_solves": n_agents * completed,
        "solver_status_counts": _count([s for r in step_rows for s in r["solver_status"]]),
        "theorem_conditions_hold_full_horizon": full_horizon,
        "full_horizon_J_bound_status": status,
        "descriptive_stats_label": label,
        "condition_failure_frames": fail_frames,
        "condition_failure_agent_indices": fail_agents,
        "zero_input_with_rho_failure_count": int(
            sum(1 for r in step_rows for z in r["zero_input_feasible_with_rho"] if not z)
        ),
        "H0": h0,
        "H_final": h_final,
        "J_bar": j_bar,
        "E_bar": e_bar,
        "E_bar_estimate_type": "post_hoc_trajectory",
        "gradient_residual_bar": float(np.mean(g_series)) if k_done else float("nan"),
        "centroid2_bar": float(np.mean(c_series)) if k_done else float("nan"),
        "posthoc_J_bound": posthoc,
        "posthoc_bound_below_geom": posthoc_below,
        "B_E_rigorous": prior["B_E"]["moment_rigorous"],
        "B_E_numerical_a_priori": prior["B_E"]["moment_numerical_a_priori"],
        "B_E_diameter": prior["B_E"]["diameter"]["value"],
        "B_J_prior_certificate": prior["B_J"]["prior_certificate_crudeH_rigorousE"]["B_J_prior"],
        "B_J_prior_P0H_rigorousE": prior["B_J"]["prior_P0H_rigorousE"]["B_J_prior"],
        "B_J_prior_numerical_a_priori": prior["B_J"]["prior_numerical_a_priori"]["B_J_prior"],
        "B_J_geom": geom,
            "prior_beats_geom_certificate": prior["beats_geometry"]["certificate_crudeH_rigorousE"],
            "prior_beats_geom_P0H": prior["beats_geometry"]["P0H_rigorousE"],
            "prior_beats_geom_numerical": prior["beats_geometry"]["numerical_a_priori"],
        "dissipation_u_command_max_slack": float(max(slacks)) if slacks else None,
        "dissipation_anomaly_frames": anomalies,
        "observer_quadrature": {
            "ntheta": len(observer.unit),
            "nradial": len(observer.radial_nodes),
        },
        "rho": rho,
        "k_c": kc,
        "a": a,
        "labels": prior["labels"],
    }
    dump(out_dir / "summary.json", summary)
    dump(out_dir / "step_records.json", step_rows)
    if positions_trace:
        np.savez_compressed(
            out_dir / "trajectory.npz",
            positions=np.stack(positions_trace),
            H_star=np.asarray(h_trace, dtype=float),
            J=j_series,
            E=e_series,
            grad_residual=g_series,
            centroid2=c_series,
            in_K0=np.asarray([r["in_K0"] for r in step_rows], dtype=np.int8),
            vertices=vertices,
        )
    with (out_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(
            [
                "frame",
                "H_star",
                "J_k",
                "E_k",
                "grad_residual_k",
                "centroid2_k",
                "in_K0",
                "qp_all_optimal",
                "dissipation_slack_u_command",
            ]
        )
        for r in step_rows:
            w.writerow(
                [
                    r["frame"],
                    r["H_star"],
                    r["J_k"],
                    r["E_k"],
                    r["grad_residual_k"],
                    r["centroid2_k"],
                    int(r["in_K0"]),
                    int(r["qp_all_optimal"]),
                    r["dissipation_slack_u_command"],
                ]
            )
    return {"summary": summary, "step_rows": step_rows, "vertices": vertices, "p0": p0}


def _dump_filter_traces(dest: Path, frame: int, positions, rec, filter_results, row: dict) -> None:
    dest.mkdir(parents=True, exist_ok=True)
    traces = []
    for i, fr in enumerate(filter_results):
        original = getattr(fr, "b_original", None)
        effective = getattr(fr, "b_effective", None)
        A = getattr(fr, "A_rows", None)
        kinds = getattr(fr, "row_kinds", None) or []
        u = np.asarray(fr.velocity, dtype=float)
        u_nom = getattr(fr, "u_nominal", None)
        b_orig = np.asarray(original, dtype=float) if original is not None else np.empty(0)
        b_eff = np.asarray(effective, dtype=float) if effective is not None else np.empty(0)
        A_arr = np.asarray(A, dtype=float) if A is not None and len(A) else np.empty((0, 2))
        orig_res = (A_arr @ u - b_orig) if len(A_arr) and len(b_orig) == len(A_arr) else np.empty(0)
        eff_res = (A_arr @ u - b_eff) if len(A_arr) and len(b_eff) == len(A_arr) else np.empty(0)
        traces.append(
            {
                "agent": i,
                "status": fr.status,
                "zero_input_feasible": bool(fr.zero_input_feasible),
                "zero_input_feasible_with_rho": bool(fr.zero_input_feasible_with_rho),
                "original_zero_input_feasible_with_rho": bool(
                    getattr(fr, "original_zero_input_feasible_with_rho", fr.zero_input_feasible_with_rho)
                ),
                "empty_feasible_set": bool(getattr(fr, "empty_feasible_set", False)),
                "zero_not_in_F": bool(getattr(fr, "zero_not_in_F", False)),
                "clamp_applied": bool(getattr(fr, "clamp_applied", False)),
                "u": u.tolist(),
                "u_nom": None if u_nom is None else np.asarray(u_nom, dtype=float).tolist(),
                "A": A_arr.tolist(),
                "b_original": b_orig.tolist(),
                "b_effective": b_eff.tolist(),
                "b_no_margin": None
                if getattr(fr, "b_no_margin", None) is None
                else np.asarray(fr.b_no_margin, dtype=float).tolist(),
                "row_kinds": list(kinds),
                "h_object": None
                if getattr(fr, "h_object", None) is None
                else np.asarray(fr.h_object, dtype=float).tolist(),
                "residual_original": orig_res.tolist(),
                "residual_effective": eff_res.tolist(),
                "position": np.asarray(positions[i], dtype=float).tolist(),
            }
        )
    dump(
        dest / f"frame_{frame:04d}.json",
        {
            "frame": frame,
            "in_K0": row.get("in_K0"),
            "k0_bad_agents": row.get("k0_bad_agents"),
            "dissipation_slack": row.get("dissipation_slack_u_command"),
            "u_cvt": row.get("u_cvt"),
            "u_saturated": row.get("u_saturated"),
            "u_command": row.get("u_command"),
            "agents": traces,
        },
    )


def _count(items: list[str]) -> dict[str, int]:
    out: dict[str, int] = {}
    for x in items:
        out[x] = out.get(x, 0) + 1
    return out


def case_key(shape: str, seed: int, grid: int) -> str:
    return f"{shape}_seed{seed}_n{grid}"


def prepare_p0_and_prior(
    cfg: dict,
    seed: int,
    observer: UniformOffsetObserver,
    observers_extra: dict,
    k_steps: int,
    geometry_cache: dict,
    density_mesh: int,
    restrict_site_grid: int,
    restrict_local_mesh: int,
) -> tuple[np.ndarray, np.ndarray, dict]:
    from dbact_sim.environment import SimulationEnvironment

    env = SimulationEnvironment(cfg, seed=seed)
    vertices = env.cargoes[0].vertices.copy()
    p0 = np.vstack([ag.position for ag in env.agents])
    prior = assemble_prior(
        cfg,
        vertices,
        p0,
        observer,
        observers_extra,
        k_steps,
        density_mesh=density_mesh,
        restrict_site_grid=restrict_site_grid,
        restrict_local_mesh=restrict_local_mesh,
        geometry_cache=geometry_cache,
    )
    prior["P0"] = p0.tolist()
    prior["config_sha256"] = config_fingerprint(cfg)
    return p0, vertices, prior


def make_figures(rows: list[dict], out_dir: Path) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    written = []
    shapes = ["l_shape", "rectangle", "c_shape"]
    seeds = sorted({r["seed"] for r in rows})

    fig, axes = plt.subplots(1, 3, figsize=(12.5, 3.8), constrained_layout=True)
    for ax, shape in zip(axes, shapes):
        for seed in seeds:
            sub = [r for r in rows if r["shape"] == shape and r["seed"] == seed]
            sub = sorted(sub, key=lambda r: r["grid_resolution"])
            ax.plot(
                [r["grid_resolution"] for r in sub],
                [r["E_bar"] for r in sub],
                marker="o",
                label=f"seed {seed} measured E",
            )
        if rows:
            ax.axhline(rows[0]["B_E_diameter"], color="k", ls=":", lw=1, label="diameter 4R²M")
        ax.set_title(shape)
        ax.set_xlabel("controller grid n")
        ax.set_ylabel(r"$\bar E$")
        ax.set_yscale("symlog", linthresh=1e-12)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7)
    fig.suptitle("Mass-weighted centroid error vs controller resolution (eval grid fixed)")
    path = fig_dir / "E_vs_resolution.png"
    fig.savefig(path, dpi=150)
    fig.savefig(fig_dir / "E_vs_resolution.pdf")
    plt.close(fig)
    written += ["E_vs_resolution.png", "E_vs_resolution.pdf"]

    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.0), constrained_layout=True)
    for ax, shape in zip(axes, shapes):
        sub = [r for r in rows if r["shape"] == shape]
        labels = [f"s{r['seed']}/n{r['grid_resolution']}" for r in sub]
        x = np.arange(len(sub))
        ax.scatter(x, [r["J_bar"] for r in sub], label="measured J", zorder=3)
        ax.scatter(
            x,
            [r["posthoc_J_bound"] if r["posthoc_J_bound"] is not None else np.nan for r in sub],
            marker="x",
            label="post-hoc",
        )
        ax.scatter(x, [r["B_J_prior_P0H_rigorousE"] for r in sub], marker="s", label="prior (rigorous E)")
        ax.scatter(x, [r["B_J_prior_numerical_a_priori"] for r in sub], marker="d", label="prior (num.)")
        ax.scatter(x, [r["B_J_geom"] for r in sub], marker="_", s=120, label="geom M u_max²")
        ax.set_xticks(x, labels, rotation=75, fontsize=7)
        ax.set_yscale("symlog", linthresh=1e-6)
        ax.set_title(shape)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=6)
    fig.suptitle("J: measured / post-hoc / a priori / geometric (M_total u_max²)")
    fig.savefig(fig_dir / "J_bounds_comparison.png", dpi=150)
    fig.savefig(fig_dir / "J_bounds_comparison.pdf")
    plt.close(fig)
    written += ["J_bounds_comparison.png", "J_bounds_comparison.pdf"]

    # Representative time series: L seed 2 all grids, C seed 5 all grids.
    for shape, seed in (("l_shape", 2), ("c_shape", 5)):
        fig, axes = plt.subplots(3, 2, figsize=(10.5, 8.5), constrained_layout=True)
        for i, grid in enumerate((20, 40, 80)):
            rec = next((r for r in rows if r["shape"] == shape and r["seed"] == seed and r["grid_resolution"] == grid), None)
            if rec is None or rec.get("metrics_csv") is None:
                continue
            data = np.genfromtxt(rec["metrics_csv"], delimiter=",", names=True)
            k = data["frame"]
            axes[i, 0].plot(k, data["E_k"], lw=0.8)
            axes[i, 1].plot(k, data["J_k"], lw=0.8)
            fails = np.where(data["in_K0"] < 0.5)[0]
            for ax in axes[i]:
                for f in fails:
                    ax.axvspan(f - 0.5, f + 0.5, color="orange", alpha=0.35)
            axes[i, 0].set_title(f"{shape} seed {seed} n={grid}  E_k")
            axes[i, 1].set_title(f"{shape} seed {seed} n={grid}  J_k")
            for ax in axes[i]:
                ax.set_yscale("symlog", linthresh=1e-12)
                ax.grid(alpha=0.25)
        fname = f"timeseries_{shape}_seed{seed}"
        fig.savefig(fig_dir / f"{fname}.png", dpi=150)
        fig.savefig(fig_dir / f"{fname}.pdf")
        plt.close(fig)
        written += [f"{fname}.png", f"{fname}.pdf"]
    return written


def write_summary_csv(path: Path, rows: list[dict]) -> None:
    fields = [
        "shape",
        "seed",
        "grid_resolution",
        "complete_30s",
        "abort_reason",
        "theorem_conditions_hold_full_horizon",
        "full_horizon_J_bound_status",
        "condition_failure_frames",
        "condition_failure_agent_indices",
        "zero_input_with_rho_failure_count",
        "H0",
        "H_final",
        "J_bar",
        "E_bar",
        "gradient_residual_bar",
        "centroid2_bar",
        "B_E_rigorous",
        "B_E_numerical_a_priori",
        "B_E_diameter",
        "posthoc_J_bound",
        "B_J_prior_P0H_rigorousE",
        "B_J_prior_numerical_a_priori",
        "B_J_geom",
        "posthoc_bound_below_geom",
        "prior_beats_geom_certificate",
        "prior_beats_geom_numerical",
        "wall_seconds",
        "config_sha256",
        "code_sha",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            row = dict(r)
            abort = r.get("abort")
            row["abort_reason"] = None if abort is None else abort.get("reason")
            row["condition_failure_frames"] = ";".join(str(x) for x in r.get("condition_failure_frames", []))
            row["condition_failure_agent_indices"] = ";".join(
                str(x) for x in r.get("condition_failure_agent_indices", [])
            )
            w.writerow(row)


def parse_spec(values: list[str], cast):
    return [cast(v) for v in values]


_SOURCE_FILES = [
    ROOT / "src/dbact/edge_green_integral.py",
    ROOT / "src/dbact/apriori_centroid_bound.py",
    ROOT / "src/dbact/local_cvt.py",
    ROOT / "src/dbact/safety_filter.py",
    ROOT / "src/dbact/theorem_mode.py",
    ROOT / "src/dbact/controller.py",
    ROOT / "scripts/run_apriori_centroid_bound.py",
]


def source_fingerprint() -> str:
    h = hashlib.sha256()
    for path in _SOURCE_FILES:
        h.update(path.name.encode())
        h.update(path.read_bytes() if path.exists() else b"missing")
    return h.hexdigest()


def prior_rule_fingerprint(args) -> str:
    payload = {
        "integration_method": args.integration_method,
        "edge_n_gon": args.edge_n_gon,
        "edge_panels": args.edge_panels,
        "edge_h_max": args.edge_h_max,
        "edge_cull_sigmas": args.edge_cull_sigmas,
        "clamp_margin": args.clamp_margin,
        "frames": args.frames,
        "grids": args.grids,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def _atomic_complete(case_dir: Path, payload: dict) -> None:
    tmp = case_dir / "COMPLETE.json.tmp"
    final = case_dir / "COMPLETE.json"
    tmp.write_text(json.dumps(payload, indent=2, default=_json_default), encoding="utf-8")
    os.replace(tmp, final)


def worker_run_case(payload: dict) -> dict:
    """Top-level Windows-spawn worker. One independent case, BLAS threads = 1."""
    from dbact.cpu_budget import limit_blas_threads

    limit_blas_threads(1)
    os.environ["DBACT_CVT_WORKERS"] = str(int(payload.get("cvt_workers", 1)))
    cfg = payload["cfg"]
    observer = make_observer(cfg, int(payload["ntheta"]), int(payload["nradial"]))
    prior = json.loads(Path(payload["prior_path"]).read_text(encoding="utf-8"))
    case_dir = Path(payload["case_dir"])
    result = run_fresh_case(
        cfg,
        case_dir,
        int(payload["seed"]),
        int(payload["frames"]),
        observer,
        prior,
        payload["code_sha"],
    )
    summary = result["summary"]
    summary["shape"] = payload["shape"]
    summary["metrics_csv"] = str(case_dir / "metrics.csv")
    _atomic_complete(
        case_dir,
        {
            "key": payload["key"],
            "source_fp": payload["source_fp"],
            "prior_fp": payload["prior_fp"],
            "frames": payload["frames"],
        },
    )
    return summary


def worker_prepare_prior(payload: dict) -> dict:
    from dbact.cpu_budget import limit_blas_threads

    limit_blas_threads(1)
    cfg = payload["cfg"]
    observer = make_observer(cfg, int(payload["ntheta"]), int(payload["nradial"]))
    observers_extra = {"fine256": make_observer(cfg, 256, 16)}
    _p0, _verts, prior = prepare_p0_and_prior(
        cfg,
        int(payload["seed"]),
        observer,
        observers_extra,
        int(payload["frames"]),
        {},
        int(payload["density_mesh"]),
        int(payload["restrict_site_grid"]),
        int(payload["restrict_local_mesh"]),
    )
    dest = Path(payload["prior_path"])
    dump(dest, prior)
    return {"key": payload["key"], "prior_path": str(dest), "beats": prior["beats_geometry"]}


def _reuse_ok(case_dir: Path, source_fp: str, prior_fp: str) -> bool:
    marker = case_dir / "COMPLETE.json"
    summary = case_dir / "summary.json"
    if not marker.exists() or not summary.exists():
        return False
    try:
        meta = json.loads(marker.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False
    return meta.get("source_fp") == source_fp and meta.get("prior_fp") == prior_fp


def _run_pool(fn, payloads: list[dict], workers: int) -> list[dict]:
    if workers <= 1 or len(payloads) <= 1:
        return [fn(p) for p in payloads]
    rows = [None] * len(payloads)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        fmap = {pool.submit(fn, p): i for i, p in enumerate(payloads)}
        for fut in as_completed(fmap):
            i = fmap[fut]
            rows[i] = fut.result()
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--shapes", nargs="+", default=["l_shape", "rectangle", "c_shape"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--grids", nargs="+", type=int, default=[20])
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--density-mesh", type=int, default=160)
    parser.add_argument("--restrict-site-grid", type=int, default=8)
    parser.add_argument("--restrict-local-mesh", type=int, default=24)
    parser.add_argument(
        "--integration-method",
        choices=["endpoint_grid", "edge_green"],
        default="edge_green",
    )
    parser.add_argument("--edge-n-gon", type=int, default=256)
    parser.add_argument("--edge-panels", type=int, default=48)
    parser.add_argument("--edge-h-max", type=float, default=0.004)
    parser.add_argument("--edge-cull-sigmas", type=float, default=6.0)
    parser.add_argument("--clamp-margin", action="store_true")
    parser.add_argument("--workers", type=int, default=0, help="0 = all usable logical CPUs")
    parser.add_argument("--serial", action="store_true")
    parser.add_argument("--benchmark-parallel", action="store_true")
    parser.add_argument("--priors-only", action="store_true")
    parser.add_argument("--only", nargs="*", default=None, help="Restrict to keys like l_shape:2 c_shape:5")
    args = parser.parse_args()

    from dbact.cpu_budget import (
        ResourceSampler,
        choose_outer_workers,
        detect_cpu_environment,
        dump_json,
        limit_blas_threads,
    )

    limit_blas_threads(1)
    env = detect_cpu_environment()
    stamp = time.strftime("%Y-%m-%d")
    out = args.out or (ROOT / "artifacts" / f"apriori_certificate_repair_{stamp}")
    out.mkdir(parents=True, exist_ok=True)
    dump_json(out / "cpu_environment.json", env.as_dict())
    sha = git_sha()
    source_fp = source_fingerprint()
    prior_fp = prior_rule_fingerprint(args)
    shapes = args.shapes
    seeds = args.seeds
    grids = args.grids
    frames = 4 if args.smoke else args.frames
    if args.smoke:
        shapes, seeds, grids = ["l_shape"], [2], [20]
        frames = 4
    elif args.baseline_only:
        shapes, seeds, grids = ["l_shape"], [2], [20]
    elif args.benchmark_parallel:
        shapes, seeds, grids = ["l_shape"], [2, 5], [20]
        frames = min(frames, 8)

    run_list = []
    only = None
    if args.only:
        only = {(part.split(":")[0], int(part.split(":")[1])) for part in args.only}
    for shape in shapes:
        for seed in seeds:
            for grid in grids:
                if only is not None and (shape, seed) not in only:
                    continue
                run_list.append({"shape": shape, "seed": seed, "grid": grid})
    if not run_list:
        raise SystemExit("empty run list")
    plan = choose_outer_workers(len(run_list), env)
    if args.serial:
        plan["outer_workers"] = 1
        plan["inner_cvt_workers"] = 1
        plan["reason"] = "user_forced_serial"
    elif args.workers > 0:
        plan["outer_workers"] = max(1, min(int(args.workers), env.usable_cpus, len(run_list)))
        plan["inner_cvt_workers"] = max(1, env.usable_cpus // plan["outer_workers"])
        plan["reason"] = "user_workers_capped_by_usable_cpus"
    dump_json(out / "cpu_plan.json", plan)
    sampler = ResourceSampler(out / "resource_usage.csv", interval_s=2.0)
    sampler.start()

    dump(
        out / "branch_manifest.json",
        {
            "branch": "feat/apriori-certificate-repair",
            "code_sha": sha,
            "source_fingerprint": source_fp,
            "prior_rule_fingerprint": prior_fp,
            "base_commit": "7b9fb36d94ffba572881c11b8050d1d4cee2811d",
            "created": stamp,
            "smoke": bool(args.smoke),
            "baseline_only": bool(args.baseline_only),
            "shapes": shapes,
            "seeds": seeds,
            "grids": grids,
            "frames": frames,
            "observer": {"ntheta": args.ntheta, "nradial": args.nradial},
            "integration_method": args.integration_method,
            "edge_n_gon": args.edge_n_gon,
            "edge_panels": args.edge_panels,
            "edge_h_max": args.edge_h_max,
            "edge_cull_sigmas": args.edge_cull_sigmas,
            "clamp_margin": bool(args.clamp_margin),
            "cpu_plan": plan,
            "controller_grid_vs_eval_grid": (
                "endpoint_grid uses LocalCVT n; edge_green uses n_gon + frozen h_max "
                "(grid_resolution kept for baseline comparison only)"
            ),
            "reads_old_artifacts": False,
        },
    )
    dump(out / "run_list.json", {"cases": run_list, "n": len(run_list)})

    cfgs = {}
    prior_payloads = []
    for item in run_list:
        shape, seed, grid = item["shape"], item["seed"], item["grid"]
        key = case_key(shape, seed, grid)
        cfg_path = out / "configs" / f"{shape}_n{grid}.yaml"
        base = load_yaml(SHAPES[shape])
        cfg = write_overlay_config(
            base,
            grid,
            cfg_path,
            integration_method=args.integration_method,
            edge_n_gon=args.edge_n_gon,
            edge_panels=args.edge_panels,
            edge_h_max=args.edge_h_max,
            edge_cull_sigmas=args.edge_cull_sigmas,
            clamp_margin=bool(args.clamp_margin),
        )
        cfgs[key] = cfg
        prior_path = out / "priors" / f"{key}.json"
        prior_payloads.append(
            {
                "key": key,
                "cfg": cfg,
                "seed": seed,
                "frames": frames,
                "prior_path": str(prior_path),
                "ntheta": args.ntheta,
                "nradial": args.nradial,
                "density_mesh": args.density_mesh,
                "restrict_site_grid": args.restrict_site_grid,
                "restrict_local_mesh": args.restrict_local_mesh,
            }
        )

    print(f"[cpu] usable={env.usable_cpus} outer={plan['outer_workers']} cvt={plan['inner_cvt_workers']}", flush=True)
    t_prior = time.perf_counter()
    prior_rows = _run_pool(worker_prepare_prior, prior_payloads, plan["outer_workers"])
    prior_store = {}
    for row, payload in zip(prior_rows, prior_payloads):
        prior_store[payload["key"]] = json.loads(Path(payload["prior_path"]).read_text(encoding="utf-8"))
    dump(out / "prior_constants.json", prior_store)
    dump(
        out / "prior_screen.json",
        {
            key: {
                "B_E": prior_store[key]["B_E"]["moment_rigorous"],
                "B_J_prior": prior_store[key]["B_J"]["prior_certificate_crudeH_rigorousE"]["B_J_prior"],
                "B_J_geom": prior_store[key]["B_J"]["geometric"]["B_J_geom"],
                "beats": prior_store[key]["beats_geometry"],
                "labels": prior_store[key]["labels"],
            }
            for key in prior_store
        },
    )
    print(f"[prior] {len(prior_store)} frozen in {time.perf_counter()-t_prior:.1f}s", flush=True)
    if args.priors_only:
        sampler.stop()
        print(json.dumps({"out": str(out), "priors_only": True, "n": len(prior_store)}, indent=2), flush=True)
        return

    case_payloads = []
    reused = []
    for item in run_list:
        shape, seed, grid = item["shape"], item["seed"], item["grid"]
        key = case_key(shape, seed, grid)
        case_dir = out / "runs" / shape / f"n{grid}" / f"seed_{seed}"
        if (args.resume or args.skip_existing) and _reuse_ok(case_dir, source_fp, prior_fp):
            summary = json.loads((case_dir / "summary.json").read_text(encoding="utf-8"))
            summary.update({"shape": shape, "seed": seed, "grid_resolution": grid})
            summary["metrics_csv"] = str(case_dir / "metrics.csv")
            reused.append(summary)
            print(f"[resume] {key}", flush=True)
            continue
        case_payloads.append(
            {
                "key": key,
                "shape": shape,
                "seed": seed,
                "cfg": cfgs[key],
                "case_dir": str(case_dir),
                "prior_path": str(out / "priors" / f"{key}.json"),
                "frames": frames,
                "ntheta": args.ntheta,
                "nradial": args.nradial,
                "code_sha": sha,
                "source_fp": source_fp,
                "prior_fp": prior_fp,
                "cvt_workers": plan["inner_cvt_workers"],
            }
        )

    t0 = time.perf_counter()
    cpu0 = time.process_time()
    run_rows = _run_pool(worker_run_case, case_payloads, plan["outer_workers"])
    wall = time.perf_counter() - t0
    cpu_time = time.process_time() - cpu0
    rows = reused + run_rows
    rows.sort(key=lambda r: (r.get("shape", ""), r.get("seed", 0), r.get("grid_resolution", 0)))

    dump_json(
        out / "parallel_benchmark.json",
        {
            "mode": "serial" if plan["outer_workers"] <= 1 else "process_pool",
            "n_cases": len(case_payloads),
            "n_reused": len(reused),
            "outer_workers": plan["outer_workers"],
            "inner_cvt_workers": plan["inner_cvt_workers"],
            "usable_cpus": env.usable_cpus,
            "logical_cpus": env.logical_cpus,
            "physical_cpus": env.physical_cpus,
            "wall_seconds": wall,
            "parent_cpu_seconds": cpu_time,
            "reason": plan["reason"],
            "source_fingerprint": source_fp,
        },
    )
    write_summary_csv(out / "summary.csv", rows)
    dump(out / "summary.json", {"cases": rows, "n": len(rows)})
    src_der = ROOT / "docs" / "apriori_centroid_bound_derivation.md"
    if src_der.exists():
        shutil.copy2(src_der, out / "derivation.md")
    figs = []
    if not args.smoke:
        try:
            figs = make_figures(rows, out)
        except Exception as exc:  # pragma: no cover
            print(f"[warn] figures failed: {exc}", flush=True)
    dump(out / "figures_written.json", figs)
    sampler.stop()
    print(
        json.dumps(
            {
                "out": str(out),
                "n_cases": len(rows),
                "workers": plan["outer_workers"],
                "wall_seconds": wall,
            },
            indent=2,
        ),
        flush=True,
    )


if __name__ == "__main__":
    from dbact.cpu_budget import limit_blas_threads

    limit_blas_threads(1)
    main()
