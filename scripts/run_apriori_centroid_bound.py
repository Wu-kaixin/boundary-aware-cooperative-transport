"""Fresh 27-case a priori centroid-bound experiment (static sampled theorem_mode).

Does not read previous theorem_audit artifacts. Priors are written before each
closed-loop run and are never fitted to trajectory E.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
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
) -> dict:
    cfg = overlay_grid_resolution(base, grid)
    cfg.setdefault("controller", {})
    cfg["controller"]["integration_method"] = str(integration_method)
    cfg["controller"]["edge_n_gon"] = int(edge_n_gon)
    cfg["controller"]["edge_panels"] = int(edge_panels)
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
            step_rows.append(
                {
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
        ax.set_yscale("log")
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
        ax.set_yscale("log")
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
                ax.set_yscale("log")
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--shapes", nargs="+", default=["l_shape", "rectangle", "c_shape"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--grids", nargs="+", type=int, default=[20, 40, 80])
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--ntheta", type=int, default=128)
    parser.add_argument("--nradial", type=int, default=12)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--skip-existing", action="store_true")
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
    args = parser.parse_args()

    stamp = time.strftime("%Y-%m-%d")
    out = args.out or (ROOT / "artifacts" / f"apriori_centroid_bound_{stamp}")
    out.mkdir(parents=True, exist_ok=True)
    sha = git_sha()
    shapes = args.shapes
    seeds = args.seeds
    grids = args.grids
    frames = 4 if args.smoke else args.frames
    if args.smoke:
        shapes, seeds, grids = ["l_shape"], [2], [20]
    elif args.baseline_only:
        shapes, seeds, grids = ["l_shape"], [2], [20]

    dump(
        out / "branch_manifest.json",
        {
            "branch": "feat/apriori-centroid-bound",
            "code_sha": sha,
            "base_sampled_theorem_mode": "a531515",
            "main_frozen": "98ba28e",
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
            "controller_grid_vs_eval_grid": (
                "endpoint_grid uses LocalCVT n; edge_green uses n_gon/panels "
                "(grid_resolution kept for baseline comparison only)"
            ),
            "reads_old_artifacts": False,
        },
    )

    run_list = []
    for shape in shapes:
        for seed in seeds:
            for grid in grids:
                run_list.append({"shape": shape, "seed": seed, "grid": grid})
    dump(out / "run_list.json", {"cases": run_list, "n": len(run_list)})

    geometry_caches: dict[str, dict] = {}
    observers: dict[str, UniformOffsetObserver] = {}
    observers_fine: dict[str, dict] = {}
    rows = []
    prior_store = {}

    for item in run_list:
        shape, seed, grid = item["shape"], item["seed"], item["grid"]
        key = case_key(shape, seed, grid)
        case_dir = out / "runs" / shape / f"n{grid}" / f"seed_{seed}"
        cfg_path = out / "configs" / f"{shape}_n{grid}.yaml"
        base = load_yaml(SHAPES[shape])
        cfg = write_overlay_config(
            base,
            grid,
            cfg_path,
            integration_method=args.integration_method,
            edge_n_gon=args.edge_n_gon,
            edge_panels=args.edge_panels,
        )
        if shape not in observers:
            observers[shape] = make_observer(cfg, args.ntheta, args.nradial)
            observers_fine[shape] = {
                "fine256": make_observer(cfg, 256, 16),
            }
            geometry_caches[shape] = {}
        print(f"[prior] {key}", flush=True)
        _p0, _verts, prior = prepare_p0_and_prior(
            cfg,
            seed,
            observers[shape],
            observers_fine[shape],
            frames,
            geometry_caches[shape],
            args.density_mesh,
            args.restrict_site_grid,
            args.restrict_local_mesh,
        )
        dump(out / "priors" / f"{key}.json", prior)
        prior_store[key] = prior
        if args.skip_existing and (case_dir / "summary.json").exists():
            summary = json.loads((case_dir / "summary.json").read_text(encoding="utf-8"))
            summary.update({"shape": shape, "seed": seed, "grid_resolution": grid})
            summary["metrics_csv"] = str(case_dir / "metrics.csv")
            rows.append(summary)
            print(f"[skip] {key}", flush=True)
            continue
        print(f"[run] {key} frames={frames}", flush=True)
        result = run_fresh_case(cfg, case_dir, seed, frames, observers[shape], prior, sha)
        summary = result["summary"]
        summary["shape"] = shape
        summary["metrics_csv"] = str(case_dir / "metrics.csv")
        rows.append(summary)
        print(
            f"[{key}] complete={summary['complete_30s']} K0={summary['theorem_conditions_hold_full_horizon']} "
            f"J_bar={summary['J_bar']} E_bar={summary['E_bar']} "
            f"priorJ={summary['B_J_prior_P0H_rigorousE']} geom={summary['B_J_geom']}",
            flush=True,
        )

    dump(out / "prior_constants.json", prior_store)
    write_summary_csv(out / "summary.csv", rows)
    dump(out / "summary.json", {"cases": rows, "n": len(rows)})
    src_der = ROOT / "docs" / "apriori_centroid_bound_derivation.md"
    if src_der.exists():
        shutil.copy2(src_der, out / "derivation.md")
    figs = []
    if not args.smoke:
        try:
            figs = make_figures(rows, out)
        except Exception as exc:  # pragma: no cover - plotting must not erase results
            print(f"[warn] figures failed: {exc}", flush=True)
    dump(out / "figures_written.json", figs)
    print(json.dumps({"out": str(out), "n_cases": len(rows)}, indent=2), flush=True)


if __name__ == "__main__":
    for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ.setdefault(_k, "1")
    main()
