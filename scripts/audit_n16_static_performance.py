"""N=16 static sampled theorem_mode performance audit.

Independent polar/erf observer for the paper truncated coverage cost under a
*uniform* cage offset (lead = cage). Controller grid centroids are never treated
as reference truth.

Outputs raw step data, residual curves, geometric vs Theorem-1 formula numbers,
sampling diagnostics H*_{k+1}-H*_k vs Delta g*^T U, and assumption annotations.
Does not loosen failure thresholds; aborts are retained as incomplete cases.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from numpy.polynomial.legendre import leggauss
from scipy.integrate import cumulative_trapezoid, quad
from scipy.special import erf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


class UniformOffsetObserver:
    """Paper-style truncated cell integrals for a uniform-offset Gaussian edge density.

    Oracle map used by the controller is a discrete arc-length sample of the same
    offset curve; this observer integrates the continuous edge measure analytically
    (erf line integrals) and does not reuse the controller 20x20 grid.
    """

    def __init__(self, domain, radius, sigma, floor, cage_offset, ntheta=256, nradial=16):
        self.domain = np.asarray(domain, dtype=float)
        self.R = float(radius)
        self.sigma = float(sigma)
        self.floor = float(floor)
        self.cage = float(cage_offset)
        theta = 2 * np.pi * (np.arange(ntheta) + 0.5) / ntheta
        self.unit = np.column_stack((np.cos(theta), np.sin(theta)))
        self.dtheta = 2 * np.pi / ntheta
        z, w = leggauss(nradial)
        self.radial_nodes, self.radial_weights = (z + 1) / 2, w / 2

    def edges(self, vertices):
        edges = np.roll(vertices, -1, axis=0) - vertices
        lengths = np.linalg.norm(edges, axis=1)
        tangent = edges / lengths[:, None]
        normal = np.column_stack((tangent[:, 1], -tangent[:, 0]))
        start = vertices + self.cage * normal
        return start, tangent, lengths

    def density(self, queries, vertices):
        queries = np.asarray(queries, dtype=float).reshape(-1, 2)
        start, tangent, lengths = self.edges(vertices)
        result = np.full(len(queries), self.floor)
        scale = math.sqrt(2) * self.sigma
        for p, u, length in zip(start, tangent, lengths):
            delta = queries - p
            along = delta @ u
            perpendicular2 = np.maximum(np.sum(delta * delta, axis=1) - along * along, 0.0)
            result += self.sigma * math.sqrt(np.pi / 2) * np.exp(
                -perpendicular2 / (2 * self.sigma**2)
            ) * (erf((length - along) / scale) + erf(along / scale))
        return result

    def total_mass(self, vertices):
        xmin, xmax, ymin, ymax = self.domain
        answer = self.floor * (xmax - xmin) * (ymax - ymin)
        scale = math.sqrt(2) * self.sigma
        start, tangent, lengths = self.edges(vertices)
        for p, u, length in zip(start, tangent, lengths):

            def integrand(s, p=p, u=u):
                x, y = p + s * u
                return (
                    np.pi
                    * self.sigma**2
                    / 2
                    * (erf((xmax - x) / scale) - erf((xmin - x) / scale))
                    * (erf((ymax - y) / scale) - erf((ymin - y) / scale))
                )

            answer += quad(integrand, 0, length, epsabs=1e-11, epsrel=1e-11)[0]
        return float(answer)

    def evaluate(self, positions, vertices):
        p = np.asarray(positions, dtype=float).reshape(-1, 2)
        n, nt = len(p), len(self.unit)
        reach = np.full((n, nt), self.R)
        xmin, xmax, ymin, ymax = self.domain
        if (
            np.any(p[:, 0] < xmin - 1e-10)
            or np.any(p[:, 0] > xmax + 1e-10)
            or np.any(p[:, 1] < ymin - 1e-10)
            or np.any(p[:, 1] > ymax + 1e-10)
        ):
            raise ValueError("Observer requires sites inside D")
        for axis, low, high in [(0, xmin, xmax), (1, ymin, ymax)]:
            v = self.unit[:, axis]
            wall = np.where(
                v[None, :] > 0,
                (high - p[:, axis, None]) / np.maximum(v[None, :], 1e-30),
                (p[:, axis, None] - low) / np.maximum(-v[None, :], 1e-30),
            )
            reach = np.minimum(reach, wall)
        for i in range(n):
            difference = p - p[i]
            squared = np.sum(difference * difference, axis=1)
            dot = difference @ self.unit.T
            denominator = 2 * np.maximum(dot, 1e-30)
            cap = squared[:, None] / denominator
            cap[dot <= 1e-14] = np.inf
            cap[i] = np.inf
            reach[i] = np.minimum(reach[i], np.min(cap, axis=0))
        reach = np.maximum(reach, 0.0)
        r = reach[:, :, None] * self.radial_nodes
        q = p[:, None, None, :] + r[:, :, :, None] * self.unit[None, :, None, :]
        phi = self.density(q.reshape(-1, 2), vertices).reshape(r.shape)
        w = self.dtheta * reach[:, :, None] * self.radial_weights * r
        weighted = phi * w
        mass = np.sum(weighted, axis=(1, 2))
        shift = np.sum(
            weighted[:, :, :, None] * r[:, :, :, None] * self.unit[None, :, None, :],
            axis=(1, 2),
        )
        # Guard empty-mass cells (should be rare with phi0 > 0).
        safe = np.maximum(mass, 1e-30)
        centroid = p + shift / safe[:, None]
        gradient = -2 * shift
        H = self.R**2 * self.total_mass(vertices) + np.sum((r * r - self.R**2) * weighted)
        return dict(
            mass=mass,
            centroid=centroid,
            gradient=gradient,
            gradient2=float(np.sum(gradient * gradient)),
            centroid2=float(np.sum((centroid - p) ** 2)),
            H=float(H),
        )


def theoretical_constants(config, cargo_vertices):
    from dbact.geometry import polygon_perimeter
    from dbact_sim.scenarios import controller_params_from_config

    p = controller_params_from_config(config)
    L = polygon_perimeter(cargo_vertices)
    N = int(config["agents"]["count"])
    R, kc, u = p.local_radius, p.kp_cage, p.max_speed
    width = config["domain"]["xmax"] - config["domain"]["xmin"]
    height = config["domain"]["ymax"] - config["domain"]["ymin"]
    r0 = min(R, p.d_min / 2, width / 2, height / 2)
    mlo = p.base_density * np.pi * r0 * r0 / 4
    mhi = np.pi * R * R * (p.base_density + L)
    a = kc / (2 * mhi)
    d_terms = math.sqrt(N) * (kc * 2 * R + max(kc * R - u, 0) + 2 * u)
    d_alt = math.sqrt(N) * (u + kc * R)
    d = min(d_terms, d_alt)
    CK = 2 * np.pi * p.sigma**2
    M = p.base_density * width * height + CK * L
    h = 2 * R / (p.grid_resolution - 1)
    rho = math.sqrt(2) * h / 2
    return {
        "N": N,
        "R": R,
        "kc": kc,
        "umax": u,
        "ds": p.d_min,
        "sigma": p.sigma,
        "phi0": p.base_density,
        "cage_offset": p.cage_offset,
        "lead_offset": p.lead_offset,
        "dt": float(config["dt"]),
        "grid": p.grid_resolution,
        "Lmax": L,
        "CK": CK,
        "m_minus": mlo,
        "m_plus": mhi,
        "a_minus": a,
        "alpha": a / 2,
        "dbar_speed_alternative": d_alt,
        "dbar": d,
        "beta_static": d * d / (2 * a),
        "B_gradient_static": d * d / (a * a),
        "B_gradient_geometry_total": 4 * R * R * M * M,
        "B_centroid_geometry": N * R * R,
        "M_total_upper": M,
        "eta_m_floor": p.base_density * (4 * np.pi * R * rho + np.pi * rho * rho),
        "gamma_agent_dt": p.gamma_agent * float(config["dt"]),
        "note": (
            "Static uniform-offset reference. Formula B_g,inf uses the optimistic "
            "static disturbance bound; it is NOT a certified guarantee for this run."
        ),
    }


def oracle_vs_continuous_note(spacing: float, cage: float) -> dict:
    return {
        "oracle_map": (
            f"Discrete boundary samples along true polygon edges, outward normals, "
            f"arc_length per sample, confidence=1, offset applied in density as "
            f"uniform d_c={cage} (no lead). Spacing≈{spacing} m."
        ),
        "continuous_reference_density": (
            "Analytic Gaussian line integrals on the same uniform-offset edge measure "
            "plus floor density; independent of oracle sample spacing."
        ),
        "difference": (
            "Controller CVT uses the oracle discrete atomic mixture on a 20x20 local "
            "grid. Observer uses continuous edge measure + polar quadrature. They share "
            "geometry and offset law but are not the same numerical object."
        ),
        "vs_old_transport_reference": (
            "Old audit seeds used transport with lead_offset and moving cargo. This "
            "audit uses static uniform cage_offset only; old residual numbers must not "
            "be reused."
        ),
    }


def assumption_table(config, consts) -> list[dict]:
    p = config["controller"]
    return [
        {
            "item": "Assumption 3 same Gaussian kernel (offset)",
            "status": "structurally_aligned_in_static_theorem_mode",
            "detail": "density_mode=offset, uniform cage_offset, gap/explore=0, lead_offset=null",
        },
        {
            "item": "Assumption 5 local Lipschitz continuous-time feedback",
            "status": "not_claimed",
            "detail": "Hard Voronoi/grid membership retained; sampled model used instead",
        },
        {
            "item": "Sample-and-hold execution without clipping",
            "status": "implemented_in_theorem_mode",
            "detail": "Wall rows + abort; clipping disabled",
        },
        {
            "item": "Assumption 2 true object motion bound",
            "status": "satisfied_as_zero_by_construction",
            "detail": "Cargo frozen movable=false; v_obj forced to 0",
        },
        {
            "item": "Fine-grid mass certificate (29)",
            "status": "fails_as_before",
            "detail": f"eta_m_floor={consts['eta_m_floor']:.3e} vs m_-={consts['m_minus']:.3e}",
        },
        {
            "item": "Theorem 1 coarse asymptotic bound as quantitative claim",
            "status": "not_established",
            "detail": "Formula value reported only as diagnostic scale; assumptions incomplete",
        },
        {
            "item": "SEARCH/APPROACH/REDEPLOY/TRANSPORT modes",
            "status": "excluded",
            "detail": "Only theorem_cvt executed",
        },
        {
            "item": "gamma_agent * Delta <= 1",
            "status": "ok" if consts["gamma_agent_dt"] <= 1 + 1e-9 else "fail",
            "detail": f"gamma_agent*dt={consts['gamma_agent_dt']}",
        },
    ]


def run_case(config_path: Path, out_dir: Path, seed: int, frames: int, observer_stride: int) -> dict:
    from dbact.theorem_mode import TheoremModeAbort
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import controller_params_from_config

    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = controller_params_from_config(cfg)
    assert params.theorem_mode is True
    assert params.lead_offset is None
    assert float(params.gap_gain) == 0.0

    started = time.perf_counter()
    env = SimulationEnvironment(cfg, seed=seed)
    vertices0 = env.cargoes[0].vertices.copy()
    domain = env.domain
    observer = UniformOffsetObserver(
        domain, params.local_radius, params.sigma, params.base_density, params.cage_offset, 256, 16
    )
    refined = UniformOffsetObserver(
        domain, params.local_radius, params.sigma, params.base_density, params.cage_offset, 1024, 32
    )

    rows = []
    positions_all = []
    nominal_all = []
    command_all = []
    abort = None
    completed_frames = 0

    # Frame 0 observation before any control (initial state).
    P0 = np.vstack([a.position for a in env.agents])
    obs0 = observer.evaluate(P0, vertices0)
    rows.append(
        {
            "frame": 0,
            "time": 0.0,
            "kind": "initial",
            "H_star": obs0["H"],
            "gradient2": obs0["gradient2"],
            "centroid2": obs0["centroid2"],
            "g_dot_U": 0.0,
            "cost_delta_controller_grid": None,
            "H_star_delta": None,
            "sampling_remainder": None,
            "saturation_effect": 0.0,
            "safety_modification_norm": 0.0,
            "hold_min_pair": None,
            "hold_min_object_clearance": None,
            "modes": [],
            "solver_statuses": [],
        }
    )

    try:
        for k in range(frames):
            P = np.vstack([a.position for a in env.agents])
            cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
            rec = env.controller.last_theorem_record
            U = np.vstack([c.velocity for c in cmds])
            U_nom = rec.nominal_velocities
            # Safety / saturation diagnostics on FINAL U before apply.
            sat = float(np.linalg.norm(U_nom - U))  # includes safety+cap relative to pre-QP? 
            # Better: modification from filter is in rec via command vs nominal.
            safety_mod = float(np.linalg.norm(U - U_nom))

            observe = (k % observer_stride == 0) or k == frames - 1
            if observe:
                obs = observer.evaluate(P, vertices0)
                g = obs["gradient"]
                g_dot_U = float(np.sum(g * U))
                # Predicted first-order change along hold (diagnostic only).
                delta_pred = float(env.dt * g_dot_U)
            else:
                obs = None
                g_dot_U = None
                delta_pred = None

            env.controller.apply_commands(env.agents, cmds, env.dt)
            env.engine.step(env.cargoes, env.agents, env.dt)
            env.t += env.dt
            env._record()
            completed_frames = k + 1

            P_next = np.vstack([a.position for a in env.agents])
            positions_all.append(P)
            nominal_all.append(U_nom)
            command_all.append(U)

            H_next = None
            H_delta = None
            remainder = None
            if observe:
                obs_next = observer.evaluate(P_next, vertices0)
                H_next = obs_next["H"]
                H_delta = float(H_next - obs["H"])
                remainder = float(H_delta - delta_pred) if delta_pred is not None else None
                rows.append(
                    {
                        "frame": k,
                        "time": float(k * env.dt),
                        "kind": "control",
                        "H_star": obs["H"],
                        "H_star_next": H_next,
                        "H_star_delta": H_delta,
                        "gradient2": obs["gradient2"],
                        "centroid2": obs["centroid2"],
                        "g_dot_U": g_dot_U,
                        "Delta_g_dot_U": delta_pred,
                        "sampling_remainder": remainder,
                        "mass_star": obs["mass"].tolist(),
                        "centroid_star": obs["centroid"].tolist(),
                        "gradient_star": obs["gradient"].tolist(),
                        "cell_mass_controller": rec.cell_mass.tolist(),
                        "centroid_hat_controller": rec.cell_centroid.tolist(),
                        "positions": P.tolist(),
                        "u_nominal": U_nom.tolist(),
                        "u_command": U.tolist(),
                        "safety_modification_norm": safety_mod,
                        "speed_max": float(np.max(np.linalg.norm(U, axis=1))),
                        "hold_min_pair": rec.hold_min_pair_distance,
                        "hold_min_object_clearance": rec.hold_min_object_clearance,
                        "hold_object_clearance_sampled": rec.hold_object_clearance_sampled,
                        "hold_object_clearance_margin": rec.hold_object_clearance_margin,
                        "agent_residual_min": float(np.min(rec.agent_residual_min)),
                        "wall_residual_min": float(np.min(rec.wall_residual_min)),
                        "object_residual_min": float(np.min(rec.object_residual_min)),
                        "wall_rows_active": rec.wall_rows_active,
                        "barrier_scale": rec.barrier_scale.tolist(),
                        "modes": rec.modes,
                        "solver_statuses": rec.solver_status,
                        "map_source": rec.map_source,
                        "cost_H_controller_grid": rec.cost_H,
                        "cost_delta_controller_grid": rec.cost_delta,
                    }
                )
    except TheoremModeAbort as exc:
        abort = exc.as_dict()
        env.theorem_abort = abort

    # Quadrature refinement at a few snapshots that exist.
    refinements = []
    for row in rows:
        if row.get("kind") != "control":
            continue
        if row["frame"] in {0, 150, 300, 450} or (
            completed_frames > 0 and row["frame"] == completed_frames - 1
        ):
            P = np.asarray(row["positions"], dtype=float)
            coarse = observer.evaluate(P, vertices0)
            fine = refined.evaluate(P, vertices0)
            refinements.append(
                {
                    "frame": row["frame"],
                    "gradient2_coarse": coarse["gradient2"],
                    "gradient2_fine": fine["gradient2"],
                    "centroid2_coarse": coarse["centroid2"],
                    "centroid2_fine": fine["centroid2"],
                    "gradient2_rel": abs(fine["gradient2"] - coarse["gradient2"])
                    / max(fine["gradient2"], 1e-30),
                    "centroid2_rel": abs(fine["centroid2"] - coarse["centroid2"])
                    / max(fine["centroid2"], 1e-30),
                    "centroid_max_diff": float(
                        np.max(np.linalg.norm(fine["centroid"] - coarse["centroid"], axis=1))
                    ),
                }
            )

    control_rows = [r for r in rows if r.get("kind") == "control"]
    times = np.asarray([r["time"] for r in control_rows], dtype=float) if control_rows else np.zeros(0)
    g2 = np.asarray([r["gradient2"] for r in control_rows], dtype=float) if control_rows else np.zeros(0)
    c2 = np.asarray([r["centroid2"] for r in control_rows], dtype=float) if control_rows else np.zeros(0)

    def time_average(y):
        if len(y) < 2:
            return float(y[-1]) if len(y) else float("nan")
        return float(cumulative_trapezoid(y, times, initial=0.0)[-1] / max(times[-1], 1e-30))

    summary = {
        "seed": seed,
        "frames_requested": frames,
        "frames_completed": completed_frames,
        "physical_seconds_completed": completed_frames * float(cfg["dt"]),
        "complete_30s": completed_frames >= frames and abort is None,
        "abort": abort,
        "wall_seconds": time.perf_counter() - started,
        "modes_seen": sorted({m for r in control_rows for m in r.get("modes", [])}),
        "solver_status_counts": {},
        "barrier_scalings": int(
            sum(1 for r in control_rows for s in r.get("barrier_scale", []) if s < 1.0 - 1e-12)
        ),
        "hold_min_pair": float(min((r["hold_min_pair"] for r in control_rows), default=float("inf"))),
        "hold_min_object_clearance": float(
            min((r["hold_min_object_clearance"] for r in control_rows), default=float("inf"))
        ),
        "agent_residual_min": float(
            min((r["agent_residual_min"] for r in control_rows), default=float("nan"))
        ),
        "wall_residual_min": float(
            min((r["wall_residual_min"] for r in control_rows), default=float("nan"))
        ),
        "gradient2_time_average": time_average(g2),
        "centroid2_time_average": time_average(c2),
        "gradient2_final": float(g2[-1]) if len(g2) else None,
        "centroid2_final": float(c2[-1]) if len(c2) else None,
        "H0": rows[0]["H_star"],
        "refinement": refinements,
        "n_observer_samples": len(control_rows),
    }
    for r in control_rows:
        for st in r.get("solver_statuses", []):
            summary["solver_status_counts"][st] = summary["solver_status_counts"].get(st, 0) + 1

    # Persist raw arrays.
    if positions_all:
        np.savez_compressed(
            out_dir / "trajectory.npz",
            positions=np.stack(positions_all),
            u_nominal=np.stack(nominal_all),
            u_command=np.stack(command_all),
            vertices=vertices0,
        )
    dump(out_dir / "observer_rows.json", rows)
    # CSV of main residual series
    with (out_dir / "residuals.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=[
                "frame",
                "time",
                "H_star",
                "H_star_delta",
                "gradient2",
                "centroid2",
                "g_dot_U",
                "Delta_g_dot_U",
                "sampling_remainder",
                "safety_modification_norm",
                "hold_min_pair",
                "hold_min_object_clearance",
            ],
        )
        writer.writeheader()
        for r in control_rows:
            writer.writerow({k: r.get(k) for k in writer.fieldnames})
    dump(out_dir / "summary.json", summary)
    return summary, rows, control_rows


def make_plots(out: Path, case_results: dict, consts: dict) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for seed, payload in case_results.items():
        rows = payload["control_rows"]
        if not rows:
            continue
        t = [r["time"] for r in rows]
        g2 = [r["gradient2"] for r in rows]
        c2 = [r["centroid2"] for r in rows]
        # cumulative trapezoid averages
        tt = np.asarray(t)
        g2a = cumulative_trapezoid(g2, tt, initial=0.0) / np.maximum(tt, 1e-30)
        c2a = cumulative_trapezoid(c2, tt, initial=0.0) / np.maximum(tt, 1e-30)
        axes[0, 0].plot(t, g2, alpha=0.35, label=f"seed {seed} inst")
        axes[0, 0].plot(t, g2a, label=f"seed {seed} avg")
        axes[1, 0].plot(t, c2, alpha=0.35, label=f"seed {seed} inst")
        axes[1, 0].plot(t, c2a, label=f"seed {seed} avg")
        rem = [r["sampling_remainder"] for r in rows if r.get("sampling_remainder") is not None]
        tr = [r["time"] for r in rows if r.get("sampling_remainder") is not None]
        if rem:
            axes[0, 1].plot(tr, rem, label=f"seed {seed}")
        dH = [r["H_star_delta"] for r in rows if r.get("H_star_delta") is not None]
        dpred = [r["Delta_g_dot_U"] for r in rows if r.get("Delta_g_dot_U") is not None]
        td = [r["time"] for r in rows if r.get("H_star_delta") is not None]
        if dH:
            axes[1, 1].plot(td, dH, label=f"seed {seed} ΔH*")
            axes[1, 1].plot(td, dpred, linestyle="--", alpha=0.7, label=f"seed {seed} Δ g*^T U")

    axes[0, 0].axhline(consts["B_gradient_geometry_total"], color="k", ls=":", label="geom bound")
    axes[0, 0].axhline(consts["B_gradient_static"], color="r", ls=":", alpha=0.5, label="Thm1 static formula*")
    axes[0, 0].set_ylabel(r"$||g^\star||^2$")
    axes[0, 0].set_title("Gradient residual (instantaneous + time average)")
    axes[0, 0].set_yscale("log")
    axes[0, 0].legend(fontsize=7)
    axes[0, 0].grid(alpha=0.2)

    axes[1, 0].axhline(consts["B_centroid_geometry"], color="k", ls=":", label="geom NR^2")
    axes[1, 0].set_ylabel(r"$\sum||p_i-c_i^\star||^2$")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_title("Centroid residual sum")
    axes[1, 0].set_yscale("log")
    axes[1, 0].legend(fontsize=7)
    axes[1, 0].grid(alpha=0.2)

    axes[0, 1].set_title("Sampling remainder H*_{k+1}-H*_k - Δ g*_kᵀ U_k")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].legend(fontsize=7)
    axes[0, 1].grid(alpha=0.2)

    axes[1, 1].set_title("Cost change vs first-order prediction (diagnostic)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].legend(fontsize=7)
    axes[1, 1].grid(alpha=0.2)

    fig.suptitle(
        "N=16 static uniform-offset theorem_mode | independent observer | *formula not a certificate",
        fontsize=12,
    )
    fig.savefig(out / "n16_residuals.png", dpi=160)
    plt.close(fig)


def environment_snapshot(repo: Path) -> dict:
    import numpy
    import scipy
    import yaml as _yaml

    try:
        import matplotlib
        mpl_v = matplotlib.__version__
    except Exception:
        mpl_v = None
    return {
        "os": platform.platform(),
        "python": sys.version,
        "numpy": numpy.__version__,
        "scipy": scipy.__version__,
        "matplotlib": mpl_v,
        "PyYAML": _yaml.__version__,
        "numpy_blas": getattr(numpy.__config__, "show", lambda: "n/a")()
        if False
        else str(numpy.__config__.blas_ilp64_opt_info if hasattr(numpy.__config__, "blas_ilp64_opt_info") else "see np.show_config"),
        "thread_env_recommended": {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        },
        "repo": str(repo),
        "git_sha": subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
        "branch": subprocess.check_output(
            ["git", "-C", str(repo), "branch", "--show-current"], text=True
        ).strip(),
    }


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
        default=Path(r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\n16_static_audit"),
    )
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--observer-stride", type=int, default=5)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    from dbact_sim.scenarios import build_cargoes

    cargo = build_cargoes(cfg, seed=0)[0]
    consts = theoretical_constants(cfg, cargo.vertices)
    dump(args.out / "constants.json", consts)
    dump(
        args.out / "oracle_vs_reference.json",
        oracle_vs_continuous_note(cfg["controller"]["theorem_oracle_spacing"], cfg["controller"]["cage_offset"]),
    )
    dump(args.out / "assumption_annotations.json", assumption_table(cfg, consts))
    dump(args.out / "environment_snapshot.json", environment_snapshot(ROOT))

    case_results = {}
    summaries = []
    for seed in args.seeds:
        seed_dir = args.out / f"seed_{seed}"
        summary, rows, control_rows = run_case(
            args.config, seed_dir, seed, args.frames, args.observer_stride
        )
        summaries.append(summary)
        case_results[seed] = {"summary": summary, "control_rows": control_rows}
        print(
            f"seed {seed}: frames={summary['frames_completed']}/{args.frames} "
            f"complete={summary['complete_30s']} abort={summary['abort']} "
            f"g2_avg={summary['gradient2_time_average']:.6g} "
            f"c2_avg={summary['centroid2_time_average']:.6g}",
            flush=True,
        )

    make_plots(args.out, case_results, consts)

    # Comparison table: formula / geometry / observed
    comparison = []
    for s in summaries:
        T = s["physical_seconds_completed"]
        H0 = s["H0"]
        finite_static = (
            H0 / (consts["alpha"] * T) + consts["B_gradient_static"] if T > 0 else None
        )
        comparison.append(
            {
                "seed": s["seed"],
                "complete_30s": s["complete_30s"],
                "frames_completed": s["frames_completed"],
                "observed_gradient2_time_average": s["gradient2_time_average"],
                "observed_centroid2_time_average": s["centroid2_time_average"],
                "geometry_gradient_bound": consts["B_gradient_geometry_total"],
                "geometry_centroid_bound": consts["B_centroid_geometry"],
                "theorem1_static_asymptotic_formula": consts["B_gradient_static"],
                "theorem1_static_finite_T_formula": finite_static,
                "formula_is_certificate": False,
                "abort": s["abort"],
            }
        )
    dump(args.out / "run_summaries.json", summaries)
    dump(args.out / "bound_comparison.json", comparison)

    # Sampling diagnostic aggregates
    sampling = {}
    for seed, payload in case_results.items():
        rems = [
            r["sampling_remainder"]
            for r in payload["control_rows"]
            if r.get("sampling_remainder") is not None
        ]
        dH = [
            r["H_star_delta"]
            for r in payload["control_rows"]
            if r.get("H_star_delta") is not None
        ]
        sampling[str(seed)] = {
            "n": len(rems),
            "remainder_mean": float(np.mean(rems)) if rems else None,
            "remainder_abs_mean": float(np.mean(np.abs(rems))) if rems else None,
            "remainder_max_abs": float(np.max(np.abs(rems))) if rems else None,
            "dH_negative_fraction": float(np.mean(np.asarray(dH) < 0)) if dH else None,
            "note": "Remainders are post-hoc diagnostics, not a priori error bounds.",
        }
    dump(args.out / "sampling_diagnostics.json", sampling)
    print(json.dumps({"summaries": summaries, "comparison": comparison}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
