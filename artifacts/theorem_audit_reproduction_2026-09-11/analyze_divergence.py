"""Divergence timeline, object-speed estimators, overlay figures."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

REF = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_2026-09-11_source\DBACT_audit\results")
NEW = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_reproduction_2026-09-11\reproduced_results")
OUT = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_reproduction_2026-09-11")


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def polygon_centroid(verts):
    v = np.asarray(verts, dtype=float)
    closed = np.vstack([v, v[0]])
    x, y = closed[:, 0], closed[:, 1]
    cross = x[:-1] * y[1:] - x[1:] * y[:-1]
    A = 0.5 * np.sum(cross)
    if abs(A) < 1e-18:
        return v.mean(axis=0)
    cx = np.sum((x[:-1] + x[1:]) * cross) / (6 * A)
    cy = np.sum((y[:-1] + y[1:]) * cross) / (6 * A)
    return np.array([cx, cy])


def object_speed_estimators(path, seed):
    data = np.load(path / f"seed_{seed}" / "trajectory.npz")
    vertices = data["vertices"]
    dt = float(data["dt"])
    vertex_mean = vertices.mean(axis=1)
    area_cent = np.array([polygon_centroid(v) for v in vertices])
    vmean_speed = np.linalg.norm(np.diff(vertex_mean, axis=0), axis=1) / dt
    area_speed = np.linalg.norm(np.diff(area_cent, axis=0), axis=1) / dt
    per_vertex = np.linalg.norm(np.diff(vertices, axis=0), axis=2) / dt
    max_vertex = per_vertex.max(axis=1)
    e0 = vertices[:, 1, :] - vertices[:, 0, :]
    yaw = np.arctan2(e0[:, 1], e0[:, 0])
    yaw_rate = np.diff(np.unwrap(yaw)) / dt
    return dict(
        max_vertex_mean_speed=float(vmean_speed.max()),
        max_area_centroid_speed=float(area_speed.max()),
        max_any_vertex_speed=float(max_vertex.max()),
        mean_area_centroid_speed=float(area_speed.mean()),
        n_area_over_0p25=int(np.sum(area_speed > 0.25)),
        n_vertexmean_over_0p25=int(np.sum(vmean_speed > 0.25)),
        max_yaw_rate=float(np.max(np.abs(yaw_rate))),
        argmax_area_frame=int(np.argmax(area_speed)),
        argmax_vertexmean_frame=int(np.argmax(vmean_speed)),
    )


def thresholds(diffs, dt):
    out = {}
    for thr in [0.0, 1e-15, 1e-12, 1e-9, 1e-6, 1e-4, 1e-3, 1e-2, 1e-1, 0.25, 0.5]:
        idx = np.nonzero(diffs > thr)[0]
        if len(idx):
            i = int(idx[0])
            out[str(thr)] = dict(frame=i, time=float(i * dt), norm=float(diffs[i]))
        else:
            out[str(thr)] = None
    return out


def refinement_stats(path, seed):
    rows = json.loads((path / f"seed_{seed}" / "observer_refinement.json").read_text(encoding="utf-8"))
    g_rel, c_rel, cmax = [], [], []
    for r in rows:
        g_rel.append(abs(r["gradient2_fine"] - r["gradient2_coarse"]) / max(abs(r["gradient2_fine"]), 1e-30))
        c_rel.append(abs(r["centroid2_fine"] - r["centroid2_coarse"]) / max(abs(r["centroid2_fine"]), 1e-30))
        cmax.append(r["centroid_max_difference"])
    return dict(
        n=len(rows),
        max_gradient2_rel=float(max(g_rel)),
        max_centroid2_rel=float(max(c_rel)),
        max_centroid_diff=float(max(cmax)),
        rows=rows,
    )


def g500(path, seed):
    sim = json.loads((path / f"seed_{seed}" / "simulation_summary.json").read_text(encoding="utf-8"))
    g = sim["cargoes"]["cargo_0"]["g500"]
    return dict(
        success=g["success"],
        failure_reasons=g["failure_reasons"],
        barrier_scalings=g["metrics"]["barrier_scalings"],
        min_barrier_scale=g["metrics"]["min_barrier_scale"],
        J=g["metrics"]["J"],
        direction_error_deg=g["metrics"]["direction_error_deg"],
        max_cross_track=g["metrics"]["max_cross_track"],
        hold_frame=g["metrics"]["hold_frame"],
        reached_frame=g["metrics"]["reached_frame"],
        final_phase=g["metrics"]["final_phase"],
        angle_deg=g["metrics"]["goal_angle_deg"],
    )


def residual_stats(seed):
    a = np.genfromtxt(REF / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
    b = np.genfromtxt(NEW / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
    out = {}
    for f in ["H", "gradient2", "centroid2", "gradient2_time_average", "centroid2_time_average"]:
        xa, xb = np.asarray(a[f], float), np.asarray(b[f], float)
        abs_e = np.abs(xa - xb)
        out[f] = dict(
            max_abs=float(abs_e.max()),
            end_abs=float(abs_e[-1]),
            mean_abs=float(abs_e.mean()),
            first_gt_1e12=int(np.argmax(abs_e > 1e-12)) if np.any(abs_e > 1e-12) else None,
            first_gt_1e6=int(np.argmax(abs_e > 1e-6)) if np.any(abs_e > 1e-6) else None,
            first_gt_1e3=int(np.argmax(abs_e > 1e-3)) if np.any(abs_e > 1e-3) else None,
        )
    return out, a, b


def main():
    report = {}
    for seed in [2, 5, 8]:
        ta = np.load(REF / f"seed_{seed}" / "trajectory.npz")
        tb = np.load(NEW / f"seed_{seed}" / "trajectory.npz")
        dt = float(ta["dt"])
        n = min(len(ta["positions"]), len(tb["positions"]))
        pos = np.linalg.norm((ta["positions"][:n] - tb["positions"][:n]).reshape(n, -1), axis=1)
        vel_n = min(len(ta["velocities"]), len(tb["velocities"]))
        vel = np.linalg.norm((ta["velocities"][:vel_n] - tb["velocities"][:vel_n]).reshape(vel_n, -1), axis=1)
        vert = np.linalg.norm((ta["vertices"][:n] - tb["vertices"][:n]).reshape(n, -1), axis=1)
        sa = json.loads((REF / f"seed_{seed}" / "safety_diagnostics.json").read_text(encoding="utf-8"))
        sb = json.loads((NEW / f"seed_{seed}" / "safety_diagnostics.json").read_text(encoding="utf-8"))
        corr = [abs(x["max_filter_correction"] - y["max_filter_correction"]) for x, y in zip(sa, sb)]
        clip_frames_ref = [i for i, x in enumerate(sa) if x["clipping_agents"]]
        clip_frames_new = [i for i, x in enumerate(sb) if x["clipping_agents"]]
        large_corr_new = [i for i, x in enumerate(sb) if x["max_filter_correction"] > 0.2]
        large_corr_ref = [i for i, x in enumerate(sa) if x["max_filter_correction"] > 0.2]
        rst, a, b = residual_stats(seed)
        report[str(seed)] = dict(
            position_thresholds=thresholds(pos, dt),
            velocity_thresholds=thresholds(vel, dt),
            vertex_thresholds=thresholds(vert, dt),
            position_last=float(pos[-1]),
            velocity_last=float(vel[-1]),
            vertex_last=float(vert[-1]),
            position_max=float(pos.max()),
            first_command_vel_gt_1e12=int(np.argmax(vel > 1e-12)) if np.any(vel > 1e-12) else None,
            first_command_vel_gt_1e9=int(np.argmax(vel > 1e-9)) if np.any(vel > 1e-9) else None,
            first_filter_corr_gt_1e12=int(np.argmax(np.array(corr) > 1e-12)) if np.any(np.array(corr) > 1e-12) else None,
            max_filter_corr_abs=float(max(corr)),
            clip_frames_ref=clip_frames_ref,
            clip_frames_new=clip_frames_new,
            large_filter_corr_frames_ref=large_corr_ref[:20],
            large_filter_corr_frames_new=large_corr_new[:20],
            object_speed_ref=object_speed_estimators(REF, seed),
            object_speed_new=object_speed_estimators(NEW, seed),
            residual_abs=rst,
            H0_ref=float(a["H"][0]),
            H0_new=float(b["H"][0]),
            g500_ref=g500(REF, seed),
            g500_new=g500(NEW, seed),
            refinement_ref=refinement_stats(REF, seed),
            refinement_new=refinement_stats(NEW, seed),
        )
    dump(OUT / "divergence_timeline.json", report)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    colors = {2: "#126E82", 5: "#B85C38", 8: "#6847A0"}
    fig, axes = plt.subplots(3, 2, figsize=(12.5, 10.5))
    for col, metric, ylabel in [
        (0, "gradient2", r"$\|g^\star\|^2$"),
        (1, "centroid2", r"$\sum_i\|p_i-c_i^\star\|^2$"),
    ]:
        for seed in [2, 5, 8]:
            a = np.genfromtxt(REF / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
            b = np.genfromtxt(NEW / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
            c = colors[seed]
            axes[0, col].plot(a["time"], a[metric], color=c, alpha=0.25, lw=1)
            axes[0, col].plot(b["time"], b[metric], color=c, alpha=0.80, lw=1, ls="--")
            axes[1, col].plot(a["time"], a[metric + "_time_average"], color=c, lw=2, label=f"seed {seed} 参考")
            axes[1, col].plot(b["time"], b[metric + "_time_average"], color=c, lw=1.6, ls="--",
                              label=f"seed {seed} 复现")
            axes[2, col].plot(a["time"], np.abs(a[metric] - b[metric]), color=c, lw=1.6, label=f"seed {seed}")
        axes[0, col].set_ylabel(ylabel)
        axes[1, col].set_ylabel(ylabel + " 时间平均")
        axes[2, col].set_ylabel("瞬时残差绝对差")
        axes[2, col].set_yscale("log")
        axes[2, col].set_xlabel("时间 (s)")
        axes[1, col].set_xlabel("时间 (s)")
        axes[0, col].grid(alpha=0.18)
        axes[1, col].grid(alpha=0.18)
        axes[2, col].grid(alpha=0.18, which="both")
    axes[0, 0].set_title("瞬时梯度平方残差：实线=参考，虚线=复现")
    axes[0, 1].set_title("瞬时质心平方残差：实线=参考，虚线=复现")
    axes[1, 0].legend(fontsize=7, ncol=2)
    axes[1, 1].legend(fontsize=7, ncol=2)
    axes[2, 0].legend(fontsize=8)
    axes[2, 1].legend(fontsize=8)
    fig.suptitle("DBACT 定理审计复现对照 | 未改算法与参数 | seeds 2/5/8 | N=16", fontsize=13)
    fig.text(0.05, 0.012,
             "参考来自 DBACT_Theorem_Audit_2026-09-11.zip。两条曲线在 seed 2 几乎重合；seed 5/8 由机器浮点差放大，未调参。",
             fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    fig.savefig(OUT / "comparison_residuals.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    for seed in [2, 5, 8]:
        ta = np.load(REF / f"seed_{seed}" / "trajectory.npz")
        tb = np.load(NEW / f"seed_{seed}" / "trajectory.npz")
        n = min(len(ta["positions"]), len(tb["positions"]))
        pos = np.linalg.norm((ta["positions"][:n] - tb["positions"][:n]).reshape(n, -1), axis=1)
        t = np.arange(n) * float(ta["dt"])
        ax.plot(t, pos, lw=2, label=f"seed {seed}")
    ax.set_yscale("log")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel(r"$\|P_{\mathrm{ref}}-P_{\mathrm{rep}}\|_F$")
    ax.set_title("机器人位置相对参考轨迹的 Frobenius 偏差")
    ax.grid(alpha=0.18, which="both")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "position_divergence.png", dpi=180)
    plt.close(fig)
    print("wrote divergence_timeline.json and figures")
    for seed, d in report.items():
        p = d["position_thresholds"]
        print(seed,
              "1e-12", p["1e-12"],
              "1e-6", p["1e-06"],
              "1e-3", p["0.001"],
              "g500", d["g500_ref"]["success"], "->", d["g500_new"]["success"],
              "area_speed", d["object_speed_new"]["max_area_centroid_speed"])


if __name__ == "__main__":
    main()
