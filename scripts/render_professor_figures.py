#!/usr/bin/env python3
"""Professor figures from replay data. Development evidence only."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Polygon as MplPolygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
OUT = ROOT / "docs/results/evidence_closure/professor_figures"


def cargo_series(folder: Path):
    payload = json.loads((folder / "paired_metrics.json").read_text(encoding="utf-8"))
    info = payload["summary"]["task"]["cargo_0"]
    goal = np.asarray(payload["summary"]["goal_direction"], dtype=float)
    start = np.asarray(info["start"], dtype=float)
    distance = float(info["target_distance"])
    rows = []
    with (folder / "trajectories.csv").open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["kind"] == "cargo":
                rows.append((float(row["time"]), float(row["x"]), float(row["y"])))
    points = np.asarray([[row[1], row[2]] for row in rows], dtype=float)
    times = np.asarray([row[0] for row in rows])
    along = (points - start) @ goal
    lateral = points - start - along[:, None] * goal
    sign = np.sign(lateral[:, 0] * (-goal[1]) + lateral[:, 1] * goal[0])
    cross = np.linalg.norm(lateral, axis=1) * sign
    endpoint = np.linalg.norm(points - (start + distance * goal), axis=1)
    return {
        "times": times, "along": along, "cross": cross, "endpoint": endpoint,
        "distance": distance,
        "force": np.asarray([step["forward_force"] for step in payload["steps"]]),
        "step_time": np.asarray([step["frame"] * 0.05 for step in payload["steps"]]),
        "phases": payload["steps"],
    }


def mark_brake(ax, series, color):
    seen = False
    for step in series["phases"]:
        if step["phase"] == "BRAKE" and not seen:
            ax.axvline(step["frame"] * 0.05, color=color, linestyle="--", linewidth=0.8)
            seen = True


def figure_problem(path: Path) -> None:
    from dbact_sim.environment import SimulationEnvironment
    from run_deployment_pairs import config_for
    env = SimulationEnvironment(config_for(6, "c_shape"), seed=8)
    cargo = env.cargoes[0]
    agent = min(env.agents, key=lambda item: np.linalg.norm(item.position - cargo.position))
    view = env.controller.sensor.sense_view(agent, env.cargoes, 0.0)
    goal = env.goal_directions[cargo.object_id]
    distance = env.tasks[cargo.object_id].distance
    sensor = float(env.controller.params.sensor_range)
    fig, ax = plt.subplots(figsize=(6.6, 5.6))
    team = np.vstack([item.position for item in env.agents])
    ax.scatter(team[:, 0], team[:, 1], s=18, color="#c47b2b", label=f"team n={len(env.agents)}", zorder=3)
    circle = plt.Circle(agent.position, sensor, fill=False, linestyle=":", color="#24527a", label=f"one sensor range {sensor:.2f} m")
    ax.add_patch(circle)
    ax.add_patch(MplPolygon(cargo.vertices, closed=True, fill=False, linestyle="--", linewidth=1.5, color="#888888", label="true outline (evaluation only)"))
    if len(view):
        ax.scatter(view.points[:, 0], view.points[:, 1], s=12, color="#24527a", label="online returns of the marked robot", zorder=4)
    ax.scatter([agent.position[0]], [agent.position[1]], color="#b33a3a", s=36, zorder=5, label="robot whose returns are shown")
    ax.arrow(cargo.position[0], cargo.position[1], 0.8 * goal[0], 0.8 * goal[1], width=0.012, color="#1b7a3a", length_includes_head=True)
    ax.text(cargo.position[0] + 0.85 * goal[0], cargo.position[1] + 0.85 * goal[1], f"task direction\n{distance:.2f} m", color="#1b7a3a", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Figure 1. C-shape seed 8 at t=0. Development.\nDashed outline is not a controller input.")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_method(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.6))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6)
    ax.axis("off")
    def box(x, y, w, h, text, face):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.03", facecolor=face, edgecolor="#24527a"))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8)
    box(0.3, 3.6, 3.4, 1.6, "deployment, before CONTACT_READY\nCVT centroid, or nearest target,\nor local spacing", "#f7f1e3")
    box(4.2, 3.6, 3.2, 1.6, "local ranges, arc weights,\noffset targets", "#e8f0f7")
    box(8.0, 3.6, 3.5, 1.6, "shared layer\nsafety QP, brake, pause,\nintegral release", "#e7f4ea")
    ax.annotate("", xy=(4.15, 4.4), xytext=(3.75, 4.4), arrowprops={"arrowstyle": "->"})
    ax.annotate("", xy=(7.95, 4.4), xytext=(7.45, 4.4), arrowprops={"arrowstyle": "->"})
    ax.annotate("", xy=(6.0, 3.55), xytext=(9.5, 3.55), arrowprops={"arrowstyle": "->", "connectionstyle": "arc3,rad=0.3"})
    ax.text(7.6, 2.55, "estimated progress and speed\nfeed the brake; pause is not success", ha="center", fontsize=8)
    ax.text(6, 0.6, "Only the centroid target changes between deployment methods.\nBrake, safety, sensing and contact physics stay common.", ha="center", fontsize=8)
    ax.set_title("Figure 2. Executed split between deployment and the shared controller.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_brake(path: Path) -> None:
    old = cargo_series(ROOT / "runs/evidence_closure/study_v3/development/dev_c_shape_8_boundary_cvt")
    new = cargo_series(ROOT / "runs/evidence_closure/phase_brake_v7/c_shape_seed8_boundary_cvt")
    fig, axes = plt.subplots(2, 1, figsize=(7.6, 6.4), sharex=False)
    axes[0].plot(old["times"], old["along"], color="#888888", label=f"before, endpoint {old['endpoint'][-1]:.2f} m")
    axes[0].plot(new["times"], new["along"], color="#24527a", label=f"after, endpoint {new['endpoint'][-1]:.3f} m")
    axes[0].axhline(new["distance"], color="#1b7a3a", linewidth=0.8, label="target distance")
    seen = set()
    for step in new["phases"]:
        if step["phase"] not in seen and step["phase"] in {"CONTACT_READY", "BRAKE", "HOLD"}:
            seen.add(step["phase"])
            for ax in axes:
                ax.axvline(step["frame"] * 0.05, color="#24527a", linestyle="--", linewidth=0.7)
            axes[0].text(step["frame"] * 0.05, new["distance"] * 0.15, step["phase"], rotation=90, fontsize=7, color="#24527a")
    zoom = axes[0].inset_axes([0.55, 0.15, 0.42, 0.45])
    window = (new["times"] > 12) & (new["times"] < 22)
    zoom.plot(new["times"][window], new["along"][window], color="#24527a")
    zoom.axhline(new["distance"], color="#1b7a3a", linewidth=0.6)
    zoom.set_title("stop window", fontsize=7)
    zoom.tick_params(labelsize=6)
    axes[0].set_ylabel("along-track progress (m)")
    axes[0].legend(fontsize=7)
    axes[1].plot(old["step_time"], old["force"], color="#888888", label="before")
    axes[1].plot(new["step_time"], new["force"], color="#b33a3a", label="after")
    axes[1].set_xlim(0, 40)
    axes[0].set_xlim(0, 40)
    axes[1].set_ylabel("forward force (N)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(fontsize=7)
    axes[0].set_title("Figure 3. C seed 8 CVT. Downstream brake only. n=1. Development.\nInset is the stop. Not a CVT-versus-baseline result.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_methods(path: Path, run_dir: Path) -> None:
    rows = []
    for result_path in sorted(run_dir.glob("*/case_result.json")):
        rows.append(json.loads(result_path.read_text(encoding="utf-8"))["result"])
    if not rows:
        return
    variants = ["boundary_cvt", "nearest_observed_target", "local_spacing"]
    colors = {"l_shape": "#24527a", "rectangle": "#1b7a3a", "c_shape": "#c47b2b"}
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 4.2))
    notes = []
    for index, variant in enumerate(variants):
        chosen = [row for row in rows if row.get("variant") == variant]
        done = sum(bool(row.get("task_completed")) for row in chosen)
        notes.append(f"{variant.split('_')[0]} {done}/{len(chosen)}")
        for row in chosen:
            jitter = 0.12 if row.get("shape") == "rectangle" else (-0.12 if row.get("shape") == "c_shape" else 0.0)
            color = "#b33a3a" if row.get("watchdog") or not row.get("task_completed") else colors.get(row.get("shape"), "#24527a")
            if row.get("contact_ready_seconds") is not None:
                axes[0].scatter([index + jitter], [row["contact_ready_seconds"]], s=28, color=color)
            if row.get("euclidean_endpoint_error") is not None:
                axes[1].scatter([index + jitter], [row["euclidean_endpoint_error"]], s=28, color=color)
    axes[1].axhline(0.12, color="#888888", linewidth=0.8, label="0.12 m task tolerance")
    for ax in axes:
        ax.set_xticks(range(3), ["CVT\nn=9", "nearest\nn=9", "spacing\nn=9"])
    axes[0].set_ylabel("CONTACT_READY (s)")
    axes[1].set_ylabel("endpoint error (m)")
    fig.suptitle(
        "Figure 4. Development L/rectangle/C, seeds 2/5/8. Same downstream controller.\n"
        "Task completion " + ", ".join(notes) + ". Red = not completed. Not an independent test."
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_safety(path: Path) -> None:
    root = ROOT / "runs/evidence_closure/static_segment_v2"
    labels, clearance, displacement, status = [], [], [], []
    for folder in sorted(p for p in root.glob("*") if p.is_dir()):
        summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
        labels.append(folder.name.replace("l_shape_seed2_", ""))
        clearance.append(1000 * float(summary.get("clearance_path_min") or 0))
        displacement.append(float(summary.get("mean_robot_displacement") or 0))
        status.append(summary.get("safety_evaluation_status"))
    fig, axes = plt.subplots(1, 2, figsize=(7.4, 4.0))
    if labels:
        bars = axes[0].bar(labels, clearance, color="#24527a")
        axes[0].axhline(65, color="#b33a3a", linewidth=0.8, label="65 mm static threshold")
        axes[1].bar(labels, displacement, color="#c47b2b")
        for bar, value, flag in zip(bars, clearance, status):
            axes[0].text(bar.get_x() + bar.get_width() / 2, value + 0.4, f"{value:.2f} mm\n{flag}", ha="center", va="bottom", fontsize=7)
        for bar, value in zip(axes[1].patches, displacement):
            axes[1].text(bar.get_x() + bar.get_width() / 2, value + 0.02, f"{value:.3f} m", ha="center", fontsize=7)
        axes[0].set_ylim(0, max(clearance) + 12)
        axes[1].set_ylim(0, max(displacement) + 0.2)
        axes[0].legend(fontsize=7)
    axes[0].set_ylabel("minimum path clearance (mm)")
    axes[1].set_ylabel("mean robot displacement (m)")
    fig.suptitle("Figure 5. One static development case, L seed 2, n=1 each.\nBelow 65 mm is an observed violation. Not a transport safety claim.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    figure_problem(OUT / "figure1_problem.png")
    figure_method(OUT / "figure2_method.png")
    figure_brake(OUT / "figure3_brake.png")
    dev = ROOT / "runs/evidence_closure/dev_common_brake_v1/development"
    if any(dev.glob("*/case_result.json")):
        figure_methods(OUT / "figure4_methods.png", dev)
    if (ROOT / "runs/evidence_closure/static_segment_v2").exists():
        figure_safety(OUT / "figure5_safety.png")
    print(OUT)


if __name__ == "__main__":
    main()
