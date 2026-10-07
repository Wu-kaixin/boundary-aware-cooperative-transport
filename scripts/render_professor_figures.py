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
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    ax.add_patch(MplPolygon(cargo.vertices, closed=True, fill=False, linestyle="--", linewidth=1.5, color="#888888", label="true outline (evaluation only)"))
    if len(view):
        ax.scatter(view.points[:, 0], view.points[:, 1], s=12, color="#24527a", label="online range returns", zorder=3)
    ax.scatter([agent.position[0]], [agent.position[1]], color="#b33a3a", s=30, label="one robot", zorder=4)
    goal = env.goal_directions[cargo.object_id]
    ax.arrow(cargo.position[0], cargo.position[1], 0.7 * goal[0], 0.7 * goal[1], width=0.01, color="#1b7a3a", length_includes_head=True)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Figure 1. C-shape seed 8 at t=0. Development.\nDashed outline is evaluation geometry, not a controller input.")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_method(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.6, 3.4))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 3)
    ax.axis("off")
    boxes = [
        (0.2, 1.15, "local ranges\nand arc weights"),
        (2.2, 1.15, "offset targets\nbefore contact"),
        (4.2, 1.15, "limited-range\nCVT centroid"),
        (6.2, 1.15, "shared safety QP"),
        (8.15, 1.15, "estimated-progress\nbrake"),
    ]
    for x, y, text in boxes:
        ax.add_patch(FancyBboxPatch((x, y), 1.7, 1.15, boxstyle="round,pad=0.03", facecolor="#e8f0f7", edgecolor="#24527a"))
        ax.text(x + 0.85, y + 0.55, text, ha="center", va="center", fontsize=8)
    ax.text(5, 0.35, "CONTACT_READY may brake without four pushers. Pause and watchdog are not task success.", ha="center", fontsize=8)
    ax.set_title("Figure 2. Modules that actually run. Development controller.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_brake(path: Path) -> None:
    old = cargo_series(ROOT / "runs/evidence_closure/study_v3/development/dev_c_shape_8_boundary_cvt")
    new = cargo_series(ROOT / "runs/evidence_closure/phase_brake_v7/c_shape_seed8_boundary_cvt")
    fig, axes = plt.subplots(2, 1, figsize=(7.4, 5.6), sharex=True)
    axes[0].plot(old["times"], old["along"], color="#888888", label=f"before, endpoint {old['endpoint'][-1]:.2f} m")
    axes[0].plot(new["times"], new["along"], color="#24527a", label=f"after, endpoint {new['endpoint'][-1]:.3f} m")
    axes[0].axhline(old["distance"], color="#1b7a3a", linewidth=0.8, label="target distance")
    mark_brake(axes[0], new, "#24527a")
    axes[0].set_ylabel("along-track progress (m)")
    axes[0].legend(fontsize=7)
    axes[1].plot(old["step_time"], old["force"], color="#888888", label="before")
    axes[1].plot(new["step_time"], new["force"], color="#b33a3a", label="after")
    mark_brake(axes[1], new, "#24527a")
    axes[1].set_ylabel("forward force (N)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(fontsize=7)
    axes[0].set_title("Figure 3. C seed 8 CVT, downstream brake only. n=1. Development.\nNot evidence that CVT beats another deployment.")
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
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 4.0))
    for index, variant in enumerate(variants):
        chosen = [row for row in rows if row.get("variant") == variant]
        ready = [row.get("contact_ready_seconds") for row in chosen if row.get("contact_ready_seconds") is not None]
        endpoint = [(row.get("euclidean_endpoint_error"), bool(row.get("watchdog"))) for row in chosen if row.get("euclidean_endpoint_error") is not None]
        axes[0].scatter(np.full(len(ready), index), ready, s=28, color="#24527a")
        if endpoint:
            axes[1].scatter(np.full(len(endpoint), index), [item[0] for item in endpoint], s=28, c=["#b33a3a" if item[1] else "#24527a" for item in endpoint])
    axes[0].set_xticks(range(3), ["CVT", "nearest", "spacing"])
    axes[1].set_xticks(range(3), ["CVT", "nearest", "spacing"])
    axes[0].set_ylabel("CONTACT_READY (s)")
    axes[1].set_ylabel("endpoint error (m)")
    axes[1].axhline(0.12, color="#888888", linewidth=0.8)
    fig.suptitle(f"Figure 4. Shared downstream controller. Development n={len(rows)}. Red = watchdog.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def figure_safety(path: Path) -> None:
    root = ROOT / "runs/evidence_closure/static_segment_v1"
    labels, clearance, displacement = [], [], []
    for folder in sorted(p for p in root.glob("*") if p.is_dir()):
        summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
        labels.append(folder.name.replace("l_shape_seed2_", ""))
        clearance.append(1000 * float(summary.get("clearance_path_min") or 0))
        displacement.append(float(summary.get("mean_robot_displacement") or 0))
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.8))
    if labels:
        axes[0].bar(labels, clearance, color="#24527a")
        axes[0].axhline(65, color="#b33a3a", linewidth=0.8)
        axes[1].bar(labels, displacement, color="#c47b2b")
    axes[0].set_ylabel("path clearance (mm)")
    axes[1].set_ylabel("mean displacement (m)")
    fig.suptitle("Figure 5. L seed 2 static local, n=1 each. Development.\nClearance and motion are separate. 65 mm is not a transport rule.")
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
    if (ROOT / "runs/evidence_closure/static_segment_v1").exists():
        figure_safety(OUT / "figure5_safety.png")
    print(OUT)


if __name__ == "__main__":
    main()
