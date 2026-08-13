#!/usr/bin/env python3
"""Generate publication figures and derived tables from reproducible run data."""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import Counter
from datetime import date
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


COLORS = {
    "blue": "#2563EB",
    "green": "#059669",
    "orange": "#EA580C",
    "red": "#DC2626",
    "purple": "#7C3AED",
    "gray": "#64748B",
    "light": "#CBD5E1",
}


def wilson(successes: int, trials: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if trials <= 0:
        return math.nan, math.nan
    p = successes / trials
    den = 1.0 + z * z / trials
    center = (p + z * z / (2.0 * trials)) / den
    half = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / den
    return max(0.0, center - half), min(1.0, center + half)


def save_figure(fig: plt.Figure, output: Path, name: str) -> list[str]:
    paths = []
    for suffix in ("png", "pdf"):
        path = output / f"{name}.{suffix}"
        metadata = {"CreationDate": None, "ModDate": None} if suffix == "pdf" else None
        fig.savefig(
            path,
            dpi=300 if suffix == "png" else None,
            bbox_inches="tight",
            metadata=metadata,
        )
        paths.append(path.name)
    plt.close(fig)
    return paths


def phase_lines(ax: plt.Axes, phases: dict[str, int | None]) -> None:
    mapping = [
        ("first_detection", "detect", COLORS["blue"]),
        ("first_map_complete", "map", COLORS["purple"]),
        ("first_enclosure", "enclose", COLORS["green"]),
        ("first_transport", "transport", COLORS["orange"]),
        ("first_brake", "brake", COLORS["red"]),
        ("first_hold", "hold", COLORS["gray"]),
    ]
    for key, label, color in mapping:
        frame = phases.get(key)
        if frame is not None:
            ax.axvline(frame, color=color, linewidth=0.9, linestyle="--", alpha=0.65)
            ax.text(frame, 0.98, label, color=color, fontsize=7, rotation=90,
                    ha="right", va="top", transform=ax.get_xaxis_transform())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--monte-carlo",
        default="runs/arbitrary_shape_final_se2_60/monte_carlo.json",
    )
    parser.add_argument("--distance", default="docs/results/distance_ablation_stage3.json")
    parser.add_argument("--robustness", default="docs/results/robustness_ablation_stage2.json")
    parser.add_argument("--performance", default="docs/results/PERFORMANCE_STAGE6.json")
    parser.add_argument("--representative", default="artifacts/publication/representative")
    parser.add_argument("--output", default="artifacts/publication")
    args = parser.parse_args()

    output = Path(args.output)
    figures = output / "figures"
    tables = output / "tables"
    data_dir = output / "data"
    for directory in (output, figures, tables, data_dir):
        directory.mkdir(parents=True, exist_ok=True)

    mc_path = Path(args.monte_carlo)
    mc = json.loads(mc_path.read_text(encoding="utf-8"))
    distance = json.loads(Path(args.distance).read_text(encoding="utf-8"))
    robustness = json.loads(Path(args.robustness).read_text(encoding="utf-8"))
    performance = json.loads(Path(args.performance).read_text(encoding="utf-8"))
    representative = Path(args.representative)
    rep_manifest = json.loads((representative / "manifest.json").read_text(encoding="utf-8"))
    cargo_ts = pd.read_csv(representative / "cargo_timeseries.csv")
    errors = pd.read_csv(representative / "perception_errors.csv")
    records = list(mc["records"])
    df = pd.DataFrame(records)
    artifact_files: list[str] = []

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
        "axes.grid": True,
        "grid.alpha": 0.22,
        "figure.facecolor": "white",
    })

    # 1. Success/eligibility by shape.
    shape_order = list(mc["manifest"]["shapes"])
    shape_rows = []
    for shape in shape_order:
        rows = df[df["shape"] == shape]
        eligible = rows[rows["domain_eligible"]]
        shape_rows.append({
            "shape": shape,
            "n": len(rows),
            "eligible": len(eligible),
            "task_success": int(rows["task_success"].sum()),
            "eligible_success": int(eligible["task_success"].sum()),
            "P_eligible": len(eligible) / len(rows),
            "P_task_success": float(rows["task_success"].mean()),
            "P_success_given_eligible": (
                float(eligible["task_success"].mean()) if len(eligible) else math.nan
            ),
        })
    shape_stats = pd.DataFrame(shape_rows)
    shape_stats.to_csv(tables / "shape_statistics.csv", index=False)
    x = np.arange(len(shape_stats))
    fig, ax = plt.subplots(figsize=(11.5, 4.6))
    w = 0.25
    ax.bar(x - w, shape_stats["P_eligible"], w, label="P(eligible)", color=COLORS["blue"])
    ax.bar(x, shape_stats["P_task_success"], w, label="P(task success)", color=COLORS["orange"])
    ax.bar(x + w, shape_stats["P_success_given_eligible"], w,
           label="P(success | eligible)", color=COLORS["green"])
    ax.set(ylim=(0, 1.05), ylabel="empirical probability", title="Outcome rates by shape family (5 seeds each)")
    ax.set_xticks(x, shape_stats["shape"], rotation=38, ha="right")
    ax.legend(ncol=3, loc="upper center")
    artifact_files += save_figure(fig, figures, "01_success_rate_vs_shape")

    # 2. Distance ablation. One seed: plot evidence flags, not a population rate.
    distance_rows = distance["results"]
    dx = np.arange(len(distance_rows))
    task_ok = np.array([row["termination"] == "SUCCESS_HOLD" for row in distance_rows], dtype=float)
    bounds_ok = np.array([row["audit"]["declared_bounds_satisfied"] for row in distance_rows], dtype=float)
    solver_ok = np.array([
        row["fallbacks"] == row["infeasible"] == row["rho_relaxations"] == 0
        for row in distance_rows
    ], dtype=float)
    fig, ax = plt.subplots(figsize=(6.8, 4.3))
    w = 0.24
    ax.bar(dx - w, task_ok, w, label="closed-loop HOLD", color=COLORS["green"])
    ax.bar(dx, solver_ok, w, label="solver full margin", color=COLORS["blue"])
    ax.bar(dx + w, bounds_ok, w, label="all measured bounds", color=COLORS["purple"])
    ax.set_xticks(dx, [f"{row['distance_m']:.2f} m" for row in distance_rows])
    ax.set(ylim=(0, 1.08), ylabel="evidence indicator (single seed)", title="Transport evidence vs distance")
    ax.legend(loc="lower left")
    artifact_files += save_figure(fig, figures, "02_success_evidence_vs_distance")

    # 3. Completion/censoring distribution.
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    classes = [
        (df["domain_eligible"] & df["task_success"], "eligible success", COLORS["green"]),
        (df["domain_eligible"] & ~df["task_success"], "eligible failure", COLORS["red"]),
        (~df["domain_eligible"], "rejected", COLORS["gray"]),
    ]
    bins = np.linspace(float(df["frame"].min()), float(df["frame"].max()), 12)
    for mask, label, color in classes:
        ax.hist(df.loc[mask, "frame"], bins=bins, alpha=0.55, label=label, color=color)
    ax.set(xlabel="termination frame", ylabel="episodes", title="Completion / failure-time distribution")
    ax.legend()
    artifact_files += save_figure(fig, figures, "03_completion_time_distribution")

    # 4. Phase time distributions with invalid ordering left blank, not clipped.
    phase_rows = []
    phase_specs = [
        ("search", None, "first_detection"),
        ("map", "first_detection", "first_map_complete"),
        ("enclose", "first_map_complete", "first_enclosure"),
        ("contact", "first_enclosure", "first_contact"),
        ("transport", "first_transport", "first_brake"),
        ("brake", "first_brake", "first_hold"),
    ]
    for record in records:
        phases = record["phase_frames"]
        row = {"case_id": record["case_id"]}
        for label, start_key, end_key in phase_specs:
            start = 0 if start_key is None else phases.get(start_key)
            end = phases.get(end_key)
            duration = None if start is None or end is None or end < start else end - start
            row[f"{label}_frames"] = duration
        phase_rows.append(row)
    phase_df = pd.DataFrame(phase_rows)
    phase_df.to_csv(tables / "phase_times.csv", index=False)
    values = [phase_df[f"{label}_frames"].dropna().to_numpy() for label, _, _ in phase_specs]
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    plot = ax.boxplot(values, tick_labels=[p[0] for p in phase_specs], patch_artist=True, showfliers=True)
    for patch, color in zip(plot["boxes"], [COLORS["blue"], COLORS["purple"], COLORS["green"],
                                               COLORS["orange"], COLORS["red"], COLORS["gray"]]):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)
    ax.set(ylabel="ordered phase duration [frames]", title="Phase-time distributions (invalid/missing order excluded)")
    artifact_files += save_figure(fig, figures, "04_phase_time_distributions")

    # Representative time-series figures 5-10.
    phases = rep_manifest["phase_frames"]
    frames = cargo_ts["iteration"].to_numpy()
    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    ax.plot(frames, cargo_ts["J"], color=COLORS["blue"], linewidth=1.8)
    ax.axhline(0.10, color="black", linestyle=":", linewidth=1.0, label="task distance")
    phase_lines(ax, phases)
    ax.set(xlabel="frame", ylabel="activation-relative J [m]", title="Directional progress J(t)")
    ax.legend(loc="lower right")
    artifact_files += save_figure(fig, figures, "05_directional_progress")

    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    ax.plot(frames, cargo_ts["cross_track"], color=COLORS["orange"], linewidth=1.6)
    ax.axhline(0.0, color="black", linewidth=0.8)
    phase_lines(ax, phases)
    ax.set(xlabel="frame", ylabel="signed cross-track error [m]", title="Cargo cross-track error")
    artifact_files += save_figure(fig, figures, "06_cross_track_error")

    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    yaw = cargo_ts["yaw_deg"] - float(cargo_ts["yaw_deg"].iloc[0])
    ax.plot(frames, yaw, color=COLORS["purple"], linewidth=1.6)
    ax.axhline(30.0, color=COLORS["red"], linestyle=":", linewidth=1.0)
    ax.axhline(-30.0, color=COLORS["red"], linestyle=":", linewidth=1.0, label="rotation limit")
    phase_lines(ax, phases)
    ax.set(xlabel="frame", ylabel="rotation from initial [deg]", title="Cargo rotation")
    ax.legend(loc="lower right")
    artifact_files += save_figure(fig, figures, "07_cargo_rotation")

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True)
    axes[0].plot(frames, cargo_ts["net_force_parallel"], label="goal-parallel", color=COLORS["blue"])
    axes[0].plot(frames, cargo_ts["net_force_cross"], label="cross-track", color=COLORS["orange"], alpha=0.8)
    axes[0].set(ylabel="net force [N]", title="Contact wrench")
    axes[0].legend()
    axes[1].plot(frames, cargo_ts["net_torque"], color=COLORS["purple"])
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set(xlabel="frame", ylabel="net torque [N m]")
    for ax in axes:
        phase_lines(ax, phases)
    artifact_files += save_figure(fig, figures, "08_net_force_and_torque")

    fig, ax = plt.subplots(figsize=(8.0, 4.0))
    ax.step(frames, cargo_ts["contacts"], where="post", color=COLORS["green"], linewidth=1.5)
    phase_lines(ax, phases)
    ax.set(xlabel="frame", ylabel="contact-agent count", title="Engaged contact agents")
    artifact_files += save_figure(fig, figures, "09_contact_agent_count")

    fig, axes = plt.subplots(2, 1, figsize=(8.0, 5.8), sharex=True)
    axes[0].plot(frames, cargo_ts["min_inter_agent_distance"], color=COLORS["blue"])
    axes[0].axhline(0.32, color=COLORS["red"], linestyle=":", label="d_min")
    axes[0].set(ylabel="minimum robot distance [m]", title="Safety distance and object penetration")
    axes[0].legend()
    axes[1].plot(frames, cargo_ts["max_penetration"], color=COLORS["red"])
    axes[1].set(xlabel="frame", ylabel="maximum penetration [m]")
    for ax in axes:
        phase_lines(ax, phases)
    artifact_files += save_figure(fig, figures, "10_safety_distance_and_penetration")

    # 11. Error distributions normalized by their declared theory bounds.
    bound_map = {
        "boundary_point_error_m": 0.023,
        "map_point_error_m": 0.27,
        "object_velocity_error_mps": 0.35,
        "boundary_velocity_error_mps": 0.35,
        "cbf_velocity_projection_error_mps": 0.35,
    }
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    error_summary = []
    for error_type, bound in bound_map.items():
        values = errors.loc[errors["error_type"] == error_type, "value"].to_numpy(dtype=float)
        if not len(values):
            continue
        if len(values) > 100_000:
            values = values[np.linspace(0, len(values) - 1, 100_000).astype(int)]
        ratio = np.sort(values / bound)
        cdf = np.arange(1, len(ratio) + 1) / len(ratio)
        ax.plot(ratio, cdf, linewidth=1.4, label=error_type.replace("_error", ""))
        error_summary.append({
            "error_type": error_type,
            "declared_bound": bound,
            "n": len(values),
            "p95": float(np.quantile(values, 0.95)),
            "max": float(np.max(values)),
            "bound_satisfied": bool(np.max(values) <= bound),
        })
    pd.DataFrame(error_summary).to_csv(tables / "perception_error_statistics.csv", index=False)
    ax.axvline(1.0, color="black", linestyle="--", linewidth=1.0, label="declared bound")
    ax.set(xlabel="measured error / declared bound", ylabel="empirical CDF", title="Perception and velocity error distributions")
    ax.set_xlim(left=0.0)
    ax.legend(fontsize=7)
    artifact_files += save_figure(fig, figures, "11_perception_error_distributions")

    # 12. Runtime profile ablation.
    baseline = performance["baseline"]["cases"]
    optimized = performance["optimized"]["cases"]
    labels = [row["shape"] for row in optimized]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(7.0, 4.3))
    ax.bar(x - 0.18, [r["fps"] for r in baseline], 0.36, label="baseline", color=COLORS["light"])
    ax.bar(x + 0.18, [r["fps"] for r in optimized], 0.36, label="optimized", color=COLORS["blue"])
    ax.axhline(20.0, color=COLORS["red"], linestyle="--", linewidth=1.0, label="20 fps target")
    ax.set_xticks(x, labels)
    ax.set(ylabel="control frames / wall second", title="Headless runtime ablation")
    ax.legend()
    artifact_files += save_figure(fig, figures, "12_runtime_profiling")

    # 13. Failure composition includes every episode.
    composition = Counter(df["failure_class"])
    labels = list(composition)
    counts = [composition[label] for label in labels]
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    order = np.argsort(counts)
    ax.barh(np.asarray(labels)[order], np.asarray(counts)[order], color=COLORS["red"])
    ax.set(xlabel="episodes", title="Failure / outcome composition (no survivor filtering)")
    artifact_files += save_figure(fig, figures, "13_failure_composition")

    # 14. Conditional-domain aggregate with Wilson intervals.
    stats = mc["statistics"]
    measures = [
        ("P(eligible)", stats["eligible"], stats["episodes"]),
        ("P(success | eligible)", stats["eligible_successes"], stats["eligible"]),
        ("P(rejected)", stats["rejected"], stats["episodes"]),
    ]
    values = np.array([s / n for _, s, n in measures])
    intervals = [wilson(s, n) for _, s, n in measures]
    yerr = np.array([[v - lo for v, (lo, hi) in zip(values, intervals)],
                     [hi - v for v, (lo, hi) in zip(values, intervals)]])
    fig, ax = plt.subplots(figsize=(7.0, 4.4))
    ax.bar(np.arange(3), values, color=[COLORS["blue"], COLORS["green"], COLORS["gray"]])
    ax.errorbar(np.arange(3), values, yerr=yerr, fmt="none", ecolor="black", capsize=4)
    ax.set_xticks(np.arange(3), [m[0] for m in measures])
    ax.set(ylim=(0, 1.05), ylabel="probability with Wilson 95% CI",
           title="Conditional theorem-domain statistics")
    artifact_files += save_figure(fig, figures, "14_conditional_domain_statistics")

    # Raw/derived publication data bundle.
    source_copies = {
        mc_path: data_dir / "arbitrary_shape_monte_carlo.json",
        mc_path.with_name("episodes.csv"): data_dir / "arbitrary_shape_episodes.csv",
        mc_path.with_name("manifest.json"): data_dir / "arbitrary_shape_manifest.json",
        Path(args.distance): data_dir / "distance_ablation.json",
        Path(args.robustness): data_dir / "robustness_ablation.json",
        Path(args.performance): data_dir / "performance_ablation.json",
    }
    for source, destination in source_copies.items():
        shutil.copyfile(source, destination)

    manifest = {
        "schema_version": 1,
        "generated_on": str(date.today()),
        "classification": "publication_artifact_manifest",
        "monte_carlo_episodes": int(stats["episodes"]),
        "figures": artifact_files,
        "tables": sorted(path.name for path in tables.iterdir()),
        "data": sorted(path.name for path in data_dir.iterdir()),
        "representative": rep_manifest,
        "finite_time": {
            "analytic_bound_available": False,
            "empirical_bound_available": False,
            "reason": stats["completion_time_bound"].get("reason"),
        },
        "claim_boundaries": [
            "operational boundary enclosure is not formal configuration-space caging",
            "shape theorem is conditional on runtime domain eligibility",
            "eligible failures prevent a finite empirical completion bound",
            "wall-clock performance is empirical and machine-dependent",
        ],
    }
    (output / "publication_manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8",
    )
    print(json.dumps({"output": str(output), "figures": len(artifact_files), "episodes": stats["episodes"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
