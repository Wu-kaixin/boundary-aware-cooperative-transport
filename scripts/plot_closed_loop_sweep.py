#!/usr/bin/env python3
"""Plot phase deadlines, outcome gates, and throughput from a batch report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="batch_report.json from scripts/run_batch.py")
    parser.add_argument("--output", default="closed_loop_sweep.png")
    args = parser.parse_args()

    report = json.loads(Path(args.report).read_text(encoding="utf-8"))
    runs = sorted(report["runs"], key=lambda row: int(row["seed"]))
    if not runs:
        raise SystemExit("batch report contains no runs")

    seeds = np.asarray([int(row["seed"]) for row in runs])
    entries = [next(iter(row["cargoes"].values())) for row in runs]
    phases = {
        name: np.asarray([entry["phase_frames"][name] for entry in entries], dtype=float)
        for name in ("first_detection", "first_enclosure", "first_transport", "first_hold")
    }
    progress = np.asarray([entry["J"] for entry in entries], dtype=float)
    coverage = np.asarray([entry["final_strict_coverage"] for entry in entries], dtype=float)
    fps = report["steps"] / np.asarray([row["wall_seconds"] for row in runs], dtype=float)
    valid = np.asarray([bool(row["valid"]) for row in runs])

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    colors = ["#2563eb", "#10b981", "#dc2626", "#374151"]
    labels = ["detect", "enclose", "transport", "hold"]
    for (name, values), color, label in zip(phases.items(), colors, labels):
        axes[0].plot(seeds, values, marker="o", linewidth=1.5, color=color, label=label)
    for deadline, color in zip((150, 300, 350, 500), colors):
        axes[0].axhline(deadline, color=color, linestyle=":", linewidth=0.8, alpha=0.5)
    axes[0].set(title="Event-driven phase frames", xlabel="seed", ylabel="frame")
    axes[0].set_xticks(seeds)
    axes[0].set_ylim(0, 520)
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend(ncol=2, fontsize=8)

    width = 0.38
    axes[1].bar(seeds - width / 2, progress, width, label="directional J [m]", color="#ef4444")
    axes[1].bar(seeds + width / 2, coverage, width, label="strict coverage", color="#14b8a6")
    axes[1].axhline(0.15, color="#ef4444", linestyle=":", linewidth=1.0)
    axes[1].axhline(0.70, color="#14b8a6", linestyle=":", linewidth=1.0)
    axes[1].set(title="Transport and enclosure gates", xlabel="seed", ylabel="value")
    axes[1].set_xticks(seeds)
    axes[1].set_ylim(0, 1.08)
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend(fontsize=8)

    bar_colors = np.where(valid, "#6366f1", "#dc2626")
    axes[2].bar(seeds, fps, color=bar_colors)
    axes[2].axhline(20.0, color="black", linestyle="--", linewidth=1.0, label="20 frame/s")
    axes[2].set(title="End-to-end simulation throughput", xlabel="seed", ylabel="frames / wall s")
    axes[2].set_xticks(seeds)
    axes[2].grid(axis="y", alpha=0.25)
    axes[2].legend(fontsize=8)

    fig.suptitle(
        f"DBACT v3 · {int(np.sum(valid))}/{len(valid)} valid · "
        f"mean J={np.mean(progress):.3f} m · mean coverage={np.mean(coverage):.3f}",
        fontsize=13,
        fontweight="bold",
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    main()
