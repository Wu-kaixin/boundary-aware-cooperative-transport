"""Paired success/failure figure for the frozen Claude v2 matrix."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from dbact_sim.trace import SimulationTrace

from .paper_figures import _draw_paper_world, _save


def write_outcome_comparison(
    success_trace: SimulationTrace,
    failure_trace: SimulationTrace,
    output_dir: str | Path,
    *,
    success_record: dict[str, Any],
    failure_record: dict[str, Any],
    frozen_success_record: dict[str, Any] | None = None,
    frozen_failure_record: dict[str, Any] | None = None,
    preview_only: bool = False,
    formats: tuple[str, ...] = ("png", "pdf", "svg"),
    dpi: int = 220,
) -> list[Path]:
    """Write Figure H without hiding the failed run or changing its time base."""
    fig = plt.figure(figsize=(13.2, 7.8))
    grid = fig.add_gridspec(2, 3, width_ratios=(1.0, 1.0, 1.08), hspace=0.34, wspace=0.28)
    success_world = fig.add_subplot(grid[0, 0])
    failure_world = fig.add_subplot(grid[0, 1])
    progress_ax = fig.add_subplot(grid[0, 2])
    coverage_ax = fig.add_subplot(grid[1, 0])
    safety_ax = fig.add_subplot(grid[1, 1])
    audit_ax = fig.add_subplot(grid[1, 2])

    _draw_paper_world(success_world, success_trace, success_trace.frame_count - 1, show_map=False)
    _draw_paper_world(failure_world, failure_trace, failure_trace.frame_count - 1, show_map=False)
    _crop_world(success_world, success_trace)
    _crop_world(failure_world, failure_trace)
    success_world.set_title(
        f"(a) Representative success\n{success_record['case_id']}", fontsize=9, fontweight="bold"
    )
    failure_world.set_title(
        f"(b) High-concavity failure\n{failure_record['case_id']}",
        fontsize=9,
        fontweight="bold",
    )
    success_world.set_xlabel("x [m]")
    success_world.set_ylabel("y [m]")
    failure_world.set_xlabel("x [m]")

    colors = {"success": "#15803d", "failure": "#b91c1c"}
    for label, trace in (("success", success_trace), ("failure", failure_trace)):
        cargo_id = trace.cargo_ids[0]
        episode = _episode_fraction(trace)
        progress_ax.plot(
            episode,
            trace.progress_ratio[cargo_id],
            color=colors[label],
            linewidth=1.8,
            label=label,
        )
        coverage_ax.plot(
            episode,
            trace.strict_coverage[cargo_id],
            color=colors[label],
            linewidth=1.8,
            label=label,
        )
        d_min = float(trace.settings.get("d_min", np.nan))
        margin = trace.min_distances - d_min if np.isfinite(d_min) else trace.min_distances
        safety_ax.plot(episode, margin, color=colors[label], linewidth=1.8, label=label)

    progress_ax.axhline(1.0, color="#111827", linestyle="--", linewidth=1.0, label="target")
    _style_axis(progress_ax, "(c) Directional target progress", "Episode fraction", "J(t) / L")
    _style_axis(coverage_ax, "(d) Strict boundary coverage", "Episode fraction", "Coverage")
    _style_axis(safety_ax, "(e) Inter-agent safety margin", "Episode fraction", "d_min margin [m]")
    progress_ax.legend(frameon=False, fontsize=8)
    coverage_ax.legend(frameon=False, fontsize=8)
    safety_ax.axhline(0.0, color="#111827", linestyle="--", linewidth=0.9)
    safety_ax.legend(frameon=False, fontsize=8)

    audit_ax.axis("off")
    audit_ax.set_title("(f) Trace and source audit", fontsize=10, fontweight="bold", loc="left")
    audit_ax.text(
        0.0,
        0.83,
        _audit_text(
            success_record,
            failure_record,
            frozen_success_record=frozen_success_record,
            frozen_failure_record=frozen_failure_record,
        ),
        transform=audit_ax.transAxes,
        ha="left",
        va="top",
        family="monospace",
        fontsize=7.6,
        linespacing=1.48,
        bbox={"facecolor": "#f8fafc", "edgecolor": "#cbd5e1", "pad": 8.0},
    )

    title = (
        "CURRENT-ENVIRONMENT PREVIEW — not evidence for frozen continuous metrics"
        if preview_only
        else "Predeclared success/failure comparison from the complete 180-case matrix"
    )
    fig.suptitle(
        title,
        fontsize=11,
        fontweight="bold",
        y=0.99,
        color="#991b1b" if preview_only else "#111827",
    )
    fig.subplots_adjust(top=0.82, left=0.055, right=0.985, bottom=0.08)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    return _save(fig, output, "figure_H_success_failure_comparison", formats, dpi)


def _episode_fraction(trace: SimulationTrace) -> np.ndarray:
    return np.linspace(0.0, 1.0, trace.frame_count)


def _crop_world(ax, trace: SimulationTrace) -> None:
    frame = trace.frame_count - 1
    points = [trace.agent_positions[frame]]
    points.extend(trace.cargo_vertices[cargo_id][frame] for cargo_id in trace.cargo_ids)
    cloud = np.vstack(points)
    low = np.min(cloud, axis=0)
    high = np.max(cloud, axis=0)
    span = max(float(np.max(high - low)), 1.0)
    padding = 0.25 * span
    xmin, xmax, ymin, ymax = trace.domain
    ax.set_xlim(max(xmin, low[0] - padding), min(xmax, high[0] + padding))
    ax.set_ylim(max(ymin, low[1] - padding), min(ymax, high[1] + padding))


def _style_axis(ax, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, color="#e5e7eb", linewidth=0.55, alpha=0.75)
    ax.spines[["top", "right"]].set_visible(False)


def _audit_text(
    success: dict[str, Any],
    failure: dict[str, Any],
    *,
    frozen_success_record: dict[str, Any] | None,
    frozen_failure_record: dict[str, Any] | None,
) -> str:
    def line(label: str, left: str, right: str) -> str:
        return f"{label:<16}{left:>12}  {right:>12}"

    rows = [
            line("", "SUCCESS", "FAILURE"),
            line("shape", str(success["shape"]), str(failure["shape"])),
            line("seed", str(success["seed"]), str(failure["seed"])),
            line("concavity", f"{success['concavity_ratio']:.3f}", f"{failure['concavity_ratio']:.3f}"),
            line("trace J / L", f"{success['J_over_target']:.3f}", f"{failure['J_over_target']:.3f}"),
            line("efficiency", f"{success['efficiency']:.3f}", f"{failure['efficiency']:.3f}"),
            line("frames", str(int(success["frames_run"])), str(int(failure["frames_run"]))),
            line("fallbacks", str(success["solver_fallbacks"]), str(failure["solver_fallbacks"])),
            line("infeasible", str(success["solver_infeasible"]), str(failure["solver_infeasible"])),
            line("final phase", str(success["final_phase"]), str(failure["final_phase"])),
            line("verdict", str(success["failure_class"]), str(failure["failure_class"])),
        ]
    if frozen_success_record is not None and frozen_failure_record is not None:
        rows.insert(
            5,
            line(
                "frozen J / L",
                f"{frozen_success_record['J_over_target']:.3f}",
                f"{frozen_failure_record['J_over_target']:.3f}",
            ),
        )
    return "\n".join(rows)


__all__ = ["write_outcome_comparison"]
