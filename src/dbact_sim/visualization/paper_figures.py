"""Paper-quality vector figures generated from an immutable simulation trace."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon

from dbact_sim.trace import SimulationTrace

from .debug_overlays import fused_boundary_polyline
from .styles import MODE_COLORS, PHASE_COLORS, get_style


PAPER_PHASES = (
    ("SEARCH", "SEARCH"),
    ("MAP", "DISCOVERY / MAP"),
    ("ENCLOSE", "ENCLOSE"),
    ("TRANSPORT", "TRANSPORT"),
    ("HOLD", "HOLD"),
)


def write_research_paper_figures(
    trace: SimulationTrace,
    output_dir: str | Path,
    *,
    formats: tuple[str, ...] = ("png", "pdf", "svg"),
    dpi: int = 220,
) -> dict[str, list[Path]]:
    """Write Figures A--G and return every created path by figure key."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    created: dict[str, list[Path]] = {}

    fig = _phase_sequence(trace)
    created["A"] = _save(fig, output, "figure_A_phase_sequence", formats, dpi)

    fig = _cargo_trajectory(trace)
    created["B"] = _save(fig, output, "figure_B_cargo_trajectory", formats, dpi)

    fig = _directional_progress(trace)
    created["C"] = _save(fig, output, "figure_C_directional_progress", formats, dpi)

    fig = _coverage_and_gap(trace)
    created["D"] = _save(fig, output, "figure_D_coverage_boundary_gap", formats, dpi)

    fig = _safety(trace)
    created["E"] = _save(fig, output, "figure_E_safety", formats, dpi)

    fig = _transport_quality(trace)
    created["F"] = _save(fig, output, "figure_F_transport_quality", formats, dpi)

    fig = _control_effort(trace)
    created["G"] = _save(fig, output, "figure_G_control_effort", formats, dpi)
    return created


def phase_keyframes(trace: SimulationTrace) -> dict[str, int | None]:
    """Choose the middle observed frame of each paper phase; never fabricate one."""
    selected: dict[str, int | None] = {}
    for phase, _ in PAPER_PHASES:
        aliases = ("ENCLOSE", "CONTACT_READY") if phase == "ENCLOSE" else (phase,)
        candidates = np.flatnonzero(np.isin(np.asarray(trace.phase_labels), aliases))
        selected[phase] = int(candidates[len(candidates) // 2]) if len(candidates) else None
    return selected


def _phase_sequence(trace: SimulationTrace):
    style = get_style("paper")
    frames = phase_keyframes(trace)
    fig, axes = plt.subplots(1, 5, figsize=(16.0, 3.35), sharex=True, sharey=True)
    for ax, (phase, title) in zip(axes, PAPER_PHASES):
        frame = frames[phase]
        display_frame = trace.frame_count - 1 if frame is None else frame
        _draw_paper_world(ax, trace, display_frame, show_map=phase in {"MAP", "ENCLOSE"})
        ax.set_title(
            title if frame is not None else f"{title}\nnot observed",
            fontsize=10,
            fontweight="bold",
            color=PHASE_COLORS[phase] if frame is not None else "#991b1b",
        )
        ax.text(
            0.03,
            0.03,
            f"frame {display_frame} | t={trace.times[display_frame]:.2f}s",
            transform=ax.transAxes,
            fontsize=6.8,
            color=style.muted,
        )
        if frame is None:
            ax.text(
                0.5,
                0.5,
                "PHASE NOT OBSERVED",
                transform=ax.transAxes,
                ha="center",
                va="center",
                color="#991b1b",
                fontsize=8,
                fontweight="bold",
                bbox={"facecolor": "white", "edgecolor": "#991b1b", "alpha": 0.9},
            )
    axes[0].set_ylabel("y [m]")
    for ax in axes:
        ax.set_xlabel("x [m]")
    handles = [
        Line2D([], [], color=style.cargo_face, linewidth=7, label="cargo"),
        Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["SEARCH"], label="search/map robot"),
        Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["ENCLOSE"], label="enclose/contact robot"),
        Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["TRANSPORT"], label="pushing robot"),
        Line2D([], [], color=style.mapped, linestyle=":", label="estimated boundary"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=5, frameon=False, fontsize=8)
    fig.subplots_adjust(left=0.045, right=0.995, bottom=0.20, top=0.88, wspace=0.10)
    return fig


def _draw_paper_world(ax, trace: SimulationTrace, frame: int, *, show_map: bool) -> None:
    style = get_style("paper")
    xmin, xmax, ymin, ymax = trace.domain
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_facecolor(style.world_face)
    ax.grid(True, color=style.grid, linewidth=0.45, alpha=0.6)
    ax.tick_params(labelsize=6.5)

    for cargo_id in trace.cargo_ids:
        centers = trace.cargo_centers[cargo_id][: frame + 1]
        ax.plot(centers[:, 0], centers[:, 1], color=style.goal, linewidth=1.2, alpha=0.65)
        vertices = trace.cargo_vertices[cargo_id][frame]
        ax.add_patch(
            Polygon(
                vertices,
                closed=True,
                facecolor=style.cargo_face,
                edgecolor=style.cargo_edge,
                linewidth=1.3,
                alpha=0.88,
                zorder=4,
            )
        )
        center = trace.cargo_centers[cargo_id][frame]
        goal = trace.goal_directions[cargo_id]
        ax.annotate(
            "",
            xy=center + 0.55 * goal,
            xytext=center,
            arrowprops={"arrowstyle": "-|>", "color": style.goal, "lw": 1.5},
            zorder=7,
        )
        if cargo_id in trace.goal_targets:
            target = trace.goal_targets[cargo_id]
            ax.scatter(target[0], target[1], marker="X", s=42, color=style.target, zorder=7)

    contacts = set(trace.contact_ready_agents[frame])
    pushers = set(trace.push_agents[frame])
    radius = float(trace.settings.get("robot_radius", 0.12))
    for index, agent_id in enumerate(trace.agent_ids):
        point = trace.agent_positions[frame, index]
        mode = trace.agent_modes[frame][index]
        color = MODE_COLORS.get(mode, "#64748b")
        if agent_id in pushers:
            color = PHASE_COLORS["TRANSPORT"]
        elif agent_id in contacts:
            color = PHASE_COLORS["CONTACT_READY"]
        ax.add_patch(
            Circle(point, radius, facecolor=color, edgecolor="#ffffff", linewidth=0.55, zorder=8)
        )

    if show_map:
        snapshot = trace.visual_snapshot(frame)
        if len(snapshot.mapped_points):
            ax.scatter(
                snapshot.mapped_points[:, 0],
                snapshot.mapped_points[:, 1],
                s=2.4,
                color=style.mapped,
                alpha=0.45,
                zorder=3,
            )
            polyline = fused_boundary_polyline(trace, frame)
            ax.plot(polyline[:, 0], polyline[:, 1], ":", color=style.mapped, linewidth=0.8)


def _cargo_trajectory(trace: SimulationTrace):
    style = get_style("paper")
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    ax.set_aspect("equal", adjustable="box")
    _paper_axes(ax, trace, "Cargo trajectory and transport goal", "x [m]", "y [m]")
    extent_points: list[np.ndarray] = []
    for cargo_id in trace.cargo_ids:
        centers = trace.cargo_centers[cargo_id]
        ax.plot(centers[:, 0], centers[:, 1], color=style.goal, linewidth=2.0, label=f"{cargo_id} trajectory")
        start_vertices = trace.cargo_vertices[cargo_id][0]
        final_vertices = trace.cargo_vertices[cargo_id][-1]
        ax.add_patch(
            Polygon(
                start_vertices,
                closed=True,
                facecolor="none",
                edgecolor="#2563eb",
                linestyle=":",
                linewidth=1.2,
                alpha=0.8,
            )
        )
        ax.add_patch(
            Polygon(
                final_vertices,
                closed=True,
                facecolor=style.cargo_face,
                edgecolor=style.cargo_edge,
                linewidth=1.1,
                alpha=0.35,
            )
        )
        ax.scatter(centers[0, 0], centers[0, 1], marker="o", s=48, color="#2563eb", label="start")
        ax.scatter(centers[-1, 0], centers[-1, 1], marker="s", s=48, color="#111827", label="final")
        goal = trace.goal_directions[cargo_id]
        goal_tip = centers[0] + 0.85 * goal
        ax.annotate(
            "",
            xy=goal_tip,
            xytext=centers[0],
            arrowprops={"arrowstyle": "-|>", "color": style.goal, "lw": 2.0},
        )
        ax.text(
            centers[0, 0] + 0.04,
            centers[0, 1] + 0.06,
            "goal direction",
            color=style.goal,
            fontsize=8,
            ha="left",
            va="bottom",
        )
        if cargo_id in trace.goal_targets:
            target = trace.goal_targets[cargo_id]
            ax.scatter(target[0], target[1], marker="X", s=85, color=style.target, label="target")
            extent_points.append(target[None, :])
        extent_points.extend([start_vertices, final_vertices, goal_tip[None, :]])
    if extent_points:
        extent = np.vstack(extent_points)
        low = np.min(extent, axis=0)
        high = np.max(extent, axis=0)
        span = np.maximum(high - low, 0.5)
        padding = 0.16 * float(np.max(span))
        ax.set_xlim(low[0] - padding, high[0] + padding)
        ax.set_ylim(low[1] - padding, high[1] + padding)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    return fig


def _directional_progress(trace: SimulationTrace):
    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    _paper_axes(ax, trace, "Directional progress", "Time [s]", "J(t) [m]")
    for cargo_id in trace.cargo_ids:
        ax.plot(trace.times, trace.directional_progress[cargo_id], linewidth=2.0, label=f"{cargo_id} J(t)")
        ax.axhline(trace.target_distance[cargo_id], color="#b91c1c", linestyle="--", linewidth=1.1, label="target L")
    _shade_phases(ax, trace)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    return fig


def _coverage_and_gap(trace: SimulationTrace):
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.4), sharex=True)
    cargo_id = trace.cargo_ids[0]
    axes[0].plot(trace.times, trace.strict_coverage[cargo_id], color="#15803d", linewidth=1.8)
    _paper_axes(axes[0], trace, "Boundary coverage and maximum gap", None, "Strict coverage")
    axes[0].set_ylim(-0.02, 1.02)
    axes[1].plot(trace.times, trace.max_uncovered_gap[cargo_id], color="#b45309", linewidth=1.8)
    _paper_axes(axes[1], trace, None, "Time [s]", "Max gap [m]")
    for ax in axes:
        _shade_phases(ax, trace)
    fig.tight_layout()
    return fig


def _safety(trace: SimulationTrace):
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.4), sharex=True)
    cargo_id = trace.cargo_ids[0]
    axes[0].plot(trace.times, trace.min_distances, color="#2563eb", linewidth=1.8)
    axes[0].axhline(trace.settings.get("d_min", np.nan), color="#991b1b", linestyle="--", linewidth=1.0, label="d_min")
    axes[0].legend(frameon=False, fontsize=8)
    _paper_axes(axes[0], trace, "Safety margins", None, "Minimum robot distance [m]")
    axes[1].plot(trace.times, trace.max_penetration[cargo_id], color="#dc2626", linewidth=1.8)
    _paper_axes(axes[1], trace, None, "Time [s]", "Maximum penetration [m]")
    for ax in axes:
        _shade_phases(ax, trace)
    fig.tight_layout()
    return fig


def _transport_quality(trace: SimulationTrace):
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.4), sharex=True)
    cargo_id = trace.cargo_ids[0]
    axes[0].plot(trace.times, trace.cross_track_error[cargo_id], color="#7c3aed", linewidth=1.8)
    _paper_axes(axes[0], trace, "Transport quality", None, "Cross-track error [m]")
    axes[1].plot(trace.times, trace.cargo_rotation_deg[cargo_id], color="#c2410c", linewidth=1.8)
    _paper_axes(axes[1], trace, None, "Time [s]", "Cargo rotation [deg]")
    for ax in axes:
        _shade_phases(ax, trace)
    fig.tight_layout()
    return fig


def _control_effort(trace: SimulationTrace):
    fig, axes = plt.subplots(3, 1, figsize=(7.0, 6.6), sharex=True)
    cargo_id = trace.cargo_ids[0]
    force = np.linalg.norm(trace.net_force[cargo_id], axis=1)
    axes[0].plot(trace.times, force, color="#dc2626", linewidth=1.6)
    _paper_axes(axes[0], trace, "Control effort and contact", None, "Net force [N]")
    axes[1].plot(trace.times, trace.net_torque[cargo_id], color="#7c3aed", linewidth=1.6)
    _paper_axes(axes[1], trace, None, None, "Net torque [Nm]")
    axes[2].step(trace.times, trace.contact_counts[cargo_id], where="post", color="#0f766e", linewidth=1.6)
    _paper_axes(axes[2], trace, None, "Time [s]", "Contact count")
    for ax in axes:
        _shade_phases(ax, trace)
    fig.tight_layout()
    return fig


def _paper_axes(ax, trace: SimulationTrace, title: str | None, xlabel: str | None, ylabel: str) -> None:
    if title:
        ax.set_title(title, fontsize=11, fontweight="bold")
    if xlabel:
        ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, color="#e5e7eb", linewidth=0.55, alpha=0.75)
    ax.spines[["top", "right"]].set_visible(False)


def _shade_phases(ax, trace: SimulationTrace) -> None:
    start = 0
    labels = trace.phase_labels
    for index in range(1, len(labels) + 1):
        if index == len(labels) or labels[index] != labels[start]:
            phase = labels[start]
            left = trace.times[start]
            right = trace.times[min(index, len(trace.times) - 1)]
            if right <= left:
                right = left + trace.dt
            ax.axvspan(left, right, color=PHASE_COLORS.get(phase, "#94a3b8"), alpha=0.045, linewidth=0)
            start = index


def _save(fig, output: Path, stem: str, formats: tuple[str, ...], dpi: int) -> list[Path]:
    paths: list[Path] = []
    for suffix in formats:
        suffix = suffix.lower().lstrip(".")
        if suffix not in {"png", "pdf", "svg"}:
            raise ValueError(f"unsupported paper figure format: {suffix}")
        path = output / f"{stem}.{suffix}"
        fig.savefig(path, dpi=dpi if suffix == "png" else None, bbox_inches="tight")
        if suffix == "svg":
            # Matplotlib writes path commands with harmless trailing spaces.
            # Normalize them so generated vector artifacts pass repository
            # whitespace checks and produce stable text diffs.
            content = path.read_text(encoding="utf-8")
            path.write_text(
                "\n".join(line.rstrip() for line in content.splitlines()) + "\n",
                encoding="utf-8",
            )
        paths.append(path)
    plt.close(fig)
    return paths


__all__ = ["PAPER_PHASES", "phase_keyframes", "write_research_paper_figures"]
