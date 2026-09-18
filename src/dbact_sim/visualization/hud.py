"""Phase-aware heads-up display that never overlaps the simulation world."""

from __future__ import annotations

import math

import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

from dbact_sim.trace import SimulationTrace

from .styles import PHASE_COLORS, VisualStyle


PHASE_PATH = (
    ("SEARCH", "SE"),
    ("MAP", "MP"),
    ("ENCLOSE", "EN"),
    ("CONTACT_READY", "CT"),
    ("TRANSPORT", "TR"),
    ("BRAKE", "BR"),
    ("HOLD", "HD"),
)


class PhaseHUD:
    def __init__(self, ax, trace: SimulationTrace, style: VisualStyle) -> None:
        self.ax = ax
        self.trace = trace
        self.style = style
        seen: set[str] = set()
        self._seen_phases: list[frozenset[str]] = []
        for phase in trace.phase_labels:
            seen.add(phase)
            self._seen_phases.append(frozenset(seen))
        ax.set_axis_off()
        ax.set_facecolor(style.panel_face)
        ax.add_patch(
            Rectangle(
                (0.0, 0.0),
                1.0,
                1.0,
                transform=ax.transAxes,
                facecolor=style.panel_face,
                edgecolor="none",
                zorder=-10,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (0.04, 0.895),
                0.92,
                0.075,
                boxstyle="round,pad=0.012,rounding_size=0.025",
                transform=ax.transAxes,
                facecolor=PHASE_COLORS["SEARCH"],
                edgecolor="none",
            )
        )
        self.phase_box = ax.patches[-1]
        self.phase_text = ax.text(
            0.5,
            0.932,
            "SEARCH",
            transform=ax.transAxes,
            ha="center",
            va="center",
            color="#ffffff",
            fontsize=13,
            fontweight="bold",
        )
        self.clock_text = ax.text(
            0.07,
            0.86,
            "",
            transform=ax.transAxes,
            ha="left",
            va="top",
            color=style.muted,
            fontsize=8.5,
        )
        self.phase_nodes: dict[str, Circle] = {}
        for index, (phase, short) in enumerate(PHASE_PATH):
            x = 0.105 + 0.132 * index
            node = Circle(
                (x, 0.81),
                0.022,
                transform=ax.transAxes,
                facecolor="#334155",
                edgecolor="none",
            )
            ax.add_patch(node)
            ax.text(
                x,
                0.81,
                short,
                transform=ax.transAxes,
                ha="center",
                va="center",
                color="#ffffff",
                fontsize=5.7,
                fontweight="bold",
            )
            self.phase_nodes[phase] = node
        self.metrics_text = ax.text(
            0.07,
            0.765,
            "",
            transform=ax.transAxes,
            ha="left",
            va="top",
            color=style.text,
            fontsize=8.1,
            linespacing=1.42,
            family="monospace",
        )
        self.solver_text = ax.text(
            0.07,
            0.18,
            "",
            transform=ax.transAxes,
            ha="left",
            va="top",
            color=style.text,
            fontsize=8.0,
            linespacing=1.4,
            family="monospace",
        )
        self.fps_text = ax.text(
            0.07,
            0.045,
            "",
            transform=ax.transAxes,
            ha="left",
            va="bottom",
            color=style.muted,
            fontsize=8.0,
            family="monospace",
        )

    def update(self, frame: int, rendering_fps: float | None = None) -> list:
        trace = self.trace
        frame = int(np.clip(frame, 0, trace.frame_count - 1))
        cargo_id = trace.cargo_ids[0]
        phase = trace.phase_labels[frame]
        self.phase_box.set_facecolor(PHASE_COLORS.get(phase, "#475569"))
        self.phase_text.set_text(phase)
        self.clock_text.set_text(
            f"Frame {frame:04d}/{trace.frame_count - 1:04d}   t = {trace.times[frame]:6.2f} s"
        )
        seen = self._seen_phases[frame]
        for name, node in self.phase_nodes.items():
            node.set_facecolor(PHASE_COLORS[name] if name in seen else "#334155")
            node.set_alpha(1.0 if name == phase else (0.58 if name in seen else 0.30))

        detected = int(trace.detection_counts[cargo_id][frame])
        coverage = float(trace.strict_coverage[cargo_id][frame])
        gap = float(trace.max_uncovered_gap[cargo_id][frame])
        contacts = trace.contact_ready_agents[frame]
        pushers = trace.push_agents[frame]
        progress = float(trace.directional_progress[cargo_id][frame])
        length = float(trace.target_distance[cargo_id])
        ratio = float(trace.progress_ratio[cargo_id][frame])
        efficiency = float(trace.direction_efficiency[cargo_id][frame])
        cross_track = float(trace.cross_track_error[cargo_id][frame])
        rotation = float(trace.cargo_rotation_deg[cargo_id][frame])
        net_force = float(np.linalg.norm(trace.net_force[cargo_id][frame]))
        torque = float(trace.net_torque[cargo_id][frame])
        min_distance = float(trace.min_distances[frame])
        penetration = float(trace.max_penetration[cargo_id][frame])

        self.metrics_text.set_text(
            "\n".join(
                [
                    f"Mapped samples  {detected:8d}",
                    f"Coverage        {coverage:8.3f}",
                    f"Max gap         {_format(gap, 'm'):>8}",
                    f"Contact agents  {len(contacts):8d}",
                    f"Push agents     {len(pushers):8d}",
                    "",
                    f"Progress J      {progress:7.3f} m",
                    f"Target L        {_format(length, 'm'):>8}",
                    f"J / L           {_format(ratio, ''):>8}",
                    f"Dir efficiency  {_format(efficiency, ''):>8}",
                    f"Cross-track     {_format(cross_track, 'm'):>8}",
                    f"Cargo rotation  {_format(rotation, 'deg'):>8}",
                    "",
                    f"Net force       {_format(net_force, 'N'):>8}",
                    f"Net torque      {_format(torque, 'Nm'):>8}",
                    f"Min robot dist  {_format(min_distance, 'm'):>8}",
                    f"Max penetration {_format(penetration, 'm', digits=4):>8}",
                ]
            )
        )

        qp = trace.qp_status_counts[frame]
        qp_summary = ", ".join(f"{name}:{count}" for name, count in sorted(qp.items())) or "not recorded"
        fallback_value = int(trace.solver_fallbacks[frame])
        infeasible_value = int(trace.solver_infeasible[frame])
        self.solver_text.set_text(
            "\n".join(
                [
                    "QP STATUS",
                    _wrap(qp_summary, 27),
                    f"fallback      {_count(fallback_value):>6}",
                    f"infeasible    {_count(infeasible_value):>6}",
                ]
            )
        )
        solver_alert = fallback_value > 0 or infeasible_value > 0
        self.solver_text.set_color("#fca5a5" if solver_alert else self.style.text)
        simulation_fps = float(trace.settings.get("simulation_fps", math.nan))
        render_value = math.nan if rendering_fps is None else float(rendering_fps)
        self.fps_text.set_text(
            f"simulation  {_format(simulation_fps, 'FPS'):>10}\n"
            f"rendering   {_format(render_value, 'FPS'):>10}"
        )
        return [
            self.phase_box,
            self.phase_text,
            self.clock_text,
            *self.phase_nodes.values(),
            self.metrics_text,
            self.solver_text,
            self.fps_text,
        ]


def _format(value: float, unit: str, digits: int = 2) -> str:
    if not np.isfinite(value):
        return f"-- {unit}".rstrip()
    return f"{value:.{digits}f} {unit}".rstrip()


def _count(value: int) -> str:
    return "--" if value < 0 else str(value)


def _wrap(value: str, width: int) -> str:
    words = value.split(", ")
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current}, {word}"
        if len(candidate) <= width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)
    return "\n".join(lines)


__all__ = ["PhaseHUD"]
