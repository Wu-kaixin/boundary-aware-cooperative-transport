"""Reusable-artist world renderer for offline demos and static figures."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Polygon

from dbact_sim.trace import SimulationTrace

from .debug_overlays import fused_boundary_polyline, sensor_segments
from .hud import PhaseHUD
from .styles import MODE_COLORS, PHASE_COLORS, VisualStyle, get_style


class ResearchVisualizer:
    """Render a saved trace without touching controller or physics state."""

    def __init__(
        self,
        trace: SimulationTrace,
        view_mode: str = "demo",
        *,
        show_ids: bool | None = None,
        show_sensor: bool | None = None,
        show_map: bool | None = None,
        show_cage: bool | None = None,
        show_truth_debug: bool | None = None,
        figure=None,
        world_ax=None,
        hud_ax=None,
    ) -> None:
        self.trace = trace
        self.style = get_style(view_mode)
        self.show_ids = self.style.show_ids if show_ids is None else bool(show_ids)
        self.show_sensor = self.style.show_sensor if show_sensor is None else bool(show_sensor)
        self.show_map = self.style.show_map if show_map is None else bool(show_map)
        self.show_cage = self.style.show_cage if show_cage is None else bool(show_cage)
        self.show_truth_debug = (
            self.style.show_truth_debug if show_truth_debug is None else bool(show_truth_debug)
        )
        if figure is None:
            if self.style.show_hud:
                figure = plt.figure(figsize=(13.4, 7.6), facecolor=self.style.figure_face)
                grid = figure.add_gridspec(1, 2, width_ratios=(4.15, 1.0), wspace=0.035)
                world_ax = figure.add_subplot(grid[0, 0])
                hud_ax = figure.add_subplot(grid[0, 1])
            else:
                figure, world_ax = plt.subplots(
                    figsize=(7.5, 7.0),
                    facecolor=self.style.figure_face,
                )
        if world_ax is None:
            raise ValueError("world_ax is required when a figure is supplied")
        self.fig = figure
        self.world_ax = world_ax
        self.hud = PhaseHUD(hud_ax, trace, self.style) if hud_ax is not None else None
        self._setup_world()
        self.update(0)

    def _setup_world(self) -> None:
        trace, style, ax = self.trace, self.style, self.world_ax
        xmin, xmax, ymin, ymax = trace.domain
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_facecolor(style.world_face)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True, color=style.grid, linewidth=0.55, alpha=0.55)
        for spine in ax.spines.values():
            spine.set_color(style.grid)
        ax.tick_params(colors="#475569", labelsize=8.5)

        self.cargo_patches: dict[str, Polygon] = {}
        self.cargo_orientation: dict[str, Line2D] = {}
        self.cargo_trails: dict[str, Line2D] = {}
        self.goal_arrows: dict[str, FancyArrowPatch] = {}
        for cargo_id in trace.cargo_ids:
            patch = Polygon(
                trace.cargo_vertices[cargo_id][0],
                closed=True,
                facecolor=style.cargo_face,
                edgecolor=style.cargo_edge,
                linewidth=2.0,
                alpha=0.88,
                zorder=4,
            )
            ax.add_patch(patch)
            self.cargo_patches[cargo_id] = patch
            self.cargo_orientation[cargo_id] = ax.plot(
                [], [], color=style.cargo_edge, linewidth=2.2, zorder=5
            )[0]
            self.cargo_trails[cargo_id] = ax.plot(
                [], [], color=style.goal, linewidth=2.1, alpha=0.75, zorder=2
            )[0]
            arrow = FancyArrowPatch(
                (0, 0),
                (0, 0),
                arrowstyle="-|>",
                mutation_scale=16,
                color=style.goal,
                linewidth=2.2,
                zorder=7,
            )
            ax.add_patch(arrow)
            self.goal_arrows[cargo_id] = arrow
            if cargo_id in trace.goal_targets:
                target = trace.goal_targets[cargo_id]
                start = trace.cargo_centers[cargo_id][0]
                ax.plot(
                    [start[0], target[0]],
                    [start[1], target[1]],
                    linestyle="--",
                    color=style.goal,
                    linewidth=1.4,
                    alpha=0.6,
                    zorder=1,
                )
                ax.scatter(
                    target[0],
                    target[1],
                    marker="X",
                    s=125,
                    color=style.target,
                    edgecolor="#ffffff",
                    linewidth=1.2,
                    zorder=8,
                    label="target",
                )

        radius = float(trace.settings.get("robot_radius", 0.12))
        self.agent_patches: list[Circle] = []
        self.agent_labels = []
        self.agent_trails: list[Line2D] = []
        for index, agent_id in enumerate(trace.agent_ids):
            point = trace.agent_positions[0, index]
            circle = Circle(
                point,
                radius,
                facecolor=MODE_COLORS.get(trace.agent_modes[0][index], "#64748b"),
                edgecolor="#ffffff",
                linewidth=1.1,
                zorder=9,
            )
            ax.add_patch(circle)
            self.agent_patches.append(circle)
            trail = ax.plot([], [], color=style.trajectory, linewidth=0.9, alpha=0.28, zorder=1)[0]
            self.agent_trails.append(trail)
            label = ax.text(
                point[0],
                point[1],
                agent_id.split("_")[-1],
                ha="center",
                va="center",
                color="#ffffff",
                fontsize=6.4,
                fontweight="bold",
                zorder=10,
                visible=self.show_ids,
            )
            self.agent_labels.append(label)

        self.sensor_collection = LineCollection(
            [], colors=style.detected, linewidths=0.55, alpha=0.22, zorder=2
        )
        ax.add_collection(self.sensor_collection)
        self.detected_points = ax.scatter(
            [], [], s=10, color=style.detected, edgecolor="none", alpha=0.8, zorder=6
        )
        self.map_points = ax.scatter(
            [], [], s=8, color=style.mapped, edgecolor="none", alpha=0.55, zorder=3
        )
        self.map_line = ax.plot(
            [], [], linestyle=":", color=style.mapped, linewidth=1.0, alpha=0.8, zorder=3
        )[0]
        self.cage_points = ax.scatter(
            [], [], s=9, facecolor="none", edgecolor=style.cage, linewidth=0.65, alpha=0.65, zorder=3
        )

        handles = [
            Line2D([], [], color=style.cargo_face, linewidth=7, label="irregular cargo"),
            Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["SEARCH"], label="search/map"),
            Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["ENCLOSE"], label="enclose/contact"),
            Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["TRANSPORT"], label="push"),
            Line2D([], [], marker="o", linestyle="", color=PHASE_COLORS["HOLD"], label="hold"),
            Line2D([], [], color=style.detected, linewidth=1.2, label="detected boundary"),
            Line2D([], [], color=style.mapped, linestyle=":", label="fused estimate"),
        ]
        ax.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.085),
            ncol=4,
            frameon=False,
            fontsize=7.5,
        )
        self.phase_banner = ax.text(
            0.025,
            0.96,
            "",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=11.5,
            fontweight="bold",
            color="#ffffff",
            bbox={"boxstyle": "round,pad=0.4", "edgecolor": "none", "facecolor": PHASE_COLORS["SEARCH"], "alpha": 0.92},
            zorder=20,
        )
        self.fig.subplots_adjust(bottom=0.13, top=0.97, left=0.07, right=0.985)

    def update(self, frame: int, rendering_fps: float | None = None) -> list:
        trace, style = self.trace, self.style
        frame = int(np.clip(frame, 0, trace.frame_count - 1))
        trail_start = 0 if style.trail_frames is None else max(0, frame - style.trail_frames)
        artists: list = []

        for cargo_id in trace.cargo_ids:
            vertices = trace.cargo_vertices[cargo_id][frame]
            center = trace.cargo_centers[cargo_id][frame]
            angle = float(trace.cargo_angles[cargo_id][frame])
            span = max(float(np.ptp(vertices[:, 0])), float(np.ptp(vertices[:, 1])))
            direction = np.array([np.cos(angle), np.sin(angle)])
            self.cargo_patches[cargo_id].set_xy(vertices)
            self.cargo_orientation[cargo_id].set_data(
                [center[0], center[0] + 0.28 * span * direction[0]],
                [center[1], center[1] + 0.28 * span * direction[1]],
            )
            centers = trace.cargo_centers[cargo_id][trail_start : frame + 1]
            self.cargo_trails[cargo_id].set_data(centers[:, 0], centers[:, 1])
            goal = trace.goal_directions[cargo_id]
            goal_length = max(0.55, 0.32 * span)
            self.goal_arrows[cargo_id].set_positions(center, center + goal_length * goal)
            artists.extend(
                [
                    self.cargo_patches[cargo_id],
                    self.cargo_orientation[cargo_id],
                    self.cargo_trails[cargo_id],
                    self.goal_arrows[cargo_id],
                ]
            )

        contacts = set(trace.contact_ready_agents[frame])
        pushers = set(trace.push_agents[frame])
        for index, agent_id in enumerate(trace.agent_ids):
            point = trace.agent_positions[frame, index]
            mode = trace.agent_modes[frame][index]
            color = MODE_COLORS.get(mode, "#64748b")
            if agent_id in pushers:
                color = PHASE_COLORS["TRANSPORT"]
            elif agent_id in contacts:
                color = PHASE_COLORS["CONTACT_READY"]
            circle = self.agent_patches[index]
            circle.center = point
            circle.set_facecolor(color)
            circle.set_linewidth(2.1 if agent_id in contacts or agent_id in pushers else 1.1)
            history = trace.agent_positions[trail_start : frame + 1, index]
            self.agent_trails[index].set_data(history[:, 0], history[:, 1])
            self.agent_labels[index].set_position(point)
            artists.extend([circle, self.agent_trails[index], self.agent_labels[index]])

        snapshot = trace.visual_snapshot(frame)
        if self.show_sensor:
            self.sensor_collection.set_segments(sensor_segments(trace, frame))
            self.detected_points.set_offsets(snapshot.detected_points)
        else:
            self.sensor_collection.set_segments([])
            self.detected_points.set_offsets(np.empty((0, 2)))
        if self.show_map:
            self.map_points.set_offsets(snapshot.mapped_points)
            polyline = fused_boundary_polyline(trace, frame)
            if len(polyline):
                self.map_line.set_data(polyline[:, 0], polyline[:, 1])
            else:
                self.map_line.set_data([], [])
        else:
            self.map_points.set_offsets(np.empty((0, 2)))
            self.map_line.set_data([], [])
        self.cage_points.set_offsets(snapshot.cage_targets if self.show_cage else np.empty((0, 2)))
        artists.extend([self.sensor_collection, self.detected_points, self.map_points, self.map_line, self.cage_points])

        phase = trace.phase_labels[frame]
        self.phase_banner.set_text(phase.replace("_", " "))
        self.phase_banner.set_bbox(
            {"boxstyle": "round,pad=0.4", "edgecolor": "none", "facecolor": PHASE_COLORS.get(phase, "#475569"), "alpha": 0.92}
        )
        self.phase_banner.set_visible(style.name != "paper")
        artists.append(self.phase_banner)

        if self.hud is not None:
            artists.extend(self.hud.update(frame, rendering_fps))
        return artists

    def save_frame(self, frame: int, path, *, dpi: int = 180) -> None:
        self.update(frame)
        self.fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor=self.fig.get_facecolor())

    def close(self) -> None:
        plt.close(self.fig)


DemoVisualizer = ResearchVisualizer


__all__ = ["DemoVisualizer", "ResearchVisualizer", "VisualStyle"]
