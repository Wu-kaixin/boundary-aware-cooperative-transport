from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Polygon

from dbact.boundary_density import BoundaryAwareDensity
from dbact.cargo import Cargo

from .environment import SimulationEnvironment


def plot_snapshot(env: SimulationEnvironment, path: str | Path, title: str = "DBACT final snapshot") -> None:
    fig, ax = plt.subplots(figsize=(7, 7))
    _draw_world(ax, env)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_trajectories(env: SimulationEnvironment, path: str | Path, title: str = "DBACT trajectories") -> None:
    fig, ax = plt.subplots(figsize=(7, 7))
    _draw_world(ax, env)
    for agent_id, points in env.log.agent_positions.items():
        arr = np.vstack(points)
        ax.plot(arr[:, 0], arr[:, 1], linewidth=1.2, label=agent_id)
    if len(env.log.agent_positions) <= 12:
        ax.legend(fontsize=7, loc="upper right")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_coverage_curve(env: SimulationEnvironment, path: str | Path, title: str = "Coverage Rate over Time") -> None:
    fig, ax = plt.subplots(figsize=(7, 5))
    for cargo_id, values in env.log.cargo_coverages.items():
        ax.plot(range(len(values)), values, label=f"{cargo_id} coverage")
    ax.set_title(title)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Coverage Rate")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    if env.log.cargo_coverages:
        ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_paper_frame(
    env: SimulationEnvironment,
    frame_index: int,
    path: str | Path,
    title: str = "Unknown Cargo + Local CVT Density + Local Agent CBF",
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    frame_index = max(0, min(frame_index, len(env.log.times) - 1))
    fig = plt.figure(figsize=(10, 5))
    world_ax = fig.add_subplot(1, 2, 1)
    density_ax = fig.add_subplot(1, 2, 2, projection="3d")
    _draw_paper_world(world_ax, env, frame_index)
    _draw_density_surface(density_ax, env, frame_index)
    fig.suptitle(f"{title} | Step {frame_index}/{len(env.log.times) - 1}", fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_paper_figures(
    env: SimulationEnvironment,
    output_dir: str | Path,
    frame_indices: list[int] | None = None,
) -> list[Path]:
    out = Path(output_dir)
    figures_dir = out / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    plot_coverage_curve(env, out / "coverage_rate_curve.png")
    if frame_indices is None:
        last = len(env.log.times) - 1
        frame_indices = sorted({0, last // 4, last // 2, (3 * last) // 4, last})
    paths: list[Path] = []
    for frame_index in frame_indices:
        path = figures_dir / f"FIG_{frame_index}.png"
        plot_paper_frame(env, frame_index, path)
        paths.append(path)
    return paths


class LivePaperViewer:
    """Interactive paper-style viewer for simulation runs."""

    def __init__(
        self,
        env: SimulationEnvironment,
        update_stride: int = 5,
        pause_s: float = 0.001,
        title: str = "Unknown Cargo + Local CVT Density + Local Agent CBF",
    ):
        self.update_stride = max(1, int(update_stride))
        self.pause_s = max(0.0, float(pause_s))
        self.title = title
        self.closed = False
        plt.ion()
        self.fig = plt.figure(num="DBACT Live Simulation", figsize=(10, 5), clear=True)
        self.world_ax = self.fig.add_subplot(1, 2, 1)
        self.density_ax = self.fig.add_subplot(1, 2, 2, projection="3d")
        self.fig.canvas.mpl_connect("close_event", self._handle_close)
        self.update(0, env, force=True)
        plt.show(block=False)

    def update(self, step_index: int, env: SimulationEnvironment, force: bool = False) -> None:
        if self.closed:
            return
        if not force and step_index % self.update_stride != 0:
            return
        if not env.log.times:
            return
        frame_index = len(env.log.times) - 1
        self.world_ax.clear()
        self.density_ax.clear()
        _draw_paper_world(self.world_ax, env, frame_index)
        _draw_density_surface(self.density_ax, env, frame_index)
        self.fig.suptitle(
            f"{self.title} | Step {step_index}",
            fontsize=11,
            fontweight="bold",
        )
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        plt.pause(self.pause_s)

    def finish(self, block: bool = True) -> None:
        if self.closed:
            return
        plt.ioff()
        if block:
            plt.show()
        else:
            plt.pause(self.pause_s)

    def _handle_close(self, *_args) -> None:
        self.closed = True


def animate_simulation(
    env: SimulationEnvironment,
    path: str | Path,
    title: str = "DBACT moving cargo demo",
    frame_stride: int = 5,
    fps: int = 12,
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    frames = list(range(0, len(env.log.times), max(1, frame_stride)))
    if frames[-1] != len(env.log.times) - 1:
        frames.append(len(env.log.times) - 1)

    fig, ax = plt.subplots(figsize=(7, 7))
    xmin, xmax, ymin, ymax = env.domain
    agent_ids = list(env.log.agent_positions)
    cargo_ids = list(env.log.cargo_vertices)
    colors = plt.get_cmap("tab10")

    def draw(frame_index: int) -> list:
        ax.clear()
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linewidth=0.4)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

        artists: list = []
        for cargo_id in cargo_ids:
            vertices = env.log.cargo_vertices[cargo_id][frame_index]
            patch = Polygon(vertices, closed=True, alpha=0.35, facecolor="tab:orange", edgecolor="black")
            ax.add_patch(patch)
            artists.append(patch)
            center = env.log.cargo_centers[cargo_id][frame_index]
            direction = next(c.transport_direction for c in env.cargoes if c.object_id == cargo_id)
            arrow = ax.arrow(
                center[0],
                center[1],
                0.35 * direction[0],
                0.35 * direction[1],
                width=0.015,
                length_includes_head=True,
                color="tab:red",
            )
            artists.append(arrow)
            coverage = env.log.cargo_coverages[cargo_id][frame_index]
            text = ax.text(center[0], center[1], f"{cargo_id}\ncoverage={coverage:.2f}", ha="center", va="center", fontsize=8)
            artists.append(text)

        for k, agent_id in enumerate(agent_ids):
            hist = np.vstack(env.log.agent_positions[agent_id])
            trail = hist[: frame_index + 1]
            line = ax.plot(trail[:, 0], trail[:, 1], color=colors(k % 10), linewidth=1.0, alpha=0.7)[0]
            point = ax.scatter(hist[frame_index, 0], hist[frame_index, 1], s=35, color=colors(k % 10))
            label = ax.text(hist[frame_index, 0], hist[frame_index, 1] + 0.08, agent_id.split("_")[-1], fontsize=6, ha="center")
            artists.extend([line, point, label])

        time_s = env.log.times[frame_index]
        min_dist = env.log.min_distances[frame_index]
        ax.set_title(f"{title} | t={time_s:.1f}s | min distance={min_dist:.2f}m")
        return artists

    ani = animation.FuncAnimation(fig, draw, frames=frames, interval=1000 / fps, blit=False)
    try:
        ani.save(path, writer=animation.PillowWriter(fps=fps), dpi=140)
    finally:
        plt.close(fig)


def _draw_paper_world(ax, env: SimulationEnvironment, frame_index: int) -> None:
    xmin, xmax, ymin, ymax = env.domain
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_facecolor("#d7dadd")
    ax.set_xlabel("x(m)", fontsize=13)
    ax.set_ylabel("y(m)", fontsize=13)
    positions = _agent_positions_at(env, frame_index)
    _draw_voronoi(ax, positions, env.domain)

    params = env.controller.params
    for cargo_id, vertices_history in env.log.cargo_vertices.items():
        vertices = vertices_history[frame_index]
        patch = Polygon(vertices, closed=True, alpha=0.55, facecolor="tab:orange", edgecolor="none")
        ax.add_patch(patch)
        center = env.log.cargo_centers[cargo_id][frame_index]
        boundary, normals = Cargo(cargo_id, vertices).boundary_samples(180)
        cage = boundary + params.cage_offset * normals
        ax.plot(cage[:, 0], cage[:, 1], "--", color="0.45", linewidth=1.0, alpha=0.8)
        ax.text(center[0], center[1], f"{cargo_id}\nunknown", ha="center", va="center", fontsize=7)

    comm_range = float(params.comm_range)
    d_min = float(params.d_min)
    for idx, point in enumerate(positions):
        comm = plt.Circle(point, comm_range, fill=False, linestyle=":", color="0.65", linewidth=0.7, alpha=0.45)
        safe = plt.Circle(point, d_min, fill=False, linestyle="-", color="#70bde8", linewidth=1.0, alpha=0.45)
        ax.add_patch(comm)
        ax.add_patch(safe)
        ax.scatter(point[0], point[1], s=260, color="#7ec8ee", alpha=0.75, edgecolor="none", zorder=4)
        ax.scatter(point[0], point[1], s=18, color="black", zorder=5)
        ax.text(point[0], point[1], str(idx), color="white", ha="center", va="center", fontsize=7, fontweight="bold", zorder=6)


def _draw_density_surface(ax, env: SimulationEnvironment, frame_index: int) -> None:
    xmin, xmax, ymin, ymax = env.domain
    xs = np.linspace(xmin, xmax, 70)
    ys = np.linspace(ymin, ymax, 70)
    xx, yy = np.meshgrid(xs, ys)
    samples = np.column_stack([xx.ravel(), yy.ravel()])
    density = _density_for_frame(env, frame_index)
    zz = density(samples).reshape(xx.shape)
    peak = float(np.max(zz))
    if peak > 1e-9:
        zz = zz / peak
    ax.plot_surface(xx, yy, zz, cmap="jet", linewidth=0, antialiased=True, alpha=0.95)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(0.0, 1.0)
    ax.set_xlabel("x(m)", labelpad=8)
    ax.set_ylabel("y(m)", labelpad=8)
    ax.set_zlabel(r"$\Phi$", labelpad=5)
    ax.view_init(elev=28, azim=-55)


def _density_for_frame(env: SimulationEnvironment, frame_index: int) -> BoundaryAwareDensity:
    params = env.controller.params
    targets = []
    for cargo_id, vertices_history in env.log.cargo_vertices.items():
        vertices = vertices_history[frame_index]
        boundary, normals = Cargo(cargo_id, vertices).boundary_samples(180)
        targets.append(boundary + params.cage_offset * normals)
    if targets:
        return BoundaryAwareDensity.from_targets(np.vstack(targets), sigma=params.sigma)
    if getattr(env.controller, "target_region_points", None) is not None:
        return BoundaryAwareDensity.from_targets(env.controller.target_region_points, sigma=params.sigma)
    return BoundaryAwareDensity.from_targets(np.empty((0, 2)), sigma=params.sigma)


def _agent_positions_at(env: SimulationEnvironment, frame_index: int) -> np.ndarray:
    return np.vstack([history[frame_index] for history in env.log.agent_positions.values()])


def _draw_voronoi(ax, points: np.ndarray, domain: tuple[float, float, float, float]) -> None:
    if len(points) < 2:
        return
    try:
        from scipy.spatial import Voronoi  # type: ignore
    except Exception:
        return
    xmin, xmax, ymin, ymax = domain
    span = max(xmax - xmin, ymax - ymin)
    mirrored = []
    for dx, dy in [
        (0.0, 0.0),
        (-span, 0.0),
        (span, 0.0),
        (0.0, -span),
        (0.0, span),
    ]:
        mirrored.append(points + np.array([dx, dy]))
    all_points = np.vstack(mirrored)
    vor = Voronoi(all_points)
    for v0, v1 in vor.ridge_vertices:
        if v0 < 0 or v1 < 0:
            continue
        p0 = vor.vertices[v0]
        p1 = vor.vertices[v1]
        if _segment_near_domain(p0, p1, domain):
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="0.25", linewidth=0.8, alpha=0.85)


def _segment_near_domain(p0: np.ndarray, p1: np.ndarray, domain: tuple[float, float, float, float]) -> bool:
    xmin, xmax, ymin, ymax = domain
    lo = np.minimum(p0, p1)
    hi = np.maximum(p0, p1)
    return not (hi[0] < xmin or lo[0] > xmax or hi[1] < ymin or lo[1] > ymax)


def _draw_world(ax, env: SimulationEnvironment) -> None:
    xmin, xmax, ymin, ymax = env.domain
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.4)
    for cargo in env.cargoes:
        patch = Polygon(cargo.vertices, closed=True, alpha=0.35)
        ax.add_patch(patch)
        c = cargo.center
        ax.text(c[0], c[1], cargo.object_id, ha="center", va="center", fontsize=8)
        ax.arrow(c[0], c[1], 0.45*cargo.transport_direction[0], 0.45*cargo.transport_direction[1], width=0.02, length_includes_head=True)
    if env.agents:
        pts = np.vstack([a.position for a in env.agents])
        ax.scatter(pts[:, 0], pts[:, 1], s=25, marker="o")
        for a in env.agents:
            ax.text(a.position[0], a.position[1] + 0.08, a.agent_id.split("_")[-1], fontsize=6, ha="center")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
