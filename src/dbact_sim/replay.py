"""D6 - offline rendering from a recorded run.

Simulation and rendering are separated, and the separation is the point. Drawing
inside the loop charges figure export to the simulation clock, so the "frame
rate" of a run becomes a statement about matplotlib; and a figure that needs
revising costs a re-run of the physics. Here the run writes ``replay.npz`` and
this module turns it into pictures, as many times as the figures need changing.

What the animation shows, and does not
--------------------------------------
The boundary points drawn are **one robot's own map**, not the true outline. A
density surface reconstructed from the simulator's polygon looks better and
answers none of the questions worth asking about it -- "how does the team know
where the concavity is" in particular. The true outline is drawn too, as the
thing being estimated, so the gap between the two is visible rather than hidden.

Everything else on the frame is a quantity a gate is written against: the phase,
the frame index against the 500-frame budget, ``J`` against the sampled target
distance, the contact count, the minimum inter-robot separation against
``d_min``, the penetration against its budget. A viewer who disagrees with the
verdict can read the number that produced it.

Cost is kept off the per-frame path: the cargo outline is one transformed
polygon, the robots are a single scatter whose offsets are reassigned, and
nothing is recomputed that does not change.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

PHASE_NAMES = ("SEARCH", "DISCOVER", "ENCLOSE", "CONTACT_READY", "TRANSPORT", "BRAKE", "HOLD")
PHASE_COLORS = ("#8a8a8a", "#5b8dd6", "#3aa07a", "#d6a13a", "#d1603a", "#a24bc4", "#2f6fb5")


@dataclass
class Replay:
    """A finished run, loaded from ``replay.npz``."""

    data: dict
    cargo_id: str

    @classmethod
    def load(cls, path: str | Path) -> "Replay":
        with np.load(Path(path), allow_pickle=False) as handle:
            data = {key: handle[key] for key in handle.files}
        ids = [key.split("/")[1] for key in data if key.startswith("cargo/")]
        if not ids:
            raise ValueError(f"{path} contains no cargo track")
        return cls(data=data, cargo_id=ids[0])

    # ------------------------------------------------------------------ #

    @property
    def frames(self) -> int:
        return len(self.data["times"])

    def cargo(self, field: str) -> np.ndarray:
        return self.data[f"cargo/{self.cargo_id}/{field}"]

    def task(self, field: str) -> np.ndarray | None:
        return self.data.get(f"task/{self.cargo_id}/{field}")

    def cargo_polygon(self, frame: int) -> np.ndarray:
        local = self.cargo("local_vertices")
        angle = float(self.cargo("angles")[frame])
        centre = self.cargo("centers")[frame]
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        return local @ rotation.T + centre[None, :]

    def sensed(self, frame: int) -> np.ndarray:
        counts = self.data["sensed_counts"]
        start = int(np.sum(counts[:frame]))
        return self.data["sensed_points"][start : start + int(counts[frame])]

    def progress(self) -> np.ndarray:
        direction = self.task("direction")
        start = self.task("start")
        if direction is None or start is None:
            return np.zeros(self.frames)
        return (self.cargo("centers") - start[None, :]) @ direction


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #


def _figure(replay: Replay):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    domain = replay.data["domain"]
    centres = replay.cargo("centers")
    task_start = replay.task("start")
    goal_point = replay.task("goal_point")

    # Frame the action rather than the whole workspace: an 8 m room around a 1.8 m
    # object renders the object at a few dozen pixels and the enclosure is not
    # legible at all.
    points = [centres, replay.data["agent_positions"].reshape(-1, 2)]
    if task_start is not None:
        points.append(np.vstack([task_start, goal_point]))
    extent = np.vstack(points)
    pad = 0.6
    lo = np.maximum(extent.min(axis=0) - pad, [domain[0], domain[2]])
    hi = np.minimum(extent.max(axis=0) + pad, [domain[1], domain[3]])
    span = max(hi[0] - lo[0], hi[1] - lo[1])
    mid = 0.5 * (lo + hi)

    figure, axes = plt.subplots(figsize=(7.2, 7.2), dpi=110)
    axes.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    axes.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    axes.set_aspect("equal")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.grid(alpha=0.15, linewidth=0.5)

    artists: dict = {}
    if task_start is not None:
        axes.annotate(
            "",
            xy=tuple(goal_point),
            xytext=tuple(task_start),
            arrowprops=dict(arrowstyle="-|>", color="#c02040", lw=2.0, alpha=0.75),
        )
        axes.plot(*goal_point, marker="x", color="#c02040", markersize=11, markeredgewidth=2.2, zorder=5)
        axes.plot(*task_start, marker="o", color="#c02040", markersize=4, alpha=0.6, zorder=5)

    artists["trail"] = axes.plot([], [], color="#c02040", lw=1.4, alpha=0.85, zorder=4)[0]
    artists["cargo"] = Polygon(
        replay.cargo_polygon(0), closed=True, facecolor="#d8dde6", edgecolor="#31415c", lw=1.8, zorder=2
    )
    axes.add_patch(artists["cargo"])
    artists["sensed"] = axes.scatter([], [], s=5, c="#3aa07a", alpha=0.65, zorder=3, label="robot 0 boundary map")
    artists["robots"] = axes.scatter([], [], s=64, zorder=6, edgecolors="#1d2733", linewidths=0.8)
    artists["discs"] = [
        Circle((0, 0), float(replay.data["robot_radius"]), fill=False, color="#1d2733", alpha=0.25, lw=0.7, zorder=5)
        for _ in range(replay.data["agent_positions"].shape[1])
    ]
    for disc in artists["discs"]:
        axes.add_patch(disc)

    artists["title"] = axes.set_title("", fontsize=11, loc="left")
    artists["readout"] = axes.text(
        0.015,
        0.015,
        "",
        transform=axes.transAxes,
        fontsize=8.5,
        family="monospace",
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.42", facecolor="white", edgecolor="#c8ccd4", alpha=0.92),
        zorder=10,
    )
    artists["bar_back"] = axes.barh(0, 0, height=0, left=0)  # placeholder, unused
    return figure, axes, artists, Circle, Polygon


def _update(replay: Replay, artists: dict, frame: int, d_min: float, penetration_budget: float) -> None:
    positions = replay.data["agent_positions"][frame]
    phase = int(replay.data["phase"][frame])
    pushing = replay.data["push_flags"][frame]
    contacting = replay.data["contact_flags"][frame]

    artists["cargo"].set_xy(replay.cargo_polygon(frame))
    trail = replay.cargo("centers")[: frame + 1]
    artists["trail"].set_data(trail[:, 0], trail[:, 1])

    sensed = replay.sensed(frame)
    artists["sensed"].set_offsets(sensed if len(sensed) else np.empty((0, 2)))

    colours = np.where(pushing, "#d1603a", np.where(contacting, "#d6a13a", "#5b8dd6"))
    artists["robots"].set_offsets(positions)
    artists["robots"].set_color(list(colours))
    for disc, point in zip(artists["discs"], positions):
        disc.center = tuple(point)

    progress = replay.progress()[frame]
    target = replay.task("distance")
    target = float(target) if target is not None else float("nan")
    artists["title"].set_text(
        f"frame {frame:3d}/{replay.frames - 1}    {PHASE_NAMES[phase]}"
    )
    artists["title"].set_color(PHASE_COLORS[phase])

    artists["readout"].set_text(
        "\n".join(
            [
                f"J / L        {progress:6.3f} / {target:5.3f} m",
                f"map coverage {replay.data['map_coverage'][frame]:6.3f}",
                f"enclosure    {replay.cargo('strict_coverage')[frame]:6.3f} strict",
                f"contacts     {int(replay.cargo('contacts')[frame]):3d}   pushing {int(np.sum(pushing)):2d}",
                f"min d_ij     {replay.data['min_distance'][frame]:6.3f} m  (>= {d_min:.3f})",
                f"penetration  {replay.cargo('penetration')[frame]:6.4f} m  (<= {penetration_budget:.4f})",
                f"cargo speed  {replay.cargo('speed')[frame]:6.4f} m/s",
            ]
        )
    )


def render_animation(
    replay: Replay,
    path: str | Path,
    stride: int = 4,
    fps: int = 20,
    d_min: float = 0.34,
    penetration_budget: float = 0.07,
) -> Path:
    """Write a GIF of the whole run. Nothing here touches the physics."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    figure, _, artists, _, _ = _figure(replay)
    frames = list(range(0, replay.frames, max(1, stride)))
    if frames[-1] != replay.frames - 1:
        frames.append(replay.frames - 1)

    def draw(frame: int):
        _update(replay, artists, frame, d_min, penetration_budget)
        return ()

    anim = animation.FuncAnimation(figure, draw, frames=frames, interval=1000 // max(fps, 1), blit=False)
    path = Path(path)
    anim.save(path, writer=animation.PillowWriter(fps=fps))
    plt.close(figure)
    return path


def render_frames(
    replay: Replay,
    directory: str | Path,
    frames: list[int] | None = None,
    d_min: float = 0.34,
    penetration_budget: float = 0.07,
) -> list[Path]:
    """Still frames for the phases a reader wants to inspect one at a time."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if frames is None:
        frames = sorted({0} | {int(f) for f in np.linspace(0, replay.frames - 1, 6)})

    written: list[Path] = []
    figure, _, artists, _, _ = _figure(replay)
    for frame in frames:
        _update(replay, artists, int(frame), d_min, penetration_budget)
        out = directory / f"frame_{int(frame):04d}.png"
        figure.savefig(out, bbox_inches="tight")
        written.append(out)
    plt.close(figure)
    return written


def render_summary_plot(replay: Replay, path: str | Path, d_min: float = 0.34) -> Path:
    """Four time series that carry the verdict: progress, phase, safety, contact."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 1, figsize=(8.0, 8.4), sharex=True, dpi=110)
    steps = np.arange(replay.frames)
    progress = replay.progress()
    target = replay.task("distance")

    axes[0].plot(steps, progress, color="#c02040", lw=1.6, label="J (truth)")
    estimate = replay.data.get(f"cargo/{replay.cargo_id}/progress_estimate")
    if estimate is not None:
        axes[0].plot(steps, estimate, color="#3aa07a", lw=1.3, ls="--", label="J estimated on board")
    if target is not None:
        axes[0].axhline(float(target), color="#31415c", ls=":", lw=1.2, label="target L")
    axes[0].set_ylabel("progress [m]")
    axes[0].legend(fontsize=8, loc="upper left")

    phase = replay.data["phase"]
    axes[1].step(steps, phase, where="post", color="#31415c", lw=1.4)
    axes[1].set_yticks(range(len(PHASE_NAMES)))
    axes[1].set_yticklabels(PHASE_NAMES, fontsize=7)
    axes[1].set_ylabel("phase")

    axes[2].plot(steps, replay.data["min_distance"], color="#5b8dd6", lw=1.3, label="min inter-robot")
    axes[2].axhline(d_min, color="#c02040", ls=":", lw=1.2, label="d_min")
    axes[2].set_ylabel("separation [m]")
    axes[2].legend(fontsize=8, loc="lower left")

    axes[3].plot(steps, replay.cargo("strict_coverage"), color="#3aa07a", lw=1.3, label="strict coverage")
    axes[3].plot(
        steps, replay.cargo("contacts") / max(replay.data["agent_positions"].shape[1], 1),
        color="#d1603a", lw=1.1, label="contacts / N",
    )
    axes[3].set_ylabel("enclosure")
    axes[3].set_xlabel("frame")
    axes[3].legend(fontsize=8, loc="lower right")

    for axis in axes:
        axis.grid(alpha=0.15, linewidth=0.5)
    figure.tight_layout()
    path = Path(path)
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)
    return path


__all__ = ["Replay", "render_animation", "render_frames", "render_summary_plot", "PHASE_NAMES"]
