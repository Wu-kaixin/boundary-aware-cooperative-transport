from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

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
