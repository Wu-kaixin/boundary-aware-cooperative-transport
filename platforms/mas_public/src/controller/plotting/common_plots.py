from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams.update(
    {
        "font.family": "Arial",
        "font.size": 12,
        "axes.titlesize": 14,
        "axes.labelsize": 12,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "legend.fontsize": 12,
    }
)


class CommonPlotter:
    def __init__(self, plots_dir: Path, plot_config: dict):
        self.plots_dir = plots_dir
        self.plot_config = plot_config

    def plot_world(self, world: pd.DataFrame) -> list[Path]:
        outputs: list[Path] = []
        if self.plot_config.get("plot_trajectory", True):
            outputs.append(self.plot_trajectory_xy(world))
        if self.plot_config.get("plot_pose_time_series", True):
            outputs.append(self.plot_pose_time_series(world))
        return outputs

    def plot_commands(self, commands: pd.DataFrame) -> list[Path]:
        if not self.plot_config.get("plot_command_time_series", True):
            return []
        return [self.plot_command_time_series(commands)]

    def plot_trajectory_xy(self, world: pd.DataFrame) -> Path:
        fig, ax = plt.subplots(figsize=(8, 6))
        for robot_id, group in world.groupby("robot_id"):
            ax.plot(group["x"], group["y"], label=robot_id)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Robot trajectories")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        return self._save(fig, "trajectory_xy.png")

    def plot_pose_time_series(self, world: pd.DataFrame) -> Path:
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
        pose_axes = {
            "x": axes[0, 0],
            "y": axes[1, 0],
            "z": axes[2, 0],
            "roll": axes[0, 1],
            "pitch": axes[1, 1],
            "yaw": axes[2, 1],
        }
        for robot_id, group in world.groupby("robot_id"):
            t = group["timestamp"] - group["timestamp"].iloc[0]
            for name, ax in pose_axes.items():
                ax.plot(t, group[name], label=robot_id)
        labels = {
            "x": "x [m]",
            "y": "y [m]",
            "z": "z [m]",
            "roll": "roll [rad]",
            "pitch": "pitch [rad]",
            "yaw": "yaw [rad]",
        }
        for name, ax in pose_axes.items():
            ax.set_ylabel(labels[name])
            ax.grid(True)
        axes[2, 0].set_xlabel("time [s]")
        axes[2, 1].set_xlabel("time [s]")
        axes[0, 0].legend()
        return self._save(fig, "pose_time_series.png")

    def plot_command_time_series(self, commands: pd.DataFrame) -> Path:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for robot_id, group in commands.groupby("robot_id"):
            t = group["timestamp"] - group["timestamp"].iloc[0]
            axes[0].plot(t, group["chassis_vx"], label=robot_id)
            axes[1].plot(t, group["chassis_vy"], label=robot_id)
            axes[2].plot(t, group["chassis_wz"], label=robot_id)
        for ax, label in zip(axes, ["vx [m/s]", "vy [m/s]", "wz [rad/s]"]):
            ax.set_ylabel(label)
            ax.grid(True)
        axes[-1].set_xlabel("time [s]")
        axes[0].legend()
        return self._save(fig, "command_time_series.png")

    def _save(self, fig: plt.Figure, filename: str) -> Path:
        return save_figure(fig, self.plots_dir / filename)


def save_figure(fig: plt.Figure, output: Path) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    for ax in fig.axes:
        ax.tick_params(axis="both", which="both", direction="in")
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output
