from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

from src.controller.cvt_utils import compute_grid_cvt, world_bounds_xy
from src.controller.plotting.common_plots import save_figure


class CVTPlotter:
    def __init__(
        self,
        plots_dir: Path,
        plot_config: dict,
        system_config: dict,
        controller_params: dict | None = None,
    ):
        self.plots_dir = plots_dir
        self.plot_config = plot_config
        self.system_config = system_config
        self.controller_params = controller_params or {}

    def plot_all(self, world: pd.DataFrame) -> list[Path]:
        outputs: list[Path] = []
        output = self.plot_cvt_voronoi(world)
        if output is not None:
            outputs.append(output)
        outputs.extend(self.plot_cvt_voronoi_sequence(world))
        return outputs

    def plot_cvt_voronoi(self, world: pd.DataFrame) -> Path | None:
        required = {"frame_id", "robot_id", "tracked", "x", "y"}
        if world.empty or not required.issubset(world.columns):
            return None

        last_frame_id = world["frame_id"].max()
        return self._plot_cvt_frame(world, last_frame_id, self.plots_dir / "cvt" / "cvt_voronoi.png")

    def plot_cvt_voronoi_sequence(self, world: pd.DataFrame) -> list[Path]:
        required = {"frame_id", "robot_id", "tracked", "x", "y"}
        if world.empty or not required.issubset(world.columns):
            return []

        step = int(self.plot_config.get("save_every_n_frames", 0))
        if step <= 0:
            return []

        frame_ids = list(pd.Series(world["frame_id"].dropna().unique()).sort_values())
        if not frame_ids:
            return []

        selected_indices = set(range(0, len(frame_ids), step))
        selected_indices.add(0)
        selected_indices.add(len(frame_ids) - 1)

        output_dir = self.plots_dir / "cvt"
        output_dir.mkdir(parents=True, exist_ok=True)

        outputs: list[Path] = []
        for index in sorted(selected_indices):
            frame_id = frame_ids[index]
            output = output_dir / f"cvt_voronoi_step_{index:06d}.png"
            plotted = self._plot_cvt_frame(world, frame_id, output)
            if plotted is not None:
                outputs.append(plotted)
        return outputs

    def _plot_cvt_frame(self, world: pd.DataFrame, frame_id: int | float, output: Path) -> Path | None:
        frame = world[(world["frame_id"] == frame_id) & self._tracked_mask(world["tracked"])]
        if frame.empty:
            return None

        world_config = self.system_config["world"]
        points = frame[["x", "y"]].to_numpy(dtype=float)
        grid_resolution = int(self.controller_params.get("grid_resolution", 120))
        cvt = compute_grid_cvt(points, world_config, grid_resolution=grid_resolution)
        x_min, x_max, y_min, y_max = world_bounds_xy(world_config)

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.pcolormesh(
            cvt.grid_x,
            cvt.grid_y,
            cvt.label_grid,
            shading="auto",
            cmap="tab20",
            alpha=0.25,
        )
        ax.scatter(points[:, 0], points[:, 1], c="black", marker="o", label="robot")
        ax.scatter(cvt.centroids[:, 0], cvt.centroids[:, 1], c="red", marker="x", label="centroid")
        for (_, row), point, centroid in zip(frame.iterrows(), points, cvt.centroids):
            ax.plot([point[0], centroid[0]], [point[1], centroid[1]], "k--", linewidth=1)
            ax.annotate(str(row["robot_id"]), (point[0], point[1]), xytext=(4, 4), textcoords="offset points")
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"CVT Voronoi cells and centroids - frame {int(frame_id)}")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.legend()
        return save_figure(fig, output)

    @staticmethod
    def _tracked_mask(series: pd.Series) -> pd.Series:
        if series.dtype == bool:
            return series
        return series.astype(str).str.lower().isin({"true", "1", "yes"})
