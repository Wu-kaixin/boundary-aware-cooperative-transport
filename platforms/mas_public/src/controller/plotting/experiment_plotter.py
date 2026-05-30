from __future__ import annotations

from pathlib import Path

import pandas as pd

from src.common.config_loader import load_config, load_yaml
from src.controller.plotting.common_plots import CommonPlotter
from src.controller.plotting.cvt_plots import CVTPlotter


class ExperimentPlotter:
    """Offline experiment plot coordinator."""

    def __init__(self, experiment_dir: str | Path):
        self.experiment_dir = Path(experiment_dir)
        self.plots_dir = self.experiment_dir / "plots"
        self.plots_dir.mkdir(parents=True, exist_ok=True)
        self.controller_config = self._load_experiment_config("controller.yaml")
        self.system_config = self._load_experiment_config("system.yaml")
        self.plot_config = self.controller_config.get("plot", {})
        self.controller_type = self.controller_config.get("controller", {}).get("type", "")
        self.world_source: Path | None = None
        self.command_source: Path | None = None

    def plot_all(self) -> list[Path]:
        outputs: list[Path] = []
        world_csv = self._world_csv_for_plotting()
        command_csv = self.experiment_dir / "control_command.csv"
        self.world_source = world_csv if world_csv.exists() else None
        self.command_source = command_csv if command_csv.exists() else None
        common_plotter = CommonPlotter(self.plots_dir, self.plot_config)

        if world_csv.exists():
            world = pd.read_csv(world_csv)
            outputs.extend(common_plotter.plot_world(world))
            outputs.extend(self._plot_controller_specific(world))

        if command_csv.exists():
            commands = pd.read_csv(command_csv)
            outputs.extend(common_plotter.plot_commands(commands))

        return outputs

    def _world_csv_for_plotting(self) -> Path:
        if self._uses_coordinate_transform():
            zup_csv = self.experiment_dir / "world_state_zup.csv"
            if zup_csv.exists():
                return zup_csv
        return self.experiment_dir / "world_state.csv"

    def _plot_controller_specific(self, world: pd.DataFrame) -> list[Path]:
        if self.controller_type == "cvt":
            controller_params = self._load_controller_params("cvt")
            return CVTPlotter(
                self.plots_dir,
                self.plot_config,
                self.system_config,
                controller_params,
            ).plot_all(world)
        return []

    def _uses_coordinate_transform(self) -> bool:
        transform_config = self.system_config.get("z_up_transform", {})
        return bool(transform_config.get("enabled", False))

    def _load_experiment_config(self, name: str) -> dict:
        snapshot_path = self.experiment_dir / "config_snapshot" / name
        if snapshot_path.exists():
            return load_yaml(snapshot_path)
        return load_config(name)

    def _load_controller_params(self, controller_type: str) -> dict:
        snapshot_path = self.experiment_dir / "config_snapshot" / "controllers" / f"{controller_type}.yaml"
        if snapshot_path.exists():
            return load_yaml(snapshot_path)
        params_by_type = self.controller_config.get("controller_params", {})
        if isinstance(params_by_type, dict) and controller_type in params_by_type:
            params = params_by_type[controller_type]
            return params if isinstance(params, dict) else {}
        config_path = f"controllers/{controller_type}.yaml"
        try:
            return load_config(config_path)
        except Exception:
            return {}
