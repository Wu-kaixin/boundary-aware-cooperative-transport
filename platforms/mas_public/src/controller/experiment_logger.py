from __future__ import annotations

import shutil
from pathlib import Path

import yaml

from src.common.config_loader import CONFIG_DIR, get_project_root
from src.common.time_utils import timestamp_for_dir


class ExperimentLogger:
    """Create an experiment directory and save a complete config snapshot."""

    def __init__(self, experiment_name: str, output_dir: str):
        root = get_project_root()
        self.base_dir = Path(output_dir)
        if not self.base_dir.is_absolute():
            self.base_dir = root / self.base_dir
        safe_name = experiment_name.replace(" ", "_")
        self.experiment_dir = self.base_dir / f"{timestamp_for_dir()}_{safe_name}"

    def create(self) -> Path:
        self.experiment_dir.mkdir(parents=True, exist_ok=True)
        snapshot_dir = self.experiment_dir / "config_snapshot"
        snapshot_dir.mkdir(exist_ok=True)
        for config_path in CONFIG_DIR.rglob("*.yaml"):
            if config_path.parent.name == "controllers":
                continue
            relative_path = config_path.relative_to(CONFIG_DIR)
            target_path = snapshot_dir / relative_path
            target_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(config_path, target_path)
        controller_type = self._controller_type()
        controller_config_path = CONFIG_DIR / "controllers" / f"{controller_type}.yaml"
        target_path = snapshot_dir / "controllers" / controller_config_path.name
        target_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(controller_config_path, target_path)
        return self.experiment_dir

    def _controller_type(self) -> str:
        controller_yaml = CONFIG_DIR / "controller.yaml"
        with controller_yaml.open(encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        controller = config.get("controller", {}) if isinstance(config, dict) else {}
        controller_type = controller.get("type")
        if not isinstance(controller_type, str) or not controller_type:
            raise ValueError("controller.type must be set before creating experiment snapshot")
        return controller_type
