from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


REQUIRED_FILES = [
    "world_state.csv",
    "world_state_zup.csv",
    "control_command.csv",
    "system_status.csv",
    "robot_status.csv",
]
WORLD_COLUMNS = [
    "timestamp",
    "frame_id",
    "robot_id",
    "tracked",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "wz",
]
CONTROL_COMMAND_COLUMNS = [
    "timestamp",
    "robot_mode",
    "robot_id",
    "chassis_vx",
    "chassis_vy",
    "chassis_wz",
    "gimbal_yaw_speed",
    "gimbal_pitch_speed",
    "controller_mode",
]
STATUS_COLUMNS = ["timestamp", "module_name", "status", "message"]
ROBOT_STATUS_COLUMNS = [
    "timestamp",
    "robot_id",
    "status_type",
    "pitch_angle",
    "yaw_angle",
    "pitch_ground_angle",
    "yaw_ground_angle",
    "requested_mode",
    "actual_mode",
]
FINAL_STATUSES = {"completed", "failed", "stopped", "error"}
LEGACY_FIELDS = {
    "coordination_mode": "robot_mode",
    "command_mode": "controller_mode",
    "params_file": "controller.type + configs/controllers/{type}.yaml",
    "transport": "backend",
}


@dataclass
class CheckReport:
    experiment_dir: Path
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    info: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors

    def add_error(self, message: str) -> None:
        self.errors.append(message)

    def add_warning(self, message: str) -> None:
        self.warnings.append(message)


def check_experiment(experiment_dir: str | Path) -> CheckReport:
    exp = Path(experiment_dir)
    report = CheckReport(exp)
    if not exp.exists() or not exp.is_dir():
        report.add_error(f"Experiment directory not found: {exp}")
        return report

    _check_required_files(exp, report)
    csv_data = _check_csv_files(exp, report)
    _check_config_snapshot(exp, report)
    _check_status_closure(csv_data.get("system_status.csv", []), report)
    _check_plots(exp, report)
    _summarize(csv_data, report)
    return report


def print_report(report: CheckReport) -> None:
    status = "OK" if report.ok else "ERROR"
    print(f"experiment: {report.experiment_dir}")
    print(f"status: {status}")
    for key, value in report.info.items():
        print(f"{key}: {value}")
    for warning in report.warnings:
        print(f"WARN: {warning}")
    for error in report.errors:
        print(f"ERROR: {error}")


def _check_required_files(exp: Path, report: CheckReport) -> None:
    for name in REQUIRED_FILES:
        if not (exp / name).exists():
            report.add_error(f"Missing required file: {name}")


def _check_csv_files(exp: Path, report: CheckReport) -> dict[str, list[dict[str, str]]]:
    csv_data: dict[str, list[dict[str, str]]] = {}
    expected_columns = {
        "world_state.csv": WORLD_COLUMNS,
        "world_state_zup.csv": WORLD_COLUMNS,
        "control_command.csv": CONTROL_COMMAND_COLUMNS,
        "system_status.csv": STATUS_COLUMNS,
        "robot_status.csv": ROBOT_STATUS_COLUMNS,
    }
    for name, expected in expected_columns.items():
        path = exp / name
        if not path.exists():
            continue
        rows, columns = _read_csv(path)
        csv_data[name] = rows
        if columns != expected:
            report.add_error(f"{name} columns do not match final format: {columns}")
        _warn_legacy_columns(name, columns, report)

    world_rows = csv_data.get("world_state.csv", [])
    zup_rows = csv_data.get("world_state_zup.csv", [])
    command_rows = csv_data.get("control_command.csv", [])
    if world_rows and zup_rows and len(world_rows) != len(zup_rows):
        report.add_error("world_state.csv and world_state_zup.csv row counts differ")
    control_cycle_command_rows = _control_cycle_command_rows(command_rows)
    if world_rows and control_cycle_command_rows and len(world_rows) != len(control_cycle_command_rows):
        report.add_warning("world_state.csv and control_command.csv row counts differ")
    return csv_data


def _control_cycle_command_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [row for row in rows if row.get("controller_mode") != "shutdown"]


def _read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        return list(reader), list(reader.fieldnames or [])


def _warn_legacy_columns(name: str, columns: list[str], report: CheckReport) -> None:
    for legacy, current in LEGACY_FIELDS.items():
        if legacy in columns:
            report.add_warning(f"{name} uses legacy field {legacy}; final field is {current}")


def _check_config_snapshot(exp: Path, report: CheckReport) -> None:
    snapshot_dir = exp / "config_snapshot"
    if not snapshot_dir.exists():
        report.add_error("Missing config_snapshot directory")
        return

    for name in ["system.yaml", "controller.yaml", "robots.yaml", "optitrack.yaml", "supervisor.yaml", "logging.yaml"]:
        if not (snapshot_dir / name).exists():
            report.add_error(f"Missing config snapshot: {name}")

    controller_yaml = snapshot_dir / "controller.yaml"
    controller_type = None
    if controller_yaml.exists():
        controller_config = _read_yaml(controller_yaml, report)
        controller = controller_config.get("controller", {}) if isinstance(controller_config, dict) else {}
        controller_type = controller.get("type")
        report.info["controller_type"] = controller_type
        report.info["robot_mode"] = controller.get("robot_mode")
        if "coordination_mode" in controller:
            report.add_warning("controller.yaml uses legacy field controller.coordination_mode")
        if "params_file" in controller:
            report.add_warning("controller.yaml uses legacy field controller.params_file")
        if "robot_mode" not in controller:
            report.add_error("controller.yaml missing final field controller.robot_mode")

    system_yaml = snapshot_dir / "system.yaml"
    if system_yaml.exists():
        system_config = _read_yaml(system_yaml, report)
        network = system_config.get("network", {}) if isinstance(system_config, dict) else {}
        if "transport" in network:
            report.add_warning("system.yaml uses legacy field network.transport")
        if "backend" not in network:
            report.add_error("system.yaml missing final field network.backend")

    if controller_type:
        controller_params = snapshot_dir / "controllers" / f"{controller_type}.yaml"
        if not controller_params.exists():
            report.add_error(f"Missing controller parameter snapshot: controllers/{controller_type}.yaml")


def _read_yaml(path: Path, report: CheckReport) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8-sig") as file:
            data = yaml.safe_load(file) or {}
    except Exception as exc:
        report.add_error(f"Failed to read YAML {path.name}: {exc}")
        return {}
    if not isinstance(data, dict):
        report.add_error(f"YAML must be a mapping: {path.name}")
        return {}
    return data


def _check_status_closure(rows: list[dict[str, str]], report: CheckReport) -> None:
    if not rows:
        report.add_error("system_status.csv has no status rows")
        return
    by_module: dict[str, list[str]] = {}
    for row in rows:
        by_module.setdefault(row.get("module_name", ""), []).append(row.get("status", ""))
    report.info["module_final_statuses"] = {
        module: statuses[-1] for module, statuses in by_module.items() if module and statuses
    }
    for module, final_status in report.info["module_final_statuses"].items():
        if final_status not in FINAL_STATUSES:
            report.add_error(f"Module {module} does not end with a final status: {final_status}")
    if "controller" not in by_module:
        report.add_error("system_status.csv missing controller status")


def _check_plots(exp: Path, report: CheckReport) -> None:
    plots_dir = exp / "plots"
    if not plots_dir.exists():
        report.add_warning("Missing plots directory")
        return
    plots = sorted(path.relative_to(exp).as_posix() for path in plots_dir.rglob("*.png"))
    report.info["plot_count"] = len(plots)
    for name in ["plots/trajectory_xy.png", "plots/pose_time_series.png", "plots/command_time_series.png"]:
        if name not in plots:
            report.add_warning(f"Missing plot: {name}")


def _summarize(csv_data: dict[str, list[dict[str, str]]], report: CheckReport) -> None:
    world_rows = csv_data.get("world_state.csv", [])
    command_rows = csv_data.get("control_command.csv", [])
    report.info["world_rows"] = len(world_rows)
    report.info["command_rows"] = len(command_rows)
    report.info["robot_ids"] = sorted({row.get("robot_id", "") for row in world_rows if row.get("robot_id")})


def main() -> None:
    parser = argparse.ArgumentParser(description="Check MAS experiment files against the release format.")
    parser.add_argument("experiment_dir", help="Path to data/experiments/<timestamp>_<name>")
    args = parser.parse_args()
    report = check_experiment(args.experiment_dir)
    print_report(report)
    if not report.ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
