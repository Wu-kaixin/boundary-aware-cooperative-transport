from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path

from src.common.config_loader import get_project_root, load_all_configs
from src.common.logger import setup_logging
from src.common.messages import ModuleStatus
from src.common.time_utils import now_s
from src.messaging.factory import TransportFactory
from src.messaging.topics import CONTROL_COMMAND, MODULE_STATUS, WORLD_STATE
from src.supervisor.process_manager import ProcessManager


@dataclass(frozen=True)
class ModuleSpec:
    name: str
    script_path: str
    status_channel: str
    ready_timeout_key: str
    ready_data_topic: str | None = None


class Supervisor:
    """Internal module."""

    START_ORDER = [
        ModuleSpec(
            "optitrack",
            "apps/run_optitrack.py",
            "world_state",
            "optitrack_ready_timeout_s",
            WORLD_STATE,
        ),
        ModuleSpec("robot", "apps/run_robot_comm.py", "module_status", "robot_ready_timeout_s"),
        ModuleSpec(
            "controller",
            "apps/run_controller.py",
            "control_command",
            "controller_ready_timeout_s",
            CONTROL_COMMAND,
        ),
    ]
    SHUTDOWN_ORDER = ["controller", "robot", "optitrack"]
    TASK_COMPLETED_SHUTDOWN_ORDER = ["robot", "optitrack"]
    CACHED_STARTUP_MODULES = {"optitrack", "robot"}
    FINAL_STATUSES = {"completed", "failed", "stopped", "error"}

    def __init__(self):
        self.root = get_project_root()
        self.configs = load_all_configs()
        self.system_config = self.configs["system"]
        self.controller_config = self.configs["controller"]
        self.supervisor_config = self.configs["supervisor"]
        self.logger = setup_logging("supervisor")
        self.manager = ProcessManager(self.root)
        self.transport = TransportFactory(self.system_config["network"], self.logger)
        self.status_subscribers = {
            "world_state": self.transport.create_subscriber("world_state", [MODULE_STATUS, WORLD_STATE]),
            "module_status": self.transport.create_subscriber("module_status", [MODULE_STATUS]),
            "control_command": self.transport.create_subscriber("control_command", [MODULE_STATUS, CONTROL_COMMAND]),
        }
        self._shutdown_done = False
        self._task_completed = False
        self._controller_started = False
        self._pending_startup_statuses: list[ModuleStatus] = []
        self._pending_shutdown_statuses: list[ModuleStatus] = []

    def run(self) -> None:
        try:
            self._start_enabled_modules()
            self.logger.info("Supervisor startup complete: %s", self.manager.statuses())
            while True:
                if self._auto_stop_requested():
                    self.logger.info("Controller finished; auto-stopping experiment")
                    self._task_completed = True
                    break
                self._check_running_processes()
                time.sleep(1.0)
        except KeyboardInterrupt:
            self.logger.info("Supervisor received Ctrl+C, shutting down")
        except Exception as exc:
            self.logger.exception("Supervisor startup/runtime error: %s", exc)
        finally:
            self.shutdown()

    def _start_enabled_modules(self) -> None:
        for spec in self.START_ORDER:
            if not self._module_enabled(spec.name):
                self.logger.info("Skipping disabled module: %s", spec.name)
                continue
            self.logger.info("Starting %s via %s", spec.name, spec.script_path)
            self.manager.start(spec.name, spec.script_path)
            if spec.name == "controller":
                self._controller_started = True
            ready = self._wait_until_ready(spec)
            if not ready:
                self.logger.error("%s failed to become ready; stopping started modules", spec.name)
                self.shutdown()
                raise RuntimeError(f"{spec.name} startup failed or timed out")
            self.logger.info("%s is ready", spec.name)

    def _wait_until_ready(self, spec: ModuleSpec) -> bool:
        timeout_s = float(self.supervisor_config[spec.ready_timeout_key])
        deadline = time.monotonic() + timeout_s
        subscriber = self.status_subscribers[spec.status_channel]
        while time.monotonic() < deadline:
            if not self.manager.is_running(spec.name):
                self.logger.error("%s exited early with returncode=%s", spec.name, self.manager.returncode(spec.name))
                return False
            received = subscriber.receive(timeout_ms=100)
            if received is None:
                continue
            topic, payload = received
            if spec.ready_data_topic is not None and topic == spec.ready_data_topic:
                self.logger.info("%s ready via first %s message", spec.name, topic)
                self._cache_startup_status(
                    ModuleStatus(spec.name, "running", f"{spec.name} ready via first {topic} message", now_s())
                )
                return True
            if topic != MODULE_STATUS:
                continue
            status = ModuleStatus.from_dict(payload)
            self._cache_startup_status(status)
            if status.module_name == spec.name and status.status in {"ready", "running"}:
                return True
            if status.module_name == spec.name and status.status == "error":
                self.logger.error("%s reported error status: %s", spec.name, status.message)
                return False
        self.logger.error("%s ready timeout after %.1fs", spec.name, timeout_s)
        return False

    def _cache_startup_status(self, status: ModuleStatus) -> None:
        if status.module_name in self.CACHED_STARTUP_MODULES:
            self._pending_startup_statuses.append(status)

    def _check_running_processes(self) -> None:
        for spec in self.START_ORDER:
            if not self._module_enabled(spec.name):
                continue
            if not self.manager.is_running(spec.name):
                raise RuntimeError(f"{spec.name} exited with returncode={self.manager.returncode(spec.name)}")

    def _auto_stop_requested(self) -> bool:
        experiment_config = self.system_config.get("experiment", {})
        subscriber = self.status_subscribers["control_command"]
        while True:
            received = subscriber.receive(timeout_ms=0)
            if received is None:
                return False
            topic, payload = received
            if topic != MODULE_STATUS:
                continue
            status = ModuleStatus.from_dict(payload)
            if status.module_name != "controller":
                continue
            if status.status == "completed" and experiment_config.get("auto_stop_on_task_completed", False):
                return True
            if status.status == "failed" and experiment_config.get("auto_stop_on_untracked", False):
                return True

    def shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        timeout_s = float(self.supervisor_config["shutdown_timeout_s"])
        base_order = self.TASK_COMPLETED_SHUTDOWN_ORDER if self._task_completed else self.SHUTDOWN_ORDER
        shutdown_order = self._enabled_started_modules(base_order)
        self.logger.info("Supervisor shutdown order: %s", shutdown_order)
        results = self.manager.stop_in_order(shutdown_order, timeout_s)
        for name, returncode in results.items():
            self.logger.info("Module %s stopped with returncode=%s", name, returncode)
            self._pending_shutdown_statuses.append(self._shutdown_status(name, returncode))
        self._record_supervisor_statuses()
        for subscriber in self.status_subscribers.values():
            subscriber.close()

    def _enabled_started_modules(self, names: list[str]) -> list[str]:
        started = getattr(self.manager, "processes", {})
        return [name for name in names if self._module_enabled(name) and name in started]

    def _module_enabled(self, name: str) -> bool:
        key = f"use_{name}"
        return bool(self.supervisor_config.get(key, True))

    def _shutdown_status(self, name: str, returncode: int | None) -> ModuleStatus:
        status = "stopped" if returncode in {0, None} else "error"
        message = f"{name} module stopped with returncode={returncode}"
        return ModuleStatus(name, status, message, now_s())

    def _record_supervisor_statuses(self) -> None:
        if not self._controller_started:
            return
        experiment_dir = self._latest_experiment_dir()
        if experiment_dir is None:
            return
        statuses = [*self._pending_startup_statuses, *self._pending_shutdown_statuses]
        if not statuses:
            return
        self._merge_module_statuses_chronological(experiment_dir, statuses)

    def _latest_experiment_dir(self) -> Path | None:
        recording = self.controller_config.get("recording", {})
        output_dir = Path(recording.get("output_dir", "data/experiments"))
        if not output_dir.is_absolute():
            output_dir = self.root / output_dir
        if not output_dir.exists():
            return None
        experiment_name = str(self.system_config.get("experiment_name", "experiment"))
        candidates = [
            path
            for path in output_dir.iterdir()
            if path.is_dir() and path.name.endswith(f"_{experiment_name}") and (path / "system_status.csv").exists()
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda path: path.name)

    def _merge_module_statuses_chronological(
        self, experiment_dir: Path, statuses: list[ModuleStatus]
    ) -> None:
        status_path = experiment_dir / "system_status.csv"
        fieldnames = ["timestamp", "module_name", "status", "message"]
        rows: list[dict[str, str]] = []
        if status_path.exists():
            with status_path.open("r", newline="", encoding="utf-8") as file:
                rows.extend(csv.DictReader(file))
        existing_final = {
            (row.get("module_name", ""), row.get("status", ""))
            for row in rows
            if row.get("status") in self.FINAL_STATUSES
        }
        for status in statuses:
            key = (status.module_name, status.status)
            if status.status in self.FINAL_STATUSES and key in existing_final:
                continue
            rows.append(
                {
                    "timestamp": str(status.timestamp),
                    "module_name": status.module_name,
                    "status": status.status,
                    "message": status.message,
                }
            )
            if status.status in self.FINAL_STATUSES:
                existing_final.add(key)
        rows.sort(key=lambda row: float(row.get("timestamp") or 0.0))
        with status_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

