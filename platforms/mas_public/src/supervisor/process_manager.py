from __future__ import annotations

import os
import signal
import subprocess
import sys
from pathlib import Path


class ProcessManager:
    """管理子进程生命周期；不包含任何模块业务逻辑。 / Manage child processes without module business logic."""

    def __init__(self, root: Path):
        self.root = root
        self.processes: dict[str, subprocess.Popen] = {}

    def start(self, name: str, script_path: str) -> subprocess.Popen:
        if name in self.processes and self.is_running(name):
            return self.processes[name]
        command = [sys.executable, script_path]
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        process = subprocess.Popen(command, cwd=self.root, creationflags=creationflags)
        self.processes[name] = process
        return process

    def is_running(self, name: str) -> bool:
        process = self.processes.get(name)
        return bool(process is not None and process.poll() is None)

    def returncode(self, name: str) -> int | None:
        process = self.processes.get(name)
        if process is None:
            return None
        return process.poll()

    def statuses(self) -> dict[str, str]:
        statuses: dict[str, str] = {}
        for name, process in self.processes.items():
            returncode = process.poll()
            statuses[name] = "running" if returncode is None else f"exited({returncode})"
        return statuses

    def stop(self, name: str, timeout_s: float) -> int | None:
        process = self.processes.get(name)
        if process is None:
            return None
        if process.poll() is not None:
            return process.returncode

        self._request_graceful_stop(process)
        try:
            return process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait(timeout=timeout_s)

    def stop_in_order(self, names: list[str], timeout_s: float) -> dict[str, int | None]:
        results: dict[str, int | None] = {}
        for name in names:
            results[name] = self.stop(name, timeout_s)
        return results

    @staticmethod
    def _request_graceful_stop(process: subprocess.Popen) -> None:
        if os.name == "nt":
            try:
                process.send_signal(signal.CTRL_BREAK_EVENT)
                return
            except Exception:
                process.terminate()
                return
        process.terminate()
