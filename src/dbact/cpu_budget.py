"""CPU/memory budget for a priori experiments.

Detects hardware, process affinity, BLAS threading, and already-running workers.
Does not assume that setting ``--workers N`` equals full-core utilisation.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


def _logical_cpus() -> int:
    n = os.cpu_count()
    return int(n) if n else 1


def _affinity_cpus() -> list[int] | None:
    getter = getattr(os, "sched_getaffinity", None)
    if getter is not None:
        try:
            return sorted(int(x) for x in getter(0))
        except OSError:
            return None
    if os.name == "nt":
        try:
            import ctypes

            kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
            mask = ctypes.c_uint64()
            sysmask = ctypes.c_uint64()
            handle = kernel32.GetCurrentProcess()
            if kernel32.GetProcessAffinityMask(handle, ctypes.byref(mask), ctypes.byref(sysmask)):
                bits = int(mask.value)
                return [i for i in range(64) if bits & (1 << i)]
        except Exception:
            return None
    return None


def _physical_cpus() -> int | None:
    if os.name == "nt":
        try:
            import subprocess

            out = subprocess.check_output(
                [
                    "powershell",
                    "-NoProfile",
                    "-Command",
                    "(Get-CimInstance Win32_Processor | Measure-Object NumberOfCores -Sum).Sum",
                ],
                text=True,
                timeout=10,
            ).strip()
            n = int(float(out))
            return n if n > 0 else None
        except Exception:
            return None
    try:
        path = Path("/proc/cpuinfo")
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="ignore")
            cores = [ln for ln in text.splitlines() if ln.lower().startswith("cpu cores")]
            if cores:
                return int(cores[0].split(":")[1].strip())
    except Exception:
        return None
    return None


def _memory_bytes() -> int | None:
    try:
        import psutil  # type: ignore

        return int(psutil.virtual_memory().total)
    except Exception:
        pass
    if os.name == "nt":
        try:
            import ctypes

            class _MEM(ctypes.Structure):
                _fields_ = [
                    ("dwLength", ctypes.c_ulong),
                    ("dwMemoryLoad", ctypes.c_ulong),
                    ("ullTotalPhys", ctypes.c_ulonglong),
                    ("ullAvailPhys", ctypes.c_ulonglong),
                    ("ullTotalPageFile", ctypes.c_ulonglong),
                    ("ullAvailPageFile", ctypes.c_ulonglong),
                    ("ullTotalVirtual", ctypes.c_ulonglong),
                    ("ullAvailVirtual", ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
                ]

            stat = _MEM()
            stat.dwLength = ctypes.sizeof(_MEM)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(stat))  # type: ignore[attr-defined]
            return int(stat.ullTotalPhys)
        except Exception:
            return None
    try:
        line = Path("/proc/meminfo").read_text(encoding="utf-8").splitlines()[0]
        return int(line.split()[1]) * 1024
    except Exception:
        return None


def _numpy_backend() -> dict[str, Any]:
    info: dict[str, Any] = {"numpy": None, "scipy": None, "blas": None}
    try:
        import numpy as np

        info["numpy"] = np.__version__
        try:
            cfg = np.show_config(mode="dicts")  # type: ignore[call-arg]
            info["blas"] = {
                "name": (cfg.get("Build Dependencies") or {}).get("blas", {}).get("name"),
                "version": (cfg.get("Build Dependencies") or {}).get("blas", {}).get("version"),
            }
        except TypeError:
            info["blas"] = {"note": "numpy.show_config(mode=dicts) unavailable"}
    except Exception as exc:
        info["numpy_error"] = str(exc)
    try:
        import scipy

        info["scipy"] = scipy.__version__
    except Exception:
        pass
    return info


def existing_worker_pids(marker: str = "run_apriori") -> list[int]:
    pids: list[int] = []
    try:
        import psutil  # type: ignore

        me = os.getpid()
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            if proc.info["pid"] == me:
                continue
            cmd = " ".join(proc.info.get("cmdline") or [])
            if marker in cmd and "python" in (proc.info.get("name") or "").lower():
                pids.append(int(proc.info["pid"]))
    except Exception:
        pass
    return pids


@dataclass
class CpuEnvironment:
    python: str
    executable: str
    physical_cpus: int | None
    logical_cpus: int
    affinity: list[int] | None
    usable_cpus: int
    memory_bytes: int | None
    numpy_backend: dict[str, Any]
    existing_workers: list[int]
    platform: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def detect_cpu_environment(worker_marker: str = "run_apriori") -> CpuEnvironment:
    logical = _logical_cpus()
    affinity = _affinity_cpus()
    usable = len(affinity) if affinity else logical
    return CpuEnvironment(
        python=sys.version.split()[0],
        executable=sys.executable,
        physical_cpus=_physical_cpus(),
        logical_cpus=logical,
        affinity=affinity,
        usable_cpus=max(1, usable),
        memory_bytes=_memory_bytes(),
        numpy_backend=_numpy_backend(),
        existing_workers=existing_worker_pids(worker_marker),
        platform=sys.platform,
    )


def limit_blas_threads(n_threads: int = 1) -> None:
    n = str(max(1, int(n_threads)))
    for key in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "BLIS_NUM_THREADS",
    ):
        os.environ[key] = n
    try:
        import threadpoolctl

        threadpoolctl.threadpool_limits(int(n))
    except Exception:
        pass


def choose_outer_workers(
    n_tasks: int,
    env: CpuEnvironment,
    *,
    memory_per_worker_gb: float = 1.5,
    leave_one_free: bool = False,
) -> dict[str, Any]:
    """Default: use every usable logical CPU, one process per independent task slot."""
    usable = int(env.usable_cpus)
    if leave_one_free and usable > 2:
        usable -= 1
    mem = env.memory_bytes
    mem_cap = usable
    if mem is not None and memory_per_worker_gb > 0:
        gb = mem / (1024**3)
        mem_cap = max(1, int(gb / float(memory_per_worker_gb)))
    workers = max(1, min(int(n_tasks), usable, mem_cap))
    leftover = max(1, int(math.ceil(usable / max(workers, 1))))
    leftover = min(leftover, 16)
    reason = "full_logical_cpus"
    if workers < usable and n_tasks < usable:
        reason = "task_count_limits_outer_pool_inner_cvt_uses_remainder"
    if mem is not None and mem_cap < min(n_tasks, usable):
        reason = f"memory_cap_{memory_per_worker_gb}GB_per_worker"
    if env.existing_workers:
        reason = "existing_workers_detected_" + ",".join(str(p) for p in env.existing_workers)
    return {
        "outer_workers": workers,
        "inner_cvt_workers": leftover,
        "usable_cpus": usable,
        "n_tasks": int(n_tasks),
        "reason": reason,
        "blas_threads_per_worker": 1,
    }


class ResourceSampler:
    def __init__(self, path: Path, interval_s: float = 2.0):
        self.path = Path(path)
        self.interval_s = float(interval_s)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.samples: list[dict[str, Any]] = []

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, name="resource_sampler", daemon=True)
        self._thread.start()

    def _snapshot(self) -> dict[str, Any]:
        row: dict[str, Any] = {"t": time.time(), "pid": os.getpid()}
        try:
            import psutil  # type: ignore

            proc = psutil.Process()
            row["cpu_percent"] = psutil.cpu_percent(interval=None)
            row["per_cpu"] = psutil.cpu_percent(interval=None, percpu=True)
            row["rss"] = proc.memory_info().rss
            row["vms"] = proc.memory_info().vms
            row["num_threads"] = proc.num_threads()
            row["children"] = len(proc.children(recursive=True))
        except Exception:
            row["cpu_percent"] = None
        return row

    def _run(self) -> None:
        while not self._stop.is_set():
            self.samples.append(self._snapshot())
            self._stop.wait(self.interval_s)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.interval_s + 1.0)
        if self.samples:
            self.path.write_text(
                "t,pid,cpu_percent,rss,children\n"
                + "\n".join(
                    f"{s.get('t')},{s.get('pid')},{s.get('cpu_percent')},{s.get('rss')},{s.get('children')}"
                    for s in self.samples
                ),
                encoding="utf-8",
            )


def dump_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, default=str), encoding="utf-8")
