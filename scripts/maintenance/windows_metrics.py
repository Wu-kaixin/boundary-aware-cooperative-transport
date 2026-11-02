"""Read-only Windows telemetry for a benchmark's own process tree.

No psutil installation. PDH disk/CPU counters are host-wide and must not be
presented as exclusive process I/O wait. Missing counters remain explicitly null.
"""
from __future__ import annotations

import ctypes as c
from ctypes import wintypes as w
import os

k = c.WinDLL("kernel32", use_last_error=True)
p = c.WinDLL("psapi", use_last_error=True)
d = c.WinDLL("pdh", use_last_error=True)
k.GetCurrentProcess.restype = w.HANDLE
k.OpenProcess.argtypes = [w.DWORD, w.BOOL, w.DWORD]
k.OpenProcess.restype = w.HANDLE
k.CloseHandle.argtypes = [w.HANDLE]
k.GetProcessAffinityMask.argtypes = [w.HANDLE, c.POINTER(c.c_size_t), c.POINTER(c.c_size_t)]


class Memory(c.Structure):
    _fields_ = [("length", w.DWORD), ("load", w.DWORD)] + [(n, c.c_ulonglong) for n in
                ("total", "available", "page_total", "page_available", "virtual_total", "virtual_available", "extended")]


class ProcessMemory(c.Structure):
    _fields_ = [("cb", w.DWORD), ("page_faults", w.DWORD)] + [(n, c.c_size_t) for n in
                ("peak_rss", "rss", "peak_paged", "paged", "peak_nonpaged", "nonpaged", "commit", "peak_commit")]


class IO(c.Structure):
    _fields_ = [(n, c.c_ulonglong) for n in ("read_ops", "write_ops", "other_ops", "read_bytes", "write_bytes", "other_bytes")]


class Entry(c.Structure):
    _fields_ = [("size", w.DWORD), ("usage", w.DWORD), ("pid", w.DWORD), ("heap", c.c_size_t),
                ("module", w.DWORD), ("threads", w.DWORD), ("parent", w.DWORD),
                ("priority", w.LONG), ("flags", w.DWORD), ("exe", w.WCHAR * 260)]


class CounterValue(c.Structure):
    # PDH_FMT_DOUBLE uses the double member of the native union (8-byte aligned).
    _fields_ = [("status", w.DWORD), ("value", c.c_double)]


k.CreateToolhelp32Snapshot.argtypes = [w.DWORD, w.DWORD]
k.CreateToolhelp32Snapshot.restype = w.HANDLE
k.Process32FirstW.argtypes = [w.HANDLE, c.POINTER(Entry)]
k.Process32NextW.argtypes = [w.HANDLE, c.POINTER(Entry)]
p.GetProcessMemoryInfo.argtypes = [w.HANDLE, c.POINTER(ProcessMemory), w.DWORD]
k.GetProcessIoCounters.argtypes = [w.HANDLE, c.POINTER(IO)]
k.GetProcessTimes.argtypes = [w.HANDLE] + [c.POINTER(w.FILETIME)] * 4
d.PdhOpenQueryW.argtypes = [w.LPCWSTR, c.c_size_t, c.POINTER(w.HANDLE)]
d.PdhAddEnglishCounterW.argtypes = [w.HANDLE, w.LPCWSTR, c.c_size_t, c.POINTER(w.HANDLE)]
d.PdhCollectQueryData.argtypes = [w.HANDLE]
d.PdhGetFormattedCounterValue.argtypes = [w.HANDLE, w.DWORD, c.POINTER(w.DWORD), c.POINTER(CounterValue)]
d.PdhCloseQuery.argtypes = [w.HANDLE]


def memory() -> dict:
    value = Memory(); value.length = c.sizeof(value)
    if not k.GlobalMemoryStatusEx(c.byref(value)):
        raise c.WinError(c.get_last_error())
    return {"total_bytes": value.total, "available_bytes": value.available, "load_percent": value.load}


def affinity() -> list[int]:
    own, system = c.c_size_t(), c.c_size_t()
    if not k.GetProcessAffinityMask(k.GetCurrentProcess(), c.byref(own), c.byref(system)):
        raise c.WinError(c.get_last_error())
    return [i for i in range(64) if own.value & (1 << i)]


def tree(root_pid: int) -> list[int]:
    handle = k.CreateToolhelp32Snapshot(2, 0)
    if handle == w.HANDLE(-1).value:
        raise c.WinError(c.get_last_error())
    parents = {}
    try:
        entry = Entry(); entry.size = c.sizeof(entry)
        success = k.Process32FirstW(handle, c.byref(entry))
        while success:
            parents[int(entry.pid)] = int(entry.parent)
            success = k.Process32NextW(handle, c.byref(entry))
    finally:
        k.CloseHandle(handle)
    selected = {root_pid}
    while True:
        more = {pid for pid, parent in parents.items() if parent in selected}
        if more <= selected:
            return sorted(selected)
        selected |= more


def process(pid: int) -> dict | None:
    handle = k.OpenProcess(0x0400 | 0x0010, False, pid)
    if not handle:
        return None
    try:
        mem = ProcessMemory(); mem.cb = c.sizeof(mem)
        if not p.GetProcessMemoryInfo(handle, c.byref(mem), c.sizeof(mem)):
            return None
        times = [w.FILETIME() for _ in range(4)]
        valid_time = k.GetProcessTimes(handle, *(c.byref(t) for t in times))
        seconds = sum((t.dwHighDateTime << 32) + t.dwLowDateTime for t in times[2:]) / 1e7 if valid_time else None
        io = IO(); valid_io = k.GetProcessIoCounters(handle, c.byref(io))
        return {"pid": pid, "rss_bytes": mem.rss, "lifetime_peak_rss_bytes": mem.peak_rss,
                "commit_bytes": mem.commit, "cpu_seconds": seconds,
                "read_bytes": io.read_bytes if valid_io else None, "write_bytes": io.write_bytes if valid_io else None}
    finally:
        k.CloseHandle(handle)


class Counters:
    PATHS = {
        "host_cpu_percent": r"\Processor(_Total)\% Processor Time",
        "disk_read_seconds": r"\PhysicalDisk(_Total)\Avg. Disk sec/Read",
        "disk_write_seconds": r"\PhysicalDisk(_Total)\Avg. Disk sec/Write",
        "disk_queue": r"\PhysicalDisk(_Total)\Current Disk Queue Length",
        "disk_read_bytes_sec": r"\PhysicalDisk(_Total)\Disk Read Bytes/sec",
        "disk_write_bytes_sec": r"\PhysicalDisk(_Total)\Disk Write Bytes/sec",
        "cpu_frequency_mhz": r"\Processor Information(_Total)\Processor Frequency",
    }

    def __init__(self):
        self.query = w.HANDLE(); self.handles = {}; self.errors = {}
        error = d.PdhOpenQueryW(None, 0, c.byref(self.query))
        if error:
            self.errors["query"] = error
            return
        for name, path in self.PATHS.items():
            handle = w.HANDLE()
            error = d.PdhAddEnglishCounterW(self.query, path, 0, c.byref(handle))
            if error:
                self.errors[name] = error
            else:
                self.handles[name] = handle
        d.PdhCollectQueryData(self.query)

    def sample(self):
        values = {name: None for name in self.PATHS}
        if not self.query or d.PdhCollectQueryData(self.query):
            return values
        for name, handle in self.handles.items():
            value = CounterValue()
            error = d.PdhGetFormattedCounterValue(handle, 0x200, None, c.byref(value))
            if not error and value.status in (0, 1):
                values[name] = value.value
        return values

    def close(self):
        if self.query:
            d.PdhCloseQuery(self.query)
            self.query = None


if __name__ == "__main__":
    import json, time
    counters = Counters(); time.sleep(1)
    print(json.dumps({"memory": memory(), "affinity": affinity(), "self": process(os.getpid()),
                      "tree": tree(os.getpid()), "host_counters": counters.sample(), "counter_errors": counters.errors}))
    counters.close()
