"""Bounded DBACT engineering benchmark; repeat frozen tasks, never tune them.

Freeze first, then run. All writes stay in this mission under runs/maintenance or
artifacts/migrations. Reuses study worker/resume and refuses scientific changes.
"""
from __future__ import annotations

import argparse
import cProfile
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MISSION = "2026-11-01-workstation-benchmark-v1"
OUT = ROOT / "runs/maintenance" / MISSION
AUDIT = ROOT / "artifacts/migrations" / MISSION
HISTORICAL = ROOT / "runs/research/2026-10-31-multishape-census-v1/run"
THREAD_NAMES = ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS", "DBACT_CVT_WORKERS")
AMBIENT_THREADS = {name: os.environ.get(name) for name in THREAD_NAMES}
for name in THREAD_NAMES:
    os.environ[name] = "1"
os.environ["MPLBACKEND"] = "Agg"
os.environ["PYTHONUTF8"] = "1"
os.environ["PYTHONIOENCODING"] = "utf-8"

import numpy as np
from dbact_sim.experiments import study
from dbact_sim.experiments.study import worker as original_worker
from maintenance import windows_metrics as metrics


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save(path, value):
    path = Path(path)
    if not any(path.resolve().is_relative_to(root.resolve()) for root in (OUT, AUDIT)):
        raise ValueError("Write outside benchmark mission")
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
    temp.replace(path)


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def own_sources():
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in
            (Path(__file__), ROOT / "scripts/maintenance/windows_metrics.py")}


def freeze():
    if OUT.exists() or AUDIT.exists():
        raise RuntimeError("Mission exists; inspect it rather than refreeze")
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError("Use repository root .venv")
    names = ["circle", "ellipse_parallel", "ellipse_perpendicular", "rectangle_parallel",
             "rectangle_perpendicular", "l_reference", "l_rotated", "c_open_toward"]
    source = study.source_hashes()
    if source != load(HISTORICAL / "manifest.json")["source_files"]:
        raise RuntimeError("Current runtime source differs from census; review before benchmarking")
    cases = []
    origins = {}
    for name in names + ["c_open_away"]:
        folder = HISTORICAL / (name + "_boundary_cvt")
        receipt = load(folder / "case_result.json")
        for filename, expected in receipt["products"].items():
            if sha(folder / filename) != expected:
                raise RuntimeError("Historical receipt mismatch")
        origins[folder.name] = {"receipt_sha256": sha(folder / "case_result.json"), "products": receipt["products"]}
        case = load(folder / "resolved_case.json")
        case["split"] = "engineering"
        cases.append(case)
    OUT.mkdir(parents=True); AUDIT.mkdir(parents=True)
    common = {"schema": "workstation-engineering-v1", "run_dir": str(OUT.relative_to(ROOT)),
              "frozen_source_sha256": study.encode(source), "purpose": "Engineering repeats, not new research samples"}
    save(AUDIT / "ladder_protocol.json", {**common, "cases": cases[:-1]})
    save(AUDIT / "profile_protocol.json", {**common, "cases": cases[-1:]})
    plan = {"mission": MISSION, "levels": [1, 4, 8], "case_order": [c["case_id"] for c in cases[:-1]],
            "profile_case": cases[-1]["case_id"], "maximum_simulation_repeats": 25,
            "wall_budget_seconds": 1800, "stage_timeout_seconds": {"w1": 600, "w4": 300, "w8": 300, "profile": 900, "media": 180},
            "thread_environment": {name: "1" for name in THREAD_NAMES}, "source_files": source,
            "harness_files": own_sources(), "historical_origins": origins,
            "protocol_hashes": {name: sha(AUDIT / name) for name in ("ladder_protocol.json", "profile_protocol.json")},
            "scientific_comparison": "exact replay arrays incl NaN positions, all paired metrics, physics substeps and summary; ignore only case receipt wall_seconds and engineering metadata",
            "advance_rule": "Stop on introduced scientific difference, execution error, resource guard or deadline. No 12+ tiers. Single pass, no cherry-picked repeats.",
            "resource_guards": {"minimum_available_ram_gib": 16, "minimum_disk_free_gib": 20, "max_tree_rss_gib": 32},
            "media": {"case": "circle_boundary_cvt", "stride": 5, "include_final": True, "fps": 10, "size": [960, 640], "software_encode_threads": [1, 4]},
            "disk_probe": "One 64 MiB write+fsync+read; cache-sensitive filesystem probe, not a raw-device benchmark"}
    save(AUDIT / "plan.json", plan)
    for relative in source | own_sources():
        dest = AUDIT / "source_snapshot" / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, dest)
    import threadpoolctl, scipy.linalg, importlib.metadata
    env = {"python": study.environment_versions(), "ambient_threads": AMBIENT_THREADS,
           "effective_threads": {n: os.getenv(n) for n in THREAD_NAMES}, "threadpools": threadpoolctl.threadpool_info(),
           "memory": metrics.memory(), "affinity": metrics.affinity(), "disk": shutil.disk_usage(ROOT)._asdict(),
           "lock_sha256": sha(ROOT / "requirements.lock.txt"), "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()}
    save(AUDIT / "environment.json", env)
    (AUDIT / "git-status-before.txt").write_text(subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True), encoding="utf-8")
    save(AUDIT / "mission_state.json", {"status": "FROZEN", "new_research_samples": 0, "plan_sha256": sha(AUDIT / "plan.json")})
    print(json.dumps({"status": "FROZEN", "plan": str(AUDIT / "plan.json"), "simulation_repeats_max": 25}))


def verify_plan():
    plan = load(AUDIT / "plan.json")
    if study.source_hashes() != plan["source_files"] or own_sources() != plan["harness_files"]:
        raise RuntimeError("Source changed after freeze")
    for name, expected in plan["protocol_hashes"].items():
        if sha(AUDIT / name) != expected:
            raise RuntimeError("Protocol changed after freeze")
    return plan


def measured_worker(payload):
    """Top-level spawn worker. Calls the existing scientific runner unchanged."""
    import threadpoolctl
    study.limit_worker_threads()
    profiler = cProfile.Profile() if os.getenv("DBACT_BENCH_PROFILE") == "1" else None
    start_cpu = time.process_time()
    result = profiler.runcall(original_worker, payload) if profiler else original_worker(payload)
    folder = Path(payload[1]); stage = folder.parent
    if profiler:
        profiler.dump_stats(str(stage / (folder.name + ".pstats")))
    save(stage / "worker_metadata" / (folder.name + ".json"), {
        "pid": os.getpid(), "cpu_seconds": time.process_time() - start_cpu,
        "process_lifetime": metrics.process(os.getpid()), "affinity": metrics.affinity(),
        "environment": {n: os.getenv(n) for n in THREAD_NAMES}, "threadpools": threadpoolctl.threadpool_info()})
    return result


def compare(stage, historical=False):
    folder = OUT / stage
    differences = []
    cases = []
    for path in sorted(folder.glob("*/case_result.json")):
        case = path.parent.name
        baseline = HISTORICAL / case if historical else OUT / "w1" / case
        receipt = load(path)
        if receipt["result"]["execution_status"] != "COMPLETED":
            differences.append(case + ": execution error")
            continue
        study.cached_result(path.parent, receipt["identity"])
        with np.load(path.parent / "replay.npz", allow_pickle=False) as a, np.load(baseline / "replay.npz", allow_pickle=False) as b:
            if a.files != b.files:
                differences.append(case + ": replay keys")
            else:
                for key in a.files:
                    same = np.array_equal(a[key], b[key], equal_nan=True) if a[key].dtype.kind in "fc" else np.array_equal(a[key], b[key])
                    if not same:
                        differences.append(case + ": replay:" + key)
        for filename in ("paired_metrics.json", "physics_substeps.json", "summary.json", "config.json"):
            if load(path.parent / filename) != load(baseline / filename):
                differences.append(case + ": " + filename)
        cases.append(case)
    expected_cases = 1 if stage == "profile" else 8
    if len(cases) != expected_cases:
        differences.append(f"Expected {expected_cases} cases, found {len(cases)}")
    result = {"status": "PASS" if not differences else "FAIL", "cases": cases, "differences": differences,
              "reference": "frozen historical census" if historical else "same-benchmark w1", "criterion": "exact scientific records; no relaxed tolerance"}
    save(OUT / (stage + "-consistency.json"), result)
    return result


def run_command(stage, command, timeout, env=None):
    """Measure only this newly created subprocess tree; retain partial output."""
    path = OUT / (stage + "-telemetry.jsonl")
    if path.exists():
        raise RuntimeError("Stage telemetry exists; do not overwrite")
    start = time.perf_counter(); status = "COMPLETED"; known = {}; rows = []
    counters = metrics.Counters()
    with (OUT / (stage + ".log")).open("x", encoding="utf-8") as log, path.open("x", encoding="utf-8") as telemetry:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        save(OUT / (stage + "-command.json"), {"command": command, "pid": process.pid, "timeout_seconds": timeout})
        try:
            while process.poll() is None:
                entries = [v for pid in metrics.tree(process.pid) if (v := metrics.process(pid)) is not None]
                for entry in entries:
                    known[entry["pid"]] = entry
                mem = metrics.memory()
                row = {"elapsed": time.perf_counter() - start, "processes": entries, "tree_rss_bytes": sum(x["rss_bytes"] for x in entries),
                       "available_ram_bytes": mem["available_bytes"], **counters.sample()}
                telemetry.write(json.dumps(row) + "\n"); telemetry.flush(); rows.append(row)
                if row["elapsed"] > timeout:
                    status = "TIME_BUDGET_EXCEEDED"
                elif mem["available_bytes"] < 16 * 1024**3 or row["tree_rss_bytes"] > 32 * 1024**3 or shutil.disk_usage(OUT).free < 20 * 1024**3:
                    status = "RESOURCE_GUARD"
                if status != "COMPLETED":
                    # Only the child tree launched immediately above is terminated.
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True, check=False)
                    process.wait(timeout=15)
                    break
                time.sleep(1)
        except BaseException:
            # Do not leave a long-lived simulation running after monitor failure.
            if process.poll() is None:
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True, check=False)
                process.wait(timeout=15)
            raise
        finally:
            counters.close()
        code = process.wait()
    if code and status == "COMPLETED":
        status = "EXECUTION_ERROR"
    summary = {"status": status, "exit_code": code, "wall_seconds_including_startup": time.perf_counter() - start,
               "sample_interval_seconds": 1, "peak_sampled_tree_rss_bytes": max((r["tree_rss_bytes"] for r in rows), default=0),
               "last_observed_processes": list(known.values()), "counter_errors": counters.errors}
    for key in metrics.Counters.PATHS:
        values = [r[key] for r in rows[1:] if r[key] is not None]
        summary[key] = {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95)), "max": max(values)} if values else None
    save(OUT / (stage + "-measurements.json"), summary)
    print(json.dumps({"stage": stage, **{k: summary[k] for k in ("status", "wall_seconds_including_startup", "peak_sampled_tree_rss_bytes")}}), flush=True)
    return summary


def media():
    import matplotlib.pyplot as plt
    import imageio_ffmpeg
    from dbact_sim.trace import SimulationTrace
    from dbact_sim.visualization.renderer import ResearchVisualizer
    folder = OUT / "media"; folder.mkdir(exist_ok=False)
    replay_path = HISTORICAL / "circle_boundary_cvt/replay.npz"
    trace = SimulationTrace.from_replay(replay_path)
    frames = list(range(0, trace.frame_count, 5))
    if frames[-1] != trace.frame_count - 1:
        frames.append(trace.frame_count - 1)
    figure, ax = plt.subplots(figsize=(9.6, 6.4), dpi=100)
    renderer = ResearchVisualizer(trace, view_mode="paper", figure=figure, world_ax=ax, show_cage=False, show_map=True, show_sensor=False)
    started = time.perf_counter()
    raw = folder / "frames.rgb"
    with raw.open("xb") as stream:
        for frame in frames:
            renderer.update(frame); figure.canvas.draw()
            stream.write(np.asarray(figure.canvas.buffer_rgba())[:, :, :3].copy().tobytes())
    rendered = time.perf_counter() - started
    static_start = time.perf_counter(); figure.savefig(folder / "final-frame.pdf"); static_time = time.perf_counter() - static_start
    figure.savefig(folder / "final-frame.png"); plt.close(figure)
    ff = imageio_ffmpeg.get_ffmpeg_exe(); encodes = []
    for threads in (1, 4):
        output = folder / f"libx264-t{threads}.mp4"
        cmd = [ff, "-nostdin", "-v", "error", "-n", "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "960x640", "-framerate", "10", "-i", str(raw),
               "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-threads", str(threads), "-pix_fmt", "yuv420p", str(output)]
        now = time.perf_counter(); subprocess.run(cmd, check=True); elapsed = time.perf_counter() - now
        subprocess.run([ff, "-v", "error", "-i", str(output), "-f", "null", "-"], check=True)
        encodes.append({"threads": threads, "seconds": elapsed, "bytes": output.stat().st_size, "sha256": sha(output), "decode": "PASS", "command": cmd})
    save(folder / "receipt.json", {"frame_count": len(frames), "frames": frames, "replay_sha256": sha(replay_path), "raw_sha256": sha(raw),
                                  "render_seconds": rendered, "render_frames_sec": len(frames) / rendered, "vector_export_seconds": static_time,
                                  "ffmpeg": subprocess.check_output([ff, "-version"], text=True).splitlines()[0], "encodes": encodes})


def disk_probe():
    path = OUT / "disk-probe-64MiB.bin"
    block = bytes((i % 251 for i in range(1024 * 1024)))
    now = time.perf_counter()
    with path.open("xb") as stream:
        for _ in range(64): stream.write(block)
        stream.flush(); os.fsync(stream.fileno())
    write = time.perf_counter() - now
    now = time.perf_counter()
    with path.open("rb") as stream:
        while stream.read(1024 * 1024): pass
    read = time.perf_counter() - now
    save(OUT / "disk-probe.json", {"bytes": path.stat().st_size, "sha256": sha(path), "write_fsync_seconds": write,
                                  "cached_read_seconds": read, "raw_device_benchmark": False})


def run():
    plan = verify_plan()
    if (OUT / "run-start.json").exists():
        raise RuntimeError("Run already started; use recorded per-study resume only, do not repeat timing trials")
    start = time.perf_counter()
    save(OUT / "run-start.json", {"utc": time.time(), "budget_seconds": plan["wall_budget_seconds"]})
    disk_probe()
    stages = []
    for stage in ("w1", "w4", "w8", "profile", "media"):
        remaining = plan["wall_budget_seconds"] - (time.perf_counter() - start)
        if remaining < 30:
            stages.append({"stage": stage, "status": "NOT_RUN_BUDGET"}); break
        command = [sys.executable, str(Path(__file__).resolve()), "child", "--stage", stage]
        result = run_command(stage, command, min(plan["stage_timeout_seconds"][stage], remaining))
        stages.append({"stage": stage, **result})
        if result["status"] != "COMPLETED": break
        if stage != "media":
            check = compare(stage, historical=stage in ("w1", "profile"))
            stages[-1]["scientific_consistency"] = check["status"]
            if check["status"] != "PASS": break
        save(AUDIT / "mission_state.json", {"status": "RUNNING", "stages": stages, "new_research_samples": 0})
    complete = len(stages) == 5 and all(s["status"] == "COMPLETED" and s.get("scientific_consistency", "PASS") == "PASS" for s in stages)
    save(AUDIT / "mission_state.json", {"status": "MEASUREMENTS_FINISHED" if complete else "PARTIAL_MEASUREMENTS", "stages": stages, "new_research_samples": 0,
                                         "wall_seconds": time.perf_counter() - start, "source_unchanged": study.source_hashes() == plan["source_files"]})


def child(stage):
    verify_plan()
    if stage == "media":
        media(); return
    profile = stage == "profile"
    os.environ["DBACT_BENCH_PROFILE"] = "1" if profile else "0"
    study.worker = measured_worker
    result = study.run_study(AUDIT / ("profile_protocol.json" if profile else "ladder_protocol.json"),
                             "engineering", OUT / stage, 1 if profile else int(stage[1:]), False)
    if result["execution_errors"] or result["source_integrity"] != "PASS":
        raise RuntimeError("Study failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("freeze", "run", "child", "check"))
    parser.add_argument("--stage", choices=("w1", "w4", "w8", "profile", "media"))
    args = parser.parse_args()
    if args.action == "child" and args.stage is None:
        parser.error("child requires --stage")
    if args.action == "freeze": freeze()
    elif args.action == "run": run()
    elif args.action == "child": child(args.stage)
    else: print(json.dumps({"status": "PASS", "plan": verify_plan()["mission"]}))
