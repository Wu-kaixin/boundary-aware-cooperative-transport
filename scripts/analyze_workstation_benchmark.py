"""Summarize saved engineering measurements; never run or alter simulations."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import pstats
import statistics

ROOT = Path(__file__).resolve().parents[1]
MISSION = "2026-11-01-workstation-benchmark-v1"
OUT = ROOT / "runs/maintenance" / MISSION
AUDIT = ROOT / "artifacts/migrations" / MISSION


def load(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def size(path):
    return sum(p.stat().st_size for p in path.rglob("*") if p.is_file())


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def cost_estimate():
    historical = ROOT / "runs/research/2026-10-31-multishape-census-v1/run"
    cases = load(historical / "resolved_protocol.json")["cases"]
    old = {p.parent.name: load(p)["result"] for p in historical.glob("*/case_result.json")}
    measured = {p.parent.name: load(p)["result"] for p in (OUT / "w1").glob("*/case_result.json")}
    ratios = [v["wall_seconds"] / old[k]["wall_seconds"] for k, v in measured.items()]
    loads = [0.0] * 8
    for case in cases:
        slot = min(range(8), key=lambda index: loads[index])
        loads[slot] += old[case["case_id"]]["wall_seconds"]
    rate = max(v["wall_seconds"] / v["termination"]["frames_run"] for v in list(old.values()) + list(measured.values()))
    bytes_per_frame = max(size(historical / key) / v["termination"]["frames_run"] for key, v in old.items())
    return {"historical_cases": len(old), "historical_sum_case_seconds": sum(v["wall_seconds"] for v in old.values()),
            "historical_fifo_8worker_model_seconds": max(loads), "matched_current_w1_over_historical_ratios": ratios,
            "planning_multiplier": 1.5, "mixed_27_conservative_model_seconds": max(loads) * max(ratios) * 1.5,
            "observed_max_seconds_per_frame": rate, "all_3000frames_27cases_conservative_seconds": math.ceil(27 / 8) * 3000 * rate * 1.5,
            "observed_max_bytes_per_frame": bytes_per_frame, "all_3000frames_27cases_raw_budget_bytes": 27 * 3000 * bytes_per_frame * 1.5,
            "caveat": "Planning envelope, not a confidence interval or observed 8-worker 27-case timing; excludes rendering and independent backup; no new matrix executed"}


def main():
    table = []
    for level in (1, 4, 8):
        stage = f"w{level}"
        measured_path = OUT / f"{stage}-measurements.json"
        if not measured_path.exists():
            continue
        measured = load(measured_path)
        cases = [load(p)["result"] for p in sorted((OUT / stage).glob("*/case_result.json"))]
        times = [c["wall_seconds"] for c in cases]
        metadata = [load(p) for p in (OUT / stage / "worker_metadata").glob("*.json")]
        telemetry = [json.loads(line) for line in (OUT / f"{stage}-telemetry.jsonl").read_text().splitlines()]
        elapsed = measured["wall_seconds_including_startup"]
        row = {"workers": level, "cases": len(cases), "wall_seconds": elapsed,
               "cases_per_hour": len(cases) * 3600 / elapsed,
               "case_seconds_min": min(times), "case_seconds_median": statistics.median(times),
               "case_seconds_max": max(times), "worker_cpu_seconds": sum(m["cpu_seconds"] for m in metadata),
               "peak_sampled_tree_rss_gib": measured["peak_sampled_tree_rss_bytes"] / 1024**3,
               "minimum_available_ram_gib": min(r["available_ram_bytes"] for r in telemetry) / 1024**3,
               "host_cpu_percent_mean": measured["host_cpu_percent"]["mean"] if measured["host_cpu_percent"] else None,
               "disk_read_p95_ms": measured["disk_read_seconds"]["p95"] * 1000 if measured["disk_read_seconds"] else None,
               "disk_write_p95_ms": measured["disk_write_seconds"]["p95"] * 1000 if measured["disk_write_seconds"] else None,
               "disk_queue_max": measured["disk_queue"]["max"] if measured["disk_queue"] else None,
               "errors": sum(c["execution_status"] != "COMPLETED" for c in cases),
               "task_completed": sum(c["task_completed"] for c in cases),
               "g500_pass": sum(c["g500_pass"] for c in cases),
               "all_threadpools_single": all(t["num_threads"] == 1 for m in metadata for t in m["threadpools"]),
               "all_thread_env_single": all(v == "1" for m in metadata for v in m["environment"].values()),
               "case_output_bytes": sum(size(p.parent) for p in (OUT / stage).glob("*/case_result.json")),
               "consistency": load(OUT / f"{stage}-consistency.json")["status"]}
        table.append(row)
    for row in table:
        row["speedup_vs_w1"] = table[0]["wall_seconds"] / row["wall_seconds"]
    profile = None
    paths = list((OUT / "profile").glob("*.pstats"))
    if paths:
        stats = pstats.Stats(str(paths[0]))
        functions = []
        for (filename, line, name), (primitive, calls, own, cumulative, callers) in stats.stats.items():
            functions.append({"file": filename.replace("\\", "/"), "line": line, "function": name,
                              "calls": calls, "primitive_calls": primitive, "self_seconds": own,
                              "cumulative_seconds": cumulative})
        functions.sort(key=lambda r: r["cumulative_seconds"], reverse=True)
        selectors = {
            "sensing": ("/sensing/sensor.py", "sense_view"),
            "registration_including_matching": ("/sensing/boundary_map.py", "register"),
            "matching_nested_in_registration": ("/sensing/boundary_map.py", "_match_rows"),
            "map_update": ("/sensing/boundary_map.py", "update"),
            "map_carve": ("/sensing/boundary_map.py", "carve"),
            "deployment_cvt": ("/deployment/cvt.py", "compute"),
            "safety_filter_including_qp": ("/safety/filter.py", "filter_velocity"),
            "qp_nested_in_filter": ("/safety/qp2d.py", "solve_min_norm_2d"),
            "contact_physics": ("/contact_dynamics.py", "step"),
            "record_state": ("/dbact_sim/environment.py", "_record"),
            "evaluation_observer": ("/experiments/deployment.py", "observe"),
            "save_environment_outputs": ("/dbact_sim/environment.py", "save_outputs"),
            "serialize_diagnostics": ("/experiments/deployment.py", "dump"),
        }
        stages = {label: [r for r in functions if r["file"].endswith(suffix) and r["function"] == name]
                  for label, (suffix, name) in selectors.items()}
        profile = {"pstats_sha256": digest(paths[0]), "profiled_seconds": stats.total_tt,
                   "total_calls": stats.total_calls, "stage_functions": stages,
                   "top_cumulative": functions[:45],
                   "top_self": sorted(functions, key=lambda r: r["self_seconds"], reverse=True)[:35],
                   "caveat": "cProfile perturbs timing. Cumulative calls overlap; do not sum nested entries. One complete C-away failure, not all shapes."}
    cost = cost_estimate() if len(table) == 3 else None
    result = {"mission": MISSION, "throughput": table, "profile": profile, "cost_estimate": cost,
              "media": load(OUT / "media/receipt.json") if (OUT / "media/receipt.json").exists() else None,
              "disk_probe": load(OUT / "disk-probe.json"),
              "raw_outputs_bytes_at_analysis": size(OUT), "audit_bytes_at_analysis": size(AUDIT),
              "analysis_script_sha256": digest(Path(__file__))}
    (AUDIT / "analysis.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    if cost:
        (AUDIT / "cost-estimate.json").write_text(json.dumps(cost, indent=2), encoding="utf-8")
    if table:
        with (AUDIT / "throughput.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader(); writer.writerows(table)
    print(json.dumps({"throughput": table, "profile_seconds": profile["profiled_seconds"] if profile else None,
                      "output_bytes": result["raw_outputs_bytes_at_analysis"]}, indent=2))


if __name__ == "__main__":
    main()
