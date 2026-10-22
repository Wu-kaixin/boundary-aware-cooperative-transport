"""Reanalyse transport outcomes without treating summary symptoms as causes.

python -m dbact_sim.experiments.reliability --receipts ... --out ...
Optional --traces points to a study directory containing case/paired_metrics.json.
The input receipt bundle is never modified. Thresholds use each recorded task.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path


def classify(result: dict, steps: list[dict] | None = None) -> dict:
    tasks = result.get("task", {})
    task = next(iter(tasks.values()), {})
    tolerance = task.get("tolerance")
    endpoint = result.get("euclidean_endpoint_error")
    signed = result.get("signed_progress_error")
    lateral = (math.sqrt(max(0.0, endpoint ** 2 - signed ** 2))
               if endpoint is not None and signed is not None else None)
    tags = []
    if result.get("execution_status") == "ERROR":
        outcome = "execution_error"
    elif result.get("task_completed") is True:
        outcome = "completed"
    elif tolerance is None or endpoint is None or signed is None:
        outcome = "unresolved_missing_metrics"
    elif endpoint > tolerance:
        longitudinal_bad = abs(signed) > tolerance
        lateral_bad = lateral > tolerance
        if longitudinal_bad and lateral_bad:
            outcome = "mixed_endpoint_error"
        elif longitudinal_bad:
            outcome = "undershoot" if signed < 0 else "overshoot"
        elif lateral_bad:
            outcome = "cross_track_error"
        else:
            outcome = "combined_endpoint_error"
    else:
        outcome = "not_settled" if not result.get("settled") else "inconsistent_completion"
    if result.get("watchdog"):
        tags.append("watchdog")
    events = result.get("events", {})
    for phase in ("CONTACT_READY", "TRANSPORT", "BRAKE", "HOLD"):
        if events.get(phase, {}).get("right_censored"):
            tags.append(f"{phase.lower()}_not_observed")
    physical = result.get("physical_contact_ready", {})
    if physical.get("right_censored"):
        tags.append("physical_contact_dwell_not_observed")
    response = result.get("phase_response", {})
    for name in ("contact_recovery_failed", "estimate_stopped", "push_suspended"):
        if response.get(name):
            tags.append(name)
    trace = {"available": bool(steps), "causal_attribution": "unresolved"}
    if steps:
        bias = [s["estimated_progress"] - s["true_progress"] for s in steps
                if s.get("estimated_progress") is not None and s.get("true_progress") is not None]
        brake = next((s for s in steps if s.get("phase") == "BRAKE"), None)
        trace.update(
            final_progress_bias_m=bias[-1] if bias else None,
            max_absolute_progress_bias_m=max(map(abs, bias), default=None),
            brake_progress_bias_m=(brake["estimated_progress"] - brake["true_progress"] if brake else None),
            zero_positive_contact_frames=sum(s.get("positive_contacts") == 0 for s in steps),
            unavailable_estimate_frames=sum(s.get("phase_guards", {}).get("estimate_quality") == "unavailable"
                                            for s in steps),
        )
        for name in ("contact_recovery_failed", "estimate_stopped", "push_suspended"):
            if any(s.get("phase_response", {}).get(name) for s in steps) and name not in tags:
                tags.append(name)
    return {"case_id": result["case_id"], "split": result.get("split"),
            "variant": result.get("variant"), "seed": result.get("seed"),
            "outcome": outcome, "endpoint_error_m": endpoint, "signed_progress_error_m": signed,
            "final_cross_track_m": lateral, "task_tolerance_m": tolerance,
            "observed_tags": tags, "trace": trace}


def analyse(receipts: Path, traces: Path | None = None) -> dict:
    data = json.loads(receipts.read_text(encoding="utf-8-sig"))
    rows = []
    identifiers = set()
    for result in data["results"]:
        if result.get("gate") != 6:
            continue
        if result["case_id"] in identifiers:
            raise ValueError(f"Duplicate case: {result['case_id']}")
        identifiers.add(result["case_id"])
        steps = None
        if traces:
            path = (traces / result["case_id"] / "paired_metrics.json").resolve()
            if not path.is_relative_to(traces.resolve()):
                raise ValueError("Trace path escapes study directory")
            if path.exists():
                steps = json.loads(path.read_text(encoding="utf-8-sig"))["steps"]
        rows.append(classify(result, steps))
    counts = defaultdict(Counter)
    for row in rows:
        counts[f"{row['split']}/{row['variant']}"][row["outcome"]] += 1
    return {"schema": "transport-reliability-v1", "receipts_sha256": hashlib.sha256(receipts.read_bytes()).hexdigest(),
            "scope": "descriptive outcomes and observed mechanism signals; no causal identification",
            "transport_episodes": len(rows), "trace_episodes": sum(r["trace"]["available"] for r in rows),
            "counts": dict(counts), "cases": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipts", type=Path, required=True)
    parser.add_argument("--traces", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = analyse(args.receipts, args.traces)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("transport_episodes", "trace_episodes", "counts")}))


if __name__ == "__main__":
    main()
