"""Fixed-observation R1 model feasibility diagnostics, with no physical replay."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import time
import traceback
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import post_acquisition_support_study as study
import yaml
from probe_post_support_contact_packet_geometry import features, packet
from research_io import ROOT, guard, read, sha
from threadpoolctl import threadpool_limits

from dbact.geometry import ray_batch_first_hits, signed_distance_and_gradient
from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.safety.post_support import build_rows
from dbact.sensing.finite_support import scan_support, support_rows

GEOMETRY = ROOT / "runs/research/2026-11-03-post-support-contact-packet-geometry-v1"
CONTACT = ROOT / "runs/research/2026-11-03-post-support-contact-compatibility-v2"
CANDIDATE = ROOT / "runs/research/2026-11-04-phase1-first-pusher-loss-v3"
DESIGN = ROOT / "experiments/ccta2027/phase2_r1_model_feasibility_v1_design.md"
CONFIG = ROOT / "configs/research/best_known_transport_v1.yaml"


def interval_width(radius, safe_radius, error, rho, gamma):
    """Hypothetical symmetric interval only; no claim that error is a true bound."""
    assert radius > safe_radius >= 0 and error >= 0 and gamma > 0 and rho >= 0
    raw = float(radius - safe_radius - 2 * error)
    reserved = float(raw - rho / gamma)
    return {
        "raw_width": raw,
        "reserved_width": reserved,
        "robust_reserved_contact_possible": bool(reserved > 1e-12),
    }


def hidden_protrusion():
    """A deterministic geometry counterexample with the protected 72 ray layout."""
    count, distance = 72, 0.1
    origin = np.array([-distance, 0.0])
    angle = 2 * math.pi / count
    height = distance * math.tan(angle)
    tip = origin + 0.02 * np.array([math.cos(angle / 2), math.sin(angle / 2)])
    flat = np.array([[0.0, -1.0], [1.0, -1.0], [1.0, 1.0], [0.0, 1.0]])
    hidden = np.array(
        [
            [0.0, -1.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.75 * height],
            tip,
            [0.0, 0.25 * height],
        ]
    )
    angles = np.linspace(0.0, 2 * math.pi, count, endpoint=False)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    first, _, _ = ray_batch_first_hits(origin, directions, flat, 1.2)
    second, _, _ = ray_batch_first_hits(origin, directions, hidden, 1.2)
    # Splitting the unchanged wall into different edge lengths changes floating
    # point intersection rounding. The added triangle is strictly between rays;
    # report numerical equality separately from the geometric argument.
    np.testing.assert_array_equal(np.isfinite(first), np.isfinite(second))
    np.testing.assert_allclose(first, second, rtol=0.0, atol=1e-12)
    hit = np.isfinite(first)
    returns = [
        {
            "object_id": "cargo_0",
            "point": origin + first[i] * directions[i],
            "ray_index": int(i),
        }
        for i in np.flatnonzero(hit)
    ]
    params = SafetyFilterParams(
        r_safe=0.065,
        max_speed=0.35,
        gamma_obj=8.0,
        rho=0.02,
        dt=0.05,
        object_velocity_bound=0.2,
        object_row_range=0.6,
    )
    scan = scan_support(returns, origin, 0.0, 0.01)
    support = support_rows(
        origin,
        [scan],
        params,
        0.0,
        0.2,
        isolated_point_guards=True,
        noise_in_radius=False,
    )
    signed = [
        float(signed_distance_and_gradient(origin[None, :], polygon)[0][0])
        for polygon in (flat, hidden)
    ]
    return {
        "scope": "Algebraic single-packet example, no physical episode or full-history equivalence",
        "origin": origin.tolist(),
        "flat_vertices": flat.tolist(),
        "hidden_vertices": hidden.tolist(),
        "ray_count": count,
        "sensor_range": 1.2,
        "same_all_ray_ranges": True,
        "range_equality_tolerance_m": 1e-12,
        "maximum_absolute_range_difference_m": float(
            np.max(np.abs(first[hit] - second[hit]))
        ),
        "directions": directions.tolist(),
        "ray_ranges": [float(x) if np.isfinite(x) else None for x in first],
        "hit_count": int(hit.sum()),
        "adjacent_chord_half_gap": 0.5 * height,
        "signed_distances": signed,
        "r_safe": params.r_safe,
        "robot_radius": 0.13,
        "noise_allowance": 0.03,
        "support_ready": support.ready,
        "support_reason": support.reason,
        "minimum_support_lower_distance": float(np.min(support.lower_distance))
        if support.ready
        else None,
        "minimum_support_h": float(np.min(support.h)) if support.ready else None,
        "unknown_boundary_bound_valid": False,
    }


def gaussian_allowance():
    probability = math.erfc(3 / math.sqrt(2))
    return {
        "single_draw_exceedance": probability,
        "at_least_one_exceedance_72_independent_draws": -math.expm1(
            72 * math.log1p(-probability)
        ),
        "scope": "Distribution calculation for 72 independent draws, not measured scan failure frequency",
    }


def save(path, value):
    with path.open("x", encoding="utf-8") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")


def analyze(
    cohort,
    condition,
    frame,
    agent_id,
    position,
    params,
    inputs,
    trace,
    expected,
    radius,
    positive_force=None,
    selected_pusher=False,
    truth=None,
):
    result = {
        "cohort": cohort,
        "condition": condition,
        "frame": frame,
        "agent_id": agent_id,
        "admitted": bool(trace["applied"]),
        "guard": trace["reason"],
        "positive_force_in_ensuing_saved_interval": positive_force,
        "selected_pusher": bool(selected_pusher),
        "rows": [],
        "truth": truth,
    }
    values = inputs.get("post_support_packets", [])
    if not values:
        assert not trace["applied"]
        return result
    packets = [packet(v) for v in values]
    support, demand = build_rows(
        np.asarray(position),
        packets,
        params,
        inputs["observation_time"],
        inputs["object_velocity"],
        inputs.get("boundary_point_velocities"),
    )
    assert support.ready == trace["applied"] and support.reason == trace["reason"]
    if not support.ready:
        return result
    assert abs(demand - trace["demand_speed"]) <= 1e-12
    mask = np.asarray(expected["kinds"]) == "object"
    A, b = np.asarray(expected["A"])[mask], np.asarray(expected["b"])[mask]
    np.testing.assert_array_equal(support.A, A)
    np.testing.assert_array_equal(support.h, expected["h"])
    np.testing.assert_allclose(
        SafetyFilter(params)._cap_to_reachable(
            A, demand - params.gamma_obj * support.h + params.rho
        ),
        b,
        atol=1e-12,
        rtol=0.0,
    )
    _, kept = features(
        np.asarray(position),
        trace["source_agents"],
        packets,
        params,
        inputs["observation_time"],
    )
    assert len(kept) == len(A)
    for index, feature in enumerate(kept):
        np.testing.assert_allclose(feature["normal"], A[index], rtol=0.0, atol=1e-12)
        assert abs(feature["h"] - support.h[index]) <= 1e-12
        error = feature["structural"] + feature["noise"]
        row = {
            "row": index,
            "source_agent": feature["source_agent"],
            "kind": feature["kind"],
            "own_source": feature["source_agent"] == agent_id,
            "distance": feature["distance"],
            "structural": feature["structural"],
            "noise": feature["noise"],
            "hypothetical_error": error,
            "lower_distance": feature["lower_distance"],
            "h": float(support.h[index]),
            "normal": A[index].tolist(),
            "capped_rhs": float(b[index]),
            "demand_speed": demand,
            "uncapped_rhs": float(
                demand - params.gamma_obj * support.h[index] + params.rho
            ),
            "rho_over_gamma": params.rho / params.gamma_obj,
            "motion_over_gamma": demand / params.gamma_obj,
            "r_safe": params.r_safe,
            "robot_radius": radius,
            **interval_width(
                radius, params.r_safe, error, params.rho, params.gamma_obj
            ),
        }
        if truth is not None:
            row["normal_alignment_with_true_nearest"] = float(
                A[index] @ truth["normal"]
            )
        result["rows"].append(row)
    result["object_rows"] = len(kept)
    result["minimum_model_h"] = float(support.h.min())
    result["minimum_model_lower_distance"] = float(support.lower_distance.min())
    if truth is not None:
        result["model_lower_minus_true_distance"] = float(
            support.lower_distance.min() - truth["signed_distance"]
        )
        result["true_safe_model_negative_h"] = bool(
            truth["h"] >= 0 and support.h.min() < 0
        )
    return result


def summarize(states):
    rows = [r for state in states for r in state["rows"]]
    admitted = [s for s in states if s["admitted"]]
    metrics = {
        "states": len(states),
        "admitted_states": len(admitted),
        "object_rows": len(rows),
        "guard_counts": dict(Counter(s["guard"] for s in states)),
        "positive_raw_width_rows": sum(r["raw_width"] > 1e-12 for r in rows),
        "positive_reserved_width_rows": sum(
            r["robust_reserved_contact_possible"] for r in rows
        ),
        "states_with_any_positive_reserved_width_row": sum(
            any(r["robust_reserved_contact_possible"] for r in s["rows"])
            for s in admitted
        ),
        "states_with_all_positive_reserved_width_rows": sum(
            all(r["robust_reserved_contact_possible"] for r in s["rows"])
            for s in admitted
        ),
    }
    for key in ["structural", "noise", "raw_width", "reserved_width"]:
        metrics[key + "_range"] = (
            [min(r[key] for r in rows), max(r[key] for r in rows)] if rows else None
        )
    labeled = [s for s in admitted if s["truth"] is not None]
    if labeled:
        metrics.update(
            truth_labeled_states=len(labeled),
            true_safe_model_negative_h=sum(
                s["true_safe_model_negative_h"] for s in labeled
            ),
            min_model_lower_above_true=sum(
                s["model_lower_minus_true_distance"] > 1e-7 for s in labeled
            ),
            model_lower_minus_true_range=[
                min(s["model_lower_minus_true_distance"] for s in labeled),
                max(s["model_lower_minus_true_distance"] for s in labeled),
            ],
        )
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    out = parser.parse_args().out.resolve()
    if not out.is_relative_to(ROOT / "runs"):
        raise ValueError("Fresh output must stay under runs/")
    guard()
    frozen = study.check()
    for folder in [GEOMETRY, CONTACT, CANDIDATE]:
        assert read(folder / "outcome.json")["analysis_sha256"] == sha(
            folder / "analysis.json"
        )
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    radius = config["controller"]["robot_radius"]
    assert config["controller"]["ray_count"] == 72
    paths = [
        GEOMETRY / "analysis.json",
        GEOMETRY / "outcome.json",
        CONTACT / "analysis.json",
        CONTACT / "outcome.json",
        CANDIDATE / "analysis.json",
        CANDIDATE / "outcome.json",
        DESIGN,
        CONFIG,
        Path(__file__),
        ROOT / "tests/test_r1_model_feasibility.py",
        ROOT / "scripts/probe_post_support_contact_packet_geometry.py",
        ROOT / "src/dbact/sensing/finite_support.py",
        ROOT / "src/dbact/safety/post_support.py",
        ROOT / "src/dbact/geometry.py",
    ]
    paths += [GEOMETRY / (c + "-packets.json") for c in study.CONDITIONS]
    paths += [CANDIDATE / (c + ".json") for c in study.CONDITIONS]
    identity = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {p.relative_to(ROOT).as_posix(): sha(p) for p in paths},
        "runtime_source_sha256": frozen["frozen_source_sha256"],
        "protocol_sha256": sha(study.PROTOCOL),
        "environment": frozen["environment"],
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
    }
    out.mkdir(parents=True, exist_ok=False)
    snapshot = out / "inputs_and_sources"
    snapshot.mkdir()
    for p in paths:
        dest = snapshot / p.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
    save(out / "started.json", identity)
    started = time.perf_counter()
    states, unique_packets, noise_rows = [], {}, []
    try:
        old = {
            (r["condition"], e["saved_frame"], s["agent_id"]): s
            for r in read(CONTACT / "analysis.json")["results"]
            for e in r["events"]
            for s in e["states"]
        }
        geometries = {
            (s["condition"], s["frame"], s["agent_id"]): s
            for s in read(GEOMETRY / "analysis.json")["states"]
        }
        with threadpool_limits(limits=1):
            for condition in study.CONDITIONS:
                for capture in read(GEOMETRY / (condition + "-packets.json"))[
                    "captures"
                ]:
                    key = condition, capture["frame"], capture["agent_id"]
                    previous, geometry = old[key], geometries[key]
                    params = SafetyFilterParams(**capture["parameters"])
                    signed, normal, _ = signed_distance_and_gradient(
                        np.asarray([capture["position"]]),
                        np.asarray(capture["evaluation_precommand_vertices"]),
                    )
                    assert (
                        abs(signed[0] - geometry["evaluation_signed_distance"]) <= 1e-12
                    )
                    truth = {
                        "signed_distance": float(signed[0]),
                        "h": float(signed[0] - params.r_safe),
                        "normal": normal[0].tolist(),
                        "time": "pre-command",
                    }
                    inputs = {
                        "post_support_packets": capture["packets"],
                        "observation_time": capture["timestamp"],
                        "object_velocity": capture["object_velocity"],
                        "boundary_point_velocities": capture[
                            "boundary_point_velocities"
                        ],
                    }
                    states.append(
                        analyze(
                            "control_128",
                            *key,
                            capture["position"],
                            params,
                            inputs,
                            previous["support"],
                            previous["counterfactual_rows"],
                            radius,
                            positive_force=bool(geometry["positive_force"]),
                            truth=truth,
                        )
                    )
                    for source, value in zip(
                        capture["source_agents"], capture["packets"]
                    ):
                        pkey = condition, capture["frame"], source
                        digest = hashlib.sha256(
                            json.dumps(value, sort_keys=True).encode()
                        ).hexdigest()
                        if pkey in unique_packets:
                            assert unique_packets[pkey] == digest
                            continue
                        unique_packets[pkey] = digest
                        origin, points = (
                            np.asarray(value["origin"]),
                            np.asarray(value["points"]),
                        )
                        if not len(points):
                            continue
                        rays = points - origin
                        measured = np.linalg.norm(rays, axis=1)
                        ranges, _, _ = ray_batch_first_hits(
                            origin,
                            rays / measured[:, None],
                            np.asarray(capture["evaluation_precommand_vertices"]),
                            config["controller"]["sensor_range"],
                        )
                        for index, (measured_range, true_range) in enumerate(
                            zip(measured, ranges)
                        ):
                            error = (
                                float(measured_range - true_range)
                                if np.isfinite(true_range)
                                else None
                            )
                            noise_rows.append(
                                {
                                    "condition": condition,
                                    "frame": capture["frame"],
                                    "source": source,
                                    "point_index": index,
                                    "range_error": error,
                                    "allowance": value["noise_allowance"],
                                    "outside_allowance": abs(error)
                                    > value["noise_allowance"] + 1e-12
                                    if error is not None
                                    else None,
                                }
                            )
                candidate_data = read(CANDIDATE / (condition + ".json"))
                for item in candidate_data["states"]:
                    s = item["actual"]
                    frame = candidate_data["frame"]
                    states.append(
                        analyze(
                            "candidate_64",
                            condition,
                            frame,
                            s["agent_id"],
                            s["position"],
                            SafetyFilterParams(**s["params"]),
                            s["inputs"],
                            s["post_support"],
                            s,
                            radius,
                            selected_pusher=item["selected_pusher"],
                        )
                    )
                assert time.perf_counter() - started < 7200
            counterexample = hidden_protrusion()
        controls = [s for s in states if s["cohort"] == "control_128"]
        candidates = [s for s in states if s["cohort"] == "candidate_64"]
        assert len(controls) == 128 and len(candidates) == 64
        assert (
            sum(s["positive_force_in_ensuing_saved_interval"] for s in controls) == 17
        )
        assert sum(s["selected_pusher"] for s in candidates) == 4
        groups = {
            "control_all": summarize(controls),
            "control_positive_force": summarize(
                [s for s in controls if s["positive_force_in_ensuing_saved_interval"]]
            ),
            "candidate_all": summarize(candidates),
            "candidate_selected_pushers": summarize(
                [s for s in candidates if s["selected_pusher"]]
            ),
        }
        noise_summary = {
            "unique_packets": len(unique_packets),
            "returns": len(noise_rows),
            "unresolved_returns": sum(r["range_error"] is None for r in noise_rows),
            "outside_three_sigma": sum(
                r["outside_allowance"] is True for r in noise_rows
            ),
            "maximum_abs_range_error": max(
                abs(r["range_error"])
                for r in noise_rows
                if r["range_error"] is not None
            ),
            "scope": "Reused control cohort unique source/frame returns, not independent task trials",
        }
        report = {
            **identity,
            "groups": groups,
            "states": states,
            "noise_summary": noise_summary,
            "noise_rows": noise_rows,
            "gaussian_model": gaussian_allowance(),
            "counterexample": counterexample,
            "wall_seconds": time.perf_counter() - started,
            "scope": "Conditional interval diagnostics and algebraic/source counterexamples; no task improvement validation",
        }
        study.check()
        assert all(
            sha(ROOT / name) == digest for name, digest in identity["inputs"].items()
        )
        save(out / "analysis.json", report)
        for filename, table in [
            (
                "all_feature_rows.csv",
                [
                    {
                        **{
                            k: s[k]
                            for k in ("cohort", "condition", "frame", "agent_id")
                        },
                        **{k: v for k, v in r.items() if k != "normal"},
                    }
                    for s in states
                    for r in s["rows"]
                ],
            ),
            (
                "all_states.csv",
                [
                    {k: v for k, v in s.items() if k not in ("rows", "truth")}
                    for s in states
                ],
            ),
            ("unique_control_returns.csv", noise_rows),
        ]:
            fields = list(dict.fromkeys(k for row in table for k in row))
            with (out / filename).open("x", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                writer.writerows(table)
        save(
            out / "outcome.json",
            {
                **identity,
                "status": "COMPLETED_R1_MODEL_FEASIBILITY_DIAGNOSTIC",
                "analysis_sha256": sha(out / "analysis.json"),
                "groups": groups,
                "noise_summary": noise_summary,
                "wall_seconds": report["wall_seconds"],
            },
        )
        print(
            json.dumps(
                {
                    "groups": groups,
                    "noise_summary": noise_summary,
                    "counterexample": {
                        k: v
                        for k, v in counterexample.items()
                        if k
                        not in (
                            "directions",
                            "ray_ranges",
                            "flat_vertices",
                            "hidden_vertices",
                        )
                    },
                    "wall_seconds": report["wall_seconds"],
                }
            ),
            flush=True,
        )
    except Exception:
        save(
            out / "failure.json",
            {
                **identity,
                "completed_states": len(states),
                "wall_seconds": time.perf_counter() - started,
                "traceback": traceback.format_exc(),
            },
        )
        raise


if __name__ == "__main__":
    main()
