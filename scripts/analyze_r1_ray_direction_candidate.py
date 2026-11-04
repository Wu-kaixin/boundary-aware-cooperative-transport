"""One fixed directional-noise candidate, using saved states only."""

from __future__ import annotations

import argparse
import csv
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
from probe_post_support_contact_direction import maximize_direction
from probe_post_support_contact_packet_geometry import features, packet
from research_io import ROOT, guard, read, sha
from threadpoolctl import threadpool_limits

from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.safety.post_support import build_rows

GEOMETRY = ROOT / "runs/research/2026-11-03-post-support-contact-packet-geometry-v1"
CONTACT = ROOT / "runs/research/2026-11-03-post-support-contact-compatibility-v2"
DIRECTION = (
    ROOT / "runs/research/2026-11-03-post-support-contact-direction-capability-v1"
)
CANDIDATE = ROOT / "runs/research/2026-11-04-phase1-first-pusher-loss-v3"
AUDIT = ROOT / "runs/research/2026-11-04-phase2-r1-model-feasibility-v1/calculation-v1"
DESIGN = ROOT / "experiments/ccta2027/phase2_r1_ray_direction_v1_design.md"


def directional_lower(position, normal, start, end, origin, structural, noise, r_safe):
    """Project the existing radial endpoint uncertainty onto the fixed normal."""
    p, n, z, o = map(np.asarray, [position, normal, [start, end], origin])
    assert p.shape == n.shape == o.shape == (2,) and z.shape == (2, 2)
    assert np.isfinite(np.r_[p, n, o, z.ravel()]).all()
    assert abs(np.linalg.norm(n) - 1) < 1e-10
    assert structural >= 0 and noise >= 0 and r_safe >= 0
    radial = z - o
    ranges = np.linalg.norm(radial, axis=1)
    assert (ranges > 0).all()
    rays = radial / ranges[:, None]
    nominal_support = float(np.max(z @ n))
    support = float(np.max(z @ n + noise * np.abs(rays @ n)))
    distance_plane = float(n @ p - nominal_support)
    old_lower = distance_plane - structural - noise
    new_lower = float(n @ p - support - structural)
    gain = new_lower - old_lower
    assert -1e-12 <= gain <= noise + 1e-12
    return {
        "noise": float(noise),
        "effective_noise": support - nominal_support,
        "distance_plane": distance_plane,
        "old_lower": old_lower,
        "new_lower": new_lower,
        "old_h": old_lower - r_safe,
        "new_h": new_lower - r_safe,
        "gain": gain,
        "directions": rays.tolist(),
        "endpoint_ranges": ranges.tolist(),
    }


def save(path, value):
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def analyze_state(case, audit_state):
    params = SafetyFilterParams(**case["params"])
    p = np.asarray(case["position"])
    trace = case["trace"]
    result = {
        key: case[key]
        for key in [
            "cohort",
            "condition",
            "frame",
            "agent_id",
            "goal",
            "command",
            "positive_force",
            "selected_pusher",
        ]
    }
    result.update(
        admitted=trace["applied"],
        guard=trace["reason"],
        rows=[],
        systems={},
        params=case["params"],
        truth=audit_state["truth"],
    )
    assert result["admitted"] == audit_state["admitted"]
    if not result["admitted"]:
        if case["cohort"] == "candidate_64":
            result["unchanged_fallback"] = {
                key: case["matrix"][key] for key in ["A", "b", "kinds", "capability"]
            }
        return result
    inputs = case["inputs"]
    packets = [packet(value) for value in inputs["post_support_packets"]]
    support, demand = build_rows(
        p,
        packets,
        params,
        inputs["observation_time"],
        inputs["object_velocity"],
        inputs.get("boundary_point_velocities"),
    )
    assert support.ready and support.reason == trace["reason"]
    assert abs(demand - trace["demand_speed"]) < 1e-12
    matrix = case["matrix"]
    A, old_b = np.asarray(matrix["A"]), np.asarray(matrix["b"])
    mask = np.asarray(matrix["kinds"]) == "object"
    old_h = np.asarray(matrix["h"])
    np.testing.assert_array_equal(support.A, A[mask])
    np.testing.assert_array_equal(support.h, old_h)
    _, kept = features(
        p, trace["source_agents"], packets, params, inputs["observation_time"]
    )
    assert len(kept) == len(old_h) == len(audit_state["rows"])
    for index, feature in enumerate(kept):
        values = directional_lower(
            p,
            A[mask][index],
            feature["start"],
            feature["end"],
            feature["origin"],
            feature["structural"],
            feature["noise"],
            params.r_safe,
        )
        assert abs(values["distance_plane"] - feature["distance"]) < 1e-12
        assert abs(values["old_h"] - old_h[index]) < 1e-12
        np.testing.assert_allclose(
            feature["normal"], A[mask][index], rtol=0, atol=1e-12
        )
        own = feature["source_agent"] == case["agent_id"]
        if own:
            np.testing.assert_allclose(p, feature["origin"], rtol=0, atol=1e-12)
            rays = np.asarray(values["directions"])
            angle = math.acos(float(np.clip(rays[0] @ rays[1], -1, 1)))
            bound = feature["noise"] * (1 - math.cos(angle / 2))
            if feature["projection"] == 0 or feature["projection"] == 1:
                bound = 0.0
            assert values["gain"] <= bound + 1e-11
        else:
            bound = None
        result["rows"].append(
            {
                "row": index,
                "source_agent": feature["source_agent"],
                "own": own,
                "kind": feature["kind"],
                "start": feature["start"].tolist(),
                "end": feature["end"].tolist(),
                "origin": feature["origin"].tolist(),
                "position": p.tolist(),
                "normal": A[mask][index].tolist(),
                "structural": feature["structural"],
                "r_safe": params.r_safe,
                "projection": feature["projection"],
                "own_gain_bound": bound,
                **values,
            }
        )
    new_h = np.asarray([row["new_h"] for row in result["rows"]])
    old_uncapped = old_b.copy()
    new_uncapped = old_b.copy()
    old_uncapped[mask] = demand - params.gamma_obj * old_h + params.rho
    new_uncapped[mask] = demand - params.gamma_obj * new_h + params.rho
    safety = SafetyFilter(params)
    np.testing.assert_allclose(
        safety._cap_to_reachable(A[mask], old_uncapped[mask]),
        old_b[mask],
        rtol=0,
        atol=1e-12,
    )
    new_capped = old_b.copy()
    new_capped[mask] = safety._cap_to_reachable(A[mask], new_uncapped[mask])
    command = np.asarray(case["command"])
    for label, rhs in [
        ("old_capped", old_b),
        ("new_capped", new_capped),
        ("old_uncapped", old_uncapped),
        ("new_uncapped", new_uncapped),
    ]:
        np.testing.assert_array_equal(rhs[~mask], old_b[~mask])
        result["systems"][label] = {
            "A": A.tolist(),
            "b": rhs.tolist(),
            "kinds": matrix["kinds"],
            "speed_limit": params.max_speed,
            "goal": case["goal"],
            "saved_command_min_object_residual": float(
                np.min(A[mask] @ command - rhs[mask])
            ),
            **maximize_direction(A, rhs, params.max_speed, case["goal"]),
        }
    for view in ["capped", "uncapped"]:
        old, new = (result["systems"][name + "_" + view] for name in ["old", "new"])
        assert np.max(np.asarray(new["b"]) - old["b"]) <= 1e-11
        if "maximum_goal_velocity" in old and "maximum_goal_velocity" in new:
            assert new["maximum_goal_velocity"] >= old["maximum_goal_velocity"] - 1e-7
        expected = case.get("expected_" + view)
        if expected is not None:
            assert old["classification"] == expected["classification"]
            if "maximum_goal_velocity" in expected:
                assert (
                    abs(
                        old["maximum_goal_velocity"] - expected["maximum_goal_velocity"]
                    )
                    < 1e-6
                )
    result.update(
        demand_speed=demand,
        minimum_old_lower=float(np.min(old_h) + params.r_safe),
        minimum_new_lower=float(np.min(new_h) + params.r_safe),
    )
    if result["truth"] is not None:
        truth = result["truth"]["signed_distance"]
        result["old_lower_overestimates_truth"] = (
            result["minimum_old_lower"] > truth + 1e-7
        )
        result["new_lower_overestimates_truth"] = (
            result["minimum_new_lower"] > truth + 1e-7
        )
        result["true_safe_new_negative_h"] = bool(truth >= params.r_safe and min(new_h) < 0)
    return result


def summarize(states):
    admitted = [state for state in states if state["admitted"]]
    rows = [row for state in admitted for row in state["rows"]]
    out = {
        "states": len(states),
        "admitted": len(admitted),
        "rows": len(rows),
        "guards": dict(Counter(s["guard"] for s in states)),
        "systems": {},
        "transitions": {},
    }
    for name in ["old_capped", "new_capped", "old_uncapped", "new_uncapped"]:
        out["systems"][name] = dict(
            Counter(s["systems"][name]["classification"] for s in admitted)
        )
    for view in ["capped", "uncapped"]:
        out["transitions"][view] = dict(
            Counter(
                s["systems"]["old_" + view]["classification"]
                + " -> "
                + s["systems"]["new_" + view]["classification"]
                for s in admitted
            )
        )
    for own, label in [(True, "own"), (False, "neighbor")]:
        selected = [r for r in rows if r["own"] == own]
        out[label + "_rows"] = len(selected)
        out[label + "_max_gain_m"] = max((r["gain"] for r in selected), default=None)
    labeled = [s for s in admitted if s["truth"] is not None]
    if labeled:
        out.update(
            truth_labeled_states=len(labeled),
            old_lower_overestimates_truth=sum(
                s["old_lower_overestimates_truth"] for s in labeled
            ),
            new_lower_overestimates_truth=sum(
                s["new_lower_overestimates_truth"] for s in labeled
            ),
            true_safe_new_negative_h=sum(
                s["true_safe_new_negative_h"] for s in labeled
            ),
        )
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    out = parser.parse_args().out.resolve()
    assert out.is_relative_to(ROOT / "runs") and not out.exists()
    guard()
    frozen = study.check()
    folders = [GEOMETRY, CONTACT, DIRECTION, CANDIDATE, AUDIT]
    for folder in folders:
        assert read(folder / "outcome.json")["analysis_sha256"] == sha(
            folder / "analysis.json"
        )
    paths = [p for f in folders for p in [f / "analysis.json", f / "outcome.json"]]
    paths += [GEOMETRY / (c + "-packets.json") for c in study.CONDITIONS]
    paths += [CANDIDATE / (c + ".json") for c in study.CONDITIONS]
    paths += [
        DESIGN,
        Path(__file__),
        ROOT / "tests/test_r1_ray_direction_candidate.py",
        ROOT / "scripts/probe_post_support_contact_direction.py",
        ROOT / "scripts/probe_post_support_contact_packet_geometry.py",
        ROOT / "src/dbact/sensing/sensor.py",
        ROOT / "src/dbact/sensing/finite_support.py",
        ROOT / "src/dbact/safety/filter.py",
        ROOT / "src/dbact/safety/post_support.py",
    ]
    identity = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {p.relative_to(ROOT).as_posix(): sha(p) for p in paths},
        "runtime_source_sha256": frozen["frozen_source_sha256"],
        "protocol_sha256": sha(study.PROTOCOL),
        "environment": frozen["environment"],
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
    }
    out.mkdir(parents=True)
    for path in paths:
        target = out / "inputs_and_sources" / path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    save(out / "started.json", identity)
    started, states = time.perf_counter(), []
    try:
        contacts = {
            (r["condition"], e["saved_frame"], s["agent_id"]): s
            for r in read(CONTACT / "analysis.json")["results"]
            for e in r["events"]
            for s in e["states"]
        }
        directions = read(DIRECTION / "analysis.json")
        capabilities = {
            (s["condition"], s["frame"], s["agent_id"]): s for s in directions["states"]
        }
        audits = {
            (s["cohort"], s["condition"], s["frame"], s["agent_id"]): s
            for s in read(AUDIT / "analysis.json")["states"]
        }
        with threadpool_limits(limits=1):
            for condition in study.CONDITIONS:
                for c in read(GEOMETRY / (condition + "-packets.json"))["captures"]:
                    key = condition, c["frame"], c["agent_id"]
                    previous, capability = contacts[key], capabilities[key]
                    case = {
                        "cohort": "control_128",
                        "condition": condition,
                        "frame": c["frame"],
                        "agent_id": c["agent_id"],
                        "position": c["position"],
                        "params": c["parameters"],
                        "trace": previous["support"],
                        "matrix": previous["counterfactual_rows"],
                        "goal": directions["goals"][condition]["direction"],
                        "command": previous["original_command"],
                        "positive_force": capability["positive_force"],
                        "selected_pusher": False,
                        "inputs": {
                            "post_support_packets": c["packets"],
                            "observation_time": c["timestamp"],
                            "object_velocity": c["object_velocity"],
                            "boundary_point_velocities": c["boundary_point_velocities"],
                        },
                    }
                    if previous["support"]["applied"]:
                        case["expected_capped"] = capability["systems"]["finite_capped"]
                        case["expected_uncapped"] = capability["systems"][
                            "finite_uncapped"
                        ]
                    states.append(analyze_state(case, audits[("control_128",) + key]))
                data = read(CANDIDATE / (condition + ".json"))
                for item in data["states"]:
                    s = item["actual"]
                    key = condition, data["frame"], s["agent_id"]
                    case = {
                        "cohort": "candidate_64",
                        "condition": condition,
                        "frame": data["frame"],
                        "agent_id": s["agent_id"],
                        "position": s["position"],
                        "params": s["params"],
                        "trace": s["post_support"],
                        "matrix": s,
                        "goal": s["goal"],
                        "command": s["command"],
                        "positive_force": None,
                        "selected_pusher": item["selected_pusher"],
                        "inputs": s["inputs"],
                        "expected_capped": s["capability"],
                    }
                    states.append(analyze_state(case, audits[("candidate_64",) + key]))
                assert time.perf_counter() - started < 7200
                print(
                    json.dumps(
                        {"condition": condition, "processed_states": len(states)}
                    ),
                    flush=True,
                )
        controls = [s for s in states if s["cohort"] == "control_128"]
        candidates = [s for s in states if s["cohort"] == "candidate_64"]
        force = [s for s in controls if s["positive_force"]]
        pushers = [s for s in candidates if s["selected_pusher"]]
        assert (len(controls), len(candidates), len(force), len(pushers)) == (
            128,
            64,
            17,
            4,
        )
        groups = {
            name: summarize(group)
            for name, group in [
                ("control_all", controls),
                ("control_positive_force", force),
                ("candidate_all", candidates),
                ("candidate_selected_pushers", pushers),
            ]
        }
        sufficient = {
            view: all(
                s["systems"]["new_" + view]["classification"] == "POSITIVE_CAPABILITY"
                for s in force + pushers
            )
            for view in ["capped", "uncapped"]
        }
        with (out / "all_feature_rows.csv").open(
            "x", encoding="utf-8", newline=""
        ) as handle:
            fields = [
                "cohort",
                "condition",
                "frame",
                "agent_id",
                "row",
                "source_agent",
                "own",
                "kind",
                "structural",
                "noise",
                "effective_noise",
                "old_h",
                "new_h",
                "gain",
                "own_gain_bound",
            ]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for state in states:
                for row in state["rows"]:
                    combined = {**state, **row}
                    writer.writerow({key: combined[key] for key in fields})
        flat = []
        for state in states:
            row = {
                k: state[k]
                for k in [
                    "cohort",
                    "condition",
                    "frame",
                    "agent_id",
                    "admitted",
                    "guard",
                    "positive_force",
                    "selected_pusher",
                ]
            }
            for label in ["old_capped", "new_capped", "old_uncapped", "new_uncapped"]:
                system = state["systems"].get(label, {})
                row[label] = system.get("classification", "UNSUPPORTED_UNCHANGED")
                row[label + "_goal_velocity"] = system.get("maximum_goal_velocity")
            flat.append(row)
        with (out / "all_states.csv").open("x", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(flat[0]))
            writer.writeheader()
            writer.writerows(flat)
        assert all(
            sha(ROOT / name) == digest for name, digest in identity["inputs"].items()
        )
        study.check()
        result = {
            **identity,
            "groups": groups,
            "sufficient_repair_on_fixed_force_and_pusher_states": sufficient,
            "states": states,
            "wall_seconds": time.perf_counter() - started,
            "scope": "Single empirical geometry candidate on reused states; no executed commands or task-success evidence.",
        }
        save(out / "analysis.json", result)
        save(
            out / "outcome.json",
            {
                **identity,
                "status": "COMPLETED_SINGLE_CANDIDATE_DIAGNOSTIC",
                "analysis_sha256": sha(out / "analysis.json"),
                "groups": groups,
                "sufficient_repair_on_fixed_force_and_pusher_states": sufficient,
                "wall_seconds": result["wall_seconds"],
            },
        )
        print(
            json.dumps(
                {
                    "groups": groups,
                    "sufficient": sufficient,
                    "wall_seconds": result["wall_seconds"],
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
                "traceback": traceback.format_exc(),
            },
        )
        raise


if __name__ == "__main__":
    main()
