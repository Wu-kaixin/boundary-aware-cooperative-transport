"""Scalar interval checks and independent Shapely geometry for the R1 audit."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

from research_io import ROOT, guard, read, sha
from shapely.geometry import LineString, Point, Polygon


def nearest_range(origin, direction, polygon, limit=1.2):
    endpoint = [origin[j] + limit * direction[j] for j in range(2)]
    intersection = LineString([origin, endpoint]).intersection(polygon.boundary)
    return None if intersection.is_empty else Point(origin).distance(intersection)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    folder, output = args.analysis_dir.resolve(), args.output.resolve()
    if output.exists() or not output.is_relative_to(ROOT / "runs"):
        raise ValueError("Use a new receipt under runs/")
    guard()
    started = time.perf_counter()
    data, outcome = read(folder / "analysis.json"), read(folder / "outcome.json")
    assert outcome["status"] == "COMPLETED_R1_MODEL_FEASIBILITY_DIAGNOSTIC"
    assert outcome["analysis_sha256"] == sha(folder / "analysis.json")
    for name, expected in data["inputs"].items():
        assert sha(ROOT / name) == expected
        assert sha(folder / "inputs_and_sources" / name) == expected
    states = {
        (s["cohort"], s["condition"], s["frame"], s["agent_id"]): s
        for s in data["states"]
    }
    assert len(states) == len(data["states"]) == 192
    checked = 0
    for state in states.values():
        if not state["admitted"]:
            assert not state["rows"]
            continue
        assert len(state["rows"]) == state["object_rows"]
        for row in state["rows"]:
            error = row["structural"] + row["noise"]
            h = row["distance"] - error - row["r_safe"]
            width = (
                row["robot_radius"] - row["r_safe"] - 2 * error - row["rho_over_gamma"]
            )
            assert abs(h - row["h"]) < 1e-12
            assert abs(width - row["reserved_width"]) < 1e-12
            assert row["robust_reserved_contact_possible"] == (width > 1e-12)
            assert abs(math.hypot(*row["normal"]) - 1.0) < 1e-12
            checked += 1
    assert checked == 3813 + 1965
    lookup = {
        (r["condition"], r["frame"], r["source"], r["point_index"]): r
        for r in data["noise_rows"]
    }
    packets, checked_returns, outside, largest_error, geometry_diff = (
        set(),
        0,
        0,
        0.0,
        0.0,
    )
    for name in data["inputs"]:
        if not name.endswith("-packets.json"):
            continue
        source = read(ROOT / name)
        for capture in source["captures"]:
            poly = Polygon(capture["evaluation_precommand_vertices"])
            assert poly.is_valid
            point = Point(capture["position"])
            signed = point.distance(poly.boundary) * (-1 if poly.contains(point) else 1)
            state = states[
                "control_128",
                source["condition"],
                capture["frame"],
                capture["agent_id"],
            ]
            geometry_diff = max(
                geometry_diff, abs(signed - state["truth"]["signed_distance"])
            )
            assert geometry_diff < 1e-12
            for aid, packet in zip(capture["source_agents"], capture["packets"]):
                pkey = source["condition"], capture["frame"], aid
                if pkey in packets:
                    continue
                packets.add(pkey)
                for index, p in enumerate(packet["points"]):
                    delta = [p[j] - packet["origin"][j] for j in range(2)]
                    length = math.hypot(*delta)
                    true_range = nearest_range(
                        packet["origin"], [v / length for v in delta], poly
                    )
                    previous = lookup[pkey + (index,)]
                    assert (
                        true_range is not None and previous["range_error"] is not None
                    )
                    error = length - true_range
                    geometry_diff = max(
                        geometry_diff, abs(error - previous["range_error"])
                    )
                    assert geometry_diff < 1e-12
                    is_outside = abs(error) > packet["noise_allowance"] + 1e-12
                    assert is_outside == previous["outside_allowance"]
                    outside += is_outside
                    checked_returns += 1
                    largest_error = max(largest_error, abs(error))
    assert checked_returns == len(lookup) == 2690 and outside == 10
    assert len(packets) == data["noise_summary"]["unique_packets"] == 121
    example = data["counterexample"]
    synthetic_gap, ray_max_diff = [], 0.0
    for vertices in [example["flat_vertices"], example["hidden_vertices"]]:
        poly = Polygon(vertices)
        assert poly.is_valid and not poly.contains(Point(example["origin"]))
        synthetic_gap.append(Point(example["origin"]).distance(poly.boundary))
        for direction, expected in zip(example["directions"], example["ray_ranges"]):
            value = nearest_range(example["origin"], direction, poly)
            assert (value is None) == (expected is None)
            if value is not None:
                ray_max_diff = max(ray_max_diff, abs(value - expected))
    assert ray_max_diff < 1e-12
    assert abs(synthetic_gap[0] - 0.1) < 1e-12 and abs(synthetic_gap[1] - 0.02) < 1e-12
    # The added triangle lies strictly in the open wedge between rays 0 and 1.
    origin = example["origin"]
    triangle = example["hidden_vertices"][4:]
    wedge_angles = [math.atan2(p[1] - origin[1], p[0] - origin[0]) for p in triangle]
    assert all(0 < a < 2 * math.pi / 72 for a in wedge_angles)
    geometry_file = next(
        name
        for name in data["inputs"]
        if name.endswith("packet-geometry-v1/analysis.json")
    )
    limiting = []
    for original in read(ROOT / geometry_file)["states"]:
        if not original["positive_force"]:
            continue
        state = states[
            "control_128",
            original["condition"],
            original["frame"],
            original["agent_id"],
        ]
        row = state["rows"][original["limiting_row"]]
        assert row["reserved_width"] < 0
        limiting.append(
            {
                "condition": original["condition"],
                "frame": original["frame"],
                "agent_id": original["agent_id"],
                "row": row["row"],
                "structural": row["structural"],
                "interval_width": 2 * row["hypothetical_error"],
                "reserved_width": row["reserved_width"],
            }
        )
    assert len(limiting) == 17
    # Additional source-led counterexample to transferring a continuous formula
    # to robot-first/object-second splitting. No dynamics engine is instantiated.
    h, dt, u, v, gamma, rho = 0.001, 0.05, -0.1, -0.2, 8.0, 0.02
    split = {
        "h": h,
        "dt": dt,
        "robot_u": u,
        "boundary_v": v,
        "gamma": gamma,
        "rho": rho,
        "continuous_relative_lhs": u - v,
        "continuous_rhs": -gamma * h + rho,
        "after_robot_commit_h": h + dt * u,
        "after_boundary_move_h": h + dt * (u - v),
        "split_template_required_robot_u": max(0.0, v) - gamma * h + rho,
        "scope": "Rejects strict continuous-to-split safety transfer; not a historical metric violation",
    }
    assert split["continuous_relative_lhs"] >= split["continuous_rhs"]
    assert split["after_robot_commit_h"] < 0 and split["after_boundary_move_h"] > 0
    result = {
        "status": "VERIFIED_R1_MODEL_DIAGNOSTIC",
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_sha256": sha(folder / "analysis.json"),
        "verifier_sha256": sha(__file__),
        "states": len(states),
        "scalar_feature_rows": checked,
        "independent_unique_packets": len(packets),
        "independent_ray_returns": checked_returns,
        "actual_noise_exceedances": outside,
        "maximum_absolute_range_error": largest_error,
        "maximum_independent_geometry_difference": geometry_diff,
        "synthetic_ray_checks": 144,
        "synthetic_maximum_range_difference": ray_max_diff,
        "synthetic_signed_distances": synthetic_gap,
        "triangle_angles_radians": wedge_angles,
        "preselected_limiting_positive_force_rows": limiting,
        "split_step_counterexample": split,
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
        "wall_seconds": time.perf_counter() - started,
        "scope": "Independent Shapely/scalar paths; not a held-out experiment or formal floating-point proof",
    }
    with output.open("x", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write("\n")
    print(
        json.dumps(
            {
                k: v
                for k, v in result.items()
                if k != "preselected_limiting_positive_force_rows"
            }
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
