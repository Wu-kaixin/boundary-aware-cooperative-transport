"""Independent scalar vertex/support and capability-certificate verification."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from research_io import ROOT, guard, read, sha
from verify_post_support_onsets import certificate


def dot(a, b):
    return math.fsum(x * y for x, y in zip(a, b))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--failed-attempt-dir", type=Path)
    args = parser.parse_args()
    folder, output = args.analysis_dir.resolve(), args.output.resolve()
    assert output.is_relative_to(ROOT / "runs") and not output.exists()
    guard()
    started = time.perf_counter()
    data, outcome = read(folder / "analysis.json"), read(folder / "outcome.json")
    assert outcome["status"] == "COMPLETED_SINGLE_CANDIDATE_DIAGNOSTIC"
    assert outcome["analysis_sha256"] == sha(folder / "analysis.json")
    for name, expected in data["inputs"].items():
        assert sha(ROOT / name) == expected, name
        assert sha(folder / "inputs_and_sources" / name) == expected, name
    audit_path = next(
        n
        for n in data["inputs"]
        if n.endswith("model-feasibility-v1/calculation-v1/analysis.json")
    )
    before = {
        (s["cohort"], s["condition"], s["frame"], s["agent_id"]): s
        for s in read(ROOT / audit_path)["states"]
    }
    controls, candidates = {}, {}
    for name in data["inputs"]:
        if name.endswith("-packets.json"):
            for c in read(ROOT / name)["captures"]:
                condition = Path(name).name.removesuffix("-packets.json")
                controls[condition, c["frame"], c["agent_id"]] = c
        elif "first-pusher-loss-v3/" in name and Path(name).name not in [
            "analysis.json",
            "outcome.json",
        ]:
            d = read(ROOT / name)
            for item in d["states"]:
                candidates[d["condition"], d["frame"], item["actual"]["agent_id"]] = (
                    item["actual"]
                )
    certificates, maximum_support_difference, checked_rows = [], 0.0, 0
    by_key = {}
    for state in data["states"]:
        key = state["cohort"], state["condition"], state["frame"], state["agent_id"]
        assert key not in by_key
        by_key[key] = state
        original = before[key]
        assert (state["admitted"], state["guard"]) == (
            original["admitted"],
            original["guard"],
        )
        if not state["admitted"]:
            assert not state["rows"] and not state["systems"]
            if state["cohort"] == "candidate_64":
                actual = candidates[key[1:]]
                for field, value in state["unchanged_fallback"].items():
                    assert value == actual[field]
            continue
        if state["cohort"] == "control_128":
            captured = controls[key[1:]]
            packets = dict(zip(captured["source_agents"], captured["packets"]))
        else:
            actual = candidates[key[1:]]
            packets = dict(
                zip(
                    actual["post_support"]["source_agents"],
                    actual["inputs"]["post_support_packets"],
                )
            )
        assert len(state["rows"]) == len(original["rows"])
        for row, old_row in zip(state["rows"], original["rows"]):
            for field in [
                "row",
                "source_agent",
                "kind",
                "structural",
                "noise",
                "normal",
                "r_safe",
            ]:
                assert row[field] == old_row[field], (key, field)
            assert abs(row["old_h"] - old_row["h"]) < 1e-12
            packet = packets[row["source_agent"]]
            assert row["origin"] == packet["origin"]
            first = packet["points"].index(row["start"])
            last = packet["points"].index(row["end"])
            assert packet["object_ids"][first] == packet["object_ids"][last]
            if row["kind"] == "chord":
                assert [first, last] in packet["chords"]
            else:
                assert first == last and all(
                    first not in pair for pair in packet["chords"]
                )
            vertices = []
            for point in [row["start"], row["end"]]:
                delta = [x - y for x, y in zip(point, row["origin"])]
                length = math.hypot(*delta)
                assert length > 0
                for sign in [-1, 1]:
                    vertices.append(
                        [
                            point[j] + sign * row["noise"] * delta[j] / length
                            for j in range(2)
                        ]
                    )
            support = max(dot(row["normal"], v) for v in vertices)
            lower = dot(row["normal"], row["position"]) - support - row["structural"]
            diff = abs(lower - row["new_lower"])
            maximum_support_difference = max(maximum_support_difference, diff)
            assert diff < 1e-12
            assert abs(lower - row["r_safe"] - row["new_h"]) < 1e-12
            assert -1e-12 <= row["gain"] <= row["noise"] + 1e-12
            assert abs(row["gain"] - (row["new_h"] - row["old_h"])) < 1e-12
            if row["own"]:
                assert row["position"] == row["origin"]
                assert (
                    row["gain"] <= row["noise"] * (1 - math.cos(math.pi / 72)) + 1e-11
                )
                if row["kind"] == "isolated_point" or row["projection"] in [0, 1]:
                    assert abs(row["gain"]) < 1e-12
            checked_rows += 1
        for system in state["systems"].values():
            certificates.append(
                certificate(
                    system["A"],
                    system["b"],
                    system["goal"],
                    system["speed_limit"],
                    system,
                )
            )
        for view in ["capped", "uncapped"]:
            old, new = (
                state["systems"][prefix + "_" + view] for prefix in ["old", "new"]
            )
            assert old["A"] == new["A"] and old["kinds"] == new["kinds"]
            assert (
                old["goal"] == new["goal"] and old["speed_limit"] == new["speed_limit"]
            )
            gains = iter(r["gain"] for r in state["rows"])
            for a, b, kind in zip(old["b"], new["b"], old["kinds"]):
                if kind != "object":
                    assert a == b
                else:
                    expected_drop = state["params"]["gamma_obj"] * next(gains)
                    assert -1e-11 <= a - b <= expected_drop + 1e-11
                    if view == "uncapped":
                        assert abs((a - b) - expected_drop) < 1e-11
            if "maximum_goal_velocity" in old and "maximum_goal_velocity" in new:
                assert (
                    new["maximum_goal_velocity"] >= old["maximum_goal_velocity"] - 1e-7
                )
    assert len(by_key) == 192 and checked_rows == 5778 and len(certificates) == 676
    controls_out = [s for s in by_key.values() if s["cohort"] == "control_128"]
    candidates_out = [s for s in by_key.values() if s["cohort"] == "candidate_64"]
    force = [s for s in controls_out if s["positive_force"]]
    pushers = [s for s in candidates_out if s["selected_pusher"]]
    assert (len(controls_out), len(candidates_out), len(force), len(pushers)) == (
        128,
        64,
        17,
        4,
    )
    for label, group in [
        ("control_all", controls_out),
        ("control_positive_force", force),
        ("candidate_all", candidates_out),
        ("candidate_selected_pushers", pushers),
    ]:
        summary = data["groups"][label]
        for system, expected in summary["systems"].items():
            assert (
                dict(
                    Counter(
                        s["systems"][system]["classification"]
                        for s in group
                        if s["admitted"]
                    )
                )
                == expected
            )
    for view in ["capped", "uncapped"]:
        sufficient = all(
            s["systems"]["new_" + view]["classification"] == "POSITIVE_CAPABILITY"
            for s in force + pushers
        )
        assert (
            sufficient
            == data["sufficient_repair_on_fixed_force_and_pusher_states"][view]
        )
    csv_identical = None
    if args.failed_attempt_dir is not None:
        first_attempt = args.failed_attempt_dir.resolve()
        assert first_attempt != folder
        csv_identical = {
            name: sha(folder / name) == sha(first_attempt / name)
            for name in ["all_states.csv", "all_feature_rows.csv"]
        }
        assert all(csv_identical.values())
    result = {
        "status": "VERIFIED_DIRECTIONAL_SUPPORT_AND_CAPABILITY",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_sha256": sha(folder / "analysis.json"),
        "verifier_sha256": sha(__file__),
        "certificate_helper_sha256": sha(
            ROOT / "scripts/verify_post_support_onsets.py"
        ),
        "states": 192,
        "scalar_vertex_rows": checked_rows,
        "explicit_vertices": 4 * checked_rows,
        "certificates": len(certificates),
        "classifications": dict(Counter(c["classification"] for c in certificates)),
        "maximum_support_difference_m": maximum_support_difference,
        "maximum_primal_dual_gap_m_per_s": max(c.get("gap", 0) for c in certificates),
        "minimum_primal_residual_m_per_s": min(
            c["minimum_residual"] for c in certificates if "minimum_residual" in c
        ),
        "failed_attempt_csv_bit_identical": csv_identical,
        "new_physical_episodes": 0,
        "control_replay_frames": 0,
        "wall_seconds": time.perf_counter() - started,
        "scope": "Independent scalar vertex enumeration and certificates; no empirical safety or task-success claim.",
    }
    with output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
