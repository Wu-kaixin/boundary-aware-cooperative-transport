"""Read frozen census episodes; derive task-axis errors without new simulation.

Only final post-physics positions are used for endpoint decomposition. The
previous-control estimate in replay is deliberately not treated as simultaneous.
All receipt products are hashed before and after analysis. Output is new-only.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from dbact_sim.scenarios import controller_params_from_config


ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def inside(path: Path, parent: Path) -> Path:
    path = path.resolve()
    if not path.is_relative_to(parent.resolve()):
        raise ValueError(f"Path leaves allowed directory: {path}")
    return path


def steering_diagnostic(case: Path) -> dict:
    """Derive, not directly observe, steering from the invertible map EWMA.

    Applicable only to this frozen single-object, every-frame perception census.
    .05 m is a post-hoc descriptive stratum, not a new control/success threshold.
    Contact normals are truth used exclusively for this independent analysis.
    """
    cfg = json.loads((case / "config.json").read_text(encoding="utf-8"))
    params = controller_params_from_config(cfg)
    if params.perception_every != 1 or params.theorem_mode:
        raise ValueError("Motion reconstruction assumptions do not hold")
    dt = float(cfg["dt"])
    alpha = float(np.clip(params.object_velocity_filter, 0, 1))
    if dt <= 0 or alpha <= 0:
        raise ValueError("Motion recurrence is not invertible")
    records = json.loads((case / "paired_metrics.json").read_text(encoding="utf-8"))["steps"]
    frames = np.array([r["frame"] for r in records], dtype=int)
    if not np.array_equal(frames, np.arange(1, len(records) + 1)):
        raise ValueError("Missing control frame")
    ids = sorted(records[0]["estimated_velocities"])
    if any(sorted(r["estimated_velocities"]) != ids for r in records):
        raise ValueError("Agent identities changed")
    velocity = np.array([[r["estimated_velocities"][aid] for aid in ids] for r in records])
    previous = np.concatenate([np.zeros((1, len(ids), 2)), velocity[:-1]])
    displacement = np.cumsum(dt / alpha * (velocity - (1 - alpha) * previous), axis=0)
    with np.load(case / "replay.npz", allow_pickle=False) as replay:
        center = replay["cargo/cargo_0/centers"].copy()
        start = replay["task/cargo_0/start"].copy()
        goal = replay["task/cargo_0/direction"].copy()
    lateral = np.array([-goal[1], goal[0]])
    offset = (center[frames - 1] - start) @ lateral
    estimated = displacement @ lateral
    correction = -(params.cross_track_gain * estimated + params.cross_track_damping * (velocity @ lateral))
    correction[np.abs(estimated) < params.cross_track_deadband] = 0
    limit = np.tan(np.deg2rad(params.cross_track_max_deg))
    capped = np.clip(correction, -limit, limit)
    steered = capped / np.sqrt(1 + capped * capped)
    diagnostics = [{d["agent_id"]: d for d in r["diagnostics"]} for r in records]
    push = np.array([[ds[aid]["push_side"] for aid in ids] for ds in diagnostics], dtype=bool)
    eligible = (np.abs(offset) > .05)[:, None] & push
    counts = {"selected_pusher_samples": int(eligible.sum()),
              "steering_opposes_offset": int(((steered * offset[:, None] < 0) & eligible).sum()),
              "steering_same_sign_as_offset": int(((steered * offset[:, None] > 0) & eligible).sum()),
              "steering_zero": int(((steered == 0) & eligible).sum()),
              "steering_at_cap": int(((np.abs(correction) >= limit) & eligible).sum()),
              "eligible_frames": 0, "no_selected_pusher_contact_frames": 0,
              "no_opposing_selected_pusher_normal_frames": 0,
              "no_opposing_any_contact_normal_frames": 0,
              "eligible_frames_with_any_contact": 0,
              "all_contact_friction_cones_lack_corrective_component_frames": 0}
    mu = float(cfg["transport"]["friction"])
    for k, row in enumerate(records):
        chosen = {aid for aid, d in diagnostics[k].items() if d["push_side"]}
        if abs(offset[k]) <= .05 or not chosen:
            continue
        counts["eligible_frames"] += 1
        normals = np.array(row["contact_normals"], dtype=float).reshape(-1, 2)
        opposing = ((-normals @ lateral) * offset[k] < -1e-6)
        active = np.array([aid in chosen for aid in row["contact_ids"]], dtype=bool)
        counts["no_selected_pusher_contact_frames"] += int(not active.any())
        counts["no_opposing_selected_pusher_normal_frames"] += int(not (active & opposing).any())
        counts["no_opposing_any_contact_normal_frames"] += int(not opposing.any())
        if len(normals):
            counts["eligible_frames_with_any_contact"] += 1
            # Maximum corrective component per unit normal force over Coulomb
            # tangential force. A positive value is necessary, not sufficient.
            corrective = -np.sign(offset[k]) * lateral
            tangents = np.column_stack([-normals[:, 1], normals[:, 0]])
            support = (-normals) @ corrective + mu * np.abs(tangents @ corrective)
            counts["all_contact_friction_cones_lack_corrective_component_frames"] += int(np.all(support <= 1e-6))
    # Five seconds is a declared retrospective window, not a changed stop rule.
    window_frames = min(len(records), int(round(5.0 / dt)))
    first_frame = len(records) - window_frames + 1
    substeps = json.loads((case / "physics_substeps.json").read_text(encoding="utf-8"))
    late_substeps = [r for r in substeps if r["frame"] >= first_frame]
    factors = json.loads((case / "resolved_case.json").read_text(encoding="utf-8"))["factors"]
    mass, inertia = float(factors["mass_kg"]), float(factors["inertia_kg_m2"])
    friction_acceleration = float(cfg["transport"]["ground_friction"]) * float(cfg["transport"]["gravity"])
    late_selected_offsets = estimated[-window_frames:][push[-window_frames:]]
    late = {
        "control_frames": window_frames, "physics_substeps": len(late_substeps),
        "max_contact_force_norm_N": max(float(np.linalg.norm(r["force"])) for r in late_substeps),
        "model_from_rest_translation_friction_threshold_N": mass * friction_acceleration,
        "max_recorded_postphysics_speed_m_s": max(float(np.linalg.norm(r["true_velocity"])) for r in records[-window_frames:]),
        "phases": sorted({r["phase"] for r in records[-window_frames:]}),
        "final_prephysics_true_lateral_offset_m": float(offset[-1]),
        "selected_pusher_estimated_lateral_offset_quartiles_m": np.quantile(late_selected_offsets, [.25, .5, .75]).tolist() if late_selected_offsets.size else None,
        "all_episode_max_abs_substep_torque_Nm": max(abs(r["torque"]) for r in substeps),
        "model_from_rest_rotation_friction_threshold_Nm": friction_acceleration * np.sqrt(mass * inertia),
    }
    return {"case": case.relative_to(ROOT).as_posix(), "status": "DERIVED_NOT_DIRECTLY_LOGGED",
            "stratum_abs_true_lateral_gt_m": .05, "geometric_sign_tolerance": 1e-6,
            "angle_limit_deg": params.cross_track_max_deg, "dt": dt, "alpha": alpha,
            "counts": counts, "last_five_seconds": late}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    batch = inside(args.batch, ROOT / "runs")
    out = inside(args.out, ROOT / "runs" / "research")
    if out.exists() or out.is_relative_to(batch) or batch.is_relative_to(out):
        raise ValueError("Output must be new and disjoint from the frozen batch")
    manifest_path = batch / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    receipts = sorted(batch.glob("*/case_result.json"))
    if len(receipts) != manifest["cases"]:
        raise ValueError("Missing case receipts; do not silently drop failures")
    if args.dry_run:
        print(json.dumps({"cases": len(receipts), "output": str(out), "write": False}))
        return

    checked = {manifest_path: digest(manifest_path)}
    for relative in ("src/dbact/sensing/boundary_map.py", "src/dbact/control/controller.py",
                     "src/dbact/control/transport.py", "src/dbact/contact_dynamics.py",
                     "src/dbact_sim/scenarios.py"):
        path = ROOT / relative
        checked[path] = digest(path)
        if checked[path] != manifest["source_files"][relative]:
            raise ValueError(f"Analysis implementation differs from execution source: {relative}")
    rows = []
    steering = []
    for receipt_path in receipts:
        checked[receipt_path] = digest(receipt_path)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        case = receipt_path.parent
        for relative, expected in receipt["products"].items():
            path = inside(case / relative, case)
            actual = digest(path)
            if actual != expected:
                raise ValueError(f"Frozen product mismatch: {path}")
            checked[path] = actual
        resolved = json.loads((case / "resolved_case.json").read_text(encoding="utf-8"))
        result = receipt["result"]
        cid, task = next(iter(result["task"].items()))
        with np.load(case / "replay.npz", allow_pickle=False) as replay:
            centers = replay[f"cargo/{cid}/centers"].copy()
            angles = replay[f"cargo/{cid}/angles"].copy()
            frames = len(replay["times"]) - 1
        direction = np.asarray(task["direction"], dtype=float)
        if not np.isclose(np.linalg.norm(direction), 1.0, atol=1e-12, rtol=0):
            raise ValueError("Task direction must already be normalized")
        perpendicular = np.array([-direction[1], direction[0]])
        endpoint = centers[-1] - np.asarray(task["goal_point"])
        parallel = float(endpoint @ direction)
        lateral = float(endpoint @ perpendicular)
        distance = float(np.linalg.norm(endpoint))
        if not np.isclose(distance, result["euclidean_endpoint_error"], atol=1e-12, rtol=0):
            raise ValueError("Replay endpoint and original result disagree")
        if not np.isclose(distance**2, parallel**2 + lateral**2, atol=1e-12, rtol=0):
            raise ValueError("Orthogonal endpoint decomposition failed")
        if frames != result["termination"]["frames_run"]:
            raise ValueError("Frame count mismatch")
        tolerance = float(task["tolerance"])
        completed = bool(result["settled"] and distance <= tolerance)
        if completed != result["task_completed"]:
            raise ValueError("Original task verdict disagrees; do not reclassify")
        rows.append({
            "condition": resolved["condition_id"], "method": resolved["variant"],
            "seed": resolved["seed"], "completed": completed,
            "settled": result["settled"], "g500": result["g500_pass"],
            "frames": frames, "endpoint_m": distance,
            "signed_longitudinal_endpoint_error_m": parallel,
            "signed_lateral_endpoint_error_m": lateral,
            "lateral_fraction_squared_endpoint_error": lateral**2 / distance**2 if distance else 0.0,
            "longitudinal_exceeds_original_tolerance": abs(parallel) > tolerance,
            "lateral_exceeds_original_tolerance": abs(lateral) > tolerance,
            "rotation_excursion_deg": float(np.max(np.abs(angles - angles[0])) * 180 / np.pi),
            "case": case.relative_to(ROOT).as_posix(),
        })
        steering.append(steering_diagnostic(case))
    for path, expected in checked.items():
        if digest(path) != expected:
            raise ValueError(f"Input changed during analysis: {path}")

    paired = []
    for condition in sorted({r["condition"] for r in rows}):
        group = {r["method"]: r for r in rows if r["condition"] == condition}
        if len(group) != 3:
            raise ValueError("Incomplete method pairing")
        for other in ("local_spacing", "nearest_observed_target"):
            a, b = group["boundary_cvt"], group[other]
            paired.append({"condition": condition, "comparison": "boundary_cvt-minus-" + other,
                           "endpoint_difference_m": a["endpoint_m"] - b["endpoint_m"],
                           "completion_difference": int(a["completed"]) - int(b["completed"])})
    summary = {"additional_episodes": 0, "cases": len(rows), "checked_input_files": len(checked),
               "source_integrity": "PASS", "analysis_script_sha256": digest(Path(__file__)),
               "scope": "post-hoc development description; one shared seed, no population inference",
               "interpretation": "error decomposition is geometry, not a causal attribution; no feedback estimate alignment claimed",
               "inputs": {p.relative_to(ROOT).as_posix(): h for p, h in checked.items()},
               "rows": rows, "paired": paired, "steering": steering,
               "steering_limitations": [
                   "Derived displacement and steering; not directly recorded in the census.",
                   "Each agent map starts at zero, is not reset, and commits exactly once per perception frame.",
                   "Estimates align to replay[f-1]; pusher decisions are before command, actual contacts after robot motion before first cargo physics substep.",
                   "Correct sign does not establish accurate magnitude or appropriate derivative control.",
                   "Missing opposing normals does not exclude corrective tangential friction; friction-cone support is reported separately.",
                   "A friction cone with positive corrective support need not supply sufficient forward force, acceptable torque, or reachable commands.",
                   "Samples/frames are correlated descriptive diagnostics, never independent transport trials.",
               ]}
    out.mkdir(parents=True, exist_ok=False)
    (out / "analysis.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for name, records in (("endpoint_decomposition.csv", rows), ("paired_differences.csv", paired)):
        with (out / name).open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    print(json.dumps({k: summary[k] for k in ("additional_episodes", "cases", "checked_input_files", "source_integrity")}))


if __name__ == "__main__":
    main()
