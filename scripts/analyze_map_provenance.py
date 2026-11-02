"""Reconstruct a known ghost's complete ancestry since its last fresh support."""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import sys

import numpy as np

from research_io import ROOT, guard, read, sha


def analyze(root: Path, frame: int):
    guard()
    identity = read(root / "trace/frozen_trace.json")
    outcome = read(root / "trace/outcome.json")
    path = root / "trace/map_stages.jsonl.gz"
    assert outcome["status"] == "EXACT_RECONSTRUCTION" and sha(path) == outcome["product_sha256"]
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        target = next(json.loads(line) for line in stream if json.loads(line)["frame"] == frame)
    state = target["after_update"]
    point_index = int(np.argmin(np.linalg.norm(np.array(state["points"]) - target["position"], axis=1)))
    point = np.array(state["points"][point_index])
    last_support = state["timestamps"][point_index]
    history = {}
    stages = ("before_register", "registered_before_rekey", "after_register", "after_carve", "after_update")
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record["frame"] > frame:
                break
            if record["time"] < last_support - 1e-9:
                continue
            for stage in stages:
                if stage in record:
                    record[stage] = {key: np.asarray(value) for key, value in record[stage].items()}
            record.pop("one_hop_scan", None)
            record.pop("own_scan", None)
            history[record["frame"]] = record
    first = min(history)
    np.testing.assert_allclose(history[first]["time"], last_support, rtol=0, atol=1e-9)
    plan = read(ROOT / "experiments/ccta2027/acquisition_geometry_v1.json")
    case = next(case for case in plan["cases"] if case["case_id"] == identity["case_id"])
    voxel = case["config"]["controller"]["voxel_size"]
    mass = {int(state["keys"][point_index]): 1.0}
    motion = np.zeros(2)
    steps, collisions = [], 0
    max_residual = 0.0

    def at(stage, key):
        index = np.searchsorted(stage["keys"], key)
        assert index < len(stage["keys"]) and stage["keys"][index] == key
        return index

    for current in range(frame, first, -1):
        record = history[current]
        before, registered = record["before_register"], record["after_register"]
        carved, updated = record["after_carve"], record["after_update"]
        shift = record["registrations"].get("cargo_0", {})
        translation = np.array(shift.get("translation", [0., 0.]))
        assert shift.get("rotation", 0.) == 0.
        points_shifted = before["points"] + translation
        cell = np.rint(points_shifted / voxel).astype(np.int64)
        shifted_keys = (cell[:, 0] + (1 << 20)) * (1 << 21) + cell[:, 1] + (1 << 20)
        prior_mass = {}
        for key, weight in mass.items():
            u, c, r = at(updated, key), at(carved, key), at(registered, key)
            # No later packet refreshed ANY ancestor: test, do not infer from age alone.
            np.testing.assert_array_equal(updated["points"][u], carved["points"][c])
            np.testing.assert_array_equal(carved["points"][c], registered["points"][r])
            assert updated["timestamps"][u] <= last_support + 1e-9
            parents = np.flatnonzero(shifted_keys == key)
            assert len(parents)
            shares = before["weights"][parents] / before["weights"][parents].sum()
            reconstructed = shares @ points_shifted[parents]
            residual = float(np.max(np.abs(reconstructed - registered["points"][r])))
            assert residual < 1e-12, (current, key, residual)
            max_residual = max(max_residual, residual)
            collisions += int(len(parents) > 1)
            for parent, share in zip(parents, shares):
                parent_key = int(before["keys"][parent])
                prior_mass[parent_key] = prior_mass.get(parent_key, 0.) + weight * float(share)
        total_mass = sum(mass.values())
        np.testing.assert_allclose(total_mass, 1., rtol=0, atol=1e-12)
        motion += total_mass * translation
        steps.append(dict(frame=current, time=record["time"], active_ancestors=len(mass),
                          registration_translation=translation.tolist(),
                          registration_quality=shift.get("quality"),
                          reference_status=record["diagnostic"]["reference_status"],
                          reference_displacement=record["diagnostic"]["reference_displacement"],
                          map_velocity=record["map_velocity"]))
        previous = history[current - 1]["after_update"]
        for key in prior_mass:
            np.testing.assert_array_equal(before["points"][at(before, key)], previous["points"][at(previous, key)])
        mass = prior_mass

    initial = history[first]["after_update"]
    indices = np.array([at(initial, key) for key in mass])
    weights = np.array(list(mass.values()))
    ancestors = initial["points"][indices]
    anchor = weights @ ancestors
    reconstructed = anchor + motion
    np.testing.assert_allclose(reconstructed, point, rtol=0, atol=1e-12)
    start = history[first]
    displacement = np.array(target["true_center"]) - start["true_center"]
    angle = target["true_angle"] - start["true_angle"]
    cosine, sine = np.cos(angle), np.sin(angle)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    material = (ancestors - start["true_center"]) @ rotation.T + target["true_center"]
    assert start["diagnostic"]["reference_status"] == target["diagnostic"]["reference_status"] == "accepted"
    reference_shift = (np.array(target["diagnostic"]["reference_displacement"])
                       - start["diagnostic"]["reference_displacement"])
    reference_point = anchor + reference_shift

    # Reconstruct a fresh raw sensor packet independently at this saved pose.
    # No controller step; only its original simulator sensor consumes geometry.
    sys.path.insert(0, str(ROOT / plan["run_dir"] / "source_snapshot/src"))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    env = SimulationEnvironment(case["config"], seed=case["seed"])
    cargo = env.cargoes[0]
    cargo.set_pose(target["true_center"], target["true_angle"])
    agent = next(a for a in env.agents if a.agent_id == identity["agent_id"])
    agent.position = np.array(target["position"])
    view = env.controller.sensor.sense_view(agent, env.cargoes, target["time"])
    np.testing.assert_array_equal(view.points, target["own_scan"]["points"])
    np.testing.assert_array_equal(view.normals, target["own_scan"]["normals"])
    returns = env.controller.sensor.raw_returns(agent, env.cargoes, target["time"])
    bearing = float(np.arctan2(*(point - agent.position)[::-1]))
    hit_angles = np.arctan2(*(view.points - agent.position).T[::-1])
    periodic = np.abs(np.angle(np.exp(1j * (hit_angles - bearing))))
    count = env.controller.sensor.params.ray_count
    ray_angles = np.linspace(0, 2 * np.pi, count, endpoint=False)
    nearest_ray = int(np.argmin(np.abs(np.angle(np.exp(1j * (ray_angles - bearing))))))
    raw = next((item for item in returns if item["ray_index"] == nearest_ray), None)
    clearance = float(np.linalg.norm(point - agent.position))
    output = dict(schema="map-provenance-v1", scope="one selected known far-C failure, no physics or online intervention",
        case_id=identity["case_id"], agent_id=identity["agent_id"], selected_frame=frame,
        trace_sha256=sha(path), analyzer_sha256=sha(__file__), verified_trace_frames=outcome["verified_frames"],
        selected_point=point.tolist(), current_true_boundary_distance_m=float(abs(cargo.signed_distance(point[None, :])[0][0])),
        last_fresh_support_frame=first, last_fresh_support_time=last_support,
        selected_time=target["time"], unsupported_age_seconds=target["time"] - last_support,
        confidence_before_age=state["confidence"][point_index],
        confidence_after_age=state["confidence"][point_index] * np.exp(-case["config"]["controller"]["age_decay"] * (target["time"] - last_support)),
        no_fusion_or_carving_on_lineage_after_support=True, lineage_collision_count=collisions,
        ancestor_points=ancestors.tolist(), ancestor_weights=weights.tolist(), ancestor_count=len(ancestors),
        ancestor_true_signed_distances=initial["true_signed_distance"][indices].tolist(),
        weighted_anchor_point=anchor.tolist(), accumulated_registration_translation=motion.tolist(),
        reconstruction_error_m=float(np.linalg.norm(reconstructed - point)), max_parent_reconstruction_residual=max_residual,
        true_center_displacement=displacement.tolist(), true_angle_change_degrees=float(np.degrees(angle)),
        reference_displacement_change=reference_shift.tolist(),
        reference_repositioned_point=reference_point.tolist(),
        reference_repositioned_boundary_distance_m=float(abs(cargo.signed_distance(reference_point[None, :])[0][0])),
        true_material_repositioned_point=(weights @ material).tolist(),
        old_point_error_to_material_m=float(np.linalg.norm(point - weights @ material)),
        reference_point_error_to_material_m=float(np.linalg.norm(reference_point - weights @ material)),
        sensor=dict(ray_count=count, raw_hit_count=len(returns), accepted_hit_count=len(view),
            complete_raw_scan_reproduces_saved_view=True, ghost_range=clearance,
            nearest_accepted_hit_bearing_gap_rad=float(periodic.min()), existing_aperture_rad=0.05 / clearance,
            nearest_calibrated_ray_index=nearest_ray, nearest_calibrated_ray_hit=raw is not None,
            nearest_calibrated_ray_measured_range=None if raw is None else raw["range"],
            nearest_calibrated_ray_bearing_gap_rad=float(abs(np.angle(np.exp(1j * (ray_angles[nearest_ray] - bearing))))),
            note="A no-hit ray is a free segment, not a certified free angular sector for arbitrary unknown shapes."),
        steps=steps[::-1])
    destination = root / "ancestry.json"
    if destination.exists():
        raise FileExistsError(destination)
    dump(destination, output)
    print(json.dumps({key: value for key, value in output.items() if key not in ("steps", "ancestor_points", "ancestor_weights", "ancestor_true_signed_distances")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=917)
    args = parser.parse_args()
    analyze(args.root, args.frame)
