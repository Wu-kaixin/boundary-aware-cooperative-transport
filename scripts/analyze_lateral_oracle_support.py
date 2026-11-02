"""Same-input steering counterfactual on three frozen development trajectories.

This does not run physics, recover exact nominal/QP commands, or measure oracle
task performance. Original nearest observed normals and full nominal commands
were not logged. True contact normals must not be substituted for those inputs.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BATCH = ROOT / "runs/research/2026-10-31-multishape-census-v1/run"
CASES = ("circle_boundary_cvt", "c_open_away_boundary_cvt", "rectangle_parallel_boundary_cvt")


def sha(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to((ROOT / "runs/research").resolve()) or out.is_relative_to(BATCH.resolve()):
        raise ValueError("Output must stay inside runs/research and outside the frozen batch")
    if out.exists():
        raise FileExistsError(out)
    identity_path = ROOT / "docs/results/multishape_census_v1/identity.json"
    identity = json.loads(identity_path.read_text(encoding="utf-8"))
    checked = {identity_path: sha(identity_path)}
    for relative in ("src/dbact/control/controller.py", "src/dbact/sensing/boundary_map.py", "src/dbact/control/transport.py"):
        path = ROOT / relative
        checked[path] = sha(path)
        if checked[path] != identity["source_files"][relative]:
            raise ValueError(f"Execution source mismatch: {relative}")
    results = []
    for case in CASES:
        folder = BATCH / case
        receipt_path = folder / "case_result.json"
        checked[receipt_path] = sha(receipt_path)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        for name in ("paired_metrics.json", "replay.npz", "config.json"):
            checked[folder / name] = sha(folder / name)
            if checked[folder / name] != receipt["products"][name]:
                raise ValueError(f"Frozen product mismatch: {case}/{name}")
        cfg = json.loads((folder / "config.json").read_text(encoding="utf-8"))
        cp = cfg["controller"]
        rows = json.loads((folder / "paired_metrics.json").read_text(encoding="utf-8"))["steps"]
        with np.load(folder / "replay.npz", allow_pickle=False) as replay:
            ids = replay["agent_ids"].astype(str).tolist()
            center = replay["cargo/cargo_0/centers"].copy()
            goal = replay["task/cargo_0/direction"].copy()
            start = replay["task/cargo_0/start"].copy()
        if [r["frame"] for r in rows] != list(range(1, len(rows) + 1)) or len(center) != len(rows) + 1:
            raise ValueError("Missing frame")
        alpha, dt = cp["object_velocity_filter"], cfg["dt"]
        if alpha <= 0 or cp.get("perception_every", 1) != 1:
            raise ValueError("Unsupported motion reconstruction")
        velocity = np.array([[r["estimated_velocities"][a] for a in ids] for r in rows])
        previous = np.concatenate([np.zeros_like(velocity[:1]), velocity[:-1]])
        displacement = np.cumsum(dt / alpha * (velocity - (1 - alpha) * previous), axis=0)

        def steer(disp, vel):
            lateral = disp - np.sum(disp * goal, axis=-1, keepdims=True) * goal
            rate = vel - np.sum(vel * goal, axis=-1, keepdims=True) * goal
            correction = -cp["cross_track_gain"] * lateral - cp["cross_track_damping"] * rate
            magnitude = np.linalg.norm(correction, axis=-1, keepdims=True)
            limit = np.tan(np.deg2rad(cp["cross_track_max_deg"]))
            correction *= np.minimum(1, np.divide(limit, magnitude, out=np.ones_like(magnitude), where=magnitude > 0))
            correction = np.where((np.linalg.norm(lateral, axis=-1) < cp["cross_track_deadband"])[..., None], 0, correction)
            direction = goal + correction
            return direction / np.linalg.norm(direction, axis=-1, keepdims=True)

        true_displacement = center[:-1] - start
        true_velocity = np.array([[0., 0.]] + [r["true_velocity"] for r in rows[:-1]])
        original = steer(displacement, velocity)
        oracle = steer(true_displacement, true_velocity)[:, None, :]
        delta = np.abs(np.arctan2(original[..., 0] * oracle[..., 1] - original[..., 1] * oracle[..., 0], np.sum(original * oracle, axis=-1)))
        post = np.array([r["phase"] in ("CONTACT_READY", "TRANSPORT", "BRAKE", "HOLD") for r in rows])
        all_post = np.broadcast_to(post[:, None], delta.shape)
        diagnostics = [{d["agent_id"]: d for d in r["diagnostics"]} for r in rows]
        selected = np.array([[d[a]["push_side"] for a in ids] for d in diagnostics], dtype=bool) & all_post
        result = {"case": case, "provenance": "DERIVED_NOT_DIRECTLY_LOGGED", "post_frames": int(post.sum()), "groups": {}}
        for name, mask in (("all_post_agents_hypothetical", all_post), ("selected_push", selected)):
            radians = delta[mask]
            degrees = np.rad2deg(radians)
            result["groups"][name] = {"n": len(radians), "changed_gt_1e-12_rad": int(np.count_nonzero(radians > 1e-12)),
                                    "median_abs_delta_deg": float(np.median(degrees)),
                                    "p95_abs_delta_deg": float(np.percentile(degrees, 95)),
                                    "max_abs_delta_deg": float(np.max(degrees))}
        results.append(result)
    for path, expected in checked.items():
        if sha(path) != expected:
            raise ValueError(f"Input changed: {path}")
    report = {"new_physics_episodes": 0, "scope": "same-input function counterfactual on original trajectories, not closed-loop oracle results",
              "missing_for_exact_action_counterfactual": ["actual nearest observed normal", "full nominal command and mutable loop state"],
              "numerical_change_threshold_rad": 1e-12, "cases": results,
              "source_sha256": sha(Path(__file__)), "inputs": {p.relative_to(ROOT).as_posix(): h for p, h in checked.items()}}
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(results))


if __name__ == "__main__":
    main()
