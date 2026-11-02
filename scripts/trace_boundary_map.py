"""Trace map operations on an exact frozen controller replay, without new physics."""
from __future__ import annotations

import os
for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_key] = "1"

import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import sys
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from replay_qp_conflict import recorded


def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def trace(case_id: str, agent_id: str, end_frame: int, out: Path):
    guard()
    if any(k == "dbact" or k.startswith(("dbact.", "dbact_sim")) for k in sys.modules):
        raise RuntimeError("Use a fresh process for original frozen-source replay")
    protocol = ROOT / "experiments/ccta2027/acquisition_geometry_v1.json"
    plan = read(protocol)
    batch = ROOT / plan["run_dir"]
    snapshot = batch / "source_snapshot"
    for name, digest in plan["source_files"].items():
        assert sha(snapshot / name) == digest, name
    sys.path.insert(0, str(snapshot / "src"))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result, environment_versions

    assert environment_versions() == plan["environment"]
    manifest = read(batch / "manifest.json")
    assert manifest["identity"]["protocol_sha256"] == sha(protocol)
    assert manifest["source_files"] == plan["source_files"]
    case = next(c for c in plan["cases"] if c["case_id"] == case_id)
    folder = batch / case_id
    receipt = cached_result(folder, {**manifest["identity"], "case_sha256": encode(case)})
    assert receipt["execution_status"] == "COMPLETED"
    target = out.resolve()
    if not target.is_relative_to(ROOT / "runs"):
        raise ValueError("Use a new directory below runs")
    target.mkdir(parents=True, exist_ok=False)
    rows = read(folder / "paired_metrics.json")["steps"]
    assert 0 < end_frame <= len(rows)
    with np.load(folder / "replay.npz", allow_pickle=False) as source:
        saved = {k: source[k].copy() for k in (
            "times", "agent_positions", "cargo/cargo_0/centers", "cargo/cargo_0/angles",
            "sensed_counts", "sensed_points")}
    offsets = np.r_[0, np.cumsum(saved["sensed_counts"])]
    env = SimulationEnvironment(case["config"], seed=case["seed"])
    controller = env.controller
    controller.trace_enabled = True
    assert controller.params.perception_every == controller.params.planning_every == 1
    controller._ensure_state(env.agents)
    index = next(i for i, agent in enumerate(env.agents) if agent.agent_id == agent_id)
    local = controller.maps[agent_id]
    frame_record = {}

    def map_state():
        # Truth is read only in this observer; never fed to any online operation.
        points = local._points.copy()
        distance = env.cargoes[0].signed_distance(points)[0] if len(points) else np.empty(0)
        return dict(points=points, normals=local._normals.copy(), keys=local._keys.copy(),
                    timestamps=local._timestamp.copy(), weights=local._weight.copy(),
                    confidence=local._confidence.copy(), arc=local._arc.copy(),
                    true_signed_distance=distance)

    def scan_state(scan):
        return dict(points=scan.points.copy(), normals=scan.normals.copy(),
                    confidence=scan.confidence.copy(), arc_length=scan.arc_length.copy())

    original_register, original_rekey = local.register, local._rekey
    original_carve, original_update = local.carve, local.update
    original_merge = local._merge

    def rekey():
        frame_record["registered_before_rekey"] = map_state()
        return original_rekey()

    def register(scan, dt):
        frame_record["before_register"] = map_state()
        frame_record["own_scan"] = scan_state(scan)
        result = original_register(scan, dt)
        frame_record["registrations"] = {name: asdict(value) for name, value in result.items()}
        frame_record["after_register"] = map_state()
        return result

    def carve(origin, scan):
        result = original_carve(origin, scan)
        frame_record["carved_cells"] = result
        frame_record["after_carve"] = map_state()
        return result

    def merge(*args, **kwargs):
        result = original_merge(*args, **kwargs)
        frame_record["fusion_drift"] = result
        return result

    def update(observations, timestamp, agent_codes=None, dt=0.0):
        frame_record["one_hop_scan"] = scan_state(observations)
        frame_record["source_codes"] = agent_codes.copy()
        result = original_update(observations, timestamp, agent_codes=agent_codes, dt=dt)
        frame_record["after_update"] = map_state()
        return result

    local.register, local._rekey, local.carve, local.update, local._merge = register, rekey, carve, update, merge
    identity = dict(case_id=case_id, agent_id=agent_id, end_frame=end_frame,
                    script_sha256=sha(__file__), protocol_sha256=sha(protocol),
                    source_sha256=plan["frozen_source_sha256"], environment=plan["environment"],
                    original_receipt_sha256=sha(folder / "case_result.json"),
                    new_physical_episodes=0)
    dump(target / "frozen_trace.json", identity)
    verified, current_frame = 0, 0
    started = time.perf_counter()
    try:
        with gzip.open(target / "map_stages.jsonl.gz", "wt", encoding="utf-8") as stream:
            for k in range(end_frame):
                current_frame = k + 1
                row = rows[k]
                env.cargoes[0].set_pose(saved["cargo/cargo_0/centers"][k], saved["cargo/cargo_0/angles"][k])
                for i, agent in enumerate(env.agents):
                    agent.position = saved["agent_positions"][k, i].copy()
                    agent.velocity = np.zeros(2) if k == 0 else np.array(rows[k - 1]["commands"][i])
                frame_record = dict(frame=current_frame, time=float(saved["times"][k]),
                                    position=env.agents[index].position.copy(),
                                    true_center=saved["cargo/cargo_0/centers"][k],
                                    true_angle=saved["cargo/cargo_0/angles"][k])
                commands = controller.step(env.agents, env.cargoes, float(saved["times"][k]), env.dt)
                np.testing.assert_array_equal([v.velocity for v in commands], row["commands"])
                assert recorded([vars(d) for d in controller.diagnostics]) == row["diagnostics"], ("diagnostics", current_frame)
                assert recorded(controller.safety.stats.as_dict()) == row["solver_statistics"], ("solver", current_frame)
                assert controller.phase.name == row["phase"] and recorded(vars(controller.phase_signals)) == row["phase_guards"]
                assert recorded(controller.phase_monitor.as_dict()) == row["phase_response"]
                np.testing.assert_array_equal(controller.map_snapshot(env.map_snapshot_agent).points,
                                              saved["sensed_points"][offsets[k + 1]:offsets[k + 2]])
                frame_record.update(phase=controller.phase.name, acquisition_status=controller.initial_reference_status,
                                    diagnostic=next(d for d in row["diagnostics"] if d["agent_id"] == agent_id),
                                    map_displacement=local.object_displacement("cargo_0"),
                                    map_velocity=local.object_velocity("cargo_0"))
                stream.write(json.dumps(plain(frame_record), separators=(",", ":"), allow_nan=False) + "\n")
                verified += 1
                if verified % 200 == 0:
                    print(json.dumps(dict(verified_frames=verified, case_id=case_id)), flush=True)
        dump(target / "outcome.json", dict(**identity, status="EXACT_RECONSTRUCTION", verified_frames=verified,
             elapsed_seconds=time.perf_counter() - started, product_sha256=sha(target / "map_stages.jsonl.gz")))
        print(json.dumps(dict(status="EXACT_RECONSTRUCTION", frames=verified, new_physical_episodes=0)), flush=True)
    except Exception as exc:
        dump(target / "failure.json", dict(**identity, status="INVALID_RECONSTRUCTION", verified_frames=verified,
                                          failing_frame=current_frame, error=repr(exc)))
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="far_c_open_away_pos0_safe_geometry")
    parser.add_argument("--agent", default="agent_05")
    parser.add_argument("--end-frame", type=int, default=933)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    trace(args.case, args.agent, args.end_frame, args.out)
