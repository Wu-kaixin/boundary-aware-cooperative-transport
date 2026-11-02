"""Reconstruct historical sensor inputs and audit map motion, without new physics.

Saved poses are used ONLY by the original simulated sensor. Estimators receive
BoundaryView returns and permitted one-hop messages, never those saved poses.
The fused map snapshot is a validation target, not a fabricated raw scan.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MISSION = "2026-11-01-displacement-stop-v1"
HISTORY = ROOT / "runs/research/2026-11-01-boundary-navigation-v1/run"


def read(path):
    return json.loads(path.read_text(encoding="utf8"))


def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def audit(case_id, out):
    if Path(sys.prefix).resolve() != (ROOT/'.venv').resolve():
        raise RuntimeError('Use this project root .venv')
    # Run the historically frozen implementation with this project's current
    # root environment. Never revert the worktree to reproduce old inputs.
    if any(k == 'dbact' or k.startswith('dbact.') for k in sys.modules):
        raise RuntimeError('Run reconstruction as a fresh standalone process')
    snapshot = HISTORY / 'source_snapshot'
    manifest = read(HISTORY / 'manifest.json')
    for name, digest in manifest['source_files'].items():
        if sha(snapshot / name) != digest:
            raise RuntimeError(f'Historical source snapshot changed: {name}')
    sys.path.insert(0, str(snapshot / 'src'))
    from dbact.control.controller import _merge_scans
    from dbact.cpu_budget import limit_blas_threads
    from dbact.sensing.registration_capture import RegistrationCapture
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    from dbact_sim.experiments.study import cached_result
    limit_blas_threads(1)
    allowed = (ROOT / "artifacts/research" / MISSION).resolve()
    out = out.resolve()
    if not out.is_relative_to(allowed):
        raise ValueError("Output must remain in this mission's artifact directory")
    out.mkdir(parents=True, exist_ok=False)
    folder = HISTORY / case_id
    if folder.resolve().parent != HISTORY.resolve():
        raise ValueError('Expected one case inside the historical batch')
    receipt = read(folder / "case_result.json")
    cached_result(folder, receipt["identity"])
    # These exact modules generate observations, neighborhood, and map dynamics.
    sources = ["src/dbact/sensing/sensor.py", "src/dbact/sensing/boundary_map.py",
               "src/dbact/control/controller.py", "src/dbact/geometry.py",
               "src/dbact/provenance.py", "src/dbact/cargo.py"]
    for name in sources:
        if sha(snapshot / name) != manifest["source_files"][name]:
            raise RuntimeError(f"Historical reconstruction source changed: {name}")
    case = read(folder / "resolved_case.json")
    env = SimulationEnvironment(case["config"], seed=case["seed"])
    c, oid = env.controller, env.cargoes[0].object_id
    assert c.params.perception_every == 1 and not c.params.estimate_object_yaw
    c._ensure_state(env.agents)
    captures = {a.agent_id: RegistrationCapture(c.maps[a.agent_id], lambda: c._frame) for a in env.agents}
    steps = read(folder / "paired_metrics.json")["steps"]
    with np.load(folder / "replay.npz", allow_pickle=False) as f:
        saved = {k: f[k].copy() for k in f.files}
    n, na = len(steps), len(env.agents)
    raw, reg, fusion, velocities = [np.zeros((n, na, 2)) for _ in range(4)]
    quality = np.zeros((n, na), dtype=np.int8)
    matches, scan_counts = [np.zeros((n, na), dtype=np.int32) for _ in range(2)]
    conditioning = np.zeros((n, na))
    team, nav_team = np.zeros((n, 2)), np.zeros((n, 2))
    nav_remaining = np.zeros(n)
    neighbors_mask = np.zeros((n, na, na), dtype=bool)
    scans_flat = []
    offset = np.r_[0, np.cumsum(saved["sensed_counts"])]
    start = time.perf_counter()
    verified = dict(map_snapshot_frames=0, local_navigation_frames=0,
                    scalar_team_frames=0, shared_navigation_frames=0)
    for k in range(n):
        c._frame, c._time = k, float(saved["times"][k])
        env.cargoes[0].set_pose(saved[f"cargo/{oid}/centers"][k], saved[f"cargo/{oid}/angles"][k])
        for i, a in enumerate(env.agents):
            a.position = saved["agent_positions"][k, i].copy()
        neighbors = c._neighbor_indices(env.agents)
        scans = [c.sensor.sense_view(a, env.cargoes, c._time) for a in env.agents]
        for i, a in enumerate(env.agents):
            neighbors_mask[k, i, [i] + neighbors[i]] = True
            scan = scans[i]
            assert not len(scan) or set(scan.object_ids) == {oid}
            scan_counts[k, i] = len(scan)
            scans_flat.append(np.column_stack([scan.points, scan.normals, scan.confidence, scan.arc_length]))
            m = c.maps[a.agent_id]
            m.register(scan, env.dt)
            if m.carve_enabled:
                m.carve(a.position, scan)
            merged, codes = _merge_scans(scans, [i] + neighbors[i])
            m.update(merged, c._time, agent_codes=codes, dt=env.dt)
            c._views[a.agent_id] = m.view(c._time)
            raw[k, i] = m.object_displacement(oid)
            velocities[k, i] = m.object_velocity(oid)
            quality[k, i] = {"unavailable": 0, "degraded": 1, "credible": 2}[m.motion_quality.get(oid, "unavailable")]
            r = captures[a.agent_id].current(oid)
            f = captures[a.agent_id].current_fusion(oid)
            if r:
                reg[k, i] = r["translation"]
                matches[k, i], conditioning[k, i] = r["matches"], r["conditioning"]
            if f:
                fusion[k, i] = f["translation"]
        c._update_progress(env.agents, neighbors)
        c._update_navigation(env.agents, neighbors)
        actual = c.map_snapshot(env.map_snapshot_agent).points
        expected = saved["sensed_points"][offset[k+1]:offset[k+2]]
        if not np.array_equal(actual, expected):
            raise RuntimeError(f"Reconstruction map mismatch at frame {k}")
        verified["map_snapshot_frames"] += 1
        if c.team_progress[oid] != steps[k]["estimated_progress"]:
            raise RuntimeError(f"Scalar progress mismatch at frame {k}")
        verified["scalar_team_frames"] += 1
        for d in steps[k]["diagnostics"]:
            feedback = c._navigation[d["agent_id"]]
            if d["navigation_voters"]:
                assert feedback is not None
                assert "|".join(feedback.voters) == d["navigation_voters"]
                np.testing.assert_array_equal(feedback.displacement, d["navigation_displacement"])
                assert feedback.remaining == d["navigation_remaining"]
        verified["local_navigation_frames"] += 1
        if c.team_navigation:
            nav_team[k] = c.team_navigation.displacement
            nav_remaining[k] = c.team_navigation.remaining
            assert env.tasks[oid].distance - nav_remaining[k] == steps[k]["phase_guards"]["progress"]
            verified["shared_navigation_frames"] += 1
        else:
            nav_team[k] = np.nan; nav_remaining[k] = np.nan
        live = (quality[k] == 2) & (matches[k] > 0)
        team[k] = np.median(raw[k, live], axis=0) if live.any() else np.nan
        if k % 1000 == 0:
            print(f"Verified {k}/{n} frames, elapsed={time.perf_counter()-start:.1f}s", flush=True)
    residual = float(np.max(np.abs(raw - np.cumsum(reg + fusion, axis=0))))
    if residual > 1e-12:
        raise RuntimeError("Registration/fusion accounting failed")
    truth = saved[f"cargo/{oid}/centers"][:-1] - saved[f"cargo/{oid}/centers"][0]
    np.savez_compressed(out / "reconstruction.npz", raw=raw, registration=reg, fusion=fusion,
        velocity=velocities, quality=quality, matches=matches, conditioning=conditioning,
        team_raw=team, team_navigation=nav_team, remaining=nav_remaining,
        truth=truth, time=saved["times"][:-1], angle=saved[f"cargo/{oid}/angles"][:-1],
        neighbors=neighbors_mask, scan_counts=scan_counts, scans=np.vstack(scans_flat))
    live = (quality[-1] == 2) & (matches[-1] > 0)
    samples = []
    for seconds in (0, 10, 30, 60, 150, 300, 450, 599.95):
        k = min(n-1, int(round(seconds/env.dt)))
        samples.append(dict(time=float(saved["times"][k]), truth=truth[k].tolist(),
            raw_median=team[k].tolist(), controller_displacement=nav_team[k].tolist(),
            signed_error=(nav_team[k]-truth[k]).tolist(), remaining=float(nav_remaining[k])))
    result = dict(case=case_id, identity="RECONSTRUCTED_INPUTS_VERIFIED_AGAINST_HISTORICAL_OUTPUTS",
        verification=verified, frames=n, seconds=time.perf_counter()-start,
        accounting_max_residual=residual, selected_final_agents=np.flatnonzero(live).tolist(),
        final_truth=truth[-1].tolist(), final_shared_displacement=nav_team[-1].tolist(),
        final_shared_goal_error=float(np.linalg.norm(np.array([1.2,0.])-nav_team[-1])),
        final_per_agent=[dict(agent=a.agent_id,quality=int(quality[-1,i]),raw=raw[-1,i].tolist(),
            registration=reg[:,i].sum(axis=0).tolist(),fusion=fusion[:,i].sum(axis=0).tolist())
            for i,a in enumerate(env.agents)], milestones=samples,
        provenance=dict(input_receipt=sha(folder/'case_result.json'), products=receipt['products'],
            reconstruction_source={name:sha(snapshot/name) for name in sources},
            source_scope='Existing frozen execution snapshot, not the current worktree',script=sha(Path(__file__))))
    dump(out / "audit.json", result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('provenance','final_per_agent')}, indent=2))


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="c_open_away_vector_cvt")
    parser.add_argument("--out", type=Path, required=True)
    args=parser.parse_args()
    audit(args.case, args.out)
