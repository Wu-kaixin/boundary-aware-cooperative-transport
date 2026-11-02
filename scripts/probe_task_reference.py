"""Read-only development probe of a pre-contact observed reference epoch."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

from dbact.control.controller import _merge_scans
from dbact.cpu_budget import limit_blas_threads
from dbact_sim.environment import SimulationEnvironment

ROOT = Path(__file__).resolve().parents[1]


def main():
    limit_blas_threads(1)
    out = ROOT / 'runs/maintenance/2026-11-02-task-reference-v1/capture-probe'
    out.mkdir(exist_ok=False)
    plan = json.loads((ROOT/'experiments/ccta2027/reference_origin_v1.json').read_text())
    reports = []
    for case in plan['cases']:
        if case['arm'] != 'reference':
            continue
        folder = ROOT/plan['run_dir']/case['case_id']
        metrics = json.loads((folder/'paired_metrics.json').read_text())
        with np.load(folder/'replay.npz', allow_pickle=False) as z:
            times, robots, xy, angles = [z[k].copy() for k in
                ('times', 'agent_positions', 'cargo/cargo_0/centers', 'cargo/cargo_0/angles')]
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        params = env.controller.params
        margin = params.robot_radius + 3*params.range_noise_std + 2*params.max_speed*case['config']['dt']
        cells = [dict() for _ in env.agents]
        first = None
        for k in range(len(metrics['steps'])):
            env.cargoes[0].set_pose(xy[k], angles[k])
            for i, agent in enumerate(env.agents):
                agent.position = robots[k, i].copy()
            scans = [env.controller.sensor.sense_view(a, env.cargoes, times[k]) for a in env.agents]
            assert [len(s) for s in scans] == [d['own_scan_points'] for d in metrics['steps'][k]['diagnostics']]
            close = any(len(s) and np.min(np.linalg.norm(s.points-a.position, axis=1)) <= margin
                        for s, a in zip(scans, env.agents))
            if close:
                break
            neighbors = env.controller._neighbor_indices(env.agents)
            for i in range(len(env.agents)):
                batch, _ = _merge_scans(scans, [i]+list(neighbors[i]))
                for p, n in zip(batch.points, batch.normals):
                    key = tuple(np.floor(p/.02).astype(int))
                    cells[i].setdefault(key, np.r_[p, n])
            if first is None and any(len(s) for s in scans):
                first = k
        arrays = {f'agent_{i:02d}': np.array(list(c.values())).reshape(-1, 4) for i, c in enumerate(cells)}
        np.savez_compressed(out/(case['condition_id']+'.npz'), **arrays)
        report = dict(condition=case['condition_id'], first_detection_frame=first,
                      freeze_frame=k, freeze_time=float(times[k]), margin_m=margin,
                      initial_rest_max_translation_m=float(np.max(np.linalg.norm(xy[:k+1]-xy[0], axis=1))),
                      initial_rest_max_rotation_rad=float(np.max(np.abs(angles[:k+1]-angles[0]))),
                      points_per_agent=[len(v) for v in arrays.values()],
                      spans=[np.ptp(v[:, :2], axis=0).tolist() if len(v) else None for v in arrays.values()])
        reports.append(report)
        print(json.dumps(report), flush=True)
    (out/'analysis.json').write_text(json.dumps(reports, indent=2))


if __name__ == '__main__':
    main()
