"""Exercise the frozen auditor using driven poses, with no physics integration."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
from pathlib import Path
import shutil
import traceback

import numpy as np

import post_acquisition_support_study as study
import post_acquisition_support_audit as original_audit
from analyze_post_acquisition_support_v1 import audit_with_exact_packets
from research_io import guard, sha
from trace_boundary_map import plain
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    plan = study.check()
    out = study.BASE / 'analysis_preflight_v1'
    out.mkdir(exist_ok=False)
    case = next(c for c in plan['cases'] if c['case_id'] == 'near_circle_post_support_candidate')
    poses = study.OUT / 'near_circle_map_bridge_control/replay.npz'
    sources = [Path(__file__), study.ROOT / 'scripts/analyze_post_acquisition_support_v1.py',
               study.ROOT / 'scripts/trace_boundary_map.py',
               study.ROOT / 'experiments/ccta2027/post_acquisition_finite_support_v1_analysis_addendum.md']
    identity = dict(protocol_sha256=sha(study.PROTOCOL), source_sha256=plan['frozen_source_sha256'],
                    driven_pose_source=str(poses.relative_to(study.ROOT)), driven_pose_sha256=sha(poses),
                    helper_sha256={p.name: sha(p) for p in sources}, frames=80,
                    new_physical_episodes=0, synthetic_controller_replay=True)
    for source in sources:
        shutil.copy2(source, out / source.name)
    dump(out / 'started.json', identity)
    try:
        keys = ('times', 'agent_positions', 'cargo/cargo_0/centers', 'cargo/cargo_0/angles')
        with np.load(poses, allow_pickle=False) as z:
            saved = {k: z[k][:80].copy() for k in keys}
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        controller = env.controller
        controller.trace_enabled = True
        original = controller.safety.prepare_velocity
        admissions = Counter()

        def prepare(*args, **kwargs):
            problem = original(*args, **kwargs)
            admissions[controller._trace['post_acquisition_support']['reason']] += 1
            return problem

        controller.safety.prepare_velocity = prepare
        rows, clouds = [], []
        for k in range(80):
            env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
            for i, agent in enumerate(env.agents):
                agent.position = saved['agent_positions'][k, i].copy()
                agent.velocity = np.zeros(2) if k == 0 else np.asarray(rows[-1]['commands'][i])
            commands = controller.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
            rows.append(plain(dict(commands=[v.velocity for v in commands],
                diagnostics=[vars(d) for d in controller.diagnostics],
                solver_statistics=controller.safety.stats.as_dict(), phase=controller.phase.name,
                phase_guards=vars(controller.phase_signals), phase_response=controller.phase_monitor.as_dict())))
            clouds.append(controller.map_snapshot(env.map_snapshot_agent).points.copy())
        dump(out / 'paired_metrics.json', dict(steps=rows, synthetic_controller_replay=True))
        np.savez(out / 'replay.npz', **saved, sensed_counts=np.array([0, *map(len, clouds)]),
                 sensed_points=np.concatenate(clouds))
        try:
            original_audit.audit(out, case)
        except ValueError as exc:
            assert 'truth value of an array' in str(exc)
            dump(out / 'original_failure.json', dict(error=str(exc), traceback=traceback.format_exc()))
        else:
            raise AssertionError('Expected the frozen auditor to reach and fail packet comparison')
        result = audit_with_exact_packets(out, case)
        assert result['verified'] and result['full_episode_verified'] and result['applied'] > 0
        dump(out / 'outcome.json', dict(**identity, status='VERIFIED_SYNTHETIC_REPLAY',
            admissions=dict(admissions), audit=result,
            scope='80 controller steps driven by saved control poses; no physics integration and no candidate task outcome.'))
        print(plain(dict(status='VERIFIED_SYNTHETIC_REPLAY', frames=80, applied=result['applied'])), flush=True)
    except Exception:
        dump(out / 'failure.json', dict(**identity, traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
