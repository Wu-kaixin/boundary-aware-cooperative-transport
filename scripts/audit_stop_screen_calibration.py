"""Post-hoc calibration of the offline dwell screen against completed old tasks.

Does not change the frozen screen, rerun controls, or turn a candidate into a
physical success. Original completed cases are selected before reading masks.
"""
import argparse
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes


def main(out):
    guard()
    out = Path(out).resolve()
    if not out.is_relative_to(ROOT/'runs') or out.exists():
        raise ValueError('Use a new output under runs/')
    study_path = ROOT/'experiments/ccta2027/task_translation_v2.json'
    study = read(study_path)
    assert source_hashes() == study['source_files']
    assert all(sha(ROOT/n) == h for n, h in study['scripts'].items())
    original_path = ROOT/'experiments/ccta2027/task_reference_v1.json'
    original = read(original_path)
    expected = dict(schema='ccta-study-v2', protocol_sha256=sha(original_path), split='development',
        source_sha256=original['frozen_source_sha256'], environment=original['environment'])
    assert expected == study['original_identity']
    selected = []
    for case in study['cases']:
        folder = ROOT/study['original_run_dir']/case['case_id']
        receipt = cached_result(folder, {**expected, 'case_sha256': encode(case)})
        if receipt['task_completed']:
            selected.append((case, folder, receipt))
    rows = []
    for case, old_folder, result in selected:
        folder = ROOT/study['run_dir']/case['case_id']
        summary = read(folder/'summary.json')
        assert summary['case_id'] == case['case_id']
        assert sha(folder/'series.npz') == summary['series_sha256']
        assert sha(folder/'templates.json') == summary['templates_sha256']
        steps = read(old_folder/'paired_metrics.json')['steps']
        hold = next(k for k, s in enumerate(steps) if s['phase'] == 'HOLD')
        ready = next(k for k, s in enumerate(steps) if s['phase'] == 'CONTACT_READY')
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        task, p = env.controller.tasks['cargo_0'], env.controller.params
        target = task.distance*task.direction
        with np.load(folder/'series.npz', allow_pickle=False) as z:
            potential = z['potential_stop'].copy()
            derived = ((np.linalg.norm(z['estimates']-target, axis=2) <= p.cross_track_deadband)
                & (np.linalg.norm(z['velocities'], axis=2) <= env.controller.phase_monitor.gates.hold_speed)
                & (z['voters'] >= p.min_push_agents) & (z['bounds'] <= task.tolerance)
                & (np.arange(len(steps))[None, :] > ready))
            np.testing.assert_array_equal(potential, derived)
        with np.load(old_folder/'replay.npz', allow_pickle=False) as z:
            # Replay includes t0; physical termination uses each post-step speed.
            speed = z['cargo/cargo_0/speed'][1:]
            assert len(speed) == len(steps)
            quiet_frames = int(np.sum(speed[hold:] <= result['termination']['settle_speed']))
        rows.append(dict(case_id=case['case_id'], original_task_completed=result['task_completed'],
            original_termination=result['termination'], first_hold_frame=hold+1,
            hold_window_frames=len(steps)-hold, physical_quiet_frames_in_hold=quiet_frames,
            arms={arm: dict(total_potential_frames=int(potential[a].sum()),
                potential_frames_in_old_hold=int(potential[a, hold:].sum()),
                longest_potential_streak=summary['arms'][arm]['longest_valid_potential_stop_streak'],
                first_potential_frame=int(np.flatnonzero(potential[a])[0]+1) if potential[a].any() else None)
                for a, arm in enumerate(study['arms'])},
            original_receipt_sha256=sha(old_folder/'case_result.json'), replay_summary_sha256=sha(folder/'summary.json'),
            replay_series_sha256=sha(folder/'series.npz')))
    dump(out, dict(scope=__doc__, selection='All original task_reference cases with task_completed=True',
        script_sha256=sha(__file__), study_protocol_sha256=sha(study_path), cases=rows,
        frozen_screen_modified=False, new_physical_episodes=0))
    print(rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    main(parser.parse_args().out)
