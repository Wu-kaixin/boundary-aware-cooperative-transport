"""Post-hoc evaluator-only check: can perfect position enter the old goal band?

Ignoring speed, quorum and model bounds gives an optimistic upper limit on
possible perfect-readout stop streaks along each fixed old trajectory. This is
not a controller input, gate revision, physical episode or adoption decision.
"""
import argparse
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from task_translation_study import longest_streak
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main(out):
    guard()
    out = Path(out).resolve()
    assert out.is_relative_to(ROOT/'runs') and not out.exists()
    study_path = ROOT/'docs/results/task_translation_completion_v1/analysis.json'
    study = read(study_path)
    prior = read(ROOT/'experiments/ccta2027/task_translation_v2.json')
    rows = []
    for case, summary in zip(prior['cases'], study['cases']):
        assert case['case_id'] == summary['case_id']
        folder = ROOT/prior['original_run_dir']/case['case_id']
        receipt = cached_result(folder, {**prior['original_identity'], 'case_sha256': encode(case)})
        band = SimulationEnvironment(case['config'], seed=case['seed']).controller.params.cross_track_deadband
        with np.load(folder/'replay.npz', allow_pickle=False) as z:
            errors = np.linalg.norm(z['cargo/cargo_0/centers'][:-1]-z['task/cargo_0/goal_point'], axis=1)
        count = summary['arms']['se2_supported']['post_ready_frames']
        errors = errors[-count:]
        inside = errors <= band
        rows.append(dict(case_id=case['case_id'], frames=count, original_task_completed=receipt['task_completed'],
            goal_band_m=band, minimum_true_goal_error_m=float(errors.min()),
            position_only_eligible_frames=int(inside.sum()),
            optimistic_longest_stop_streak=longest_streak(inside), replay_sha256=sha(folder/'replay.npz')))
    assert len(rows) == 4
    dump(out, dict(scope=__doc__, script_sha256=sha(__file__), source_analysis_sha256=sha(study_path),
        selection='All four original fixed study cases, entire post-ready windows', cases=rows,
        original_gates_modified=False, new_physical_episodes=0))
    print(rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    main(parser.parse_args().out)
