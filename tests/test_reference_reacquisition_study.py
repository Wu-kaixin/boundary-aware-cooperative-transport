"""Scientific gates must retain failures and compare the correct capture prefix."""
from pathlib import Path
import copy
import json
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def write_episode(folder, after_capture=0., inside_capture=0.):
    folder.mkdir()
    points = np.arange(18, dtype=float).reshape(9, 2)
    points[6:] += after_capture
    points[1] += inside_capture
    positions = np.zeros((5, 1, 2))
    positions[4] += after_capture
    np.savez(folder / 'replay.npz', times=np.arange(5)*.05,
             agent_positions=positions, agent_ids=np.array(['agent_00']),
             sensed_counts=np.array([0, 1, 2, 3, 3]), sensed_points=points,
             **{'task/cargo_0/start': np.array([1., 1.])})
    rows = [dict(frame=i, initial_reference=dict(status='acquiring' if i<3 else 'frozen'),
                 commands=[[0., 0.]]) for i in range(1, 5)]
    rows[-1]['commands'] = [[after_capture, 0.]]
    (folder / 'paired_metrics.json').write_text(json.dumps(dict(steps=rows)))


def test_capture_prefix_includes_freeze_frame_but_not_later_motion(tmp_path):
    from reference_reacquisition_study import compare_capture_prefix
    old, candidate = tmp_path/'old', tmp_path/'new'
    write_episode(old)
    write_episode(candidate, after_capture=4.)
    result = compare_capture_prefix(old, candidate)
    assert result['exact'] and result['freeze_frame'] == 3
    assert result['paired_steps_exact'] and all(result['arrays'].values())


def test_capture_prefix_detects_changed_ragged_observations(tmp_path):
    from reference_reacquisition_study import compare_capture_prefix
    old, candidate = tmp_path/'old', tmp_path/'new'
    write_episode(old)
    write_episode(candidate, inside_capture=1.)
    result = compare_capture_prefix(old, candidate)
    assert not result['exact'] and not result['arrays']['sensed_points']


def test_capture_prefix_requires_matching_epoch_frame(tmp_path):
    from reference_reacquisition_study import compare_capture_prefix
    old, candidate = tmp_path/'old', tmp_path/'new'
    write_episode(old)
    write_episode(candidate)
    path = candidate/'paired_metrics.json'
    rows = json.loads(path.read_text())
    rows['steps'][2]['initial_reference']['status'] = 'acquiring'
    path.write_text(json.dumps(rows))
    assert not compare_capture_prefix(old, candidate)['exact']


def test_mechanism_success_cannot_hide_acquisition_scaling_or_a_failed_case():
    from reference_reacquisition_study import ARMS, evaluate_gates
    conditions = ['circle', 'l', 'c', 'skew']
    rows = [dict(condition=condition, arm=arm, execution='COMPLETED',
                 safe_completed=True, false_hold=False, hard_safety=[],
                 zero_epoch_valid=True, g500=True,
                 scaling=dict(acquisition=13 if condition in ('c','skew') else 0,
                              post_acquisition=0))
            for condition in conditions for arm in ARMS]
    exact = {condition:dict(exact=True) for condition in conditions}
    mechanism, complete = evaluate_gates(rows, exact, exact, conditions)
    assert all(mechanism.values())
    assert not all(complete.values()) and not complete['candidate_acquisition_scalings_zero']
    failed = copy.deepcopy(rows)
    failed[-1]['safe_completed'] = False
    mechanism, _ = evaluate_gates(failed, exact, exact, conditions)
    assert not all(mechanism.values())
    shortened = exact.copy()
    shortened.pop('skew')
    mechanism, _ = evaluate_gates(rows, shortened, exact, conditions)
    assert not mechanism['all_four_old_arms_exact']
