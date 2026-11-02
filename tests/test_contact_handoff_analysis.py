"""Scientific prefix checks must respect variable-length observation records."""
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def compare(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'scripts'))
    from analyze_contact_handoff_study import replay_prefix_equal
    return replay_prefix_equal


def fixture_files(tmp_path, *, alter_point=False, corrupt_count=False):
    first,second=tmp_path/'short.npz',tmp_path/'long.npz'
    # Two frames with 1 + 3 points, followed by a new frame with two more.
    points=np.arange(12,dtype=float).reshape(6,2)
    np.savez(first,times=[0.,.05],sensed_counts=[1,3],sensed_points=points[:4],
             agent_positions=np.zeros((2,1,2)),**{'task/cargo/direction':np.array([1.,0.])})
    if alter_point: points[3,1]+=1e-12  # exact equality: no post-hoc tolerance
    np.savez(second,times=[0.,.05,.1],sensed_counts=[1,3,3 if corrupt_count else 2],sensed_points=points,
             agent_positions=np.zeros((3,1,2)),**{'task/cargo/direction':np.array([1.,0.])})
    return first,second


def test_prefix_compares_all_packed_points_and_keeps_static_metadata(compare,tmp_path):
    result=compare(*fixture_files(tmp_path))
    assert result and all(result.values())
    assert result['task/cargo/direction']  # length two does not make it a time series


def test_a_changed_point_in_the_last_prefix_frame_is_detected_exactly(compare,tmp_path):
    result=compare(*fixture_files(tmp_path,alter_point=True))
    assert result['sensed_counts'] and not result['sensed_points']


def test_malformed_packed_observations_are_not_silently_truncated(compare,tmp_path):
    with pytest.raises(ValueError,match='Packed observation length'):
        compare(*fixture_files(tmp_path,corrupt_count=True))
