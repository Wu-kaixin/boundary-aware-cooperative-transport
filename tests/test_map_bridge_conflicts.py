"""Scientific replay must preserve world geometry and original hard rows."""
import importlib
import json
from pathlib import Path

import numpy as np

from dbact.cargo import Cargo


def load(monkeypatch, name):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module(name)


def test_saved_state_records_translated_rotated_world_polygon(monkeypatch):
    replay = load(monkeypatch, 'replay_map_bridge_conflicts')
    assert hasattr(replay, 'state_record'), 'Selected-state writer must be independently exercised'
    cargo = Cargo('box', np.array([[-1., -.5], [1., -.5], [1., .5], [-1., .5]]))
    cargo.set_pose(np.array([3., 5.]), np.pi / 2)
    agents = {'a': {'A': np.array([[1., 0.]]), 'b': np.array([.1]), 'actual': np.array([0., .2])}}
    saved = replay.state_record(402, 'ENCLOSE', 'a', 'frozen', agents, cargo)
    saved = json.loads(json.dumps(saved, allow_nan=False))
    np.testing.assert_allclose(saved['true_vertices'], [[3.5, 4.], [3.5, 6.], [2.5, 6.], [2.5, 4.]])
    assert saved['true_center'] == [3., 5.]
    assert saved['true_angle'] == np.pi / 2
    assert saved['frame'] == 402 and saved['target'] == 'a'
    assert saved['agents']['a']['b'] == [.1]
    cargo.set_pose(np.array([9., 9.]), 0.)
    assert saved['true_center'] == [3., 5.]
