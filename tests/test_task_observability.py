from types import SimpleNamespace

import numpy as np
import pytest

from dbact.sensing.task_observability import (
    directional_certificate,
    select_additional_normal,
)
from dbact_sim.experiments.readiness_diagnostic import install_progress_oracle


def test_rank_one_oblique_direction_is_not_identifiable():
    d = np.array([1.0, 1.0]) / np.sqrt(2)
    assert abs(d[0]) >= 0.5  # existing coarse direction gate would allow it
    r = directional_certificate([[1.0, 0.0]], d)
    assert not r["identifiable"] and r["amplification"] is None


def test_rank_one_aligned_direction_and_noise_weights():
    r = directional_certificate([[1.0, 0.0], [1.0, 0.0]], [1.0, 0.0])
    assert r["identifiable"]
    assert r["amplification"] == pytest.approx(1 / np.sqrt(2))
    assert np.allclose(r["weights"], [0.5, 0.5])


def test_targeted_probe_repairs_missing_direction():
    i, r = select_additional_normal([[1.0, 0.0]], [[1.0, 0.0], [0.0, 1.0]], [0.0, 1.0])
    assert i == 1 and r["identifiable"] and r["amplification"] == pytest.approx(1.0)


def test_bad_input_is_rejected():
    with pytest.raises(ValueError):
        directional_certificate([[np.nan, 0]], [1, 0])
    with pytest.raises(ValueError):
        directional_certificate([[1, 0]], [2, 0])
    with pytest.raises(ValueError):
        directional_certificate([[1, 0]], [1.000001, 0])


def test_oracle_changes_only_progress_after_original_update():
    agent = SimpleNamespace(agent_id="a")
    cargo = SimpleNamespace(object_id="c", position=np.zeros(2))
    control = SimpleNamespace(
        team_progress={}, _progress={}, quality="fixed", velocity=np.array([7.0, 8.0])
    )

    def original(a, n):
        control.team_progress = {"c": 0.2}
        control._progress = {"a": {"c": 0.2}}

    control._update_progress = original
    env = SimpleNamespace(
        controller=control,
        cargoes=[cargo],
        tasks={"c": SimpleNamespace(progress=lambda p: 0.7)},
        t=0.0,
    )
    install_progress_oracle(env)
    control._update_progress([agent], [[]])
    assert control.team_progress["c"] == 0.7 and control._progress["a"]["c"] == 0.7
    assert control.quality == "fixed" and np.array_equal(control.velocity, [7, 8])
    assert env.oracle_events[0]["estimated_before"] == 0.2
