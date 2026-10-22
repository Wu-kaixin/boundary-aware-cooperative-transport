"""Failures exposed while preparing the CCTA experimental revision."""
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.control.controller import DBACTController, DBACTParams
from dbact.control.phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals
from dbact.types import AgentState
from dbact_sim.experiments.evidence import episode_row, pair_rows


def test_hold_dwell_requires_consecutive_ready_frames():
    monitor = PhaseMonitor(gates=PhaseGates(hold_dwell=3), phase=Phase.BRAKE)
    ready = PhaseSignals(progress=1.0, target_distance=1.0, estimated_speed=0.0)
    moving = PhaseSignals(progress=1.0, target_distance=1.0, estimated_speed=0.2)
    assert monitor.update(ready, 0) is Phase.BRAKE
    assert monitor.update(ready, 1) is Phase.BRAKE
    assert monitor.update(moving, 2) is Phase.BRAKE
    assert monitor.settle_streak == 0
    assert monitor.update(ready, 3) is Phase.BRAKE
    assert monitor.update(ready, 4) is Phase.BRAKE
    assert monitor.update(ready, 5) is Phase.HOLD


def test_unavailable_motion_does_not_vote_in_progress_or_speed():
    controller = DBACTController(DBACTParams(progress_estimate_gain=1.0), (0.0, 8.0, 0.0, 8.0))
    agents = [AgentState(f"a{i}", np.array([float(i), 0.0])) for i in range(3)]
    controller.tasks = {"cargo": SimpleNamespace(direction=np.array([1.0, 0.0]), distance=1.0)}
    controller.goal_directions = {"cargo": np.array([1.0, 0.0])}
    controller._credible_progress = {"a1": {"cargo": 0.0}, "a2": {"cargo": 0.0}}
    for agent in agents:
        live = agent.agent_id == "a0"
        controller.maps[agent.agent_id] = SimpleNamespace(
            motion_quality={"cargo": "credible" if live else "unavailable"},
            last_registration={"cargo": SimpleNamespace(matches=10)},
            object_displacement=lambda _: np.array([0.7, 0.0]),
            object_velocity=lambda _, active=live: np.array([0.1 if active else 0.0, 0.0]),
        )
    controller._update_progress(agents, [[1, 2], [0, 2], [0, 1]])
    assert controller.team_progress["cargo"] == pytest.approx(0.7)
    assert controller._agent_progress("a1", "cargo") == pytest.approx(0.7)
    assert controller._estimated_stop("cargo")[0] == pytest.approx(0.1)
    assert controller._transport_feedback["a1"]["cargo"] == pytest.approx([.1, 0.])
    # Losing the last live one-hop source must invalidate the relay; the previous
    # nonzero velocity cannot survive in the next control hold.
    controller.maps["a0"].motion_quality["cargo"] = "unavailable"
    controller._update_progress(agents, [[1, 2], [0, 2], [0, 1]])
    assert all(not slot for slot in controller._transport_feedback.values())


def test_default_controller_satisfies_its_own_disc_separation_contract():
    controller = DBACTController(DBACTParams(), (0.0, 8.0, 0.0, 8.0))
    assert controller.params.d_min >= 2 * controller.params.robot_radius


def _episode(condition, method):
    result = {"case_id": f"{condition}_{method}", "condition_id": condition, "shape": "rectangle",
              "seed": 2, "variant": method, "stratum": "near",
              "events": {"CONTACT_READY": {"seconds": 1.0, "right_censored": False}}}
    return episode_row(result, "generalization")


def test_factor_conditions_with_same_seed_do_not_overwrite_each_other():
    rows = [_episode(condition, method) for condition in ("scale1", "scale2")
            for method in ("boundary_cvt", "nearest_observed_target")]
    pairs = pair_rows(rows, "boundary_cvt", "nearest_observed_target")
    assert len(pairs) == 2
    assert {p["condition_id"] for p in pairs} == {"scale1", "scale2"}


def test_duplicate_method_episode_is_an_error_not_a_silent_replacement():
    rows = [_episode("scale1", "boundary_cvt"), _episode("scale1", "boundary_cvt"),
            _episode("scale1", "nearest_observed_target")]
    with pytest.raises(ValueError, match="Duplicate"):
        pair_rows(rows, "boundary_cvt", "nearest_observed_target")
