"""Information boundary, real trim capture, and scalar-only intervention tests."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dbact.control.task_progress_readout import ProgressReadout, anchored_candidate
from dbact.sensing.boundary_map import LocalBoundaryMap
from dbact.sensing.registration_capture import RegistrationCapture
from dbact.types import BoundaryView


def record():
    return {
        "A": np.eye(2),
        "b": np.array([0.01, 0]),
        "m_pre": np.array([0.08, 0]),
        "clamped": False,
        "dt": 0.05,
    }


def test_anchored_readout_is_not_a_second_integral():
    r = record()
    before = copy.deepcopy(r)
    for _ in range(2):
        x = anchored_candidate(
            1.24 * 0.0895, r, np.array([1.0, 0]), 1.24, "credible", 0.6
        )
        assert x["admitted"] and x["candidate"] == pytest.approx(1.24 * 0.09)
    for key in ("A", "b", "m_pre"):
        assert np.array_equal(r[key], before[key])
    # Retaining the inherited gain deliberately does not recover true .09.
    assert x["candidate"] != pytest.approx(0.09)


@pytest.mark.parametrize(
    "reason", ["missing", "rank", "quality", "clamp", "nonfinite", "large", "yaw"]
)
def test_admission_failures_return_original_without_clipping(reason):
    r = record()
    direction = np.array([1.0, 0])
    quality = "credible"
    yaw = False
    if reason == "missing":
        r = None
    if reason == "rank":
        r["A"] = np.array([[0.0, 1.0], [0.0, 1.0]])
    if reason == "quality":
        quality = "unavailable"
    if reason == "clamp":
        r["clamped"] = True
    if reason == "nonfinite":
        r["b"][0] = np.nan
    if reason == "large":
        r["b"][0] = 0.03000001
    if reason == "yaw":
        yaw = True
    x = anchored_candidate(0.4, r, direction, 1.24, quality, 0.6, yaw)
    assert not x["admitted"] and x["candidate"] == 0.4


def test_negative_measurement_and_bound_equality_are_not_clipped():
    r = record()
    r["b"][0] = -0.03
    x = anchored_candidate(0.4, r, np.array([1.0, 0]), 1.24, "credible", 0.6)
    assert x["admitted"] and x["candidate"] == pytest.approx(1.24 * 0.05)


def wall():
    return BoundaryView(
        points=np.column_stack([np.arange(10) * 0.2, np.zeros(10)]),
        normals=np.tile([0.0, 1.0], (10, 1)),
        confidence=np.ones(10),
        arc_length=np.full(10, 0.05),
        object_ids=np.full(10, "obj"),
    )


def test_capture_is_trimmed_input_and_never_reuses_stale_registration():
    m = LocalBoundaryMap(carve_enabled=False)
    scan = wall()
    m.update(scan, 0, dt=0.05)
    clock = [1]
    capture = RegistrationCapture(m, lambda: clock[0])
    moved = copy.deepcopy(scan)
    moved.points[:, 1] = 0.01
    moved.points[-1, 1] = 0.08
    result = m.register(moved, 0.05)["obj"]
    r = capture.current("obj")
    assert r["matched_before_trim"] == 10 and result.matches == 9
    assert r["A"].shape == (9, 2) and np.allclose(r["b"], 0.01)
    saved = r["A"].copy()
    pre = m.object_displacement("obj")
    m.update(moved, 0.05, dt=0.05)
    fusion = capture.current_fusion("obj")
    f = np.zeros(2) if fusion is None else fusion["translation"]
    assert np.allclose(
        m.object_displacement("obj") - pre, r["translation"] + f, atol=1e-15
    )
    m._normals[:] = [1, 0]
    assert np.array_equal(r["A"], saved)
    clock[0] = 2
    assert capture.current("obj") is None
    m.register(BoundaryView.empty(), 0.1)
    assert "obj" in m.last_registration and capture.current("obj") is None


def test_capture_does_not_change_map_or_velocity():
    a = LocalBoundaryMap(carve_enabled=False)
    b = LocalBoundaryMap(carve_enabled=False)
    clock = [0]
    RegistrationCapture(b, lambda: clock[0])
    for i in range(8):
        clock[0] = i
        s = wall()
        s.points[:, 1] = 0.005 * i
        for m in (a, b):
            m.register(s, 0.05)
            m.update(s, i * 0.05, dt=0.05)
        for field in ("_points", "_normals", "_keys"):
            assert np.array_equal(getattr(a, field), getattr(b, field))
        assert np.array_equal(a.object_velocity("obj"), b.object_velocity("obj"))


@pytest.mark.parametrize("arm", ["baseline", "certificate_only", "anchored_readout"])
def test_actual_transport_readout_cannot_access_hidden_truth_after_sensing(
    monkeypatch, arm
):
    from dbact_sim.environment import SimulationEnvironment

    plan = json.loads(
        (
            Path(__file__).parents[1] / "experiments/ccta2027/progress_local_v2.json"
        ).read_text()
    )
    env = SimulationEnvironment(plan["cases"][0]["config"], seed=2)
    observer = ProgressReadout(env.controller, env.agents, arm)
    scans = {
        a.agent_id: env.controller.sensor.sense_view(a, env.cargoes, 0.0)
        for a in env.agents
    }
    monkeypatch.setattr(
        env.controller.sensor, "sense_view", lambda a, c, t: scans[a.agent_id]
    )

    class Hidden:
        def __getattribute__(self, name):
            raise AssertionError("truth access: " + name)

    for frame in range(3):
        commands = env.controller.step(env.agents, [Hidden()], frame * env.dt, env.dt)
        assert len(commands) == len(env.agents)
    assert observer.readouts


def test_new_core_implementation_has_no_simulation_dependency():
    root = Path(__file__).parents[1] / "src/dbact"
    for name in ("control/task_progress_readout.py", "sensing/registration_capture.py"):
        assert "dbact_sim" not in (root / name).read_text()


@pytest.mark.parametrize("arm", ["baseline", "certificate_only", "anchored_readout"])
def test_hold_preserves_this_arms_last_admitted_scalar(arm):
    m = LocalBoundaryMap()
    m.displacement["obj"] = np.array([0.0895, 0.0])
    m.velocity["obj"] = np.array([0.2, 0.3])
    c = SimpleNamespace(
        maps={"a": m},
        _frame=0,
        _progress_quality={"a": {"obj": "credible"}},
        _credible_progress={"a": {}},
        params=SimpleNamespace(progress_estimate_gain=1.24),
    )
    c._ensure_state = lambda agents: None

    def original(a, cid, m, d):
        if c._progress_quality[a][cid] != "unavailable":
            c._credible_progress[a][cid] = 1.24 * float(m.object_displacement(cid) @ d)
        return c._credible_progress[a][cid]

    c._credible_task_progress = original
    observer = ProgressReadout(c, [], arm)
    capture = observer.captures["a"]
    capture.epoch = 0
    capture.records["obj"] = record()
    got = c._credible_task_progress("a", "obj", m, np.array([1.0, 0.0]))
    expected = 1.24 * (0.09 if arm == "anchored_readout" else 0.0895)
    assert got == pytest.approx(expected)
    c._frame = 1
    c._progress_quality["a"]["obj"] = "unavailable"
    assert c._credible_task_progress("a", "obj", m, np.array([1.0, 0.0])) == got
    c._frame = 2
    c._progress_quality["a"]["obj"] = "credible"
    assert c._credible_task_progress(
        "a", "obj", m, np.array([1.0, 0.0])
    ) == pytest.approx(1.24 * 0.0895)
    assert np.array_equal(m.velocity["obj"], [0.2, 0.3])
