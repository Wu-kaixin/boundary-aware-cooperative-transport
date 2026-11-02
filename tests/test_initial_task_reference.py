"""Reference semantics and information-boundary tests, not physical successes."""
import numpy as np
import pytest

from dbact.control.initial_task_reference import (
    InitialReferenceCapture, TaskReferenceTracker, center_error_bound,
    geometric_median, workspace_vertices,
)
from dbact.control.observed_reference import rotation
from dbact.types import BoundaryView


def rectangle(center=(3., 5.)):
    u = np.linspace(-.47, .47, 40)
    points = np.vstack([np.c_[u, np.full(40, -.5)], np.c_[np.full(40, .5), u],
                       np.c_[u, np.full(40, .5)], np.c_[np.full(40, -.5), u]]) + center
    normals = np.repeat([[0., -1.], [1., 0.], [0., 1.], [-1., 0.]], 40, axis=0)
    return BoundaryView(points, normals, np.ones(160), np.ones(160)*.025, np.full(160, 'cargo_0'))


def test_different_observed_anchors_convert_to_the_same_task_reference():
    scan = rectangle()
    q = np.array([4., 4.])
    trackers = []
    for indices in (np.arange(160), np.arange(120)):
        tracker = TaskReferenceTracker(reference_point=q, max_speed=1.)
        tracker.initialize_initial(scan.select(indices), 0.)
        trackers.append(tracker)
    for k in range(1, 11):
        theta = k*.002
        delta = np.array([k*.004, -.001*k])
        points = (scan.points-[3., 5.])@rotation(theta).T+[3., 5.]+delta
        moved = BoundaryView(points, scan.normals@rotation(theta).T, scan.confidence, scan.arc_length, scan.object_ids)
        expected = delta+(rotation(theta)-np.eye(2))@(q-[3., 5.])
        for tracker in trackers:
            reading = tracker.update(moved, k*.05)
            assert reading is not None
            np.testing.assert_allclose(reading.displacement, expected, atol=3e-5)
            np.testing.assert_array_equal(tracker.offset, [0., 0.])


def test_workspace_bound_covers_hidden_centers_but_does_not_equate_anchor_with_center():
    vertices = workspace_vertices((0., 8., 0., 8.))
    q = vertices.mean(axis=0)
    goal = np.array([1.2, 0.])
    assert center_error_bound(goal, 0., q, vertices, goal) == pytest.approx(0.)
    bound = center_error_bound(goal, .05, q, vertices, goal)
    assert bound > .12  # anchor arrived, yet an unknown center need not have arrived
    rng = np.random.default_rng(7)
    for center in rng.uniform(0, 8, (100, 2)):
        error = (rotation(.05)-np.eye(2))@(center-q)
        assert np.linalg.norm(error) <= bound+1e-12


def test_readout_and_bound_are_rigid_frame_covariant():
    q = np.array([4., 4.]); vertices = workspace_vertices((0., 8., 0., 8.))
    d = np.array([1.1, .02]); goal = np.array([1.2, 0.]); turn = rotation(.73); shift = np.array([20., -13.])
    bound = center_error_bound(d, .02, q, vertices, goal)
    assert center_error_bound(turn@d, .02, turn@q+shift, vertices@turn.T+shift, turn@goal) == pytest.approx(bound)
    samples = np.array([[1., 0.], [0., 1.], [0., 0.], [20., 30.]])
    np.testing.assert_allclose(geometric_median(samples@turn.T+shift), turn@geometric_median(samples)+shift, atol=1e-7)


def test_capture_rejects_noisy_line_and_cannot_reset_after_freeze():
    rng = np.random.default_rng(1)
    scan = rectangle()
    scan.points = np.c_[np.linspace(0., 1., 160), rng.normal(0, .01, 160)]
    capture = InitialReferenceCapture(noise=.01)
    capture.add('robot', scan)
    assert not capture.supported('robot')
    capture.freeze()
    with pytest.raises(RuntimeError, match='frozen'):
        capture.add('late', rectangle())
    assert len(capture.view('late')) == 0


def test_missing_scan_never_becomes_a_fresh_stopping_vote():
    tracker = TaskReferenceTracker(reference_point=[4., 4.])
    tracker.initialize_initial(rectangle(), 2.)
    assert tracker.last_reading is None
    assert tracker.update(BoundaryView.empty(), 2.05) is None
    assert tracker.last_reading is None
    with pytest.raises(RuntimeError):
        tracker.initialize_initial(rectangle(), 3.)


def test_circular_yaw_gauge_is_not_reported_as_observed_zero_rotation():
    angle = np.linspace(0., 2*np.pi, 128, endpoint=False)
    normals = np.c_[np.cos(angle), np.sin(angle)]
    scan = BoundaryView(.5*normals+[3., 5.], normals, np.ones(128), np.ones(128)*.02, np.full(128, 'cargo_0'))
    tracker = TaskReferenceTracker(reference_point=[4., 4.])
    tracker.initialize_initial(scan, 0.)
    scan.points = scan.points+[.01, 0.]
    assert tracker.update(scan, .05) is None
    assert tracker.reason == 'task_rotation_unobservable'


def test_task_guard_prevents_hold_without_changing_nominal_arrival():
    from dbact.control.phase import Phase, PhaseMonitor, PhaseSignals
    monitor = PhaseMonitor(phase=Phase.BRAKE)
    signal = PhaseSignals(progress=1.2, target_distance=1.2, estimated_speed=0., task_reference_ready=False)
    assert monitor.update(signal, 1) is Phase.BRAKE
    signal.task_reference_ready = True
    assert monitor.update(signal, 2) is Phase.HOLD


def test_acquisition_is_opt_in_and_cannot_silently_use_old_feedback():
    from dbact_sim.scenarios import load_yaml
    from dbact_sim.environment import SimulationEnvironment
    from dbact.control.phase import Phase
    cfg = load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(boundary_navigation='vector_goal', navigation_estimator='observed_reference',
                             initial_reference_acquisition=True, initial_task_reference=True)
    env = SimulationEnvironment(cfg, seed=2)
    original_contract = env.controller.params.contact_contract()
    for _ in range(3):
        env.step()
        assert env.controller.phase < Phase.CONTACT_READY
        assert env.controller.team_navigation is None
    assert env.controller.params.contact_contract() == original_contract
    assert env.controller.safety.params.r_safe > env.controller.params.robot_radius


def test_normal_rank_must_support_initialization_before_capture_release():
    x = np.linspace(-.5, .5, 80)
    points = np.vstack([np.c_[x, np.full(80, -.1)], np.c_[x, np.full(80, .1)]])
    normals = np.c_[np.tile([-.01, .01], 80), np.repeat([-1., 1.], 80)]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    scan = BoundaryView(points, normals, np.ones(160), np.full(160, .02), np.full(160, 'cargo_0'))
    capture = InitialReferenceCapture(noise=.01)
    capture.add('robot', scan)
    assert not capture.supported('robot')


def test_late_support_cannot_release_after_acquisition_deadline():
    from dbact_sim.scenarios import load_yaml
    from dbact_sim.environment import SimulationEnvironment
    cfg = load_yaml('configs/sim/d/l_shape_closed_loop.yaml')
    cfg['controller'].update(boundary_navigation='vector_goal', navigation_estimator='observed_reference',
                             initial_reference_acquisition=True, initial_task_reference=True)
    env = SimulationEnvironment(cfg, seed=2)
    controller = env.controller
    controller._update_initial_reference(env.agents, {a.agent_id:rectangle() for a in env.agents}, 60.05)
    assert controller.initial_reference_status == 'unavailable'
    assert controller.initial_reference_epoch is None
