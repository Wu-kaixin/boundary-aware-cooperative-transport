"""D2 - the supervisor: every transition is a guard, never a frame number."""

import pytest

from dbact.phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals


def gates(**kwargs) -> PhaseGates:
    defaults = dict(
        informed_fraction=0.55,
        map_coverage_min=0.70,
        contact_quorum=4,
        contact_dwell=20,
        transport_quorum=3,
        brake_fraction=0.80,
    )
    defaults.update(kwargs)
    return PhaseGates(**defaults)


def signals(**kwargs) -> PhaseSignals:
    defaults = dict(
        agent_count=16,
        informed_agents=0,
        map_coverage=0.0,
        contact_ready=0,
        transport_active=0,
        progress=0.0,
        target_distance=0.5,
    )
    defaults.update(kwargs)
    return PhaseSignals(**defaults)


def drive(monitor: PhaseMonitor, sig: PhaseSignals, steps: int, start: int = 0) -> Phase:
    phase = monitor.phase
    for k in range(steps):
        phase = monitor.update(sig, start + k)
    return phase


def test_a_run_that_sees_nothing_stays_in_search():
    monitor = PhaseMonitor(gates=gates())
    assert drive(monitor, signals(), 200) is Phase.SEARCH
    assert monitor.frame_of(Phase.DISCOVER) is None


def test_the_first_observation_is_what_ends_search():
    monitor = PhaseMonitor(gates=gates())
    drive(monitor, signals(), 40)
    monitor.update(signals(informed_agents=1), 40)
    assert monitor.phase is Phase.DISCOVER
    assert monitor.frame_of(Phase.DISCOVER) == 40


def test_enclose_needs_both_a_quorum_of_informed_robots_and_map_coverage():
    monitor = PhaseMonitor(gates=gates())
    monitor.update(signals(informed_agents=1), 0)
    # Plenty of robots, but nobody has seen round the object.
    drive(monitor, signals(informed_agents=16, map_coverage=0.4), 50, start=1)
    assert monitor.phase is Phase.DISCOVER
    # Good coverage, but only one robot holds it.
    drive(monitor, signals(informed_agents=1, map_coverage=0.95), 50, start=51)
    assert monitor.phase is Phase.DISCOVER
    monitor.update(signals(informed_agents=10, map_coverage=0.75), 101)
    assert monitor.phase is Phase.ENCLOSE


def reach_enclose(monitor: PhaseMonitor, frame: int = 0) -> int:
    monitor.update(signals(informed_agents=1), frame)
    monitor.update(signals(informed_agents=12, map_coverage=0.8), frame + 1)
    assert monitor.phase is Phase.ENCLOSE
    return frame + 2


def test_a_single_step_of_quorum_does_not_arm_transport():
    """Robots swing through the contact band on the way to the ring, and a machine
    that armed on the instantaneous count started pushing while the enclosure was
    still open on the far side."""
    monitor = PhaseMonitor(gates=gates(contact_dwell=20))
    frame = reach_enclose(monitor)
    for k in range(19):
        monitor.update(signals(informed_agents=12, map_coverage=0.8, contact_ready=6), frame + k)
    assert monitor.phase is Phase.ENCLOSE
    monitor.update(signals(informed_agents=12, map_coverage=0.8, contact_ready=6), frame + 19)
    assert monitor.phase is Phase.CONTACT_READY


def test_an_interrupted_quorum_restarts_the_dwell():
    monitor = PhaseMonitor(gates=gates(contact_dwell=10))
    frame = reach_enclose(monitor)
    held = signals(informed_agents=12, map_coverage=0.8, contact_ready=6)
    lost = signals(informed_agents=12, map_coverage=0.8, contact_ready=1)
    for k in range(9):
        monitor.update(held, frame + k)
    monitor.update(lost, frame + 9)
    for k in range(9):
        monitor.update(held, frame + 10 + k)
    assert monitor.phase is Phase.ENCLOSE


def test_transport_brake_and_hold_are_driven_by_progress_not_by_frames():
    monitor = PhaseMonitor(gates=gates(contact_dwell=2, brake_fraction=0.8))
    frame = reach_enclose(monitor)
    held = signals(informed_agents=12, map_coverage=0.8, contact_ready=6)
    for k in range(2):
        monitor.update(held, frame + k)
    assert monitor.phase is Phase.CONTACT_READY

    pushing = signals(informed_agents=12, map_coverage=0.8, contact_ready=6, transport_active=4)
    monitor.update(pushing, frame + 2)
    assert monitor.phase is Phase.TRANSPORT

    monitor.update(
        signals(informed_agents=12, map_coverage=0.8, contact_ready=6, transport_active=4,
                progress=0.39, target_distance=0.5),
        frame + 3,
    )
    assert monitor.phase is Phase.TRANSPORT
    monitor.update(
        signals(informed_agents=12, map_coverage=0.8, contact_ready=6, transport_active=4,
                progress=0.41, target_distance=0.5),
        frame + 4,
    )
    assert monitor.phase is Phase.BRAKE
    monitor.update(
        signals(informed_agents=12, map_coverage=0.8, contact_ready=6, transport_active=4,
                progress=0.51, target_distance=0.5),
        frame + 5,
    )
    assert monitor.phase is Phase.HOLD


def test_the_supervisor_never_goes_backwards():
    """Enclosure quality dips every time the cargo breaks loose. A machine that
    fell back would re-arm the dwell and chatter at the stick-slip frequency."""
    monitor = PhaseMonitor(gates=gates(contact_dwell=1))
    frame = reach_enclose(monitor)
    monitor.update(signals(informed_agents=12, map_coverage=0.8, contact_ready=6), frame)
    monitor.update(signals(informed_agents=12, map_coverage=0.8, contact_ready=6, transport_active=4), frame + 1)
    assert monitor.phase is Phase.TRANSPORT
    for k in range(30):
        monitor.update(signals(informed_agents=0, map_coverage=0.0, contact_ready=0), frame + 2 + k)
    assert monitor.phase is Phase.TRANSPORT


def test_several_guards_can_open_in_one_step():
    """A run that arrives fully formed must not need one frame per transition."""
    monitor = PhaseMonitor(gates=gates(contact_dwell=1))
    monitor.update(
        signals(informed_agents=16, map_coverage=0.95, contact_ready=8, transport_active=6,
                progress=0.9, target_distance=0.5),
        0,
    )
    assert monitor.phase is Phase.HOLD
    assert monitor.frame_of(Phase.TRANSPORT) == 0


def test_phases_are_ordered_so_that_reached_means_at_least_this_far():
    monitor = PhaseMonitor(gates=gates(contact_dwell=1))
    frame = reach_enclose(monitor)
    assert monitor.reached(Phase.DISCOVER)
    assert monitor.reached(Phase.ENCLOSE)
    assert not monitor.reached(Phase.TRANSPORT)
    del frame


def test_as_dict_reports_every_deadline_the_contract_scores():
    monitor = PhaseMonitor(gates=gates(contact_dwell=1))
    reach_enclose(monitor, frame=7)
    payload = monitor.as_dict()
    assert payload["first_detection_frame"] == 7
    assert payload["enclosure_frame"] == 8
    assert payload["contact_ready_frame"] is None
    assert payload["final_phase"] == "ENCLOSE"
