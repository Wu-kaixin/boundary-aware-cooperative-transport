"""Brake permission is not the pusher quorum, and HOLD is a settle."""

from dbact.phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals


def _ready(**kwargs) -> PhaseSignals:
    base = dict(
        agent_count=16,
        informed_agents=16,
        map_coverage=1.0,
        contact_ready=8,
        transport_active=3,
        progress=0.2,
        target_distance=1.0,
        estimated_speed=0.05,
        estimate_quality="credible",
    )
    base.update(kwargs)
    return PhaseSignals(**base)


def test_brake_does_not_wait_for_four_pushers():
    """C seed 8: three push-side robots and progress already inside the stop."""
    monitor = PhaseMonitor(
        gates=PhaseGates(contact_quorum=4, transport_quorum=4, brake_fraction=0.65),
        phase=Phase.CONTACT_READY,
    )
    assert monitor.update(_ready(progress=0.9, transport_active=3), 80) is Phase.BRAKE
    assert monitor.phase is not Phase.TRANSPORT
    assert monitor.brake_reason == "estimated_stop_from_contact_ready"
    assert monitor.update(_ready(progress=1.0, estimated_speed=0.0, transport_active=0), 81) is Phase.HOLD


def test_below_the_stop_three_pushers_do_not_start_transport():
    monitor = PhaseMonitor(
        gates=PhaseGates(transport_quorum=4, brake_fraction=0.65),
        phase=Phase.CONTACT_READY,
    )
    assert monitor.update(_ready(progress=0.2, transport_active=3), 0) is Phase.CONTACT_READY
    assert monitor.update(_ready(progress=0.2, transport_active=4), 1) is Phase.TRANSPORT


def test_contact_loss_pauses_then_stops_without_rewinding():
    gates = PhaseGates(contact_loss_dwell=3, contact_regain_dwell=3, pause_limit=4, transport_quorum=4)
    monitor = PhaseMonitor(gates=gates, phase=Phase.TRANSPORT)
    lost = _ready(transport_active=0, progress=0.2)
    for frame in range(3):
        assert monitor.update(lost, frame) is Phase.TRANSPORT
    assert monitor.push_suspended
    for frame in range(3, 8):
        monitor.update(lost, frame)
    assert monitor.phase is Phase.BRAKE
    assert monitor.contact_recovery_failed
    assert monitor.short_of_target
    assert monitor.phase is not Phase.HOLD


def test_contact_flicker_shorter_than_the_dwell_does_not_toggle():
    gates = PhaseGates(contact_loss_dwell=4, contact_regain_dwell=4, transport_quorum=4, sustain_contacts=1)
    monitor = PhaseMonitor(gates=gates, phase=Phase.TRANSPORT)
    for frame in range(12):
        active = 0 if frame % 2 == 0 else 4
        monitor.update(_ready(transport_active=active, progress=0.2), frame)
    assert monitor.phase is Phase.TRANSPORT
    assert not monitor.push_suspended


def test_unavailable_estimate_stops_the_push_and_does_not_count_as_hold():
    gates = PhaseGates(estimate_loss_dwell=2, transport_quorum=4)
    monitor = PhaseMonitor(gates=gates, phase=Phase.TRANSPORT)
    blind = _ready(progress=0.4, estimate_quality="unavailable", estimated_speed=0.0, transport_active=4)
    assert monitor.update(blind, 0) is Phase.TRANSPORT
    assert monitor.update(blind, 1) is Phase.BRAKE
    assert monitor.estimate_stopped
    assert monitor.update(blind, 2) is Phase.BRAKE


def test_overshoot_is_recorded_and_is_not_hold_until_speed_falls():
    monitor = PhaseMonitor(gates=PhaseGates(brake_fraction=0.65), phase=Phase.CONTACT_READY)
    assert monitor.update(_ready(progress=1.4, estimated_speed=0.2, transport_active=0), 0) is Phase.BRAKE
    assert monitor.overshoot
    assert monitor.update(_ready(progress=1.4, estimated_speed=0.2, transport_active=0), 1) is Phase.BRAKE
    assert monitor.update(_ready(progress=1.4, estimated_speed=0.0, transport_active=0), 2) is Phase.HOLD
