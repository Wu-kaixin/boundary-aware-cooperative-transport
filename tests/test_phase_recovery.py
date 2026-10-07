from dbact.phase import Phase, PhaseGates, PhaseMonitor, PhaseSignals

def test_past_the_brake_point_stops_without_a_pusher_quorum_or_simulator_truth():
    signals = PhaseSignals(agent_count=16, contact_ready=8, transport_active=0, progress=1.1, target_distance=1.0)
    monitor = PhaseMonitor(gates=PhaseGates(transport_quorum=4), phase=Phase.CONTACT_READY)
    assert monitor.update(signals, 100) is Phase.BRAKE
    assert monitor.entered.get("TRANSPORT") is None
    assert monitor.update(PhaseSignals(progress=1.1, target_distance=1.0, estimated_speed=0.0), 101) is Phase.HOLD

def test_recovery_does_not_trigger_below_existing_brake_threshold():
    monitor = PhaseMonitor(gates=PhaseGates(recover_on_brake_progress=True), phase=Phase.CONTACT_READY)
    assert monitor.update(PhaseSignals(progress=0.79, target_distance=1.0, transport_active=0), 0) is Phase.CONTACT_READY
