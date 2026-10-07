from dbact.phase import Phase,PhaseGates,PhaseMonitor,PhaseSignals

def test_recovery_uses_existing_brake_progress_without_truth_or_new_threshold():
 s=PhaseSignals(agent_count=16,contact_ready=8,transport_active=0,progress=1.1,target_distance=1.)
 original=PhaseMonitor(phase=Phase.CONTACT_READY)
 assert original.update(s,100)==Phase.CONTACT_READY
 recovery=PhaseMonitor(gates=PhaseGates(recover_on_brake_progress=True),phase=Phase.CONTACT_READY)
 assert recovery.update(s,100)==Phase.HOLD
 assert recovery.entered['TRANSPORT']==100 and recovery.entered['BRAKE']==100

def test_recovery_does_not_trigger_below_existing_brake_threshold():
 monitor=PhaseMonitor(gates=PhaseGates(recover_on_brake_progress=True),phase=Phase.CONTACT_READY)
 assert monitor.update(PhaseSignals(progress=.79,target_distance=1.,transport_active=0),0)==Phase.CONTACT_READY
