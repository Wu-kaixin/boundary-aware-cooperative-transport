"""Synthetic log records test event accounting, not physical evidence."""
import importlib.util
from pathlib import Path
import sys
import numpy as np

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from analyze_contact_reinforcement import event_counts


def record(on,phase='TRANSPORT',event='none',state='idle',attempt=0):
    return dict(phase=phase,phase_response=dict(push_suspended=on),phase_guards=dict(recovery_ready=4),
        contact_ids=['b'],contact_forces=[[3.,4.]],diagnostics=[dict(agent_id='b',reinforcement_state=state,
        reinforcement_attempts=attempt,reinforcement_path_length=.2,reinforcement_confirmed=False,
        reinforcement_retain=False,reinforcement_event=event)])


def test_simultaneous_events_preserved_and_timeout_is_not_a_resume():
    rows=[record(True,event='recruit_started|arrived_not_confirmed',state='joining',attempt=1),
          record(False),record(True),record(False,phase='BRAKE',event='rollback:response_timeout|rollback_finished')]
    r=event_counts(rows)
    assert r['events']['recruit_started']==r['events']['arrived_not_confirmed']==1
    assert r['events']['rollback:response_timeout']==r['events']['rollback_finished']==1
    assert r['paused_frames']==2 and r['resumed_pauses']==1
    assert r['robots']['b']['recruited_physical_contact_frames']==1
    assert r['robots']['b']['recruited_max_force_N']==5.
    assert r['started_attempts']==1


def test_observation_ending_while_paused_is_not_a_resume():
    r=event_counts([record(True),record(True)])
    assert r['resumed_pauses']==0 and r['pauses'][0]['end_frame']==1
