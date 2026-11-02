"""Control observations are independent of whether diagnostics are recorded."""
import copy
import json
from pathlib import Path
import numpy as np
from dbact_sim.environment import SimulationEnvironment
from dbact.control import observed_geometry

def test_contact_distance_receives_current_own_scan_without_trace(monkeypatch):
    plan=json.loads((Path(__file__).parents[1]/'experiments/ccta2027/cargo_near_v1.json').read_text())
    cfg=next(c['config'] for c in plan['cases'] if c['case_id']=='circle_boundary_cvt')
    original=observed_geometry.patch_distance
    received=[]
    def recorded(scan,*args,**kwargs):
        received.append(len(scan))
        return original(scan,*args,**kwargs)
    monkeypatch.setattr(observed_geometry,'patch_distance',recorded)
    outcomes=[]; observed_counts=[]
    for trace in (False,True):
        env=SimulationEnvironment(copy.deepcopy(cfg),seed=2)
        env.controller.trace_enabled=trace
        received.clear()
        trajectory=[]
        for _ in range(8):
            env.step()
            trajectory.append(np.vstack([a.position for a in env.agents]))
        assert received and max(received)>0, 'Control must receive own live observations even with logging off'
        outcomes.append(np.array(trajectory));observed_counts.append(list(received))
    assert observed_counts[0]==observed_counts[1]
    np.testing.assert_array_equal(outcomes[0],outcomes[1])
