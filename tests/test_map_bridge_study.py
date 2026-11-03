import copy
import importlib.util
from pathlib import Path


def load(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    path=root/'scripts/map_bridge_study.py'
    assert path.exists(), 'Missing bridge study gate'
    spec=importlib.util.spec_from_file_location('map_bridge_study_test',path)
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def rows():
    return [dict(condition=c,arm=a,execution='COMPLETED',safe_completed=True,false_hold=False,
        hard_safety=[],zero_epoch_valid=True,g500=True,phase_response={'transport_frame':10},
        scaling={'acquisition':0,'post_acquisition':int(c=='skew' and a=='local_pair_candidate')},
        bridge_audit={'verified':True}) for c in ('circle','l','c','skew')
        for a in ('local_pair_candidate','map_bridge_candidate')]


def test_gate_requires_actual_skew_improvement_and_all_matched_conditions(monkeypatch):
    m=load(monkeypatch)
    data=rows()
    repro={c:{'exact':True} for c in ('circle','l','c','skew')}
    mechanism,full=m.evaluate(data,repro,list(repro),'skew')
    assert all(mechanism.values()) and all(full.values())
    data[-1]['scaling']['post_acquisition']=1
    mechanism,_=m.evaluate(data,repro,list(repro),'skew')
    assert not all(mechanism.values())
    mechanism,_=m.evaluate(data[:-1],repro,list(repro),'skew')
    assert not all(mechanism.values())


def test_gate_does_not_hide_false_hold_or_missing_formal_transport(monkeypatch):
    m=load(monkeypatch)
    data=rows()
    repro={c:{'exact':True} for c in ('circle','l','c','skew')}
    data[1]['false_hold']=True
    mechanism,_=m.evaluate(data,repro,list(repro),'skew')
    assert not all(mechanism.values())
    data=rows()
    data[1]['phase_response']['transport_frame']=None
    mechanism,full=m.evaluate(data,repro,list(repro),'skew')
    assert all(mechanism.values()) and not all(full.values())
