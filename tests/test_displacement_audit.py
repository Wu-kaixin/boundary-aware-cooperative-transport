"""Reporting recovery must preserve numerical evidence and reject mismatched inputs."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest


def fixture(monkeypatch,tmp_path):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'scripts'))
    import replay_observed_reference as m
    monkeypatch.setattr(m,'ROOT',tmp_path)
    monkeypatch.setattr(m,'HISTORY',tmp_path/'history')
    script=tmp_path/'scripts/replay_observed_reference.py';script.parent.mkdir();script.write_text('# fixture')
    monkeypatch.setattr(m,'__file__',str(script))
    module=tmp_path/'src/dbact/control/observed_reference.py';module.parent.mkdir(parents=True);module.write_text('# fixture')
    source=tmp_path/'artifacts/research'/m.MISSION/'reconstruction';source.mkdir(parents=True)
    output=source.parent/'report';output.mkdir()
    (source/'audit.json').write_text(json.dumps({'case':'fixture','identity':'RECONSTRUCTED_INPUTS_VERIFIED_AGAINST_HISTORICAL_OUTPUTS'}))
    history=m.HISTORY/'fixture';history.mkdir(parents=True)
    (history/'resolved_case.json').write_text(json.dumps({'config':{'controller':{}}}))
    (history/'case_result.json').write_text(json.dumps({'result':{'events':{'CONTACT_READY':{'frame':0}}}}))
    truth=np.array([[.1,0.],[.2,0.],[.3,0.]]);old=truth+.2;time=np.array([31.,32.,33.])
    np.savez_compressed(source/'reconstruction.npz',scan_counts=np.zeros((3,2),dtype=int),
        time=time,truth=truth,team_navigation=old)
    np.savez_compressed(output/'reference_readout.npz',time=time,truth=truth,old=old,
        shared=truth+.01,positions=np.zeros((3,2,2)),velocity=np.zeros((3,2,2)),
        rms=np.zeros((3,2)),reasons=np.full((3,2),'accepted'))
    monkeypatch.chdir(tmp_path)
    return m,source.relative_to(tmp_path),output.relative_to(tmp_path)


def test_report_recovery_accepts_relative_paths_without_replaying_or_overwriting(monkeypatch,tmp_path):
    m,source,output=fixture(monkeypatch,tmp_path)
    cache=output/'reference_readout.npz';before=hashlib.sha256(cache.read_bytes()).hexdigest()
    # The tracker must not run in report-only recovery.
    monkeypatch.setattr(m.ObservedReferenceTracker,'update',lambda *args:pytest.fail('Estimator replayed'))
    m.replay(source,output,recover_report=True)
    assert hashlib.sha256(cache.read_bytes()).hexdigest()==before
    report=json.loads((output/'offline_gate.json').read_text())
    assert report['status']=='PASS' and report['reporting_recovery']
    assert report['wall_seconds'] is None
    with pytest.raises(RuntimeError,match='no existing report'):
        m.replay(source,output,recover_report=True)


def test_report_recovery_rejects_a_different_truth_or_input_identity(monkeypatch,tmp_path):
    m,source,output=fixture(monkeypatch,tmp_path)
    path=source/'reconstruction.npz'
    with np.load(path,allow_pickle=False) as z:data={k:z[k] for k in z.files}
    data['truth']=data['truth']+1.
    np.savez_compressed(path,**data)
    with pytest.raises(AssertionError):m.replay(source,output,recover_report=True)
    assert not (output/'offline_gate.json').exists()
