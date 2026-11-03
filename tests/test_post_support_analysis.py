"""Planned arm labels are metadata; scientific differences must still fail."""
import copy
from dataclasses import asdict, dataclass
import importlib
import json
from pathlib import Path

import numpy as np
import pytest


def fixture(monkeypatch,tmp_path):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    module=importlib.import_module('analyze_post_acquisition_support_v1')
    folders=[tmp_path/'old',tmp_path/'new']
    for i,folder in enumerate(folders):
        folder.mkdir()
        np.savez(folder/'replay.npz',positions=np.array([[1.,2.],[3.,4.]]))
        for name in ('config.json','paired_metrics.json','physics_substeps.json','summary.json'):
            value={'measurements':[1.,2.]}
            if name=='summary.json': value['provenance']={'git_sha':str(i)}
            (folder/name).write_text(json.dumps(value),encoding='utf-8')
    old_case=dict(condition_id='near_circle',arm='map_bridge_candidate',case_id='old')
    new_case=dict(condition_id='near_circle',arm='map_bridge_control',case_id='new')
    old_result=dict(arm=old_case['arm'],case_id='old',factors={'cohort':'old'},wall_seconds=1.,endpoint=.02)
    new_result=dict(arm=new_case['arm'],case_id='new',factors={'cohort':'new'},wall_seconds=2.,endpoint=.02)
    return module,folders,old_result,new_result,old_case,new_case


def test_planned_arm_rename_is_recorded_without_mutating_results(monkeypatch,tmp_path):
    module,folders,old,new,old_case,new_case=fixture(monkeypatch,tmp_path)
    before=copy.deepcopy((old,new))
    result=module.control_reproduction(*folders,old,new,old_case,new_case)
    assert result['exact'] and result['execution_metadata']['arm']==['map_bridge_candidate','map_bridge_control']
    assert (old,new)==before


def test_scientific_change_is_not_ignored(monkeypatch,tmp_path):
    module,folders,old,new,old_case,new_case=fixture(monkeypatch,tmp_path)
    new['endpoint']=.04
    assert not module.control_reproduction(*folders,old,new,old_case,new_case)['exact']


def test_unplanned_arm_or_condition_is_rejected(monkeypatch,tmp_path):
    module,folders,old,new,old_case,new_case=fixture(monkeypatch,tmp_path)
    new_case['condition_id']='far_c_open_away_pos0'
    with pytest.raises(ValueError,match='identity'):
        module.control_reproduction(*folders,old,new,old_case,new_case)


@pytest.mark.parametrize('altered', [False, True])
def test_packet_arrays_compare_exactly_and_converter_is_restored(monkeypatch, tmp_path, altered):
    module, *_ = fixture(monkeypatch, tmp_path)
    audit_module = importlib.import_module('post_acquisition_support_audit')
    original_converter = audit_module.recorded

    @dataclass
    class Packet:
        points: np.ndarray
        timestamp: float

    packet = Packet(np.array([[1., 2.], [3., 4.]]), 1.)
    received = copy.deepcopy(packet)
    if altered:
        received.points[1, 1] = np.nextafter(4., 5.)

    def check_packets(*args, **kwargs):
        assert audit_module.recorded(asdict(packet)) == audit_module.recorded(asdict(received))
        return {'verified': True}

    monkeypatch.setattr(audit_module, 'audit', check_packets)
    if altered:
        with pytest.raises(AssertionError):
            module.audit_with_exact_packets(tmp_path, {})
    else:
        assert module.audit_with_exact_packets(tmp_path, {}) == {'verified': True}
    assert audit_module.recorded is original_converter
