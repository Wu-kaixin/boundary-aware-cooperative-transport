"""Physical gates cannot pass on missing cases or a hidden C-shape regression."""
import importlib
import copy
import json
from pathlib import Path

import pytest


def load(monkeypatch):
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    assert (root/'scripts/post_acquisition_support_study.py').exists(), 'Missing frozen study gates'
    return importlib.import_module('post_acquisition_support_study')


def examples():
    conditions=['near_circle','near_l_reference','far_c_open_away_pos0','far_skew_box_s0_r0_pos0']
    rows=[]
    for condition in conditions:
        for arm in ('map_bridge_control','post_support_candidate'):
            rows.append(dict(condition=condition,arm=arm,execution='COMPLETED',safe_completed=True,
                zero_epoch_valid=True,false_hold=False,hard_safety=[],budget_terminated=False,g500=True,
                scaling=dict(acquisition=0,post_acquisition=17 if arm=='map_bridge_control' and condition==conditions[-1] else 0),
                maximum_absolute_rotation_deg=1.,phase_response=dict(transport_frame=100),
                pair_audit=dict(verified=True),bridge_audit=dict(verified=True),
                post_support_audit=dict(verified=True,full_episode_verified=True,applied=10,
                    hard_violations=0,uncapped_object_violations=0)))
    return conditions,rows,{c:dict(exact=True) for c in conditions}


def test_missing_episode_and_false_hold_cannot_pass(monkeypatch):
    module=load(monkeypatch)
    conditions,rows,originals=examples()
    mechanism,full=module.evaluate(rows,originals,conditions)
    assert all(mechanism.values()) and all(full.values())
    assert not all(module.evaluate(rows[:-1],originals,conditions)[0].values())
    rows[-1]['false_hold']=True
    assert not all(module.evaluate(rows,originals,conditions)[0].values())


def test_c_rotation_and_uncapped_violation_remain_visible(monkeypatch):
    module=load(monkeypatch)
    conditions,rows,originals=examples()
    candidate=next(r for r in rows if r['condition']=='far_c_open_away_pos0' and r['arm']=='post_support_candidate')
    candidate['maximum_absolute_rotation_deg']=2.
    assert not module.evaluate(rows,originals,conditions)[0]['c_rotation_not_increased']
    candidate['maximum_absolute_rotation_deg']=1.
    candidate['post_support_audit']['uncapped_object_violations']=1
    mechanism,full=module.evaluate(rows,originals,conditions)
    assert all(mechanism.values()) and not full['uncapped_admitted_object_rows_satisfied']


def test_physical_cases_retain_frozen_parameters_and_initial_states(monkeypatch):
    module=load(monkeypatch)
    root=Path(__file__).resolve().parents[1]
    old=json.loads((root/'experiments/ccta2027/map_reference_bridge_v1.json').read_text())
    cases,preflight=module.build_cases(old)
    assert len(cases)==len(preflight)==8
    for case in cases:
        original=next(c for c in old['cases'] if c['condition_id']==case['condition_id']
                      and c['arm']=='map_bridge_candidate')
        config=copy.deepcopy(case['config'])
        assert config['controller'].pop('post_acquisition_finite_support',False) is (case['arm']=='post_support_candidate')
        assert config==original['config']
        for key in ('seed','frames','expected_initial_state_sha256'):
            assert case[key]==original[key]
    altered=copy.deepcopy(old)
    for case in altered['cases']:
        if case['arm']=='map_bridge_candidate':
            case['config']['controller']['max_speed']=.36
    with pytest.raises(AssertionError):
        module.build_cases(altered)


def test_existing_physical_attempt_cannot_be_overwritten(monkeypatch,tmp_path):
    module=load(monkeypatch)
    monkeypatch.setattr(module,'FREEZE',tmp_path/'freeze')
    monkeypatch.setattr(module,'OUT',tmp_path/'run')
    monkeypatch.setattr(module,'PROTOCOL',tmp_path/'protocol.json')
    (tmp_path/'protocol.json').write_text('preserve original')
    with pytest.raises(RuntimeError,match='Preserve'):
        module.freeze()
    assert (tmp_path/'protocol.json').read_text()=='preserve original'


@pytest.mark.parametrize('kind',['prefix','pair','bridge'])
def test_partial_or_missing_constraint_audit_cannot_pass(monkeypatch,kind):
    module=load(monkeypatch)
    conditions,rows,originals=examples()
    row=rows[-1]
    if kind=='prefix':
        row['post_support_audit']['full_episode_verified']=False
    else:
        row[kind+'_audit']={'verified':False}
    mechanism,full=module.evaluate(rows,originals,conditions)
    assert not all(mechanism.values()) and not all(full.values())
