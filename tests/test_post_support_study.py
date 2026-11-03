"""Physical gates cannot pass on missing cases or a hidden C-shape regression."""
import importlib
from pathlib import Path


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
                post_support_audit=dict(verified=True,applied=10,hard_violations=0,uncapped_object_violations=0)))
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
