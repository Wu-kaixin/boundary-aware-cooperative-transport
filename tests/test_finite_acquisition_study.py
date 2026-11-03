"""The new physical screen must not confuse completion with full-chain success."""
import copy
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def examples():
    conditions=['circle','l','c','skew']
    rows=[]
    for condition in conditions:
        for arm in ('reference_reacquisition','finite_acquisition_support'):
            rows.append(dict(condition=condition,arm=arm,execution='COMPLETED',safe_completed=True,
                false_hold=False,hard_safety=[],zero_epoch_valid=True,
                g500=not (arm=='reference_reacquisition' and condition in ('c','skew')),
                phase_response={'transport_frame':20},scaling={'acquisition':0,'post_acquisition':0}))
    return rows,{c:{'exact':True} for c in conditions},conditions


def test_complete_chain_requires_original_g500_and_formal_activation():
    from finite_acquisition_study import evaluate_gates
    rows,reproductions,conditions=examples()
    target=next(r for r in rows if r['condition']=='c' and r['arm']=='finite_acquisition_support')
    target['g500']=False
    target['phase_response']['transport_frame']=None
    mechanism,complete=evaluate_gates(rows,reproductions,conditions)
    assert all(mechanism.values()) and not all(complete.values())


def test_any_acquisition_scaling_or_post_capture_increase_fails_mechanism():
    from finite_acquisition_study import evaluate_gates
    for field in ('acquisition','post_acquisition'):
        rows,reproductions,conditions=examples()
        rows[1]['scaling'][field]=1
        mechanism,complete=evaluate_gates(rows,reproductions,conditions)
        assert not all(mechanism.values()) and not all(complete.values())


def test_missing_episode_and_inexact_baseline_cannot_pass():
    from finite_acquisition_study import evaluate_gates
    rows,reproductions,conditions=examples()
    for data,repro in ((rows[:-1],reproductions),(rows,dict(reproductions,c={'exact':False}))):
        mechanism,complete=evaluate_gates(data,repro,conditions)
        assert not all(mechanism.values()) and not all(complete.values())


def test_safe_endpoint_does_not_excuse_false_hold_invalid_epoch_or_hard_failure():
    from finite_acquisition_study import evaluate_gates
    for key,value in (('false_hold',True),('zero_epoch_valid',False),('hard_safety',['separation'])):
        rows,reproductions,conditions=examples()
        rows[1][key]=value
        mechanism,complete=evaluate_gates(rows,reproductions,conditions)
        assert not all(mechanism.values()) and not all(complete.values())


def test_all_predefined_gates_can_pass_without_rewriting_inputs():
    from finite_acquisition_study import evaluate_gates
    rows,reproductions,conditions=examples()
    original=copy.deepcopy(rows)
    mechanism,complete=evaluate_gates(rows,reproductions,conditions)
    assert all(mechanism.values()) and all(complete.values()) and rows==original
