"""Catch erased hard rows, wrong mutual signs and misleading command decomposition."""
import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest


def load(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    path = root/'scripts/diagnose_pair_residuals.py'
    assert path.exists(), 'Missing pair residual diagnostic implementation'
    spec = importlib.util.spec_from_file_location('pair_residual_diagnostic', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def snapshot():
    return dict(agent_id='a', position=[0., 0.], neighbors=['b'],
        A=[[-2., 0.], [0., 1.], [1., 0.]], b=[-.9, -.5, .1],
        kinds=['agent', 'wall', 'object'], h=[-.1], normal_motion=[.2],
        params=dict(gamma_obj=2., rho=.05), nominal=[0., 0.])


def test_uncapped_modes_preserve_hard_rows_and_motion_sign(monkeypatch):
    module = load(monkeypatch)
    original = snapshot()
    before = copy.deepcopy(original)
    modes = module.systems(original)
    np.testing.assert_allclose(modes['capped_full'], [-.9, -.5, .1])
    np.testing.assert_allclose(modes['uncapped_full'], [-.9, -.5, .45])
    np.testing.assert_allclose(modes['uncapped_no_rho'], [-.9, -.5, .4])
    np.testing.assert_allclose(modes['uncapped_zero_motion'], [-.9, -.5, .25])
    np.testing.assert_allclose(modes['uncapped_zero_motion_no_rho'], [-.9, -.5, .2])
    assert original == before


def test_pair_system_retains_wall_third_neighbor_and_mutual_sum(monkeypatch):
    module = load(monkeypatch)
    first = snapshot()
    first['A'].insert(1, [0., -2.]); first['b'].insert(1, -.3)
    first['kinds'].insert(1, 'agent'); first['neighbors'].append('c')
    second = dict(agent_id='b', position=[1., 0.], neighbors=['a'],
        A=[[2., 0.], [0., -1.], [-1., 0.]], b=[-.9, -.2, .15],
        kinds=['agent', 'wall', 'object'], h=[0.], normal_motion=[.1],
        params=dict(gamma_obj=2.,rho=.05), nominal=[0.,0.])
    A, b = module.pair_system(first, second, 'capped_full')
    np.testing.assert_array_equal(A, [[0.,-2.,0.,0.], [0.,1.,0.,0.],
        [1.,0.,0.,0.], [0.,0.,0.,-1.], [0.,0.,-1.,0.], [-2.,0.,2.,0.]])
    np.testing.assert_allclose(b, [-.3,-.5,.1,-.2,.15,-1.8])


def test_pair_system_rejects_nonmutual_or_wrong_position(monkeypatch):
    module = load(monkeypatch)
    first = snapshot()
    second = dict(agent_id='b', position=[1.,0.], neighbors=[],
        A=[], b=[], kinds=[], h=[], normal_motion=[],params=first['params'],nominal=[0.,0.])
    with pytest.raises(ValueError): module.pair_system(first,second,'capped_full')
    second.update(neighbors=['a'],A=[[2.,0.]],b=[-.9],kinds=['agent'])
    second['position']=[2.,0.]
    with pytest.raises(ValueError): module.pair_system(first,second,'capped_full')


def test_normal_components_use_executed_nominal_and_returned_bias(monkeypatch):
    module = load(monkeypatch)
    result = module.normal_components([-.2,.3],[.1,.05],[-.05,.2],[1.,0.])
    assert result == pytest.approx(dict(nominal=-.2,transport=.1,coverage_residual=-.3,actual=-.05))


def test_application_uses_actual_domain_and_checks_whole_hold(monkeypatch):
    module = load(monkeypatch)
    assert hasattr(module, 'application_certificate'), 'Missing clipped application audit'
    first = dict(position=[.05,.5], domain=[0.,1.,0.,1.],
        params=dict(domain=None, dt=1., max_speed=.35, d_min=.28))
    second = dict(first, position=[.35,.5])
    result = module.application_certificate(first, second, [-.1,0.,-.2,0.],
        np.array([[-1.,0.,0.,0.]]), np.array([.08]))
    assert result['applied_commands'][0] == pytest.approx([-.05,0.])
    assert result['minimum_hold_separation'] == pytest.approx(.15)
    assert result['minimum_applied_residual'] == pytest.approx(-.03)
    assert not result['application_valid']
