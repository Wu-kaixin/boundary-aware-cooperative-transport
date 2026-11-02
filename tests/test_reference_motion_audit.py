"""Physical counterexamples protect interpretation of offline estimator diagnosis."""
import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest


@pytest.fixture(scope='module')
def audit():
    path=Path(__file__).resolve().parents[1]/'scripts/audit_reference_motion.py'
    sys.path.insert(0,str(path.parent))
    try:
        spec=importlib.util.spec_from_file_location('reference_motion_audit_tests',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        yield module
    finally:sys.path.remove(str(path.parent))


def test_perfect_rotation_can_look_like_center_translation(audit):
    meta=dict(anchor=[0.,1.],pivot_shift=[0.,0.],offset=[0.,0.])
    angle=.2;point=(audit.rotation(angle)-np.eye(2))@np.array(meta['anchor'])
    terms=audit.local_terms(meta,dict(reference_displacement=point,reference_angle=angle),angle,
                            np.zeros(2),np.zeros(2),0.,np.zeros(2))
    np.testing.assert_allclose(terms['fit_error'],0.,atol=1e-14)
    assert np.linalg.norm(terms['actual'])>.12
    readings=audit.counterfactual_readouts([terms],np.zeros(2))
    np.testing.assert_allclose(readings['perfect_registration'],point)
    np.testing.assert_allclose(readings['remove_point_rotation'],0.,atol=1e-14)


def test_translation_bootstrap_bias_survives_perfect_registration(audit):
    meta=dict(anchor=[.3,.6],pivot_shift=[0.,0.],offset=[.1,.02])
    terms=audit.local_terms(meta,dict(reference_displacement=[.6,.02],reference_angle=0.),0.,
                            np.array([.5,0.]),np.zeros(2),0.,np.zeros(2))
    np.testing.assert_allclose(terms['fit_error'],0.,atol=1e-14)
    np.testing.assert_allclose(terms['initial'],[.1,.02])
    readings=audit.counterfactual_readouts([terms],np.array([.5,0.]))
    np.testing.assert_allclose(readings['perfect_registration'],[.6,.02])


def test_counterfactual_uses_median_after_pointwise_correction(audit):
    b=np.array([[0.,0.],[0.,0.],[100.,0.]])
    r=np.array([[0.,0.],[100.,0.],[0.,0.]])
    terms=[dict(actual=b[i]+r[i],initial=b[i],point_rotation=r[i]) for i in range(3)]
    readings=audit.counterfactual_readouts(terms,np.zeros(2))
    np.testing.assert_array_equal(readings['perfect_registration'],[100.,0.])
    np.testing.assert_array_equal(readings['remove_bootstrap'],[0.,0.])
    assert not np.array_equal(readings['perfect_registration'],np.median(b,axis=0)+np.median(r,axis=0))


def test_pivot_yaw_error_is_not_anchor_translation_error(audit):
    meta=dict(anchor=[0.,1.],pivot_shift=[0.,-1.],offset=[0.,0.])
    real_angle=.2;fit_angle=.3
    true_translation=(audit.rotation(real_angle)-np.eye(2))@np.array(meta['anchor'])
    reading=true_translation+(audit.rotation(fit_angle)-np.eye(2))@np.array(meta['pivot_shift'])
    terms=audit.local_terms(meta,dict(reference_displacement=reading,reference_angle=fit_angle),real_angle,
                            np.zeros(2),np.zeros(2),0.,np.zeros(2))
    np.testing.assert_allclose(terms['translation_error'],0.,atol=1e-14)
    np.testing.assert_allclose(terms['point_rotation'],0.,atol=1e-14)
    assert np.linalg.norm(terms['pivot_angle_error'])>0


def test_unknown_reference_center_has_rotation_dependent_ambiguity(audit):
    offset=np.array([.4,.9]);angle=.12
    difference=(audit.rotation(angle)-np.eye(2))@offset
    assert np.linalg.norm(difference)==pytest.approx(2*abs(np.sin(angle/2))*np.linalg.norm(offset))
    np.testing.assert_allclose((audit.rotation(0)-np.eye(2))@offset,0.)


def test_position_gate_resolves_historical_override_not_default(audit):
    case={'config':{'controller':{'cross_track_deadband':.02}}}
    assert audit.DBACTParams().cross_track_deadband==.03
    assert audit.position_band(case)==.02
    assert not .025<=audit.position_band(case)
