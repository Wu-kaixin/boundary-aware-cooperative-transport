"""The same local contact geometry must not change with the world origin."""
from dataclasses import replace
import numpy as np
import pytest
from dbact.safety.filter import SafetyFilter, SafetyFilterParams

def fixture():
    p=np.array([5.,4.])
    points=np.array([[5.05,4.18],[4.95,4.18],[5.0,4.175]])
    normals=np.array([[.2,-np.sqrt(.96)],[-.2,-np.sqrt(.96)],[.05,-np.sqrt(.9975)]])
    return p,points,normals

@pytest.mark.parametrize('shift',[[0.,-4.],[3.,4.],[-8.,-6.]])
def test_translated_geometry_preserves_rows_and_filtered_command(shift):
    p,points,normals=fixture();shift=np.array(shift)
    filt=SafetyFilter(SafetyFilterParams(aggregate_frame_consistent=True))
    vel=np.array([.01,-.02]);nominal=np.array([.02,.2]);near=[p+np.array([.4,0.])]
    rows=filt._object_rows(p,points,normals,vel)
    moved=filt._object_rows(p+shift,points+shift,normals,vel)
    for a,b in zip(rows,moved):np.testing.assert_allclose(a,b,atol=1e-12,rtol=0)
    original=filt.filter_velocity(p,nominal,near,points,normals,vel)
    translated=filt.filter_velocity(p+shift,nominal,[x+shift for x in near],points+shift,normals,vel)
    np.testing.assert_allclose(original.velocity,translated.velocity,atol=1e-12,rtol=0)
    assert original.status==translated.status

def test_old_formula_has_a_measurable_origin_defect():
    p,pts,n=fixture();f=SafetyFilter(SafetyFilterParams())
    distance=np.linalg.norm(pts-p,axis=1);shift=np.array([0.,-4.])
    a=f._aggregate_face(p,pts,n,distance)[1]
    b=f._aggregate_face(p+shift,pts+shift,n,distance)[1]
    assert abs(float(a[0]-b[0]))>.05

def test_rotation_covariance_with_moving_boundary_and_neighbors():
    p,pts,n=fixture();theta=.73
    Q=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    f=SafetyFilter(SafetyFilterParams(aggregate_frame_consistent=True))
    v=np.array([.01,-.02]);u=np.array([.02,.2]);near=[p+np.array([.4,0.])]
    a=f.filter_velocity(p,u,near,pts,n,v)
    b=f.filter_velocity(Q@p,Q@u,[Q@x for x in near],pts@Q.T,n@Q.T,Q@v)
    np.testing.assert_allclose(Q@a.velocity,b.velocity,atol=1e-11,rtol=0)

def test_common_normal_is_unchanged_and_radius_remains_in_distance_units():
    p,pts,_=fixture();n=np.tile([0.,-1.],(len(pts),1));d=np.linalg.norm(pts-p,axis=1)
    params=SafetyFilterParams();old=SafetyFilter(params)
    new=SafetyFilter(replace(params,aggregate_frame_consistent=True))
    np.testing.assert_allclose(old._aggregate_face(p,pts,n,d)[1],new._aggregate_face(p,pts,n,d)[1],atol=1e-14,rtol=0)
    more=SafetyFilter(replace(params,aggregate_frame_consistent=True,r_safe=params.r_safe+.02))
    np.testing.assert_allclose(more._aggregate_face(p,pts,n,d)[1],new._aggregate_face(p,pts,n,d)[1]-.02,atol=1e-14,rtol=0)

def test_offset_and_normal_share_normalization():
    p,pts,n=fixture();f=SafetyFilter(SafetyFilterParams(aggregate_frame_consistent=True));d=np.linalg.norm(pts-p,axis=1)
    trace={};A,h,_=f._aggregate_face(p,pts,n,d,trace=trace)
    weights=np.array(trace['weights']);normal=weights@n;offset=weights@np.einsum('ij,ij->i',n,pts)
    expected=(normal@p-offset)/np.linalg.norm(normal)-f.params.r_safe
    assert h[0]==pytest.approx(expected,abs=1e-12)
    assert trace['translation_defect']==0
    assert f.params.rho==SafetyFilterParams().rho

def test_configuration_rejects_irrelevant_theorem_or_pointwise_use():
    from dbact.control.controller import DBACTController, DBACTParams
    with pytest.raises(ValueError,match='aggregate row model'):
        DBACTController(DBACTParams(aggregate_frame_consistent=True,object_row_mode='pointwise'),(0.,8.,0.,8.))
