"""Observable geometry contracts, with hand-derived distances and directions."""
from types import SimpleNamespace

import numpy as np
import pytest


def module():
    import dbact.sensing.finite_support as support
    return support


def returns(points, rays=None, objects=None):
    return [dict(point=np.array(p,dtype=float),ray_index=i if rays is None else rays[i],
                 object_id='cargo' if objects is None else objects[i]) for i,p in enumerate(points)]


def params():
    return SimpleNamespace(r_safe=.195,max_speed=.35,dt=.05,object_row_range=.6)


def test_straight_chord_counts_acquisition_noise_once():
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0]]),[0,.3],1.,.01)
    row=m.support_rows([0,.3],[packet],params(),1.,0.)
    assert row.ready and row.reason=='supported'
    np.testing.assert_allclose(row.A,[[0,1]],atol=1e-12)
    np.testing.assert_allclose(row.h,[.085],atol=1e-12)
    np.testing.assert_allclose(row.lower_distance,[.25],atol=1e-12)


def test_endpoint_uses_radial_distance_not_infinite_plane():
    m=module()
    packet=m.scan_support(returns([[0,0],[0,.04]]),[.3,-.4],0.,0.)
    row=m.support_rows([.3,-.4],[packet],params(),0.,0.)
    assert row.ready
    np.testing.assert_allclose(row.A,[[.6,-.8]],atol=1e-12)
    np.testing.assert_allclose(row.h,[.285],atol=1e-12)


@pytest.mark.parametrize('points,rays,objects',[
    ([[0,0],[.02,0]],[0,2],None),
    ([[0,0],[.081,0]],None,None),
    ([[0,0],[.02,0]],None,['a','b']),
    ([[0,0],[0,0]],None,None),
])
def test_missing_ray_sparse_object_switch_and_duplicate_point_do_not_bridge(points,rays,objects):
    m=module()
    packet=m.scan_support(returns(points,rays,objects),[0,.3],0.,0.)
    row=m.support_rows([0,.3],[packet],params(),0.,0.)
    assert not row.ready and len(row.A)==0


def test_two_sources_cannot_create_a_chord():
    m=module()
    packets=[m.scan_support(returns([[0,0]]),[0,.3],0.,0.),
             m.scan_support(returns([[.02,0]]),[0,.3],0.,0.)]
    assert not m.support_rows([0,.3],packets,params(),0.,0.).ready


def test_uncovered_nearer_return_prevents_cherry_picking_far_chord():
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0],[0,.09]],[0,1,5]),[0,.3],0.,0.)
    row=m.support_rows([0,.3],[packet],params(),0.,0.)
    assert not row.ready and row.reason=='uncovered_near_return'


@pytest.mark.parametrize('stamp',[.95,1.05])
def test_stale_or_future_packet_cannot_substitute_current_rows(stamp):
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0]]),[0,.3],stamp,0.)
    row=m.support_rows([0,.3],[packet],params(),1.,0.)
    assert not row.ready and len(row.A)==0


def test_truth_fields_do_not_change_observational_rows():
    m=module()
    raw=returns([[-.02,0],[.02,0]])
    packet=m.scan_support(raw,[0,.3],0.,.01)
    for point in raw: point.update(true_normal=[999,999],true_edge=-500,centroid=[-5,2])
    other=m.scan_support(raw,[0,.3],0.,.01)
    first=m.support_rows([0,.3],[packet],params(),0.,.2)
    second=m.support_rows([0,.3],[other],params(),0.,.2)
    np.testing.assert_array_equal(first.A,second.A)
    np.testing.assert_array_equal(first.h,second.h)


@pytest.mark.parametrize('bound',[-.1,float('nan'),float('inf'),None])
def test_motion_bound_must_be_explicit_finite_and_nonnegative(bound):
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0]]),[0,.3],0.,0.)
    with pytest.raises(ValueError):m.support_rows([0,.3],[packet],params(),0.,bound)


def test_zero_distance_has_finite_source_facing_normal():
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0]]),[0,.3],0.,0.)
    row=m.support_rows([0,0],[packet],params(),0.,0.)
    assert row.ready
    np.testing.assert_allclose(row.A,[[0,1]],atol=1e-12)
    np.testing.assert_allclose(row.h,[-.215],atol=1e-12)


def test_boundary_motion_expands_the_feature_switch_cover():
    m=module()
    packets=[m.scan_support(returns([[-.01,y],[.01,y]]),[0,.3],0.,0.) for y in (0.,-.07)]
    static=m.support_rows([0,.3],packets,params(),0.,0.)
    moving=m.support_rows([0,.3],packets,params(),0.,.2)
    assert static.ready and moving.ready
    assert len(static.A)==1 and len(moving.A)==2


@pytest.mark.parametrize('points,noise',[
    ([[0,0],[float('nan'),0]],0.), ([[0,0],[.02,0]],-.01),
    ([[0,0],[.02,0]],float('inf')),
])
def test_invalid_measurements_are_rejected(points,noise):
    m=module()
    with pytest.raises(ValueError):m.scan_support(returns(points),[0,.3],0.,noise)


def test_empty_packets_are_unavailable():
    m=module()
    assert not m.support_rows([0,.3],[],params(),0.,0.).ready


def test_isolated_return_gets_a_radial_guard_without_a_fictitious_chord():
    m=module()
    packet=m.scan_support(returns([[-.02,0],[.02,0],[0,.05]],[0,1,5]),[0,.3],0.,0.)
    row=m.support_rows([0,.3],[packet],params(),0.,0.,isolated_point_guards=True)
    assert row.ready and row.point_guard_rows==1
    assert row.h.min()==pytest.approx(.0280741759643275)
    assert not m.support_rows([0,.3],[packet],params(),0.,0.).ready


def test_missing_ray_is_not_reconnected_by_point_guards():
    m=module()
    packet=m.scan_support(returns([[0,0],[.02,0]],[0,2]),[.01,.3],0.,0.)
    row=m.support_rows([.01,.001],[packet],params(),0.,0.,isolated_point_guards=True)
    assert row.ready and row.point_guard_rows==2
    assert row.A[0,0]>.99 and row.A[1,0]<-.99


@pytest.mark.parametrize('raw',[
    returns([[0,0],[.02,0]],objects=['a','b']),
    returns([[0,0],[.081,0]],[0,2]),
    returns([[0,0],[0,0]],[0,2]),
])
def test_isolated_gap_cannot_use_other_objects_cap_truncation_or_duplicates(raw):
    m=module()
    packet=m.scan_support(raw,[0,.3],0.,0.)
    row=m.support_rows([0,.3],[packet],params(),0.,0.,isolated_point_guards=True)
    assert not row.ready


def test_isolated_spacing_can_use_permitted_peer_points_without_a_line_bridge():
    m=module()
    packets=[m.scan_support(returns([[x,0]]),[x,.3],0.,0.) for x in (0.,.02)]
    row=m.support_rows([.01,.3],packets,params(),0.,0.,isolated_point_guards=True)
    assert row.ready and row.point_guard_rows==2


def test_malformed_point_shape_cannot_be_reinterpreted_as_additional_returns():
    m=module()
    with pytest.raises(ValueError):
        m.scan_support(returns([[0,0,1,2],[.02,0,3,4]]),[0,.3],0.,0.)
