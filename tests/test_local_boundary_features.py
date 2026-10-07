import numpy as np
from dbact.local_boundary_features import scan_segments,segment_rows
from dbact.safety_filter import SafetyFilterParams

def returns(points,rays=None,objects=None):
 return [dict(point=np.array(p),ray_index=(rays or list(range(len(points))))[i],object_id=(objects or ['x']*len(points))[i]) for i,p in enumerate(points)]
def test_straight_tangent_and_expiry():
 f=scan_segments(returns([[0,0],[.02,0],[.04,0]]),[.02,.1],0)
 assert len(f)==1 and f[0].support.shape==(3,2)
 A,h=segment_rows([.02,.07],f,SafetyFilterParams(),0)
 assert len(A)==1 and np.isclose(A[0]@[1.,0.],0)
 assert len(segment_rows([.02,.07],f,SafetyFilterParams(),1)[0])==0

def test_sparse_occluded_different_objects_empty_are_unknown():
 assert not scan_segments([], [0,1],0)
 assert not scan_segments(returns([[0,0],[.2,0],[.4,0]]),[0,1],0)
 assert not scan_segments(returns([[0,0],[.02,0],[.04,0]],[0,1,3]),[0,1],0)
 assert not scan_segments(returns([[0,0],[.02,0],[.04,0]],objects=['x','x','y']),[0,1],0)

def test_convex_concave_and_noisy_corner_not_bridged():
 for sign in (-1,1):
  assert not scan_segments(returns([[0,0],[.02,0],[.02,sign*.02]]),[1,1],0)
 assert not scan_segments(returns([[0,0],[.02,.01],[.04,0]]),[0,1],0)

def test_truth_normal_is_not_consumed():
 r=returns([[0,0],[.02,0],[.04,0]])
 first=scan_segments(r,[0,1],0)
 for x in r: x['true_normal']=np.array([100,-20])
 second=scan_segments(r,[0,1],0)
 np.testing.assert_array_equal(first[0].normal,second[0].normal)
