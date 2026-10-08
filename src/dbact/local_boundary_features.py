"""Fresh, scan-adjacent empirical segments; never a complete-map certificate."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class ObservedSegment:
    start: np.ndarray
    end: np.ndarray
    support: np.ndarray
    normal: np.ndarray
    fit_residual: float
    empirical_error: float
    timestamp: float
    confidence: float
    object_id: str
    ray_indices: tuple


def scan_segments(returns, origin, timestamp, *, max_gap=.08, residual_max=.004, noise_std=0.):
    """Accept only triples of consecutive rays on one observed object/flat patch.

    Input consumes point, ray_index and object_id only (never true_normal/edge).
    Three samples cannot rule out an unobserved feature between rays; even an
    accepted segment is EMPIRICAL_ONLY. No wrapping across missing rays.
    """
    items=sorted(returns,key=lambda x:x['ray_index']); output=[]
    for triple in zip(items,items[1:],items[2:]):
        rays=tuple(int(x['ray_index']) for x in triple)
        if rays[1]!=rays[0]+1 or rays[2]!=rays[1]+1: continue
        if len({x['object_id'] for x in triple})!=1: continue
        points=np.asarray([x['point'] for x in triple],dtype=float)
        gaps=np.linalg.norm(np.diff(points,axis=0),axis=1)
        if np.any(gaps>max_gap) or np.any(gaps<1e-8): continue
        centered=points-points.mean(axis=0)
        _,_,vh=np.linalg.svd(centered,full_matrices=False)
        normal=vh[-1]; residual=float(np.sqrt(np.mean((centered@normal)**2)))
        tangents=np.diff(points,axis=0)/gaps[:,None]
        if residual>residual_max or float(tangents[0]@tangents[1])<.98: continue
        if normal@(np.asarray(origin)-points.mean(axis=0))<0: normal=-normal
        error=residual+3*float(noise_std)
        output.append(ObservedSegment(points[0].copy(),points[-1].copy(),points.copy(),normal.copy(),residual,error,float(timestamp),float(max(0,1-residual/residual_max)),str(triple[0]['object_id']),rays))
    return output


def gap_guarded_samples(origin, points, normals):
    """Pull each sample toward the sensor by half the local gap.

    A sample point is not the true nearest boundary. On L seed 2 the nearest
    stored sample was 76.6 mm from the robot while the convex corner, lying
    between two samples 40.5 mm apart, was 65.1 mm away. Half that chord is a
    geometric allowance for the unobserved corner, not a change of r_safe.
    """
    origin = np.asarray(origin, dtype=float).reshape(2)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    normals = np.asarray(normals, dtype=float).reshape(-1, 2)
    if len(points) < 2:
        return points, normals
    guarded = points.copy()
    for index, point in enumerate(points):
        others = np.linalg.norm(points - point, axis=1)
        others[index] = np.inf
        gap = float(np.min(others))
        if not np.isfinite(gap) or gap <= 0.0:
            continue
        direction = origin - point
        length = float(np.linalg.norm(direction))
        if length < 1e-9:
            continue
        guarded[index] = point + direction / length * min(0.5 * gap, 0.04)
    return guarded, normals


def segment_rows(position, features, params, timestamp):
    """All observed segments in the one-hold switch cover, without row truncation."""
    features=[f for f in features if 0 <= timestamp-f.timestamp <= params.dt+1e-9]
    if not features: return np.empty((0,2)),np.empty(0)
    p=np.asarray(position); a=np.asarray([f.start for f in features]); b=np.asarray([f.end for f in features]); ab=b-a
    t=np.clip(np.sum((p-a)*ab,axis=1)/np.sum(ab*ab,axis=1),0,1)
    feet=a+t[:,None]*ab; delta=p-feet; distances=np.linalg.norm(delta,axis=1)
    error=np.asarray([f.empirical_error for f in features])
    # Include uncertainty in both competing-distance and reach comparisons.
    keep=(distances-error <= np.min(distances+error)+2*params.max_speed*params.dt)&(distances-error<=params.object_row_range)
    normals=np.divide(delta,np.maximum(distances[:,None],1e-12))
    for i in np.where(distances<=1e-12)[0]: normals[i]=features[i].normal
    return normals[keep], (distances-error-params.r_safe)[keep]
