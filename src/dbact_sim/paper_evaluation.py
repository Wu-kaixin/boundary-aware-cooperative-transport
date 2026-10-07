"""Independent paper evaluators. Never imported by the controller or safety layer."""
from __future__ import annotations
import numpy as np
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import nearest_points

SCHEMA_VERSION = 'paper-metrics-v1'

def evaluate_static_hold(positions, velocities, vertices, dt, r_safe, tolerance=1e-9):
    """Whole linear path versus a fixed solid polygon, using GEOS double precision.

    CERTIFIED_SAFE is conditional on the polygon/linear-hold model and numerical
    tolerance; it is not a formal interval-arithmetic or unknown-map certificate.
    Distances are to the solid (zero inside), not negative penetration depths.
    """
    polygon = Polygon(np.asarray(vertices, dtype=float))
    p = np.asarray(positions, dtype=float)
    u = np.asarray(velocities, dtype=float)
    if dt <= 0 or r_safe <= 0 or p.shape != u.shape or p.ndim != 2 or p.shape[1] != 2:
        raise ValueError('invalid hold geometry or threshold')
    if not polygon.is_valid or polygon.is_empty or not np.all(np.isfinite(p)) or not np.all(np.isfinite(u)):
        return {'status':'UNRESOLVED','clearance_path_min':None,'actual_violation_count':None,'robots':[]}
    rows=[]
    for i,(start,velocity) in enumerate(zip(p,u)):
        end=start+dt*velocity
        path=Point(start) if np.array_equal(start,end) else LineString([start,end])
        distance=float(path.distance(polygon))
        witness,_=nearest_points(path,polygon)
        point=np.asarray(witness.coords[0]); delta=end-start
        fraction=float(np.clip((point-start)@delta/(delta@delta),0,1)) if delta@delta>0 else 0.0
        status='OBSERVED_VIOLATION' if distance < r_safe-tolerance else ('CERTIFIED_SAFE' if distance > r_safe+tolerance else 'UNRESOLVED')
        rows.append({'robot_index':i,'clearance_to_solid':distance,'intersects_solid':bool(path.intersects(polygon)),
                     'status':status,'witness_position':point.tolist(),'witness_time_offset':fraction*dt})
    status=('OBSERVED_VIOLATION' if any(r['status']=='OBSERVED_VIOLATION' for r in rows) else
            'CERTIFIED_SAFE' if rows and all(r['status']=='CERTIFIED_SAFE' for r in rows) else 'UNRESOLVED')
    return {'status':status,'clearance_path_min':min((r['clearance_to_solid'] for r in rows),default=None),
            'actual_violation_count':sum(r['status']=='OBSERVED_VIOLATION' for r in rows) if rows else None,'robots':rows}

def progress_errors(start, final, direction, distance):
    start, final, direction = (np.asarray(x,dtype=float) for x in (start,final,direction))
    if not np.isclose(np.linalg.norm(direction),1): raise ValueError('direction must be unit length')
    progress=float((final-start)@direction); error=progress-float(distance)
    return {'true_progress':progress,'signed_progress_error':error,'absolute_progress_error':abs(error),
            'euclidean_endpoint_error':float(np.linalg.norm(final-(start+distance*direction)))}

def event_record(frame, dt, horizon_frames):
    return {'frame':frame,'seconds':None if frame is None else frame*dt,
            'right_censored':frame is None,'observation_end_seconds':horizon_frames*dt}

def upgrade_legacy_static(result):
    """Add unambiguous aliases; never infer actual violations from a bound."""
    value=dict(result)
    value.setdefault('schema_version',SCHEMA_VERSION)
    value.setdefault('clearance_lower_bound_min',value.get('minimum_true_clearance'))
    value.setdefault('lower_bound_check_failed_frames',value.get('object_safety_violation_frames'))
    value.setdefault('actual_violation_observed',None)
    value.setdefault('actual_violation_count',None)
    value.setdefault('safety_evaluation_status','UNRESOLVED')
    return value
