"""Empirical finite scan support; never a certificate for an unseen boundary.

The acquisition safety radius already contains three range-noise standard
deviations. Keep that allowance exactly once, and add the existing half-gap
structural allowance for unsampled shape. No ground-truth field is consumed.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dbact.sensing.features import GAP_GUARD_CAP_M


@dataclass(frozen=True)
class ScanSupport:
    points: np.ndarray
    object_ids: np.ndarray
    origin: np.ndarray
    timestamp: float
    noise_allowance: float
    chords: np.ndarray
    half_gaps: np.ndarray


@dataclass(frozen=True)
class SupportRows:
    ready: bool
    reason: str
    A: np.ndarray
    h: np.ndarray
    lower_distance: np.ndarray
    near_returns: int = 0
    uncovered_returns: int = 0
    point_guard_rows: int = 0


def scan_support(returns, origin, timestamp, noise_std) -> ScanSupport:
    """Pair consecutive rays within one scan/object and the original gap cap."""
    origin = np.asarray(origin,dtype=float).reshape(2)
    timestamp, noise_std = float(timestamp), float(noise_std)
    if not np.isfinite(origin).all() or not np.isfinite(timestamp):
        raise ValueError('Scan origin and time must be finite')
    if not np.isfinite(noise_std) or noise_std < 0.:
        raise ValueError('Range noise must be finite and nonnegative')
    rays = [r['ray_index'] for r in returns]
    if any(not isinstance(ray,(int,np.integer)) or ray < 0 for ray in rays) or len(set(rays)) != len(rays):
        raise ValueError('Ray indices must be unique nonnegative integers')
    items = sorted(returns,key=lambda r:r['ray_index'])
    points = np.asarray([r['point'] for r in items],dtype=float)
    if not items:
        points = np.empty((0,2))
    if points.shape != (len(items),2):
        raise ValueError('Each return must contain exactly one two-dimensional point')
    if not np.isfinite(points).all():
        raise ValueError('Scan points must be finite')
    chords, half_gaps = [], []
    for i,(left,right) in enumerate(zip(items,items[1:])):
        gap = float(np.linalg.norm(points[i+1]-points[i]))
        if (right['ray_index'] == left['ray_index']+1
                and right['object_id'] == left['object_id']
                and 1e-8 <= gap <= 2.*GAP_GUARD_CAP_M):
            chords.append((i,i+1))
            half_gaps.append(.5*gap)
    return ScanSupport(points.copy(),np.asarray([str(r['object_id']) for r in items]),
        origin.copy(),timestamp,3.*noise_std,
        np.asarray(chords,dtype=int).reshape(-1,2),np.asarray(half_gaps,dtype=float))


def support_rows(position, packets, params, timestamp, motion_bound, *, isolated_point_guards=False,
                 noise_in_radius=True) -> SupportRows:
    """Guard current observed coverage, then keep all competing finite chords.

    ``motion_bound`` is a speed ball, independent of the resulting radial normal.
    This function only builds geometry; the safety layer must add that bound to
    each row's demand. Acquisition r_safe includes the noise allowance by default;
    callers using a physical contact floor pass noise_in_radius=False.
    """
    if motion_bound is None or not np.isfinite(motion_bound) or motion_bound < 0.:
        raise ValueError('An explicit finite nonnegative boundary motion bound is required')
    if not isinstance(noise_in_radius, bool):
        raise ValueError('Noise convention must be explicitly boolean')
    p = np.asarray(position,dtype=float).reshape(2)
    if not np.isfinite(p).all() or not np.isfinite(timestamp):
        raise ValueError('Query position and time must be finite')

    def missing(reason, near=0, uncovered=0):
        return SupportRows(False,reason,np.empty((0,2)),np.empty(0),np.empty(0),near,uncovered)

    packets = [packet for packet in packets if abs(timestamp-packet.timestamp) <= 1e-9 and len(packet.points)]
    if not packets:
        return missing('no_current_returns')
    if noise_in_radius and any(packet.noise_allowance > params.r_safe for packet in packets):
        raise ValueError('Acquisition radius must contain the existing range-noise allowance')
    points = np.vstack([packet.points for packet in packets])
    object_ids = np.concatenate([packet.object_ids for packet in packets])
    point_origins = np.vstack([np.tile(packet.origin,(len(packet.points),1)) for packet in packets])
    errors = np.concatenate([np.full(len(packet.points),packet.noise_allowance) for packet in packets])
    covered = np.concatenate([np.isin(np.arange(len(packet.points)),packet.chords.reshape(-1)) for packet in packets])
    distances = np.linalg.norm(points-p,axis=1)
    reach = 2.*(params.max_speed+motion_bound)*params.dt
    near = distances-errors <= np.min(distances+errors)+reach
    missing_count = int(np.count_nonzero(near & ~covered))
    guarded_indices, point_radii = [], []
    if isolated_point_guards:
        # Raw distance cannot exclude a point before its structural allowance is
        # known: a farther return can induce a stricter guarded feature. Build
        # every isolated candidate, then apply the common feature cover below.
        for index in np.flatnonzero(~covered):
            gaps = np.linalg.norm(points-points[index],axis=1)
            eligible = (object_ids == object_ids[index]) & (gaps > 1e-8)
            guarded_indices.append(index)
            # Unknown gaps stay unbounded; over-cap gaps are not truncated.
            # A potentially competing unsupported feature must force fallback.
            point_radii.append(.5*float(gaps[eligible].min()) if np.any(eligible) else np.inf)
    elif missing_count:
        return missing('uncovered_near_return',int(near.sum()),missing_count)
    active = [packet for packet in packets if len(packet.chords)]
    if not active and not guarded_indices:
        return missing('no_chords',int(near.sum()))
    starts = np.vstack([packet.points[packet.chords[:,0]] for packet in active]+[points[guarded_indices]])
    ends = np.vstack([packet.points[packet.chords[:,1]] for packet in active]+[points[guarded_indices]])
    origins = np.vstack([np.tile(packet.origin,(len(packet.chords),1)) for packet in active]+[point_origins[guarded_indices]])
    structural = np.concatenate([packet.half_gaps for packet in active]+[np.asarray(point_radii)])
    noise = np.concatenate([np.full(len(packet.chords),packet.noise_allowance) for packet in active]+[errors[guarded_indices]])
    point_feature = np.r_[np.zeros(sum(len(packet.chords) for packet in active),dtype=bool),
                          np.ones(len(guarded_indices),dtype=bool)]
    ab = ends-starts
    squared_length = np.einsum('ij,ij->i',ab,ab)
    t = np.clip(np.divide(np.einsum('ij,ij->i',p-starts,ab),squared_length,
        out=np.zeros(len(ab)),where=squared_length>1e-16),0.,1.)
    feet = starts+t[:,None]*ab
    delta = p-feet
    distances = np.linalg.norm(delta,axis=1)
    lower = distances-structural-noise
    keep = ((lower <= np.min(distances+structural+noise)+reach)
            & (lower <= params.object_row_range))
    if guarded_indices:
        near[guarded_indices] |= keep[point_feature]
        unsupported = keep & point_feature & (structural > GAP_GUARD_CAP_M)
        if np.any(unsupported):
            reason = ('isolated_gap_unknown' if np.any(~np.isfinite(structural[unsupported]))
                      else 'isolated_gap_exceeds_cap')
            return missing(reason,int(near.sum()),int(unsupported.sum()))
    if not np.any(keep):
        return missing('outside_range',int(near.sum()))
    normals = delta/np.maximum(distances[:,None],1e-12)
    for index in np.flatnonzero(distances <= 1e-12):
        if point_feature[index]:
            normal = origins[index]-feet[index]
            length = float(np.linalg.norm(normal))
            normal = normal/length if length > 1e-12 else np.array([1.,0.])
        else:
            tangent = ab[index]/np.linalg.norm(ab[index])
            normal = np.array([-tangent[1],tangent[0]])
        if normal@(origins[index]-feet[index]) < 0.:
            normal = -normal
        normals[index] = normal
    h = distances-structural-params.r_safe if noise_in_radius else lower-params.r_safe
    return SupportRows(True,'supported',normals[keep],
        h[keep],lower[keep],int(near.sum()),0,
        int(np.count_nonzero(point_feature & keep)))
