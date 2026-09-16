"""Barrier-update diagnosis for static sampled theorem_mode.

Reconstructs object-row internals of ``SafetyFilter._object_rows`` and compares
them with true polygon signed distance.  Not used on the control path.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .geometry import closest_point_on_segment, signed_distance_and_gradient
from .safety_filter import SafetyFilter


def _to_list(value):
    if value is None:
        return None
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def nearest_feature(vertices: np.ndarray, point: np.ndarray) -> dict[str, Any]:
    """Closest polygon feature: edge interior or vertex, with supporting data."""
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    p = np.asarray(point, dtype=float).reshape(2)
    best = None
    for i in range(len(v)):
        a, b = v[i], v[(i + 1) % len(v)]
        q, t = closest_point_on_segment(p, a, b)
        d = float(np.linalg.norm(p - q))
        edge = b - a
        length = float(np.linalg.norm(edge))
        n_out = np.array([edge[1], -edge[0]], dtype=float)
        n_norm = float(np.linalg.norm(n_out))
        if n_norm > 1e-15:
            n_out = n_out / n_norm
        kind = "vertex" if t <= 1e-9 or t >= 1.0 - 1e-9 else "edge"
        vertex_index = i if t <= 0.5 else (i + 1) % len(v)
        rec = {
            "edge_index": i,
            "kind": kind,
            "t": float(t),
            "foot": q.tolist(),
            "distance": d,
            "edge_normal": n_out.tolist(),
            "vertex_index": int(vertex_index),
            "edge_length": length,
        }
        if best is None or d < best["distance"] - 1e-15:
            best = rec
        elif best is not None and abs(d - best["distance"]) <= 1e-15 and kind == "vertex":
            best = rec
    assert best is not None
    signed, grad, foot = signed_distance_and_gradient(p[None, :], v)
    best["signed_distance"] = float(signed[0])
    best["sd_gradient"] = grad[0].tolist()
    best["sd_foot"] = foot[0].tolist()
    best["h_true"] = float(signed[0])  # caller subtracts r_safe
    return best


def plane_cuts_object(n_bar: np.ndarray, offset: float, vertices: np.ndarray, tol: float = 1e-9) -> dict[str, Any]:
    """Whether {n·x = offset} is a supporting plane of the polygon.

    Outward n means the object should satisfy n·v <= offset for every vertex.
    A vertex with n·v > offset lies on the outer side: the plane cuts the body.
    """
    n = np.asarray(n_bar, dtype=float).reshape(2)
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    dots = v @ n
    max_dot = float(np.max(dots))
    min_dot = float(np.min(dots))
    cuts_body = bool(max_dot > float(offset) + tol)
    return {
        "max_n_dot_vertex": max_dot,
        "min_n_dot_vertex": min_dot,
        "offset": float(offset),
        "cuts_object": cuts_body,
        "support_gap": float(max_dot - offset),
        "all_vertices_on_inner_side": bool(max_dot <= float(offset) + tol),
    }


def infinite_plane_h(n_bar: np.ndarray, offset: float, position: np.ndarray, r_safe: float) -> float:
    n = np.asarray(n_bar, dtype=float).reshape(2)
    p = np.asarray(position, dtype=float).reshape(2)
    return float(np.dot(n, p) - float(offset) - float(r_safe))


def explain_agent_object_rows(
    filt: SafetyFilter,
    position: np.ndarray,
    boundary_points: np.ndarray,
    boundary_normals: np.ndarray,
    vertices: np.ndarray,
) -> dict[str, Any]:
    trace: dict[str, Any] = {}
    A, rhs, rhs_free, h = filt._object_rows(
        position,
        boundary_points,
        boundary_normals,
        np.zeros(2),
        None,
        trace=trace,
        obstacle_vertices=vertices,
    )
    r_safe = float(filt.params.r_safe)
    feat = nearest_feature(vertices, position)
    feat["h_true"] = float(feat["signed_distance"] - r_safe)
    raw_n = trace.get("n_bar")
    if raw_n is None and len(A):
        raw_n = A[0]
    elif raw_n is None:
        raw_n = [0.0, 0.0]
    n_bar = np.asarray(raw_n, dtype=float).reshape(-1)
    if n_bar.size != 2:
        n_bar = np.array([0.0, 0.0], dtype=float)
    offset = float(trace.get("offset_from_n_k", 0.0)) if trace.get("offset_from_n_k") is not None else None
    support = None
    if offset is not None and len(A):
        support = plane_cuts_object(n_bar, offset, vertices)
    h_true = float(feat["h_true"])
    h_agg = float(np.min(h)) if len(h) else None
    return {
        "trace": trace,
        "A": np.asarray(A, dtype=float).tolist(),
        "rhs": np.asarray(rhs, dtype=float).tolist(),
        "rhs_no_margin": np.asarray(rhs_free, dtype=float).tolist(),
        "h_object": np.asarray(h, dtype=float).tolist(),
        "feature": feat,
        "support": support,
        "h_aggregate": h_agg,
        "h_true": h_true,
        "h_agg_minus_h_true": None if h_agg is None else float(h_agg - h_true),
        "plane_more_conservative_than_sd": None if h_agg is None else bool(h_agg < h_true - 1e-9),
        "zero_in_F_hard": bool(len(rhs_free) == 0 or np.all(np.asarray(rhs_free) <= 1e-9)),
        "zero_in_F_rho": bool(len(rhs) == 0 or np.all(np.asarray(rhs) <= 1e-9)),
        "rho_over_gamma": float(filt.params.rho / filt.params.gamma_obj)
        if filt.params.gamma_obj > 0
        else None,
    }


def match_object_barrier(prev: dict | None, cur: dict) -> dict[str, Any]:
    """Pair aggregated object rows across frames; unmatched new rows are flagged."""
    if prev is None:
        return {"matched": False, "reason": "no_previous_object_row", "new_activation": True}
    n0 = prev.get("n_bar")
    n1 = cur.get("n_bar")
    if n0 is None or n1 is None:
        if n0 is None and n1 is not None:
            return {"matched": False, "reason": "new_object_row", "new_activation": True}
        if n0 is not None and n1 is None:
            return {"matched": False, "reason": "object_row_dropped", "new_activation": False}
        return {"matched": False, "reason": "no_object_row_either_frame", "new_activation": False}
    a = np.asarray(n0, dtype=float).reshape(2)
    b = np.asarray(n1, dtype=float).reshape(2)
    cosine = float(np.clip(np.dot(a, b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-15), -1.0, 1.0))
    offset0 = float(prev.get("offset_from_n_k", 0.0))
    offset1 = float(cur.get("offset_from_n_k", 0.0))
    face_switch = bool(cosine < 0.5)
    return {
        "matched": True,
        "new_activation": False,
        "normal_cosine": cosine,
        "offset_delta": float(offset1 - offset0),
        "face_switch": face_switch,
        "angle_deg": float(np.degrees(np.arccos(cosine))),
    }


def decompose_h(
    p_k: np.ndarray,
    p_next: np.ndarray,
    u_k: np.ndarray,
    dt: float,
    prev: dict | None,
    cur: dict,
) -> dict[str, Any]:
    """Split h_{k+1}(p_{k+1}) - h_k(p_k) into motion vs barrier-update terms.

    Requires a matched affine representation h = n·p - d - r_safe.
    """
    p0 = np.asarray(p_k, dtype=float).reshape(2)
    p1 = np.asarray(p_next, dtype=float).reshape(2)
    u = np.asarray(u_k, dtype=float).reshape(2)
    r_safe = float(cur.get("r_safe", prev.get("r_safe", 0.0) if prev else 0.0))
    h1 = cur.get("h_bar")
    h0 = prev.get("h_bar") if prev else None
    out: dict[str, Any] = {
        "delta_p": (p1 - p0).tolist(),
        "delta_p_minus_dt_u": (p1 - p0 - dt * u).tolist(),
        "h_k": h0,
        "h_kplus": h1,
        "delta_h": None if h0 is None or h1 is None else float(h1 - h0),
    }
    match = match_object_barrier(prev, cur)
    out["match"] = match
    if not match.get("matched") or h0 is None or h1 is None:
        out["unmatched_new_row"] = bool(match.get("new_activation"))
        return out
    n0 = np.asarray(prev.get("n_bar"), dtype=float).reshape(2)
    n1 = np.asarray(cur.get("n_bar"), dtype=float).reshape(2)
    d0 = float(prev.get("offset_from_n_k"))
    d1 = float(cur.get("offset_from_n_k"))
    h_k_at_p1 = float(np.dot(n0, p1) - d0 - r_safe)
    h_k1_at_p1 = float(np.dot(n1, p1) - d1 - r_safe)
    motion = h_k_at_p1 - float(h0)
    update = h_k1_at_p1 - h_k_at_p1
    predicted_motion = float(np.dot(n0, dt * u))
    out.update(
        {
            "h_k_at_p_{k+1}": h_k_at_p1,
            "h_{k+1}_at_p_{k+1}": h_k1_at_p1,
            "motion_term": motion,
            "barrier_update_term": update,
            "sum_check": float(motion + update - (float(h1) - float(h0))),
            "predicted_motion_n_k_dot_dt_u": predicted_motion,
            "motion_minus_predicted": float(motion - predicted_motion),
            "normal_delta": (n1 - n0).tolist(),
            "offset_delta": float(d1 - d0),
            "face_switch": bool(match.get("face_switch")),
            "numerical_rebuild_h0": float(np.dot(n0, p0) - d0 - r_safe - float(h0)),
            "numerical_rebuild_h1": float(h_k1_at_p1 - float(h1)),
        }
    )
    return out


def hold_min_true_clearance(
    p: np.ndarray,
    u: np.ndarray,
    vertices: np.ndarray,
    dt: float,
    n_samples: int = 21,
) -> dict[str, Any]:
    """Sampled hold-segment signed distance plus 1-Lipschitz remainder."""
    p0 = np.asarray(p, dtype=float).reshape(2)
    u0 = np.asarray(u, dtype=float).reshape(2)
    taus = np.linspace(0.0, float(dt), int(n_samples))
    pts = p0[None, :] + taus[:, None] * u0[None, :]
    sd, _, _ = signed_distance_and_gradient(pts, vertices)
    sampled_min = float(np.min(sd))
    h = float(dt) / float(max(n_samples - 1, 1))
    speed = float(np.linalg.norm(u0))
    lipschitz_lb = sampled_min - speed * h
    return {
        "sampled_min_sd": sampled_min,
        "lipschitz_lower_bound": lipschitz_lb,
        "n_samples": int(n_samples),
        "h": h,
        "speed": speed,
    }
