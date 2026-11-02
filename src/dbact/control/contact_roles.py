"""Experimental push-only contact roles from a robot's own boundary map.

The cone residual below is a *geometric diagnostic*: unit normal directions have
no measured force limits, torque model or guaranteed contact. A small residual
does not certify transport. No cargo pose, physical contact or global map enters
this module. Redeployment follows only a short observed patch, under the existing
safety filter; it is not a collision-free global path planner.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import nnls


@dataclass(frozen=True)
class ContactRoles:
    weights: np.ndarray
    normals: np.ndarray
    observed: np.ndarray
    residual: float


def local_contact_roles(points, normals, positions, direction, contact_band) -> ContactRoles:
    """Nonnegative directional allocation using only one robot's observations.

Positions are [self, received neighbours]. A neighbour's normal is inferred from
the observer's map, not read from that neighbour's private map. The small fixed
regularizer selects a distributed solution in redundant geometry. Coefficients
are relative press weights, normalized to at most one, never tensile forces.
"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    normals = np.asarray(normals, dtype=float).reshape(-1, 2)
    positions = np.asarray(positions, dtype=float).reshape(-1, 2)
    direction = np.asarray(direction, dtype=float).reshape(2)
    count = len(positions)
    if not len(points) or not count:
        return ContactRoles(np.zeros(count), np.zeros((count, 2)), np.zeros(count, bool), 1.)
    distances = np.linalg.norm(positions[:, None] - points[None, :], axis=2)
    nearest = distances.argmin(axis=1)
    selected = normals[nearest].copy()
    lengths = np.linalg.norm(selected, axis=1)
    valid = lengths > 1e-9
    selected[valid] /= lengths[valid, None]
    observed = (distances[np.arange(count), nearest] <= contact_band) & valid
    eligible = observed & ((-selected @ direction) > 0.)
    weights = np.zeros(count)
    if np.any(eligible):
        matrix = -selected[eligible].T
        # Fixed before the development batch; not fitted to task outcomes.
        system = np.vstack((matrix, .01 * np.eye(matrix.shape[1])))
        coefficients, _ = nnls(system, np.r_[direction, np.zeros(matrix.shape[1])])
        residual = float(np.linalg.norm(direction - matrix @ coefficients))
        peak = float(coefficients.max())
        if peak > 0.:
            weights[eligible] = coefficients / peak
    else:
        residual = float(np.linalg.norm(direction))
    return ContactRoles(weights, selected, observed, residual)


def select_observed_target(points, normals, position, neighbour_positions, direction,
                           *, clearance, radius, separation, improvement=.15):
    """A nearby, unoccupied, better aligned patch; no invented unseen boundary.

    A short tangential displacement and compatible normals reject a straight
    shortcut across a concavity. This conservative screen is not path feasibility
    proof. The QP may still prevent travel, which is logged as a failed attempt.
    """
    points = np.asarray(points, float).reshape(-1, 2)
    normals = np.asarray(normals, float).reshape(-1, 2)
    if not len(points):
        return None
    position = np.asarray(position, float)
    neighbours = np.asarray(neighbour_positions, float).reshape(-1, 2)
    nearest = int(np.linalg.norm(points - position, axis=1).argmin())
    normal = normals[nearest]
    tangent = np.array([-normal[1], normal[0]])
    targets = points + clearance * normals
    advantage = -normals @ direction
    displacement = points - points[nearest]
    distance = np.linalg.norm(targets - position, axis=1)
    valid = ((np.linalg.norm(displacement, axis=1) <= radius)
             & (np.abs(displacement @ tangent) > separation / 2)
             & ((normals @ normal) > 0.)
             & (advantage > max(0., float(advantage[nearest])) + improvement))
    if len(neighbours):
        near = np.linalg.norm(targets[:, None] - neighbours[None, :], axis=2)
        # Do not displace an occupant or race a closer neighbour to the patch.
        valid &= near.min(axis=1) > separation
        valid &= near.min(axis=1) >= distance
    indices = np.flatnonzero(valid)
    if not len(indices):
        return None
    score = advantage[indices] - distance[indices] / max(radius, 1e-9) * .15
    return targets[indices[int(score.argmax())]].copy()


def redeployment_velocity(position, point, normal, target, velocity, *, clearance, gain, max_speed):
    """Exit the contact before moving tangentially; never command inward pull."""
    normal = np.asarray(normal, float)
    normal = normal / max(float(np.linalg.norm(normal)), 1e-9)
    measured = float((position - point) @ normal)
    deficit = max(0., clearance - measured)
    relative = gain * deficit * normal
    if deficit <= .01:  # fixed geometric switching band, not an evaluation tolerance
        delta = np.asarray(target) - position
        relative += gain * (delta - float(delta @ normal) * normal)
    result = np.asarray(velocity) + relative
    speed = float(np.linalg.norm(result))
    return result * min(1., max_speed / max(speed, 1e-9))


@dataclass
class _RobotState:
    target: np.ndarray | None = None
    elapsed: float = 0.
    stalled: float = 0.
    deficit_dwell: float = 0.
    cooldown: float = 0.
    attempts: int = 0


class DynamicContactPolicy:
    """One bounded, opt-in v1 policy. Constants are frozen in its protocol.

    Keep useful contacts, recruit lateral contacts with nonnegative weights, and
    allow at most three locally committed relocations per robot. The original
    progress readout, phase supervisor and stopping predicates remain in charge.
    """

    def __init__(self, params):
        self.params = params
        self.states: dict[str, _RobotState] = {}

    def decide(self, agent_id, points, normals, position, neighbours, direction,
               goal, lateral, velocity, dt, *, active):
        p = self.params
        state = self.states.setdefault(agent_id, _RobotState())
        state.cooldown = max(0., state.cooldown - dt)
        positions = np.vstack((position, np.asarray(neighbours).reshape(-1, 2)))
        band = p.cage_offset + p.contact_band_tolerance
        roles = local_contact_roles(points, normals, positions, direction, band)
        trace = {"contact_role": "inactive", "contact_cone_residual": roles.residual,
                 "contact_allocation": float(roles.weights[0]),
                 "contact_redeploy_attempts": state.attempts,
                 "contact_redeploy_reason": "inactive",
                 "contact_selected_normal": roles.normals[0],
                 "contact_desired_direction": direction,
                 "contact_lateral_estimate": lateral}
        if not active or not len(points):
            state.target = None
            state.stalled = state.deficit_dwell = 0.
            return roles, None, trace

        if not p.dynamic_contact_relocation:
            state.target = None
            trace.update(contact_redeploy_reason="allocation_only",
                         contact_role="press" if roles.weights[0] > .05 else "yield")
            return roles, None, trace

        dwell = max(p.dt, p.contact_dwell * p.dt)
        state.stalled = (state.stalled + dt if float(velocity @ goal)
                         < p.transport_deadband * p.transport_reference_speed else 0.)
        cross = float(np.linalg.norm(lateral)) > p.cross_track_deadband
        normal = roles.normals[0]
        correcting = float(normal @ lateral) > 0.
        weak = float(-normal @ direction) < 1. - p.push_share_floor
        deficit = (cross and (not correcting or roles.residual > .25)) or (state.stalled >= dwell and weak)
        state.deficit_dwell = state.deficit_dwell + dt if deficit else 0.
        clearance = max(p.hold_offset, p.robot_radius + p.voxel_size)
        lifetime = 2. * p.local_radius / max(p.max_speed, 1e-9)
        reason = "current_contact" if roles.weights[0] > .05 else "yield"
        if state.target is not None:
            # Advect by this robot's local motion estimate; never true object pose.
            state.target = state.target + velocity * dt
            state.elapsed += dt
            arrival = float(np.linalg.norm(state.target - position)) <= p.voxel_size
            occupied = (len(neighbours) > 0 and float(np.linalg.norm(
                np.asarray(neighbours) - state.target, axis=1).min()) < p.d_min)
            lost = float(np.linalg.norm(np.asarray(points) - state.target, axis=1).min()) > clearance + 2*p.voxel_size
            expired = state.elapsed >= lifetime
            if arrival or occupied or lost or expired:
                reason = "arrived" if arrival else "occupied" if occupied else "map_lost" if lost else "lease_expired"
                state.target = None
                state.cooldown = dwell
                state.deficit_dwell = 0.
            else:
                reason = "committed"
        if (state.target is None and state.cooldown == 0. and state.deficit_dwell >= dwell
                and state.attempts < 3):
            target = select_observed_target(
                points, normals, position, neighbours, direction, clearance=clearance,
                radius=p.local_radius, separation=p.d_min, improvement=p.push_share_floor)
            reason = "no_local_target"
            if target is not None:
                state.target = target
                state.elapsed = 0.
                state.attempts += 1
                reason = "committed_new"
        trace.update(contact_redeploy_reason=reason, contact_redeploy_attempts=state.attempts,
                     contact_role="press" if roles.weights[0] > .05 else "yield")
        if state.target is None:
            return roles, None, trace
        nearest = int(np.linalg.norm(np.asarray(points) - position, axis=1).argmin())
        command = redeployment_velocity(
            position, np.asarray(points)[nearest], np.asarray(normals)[nearest], state.target,
            velocity, clearance=clearance, gain=p.kp_cage, max_speed=p.max_speed)
        trace.update(contact_role="redeploy", contact_redeploy_target=state.target.copy())
        return roles, command, trace
