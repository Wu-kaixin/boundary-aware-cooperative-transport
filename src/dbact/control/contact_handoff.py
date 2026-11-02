"""Opt-in local, push-only contact handoff with acknowledgement and rollback.

Messages are one-hop, previous-frame snapshots. A local geometric gap, a press
command and sustained estimated motion form an *operational proxy*, not a force
measurement, physical-contact certificate or proof of individual contribution.
Incumbents keep the original pressure law until this proxy admits a trial unload.
No object truth, global boundary or other robot's private map is accepted here.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from dbact.control.contact_roles import redeployment_velocity, select_observed_target


@dataclass(frozen=True)
class HandoffMessage:
    robot: str
    timestamp: float
    state: str = 'idle'
    partner: str = ''
    donor_state: str = 'idle'
    donor_partner: str = ''
    normal: tuple[float, float] = (0., 0.)
    target_normal: tuple[float, float] = (0., 0.)
    press: bool = False
    contact_proxy: bool = False
    usable: bool = False


@dataclass
class HandoffState:
    state: str = 'idle'
    partner: str = ''
    target: np.ndarray | None = None
    origin: np.ndarray | None = None
    target_normal: np.ndarray = field(default_factory=lambda: np.zeros(2))
    elapsed: float = 0.
    confirm: float = 0.
    bad: float = 0.
    deficit: float = 0.
    cooldown: float = 0.
    attempts: int = 0
    donor_state: str = 'idle'
    donor_partner: str = ''
    fraction: float = 0.
    reference_velocity: float = 0.
    donor_bad: float = 0.
    samples: deque = field(default_factory=deque)


class ConfirmedContactHandoff:
    """One frozen candidate; at most three recruitment attempts per robot.

    Pressure reallocation is restricted to an acknowledged pair. Unrelated
    original pushers retain their original membership and share. Only a robot
    outside the original pushing arc may volunteer for relocation. A donor
    acknowledges one volunteer; a busy one-hop neighbourhood suppresses new
    acknowledgements. This is not a global mutual-exclusion guarantee.
    """

    def __init__(self, params):
        self.params = params
        self.states: dict[str, HandoffState] = {}
        self.outbox: dict[str, HandoffMessage] = {}

    def decide(self, robot, points, normals, position, neighbours, direction, goal,
               lateral, velocity, dt, timestamp, inbox, *, active, usable,
               previous_press=False, reference_speed=None):
        p = self.params
        s = self.states.setdefault(robot, HandoffState())
        points = np.asarray(points, float).reshape(-1, 2)
        normals = np.asarray(normals, float).reshape(-1, 2)
        position = np.asarray(position, float)
        neighbours = np.asarray(neighbours, float).reshape(-1, 2)
        dwell = max(p.dt, p.contact_dwell * p.dt)
        lifetime = 2 * p.local_radius / max(p.max_speed, 1e-9)
        floor_speed = p.transport_deadband * p.transport_reference_speed
        clearance = max(p.hold_offset, p.robot_radius + p.voxel_size)
        # No queued/stale/relayed-through-another-robot permissions.
        messages = {m.robot:m for m in inbox if 0 <= timestamp-m.timestamp <= 2*dt+1e-9}
        s.cooldown = max(0., s.cooldown-dt)
        event = 'none'
        rollback_reason = ''
        override, command = None, None
        if len(points):
            near = int(np.linalg.norm(points-position, axis=1).argmin())
            normal = normals[near].copy()
            normal /= max(float(np.linalg.norm(normal)), 1e-9)
            gap = float((position-points[near]) @ normal)
        else:
            near, normal, gap = 0, np.zeros(2), float('inf')
        contact = bool(len(points) and 0 <= gap <= p.robot_radius and usable)
        baseline_member = float(normal @ goal) <= -p.push_side_threshold
        share = float(np.clip(-normal @ direction, p.push_share_floor, 1.))
        forward = float(velocity @ goal)
        lateral_norm = float(np.linalg.norm(lateral))
        lateral_speed = float(velocity @ lateral / max(lateral_norm, 1e-9))
        s.samples.append((forward, lateral_speed))
        while len(s.samples) > max(1, round(dwell/dt)):
            s.samples.popleft()
        median_forward, median_lateral = np.median(np.asarray(s.samples), axis=0)
        motion_ok = (usable and len(s.samples) >= max(1, round(dwell/dt))
                     and median_forward > floor_speed
                     and (lateral_norm <= p.cross_track_deadband or median_lateral <= floor_speed))
        deficit = (lateral_norm > p.cross_track_deadband or median_forward <= floor_speed)
        s.deficit = s.deficit+dt if deficit else 0.

        def clear_donor(reason):
            nonlocal event, rollback_reason
            s.donor_state, s.donor_partner = 'idle', ''
            s.fraction = s.donor_bad = 0.
            s.cooldown = dwell
            event = 'rollback:' + reason
            rollback_reason = reason

        def rollback(reason):
            nonlocal event, rollback_reason
            if s.state != 'rollback':
                s.state, s.elapsed, s.confirm, s.bad = 'rollback', 0., 0., 0.
                event = 'rollback:' + reason
                rollback_reason = reason

        if not active or not len(points):
            if s.state != 'idle' or s.donor_state != 'idle': event = 'phase_or_information_abort'
            s.state, s.partner, s.donor_state, s.donor_partner = 'idle', '', 'idle', ''
            s.target = s.origin = None
            s.fraction = s.confirm = s.bad = s.deficit = 0.
        else:
            # DONOR: no pressure reduction during offer/travel/initial engagement.
            if s.donor_partner:
                incoming = messages.get(s.donor_partner)
                linked = incoming is not None and incoming.partner == robot
                if not linked or incoming.state in ('idle', 'rollback'):
                    clear_donor('replacement_or_link_lost')
                elif s.donor_state == 'holding' and incoming.state in ('ready', 'support'):
                    if incoming.contact_proxy and incoming.press and incoming.usable and motion_ok:
                        s.donor_state = 'trial'
                        s.reference_velocity = float(median_forward)
                        event = 'trial_unload'
                if s.donor_state in ('trial', 'retired'):
                    reference = p.transport_reference_speed if reference_speed is None else reference_speed
                    min_speed = .8 * min(s.reference_velocity, reference)
                    response = (linked and incoming.contact_proxy and incoming.press and incoming.usable
                                and usable and median_forward >= min_speed
                                and (lateral_norm <= p.cross_track_deadband or median_lateral <= floor_speed))
                    s.donor_bad = 0. if response else s.donor_bad+dt
                    if s.donor_bad >= dwell:
                        clear_donor('unload_response_lost')
                    elif response:
                        s.fraction = min(1., s.fraction + dt/dwell)
                        if s.fraction >= 1. and s.donor_state != 'retired':
                            s.donor_state = 'retired'
                            event = 'release_complete'
                    if s.donor_state in ('trial', 'retired'):
                        override = share*(1.-s.fraction)
            if not s.donor_partner and s.state == 'idle' and s.cooldown == 0. and previous_press and contact:
                busy = any(m.donor_state != 'idle' or m.state in ('travel','engage','ready','support')
                           for m in messages.values())
                offers = [m for m in messages.values() if m.state == 'offer' and m.partner == robot
                          and -np.asarray(m.target_normal) @ direction > -normal @ direction+p.push_share_floor
                          and -np.asarray(m.target_normal) @ goal >= 0.]
                if offers and not busy:
                    chosen = min(offers, key=lambda m:m.robot)
                    s.donor_partner, s.donor_state = chosen.robot, 'holding'
                    event = 'accepted_keep_pressing'

            # INCOMING: only spare robots volunteer; no incumbent departs first.
            if s.state == 'idle' and not s.donor_partner and not baseline_member and not previous_press:
                if s.cooldown == 0. and s.deficit >= dwell and s.attempts < 3 and usable:
                    target, target_normal = None, None
                    correcting_here = (-normal @ goal >= 0. and -normal @ direction > p.push_share_floor
                                       and (lateral_norm <= p.cross_track_deadband or normal @ lateral > 0.))
                    if correcting_here and gap <= p.cage_offset+p.contact_band_tolerance:
                        target, target_normal = position.copy(), normal.copy()
                    else:
                        target = select_observed_target(points,normals,position,neighbours,direction,
                            clearance=clearance,radius=p.local_radius,separation=p.d_min,
                            improvement=p.push_share_floor)
                        if target is not None:
                            target_normal = normals[int(np.linalg.norm(points-target,axis=1).argmin())]
                            if -target_normal @ goal < 0.: target = None
                    donors = [m for m in messages.values() if m.press and m.contact_proxy and m.usable
                              and m.donor_state == 'idle' and m.state == 'idle'
                              and target is not None and -target_normal @ direction > -np.asarray(m.normal) @ direction+p.push_share_floor]
                    if donors:
                        donor = min(donors, key=lambda m:(float(-np.asarray(m.normal) @ direction), m.robot))
                        s.state,s.partner,s.target,s.origin = 'offer',donor.robot,target,position.copy()
                        s.target_normal = np.asarray(target_normal).copy()
                        s.elapsed = s.confirm = s.bad = 0.
                        s.attempts += 1
                        event = 'proposed'
                    else:
                        event = 'no_observed_replacement_pair'

            if s.state != 'idle':
                s.elapsed += dt
                if s.target is not None: s.target = s.target + velocity*dt
                if s.origin is not None: s.origin = s.origin + velocity*dt
                donor = messages.get(s.partner)
                acknowledged = donor is not None and donor.donor_partner == robot and donor.donor_state != 'idle'
                if s.state == 'offer':
                    if acknowledged:
                        s.state = 'travel'
                        s.elapsed = 0.
                        event = 'travel_authorized'
                    elif s.elapsed >= dwell:
                        rollback('offer_unacknowledged')
                elif s.state not in ('rollback',) and not acknowledged:
                    rollback('donor_or_link_lost')
                if s.state == 'travel':
                    occupied = len(neighbours) and np.linalg.norm(neighbours-s.target,axis=1).min() < p.d_min
                    lost = np.linalg.norm(points-s.target,axis=1).min() > clearance+2*p.voxel_size
                    if occupied or lost or s.elapsed >= lifetime:
                        rollback('occupied' if occupied else 'map_lost' if lost else 'travel_expired')
                    elif np.linalg.norm(s.target-position) <= p.voxel_size:
                        s.state, s.elapsed = 'engage', 0.
                        event = 'arrived_not_confirmed'
                    else:
                        command = redeployment_velocity(position,points[near],normal,s.target,velocity,
                            clearance=clearance,gain=p.kp_cage,max_speed=p.max_speed)
                if s.state in ('engage','ready','support'):
                    eligible = -normal @ goal >= 0. and -normal @ direction > p.push_share_floor
                    if not eligible or not usable:
                        rollback('patch_or_estimate_lost')
                    else:
                        override = share
                        qualified = contact and previous_press and motion_ok
                        s.confirm = s.confirm+dt if qualified else 0.
                        if s.state == 'engage':
                            if s.confirm >= dwell:
                                s.state = 'ready'
                                event = 'locally_confirmed'
                            elif s.elapsed >= 2*lifetime:
                                rollback('confirmation_expired')
                        else:
                            s.bad = 0. if qualified else s.bad+dt
                            if s.bad >= dwell: rollback('confirmation_lost')
                            elif donor is not None and donor.donor_state == 'retired': s.state = 'support'
                if s.state == 'rollback':
                    # Notify first; wait at most one round trip for donor restoration.
                    override = None
                    restored = donor is not None and donor.donor_state == 'idle'
                    if not restored and s.elapsed <= 2*dt and usable and previous_press:
                        override = share
                    else:
                        command = redeployment_velocity(position,points[near],normal,s.origin,velocity,
                            clearance=clearance,gain=p.kp_cage,max_speed=p.max_speed)
                        if np.linalg.norm(s.origin-position) <= p.voxel_size or s.elapsed >= lifetime:
                            event = 'rollback_returned' if np.linalg.norm(s.origin-position) <= p.voxel_size else 'rollback_return_expired'
                            s.state, s.partner, s.target, s.origin = 'idle','',None,None
                            s.cooldown, s.deficit = dwell, 0.
                            command = None

        self.outbox[robot] = HandoffMessage(robot,float(timestamp),s.state,s.partner,s.donor_state,s.donor_partner,
            tuple(normal),tuple(s.target_normal),bool(previous_press),contact,bool(usable and active))
        trace = {'handoff_state':s.state, 'handoff_donor_state':s.donor_state,
                 'handoff_partner':s.partner or s.donor_partner, 'handoff_event':event,
                 'handoff_rollback_reason':rollback_reason,
                 'handoff_attempts':s.attempts, 'handoff_contact_proxy':contact,
                 'handoff_confirm_seconds':s.confirm, 'handoff_release_fraction':s.fraction,
                 'handoff_gap':gap, 'handoff_motion_ok':bool(motion_ok),
                 'handoff_local_forward_velocity':float(median_forward),
                 'handoff_local_lateral_velocity':float(median_lateral),
                 'contact_selected_normal':normal, 'contact_desired_direction':direction,
                 'contact_lateral_estimate':lateral}
        return override, command, trace
