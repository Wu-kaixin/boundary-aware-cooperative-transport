"""Bounded, push-only recruitment from own boundary observations and one-hop messages.

This policy never measures contact force, knows the cargo pose, or reduces an
incumbent's pressure. Geometric engagement and group motion response are logged
separately. A response is not a certificate of individual force contribution.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import heapq
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ReinforcementMessage:
    robot: str
    timestamp: float
    state: str
    press: bool=False


@dataclass
class ReinforcementState:
    state: str='idle'
    attempts: int=0
    elapsed: float=0.
    deficit: float=0.
    cooldown: float=0.
    confirm: float=0.
    confirmed: bool=False
    route: list=field(default_factory=list)
    origin: np.ndarray|None=None
    path_length: float=0.
    last_position: np.ndarray|None=None


def observed_route(points,normals,position,neighbours,direction,params):
    """A short connected route through observed offset patches, not a global planner.

    Edges span at most three existing voxels and turn at most 60 degrees.
    Every offset waypoint is checked against currently received neighbours.
    These conservative screens do not certify a swept path; the original QP
    still filters every command. No unobserved point is extrapolated.
    """
    points=np.asarray(points,float);normals=np.asarray(normals,float)
    if not len(points):return []
    clearance=max(params.hold_offset,params.robot_radius+params.voxel_size)
    targets=points+clearance*normals
    start=int(np.linalg.norm(points-position,axis=1).argmin())
    free=np.ones(len(points),bool)
    if len(neighbours):free=np.linalg.norm(targets[:,None]-neighbours[None,:],axis=2).min(axis=1)>=params.d_min
    graph=[[] for _ in points]
    for i,j in sorted(cKDTree(points).query_pairs(3*params.voxel_size)):
        if normals[i]@normals[j]<.5 or not (free[i] and free[j]):continue
        distance=float(np.linalg.norm(targets[i]-targets[j]))
        graph[i].append((j,distance));graph[j].append((i,distance))
    if not free[start]:return []
    costs={start:0.};parents={};queue=[(0.,start)]
    while queue:
        cost,i=heapq.heappop(queue)
        if cost!=costs[i]:continue
        for j,weight in graph[i]:
            new=cost+weight
            if new<=2*params.local_radius and new<costs.get(j,float('inf')):
                costs[j]=new;parents[j]=i;heapq.heappush(queue,(new,j))
    score=-normals@direction
    valid=[i for i in costs if free[i] and score[i]>=params.push_side_threshold]
    if not valid:return []
    selected=max(valid,key=lambda i:(score[i]-.15*costs[i]/max(params.local_radius,1e-9),-i))
    path=[selected]
    while path[-1]!=start:path.append(parents[path[-1]])
    return [targets[i].copy() for i in reversed(path)]


class ContactReinforcement:
    """One frozen candidate: retain, recruit, engage, observe; no donor is required."""
    def __init__(self,params):
        self.params=params
        self.states={}
        self.outbox={}

    def decide(self,robot,points,normals,position,neighbours,direction,velocity,
               remaining,inbox,*,timestamp,dt,active,ready,previous_press,suspended):
        p=self.params;s=self.states.setdefault(robot,ReinforcementState())
        points=np.asarray(points,float).reshape(-1,2);normals=np.asarray(normals,float).reshape(-1,2)
        position=np.asarray(position,float);neighbours=np.asarray(neighbours,float).reshape(-1,2)
        direction=np.asarray(direction,float);velocity=np.asarray(velocity,float)
        dwell=max(dt,p.contact_dwell*dt);lifetime=2*p.local_radius/max(p.max_speed,1e-9)
        floor_speed=p.transport_deadband*p.transport_reference_speed
        # Delivery is restricted by the caller's current one-hop neighbour list.
        messages=[m for m in inbox if 0<=timestamp-m.timestamp<=2*dt+1e-9]
        s.cooldown=max(0.,s.cooldown-dt)
        events=[];command=None;retain=False
        gap=float('inf');alignment=0.
        if len(points):
            near=int(np.linalg.norm(points-position,axis=1).argmin());normal=normals[near]
            gap=float((position-points[near])@normal);alignment=float(-normal@direction)
        if s.last_position is not None and s.state in ('travel','joining','rollback'):
            s.path_length+=float(np.linalg.norm(position-s.last_position))
        s.last_position=position.copy()

        def rollback(reason):
            s.state='rollback';s.elapsed=s.confirm=0.;s.confirmed=False;s.route=[]
            events.append('rollback:'+reason)

        if not active or not len(points) or remaining<=0:
            if s.state!='idle':events.append('phase_or_information_release')
            s.state='idle';s.route=[];s.confirmed=False;s.confirm=s.deficit=0.
        else:
            # Incumbents are never assigned a donor/release fraction. Holding a
            # patch is a controller role, not a declaration of physical contact.
            if s.state=='idle' and previous_press and alignment>p.push_share_floor:
                s.state='support';s.confirmed=False;events.append('retain_incumbent')
            if s.state=='support' and alignment<=p.push_share_floor:
                s.state='idle';s.cooldown=dwell;s.confirmed=False;events.append('direction_no_longer_useful')
            local_pushers=int(previous_press)+sum(m.press for m in messages)
            deficit=float(velocity@direction)<=floor_speed or local_pushers<p.min_push_agents
            s.deficit=s.deficit+dt if deficit else 0.
            if s.state=='idle' and s.cooldown==0 and s.deficit>=dwell and s.attempts<3:
                if not any(m.state in ('request','travel') for m in messages):
                    s.route=observed_route(points,normals,position,neighbours,direction,p)
                    s.cooldown=dwell
                    if s.route:
                        s.state='request';s.elapsed=0.;s.origin=position.copy();events.append('recruit_request')
                    else:events.append('no_observed_free_route')
            elif s.state=='request':
                # Deterministic local arbitration, not global mutual exclusion.
                if any(m.state=='travel' or (m.state=='request' and m.robot<robot) for m in messages):
                    s.state='idle';s.route=[];s.cooldown=dwell;events.append('local_request_deferred')
                else:
                    s.state='travel';s.elapsed=0.;s.attempts+=1;s.path_length=0.;events.append('recruit_started')
            if s.state in ('travel','joining','rollback'):
                s.elapsed+=dt
                s.route=[q+velocity*dt for q in s.route]
                if s.origin is not None:s.origin=s.origin+velocity*dt
            if s.state=='request':command=velocity.copy()
            if s.state=='travel':
                if s.elapsed>=lifetime:rollback('travel_timeout')
                elif not s.route:rollback('empty_route')
                else:
                    target=s.route[0]
                    clearance=max(p.hold_offset,p.robot_radius+p.voxel_size)
                    available=points+clearance*normals
                    j=int(np.linalg.norm(available-target,axis=1).argmin())
                    occupied=len(neighbours) and np.linalg.norm(neighbours-target,axis=1).min()<p.d_min
                    if occupied or np.linalg.norm(available[j]-target)>2*p.voxel_size:
                        rollback('occupied' if occupied else 'patch_lost')
                    else:
                        s.route[0]=available[j].copy();target=s.route[0]
                        if np.linalg.norm(position-target)<=p.voxel_size:
                            s.route.pop(0)
                            command=velocity.copy()
                            if not s.route:
                                s.state='joining';s.elapsed=s.confirm=0.;events.append('arrived_not_confirmed')
                        else:
                            # First leave contact, then follow connected offset
                            # waypoints. The existing safety QP always remains.
                            outward=max(0.,clearance-gap)
                            delta=target-position
                            command=velocity+p.kp_cage*(outward*normal+
                                (delta-float(delta@normal)*normal if outward<=p.voxel_size/2 else 0.))
            if s.state in ('joining','support'):
                if alignment<=p.push_share_floor:
                    if s.state=='joining':rollback('direction_lost')
                    else:s.state='idle';s.cooldown=dwell;events.append('direction_no_longer_useful')
                else:
                    retain=ready
                    # Approach without commanding penetration while suspended;
                    # this point lies in the ORIGINAL geometric readiness band.
                    if not ready or suspended:
                        desired=p.robot_radius+p.voxel_size/2
                        command=velocity+p.kp_press*(gap-desired)*(-normal)
                        retain=False
                    proxy=ready and gap<=p.robot_radius and previous_press and not suspended
                    response=proxy and float(velocity@direction)>floor_speed
                    s.confirm=s.confirm+dt if response else 0.
                    if not response:s.confirmed=False
                    if s.confirm>=dwell and not s.confirmed:
                        s.confirmed=True;s.state='support';events.append('group_response_confirmed')
                    # Joining does not block another spare's recruitment. More
                    # than one new contact may be needed to overcome friction.
                    if s.state=='joining' and s.elapsed>=2*lifetime and not s.confirmed:
                        rollback('response_timeout');retain=False
            if s.state=='rollback':
                retain=False
                if s.origin is None or np.linalg.norm(position-s.origin)<=p.voxel_size or s.elapsed>=lifetime:
                    s.state='idle';s.cooldown=dwell;events.append('rollback_finished');command=None
                else:
                    clearance=max(p.hold_offset,p.robot_radius+p.voxel_size)
                    outward=max(0.,clearance-gap);delta=s.origin-position
                    command=velocity+p.kp_cage*(outward*normal+
                        (delta-float(delta@normal)*normal if outward<=p.voxel_size/2 else 0.))
        if command is not None:
            speed=float(np.linalg.norm(command));command=command*min(1.,p.max_speed/max(speed,1e-9))
        self.outbox[robot]=ReinforcementMessage(robot,float(timestamp),s.state,previous_press)
        trace=dict(reinforcement_state=s.state,reinforcement_event='|'.join(events) or 'none',reinforcement_attempts=s.attempts,
            reinforcement_confirmed=s.confirmed,reinforcement_confirm_seconds=s.confirm,
            reinforcement_path_length=s.path_length,reinforcement_gap=gap,
            reinforcement_retain=retain,reinforcement_suspended_approach=bool(suspended and command is not None))
        return command,retain,trace
