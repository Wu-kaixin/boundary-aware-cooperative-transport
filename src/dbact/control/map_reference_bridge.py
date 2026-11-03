"""Local map motion from common fresh voters, not changing absolute rosters.

Constant per-voter offsets cancel in increments. This empirical bridge does not
certify registration, remove time-varying bias, or supply a navigation/stop vote.
"""
from __future__ import annotations

import numpy as np

from dbact.control.initial_task_reference import geometric_median


class ReferenceMapBridge:
    def __init__(self):
        self.previous = {}
        self.displacement = np.zeros(2)
        self.started = False

    def update(self, displacements, velocities, permitted, *, stationary=False):
        if stationary:
            self.previous = {}
            self.displacement = np.zeros(2)
            self.started = False
            return dict(status='stationary', displacement=np.zeros(2), velocity=np.zeros(2),
                        delta=np.zeros(2), voters=[], common_voters=[], reset_history=False,
                        current={}, previous={}, raw_displacement=np.zeros(2))
        ids = list(dict.fromkeys(a for a in permitted if a in displacements and a in velocities))
        if not ids:
            self.previous = {}
            return None
        current = {a:np.array(displacements[a], dtype=float, copy=True).reshape(2) for a in ids}
        speed = {a:np.array(velocities[a], dtype=float, copy=True).reshape(2) for a in ids}
        if not all(np.isfinite(x).all() for x in [*current.values(), *speed.values()]):
            raise ValueError('Finite current map reference readings required')
        common = [a for a in ids if a in self.previous]
        raw = geometric_median(np.array(list(current.values())))
        before = self.displacement.copy()
        reset = self.started and not common
        if common:
            increment = geometric_median(np.array([current[a]-self.previous[a] for a in common]))
            displacement = before+increment
            velocity = geometric_median(np.array([speed[a] for a in common]))
            status = 'continuous'
        else:
            displacement = raw.copy()
            increment = displacement-before
            velocity = geometric_median(np.array(list(speed.values())))
            status = 'reanchored' if reset else 'anchored'
        result = dict(status=status, displacement=displacement, velocity=velocity,
            delta=increment, voters=ids, common_voters=common, reset_history=reset,
            current={a:current[a].tolist() for a in ids},
            previous={a:d.tolist() for a,d in self.previous.items()}, raw_displacement=raw)
        self.previous = current
        self.displacement = displacement.copy()
        self.started = True
        return result
