"""Opt-in map motion from the same local initial translation reference as navigation.

No object geometry/pose enters this interface. Stationarity before the capture
epoch and translation afterwards are model assumptions, not safety certificates.
"""
from __future__ import annotations

import numpy as np

from dbact.control.boundary_navigation import point_goal_feedback


def update_reference_maps(controller, agents, neighbors, scans, timestamp, interval):
    # Imported at call time to retain the existing batch implementation exactly.
    from dbact.control.controller import _merge_scans

    batches, codes = {}, {}
    for i, agent in enumerate(agents):
        batches[agent.agent_id], codes[agent.agent_id] = _merge_scans(scans, [i] + list(neighbors[i]))
    prior_status = controller.initial_reference_status
    controller._update_reference(agents, batches, timestamp)
    stationary = (prior_status == "acquiring"
                  and controller.initial_reference_status in ("acquiring", "frozen"))
    object_id, task = next(iter(controller.tasks.items()))
    readings = controller._reference_readings
    displacements = {aid: reading.displacement for aid, reading in readings.items()}
    velocities = {aid: reading.velocity for aid, reading in readings.items()}
    controller._map_reference_state = {}
    for i, agent in enumerate(agents):
        aid = agent.agent_id
        local = controller.maps[aid]
        permitted = [aid] + [agents[j].agent_id for j in neighbors[i]]
        feedback = point_goal_feedback(displacements, velocities, permitted,
            task.distance * task.direction, controller.params.cross_track_deadband, rotation_invariant=True)
        if stationary:
            displacement, velocity, voters = np.zeros(2), np.zeros(2), []
            status = "stationary_acquisition"
        elif feedback is not None:
            displacement, velocity, voters = feedback.displacement, feedback.velocity, list(feedback.voters)
            status = "fresh_reference"
        else:
            controller._map_reference_state[aid] = dict(status="unavailable", voters=[])
            local.motion_quality[object_id] = "unavailable"
            local.prune(timestamp)
            # Preserve the last anchored history for possible reacquisition. A
            # current packet without a motion reference cannot safely refresh it.
            controller._views[aid] = batches[aid]
            continue
        local.align_translation(object_id, displacement, velocity)
        local.register(scans[i], interval, apply_motion=False)
        if local.carve_enabled:
            local.carve(agent.position, scans[i])
        local.update(batches[aid], timestamp, agent_codes=codes[aid], dt=interval, integrate_motion=False)
        controller._views[aid] = local.view(timestamp)
        controller._map_reference_state[aid] = dict(status=status, voters=voters,
            displacement=displacement.tolist(), velocity=velocity.tolist())
