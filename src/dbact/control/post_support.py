"""Current own/one-hop packet admission after initial reference acquisition."""


def filter_inputs(controller, agent_id, permitted_ids, timestamp):
    trace = dict(applied=False, reason='outside_frozen_reference', source_agents=[])
    if controller.initial_reference_status != 'frozen':
        return {}, trace
    state = controller._map_reference_state.get(agent_id, {}).get('status', 'unknown')
    if state != 'fresh_reference':
        trace['reason'] = 'no_current_map_motion'
        return {}, trace
    packets = getattr(controller.sensor, 'finite_support_packets', {})
    own = packets.get(agent_id)
    if own is None or not len(own.points) or abs(own.timestamp-timestamp) > 1e-9:
        trace['reason'] = 'no_current_own_returns'
        return {}, trace
    selected = [(aid, packets[aid]) for aid in dict.fromkeys([agent_id, *permitted_ids])
        if aid in packets and abs(packets[aid].timestamp-timestamp) <= 1e-9]
    trace.update(reason='pending_geometry_guard', source_agents=[aid for aid, _ in selected],
        source_timestamps=[packet.timestamp for _, packet in selected],
        motion_model='bounded_received_speed_norm_empirical')
    return dict(post_support_packets=[packet for _, packet in selected],
        observation_time=timestamp, post_support_trace=trace), trace
