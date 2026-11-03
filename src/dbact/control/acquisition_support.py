"""Current local packets for the acquisition-only finite-support experiment."""


def filter_inputs(controller, agent_id, permitted_ids, timestamp):
    state = controller._map_reference_state.get(agent_id,{}).get('status','unknown')
    trace = dict(applied=False,reason='outside_stationary_acquisition',stage=state,source_agents=[])
    if state != 'stationary_acquisition' or controller.initial_reference_status != 'acquiring':
        return {},trace
    packets = getattr(controller.sensor,'finite_support_packets',{})
    own = packets.get(agent_id)
    if own is None or abs(timestamp-own.timestamp)>1e-9 or not len(own.points):
        trace['reason'] = 'no_current_own_returns'
        return {},trace
    selected = [(aid,packets[aid]) for aid in dict.fromkeys(permitted_ids)
                if aid in packets and abs(timestamp-packets[aid].timestamp)<=1e-9]
    trace.update(reason='pending_geometry_guard',source_agents=[aid for aid,_ in selected],
                 source_timestamps=[packet.timestamp for _,packet in selected],
                 motion_model='existing_stationary_acquisition_assumption')
    return dict(finite_support_packets=[packet for _,packet in selected],
        observation_time=timestamp,finite_support_motion_bound=0.,finite_support_trace=trace),trace
