from __future__ import annotations

import numpy as np
import pytest

from dbact.controller import DBACTController, DBACTParams
from dbact.progress_control import ProgressPIController, ProgressPIParams
from dbact.types import AgentState, BoundaryObservation
from dbact_sim.scenarios import controller_params_from_config, load_yaml


def test_progress_pi_accelerates_then_commands_counterpressure_for_braking():
    regulator = ProgressPIController(
        ProgressPIParams(target=0.50, max_reference_speed=0.15, effort_limit=0.20)
    )
    drive = regulator.update(progress=0.0, parallel_velocity=0.0, dt=0.05)
    assert drive.position_error == pytest.approx(0.50)
    assert drive.velocity_reference == pytest.approx(0.15)
    assert drive.effort > 0.0

    brake = regulator.update(progress=0.49, parallel_velocity=0.12, dt=0.05, braking=True)
    assert brake.velocity_reference == 0.0
    assert brake.velocity_error < 0.0
    assert brake.effort < 0.0


def test_progress_pi_anti_windup_keeps_integral_bounded_under_saturation():
    params = ProgressPIParams(
        target=10.0,
        max_reference_speed=0.2,
        velocity_ki=2.0,
        effort_limit=0.03,
        integral_limit=0.12,
        anti_windup_gain=2.0,
    )
    regulator = ProgressPIController(params)
    outputs = [regulator.update(0.0, -1.0, 0.1) for _ in range(100)]
    assert any(output.saturated for output in outputs)
    assert abs(regulator.integral) <= params.integral_limit + 1e-12


def test_reset_removes_transport_integral_before_braking():
    regulator = ProgressPIController(
        ProgressPIParams(target=0.1, velocity_ki=1.0, integral_limit=2.0)
    )
    for _ in range(10):
        regulator.update(0.0, 0.0, 0.1)
    assert regulator.integral > 0.0
    regulator.reset()
    output = regulator.update(0.16, 0.0, 0.0, braking=True)
    assert output.integral == 0.0
    assert output.effort < 0.0


def test_feedback_supervisor_requires_brake_before_hold():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.50,
        brake_activation_distance=0.03,
        brake_position_tolerance=0.02,
        brake_speed_tolerance=0.02,
        brake_dwell_steps=2,
        brake_reengage_error=0.06,
        hold_exit_error=0.08,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    observations = [
        BoundaryObservation(
            object_id="obj",
            agent_id="a0",
            point=np.array([4.0, 4.0]),
            normal=np.array([-1.0, 0.0]),
            timestamp=0.0,
        )
    ]
    controller._transport_phase["a0"] = "transport"
    controller._transport_progress["a0"] = {"obj": 0.48}
    controller.object_velocity["a0"] = {"obj": np.array([0.10, 0.0])}
    controller._update_progress_feedback("a0", observations, 0.05, braking=False)
    assert controller._advance_transport_phase("a0", observations, ready=True) == "brake"

    controller._transport_progress["a0"]["obj"] = 0.50
    controller.object_velocity["a0"]["obj"] = np.zeros(2)
    controller._update_progress_feedback("a0", observations, 0.05, braking=True)
    assert controller._advance_transport_phase("a0", observations, ready=True) == "brake"
    assert controller._advance_transport_phase("a0", observations, ready=True) == "hold"


def test_one_hop_wrench_allocation_balances_opposite_contact_torques():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        wrench_allocation=True,
        d_min=0.32,
        transport_distance=0.50,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    agents = [
        AgentState("a0", np.array([-1.13, 0.50])),
        AgentState("a1", np.array([-1.13, -0.50])),
    ]
    observations = {
        "a0": [
            BoundaryObservation("obj", "a0", np.array([-1.0, 0.50]), np.array([-1.0, 0.0]), 0.0),
            BoundaryObservation("obj", "a1", np.array([-1.0, -0.50]), np.array([-1.0, 0.0]), 0.0),
        ],
        "a1": [
            BoundaryObservation("obj", "a0", np.array([-1.0, 0.50]), np.array([-1.0, 0.0]), 0.0),
            BoundaryObservation("obj", "a1", np.array([-1.0, -0.50]), np.array([-1.0, 0.0]), 0.0),
        ],
    }
    controller._transport_progress = {"a0": {"obj": 0.0}, "a1": {"obj": 0.0}}
    controller.object_velocity = {"a0": {"obj": np.zeros(2)}, "a1": {"obj": np.zeros(2)}}
    for agent in agents:
        controller._update_progress_feedback(agent.agent_id, observations[agent.agent_id], 0.05, braking=False)
    controller._update_wrench_allocations(
        agents,
        neighbors=[[1], [0]],
        fused=observations,
        contact_ready=[True, True],
        phases=["transport", "transport"],
    )
    assert controller._wrench_feasible == {"a0": True, "a1": True}
    assert controller._wrench_residuals["a0"] < 1e-8
    assert controller._wrench_weights["a0"] == pytest.approx(controller._wrench_weights["a1"])


def test_object_rows_use_estimated_se2_point_velocity():
    params = DBACTParams(d_min=0.32)
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0))
    observations = [
        BoundaryObservation("obj", "a0", np.array([1.0, 0.0]), np.array([1.0, 0.0]), 0.0),
        BoundaryObservation("obj", "a0", np.array([0.0, 1.0]), np.array([0.0, 1.0]), 0.0),
    ]
    controller.object_velocity = {"a0": {"obj": np.array([0.1, -0.2])}}
    controller.object_angular_velocity = {"a0": {"obj": 0.5}}
    controller._object_centroid = {"a0": {"obj": np.zeros(2)}}
    _, _, point_velocities = controller._object_rows_from_map(
        "a0",
        np.array([1.1, 0.0]),
        observations,
        timestamp=0.0,
    )
    assert np.allclose(point_velocities, [[0.1, 0.3], [-0.4, -0.2]])


def test_contact_release_overrides_inward_cvt_component_on_leading_face():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.10,
        lead_offset=0.22,
        contact_release_gain=3.0,
        contact_release_speed=0.2,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    agent = AgentState("a0", np.array([0.135, 0.0]))
    observations = [
        BoundaryObservation("obj", "a0", np.zeros(2), np.array([1.0, 0.0]), 0.0)
    ]
    released = controller._release_unallocated_contact(
        agent,
        observations,
        nominal=np.array([-0.1, 0.05]),
        drive_direction=np.array([1.0, 0.0]),
        contact_ready=True,
        allocation_weight=0.0,
    )
    assert released[0] > 0.0
    assert released[1] == pytest.approx(0.05)


def test_transport_drive_direction_rejects_estimated_cross_track_drift():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.10,
        cross_track_gain=2.0,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    controller._transport_displacement = {"a0": {"obj": np.array([0.4, 0.2])}}

    forward = controller._transport_drive_direction("a0", "obj", longitudinal_sign=1.0)
    braking = controller._transport_drive_direction("a0", "obj", longitudinal_sign=-1.0)

    assert forward[0] > 0.0
    assert braking[0] < 0.0
    assert forward[1] < 0.0
    assert braking[1] < 0.0


def test_contact_release_prefers_latest_local_safety_scan():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.10,
        lead_offset=0.22,
        contact_release_gain=3.0,
        contact_release_speed=0.2,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    agent = AgentState("a0", np.array([0.135, 0.0]))
    stale_planning = [
        BoundaryObservation("obj", "a0", np.zeros(2), np.array([-1.0, 0.0]), 0.0)
    ]
    controller._safety_observation_cache["a0"] = [
        BoundaryObservation("obj", "a0", np.zeros(2), np.array([1.0, 0.0]), 1.0)
    ]

    released = controller._release_unallocated_contact(
        agent,
        stale_planning,
        nominal=np.array([-0.1, 0.0]),
        drive_direction=np.array([1.0, 0.0]),
        contact_ready=True,
        allocation_weight=1.0,
    )

    assert released[0] > 0.0


def test_contact_robust_margin_detects_exhausted_inward_wrench_capacity():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.10,
        robot_radius=0.16,
        delta_max=0.05,
        boundary_error_bound=0.023,
        gamma_obj=20.0,
        rho=0.35,
        max_speed=0.45,
        cage_offset=0.159,
        lead_offset=0.22,
        contact_release_gain=3.0,
        contact_release_speed=0.2,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    # r_safe + epsilon_b = 0.133; distance 0.158 gives h=0.025,
    # inside the rho/gamma + one-time-constant release guard (0.0275).
    agent = AgentState("a0", np.array([0.158, 0.0]))
    controller._safety_observation_cache["a0"] = [
        BoundaryObservation("obj", "a0", np.zeros(2), np.array([-1.0, 0.0]), 0.0)
    ]
    controller.object_velocity["a0"] = {"obj": np.zeros(2)}

    assert controller._contact_robust_margin(agent, "obj") == pytest.approx(0.0075)


def test_research_config_disables_fixed_feedforward_and_enables_feedback():
    cfg = load_yaml("configs/sim/research/adaptive_progress_closed_loop.yaml")
    params = controller_params_from_config(cfg)
    assert params.progress_feedback is True
    assert params.transport_speed == 0.0
    assert params.transport_progress_estimator == "motion_integral"
    assert params.wrench_allocation is True
    assert params.map_gossip_every == 3


def test_map_gossip_decimation_starts_only_after_certified_search_relay():
    params = DBACTParams(
        search_pattern="paired_lanes",
        map_gossip=True,
        perception_every=3,
        map_gossip_every=3,
        d_min=0.32,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0))
    controller._paired_search_durations = lambda: (1.0, 1.0, 1.0)
    controller._frame = 3
    controller._time = 0.5
    assert not controller._map_gossip_due(refresh_perception=True)

    controller._frame = 3
    controller._time = 2.5
    assert controller._map_gossip_due(refresh_perception=True)

    controller._time = 3.5
    controller._frame = 12  # perception index 4: not a decimated gossip cycle
    assert not controller._map_gossip_due(refresh_perception=True)
    controller._frame = 18  # perception index 6: one full-map exchange
    assert controller._map_gossip_due(refresh_perception=True)
    assert not controller._map_gossip_due(refresh_perception=False)


def test_communication_dropout_is_symmetric_deterministic_and_measured():
    params = DBACTParams(
        task_mode="transport",
        progress_feedback=True,
        d_min=0.32,
        transport_distance=0.1,
        communication_dropout_prob=0.5,
    )
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0), {"obj": np.array([1.0, 0.0])})
    agents = [AgentState("a0", np.array([0.0, 0.0])), AgentState("a1", np.array([0.5, 0.0]))]
    physical = [[1], [0]]
    outcomes = []
    for frame in range(100):
        controller._frame = frame
        delivered = controller._communication_neighbor_indices(agents, physical)
        assert (1 in delivered[0]) == (0 in delivered[1])
        outcomes.append(bool(delivered[0]))
    assert any(outcomes) and not all(outcomes)
    assert controller.communication_candidate_links == 100
    assert controller.communication_delivery_rate == pytest.approx(sum(outcomes) / 100)


def test_controller_rejects_rho_above_kinematic_authority():
    with pytest.raises(ValueError, match="rho must be strictly below max_speed"):
        DBACTController(
            DBACTParams(
                task_mode="transport",
                progress_feedback=True,
                d_min=0.32,
                transport_distance=0.1,
                rho=0.3,
                max_speed=0.3,
            ),
            (0.0, 8.0, 0.0, 8.0),
            {"obj": np.array([1.0, 0.0])},
        )
