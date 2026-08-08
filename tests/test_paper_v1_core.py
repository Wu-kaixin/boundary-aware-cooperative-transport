"""Unit tests for paper-v1 boundary-aware core modules."""

from __future__ import annotations

import numpy as np

from dbact.boundary_density import BoundaryAwareDensity
from dbact.boundary_map import LocalBoundaryMap
from dbact.cargo import Cargo
from dbact.distributed_cbf import DistributedCBFQP
from dbact.local_cvt import LocalCVT
from dbact.local_sensing import LocalBoundarySensor, _stable_rng_seed
from dbact.types import AgentState, BoundaryObservation


def test_raycast_occludes_back_side_of_l_shape():
    """A robot facing one arm of an L must not see the occluded far arm."""
    cargo = Cargo.l_shape("L", center=[0.0, 0.0], scale=1.0)
    # Stand south of the bottom arm; north-side recess should be occluded.
    agent = AgentState("a0", position=np.array([0.0, -1.2]))
    sensor = LocalBoundarySensor(sensor_range=2.5, num_rays=120, max_points_per_object=40)
    obs = sensor.sense(agent, [cargo], timestamp=0.0)
    assert obs, "expected some visible boundary hits"
    ys = np.array([o.point[1] for o in obs])
    # Hits should concentrate on the south-facing exterior, not deep into the pocket.
    assert float(np.max(ys)) < 0.2
    # Normals should generally point toward / past the agent (outside).
    for o in obs:
        assert float(np.dot(o.normal, agent.position - o.point)) > -1e-6


def test_sensor_seed_is_process_independent_digest():
    assert _stable_rng_seed(7, "agent_03", 1.25) == 13852681213353130009


def test_boundary_map_voxel_dedup_and_decay():
    m = LocalBoundaryMap(ttl=5.0, voxel_size=0.1, decay_lambda=0.5, fusion="confidence_priority")
    a = BoundaryObservation("obj", "a0", np.array([0.01, 0.0]), np.array([1.0, 0.0]), 0.0, confidence=0.4)
    b = BoundaryObservation("obj", "a1", np.array([0.02, 0.0]), np.array([1.0, 0.0]), 0.1, confidence=0.9)
    m.update([a], 0.0)
    m.update([b], 0.1)
    kept = m.all_observations(0.1)
    assert len(kept) == 1
    assert kept[0].confidence == 0.9
    w0 = m.age_weight(kept[0], timestamp=0.1)
    w1 = m.age_weight(kept[0], timestamp=2.1)
    assert w1 < w0


def test_boundary_map_retains_voxels_instead_of_two_frames():
    m = LocalBoundaryMap(ttl=4.0, voxel_size=0.1, max_points_per_object=160)
    for step in range(100):
        timestamp = 0.05 * step
        batch = [
            BoundaryObservation(
                "obj",
                "a0",
                np.array([0.1 * index, 0.0]),
                np.array([0.0, 1.0]),
                timestamp,
                confidence=0.8,
            )
            for index in range(24)
        ]
        m.update(batch, timestamp)
    assert m.voxel_count("obj") == 24
    assert all(abs(obs.timestamp - 4.95) < 1e-12 for obs in m.all_observations(4.95))


def test_boundary_map_compensates_world_frame_translation_once():
    m = LocalBoundaryMap(ttl=4.0, voxel_size=0.05, motion_match_radius=0.2)
    old = [
        BoundaryObservation("obj", "a0", np.array([0.1 * k, 0.0]), np.array([0.0, 1.0]), 0.0)
        for k in range(5)
    ]
    new = [
        BoundaryObservation("obj", "a0", np.array([0.1 * k + 0.03, 0.0]), np.array([0.0, 1.0]), 0.1)
        for k in range(5)
    ]
    m.update(old, 0.0)
    m.update(new, 0.1)
    xs = sorted(float(obs.point[0]) for obs in m.all_observations(0.1))
    assert min(xs) >= 0.03 - 1e-9


def test_boundary_measure_density_uses_arc_confidence_decay_gap():
    obs = BoundaryObservation(
        "obj",
        "a0",
        np.array([0.0, 0.0]),
        np.array([1.0, 0.0]),
        timestamp=0.0,
        confidence=1.0,
        arc_length=2.0,
        gap_score=1.0,
    )
    field = BoundaryAwareDensity.from_observations(
        [obs],
        cage_offset=0.5,
        sigma=0.2,
        timestamp=0.0,
        decay_lambda=0.0,
        gap_gain=1.0,
    )
    # weight = Δs * c * age * (1 + κ g) = 2 * 1 * 1 * 2 = 4
    assert abs(field.density_points[0].weight - 4.0) < 1e-9
    near = field(np.array([0.5, 0.0]))
    far = field(np.array([2.0, 0.0]))
    assert near > far


def test_local_cvt_samples_strictly_inside_ball():
    cvt = LocalCVT(grid_spacing=0.10, local_radius=1.0)
    agent = AgentState("a0", position=np.array([2.0, 2.0]))
    samples = cvt._sample_local_region(agent.position, domain=(0.0, 8.0, 0.0, 8.0))
    assert len(samples) > 0
    radii = np.linalg.norm(samples - agent.position[None, :], axis=1)
    assert float(np.max(radii)) <= 1.0 + 1e-9


def test_local_cvt_uses_fixed_world_grid_spacing():
    cvt = LocalCVT(grid_spacing=0.10, local_radius=0.55)
    samples = cvt._sample_local_region(np.array([2.03, 2.07]), (0.0, 8.0, 0.0, 8.0))
    xs = np.unique(np.round(samples[:, 0], 10))
    diffs = np.diff(xs)
    assert len(diffs) > 0
    assert np.allclose(diffs, 0.10)


def test_distributed_cbf_ignores_neighbor_velocity():
    cbf = DistributedCBFQP(d_min=0.5, gamma=6.0, max_speed=1.0, use_qp=False)
    p_i = np.array([0.0, 0.0])
    p_j = np.array([0.4, 0.0])  # already close: h < 0
    # Nominal motion further toward neighbor.
    u_nom = np.array([1.0, 0.0])
    u_safe = cbf.filter_velocity(
        p_i,
        u_nom,
        [p_j],
        neighbor_velocities=[np.array([-10.0, 0.0])],  # must be ignored
    )
    # Half-responsibility constraint should push control leftward / reduce approach.
    assert u_safe[0] < u_nom[0]


def test_object_boundary_cbf_blocks_penetration():
    cbf = DistributedCBFQP(d_min=0.2, gamma=6.0, max_speed=1.0, use_qp=False, robot_radius=0.1, alpha_object=5.0)
    position = np.array([0.05, 0.0])  # nearly at boundary, slightly outside if n=[1,0], b=0
    boundary_point = np.array([0.0, 0.0])
    boundary_normal = np.array([1.0, 0.0])
    # Command that would drive into the object (negative normal direction).
    u_nom = np.array([-1.0, 0.0])
    u_safe = cbf.filter_velocity(
        position,
        u_nom,
        neighbor_positions=[],
        boundary_points=[boundary_point],
        boundary_normals=[boundary_normal],
    )
    assert u_safe[0] >= -1e-6


def test_hard_qp_has_box_input_and_no_safety_slack():
    cbf = DistributedCBFQP(d_min=0.4, gamma=6.0, max_speed=0.6, use_qp=True)
    position = np.array([0.0, 0.0])
    neighbors = [np.array([0.5, 0.0]), np.array([0.0, 0.6])]
    velocity = cbf.filter_velocity(position, np.array([1.0, 1.0]), neighbors)
    assert cbf.last_feasible
    assert cbf.last_solver == "qp"
    assert np.all(np.abs(velocity) <= cbf.component_limit + 1e-7)
    for neighbor in neighbors:
        a, b = cbf._robot_constraint(position, neighbor)
        assert float(np.dot(a, velocity)) >= b - 1e-6
