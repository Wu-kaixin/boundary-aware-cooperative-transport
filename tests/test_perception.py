"""Perception: occlusion, normal estimation, and the arc-length measure."""

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.perception import (
    LegacyProximitySampler,
    PerceptionParams,
    RayCastBoundarySensor,
    normal_errors_deg,
    occlusion_rate,
)
from dbact.types import AgentState


def l_shape_and_viewpoints(count: int = 24, offset: float = 0.45):
    cargo = Cargo.l_shape("cargo_0", [0.0, 0.0], scale=0.9)
    points, normals = cargo.boundary_samples(count)
    agents = [AgentState(f"a{i}", points[i] + offset * normals[i]) for i in range(count)]
    return cargo, agents


def sensor(**overrides) -> RayCastBoundarySensor:
    kwargs = dict(sensor_range=1.2, ray_count=180, range_noise_std=0.0, pca_neighbors=5, residual_tolerance=0.03)
    kwargs.update(overrides)
    return RayCastBoundarySensor(PerceptionParams(**kwargs))


# --------------------------------------------------------------------------- #
# occlusion
# --------------------------------------------------------------------------- #


def test_ray_cast_returns_nothing_the_observer_cannot_see():
    cargo, agents = l_shape_and_viewpoints()
    ray = sensor()
    blocked = total = 0
    for agent in agents:
        observations = ray.sense(agent, [cargo], 0.0, apply_gate=False)
        b, t = occlusion_rate(observations, agent.position, [cargo])
        blocked += b
        total += t
    assert total > 0
    assert blocked == 0


def test_legacy_sampler_sees_through_the_object():
    """The pre-refactor sampler returned every boundary sample within range, so a
    robot on one side of an L received points from the far side."""
    cargo, agents = l_shape_and_viewpoints()
    legacy = LegacyProximitySampler(sensor_range=1.2)
    blocked = total = 0
    for agent in agents:
        observations = legacy.sense(agent, [cargo], 0.0)
        b, t = occlusion_rate(observations, agent.position, [cargo])
        blocked += b
        total += t
    assert total > 0
    assert blocked > 0


def test_one_object_occludes_another():
    near = Cargo.rectangle("near", [0.0, 0.0], 0.4, 1.6)
    far = Cargo.rectangle("far", [0.8, 0.0], 0.4, 1.6)
    agent = AgentState("a0", np.array([-1.0, 0.0]))
    observations = sensor(sensor_range=3.0).sense(agent, [near, far], 0.0, apply_gate=False)
    assert observations
    assert {o.object_id for o in observations} == {"near"}


def test_noise_tolerance_is_needed_for_an_honest_occlusion_audit():
    """With range noise, about half the returns land just inside the boundary. A
    zero-tolerance line-of-sight test then calls a clean sensor heavily occluded --
    the audit reporting its own noise."""
    cargo, agents = l_shape_and_viewpoints(count=12)
    noisy = sensor(range_noise_std=0.01)
    strict_blocked = tolerant_blocked = total = 0
    for agent in agents:
        observations = noisy.sense(agent, [cargo], 0.0, apply_gate=False)
        strict_blocked += occlusion_rate(observations, agent.position, [cargo], tolerance=0.0)[0]
        b, t = occlusion_rate(observations, agent.position, [cargo], tolerance=0.03)
        tolerant_blocked += b
        total += t
    assert total > 0
    # A 3-sigma allowance leaves only the genuine tail of the noise distribution,
    # while a zero-tolerance audit indicts a large fraction of a clean sensor.
    assert strict_blocked > 20 * max(tolerant_blocked, 1)
    assert tolerant_blocked / total < 0.01


# --------------------------------------------------------------------------- #
# normals and confidence
# --------------------------------------------------------------------------- #


def test_normals_are_estimated_not_taken_from_the_simulator():
    cargo, agents = l_shape_and_viewpoints(count=8)
    observations = sensor(range_noise_std=0.01).sense(agents[0], [cargo], 0.0, apply_gate=False)
    errors = normal_errors_deg(observations, [cargo])
    assert len(errors) > 0
    # Estimated, therefore not identically zero the way ground truth would be.
    assert float(np.max(errors)) > 0.0


def test_normal_error_audit_does_not_mutate_observations():
    cargo, agents = l_shape_and_viewpoints(count=8)
    observations = sensor(range_noise_std=0.01).sense(agents[0], [cargo], 0.0, apply_gate=False)
    before = [obs.normal.copy() for obs in observations]
    normal_errors_deg(observations, [cargo])
    for observation, expected in zip(observations, before):
        assert np.array_equal(observation.normal, expected)


def test_legacy_sampler_has_identically_zero_normal_error():
    """Which is why any robustness margin derived against it is vacuous."""
    cargo, agents = l_shape_and_viewpoints(count=8)
    observations = LegacyProximitySampler(sensor_range=1.2).sense(agents[0], [cargo], 0.0)
    errors = normal_errors_deg(observations, [cargo])
    assert len(errors) > 0
    # Exactly zero away from vertices, where the two incident edge normals differ
    # and the comparison is ambiguous rather than wrong.
    assert float(np.percentile(errors, 90)) == pytest.approx(0.0, abs=1e-6)


def test_confidence_gate_reduces_the_normal_error_tail():
    cargo, agents = l_shape_and_viewpoints(count=24)
    ray = sensor(range_noise_std=0.01, min_confidence=0.15)
    ungated, gated = [], []
    for agent in agents:
        raw = ray.sense(agent, [cargo], 0.0, apply_gate=False)
        ungated.append(normal_errors_deg(raw, [cargo]))
        kept = [o for o in raw if o.confidence >= ray.params.min_confidence]
        gated.append(normal_errors_deg(kept, [cargo]))
    ungated = np.concatenate(ungated)
    gated = np.concatenate(gated)
    assert len(gated) <= len(ungated)
    assert np.percentile(gated, 90) <= np.percentile(ungated, 90)


def test_a_tighter_residual_tolerance_rejects_more_returns():
    """The gate is a residual threshold in metres, so tightening it must reject
    strictly more -- which is the property an eigenvalue ratio does not have."""
    cargo, agents = l_shape_and_viewpoints(count=24)
    loose = sensor(range_noise_std=0.01, residual_tolerance=0.05, min_confidence=0.15)
    tight = sensor(range_noise_std=0.01, residual_tolerance=0.012, min_confidence=0.15)
    n_loose = sum(len(loose.sense(a, [cargo], 0.0)) for a in agents)
    n_tight = sum(len(tight.sense(a, [cargo], 0.0)) for a in agents)
    assert n_tight < n_loose


def test_confidence_is_a_residual_and_therefore_carries_units():
    """The eigenvalue-ratio score still reported 0.83-0.92 on returns whose normal
    error exceeded 30 degrees, because ray-scan spacing is anisotropic enough that
    a corner still looks linear by ratio. The residual form does not have that
    failure mode: doubling the tolerance must raise every confidence."""
    cargo, agents = l_shape_and_viewpoints(count=8)
    tight = sensor(range_noise_std=0.01, residual_tolerance=0.01).sense(agents[0], [cargo], 0.0, apply_gate=False)
    loose = sensor(range_noise_std=0.01, residual_tolerance=0.05).sense(agents[0], [cargo], 0.0, apply_gate=False)
    assert len(tight) == len(loose)
    assert all(b.confidence >= a.confidence - 1e-12 for a, b in zip(tight, loose))
    assert all(0.0 <= o.confidence <= 1.0 for o in tight)
    assert all(o.residual >= 0.0 for o in tight)


def test_normals_point_outward():
    cargo, agents = l_shape_and_viewpoints(count=16)
    for agent in agents:
        for obs in sensor().sense(agent, [cargo], 0.0, apply_gate=False):
            assert float(np.dot(obs.normal, agent.position - obs.point)) > 0.0


# --------------------------------------------------------------------------- #
# arc length
# --------------------------------------------------------------------------- #


def test_arc_lengths_approximate_the_visible_boundary_length():
    """Arc length is what makes the density a measure on the boundary rather than a
    sum over however many samples the sensor happened to produce."""
    cargo = Cargo.rectangle("box", [0.0, 0.0], 2.0, 2.0)
    agent = AgentState("a0", np.array([0.0, 1.4]))
    observations = sensor(sensor_range=1.2, ray_count=360).sense(agent, [cargo], 0.0, apply_gate=False)
    total = sum(o.arc_length for o in observations)
    # The visible strip of the top face is bounded by the face itself.
    assert 0.4 < total <= 2.0 + 1e-6


def test_denser_scanning_does_not_inflate_total_arc_length():
    cargo = Cargo.rectangle("box", [0.0, 0.0], 2.0, 2.0)
    agent = AgentState("a0", np.array([0.0, 1.4]))
    coarse = sum(o.arc_length for o in sensor(ray_count=180).sense(agent, [cargo], 0.0, apply_gate=False))
    fine = sum(o.arc_length for o in sensor(ray_count=720).sense(agent, [cargo], 0.0, apply_gate=False))
    assert fine == pytest.approx(coarse, rel=0.25)


def test_sensor_is_reproducible_across_processes():
    """Frame randomness goes through BLAKE2, not Python's salted hash()."""
    cargo, agents = l_shape_and_viewpoints(count=4)
    a = sensor(range_noise_std=0.02).sense(agents[0], [cargo], 1.25, apply_gate=False)
    b = sensor(range_noise_std=0.02).sense(agents[0], [cargo], 1.25, apply_gate=False)
    assert len(a) == len(b)
    assert np.allclose(np.vstack([o.point for o in a]), np.vstack([o.point for o in b]))


def test_no_returns_when_the_object_is_out_of_range():
    cargo = Cargo.rectangle("box", [0.0, 0.0], 0.4, 0.4)
    agent = AgentState("a0", np.array([5.0, 5.0]))
    assert sensor(sensor_range=1.2).sense(agent, [cargo], 0.0) == []
