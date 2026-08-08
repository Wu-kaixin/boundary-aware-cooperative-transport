"""The contact model: the only channel through which the cargo can move.

These are falsification tests. Each one is a way the pre-refactor dynamics could
have produced "transport" without any transport happening.
"""

import math

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.contact_dynamics import ContactParams, PenaltyContactModel
from dbact.transport_dynamics import ScriptedParams, ScriptedTransportEngine, build_engine
from dbact.types import AgentState

DT = 0.05


def model(**overrides) -> PenaltyContactModel:
    kwargs = dict(
        robot_radius=0.16,
        stiffness=500.0,
        damping=12.0,
        friction=0.6,
        ground_friction=0.45,
        gravity=9.81,
        substeps=4,
    )
    kwargs.update(overrides)
    return PenaltyContactModel(ContactParams(**kwargs))


def drive(m: PenaltyContactModel, cargo: Cargo, agents: list[AgentState], steps: int) -> None:
    for _ in range(steps):
        for a in agents:
            a.position = a.position + a.velocity * DT
        m.step(cargo, agents, DT)


# --------------------------------------------------------------------------- #
# the three falsification tests
# --------------------------------------------------------------------------- #


def test_no_contact_means_exactly_zero_displacement():
    """Not "small", zero. The legacy model moved the cargo whenever a coverage
    threshold was met, whether or not anything touched it."""
    m = model()
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6, surface_density=2.0)
    far = [AgentState("a0", np.array([4.0, 4.0]), velocity=np.array([0.3, 0.0]))]
    for _ in range(200):
        m.step(cargo, far, DT)
    assert float(np.linalg.norm(cargo.displacement)) == 0.0
    assert cargo.angle == 0.0


def test_motion_direction_comes_from_contact_geometry_not_configuration():
    """Push from the left; the cargo goes right. No field of the configuration
    reaches this module, so there is nothing for a configured direction to do."""
    m = model()
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6, surface_density=1.0)
    r = m.params.robot_radius
    pushers = [
        AgentState("a0", np.array([-0.5 - r + 0.02, -0.15]), velocity=np.array([0.25, 0.0])),
        AgentState("a1", np.array([-0.5 - r + 0.02, 0.15]), velocity=np.array([0.25, 0.0])),
    ]
    drive(m, cargo, pushers, 200)
    displacement = cargo.displacement
    assert np.linalg.norm(displacement) > 0.05
    direction = displacement / np.linalg.norm(displacement)
    assert direction[0] > 0.99
    # ~90 degrees away from a configured direction of (0, -1).
    angle = math.degrees(math.acos(float(np.clip(np.dot(direction, [0.0, -1.0]), -1.0, 1.0))))
    assert angle > 80.0


def test_single_offcentre_push_rotates_the_body():
    """A cargo that can only translate hides the failure mode where the team
    applies a net torque it cannot resist."""
    m = model()
    cargo = Cargo.rectangle("spin", [0.0, 0.0], 1.0, 0.6, surface_density=1.0)
    r = m.params.robot_radius
    spinner = [AgentState("a0", np.array([-0.5 - r + 0.02, 0.25]), velocity=np.array([0.25, 0.0]))]
    drive(m, cargo, spinner, 200)
    assert abs(math.degrees(cargo.angle)) > 1.0


def test_scripted_engine_reproduces_the_configured_direction_exactly():
    """This is the pre-refactor baseline and the cleanest evidence that its
    "transport" result was a restatement of its configuration: the displacement
    direction agrees with the configured direction to 0.000000 degrees."""
    configured = np.array([0.0, -1.0])
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6)
    engine = ScriptedTransportEngine(
        ScriptedParams(
            contact_radius=0.5,
            coverage_threshold=0.0,
            min_contact_agents=1,
            speed=0.16,
            goal_directions={"box": configured},
        )
    )
    agents = [AgentState("a0", np.array([-0.7, 0.0]), velocity=np.array([0.25, 0.0]))]
    for _ in range(200):
        engine.step([cargo], agents, DT)
    displacement = cargo.displacement
    assert np.linalg.norm(displacement) > 0.0
    direction = displacement / np.linalg.norm(displacement)
    angle = math.degrees(math.acos(float(np.clip(np.dot(direction, configured), -1.0, 1.0))))
    assert angle == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------- #
# Coulomb ground friction
# --------------------------------------------------------------------------- #


def test_force_below_breakaway_does_not_move_the_object():
    """Stiction is what makes the task cooperative. Under viscous drag there is no
    threshold at all, and the object creeps for any force."""
    m = model(ground_friction=2.0)
    cargo = Cargo.rectangle("heavy", [0.0, 0.0], 1.0, 0.6, surface_density=2.0)
    r = m.params.robot_radius
    weak = [AgentState("a0", np.array([-0.5 - r + 0.004, 0.0]), velocity=np.zeros(2))]
    for _ in range(200):
        m.step(cargo, weak, DT)
    report = m.compute_contacts(cargo, weak)
    assert report.contact_count == 1
    assert np.linalg.norm(report.net_force) < m.params.breakaway_force(cargo.mass)
    assert float(np.linalg.norm(cargo.displacement)) == 0.0


def test_enough_robots_break_the_object_loose():
    m = model(ground_friction=0.45)
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6, surface_density=2.0)
    r = m.params.robot_radius
    team = [
        AgentState(f"a{i}", np.array([-0.5 - r + 0.02, y]), velocity=np.array([0.25, 0.0]))
        for i, y in enumerate((-0.2, 0.0, 0.2))
    ]
    drive(m, cargo, team, 200)
    assert cargo.displacement[0] > 0.05


def test_breakaway_force_and_cooperation_count_are_reported():
    params = ContactParams(robot_radius=0.16, stiffness=500.0, ground_friction=0.45, gravity=9.81)
    mass = 4.12
    assert params.breakaway_force(mass) == pytest.approx(0.45 * mass * 9.81)
    # At the cage ring one robot supplies k_p * (r_robot - d_c).
    assert params.min_cooperating_robots(mass, 0.135) == pytest.approx(
        params.breakaway_force(mass) / (500.0 * 0.025)
    )


def test_penetration_produces_force_proportional_to_overlap():
    m = model()
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6, surface_density=2.0)
    r = m.params.robot_radius
    shallow = [AgentState("a0", np.array([-0.5 - r + 0.01, 0.0]))]
    deep = [AgentState("a0", np.array([-0.5 - r + 0.04, 0.0]))]
    f_shallow = np.linalg.norm(m.compute_contacts(cargo, shallow).net_force)
    f_deep = np.linalg.norm(m.compute_contacts(cargo, deep).net_force)
    assert f_deep > f_shallow
    assert f_deep / f_shallow == pytest.approx(4.0, rel=0.1)


def test_immovable_cargo_stays_put_but_still_reports_contacts():
    m = model()
    cargo = Cargo.rectangle("wall", [0.0, 0.0], 1.0, 0.6, movable=False)
    r = m.params.robot_radius
    pusher = [AgentState("a0", np.array([-0.5 - r + 0.03, 0.0]), velocity=np.array([0.25, 0.0]))]
    report = m.step(cargo, pusher, DT)
    assert report.contact_count == 1
    assert float(np.linalg.norm(cargo.displacement)) == 0.0


# --------------------------------------------------------------------------- #
# engine registry
# --------------------------------------------------------------------------- #


def test_unknown_engine_is_rejected():
    with pytest.raises(ValueError, match="unknown transport engine"):
        build_engine("magic", ContactParams())


def test_pymunk_engine_agrees_on_the_direction_of_motion():
    """An independent rigid-body solver, so the transport result is not an
    artefact of the penalty contact model."""
    pytest.importorskip("pymunk")
    params = ContactParams(robot_radius=0.16, stiffness=500.0, ground_friction=0.45, gravity=9.81, substeps=4)
    engine = build_engine("pymunk", params)
    cargo = Cargo.l_shape("cargo_0", [0.0, 0.0], scale=1.5, surface_density=2.0)
    points, normals = cargo.boundary_samples(360)
    direction = np.array([1.0, 0.0])
    trailing = np.where(normals @ direction < -0.8)[0]
    picks = trailing[np.linspace(0, len(trailing) - 1, 4).astype(int)]
    agents = [
        AgentState(f"a{i}", points[k] + (params.robot_radius + 0.01) * normals[k], velocity=direction * 0.25)
        for i, k in enumerate(picks)
    ]
    for _ in range(200):
        for a in agents:
            a.position = a.position + a.velocity * DT
        engine.step([cargo], agents, DT)
    assert cargo.displacement[0] > 0.02
