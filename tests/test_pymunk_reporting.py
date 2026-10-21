import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.contact_dynamics import ContactParams
from dbact.transport_dynamics import build_engine
from dbact.types import AgentState
from dbact_sim.experiments.deployment import config_for, transport_arm

pytest.importorskip("pymunk")


def test_kinematic_robot_finishes_at_the_commanded_endpoint():
    engine = build_engine("pymunk", ContactParams(ground_friction=0.0))
    agent = AgentState("a", np.array([-1.0, 0.0]), velocity=np.array([0.2, 0.0]))
    engine.step([], [agent], 0.05)
    body = engine._agent_bodies["a"]
    np.testing.assert_allclose([body.position.x, body.position.y], agent.position, atol=1e-12)


def test_reported_contact_impulse_and_torque_match_momentum_change():
    engine = build_engine("pymunk", ContactParams(robot_radius=0.16, ground_friction=0.0, substeps=4))
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6, surface_density=2.0)
    agent = AgentState("a", np.array([-0.64, 0.15]), velocity=np.array([0.3, 0.0]))
    before = cargo.linear_velocity.copy()
    omega = cargo.angular_velocity
    status = engine.step([cargo], [agent], 0.05)[0]
    report = engine.last_reports["box"]
    assert report.contact_count == 1
    assert report.contacts[0].agent_id == "a"
    assert report.net_force[0] > 0
    assert report.contacts[0].force[0] > 0
    np.testing.assert_allclose(report.net_force, report.contacts[0].force, atol=1e-8)
    np.testing.assert_allclose(report.net_force * 0.05, cargo.mass * (cargo.linear_velocity - before), atol=1e-8)
    assert report.net_torque * 0.05 == pytest.approx(cargo.inertia * (cargo.angular_velocity - omega), abs=1e-8)
    np.testing.assert_allclose(status.net_force, report.net_force)
    assert status.net_torque == report.net_torque
    assert report.contacts[0].torque == pytest.approx(report.net_torque, abs=1e-8)


def test_pymunk_can_use_the_complete_paper_episode_recorder(tmp_path):
    cfg = config_for(6, "l_shape")
    cfg["transport"]["engine"] = "pymunk"
    result = transport_arm(cfg, 2, "boundary_cvt", 3, tmp_path)
    assert result["termination"]["frames_run"] == 3
    assert result["contact_reporting"] == "control_interval_impulse_average"
    assert result["peak_normal_only_forward_capacity"] is None
