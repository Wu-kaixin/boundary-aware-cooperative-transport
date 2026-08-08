"""Phase 2/3 tests: physics transport, baselines, paper metrics."""

from __future__ import annotations

import numpy as np
import pytest

from dbact.controller import DBACTController, DBACTParams
from dbact.metrics import enclosure_time, summarize_seeds, success_flag
from dbact.transport_dynamics import TransportParams, build_transport
from dbact.types import AgentState
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def test_scripted_transport_backend_factory():
    params = TransportParams(backend="scripted")
    transport = build_transport(params)
    assert transport.__class__.__name__ == "SimpleCagingTransportDynamics"


def test_pymunk_transport_moves_on_contact():
    pymunk = pytest.importorskip("pymunk")
    del pymunk
    cfg = load_yaml("configs/sim/paper/pymunk_push.yaml")
    cfg["controller"]["cbf_use_qp"] = False
    env = SimulationEnvironment(cfg)
    assert env.transport.__class__.__name__ == "PymunkTransportDynamics"
    # Place agents around the rectangle so contact can occur quickly.
    center = env.cargoes[0].center.copy()
    offsets = [
        np.array([0.55, 0.0]),
        np.array([-0.55, 0.0]),
        np.array([0.0, 0.40]),
        np.array([0.0, -0.40]),
    ]
    for i, offset in enumerate(offsets):
        if i < len(env.agents):
            env.agents[i].position = center + offset
            env.agents[i].velocity = np.array([0.25, 0.0])
    before = env.cargoes[0].center.copy()
    for _ in range(40):
        # Bypass full controller for a pure contact push smoke test.
        for agent in env.agents[:4]:
            agent.velocity = np.array([0.35, 0.0])
            agent.position = agent.position + agent.velocity * env.dt
        env.transport.step(env.cargoes, env.agents, env.dt)
    after = env.cargoes[0].center.copy()
    assert float(np.linalg.norm(after - before)) > 1e-3


def test_pymunk_transport_decomposes_and_pushes_l_shape():
    pytest.importorskip("pymunk")
    from dbact.cargo import Cargo

    cargo = Cargo.l_shape("L", center=[0.0, 0.0], scale=1.0)
    start = cargo.center.copy()
    ys = np.linspace(start[1] - 0.3, start[1] + 0.3, 8)
    agents = [
        AgentState(f"a{i}", np.array([-0.85, y]), velocity=np.array([0.35, 0.0]))
        for i, y in enumerate(ys)
    ]
    transport = build_transport(
        TransportParams(
            backend="pymunk",
            robot_radius=0.06,
            cargo_mass=5.0,
            linear_damping=0.25,
            substeps=4,
        ),
        [cargo],
        agents,
    )
    assert len(transport.world._cargo_shapes["L"]) > 1
    for _ in range(120):
        for agent in agents:
            agent.velocity = np.array([0.35, 0.0])
        transport.step([cargo], agents, 0.02)
    assert float(cargo.center[0] - start[0]) > 0.05


def test_baseline_methods_smoke():
    for method in ("arm", "oracle", "no_cbf", "dbact"):
        params = DBACTParams(method=method, cbf_use_qp=False, sensor_range=2.0, enable_transport_bias=False)
        controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0))
        agents = [
            AgentState("a0", np.array([2.0, 1.2])),
            AgentState("a1", np.array([2.5, 1.2])),
            AgentState("a2", np.array([1.5, 1.2])),
        ]
        from dbact.cargo import Cargo

        cargoes = [Cargo.rectangle("c0", center=[2.0, 2.0], width=0.8, height=0.5)]
        commands = controller.step(agents, cargoes, timestamp=0.0, dt=0.05)
        assert len(commands) == 3
        assert all(np.linalg.norm(cmd.velocity) <= params.max_speed + 1e-9 for cmd in commands)


def test_paper_metrics_helpers():
    assert enclosure_time([0.1, 0.2, 0.6], [0.0, 0.1, 0.2], threshold=0.5) == 0.2
    assert enclosure_time([0.1, 0.2], [0.0, 0.1], threshold=0.5) is None
    assert success_flag(0.7, 0.0, require_transport=False)
    assert not success_flag(0.7, 0.0, require_transport=True, min_displacement=0.2)
    summary = summarize_seeds([{"final_coverage": 0.5, "success": 1.0}, {"final_coverage": 0.7, "success": 0.0}])
    assert summary["n_seeds"] == 2
    assert abs(summary["final_coverage"]["mean"] - 0.6) < 1e-9


def test_b3_config_emits_extended_metrics():
    cfg = load_yaml("configs/sim/paper/b3_dbact.yaml")
    cfg["controller"]["cbf_use_qp"] = False
    env = SimulationEnvironment(cfg)
    env.run(steps=30)
    metrics = env.compute_metrics()
    assert "T_enclosure" in metrics
    assert "R_CBF" in metrics
    assert "T_solve" in metrics
    assert "P_success" in metrics
    assert metrics["method"] == "dbact"
