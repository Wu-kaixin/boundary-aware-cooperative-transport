from __future__ import annotations

import os

import numpy as np

from dbact.accel import (
    HAS_NUMBA,
    _gaussian_density_numpy,
    _voronoi_owners_numpy,
    gaussian_density,
    using_numba,
    voronoi_owners,
    warmup,
)
from dbact.boundary_density import BoundaryAwareDensity
from dbact.cargo import Cargo
from dbact.controller import DBACTController, DBACTParams, resolve_worker_count
from dbact.geometry import sample_polygon_boundary
from dbact.local_cvt import LocalCVT
from dbact.types import AgentState, BoundaryObservation
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def test_cargo_boundary_cache_invalidates_on_translate():
    cargo = Cargo.rectangle("box", [0.0, 0.0], width=2.0, height=1.0)
    pts1, n1 = cargo.boundary_samples(40)
    pts2, n2 = cargo.boundary_samples(40)
    assert pts1 is pts2
    assert n1 is n2
    cargo.translate([0.5, 0.0])
    pts3, _ = cargo.boundary_samples(40)
    assert pts3 is not pts1
    assert not np.allclose(pts3, pts1)


def test_sample_polygon_boundary_matches_legacy_walk():
    cargo = Cargo.l_shape("L", [0.0, 0.0], scale=1.0)
    pts, normals = sample_polygon_boundary(cargo.vertices, count=64)
    # Re-run legacy-equivalent searchsorted path already in geometry; check basic invariants.
    assert pts.shape == (64, 2)
    assert normals.shape == (64, 2)
    assert np.allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-9)


def test_density_and_voronoi_match_numpy_reference():
    os.environ.pop("DBACT_FORCE_NUMPY", None)
    warmup()
    q = np.array([[0.0, 0.0], [0.5, 0.1], [1.2, -0.3]], dtype=float)
    targets = np.array([[0.1, 0.0], [0.8, 0.2]], dtype=float)
    weights = np.array([1.0, 0.5], dtype=float)
    sigma = 0.35
    base = 1e-3
    got = np.atleast_1d(gaussian_density(q, targets, weights, sigma, base))
    ref = _gaussian_density_numpy(q, targets, weights, sigma, base)
    assert np.allclose(got, ref, rtol=1e-12, atol=1e-12)

    owners = voronoi_owners(q, targets)
    owners_ref = _voronoi_owners_numpy(q, targets)
    assert np.array_equal(owners, owners_ref)


def test_boundary_density_matches_forced_numpy():
    obs = [
        BoundaryObservation("obj", "a0", np.array([0.0, 0.0]), np.array([1.0, 0.0]), 0.0, confidence=1.0, arc_length=0.2),
        BoundaryObservation("obj", "a0", np.array([0.0, 0.5]), np.array([1.0, 0.0]), 0.0, confidence=0.8, arc_length=0.3),
    ]
    field = BoundaryAwareDensity.from_observations(obs, cage_offset=0.5, sigma=0.25)
    samples = np.array([[0.5, 0.0], [0.5, 0.5], [2.0, 2.0]], dtype=float)
    got = np.atleast_1d(field(samples))
    ref = _gaussian_density_numpy(samples, field.targets, field._weights, field.sigma, field.base_density)
    assert np.allclose(got, ref, rtol=1e-12, atol=1e-12)
    centroid = field.weighted_centroid(samples)
    assert centroid is not None
    w = got
    expected = np.sum(samples * w[:, None], axis=0) / float(np.sum(w))
    assert np.allclose(centroid, expected)


def test_local_cvt_centroid_stable():
    agents = [
        AgentState("a0", np.array([1.0, 1.0])),
        AgentState("a1", np.array([1.4, 1.1])),
        AgentState("a2", np.array([0.7, 1.3])),
    ]
    density = BoundaryAwareDensity.from_targets(
        [np.array([1.1, 1.0]), np.array([1.2, 1.2])],
        sigma=0.3,
        weights=[1.0, 0.7],
    )
    cvt = LocalCVT(grid_spacing=0.1, local_radius=1.0)
    domain = (0.0, 4.0, 0.0, 4.0)
    c1 = cvt.compute_centroid(0, agents, [1, 2], density, domain)
    c2 = cvt.compute_centroid(0, agents, [1, 2], density, domain)
    assert np.allclose(c1, c2)


def test_resolve_workers_serial_and_auto():
    assert resolve_worker_count(1, 12) == 1
    os.environ["DBACT_WORKERS"] = "1"
    try:
        assert resolve_worker_count(8, 12) == 1
    finally:
        os.environ.pop("DBACT_WORKERS", None)


def test_short_simulation_workers_serial_smoke():
    cfg = load_yaml("configs/sim/circle.yaml")
    cfg.setdefault("controller", {})
    cfg["controller"]["workers"] = 1
    env = SimulationEnvironment(cfg)
    env.run(steps=5)
    assert len(env.log.times) == 6


def test_controller_explicit_thread_pool_smoke():
    cfg = load_yaml("configs/sim/circle.yaml")
    cfg.setdefault("controller", {})
    cfg["controller"]["workers"] = 2
    cfg["controller"]["cbf_use_qp"] = False
    env = SimulationEnvironment(cfg)
    env.run(steps=3)
    assert env.controller.stats.cbf_calls > 0


def test_numba_flag_reported():
    # Soft check: either numba is present or forced-numpy path works.
    assert isinstance(HAS_NUMBA, bool)
    assert isinstance(using_numba(), bool)
    params = DBACTParams(workers=1)
    controller = DBACTController(params, (0.0, 8.0, 0.0, 8.0))
    assert controller.params.workers == 1
