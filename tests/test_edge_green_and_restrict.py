"""Tests for partition restrict bound and edge-green cell integrals."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dbact.apriori_centroid_bound import (
    line_edge_phi_max,
    restrict_partition_mass_bound,
    restrict_pointwise_bound,
)
from dbact.boundary_density import BoundaryAwareDensity, DensityParams
from dbact.edge_green_integral import (
    clip_cell_polygon,
    gaussian_mass_moment_over_polygon,
    inscribed_disk_area_deficit,
    mixture_mass_centroid_over_polygon,
    partition_edge_mass_moment_remainder,
)
from dbact.local_cvt import LocalCVT
from dbact.types import AgentState


def test_restrict_partition_beats_n_fold_disk():
    L, sigma, s, R, N = 7.2, 0.2, 3.0, 0.8, 16
    part = restrict_partition_mass_bound(L, sigma, s)
    point = restrict_pointwise_bound(L, s)
    n_fold = N * point.value * math.pi * R * R
    assert part.value < 0.05
    assert part.value < n_fold / 50.0
    # Wolfram identity
    expected = 2 * math.pi * sigma**2 * math.exp(-0.5 * s * s) * L
    assert abs(part.value - expected) < 1e-12


def test_restrict_delete_implies_xi_tail_on_disk():
    """Mirror of boundary_density.restrict distance relation used in the proof."""
    rng = np.random.default_rng(0)
    R, dc, sigma, s = 0.8, 0.105, 0.2, 3.0
    reach = R + dc + s * sigma
    for _ in range(40):
        p = rng.uniform(1.0, 7.0, size=2)
        b = p + rng.normal(size=2)
        b = p + b / np.linalg.norm(b) * (reach + abs(rng.normal()) + 1e-6)
        nrm = rng.normal(size=2)
        nrm /= np.linalg.norm(nrm)
        xi = b + dc * nrm
        for __ in range(10):
            q = p + rng.uniform(-R, R, size=2)
            if np.linalg.norm(q - p) > R:
                continue
            assert np.linalg.norm(q - xi) >= s * sigma - 1e-9


def test_gaussian_mass_over_disk_polygon_near_plane():
    sigma = 0.2
    xi = np.array([4.0, 4.0])
    poly = clip_cell_polygon(xi, 3.0, np.empty((0, 2)), (0.0, 8.0, 0.0, 8.0), 256)
    mass, _ = gaussian_mass_moment_over_polygon(poly, xi, sigma, panels=64)
    plane = 2 * math.pi * sigma**2
    # Large disk should capture most of the plane mass.
    assert 0.9 * plane < mass < 1.01 * plane


def test_local_cvt_edge_green_runs():
    agents = [
        AgentState(agent_id=0, position=np.array([3.5, 4.0])),
        AgentState(agent_id=1, position=np.array([4.5, 4.0])),
    ]
    dens = BoundaryAwareDensity.from_targets(
        [np.array([4.0, 4.2]), np.array([4.0, 3.8])],
        sigma=0.2,
        weights=[0.2, 0.2],
        base_density=0.001,
    )
    cvt = LocalCVT(
        local_radius=0.8,
        grid_resolution=20,
        integration_method="edge_green",
        edge_n_gon=64,
        edge_panels=16,
    )
    out = cvt.compute(0, agents, [1], dens, (0.0, 8.0, 0.0, 8.0))
    assert out.cell_mass > 0.0
    assert np.linalg.norm(out.centroid - agents[0].position) <= 0.8 + 1e-9


def test_edge_remainder_budget_clears_with_defaults():
    rem = partition_edge_mass_moment_remainder(16, 256, 200, 0.2, 0.8, 48)
    deficit = inscribed_disk_area_deficit(0.8, 256)
    phi_max = line_edge_phi_max(0.001, 0.2, 6)
    dm_geom = 16 * phi_max * deficit
    assert rem["sum_abs_dm"] + dm_geom < 0.05
