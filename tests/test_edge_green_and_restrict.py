"""Tests for partition restrict bound and edge-green cell integrals."""

from __future__ import annotations

import math
from pathlib import Path

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
    mass, _ = gaussian_mass_moment_over_polygon(poly, xi, sigma, h_max=0.02)
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
        edge_h_max=0.05,
    )
    out = cvt.compute(0, agents, [1], dens, (0.0, 8.0, 0.0, 8.0))
    assert out.cell_mass > 0.0
    assert np.linalg.norm(out.centroid - agents[0].position) <= 0.8 + 1e-9


def test_clipped_voronoi_edges_exceed_unclipped_chord():
    """Regression: wall/Voronoi clips produce edges longer than 2 R sin(π/n)."""
    from dbact.edge_green_integral import (
        a_priori_max_edge_length,
        polygon_edge_lengths,
        unclipped_regular_chord,
    )

    r, n_gon = 0.8, 256
    chord = unclipped_regular_chord(r, n_gon)
    p0 = np.array([4.0, 4.0])
    p1 = np.array([4.05, 4.0])  # Voronoi bisector cuts a long chord through the disk
    poly = clip_cell_polygon(p0, r, p1.reshape(1, 2), (0.0, 8.0, 0.0, 8.0), n_gon)
    lengths = polygon_edge_lengths(poly)
    assert lengths.max() > 10.0 * chord
    assert lengths.max() <= a_priori_max_edge_length(r) + 1e-12
    assert abs(chord - 2.0 * r * math.sin(math.pi / n_gon)) < 1e-12


def test_lshape_seed2_all_cells_have_long_clipped_edges():
    """Reproduce the audit: every L-shape seed-2 cell has an edge above the n-gon chord."""
    from dbact.edge_green_integral import (
        a_priori_max_edge_length,
        polygon_edge_lengths,
        unclipped_regular_chord,
    )
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import load_yaml

    cfg_path = Path("configs/sim/theorem/static_l_shape_n16_oracle.yaml")
    if not cfg_path.exists():
        pytest.skip("L-shape theorem config missing")
    cfg = load_yaml(cfg_path)
    env = SimulationEnvironment(cfg, seed=2)
    positions = np.vstack([ag.position for ag in env.agents])
    domain = (
        float(cfg["domain"]["xmin"]),
        float(cfg["domain"]["xmax"]),
        float(cfg["domain"]["ymin"]),
        float(cfg["domain"]["ymax"]),
    )
    r, n_gon = 0.8, 256
    chord = unclipped_regular_chord(r, n_gon)
    n_long = 0
    max_edge = 0.0
    for i, p in enumerate(positions):
        nbrs = np.delete(positions, i, axis=0)
        poly = clip_cell_polygon(p, r, nbrs, domain, n_gon)
        if len(poly) < 3:
            continue
        m = float(polygon_edge_lengths(poly).max())
        max_edge = max(max_edge, m)
        if m > chord + 1e-12:
            n_long += 1
    assert n_long == 16
    assert max_edge > 1.0
    assert max_edge <= a_priori_max_edge_length(r) + 1e-9
    # Honest remainder must declare 2R, not the 0.0196 m chord.
    rem = partition_edge_mass_moment_remainder(
        16, 256, 200, 0.2, 0.8, 48, weight_sum=7.2, h_max=0.004
    )
    assert rem["max_edge_length"] == pytest.approx(1.6)
    assert rem["legacy_unclipped_chord"] == pytest.approx(chord)
    assert rem["max_edge_length"] >= max_edge - 1e-9
    assert rem["m2_status"] == "analytic"


def test_analytic_m2_not_nmaximize_scaling():
    from dbact.edge_green_integral import analytic_m2_f1, analytic_m2_sigma2_kernel

    m2_f1 = analytic_m2_f1(0.2)
    m2_s2 = analytic_m2_sigma2_kernel(0.2)
    assert m2_s2 == pytest.approx(1.0)
    # F1 scales as 1/sigma, not (0.2/sigma)^4.
    assert analytic_m2_f1(0.1) == pytest.approx(2.0 * m2_f1)
    assert analytic_m2_f1(0.4) == pytest.approx(0.5 * m2_f1)
    # Old code used 15*(0.2/sigma)^4 = 15 at sigma=0.2; the analytic F1 bound is C/sigma.
    assert m2_f1 > 10.0
    assert m2_f1 < 25.0


def test_robot_centered_moment_includes_xi_shift():
    rem = partition_edge_mass_moment_remainder(
        16, 256, 200, 0.2, 0.8, 48, weight_sum=7.2, h_max=0.004, cull_sigmas=6.0
    )
    assert rem["sum_abs_dmu_robot"] > rem["sum_abs_dmu_xi"]
    assert rem["xi_to_site_reach"] == pytest.approx(0.8 + 6.0 * 0.2)
    assert rem["weight_sum_W"] == pytest.approx(7.2)
    # Using W=7.2 is much tighter than 200 unit-weight sources.
    rem_unit = partition_edge_mass_moment_remainder(16, 256, 200, 0.2, 0.8, 48, h_max=0.004)
    assert rem["sum_abs_dm"] < rem_unit["sum_abs_dm"] / 10.0


def test_projected_centroid_stays_in_disk():
    from dbact.edge_green_integral import project_to_disk

    c = np.array([4.0, 4.0])
    q = np.array([6.0, 4.0])
    p, hit = project_to_disk(q, c, 0.8)
    assert hit
    assert np.linalg.norm(p - c) == pytest.approx(0.8)
    inside, hit2 = project_to_disk(c + np.array([0.1, 0.0]), c, 0.8)
    assert not hit2
    assert np.linalg.norm(inside - c) == pytest.approx(0.1)


def test_small_mass_fallback_bound_covers_site_centroid():
    from dbact.apriori_centroid_bound import moment_form_E_bound_with_mass_fallback

    base_plus = moment_form_E_bound_with_mass_fallback(0.8, 0.01, 0.008, n_agents=16)
    from dbact.apriori_centroid_bound import moment_form_E_bound

    assert base_plus > moment_form_E_bound(0.8, 0.01, 0.008)
    assert base_plus < 0.1


def test_edge_remainder_budget_clears_with_hmax():
    rem = partition_edge_mass_moment_remainder(
        16, 256, 200, 0.2, 0.8, 48, weight_sum=7.2, h_max=0.004
    )
    deficit = inscribed_disk_area_deficit(0.8, 256)
    from dbact.edge_green_integral import discrete_mixture_phi_max

    phi = discrete_mixture_phi_max(0.001, 7.2, 6, 0.04, 0.2)["phi_max_used"]
    dm_geom = 16 * phi * deficit
    assert rem["sum_abs_dm"] + dm_geom < 0.15
    assert rem["legacy_unclipped_chord"] < 0.03
    assert rem["max_edge_length"] == pytest.approx(1.6)
