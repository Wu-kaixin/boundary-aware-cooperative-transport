"""The limited-range CVT: strict locality and the truncated descent property."""

import warnings

import numpy as np
import pytest

from dbact.boundary_density import BoundaryAwareDensity, DensityParams
from dbact.local_cvt import LocalCVT, coverage_cost, empty_cell_threshold
from dbact.types import AgentState, BoundaryObservation

DOMAIN = (0.0, 8.0, 0.0, 8.0)


def wall_density(sigma: float = 0.12, cage_offset: float = 0.135) -> BoundaryAwareDensity:
    xs = np.arange(1.0, 7.0001, 0.06)
    observations = [
        BoundaryObservation("obj", "m", np.array([x, 4.0]), np.array([0.0, 1.0]), 0.0, 1.0, arc_length=0.06)
        for x in xs
    ]
    params = DensityParams(mode="offset", cage_offset=cage_offset, sigma=sigma, base_density=1e-3)
    return BoundaryAwareDensity.from_observations(observations, params)


# --------------------------------------------------------------------------- #
# locality
# --------------------------------------------------------------------------- #


def test_integration_domain_is_the_disk_not_a_bounding_box():
    """The previous version integrated over the local box unioned with the bounding
    box of every density target, so local_radius barely bound anything: a robot at
    (6, 6) in an 8x8 domain integrated over a 7.06 x 7.06 box and sampled points
    7.16 m away."""
    cvt = LocalCVT(local_radius=0.8, grid_resolution=31, comm_range=1.6)
    agents = [AgentState("a0", np.array([6.0, 6.0]))]
    samples, _ = cvt.cell_samples(0, agents, [], DOMAIN)
    assert len(samples) > 0
    reach = np.linalg.norm(samples - agents[0].position[None, :], axis=1)
    assert float(np.max(reach)) <= 0.8 + 1e-9


def test_cell_is_cut_by_neighbours():
    cvt = LocalCVT(local_radius=0.8, grid_resolution=31, comm_range=1.6)
    agents = [AgentState("a0", np.array([4.0, 4.0])), AgentState("a1", np.array([4.5, 4.0]))]
    alone, _ = cvt.cell_samples(0, agents, [], DOMAIN)
    shared, _ = cvt.cell_samples(0, agents, [1], DOMAIN)
    assert len(shared) < len(alone)
    # Every owned sample is at least as close to a0 as to a1.
    assert np.all(
        np.linalg.norm(shared - agents[0].position, axis=1) <= np.linalg.norm(shared - agents[1].position, axis=1) + 1e-9
    )


def test_neighbour_completeness_violation_warns():
    """R_l <= R_comm/2 makes the local cell exact rather than approximate, so
    exceeding it must be visible."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        LocalCVT(local_radius=1.2, comm_range=1.6)
    assert any(issubclass(w.category, RuntimeWarning) for w in caught)


def test_neighbour_completeness_at_the_bound_does_not_warn():
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        LocalCVT(local_radius=0.8, comm_range=1.6)
    assert not any(issubclass(w.category, RuntimeWarning) for w in caught)


# --------------------------------------------------------------------------- #
# equilibrium
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("sigma", [0.06, 0.12, 0.20])
def test_single_robot_converges_to_the_cage_offset(sigma):
    """The coverage law must place the robot exactly on the cage ring; if it did
    not, C1 would be asserted about a distance the robot never reaches."""
    density = wall_density(sigma=sigma, cage_offset=0.135)
    cvt = LocalCVT(local_radius=0.5, grid_resolution=41, comm_range=4.0)
    position = np.array([4.0, 4.6])
    for _ in range(300):
        agents = [AgentState("a0", position)]
        centroid = cvt.compute(0, agents, [], density, DOMAIN).centroid
        position = position + 0.25 * (centroid - position)
    assert position[1] - 4.0 == pytest.approx(0.135, abs=0.005)


# --------------------------------------------------------------------------- #
# truncated cost
# --------------------------------------------------------------------------- #


def test_truncated_cost_descends_under_a_small_step():
    density = wall_density()
    # The descent statement is about the exact functional, so the quadrature has to
    # be fine enough that its own error is below the step-to-step change in H.
    cvt = LocalCVT(local_radius=0.8, grid_resolution=31, comm_range=1.6)
    rng = np.random.default_rng(7)
    positions = np.column_stack([rng.uniform(1.5, 6.5, 8), rng.uniform(3.0, 5.5, 8)])
    agents = [AgentState(f"a{i}", positions[i].copy()) for i in range(len(positions))]

    history = [coverage_cost(np.vstack([a.position for a in agents]), density, 0.8, DOMAIN, resolution=180)]
    rises = 0
    for _ in range(40):
        centroids = [
            cvt.compute(i, agents, [j for j in range(len(agents)) if j != i], density, DOMAIN).centroid
            for i in range(len(agents))
        ]
        for agent, centroid in zip(agents, centroids):
            agent.position = agent.position + 0.25 * (centroid - agent.position)
        history.append(coverage_cost(np.vstack([a.position for a in agents]), density, 0.8, DOMAIN, resolution=180))
        if history[-1] > history[-2] * (1.0 + 1e-6):
            rises += 1
    assert history[-1] < history[0]
    assert rises == 0


def test_descent_is_not_unconditional_in_the_step_size():
    """Move-to-centroid decreases H only below a step-size bound. Stating it
    without the bound would be false, so the bound is exercised here."""
    density = wall_density()
    cvt = LocalCVT(local_radius=0.8, grid_resolution=31, comm_range=1.6)
    rng = np.random.default_rng(7)
    positions = np.column_stack([rng.uniform(1.5, 6.5, 8), rng.uniform(3.0, 5.5, 8)])

    def rises_at(gain: float) -> int:
        agents = [AgentState(f"a{i}", positions[i].copy()) for i in range(len(positions))]
        previous = coverage_cost(np.vstack([a.position for a in agents]), density, 0.8, DOMAIN, resolution=180)
        rises = 0
        for _ in range(40):
            centroids = [
                cvt.compute(i, agents, [j for j in range(len(agents)) if j != i], density, DOMAIN).centroid
                for i in range(len(agents))
            ]
            for agent, centroid in zip(agents, centroids):
                agent.position = agent.position + gain * (centroid - agent.position)
            current = coverage_cost(np.vstack([a.position for a in agents]), density, 0.8, DOMAIN, resolution=180)
            if current > previous * (1.0 + 1e-6):
                rises += 1
            previous = current
        return rises

    assert rises_at(0.25) == 0
    assert rises_at(1.0) > 0


def test_uncovered_mass_is_charged_at_the_saturation_value():
    """Dropping the mass outside every disk made an earlier version of H fall for
    the wrong reason, so the descent property appeared to be violated when it was
    not."""
    density = wall_density()
    far = coverage_cost(np.array([[0.5, 0.5]]), density, 0.8, DOMAIN, resolution=90)
    none = coverage_cost(np.empty((0, 2)), density, 0.8, DOMAIN, resolution=90)
    assert none > 0.0
    assert far == pytest.approx(none, rel=0.05)


# --------------------------------------------------------------------------- #
# cell mass and the redeploy test
# --------------------------------------------------------------------------- #


def test_far_robot_has_an_essentially_empty_cell():
    density = wall_density()
    cvt = LocalCVT(local_radius=0.8, grid_resolution=21, comm_range=1.6)
    far = cvt.compute(0, [AgentState("a0", np.array([1.0, 1.0]))], [], density, DOMAIN)
    near = cvt.compute(0, [AgentState("a0", np.array([4.0, 4.14]))], [], density, DOMAIN)
    threshold = empty_cell_threshold(0.8, 1e-3, 3.0)
    assert far.cell_mass <= threshold < near.cell_mass


def test_held_fraction_rises_when_neighbours_cover_the_boundary():
    xs = np.arange(3.0, 5.0001, 0.06)
    observations = [
        BoundaryObservation("obj", "m", np.array([x, 4.0]), np.array([0.0, 1.0]), 0.0, 1.0, arc_length=0.06)
        for x in xs
    ]
    params = DensityParams(mode="offset", cage_offset=0.135, sigma=0.12, base_density=1e-3, gap_gain=0.6, gap_radius=0.35)
    cvt = LocalCVT(local_radius=0.8, grid_resolution=21, comm_range=1.6)
    agents = [AgentState("a0", np.array([4.0, 4.135]))]

    lonely = BoundaryAwareDensity.from_observations(
        observations, params, robot_positions=np.array([[4.0, 4.135]])
    )
    crowd = np.array([[4.0, 4.135], [3.6, 4.135], [4.4, 4.135], [3.2, 4.135], [4.8, 4.135]])
    crowded = BoundaryAwareDensity.from_observations(observations, params, robot_positions=crowd)

    alone_cell = cvt.compute(0, agents, [], lonely, DOMAIN)
    crowded_cell = cvt.compute(0, agents, [], crowded, DOMAIN)
    assert crowded_cell.held_fraction > alone_cell.held_fraction


def test_empty_cell_returns_the_agent_position():
    density = wall_density()
    cvt = LocalCVT(local_radius=0.05, grid_resolution=5, comm_range=1.6)
    agents = [AgentState("a0", np.array([1.0, 1.0]))]
    result = cvt.compute(0, agents, [], density, DOMAIN)
    assert result.centroid == pytest.approx(agents[0].position)
