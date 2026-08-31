"""S5 - coverage layer: limited-range Voronoi coverage with a truncated cost.

    Omega_i = D  intersect  B(p_i, R_l)                 strict local disk
    V_i     = { q in Omega_i : ||q-p_i|| <= ||q-p_j||, j in N_i }
    f(r)    = min(r^2, R_l^2)                            truncated performance
    H(p)    = integral_D  min_i f(||q - p_i||) phi(q) dq

Two things here are load-bearing.

**The truncation.** ``f`` is continuous at ``r = R_l``, so the flux term over the
circle ``dB(p_i, R_l)`` cancels when ``H`` is differentiated and
``dH/dp_i = -2 m_i (c_i - p_i)``: move-to-centroid is a descent direction. This is
the construction of Cortes, Martinez and Bullo (ESAIM COCV 2005). Using
``||q-p_i||^2`` without truncating leaves an uncancelled boundary term and the
descent statement is simply false. Note also that descent holds for a step size
below an explicit bound -- with a large gain the cost does rise on some steps, so
the bound belongs in the paper rather than an unqualified "move-to-centroid
decreases H".

**The integration domain.** It must be the disk itself. The previous version
integrated over the local box *unioned with the bounding box of all density
targets*, which meant ``local_radius`` barely bound anything: in an 8x8 domain a
robot at (6, 6) integrated over a 7.06 x 7.06 box and sampled points 7.16 m away.
That is a centralised computation wearing a local name.

**Neighbour completeness.** ``R_l <= R_comm/2`` makes the cell computed from
communication neighbours *equal* to the true Voronoi cell restricted to the disk,
not an approximation of it: if ``q`` in ``B(p_i, R_l)`` is closer to ``p_j``, then
``||p_i - p_j|| <= ||p_i - q|| + ||q - p_j|| <= 2 R_l <= R_comm``.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from .boundary_density import BoundaryAwareDensity
from .contracts import CoverageContract
from .types import AgentState


@dataclass
class CVTResult:
    centroid: np.ndarray
    cell_mass: float
    sample_count: int
    owned_samples: int
    unheld_mass: float = 0.0

    @property
    def held_fraction(self) -> float:
        """Fraction of this cell's boundary mass that neighbours already hold."""
        if self.cell_mass <= 0.0:
            return 1.0
        return float(np.clip(1.0 - self.unheld_mass / self.cell_mass, 0.0, 1.0))


@dataclass
class LocalCVT:
    """Grid-quadrature limited-range Voronoi centroid over a strict disk."""

    local_radius: float = 0.8
    grid_resolution: int = 24
    comm_range: float | None = None
    warn_on_contract: bool = True

    def __post_init__(self) -> None:
        if self.comm_range is not None and self.warn_on_contract:
            problems = CoverageContract(self.local_radius, self.comm_range).violations()
            for problem in problems:
                warnings.warn(problem, RuntimeWarning, stacklevel=2)

    # ------------------------------------------------------------------ #

    def cell_samples(
        self,
        agent_index: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        domain: tuple[float, float, float, float],
    ) -> tuple[np.ndarray, float]:
        """Quadrature points of ``V_i`` plus the area element."""
        position = np.asarray(agents[agent_index].position, dtype=float).reshape(2)
        xmin, xmax, ymin, ymax = domain
        lo = np.array([max(position[0] - self.local_radius, xmin), max(position[1] - self.local_radius, ymin)])
        hi = np.array([min(position[0] + self.local_radius, xmax), min(position[1] + self.local_radius, ymax)])
        if hi[0] <= lo[0] or hi[1] <= lo[1]:
            return np.empty((0, 2)), 0.0

        n = max(4, int(self.grid_resolution))
        xs = np.linspace(lo[0], hi[0], n)
        ys = np.linspace(lo[1], hi[1], n)
        cell_area = ((hi[0] - lo[0]) / (n - 1)) * ((hi[1] - lo[1]) / (n - 1))
        xx, yy = np.meshgrid(xs, ys)
        samples = np.column_stack([xx.ravel(), yy.ravel()])

        # Strict disk, not the bounding box.
        rel = samples - position[None, :]
        samples = samples[np.sum(rel * rel, axis=1) <= self.local_radius ** 2]
        if len(samples) == 0:
            return samples, cell_area

        if neighbor_indices:
            neighbors = np.vstack([agents[j].position for j in neighbor_indices])
            own = np.sum((samples - position[None, :]) ** 2, axis=1)
            other = np.min(np.sum((samples[:, None, :] - neighbors[None, :, :]) ** 2, axis=2), axis=1)
            samples = samples[own <= other]
        return samples, cell_area

    def compute(
        self,
        agent_index: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        density: BoundaryAwareDensity,
        domain: tuple[float, float, float, float],
    ) -> CVTResult:
        position = np.asarray(agents[agent_index].position, dtype=float).reshape(2)
        samples, cell_area = self.cell_samples(agent_index, agents, neighbor_indices, domain)
        if len(samples) == 0:
            return CVTResult(position.copy(), 0.0, 0, 0)

        local_density = density.restrict(position, self.local_radius)
        weights = np.atleast_1d(local_density(samples))
        total = float(np.sum(weights))
        cell_mass = total * cell_area
        unheld_mass = float(np.sum(local_density.unheld_field(samples))) * cell_area
        if total <= 1e-12:
            return CVTResult(position.copy(), cell_mass, len(samples), len(samples), unheld_mass)

        centroid = np.sum(samples * weights[:, None], axis=0) / total
        return CVTResult(centroid, cell_mass, len(samples), len(samples), unheld_mass)

    # backwards-compatible thin wrapper
    def compute_centroid(
        self,
        agent_index: int,
        agents: list[AgentState],
        neighbor_indices: list[int],
        density: BoundaryAwareDensity,
        domain: tuple[float, float, float, float],
    ) -> np.ndarray:
        return self.compute(agent_index, agents, neighbor_indices, density, domain).centroid


def empty_cell_threshold(local_radius: float, base_density: float, ratio: float) -> float:
    """Cell mass below which a cell carries no boundary information.

    A cell filled with nothing but base density has mass
    ``phi_0 * pi * R_l^2``; ``ratio`` says how many multiples of that still count
    as empty.
    """
    return ratio * base_density * np.pi * local_radius ** 2


def coverage_cost(
    positions: np.ndarray,
    density: BoundaryAwareDensity,
    local_radius: float,
    domain: tuple[float, float, float, float],
    resolution: int = 120,
) -> float:
    """The truncated coverage functional ``H`` evaluated by grid quadrature.

    Written in the equivalent global form ``H = int_D min_i f(||q-p_i||) phi dq``.
    The mass that lies outside every disk is *not* dropped: it is charged at the
    saturation value ``R_l^2``. Omitting it was what made an earlier version of
    this functional rise on 21 of 80 move-to-centroid steps -- mass leaving every
    disk simply stopped being counted, so the number went down for the wrong
    reason and the descent property looked violated when it was not.
    """
    p = np.asarray(positions, dtype=float).reshape(-1, 2)
    xmin, xmax, ymin, ymax = domain
    xs = np.linspace(xmin, xmax, resolution)
    ys = np.linspace(ymin, ymax, resolution)
    cell_area = ((xmax - xmin) / (resolution - 1)) * ((ymax - ymin) / (resolution - 1))
    xx, yy = np.meshgrid(xs, ys)
    q = np.column_stack([xx.ravel(), yy.ravel()])

    weights = np.atleast_1d(density(q))
    if len(p) == 0:
        return float(np.sum(local_radius ** 2 * weights) * cell_area)
    d2 = np.min(np.sum((q[:, None, :] - p[None, :, :]) ** 2, axis=2), axis=1)
    f = np.minimum(d2, local_radius ** 2)
    return float(np.sum(f * weights) * cell_area)


__all__ = ["LocalCVT", "CVTResult", "coverage_cost", "empty_cell_threshold"]
