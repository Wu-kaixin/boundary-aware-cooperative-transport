"""D10-DIAG - where the frames between detection and contact-ready actually go.

Evaluation-only, like ``metrics``: everything here may read the true cargo
geometry, and nothing here is on the control path. The question it answers is the
one three approach heuristics failed to answer by being tried --

    the far-field pipeline detects the object in 78 frames and reaches
    contact-ready in 591; which states is the team in for the other 513?

Two design decisions are what make the answer usable rather than another opinion.

**Coverage is measured on the true boundary, in arc length.** A bearing histogram
about a centroid is cheap and wrong on a non-convex shape: two points of the L's
perimeter at the same bearing are different pieces of boundary, and the concave
notch is systematically under-counted. The observed set is therefore the set of
uniformly-spaced boundary samples that lie within a tolerance of *some* robot's
map point, and the unobserved arc is the longest cyclic run of samples that do
not -- in metres, comparable directly against the sensor range and the object's
own perimeter.

**The segmentation is an exclusive cascade on measured state, not a schedule.**
Each post-detection frame is labelled by the furthest stage whose precondition
holds, so the labels partition the interval and the durations sum to it. No frame
range is assigned by hand, and no segment can be created or removed by choosing
where to cut. The one threshold that matters -- how much unobserved boundary
counts as "the far side is unknown" -- is a parameter, and the driver reports the
segmentation at three values of it rather than at one.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .cargo import Cargo

# The seven post-detection stages, in the order a working pipeline would pass
# through them. ``D`` is placed before ``E`` and ``F`` on purpose: it is the
# stage in which the existing redeploy rule is actually driving somebody, so if
# the rule were doing the work the mass would land there rather than in ``E``.
SEGMENTS: tuple[tuple[str, str], ...] = (
    ("A", "TOKEN_RECALL"),
    ("B", "FIRST_ARRIVAL"),
    ("C", "LOCAL_MAPPING"),
    ("D", "REDEPLOY"),
    ("E", "BACKSIDE_DISCOVERY"),
    ("F", "ENCLOSURE_CONVERGENCE"),
    ("G", "CONTACT_FORMATION"),
)
SEGMENT_LABELS: dict[str, str] = dict(SEGMENTS)


@dataclass
class SegmentRules:
    """Thresholds the cascade reads. Each one is a quantity, not a frame number."""

    quorum: int = 4
    arrival_radius: float = 0.80
    informed_fraction: float = 0.50
    # Fraction of the true perimeter that has to be missing from the union of all
    # robots' maps before the far side counts as unknown.
    unobserved_arc_fraction: float = 0.20


@dataclass
class FrameRecord:
    """One frame of team state. Onboard quantities and truth, kept apart."""

    frame: int
    phase: int
    agents: int = 0
    # --- onboard ---
    informed: int = 0
    direct_visible: int = 0
    arrived: int = 0
    contact_ready: int = 0
    redeploy_active: int = 0
    redeploy_requested: int = 0
    redeploy_no_candidate: int = 0
    candidates_total: int = 0
    candidates_max: int = 0
    agents_with_candidates: int = 0
    cells_saturated: int = 0
    map_angular_coverage_max: float = 0.0
    # Mean commanded speed of the robots that have arrived but are not yet
    # contact-ready. This is the discriminator between "stuck" and "busy": a team
    # sitting in a local equilibrium reports near zero here, and a team that is
    # moving hard and getting nowhere reports the speed limit.
    mean_speed_waiting: float = 0.0
    mean_centroid_distance_waiting: float = 0.0
    # --- truth (evaluation only) ---
    union_map_coverage: float = 0.0
    largest_unobserved_arc: float = 0.0
    largest_unheld_arc: float = 0.0
    strict_coverage: float = 0.0
    contact_count: int = 0
    min_inter_agent: float = float("inf")
    backside_observed: bool = False
    # --- per agent ---
    modes: list[str] = field(default_factory=list)
    boundary_distance: np.ndarray | None = None
    redeploy_reasons: list[str] = field(default_factory=list)
    positions: np.ndarray | None = None
    mode_counts: dict[str, int] = field(default_factory=dict)
    reason_counts: dict[str, int] = field(default_factory=dict)
    # Robots whose CVT cell carries no boundary mass at all. They are in
    # ``approach`` mode and never reach the redeploy rule, so counting them as
    # "cell saturated" -- which ``held_fraction`` does, since an empty cell is
    # vacuously fully held -- would report a rule as gated when it was never
    # consulted.
    empty_cells: int = 0


def observed_boundary_mask(
    cargo: Cargo, map_points: np.ndarray, tolerance: float, samples: int = 160
) -> np.ndarray:
    """Which uniformly-spaced boundary samples lie within ``tolerance`` of a map point.

    ``tolerance`` should be sized against the map's own resolution -- a voxel map
    cannot place a point closer to the surface than half a voxel, so anything
    tighter measures the discretisation rather than the observation.
    """
    boundary, _ = cargo.boundary_samples(samples)
    if len(map_points) == 0:
        return np.zeros(len(boundary), dtype=bool)
    distance = np.min(
        np.linalg.norm(boundary[:, None, :] - np.asarray(map_points)[None, :, :], axis=2), axis=1
    )
    return distance <= tolerance


def occupied_boundary_mask(
    cargo: Cargo, positions: np.ndarray, contact_radius: float, samples: int = 160
) -> np.ndarray:
    """Which boundary samples have a robot within ``contact_radius`` of them."""
    boundary, _ = cargo.boundary_samples(samples)
    if len(positions) == 0:
        return np.zeros(len(boundary), dtype=bool)
    distance = np.min(
        np.linalg.norm(boundary[:, None, :] - np.asarray(positions)[None, :, :], axis=2), axis=1
    )
    return distance <= contact_radius


def longest_false_run(mask: np.ndarray) -> int:
    """Longest cyclic run of ``False`` in a boolean mask over a closed curve.

    Cyclic because the perimeter is closed: a gap that straddles sample 0 is one
    gap, and reporting it as two is how a straightforward scan understates exactly
    the case the diagnosis is looking for.
    """
    n = len(mask)
    if n == 0:
        return 0
    if not np.any(mask):
        return n
    doubled = np.concatenate([mask, mask])
    best = run = 0
    for value in doubled:
        run = 0 if value else run + 1
        best = max(best, run)
    return min(best, n)


def arc_of_run(run_length: int, perimeter: float, samples: int) -> float:
    return perimeter * run_length / max(samples, 1)


def backside_samples(cargo: Cargo, observer: np.ndarray, samples: int = 160) -> np.ndarray:
    """Boundary samples on the far side of the object from a given observer.

    "Far side" is fixed once, at the moment of first detection, from the bearing
    of the robot that made it. Re-deriving it each frame would make it a function
    of where the team has already got to, and the quantity would then measure the
    team rather than the object.
    """
    boundary, _ = cargo.boundary_samples(samples)
    centre = cargo.position
    front = np.asarray(observer, dtype=float).reshape(2) - centre
    norm = float(np.linalg.norm(front))
    if norm < 1e-9:
        return np.zeros(len(boundary), dtype=bool)
    front = front / norm
    bearing = boundary - centre[None, :]
    bearing = bearing / np.maximum(np.linalg.norm(bearing, axis=1), 1e-12)[:, None]
    return (bearing @ front) < 0.0


def classify_frame(record: FrameRecord, rules: SegmentRules, perimeter: float) -> str:
    """Label one post-detection frame with the furthest stage it has reached.

    Read top to bottom; the first match wins, so the labels are exclusive and the
    durations partition the interval.

    ``G`` a quorum is already in the contact band -- what remains is the dwell.
    ``D`` a quorum has arrived and the redeploy rule is driving somebody.
    ``E`` a quorum has arrived, nobody is redeploying, and a fifth or more of the
          perimeter is in nobody's map. The team is at the object and cannot see
          where it has to go.
    ``F`` a quorum has arrived, the boundary is mapped, and the coverage law is
          spreading onto it.
    ``C`` somebody has reached the object; the rest have not.
    ``B`` the team knows and is travelling.
    ``A`` the news has not reached half the team.
    """
    if record.contact_ready >= rules.quorum:
        return "G"
    if record.arrived >= rules.quorum:
        if record.redeploy_active > 0:
            return "D"
        if record.largest_unobserved_arc > rules.unobserved_arc_fraction * perimeter:
            return "E"
        return "F"
    if record.arrived >= 1:
        return "C"
    if record.informed >= max(1, int(np.ceil(rules.informed_fraction * max(record.agents, 1)))):
        return "B"
    return "A"


def segment(
    records: list[FrameRecord],
    rules: SegmentRules,
    perimeter: float,
    start: int,
    end: int,
) -> dict[str, int]:
    """Frame counts per stage over ``[start, end)``, one label per frame."""
    counts = {key: 0 for key, _ in SEGMENTS}
    for record in records:
        if start <= record.frame < end:
            counts[classify_frame(record, rules, perimeter)] += 1
    return counts


__all__ = [
    "SEGMENTS",
    "SEGMENT_LABELS",
    "SegmentRules",
    "FrameRecord",
    "observed_boundary_mask",
    "occupied_boundary_mask",
    "longest_false_run",
    "arc_of_run",
    "backside_samples",
    "classify_frame",
    "segment",
]
