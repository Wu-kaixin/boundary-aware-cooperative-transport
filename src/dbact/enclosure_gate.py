"""D10-ENC - what "the team has enclosed the object" is allowed to mean.

The committed `DISCOVER -> ENCLOSE` guard is

    max_i  angular_coverage( map_i )  >=  0.70

-- the *best single robot's own map*, measured about *that map's own centroid*.
D10-DIAG measured what that costs: it correlated 0.943 with the contact-ready
frame and on three of eight seeds it *was* the contact-ready frame, while a
contact quorum stood on the object waiting for it. Two things are wrong with it
as a proxy for a team-level property, and they are different faults:

**It is a statement about one robot's knowledge, not the team's.** Sixteen robots
each holding a quarter of the boundary have enclosed the object and the guard
reads 0.25. The quantity that matches the claim is the union over the team, and
the union is not a centralised quantity: it is a max-consensus, which is what
this module provides.

**It is a bearing histogram about a point derived from the evidence, and that is
ill-posed.** ``angular_coverage`` bins bearings about the centroid *of the map
points themselves*. When the map is a sliver the centroid lies on the sliver, and
the bearings of the points about it say nothing about how much of the object has
been seen -- a scatter of noisy returns around a corner fills most of the circle.
Repairing this by moving the origin to the relayed token does not help, because
the token is another robot's map centroid: measured on seed 1, a union bearing
histogram about the token reaches 0.80 at **frame 22, with true strict coverage
0.000 and 1.5 m of the perimeter unheld.** The fault is the family of measure,
not which robot supplies it, and the fix is to stop needing an origin.

**Observed normals need no origin.** A robot in the contact band knows the
outward normal of the surface it is touching. If the team's normals leave no
angular gap wider than ``gap_max``, then for every direction there is boundary
known -- and, for the held version, a robot standing on it -- with a component
opposing that direction. That is the property "enclosure" is supposed to name,
it is what the transport actually needs, and it is invariant to translation of
the object, to where the team happens to be, and to any estimate of the object's
size or centre.

Nothing here may read the simulator. A gate that consults truth is not a gate,
and the counterfactual evaluation in `scripts/diagnose_enclosure_gate.py` keeps
the truth quantities strictly on the scoring side.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

BINS = 36


def bearing_bins(points: np.ndarray, reference: np.ndarray, bins: int = BINS) -> np.ndarray:
    """Which bearing bins about ``reference`` contain at least one of ``points``."""
    mask = np.zeros(bins, dtype=bool)
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return mask
    delta = pts - np.asarray(reference, dtype=float).reshape(1, 2)
    radius = np.linalg.norm(delta, axis=1)
    live = radius > 1e-9
    if not np.any(live):
        return mask
    angle = np.arctan2(delta[live, 1], delta[live, 0])
    index = ((angle + np.pi) / (2.0 * np.pi) * bins).astype(int) % bins
    mask[index] = True
    return mask


def direction_bins(vectors: np.ndarray, bins: int = BINS) -> np.ndarray:
    """Which direction bins the given unit vectors occupy.

    The same binning as ``bearing_bins`` with the subtraction removed, which is
    the whole difference: a direction is an intrinsic property of an observation
    and a bearing is a property of an observation *and a chosen origin*. Only the
    first can be compared between robots that disagree about where the object is.
    """
    mask = np.zeros(bins, dtype=bool)
    v = np.asarray(vectors, dtype=float).reshape(-1, 2)
    if len(v) == 0:
        return mask
    norm = np.linalg.norm(v, axis=1)
    live = norm > 1e-9
    if not np.any(live):
        return mask
    angle = np.arctan2(v[live, 1], v[live, 0])
    index = ((angle + np.pi) / (2.0 * np.pi) * bins).astype(int) % bins
    mask[index] = True
    return mask


def occupancy(mask: np.ndarray) -> float:
    return float(np.count_nonzero(mask)) / max(len(mask), 1)


def largest_gap(mask: np.ndarray) -> int:
    """Longest cyclic run of empty bins.

    Cyclic, because the bearing circle is closed: a gap straddling bin 0 is one
    gap, and counting it as two understates exactly the configuration -- a team
    piled on one side -- that this whole exercise is about.
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


def gap_degrees(mask: np.ndarray) -> float:
    return 360.0 * largest_gap(mask) / max(len(mask), 1)


@dataclass
class BitmapConsensus:
    """Max-consensus on a per-bin timestamp, one hop per step.

    The message is 36 numbers per object -- smaller than the boundary observations
    already on the link -- and the update is the same one the object token
    already uses: keep the freshest value per key, drop anything older than a TTL.
    Over a connected communication graph the value at every robot reaches the
    team-wide maximum in at most graph-diameter steps, so this is decentralised in
    the same sense the recall is, and the lag it costs is measurable rather than
    assumed.

    ``ttl = None`` makes a bin latch permanently once anybody has filled it. That
    is right for *observation* -- "has this bearing ever been seen" is monotone,
    and so is the phase machine it feeds. It is wrong for *occupancy*: a robot
    that leaves the contact band must stop counting, so the held-bin consensus
    runs with a finite TTL.
    """

    bins: int = BINS
    ttl: float | None = None
    stamps: dict[str, np.ndarray] = field(default_factory=dict)

    def ensure(self, agent_ids: list[str]) -> None:
        for agent_id in agent_ids:
            if agent_id not in self.stamps:
                self.stamps[agent_id] = np.full(self.bins, -np.inf)

    def step(
        self,
        own: dict[str, np.ndarray],
        neighbours: dict[str, list[str]],
        timestamp: float,
    ) -> None:
        """One round: fold in own observations, then one hop of neighbour maxima."""
        self.ensure(list(own))
        for agent_id, mask in own.items():
            self.stamps[agent_id] = np.where(mask, timestamp, self.stamps[agent_id])
        previous = {agent_id: stamp.copy() for agent_id, stamp in self.stamps.items()}
        for agent_id, others in neighbours.items():
            merged = previous[agent_id]
            for other in others:
                if other in previous:
                    merged = np.maximum(merged, previous[other])
            self.stamps[agent_id] = merged

    def view(self, agent_id: str, timestamp: float) -> np.ndarray:
        stamp = self.stamps.get(agent_id)
        if stamp is None:
            return np.zeros(self.bins, dtype=bool)
        if self.ttl is None:
            return np.isfinite(stamp)
        return (timestamp - stamp) <= self.ttl


# --------------------------------------------------------------------------- #
# candidate certificates
# --------------------------------------------------------------------------- #


@dataclass
class GateInputs:
    """One frame of everything a candidate gate is allowed to read.

    Every field is computable on board. The truth quantities live in the
    evaluation record, not here, and keeping them in separate objects is what
    stops a gate from quietly acquiring one.
    """

    informed: int
    agents: int
    best_own_coverage: float
    # Union over the team, by consensus, of the *directions* of observed boundary
    # normals and of the normals under robots that are in the contact band.
    known_normal_gap_deg: float
    held_normal_gap_deg: float
    held_normal_bins: int
    contact_ready: int
    # Kept only so the diagnosis can show the bearing family failing; no candidate
    # gate below reads them.
    union_bearing_coverage: float = 0.0
    union_bearing_gap_deg: float = 360.0


def g0_current(x: GateInputs, coverage_min: float = 0.70, informed_fraction: float = 0.55) -> bool:
    """The committed guard: the best single robot's own map, about its own centroid."""
    enough = x.informed >= max(1, int(round(informed_fraction * max(x.agents, 1))))
    return enough and x.best_own_coverage >= coverage_min


def g1_known(x: GateInputs, gap_max_deg: float = 120.0, informed_fraction: float = 0.55) -> bool:
    """The team has seen boundary facing every direction, to within a gap."""
    enough = x.informed >= max(1, int(round(informed_fraction * max(x.agents, 1))))
    return enough and x.known_normal_gap_deg <= gap_max_deg


def g2_operational(x: GateInputs, quorum: int = 4, gap_max_deg: float = 120.0) -> bool:
    """Enclosure as a property of where the robots are standing.

    Robots in the contact band whose observed outward normals leave no gap wider
    than ``gap_max_deg``: for every direction, somebody is on a face that opposes
    it. This says nothing about what the team knows and everything about where it
    is, which is the half of "enclosure" the current guard does not measure.
    """
    return x.held_normal_bins >= quorum and x.held_normal_gap_deg <= gap_max_deg


def g3_hybrid(
    x: GateInputs,
    known_gap_deg: float = 120.0,
    held_gap_deg: float = 120.0,
    quorum: int = 4,
    informed_fraction: float = 0.55,
) -> bool:
    """Both: the team has seen boundary facing every way *and* is standing round it."""
    return g1_known(x, known_gap_deg, informed_fraction) and g2_operational(x, quorum, held_gap_deg)


__all__ = [
    "BINS",
    "BitmapConsensus",
    "GateInputs",
    "bearing_bins",
    "direction_bins",
    "occupancy",
    "largest_gap",
    "gap_degrees",
    "g0_current",
    "g1_known",
    "g2_operational",
    "g3_hybrid",
]
