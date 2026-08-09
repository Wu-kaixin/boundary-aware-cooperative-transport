"""S4 - density layer: a measure on the observed boundary.

Offset model
------------
    phi(q) = phi_0 + sum_k  ds_k * c_k * (1 + kappa * g_k) * K_sigma(q - xi_k)
    xi_k   = b_k + d_c * n_k

The four factors are each a physical quantity rather than a tuning knob:
``ds_k`` the arc length the return stands for (this is what makes phi a boundary
measure instead of a sample count), ``c_k`` the normal-estimate confidence
already faded by observation age in the map layer, and ``g_k`` an uncovered-gap
term computed from neighbour positions alone.

Where the offset model fails, and by how much
---------------------------------------------
Pointwise offsetting ``b -> b + d_c n`` has Jacobian ``|1 - d_c * kappa_curv|``,
which degenerates at a concave point of curvature radius ``R_c`` once
``d_c = R_c``: the offset curve folds into a swallowtail and targets pile up in
the concavity. The effect is real and locatable -- for the L shape at scale 0.9
with ``d_c = 0.32`` the analytic self-intersection sits at ``(0.23, 0.185)`` and
the measured density peak sits at ``(0.232, 0.185)`` -- but its magnitude is
moderate, around 1.6x the median. The serious symptom is not the peak: it is that
a fraction of the offset targets land *inside* the object, i.e. the cage target
is geometrically unreachable.

Distance-field model
--------------------
    d(q)   = min_k ||q - b_k||
    phi(q) = phi_0 + w(q) * exp(-(d(q) - d_c)^2 / (2 sigma^2))   on the outer side

The level set is the boundary of the Minkowski sum of the object with a disk of
radius ``d_c``, which is always a simple curve and never self-intersects. Normals
are used only to pick the outer side, so a large normal error cannot fold it.
The cost is interpretive: total mass is no longer proportional to estimated
perimeter, because the measure now lives on the level-set curve rather than on
weighted samples. Both modes are kept and both are run, so the difference is an
ablation row instead of an unexplained choice.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .types import BoundaryObservation, BoundaryView

_EPS = 1e-12


@dataclass
class DensityParams:
    mode: str = "offset"
    cage_offset: float = 0.135
    sigma: float = 0.20
    base_density: float = 1e-3
    gap_gain: float = 0.0
    gap_radius: float = 0.35
    influence_sigmas: float = 3.0
    lead_offset: float | None = None
    lead_threshold: float = 0.35
    # D10 -- exploration demand for boundary nobody has observed. ``0.0`` is off
    # and reproduces the density exactly, which is what makes the A/B an ablation
    # of one term rather than a comparison of two controllers.
    explore_gain: float = 0.0
    explore_step: float = 0.25
    explore_window: float = 0.18

    def __post_init__(self) -> None:
        if self.mode not in ("offset", "distance_field"):
            raise ValueError(f"density mode must be 'offset' or 'distance_field', got {self.mode!r}")
        if self.explore_gain < 0.0:
            raise ValueError("explore_gain must be non-negative")

    def offsets_for(self, normals: np.ndarray, goal_direction: np.ndarray | None) -> np.ndarray:
        """Per-observation cage offset, graded by how much that face would resist.

        A cage of rigid kinematic robots at a single offset cannot be pushed
        anywhere. A robot in contact at a boundary point with outward normal ``n``
        applies force along ``-n``, so its contribution along the goal is
        ``-(n . u_goal) |F|``: every robot on the leading half subtracts exactly in
        proportion to how far forward it sits. Measured on the L shape with a
        uniform cage, three robots were in contact and their goal components came to
        +1.33, +1.89 and -3.31 N -- a net of -0.09 N against a 12.7 N breakaway
        force, so the object sat still no matter what the gains were.

        The offset therefore ramps with the resisting component: zero alignment or
        better keeps the contact offset ``d_c``, and by ``lead_threshold`` the robot
        is lifted to ``lead_offset``, outside contact, where it still bounds the
        object's forward motion but cannot oppose it. The lateral arc stays in
        contact, which is what supplies containment and the tangential friction that
        does help. The grading is continuous, so a robot near the boundary between
        arcs does not chatter between two targets.

        One dot product per observation, computed from the robot's own estimated
        normal and the task direction, so the allocation stays decentralised.
        """
        base = np.full(len(normals), self.cage_offset)
        if self.lead_offset is None or goal_direction is None or len(normals) == 0:
            return base
        alignment = normals @ np.asarray(goal_direction, dtype=float).reshape(2)
        ramp = np.clip(alignment / max(self.lead_threshold, 1e-9), 0.0, 1.0)
        return base + ramp * (self.lead_offset - self.cage_offset)


class BoundaryAwareDensity:
    """Boundary-aware coverage density in either of the two models."""

    def __init__(
        self,
        points: np.ndarray,
        normals: np.ndarray,
        weights: np.ndarray,
        params: DensityParams,
        object_ids: list[str] | None = None,
        gap: np.ndarray | None = None,
        offsets: np.ndarray | None = None,
    ):
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        self.normals = np.asarray(normals, dtype=float).reshape(-1, 2)
        self.weights = np.asarray(weights, dtype=float).reshape(-1)
        self.params = params
        self.object_ids = list(object_ids) if object_ids is not None else []
        self.offsets = (
            np.full(len(self.points), params.cage_offset)
            if offsets is None
            else np.asarray(offsets, dtype=float).reshape(-1)
        )
        # Per-observation uncovered-gap factor, kept separately from the weights it
        # already multiplies. The coverage functional uses the combined weight; the
        # redeploy decision needs to ask specifically how much *unheld* boundary a
        # cell contains, which the combined weight cannot answer because a fully
        # held arc still carries its arc length and confidence.
        self.gap = (
            np.ones(len(self.points))
            if gap is None
            else np.asarray(gap, dtype=float).reshape(-1)
        )

    # ------------------------------------------------------------------ #
    # construction
    # ------------------------------------------------------------------ #

    @classmethod
    def from_observations(
        cls,
        observations: list[BoundaryObservation],
        params: DensityParams,
        robot_positions: np.ndarray | None = None,
        goal_direction: np.ndarray | None = None,
    ) -> "BoundaryAwareDensity":
        return cls.from_view(
            BoundaryView.from_observations(list(observations)),
            params,
            robot_positions=robot_positions,
            goal_direction=goal_direction,
        )

    @classmethod
    def from_view(
        cls,
        view: BoundaryView,
        params: DensityParams,
        robot_positions: np.ndarray | None = None,
        goal_direction: np.ndarray | None = None,
    ) -> "BoundaryAwareDensity":
        if len(view) == 0:
            return cls(np.empty((0, 2)), np.empty((0, 2)), np.empty(0), params)
        points = view.points
        normals = view.normals
        offsets = params.offsets_for(normals, goal_direction)
        arc = np.maximum(view.arc_length, 0.0)

        # A map that never had arc lengths (e.g. a hand-built observation) still
        # has to produce a usable density, so fall back to unit weight per point.
        if float(np.sum(arc)) <= _EPS:
            arc = np.ones_like(arc)

        weights = arc * view.confidence
        object_ids = [str(name) for name in view.object_ids]

        # D10 -- the exploration term. Everything above is a measure on boundary
        # that has been *seen*; this adds mass just past the ends of what has been
        # seen, so the same coverage law that spreads robots along observed
        # boundary also pulls one of them off the end of it.
        # Offset mode only. The distance-field model reads ``points`` as the
        # boundary itself, so a virtual target appended to it would move the level
        # set rather than add demand beside it.
        if params.explore_gain > 0.0 and params.mode == "offset":
            frontier, frontier_normals, source = _frontier_targets(
                points, normals, offsets, params
            )
            if len(frontier):
                unit = float(np.mean(weights)) if len(weights) else 1.0
                points = np.vstack([points, frontier])
                normals = np.vstack([normals, frontier_normals])
                offsets = np.concatenate(
                    [offsets, np.full(len(frontier), params.cage_offset)]
                )
                weights = np.concatenate(
                    [weights, np.full(len(frontier), params.explore_gain * unit)]
                )
                object_ids = object_ids + [object_ids[k] for k in source]

        gap = None
        if robot_positions is not None and len(robot_positions) > 0:
            targets = points + offsets[:, None] * normals
            gap = _gap_weights(targets, robot_positions, params.gap_radius)
            if params.gap_gain > 0.0:
                weights = weights * (1.0 + params.gap_gain * gap)
        return cls(
            points,
            normals,
            weights,
            params,
            object_ids,
            gap=gap,
            offsets=offsets,
        )

    @classmethod
    def from_targets(
        cls,
        targets: list[np.ndarray] | np.ndarray,
        sigma: float,
        weights: list[float] | np.ndarray | None = None,
        base_density: float = 1e-3,
    ) -> "BoundaryAwareDensity":
        """Density peaked directly on given points. Used by the coverage task mode."""
        arr = np.asarray(targets, dtype=float).reshape(-1, 2)
        w = np.ones(len(arr)) if weights is None else np.asarray(weights, dtype=float).reshape(len(arr))
        params = DensityParams(mode="offset", cage_offset=0.0, sigma=sigma, base_density=base_density)
        return cls(arr, np.zeros_like(arr), w, params)

    # ------------------------------------------------------------------ #
    # evaluation
    # ------------------------------------------------------------------ #

    @property
    def targets(self) -> np.ndarray:
        """Cage targets of the offset model (the density peaks in that mode)."""
        if len(self.points) == 0:
            return np.empty((0, 2))
        return self.points + self.offsets[:, None] * self.normals

    @property
    def total_mass(self) -> float:
        return float(np.sum(self.weights))

    @property
    def influence_radius(self) -> float:
        return self.params.influence_sigmas * self.params.sigma

    def restrict(self, center: np.ndarray, radius: float) -> "BoundaryAwareDensity":
        """Drop points that cannot influence a query inside ``B(center, radius)``.

        Purely a cost reduction: a Gaussian beyond ``influence_sigmas`` sigma
        contributes below the base density, so the restricted field agrees with
        the full one to within that tolerance on the disk.
        """
        if len(self.points) == 0:
            return self
        c = np.asarray(center, dtype=float).reshape(2)
        reach = radius + float(np.max(self.offsets)) + self.influence_radius
        keep = np.linalg.norm(self.points - c[None, :], axis=1) <= reach
        if np.all(keep):
            return self
        return BoundaryAwareDensity(
            self.points[keep],
            self.normals[keep],
            self.weights[keep],
            self.params,
            [oid for oid, flag in zip(self.object_ids, keep) if flag] if self.object_ids else None,
            gap=self.gap[keep],
            offsets=self.offsets[keep],
        )

    def __call__(self, q: np.ndarray) -> np.ndarray | float:
        query = np.asarray(q, dtype=float)
        single = query.ndim == 1
        query = query.reshape(-1, 2)
        if len(self.points) == 0 or len(query) == 0:
            out = np.full(len(query), self.params.base_density)
            return float(out[0]) if single else out

        if self.params.mode == "distance_field":
            out = self._eval_distance_field(query)
        else:
            out = self._eval_offset(query, self.weights) + self.params.base_density
        return float(out[0]) if single else out

    def unheld_field(self, q: np.ndarray) -> np.ndarray:
        """Density restricted to boundary no robot is holding.

        Same kernel and same measure, with each observation scaled by its gap
        factor alone. Integrated over a Voronoi cell this answers "how much of the
        boundary in my cell still needs somebody", which is what decides whether a
        robot should stay or go somewhere more useful.
        """
        query = np.asarray(q, dtype=float).reshape(-1, 2)
        if len(self.points) == 0 or len(query) == 0:
            return np.zeros(len(query))
        return self._eval_offset(query, self.weights * self.gap)

    def _eval_offset(self, query: np.ndarray, weights: np.ndarray) -> np.ndarray:
        targets = self.targets
        diff = query[:, None, :] - targets[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        kernel = np.exp(-dist2 / (2.0 * self.params.sigma ** 2))
        return kernel @ weights

    def _eval_distance_field(self, query: np.ndarray) -> np.ndarray:
        diff = query[:, None, :] - self.points[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        nearest = np.argmin(dist2, axis=1)
        distance = np.sqrt(dist2[np.arange(len(query)), nearest])

        rel = query - self.points[nearest]
        outer = np.sum(rel * self.normals[nearest], axis=1) > 0.0

        # Weight is carried by the nearest observation so that confidence and the
        # gap term stay spatially local; arc length is not needed here because the
        # level-set curve is itself the measure.
        scale = np.sum(self.weights) / max(len(self.weights), 1)
        local_weight = self.weights[nearest] / max(scale, _EPS) if scale > _EPS else np.ones(len(query))
        ridge = np.exp(-((distance - self.offsets[nearest]) ** 2) / (2.0 * self.params.sigma ** 2))
        return self.params.base_density + np.where(outer, local_weight * scale * ridge, 0.0)

    def weighted_centroid(self, samples: np.ndarray) -> np.ndarray | None:
        if len(samples) == 0:
            return None
        weights = np.atleast_1d(self(samples))
        total = float(np.sum(weights))
        if total <= 1e-12:
            return None
        return np.sum(samples * weights[:, None], axis=0) / total


def _frontier_targets(
    points: np.ndarray, normals: np.ndarray, offsets: np.ndarray, params: DensityParams
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Where the observed boundary stops, continued one step along its own tangent.

    This is the whole of the D10 mechanism, and what it is *not* matters as much
    as what it is.

    Every navigation target in the post-arrival control path -- the CVT centroid,
    the approach target, the redeploy target -- is an affine function of the
    robot's own map points, so the reachable target set is contained in the offset
    ring over *observed* boundary. Nothing in the controller can therefore ask for
    boundary nobody has seen. Measured over eight far-field seeds, 84.5% of the
    redeploy rule's requests returned no candidate while 4.34 m of a 7.2 m
    perimeter sat in nobody's map: the rule was not choosing badly among
    candidates, it had none, and the missing ones were missing because they were
    unobserved rather than because they were owned.

    A frontier is an observation with no neighbour on one side of it along its own
    tangent. Continuing by ``explore_step`` in that direction names a place the
    boundary probably goes and certainly is not yet known, using only the map:

    * no object radius and no shape prior -- a tangent is local;
    * the demand disappears on its own, because once a robot stands there and
      scans, that direction acquires a neighbour and stops being a frontier;
    * it is a *density* term, so the existing limited-range CVT decides which
      robot goes. Nobody is put in a new mode, no robot is told to wall-follow,
      and the whole team cannot pile onto one target because the cells partition
      and the gap factor empties a target somebody is already standing on;
    * the safety layer never sees it. The barrier rows are built from the map, and
      a frontier target is not a map cell.

    The last filter is the one that took a test to get right. A convex corner
    passes the tangential test honestly -- the boundary really does stop going
    that way -- but the space past it is known, not unknown, and a naive version
    of this function put a phantom frontier outside all four corners of a fully
    mapped square. Comparing the candidate's *ring target* against the existing
    ring targets separates the two cases, because at a convex corner the offset
    ring wraps and the neighbouring face's target comes close, while past a
    genuinely open end the nearest existing target is a full step away. The test
    is on the ring rather than on the boundary for the same reason the ring is
    what the robots are aiming at.
    """
    count = len(points)
    if count < 2:
        return np.empty((0, 2)), np.empty((0, 2)), np.empty(0, dtype=int)

    tangents = np.column_stack([-normals[:, 1], normals[:, 0]])
    delta = points[None, :, :] - points[:, None, :]          # (k, j, 2)
    along = np.einsum("kjd,kd->kj", delta, tangents)
    across = np.abs(np.einsum("kjd,kd->kj", delta, normals))
    # A neighbour is a map point that lies within the tangential window of this
    # observation's own plane, on one side or the other.
    near = across <= params.explore_window
    forward = np.any(near & (along > _EPS) & (along <= params.explore_step), axis=1)
    backward = np.any(near & (along < -_EPS) & (along >= -params.explore_step), axis=1)

    open_forward = np.flatnonzero(~forward)
    open_backward = np.flatnonzero(~backward)
    if len(open_forward) == 0 and len(open_backward) == 0:
        return np.empty((0, 2)), np.empty((0, 2)), np.empty(0, dtype=int)

    step = params.explore_step
    proposed = np.vstack(
        [
            points[open_forward] + step * tangents[open_forward],
            points[open_backward] - step * tangents[open_backward],
        ]
    )
    source = np.concatenate([open_forward, open_backward])
    existing = points + offsets[:, None] * normals
    candidate_targets = proposed + params.cage_offset * normals[source]
    distance = np.min(
        np.linalg.norm(candidate_targets[:, None, :] - existing[None, :, :], axis=2), axis=1
    )
    # The source's own target sits a full step away, so this keeps a continuation
    # only when no existing ring target is nearer than that -- which is exactly
    # the statement that the step leaves the ring the map already implies.
    keep = distance >= 0.9 * step
    return proposed[keep], normals[source[keep]], source[keep]


def _gap_weights(targets: np.ndarray, robot_positions: np.ndarray, gap_radius: float) -> np.ndarray:
    """Uncovered-gap term: 1 far from every robot, 0 where a robot already sits."""
    positions = np.asarray(robot_positions, dtype=float).reshape(-1, 2)
    if len(positions) == 0:
        return np.ones(len(targets))
    d = np.min(np.linalg.norm(targets[:, None, :] - positions[None, :, :], axis=2), axis=1)
    return 1.0 - np.exp(-(d / max(gap_radius, _EPS)) ** 2)


__all__ = ["BoundaryAwareDensity", "DensityParams"]
