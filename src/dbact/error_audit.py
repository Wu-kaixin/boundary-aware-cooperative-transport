"""T2 - the six error terms the object-boundary barrier actually depends on.

``configs/sim/v2/shape_matrix.yaml`` declares two error premises,
``guarantee.bounded_errors.normal_error_deg`` and ``velocity_error``, with a comment
saying in as many words: DECLARED, NOT YET VERIFIED. The certificate in
:mod:`dbact.guarantees` is conditional on them, and nothing measured them. This
module measures them, and four more terms that the same barrier row depends on and
that nobody had written down at all.

Why six and not two
-------------------
The object row is

    n_k^T u_i  >=  n_k^T v_{b_k}  -  gamma_obj ( n_k^T (p_i - b_k) - r_safe )  +  rho

Every symbol in it except ``u_i``, ``p_i`` and the constants is an estimate, and the
estimates enter in different places with different consequences:

1. ``normal_error_deg``      the angle between the stored normal and the true
   outward normal at that piece of surface. It tilts the half-plane, and the
   tangential window ``W`` is what bounds the resulting plane displacement to
   ``W sin(eps_n)``.
2. ``boundary_point_error_m``  how far a stored cell is from the true boundary. It
   translates the half-plane along its own normal, directly offsetting ``h_k``.
3. ``map_gap_m``             the one-sided Hausdorff distance from the true
   boundary to the map: not an error in any single row, but the size of the piece of
   boundary that has no row at all. A notch nobody has observed cannot be
   constrained however accurate the observed cells are.
4. ``object_velocity_error_mps``   error in the body's translational velocity
   estimate. This is the term v1 had.
5. ``point_velocity_error_mps``    error in the *material point* velocity at each
   cell, ``v_c + omega R90 (b_k - c)``. On a rotating object this is the term that
   differs from 4 by ``omega |b_k - c|``, and it is the one the barrier needs.
6. ``normal_projection_error_mps`` ``| n_hat^T v_hat  -  n_true^T v_true |``, the
   scalar that is literally the row's right-hand side. It is not implied by 1, 4 and
   5: a normal error and a velocity error can cancel in the projection, or compound.
   It is reported separately because it is the only one of the six whose numerical
   value the constraint sees, and therefore the only one an ISSf constant can honestly
   be stated over.

The terms are not independent, and one coupling is worth naming
--------------------------------------------------------------
On a rotating object terms 2 and 5 are linked by ``omega``. A robot whose twist
estimate is exact still evaluates the velocity field at the cell it *believes* the
boundary occupies, so a cell displaced by ``d`` from the true surface yields a
boundary-point velocity wrong by ``omega d`` even with a perfect twist. That is real
physics rather than double counting -- being wrong about where a rotating surface is
entails being wrong about how fast it is moving -- but it means the two maxima must not
be read as separate error budgets. With ``omega = 0`` the coupling vanishes and the two
terms are independent, which is the regime the baseline 12-seed sweep runs in.

Corner cells inflate term 1 irreducibly. At a convex vertex the outward normal does not
exist: the incident edges disagree by the exterior angle, 90 degrees on a rectangle. A
cell sitting on a vertex therefore reports a large normal error with no perception error
present. The perception layer's plane-fit-residual confidence already down-weights corner
neighbourhoods, so the effect on a real map is bounded by that gate; but a reported
``normal_error_deg`` maximum on a polygonal object should be read as including it. It is
one more reason the declared *velocity* premise is checked against term 6, which is
continuous across a corner, rather than against term 5.

Measurement only, and fail-closed
---------------------------------
Nothing here is fed back. Every function takes the true :class:`~dbact.cargo.Cargo`
and is therefore forbidden from the control path -- ``tests/test_error_audit.py``
asserts by import graph that no control module reaches this one. What the audit does
have authority to do is *withhold a label*: :meth:`ErrorAudit.verdict` reports
``within_declared_bounds = False`` when a declared premise was breached, and the
episode is then not entitled to the conditional-guarantee claim even though every
geometric predicate held. A premise that is measured and then ignored when it fails
is not a premise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .boundary_map import rot90
from .cargo import Cargo
from .geometry import signed_distance_and_gradient
from .guarantees import boundary_map_gap_upper_bound

#: The six term names, in the order the module docstring introduces them. Written
#: once so that a report cannot silently carry five.
ERROR_TERMS = (
    "normal_error_deg",
    "boundary_point_error_m",
    "map_gap_m",
    "object_velocity_error_mps",
    "point_velocity_error_mps",
    "normal_projection_error_mps",
)


@dataclass
class Running:
    """Max, mean, quantiles and a breach fraction for one term.

    The quantiles are not decoration. The first run of this audit reported a
    ``normal_error_deg`` maximum of exactly 180 degrees against a declared premise of
    30, and a maximum alone cannot distinguish "the premise is wrong" from "a handful of
    pathological cells out of several million are wrong" -- which have different
    consequences and different fixes. ``breach_fraction`` is the number that settles it.

    Quantiles come from a fixed-size reservoir rather than the full sample: at 16 robots
    and 600 cells over 3000 frames a term has tens of millions of values, which would
    cost more memory than the simulation. A uniform reservoir of 20000 gives quantiles
    good to well under a percent, and the max and mean are exact because they are
    accumulated separately.
    """

    count: int = 0
    total: float = 0.0
    worst: float = 0.0
    #: Samples exceeding ``bound``, when one was supplied to :meth:`add`.
    breaches: int = 0
    bounded: int = 0
    reservoir_limit: int = 20000
    _reservoir: list = field(default_factory=list)
    _rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(20260812))

    def add(self, values: np.ndarray, bound: float | None = None) -> None:
        v = np.asarray(values, dtype=float).reshape(-1)
        v = v[np.isfinite(v)]
        if len(v) == 0:
            return
        self.count += len(v)
        self.total += float(np.sum(v))
        self.worst = max(self.worst, float(np.max(v)))
        if bound is not None:
            self.bounded += len(v)
            self.breaches += int(np.count_nonzero(v > bound))

        # Reservoir sampling: fill, then replace with probability limit/count so that
        # every value seen has an equal chance of being held.
        room = self.reservoir_limit - len(self._reservoir)
        if room > 0:
            take = v[:room]
            self._reservoir.extend(float(x) for x in take)
            v = v[room:]
        if len(v) and self._reservoir:
            keep = self._rng.random(len(v)) < (self.reservoir_limit / self.count)
            if np.any(keep):
                chosen = v[keep]
                slots = self._rng.integers(0, self.reservoir_limit, len(chosen))
                for slot, value in zip(slots, chosen):
                    self._reservoir[int(slot)] = float(value)

    def as_dict(self) -> dict:
        if not self.count:
            return {"n": 0, "mean": None, "max": None}
        sample = np.asarray(self._reservoir, dtype=float)
        return {
            "n": self.count,
            "mean": self.total / self.count,
            "max": self.worst,
            "p50": float(np.quantile(sample, 0.50)),
            "p95": float(np.quantile(sample, 0.95)),
            "p99": float(np.quantile(sample, 0.99)),
            "p999": float(np.quantile(sample, 0.999)),
            "sampled": len(sample),
            # ``None`` when no bound was declared for this term, so that a zero cannot
            # be read as "nothing breached a bound that was never set".
            "breach_fraction": (self.breaches / self.bounded) if self.bounded else None,
            "breaches": self.breaches if self.bounded else None,
        }


def frame_errors(
    cargo: Cargo,
    map_points: np.ndarray,
    map_normals: np.ndarray,
    estimated_velocity: np.ndarray,
    estimated_angular_velocity: float = 0.0,
    reference_point: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Per-cell errors for one robot's map of one object at one instant.

    ``estimated_velocity`` and ``estimated_angular_velocity`` are the robot's own
    twist estimate about ``reference_point``, which is the *map's* centroid rather
    than the object's true centre -- so this compares what the robot believes against
    what is true, not two truths against each other.
    """
    points = np.asarray(map_points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return {name: np.empty(0) for name in ERROR_TERMS if name != "map_gap_m"}
    normals = np.asarray(map_normals, dtype=float).reshape(-1, 2)

    # Truth at each stored cell: the signed distance carries the outward normal of
    # the true surface and the footpoint on it.
    signed, true_normals, footpoints = signed_distance_and_gradient(points, cargo.vertices)

    normal_cosine = np.clip(np.einsum("ij,ij->i", normals, true_normals), -1.0, 1.0)
    normal_error_deg = np.degrees(np.arccos(normal_cosine))
    # Distance to the true boundary, unsigned: a cell 0.02 m inside and one 0.02 m
    # outside are equally wrong about where the surface is.
    boundary_point_error = np.abs(signed)

    v_hat = np.asarray(estimated_velocity, dtype=float).reshape(2)
    omega_hat = float(estimated_angular_velocity)
    estimated_points = np.repeat(v_hat[None, :], len(points), axis=0)
    if omega_hat != 0.0 and reference_point is not None:
        reference = np.asarray(reference_point, dtype=float).reshape(2)
        estimated_points = estimated_points + omega_hat * rot90(points - reference[None, :])

    # Truth is evaluated at the *footpoint*, not at the stored cell. The material
    # point the row is about is on the object; a cell floating 0.02 m off the surface
    # has no material point of its own, and evaluating the rigid velocity field there
    # would fold the boundary-point error back into the velocity error and report the
    # same mistake twice.
    true_linear = np.asarray(cargo.linear_velocity, dtype=float).reshape(2)
    true_points = true_linear[None, :] + cargo.angular_velocity * rot90(
        footpoints - cargo.position[None, :]
    )

    object_velocity_error = np.full(len(points), float(np.linalg.norm(v_hat - true_linear)))
    point_velocity_error = np.linalg.norm(estimated_points - true_points, axis=1)
    projection_error = np.abs(
        np.einsum("ij,ij->i", normals, estimated_points)
        - np.einsum("ij,ij->i", true_normals, true_points)
    )
    return {
        "normal_error_deg": normal_error_deg,
        "boundary_point_error_m": boundary_point_error,
        "object_velocity_error_mps": object_velocity_error,
        "point_velocity_error_mps": point_velocity_error,
        "normal_projection_error_mps": projection_error,
    }


@dataclass
class ErrorAudit:
    """Accumulator over frames, robots and objects, with the declared bounds.

    ``declared_normal_error_deg`` and ``declared_velocity_error`` come from the
    scenario's ``guarantee.bounded_errors`` block. They are ``None`` when the scenario
    declared none, and the verdict then says so rather than inventing a bound --
    the same rule :func:`dbact.guarantees._required` follows.
    """

    declared_normal_error_deg: float | None = None
    declared_velocity_error: float | None = None
    terms: dict[str, Running] = field(default_factory=lambda: {n: Running() for n in ERROR_TERMS})
    frames: int = 0
    #: Frames on which some robot's estimate breached a declared bound. Counted per
    #: frame rather than per cell, because one bad cell out of six hundred and six
    #: hundred bad cells are different facts about the run.
    normal_breach_frames: int = 0
    velocity_breach_frames: int = 0

    def observe(
        self,
        cargo: Cargo,
        map_points: np.ndarray,
        map_normals: np.ndarray,
        estimated_velocity: np.ndarray,
        estimated_angular_velocity: float = 0.0,
        reference_point: np.ndarray | None = None,
    ) -> None:
        errors = frame_errors(
            cargo,
            map_points,
            map_normals,
            estimated_velocity,
            estimated_angular_velocity,
            reference_point,
        )
        bounds = {
            "normal_error_deg": self.declared_normal_error_deg,
            "normal_projection_error_mps": self.declared_velocity_error,
        }
        for name, values in errors.items():
            self.terms[name].add(values, bounds.get(name))
        if len(np.asarray(map_points).reshape(-1, 2)) == 0:
            return
        if self.declared_normal_error_deg is not None:
            if float(np.max(errors["normal_error_deg"])) > self.declared_normal_error_deg:
                self.normal_breach_frames += 1
        if self.declared_velocity_error is not None:
            # The projection is the term the constraint sees, so it is the term the
            # declared velocity bound is checked against. Checking the vector error
            # instead would fail runs whose error was entirely tangential and
            # therefore invisible to the barrier.
            if float(np.max(errors["normal_projection_error_mps"])) > self.declared_velocity_error:
                self.velocity_breach_frames += 1

    def observe_map_gap(self, cargo: Cargo, map_points: np.ndarray, sample_count: int = 512) -> None:
        """The one term that is a property of the whole map rather than of a cell."""
        gap = boundary_map_gap_upper_bound(cargo.vertices, map_points, sample_count=sample_count)
        value = gap["max_boundary_gap"]
        if math.isfinite(value):
            self.terms["map_gap_m"].add(np.array([value]))

    def verdict(self) -> dict:
        """The report, with the fail-closed judgement separated from the numbers.

        ``within_declared_bounds`` is ``None`` -- not ``True`` -- when the scenario
        declared no bounds. There is nothing to be within, and returning ``True``
        would let a config that declared nothing appear to have passed something.
        """
        declared = {
            "normal_error_deg": self.declared_normal_error_deg,
            "velocity_error": self.declared_velocity_error,
        }
        breaches = {
            "normal_error_deg": self.normal_breach_frames,
            "velocity_error": self.velocity_breach_frames,
        }
        checked = [name for name, bound in declared.items() if bound is not None]
        within = None if not checked else all(breaches[name] == 0 for name in checked)
        return {
            "audited_frames": self.frames,
            "declared_bounds": declared,
            "breach_frames": breaches,
            "within_declared_bounds": within,
            "fail_closed_reasons": [name for name in checked if breaches[name] > 0],
            "terms": {name: running.as_dict() for name, running in self.terms.items()},
            # The measured maxima, in the form the guarantee block would have to
            # declare to be honest about this run. This is what turns
            # "DECLARED, NOT YET VERIFIED" into a number.
            "measured_bounds": {
                "normal_error_deg": self.terms["normal_error_deg"].worst,
                "velocity_error": self.terms["normal_projection_error_mps"].worst,
            },
            # What share of measured cells actually exceeded each declared premise. This
            # is the figure that distinguishes a premise that is wrong from a premise
            # violated by a thin tail of pathological cells, and a report that gave only
            # the maximum could not tell the two apart.
            "breach_fractions": {
                "normal_error_deg": self.terms["normal_error_deg"].as_dict()["breach_fraction"],
                "velocity_error": self.terms["normal_projection_error_mps"].as_dict()[
                    "breach_fraction"
                ],
            },
            # The premise that *would* hold for 99.9% of cells, offered beside the
            # maximum rather than instead of it. A bound is about the worst case; a
            # 99.9th percentile is not a bound, and it is labelled so.
            "p999_not_a_bound": {
                "normal_error_deg": self.terms["normal_error_deg"].as_dict().get("p999"),
                "velocity_error": self.terms["normal_projection_error_mps"].as_dict().get("p999"),
            },
        }


__all__ = ["ERROR_TERMS", "ErrorAudit", "Running", "frame_errors"]
