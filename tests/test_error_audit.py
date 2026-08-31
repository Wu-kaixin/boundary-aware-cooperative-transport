"""T2 - the six-term error audit, and the isolation that makes it an audit.

``configs/sim/v2/shape_matrix.yaml`` declares ``normal_error_deg: 30.0`` and
``velocity_error: 0.02`` with a comment reading DECLARED, NOT YET VERIFIED. These
tests are what the verification rests on, so they check three separate things:

* the six terms are each the quantity they are named after;
* the audit withholds the conditional-guarantee label when a declared bound is
  breached, rather than reporting the breach and carrying on;
* no control module can reach the audit, by import graph.

The third is the one that keeps the other two honest. An audit that the controller
could read would be a sensor, and a truth-reading sensor is the thing this whole
branch is built to avoid.
"""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.error_audit import ERROR_TERMS, ErrorAudit, Running, frame_errors
from dbact.geometry import sample_polygon_boundary, signed_distance_and_gradient

SQUARE = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])


def square_cargo(**kwargs) -> Cargo:
    return Cargo("obj", SQUARE, **kwargs)


def perfect_map(cargo: Cargo, count: int = 51):
    """Map cells exactly on the true boundary, with the true outward normals.

    The count must be odd. Samples sit at arclength ``4k/n`` on a unit square of
    perimeter 4, whose corners are at 1, 2 and 3; an even ``n`` always places a sample
    at ``n/2`` -> arclength 2, the far corner, and a multiple of four hits all of them.
    Only an odd ``n`` misses every corner, because ``n/4``, ``n/2`` and ``3n/4`` are
    then all non-integers.

    It matters because at a corner the outward normal does not exist. The two incident
    edges disagree by the exterior angle, ``sample_polygon_boundary`` returns the
    following edge's normal, and ``signed_distance_and_gradient`` returns whichever
    incident edge the closest-point search picked -- 90 degrees apart on a square.
    A calibration case built on an even count reports a 90 degree "perception error"
    that no perception was involved in. See
    :func:`test_corner_samples_carry_an_irreducible_normal_ambiguity`.
    """
    assert count % 2 == 1, "an even sample count lands on the square's far corner"
    points, normals = sample_polygon_boundary(cargo.vertices, count=count)
    return points, normals


# --------------------------------------------------------------------------- #
# the terms
# --------------------------------------------------------------------------- #


def test_a_perfect_map_of_a_stationary_object_has_zero_error_everywhere():
    """The calibration case. Any non-zero term here is the audit's own noise."""
    cargo = square_cargo()
    points, normals = perfect_map(cargo)
    errors = frame_errors(cargo, points, normals, np.zeros(2))

    for name, values in errors.items():
        assert np.max(values) == pytest.approx(0.0, abs=1e-9), name


def test_corner_samples_carry_an_irreducible_normal_ambiguity():
    """A measurement caveat, not a bug, and it inflates the measured normal bound.

    At a convex corner the outward normal is not defined -- the two incident edges
    disagree by the exterior angle, 90 degrees on a square. A map cell sitting on a
    corner therefore reports a large ``normal_error_deg`` with no perception error
    present at all, and the audit has no way to tell that apart from a genuinely bad
    return.

    Recorded rather than corrected. The perception layer's confidence is a plane-fit
    residual precisely so that corner neighbourhoods are down-weighted, so the effect
    on a real map is bounded by that gate rather than by anything here; but any
    ``normal_error_deg`` maximum this audit reports on a polygonal object should be
    read as including the corner cells. It is one more reason the ISSf statement is
    made over the *projection* term, which is continuous across a corner.
    """
    cargo = square_cargo()
    on_corners, corner_normals = sample_polygon_boundary(cargo.vertices, count=8)
    errors = frame_errors(cargo, on_corners, corner_normals, np.zeros(2))
    assert np.max(errors["normal_error_deg"]) == pytest.approx(90.0)
    # Lifting off the corner by a micron resolves it: the ambiguity is exactly at the
    # measure-zero set, which is why a non-corner-aligned sample count avoids it.
    lifted = on_corners + 1e-6 * corner_normals
    assert np.max(frame_errors(cargo, lifted, corner_normals, np.zeros(2))["normal_error_deg"]) < 1e-3


def test_normal_error_is_the_angle_to_the_true_outward_normal():
    cargo = square_cargo()
    points, normals = perfect_map(cargo, count=31)
    # Tilt every normal by a known angle in its own tangent plane.
    tilt = np.radians(12.0)
    rotated = np.column_stack(
        [
            normals[:, 0] * np.cos(tilt) - normals[:, 1] * np.sin(tilt),
            normals[:, 0] * np.sin(tilt) + normals[:, 1] * np.cos(tilt),
        ]
    )
    errors = frame_errors(cargo, points, rotated, np.zeros(2))
    assert errors["normal_error_deg"] == pytest.approx(np.full(len(points), 12.0))


def test_boundary_point_error_is_unsigned():
    """A cell 20 mm inside and one 20 mm outside are equally wrong about the surface.

    Signing it would let a map that straddles the boundary average to zero error.
    """
    cargo = square_cargo()
    # Explicit face midpoints rather than an arclength sampling. Sample 0 of
    # ``sample_polygon_boundary`` is always the polygon's first vertex, and stepping
    # 20 mm inward along one edge's normal from a corner slides *along* the adjacent
    # edge -- distance to the boundary 0, not 0.02. That is a property of corners, not
    # of this term, and it is tested on its own above.
    points = np.array([[0.0, -0.5], [0.5, 0.0], [0.0, 0.5], [-0.5, 0.0]])
    normals = np.array([[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]])

    out = frame_errors(cargo, points + 0.02 * normals, normals, np.zeros(2))
    into = frame_errors(cargo, points - 0.02 * normals, normals, np.zeros(2))
    assert out["boundary_point_error_m"] == pytest.approx(np.full(4, 0.02), abs=1e-9)
    assert into["boundary_point_error_m"] == pytest.approx(np.full(4, 0.02), abs=1e-9)


def test_object_velocity_error_is_the_translational_term_only():
    cargo = square_cargo()
    cargo.set_twist(np.array([0.10, 0.0]), 0.0)
    points, normals = perfect_map(cargo, count=19)

    errors = frame_errors(cargo, points, normals, np.array([0.06, 0.0]))
    assert errors["object_velocity_error_mps"] == pytest.approx(np.full(len(points), 0.04))
    # With no rotation anywhere, the point-velocity error equals the body one.
    assert errors["point_velocity_error_mps"] == pytest.approx(np.full(len(points), 0.04))


def test_point_velocity_error_exposes_unmodelled_rotation():
    """The term the whole SE(2) item exists for.

    A robot that estimates the translation perfectly but believes the object is not
    rotating is wrong about the boundary-point velocity by ``omega |b_k - c|``, and the
    error grows with the lever arm -- largest on the widest part of the object, which is
    where the pushing robots stand. The body-velocity term reports zero for exactly the
    same run, which is why five terms were not enough.
    """
    cargo = square_cargo()
    omega = 0.5
    cargo.set_twist(np.zeros(2), omega)
    points, normals = perfect_map(cargo, count=63)

    blind = frame_errors(cargo, points, normals, np.zeros(2), 0.0)
    assert np.max(blind["object_velocity_error_mps"]) == pytest.approx(0.0, abs=1e-12)
    lever = np.linalg.norm(points - cargo.position[None, :], axis=1)
    assert blind["point_velocity_error_mps"] == pytest.approx(omega * lever, abs=1e-9)

    # Knowing the rotation removes it. The reference has to be the *same* point the
    # truth is expressed about; a uniform arclength sampling of a square has a
    # centroid slightly off centre, so this passes ``cargo.position`` to isolate the
    # velocity term from the reference-point error, which is measured separately.
    aware = frame_errors(cargo, points, normals, np.zeros(2), omega, cargo.position)
    assert np.max(aware["point_velocity_error_mps"]) < 1e-9


def test_projection_error_is_not_implied_by_the_other_terms():
    """A normal error and a velocity error can cancel in the projection, or compound.

    This is why the sixth term is reported separately: it is the only one whose
    numerical value the barrier row sees, so it is the only one an ISSf constant can
    honestly be stated over. Here the velocity error is entirely tangential to the
    surface, so the row does not see it at all -- and a report that stated the bound
    over ``point_velocity_error_mps`` would have failed a run the constraint never
    noticed.
    """
    cargo = square_cargo()
    cargo.set_twist(np.array([0.0, 0.20]), 0.0)
    # One cell on the +x face: its true normal is +x, and the object's motion is +y.
    point = np.array([[0.5, 0.0]])
    normal = np.array([[1.0, 0.0]])
    errors = frame_errors(cargo, point, normal, np.zeros(2))

    assert errors["point_velocity_error_mps"][0] == pytest.approx(0.20)
    # Tangential, therefore invisible to the row.
    assert errors["normal_projection_error_mps"][0] == pytest.approx(0.0, abs=1e-12)


def test_truth_is_evaluated_at_the_footpoint_and_position_error_couples_in():
    """Truth is read at the footpoint, and the coupling that leaves is real.

    A cell floating off the surface has no material point of its own, so the true
    velocity has to be evaluated at its footpoint -- the piece of surface the row is
    actually about. What that does *not* do is decouple the terms, and an earlier draft
    of this test wrongly asserted that it did.

    On a rotating object a robot whose twist estimate is exact still computes the
    velocity at the cell it believes the boundary occupies, so its estimate differs from
    the truth at the footpoint by exactly ``omega * |cell - footpoint|``. That is the
    position error propagated through the rigid velocity field. It is genuine physics
    rather than double counting: a robot wrong about *where* the surface is on a
    rotating body is necessarily wrong about how fast that surface is moving.

    Recording it matters for reading the audit. ``boundary_point_error_m`` and
    ``point_velocity_error_mps`` are not independent on a rotating object, and the
    coupling constant is ``omega``.
    """
    cargo = square_cargo()
    omega, lift = 0.4, 0.05
    cargo.set_twist(np.zeros(2), omega)
    points, normals = perfect_map(cargo, count=19)
    lifted = points + lift * normals

    _, _, footpoints = signed_distance_and_gradient(lifted, cargo.vertices)
    assert np.allclose(footpoints, points, atol=1e-9), "the footpoint is the original cell"

    aware = frame_errors(cargo, lifted, normals, np.zeros(2), omega, cargo.position)
    assert aware["boundary_point_error_m"] == pytest.approx(np.full(len(points), lift), abs=1e-9)
    assert aware["point_velocity_error_mps"] == pytest.approx(
        np.full(len(points), omega * lift), abs=1e-9
    )

    # And with no rotation the two terms *are* independent: the same 50 mm position
    # error produces no velocity error at all.
    cargo.set_twist(np.array([0.07, 0.0]), 0.0)
    still = frame_errors(cargo, lifted, normals, np.array([0.07, 0.0]), 0.0, cargo.position)
    assert np.max(still["point_velocity_error_mps"]) < 1e-12
    assert still["boundary_point_error_m"] == pytest.approx(np.full(len(points), lift), abs=1e-9)


def test_frame_errors_on_an_empty_map_returns_empty_arrays():
    cargo = square_cargo()
    errors = frame_errors(cargo, np.empty((0, 2)), np.empty((0, 2)), np.zeros(2))
    assert all(len(values) == 0 for values in errors.values())


# --------------------------------------------------------------------------- #
# accumulation
# --------------------------------------------------------------------------- #


def test_running_keeps_a_max_and_a_mean_and_ignores_non_finite_values():
    running = Running()
    assert running.as_dict() == {"n": 0, "mean": None, "max": None}
    running.add(np.array([1.0, 3.0, np.inf, np.nan]))
    stats = running.as_dict()
    assert (stats["n"], stats["mean"], stats["max"]) == (2, 2.0, 3.0)
    # No bound was supplied, so the breach fields are ``None`` rather than zero: a zero
    # would read as "nothing exceeded a bound" when no bound was ever set.
    assert stats["breach_fraction"] is None and stats["breaches"] is None


def test_running_counts_breaches_only_against_a_supplied_bound():
    running = Running()
    running.add(np.array([1.0, 5.0, 9.0, 11.0]), bound=8.0)
    stats = running.as_dict()
    assert stats["breaches"] == 2
    assert stats["breach_fraction"] == pytest.approx(0.5)


def test_running_quantiles_are_accurate_from_the_reservoir():
    """The statistic that distinguishes a wrong premise from a pathological tail.

    A maximum alone cannot tell "the 30 degree premise is wrong" from "twelve cells out
    of eleven million are wrong", and those have different consequences. The reservoir
    is a uniform sample, so its quantiles track the population's; the max and mean are
    accumulated exactly and do not depend on it.
    """
    rng = np.random.default_rng(7)
    population = rng.uniform(0.0, 1.0, 500_000)
    running = Running(reservoir_limit=20000)
    for chunk in np.array_split(population, 250):
        running.add(chunk)

    stats = running.as_dict()
    assert stats["n"] == len(population)
    assert stats["max"] == pytest.approx(population.max())
    assert stats["mean"] == pytest.approx(population.mean(), rel=1e-9)
    assert stats["sampled"] == 20000
    for key, q in (("p50", 0.50), ("p95", 0.95), ("p99", 0.99)):
        assert stats[key] == pytest.approx(float(np.quantile(population, q)), abs=0.02), key


def test_a_thin_tail_and_a_wrong_premise_are_distinguishable():
    """One pathological value in a hundred thousand must not read like a bad premise."""
    thin = Running()
    values = np.full(100_000, 1.0)
    values[0] = 500.0
    thin.add(values, bound=30.0)
    stats = thin.as_dict()
    assert stats["max"] == 500.0
    assert stats["breach_fraction"] == pytest.approx(1e-5)
    assert stats["p999"] == pytest.approx(1.0)

    wrong = Running()
    wrong.add(np.full(100_000, 45.0), bound=30.0)
    assert wrong.as_dict()["breach_fraction"] == pytest.approx(1.0)


def test_audit_reports_all_six_terms():
    """Named once in ``ERROR_TERMS``, so a report cannot silently carry five."""
    assert len(ERROR_TERMS) == 6
    cargo = square_cargo()
    points, normals = perfect_map(cargo, count=19)
    audit = ErrorAudit()
    audit.observe(cargo, points, normals, np.zeros(2))
    audit.observe_map_gap(cargo, points)
    assert set(audit.verdict()["terms"]) == set(ERROR_TERMS)
    assert audit.verdict()["terms"]["map_gap_m"]["n"] == 1


# --------------------------------------------------------------------------- #
# fail-closed
# --------------------------------------------------------------------------- #


def test_undeclared_bounds_give_no_verdict_rather_than_a_pass():
    """``None``, not ``True``.

    There is nothing to be within, and returning ``True`` would let a config that
    declared no premise appear to have satisfied one.
    """
    cargo = square_cargo()
    points, normals = perfect_map(cargo, count=11)
    audit = ErrorAudit()
    audit.observe(cargo, points, normals, np.zeros(2))
    verdict = audit.verdict()

    assert verdict["within_declared_bounds"] is None
    assert verdict["fail_closed_reasons"] == []
    assert verdict["declared_bounds"] == {"normal_error_deg": None, "velocity_error": None}


def test_a_run_within_its_declared_bounds_passes():
    cargo = square_cargo()
    points, normals = perfect_map(cargo, count=19)
    audit = ErrorAudit(declared_normal_error_deg=30.0, declared_velocity_error=0.02)
    audit.observe(cargo, points, normals, np.zeros(2))

    verdict = audit.verdict()
    assert verdict["within_declared_bounds"] is True
    assert verdict["breach_frames"] == {"normal_error_deg": 0, "velocity_error": 0}


def test_a_breached_normal_bound_withholds_the_label():
    """The premise is measured *and* enforced. Measuring it and carrying on is worse.

    The certificate's geometric predicates can all hold on a run whose perception was
    outside the declared error budget, and such a run is not entitled to the
    conditional-guarantee claim -- the claim is conditional on that budget.
    """
    cargo = square_cargo()
    points, normals = perfect_map(cargo, count=19)
    tilt = np.radians(45.0)
    wrong = np.column_stack(
        [
            normals[:, 0] * np.cos(tilt) - normals[:, 1] * np.sin(tilt),
            normals[:, 0] * np.sin(tilt) + normals[:, 1] * np.cos(tilt),
        ]
    )
    audit = ErrorAudit(declared_normal_error_deg=30.0, declared_velocity_error=0.02)
    audit.observe(cargo, points, wrong, np.zeros(2))

    verdict = audit.verdict()
    assert verdict["within_declared_bounds"] is False
    assert verdict["fail_closed_reasons"] == ["normal_error_deg"]
    assert verdict["breach_frames"]["normal_error_deg"] == 1
    assert verdict["measured_bounds"]["normal_error_deg"] == pytest.approx(45.0)


def test_a_breached_velocity_bound_is_checked_on_the_projection():
    """The declared velocity bound is about the term the constraint sees.

    Checking the vector error instead would fail a run whose error was entirely
    tangential, and therefore entirely invisible to the barrier row it is supposed to
    be a premise about.
    """
    cargo = square_cargo()
    cargo.set_twist(np.array([0.30, 0.0]), 0.0)
    points, normals = perfect_map(cargo, count=19)

    breached = ErrorAudit(declared_velocity_error=0.02)
    breached.observe(cargo, points, normals, np.zeros(2))
    assert breached.verdict()["fail_closed_reasons"] == ["velocity_error"]

    # The same magnitude of error, entirely tangential to the one face observed.
    cargo.set_twist(np.array([0.0, 0.30]), 0.0)
    tangential = ErrorAudit(declared_velocity_error=0.02)
    tangential.observe(cargo, np.array([[0.5, 0.0]]), np.array([[1.0, 0.0]]), np.zeros(2))
    assert tangential.verdict()["fail_closed_reasons"] == []


def test_measured_bounds_are_what_the_config_would_have_to_declare():
    """The field that turns DECLARED, NOT YET VERIFIED into a number."""
    cargo = square_cargo()
    cargo.set_twist(np.array([0.10, 0.0]), 0.0)
    points, normals = perfect_map(cargo, count=19)
    audit = ErrorAudit(declared_normal_error_deg=30.0, declared_velocity_error=0.02)
    audit.observe(cargo, points, normals, np.zeros(2))

    measured = audit.verdict()["measured_bounds"]
    assert measured["normal_error_deg"] == pytest.approx(0.0, abs=1e-9)
    # The worst projection of a 0.10 m/s unmodelled translation is 0.10 m/s, on the
    # face whose normal is parallel to the motion.
    assert measured["velocity_error"] == pytest.approx(0.10, abs=1e-9)


# --------------------------------------------------------------------------- #
# isolation
# --------------------------------------------------------------------------- #

#: Modules that run inside the control loop. Each one produces or shapes a command,
#: so none of them may reach a module that reads the true cargo state.
CONTROL_PATH = (
    "controller",
    "safety_filter",
    "boundary_map",
    "boundary_density",
    "local_cvt",
    "qp2d",
    "transport_control",
    "agent_control",
    "enclosure_gate",
    "phase",
    "task",
    "contracts",
)

#: Modules that are allowed to read truth, because none of them is in the loop.
AUDIT_ONLY = ("error_audit", "metrics", "guarantees", "diagnosis")


def imported_dbact_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            # Relative ``from .x import y`` has module == "x" at level 1.
            found.add(node.module.split(".")[-1])
        elif isinstance(node, ast.Import):
            for alias in node.names:
                found.add(alias.name.split(".")[-1])
    return found


def test_no_control_module_imports_an_audit_module():
    """Truth isolation, by import graph rather than by convention.

    ``dbact.error_audit`` takes the true :class:`Cargo`. If any module in the control
    path could import it, the controller would have a path to the simulator's state and
    every result on this branch would be unfalsifiable. Checked statically, at every
    import depth, because a single ``from .error_audit import`` inside a function body
    would be enough and would never show up in a run that happened to succeed.
    """
    root = Path(__file__).resolve().parents[1] / "src" / "dbact"
    offences = []
    for name in CONTROL_PATH:
        module = root / f"{name}.py"
        assert module.exists(), name
        for imported in imported_dbact_modules(module) & set(AUDIT_ONLY):
            offences.append(f"{name} imports {imported}")
    assert offences == [], offences


def test_the_audit_modules_do_read_truth_which_is_why_they_are_separated():
    """The complement of the check above, so it cannot pass by being vacuous.

    If ``error_audit`` stopped importing ``Cargo`` the isolation test would still pass
    while testing nothing. This pins the reason the separation exists.
    """
    root = Path(__file__).resolve().parents[1] / "src" / "dbact"
    assert "cargo" in imported_dbact_modules(root / "error_audit.py")
    assert "cargo" in imported_dbact_modules(root / "metrics.py")


def test_the_controller_never_reads_a_cargo_pose_or_velocity():
    """The narrower statement, on the attribute names rather than the module graph.

    ``controller.py`` and ``safety_filter.py`` receive ``Cargo`` objects -- the
    controller is handed the list so it can pass them to the *sensor* -- so an import
    check alone is not enough. These are the attributes that would leak a pose.
    """
    root = Path(__file__).resolve().parents[1] / "src" / "dbact"
    forbidden = (".center", ".linear_velocity", ".angular_velocity", ".local_vertices", ".angle")
    for name in ("controller", "safety_filter", "boundary_map", "transport_control"):
        source = (root / f"{name}.py").read_text(encoding="utf-8")
        # Strip the ``cargo.`` qualified forms only: an agent's own ``.angle`` or a
        # local variable named ``center`` is not a truth read.
        for attribute in forbidden:
            assert f"cargo{attribute}" not in source, f"{name} reads cargo{attribute}"
            assert f"c{attribute}" not in source.replace("self", ""), f"{name} may read c{attribute}"
