"""The harness that produced the 180-episode decisive matrix.

Ported from the CODEX branch's ``tests/test_monte_carlo_runner.py``. Two of its
four tests do not survive the port and are replaced rather than dropped:

* CODEX had ``shape_config(name, rng)`` returning a cargo config dict. v1 has
  ``local_outline(name, rng)`` returning a body-frame outline that
  ``build_case_config`` then scales, rotates and places. The premise being tested
  -- every catalogued family materialises as a simple polygon -- is unchanged, so
  it is re-stated over v1's two functions.

* CODEX had ``empirical_completion_bound``. v1 has no such function, deliberately:
  an empirical completion-time bound over a set of episodes that contains eligible
  failures is right-censored, and v1's answer to that was to not compute one at
  all rather than to compute one and label it. What is testable here is the
  absence -- see
  :func:`test_summary_reports_completion_time_without_claiming_a_bound`.

These tests must not run episodes. The matrix took 180 of them; what is checked
here is the case construction, the classification order and the interval
arithmetic, all of which are pure functions of their inputs.
"""

from __future__ import annotations

import copy

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.geometry import is_simple_polygon, polygon_diameter
from dbact_sim.scenarios import load_yaml

MATRIX_CONFIG = "configs/sim/v2/shape_matrix.yaml"


# --------------------------------------------------------------------------- #
# argument parsing
# --------------------------------------------------------------------------- #


def test_seed_range_parser_is_inclusive_and_deduplicated(matrix_runner):
    assert matrix_runner.parse_seeds("0..2,2,5") == [0, 1, 2, 5]


def test_seed_range_parser_accepts_a_descending_range(matrix_runner):
    """``11..0`` and ``0..11`` must name the same 12 seeds.

    The acceptance gate is written ``--seeds 0..11``. A parser that returned an
    empty list for the reversed form would silently run zero episodes and report a
    clean sweep.
    """
    assert matrix_runner.parse_seeds("11..0") == list(range(12))
    assert matrix_runner.parse_seeds("0..11") == list(range(12))
    assert matrix_runner.parse_seeds(" 3 , , 1 ") == [1, 3]


def test_alpha_parser_reads_the_matrix_levels(matrix_runner):
    assert matrix_runner.parse_alphas("0.1,0.4,0.8") == [0.1, 0.4, 0.8]


# --------------------------------------------------------------------------- #
# the shape catalogue
# --------------------------------------------------------------------------- #


def test_the_catalogue_has_the_twelve_families_the_matrix_reports(matrix_runner):
    assert len(matrix_runner.SHAPE_NAMES) == 12
    # The two families that scored 0/15 are in the catalogue under these names; the
    # documentation refers to them, so a rename must break a test.
    assert "star10" in matrix_runner.SHAPE_NAMES
    assert "concave_random15" in matrix_runner.SHAPE_NAMES


def test_every_catalog_shape_materialises_as_a_simple_polygon(matrix_runner):
    """No family may be self-intersecting at any seed used by the matrix.

    A bow-tie has no unambiguous inside, no cage offset and no boundary ordering, so
    a case built on one would be scored against a certificate about nothing. The
    random families are checked across all five matrix seeds, not one, because
    ``radial_polygon`` and ``ConvexHull`` are the two places a degenerate draw could
    appear.
    """
    for name in matrix_runner.SHAPE_NAMES:
        for seed in range(5):
            rng = np.random.default_rng(
                np.random.SeedSequence([seed, matrix_runner.SHAPE_NAMES.index(name), 20260812])
            )
            outline = matrix_runner.local_outline(name, rng) * matrix_runner.SHAPE_SCALE
            cargo = Cargo("probe", outline)
            assert is_simple_polygon(cargo.vertices), f"{name} seed {seed}"


def test_unknown_shape_name_raises(matrix_runner):
    with pytest.raises(ValueError, match="unknown shape"):
        matrix_runner.local_outline("dodecahedron", np.random.default_rng(0))


def test_shape_scale_puts_the_shared_family_on_v1s_object(matrix_runner):
    """``SHAPE_SCALE`` is one declared factor, not an accident of which branch drew it.

    CODEX drew its l_shape at scale 0.95; v1's baseline cargo is an l_shape at 1.50.
    Multiplying by ``1.50 / 0.95`` puts the shared family exactly on v1's object, so
    the diameter regression in the analysis is a real sweep rather than a scatter
    about one point.
    """
    assert matrix_runner.SHAPE_SCALE == pytest.approx(1.50 / 0.95)
    outline = matrix_runner.local_outline("l_shape", np.random.default_rng(0))
    assert np.max(np.abs(outline)) == pytest.approx(0.60 * 0.95)
    diameters = [
        polygon_diameter(
            Cargo("p", matrix_runner.local_outline(name, np.random.default_rng(i)) * matrix_runner.SHAPE_SCALE).vertices
        )
        for i, name in enumerate(matrix_runner.SHAPE_NAMES)
    ]
    assert 1.5 < min(diameters) and max(diameters) < 3.0


def test_concavity_ratio_is_zero_for_convex_and_positive_for_a_notch(matrix_runner):
    """Concavity is measured, not inferred from which names sound concave.

    The matrix's headline correlation -- concavity hurts enclosure and not
    displacement -- rests on this being a real quantity.
    """
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    assert matrix_runner.concavity_ratio(square) == pytest.approx(0.0)
    notched = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.5, 0.4], [0.0, 1.0]])
    assert matrix_runner.concavity_ratio(notched) > 0.0
    assert matrix_runner.concavity_ratio(np.array([[0.0, 0.0], [1.0, 1.0]])) == 0.0


def _mean_concavity(matrix_runner) -> dict[str, float]:
    ratios = {}
    for name in matrix_runner.SHAPE_NAMES:
        per_seed = []
        for seed in range(5):
            rng = np.random.default_rng(
                np.random.SeedSequence([seed, matrix_runner.SHAPE_NAMES.index(name), 20260812])
            )
            outline = matrix_runner.local_outline(name, rng) * matrix_runner.SHAPE_SCALE
            per_seed.append(matrix_runner.concavity_ratio(Cargo("p", outline).vertices))
        ratios[name] = float(np.mean(per_seed))
    return ratios


def test_the_two_zero_of_fifteen_families_are_not_the_two_most_concave(matrix_runner):
    """A stated attribution that does not survive being measured.

    star10 and concave_random15 both scored 0/15 in the matrix, and that has been
    described as "the two most concave families". The measured ordering is

        star10 0.400 > c_shape 0.352 > u_shape 0.336 > concave_random15 0.250
        > l_shape 0.223 > polygon32 0.079 > concave_random7 0.077 > (five convex)

    star10 is indeed the most concave. concave_random15 is *fourth*, behind c_shape
    and u_shape -- neither of which scored 0/15. So concavity ratio alone does not
    pick out the two failing families, and the matrix's own correlation is the
    narrower claim it supports: concavity hurts peak coverage
    (rho ~= -0.70), not that the two highest-concavity families are the two that
    scored zero.

    This test exists to keep the weaker, true statement in place of the stronger,
    false one. What star10 and concave_random15 share and c_shape/u_shape do not is
    *many deep narrow notches* rather than one wide slot -- ten alternating lobes and
    ``radii[1::3] *= 0.55`` respectively -- which is a different geometric property
    and is not measured by the area-ratio concavity at all.
    """
    ratios = _mean_concavity(matrix_runner)
    ranked = sorted(ratios, key=ratios.get, reverse=True)
    assert ranked[0] == "star10", ratios
    assert set(ranked[:2]) != {"star10", "concave_random15"}, ratios
    assert ratios["c_shape"] > ratios["concave_random15"], ratios
    assert ratios["u_shape"] > ratios["concave_random15"], ratios


def test_concavity_ratio_separates_convex_families_from_notched_ones(matrix_runner):
    """The five convex families measure zero to numerical precision, and the gap is wide.

    ``circle``, ``rectangle``, ``ellipse24`` and ``high_aspect`` are exactly zero --
    the outline *is* its own hull vertex list. ``convex_random`` is a ``ConvexHull``
    result compared against its own polygon area, so it lands at float noise rather
    than at zero. Both are fine; what would not be fine is a convex family measuring
    at the same order as ``concave_random7`` (0.077), because then every reported
    concavity correlation would be partly a correlation with numerical noise. The
    measured separation is ten orders of magnitude.
    """
    ratios = _mean_concavity(matrix_runner)
    convex = {"circle", "rectangle", "ellipse24", "high_aspect", "convex_random"}
    notched = set(matrix_runner.SHAPE_NAMES) - convex
    assert max(ratios[n] for n in convex) < 1e-12, ratios
    assert min(ratios[n] for n in notched) > 0.05, ratios
    assert {n for n, r in ratios.items() if r == 0.0} == convex - {"convex_random"}, ratios


# --------------------------------------------------------------------------- #
# case construction
# --------------------------------------------------------------------------- #


def test_case_config_writes_the_scale_relative_task_distance(matrix_runner):
    """``L = alpha * diameter``, and CODEX's fixed 0.10 m is not inherited.

    A fixed metric distance makes the reported displacement a statement about the
    number in the config rather than about the object, which is why CODEX's
    J/diameter came out at 4-10%. Both ends of the sampler window are pinned so the
    sampler cannot widen the target.
    """
    base = load_yaml(MATRIX_CONFIG)
    for alpha in (0.1, 0.4, 0.8):
        config, meta = matrix_runner.build_case_config(base, "l_shape", 0, alpha)
        assert meta["target_distance_m"] == pytest.approx(alpha * meta["diameter_m"])
        assert config["task"]["distance_min"] == config["task"]["distance_max"]
        assert config["task"]["distance_min"] == pytest.approx(meta["target_distance_m"])


def test_case_config_is_reproducible_and_seed_dependent(matrix_runner):
    """Same seed, same object; different seed, different object.

    The families that draw randomly would otherwise make the matrix unreproducible,
    and the two unexplained cases could not be re-run.
    """
    base = load_yaml(MATRIX_CONFIG)
    first, meta_a = matrix_runner.build_case_config(base, "concave_random15", 2, 0.4)
    again, meta_b = matrix_runner.build_case_config(base, "concave_random15", 2, 0.4)
    other, meta_c = matrix_runner.build_case_config(base, "concave_random15", 3, 0.4)

    assert np.allclose(first["cargoes"][0]["vertices"], again["cargoes"][0]["vertices"])
    assert meta_a == meta_b
    assert not np.allclose(
        np.asarray(first["cargoes"][0]["vertices"]),
        np.asarray(other["cargoes"][0]["vertices"]),
    )
    assert meta_c["seed"] == 3


def test_case_config_does_not_mutate_the_base_config(matrix_runner):
    """180 cases are built from one loaded config; case 2 must not inherit case 1."""
    base = load_yaml(MATRIX_CONFIG)
    snapshot = copy.deepcopy(base)
    matrix_runner.build_case_config(base, "star10", 1, 0.8)
    assert base == snapshot


def test_case_config_clears_the_annulus_but_does_not_centre_asymmetric_objects(matrix_runner):
    """The declared 0.35 m of annulus slack is 0.06 m on the tightest family.

    ``build_case_config`` places the *drawn outline* at the workspace centre and then
    ``Cargo.__init__`` re-centres on the area centroid. For a family whose drawn
    outline is already centroid-centred those are the same point; for an asymmetric
    one they are not, and the object ends up offset by ``|centroid of outline|``.
    Measured over all 60 (shape, seed) placements the worst offset is 0.290 m, on
    l_shape.

    The annulus, however, is centred on the workspace centre and its inner radius is
    ``reach + 0.35`` with ``reach`` measured from the *centroid*. The two centres
    disagreeing by 0.29 m means the true worst-case clearance between the object and
    the innermost possible robot is 0.0605 m, not 0.35 m -- a factor of six less than
    the comment in the harness claims.

    This did not invalidate the matrix: ``assert_initial_state_valid`` passed on all
    180 episodes and there were no construction failures, so no robot started inside
    an object. What it does mean is that the four asymmetric families (l_shape,
    c_shape, u_shape, convex_random) started with less room than intended, and that
    the effect is largest on l_shape -- which is also the only family whose
    explore_gain control experiment moved. Recorded rather than corrected, because
    changing the placement now would invalidate the committed 180 episodes.
    """
    base = load_yaml(MATRIX_CONFIG)
    domain = base["domain"]
    centre = np.array(
        [0.5 * (domain["xmin"] + domain["xmax"]), 0.5 * (domain["ymin"] + domain["ymax"])]
    )
    offsets, slacks = {}, {}
    for name in matrix_runner.SHAPE_NAMES:
        for seed in range(5):
            config, meta = matrix_runner.build_case_config(base, name, seed, 0.4)
            cargo = Cargo.from_config(config["cargoes"][0])
            offset = float(np.linalg.norm(cargo.center - centre))
            offsets[name] = max(offsets.get(name, 0.0), offset)
            slack = config["agents"]["radius_min"] - meta["object_reach_m"] - offset
            slacks[name] = min(slacks.get(name, 1e9), slack)
            assert config["agents"]["radius_max"] > config["agents"]["radius_min"]
            # Whatever the offset, no robot may be placed inside the object.
            assert slack > 0.0, (name, seed, slack)

    # Symmetric families are exactly centred; asymmetric ones are not.
    for name in ("circle", "rectangle", "ellipse24", "high_aspect", "star10"):
        assert offsets[name] == pytest.approx(0.0, abs=1e-9), name
    assert offsets["l_shape"] == pytest.approx(0.290, abs=2e-3)
    assert max(offsets.values()) == pytest.approx(offsets["l_shape"])

    # The nominal slack is 0.35 m; the realised worst case is an order tighter.
    assert min(slacks.values()) == pytest.approx(0.0605, abs=2e-3)
    assert min(slacks, key=slacks.get) == "l_shape"


# --------------------------------------------------------------------------- #
# the failure taxonomy
# --------------------------------------------------------------------------- #


def test_a_pre_run_rejection_is_not_labelled_by_what_the_run_then_did(matrix_runner):
    """An object 16 robots cannot surround does not get to be a transport stall.

    The order in ``classify`` is the claim that the failure composition table is
    about causes rather than about symptoms. A record that would otherwise read
    ``TRANSPORT_STALL`` must be attributed to the certificate check that rejected it
    before a frame ran.
    """
    stalled = {
        "certificate_checks": {"boundary_covering_number": False},
        "first_detection_frame": 10,
        "contact_ready_frame": 40,
        "transport_frame": 60,
        "hold_frame": None,
        "success": False,
    }
    assert matrix_runner.classify(stalled) == "COVER_INFEASIBLE"
    del stalled["certificate_checks"]["boundary_covering_number"]
    assert matrix_runner.classify(stalled) == "TRANSPORT_STALL"


def test_construction_and_solver_failures_outrank_every_geometric_verdict(matrix_runner):
    """A run whose QP fell back is a solver failure regardless of its geometry.

    This is the label ``rectangle__a0.10__seed004`` carries -- the only solver
    failure in 225 episodes -- and it must not be reclassified as something tidier
    by a check that also happened to fail.
    """
    assert matrix_runner.classify({"construction_error": "ValueError: x"}) == "CONSTRUCTION_FAILURE"
    assert (
        matrix_runner.classify(
            {"solver_fallbacks": 124, "solver_infeasible": 124, "certificate_checks": {"simple_polygon": False}}
        )
        == "SOLVER_FAILURE"
    )


#: A record that reached every phase and passed every certificate check. Named so
#: the taxonomy tests below differ from it in exactly one field.
COMPLETE_SUCCESS = {
    "certificate_checks": {},
    "first_detection_frame": 10,
    "contact_ready_frame": 40,
    "transport_frame": 60,
    "hold_frame": 100,
    "success": True,
}


def test_safety_violation_outranks_success(matrix_runner):
    """A run that broke separation is not a success even if it reached the goal."""
    assert matrix_runner.classify(dict(COMPLETE_SUCCESS)) == "SUCCESS"
    violated = dict(COMPLETE_SUCCESS, min_inter_agent_distance=0.20, d_min=0.32)
    assert matrix_runner.classify(violated) == "SAFETY_VIOLATION"


def test_penetration_over_budget_is_also_a_safety_violation(matrix_runner):
    """Two independent safety quantities, and either one alone must fire."""
    over = dict(COMPLETE_SUCCESS, max_penetration=0.05, penetration_budget=0.01)
    assert matrix_runner.classify(over) == "SAFETY_VIOLATION"
    under = dict(COMPLETE_SUCCESS, max_penetration=0.01, penetration_budget=0.01)
    assert matrix_runner.classify(under) == "SUCCESS"


def test_the_d_min_comparison_carries_the_documented_tolerance(matrix_runner):
    """``< d_min - 1e-6``, so equality at the barrier is not a violation.

    The acceptance gate is written with the same 1e-6 slack. A strict ``<`` would
    make every run that sits exactly on its own constraint a safety violation.
    """
    ok = dict(COMPLETE_SUCCESS, min_inter_agent_distance=0.32 - 5e-7, d_min=0.32)
    assert matrix_runner.classify(ok) == "SUCCESS"
    bad = dict(COMPLETE_SUCCESS, min_inter_agent_distance=0.32 - 5e-6, d_min=0.32)
    assert matrix_runner.classify(bad) == "SAFETY_VIOLATION"


def test_a_missing_phase_frame_is_not_silently_treated_as_reached(matrix_runner):
    """Each phase frame going missing must produce its own label.

    A record with no ``first_detection_frame`` is a search timeout, not a success
    with a gap in it. This ordering is what makes the failure composition table add
    up to the episode count.
    """
    assert matrix_runner.classify(dict(COMPLETE_SUCCESS, first_detection_frame=None)) == "SEARCH_TIMEOUT"
    assert matrix_runner.classify(dict(COMPLETE_SUCCESS, contact_ready_frame=None)) == "ENCLOSURE_TIMEOUT"
    assert matrix_runner.classify(dict(COMPLETE_SUCCESS, transport_frame=None)) == "TRANSPORT_NEVER_ARMED"
    assert matrix_runner.classify(dict(COMPLETE_SUCCESS, hold_frame=None)) == "TRANSPORT_STALL"


def test_every_taxonomy_label_the_matrix_reported_is_reachable(matrix_runner):
    """The nine labels in the committed failure composition must all be producible.

    A label that no input can reach is a label that silently never appears, and its
    absence from the table would read as evidence.
    """
    reported = {
        "SUCCESS",
        "CONTRACT_FAILURE",
        "TRANSPORT_STALL",
        "COVER_INFEASIBLE",
        "WRENCH_INFEASIBLE",
        "TRANSPORT_NEVER_ARMED",
        "MAP_INCOMPLETE",
        "SAFETY_VIOLATION",
        "SOLVER_FAILURE",
    }
    complete = {
        "first_detection_frame": 1,
        "contact_ready_frame": 2,
        "transport_frame": 3,
        "hold_frame": 4,
        "certificate_checks": {},
    }
    produced = {
        matrix_runner.classify(dict(complete, success=True)),
        matrix_runner.classify(dict(complete, success=False)),
        matrix_runner.classify(dict(complete, hold_frame=None, success=False)),
        matrix_runner.classify(
            dict(complete, certificate_checks={"boundary_covering_number": False})
        ),
        matrix_runner.classify(
            dict(complete, certificate_checks={"goal_wrench_feasibility": False})
        ),
        matrix_runner.classify(dict(complete, transport_frame=None, success=False)),
        matrix_runner.classify(dict(complete, map_complete=False)),
        matrix_runner.classify(dict(complete, min_inter_agent_distance=0.0, d_min=0.3)),
        matrix_runner.classify(dict(complete, solver_fallbacks=1)),
    }
    assert produced == reported


# --------------------------------------------------------------------------- #
# interval arithmetic
# --------------------------------------------------------------------------- #


def test_wilson_interval_contains_observed_proportion(matrix_runner):
    lower, upper = matrix_runner.wilson(2, 4)
    assert lower < 0.5 < upper
    assert lower == pytest.approx(0.15003898915214947)


def test_wilson_reproduces_the_committed_matrix_intervals(matrix_runner):
    """The three headline intervals in the results README, recomputed.

    If this drifts, either the interval code changed or the documented numbers were
    not produced by it. Both are things a reader is entitled to be able to check.
    """
    assert matrix_runner.wilson(149, 180) == pytest.approx([0.766, 0.876], abs=5e-4)
    assert matrix_runner.wilson(41, 149) == pytest.approx([0.210, 0.352], abs=5e-4)
    assert matrix_runner.wilson(54, 180) == pytest.approx([0.238, 0.371], abs=5e-4)


def test_wilson_is_defined_at_the_endpoints_and_undefined_at_zero_trials(matrix_runner):
    """0/15 is the score two families got, so the interval has to handle it.

    Clamped to [0, 1] and strictly above zero on the upper end: a 0/15 result is
    evidence of a low rate, not proof of an impossible one.
    """
    assert matrix_runner.wilson(0, 15)[0] == 0.0
    assert 0.0 < matrix_runner.wilson(0, 15)[1] < 0.25
    assert matrix_runner.wilson(15, 15)[1] == 1.0
    assert matrix_runner.wilson(0, 0) is None


def test_describe_is_empty_rather_than_nan_for_no_finite_samples(matrix_runner):
    """A family with no measurable episodes reports ``n = 0``, not a mean of nan.

    A nan mean propagates into a table and reads as a number. ``{"n": 0}`` does not.
    """
    assert matrix_runner.describe([]) == {"n": 0}
    assert matrix_runner.describe([None, float("nan"), float("inf")]) == {"n": 0}
    single = matrix_runner.describe([2.0])
    assert single["n"] == 1 and single["sd"] == 0.0
    assert single["ci95_mean"] == [2.0, 2.0]


def test_summary_reports_completion_time_without_claiming_a_bound(matrix_runner):
    """v1 computes no empirical completion-time bound, and that is the finding.

    With eligible failures present the observed completion times are right-censored,
    so any bound derived from the finishers is a bound on the finishers. CODEX had a
    function that returned ``available: False`` for this reason; v1's answer is to
    report the distribution and make no bound claim at all, which means the testable
    property is an absence.
    """
    records = [
        {
            "shape": "l_shape",
            "alpha": 0.4,
            "seed": s,
            "case_id": f"l_shape__a0.40__seed{s:03d}",
            "success": s == 0,
            "runtime_domain_eligible": True,
            "domain_eligible": True,
            "completion_time_s": 40.0 + s,
            "failure_class": "SUCCESS" if s == 0 else "CONTRACT_FAILURE",
        }
        for s in range(4)
    ]
    summary = matrix_runner.summarise(records)

    assert summary["episodes"] == 4
    assert summary["P_success"] == pytest.approx(0.25)
    assert summary["P_success_given_eligible"] == pytest.approx(0.25)
    assert summary["overall"]["completion_time_s"]["n"] == 4
    # No key anywhere in the summary offers a completion-time *bound*.
    keys = repr(summary)
    assert "completion_bound" not in keys
    assert "time_bound" not in keys


def test_summary_separates_pre_run_and_runtime_rejection(matrix_runner):
    """The matrix's 149/180 is a runtime figure, and the two must not be conflated.

    A case can hold every geometric premise and still be ineligible because the
    team's map never reached the declared epsilon density. Reporting only the
    pre-run count would overstate the conditional domain.
    """
    records = [
        {"shape": "s", "alpha": 0.1, "seed": 0, "case_id": "a", "domain_eligible": True,
         "runtime_domain_eligible": True, "success": True, "failure_class": "SUCCESS"},
        {"shape": "s", "alpha": 0.1, "seed": 1, "case_id": "b", "domain_eligible": True,
         "runtime_domain_eligible": False, "success": False, "failure_class": "MAP_INCOMPLETE"},
        {"shape": "s", "alpha": 0.1, "seed": 2, "case_id": "c", "domain_eligible": False,
         "runtime_domain_eligible": False, "success": False,
         "certificate_failures": ["boundary_covering_number"], "failure_class": "COVER_INFEASIBLE"},
    ]
    summary = matrix_runner.summarise(records)
    assert summary["rejected_pre_run"] == 1
    assert summary["rejected_runtime"] == 1
    assert summary["P_eligible_pre_run"] == pytest.approx(2 / 3)
    assert summary["P_eligible"] == pytest.approx(1 / 3)
    assert summary["rejection_composition"] == {"boundary_covering_number": 1}


def test_summary_worst_five_puts_construction_failures_first(matrix_runner):
    """The table cannot be read as a highlight reel: no J at all sorts worst.

    A construction failure has no ``J_over_diameter``, and treating a missing value
    as anything other than the worst case would quietly drop the cases that failed
    hardest out of the only per-case table in the summary.
    """
    records = [
        {"shape": "s", "alpha": 0.1, "seed": 0, "case_id": "good", "J_over_diameter": 0.9,
         "success": True, "failure_class": "SUCCESS"},
        {"shape": "s", "alpha": 0.1, "seed": 1, "case_id": "broken", "construction_error": "x",
         "success": False, "failure_class": "CONSTRUCTION_FAILURE"},
        {"shape": "s", "alpha": 0.1, "seed": 2, "case_id": "poor", "J_over_diameter": 0.05,
         "success": False, "failure_class": "CONTRACT_FAILURE"},
    ]
    summary = matrix_runner.summarise(records)
    assert [r["case_id"] for r in summary["worst_five"]] == ["broken", "poor", "good"]
    assert summary["construction_failures"] == 1
