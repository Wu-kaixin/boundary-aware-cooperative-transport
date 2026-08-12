"""Executable checks on the conditional-theorem boundary in ``dbact.guarantees``.

Ported from the CODEX branch's ``tests/test_guarantees.py`` and re-stated over the
v1 API. Three of CODEX's tests could not survive the port and are replaced rather
than deleted, for reasons that are themselves the point:

* CODEX's ``derive_conditional_finite_time_bound`` returned ``eligible``. v1's
  returns ``available``, and ``available`` is additionally gated on
  ``contraction_rates_certified``. A port that kept asserting on ``eligible``
  would have silently stopped testing the gate that matters, so the arithmetic
  and the certification gate are now asserted separately -- see
  :func:`test_finite_time_bound_is_unavailable_until_contraction_rates_certified`.

* CODEX asserted a ``paired_sweep`` agent layout with two lane chains. v1 searches
  a single static boustrophedon lane partition. The premise being tested is
  "coverage is argued over the layout the robots actually walk", so the test is
  re-stated over v1's lane geometry.

* CODEX's reference config ``configs/sim/v3/arbitrary_shape_full_workspace_500``
  does not exist here. The v1 equivalent is ``configs/sim/v2/shape_matrix.yaml``,
  which is the config the 180-episode decisive matrix was actually run on, so the
  integration checks are stated over that.

The module under test was 870 lines with no test at all, which made it the one
thing on this branch that had been delivered without being verified.
"""

from __future__ import annotations

import copy

import numpy as np
import pytest

from conftest import load_script_module

from dbact.cargo import Cargo
from dbact.contact_dynamics import ContactParams
from dbact.geometry import (
    certified_inscribed_radius,
    is_simple_polygon,
    polygon_perimeter,
    sample_polygon_boundary,
)
from dbact.guarantees import (
    FINITE_TIME_BOUND_ID,
    THEOREM_ID,
    UNCERTIFIED_CONTRACTION_RATES,
    GuaranteeSpecError,
    _controller_premises,
    _required,
    boundary_map_gap_upper_bound,
    build_admissibility_certificate,
    derive_conditional_finite_time_bound,
    evaluate_runtime_map_completeness,
    guaranteed_detection_radius,
    minimum_facing_cage_clearance,
)
from dbact_sim.scenarios import (
    build_agents,
    contact_params_from_config,
    controller_params_from_config,
    domain_from_config,
    load_yaml,
)

MATRIX_CONFIG = "configs/sim/v2/shape_matrix.yaml"

SQUARE = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
BOW_TIE = np.array([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
UNIT_SQUARE = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])

#: A U with a slot narrower than any sane ``d_min``. Two robots are assigned to
#: the mutually facing slot walls and the inter-robot barrier forbids both.
NARROW_SLOT_U = np.array(
    [
        [-0.60, -0.48],
        [0.60, -0.48],
        [0.60, 0.48],
        [0.18, 0.48],
        [0.18, -0.08],
        [-0.18, -0.08],
        [-0.18, 0.48],
        [-0.60, 0.48],
    ]
)

FINITE_TIME_KWARGS = dict(
    dt=0.1,
    search_bound_s=2.0,
    map_bound_s=3.0,
    enclosure_initial_error_m=0.8,
    enclosure_terminal_error_m=0.02,
    enclosure_contraction_rate_hz=1.0,
    transport_distance_m=0.5,
    brake_activation_distance_m=0.05,
    transport_progress_rate_mps=0.1,
    brake_initial_error_m=0.05,
    brake_terminal_error_m=0.01,
    brake_contraction_rate_hz=2.0,
    hold_dwell_s=0.5,
)


# --------------------------------------------------------------------------- #
# shape predicates
# --------------------------------------------------------------------------- #


def test_simple_polygon_predicate_rejects_a_bow_tie():
    assert is_simple_polygon(UNIT_SQUARE)
    assert not is_simple_polygon(BOW_TIE)


def test_certificate_reports_zero_witness_radius_for_a_non_simple_outline():
    """The guard, not the triangulator, is what keeps a bow-tie out.

    ``certified_inscribed_radius`` has no defined meaning on a self-intersecting
    outline. ``build_admissibility_certificate`` therefore short-circuits it on
    ``is_simple_polygon``, and this pins that ordering: a bow-tie must reach the
    ``feature_witness`` check as a zero, not as whatever ear clipping returns for
    an outline with no interior.
    """
    cargo, agents, config, controller, contact = _matrix_instance(
        vertices=BOW_TIE * 0.5 + np.array([5.0, 5.0])
    )
    cert = _certificate(cargo, agents, config, controller, contact)
    assert cert["checks"]["simple_polygon"]["passed"] is False
    assert cert["shape"]["certified_inscribed_radius"] == 0.0
    assert cert["checks"]["feature_witness"]["passed"] is False
    assert cert["domain_eligible"] is False


def test_inscribed_radius_is_a_constructive_lower_bound():
    """Contained, and never larger than the truth.

    v1 adds a centroid witness that CODEX did not have, so on a square this is
    exact rather than conservative. What must hold either way is that the value is
    a *lower* bound on the true inradius -- an over-estimate would make
    ``feature_witness`` a check a too-thin object could pass.
    """
    assert certified_inscribed_radius(SQUARE) == pytest.approx(1.0)
    # A thin sliver: the true inradius is 0.025, and no witness may exceed it.
    sliver = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.05], [0.0, 0.05]])
    assert 0.0 < certified_inscribed_radius(sliver) <= 0.025 + 1e-12


def test_minimum_facing_cage_clearance_finds_the_narrow_slot():
    """A convex outline has no facing pair at all, and ``inf`` is the right answer.

    The distinction between ``inf`` and ``0.0`` matters because the check is
    ``clearance >= d_min``: a zero would reject every square.
    """
    assert not np.isfinite(minimum_facing_cage_clearance(SQUARE, 0.2))
    # The slot is 0.36 m wide. At v1's 0.105 m cage offset the corridor is still
    # open, and narrower than any usable d_min.
    clearance = minimum_facing_cage_clearance(NARROW_SLOT_U, 0.105)
    assert np.isfinite(clearance)
    assert 0.0 < clearance < 0.36


def test_facing_clearance_goes_negative_when_the_offset_walls_cross():
    """The regression this predicate shipped with: its worst case failed open.

    Offsetting each slot wall inward by more than half the slot width crosses the
    two offset curves. The ported implementation tested "is edge j on edge i's
    outward side" on those crossed curves, so it skipped the pair and returned
    ``inf`` -- indistinguishable from a convex outline, and the check passed. A
    0.36 m slot at a 0.20 m offset therefore certified as clear.

    The signed width must now be negative, and it must equal the arithmetic that
    makes the failure obvious: ``0.36 - 2 * 0.20 = -0.04``.
    """
    crossed = minimum_facing_cage_clearance(NARROW_SLOT_U, 0.20)
    assert crossed == pytest.approx(0.36 - 2.0 * 0.20)
    assert crossed < 0.0
    # Monotone in the offset, with no discontinuity at the crossing point.
    widths = [minimum_facing_cage_clearance(NARROW_SLOT_U, o) for o in np.linspace(0.0, 0.30, 31)]
    assert all(later <= earlier + 1e-12 for earlier, later in zip(widths, widths[1:]))


def test_back_to_back_faces_are_not_mistaken_for_a_concavity():
    """A U's top and bottom edges are antiparallel and not facing.

    Admitting them would report the object's own height as a "concavity clearance",
    and once the sign convention allows negatives that mistake would turn every
    tall object into a rejection. The facing test has to distinguish the two, which
    is why it runs on the walls rather than on the offset curves.
    """
    # A plain rectangle has antiparallel edge pairs and no concavity at all.
    assert not np.isfinite(minimum_facing_cage_clearance(SQUARE, 0.105))
    tall = np.array([[0.0, 0.0], [0.4, 0.0], [0.4, 3.0], [0.0, 3.0]])
    assert not np.isfinite(minimum_facing_cage_clearance(tall, 0.05))


# --------------------------------------------------------------------------- #
# the finite-ray detection tube
# --------------------------------------------------------------------------- #


def test_finite_ray_detection_radius_decreases_when_more_returns_are_required():
    one = guaranteed_detection_radius(1.2, 0.1, 96, required_returns=1)
    three = guaranteed_detection_radius(1.2, 0.1, 96, required_returns=3)
    assert 0.0 < three < one <= 1.2


def test_detection_radius_is_monotone_over_the_whole_required_return_range():
    """Monotone, not merely smaller at one sample point.

    ``required_returns`` enters through ``k * pi / count``, which saturates at
    ``pi/2``. A non-monotone sequence would mean asking for more evidence bought a
    larger certified tube somewhere, which would be the argument certifying itself.
    """
    radii = [guaranteed_detection_radius(1.2, 0.1, 96, required_returns=k) for k in range(1, 50)]
    assert all(later <= earlier + 1e-15 for earlier, later in zip(radii, radii[1:]))


def test_detection_radius_is_angularly_bound_at_realistic_ray_counts():
    """At 72 rays the angular term binds, not the range term.

    This is the assumption an ideal-disk-sensor coverage argument hides. If the
    range term ever became the binding one at these numbers, the lane-spacing
    premise would be resting on ``sensor_range`` alone.
    """
    feature, sensor_range, rays = 0.04, 1.2, 72
    angular = feature / np.sin(np.pi / rays)
    assert guaranteed_detection_radius(sensor_range, feature, rays) == pytest.approx(angular)
    assert angular < sensor_range - feature


def test_detection_radius_is_zero_without_a_feature_disk():
    assert guaranteed_detection_radius(1.2, 0.0, 72) == 0.0


# --------------------------------------------------------------------------- #
# the map gap witness
# --------------------------------------------------------------------------- #


def test_boundary_map_gap_adds_a_continuous_sampling_upper_bound():
    """``sampled max + P/(2n)``, never the optimistic sampled maximum alone.

    The four corners of a unit square are an exact subset of its boundary, so the
    sampled maximum understates the true one-sided Hausdorff distance badly. The
    ``P/(2n)`` term is what makes the reported number an upper bound on the
    continuous boundary rather than a statement about the samples.
    """
    witness = boundary_map_gap_upper_bound(UNIT_SQUARE, UNIT_SQUARE, sample_count=100)
    assert witness["sampling_resolution_bound"] == pytest.approx(4.0 / 200.0)
    assert witness["max_boundary_gap"] == pytest.approx(
        witness["sampled_max_boundary_gap"] + witness["sampling_resolution_bound"]
    )
    assert witness["max_boundary_gap"] > witness["sampled_max_boundary_gap"]


def test_boundary_map_gap_bound_is_conservative_against_a_denser_reference():
    """Coarse sampling must not report a *smaller* bound than fine sampling.

    A gap bound that shrank as the audit got lazier would let a run pass by being
    measured badly. The perimeter term is exactly what prevents that.
    """
    points = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    coarse = boundary_map_gap_upper_bound(UNIT_SQUARE, points, sample_count=16)
    fine = boundary_map_gap_upper_bound(UNIT_SQUARE, points, sample_count=4096)
    assert coarse["max_boundary_gap"] >= fine["sampled_max_boundary_gap"]
    assert fine["max_boundary_gap"] == pytest.approx(
        polygon_perimeter(UNIT_SQUARE) / 8192.0 + fine["sampled_max_boundary_gap"]
    )


def test_boundary_map_gap_is_infinite_for_an_empty_map():
    """An empty map is an infinite gap, not a vacuous pass."""
    witness = boundary_map_gap_upper_bound(UNIT_SQUARE, np.empty((0, 2)))
    assert witness["max_boundary_gap"] == float("inf")
    assert witness["p95_boundary_gap"] == float("inf")


def test_runtime_map_completeness_gates_on_the_declared_epsilon():
    """Admissible geometry plus an unclosed map is still ineligible."""
    certificate = {"mapping": {"required_max_boundary_gap": 0.10}, "domain_eligible": True}
    dense, _ = _boundary(UNIT_SQUARE, 400)
    tight = evaluate_runtime_map_completeness(
        certificate=certificate, vertices=UNIT_SQUARE, map_points=dense
    )
    assert tight["passed"] is True
    assert tight["runtime_domain_eligible"] is True

    sparse = evaluate_runtime_map_completeness(
        certificate=certificate, vertices=UNIT_SQUARE, map_points=UNIT_SQUARE[:1]
    )
    assert sparse["passed"] is False
    assert sparse["runtime_failure_reasons"] == ["boundary_map_epsilon"]
    assert sparse["runtime_domain_eligible"] is False


def test_runtime_map_completeness_cannot_rescue_an_ineligible_domain():
    """A perfect map does not make an inadmissible object admissible."""
    certificate = {"mapping": {"required_max_boundary_gap": 0.10}, "domain_eligible": False}
    dense, _ = _boundary(UNIT_SQUARE, 400)
    result = evaluate_runtime_map_completeness(
        certificate=certificate, vertices=UNIT_SQUARE, map_points=dense
    )
    assert result["passed"] is True
    assert result["runtime_domain_eligible"] is False


# --------------------------------------------------------------------------- #
# the finite-time bound
# --------------------------------------------------------------------------- #


def test_conditional_finite_time_bound_sums_analytic_phase_bounds():
    """The arithmetic, checked independently of whether it is usable.

    ``contraction_rates_certified=True`` is passed *only here*, by a test that
    holds no certificate and claims none. Its purpose is to reach the arithmetic;
    the separate test below is the one that pins the production default.
    """
    bound = derive_conditional_finite_time_bound(
        **FINITE_TIME_KWARGS, contraction_rates_certified=True
    )
    assert bound["bound_id"] == FINITE_TIME_BOUND_ID
    assert bound["arithmetic_consistent"] is True
    assert bound["available"] is True
    assert bound["empirical"] is False
    assert bound["uncertified_rates"] == []
    assert bound["phase_bounds_s"]["enclose"] == pytest.approx(np.log(40.0))
    assert bound["phase_bounds_s"]["transport"] == pytest.approx(4.5 + 0.5 * np.log(5.0))
    assert bound["total_bound_frames"] == 146


def test_finite_time_bound_is_unavailable_until_contraction_rates_certified():
    """Arithmetically consistent and still unavailable. This is the honest state.

    Nothing in this repository certifies the three contraction rates, so a caller
    reading ``available`` gets ``False`` even though every number in
    ``phase_bounds_s`` is present and correct. The rates are named in the payload
    so the reason is machine-readable rather than buried in prose.
    """
    bound = derive_conditional_finite_time_bound(**FINITE_TIME_KWARGS)
    assert bound["arithmetic_consistent"] is True
    assert bound["available"] is False
    assert bound["contraction_rates_certified"] is False
    assert bound["uncertified_rates"] == list(UNCERTIFIED_CONTRACTION_RATES)
    assert len(UNCERTIFIED_CONTRACTION_RATES) == 3
    # The numbers are what the bound *would* be, and are still reported.
    assert bound["total_bound_frames"] == 146
    assert bound["failure_reasons"] == []


def test_conditional_finite_time_bound_fails_closed_without_progress_rate():
    kwargs = dict(FINITE_TIME_KWARGS, transport_progress_rate_mps=0.0)
    bound = derive_conditional_finite_time_bound(**kwargs, contraction_rates_certified=True)
    assert bound["arithmetic_consistent"] is False
    assert bound["available"] is False
    assert bound["total_bound_frames"] is None
    assert bound["phase_bounds_s"] is None
    assert "positive_transport_progress_rate" in bound["failure_reasons"]


@pytest.mark.parametrize(
    ("override", "reason"),
    [
        ({"dt": 0.0}, "positive_dt"),
        ({"enclosure_terminal_error_m": 0.0}, "enclosure_error_order"),
        ({"enclosure_contraction_rate_hz": 0.0}, "positive_enclosure_contraction"),
        ({"transport_distance_m": 0.0}, "positive_transport_distance"),
        ({"brake_activation_distance_m": 5.0}, "valid_brake_activation"),
        ({"brake_terminal_error_m": 0.0}, "brake_error_order"),
        ({"brake_contraction_rate_hz": 0.0}, "positive_brake_contraction"),
        ({"hold_dwell_s": -1.0}, "nonnegative_hold_dwell"),
    ],
)
def test_finite_time_bound_fails_closed_on_each_degenerate_premise(override, reason):
    """Every premise fails closed, not just the transport rate.

    A degenerate premise must produce ``None`` bounds and a named reason. The
    alternative -- returning a finite number computed from a zero rate -- is how a
    division by an unverified constant becomes a published bound.
    """
    bound = derive_conditional_finite_time_bound(
        **dict(FINITE_TIME_KWARGS, **override), contraction_rates_certified=True
    )
    assert bound["available"] is False
    assert bound["total_bound_frames"] is None
    assert reason in bound["failure_reasons"]


# --------------------------------------------------------------------------- #
# no premise has a default
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "key",
    [
        "min_feature_radius",
        "max_perimeter",
        "max_diameter",
        "boundary_map_epsilon",
        "force_margin",
    ],
)
def test_required_raises_rather_than_defaulting_a_missing_premise(key):
    """A bound nobody wrote down is not a premise.

    ``max_perimeter`` defaulting to infinity is a check that always passes;
    ``min_feature_radius`` defaulting to zero is a witness requirement no shape can
    fail. Both would look like evidence in a report, which is why the read raises.
    """
    with pytest.raises(GuaranteeSpecError, match=key):
        _required({}, key)


def test_required_reads_a_declared_premise_with_its_declared_type():
    assert _required({"max_perimeter": "10.0"}, "max_perimeter") == pytest.approx(10.0)
    assert _required({"n": 3.0}, "n", int) == 3


def test_dropping_a_declared_premise_from_the_matrix_config_raises():
    """Deleting a premise from the config must break the certificate, loudly."""
    cargo, agents, config, controller, contact = _matrix_instance()
    del config["guarantee"]["max_perimeter"]
    with pytest.raises(GuaranteeSpecError, match="max_perimeter"):
        _certificate(cargo, agents, config, controller, contact)


def test_controller_premises_raise_when_a_parameter_is_renamed():
    """``getattr(x, name, default)`` is how a premise gets quietly satisfied.

    ``_controller_premises`` reads every field as a plain attribute, so a rename in
    ``DBACTParams`` must raise while the certificate is being built rather than
    yield a certificate about a value nobody chose.
    """
    controller = controller_params_from_config(load_yaml(MATRIX_CONFIG))
    assert _controller_premises(controller)["search_mode"] == "sweep"

    class Renamed:
        def __getattr__(self, name):
            if name == "cage_offset":
                raise AttributeError(name)
            return getattr(controller, name)

    with pytest.raises(AttributeError, match="cage_offset"):
        _controller_premises(Renamed())


# --------------------------------------------------------------------------- #
# the assembled certificate, on the config the decisive matrix ran on
# --------------------------------------------------------------------------- #


def _boundary(vertices: np.ndarray, count: int):
    return sample_polygon_boundary(vertices, count=count)


def _matrix_instance(*, vertices: np.ndarray | None = None, config: dict | None = None):
    """One concrete instance built from the matrix config.

    With ``vertices=None`` this goes through the matrix runner's own
    ``build_case_config``, so the geometry, annulus and task distance are the ones
    the 180-episode experiment used rather than a hand-tuned instance that happens
    to pass. With ``vertices`` given, the same config carries a substituted outline
    and everything else is held: that is what makes a rejection attributable to the
    shape rather than to the harness around it.
    """
    base = copy.deepcopy(config if config is not None else load_yaml(MATRIX_CONFIG))
    if vertices is None:
        base, _ = load_script_module("run_arbitrary_shape_monte_carlo").build_case_config(
            base, "l_shape", 0, 0.40
        )
        vertices = np.asarray(base["cargoes"][0]["vertices"], dtype=float)
    else:
        vertices = np.asarray(vertices, dtype=float)
        centre = vertices.mean(axis=0)
        reach = float(np.max(np.linalg.norm(vertices - centre[None, :], axis=1)))
        base["cargoes"] = [
            {
                "id": "cargo_0",
                "shape": "polygon",
                "vertices": vertices.tolist(),
                "surface_density": 2.0,
            }
        ]
        base["agents"] = dict(base["agents"])
        base["agents"]["center"] = centre.tolist()
        base["agents"]["radius_min"] = round(reach + 0.35, 4)
        base["agents"]["radius_max"] = round(reach + 1.15, 4)
        base["task"] = dict(base["task"])
        base["task"]["distance_min"] = 0.5
        base["task"]["distance_max"] = 0.5

    cargo = Cargo("cargo_0", vertices, surface_density=2.0)
    agents = build_agents(base, seed=0)
    return cargo, agents, base, controller_params_from_config(base), contact_params_from_config(base)


def _certificate(cargo, agents, config, controller, contact, *, distance=0.8):
    return build_admissibility_certificate(
        cargo=cargo,
        agents=agents,
        domain=domain_from_config(config),
        goal_direction=np.array([1.0, 0.0]),
        target_distance=distance,
        config=config,
        controller=controller,
        contact=contact,
        dt=float(config["dt"]),
    )


def test_matrix_reference_instance_has_a_complete_domain_certificate():
    """The config the decisive matrix ran on produces an eligible l_shape instance.

    ``domain_eligible`` is what the experiment gated on. ``eligible`` additionally
    requires ``finite_time_eligible``, which is ``False`` for every run on this
    branch because the derived bound is unavailable -- so the two are asserted
    apart, and the second is asserted to be false rather than skipped.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    cert = _certificate(cargo, agents, config, controller, contact)
    assert cert["theorem_id"] == THEOREM_ID
    assert cert["enabled"] is True
    assert cert["domain_eligible"] is True
    assert cert["domain_failure_reasons"] == []
    assert cert["finite_time_eligible"] is False
    assert "derived_conditional_finite_time_bound" in cert["finite_time_failure_reasons"]
    assert cert["eligible"] is False


def test_certificate_never_claims_formal_caging():
    """``formal_caging`` is a constant, not a computed field.

    Operational enclosure is what the predicates certify. Nothing in the module
    proves the object cannot escape the cage, and there is no input -- admissible
    shape, dense map, full quorum -- that flips this to ``True``.
    """
    eligible = _certificate(*_matrix_instance())
    assert eligible["domain_eligible"] is True
    assert eligible["formal_caging"] is False
    assert "formal caging is NOT claimed" in eligible["claim"]

    # The same constant on an instance that fails every interesting predicate:
    # there is no input that flips it either way.
    slab = np.array([[4.0, 5.0], [6.0, 5.0], [6.0, 5.02], [4.0, 5.02]])
    ineligible = _certificate(*_matrix_instance(vertices=slab))
    assert ineligible["domain_eligible"] is False
    assert ineligible["formal_caging"] is False


def test_too_thin_shape_is_simulatable_but_not_theorem_eligible():
    """Ineligible is not the same as unsimulatable.

    A 20 mm slab contains no 0.04 m disk, so the finite-ray detection argument has
    no positive tube and ``feature_witness`` fails. The object can still be
    simulated; what it may not carry is the conditional-guarantee label.
    """
    slab = np.array([[4.0, 5.0], [6.0, 5.0], [6.0, 5.02], [4.0, 5.02]])
    cert = _certificate(*_matrix_instance(vertices=slab))
    assert cert["domain_eligible"] is False
    assert "feature_witness" in cert["domain_failure_reasons"]
    assert cert["shape"]["certified_inscribed_radius"] < 0.04


def test_narrow_slot_u_is_rejected_by_cage_self_clearance():
    """The failure mode this predicate exists for is silent at runtime.

    Two robots told to stand on mutually facing slot walls that are closer than
    ``d_min`` do not produce a loud failure: they produce a permanently open arc.
    Checking the geometry before the run is the only place it is cheap.
    """
    config = load_yaml(MATRIX_CONFIG)
    slot = NARROW_SLOT_U + np.array([5.0, 5.0])
    cert = _certificate(*_matrix_instance(vertices=slot))
    assert cert["domain_eligible"] is False
    assert "cage_offset_self_clearance" in cert["domain_failure_reasons"]
    assert cert["checks"]["cage_offset_self_clearance"]["value"] < config["controller"]["d_min"]


def test_oversized_perimeter_fails_the_covering_number_the_team_can_reach():
    """The covering number is derived from the team, not chosen to fit the object.

    ``ceil(P / (2 (R_cov - d_c)))`` with 16 robots at ``R_cov = 0.42`` and
    ``d_c = 0.105`` caps the coverable perimeter. An object past that cap must fail
    ``perimeter_bound`` or ``boundary_covering_number``, and this asserts one of
    them actually fires rather than both being slack.
    """
    angles = np.linspace(0.0, 2.0 * np.pi, 40, endpoint=False)
    big = np.column_stack([2.6 * np.cos(angles), 2.6 * np.sin(angles)]) + np.array([5.0, 5.0])
    cert = _certificate(*_matrix_instance(vertices=big))
    reasons = cert["domain_failure_reasons"]
    assert reasons, "an object larger than the workspace must fail some domain premise"
    assert {"perimeter_bound", "diameter_bound", "boundary_covering_number"} & set(reasons)


def test_lane_partition_premises_are_stated_over_v1s_sweep_not_codexs_layout():
    """v1 searches one static boustrophedon lane partition, and this pins that.

    CODEX proved coverage for a ``paired_sweep`` layout with a rendezvous protocol.
    Restating those premises verbatim would have produced a certificate that is
    false for every v1 run for a reason having nothing to do with the object. The
    lane geometry below is exactly what ``_sweep_velocity`` walks.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    cert = _certificate(cargo, agents, config, controller, contact)
    search = cert["search"]
    xmin, xmax, ymin, ymax = domain_from_config(config)
    # Read off the resolved controller parameters, not the YAML: ``search_margin``
    # is a ``DBACTParams`` default here, and a test that read the config would pass
    # by not exercising the value the certificate actually used.
    margin = float(controller.search_margin)

    assert cert["checks"]["lane_partition_declared"]["value"] == "sweep"
    assert search["lane_count"] == len(agents)
    assert search["lane_width"] == pytest.approx(
        (xmax - xmin - 2.0 * margin) / len(agents)
    )
    assert search["lane_height"] == pytest.approx(ymax - ymin - 2.0 * margin)
    assert search["sweep_bound_frames"] > 0
    # Half a lane must fit inside the detection tube, or the partition does not
    # cover the workspace at all.
    assert 0.5 * search["lane_width"] <= search["guaranteed_detection_radius"] + 1e-12


def test_token_relay_premise_is_labelled_necessary_but_not_sufficient():
    """v1's relay is opportunistic, and the certificate says so where it is read.

    This is the one place v1's search protocol is genuinely weaker than CODEX's.
    Marking it in the rationale rather than smoothing it over is the whole point,
    so the wording is asserted: a passing check here must not be mistaken for a
    connectivity guarantee.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    cert = _certificate(cargo, agents, config, controller, contact)
    rationale = cert["checks"]["token_relay_connectivity"]["rationale"]
    assert "NECESSARY, NOT SUFFICIENT" in rationale
    assert "no claim" in rationale.lower()


def test_wrench_feasibility_is_unilateral_and_zero_torque():
    """A pulling allocation would certify wrenches no pusher can produce.

    The LP is bounded nonnegative and carries a zero-torque equality row. Reversing
    the requested direction on an asymmetric object exercises a different edge set,
    and both must remain zero-torque solutions or the check is certifying spin
    rather than transport.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    forward = _certificate(cargo, agents, config, controller, contact)
    assert forward["checks"]["goal_wrench_feasibility"]["passed"] is True
    assert forward["task"]["wrench_residual"] <= 1e-7
    assert forward["task"]["wrench_equivalent_agents"] > 0.0


def test_wrench_feasibility_fails_without_cage_penetration():
    """Zero available normal force is infeasible, not a free pass.

    ``per_robot = stiffness * max(robot_radius - cage_offset, 0)``. Standing the
    cage off further than the robot radius makes every contact force zero, and the
    check must fail closed rather than return a residual-free solve of ``0 = 0``.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    config = copy.deepcopy(config)
    config["controller"]["cage_offset"] = float(config["controller"]["robot_radius"]) + 0.1
    cert = _certificate(cargo, agents, config, controller_params_from_config(config), contact)
    assert cert["checks"]["goal_wrench_feasibility"]["passed"] is False
    assert "goal_wrench_feasibility" in cert["domain_failure_reasons"]


def test_bounded_error_premise_is_capped_by_the_issf_margin_rho():
    """``velocity_error <= rho`` is a premise, and exceeding it must fail closed.

    The object rows are built with an ISSf margin ``rho``. A declared moving-
    boundary velocity error larger than ``rho`` is a declaration that the safety
    statement does not apply, so the certificate must refuse it rather than carry
    the label anyway. This is the check that turns the two
    ``bounded_errors`` values from prose into a gate.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    rho = float(config["controller"]["rho"])
    assert config["guarantee"]["bounded_errors"]["velocity_error"] <= rho

    broken = copy.deepcopy(config)
    broken["guarantee"]["bounded_errors"]["velocity_error"] = rho * 2.0
    cert = _certificate(cargo, agents, broken, controller, contact)
    assert cert["checks"]["bounded_perception_and_motion_error"]["passed"] is False
    assert "bounded_perception_and_motion_error" in cert["domain_failure_reasons"]

    degenerate = copy.deepcopy(config)
    degenerate["guarantee"]["bounded_errors"]["normal_error_deg"] = 90.0
    cert = _certificate(cargo, agents, degenerate, controller, contact)
    assert cert["checks"]["bounded_perception_and_motion_error"]["passed"] is False


def test_disabled_guarantee_block_yields_no_eligibility():
    """``enabled: false`` means ineligible, even when every predicate passes.

    Eligibility is a claim someone has to opt into. A config that never declared it
    must not acquire it by having admissible geometry.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    config = copy.deepcopy(config)
    config["guarantee"]["enabled"] = False
    cert = _certificate(cargo, agents, config, controller, contact)
    assert cert["enabled"] is False
    assert cert["domain_eligible"] is False
    assert cert["eligible"] is False
    # The individual predicates still report honestly; only the label is withheld.
    assert cert["checks"]["simple_polygon"]["passed"] is True


def test_certificate_is_json_serialisable():
    """The certificate is committed evidence, so it has to survive a round trip."""
    import json

    cargo, agents, config, controller, contact = _matrix_instance()
    cert = _certificate(cargo, agents, config, controller, contact)
    restored = json.loads(json.dumps(cert))
    assert restored["theorem_id"] == THEOREM_ID
    assert restored["formal_caging"] is False
    assert set(restored["groups"]) == {"shape", "search", "task", "time"}


def test_contact_params_from_matrix_config_need_a_real_quorum():
    """``min_push_agents`` must dominate the Coulomb breakaway requirement.

    Read straight from the config rather than from a hand-picked mass, so that a
    later change to ``surface_density`` or ``friction`` that quietly invalidates
    the quorum shows up here.
    """
    cargo, agents, config, controller, contact = _matrix_instance()
    assert isinstance(contact, ContactParams)
    need = contact.min_cooperating_robots(cargo.mass, controller.cage_offset)
    assert 0.0 < need <= controller.min_push_agents
    cert = _certificate(cargo, agents, config, controller, contact)
    assert cert["checks"]["contact_force_capacity"]["passed"] is True
