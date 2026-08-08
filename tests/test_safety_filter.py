import numpy as np
import pytest

from dbact.contracts import ContactSafetyContract, ContractViolation
from dbact.safety_filter import SafetyFilter, SafetyFilterParams


def make_filter(**overrides) -> SafetyFilter:
    kwargs = dict(
        d_min=0.34,
        gamma_agent=6.0,
        gamma_obj=8.0,
        rho=0.05,
        r_safe=0.11,
        max_speed=0.30,
        backend="qp",
        object_row_range=0.60,
        object_row_window=0.28,
        object_row_inner_limit=0.16,
    )
    kwargs.update(overrides)
    return SafetyFilter(SafetyFilterParams(**kwargs))


def flat_boundary(x: float, count: int = 21, spacing: float = 0.06):
    """A horizontal face at y = 0 with outward normals pointing up."""
    ys = np.zeros(count)
    xs = x + (np.arange(count) - count // 2) * spacing
    points = np.column_stack([xs, ys])
    normals = np.tile(np.array([0.0, 1.0]), (count, 1))
    return points, normals


# --------------------------------------------------------------------------- #
# contract wiring
# --------------------------------------------------------------------------- #


def test_filter_rejects_an_r_safe_that_disagrees_with_the_contract():
    contract = ContactSafetyContract(robot_radius=0.16, cage_offset=0.135, delta_max=0.05, gamma_obj=8.0, rho=0.05)
    with pytest.raises(ContractViolation, match="disagrees with the C1 contract"):
        SafetyFilter(SafetyFilterParams(r_safe=0.05, gamma_obj=8.0, rho=0.05), contract=contract)


def test_filter_accepts_a_matching_contract():
    contract = ContactSafetyContract(robot_radius=0.16, cage_offset=0.135, delta_max=0.05, gamma_obj=8.0, rho=0.05)
    SafetyFilter(SafetyFilterParams(r_safe=0.11, gamma_obj=8.0, rho=0.05), contract=contract)


# --------------------------------------------------------------------------- #
# inter-robot rows
# --------------------------------------------------------------------------- #


def test_unconstrained_command_passes_through():
    result = make_filter().filter_velocity(np.zeros(2), np.array([0.1, 0.0]))
    assert result.velocity == pytest.approx([0.1, 0.0])
    assert result.status == "optimal"


def test_speed_cap_is_enforced_on_the_nominal_command():
    result = make_filter().filter_velocity(np.zeros(2), np.array([10.0, 0.0]))
    assert np.linalg.norm(result.velocity) == pytest.approx(0.30)


def test_approach_to_a_close_neighbour_is_blocked():
    f = make_filter()
    result = f.filter_velocity(np.zeros(2), np.array([0.30, 0.0]), [np.array([0.35, 0.0])])
    # Motion towards the neighbour is limited by the half-responsibility row.
    assert result.velocity[0] < 0.30
    assert result.agent_rows == 1


def test_pairwise_barrier_is_maintained_over_time_by_both_robots():
    """Both robots taking half responsibility keeps h_ij >= 0 without either
    knowing the other's input."""
    f = make_filter()
    dt = 0.05
    p = [np.array([-0.30, 0.0]), np.array([0.30, 0.0])]
    for _ in range(400):
        towards = [np.array([0.30, 0.0]), np.array([-0.30, 0.0])]
        u = [f.filter_velocity(p[i], towards[i], [p[1 - i]]).velocity for i in (0, 1)]
        p = [p[i] + u[i] * dt for i in (0, 1)]
        assert np.linalg.norm(p[0] - p[1]) >= f.params.d_min - 1e-3
    assert f.stats.fallbacks == 0


# --------------------------------------------------------------------------- #
# object rows
# --------------------------------------------------------------------------- #


def test_object_row_blocks_approach_below_r_safe():
    f = make_filter()
    points, normals = flat_boundary(0.0)
    dt = 0.05
    position = np.array([0.0, 0.40])
    for _ in range(400):
        result = f.filter_velocity(position, np.array([0.0, -0.30]), (), points, normals)
        position = position + result.velocity * dt
        assert position[1] >= f.params.r_safe - 1e-3
    assert position[1] == pytest.approx(f.params.r_safe, abs=0.02)
    assert f.stats.fallbacks == 0


def test_zero_input_is_feasible_while_the_barrier_holds():
    f = make_filter()
    points, normals = flat_boundary(0.0)
    result = f.filter_velocity(np.array([0.0, 0.30]), np.array([0.0, -0.20]), (), points, normals)
    assert result.zero_input_feasible
    assert f.stats.as_dict()["zero_input_feasible"]
    assert f.stats.as_dict()["max_slack"] == 0.0


def test_the_filter_carries_no_slack_variable_at_all():
    """A soft quadratic penalty can never drive a violation exactly to zero, so a
    reported zero violation under a soft filter would be an artefact of the
    weight. There is no slack here, and the statistics say so."""
    assert make_filter().stats.max_slack == 0.0


def test_rows_outside_the_tangential_window_are_dropped():
    """A local plane only describes the boundary near the point it was fitted at.
    Without the window, a point far along the face still constrains the robot,
    which on a non-convex object means a tangent plane from around a corner."""
    f = make_filter(object_row_window=0.10)
    # One point directly below, one 0.5 m along the face.
    points = np.array([[0.0, 0.0], [0.5, 0.0]])
    normals = np.array([[0.0, 1.0], [0.0, 1.0]])
    result = f.filter_velocity(np.array([0.0, 0.20]), np.array([0.0, -0.1]), (), points, normals)
    assert result.object_rows == 1


def test_rows_from_the_far_face_of_a_thin_part_are_dropped():
    """A robot outside one face is, by construction, on the inner side of the
    opposite face's plane. Keeping that row makes it retreat at full speed from a
    position with ample true clearance -- and if a neighbour blocks the retreat,
    the QP becomes infeasible."""
    f = make_filter()
    near = np.array([[0.0, 0.0]])
    near_normal = np.array([[0.0, 1.0]])
    # Opposite face of a 0.40 m thick slab: normal points the other way.
    far = np.array([[0.0, -0.40]])
    far_normal = np.array([[0.0, -1.0]])
    points = np.vstack([near, far])
    normals = np.vstack([near_normal, far_normal])
    result = f.filter_velocity(np.array([0.0, 0.20]), np.array([0.1, 0.0]), (), points, normals)
    assert result.object_rows == 1


def test_object_row_count_is_capped():
    f = make_filter(max_object_rows=4)
    points, normals = flat_boundary(0.0, count=41, spacing=0.01)
    result = f.filter_velocity(np.array([0.0, 0.20]), np.array([0.0, -0.1]), (), points, normals)
    assert result.object_rows == 4


def test_moving_object_velocity_enters_the_row():
    """The ISSf form feeds forward the estimated object velocity, so a boundary
    advancing towards the robot demands more retreat than a static one."""
    f = make_filter()
    points, normals = flat_boundary(0.0)
    # Close enough that the row binds; further out u = 0 satisfies both and the
    # feed-forward term has nothing to show.
    static = f.filter_velocity(np.array([0.0, 0.12]), np.zeros(2), (), points, normals)
    advancing = f.filter_velocity(
        np.array([0.0, 0.12]), np.zeros(2), (), points, normals, object_velocity=np.array([0.0, 0.25])
    )
    assert advancing.velocity[1] > static.velocity[1]


# --------------------------------------------------------------------------- #
# tiers and backends
# --------------------------------------------------------------------------- #


def test_margin_relaxation_is_counted_rather_than_hidden():
    """When the ISSf margin is what makes the problem infeasible, dropping it
    keeps h_dot >= -gamma h, so the barrier is intact -- but the ISSf constant is
    not, and the count has to be reportable."""
    f = make_filter(rho=0.20, gamma_obj=1.0, max_speed=0.05)
    points, normals = flat_boundary(0.0)
    # In the margin band, so the object row demands a retreat along +y, with a
    # neighbour directly above at exactly d_min forbidding any +y motion. The
    # barrier itself (rho = 0) is still satisfiable by u = 0; only the margin is not.
    position = np.array([0.0, 0.12])
    neighbours = [position + np.array([0.0, f.params.d_min])]
    result = f.filter_velocity(position, np.zeros(2), neighbours, points, normals)
    stats = f.stats.as_dict()
    assert result.status == "relaxed_margin"
    assert stats["margin_relaxations"] == 1
    assert stats["infeasible"] == 0
    assert stats["fallbacks"] == 0


def test_projection_backend_is_reported_as_projection():
    f = make_filter(backend="projection")
    result = f.filter_velocity(np.zeros(2), np.array([0.3, 0.0]), [np.array([0.2, 0.0])])
    assert result.status == "projection"
    assert np.linalg.norm(result.velocity) <= 0.30 + 1e-9


def test_object_rows_can_be_disabled_for_the_b0_ablation():
    f = make_filter(enable_object_rows=False)
    points, normals = flat_boundary(0.0)
    result = f.filter_velocity(np.array([0.0, 0.12]), np.array([0.0, -0.30]), (), points, normals)
    assert result.object_rows == 0
    assert result.velocity[1] == pytest.approx(-0.30)


def test_cvxpy_backend_agrees_with_the_exact_backend():
    pytest.importorskip("cvxpy")
    points, normals = flat_boundary(0.0)
    neighbours = [np.array([0.30, 0.30])]
    exact = make_filter(backend="qp").filter_velocity(
        np.array([0.0, 0.18]), np.array([0.05, -0.20]), neighbours, points, normals
    )
    reference = make_filter(backend="cvxpy").filter_velocity(
        np.array([0.0, 0.18]), np.array([0.05, -0.20]), neighbours, points, normals
    )
    assert exact.velocity == pytest.approx(reference.velocity, abs=1e-5)
