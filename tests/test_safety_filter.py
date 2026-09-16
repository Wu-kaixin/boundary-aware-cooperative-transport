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


def test_object_row_count_is_capped_in_pointwise_mode():
    f = make_filter(max_object_rows=4, object_row_mode="pointwise")
    points, normals = flat_boundary(0.0, count=41, spacing=0.01)
    result = f.filter_velocity(np.array([0.0, 0.20]), np.array([0.0, -0.1]), (), points, normals)
    assert result.object_rows == 4


def test_aggregate_mode_summarises_a_face_as_one_row():
    f = make_filter(object_row_mode="aggregate")
    points, normals = flat_boundary(0.0, count=41, spacing=0.01)
    result = f.filter_velocity(np.array([0.0, 0.20]), np.array([0.0, -0.1]), (), points, normals)
    assert result.object_rows == 1


def test_aggregate_mode_is_continuous_when_a_map_cell_disappears():
    """The point of the aggregate: ``h`` must not step when the *set* of map cells
    changes. Pointwise, deleting the cell that happened to be nearest replaces the
    binding row with one a whole voxel away and ``h`` jumps with the robot
    stationary -- which is the disturbance no affordable ``rho`` can absorb."""
    # A sparse face with one cell standing proud of the rest -- the situation a
    # voxel map is in, and the one carving creates when it deletes exactly that
    # cell. On a uniformly flat face every sample reports the same offset and
    # deleting one changes nothing either way, so the test would measure nothing.
    xs = (np.arange(7) - 3) * 0.06
    ys = np.zeros(len(xs))
    ys[3] = 0.014
    points = np.column_stack([xs, ys])
    normals = np.tile([0.0, 1.0], (len(points), 1))
    position = np.array([0.0, 0.20])
    dropped = np.arange(len(points)) != 3

    def barrier(mode: str, mask) -> float:
        f = make_filter(object_row_mode=mode)
        rows = f._object_rows(position, points[mask], normals[mask], np.zeros(2))
        return float(np.min(rows[3]))

    aggregate_step = abs(barrier("aggregate", np.ones(len(points), bool)) - barrier("aggregate", dropped))
    pointwise_step = abs(barrier("pointwise", np.ones(len(points), bool)) - barrier("pointwise", dropped))
    # The aggregate moves by O(weight of the lost cell / total weight); the
    # pointwise minimum moves by the full gap to the next-lowest sample.
    assert pointwise_step > 0.01
    assert aggregate_step < 0.4 * pointwise_step


def test_a_barrier_that_cannot_decrease_in_one_step_is_rejected():
    """``gamma_obj * dt <= 1`` is what makes the sampled row a discrete-time CBF."""
    with pytest.raises(ContractViolation, match="discrete-time CBF admissibility"):
        make_filter(gamma_obj=30.0, dt=0.05)


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


def test_nearest_feature_at_c_shape_convex_corner_keeps_true_h():
    """C5 frame-228 geometry: aggregate plane goes negative, nearest-feature does not."""
    verts = np.array(
        [
            [3.2, 3.4],
            [4.8, 3.4],
            [4.8, 4.6],
            [4.35, 4.6],
            [4.35, 3.8],
            [3.65, 3.8],
            [3.65, 4.6],
            [3.2, 4.6],
        ],
        dtype=float,
    )
    p = np.array([3.1325, 4.662185624430335])
    agg = make_filter(
        object_row_mode="aggregate",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        object_velocity_bound=0.0,
        dt=0.05,
    )
    feat = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        object_velocity_bound=0.0,
        dt=0.05,
    )
    # Dummy samples so aggregate can run; vertices drive nearest_feature.
    pts = verts.copy()
    nrm = np.tile(np.array([0.0, 1.0]), (len(pts), 1))
    h_agg = float(np.min(agg._object_rows(p, pts, nrm, np.zeros(2), obstacle_vertices=verts)[3]))
    rows = feat._object_rows(p, pts, nrm, np.zeros(2), obstacle_vertices=verts)
    h_feat = float(np.min(rows[3]))
    assert h_agg < 0.0
    assert h_feat > 0.02
    result = feat.filter_velocity(
        p, np.array([0.03, 0.0]), (), pts, nrm, np.zeros(2), obstacle_vertices=verts
    )
    assert result.zero_input_feasible
    assert result.zero_input_feasible_with_rho
    assert result.status == "optimal"


def test_nearest_feature_flat_edge_matches_supporting_plane():
    verts = np.array([[-1.0, 0.0], [1.0, 0.0], [1.0, -1.0], [-1.0, -1.0]], dtype=float)
    f = make_filter(object_row_mode="nearest_feature", r_safe=0.11, gamma_obj=8.0, rho=0.05, dt=0.05)
    p = np.array([0.0, 0.30])
    A, rhs, rhs_free, h = f._object_rows(p, np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), obstacle_vertices=verts)
    assert len(h) == 1
    assert h[0] == pytest.approx(0.30 - 0.11)
    assert A[0] == pytest.approx([0.0, 1.0], abs=1e-9)
    assert float(rhs_free[0]) <= 1e-9


def test_feature_cover_keeps_true_h_not_infinite_plane():
    """C5 geometry: the top infinite plane is negative; cover h equals true sd."""
    verts = np.array(
        [
            [3.2, 3.4],
            [4.8, 3.4],
            [4.8, 4.6],
            [4.35, 4.6],
            [4.35, 3.8],
            [3.65, 3.8],
            [3.65, 4.6],
            [3.2, 4.6],
        ],
        dtype=float,
    )
    p = np.array([3.1325, 4.662185624430335])
    f = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        object_velocity_bound=0.0,
        dt=0.05,
    )
    A, rhs, rhs_free, h = f._object_rows(
        p, np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), obstacle_vertices=verts
    )
    from dbact.geometry import signed_distance_and_gradient

    sd, _, _ = signed_distance_and_gradient(p[None, :], verts)
    h_true = float(sd[0]) - 0.065
    assert float(np.min(h)) == pytest.approx(h_true, abs=1e-12)
    assert float(np.min(h)) > 0.02
    assert np.all(np.asarray(rhs) <= 1e-9)
    # The infinite top plane n=(0,1) through y=4.6 under-estimates.
    h_plane_top = float(p[1] - 4.6 - 0.065)
    assert h_plane_top < 0.0
    assert float(np.min(h)) > h_plane_top + 0.02


def test_feature_cover_includes_adjacent_edge_near_a_convex_corner():
    """Along an edge, close to a vertex, Φ contains the incident neighbour."""
    verts = np.array([[-1.0, 0.0], [1.0, 0.0], [1.0, -1.0], [-1.0, -1.0]], dtype=float)
    f = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        dt=0.05,
        object_row_range=0.60,
    )
    # 2 u_max Δ = 0.035. Vertex at (1,0); stand on the top edge 0.02 m west of it.
    p = np.array([0.98, 0.08])
    A, rhs, rhs_free, h = f._object_rows(
        p, np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), obstacle_vertices=verts
    )
    assert len(h) >= 2
    assert float(np.min(h)) == pytest.approx(0.08 - 0.065, abs=1e-12)
    assert np.all(np.asarray(rhs) <= 1e-9)


def test_feature_cover_discrete_inequality_across_a_convex_corner():
    """A hold that crosses the vertex cell still obeys the sampled CBF bound."""
    verts = np.array([[-1.0, 0.0], [1.0, 0.0], [1.0, -1.0], [-1.0, -1.0]], dtype=float)
    f = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        dt=0.05,
        object_velocity_bound=0.0,
        forbid_fallback=True,
        allow_object_barrier_scaling=False,
    )
    p = np.array([0.99, 0.09])
    result = f.filter_velocity(
        p,
        np.array([0.35, -0.05]),
        (),
        np.empty((0, 2)),
        np.empty((0, 2)),
        np.zeros(2),
        obstacle_vertices=verts,
    )
    assert result.status == "optimal"
    assert result.zero_input_feasible_with_rho
    u = np.asarray(result.velocity, dtype=float)
    dt = f.params.dt
    p1 = p + dt * u
    from dbact.geometry import signed_distance_and_gradient

    sd0, _, _ = signed_distance_and_gradient(p[None, :], verts)
    sd1, _, _ = signed_distance_and_gradient(p1[None, :], verts)
    h0 = float(sd0[0]) - f.params.r_safe
    h1 = float(sd1[0]) - f.params.r_safe
    gamma = f.params.gamma_obj
    rho = f.params.rho
    bound = (1.0 - gamma * dt) * h0 + rho * dt
    assert h1 + 1e-12 >= bound
    # Interval samples.
    for s in (0.25, 0.5, 0.75):
        ps = p + s * dt * u
        sds, _, _ = signed_distance_and_gradient(ps[None, :], verts)
        hs = float(sds[0]) - f.params.r_safe
        assert hs + 1e-12 >= (1.0 - gamma * (s * dt)) * h0 + rho * (s * dt)


def test_zero_in_F_rho_iff_true_h_meets_rho_over_gamma():
    verts = np.array([[-1.0, 0.0], [1.0, 0.0], [1.0, -1.0], [-1.0, -1.0]], dtype=float)
    f = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        dt=0.05,
        object_velocity_bound=0.0,
    )
    rho_over_gamma = f.params.rho / f.params.gamma_obj
    p_ok = np.array([0.0, 0.065 + rho_over_gamma + 1e-4])
    p_bad = np.array([0.0, 0.065 + rho_over_gamma - 1e-4])
    ok = f.filter_velocity(
        p_ok, np.zeros(2), (), np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), obstacle_vertices=verts
    )
    bad = f.filter_velocity(
        p_bad, np.zeros(2), (), np.empty((0, 2)), np.empty((0, 2)), np.zeros(2), obstacle_vertices=verts
    )
    assert ok.zero_input_feasible_with_rho
    assert not bad.zero_input_feasible_with_rho


def test_paper_range_truncation_numbers_hold_and_are_the_filter_contract():
    from dbact.safety_filter import range_truncation_holds, range_truncation_numbers

    nums = range_truncation_numbers(0.60, 0.35, 0.05, 0.065, 0.02, 8.0)
    assert nums["R_row_minus_u_max_Delta"] == pytest.approx(0.5825)
    assert nums["r_safe_plus_rho_over_gamma"] == pytest.approx(0.0675)
    assert nums["holds"]
    assert range_truncation_holds(0.60, 0.35, 0.05, 0.065, 0.02, 8.0)
    f = make_filter(
        object_row_mode="nearest_feature",
        r_safe=0.065,
        gamma_obj=8.0,
        rho=0.02,
        max_speed=0.35,
        dt=0.05,
        object_row_range=0.60,
    )
    assert f.params.object_row_range == pytest.approx(0.60)
    assert f.params.max_speed == pytest.approx(0.35)
    assert f.params.dt == pytest.approx(0.05)
    with pytest.raises(ContractViolation, match="range truncation is insufficient"):
        make_filter(
            object_row_mode="nearest_feature",
            r_safe=0.065,
            gamma_obj=8.0,
            rho=0.02,
            max_speed=0.35,
            dt=0.05,
            object_row_range=0.05,
        )

