# Algorithm Notes

Current as of the C1–C3 + S1–S7 rebuild. Measured numbers and withdrawals are in
[REFACTOR_2026-08-08.md](REFACTOR_2026-08-08.md).

## Boundary-aware density (S4)

Each observation `(b_k, n̂_k)` carries an arc length `Δs_k`, a confidence `c_k` and
an age. The offset model places a kernel at a cage target outside the object:

```text
ξ_k    = b_k + d_k n̂_k
φ(q)   = φ_0 + Σ_k  Δs_k · c_k · (1 + κ g_k) · K_σ(q − ξ_k)
```

`Δs_k` is what makes `φ` a **measure on the boundary** rather than a sum over
however many samples the sensor happened to return: a densely and a sparsely
scanned arc of equal length carry equal mass. `g_k` is an uncovered-gap term
computed from neighbour positions alone.

**Offset self-intersection.** The Jacobian of `b → b + d_c n̂` is `|1 − d_c κ|`,
which degenerates at a concave point of curvature radius `R_c` once `d_c = R_c`:
the offset curve folds and mass piles up in the concavity. On a polygon with a
*sharp* reflex corner the folded targets remain outside the outline, so the
symptom is the pile-up (peak/median), not targets inside the object.

**Distance-field alternative** (`density_mode: distance_field`):

```text
d̂(q)  = min_k ‖q − b̂_k‖
φ(q)  = φ_0 + w(q) · exp(−(d̂(q) − d_c)² / 2σ²)      on the outer side only
```

This level set is the boundary of the Minkowski sum of the object with a disk of
radius `d_c`, so it is always a simple curve and cannot self-intersect. Normals
are used only to pick the outer side, so a large `ε_n` cannot fold it. The cost,
reported as such: total mass is no longer proportional to estimated perimeter.

**Arc allocation for transport.** A robot in contact at a point with outward
normal `n̂` applies force along `−n̂`, so its contribution along the goal is
`−(n̂·û_goal)|F|`. A uniform cage therefore cancels itself. The per-observation
offset ramps with the resisting component:

```text
d_k = d_c + clip(n̂_k·û_goal / a_lead, 0, 1) · (d_lead − d_c)
```

Zero alignment or better keeps the contact offset; by `a_lead` the robot is lifted
to `d_lead > r_robot`, outside contact, where it bounds forward motion without
opposing it. The lateral arc stays in contact and supplies containment plus
tangential friction. The ramp is continuous, so a robot near an arc boundary does
not chatter.

## Limited-range CVT (S5)

```text
Ω_i = D ∩ B(p_i, R_l)                     strict local disk
V_i = { q ∈ Ω_i : ‖q−p_i‖ ≤ ‖q−p_j‖, j ∈ N_i }
f(r) = min(r², R_l²)                      truncated performance function
H(p) = ∫_D  min_i f(‖q − p_i‖) φ(q) dq
```

Truncation is load-bearing. `f` is continuous at `r = R_l`, so the flux term over
`∂B(p_i, R_l)` cancels when `H` is differentiated and
`∂H/∂p_i = −2 m_i (c_i − p_i)`: move-to-centroid is a descent direction (Cortés,
Martínez & Bullo, ESAIM COCV 2005). Using `‖q−p_i‖²` untruncated leaves an
uncancelled boundary term and the descent statement is false.

Descent holds **below a step-size bound**: measured 0 rising steps of 40 at gain
0.25, and more than 0 at gain 1.0. The bound belongs in any write-up; an
unqualified monotonicity claim does not.

Mass outside every disk is charged at the saturation value `R_l²` rather than
dropped. Dropping it makes `H` fall for the wrong reason.

**Neighbour completeness.** If `R_l ≤ R_comm/2` then for any `q ∈ B(p_i,R_l)`
closer to `p_j`, `‖p_i−p_j‖ ≤ ‖p_i−q‖ + ‖q−p_j‖ ≤ 2R_l ≤ R_comm`, so `j` is
already a communication neighbour. The cell computed from neighbours therefore
**equals** the true Voronoi cell restricted to the disk — decentralisation is
exact, not approximate. Violating the condition raises a `RuntimeWarning`.

## Safety filter (S1)

One QP, two constraint families, no slack variable.

Inter-robot, shared responsibility (Wang, Ames & Egerstedt, T-RO 2017 — a known
result, cited in support of the feasibility proposition, not claimed):

```text
h_ij = ‖p_i − p_j‖² − d_min²
2 (p_i − p_j)ᵀ u_i  ≥  −(γ/2) h_ij
```

Object boundary, one row per nearby observed point, as an intersection of
half-spaces rather than a smooth CBF (nonsmooth barrier construction of
Glotfelter, Cortés & Egerstedt, L-CSS 2017):

```text
h_k = n̂_kᵀ (p_i − b_k) − r_safe
n̂_kᵀ u_i  ≥  n̂_kᵀ v̂_obj − γ_obj h_k + ρ
```

`ρ` is not a tuning margin: it is the price of dropping the `d/dt(n̂_k)ᵀ(p_i−b_k)`
term, and it is what turns the exact CBF statement into an **ISSf** one. Its value
belongs in the paper.

**Row admission.** A local plane describes the boundary only near where it was
fitted, so a row is kept only while the robot is inside a tangential window `W`
and its normal offset exceeds `−r_robot`. Without the window the construction
silently becomes a convex-hull constraint; without the inner limit, relayed points
from the far face of a thin part demand a full-speed retreat from a robot with
ample clearance. `W` also bounds the effect of normal error: a normal wrong by
`ε_n` misplaces the plane by at most `W sin ε_n`.

**Feasibility certificate.** With `u_i = 0` the inter-robot rows hold whenever
`h_ij ≥ 0` and the object rows hold whenever `γ_obj h_k ≥ n̂ᵀv̂_obj + ρ`. The QP is
feasible without slack, which is why none is present: a soft quadratic penalty can
never drive a violation exactly to zero, so a reported zero violation under a soft
filter would be an artefact of the weight.

**Two-tier solve.** If the ISSf margin `ρ` is what makes the problem infeasible,
the solve is retried without it. `ḣ ≥ −γh` still holds, so the barrier survives
while the ISSf constant does not; every relaxation is counted and reported.

## Contact dynamics (S6)

```text
δ    = r_robot − s
f_n  = max(0, k_p δ − k_d n̂ᵀv_rel)
F    = −f_n n̂ + f_t                      (Coulomb, viscously regularised)
τ    = (q − x) × F
```

`k_p` is tied to C1: the largest force available at the cage ring is
`k_p (r_robot − d_c)`, and the penetration budget is `δ_max = r_robot − r_safe`,
exactly what the object CBF leaves open.

**Floor friction is Coulomb, not viscous.** Below `μ_g m g` the object does not
move at all. Linear drag instead gives a terminal speed `‖F‖/(mc)` with no
threshold, which for any reasonable stiffness exceeds the robots' own speed limit:
the cargo is kicked away by the first robot to touch it, penetration relaxes to
zero, and the cage never closes. Coulomb friction also makes the task genuinely
cooperative — with `μ_g m g > k_p δ_max` a single robot cannot move the object, so
a transport result is a result about the team.

## Success criterion (S7 / C3)

`J = (x_T − x_0)·û_goal` is a signed projection, so a run that ends up behind
where it started scores negative and cannot pass. Efficiency `J/‖Δx‖` is gated at
`‖Δx‖ ≥ 0.1 m` because direction is close to undefined below that. Safety is part
of the test, not a separate report: reaching the goal by driving robots through
the cargo is not success. Failures are recorded as reason strings.
