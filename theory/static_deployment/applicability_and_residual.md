# Applicability and the meaning of a small deployment residual

Revision: 22 October 2026. This note supplements, rather than enlarges, the
[exact static theorem](main_theorem.md). It separates a proved algebraic bridge,
a conditional local-map interface, and implementation obligations.

## 1. Projected residual: what the existing bound controls

Fix one sampled configuration, its frozen reference density, positive-mass cells,
and the same nonempty closed convex product of velocity sets F used in the
theorem. Write s(v) for Euclidean saturation to the speed ball and define

    v*_i = s(k_c(c*_i-p_i)),   vhat_i = s(k_c(chat_i-p_i)),
    r_i = projection_{F_i}(v*_i),   U_i = projection_{F_i}(vhat_i),
    G = sum_i m*_i ||r_i||^2.

Zero-mass cells contribute zero; their centroids need not be defined. Saturation
and convex projection are nonexpansive, so

    ||r_i-U_i|| <= k_c ||c*_i-chat_i||,
    sqrt(G) <= sqrt(J) + k_c sqrt(E),
    G <= 2 J + 2 k_c^2 E.

The second line is the triangle inequality in the mass-weighted product norm.
Applying it at every sample and the existing dissipation estimate gives

    average(G) <= 4 B_H0/(a K Delta) + (8/a^2 + 2 k_c^2) average(E).

This is a bound on the exact-centroid **projected velocity residual of the
declared filter**, not global CVT convergence. When r_i=0 and m*_i>0, projection
optimality is equivalent to v*_i in N_{F_i}(0). Because v*_i is a positive scalar
multiple of -g*_i, with g*_i=2m*_i(p_i-c*_i), it also implies -g*_i in N_{F_i}(0).
Here N_C(x)={z: <z,y-x> <= 0 for every y in C}. This statement requires 0 in F_i,
as established by P0 in the oracle theorem. It says that the filter admits no
descent direction at zero under these frozen velocity constraints.

To infer stationarity for the original geometric feasible-position problem,
one would additionally need an appropriate tangent-cone/normal-cone equivalence
between that problem and F. Conservative sampled rows, decentralized responsibility
allocation, missing geometry and gap guards do not automatically satisfy it.
Small mass-weighted G can also hide poorly served, nearly massless cells. A
positive lower mass bound is required to turn this into an unweighted per-robot
bound. Coverage completeness, transport readiness and force capacity require
separate measurements even when G vanishes. A nonzero quadrature error floor
gives a residual neighbourhood, not asymptotic convergence.

## 2. What a local-boundary safety extension would have to supply

An observed point cloud and a clipped gap statistic are insufficient. One
possible sufficient interface, not presently supplied by the sensor, is:

1. The polygon is fixed throughout each hold; the initial robot is outside it.
2. Every true boundary point reachable by the robot's hold tube has an estimated
   boundary point within a **certified** epsilon; omitted boundary features are
   excluded by an independently valid distance bound. Spurious estimated points
   may be conservative, but do not cure missing true boundary.
3. A whole-hold certificate establishes distance to the estimated boundary at
   least r_safe + epsilon + eta, with eta covering numerical and tracking error.
4. The same certificate applies at every next hold; stale maps are accompanied by
   valid error envelopes, and inter-robot/wall constraints are certified too.

For any point x in the tube, choose a closest true boundary point q and a covered
estimate qhat. The triangle inequality yields

    dist(x, estimated boundary) <= ||x-qhat|| <= dist(x, true boundary)+epsilon.

Thus true clearance is at least r_safe+eta. Continuity and the exterior initial
condition exclude first crossing. This is a conditional distance-transfer lemma,
not a proof that an unknown map has the required coverage. Its use of a whole-hold
certificate also leaves recursive feasibility to be proved separately. An empty
map cannot satisfy the coverage premise for a nearby unseen object. The current
40 mm gap cap does not bound epsilon and must never be substituted for it.

## 3. Floating-point residual budget

For an exact feature distance h_F=dist(p,F)-r_safe, an approximate command that
satisfies the physical row only to velocity residual epsilon_u has

    n_F dot u >= -gamma h_F + rho - epsilon_u.

Convexity gives along a hold (gamma Delta <= 1)

    h_F(p+tau u) >= (1-gamma tau) h_F(p) + (rho-epsilon_u) tau.

If epsilon_u is uniformly bounded below rho, the corresponding invariant margin
is (rho-epsilon_u)/gamma, provided the remaining feasibility, feature-cover,
range, speed and pairwise assumptions still hold. Retaining the original
rho/gamma margin instead requires enough additional row tightening to pay this
budget. Solver tolerance alone is not epsilon_u: row coefficient/normal errors,
state errors, dot-product rounding, and actuator tracking contribute as well.
Rounded residual recomputation without an outward error bound is diagnostic,
not an interval certificate. No floating-point guarantee is asserted here.

## 4. Moving objects and contact

For S(t), differentiating a feature distance introduces its boundary velocity;
rotation also changes the closest feature. A new theorem would need a swept
boundary bound, velocity/rotation uncertainty, a relative-speed cover radius,
sampled feasibility under moving constraints, and the actual contact dynamics.
The static zero-input feasibility argument does not transfer to a moving body.
Neither penalty substep observations nor successful Pymunk execution close these
obligations. Hardware pose/velocity and rigid-body observations remain separate
validation work. The paper's theorem remains static and oracle-map only.

## Claim map

| Statement | Status |
| --- | --- |
| Static exact arithmetic, oracle feature cover, P0 | Existing conditional theorem |
| Exact projected residual bound above | Proved algebraic corollary under the same assumptions |
| Local-map distance transfer | Conditional lemma; sensor coverage premise unavailable |
| IEEE-754 safety with original margins | Unproved without an implementation error budget |
| Global coverage / moving contact / robust transport | Not established |
