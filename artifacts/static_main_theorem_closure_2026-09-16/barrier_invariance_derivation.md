# Barrier invariance under nearest-feature cover

This note is the object-family half of recursive feasibility. Agent and wall
rows are in `theorem_mode.py`. The construction is `SafetyFilter._nearest_feature_rows`
with `object_row_mode=nearest_feature` (theorem_mode). ρ, r_safe, u_max are unchanged.

## Setup

Frozen simple polygon S, oracle vertices, v_obj = 0. Sampling period Δ, speed limit
u_max, γ = γ_obj > 0, 0 ≤ γΔ ≤ 1, ρ > 0. Robot centre p, hold p(τ) = p + τ u,
τ ∈ [0, Δ], ‖u‖ ≤ u_max.

Unsigned distance to the boundary:

    dist(p, ∂S) = min_{edges F} dist(p, F).

Outside S, signed distance sd(p, S) = dist(p, ∂S). Barrier

    h_true(p) := sd(p, S) − r_safe.

Each edge F is a compact segment, hence convex. dist(·, F) is convex and
1-Lipschitz, differentiable off F, with ∇dist(p, F) = (p − q)/‖p − q‖ where q
is the unique closest point (radial at an endpoint, outward unit normal of the
line when q is interior and p is on the outer side).

    h_F(p) := dist(p, F) − r_safe,    n_F(p) := ∇ dist(p, F).

Then h_true(p) = min_F h_F(p) outside S.

## Cover Φ(p)

    Φ(p) := { edges F : dist(p, F) ≤ dist(p, ∂S) + 2 u_max Δ }
            ∩ { dist(p, F) ≤ object_row_range }.

Lemma (one-step cover). If q ∈ B(p, u_max Δ) and F_q is a nearest edge of q, then
either dist(p, ∂S) > object_row_range − u_max Δ (the robot cannot enter the
sensing ball of a feature outside range in one hold; with paper numbers
0.60 − 0.0175 ≫ r_safe + ρ/γ), or F_q ∈ Φ(p).

Proof. dist(p, F_q) ≤ dist(q, F_q) + u_max Δ = sd(q) + u_max Δ
≤ sd(p) + 2 u_max Δ.

## Discrete CBF on one convex feature

Fix F. The QP row is n_F(p)^T u ≥ −γ h_F(p) + ρ, uncapped. Convexity:

    h_F(p + τ u) ≥ h_F(p) + τ n_F(p)^T u ≥ (1 − γ τ) h_F(p) + ρ τ,  τ ∈ [0, Δ].

If h_F(p) ≥ ρ/γ and γτ ≤ 1, then h_F(p+τu) ≥ ρ/γ.

Cap: when v_obj = 0 and h_F ≥ ρ/γ, the uncapped RHS is −γ h_F + ρ ≤ 0. Reachable
caps are nonnegative, so `_cap_to_reachable` does not bind. The implemented row
is the uncapped CBF.

## Invariance of the minimum

Let Φ = Φ(p). For every F ∈ Φ the row is enforced, hence

    min_{F ∈ Φ} h_F(p + τ u) ≥ (1 − γ τ) min_{F ∈ Φ} h_F(p) + ρ τ
                             = (1 − γ τ) h_true(p) + ρ τ.

For F ∉ Φ, 1-Lipschitz gives

    h_F(p + τ u) ≥ h_F(p) − τ u_max > h_true(p) + 2 u_max Δ − τ u_max
                 ≥ h_true(p) + u_max Δ.

Therefore, outside S,

    h_true(p + τ u) = min_F h_F(p + τ u)
                    ≥ min( (1−γτ) h_true(p) + ρ τ ,  h_true(p) + u_max Δ )
                    = (1−γτ) h_true(p) + ρ τ

whenever h_true(p) ≥ ρ/γ (the second argument is larger). In particular the
endpoint satisfies the same bound, and {h_true ≥ ρ/γ} is positively invariant
along the hold.

A single nearest-edge row does not give the min-over-Φ step: an uncovered
neighbour that becomes nearest during the hold is unconstrained. That is why
the controller covers Φ, not only argmin.

## Zero-input feasibility of object rows

u = 0 satisfies n^T u ≥ −γ h + ρ iff h ≥ ρ/γ. Every covered h_F ≥ h_true, so

    0 ∈ F_ρ^{object}  ⇔  h_true(p) ≥ ρ/γ.

Infinite supporting planes of non-nearest edges are not used. They can take
values strictly below h_true (C5/L11/L17: convex-corner 90° switch of an
aggregate infinite plane). Reflex two-plane infinite AND is also not used: in a
concave crook it under-estimates sd and can destroy 0 ∈ F_ρ while h_true ≥ ρ/γ.

## What this is not

- Not a claim that one halfspace ∩ speed ball implies joint QP feasibility.
  Agents and walls are separate lemmas; conjunction is §joint.
- Not a proof from trajectory observation. The cover radius 2 u_max Δ is
  geometry of 1-Lipschitz distance, not a measured switch count.
- Oracle vertices are already on the theorem_mode path. The cover uses the same
  polygon, not future states or a global Hessian.

## Implemented correspondence

| Symbol | Code |
|---|---|
| dist(p, F) | `closest_point_on_segment` |
| Φ | `_nearest_feature_rows` loop, `cover = sd + 2 max_speed dt` |
| n_F, h_F | (p−q)/d and d − r_safe; duplicates collapsed |
| RHS | `_finish_object_rows` then `_cap_to_reachable` |
| v_obj | 0 in static theorem_mode |
