# Barrier-update diagnosis (aggregate baseline)

Status: mechanism identified. Controller change is `object_row_mode=nearest_feature` in theorem_mode, **not** ρ reduction, clamp, or RHS truncation.

Replay: `artifacts/static_main_theorem_closure_2026-09-16/barrier_diagnosis/`  
Hardware: 16 physical / 16 logical, 3 process workers × 6 CVT threads, wall 70.8 s, 9600/9600 QP `optimal` on each case. Polar observer off.  
This replay used the **aggregate** filter (workers imported before `nearest_feature` was wired). Frame indices shifted by ~10 vs the 2026-09-15 dumps (C5 228→241) under a different CVT thread count; the geometric mechanism is the same. The 15-Sep C5 dump at frame 228 matches the closed-form left/top-corner geometry exactly.

## Parameters (unchanged)

`r_safe = 0.065 m`, `ρ = 0.02 m/s`, `γ_obj = 8 s^{-1}`, `ρ/γ = 0.0025 m`, `Δ = 0.05 s`, `γΔ = 0.4 ≤ 1`, `u_max = 0.35 m/s`. Frozen object, `v_obj = 0`. Cap `0.6 u_max = 0.21 m/s` **does not bind** on any failing object row (`rhs_uncapped = rhs_capped`).

## Fixed-affine baseline (not the implemented barrier)

For a **fixed** plane `h(p) = nᵀp − d − r_safe` with uncapped `nᵀ u ≥ −γ h + ρ` and `p⁺ = p + Δ u`:

`h(p⁺) ≥ (1 − γΔ) h(p) + ρΔ`. With `h̃ = h − ρ/γ`, `h̃(p⁺) ≥ (1−γΔ) h̃(p)`. If `h(p₀) ≥ ρ/γ` then `0 ∈ F_ρ` is kept for that plane.

This lemma applies only while `(n,d)` is constant. It is **not** a theorem about `_aggregate_face`.

## C-shape seed 5 (dev), agent 14

True nearest feature at the failure is the **convex top-left vertex** `(3.2, 4.6)`.

| frame | `h_plane` | `h_true = sd − r_safe` | `n_bar` | match | notes |
|---:|---:|---:|---|---|---|
| 237–240 | `0.0025 = ρ/γ` | `≈ 0.025` | `(−1,0)` left face | same face | riding the **infinite left plane** at the K0 boundary |
| 241 | `−0.00274` | `0.0268` | `(0,1)` top face | **90° switch** | `Δh = motion −0.00104 + update −0.00211` |
| 242 | `−0.00065` | `0.0275` | `(0,1)` | same | `0 ∉ F_hard` |
| 243–246 | `0.00061 → 0.00218` | `≈ 0.027` | `(0,1)` | same | `0 ∈ F_hard`, `0 ∉ F_ρ` |
| 247 | `0.00270` | `0.0266` | `(0,1)` | same | K0 recovers |

Hold-segment sampled signed distance stays `≈ 0.09 m > r_safe`. Physical collision never occurs. The aggregate plane is a supporting plane of the **vertex set** (does not cut the body) but is the supporting plane of the **convex hull**: `{y ≥ 4.6}` forbids the free half-space to the left of `x = 3.2`.

Selection path (`_object_rows`):

1. Tangential window `W = 0.28 m` admits top-edge samples even when the robot is past the endpoint (`|p_x − 3.2| ≈ 0.067 < W`).
2. Anchor = nearest sample; face cosine `0.26` **drops** the orthogonal left face (cos 90° = 0).
3. `_aggregate_face` on the remaining parallel normals: `n_bar = (0,1)`, `alignment = 1`, translation defect `0`. Normalisation/offset inconsistency is **not** the cause here.
4. RHS cap inactive. QP remains `optimal` (F nonempty). `b = −γ h + ρ`.

Decomposition at the switch (matched affine rows, not forced pairing of unrelated constraints):

```
h_{k+1}(p_{k+1}) − h_k(p_k)
  = [h_k(p_{k+1}) − h_k(p_k)]     motion,  n_kᵀ Δp
  + [h_{k+1}(p_{k+1}) − h_k(p_{k+1})]  barrier update (n,d jump)
```

Frame 241: motion `−0.00104`, update `−0.00211`, cosine `0`, offset jump `+7.8 m` (d from `x=3.2` to `y=4.6`). New activation does not apply (an object row existed on both frames). Numerical rebuild error `~ 1e-16`.

## L-shape seed 11 (val), agent 15, and seed 17 (val), agent 14

Same 90° convex-corner switch. Before the switch, `h_plane = ρ/γ` on one incident face while `h_true ≈ 0.028–0.029`. After the switch, `0 ∈ F_hard` and `0 ∉ F_ρ` (margin-only). Hold clearance stays above `r_safe`.

L17 after the switch rides the new plane for 17 frames with `h_plane ∈ (0.0011, 0.0023) < ρ/γ`. True clearance then **falls** toward `r_safe` (`h_true → 0.0018`) because the wrong plane is steering the robot around the corner. That subsequent true-margin entry is an *effect* of the spurious plane, not an independent K0 mechanism.

## What is not the cause

* Empty F / infeasible QP (9600/9600 optimal).
* `_cap_to_reachable` (uncapped = capped).
* Clamp (off).
* Aggregate weight jump with parallel normals (alignment = 1).
* Rigid-translation defect of `n_bar` vs `Σ g n_k`.
* Treating the C as a convex hull in the *body-cutting* sense: the plane does not cut vertices. It still over-approximates the obstacle **in free space** at a convex corner (union of half-spaces vs a single half-space).

## Controller change (after this diagnosis)

`theorem_mode` now uses `object_row_mode=nearest_feature`:

* nearest **segment** (not infinite line);
* convex vertex → radial `h = ‖p−v‖ − r_safe`, `n = ∇ sd`;
* reflex vertex → both incident edge planes;
* same `ρ`, `r_safe`, `u_max`.

This is the true object-safety function already used by `hold_segment_object_clearance`. Oracle vertices are already on the theorem_mode path. Stage C (L2, C5, L11, L17) evaluates whether `0 ∈ F_ρ` is then kept.

## Fixed-plane lemma still required after the change

`h = sd − r_safe` is one Lipschitz function of a static polygon. It is convex on outer edges and convex vertices, concave (min of two planes) at reflex corners — those use the two-plane AND. Discrete invariance of `{h ≥ ρ/γ}` still has to be proved for feature switches of the true sd; continuity of sd removes the discontinuous plane jump that killed K0 here.
