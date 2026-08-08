# Algorithm Notes (paper-v1)

Paper focus: **boundary-measure-induced density + limited local CVT + distributed responsibility CBF**. Terminology prefers *boundary-enclosing formation* over unproven formal caging.

## Local boundary measurements

The controller never observes complete object geometry. The simulator polygon is used only by the ray-casting sensor to produce

```text
z_ik = (b̂_ik, n̂_ik, c_ik, t_ik)
```

Occlusion is handled by nearest-hit ray casting. Normals are estimated by local PCA tangents with sign `n̂ᵀ(p_i − b̂) > 0` (robot outside the object).

## Boundary-measure density

Cage targets:

```text
ξ_k = b_k + d_c n_k
```

Discrete density:

```text
φ_i(q,t) = φ_0 + Σ_k Δs_k c_k e^{-λ(t-t_k)} (1 + κ g_k) K_σ(q − ξ_k)
```

Total density mass grows with estimated boundary length, so larger / more complex objects request more allocation without knowing radius, perimeter, or team size.

## Limited local CVT

Integration domain is strictly

```text
Ω_i = D ∩ B(p_i, R_ℓ)
```

Local Voronoi cell uses only communication neighbors `N_i`. Nominal enclosure control:

```text
u_i^nom = -k_c (p_i − c_i)
```

Transport phase (optional task instruction `v_d`):

```text
u_i^nom = -k_c (p_i − c_i) + v_d
```

`v_d` is configured as `task_velocity` / cargo transport direction. Discovery of the task goal is not claimed as a contribution.

## Distributed CBF-QP

Robot–robot half-responsibility (no neighbor control inputs required):

```text
h_ij = ||p_i − p_j||^2 − d_min^2
2(p_i − p_j)ᵀ u_i + (γ/2) h_ij ≥ 0
```

Object–boundary (contact allowed, penetration forbidden):

```text
h_iO = n̂ᵀ(p_i − b̂) − r_r
n̂ᵀ u_i + α(h_iO) ≥ 0
```

Solved as a local QP when CVXPY is available; otherwise iterative half-plane projection.

## Transport backends

- `scripted` (`SimpleCagingTransportDynamics`): coverage/contact threshold moves the object (fast validation).
- `pymunk` (`PymunkTransportDynamics` + `dbact_sim/rigid_body_world.py`): planar rigid-body contact pushing. Paper transportation claims should use this backend.

## Paper baselines (`controller.method`)

| Tag | Method |
|-----|--------|
| `arm` / B0 | Agent-centered Gaussian peaks (ARM-style) |
| `oracle` / B1 | Geometry-aware density from true cargo boundary |
| `no_cbf` / B2 | Boundary density + CVT, CBF disabled |
| `dbact` / B3 | Full proposed stack |

## Paper metrics

`T_enclosure`, `d_min_obs`, `R_CBF`, `T_solve`, `P_success`, plus coverage / displacement / recruited agents. Multi-seed aggregation via `scripts/run_paper_matrix.py`.
