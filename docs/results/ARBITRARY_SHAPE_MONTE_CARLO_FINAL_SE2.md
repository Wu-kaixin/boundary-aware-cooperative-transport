# Final SE(2) arbitrary-shape Monte Carlo

Date: 2026-08-09

Branch: `Codex-boundary-aware-closed-loop-v1`

Configuration: `configs/sim/research/adaptive_progress_closed_loop.yaml`

## Protocol

The retained matrix contains 60 unscreened episodes: 12 geometry families by
five seeds, a 0.10 m activation-relative task, and a 1500-frame safety timeout.
Each episode is run before its theorem-domain eligibility is classified. No
failed or rejected case is removed from the denominator.

The matrix covers circle, rectangle, a 24-vertex ellipse approximation, L, U,
C, a 10-vertex star, random convex polygons, random concave polygons with 7 and
15 vertices, a high-aspect-ratio polygon, and a 32-vertex polygon. Position
class, yaw, goal direction, density, ground friction, and contact friction are
varied by the deterministic manifest.

This rerun includes the SE(2) moving-boundary correction: each safety row uses
the estimated rigid boundary-point velocity

```text
v_hat_boundary(q) = v_hat_object + omega_hat * [-r_y, r_x].
```

## Conditional-domain statistics

| Statistic | Count | Estimate | Wilson 95% interval |
|---|---:|---:|---:|
| `P(eligible)` | 7 / 60 | 0.1167 | [0.0577, 0.2218] |
| `P(success | eligible)` | 2 / 7 | 0.2857 | [0.0822, 0.6411] |
| `P(rejected)` | 53 / 60 | 0.8833 | [0.7782, 0.9423] |

Fourteen of 60 episodes reached the simulation task contract without regard to
the theorem domain. Two were both eligible and successful. Five eligible cases
failed and remain in the conditional denominator.

## Failure and rejection composition

| Outcome | Count |
|---|---:|
| SUCCESS | 2 |
| WRENCH_INFEASIBLE | 21 |
| MAP_INCOMPLETE | 28 |
| SOLVER_FAILURE | 2 |
| CONTRACT_FAILURE | 5 |
| CAGE_INFEASIBLE | 2 |

Rejection predicates overlap:

| Rejection predicate | Count |
|---|---:|
| contact-force capacity | 42 |
| zero-torque goal-wrench feasibility | 33 |
| runtime boundary-map epsilon | 29 |
| cage-offset self-clearance | 5 |

The two solver failures occurred in rejected-domain episodes. Across the full
matrix, 58/60 episodes were hard-QP-fallback-free, 58/60 were
infeasibility-free, and 55/60 used the full configured rho margin on every
frame. Therefore the matrix does **not** support a global `fallback = 0`,
`infeasible = 0`, or `rho relaxation = 0` claim.

## Finite-time status

No finite empirical completion-time bound is reported. The five eligible
failures are right-censored, so the maximum of the two eligible successes would
be survivor-biased. The analytic sufficient bound also remains unavailable
until positive domain-wide enclosure, transport-progress, and braking rates are
independently certified.

## Interpretation

- **Empirically validated:** the exact 60-episode manifest, outcomes, Wilson
  intervals, failure composition, and solver counts above.
- **Conditionally supported:** two closed-loop successes satisfy every current
  runtime theorem-domain predicate.
- **Unsupported:** universal arbitrary-simple-shape convergence, high
  conditional reliability, a finite empirical completion-time upper bound, or
  full-margin robustness over the complete matrix.

Retained aggregate data:

- `runs/arbitrary_shape_final_se2_60/manifest.json`
- `runs/arbitrary_shape_final_se2_60/episodes.csv`
- `runs/arbitrary_shape_final_se2_60/monte_carlo.json`
