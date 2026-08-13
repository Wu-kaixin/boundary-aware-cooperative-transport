# Stage 5: unscreened arbitrary-shape Monte Carlo

Date: 2026-08-09
Branch: `Codex-boundary-aware-closed-loop-v1`
Runner: `scripts/run_arbitrary_shape_monte_carlo.py`

## Manifest

The final matrix contains 36 episodes: 12 geometry families × 3 seeds, all at a requested activation-relative transport distance of 0.10 m and a 1500-frame safety timeout.

Geometry families: circle, rectangle, 24-vertex ellipse approximation, L, U, C, 10-vertex star, random convex polygon, random concave polygons with 7 and 15 vertices, high-aspect-ratio rectangle, and a 32-vertex polygon.

The deterministic manifest also varies:

- workspace centre / edge / corner / random position classes;
- uniform random yaw;
- feasible uniform 360-degree goal direction;
- surface density in `{0.8, 1.5, 2.2}`;
- ground friction in `{0.15, 0.30, 0.45}`; and
- contact friction in `{0.40, 0.60, 0.80}`.

Every episode was simulated before classification. Rejected shapes and failed runs remain in the denominator and raw records.

## Conditional-domain statistics

| Statistic | Count | Estimate | Wilson 95% interval |
|---|---:|---:|---:|
| `P(domain eligible)` | 4 / 36 | 0.111 | [0.044, 0.253] |
| `P(success | domain eligible)` | 2 / 4 | 0.500 | [0.150, 0.850] |
| `P(rejected)` | 32 / 36 | 0.889 | [0.747, 0.956] |

Ten of 36 episodes met the simulation task contract irrespective of theoretical-domain eligibility. Only two were both runtime-domain-eligible and successful: `circle__seed_000` and `high_aspect__seed_001`.

The two eligible failures were retained:

- `ellipse24__seed_002`: HOLD occurred, but activation-relative `J=0.6589 m` exceeded `J_max=0.25 m` (overshoot).
- `concave_random7__seed_002`: transport activated at frame 282 before the operational enclosure certificate first passed at frame 327.

## Failure and rejection composition

Episode outcome composition over all 36 cases:

| Outcome | Count |
|---|---:|
| SUCCESS | 2 |
| WRENCH_INFEASIBLE | 15 |
| MAP_INCOMPLETE | 15 |
| SOLVER_FAILURE | 1 |
| CONTRACT_FAILURE | 2 |
| CAGE_INFEASIBLE | 1 |

Domain rejection predicates overlap and therefore sum above 32:

| Rejection predicate | Count |
|---|---:|
| contact force capacity | 25 |
| zero-torque goal wrench feasibility | 21 |
| runtime map epsilon | 16 |
| cage-offset self-clearance | 3 |

This low eligible fraction is a result, not a threshold-tuning target. In particular, a visual/transport success is still rejected when the declared contact, wrench, map or cage premise fails.

## Safety and runtime

- 35/36 episodes had zero hard-QP fallback and infeasibility.
- 32/36 episodes had zero rho relaxation.
- Mean headless rate: 19.97 fps.
- Median headless rate: 18.76 fps.
- Range: 3.91–37.92 fps.

The performance tail is concentrated in concave/high-vertex cases. This fails the desired >20 fps target at the distribution median and motivates Stage 6 profiling; no safety frequency or certificate threshold was reduced.

## Empirical finite-time status

No finite empirical completion-time tolerance bound is reported. Two eligible episodes failed and are right-censored for completion time, so taking the maximum of only the two eligible successes would be survivor-biased. The aggregate correctly serialises `completion_time_bound.available=false`.

## Claim status

- **Empirically validated:** the exact 36-case manifest and the statistics above.
- **Conditionally supported:** two successful cases satisfied every runtime domain predicate; this is not enough to claim high conditional reliability.
- **Unsupported:** a finite empirical time bound, domain-wide 100% success, full-margin robustness over the matrix, or arbitrary-simple-shape unconditional convergence.

Raw reproducible artifacts:

- `runs/arbitrary_shape_stage5_final_36/manifest.json`
- `runs/arbitrary_shape_stage5_final_36/episodes.csv`
- `runs/arbitrary_shape_stage5_final_36/monte_carlo.json`
- `runs/arbitrary_shape_stage5_final_36/episodes/*/{summary.json,trajectories.csv,safety_timeseries.csv}`
