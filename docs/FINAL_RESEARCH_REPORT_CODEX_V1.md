# DBACT Codex closed-loop v1 research report

Date: 2026-08-09

Branch: `Codex-boundary-aware-closed-loop-v1`

Base: `A-boundary-aware-closed-loop-v3`

## Outcome

This branch replaces the fixed 500-frame objective with a fail-closed episode
contract:

```text
SEARCH -> MAP -> ENCLOSE -> TRANSPORT -> BRAKE -> HOLD
```

An episode runs until HOLD, an explicit classified failure, or a configured
timeout. The controller never writes cargo pose or velocity. Cargo motion is
produced only by simulated robot contact.

The branch is a simulation-complete research implementation and a reproducible
evidence package. It is **not** yet a universal arbitrary-shape solution: the
final unscreened matrix has a low eligible fraction and eligible failures, so a
finite empirical upper bound and domain-wide convergence claim are correctly
withheld.

## Implemented system

### Closed-loop progress and wrench control

Transport uses activation-relative directional progress

```text
J = (x_hat_object - x_hat_activation)^T d_goal
v_parallel = v_hat_object^T d_goal
e_J = J_target - J
e_v = v_ref - v_parallel.
```

A saturated PI/adaptive-pressure loop with anti-windup regulates contact effort.
One-hop decentralized wrench weights seek goal-aligned net force and low net
torque. A persistent progress watchdog produces `TRANSPORT_STALL`; task and
brake corridors can produce `GOAL_CORRIDOR_INFEASIBLE`; BRAKE and HOLD have
explicit position/speed/streak gates.

### Robust safety closure

The hard safety QP runs every physics frame. Perception and planning are
multi-rate, while raw local safety scans remain independent of planning-map
gossip. The object CBF uses closest polyline features and SE(2) boundary-point
velocity, including angular motion. Ground-truth audits measure, but never feed
back to the controller:

- boundary-normal and boundary-point error;
- runtime map error;
- object translational-velocity error;
- rigid boundary-point velocity error; and
- CBF normal-velocity projection error.

Noise, normal error, update-rate variation, and communication dropout are
actually injected by the simulator. Declared error-bound violations fail
closed instead of being hidden.

### Operational enclosure

The enclosure certificate jointly requires strict exterior boundary coverage,
a bounded maximum uncovered arc, all robot centres outside the cargo,
inter-agent separation, cage-offset feasibility, and an engaged/contact quorum.
It is an **operational boundary enclosure**, not a formal configuration-space
caging proof. `formal_caging` is always reported as false.

### Arbitrary-shape conditional domain

The theorem domain is predicate-based rather than a shape whitelist. It covers
simple, nondegenerate, bounded-perimeter objects when the complete-workspace
sensing, epsilon-dense map, cage/corridor, contact wrench, error bound, safety,
and positive contraction/progress premises all hold. Ineligible shapes remain
simulatable and remain in empirical denominators.

## Quantitative evidence

### Audited representative closed loop

The retained circle/seed-0 truth-audit episode reached HOLD at frame 327:

| Metric | Value |
|---|---:|
| directional progress `J` | 0.118668 m |
| directional efficiency | 0.998285 |
| maximum cross-track error | 0.006959 m |
| maximum absolute cargo rotation | 4.036 deg |
| QP solves | 5886 |
| fallback / infeasible / rho relaxation | 0 / 0 / 0 |
| max boundary-point error / bound | 0.018395 / 0.023 m |
| max runtime-map error / bound | 0.163481 / 0.270 m |
| max object-velocity error / bound | 0.329875 / 0.350 m/s |
| max boundary-point velocity error / bound | 0.329881 / 0.350 m/s |
| max CBF projection error / bound | 0.329867 / 0.350 m/s |

This is an audited witness, not a probability statement.

### Robustness ablation

Nominal, 5 mm range noise, five-frame update interval, 10% communication
dropout, and the combined perturbation all reached HOLD with zero fallback,
infeasibility, and rho relaxation. The excluded 10 mm range-noise stress case
exceeded the boundary-error premise and timed out; it is rejected rather than
reported as a survivor.

### Longer-distance transport

Seed 0 reached HOLD for requested 0.10, 0.25, and 0.50 m activation-relative
tasks. The 0.10 m case satisfied all declared measured-error bounds and full rho
margin. The 0.25 m case retained full rho margin but exceeded the runtime map
bound. The 0.50 m case exceeded the map bound and used 414 rho relaxations.
Consequently, only 0.10 m is a fully audited bound-compliant distance result;
the longer runs are empirical transport witnesses.

### Unscreened arbitrary-shape matrix

The final 60-episode matrix reports all cases:

| Statistic | Result |
|---|---:|
| `P(eligible)` | 7/60 = 0.1167, Wilson 95% [0.0577, 0.2218] |
| `P(success | eligible)` | 2/7 = 0.2857, Wilson 95% [0.0822, 0.6411] |
| `P(rejected)` | 53/60 = 0.8833, Wilson 95% [0.7782, 0.9423] |
| task-contract successes, all cases | 14/60 |
| fallback-free / infeasible-free | 58/60 / 58/60 |
| rho-relaxation-free | 55/60 |

The dominant classified outcomes are `MAP_INCOMPLETE` (28) and
`WRENCH_INFEASIBLE` (21). This is negative as well as positive evidence and is
not corrected by weakening thresholds.

### Runtime

The fixed three-case performance ablation improved from 16.65 mean fps to
28.43 mean fps (+70.71%). Optimized rates were 35.49 fps for circle, 26.65 fps
for a 7-vertex concave case, and 23.15 fps for a 32-vertex case. Safety QP
frequency and theorem predicates were unchanged. Timing is machine-dependent
empirical evidence, not a theoretical runtime bound.

## Finite-time result

The implementation derives the sufficient decomposition

```text
T_total <= T_search + T_map + T_enclose + T_drive + T_brake + T_hold
```

with geometric search/map bounds and conditional phase terms

```text
T_enclose <= log(E0 / E_tol) / lambda_e
T_drive   <= max(0, L - e_brake) / v_progress_min
T_brake   <= log(e_brake,0 / e_hold) / lambda_b.
```

This is a mathematically valid sufficient bound only after positive
domain-wide rates `lambda_e`, `v_progress_min`, and `lambda_b` are independently
certified under the safety filter and communication schedule. Those constants
are not currently proved, so the current analytic certificate returns
`available=false`.

An empirical upper-confidence completion bound is also unavailable: five of
seven eligible Monte Carlo episodes failed and are right-censored. Successful
survivors are not used to manufacture a finite bound.

## Claim ledger

### Mathematically proved

- Complete rectangular-workspace detection under the finite-ray,
  contained-feature, lane-spacing, speed, and bounded-workspace premises.
- The algebraic finite-time phase composition under the explicitly stated
  positive contraction/progress premises.
- Hard CBF row construction and robust-margin implication when the adopted
  sensing/velocity error bounds hold and the hard QP remains feasible.

### Conditionally guaranteed

- Discovery, operational enclosure, contact transport, braking, and HOLD for
  admissible simple shapes satisfying every executable geometry, mapping,
  corridor, wrench, error, safety, and rate predicate.
- The condition is on measurable predicates, not on shape-family names.

### Empirically validated

- One full truth-audited closed-loop witness with all declared error and solver
  margins satisfied.
- Five retained robustness variants plus one rejected out-of-domain stress case.
- 0.10/0.25/0.50 m seed-0 contact-transport witnesses, with the stated audit
  distinctions.
- The unscreened 60-episode arbitrary-shape matrix and its failure composition.
- A >20 fps minimum on the fixed three-case optimized performance ablation.
- Fourteen PNG and fourteen PDF figures, raw CSV/JSON, a manifest, and a
  representative GIF.

### Unsupported or remaining limitation

- Unconditional success for every simple planar shape or every workspace pose.
- Formal caging or a configuration-space escape theorem.
- A positive analytic finite-time bound for the current controller.
- A finite empirical upper-confidence completion-time bound.
- Domain-wide fallback/infeasibility/rho-relaxation counts of zero.
- Persistent map-bound compliance at 0.25 and 0.50 m, or full rho margin at
  0.50 m.
- High conditional success probability: only seven eligible samples exist and
  five failed.
- Hardware validity; this branch intentionally performs no robot experiment.
- MP4 output in the retained environment because ffmpeg is unavailable. The
  reproducible GIF is included and the script emits MP4 when an ffmpeg writer is
  installed.

## Reproduction and artifacts

```powershell
python scripts/run_publication_representative.py `
  --output artifacts/publication/representative `
  --animation-stride 5 --animation-fps 16 --skip-mp4

python scripts/run_arbitrary_shape_monte_carlo.py `
  --seeds 0..4 --max-steps 1500 `
  --output runs/arbitrary_shape_final_se2_60

python scripts/generate_publication_artifacts.py `
  --monte-carlo runs/arbitrary_shape_final_se2_60/monte_carlo.json `
  --output artifacts/publication
```

Primary retained outputs:

- `artifacts/publication/publication_manifest.json`
- `artifacts/publication/figures/` (14 PNG and 14 PDF figures)
- `artifacts/publication/tables/`
- `artifacts/publication/data/`
- `artifacts/publication/representative/closed_loop.gif`
- `runs/arbitrary_shape_final_se2_60/{manifest.json,episodes.csv,monte_carlo.json}`
