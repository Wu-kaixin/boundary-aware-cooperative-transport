# The conditional guarantee, v2 — predicates, and a four-way claim ledger

This document states what this branch claims, on what conditions, and what it declines
to claim. The conditions are written on **measurable predicates**, never on shape names:
"admissible simple polygon satisfying the predicates in §1" is a statement someone can
check on a new object, and "L-shapes and rectangles" is not.

Every number below is traceable to a committed file. §6 lists which.

The headline is a **conditional pass**. The supportable wording is:

> Conditional transport of an admissible simple polygon, at a scale-relative task
> distance, with measured failures on high-notch-count outlines and on small normalised
> targets.

The words **arbitrary shape** and **unknown shape** are not supportable and are not used.

---

## 1. The conditional domain, as predicates

These are the premises `src/dbact/guarantees.py` evaluates before a run. Each is a
computable function of the object, the team and the task. The values are the ones
declared in `configs/sim/v2/shape_matrix.yaml`.

| predicate | condition | declared value | why it is a premise |
| --- | --- | --- | --- |
| `simple_polygon` | outline is simple and non-degenerate | — | a self-intersecting outline has no inside, no cage offset, no boundary order |
| `feature_witness` | certified inscribed radius ≥ `min_feature_radius` | 0.04 m | the finite-ray detection tube is positive only if a disk fits |
| `perimeter_bound` | perimeter ≤ `max_perimeter` | 10.0 m | a fixed team cannot cover an unbounded boundary |
| `diameter_bound` | diameter ≤ `max_diameter` | 3.00 m | object, cage and swept corridor must fit the workspace |
| `boundary_covering_number` | `ceil(P / 2(R_cov − d_c))` ≤ team size | 16 robots | arclength plus the triangle inequality |
| `cage_offset_self_clearance` | facing offset-wall gap ≥ `d_min` | 0.28 m | two robots told to stand in one narrow slot cannot both stand |
| `swept_cargo_corridor` | start and end footprints inside the domain | ≥ 0 m | a rectangle is convex, so endpoints certify the interior |
| `swept_cage_corridor` | cage centres clear by `robot_radius` | — | the ring must fit at both ends |
| `contact_force_capacity` | Coulomb breakaway need ≤ `min_push_agents` | — | the gated quorum must be able to move the mass |
| `goal_wrench_feasibility` | a nonnegative zero-torque edge allocation exists | ≥ 1.05 × breakaway | pushing is unilateral; a pulling LP certifies nothing |
| `lane_spacing_cover` | half a lane ≤ finite-ray detection tube | — | not an ideal-disk-sensor assumption |
| `boundary_map_epsilon` | runtime map gap ≤ ε | 0.10 m | the post-discovery statement needs a dense map |
| `bounded_errors.normal_error_deg` | perception normal error ≤ bound | 30.0° | **measured, and violated — see §4** |
| `bounded_errors.velocity_error` | barrier-visible velocity error ≤ ρ | 0.02 m/s | **measured, and violated — see §4** |

The domain is additionally conditional on a **regime** the matrix held fixed and did not
explore: surface density 2.0, and the baseline ground and contact friction. Nothing here
supports a claim across mass or friction.

---

## 2. Proved

Statements that hold by construction or by an argument independent of any run.

| claim | where | note |
| --- | --- | --- |
| The two-tier-plus-scaled-barrier QP is feasible without a slack variable | `safety_filter._solve`, docstring | `u = 0` satisfies the inter-robot rows when `h_ij ≥ 0` and the object rows when `γ h_k ≥ n_k^T v_obj + ρ`. No soft penalty, so a reported zero violation is not a weight artefact. |
| The reachability cap leaves the object row family non-empty | `_cap_to_reachable` | Stated against an explicit witness `u* = f v_max w`, `w = normalize(Σ n_k)`, rather than a flat constant. |
| The ear-clipping inscribed radius is a constructive lower bound | `geometry.certified_inscribed_radius` | An ear triangle's incircle is contained in the polygon. Not a grid estimate. |
| The sampled map gap is a one-sided Hausdorff **upper** bound | `guarantees.boundary_map_gap_upper_bound` | Distance to a fixed point set is 1-Lipschitz in arclength, so `sampled max + P/(2n)` is rigorous. |
| The cyclic uncovered-arc bound is conservative | `metrics.maximum_uncovered_boundary_arc` | `(longest + 1) × resolution`: the true transition lies inside each bounding sampling interval. |
| Discrete-time CBF admissibility is enforced, not assumed | `SafetyFilter.__init__` | `γ_obj · dt ≤ 1` raises at construction. Above it the row asks for more decrease than a step can contain. |
| The net contact force lies in the convex cone of the press directions | §5, `analyse_lateral_authority` | Coefficients are nonnegative, so the achievable set is the convex hull of `{−n_k}`. This is what bounds lateral authority. |
| A rotationally symmetric object's yaw is unobservable from boundary geometry | `test_a_rotationally_symmetric_object_has_no_observable_yaw` | Rotation maps a disc's boundary onto itself; the point-to-plane residual is identically zero. No method reading the boundary can recover the rate. |

---

## 3. Conditional — holds given the §1 premises

| claim | condition | evidence |
| --- | --- | --- |
| Lane-swept discovery covers the workspace | `search_mode = sweep`, `lane_spacing_cover`, `workspace_edge_cover` | `guarantees.build_admissibility_certificate`, `search` group |
| Operational enclosure: enough boundary held, closely enough, to apply the required wrench | `boundary_covering_number`, `cage_offset_self_clearance`, `goal_wrench_feasibility` | `guarantees`, `task` group; `metrics.operational_enclosure_certificate` at runtime |
| Input-to-state-safe object-boundary barrier | `velocity_error ≤ ρ`, bounded normal error | **the condition is violated in practice — §4.** The statement is intact; its premise is not met. |
| The token relay can propagate at all | `lane_width ≤ comm_range` | Marked **NECESSARY, NOT SUFFICIENT** in the certificate itself. v1's relay is opportunistic; no claim is made that the graph is connected when it matters. |

---

## 4. Empirically verified — and what the measurements refuted

### 4.1 The decisive matrix, 180 episodes

12 families × α ∈ {0.1, 0.4, 0.8} × 5 seeds, `L = α · diameter`.

| quantity | value | 95% Wilson |
| --- | --- | --- |
| `J / diameter` | 0.470 ± 0.424 | — |
| — by α | 0.186 / 0.435 / 0.789 | — |
| `P(eligible)` | 149/180 = 0.828 | [0.766, 0.876] |
| `P(success)` | 54/180 = 0.300 | [0.238, 0.371] |
| `P(success | eligible)` | 41/149 = 0.275 | [0.210, 0.352] |
| reached HOLD | 138/180 | — |
| watchdog | 42/180 | — |

Failure composition: CONTRACT_FAILURE 70, SUCCESS 41, TRANSPORT_STALL 30,
COVER_INFEASIBLE 13, WRENCH_INFEASIBLE 12, TRANSPORT_NEVER_ARMED 8, MAP_INCOMPLETE 3,
SAFETY_VIOLATION 2, SOLVER_FAILURE 1.

Of the 31 ineligible episodes, 27 were rejected before the run and 4 at runtime, by the
map never reaching ε.

### 4.2 Displacement generalises; contract satisfaction does not

`J / diameter` rises cleanly with α (0.186 → 0.435 → 0.789) while the normalised
cross-track error triples (0.057 → 0.131 → 0.233) and enclosure timeouts stay at
identically zero. The binding constraint at high α is lateral error, not transport
authority.

### 4.3 Two families score 0/15, and concavity does not explain it

star10 0/15 and concave_random15 0/15, unchanged by the `explore_gain` control
experiment. **The attribution to concavity does not survive measurement.** Measured
mean concavity ratio:

| family | concavity | success |
| --- | --- | --- |
| star10 | 0.400 | 0/15 |
| c_shape | 0.352 | **11/15 — the best family in the matrix** |
| u_shape | 0.336 | 5/15 |
| concave_random15 | 0.250 | 0/15 |
| l_shape | 0.223 | 2/15 |
| polygon32 | 0.079 | 5/15 |
| concave_random7 | 0.077 | 1/15 |
| convex_random, rectangle, high_aspect, ellipse24, circle | 0.000 | 8, 1, 7, 8, 6 / 15 |

The most concave family scores zero and the second most concave scores best. What the
two failing families share is **many deep narrow notches** — ten alternating lobes, and
`radii[1::3] *= 0.55` — rather than one wide slot, and the area-ratio concavity does not
measure notch count or notch width at all. The matrix's own correlation, concavity vs
*peak coverage* ρ ≈ −0.70, is unaffected and is the narrower claim the data supports:
concavity hurts enclosure quality, and it does not order success.

### 4.4 The declared error premises are violated

Measured by `dbact.error_audit` over the 12 baseline seeds
(`docs/results/se2/se2_ablation.json`):

| term | declared | measured mean | share of cells over the bound | max |
| --- | --- | --- | --- | --- |
| `normal_error_deg` | 30.0° | 11.4° | **9.2%** | 180° |
| `velocity_error` (barrier projection) | 0.02 m/s | 0.051 m/s | **60.4%** | 0.534 m/s |
| `boundary_point_error_m` | — | — | — | 1.169 m |
| `map_gap_m` | ε = 0.10 m | — | — | 0.725 m |

`velocity_error: 0.02` is not violated by a thin tail. It is exceeded by the majority of
measured cells, by 2.6× in the mean and 27× at the worst — and 0.02 **is** ρ, the ISSf
margin the object rows are built with. All 12 seeds fail closed on it.

Two measurement caveats are recorded rather than smoothed away. At a convex vertex the
outward normal does not exist, so corner cells inflate the normal term irreducibly; and
on a rotating object the boundary-point and velocity terms are coupled by ω, so their
maxima are not independent budgets.

### 4.5 SE(2) boundary-point velocity: measured, and it made things worse

| | pass | J | efficiency | cross-track | direction error | barrier scalings |
| --- | --- | --- | --- | --- | --- | --- |
| `estimate_object_yaw: false` (v1) | 8/12 | 1.4908 | 0.9915 | 0.1857 | 6.25° | 68 |
| `estimate_object_yaw: true` | 7/12 | 1.5413 | 0.9846 | 0.2445 | 8.07° | 108 |

Kept, and **default off**. The baseline cargo's true rotation over an entire episode is
at most 0.086°; the estimator reports up to 2.63°, thirty times the truth, and the
audit's fifth term shows the cost: peak boundary-point velocity error rises from 0.303 to
0.595 m/s. On a near-non-rotating object the yaw term is noise, and noise × lever arm in
the barrier's right-hand side is strictly harmful.

### 4.6 The two unexplained cases are now located

**`rectangle__a0.10__seed004`** — the separation breach came **first**, at frame 2012,
and the first infeasible solve at frame 2020. So it is not a solver failure: the
inter-robot barrier is feasible at `u = 0` whenever `h_ij ≥ 0`, and once two robots are
inside `d_min` the row demands a separation rate the speed limit cannot deliver. The 124
infeasible solves are the consequence. The episode also travelled **6.29 m on a 0.214 m
task** with transport never armed, peak cargo speed 0.179 m/s, and strict coverage
collapsing from 1.000 to 0.319 — the ring lost the object it was pressing. Its minimum
inter-agent distance was 0.2038 m against `d_min` 0.28: **a 76 mm safety violation that
the taxonomy did not report**, because `classify` ranks SOLVER_FAILURE first and returns
on the first match.

**`polygon32` seed 2** — the reported "93° perpendicular push" is an artefact. The
displacement is of order 10⁻⁵ m and the direction error is `arccos(J/|dx|)` on two
numbers at the numerical floor. The finding is that the object never moved: 8–10 robots
met the push-set alignment test but only 3–4 ever pushed, against a quorum of 4, so
transport armed once (frame 2890, at α = 0.10) and never otherwise. α = 0.40 and α = 0.80
are bit-identical across every metric, which confirms the target distance never entered
the run.

### 4.7 The overshoot is a scale-invariant gain error, and the gate is what fails with alpha

`scripts/run_distance_ablation.py`, five alpha levels x 12 seeds on the baseline l_shape
(diameter 2.546 m), 60 episodes, zero fallbacks and separation held throughout.

| alpha | L (m) | pass | J/diam | **J/L** | direction error | **gate's implied limit** | peak coverage | over the gate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.2 | 0.509 | 9/12 | 0.253 | 1.266 | 6.39° | 13.52° | 0.984 | 1/12 |
| 0.4 | 1.018 | 8/12 | 0.497 | 1.243 | 6.04° | 6.83° | 0.979 | 7/12 |
| 0.6 | 1.527 | 8/12 | 0.741 | 1.236 | 5.09° | 4.57° | 0.979 | 7/12 |
| 0.8 | 2.036 | 5/12 | 0.986 | 1.233 | 5.83° | 3.44° | 0.969 | 7/12 |
| 1.0 | 2.546 | 4/12 | 1.278 | 1.278 | 7.81° | **2.65°** | 0.976 | **12/12** |

Two results, both new:

**The overshoot is one constant.** `J/L` is 1.23–1.28 at every alpha and
`corr(alpha, J/L) = +0.029`. The team travels about **24% further than asked**, whether asked
for 0.5 m or 2.5 m. v1 recorded the on-board progress estimate as biased low by 10–15%; this
measures ~24% and, more usefully, shows it is a scale-invariant *gain* error rather than an
offset — which is a fixable thing, being one constant.

**Success falls with alpha because the gate tightens, not because the control degrades.**
Peak coverage (0.969–0.984), efficiency (0.989–0.994), separation and solver behaviour are all
flat across the sweep. What moves is `arcsin(0.15 / J)`, the direction accuracy an absolute
0.15 m cross-track gate implies, which falls from 13.52° to 2.65° purely as arithmetic while
the measured direction error stays in a 5–8° band. At alpha = 1.0 all twelve episodes exceed
the gate while travelling 1.28 diameters at 0.989 efficiency.

`corr(alpha, J/diameter) = +0.992`: displacement tracks demand almost exactly, which is the
matrix's alpha finding reproduced on one shape at 12 seeds.

### 4.8 Robustness, and the premise that fails with a perfect sensor

Eight arms x 12 seeds (`docs/results/t5/robustness_ablation.json`). `nominal` reproduces v1 to
ten digits, and the plan's `range_noise_010` arm is **bit-identical** to it, because the
baseline config already sets `range_noise_std: 0.01`.

| arm | pass | J | worst separation slack | fallbacks | velocity-premise breach |
| --- | --- | --- | --- | --- | --- |
| `nominal` (= 10 mm) | 8/12 | 1.491 | +0.00012 | 0 | 0.603 |
| `range_noise_000` | 9/12 | 1.601 | **−0.00434** | 0 | **0.367** |
| `range_noise_020` | **0/12** | 0.761 | −0.00000 | 0 | 0.511 |
| `slow_updates_5` | 4/12 | 1.523 | +0.00003 | 0 | 0.567 |
| `comm_dropout_10` | **0/12** | 1.470 | **−0.05803** | 3 | 0.609 |
| `combined` | 3/12 | 1.699 | **−0.04385** | **592** | 0.563 |

Three results bear on the claim directly:

**The declared velocity premise fails with a noiseless sensor** — 36.7% of cells, against
nominal's 60.3%. So §4.4's violation is not attributable to range noise; it is the registration
and fusion pipeline, and no sensor improvement can bring it inside the bound. Measured
out-of-domain fires on **all eight arms**.

**The noise is load-bearing.** A perfect sensor breaches `d_min` by 4.3 mm and needs 389 barrier
scalings against nominal's 68, while achieving the best pass rate. The reading that fits is
dither: with zero noise every robot's returns quantise into the same voxels, the maps agree
exactly, the targets coincide, and robots converge onto each other. Recorded as a reading rather
than a mechanism — establishing it would need an ablation that decorrelates targets without
adding sensor noise.

**Robustness ends at 20 mm noise and at 10% link loss**, both 0/12, the latter with a 58 mm
separation breach. `combined` produces 592 fallbacks, a failure mode no single perturbation
produces.

**Pseudo-frontier**: every frontier target emitted after the pooled map satisfies ε is provably
spurious. Noise inflates the rate from 52.3 to 68.7 per frame between 0 and 20 mm, but 52.3 at
*zero* noise means the great majority is intrinsic to the tangential-neighbour predicate rather
than noise-induced.

### 4.9 Runtime performance

22.0–23.2 fps at 16 robots on the baseline over full until-settled episodes. On a fixed
400-frame budget with the machine otherwise quiet, `explore_gain: 0` runs at 28.77 ± 0.88 fps
and `explore_gain: 6` at 23.63 ± 2.49 fps — a paired per-frame cost ratio of
**1.226 ± 0.096**, about 23% more time per frame.

Reported as **machine-dependent empirical evidence, not a runtime bound**: one machine, one
Python, one BLAS. The three-fold larger spread on the `explore_gain: 6` arm indicates the
cost is state-dependent rather than a fixed overhead, which is a further reason it does not
transfer.

---

## 5. Lateral authority: what the cross-track gate really demands

The identity is confirmed at correlation **0.981** over 12 seeds, mean absolute residual
0.0174 m:

```
max cross-track  =  J · sin(direction error)
```

So the G500 gate `cross_track_max ≤ 0.15 m`, at these J, **is** the requirement "hold the
net force direction to within 5.94° for the whole push". The measured mean direction error
is 6.23°. The two gates are one gate.

The net force is a nonnegative combination of the press directions `{−n_k}`, so the
achievable set is their convex hull, an angular interval Φ. Measured over the transport
phase:

| quantity | value |
| --- | --- |
| distinct press directions in the push set | 3.07 on average |
| reachable half-width of Φ | **27.4°** |
| frames with the arc on a single normal | 12.4% |
| frames with the goal direction **outside** Φ | 33.1% |
| corr(direction error, reachable half-width) | **+0.909** |
| corr(direction error, goal-outside-Φ fraction) | **−0.777** |

Read on its own this looks like a refutation of v1's authority-saturation explanation: Φ is
**4.6× wider** than the gate needs, and the seeds whose goal most often lies *outside* Φ aim
*best* (the 7 seeds over the 0.15 m gate have the goal outside Φ on 12% of frames; the 5
passing seeds, 62%). The hypothesis that fits is dispersion rather than authority.

**The controlled test refutes that hypothesis.** `scripts/run_push_arc_ablation.py` raises
the membership threshold `τ` in `n_k · d_goal ≤ −τ`, narrowing Φ without touching any gain:

| `τ` | Φ half-cone | pass | direction error | cross-track |
| --- | --- | --- | --- | --- |
| 0.35 (default) | 69.5° | 8/12 | 6.25° | 0.1857 |
| 0.55 | 56.6° | 6/12 | 6.91° | 0.1995 |
| 0.75 | 41.4° | 6/12 | 7.19° | 0.2179 |

Narrowing Φ makes every measure monotonically worse, so **v1's authority-saturation
explanation stands** and the +0.909 correlation is a confound: in observational data Φ's
width varies because the sampled goal direction varies, and goal direction independently
sets episode difficulty. Both results are recorded because a reader who computes those
correlations should find them already noted together with the reason they mislead.

**The gate should still be restated on Φ, not on an absolute distance** — for v1's reason
rather than a new one. The defensible form is

```
cross-track  ≤  J · sin( dist(d_goal, Φ) + resolution )
```

where `dist(d_goal, Φ)` is zero when the goal is reachable and the angular shortfall
otherwise. Every term is measurable on board: each robot knows its own normal, and the
interval bounds max-consensus across the team exactly as the enclosure bitmap does.

---

## 6. Not claimed

Each of these is a statement someone could reasonably expect this work to make, and does
not.

### 6.1 Unconditional convergence for an arbitrary or unknown shape

Not claimed, and not close. `P(success) = 0.300`. Two of twelve families score 0/15. The
domain is the predicate list in §1, and the words *arbitrary* and *unknown* are barred
from this branch's write-ups.

### 6.2 Formal caging

Not claimed anywhere, and the non-claim is a constant rather than a computed field:
`guarantees.build_admissibility_certificate` returns `formal_caging: False` and
`metrics.operational_enclosure_certificate` returns the same with a
`formal_caging_nonclaim` string beside it. No input — admissible shape, dense map, full
quorum, complete exterior ring — flips either one. What is certified is operational
enclosure: enough boundary held, closely enough, to apply the wrench. That is not a proof
that the object cannot escape.

### 6.3 A positive analytic finite-time bound for the current controller

Not claimed. `derive_conditional_finite_time_bound` reports `available: False` on every
run in this repository, and lists why in machine-readable form: the three contraction
rates `enclosure_contraction_rate_hz`, `transport_progress_rate_mps` and
`brake_contraction_rate_hz` hold no independent certificate. The arithmetic is reported —
the numbers are what the bound *would* be — and `arithmetic_consistent: True` alongside
`available: False` is the honest state. Nothing in the repository passes
`contraction_rates_certified=True`; the only place it is ever passed is a test that says
in its own docstring that it holds no certificate.

### 6.4 An empirically confident completion-time bound

Not claimed, and deliberately not computed. 42 of 180 episodes ended on the watchdog and
139 of 180 failed the contract, so the observed completion times are **right-censored**:
any bound derived from the finishers is a bound on the finishers. The distribution is
reported; no bound is offered. `tests/test_monte_carlo_runner.py` asserts the absence, so
that a future summary cannot acquire one quietly.

### 6.5 Global zero slack

Not claimed. The QP carries no slack variable, so where it solves, the violation is
exactly zero — but 68 solves out of 64 576 on the baseline needed the scaled-barrier
tier, giving up part of the object barrier's decrease rate, with `min_barrier_scale`
reaching 0.0 on the matrix. The inter-robot rows stayed hard on every one of them. On the
matrix, 104 of 180 episodes reached that tier, 12 324 times in total, and one episode
(§4.6) went infeasible outright, 124 times, after robots had already broken `d_min`. The
S1 certificate failure rate is over its own criterion: 0.1054 against a stated `< 0.10`
(§6.8).

### 6.6 Hardware validity

Not claimed. Everything here is simulation. The perception layer is a ray-cast sensor
over the true polygons; the contact model is a penalty model; there is no actuator
dynamics, no latency, no state estimation drift and no wheel slip. The
`velocity_error` premise is already violated in simulation by 27× at the worst (§4.4),
which is the term hardware would make worse, not better.

### 6.7 That the predicate characterises the operational domain

**This is the most important non-claim on the list, and it is new in v2.**

The certificate is a conservative *filter*, and the matrix shows it is not a
*characterisation*:

| | successes | failures | rate |
| --- | --- | --- | --- |
| eligible | 41 | 108 | 0.275 |
| rejected | 13 | 18 | **0.419** |

`P(success | eligible) = 0.275` is **below** the unconditional `P(success) = 0.300`, and
13 of the 31 episodes the predicate rejected went on to succeed anyway. Fisher's exact
test on the 2×2 table gives an odds ratio of 0.526 with **p = 0.13**.

The honest reading, stated carefully:

* There is **no evidence that the predicate is informative** about success. Passing it
  does not raise the success rate.
* The point estimate points the *wrong way* — rejected cases succeeded more often — but
  with 31 rejected episodes that difference is **not statistically significant**. This
  branch does **not** claim the predicate is anti-informative; it claims the predicate has
  not been shown to be informative, which is a weaker and defensible statement.
* Either way, the predicate cannot be presented as a description of where the method
  works. It is a set of *necessary geometric conditions for the safety and wrench
  arguments to be stated at all*, and it is silent on whether the controller will then
  succeed.

What follows for the paper: eligibility may be reported as a precondition for the
conditional safety statement. It may **not** be reported as an operational envelope, and
`P(success | eligible)` must be reported next to `P(success)` so a reader can see that
conditioning bought nothing.

### 6.8 That the S1 certificate criterion is met

Not claimed. The zero-input certificate failure rate is **0.1054** against a stated S1
criterion of `< 0.10`. The threshold has not been touched and will not be. See
`CLOSED_LOOP_V2.md` §3 for the decision taken: the criterion is rewritten with its reason
stated, rather than the number being moved.

### 6.9 That far-field enclosure is solved

Not claimed. The boundary is: **discovery is solved, post-discovery enclosure is not.**
`l_shape_search.yaml` finds a randomly placed object in 75 ± 80 frames against a ~510-frame
coverage bound — that is the solved half. Contact-ready then takes 344 ± 135 frames
against a 75 ± 8 ring start, four to five times longer; one seed of eight got worse under
the D10 change, and peak strict coverage is still under 0.60 on three of eight seeds. The
claim is about finding the object, not about closing a ring around it once found.

### 6.10 That the taxonomy reports causes rather than symptoms

Not claimed, and §4.6 is the counterexample. `classify` returns on its first match, so
`rectangle__a0.10__seed004`'s 76 mm `d_min` breach is reported as SOLVER_FAILURE and the
safety violation does not appear in the composition table at all. The ordering is
defensible — a pre-run rejection should not be labelled by what the run then did — but a
single label per episode cannot represent an episode that failed two ways, and the
composition table should be read as "the most structural cause found", not as a partition
of failure modes.

---

## 7. Provenance

| claim group | file |
| --- | --- |
| decisive matrix, 180 episodes | `docs/results/v2_shape_matrix/{episodes.csv, monte_carlo.json, manifest.json, REPORT.txt}` |
| `explore_gain` control, 45 episodes | `docs/results/v2_control_explore_gain0/{episodes.csv, monte_carlo.json, manifest.json}` |
| 12-seed baseline | `docs/results/v2_baseline_12seed/g500_sweep.json` |
| SE(2) arms and the six-term audit | `docs/results/se2/se2_ablation.json` |
| lateral authority and the reachable cone | `docs/results/t3/lateral_authority.json` |
| push-arc threshold ablation | `docs/results/t3/push_arc_ablation.json` |
| the two unexplained cases | `docs/results/t4/{FINDINGS.txt, t4_traces.json}` |
| `explore_gain` frame-rate cost | `docs/results/t7/explore_gain_profile.json` |
| every deleted line under `src/` | `docs/SE2_DIFF_AUDIT.md` |

Reproduction:

```bash
python -m pytest -q
```

```bash
python scripts/verify_refactor.py
```

```bash
python scripts/evaluate_closed_loop.py --seeds 0..11 --until-settled --out runs/v2_sweep
```
