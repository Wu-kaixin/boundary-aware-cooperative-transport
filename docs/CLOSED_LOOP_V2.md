# Closed loop, v2 — what was ported onto v1, what it cost, and what it refuted

This continues `docs/CLOSED_LOOP_D.md`. That document ends with a list of open items;
this one reports what happened to them, plus the decisive shape-matrix experiment and the
verification work that followed it.

The structure is deliberate. Every section states what was measured, what the number was
before, what it is now, and — where the answer was "worse" or "the premise was wrong" —
says so in the section rather than in a footnote. Four negative results from v1 are
carried forward unchanged in §8, and five new ones are recorded in the same format.

The claim this branch supports is in `docs/CONDITIONAL_GUARANTEE_V2.md`. It is a
**conditional pass**, and the words *arbitrary shape* and *unknown shape* are not used.

---

## 0. Provenance, and one commit that does not meet the branch's own convention

Branch `Claude-boundary-aware-closed-loop-v2`, from
`Claude-boundary-aware-closed-loop-v1` at `92ee6f6`.

Commit **`ab8f750`** carries the message `checkpoint before checking out main`. It is the
commit that introduced `src/dbact/guarantees.py`, `configs/sim/v2/shape_matrix.yaml` and
`scripts/run_arbitrary_shape_monte_carlo.py` — the whole CODEX port that the decisive
matrix was run on. The message was generated automatically by an external process
operating on the shared repository, not written for this branch, and it does not meet the
`port(codex):` convention every other commit here follows. **The content is correct and
has been verified; only the message is non-conforming.** It has not been amended, because
the branch is pushed and rewriting it needs `--force-with-lease`. That is left as the
user's decision, recorded here rather than quietly fixed.

Test count: 301 at `d5ce40a`, 433 now. `scripts/verify_refactor.py` reports all requested
stages PASS. `src/dbact/phase.py` is byte-identical to v1 and the seven-phase machine is
unchanged, with no MAP phase.

---

## 1. The decisive matrix: does the v1 controller generalise?

`configs/sim/v2/shape_matrix.yaml` is `configs/sim/d/l_shape_closed_loop.yaml` with four
declared changes and nothing else — a 10 × 10 domain rather than 8 × 8, per-case cargo and
annulus, `L = α · diameter` rather than CODEX's fixed 0.10 m, and a `guarantee` block.
Every controller parameter is byte-identical to the baseline, because a controller retuned
per shape answers a different question.

12 families × α ∈ {0.1, 0.4, 0.8} × 5 seeds = 180 episodes.

| quantity | value | 95% Wilson |
| --- | --- | --- |
| `J / diameter` | 0.470 ± 0.424 | — |
| — by α | 0.186 / 0.435 / 0.789 | — |
| `P(eligible)` | 149/180 = 0.828 | [0.766, 0.876] |
| `P(success | eligible)` | 41/149 = 0.275 | [0.210, 0.352] |
| `P(success)` | 54/180 = 0.300 | [0.238, 0.371] |
| reached HOLD | 138/180 | — |
| watchdog | 42/180 | — |

Failure composition: CONTRACT_FAILURE 70, SUCCESS 41, TRANSPORT_STALL 30,
COVER_INFEASIBLE 13, WRENCH_INFEASIBLE 12, TRANSPORT_NEVER_ARMED 8, MAP_INCOMPLETE 3,
SAFETY_VIOLATION 2, SOLVER_FAILURE 1.

**Displacement generalises; contract satisfaction does not.** `J / diameter` rises cleanly
with α while the normalised cross-track error triples (0.057 → 0.131 → 0.233) and
enclosure timeouts stay at identically zero. What fails at high α is lateral accuracy, not
transport authority.

The regression correlations, computed per α so that α is not the confound:

| pair | result |
| --- | --- |
| diameter vs `J/L` | ρ ≈ +0.5 — bigger objects transport better |
| diameter vs peak coverage | ρ ≈ −0.44 — bigger objects are enclosed worse |
| concavity vs `J/L` | p > 0.39 — no relationship |
| concavity vs peak coverage | ρ ≈ −0.70, p < 1e-8 — concavity hurts enclosure only |

---

## 2. The `explore_gain` confound, and the control experiment that resolved it

`shape_matrix.yaml` ran with `explore_gain: 6.0`; the 12-seed baseline runs with the
default `0.0`. That is a second difference between the two experiments, so a per-family
success rate from the matrix could not be attributed to shape alone. The control
experiment `configs/sim/v2/shape_matrix_eg0.yaml` (45 episodes, three families ×
α{0.1,0.4,0.8} × 5 seeds) is identical but for that one field:

| family | `explore_gain = 0` | `explore_gain = 6` |
| --- | --- | --- |
| l_shape | 2/15 | **5/15** |
| star10 | 0/15 | 0/15 |
| concave_random15 | 0/15 | 0/15 |

So the term matters on l_shape and changes nothing on the two families that score zero.
Those two zeros are a property of the configuration-independent behaviour, not of
`explore_gain`. The term is also **not** a solver effect: fallbacks were 0 in both arms.

### 2.1 The frame-rate cost

See §5.

---

## 3. Three v1 debts

### 3.1 The S1 certificate rate is over its own criterion — the criterion is rewritten, not the number

| | A branch | D branch | stated S1 criterion |
| --- | --- | --- | --- |
| zero-input certificate failure rate | 0.0415 | **0.1054** | `< 0.10` |

The threshold has not been touched. The decision taken is to **rewrite the criterion with
its reason stated**, and the reason is that the quantity does not mean the same thing in
the two configurations being compared.

The rate is the fraction of solves on which `u = 0` does not satisfy the margin-free
barrier, so the robot must actively retreat. That is not a safety failure — every hard
invariant held, and clearance and penetration are both *better* than the A branch — it is
a statement about how often a robot sits inside the ISSf band. And a robot sits inside the
band precisely when the boundary it is standing off is **moving**. The A branch's cargo
was stationary from frame 97; the D branch's moves for the whole episode. Comparing the
two rates compares a mostly-static scene against a moving one and reports the difference
as a regression.

The criterion is therefore restated as:

> **S1-rate (v2).** The zero-input certificate failure rate is reported, not gated, and is
> reported **separately for frames in which the estimated object speed exceeds
> `0.2 · max_object_speed` and for frames in which it does not.** The gated quantities
> remain the hard invariants: robots inside the cargo `== 0`, minimum signed clearance
> `≥ 0`, maximum penetration `≤ budget`, maximum slack `== 0`. A single scalar rate over a
> mixed-motion episode is not a quantity a threshold can be set on.

Two things this does **not** do. It does not re-derive the ISSf constant against a moving
boundary — that work is not done, and §4.4 of `CONDITIONAL_GUARANTEE_V2.md` records that
the ISSf premise `velocity_error ≤ ρ` is in fact violated by 27× at the worst, which is a
larger problem than the rate. And it does not relax a gate: the four hard invariants stay
exactly as they were, and the rate moves from "gated on a number chosen for a static
scene" to "reported, split by the condition that drives it".

### 3.2 Cross-track: the identity holds, and the received explanation is wrong

The identity is confirmed at **correlation 0.981** over 12 seeds, mean absolute residual
0.0174 m:

```
max cross-track  =  J · sin(direction error)
```

So `cross_track_max ≤ 0.15 m` at these J **is** "hold the net force direction to within
5.94°". Measured mean direction error: 6.23°. The loop sits just over the gate's implicit
requirement, exactly as v1 recorded.

v1 attributed this to **authority saturation** — the press is along each robot's own
observed normal, so the only steering authority is how much each robot presses, and on a
faceted object the available normals are discrete.
`scripts/analyse_lateral_authority.py` measured the reachable set, and the observational
numbers appeared to refute that. They do not, and the sequence is recorded in full because
the wrong conclusion was reachable from the correlations alone.

The net force is a nonnegative combination of the press directions `{−n_k}`, so the
achievable set is their convex hull: an angular interval Φ. Over the transport phase, 12
seeds:

| quantity | value |
| --- | --- |
| distinct press directions in the push set | 3.07 on average |
| reachable half-width of Φ | **27.4°** |
| frames with the arc on a single normal | 12.4% |
| frames with the goal direction outside Φ | 33.1% |
| corr(direction error, reachable half-width) | **+0.909** |
| corr(direction error, goal-outside-Φ fraction) | **−0.777** |

Read on its own this says authority is not binding: Φ is 4.6× wider than the gate needs,
and the 7 seeds over the 0.15 m gate have the goal outside Φ on 12% of frames against 62%
for the 5 passing seeds. Seeds that apparently *cannot* aim at the goal aim *better*. The
hypothesis that fits is **dispersion**: Φ's half-width measures how spread out the arc is,
and the sensitivity of the net force direction to an allocation error scales with the span
being combined.

**The controlled test refutes the dispersion hypothesis and vindicates v1.** See §4.

**The gate should still be restated on Φ**, and now for v1's reason rather than a new one.
The defensible form is

```
cross-track  ≤  J · sin( dist(d_goal, Φ) + resolution )
```

with `dist(d_goal, Φ)` zero when the goal is reachable. Every term is measurable on
board — each robot knows its own normal, and the interval bounds max-consensus across the
team exactly as the enclosure bitmap does — so this is a gate the team could evaluate on
itself, which an absolute 0.15 m cannot be. What §4 changes is the *sign* of the expected
effect of widening Φ, not whether Φ is the right quantity to write the gate over.

### 3.3 Far field: where the boundary lies

Not solved, and not attempted here. The boundary is stated plainly:

* **Discovery is solved.** `l_shape_search.yaml` places the object at random with every
  robot outside its own sensor range and finds it in **75 ± 80 frames** against a ~510-frame
  coverage bound.
* **Post-discovery enclosure is not.** Contact-ready then takes **344 ± 135 frames**
  against **75 ± 8** for a ring start — four to five times longer. One seed of eight got
  worse under the D10 change. Peak strict coverage is still under 0.60 on three of eight
  seeds.

Any claim on this branch about far-field operation is a claim about *finding* the object.
It is not a claim about closing a ring around it once found.

---

## 4. The controlled test: v1 was right, and the correlation was confounded

`scripts/run_push_arc_ablation.py` varies the push-set membership threshold `τ` in
`n_k · d_goal ≤ −τ`. Raising `τ` admits only robots whose own normal opposes the goal more
directly, which narrows Φ **without touching any gain**. The two readings make opposite
predictions: under dispersion the aim improves as `τ` rises; under authority saturation it
degrades, because a narrower Φ is less authority.

12 seeds per arm, everything else identical:

| `τ` | Φ half-cone at the membership limit | pass | transport armed | direction error | cross-track | over the 0.15 m gate | J | barrier scalings |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **0.35** (default) | 69.5° | **8/12** | 12/12 | **6.25°** | **0.1857** | 7/12 | 1.491 | 68 |
| 0.55 | 56.6° | 6/12 | 12/12 | 6.91° | 0.1995 | 7/12 | 1.475 | 117 |
| 0.75 | 41.4° | 6/12 | **11/12** | 7.19° | 0.2179 | 7/12 | 1.576 | 58 |

**Narrowing Φ makes the aim monotonically worse.** Direction error rises 6.25 → 6.91 →
7.19°, cross-track rises 0.186 → 0.200 → 0.218, the pass rate falls 8 → 6 → 6, and at
`τ = 0.75` one seed never arms transport at all — the quorum failure mode that
`polygon32` seed 2 already exhibits at the default threshold (§7.2).

So **the dispersion hypothesis is refuted and v1's authority-saturation explanation
stands**, now supported by a controlled experiment rather than by reasoning about facets.

The confound, stated plainly because it is the lesson: in the observational data Φ's width
varies *because the sampled goal direction varies*. Some goal directions put the trailing
arc on one of the L's flat faces and others across a corner, and goal direction
independently determines how hard the episode is. Cone width and difficulty therefore share
a cause, and the +0.909 correlation is that shared cause rather than a mechanism. The
ablation holds the goal fixed per seed and varies only membership, which is the comparison
that answers the question.

Two things follow for the write-up. The correlational result is **retained** in §3.2 rather
than deleted, because a reader who computes those correlations should find them already
recorded together with the reason they mislead. And the earlier claim in this document's
own draft — that the authority explanation "does not survive" — was wrong and is corrected
here rather than silently edited away.

`τ = 0.35` is confirmed as the better setting of the three tested. It is not proposed as
tuned: it is the value v1 already had, and the ablation's finding is that neither
alternative improves on it.

---

## 5. `explore_gain = 6`: the frame-rate cost

6 seeds, 400 frames each, both arms timed in the same process and **alternated seed by
seed** so a thermal drift or a background task perturbs both arms rather than one. A fixed
frame count rather than run-until-settled, because timing an until-settled run measures
episode *length* as much as per-frame cost and the two arms do not settle at the same frame.

| arm | frame rate | per frame |
| --- | --- | --- |
| `explore_gain: 0.0` | 28.77 ± 0.88 fps | 34.79 ms |
| `explore_gain: 6.0` | 23.63 ± 2.49 fps | 42.70 ms |

**Paired per-seed cost ratio: 1.226 ± 0.096** on per-frame time. So the term costs roughly
**23% more wall-clock per frame**, and about 18% of the frame rate — and it buys l_shape
2/15 → 5/15 while doing nothing for the two families that score zero (§2).

The spread on the `explore_gain: 6` arm is nearly three times the spread on the control
(± 2.49 vs ± 0.88 fps), which is itself informative: the cost is state-dependent, not a
fixed overhead. The term's work scales with how much frontier the robot's own map currently
has, so an episode that spends longer with a partial map pays more.

This is a stopwatch reading on one machine, in one Python, against one BLAS. It is
**machine-dependent empirical evidence and not a runtime bound**, it is not a complexity
result, and it does not transfer. It is recorded because "explore_gain is free" and
"explore_gain costs a third of the frame rate" are different facts about the same
configuration, and the branch reports 22.0 fps as a headline without saying which arm
produced it.

---

## 6. What was ported, and the two things the port found wrong

### 6.1 `guarantees.py` had 870 lines and no test

It was the one thing on this branch that had been delivered without being checked, and it
is on the critical path: `scripts/run_arbitrary_shape_monte_carlo.py` builds every
episode's certificate from it. CODEX's three test files were ported and re-stated over the
v1 API; 89 tests added.

Three CODEX tests could not survive the port and were replaced rather than dropped:

| CODEX test | why it could not be kept | replacement |
| --- | --- | --- |
| finite-time bound asserted on `eligible` | v1 returns `available`, additionally gated on `contraction_rates_certified`. Keeping the old assertion would have silently stopped testing the gate that matters. | arithmetic and certification gate asserted separately; the bound is arithmetically consistent and still unavailable |
| `paired_sweep` two-lane-chain layout | v1 walks a single static boustrophedon lane partition. CODEX's premises are false for every v1 run for a reason that has nothing to do with the object. | premises re-stated over v1's lane geometry — same theorem shape, v1's constants |
| `empirical_completion_bound` | no v1 counterpart, deliberately: with eligible failures present the completion times are right-censored | the testable property is an **absence**, and is tested as one |

**Finding: `minimum_facing_cage_clearance` failed open on its own worst case.** The facing
test ran on the *offset* midpoints. Offsetting moves each midpoint outward, so the offset
separation is roughly the wall separation minus `2 × offset`; once a slot was narrower than
that, the test flipped sign and the pair was skipped — returning `inf`, indistinguishable
from a convex outline. **A 0.36 m slot at a 0.20 m cage offset certified as clear.** Facing
is a property of the walls, so the test now runs on the un-offset midpoints and crossed
pairs report a negative width. The change is one-directional: every pair the old test
admitted is still admitted and measured identically, so nothing that used to be rejected is
now accepted.

**Finding: the matrix harness's annulus margin was overstated by 6×.** The comment claimed
0.35 m of slack past the object's reach. The annulus is centred on the workspace centre
while `reach` is measured from the object's centroid, and for an asymmetric family those
differ by up to **0.290 m** (l_shape). The realised worst-case clearance is **0.0605 m**.
No episode started with a robot inside an object and `assert_initial_state_valid` passed on
all 180, so the matrix stands; the four asymmetric families simply started with less room
than intended, l_shape with the least. Comments corrected; the placement deliberately
**not** changed, because changing it would invalidate the committed 180 episodes.

Also ported into `metrics.py` — audit-only, already licensed to read truth —
`maximum_uncovered_boundary_arc` and `operational_enclosure_certificate`. v1 measured mean
coverage but never the longest uncovered arc, and mean coverage cannot separate a team
spread evenly from a team piled on one side. Only the second has an opening the object can
leave through.

### 6.2 SE(2) boundary-point velocity in the CBF — measured, and worse

The object row is built on `h_k = n_k^T (p_i − b_k) − r_safe`, whose exact derivative
carries the velocity of the **material point** at `b_k`. For a rigid body that is
`v_c + ω R90 (b_k − c)`, not `v_c`. Registration now estimates the yaw rate as a third
unknown in the same point-to-plane least squares, about the **map's own centroid** — never
a simulator pose — and each barrier row carries its own point's velocity. The aggregate row
takes the **maximum** `n̄^T v_k` over the face rather than the weighted mean, because a row
standing for a whole face must demand at least as much retreat as its fastest-approaching
point.

12 seeds, one config field apart:

| | pass | J | efficiency | cross-track | direction error | barrier scalings | fallback / infeasible |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `estimate_object_yaw: false` (v1) | 8/12 | 1.4908 ± 0.2450 | 0.9915 | 0.1857 | 6.25° | 68 | 0 / 0 |
| `estimate_object_yaw: true` | 7/12 | 1.5413 ± 0.2470 | 0.9846 | 0.2445 | 8.07° | 108 | 0 / 0 |

**The gate fails** on `cross_track_not_worse` and `barrier_scalings_not_worse`. Per the
plan, the estimate is kept and `estimate_object_yaw` **defaults to false**. With it off the
registration keeps two unknowns, no stored normal is rotated, and the boundary-point
velocity reduces exactly to the translational one — the 12-seed sweep reproduces v1's
J = 1.4908 and 68 barrier scalings, which is the check that the off path *is* v1.

The reason is measurable, and the audit supplies it. The baseline cargo's **true** rotation
over an entire episode is at most **0.086°**. The estimator reports up to **2.63°** —
thirty times the truth — and the audit's fifth term shows the cost: peak boundary-point
velocity error rises from 0.303 to 0.595 m/s. **The correction is larger than the quantity
it corrects.** On a near-non-rotating object the yaw term is noise, and noise × lever arm in
the barrier's right-hand side is strictly harmful.

Two limits on what this arm could ever have established, both worth stating because they
bound the value of any future attempt:

* With true rotation under a tenth of a degree there is nothing on the baseline for the
  term to capture. The gate can only test for **harm**, never for benefit. The shape matrix
  is where rotation is present and where the term would have to earn its keep.
* **A rotating disc is geometrically unobservable.** Rotation about the centre maps a
  circle's boundary onto itself, so the point-to-plane residual is identically zero and *no*
  method reading boundary geometry can recover the rate. `circle`, `ellipse24` and
  `polygon32` are all near-symmetric, so a quarter of the twelve families are out of reach
  by construction.

`docs/SE2_DIFF_AUDIT.md` justifies all 23 deleted lines under `src/` segment by segment.
Four of the seven touched files have zero deletions.

### 6.3 The six-term error audit, and the premise it retired

`configs/sim/v2/shape_matrix.yaml` declares `normal_error_deg: 30.0` and
`velocity_error: 0.02` with the comment **DECLARED, NOT YET VERIFIED**. The certificate is
conditional on them and nothing measured them. `src/dbact/error_audit.py` measures those
two and four more that the same barrier row depends on and that nobody had written down:

1. `normal_error_deg` — tilts the half-plane
2. `boundary_point_error_m` — translates it along its own normal
3. `map_gap_m` — the boundary that has no row at all
4. `object_velocity_error_mps` — the term v1 had
5. `point_velocity_error_mps` — the material-point velocity the barrier needs
6. `normal_projection_error_mps` — `|n̂^T v̂ − n^T v|`, the only one the constraint sees

Measured over 12 seeds:

| term | declared | mean | share of cells over the bound | max |
| --- | --- | --- | --- | --- |
| `normal_error_deg` | 30.0° | 11.4° | **9.2%** | 180° |
| `velocity_error` (projection) | 0.02 m/s | 0.051 m/s | **60.4%** | 0.534 m/s |
| `boundary_point_error_m` | — | — | — | 1.169 m |
| `map_gap_m` | ε = 0.10 m | — | — | 0.725 m |

**`velocity_error: 0.02` is not violated by a thin tail.** It is exceeded by the majority
of measured cells, by 2.6× in the mean and 27× at the worst — and 0.02 **is** ρ, the ISSf
margin the object rows are built with. All 12 seeds fail closed on it. The certificate's
`bounded_perception_and_motion_error` check passes only because it tests that the declared
numbers are internally consistent (`< 90°`, `≤ ρ`), never that they are achieved.

The breach *fraction* is reported because a maximum alone cannot distinguish a wrong
premise from a handful of pathological cells, and the first version of this audit reported
a 180° maximum with no way to tell which it was.

Two artefacts are recorded rather than smoothed away:

* At a convex vertex the outward normal does not exist — the incident edges disagree by the
  exterior angle — so corner cells inflate term 1 irreducibly. This is one reason the
  declared *velocity* premise is checked against term 6, which is continuous across a
  corner, rather than against term 5.
* On a rotating object terms 2 and 5 are coupled by ω: a robot with an exact twist estimate
  still evaluates the velocity field at the cell it *believes* the boundary occupies, so a
  cell displaced by `d` yields a velocity wrong by `ω d`. Their maxima are not independent
  budgets. With ω = 0 the coupling vanishes, which is the regime the baseline runs in.

Truth isolation is asserted **by import graph**, at every import depth, plus an attribute
check on the four control modules that receive `Cargo` objects. An audit the controller
could read would be a sensor.

---

## 7. The two unexplained cases — both located

### 7.1 `rectangle__a0.10__seed004` is a safety violation reported as a solver failure

The only solver failure in 225 episodes: 124 fallbacks, 124 infeasible solves, 658 barrier
scalings, `min_barrier_scale` 0.0. Re-run with per-frame tracing
(`docs/results/t4/FINDINGS.txt`), the **order** is decisive:

| event | frame |
| --- | --- |
| first inter-agent separation breach | **2012** |
| first infeasible solve | 2020 |
| first fallback projection | 2020 |

**The separation broke first, by 8 frames.** So this is not a solver failure. The
inter-robot barrier is feasible at `u = 0` whenever `h_ij ≥ 0`; once two robots are inside
`d_min` the row demands a separation rate the speed limit cannot deliver, and the QP is
genuinely infeasible. The 124 infeasible solves are the consequence, and SOLVER_FAILURE
names the symptom.

What put them inside `d_min`: the episode travelled **6.29 m on a 0.214 m task** with
transport never armed, at a peak cargo speed of 0.179 m/s, while strict coverage collapsed
from 1.000 to 0.319. The ring was pressing an object it could not keep up with, and the
robots chasing the trailing arc crowded together.

Its minimum inter-agent distance was **0.2038 m against `d_min` 0.28** — a 76 mm safety
violation. **The taxonomy does not report it.** `classify` ranks SOLVER_FAILURE above
SAFETY_VIOLATION and returns on the first match, so the composition table shows
SOLVER_FAILURE 1 and SAFETY_VIOLATION 2 without this episode among them. The ordering is
defensible in general — a pre-run rejection should not be labelled by what the run then
did — but a single label per episode cannot represent an episode that failed two ways.

### 7.2 `polygon32` seed 2 never moved, and the 93° is an artefact

The reported "pushes at ~93° to the goal with J ≈ 0" is not a measurement of a push
direction. The displacement is of order **10⁻⁵ m**, and the direction error is
`arccos(J/|dx|)` with both arguments at the numerical floor. Dividing two near-zero numbers
produces an angle, not a finding.

What needs explaining is why the object never moved, and the trace answers it: the push set
never reached quorum.

| α | displacement | robots meeting alignment | robots actually pushing | quorum | transport |
| --- | --- | --- | --- | --- | --- |
| 0.10 | 0.005639 m | 10 | **4** | 4 | armed at frame 2890 |
| 0.40 | 0.000417 m | 8 | **3** | 4 | never armed |
| 0.80 | 0.000417 m | 8 | **3** | 4 | never armed |

So the alignment test `n_k · d_goal ≤ −0.35` is **not** the binding constraint — 8 to 10
robots pass it. `α = 0.40` and `α = 0.80` are bit-identical across every recorded metric,
which confirms the target distance never entered the run: the episode is entirely determined
by a pre-transport phase that does not depend on α.

The extended trace locates the loss in the **contact band**, and the mechanism is a
dimensional mismatch:

| quantity | value |
| --- | --- |
| contact band limit, `cage_offset + contact_band_tolerance` | **0.185 m** |
| polygon32's radial spread, `max r − min r` | **0.2545 m** |
| robots in the band, mean over frames (of 16) | 4.4 |
| robots contact-ready, mean | 4.5 |
| robots meeting alignment, mean | 5.0 |
| robots actually pushing, mean | **0.13** |
| spread of per-robot distance-to-own-nearest-map-point | 0.377 m mean, 0.754 m max |

**The object's radius varies by more than the contact band is wide.** `polygon32` is a
32-gon with radii `0.58 ± 0.08` scaled by 1.579, so its surface undulates over a 0.2545 m
range while the band that defines "in contact" is 0.185 m deep. A single scalar stand-off
band cannot be satisfied all the way round such an outline at once: a robot at the nominal
offset from a crest is outside the band in the adjacent valley, and vice versa. Only about
4.4 of 16 robots are in the band at any instant, and *which* 4.4 keeps changing.

The push predicate is a conjunction of in-band, aligned, and having enough contact-ready
neighbours locally. Each of the three sets holds roughly 4–5 robots, but their intersection
holds a mean of **0.13** and never more than 4. Against a transport quorum of 4, transport
armed once — at frame 2890 of 3000, at α = 0.10 — and never otherwise. The object never
moved, and the 93° is arithmetic on the resulting 10⁻⁵ m displacement.

This is a shape-class failure with a measurable predicate behind it, and it is **not** one
of the §1 premises: nothing in the certificate compares the object's radial variation with
the contact band depth. A candidate premise — `radial_spread ≤ cage_offset +
contact_band_tolerance` — would have rejected `polygon32` before the run. It is *not* added
here, because adding a premise that rejects a family already known to fail is fitting the
domain to the results, and because `polygon32` still scored 5/15 overall, so the predicate
would reject cases that succeed. Recorded as the mechanism, and left for a decision made on
its own evidence.

---

## 7a. The distance ablation settles the cross-track gate argument

`scripts/run_distance_ablation.py`, five alpha levels at 12 seeds each on the baseline
l_shape (diameter 2.546 m), 60 episodes. CODEX's fixed metric distances are deliberately not
inherited — see the script's own header for why — and `alpha = 1.0` is included past the
matrix's 0.8 because the point of a sweep is to find where the method stops.

| alpha | L (m) | pass | J/diam | **J/L** | cross/diam | direction error | **gate's implied direction limit** | peak coverage | over the 0.15 m gate | efficiency |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.2 | 0.509 | 9/12 | 0.253 | 1.266 | 0.031 | 6.39° | 13.52° | 0.984 | 1/12 | 0.991 |
| 0.4 | 1.018 | 8/12 | 0.497 | 1.243 | 0.060 | 6.04° | 6.83° | 0.979 | 7/12 | 0.993 |
| 0.6 | 1.527 | 8/12 | 0.741 | 1.236 | 0.081 | 5.09° | 4.57° | 0.979 | 7/12 | 0.994 |
| 0.8 | 2.036 | 5/12 | 0.986 | 1.233 | 0.118 | 5.83° | 3.44° | 0.969 | 7/12 | 0.992 |
| 1.0 | 2.546 | 4/12 | 1.278 | 1.278 | 0.197 | 7.81° | **2.65°** | 0.976 | **12/12** | 0.989 |

Separation held on all 60 episodes and there were **zero** fallbacks and zero infeasible
solves at every alpha.

**Displacement tracks demand almost exactly**: `corr(alpha, J/diameter) = +0.992`. That is
the matrix's alpha finding, reproduced on one shape with 12 seeds instead of 12 shapes with 5.

**The overshoot is a scale-invariant multiplicative bias.** `J/L` sits at 1.23–1.28 at
*every* alpha, and `corr(alpha, J/L) = +0.029` — no distance dependence at all. So the team
consistently travels about **24% further than asked**, whether asked for 0.5 m or 2.5 m. v1
recorded the on-board progress estimate as "biased low by roughly 10–15%"; this measures it
at ~24% and, more usefully, establishes that it is a *gain* error rather than an offset. A
scale-invariant multiplicative bias is a fixable thing — it is one constant — and an
alpha-dependent one would not have been.

**And the gate, not the controller, is what fails as alpha rises.** Success falls 9 → 8 → 8
→ 5 → 4 while nothing about the control degrades: peak coverage stays flat at 0.969–0.984,
efficiency stays 0.989–0.994, separation holds, and the solver never falls back. What changes
is the *gate*. Because `max cross-track = J sin(direction error)`, an absolute 0.15 m
cross-track limit demands

```
direction error  <=  arcsin(0.15 / J)
```

which falls from 13.52° at alpha = 0.2 to **2.65°** at alpha = 1.0 purely as arithmetic. The
controller's measured direction error stays in a 5–8° band across the whole sweep. At
alpha = 1.0 all twelve episodes exceed the gate, and they do so while travelling 1.28
diameters at 0.989 efficiency.

This is the empirical case for §3.2's conclusion, and it is stronger than the correlation
that motivated it: **an absolute cross-track gate becomes arbitrarily strict as the task
lengthens, for reasons that have nothing to do with the team's ability to aim.** A gate
stated on the reachable cone, or at minimum on `cross-track / diameter`, measures the
controller. The one currently in `g500` measures the task length.

---

## 7b. The robustness ablation, and the noise the controller turns out to need

`scripts/run_robustness_ablation.py`, eight arms x 12 seeds, 96 episodes. The three
degradation mechanisms did not exist in v1 and were added for this experiment; all three
default to exact no-ops, and `nominal` reproduces v1 to ten digits — J = 1.4907537944, 68
barrier scalings, 8/12.

| arm | pass | J | worst separation slack | barrier scalings | fallbacks | velocity-premise breach | spurious frontiers/frame |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `nominal` | 8/12 | 1.491 | +0.00012 | 68 | 0 | 0.603 | 56.6 |
| `range_noise_000` | **9/12** | 1.601 | **−0.00434** | **389** | 0 | **0.367** | 52.3 |
| `range_noise_005` | 8/12 | 1.367 | +0.00001 | 119 | 0 | 0.519 | 53.3 |
| `range_noise_010` | 8/12 | 1.491 | +0.00012 | 68 | 0 | 0.603 | 56.6 |
| `range_noise_020` | **0/12** | **0.761** | −0.00000 | 99 | 0 | 0.511 | **68.7** |
| `slow_updates_5` | 4/12 | 1.523 | +0.00003 | 63 | 0 | 0.567 | 49.3 |
| `comm_dropout_10` | **0/12** | 1.470 | **−0.05803** | 87 | **3** | 0.609 | 56.5 |
| `combined` | 3/12 | 1.699 | **−0.04385** | 113 | **592** | 0.563 | 50.9 |

### 7b.1 The plan's `range_noise_010` arm is the baseline

`configs/sim/d/l_shape_closed_loop.yaml` sets `range_noise_std: 0.01`, so the arm the plan
names as the out-of-domain case is the configuration every headline number on this branch was
produced at. The prediction and the check: `range_noise_010` is **bit-identical** to `nominal`
on every measure, per-seed J included.

By the plan's own rule — reject anything at or above 10 mm as out-of-domain — the **nominal
arm is out-of-domain**, which is why the arm list was extended below and above the baseline
rather than only degrading a configuration already at the rejection threshold.

### 7b.2 The declared velocity premise is not a sensor-noise problem

With a **noiseless** sensor the barrier-visible velocity error still exceeds its declared
0.02 m/s bound on **36.7%** of measured cells, and the normal-error premise is breached on
10.8% — slightly *worse* than nominal's 9.2%. Measured out-of-domain therefore fires on all
eight arms, and the declared-versus-measured verdicts disagree on four of them.

That sharpens §6.3 considerably. The violated premise is not attributable to range noise: it
is the registration and fusion pipeline, and removing the sensor noise entirely does not bring
it inside the bound. Re-deriving the ISSf constant against a moving boundary is therefore the
outstanding item, not tightening the sensor.

### 7b.3 The noise is load-bearing

`range_noise_000` — a *perfect* sensor — is the only arm besides the two worst that **breaches
`d_min`**, by 4.3 mm, and it needs **389 barrier scalings against nominal's 68**, a factor of
5.7. It also achieves the best pass rate, 9/12, and the largest J.

So removing the noise makes the transport better and the safety filter worse. The reading that
fits is that the 10 mm noise acts as **dither**: with zero noise every robot's returns quantise
into the same voxels, the maps agree exactly, the density and CVT targets coincide more tightly,
and robots converge onto each other until the inter-robot barrier has to fight them apart. Noise
decorrelates the targets.

This is recorded rather than acted on, and it is stated as the reading that fits rather than as
a mechanism: one arm at 12 seeds, and the causal claim would need an ablation that decorrelates
the targets *without* adding sensor noise. What it does establish is that the baseline's noise
is not a nuisance being tolerated — the safety filter is quieter with it than without it, and
any future move to a cleaner sensor has to deal with that.

### 7b.4 Where robustness actually ends

* **20 mm range noise: 0/12, and J halves** to 0.761 while peak coverage stays at 0.998 — the
  highest of any arm. Enclosure is unaffected; transport stops. Doubling the baseline noise is
  past the limit.
* **10% directed link loss: 0/12, separation breached by 58 mm, 3 fallbacks.** This is the
  sharpest limit found. The dropout degrades the scan relay, the token flood, the progress
  consensus and the local contact-ready quorum together, which is what a dropped packet costs.
* **`slow_updates_5`: 4/12** with J *above* nominal (1.523) and cross-track worse (0.204). A
  five-fold slower sensor and planner does not stop transport; it degrades aim.
* **`combined`: 592 fallbacks** and a 43.9 mm separation breach. Compounding the perturbations
  produces a solver failure mode none of them produces alone.

### 7b.5 The pseudo-frontier rate is mostly not noise

Every frontier target emitted after the pooled map satisfies the declared
`boundary_map_epsilon` is provably spurious: there is no unobserved boundary left for it to
point at. Measured over the 65–93% of sampled frames that fall after closure:

Noise does inflate it, from 52.3 to 68.7 targets per frame between 0 and 20 mm — a 31% rise,
which is the predicted mechanism, since a perturbed normal rotates the tangential window and a
known neighbour falls outside it. But the rate at **zero** noise is already 52.3 per frame
across a 16-robot team. So the great majority of spurious frontier demand is **intrinsic to the
predicate**, not noise-induced: on a fully mapped object the tangential-neighbour test keeps
declaring known boundary open. The noise-induced pseudo-frontier problem the plan asked about is
real and is the smaller half of the effect.

### 7b.6 What these arms can and cannot say

The nominal contract rate is 8/12 here and 0.300 on the matrix, so an arm that moves two
episodes has moved them across a gate most episodes already fail. The pass column is therefore
not the headline, and the two arms that reach 0/12 and the three that breach `d_min` are
stronger evidence than any of the intermediate pass counts. Separation and fallback counts are
gate-independent and are the columns worth reading.

---

## 8. Negative results carried forward from v1, unchanged

These four are reproduced verbatim in substance from `CLOSED_LOOP_D.md`. None has been
softened.

* **The ring approach was tried and rejected.** Kept as the A1 ablation.
* **Wall-following was tried and rejected.**
* **The D10-DWELL one-line fix was reverted.** The cause was found — 96–97% of contact-band
  exits are robots whose own standoff floor sits above the band, driven out by the loop that
  band membership switches on — and the minimal repair bought 76 frames of contact-ready
  while breaking `d_min` on four seeds and producing 2481 infeasible solves. Any fix has to
  let the leading arc retreat *before* the press starts, which is a second mechanism.
* **D9 was withdrawn.**

And the v1 open items that remain open: the direction-bitmap enclosure certificate is in
the tree, tested, and unused, because on the current runs every threshold of it carrying
real content fails to fire on at least one seed; some goal directions never form a pushing
quorum, the trailing arc for those being the concave notch of the L; and the on-board
progress estimate is biased low by roughly 10–15%.

## 9. New negative results, in the same format

1. **The SE(2) boundary-point velocity made the baseline worse** and ships default-off.
   §6.2. Cross-track +32%, barrier scalings +59%, direction error +29%, 8/12 → 7/12.
2. **The declared error premises are not met.** §6.3. `velocity_error: 0.02` is exceeded by
   60.4% of measured cells, and it is ρ.
3. **A hypothesis of this phase's own was refuted by its own controlled test.** §3.2 and
   §4. The reachable-cone correlations appeared to overturn v1's authority-saturation
   explanation (+0.909 with direction error, in the wrong direction). Narrowing the cone
   directly made every measure worse, so the correlation was a confound with the sampled
   goal direction and v1's explanation stands. Recorded as a negative result of this phase,
   not of v1.
4. **Concavity does not order success.** §1 and `CONDITIONAL_GUARANTEE_V2.md` §4.3. The
   most concave family scores 0/15 and the second most concave scores 11/15, the best in the
   matrix. The stated attribution of the two zeros to "the two most concave families" is
   false; concave_random15 is fourth of twelve.
5. **The eligibility predicate has not been shown to be informative about success.**
   `CONDITIONAL_GUARANTEE_V2.md` §6.7. `P(success | eligible) = 0.275` against
   `P(success) = 0.300`, and 13 of the 31 rejected episodes succeeded. Fisher's exact gives
   an odds ratio of 0.526 at **p = 0.13** — the point estimate is in the wrong direction and
   the difference is not significant, so the claim made is the weak one: no evidence of
   informativeness, not evidence of anti-informativeness.
6. **The failure taxonomy hid a safety violation behind a solver label.** §7.1.
7. **A noiseless sensor breaks `d_min` and needs 5.7x the barrier scalings.** §7b.3. The
   baseline's 10 mm range noise is load-bearing, and the safety filter is quieter with it than
   without it.
8. **The velocity premise is violated even with a perfect sensor** — 36.7% of cells. §7b.2. It
   is the registration pipeline, not the sensor, so tightening the sensor cannot fix it.
9. **10% directed link loss takes the contract to 0/12 and breaks separation by 58 mm.** §7b.4.
10. **Most spurious frontier demand is intrinsic to the predicate, not noise-induced** — 52.3
    provably-spurious targets per frame at zero noise. §7b.5.
11. **Success falls with task distance because the gate tightens, not because control
    degrades.** §7a. `arcsin(0.15/J)` falls to 2.65° at alpha = 1.0 while measured direction
    error stays in a 5–8° band.
12. **The progress overshoot is ~24% and scale-invariant**, not the 10–15% v1 recorded. §7a.

---

## 10. The artefact pipeline, and what is still not done

### 10.1 What the pipeline produces

`scripts/generate_publication_artifacts.py` reads only files already committed under
`docs/results/` and runs no episodes — the same separation `render_closed_loop.py` enforces
for the animation, so that "the numbers in the paper" cannot become a different set from "the
numbers in the repository". Every figure is written as **PNG and PDF from one call** so the
two cannot drift, and every proportion carries a Wilson interval. A missing source **skips
the figure and records the skip with the path it wanted**: a missing panel that leaves no
trace is how a figure set comes to describe a different experiment than the one that ran.

Fifteen figure families: success by shape; `J/diameter` and normalised cross-track against
alpha; phase durations; directional progress; the cross-track identity; the reachable cone
observational-vs-controlled pair; cargo rotation; net wrench and contact count; safety
distances; perception error against the declared premises; failure composition; the
conditional-domain comparison; runtime; the robustness arms with the pseudo-frontier rate; and
the distance ablation.

Closed-loop frames are **not** drawn there. They go through v1's `dbact_sim.replay` via
`scripts/render_closed_loop.py`, which keeps the v1 phase palette and draws one robot's own
map beside the true outline rather than reconstructing a surface from ground truth.
`docs/results/representative/figures/frame_0233.png` shows exactly that, with every panel
quantity tied to a written gate.

`scripts/derive_finite_time_bound.py` reproduces the bound's *unavailability* in one line:
`available: false` with the three uncertified contraction rates named, and a banner saying the
phase totals are what the bound would be rather than a bound.

### 10.2 Still not done

Stated so the gaps are visible rather than inferred from absent sections.

* **`enclosure_bound_frames`, `transport_bound_frames` and `hold_bound_frames` remain
  premises**, not measurements. They exist so the `frame_budget` check has something to add
  up, and the derived analytic bound still reports itself unavailable regardless.
* **The three contraction rates are still uncertified**, which is the reason the finite-time
  bound is unavailable. Certifying any one of them is a proof obligation, not a measurement,
  and nothing here attempts it.
* **The ISSf constant has not been re-derived against a moving boundary.** §3.1 rewrites the
  S1 *criterion* with its reason; it does not re-derive the constant. §6.3 shows the premise
  that constant rests on is violated by 60.4% of measured cells, which makes the re-derivation
  the larger of the two outstanding items.
* **The ~24% progress overshoot measured in §7a is not corrected.** It is now known to be a
  scale-invariant gain error rather than an offset, which is what makes it fixable, but
  changing the transport loop's gain would invalidate every number on this branch and was out
  of scope for a verification phase.
* **The cross-track gate is not changed.** §3.2 and §7a make the case for restating it on the
  reachable cone and give the form; the `g500` gate still reads `cross_track_max: 0.15`,
  deliberately, so that every result here is scored by the gate the earlier results were
  scored by.
* **No hardware.** See `CONDITIONAL_GUARANTEE_V2.md` §6.6.
