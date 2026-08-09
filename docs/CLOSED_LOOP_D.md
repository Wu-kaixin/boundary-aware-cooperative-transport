# D-closed-loop-500f — discovery, enclosure and bounded directional transport in 500 frames

Branch `D-closed-loop-500f`, cut from `A-boundary-aware` at `1751eea`.
`main`, `A-boundary-aware` and every other branch are untouched.

## What this branch is for

The A branch could enclose an unknown non-convex object and could not move it.
Its directional progress `J` reached 0.0561 m and was flat from frame 97 to frame
500. This branch turns that into a closed loop: the team discovers the object,
encloses it, presses on it, moves it a randomly sampled distance in a randomly
sampled direction, brakes, and stops — inside one 500-frame budget, scored by one
contract.

Everything in `S1`–`S6` that the A branch had already established is kept. What
changed is the map layer, the transport law, and the parts of the safety filter
that were producing infeasible problems.

## The stall, and why it was not a gain

`scripts/probe_transport_ceiling.py` answers two questions before any controller
is written.

| Question | Measurement |
| --- | --- |
| Does the stall reproduce? | `J = 0.0561 m`, flat from frame 97 |
| Can this configuration be moved at all? | open loop, safety filter on: 32.80 N along the goal, 0.055 m/s steady, 1.01 m in 400 frames |

So the geometry was fine and the feedback was not. Instrumenting the stall showed
what was actually wrong, and it was not the controller gain:

* from frame ~125 the true contact count was **zero** while **fifteen of sixteen**
  robots reported themselves contact-ready from their own maps;
* the net force along the task direction read **−5.4 N** — the enclosure was
  pushing the cargo backwards.

The team was caging where the object had been.

## Four defects, each with a before and after

### 1. The map did not move with the object

A world-frame map of a moving body is wrong as soon as the body moves.
`LocalBoundaryMap.register` estimates a translation from the robot's own
consecutive scans by point-to-plane least squares,

```
( sum_k n_k n_k^T ) t = sum_k n_k ( n_k^T (p_k - b_k) )
```

and shifts the map rigidly. Point-to-plane rather than point-to-point because a
range scan slides freely along a surface; the normal matrix is rank deficient
exactly when every visible normal is parallel, which is the honest statement that
a robot looking at one flat face cannot observe motion along it.

Two follow-ons were needed before it worked at all:

* **cells must be rekeyed after a shift.** Otherwise the next scan lands in the
  cell the record moved into, finds it empty, and creates a second record for the
  same boundary. The map doubles along the direction of travel and the estimate
  collapses: 0.02 m/s reported against a true 0.085 m/s.
* **fusion weight must saturate.** Without a cap, a cell observed for a hundred
  steps carries a hundred units of prior and a fresh return moves it by under a
  percent — an archive of where the boundary first was, not an estimate of where
  it is.

### 2. The map kept a ghost trail

Registration and fusion both move the cells a new scan lands in. Neither removes
the cell the object walked away from: the new returns land in a *different* voxel,
so the stale one survives until age decay gets round to it, and meanwhile it is
the nearest boundary point the robot has. Measured on the trailing arc, that ghost
sat 0.06 m inside the true surface, so the pushing robots pressed to the barrier
limit against a boundary that was no longer there and applied no force at all.

`LocalBoundaryMap.carve` drops cells the current scan sees through — the same test
an occupancy map uses, and it only ever removes what the scan contradicts, never
what it merely fails to confirm.

### 3. The transport law had no feedback on the task

The A-branch nominal input was a coverage term pulling the robot out and a
*constant* transport term pressing it in. Both are functions of position, they
balance at some depth, the team applies a constant force, and nothing in that loop
is a function of whether the cargo is moving. `transport_control` closes the loop
on the object's speed along the task direction:

```
v_par  = v_obj_hat . d_goal
e_v    = v_ref - v_par
s     += dt e_v
effort = clip(kp e_v + ki s, 0, effort_max)
```

The integral is the part that matters: while the cargo is stuck `e_v` sits at
`v_ref` and the press deepens every step, and the moment the cargo breaks loose
`e_v` collapses and the demand is released. Integral action against a static
friction dead zone is textbook; it is used here because it is standard.

The bound on `s` comes from the actuator limit rather than from tuning —
`s_max = (effort_max − kp·v_ref)/ki`, the state at which the loop already commands
everything the robot can deliver — and `v_ref` falls linearly inside braking
distance, which is what makes the run stop instead of running on.

`v_obj_hat` is the robot's own map registration. Nothing in the control path reads
a simulator pose: not the barrier rows, not the velocity, not the stopping
condition.

### 4. Object rows demanded retreats no input could satisfy

On a non-convex object two faces can both pass the tangential window from opposite
sides of a corner, and their barrier rows then ask for retreat in directions up to
180° apart. The QP reported infeasible for a robot with 0.20 m of true clearance,
fell back to iterated projection, and the projection satisfies *nothing* exactly —
inter-robot separation fell to 0.218 m against a `d_min` of 0.34, after which the
next step demanded a harder retreat and was infeasible again.

Three changes, in order of how much they were needed:

* rows are filtered to the face the robot's own nearest return names;
* the right-hand side is capped at what a speed-limited robot can deliver, stated
  against an explicit witness `w = normalize(sum_k n_k)`, so the object family is
  feasible by construction and the point that proves it is named;
* the last tier scales the object rows by the largest factor that keeps the whole
  set feasible instead of abandoning the problem. The inter-robot rows stay hard,
  and what was given up is a number in the summary.

Minimum inter-robot separation now holds at `d_min` on every seed (0.280–0.304 m against `d_min = 0.28`).

## The closed loop

```
SEARCH -> DISCOVER -> ENCLOSE -> CONTACT_READY -> TRANSPORT -> BRAKE -> HOLD
```

Every transition is a guard on a measured quantity, never a frame number. The 500
frames are a deadline the episode is scored against, not a schedule the phases are
cut to. Two properties are structural rather than checked afterwards: the
supervisor never goes backwards (enclosure quality dips every time the cargo
breaks loose, and a machine that fell back would chatter at the stick-slip
frequency), and `CONTACT_READY` needs a quorum *held* for a dwell.

Each episode samples its own task: `theta ~ U(0, 2pi)`, `L ~ U(0.90, 1.60)`, with
rejection unless the object and its ring still fit in the workspace at the end.
That rejection is what makes "a random direction, but within the controllable
range" a definition rather than a hope — the admissible set is the set of accepted
draws and the sampler reports its acceptance rate.

The direction reaches the controller and the success criterion. It cannot reach
the contact engine: no dataclass in `dbact.contact_dynamics` has a field it could
be written into.

## Running it

```bash
export PYTHONPATH=src
python scripts/probe_transport_ceiling.py --steps 400 --out runs/d0_probe
python scripts/run_closed_loop.py --seed 2 --until-settled --out runs/d_seed2
python scripts/render_closed_loop.py runs/d_seed2 --stride 2 --fps 25
python scripts/evaluate_closed_loop.py --seeds 0..11 --until-settled --out runs/d_sweep
python -m pytest tests -q
```

Simulation and rendering are separate. A run writes `replay.npz` and never draws;
the pictures are made from that file afterwards, so the frame rate a run reports
is the frame rate of the control loop rather than of matplotlib, and revising a
figure costs seconds instead of a re-run.

The animation draws **one robot's own boundary map**, not the true outline. A
density surface reconstructed from the simulator's polygon looks better and
answers none of the questions worth asking about it. The true outline is drawn
beside it, so the estimation gap is visible rather than hidden.

## C5: where the press may stop, and why the QP was infeasible

The object-boundary row's right-hand side is `n^T v_obj - gamma_obj h + rho`, so
it is non-positive — and therefore satisfied by `u = 0` — exactly when

```
h  >=  ( n^T v_obj + rho ) / gamma_obj .
```

Bounding `n^T v_obj` by the ISSf disturbance bound `V` gives a **demand band** of
width `(V + rho)/gamma_obj` above `r_safe`, inside which every object row asks for
active retreat. With `V = 0.20`, `rho = 0.02`, `gamma_obj = 8` that band is
**0.0275 m**, and the transport press was stopping **0.015 m** above `r_safe`.

**Every pushing robot therefore sat permanently inside the band, by construction.**
The press generates force by driving robots against the object barrier, so their
steady state is wherever the press stops; each one then demanded retreat on every
step, and any neighbour at `d_min` made the constraint set empty. That is exactly
what the instrumentation showed: 168 of 168 scaled-barrier events were object rows
demanding retreat, with **zero** positive inter-robot demands.

`TransportFeasibilityContract` (C5) turns this into an assertion, and with it a
constructive feasibility certificate:

> **Proposition.** If every inter-agent barrier satisfies `h_ij >= 0` and every
> object row satisfies `h_k >= (V + rho)/gamma_obj`, then `u = 0` satisfies every
> row of the QP, so the problem is feasible and no relaxation is needed.

Both hypotheses are *maintained*, not assumed: the first by the inter-agent CBF
from a valid initial state, the second by the press floor C5 fixes at
`r_safe + safety_factor * (V + rho)/gamma_obj`. The contract also checks what the
floor costs — the penetration left at the floor is what makes the force — and
refuses a configuration whose quorum could not move the cargo. On the current
scenario: floor 0.098 m, penetration 0.032 m, 16 N a robot, **64 N for a
four-robot quorum against a 24.2 N breakaway**.

Applied to the legacy `configs/sim/v2` scenarios, C5 is also an immediate
explanation of the original stall: four robots at their press floor supply 25 N
against a 31.1 N breakaway. Those configurations are *not transportable by their
own quorum*, and the contract says so at construction instead of after 500 frames.

Measured effect: scaled-barrier events on the worst seed fell from 32 to 7.
**Not to zero**, because C5 removes the systematic cause and leaves a transient —
which is what the second half of T4 is about.

## T4, second half: the barrier has to be a smooth function of the map

C5 fixes *where* the press stops. What remained was a **discontinuity**, and it is
worth separating the two because only one of them is a magnitude.

The sampled row `n^T u >= n^T v_obj - gamma_obj h + rho` is a valid discrete-time
CBF — it enforces `h_{t+1} >= (1 - gamma_obj dt) h_t` — **provided `h` evolves as
`h + dt n^T (u - v_obj)`**. It did not. `h` was read off a *set* of map cells and
the set changes: a carve deletes the cell that happened to be nearest, a fresh
return creates one, and the row that was binding is replaced by a different row a
whole voxel away. `h` then steps with the robot stationary. The honest robust
margin for a jump `W` is `rho = W/dt`, which for a 0.0125 m jump at 20 Hz is
**0.25 m/s — larger than the speed limit**. That is why no value of `rho` could
ever have bought feasibility, and why this was never a tuning problem.

`object_row_mode: aggregate` removes the discontinuity at its source. Each face is
summarised by one confidence- and proximity-weighted plane

```
n_bar = normalize( sum_k g_k n_k ),
d_bar = ( sum_k g_k n_k^T b_k ) / ( sum_k g_k ),
h_bar = n_bar^T p - d_bar - r_safe ,
```

so adding or removing a cell moves the constraint by `O(g_k / sum g)` instead of
switching which sample defines it. The face filter has already reduced the set to
a single face, which is what makes one plane the right summary rather than a
convex-hull approximation of a non-convex object. It also leaves the object family
trivially feasible — one half-plane against the speed ball — so an empty set can
now only come from a conflict with the inter-robot rows.

`gamma_obj * dt <= 1` is asserted at construction: above it the row asks for more
decrease than one step can deliver, which is a modelling error rather than a
solver one.

| | pointwise (pre-T4) | aggregate |
| --- | --- | --- |
| scaled-barrier events, 12 seeds | **293** | **54** |
| per seed | 7, 14, 0, 60, 31, 4, 0, 39, 15, 94, 29, 0 | 0, 13, 3, 25, 0, 11, 1, 0, 0, 1, 0, 0 |
| seeds completely clean | 3 / 12 | **6 / 12** |
| seeds with <= 3 events | 5 / 12 | **9 / 12** |
| failures on the barrier alone | 5 | **3** |
| cross-track, mean | 0.161 m | 0.192 m |
| `J`, mean | 1.474 m | 1.509 m |

54 events across roughly 84,000 solves is **0.064%**. The remaining three seeds
carry almost all of it, and the cost is a slightly softer constraint and therefore
a little more lateral slip.

## Measured, 12 seeds, run to completion

`configs/sim/d/l_shape_closed_loop.yaml`,
`scripts/evaluate_closed_loop.py --seeds 0..11 --until-settled`.
No frame budget: each episode runs until HOLD plus a 40-frame settle window, with
a 3000-frame watchdog. **All twelve settled; none reached the watchdog.** Every
seed is in these numbers, including the failures.

**G500: 2 / 12** (Wilson 95%: 0.05–0.45), with only two gates ever failing.

| quantity | mean ± sd | min–max | gate |
| --- | --- | --- | --- |
| directional progress `J` | 1.474 ± 0.231 m | 1.110 – 1.853 | `>= L`, 12/12 |
| sampled target `L` | 1.211 ± 0.190 m | 0.941 – 1.501 | — |
| efficiency `J/‖dx‖` | 0.993 ± 0.008 | 0.975 – 1.000 | `>= 0.80`, pass |
| direction error | 5.8 ± 3.9° | 1.3 – 12.9 | `<= 20°`, pass |
| cross-track | 0.161 ± 0.122 m | 0.037 – 0.387 | `<= 0.15`, **5 over** |
| cargo yaw | +0.08 ± 1.28° | −2.19 – +3.83 | `<= 15°`, pass |
| strict coverage (peak) | 0.981 ± 0.027 | 0.938 – 1.000 | `>= 0.70`, pass |
| min inter-robot distance | 0.281 ± 0.002 m | 0.280 – 0.285 | `>= 0.28`, pass |
| min signed clearance | 0.085 ± 0.005 m | 0.077 – 0.092 | `>= 0`, pass |
| max penetration | 0.045 ± 0.005 m | 0.038 – 0.053 | `<= 0.098`, pass |
| **contact-ready frame** | **75.5 ± 8.4** | 57 – 89 | derived |
| **transport frame** | **122.8 ± 60.3** | 69 – 237 | derived |
| **HOLD frame** | **274.0 ± 102.0** | 169 – 530 | derived |
| **episode length** | **314.5 ± 101.7** | 210 – 570 | derived |
| on-board progress / truth | 0.832 ± 0.042 | 0.772 – 0.900 | — |
| simulation rate | 23.9 ± 0.4 frame/s | 23.3 – 24.7 | — |

Solver: **0 fallbacks and 0 infeasible on every seed.** Scaled-barrier events per
seed: `7, 14, 0, 60, 31, 4, 0, 39, 15, 94, 29, 0` — three seeds clean.

Of the ten failures, **five fail on the scaled barrier alone and one on cross-track
alone**; every other gate passes on all twelve.

### Lifting the frame budget is what made the timings measurements

The enclosure and transport times are now outputs. Contact-ready lands at
**75 ± 8** frames with a spread of 32 across twelve random directions — much
tighter than transport activation (**123 ± 60**), which is where the direction
dependence lives, and than HOLD (**274 ± 102**), which additionally carries the
distance. Under the old 500-frame cap, seeds 2 and 6 read as "never transported";
run to completion they are two of the three cleanest runs, finishing at frames 426
and 570. **The budget was rejecting runs that were working, and hiding that the
times are a distribution rather than a number.**

Transport distance went from 0.46 ± 0.07 m sampled to **1.21 ± 0.19 m sampled and
1.47 ± 0.23 m achieved** — 0.82 of the cargo's own 1.8 m width, so the displacement
now reads as transport rather than as settling.

### What is still open, and what was measured and rejected

**Scaled barrier, 8 of 12 seeds.** C5 removed the systematic cause; what remains is
the transient it predicts — a map correction that moves a robot into the band. Two
further mechanisms were built and measured, and both are off by default because
they did not pay:

| mechanism | scaled barrier | cross-track |
| --- | --- | --- |
| C5 press floor (kept) | worst seed 32 → 7 | worst seed 0.33 → 0.21 m |
| soft inter-robot repulsion above `d_min` | 9 → 8 seeds | 5 → 7 failures |
| direct lateral differential allocation | 9 → 8 seeds | max 0.39 → **0.68 m** |

The repulsion keeps the ring apart and therefore slower to close on a moving
object; the differential allocation closes a loop that has delay in it and
oscillates. Closing the residual needs the discrete-time formulation, not another
term: the barrier is still a continuous-time condition evaluated on an estimate
that updates in jumps, and only a DT-CBF makes the demand consistent with one step
of that.

**Cross-track, 7 of 12 seeds — now the leading gate.** It is not an independent
quantity. Measured over twelve seeds,

```
max cross-track  =  J * sin(direction error) ,     correlation 0.968
```

so the gate "cross-track <= 0.15 m" at `J ~ 1.5 m` *is* the requirement "hold the
net force direction to within 5.7 degrees for the whole push". The two gates are
one gate, and lifting the frame budget made it harder in exactly the way it should
have: tripling the distance triples the lateral error a given direction bias
produces. At 0.46 m of travel a 6-degree bias cost 0.05 m; at 1.5 m it costs 0.16.

That identity also says what the loop is. Lateral position is the *integral* of
direction error, so a proportional law on it closes a second-order loop with no
damping — which is why giving the allocation more authority produced 0.68 m of
ringing rather than a smaller offset. The lateral component of the robot's own
velocity estimate is the damping signal, and it is free: the same registration
output the transport loop already runs on.

| | P only | PD |
| --- | --- | --- |
| G500 | 2/12 | **3/12** |
| cross-track mean | 0.192 m | **0.165 m** |
| cross-track max | 0.536 m | **0.321 m** |
| direction error | 6.61 ± 5.34° | **5.71 ± 3.76°** |
| seeds over 0.15 m | 7 | 7 |

The worst case nearly halved and the spread on direction error fell by 30%, but
the *number* of seeds over the line did not move: five of them now sit between
0.16 and 0.24 m, just outside. The mean direction error is 5.71° against the
5.7° the gate implies — the loop is sitting exactly on its own requirement, which
is the signature of a loop that is at the limit of its authority rather than
badly tuned. The press is always along a robot's own observed normal, so the
achievable force directions are the cone spanned by whichever faces the arc is
touching; on a faceted object that cone is coarse, and no gain makes it finer.

## D9: far-field search, and what it exposed

`configs/sim/d/l_shape_search.yaml`. The cargo centre is sampled anywhere the
workspace admits; the robots start on a line along one wall;
`require_initial_ignorance: true` refuses any draw in which a robot begins inside
its own sensor range of the object. So `first_detection_frame` measures the search
rather than the layout — the near-field configuration detects at frame 0 on every
seed and supports no such claim.

**The sweep.** Robot `i` of `N` owns the vertical lane centred at
`xmin + m + (i + 0.5)(W - 2m)/N` and walks it end to end, reversing at the ends.
Sixteen lanes across 7.1 m of workspace is 0.443 m a lane against a 1.20 m sensor
range, so every point of the workspace lies inside some robot's swath and a single
lane traversal covers it:

```
T_cover  <=  ( d_to_lane + H ) / v_search  =  ( <=4 + 7.1 ) / 0.28  ~  510 frames
```

independent of where the object is. That is a coverage argument, which the earlier
outward spiral could not make. The partition is a static function of the robot
index and the workspace bounds — it is **not** a frontier method and does not
re-plan as the map fills; that costs efficiency on a partly-explored workspace and
buys a bound that holds with no assumption about the communication graph.

**The token.** A detection has to cross the workspace, and robots sweeping
adjacent lanes are the only ones inside each other's `comm_range`. Each robot that
holds boundary refreshes a token — object id, an estimated position, the time it
was seen — and every step each robot merges its neighbours' tokens and keeps the
freshest per object. Tokens older than `token_ttl` expire, so a stale rumour does
not pull the team to where the object used to be. No polygon and no shape travel,
only where to look.

### Measured, 8 seeds, run to completion

| quantity | mean ± sd | min–max |
| --- | --- | --- |
| **`T_detect`** | **77.9 ± 78.0** | **5 – 227** |
| `T_contact_ready` | 590.6 ± 219.0 | 248 – 938 |
| `T_transport` | 1007.4 ± 592.3 | 249 – 2142 |
| `T_hold` | 1131.0 ± 604.3 | 292 – 2198 |
| episode length | 1194.2 ± 608.7 | 333 – 2238 |
| peak strict coverage | 0.71 ± 0.22 | 0.41 – 1.00 |

Detection lands at **78 ± 78 frames against a ~510-frame coverage bound**, so the
sweep finds the object in about a seventh of its worst case — the object is
usually not in the last lane visited. Every seed detected, and every episode
terminated by settling rather than by the watchdog.

### What it exposed, which is the more useful result

Everything after detection got worse, and the reason is structural rather than
incidental. **The enclosure stage was built and tuned for a ring initial
condition.** In the near-field scenario the team starts distributed around the
object and reaches contact-ready at 75 ± 8 frames. Arriving from one wall it takes
**591 ± 219** — nearly eight times longer, with far more spread — and half the
seeds never enclose at all (peak strict coverage 0.41–0.48 on four of eight, and
0.82–1.00 on the other four).

This is the local-equilibrium failure the redeploy rule was written for, at a scale
it was not designed for: with every robot arriving from the same side, the near
robots converge onto the arc they can see, no robot's disc ever overlaps the far
side, and there is no gradient pointing around the object. That, and not safety, is
the real cost of arriving in a heap.

**A retraction.** An earlier version of this document reported that four of eight
seeds violated `d_min` and called it a T1 safety regression caused by the recall
crowding robots together. That was wrong, and the error was mine: the gate compared
floats exactly against a barrier that is *exactly binding by design* — the ring
sits on `d_min` for most of a transport run — so it reported the last bit of the
QP's arithmetic as a collision. The measured deficits at those "breaches" were
1e-16 to 3e-8 m. Thirty-five nanometres is not a collision. With a 1e-6 m tolerance
on the comparison, **the far-field runs have no inter-agent failures at all**, and
neither did they before.

The cost of that mistake was a round of work aimed at a safety problem that had
never happened, so the lesson is recorded rather than quietly patched: a gate on a
quantity the controller drives *to* its limit needs a tolerance sized against the
arithmetic, or it will report success as failure.

### The approach phase: attempted, not landed

The obvious fix is to give each robot a bearing on the ring the token implies and
let it travel *there* rather than to the token's centre, spreading the team while
it is still on its way in. The bearing half works and is cheap: robot ``i`` takes
``2 pi i / N`` under a polar controller about the token, which sends a robot
assigned to the far side *around* the object instead of through it, and needs no
more agreement than the search lanes already assume.

The radius half does not work, and the reason is worth recording because it is not
a tuning failure.

**A token cannot yet say how big the object is.** Two candidate estimates were
measured on the same run, and both are wrong in opposite directions:

| estimate | seed 2 at first sight | implied ring | object radius |
| --- | --- | --- | --- |
| extent of the observed points | 0.104 m | 1.005 m | 1.320 m |
| holder's own distance to the centroid | 2.613 m | 3.018 m | 1.320 m |

The first is a sliver of one face, so the ring lands *inside* the object and the
whole team is aimed through it. The second is not the finder's standoff at all --
the fused view contains relayed points, so a robot 2.6 m away computes its own
distance to a centroid it never saw directly, and the ring lands so far out that
nobody ever gets within sensor range. Measured against the committed go-to-point
recall, the first version made things worse: three of eight seeds hit the watchdog
without transporting and one pushed the cargo 4.04 m without ever stopping,
against zero watchdog timeouts before.

Both fixes were then implemented and measured. The token now takes its extent from
the robot's **own scan** rather than its fused view, as `max(visible extent,
observer standoff)`, and holds it as a **running maximum** while the token lives,
so the ring grows as more boundary is seen. The estimate lands where it should:
1.51–1.55 m against a true radius of 1.320 m, just outside and slowly tightening,
where before it was 1.005 m (inside the object) or 3.018 m (beyond everyone's
sensor range). The abandon-and-resweep rule was also removed — dropping a token
only re-merges the same rumour from a neighbour on the next step, which is an
oscillation rather than a recovery; a robot seated on its bearing with nothing in
view now closes the ring in instead.

It still does not pay, and the honest comparison is:

| | go-to-point recall | ring approach v2 |
| --- | --- | --- |
| `d_min` failures | 0 / 8 | 0 / 8 |
| peak strict coverage | **0.689 ± 0.240** | 0.573 ± 0.214 |
| `T_contact_ready` | 591 ± 219 | **545 ± 223** |
| watchdog timeouts | **0 / 8** | 3 / 8 |

A marginally faster contact-ready, bought with worse coverage and three runs that
never finished. With the safety motivation retracted there is nothing left that it
buys, so it is **not merged**; the branch keeps the go-to-point recall. The bearing
partition and the polar controller are the parts worth keeping if this is revisited
— what they do not address is the thing that actually costs the 591 frames, which
is that a team arriving on one side has to get *around* an object whose far side
nobody has seen, and a ring computed from one face is not that.

### Wall-following: also attempted, also not landed

The obvious follow-on is to have the first arrivals crawl the boundary and close
the map before the coverage law is asked to do anything with it. It fits the
architecture: `DISCOVER` is a phase in which nothing currently happens, and the law
that runs there is exactly the one that cannot help — move-to-centroid on a
limited-range cell has no gradient pointing *around* an object, so with the team
arriving from one side nobody ever learns the far side exists.

The primitive is standard. With `b` the nearest point in the robot's own map and
`n` its normal, `kp (d_follow - n^T(p - b))(-n)` holds a standoff outside contact
and `v_follow * sigma * perp(n)` slides along, with `sigma` alternating so scouts
split and go both ways round. It ends on a quantity the robot owns: its own map's
angular coverage about its own centroid.

Two versions were measured against the committed baseline:

| | coverage | `T_contact_ready` | watchdog | solver fallbacks |
| --- | --- | --- | --- | --- |
| go-to-point recall (committed) | **0.689 ± 0.240** | **591 ± 219** | **0 / 8** | **0** |
| follow, all robots | 0.000 ± 0.000 | never | 8 / 8 | 10 843 |
| follow, 4 scouts by stride | 0.633 ± 0.167 | 659 ± 317 | 4 / 8 | 0 |

Putting every robot on the boundary is not a stronger version of the idea, it is a
different and much worse one: sixteen robots on a 7.2 m perimeter at a 0.22 m
standoff are packed tighter than `d_min`, the two directions meet head-on, and the
QP loses feasibility outright — ten thousand fallbacks and not one seed that ever
enclosed. Restricting to four scouts recovers the solver completely, and still does
not beat the baseline: coverage slightly worse, contact-ready slightly slower, and
half the seeds never finished.

**Neither is merged.** Three attempts at the far-field approach problem — ring
bearings, ring bearings with a corrected extent, and wall-following at two scout
densities — and none beats a go-to-point recall. That is worth recording as a
result rather than as three failures: what makes the far-field case hard is not
*where the robots go on the way in*, which is what all three changed. The ~890
frames between detection and enclosure are spent by a team that is **already at the
object** and cannot distribute around it, and every one of these mechanisms stops
acting exactly when a robot acquires the object and hands over to the coverage law.
The next attempt belongs **after** hand-over, in the redeploy rule — which was
written for a ring start and is the thing that has to escape the one-sided
equilibrium.

So the honest reading of D9 is: **discovery is solved and enclosure-after-discovery
is not.** The claim "the team finds an object at an unknown position" is now
supported by 8 seeds with a coverage bound behind it. The claim "and then encloses
and transports it" is supported only from a ring start, and the far-field pipeline
has an open safety failure. The next piece of work is an approach phase that
distributes robots around the token as they travel rather than converging on it —
the cage ring is known from the token position and the object's estimated extent,
so the targets exist; nothing assigns them.

## Regression against the A branch: S1's certificate rate

`scripts/verify_refactor.py` reports:

| | S1 | S2–S6 | S7 |
| --- | --- | --- | --- |
| A branch | PASS | PASS | **FAIL** (the stall) |
| D branch | **FAIL** | PASS | PASS |

So the trade is exactly one stage each way, and it is worth stating precisely
rather than as a net score. The single number that moved in S1 is the zero-input
certificate failure rate — the fraction of solves on which `u = 0` does not
satisfy the margin-free barrier, so the robot has to actively retreat:

| | A branch | D branch | S1 criterion |
| --- | --- | --- | --- |
| certificate failure rate | 0.0415 | 0.1054 | `< 0.10` |
| robots inside the cargo | 0 | 0 | `== 0` |
| min signed clearance | 0.1067 m | 0.1207 m | `>= 0` |
| max penetration | 0.0533 m | 0.0393 m | `<= budget` |
| max slack | 0.0 | 0.0 | `== 0` |

Every hard invariant still holds, and clearance and penetration are both *better*
than the A branch. What got worse is how often a robot finds itself inside the
barrier's band and has to retreat — which is what happens when the cargo is
actually moving, since S1 is run on the A configuration where the A branch's cargo
was stationary from frame 97. That is an explanation, not an excuse: the rate is a
stated S1 criterion, it is over, and the threshold has not been touched. Deciding
whether 10% is the right number for a scenario with a moving boundary, or whether
the ISSf constant needs re-deriving against the measured disturbance, is work this
branch has not done.

## What is still open

* **Discovery is done; enclosure-after-discovery is not.** `l_shape_search.yaml`
  places the object at random, starts every robot outside its own sensor range,
  and finds it in 78 ± 78 frames against a ~510-frame coverage bound. What follows
  detection is the open problem: contact-ready takes 591 ± 219 frames instead of
  75 ± 8, four of eight seeds never enclose, and four of eight breach `d_min`
  while converging on the token. See D9 above.
* **Cross-track is the dominant remaining G500 failure.** The press is always
  along a robot's own observed normal — a press along the commanded direction is
  inward only at the centre of the trailing face and tangential everywhere else —
  so the only steering authority the arc has is *how much* each robot presses.
  On a faceted object the arc is three or four robots wide and the available
  normals are discrete, so the achievable lateral authority is limited. Steering
  by effort allocation helps; concentrating it further was measured and made
  things worse.
* **Some goal directions never form a pushing quorum.** The trailing arc for those
  directions is the concave notch of the L, where robots cannot both reach the
  surface and keep `d_min` from each other.
* **The on-board progress estimate is biased low** by roughly 10–15%, so the cargo
  travels somewhat past the target before the team's own estimate says it has
  arrived. That is why G500 bounds the overshoot as well as the shortfall.
* **Only translation is estimated.** Yaw is not. That is a stated limitation, not
  an approximation: the estimate goes into a safety constraint, so claiming SE(2)
  without an error bound would put an unmeasured quantity inside a barrier.
* **One shape.** Everything here is the L at scale 1.5. Nothing about
  arbitrary-shape generalisation is demonstrated.
