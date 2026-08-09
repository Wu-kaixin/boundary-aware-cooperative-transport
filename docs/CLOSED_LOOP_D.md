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

Each episode samples its own task: `theta ~ U(0, 2pi)`, `L ~ U(0.35, 0.60)`, with
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
python scripts/run_closed_loop.py --seed 4 --out runs/d_seed4
python scripts/render_closed_loop.py runs/d_seed4 --stride 2 --fps 25
python scripts/evaluate_closed_loop.py --seeds 0..11 --out runs/d_sweep
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

## Measured, 12 seeds, 500 frames

`configs/sim/d/l_shape_closed_loop.yaml`, `scripts/evaluate_closed_loop.py --seeds 0..11`.
Every seed is in these numbers, including the failures.

**G500: 2 / 12** (Wilson 95%: 0.05–0.45). **Seven of the twelve failures are the
scaled-barrier gate and nothing else.**

| quantity | mean ± sd | min–max | gate |
| --- | --- | --- | --- |
| directional progress `J` | 0.533 ± 0.178 m | 0.008 – 0.699 | `>= L` |
| efficiency `J/‖dx‖` | 0.975 ± 0.044 | 0.844 – 1.000 | `>= 0.80` |
| cross-track | 0.082 ± 0.066 m | 0.005 – 0.227 | `<= 0.15` (2 over) |
| direction error | 8.7 ± 9.5° | 0.4 – 32.4 | `<= 20°` (1 over) |
| cargo yaw | +0.10 ± 0.18° | −0.06 – +0.57 | `<= 15°` |
| strict coverage (peak) | 0.994 ± 0.013 | 0.956 – 1.000 | `>= 0.70` |
| min inter-robot distance | 0.283 ± 0.007 m | 0.280 – 0.304 | `>= 0.28` |
| min signed clearance | 0.091 ± 0.003 m | 0.086 – 0.096 | `>= 0` |
| max penetration | 0.039 ± 0.003 m | 0.034 – 0.044 | `<= 0.078` |
| transport frame | 120 ± 66 | 58 – 278 | `<= 350` |
| HOLD frame | 208 ± 84 (11/12) | 137 – 397 | — |
| on-board progress / truth | 0.856 ± 0.055 | 0.761 – 0.951 | — |
| simulation rate | 24.0 ± 0.5 frame/s | 23.5 – 25.4 | — |

Solver, 8,000 QP solves per run: **0 fallbacks and 0 infeasible on every seed**.
Scaled-barrier events per seed: `0, 79, 1, 207, 0, 21, 3, 25, 3, 93, 0, 1` — four
seeds are clean, five are in single figures, and three carry almost all of it.

| count | gate that failed |
| --- | --- |
| 9 | scaled barrier |
| 2 | cross-track |
| 2 | target not reached |
| 1 | direction error |
| 1 | not holding |

The A branch, same object, same budget: `J = 0.0561 m`, flat from frame 97, one
solver fallback, 4.9 frame/s.

### Cross-track: fixed, by decoupling membership from weight

The press is always along a robot's own observed normal, so the only steering
authority the arc has is *how much* each robot presses. Deciding both the arc's
membership and its weights against the steered direction coupled them: correcting
a lateral error rotated the membership test too, robots at the edge of a
three-robot arc dropped out entirely, and the correction cost more force than it
bought aim. Membership is now fixed by the task direction and only the weights
follow the steered one, with a floor so a robot the steering has turned away from
still holds its patch. Cross-track went from 0.104 ± 0.103 m (max 0.330) to
0.082 ± 0.066 m (max 0.227), and efficiency from 0.935 to 0.975.

### The scaled-barrier count is structural, not a parameter

Instrumenting every event on the worst seed gave an unambiguous answer: **168 of
168 were object rows demanding retreat, and zero involved a positive inter-robot
demand** — the agent rows were satisfiable at `u = 0`, just barely (`h_ij` between
0.0007 and 0.006), so the robot had to retreat from the object in a direction its
neighbours forbade.

Six different attacks were measured, and the count moved without ever reaching
zero:

| change | scaled-barrier seeds | what else it did |
| --- | --- | --- |
| tier-2 right-hand side computed before the reachability cap (a real bug) | 11/12 | fixed a case where tier 2 relaxed nothing |
| `rho` 0.05 → 0.02 | 11/12 | — |
| 16 robots → 12 | 4/12 | four seeds never activated transport |
| `r_robot` 0.16 → 0.13 at 16 robots | 9/12 | transport strong again |
| `max_speed` 0.35 → 0.45 | 12/12 | *worse*: faster robots spend more time in the band |
| map-jump clamp 0.60 → 0.25 m/s | 11/12 | — |
| press stops `press_margin` short of `r_safe` | 9/12 | best overall state |

The pattern across all of them is one trade: anything that makes the transport
stronger moves the cargo faster, and a faster boundary produces larger `n·v_obj`
terms in rows whose robots are already at their own barrier. That is a property of
the formulation rather than of a gain. The object barrier is a continuous-time CBF
evaluated on an *estimate* that updates in discrete jumps, and the transport loop's
job is to hold robots against that barrier, because the barrier boundary is where
the contact force comes from. Two constraint families both active at their
boundaries, driven by a state that steps, will occasionally have an empty feasible
set.

What would actually close it, none of which is done here:

* a **discrete-time CBF**, whose demands are consistent with one step by
  construction instead of being a continuous-time rate sampled at 20 Hz;
* a **prioritised QP with a proven bound** on the object-row violation, so the
  relaxation carries a guarantee rather than a count;
* or **re-deriving `rho`** to absorb the map-update jump explicitly, which would
  make the scaled tier unnecessary rather than merely rarer.

The gate stays at zero and those seven seeds stay FAIL.

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

* **Discovery is near-field.** The robots are deployed in a ring around the work
  area and the object is inside it, so `first_detection` is frame 0–1. The
  controller does not know the shape, but this experiment does not demonstrate
  search for an object at an unknown location. A far-field initial state with a
  frontier sweep is the next piece of work, and until it exists the word
  "discovery" should be read as "the boundary was found and estimated locally",
  not as "the object was located".
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
