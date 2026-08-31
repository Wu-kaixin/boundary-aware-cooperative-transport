<div align="center">

# DBACT: Decentralized Boundary-Aware Cooperative Transport

Search, enclose and transport an object of unknown shape — with every claim attached to a measurement.

[English](README.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)
![Tests](https://img.shields.io/badge/Tests-287%20passed-brightgreen.svg)
![Branch](https://img.shields.io/badge/Branch-Claude--boundary--aware--closed--loop--v1-informational.svg)
![Visualization](https://img.shields.io/badge/Visualization-Matplotlib-orange.svg)
![Platform](https://img.shields.io/badge/Platform-MAS%20%7C%20RoboMaster%20S1-lightgrey.svg)

</div>

A team of mobile robots is dropped into a workspace. Nobody tells them where the
object is, what shape it is, how big it is, or how many of them it takes to move
it. They sweep the workspace until somebody sees it, relay that fact, gather, form
a cage around a boundary they are estimating as they go, press on it, move it a
sampled distance in a sampled direction, brake, and stop.

**Nothing in the control path reads the simulator.** Not the barrier rows, not the
velocity estimate, not the stopping condition — every robot acts on its own range
returns, its own voxel map, and one hop of neighbour messages.

> **Branch `Claude-boundary-aware-closed-loop-v1`.** The closed loop runs end to
> end and is measured seed by seed. This README reports what is demonstrated and,
> in equal detail, what is not. The full derivations, the failed attempts and the
> retractions are in [`docs/CLOSED_LOOP_D.md`](docs/CLOSED_LOOP_D.md).

---

## The loop

```text
SEARCH ──▶ DISCOVER ──▶ ENCLOSE ──▶ CONTACT_READY ──▶ TRANSPORT ──▶ BRAKE ──▶ HOLD
   │           │            │             │               │           │        │
 lane        object      boundary-      quorum in       press on    stop on   release
 sweep       token       aware CVT      the band       own speed    own est.  the ring
             relay       coverage       for a dwell    estimate
```

Every transition is a guard on a measured quantity, never a frame number. The
supervisor is monotone: enclosure quality dips every time the cargo breaks loose,
and a machine that fell back would chatter at the stick-slip frequency.

---

## Visual showcase

| Near-field closed loop (seed 2) | Far-field search (seed 7) |
| --- | --- |
| <img src="docs/assets/closed_loop_d_seed2.gif" alt="Closed-loop transport, seed 2" width="100%"> | <img src="docs/assets/search_d_seed7.gif" alt="Far-field lane sweep and discovery, seed 7" width="100%"> |
| Discovery, enclosure, directional transport and stop. The panel draws **one robot's own boundary map**, not the true outline — the true outline is drawn beside it so the estimation gap is visible rather than hidden. | Sixteen robots sweep a static lane partition until one of them sees the object, then relay a token and converge. |

| Closed loop, seed 4 | Closed loop, seed 8 |
| --- | --- |
| <img src="docs/assets/closed_loop_d_seed4.gif" alt="Closed-loop transport, seed 4" width="100%"> | <img src="docs/assets/closed_loop_d_seed8.gif" alt="Closed-loop transport, seed 8" width="100%"> |

| Density and local CVT | Agent trajectories |
| --- | --- |
| <img src="docs/assets/dbact-density-cvt-frame.png" alt="Boundary-aware density and local CVT" width="100%"> | <img src="docs/assets/dbact-trajectory.png" alt="Agent trajectories" width="100%"> |

Simulation and rendering are separate. A run writes `replay.npz` and never draws;
the pictures are made from that file afterwards, so the frame rate a run reports
is the frame rate of the control loop rather than of Matplotlib.

---

## What is demonstrated

### Near-field: enclosure and directional transport, 12 seeds, run to completion

`configs/sim/d/l_shape_closed_loop.yaml`. The team starts distributed around the
object. Each episode samples its own task: direction `θ ~ U(0, 2π)`, distance
`L ~ U(0.90, 1.60)` m, rejected unless the object and its ring still fit in the
workspace at the end. **All twelve settled; none reached the watchdog.**

| quantity | mean ± sd | min–max | gate |
| --- | --- | --- | --- |
| directional progress `J` | 1.474 ± 0.231 m | 1.110 – 1.853 | `>= L`, 12/12 |
| efficiency `J/‖dx‖` | 0.993 ± 0.008 | 0.975 – 1.000 | `>= 0.80`, pass |
| direction error | 5.8 ± 3.9° | 1.3 – 12.9 | `<= 20°`, pass |
| cross-track | 0.161 ± 0.122 m | 0.037 – 0.387 | `<= 0.15`, **5 over** |
| strict coverage (peak) | 0.981 ± 0.027 | 0.938 – 1.000 | `>= 0.70`, pass |
| min inter-robot distance | 0.281 ± 0.002 m | 0.280 – 0.285 | `>= 0.28`, pass |
| min signed clearance | 0.085 ± 0.005 m | 0.077 – 0.092 | `>= 0`, pass |
| contact-ready frame | 75.5 ± 8.4 | 57 – 89 | derived |
| HOLD frame | 274.0 ± 102.0 | 169 – 530 | derived |

Solver: **0 fallbacks and 0 infeasible on every seed.** The composite gate
(`G500`, all criteria at once) passes on **2 / 12**; of the ten failures, five are
the scaled barrier alone and one is cross-track alone. Every other criterion
passes on all twelve.

### Far-field: the team finds an object it was never told about, 8 seeds

`configs/sim/d/l_shape_search.yaml`. The cargo centre is sampled anywhere the
workspace admits and `require_initial_ignorance` refuses any draw in which a robot
starts inside its own sensor range of it — so the detection time measures the
search rather than the layout.

Robot `i` of `N` owns one vertical lane and walks it end to end. Sixteen lanes
across 7.1 m against a 1.20 m sensor range means every point of the workspace lies
inside some robot's swath, so a single traversal covers it:

```text
T_cover  <=  ( d_to_lane + H ) / v_search  =  ( <=4 + 7.1 ) / 0.28  ~  510 frames
```

That is a coverage *bound*, independent of where the object is — which the earlier
outward spiral could not offer.

| quantity | measured |
| --- | --- |
| `T_detect` | **74.6 ± 80.3 frames** against the ~510-frame bound |
| `T_contact_ready` | 344 ± 135 frames |
| peak strict coverage | 0.783 ± 0.214 |
| `d_min` breaches / watchdogs / solver fallbacks | **0 / 0 / 0** |

Detection lands in about a seventh of its worst case: the object is usually not in
the last lane visited. Every seed detected, and every episode terminated by
settling rather than by the watchdog.

---

## The part worth reading: how the far-field gap was closed

Arriving from one wall used to take **591 ± 234** frames to reach contact-ready
against **75 ± 8** from a ring start. Three mechanisms were built and measured —
ring bearings, extent-corrected ring bearings, and wall-following at two scout
densities — and **none beat a plain go-to-point recall.** They all changed *where
the robots go on the way in*, and the frames are spent by a team that has already
arrived.

So the next step was instrumentation, not a fourth heuristic.

<img src="docs/assets/d10-post-detection-stages.png" alt="Post-detection stage durations per seed" width="100%">

Every frame between detection and contact-ready, labelled by an exclusive cascade
on measured state. The stage the pipeline was designed around — *a quorum has
arrived and the boundary is mapped* — **never occurs: 0 frames of 4128.**

<img src="docs/assets/d10-coverage-and-gap.png" alt="Union map coverage, strict coverage, and the largest unobserved arc" width="100%">

The reason, in one measurement: **4.34 ± 0.79 m of a 7.2 m perimeter sits in
nobody's map throughout, and never once falls below 0.72 m.** 84.5% of the
redeploy rule's requests returned no candidate — not because the boundary was
owned, but because it was not there.

That agrees with something readable off the source. For a robot with a non-empty
map, *every* target in the post-arrival path — the CVT centroid, the approach
target, the redeploy target — is an affine function of its own map points, so the
reachable target set is contained in the offset ring over **observed** boundary.
Nothing in the controller could ask for boundary nobody had seen.

**The fix is one term in the same density**, not a new navigation law:

```text
φ  =  φ_boundary  +  λ_e · φ_explore
```

`φ_explore` places demand one tangent step past the ends of what has been
observed. It needs no object radius, no shape prior and no truth polygon; it
switches itself off (on a fully mapped outline it adds exactly zero targets); and
it is a *density* term, so the existing limited-range CVT decides which robot
goes — nobody enters a new mode and the safety layer never sees it.

| A/B, 8 seeds, one parameter apart | `λ_e = 0` | `λ_e = 6` |
| --- | --- | --- |
| far-side discovery | 281 ± 284 | **82 ± 42** |
| contact-ready | 591 ± 234 | **344 ± 135** |
| HOLD | 1131 ± 646 | **642 ± 284** |
| peak strict coverage | 0.689 ± 0.256 | **0.783 ± 0.214** |
| min inter-agent / watchdog / fallbacks | 0.280 / 0 / 0 | 0.280 / 0 / 0 |
| scaled-barrier events | 1415 | **975** |

Seven of eight seeds improve; **one gets worse by 128 frames**, and it is recorded
rather than averaged away.

**The gain is chosen by the solver, not the clock.** Contact-ready keeps improving
up to `λ_e = 60`, but at `λ_e = 20` the inter-agent barrier breaks on two of three
seeds (0.207 and 0.213 m against `d_min = 0.28`) with **589 infeasible solves**
against zero at `λ_e = 6`. Exploration demand pulls robots off the ring; past some
weight it pulls harder than the separation terms can hold them apart.

### A negative result, kept

<img src="docs/assets/d10-gate-tradeoff.png" alt="Enclosure gate: transition delay against the enclosure it certifies" width="100%">

The `DISCOVER → ENCLOSE` guard reads the *best single robot's* own map. That looks
like the wrong quantity for a team-level claim, and four families of replacement
were built — including a reference-free enclosure certificate based on
max-consensus over observed boundary **normals**, which is the quantity that
actually means "for every direction, somebody is on a face opposing it".

Because nothing in the control path reads `ENCLOSE`, the counterfactual is exact
rather than a screen:

```text
T_contact_ready  =  max( T_gate, T_streak20 )  -  1        (residual 0 on all 8 seeds)
```

which puts a hard ceiling on the whole exercise: **an oracle gate firing at frame 0
saves 38.6 frames of 343.8 — 11.2%.** Every candidate with real enclosure content
fails to fire on at least one seed, which under a monotone machine is a deadlock
rather than a delay. The gate is **kept unchanged**, and the certificate stays in
the tree, tested and unused, because the honest measurement is that adopting it
today would deadlock two runs in eight.

The lower-right of that figure — earlier *and* certifying a real enclosure — is
empty. That is the result.

---

## What is **not** demonstrated

Stated as plainly as the results, because a README that only lists wins is not
evidence of anything.

- **The far-field composite gate passes on 0 of 8 seeds.** Discovery and enclosure
  improved; the quality criteria (cross-track above all) did not, and nothing in
  D10 was aimed at them.
- **Cross-track is the leading near-field failure.** Measured over twelve seeds,
  `max cross-track = J · sin(direction error)` with correlation 0.968 — so
  "cross-track ≤ 0.15 m at `J ≈ 1.5 m`" *is* "hold the net force direction to
  within 5.7° for the whole push". The measured direction error is 5.71°: the loop
  is sitting exactly on its own requirement, which is the signature of a loop at
  the limit of its authority rather than a badly tuned one.
- **Some goal directions never form a pushing quorum**, because the trailing arc
  for those directions is the concave notch of the L.
- **The on-board progress estimate is biased low** by 10–15%, so the cargo travels
  past the target before the team's own estimate says it has arrived.
- **Only translation is estimated. Yaw is not.** That is a stated limitation, not
  an approximation: the estimate goes into a safety constraint, so claiming SE(2)
  without an error bound would put an unmeasured quantity inside a barrier.
- **One shape.** Everything here is the L at scale 1.5.
- **No physical transport.** Simulation and MAS dry-run paths work; hardware
  remains a staged validation target.

### Retracted

An earlier version of this document reported that four of eight far-field seeds
violated `d_min`. That was wrong: the gate compared floats exactly against a
barrier that is *exactly binding by design*, so it reported the last bit of the
QP's arithmetic as a collision. The measured deficits were 1e-16 to 3e-8 m.
Thirty-five nanometres is not a collision. The cost of that mistake was a round of
work aimed at a safety problem that had never happened, so the lesson is recorded
rather than quietly patched: **a gate on a quantity the controller drives *to* its
limit needs a tolerance sized against the arithmetic.**

Every coverage number measured before the object-boundary barrier existed is also
withdrawn, because robots standing *inside* the cargo counted as covering its
boundary. With the barrier disabled, 9 of 16 robots end up inside the object while
the old metric still reports 1.000. All coverage figures above are **strict**
coverage, which counts only robots whose centre is outside the cargo.

---

## Quick start

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
python -m venv .venv && source .venv/bin/activate      # PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install -U pip && python -m pip install -e ".[dev]"
export PYTHONPATH=src                                   # PowerShell: $env:PYTHONPATH = "src"
```

One closed-loop episode, then render it:

```bash
python scripts/run_closed_loop.py --seed 2 --until-settled --out runs/d_seed2
```

```bash
python scripts/render_closed_loop.py runs/d_seed2 --stride 2 --fps 25
```

The far-field search scenario:

```bash
python scripts/run_closed_loop.py --config configs/sim/d/l_shape_search.yaml --seed 7 --until-settled --out runs/search_seed7
```

Tests:

```bash
python -m pytest tests -q
```

---

## Reproducing the numbers in this README

Each table above is one command. Runs write to `runs/`, which is Git-ignored.

```bash
python scripts/evaluate_closed_loop.py --seeds 0..11 --until-settled --out runs/d_sweep
```

```bash
python scripts/diagnose_redeployment.py --seeds 0..7 --out runs/d10_diag
```

```bash
python scripts/ab_explore.py --seeds 0..7 --gains 0,6 --out runs/d10_ab
```

```bash
python scripts/diagnose_enclosure_gate.py --seeds 0..7 --out runs/d10_enc
```

```bash
python scripts/analyse_enclosure_gate.py --run runs/d10_enc --figure
```

Contract checks, which refuse a configuration the controller could not satisfy:

```bash
python scripts/check_contracts.py --config configs/sim/d/l_shape_search.yaml
```

---

## How it works

1. **Ray-cast scan.** The simulator generates local boundary returns with normals
   and a confidence derived from the local plane-fit residual. The controller
   never receives the polygon.
2. **Map registration.** `LocalBoundaryMap.register` estimates a translation from
   the robot's own consecutive scans by point-to-plane least squares and shifts
   the map rigidly. A world-frame map of a moving body is wrong the moment the
   body moves; the normal matrix is rank deficient exactly when every visible
   normal is parallel, which is the honest statement that a robot looking at one
   flat face cannot observe motion along it.
3. **Free-space carving.** Cells the current scan sees *through* are dropped.
   Without it a ghost trail sits 0.06 m inside the true surface and the pushing
   robots press against a boundary that is no longer there.
4. **Boundary-aware density.** Each observation contributes
   `ds · c · (1 + κ·g) · K_σ(q − ξ)` at the cage target `ξ = b + d_c·n`, where
   `ds` is the arc length the return stands for — which makes `φ` a measure on the
   boundary rather than a sum over however many samples the sensor produced. Since
   D10 it also carries the exploration term described above.
5. **Limited-range CVT.** Move-to-centroid on a truncated cost `f(r) = min(r², R_l²)`
   over the strict disk `B(p_i, R_l)`. The truncation is load-bearing: without it
   the flux term does not cancel and "move-to-centroid is a descent direction" is
   simply false. `R_l ≤ R_comm/2` makes the cell computed from communication
   neighbours *equal* to the true Voronoi cell restricted to the disk.
6. **Transport outer loop.** A PI law on the object's speed along the task
   direction, integrating against a static-friction dead zone, with the integral
   bounded by the actuator limit rather than by tuning.
7. **CBF-QP safety filter.** Inter-robot rows stay hard; object rows are filtered
   to the face the robot's own nearest return names, aggregated into one smooth
   plane per face, and capped at what a speed-limited robot can deliver against an
   explicit witness — so the object family is feasible by construction and the
   point that proves it is named.

---

## Repository structure

```text
boundary-aware-cooperative-transport/
├── configs/sim/d/                  # Closed-loop and far-field search scenarios
├── src/
│   ├── dbact/                      # Controller, perception, map, density, CVT, safety, contracts
│   │   ├── controller.py           # S7: the decentralised controller
│   │   ├── boundary_map.py         # Registration, fusion, carving
│   │   ├── boundary_density.py     # Boundary measure + D10 exploration term
│   │   ├── safety_filter.py        # CBF-QP, four solver tiers
│   │   ├── phase.py                # Monotone supervisor with a dwell
│   │   ├── diagnosis.py            # D10-DIAG: post-detection stage segmentation
│   │   └── enclosure_gate.py       # D10-ENC: consensus enclosure certificate (unused)
│   ├── dbact_sim/                  # Environment, scenarios, replay, rendering
│   └── mas_adapter/                # MAS-compatible controller adapter
├── scripts/                        # Runs, sweeps, diagnoses, A/Bs, contract checks
├── docs/
│   ├── CLOSED_LOOP_D.md            # The full account: derivations, failures, retractions
│   ├── ALGORITHM.md · ARCHITECTURE.md · MAS_INTEGRATION.md
│   └── assets/                     # Git-tracked, GitHub-renderable media
├── platforms/mas_public/           # Vendored MAS platform code
├── tests/                          # 287 tests
└── runs/                           # Local outputs, Git-ignored
```

---

## Hardware staging and safety

The path to a physical run is deliberately staged: DBACT simulation → MAS dry-run
→ OptiTrack read-only logging → RoboMaster S1 command smoke test. See
[`docs/MAS_INTEGRATION.md`](docs/MAS_INTEGRATION.md).

- Run read-only OptiTrack logging before enabling any controller output.
- Verify robot ID to rigid-body mapping one robot at a time.
- Use very low speed limits for the first physical run.
- Keep a physical emergency stop available during hardware tests.
- Inspect command and state logs after every run.

---

## Further reading

| Document | What it covers |
| --- | --- |
| [`docs/CLOSED_LOOP_D.md`](docs/CLOSED_LOOP_D.md) | The complete account of this branch, including every measured-and-rejected mechanism |
| [`docs/ALGORITHM.md`](docs/ALGORITHM.md) | Density, CVT and safety derivations |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Module boundaries and data flow |
| [`docs/MAS_INTEGRATION.md`](docs/MAS_INTEGRATION.md) | Adapter, dry-run and hardware staging |

---

## Contributing & license

Contributions are welcome through Issues and Pull Requests. The most useful
contributions here are **measurements**: a scenario that breaks a stated claim, a
gate whose tolerance is wrong, or a mechanism tried and reported honestly when it
did not pay.

Released under the [MIT License](LICENSE).
