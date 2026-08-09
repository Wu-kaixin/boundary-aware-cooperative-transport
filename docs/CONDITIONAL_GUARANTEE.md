# DBACT Conditional Full-Workspace / Arbitrary-Shape Guarantee

## Claim

The implementation supports the following precise claim; it does **not** claim
unconditional success for every mathematical subset of the plane.

> In a bounded rectangular workspace containing one stationary, initially
> unknown simple-polygon cargo, the paired-lane policy detects every cargo in the
> admissible class from every collision-free, physically contained initial pose.
> If the runtime
> boundary map is epsilon-dense and the geometry, cage corridor, contact wrench,
> bounded-error, safety, and finite-time certificates below pass, the
> decentralised controller encloses and transports the cargo along the feasible
> task direction within the declared 500-frame horizon.

The theorem domain is defined by predicates, not shape names. A circle, L, U,
star, rectangle, or a previously unseen concave polygon is treated identically.

## Admissibility predicates

For cargo polygon `O`, rectangular domain `D`, and team size `N`, the executable
certificate requires all of the following:

1. `O` is non-degenerate and simple (no self-intersection).
   The initial cargo pose is contained in the workspace and does not overlap a
   robot; this is enforced when the scenario is built.
2. Ear triangulation produces an incircle witness of radius at least `r_f`.
   This is a constructive lower bound: that disk is contained in `O`.
3. Diameter and perimeter are bounded. The latter is essential because fixed
   `N` cannot cover an unbounded boundary.
4. `N` exceeds the conservative boundary covering number
   `ceil(P / (2 (R_cov - d_c)))`.
5. The start and goal footprints lie in `D`. Convexity of the rectangular domain
   then contains the entire pure-translation swept footprint.
6. All edge-offset cage centres, enlarged by the robot radius, fit at both ends
   of the swept corridor.
7. A nonnegative polygon-edge contact-force LP can generate the requested force
   with zero net torque and a configured breakaway-force margin.
8. The nominal pushing quorum exceeds the Coulomb breakaway force.
9. Normal and moving-boundary estimation errors are finitely bounded by the
   configured robustness premise.
10. At rendezvous release, the independent evaluator measures a maximum
    true-boundary-to-map gap no greater than `epsilon_map`. The reported value
    is the sampled maximum plus `P/(2n)`, a Lipschitz upper bound between `n`
    uniform boundary samples, rather than an optimistic sampled maximum. Ground
    truth is used only for this witness and never reaches the controller.
11. The declared search, enclosure, transport, and hold bounds sum to no more
    than 500 frames. The finished-run validator independently checks the actual
    event frames, solver provenance, safety, progress, and map witness.

An object that fails any predicate remains simulatable, but its output is
labelled ineligible and the strict validator rejects the theoretical claim.

The guarantee has two proof levels. Full-workspace discovery is an a-priori
geometric/finite-time result. Global convergence of the post-search Local-CVT
controller on every admissible nonconvex polygon is **not** asserted; finite-time
enclosure and transport are declared conditional premises and are checked
fail-closed on the completed trajectory. This distinction prevents a
post-search heuristic from being presented as an unconditional global theorem.

## Full-workspace discovery bound

Let the scanner range be `R`, contain `M` equally spaced rays, and require `k`
returns. If the polygon contains a disk of radius `r_f`, a conservative guaranteed
sensor-tube radius is

```text
R_det = min(R - r_f, r_f / sin(k*pi/M)).
```

Each half of the workspace has `N/2` horizontal lanes. The certificate verifies

```text
lane_spacing / 2 <= configured_tube <= R_det
edge_padding <= configured_tube.
```

The left and right teams therefore cover their complete half-rectangles. The
latest discovery occurs before

```text
T_detect = ceil((W/2 - edge_padding - configured_tube) / (v_search * dt)).
```

This result includes finite angular ray spacing; it is not based on an ideal
omnidirectional disk sensor.

## Distributed map dissemination and boundary completion

After a local detection, informed non-relay robots remain near the cargo. They
first keep the lane chain intact for a fixed local gossip interval, then occupy
golden-angle mapping slots around the locally estimated map centroid. One courier
on each side completes the deterministic lane path.

At rendezvous the two vertical lane chains and their cross-links form a connected
ladder graph. With one-hop map flooding every perception update, dissemination
takes no more than the graph hop diameter. The certificate verifies that the
gossip interval is at least this bound. The epsilon-dense map witness is evaluated
at release; it prevents partial, one-sided scans from being passed off as an
arbitrary-shape enclosure result.

## Safety and transport

The existing contracts remain active:

- `C1`: `r_safe < d_c < r_robot`, with positive ISSf margin;
- `C2`: an exact hard-QP backend with no silent solver downgrade;
- `C3`: signed goal progress, progress efficiency, coverage, rotation, collision,
  and penetration all participate in success.

The contact engine has no access to the task direction. Cargo motion therefore
arises only from simulated robot contact. A polygon-edge linear program provides
a geometry-specific zero-torque wrench witness before a conditional guarantee is
allowed. The runtime controller still has to satisfy every phase and safety gate;
the LP is a feasibility premise, not a substitute for the closed-loop run.

The QP may explicitly relax the optional `rho` robustness margin when that
margin is infeasible while retaining the nominal hard barrier. Such frames are
counted in the output. A run with margin relaxations supports nominal safety but
must not be described as maintaining the full configured robustness margin at
every frame.

## 500-frame accounting

For the reference configuration the static bounds are:

| Component | Bound (frames) |
| --- | ---: |
| Full half-workspace lane sweep | 204 |
| Rendezvous | 37 |
| Finite-hop map gossip | 27 |
| Post-release enclosure premise | 1 |
| Transport premise | 210 |
| Hold premise | 20 |
| Total | 499 |

Several activities overlap: informed robots map and may enclose while the courier
finishes the search. The 499-frame sum is deliberately conservative and does not
credit that overlap.

## Reproduction

```bash
python scripts/run_500_closed_loop.py --seed 0

python scripts/run_batch.py \
  --configs configs/sim/v3/arbitrary_shape_full_workspace_500.yaml \
  --seeds 0..9 --steps 500 --out runs/full_workspace_sweep

python scripts/run_shape_workspace_matrix.py \
  --steps 500 --out runs/full_workspace_shape_matrix
```

`scripts/validate_run.py` is fail-closed: missing certificates, a non-dense map,
QP fallback/infeasibility, safety violation, missed deadline, insufficient goal
progress, or failed coverage rejects the run.

## Explicit non-claims

The theorem does not cover self-intersecting/disconnected outlines, multiple
mutually occluding cargoes, unbounded perimeter, sub-resolution features,
unreachable concavities that fail the map witness, wall-blocked cages, infeasible
transport corridors, insufficient contact wrench, unlimited mass/friction,
unbounded sensing error, or shapes outside the declared finite-time bounds.

This separation follows the standard discipline in coverage and cooperative
transport work: correctness statements are attached to a stated sensing and
physics model, rather than inferred from visual success on selected examples.

## Primary references

- J. Cortés, S. Martínez, T. Karatas, and F. Bullo, *Coverage Control for
  Mobile Sensing Networks*, IEEE TRA 20(2), 2004.
  <https://arxiv.org/abs/math/0212212>
- M. Rubenstein et al., *Collective Transport of Complex Objects by Simple
  Robots: Theory and Experiments*, AAMAS 2013.
  <https://collaborate.princeton.edu/en/publications/collective-transport-of-complex-objects-by-simple-robots-theory-a/>
