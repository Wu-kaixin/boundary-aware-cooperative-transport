# DBACT Conditional Full-Workspace / Arbitrary-Shape Guarantee

## Claim

The implementation supports the following precise claim; it does **not** claim
unconditional success for every mathematical subset of the plane.

> In a bounded rectangular workspace containing one stationary, initially
> unknown simple-polygon cargo, the paired-lane policy detects every cargo in the
> admissible class from every collision-free, physically contained initial pose.
> If the runtime boundary map is epsilon-dense and the geometry, operational
> enclosure, cage corridor, contact wrench, bounded-error, safety, and
> contraction/progress certificates below pass, the decentralised controller
> encloses and transports the cargo along the feasible task direction within a
> derived finite-time bound. No fixed 500-frame premise is used.

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
11. The enclosure error has a certified contraction rate, transport has a
    certified positive lower progress rate, and BRAKE has a certified
    contraction rate to the terminal tolerance. These premises produce a finite
    analytic phase sum. Without them the finite-time certificate fails closed.
12. The finished-run validator independently checks actual event frames, the
    operational enclosure certificate, solver provenance, safety, progress and
    the map witness. Observed times are not substituted for analytic premises.

An object that fails any predicate remains simulatable, but its output is
labelled ineligible and the strict validator rejects the theoretical claim.

The guarantee has two proof levels. Full-workspace discovery is an a-priori
geometric/finite-time result. Post-search enclosure and transport are conditional
theorems over shapes and episodes satisfying explicit contraction, geometry,
wrench and bounded-error premises. Global convergence of the unqualified
Local-CVT heuristic on every simple polygon is **not** asserted.

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

## Operational boundary enclosure, not formal caging

An enclosure frame is certified only if all of the following pass together:

- strict exterior-only boundary coverage exceeds its threshold;
- the conservative sampled upper bound on the longest cyclic uncovered arc is
  below `max_uncovered_arc_m`;
- every robot centre is outside the object;
- inter-agent distance is at least `d_min`;
- mutually facing cage-offset edges admit the required separation; and
- the configured engaged quorum is present.

This is an **operational boundary-enclosure certificate**. It is not a
configuration-space escape proof. The implementation serialises
`formal_caging: false`, and the paper must not call this a formal caging theorem.

## Conditional finite-time bound

Let `E` be the maximum assigned cage-target error after map completion. Assume a
certified Dini-derivative inequality

```text
D+ E <= -lambda_e E,       E > E_tol,
```

including the effect of the hard safety filter and communication schedule. Then

```text
T_enclose <= log(E0 / E_tol) / lambda_e.
```

During TRANSPORT, assume the admissible contact allocation and closed-loop
pressure controller guarantee

```text
dot J >= v_progress_min > 0
```

outside the BRAKE band. If that band starts `e_brake` before the target, the
drive part satisfies

```text
T_drive <= max(0, L - e_brake) / v_progress_min.
```

Assume BRAKE satisfies

```text
D+ |e_J| <= -lambda_b |e_J|
```

from `e_brake,0` to the HOLD tolerance `e_hold`. Then

```text
T_brake <= log(e_brake,0 / e_hold) / lambda_b.
```

The executable bound is therefore

```text
T_total <= T_search + T_map + T_enclose + T_drive + T_brake + T_hold.
```

`derive_conditional_finite_time_bound` computes the corresponding ceiling in
control frames and labels it `provable_sufficient_conditional`. It never reads
episode completion times. A future Monte Carlo upper confidence bound is an
empirical statistic and must be reported under a separate label.

## Reproduction

```powershell
python scripts/run_publication_representative.py `
  --max-steps 1500 --output artifacts/publication/representative

python scripts/run_arbitrary_shape_monte_carlo.py `
  --seeds 0..4 --max-steps 1500 `
  --output runs/arbitrary_shape_final_se2_60

python scripts/generate_publication_artifacts.py `
  --monte-carlo runs/arbitrary_shape_final_se2_60/monte_carlo.json `
  --output artifacts/publication
```

The frame count is a safety timeout, not a success premise. Each episode stops at
HOLD, an explicit failure classification, or timeout. Missing certificates, a
non-dense map, QP fallback/infeasibility, safety violation, insufficient goal
progress, or failed enclosure rejects the theoretical claim.

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
