# v2 results — the decisive experiment

Raw data for the v2 decisive matrix. Every number quoted in the v2 write-up has
its source here, so a reader can check any claim without re-running anything.

## What is in here

| path | what |
|---|---|
| `v2_shape_matrix/` | 180 episodes: 12 shape families x alpha {0.1, 0.4, 0.8} x 5 seeds |
| `v2_control_explore_gain0/` | 45 episodes: the `explore_gain` control, see below |
| `v2_baseline_12seed/` | v1's 12-seed L-shape sweep, the regression reference |

Phase-2 verification evidence, added after the matrix:

| path | what | produced by |
|---|---|---|
| `se2/se2_ablation.json` | SE(2) boundary-point velocity, 12 seeds x {off, on}, plus the six-term error audit. **The gate fails and the estimator ships default-off.** | `scripts/run_se2_ablation.py` |
| `t3/lateral_authority.json` | the reachable normal cone over the transport phase, 12 seeds. Refutes the authority-saturation explanation of the cross-track debt. | `scripts/analyse_lateral_authority.py` |
| `t3/push_arc_ablation.json` | the controlled test of the dispersion reading: push-set membership threshold at 0.35 / 0.55 / 0.75 | `scripts/run_push_arc_ablation.py` |
| `t4/FINDINGS.txt`, `t4/t4_traces.json` | the two unexplained cases, per-frame. Both located. | `scripts/diagnose_unexplained_cases.py` |
| `t7/explore_gain_profile.json` | `explore_gain` frame-rate cost. Machine-dependent empirical evidence, **not** a runtime bound. | `scripts/profile_explore_gain.py` |

The two v2 write-ups that read these files are `docs/CLOSED_LOOP_V2.md` (what was ported
and what it cost) and `docs/CONDITIONAL_GUARANTEE_V2.md` (the claim ledger). Deleted lines
under `src/` are justified segment by segment in `docs/SE2_DIFF_AUDIT.md`.

Two findings here contradict statements made when the matrix was first written up, and the
write-ups say so rather than quietly correcting them: concavity does **not** order success
(the second most concave family is the best performer), and the declared perception error
premises `normal_error_deg: 30.0` / `velocity_error: 0.02` are **not met** — the second by
60% of measured cells.

In each directory:

* `monte_carlo.json` — manifest, aggregate statistics, and one full record per
  episode. This is the authoritative file.
* `episodes.csv` — the same per-episode records, scalar fields only, for reading
  into anything that eats CSV.
* `manifest.json` — git SHA, config SHA-256, seeds, alphas, per-case shape
  geometry (diameter, perimeter, area, mass, concavity, yaw), and the list of
  premises that are declared but not verified.
* `REPORT.txt` — the rendered analysis, produced by
  `scripts/analyse_shape_matrix.py`.

## Reproducing

```
export PYTHONPATH=src
python scripts/run_arbitrary_shape_monte_carlo.py \
    --seeds 0..4 --alpha 0.1,0.4,0.8 --max-steps 3000 \
    --output runs/v2_shape_matrix
python scripts/analyse_shape_matrix.py runs/v2_shape_matrix
```

The control run differs only in its config:

```
python scripts/run_arbitrary_shape_monte_carlo.py \
    --config configs/sim/v2/shape_matrix_eg0.yaml \
    --shapes l_shape star10 concave_random15 \
    --seeds 0..4 --alpha 0.1,0.4,0.8 --max-steps 3000 \
    --output runs/v2_control_eg0
```

## The headline

```
J / diameter   0.470 +- 0.424   pooled over alpha, 180 episodes
               0.186 / 0.435 / 0.789   at alpha = 0.1 / 0.4 / 0.8

P(eligible)              149/180 = 0.828   Wilson95 [0.766, 0.876]
P(success | eligible)     41/149 = 0.275   Wilson95 [0.210, 0.352]
P(success)                54/180 = 0.300   Wilson95 [0.238, 0.371]
```

Against CODEX's 0.076-0.119 m on objects 1.05-1.87 m across -- 4 to 10 percent
of the object's own size -- this is a five- to tenfold improvement in normalised
transport. Against v1's own 12-seed sweep, which passes 8/12, the contract
success rate falls to 0.300.

**Both halves of that are the result.** The displacement generalises across shape
and scale; full closed-loop contract satisfaction does not. Anything written
about this matrix has to carry both numbers or it is not describing the
experiment.

### How to read J / diameter

The task distance is `L = alpha * diameter`, so a team that merely arrives scores
`J / diameter ~ alpha`. At `alpha = 0.1` no controller can score 0.5, and pooling
the ratio over alpha makes the headline depend on the mix of alphas rather than
on the controller. Read it per alpha; read `J / L` for whether the team covered
the distance it was asked to cover.

## The `explore_gain` control, and what it corrects

The 12-seed baseline runs `explore_gain = 0.0` -- `configs/sim/d/l_shape_closed_loop.yaml`
does not set the field, so it takes the dataclass default. The matrix config sets
`explore_gain: 6.0`. That made the matrix differ from the baseline in two ways
rather than one, and it mattered most for the family the baseline is built on.

`v2_control_explore_gain0/` re-runs the three most affected families at 0.0:

```
shape               explore_gain=6      explore_gain=0
l_shape              2/15                5/15
star10               0/15                0/15
concave_random15     0/15                0/15

matched solver:      fallbacks 0 -> 0    barrier scalings 8951 -> 10997
```

Two things follow, and the second is the one that survives into the paper.

**`l_shape` was depressed by the configuration, not only by the experiment.** It
roughly doubles at the baseline's gain. At 5/15 it is still mid-pack -- below
`ellipse24` and `convex_random` at 8/15 -- so "v1 is an L-shape special case" is
refuted either way, and the residual gap against the baseline's 8/12 has
overlapping Wilson intervals ([0.15, 0.58] against [0.39, 0.86]) and is not
established. The most likely remaining cause is the alpha range: the baseline
only ever sampled `alpha ~ 0.35-0.63`.

**The two systematic family failures are not a configuration artifact.**
`star10` and `concave_random15` score 0/15 under both settings. They are the two
highest-concavity families in the matrix (0.40 and 0.28).

The control also disposes of a guess worth recording as wrong: `explore_gain` is
not implicated in the solver failures. Fallbacks are 0 in both arms, and turning
the explore term off slightly *increased* barrier scalings.

## What the matrix says, in five answers

1. **Diameter does not degrade transport; it helps.** `diameter vs J/L` gives
   Spearman rho = +0.45 / +0.52 / +0.49 at alpha = 0.1 / 0.4 / 0.8, all
   p <= 0.0003. But `diameter vs peak strict coverage` gives rho ~ -0.44,
   p <= 0.0006: enclosure quality degrades with size even as transport improves.
2. **Concavity costs the cage, not the push.** `concavity vs J/L` is flat --
   rho ~ 0.0, p > 0.39 at every alpha -- while `concavity vs peak coverage` is
   rho = -0.68 / -0.70 / -0.72, p < 1e-8. Convex families pass 30/75, concave
   24/105.
3. **Rising alpha binds on cross-track, not authority.** `J/L` stays near 1.0 and
   transport stalls barely move (7 -> 11 -> 12), while normalised cross-track
   triples: 0.057 -> 0.131 -> 0.233. Enclosure timeouts are 0 at every alpha.
4. **Four families are separated from the matrix** at Wilson upper bound below the
   pooled 0.300: `star10` 0/15 and `concave_random15` 0/15 (upper 0.204),
   `concave_random7` 1/15 and `rectangle` 1/15 (upper 0.298).
5. **The supportable claim is conditional**, not "arbitrary shape". See below.

## Failure taxonomy

```
CONTRACT_FAILURE       70  (0.389)     TRANSPORT_NEVER_ARMED   8  (0.044)
SUCCESS                41  (0.228)     MAP_INCOMPLETE          3  (0.017)
TRANSPORT_STALL        30  (0.167)     SAFETY_VIOLATION        2  (0.011)
COVER_INFEASIBLE       13  (0.072)     SOLVER_FAILURE          1  (0.006)
WRENCH_INFEASIBLE      12  (0.067)
```

`CONTRACT_FAILURE` dominates: the team reaches HOLD and misses a quality gate.
The mechanism is visible at `alpha = 0.1`, where `J/L` is 1.855 +- 4.038 with a
maximum of 29.4 -- on a 0.20 m target the roughly 30 percent overshoot the
baseline also shows becomes a 3000 percent one, and `progress_max_ratio <= 1.40`
fires. This is a small-target regime v1's outer loop was never built for.

Solver failures are one episode, not a trend: all 124 fallbacks and 124
infeasible solves come from `rectangle__a0.10__seed004`. The other 179 episodes
hold 0 fallback / 0 infeasible. Barrier scalings, by contrast, appear in 104 of
180 episodes (12324 events, against 68 in the whole 12-seed baseline).

`P(success | eligible) = 0.275` is *below* `P(success) = 0.300`: of the 31
ineligible cases, 13 succeeded. The admissibility predicates are conservative but
not predictive, and should be described as a filter rather than as a
characterisation of the operating envelope.

## Two unexplained results, recorded rather than dropped

* `rectangle__a0.10__seed004` — the only solver failure in 225 episodes across
  both runs. Cause not identified.
* `polygon32` seed 2 — fails at all three alphas with `J ~ 0` and a direction
  error near 93 degrees, i.e. the team pushes perpendicular to the goal. Two of
  the three never arm transport at all. Same seed, three alphas: a seed-specific
  geometry/yaw pathology rather than an alpha effect. Cause not identified.

## What this matrix does not establish

Mass and friction are held at the baseline values rather than stratified, and
every object is placed with its centroid at the workspace centre. The claim is
therefore conditional on the baseline mass/friction regime and on centred
placement. Five premises in the guarantee block are declared and not verified --
the two bounded-error terms and the three contraction rates -- and are listed as
such in each `manifest.json`. The finite-time certificate reports
`available = false` for exactly that reason, and `formal_caging` is constant
false: what the predicates certify is operational enclosure, not an escape proof.

The wording the data supports:

> conditional transport over an admissible simple-polygon class at scale-relative
> distances, with measured failure on high-concavity outlines and at small
> normalised targets

Not "arbitrary" and not "unknown". The two 0/15 families belong in the paper next
to the headline ratio.
