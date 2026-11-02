# Observed geometry factors: fixed development comparison

Status: design fixed before the new closed-loop batch. Mission:
`2026-11-02-contact-geometry-v1`. Simulation only, manuscript/title unchanged.

## Evidence motivating the intervention

Six preserved 600 s trajectories from contact-reinforcement-v1 were inspected:
reference and reinforcement in circle, ellipse-parallel and C-open-away, seed 2.
All original failures remain. Full saved logs were inspected; fresh sensor
geometry was reconstructed only every 20 frames (1 s) and at bootstrap, with
return counts checked against receipts. Reconstructed scans are not stored
historical raw scans. Frame/robot counts are diagnostics, not independent trials.

The circular observed-reference centroid is off center. Its reported translation
includes the displacement of that point under the fitted rotation. Correcting
the lever arm with an evaluator-only true center reduces mean circle-reference
position error from 202.181 to 4.975 mm on identical accepted records. This is a
diagnostic, **not a deployable oracle**. A concurrence fit to noisy normals fails
the existing residual gate for all robots, and is not promoted or retuned.

A separate fit using observed point radii, with a fixed one-range-noise-scale
RMS gate and three-noise-scale alternating-split center stability gate, admits
the circular bootstraps and rejects all ellipse/C bootstraps in these records.
The same-input circular readout error is 5.530 mm (reference) and 5.408 mm
(reinforcement). This does not yet establish closed-loop completion or general
model identification. A partial noncircular arc can resemble a circle.

Selecting the nearest noisy return underestimates distance (about 11--12 mm
mean signed error in sampled ready records). Fresh normals are worse than fused
map normals on the sampled paired records, so wholesale fresh-scan replacement
is rejected. A quadratic fit to an observed local patch reduces mean distance
error in all six sampled sets but has worse extremes in some sets. Safety rows
must therefore remain unchanged. Offline patch localization used the fresh
nearest point because historical map point locations were not saved; the runtime
candidate uses the actual map point. Offline and runtime are not identical tests.

## Two factors and information rights

1. `reference_radial_pivot`: bootstrap an optional circular pivot from own plus
   existing one-hop points. No shape name, complete geometry or true center.
   Keep ICP correspondences, iterations, admission, bootstrap offset and reference
   immutable. With anchor a, learned pivot c, fitted translation t and rotation R,
   read out `offset + t + (R-I)(c-a)`. Filter its finite-difference velocity with
   the original coefficient. Rejecting the radial model preserves original output.
   This is a geometry-based readout, not a center-of-mass or yaw certificate.
2. `contact_patch_distance`: own fresh points only, within two existing voxels of
   the map's nearest point and with normal agreement >= 0.5. Fit a quadratic in
   the mapped tangent coordinates; require >= 5 points, interpolation across the
   robot's projection, rank 3 and the existing sensor residual tolerance.
   Replace only the nominal projected standoff/press distance. Preserve mapped
   normals, readiness, phase gates, pressure gains and every QP row/threshold.
   Missing/unreliable patches fall back to the original map distance.

Existing common phase supervision remains explicit; the complete system is not
claimed fully decentralized. Neither factor receives true contact or object pose.

## Frozen budget and comparisons

| Conditions (seed 2) | Arms | Physical cap |
|---|---|---|
| circle; ellipse_parallel; c_open_away | reference; radial; patch; combined | 600 s each |

Exactly 12 new episodes, 12000 frames at dt=0.05; original settling termination
may end a case earlier. Eight outer workers, one internal BLAS/OpenMP/CVT thread,
one batch, external wall cap 3600 s. No new seeds, no gains search, no extra
episodes after seeing results. Partial execution and errors remain in the ledger.
Reference is the preserved vector-goal CVT with both observed-reference channels,
not historical scalar CVT or spacing. Recovery/reinforcement remain off in all arms.
Initial states, scenario, random streams, physics and sensor budgets are paired.
Only the two new flags differ within a condition. Source/environment snapshots
and historical hashes are frozen by the protocol; do not edit runtime after freeze.

Primary outcomes: all-started task completion and independent hard safety.
Report endpoint error, physical duration, settling, G500, first braking cause,
displacement error, true contact, net force, patch admission and model admission.
Use paired condition-level differences; no population confidence intervals or
independence claims from frames. Timing is not a stable science field.

H1: a 50 mm or larger circular displacement-MAE reduction would support the
reference-point explanation. A 50 mm endpoint improvement is task-relevant but
does not count as completion. Reject promotion if estimation improves without
task rescue; record it as a mechanism result instead.
H2: a 2 mm contact-distance-MAE reduction is meaningful relative to the original
few-millimeter map bias. It must translate to physical contact/force and endpoint
benefit to support this nominal-control intervention. The unchanged safety map
may remain limiting; do not silently replace its geometry after a negative result.

Continue a task-level candidate only if at least one failure becomes completed,
with no candidate hard-safety violation, verified reference reproduction, and
recorded mechanism use. Report every regression; no default promotion from this
development batch. All four arms failing means stop this version, not add budget.

## Mathematical boundary and prior work

For a rigid transform written about a, a different fixed reference point c has
displacement `t+(R-I)(c-a)`. This kinematic identity explains reference-point
dependence; it does not prove that an observed partial-boundary pivot is the
physical center or that any transport task will finish. For an ideal circular
boundary, center translation can be observable while material yaw is not.
The earlier progress-local-v2 rank-negative finding concerned another estimator
and is not reversed by this different coordinate/readout diagnosis.

Point-to-plane registration observability and covariance already have an extensive
literature; see Bonnabel, Barczyk and Goulette,
[On the Covariance of ICP-based Scan-matching Techniques](https://arxiv.org/pdf/1410.7632).
That work distinguishes geometry/measurement uncertainty and rematching effects.
No new ICP theorem or novelty over that work is claimed here.

## Validation and provenance

Root `.venv`; baseline 771 passed, two historical skips. The first new synthetic
test incorrectly required a finite-iteration ICP fit to match physical truth to
20 micrometers; its 31-micrometer discrepancy is retained in targeted-01.log.
The corrected test checks the center of the independently refitted **registered
geometry** and its velocity, retaining the numerical tolerance. No historical
test, success or safety tolerance changed. Targeted-02: 32 passed.
New integration checks additionally assert identical physics, phase and QP rows.

Local audit and full screens: `artifacts/research/2026-11-02-contact-geometry-v1/`.
Recovery copies and state: `artifacts/migrations/2026-11-02-contact-geometry-v1/`.
Validation logs: `runs/maintenance/2026-11-02-contact-geometry-v1/`.
