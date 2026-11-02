# Preserve the best known parameters; test reference-surface fitting

2026-11-03. User instruction: preserve all parameters of the best simulation
configuration, improve from that baseline, continue experiments, and report
evaluation metrics and the next research decision.

The final research objective is safe local-observation deployment and unattached
cooperative transport of unknown shapes to the requested position, followed by
reliable settling, with separately supported boundary-deployment contributions.
Simulation evidence and reproducible claims precede hardware or generalization
claims. The existing shared phase supervisor is acknowledged.

## Protected baseline

Use the common 0.45 motion-prior configuration from the frozen transport
validation study: 33/33 old near conditions safely complete; 11/12 later starts
safely complete, G500 9/12. This is the best known common configuration under
those criteria, not a per-shape combination or a global-optimality claim.
Archive a runnable canonical configuration and the resolved controller values.
Preserve each scene's geometry, initial state, task, sensor noise, dynamics,
gains, safety settings, dt=.05, 600-second budget, .02 control arrival band,
.12 task tolerance, and 40-frame/.005 physical settling rule.

Do not enable the failed initial-task-reference/acquisition, innovation-trim,
voter-continuity, support-only or early-origin candidates. Do not alter numeric
parameters or install dependencies. The new opt-in switch is the sole treatment.

## Mechanism

Noisy per-return normals can create spurious motion information. Test an immutable
local surface representation constructed once from the same permitted initial
own/one-hop scan. For each reference point, collect spatial neighbors within the
existing registration gate whose observed normals pass the existing normal-cosine
test. With at least eight points (the original correspondence quorum), fit a
local PCA line. Accept it only under the existing residual tolerance and normal
alignment criterion; otherwise retain the original point/normal. Project the
point onto the accepted local line and use its fitted normal for correspondence.

Keep the original anchor, bootstrap offset, radial pivot, registration loop,
velocity filter, motion prior, navigation aggregation and stopping logic.
The safety map continues to use its unchanged observations and estimates.
This isolates initial reference geometry. It does not identify the area center,
remove bootstrap bias, or certify correspondences. Curvature/corners and correlated
one-hop returns are explicit risks. No simulator truth enters the component.

## Sequence and fixed decisions

1. Save and verify the best-known parameter configuration and source provenance.
2. Synthetic mechanism tests: noise-induced false rank on a straight surface,
   exact polygon motion, corner rejection, immutable template, missing returns,
   preserved anchor/offset/pivot, and default-off compatibility.
3. Freeze an eight-trajectory same-input comparison using all original arms of
   reference_origin_v1. Reconstruct scans and exactly reproduce the old tracker
   statuses/readings/templates before interpreting the new candidate. Truth only
   reconstructs sensors and scores outputs. No new physical episode here.
4. Offline admission: availability >=90% of original in every block; far-skew
   paired vector MAE <=75% of original and a fresh outside-band estimate at its
   original false HOLD; other seven blocks MAE no worse by more than .01 m.
   These repeat the earlier support study's admission standard.
5. Only after admission, freeze 12 known starts x two arms, common original
   parameters, randomize execution order with seed 2026110301, four workers and
   one internal thread. Adoption requires rescue of far skew, retention of all
   eleven original safe completions, no false HOLD/new hard failure, G500 >=9/12.
6. If this succeeds, verify the remaining old 33-condition baseline and then
   freeze unused starts for all three deployment methods. If it fails, retain
   baseline and every adverse result; do not search neighborhood multipliers or
   combine the successful cases from different candidates.

## Metrics (units and denominator kept separate)

Primary: safe complete tasks / all episodes; hard failures / all episodes; false
HOLD / all episodes; watchdog/censored episodes. Completion requires original
HOLD and physical settling and final Euclidean error <=.12 m.

Secondary: final 2D error (m), completion time only for successes (s), G500 and
each failure reason, minimum separation and signed clearance, peak penetration
vs its original budget, solver failures/relaxations, reference availability,
paired same-input vector MAE and old-HOLD bias. For future matched deployment
comparisons retain common-time coverage, uncovered arc length and robot path.

Report episodes/blocks rather than frames as independent observations. The reused
development conditions provide no population reliability or new holdout claim.
