# Contact recovery and reinforcement: frozen development design

This document is written before the nine-episode batch. The protocol stores its
hash; later results are a separate report. Manuscript and title remain unchanged.

## Question and evidence

Can locally observed spare contacts improve the three remaining failures without
withdrawing incumbents? This tests a bounded candidate, not a promise that all
shapes can transport. The reference is the **both-reference vector CVT** from
feedback-components-v1, not the earlier scalar CVT/spacing.

Full historical logs for circle, ellipse_parallel and c_open_away were verified
against receipts. Reconstruction of every designated-pusher count matched exactly.
Circle and ellipse never paused. C-away paused for 80 frames and had at most two
prospective geometric pushers during that pause, below the unchanged four-robot
recovery quorum. Thus the circular recovery dependency is real, but fixing it
alone is not predicted to rescue these records.

The last 200 frames (10 s, not a whole-run force survey) showed peak net force
21.7256 N (circle), 24.2401 N (ellipse parallel), and 19.5674 N (C-away).
Ground static breakaway was 24.2356 N. These numbers explain why a geometric
pusher count is insufficient: actual forces, counterforces and geometry matter.
This is evidence from the independent evaluator, never a controller input.

## Fixed comparison

Three conditions x three arms x the existing paired seed 2 = nine new episodes.
Each has 12,000 frames at dt=0.05 s (600 s), with original earlier termination.
Report the original 150 s horizon too. All original failures remain in the data.
No follow-on parameter search or extra episode is included. Maximum eight workers,
one numerical thread each; 60-minute external resource bound.

1. `reference`: repeat the prior both-reference controller exactly.
2. `recovery`: prospective own-map/neighbor geometric eligibility can satisfy the
   original recovery quorum while pushing is disabled. Original dwell, pause
   timeout, brake and HOLD rules remain. Geometric eligibility is not contact.
3. `reinforcement`: recovery plus incumbent patch retention and spare-first
   recruitment. This is a bundled retention/recruitment intervention, so any
   benefit cannot be attributed solely to adding robots.

## Frozen candidate and information permissions

An incumbent with a useful observed normal retains its patch and the original
pressure law. Tangential CVT deployment no longer pulls that designated supporter
off its patch. The original safety QP, speed/pressure bounds, quorum and stopping
permissions always apply. A supporter loses its special role if its normal is no
longer useful; no donor is unloaded to make room for a replacement.

Only a spare with sustained local estimated stagnation or insufficient received
pusher flags recruits. It searches its observed connected boundary, with edges
no longer than three map voxels and adjacent normal dot product at least 0.5.
Offset waypoints must clear current received neighbors. Path length is bounded
by twice local radius. This geometric screen is not a continuous path guarantee.
The existing QP filters every movement. There is no unknown-boundary extrapolation,
attachment, suction or negative contact force command.

Previous-frame one-hop messages arbitrate competing requests by robot ID. A
traveling helper blocks another local departure, but a joining helper does not:
several contacts may be needed before cargo moves. Messages older than 2dt are
ignored. Each robot starts at most three attempts. Travel and rollback time are
bounded by 2*local_radius/max_speed; joining gets twice that bound. No-response
newcomers roll back. Incumbents are not withdrawn on newcomer failure.

Approach during suspension targets the original geometric readiness band, with
nominal gap robot_radius + voxel_size/2. It does not command a penetrated setpoint.
Original contact_dwell is used for local response confirmation. Confirmation needs
own geometric engagement, own previous push command, and positive locally
estimated group motion. This is **not measured individual contact force or a
certificate that a particular helper caused the motion**. All transitions within
a control frame are logged, including immediate arrival and immediate rollback.

Inputs are own observations/state, one-hop neighbor positions/messages and the
unchanged observed-reference readout. Existing common team phase supervision is
retained and disclosed; the system is not claimed to be fully decentralized.

## Acceptance and stopping

Before running: baseline, targeted recovery/recruitment tests, full core tests,
source-snapshot check, paired initial-state check, frozen protocol and dry run.
After running: verify every receipt/product, repeat all 25 reference replay arrays
and historical scientific fields, recompute original success and hard limits.
Recovery-only circle/ellipse are expected to reproduce the reference trajectories.

Report completion, endpoint error, G500, safety, off-target settling, observation
cutoff, execution errors, pauses/resumes, recruitment/rollback and physical contact
separately. A candidate may justify further research only if it rescues at least
one failure without a candidate hard-safety violation, its mechanism activates,
and reference compatibility passes. A negative result closes this candidate;
thresholds and budgets are not relaxed. Even a positive result is only evidence
on three reused development conditions, not generalization or algorithm superiority.
