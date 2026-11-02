# Persistent observed-reference displacement investigation

Mission: `2026-11-01-displacement-stop-v1`. Simulation only; no attachment or
hardware use. The manuscript and its title are unchanged. This investigation
follows the negative boundary-navigation comparison; it does not overwrite it.

## What the historical C-away record actually establishes

The complete 12,000-frame sensor/map reconstruction agrees exactly with the
historical selected-agent map snapshots, scalar team progress, recorded local
vector messages, and all 11,999 available shared vector feedbacks. Reconstruction
uses the frozen sensor implementation, saved poses and random identities. The
poses enter the simulated sensor only, not the displacement estimator. Recreated
scans are **verified reconstructed inputs**, not newly discovered raw recordings.

At the final pre-control instant, physical displacement is
`[1.184991, 0.018301] m`; the exact shared vector feedback is
`[0.702140, 0.042038] m`. Its signed error is `[-0.482851, 0.023737] m`,
and the supervisor's estimated goal error is `0.499632 m`. The earlier reported
median of local votes was not this exact supervisor vector.

The unscaled map median is `[0.566242, 0.033902] m`. The 1.24 calibration adds
`[0.135898, 0.008136] m`, reducing the final longitudinal underestimate. Earlier
in the same trajectory the estimate overestimated progress. A new constant gain
chosen to make this endpoint succeed would not explain both regimes.

The current registration-quality flag tests current matching observability, not
the accuracy of historical accumulated displacement. Twelve agents are admitted
at the end, although their unscaled longitudinal accumulations range from
`-0.450947` to `4.208921 m`. Becoming observable again does not reset an earlier
bias. The existing coordinate median rejects some extreme values but cannot
recover a missing common displacement reference.

The signed accounting identity (maximum residual `2.23e-14 m`) attributes the
final longitudinal error as follows:

| Additive accounting term | Contribution (m) |
|---|---:|
| Initial error | -0.005846 |
| Registration increments minus physical increments | -2.893282 |
| Fusion increments | +1.981239 |
| Gain applied to increments | +0.065507 |
| Change of coordinate-median voters | +0.369532 |
| Sum | -0.482851 |

This is accounting along one endogenous trajectory, **not an ablation proving
that deleting a term would improve control**. Tie handling uses agent-ID order.
The instantaneous gain contribution differs from its accumulated increment term
because changes of voters carry previously gained values in the switch term.

There are 10,662 intervals (533.1 s) with both recorded center and yaw increments
below `1e-14`. Map updates still register apparent motion during these intervals.
For agent 04, one of the two final longitudinal median voters, those intervals
sum to `-0.335265 m` of longitudinal map drift and 32.704 m of estimated path
length. This is a concrete example of scan/map refinement and changing viewpoints
being integrated as motion. It is not 10,662 independent experiments.

Final yaw is approximately -6.39 degrees. A fixed boundary material point's
displacement can differ from center displacement by at most 0.144244 m for this
recorded shape and angle. That geometry-based bound is an offline diagnostic;
rotation of the reference point alone cannot explain the observed 0.483 m error.

At the end the estimated speed is 0.012026 m/s, below the unchanged 0.02 m/s HOLD
guard. Estimated remaining distance is still 0.479632 m after the existing 0.02 m
controller arrival band. The arrival guard never passes in the terminal physical
near-target window. There is no evidence of a satisfied arrival guard being lost
by the state machine. The original task remains unsuccessful.

## One candidate, with an explicit information boundary

At the first usable fresh observation after the existing CONTACT_READY event,
each agent freezes its own/one-hop observed boundary returns. It retains the
existing calibrated map displacement at that epoch as its bootstrap offset.
Subsequent fresh returns are fitted to that fixed observed reference by five
iterations of point-to-plane SE(2) registration. The new transport readout is

    estimated displacement = bootstrap offset + reference translation.

The metric registration translation is not multiplied by 1.24 again. Rotation
is estimated about the observed reference centroid. That centroid is not known
to be the object's center of mass, and bootstrap bias remains. The algorithm
uses no object geometry, true center, force, success flag or true target error.

The original map still supplies deployment, local boundary geometry and safety
velocities. It is not rewritten. Missing/rejected reference matches abstain;
initialization supplies no fresh velocity vote. Only own/one-hop messages enter
local control; the existing all-team median remains a disclosed shared phase
supervisor. This is not a claim of fully decentralized end-to-end execution.

Admission reuses the original matching distance, normal alignment, speed and yaw
bounds, with eight matched rows and the existing full-vector rank ratio 0.001.
Residual/rank checks are empirical checks, not an absolute-error certificate.
Arrival tube, HOLD guards, physical settling, safety, and independent task success
remain unchanged. No tensile forces, oracle stop or post-HOLD restart is added.

Under fixed correct correspondences and a translation model, the reference fit
has the form `y_k = A d_k + epsilon_k`. For full column rank,
`d_hat_k - d_k = A^dagger epsilon_k`, with norm at most
`||epsilon_k|| / sigma_min(A)`. This explains the intended benefit: the readout
is tied to a fixed observation reference instead of summing thousands of small
motion estimates. It is a standard conditional least-squares observation, not a
new transport theorem. Changing correspondences, normal errors, SE(2)
linearization, partial visibility, bootstrap bias and the unknown center invalidate
an unconditional bound. The implemented rank/residual tests are not an online
certificate for the 0.12 m physical success tolerance.

Even a perfect position estimate does not imply that available unilateral contact
forces can decrease target error. The closed-loop experiment must separately test
whether useful contacts remain and whether the original stopping guards are
actually reached. Estimator accuracy and contact controllability remain distinct.

## Fixed decision sequence

1. One full historical C-away reconstruction; no physics rerun.
2. One candidate on those same reconstructed inputs. Before implementation, freeze
   a 30–600 s evaluation window, at least 25% same-input shared vector MAE reduction,
   final displacement error at most 0.12 m, and at least 90% availability.
3. Only if that gate passes: nine conditions × four methods = 36 new episodes,
   original CVT, original spacing, previous vector-CVT, and reference-CVT. Reused
   development seed 2, at most 600 physical seconds, eight workers with internal
   single threading, one candidate version and a 60-minute batch wall limit.
4. Preserve every failure. Compare 150/600 s endpoints, completion, original hard
   safety checks, G500 and false/off-target stops separately. The previous vector
   arm isolates the estimator change from the earlier transport intervention.

A failed offline gate stops this candidate, without fitting thresholds after the
result. A favorable reused development trajectory is not evidence of reliability
or a general transport guarantee. Closed-loop outcomes are appended separately
after they actually exist.

## Completed evidence and decision

The same-input gate passed on the single reconstructed C-away trajectory.
Across the predeclared 30–600 s window (11,400 paired frames), shared vector MAE
fell from 0.281922 m to 0.011954 m, final error was 0.014438 m, and availability
was 100%. These are temporal descriptive statistics, not 11,400 independent
samples. A shadow arrival guard was eligible for one frame at 593.70 s; this is
not a HOLD dwell or a relabeling of the unsuccessful historical episode.

All 36 frozen new episodes finished execution, with no execution errors or
budget extension. Wall time was 2055.87 s. The 27 original-CVT, spacing and
previous-vector-CVT episodes exactly reproduce their historical 25 replay arrays
and all historical per-frame fields. Source snapshots and historical input
hashes pass verification.

| Method | Completed by 150 s | Completed by 600 s | G500 pass | Hard-limit violations |
|---|---:|---:|---:|---:|
| Original CVT | 5/9 | 5/9 | 5/9 | 0/9 |
| Original spacing | 5/9 | 5/9 | 4/9 | 0/9 |
| Previous vector-CVT | 5/9 | 5/9 | 1/9 | 0/9 |
| Observed-reference vector-CVT | 5/9 | 6/9 | 3/9 | 0/9 |

Hard-limit checks here mean original separation, inside-object clearance,
penetration budget and solver failure checks. They do not include permission to
silently relax G500: reference-CVT has 35 barrier-scaling events (25 on the
newly completed perpendicular ellipse, 5 on the circle, 3 on the perpendicular
rectangle and 2 on the reference L). Thus task completion does not imply G500
acceptance or a continuous-time safety proof.

The new completion is the perpendicular ellipse: 483.85 s and 0.044782 m endpoint
error, versus all three controls failing that condition. The five previously
completed conditions remain completed. The circle, parallel ellipse and C-away
remain unfinished at the 600 s observation cutoff. This is one reused
development seed per condition; no confidence interval for general reliability
or superiority claim is justified by the 6/9 versus 5/9 counts.

The primary C-away target **failed**. Its physical endpoint error worsened from
the previous vector-CVT's 0.023669 m to 0.783854 m, although its final shared
reference error was only 0.026086 m. In the final 10 s it had on average 0.49
designated pushers, 1.01 actual contacts and zero physical cargo speed. The
supervisor had entered BRAKE after contact recovery failed; accurate position
alone did not create an effective unilateral contact configuration. These are
descriptive post-hoc diagnostics, not proof of one causal explanation. Both
position and velocity transport feedback changed, so their effects cannot yet
be separated by this comparison.

**Decision:** preserve the candidate with its default switch off. This version
does not solve reliable stopping of the full transport system, despite fixing
much of the recorded displacement drift and adding one completion. Do not tune
gain, rank thresholds, horizon or success criteria to rescue this batch. A next
study should first separate position versus velocity feedback and inspect the
loss of effective contacts, with a new frozen budget; no such episodes were
added here. Current verification covers a single observed cargo only; target
association and filtering in multi-object scenes are not validated.

See the [complete results](results/displacement_stop_v1/README.md),
[36-episode table](results/displacement_stop_v1/all_episodes.csv) and
[figure/data/claim mapping](results/displacement_stop_v1/FIGURES.md).

## Reproduction and local records

- Reconstruction: `scripts/audit_displacement_replay.py` (historical source snapshot).
- Signed error analysis: `scripts/analyze_displacement_audit.py`.
- Same-input candidate: `scripts/replay_observed_reference.py`.
- Conditional freeze: `scripts/freeze_displacement_stop_study.py`.
- Local derivatives: `artifacts/research/2026-11-01-displacement-stop-v1/`.
- Protection, execution state and reports: `artifacts/migrations/2026-11-01-displacement-stop-v1/`.
- Engineering validation: `runs/maintenance/2026-11-01-displacement-stop-v1/`.
