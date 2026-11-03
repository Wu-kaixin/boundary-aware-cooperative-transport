# Corrected finite support: fresh eight-episode physical validation

The previous finite-support study remains frozen and negative at source dab8a22a.
Its post-review correction (source cce9a4fdc1677a54a7faa9ea6d9baf6c5110387b8acee63e158f6da378952177)
computes all isolated half-gaps before feature exclusion. No best numerical
parameter, physical model, default method or original gate changes.

Prerequisite saved-state audit: original4 candidate prefixes exactly reconstructed
through reference freeze (874frames,13,984robot filter calls), plus separate40
preflight frames. All7 selected witnesses/14QP comparisons agree with CLARABEL.
New admission1,738/3,476 supplied calls vs old1,772/3,476. Both old residuals
remain rho-free infeasible. The other two full-rho old-command violations occur
at already-recorded relaxed_margin steps, not newly demonstrated unsafe commands.
These are diagnostics, not evidence of corrected closed-loop success.

Question: what is the corrected runtime's actual paired task/safety performance?
Exactly8 NEW research physical episodes:4 original reference_reacquisition
controls and4 corrected finite_acquisition_support candidates, on the SAME reused
near-circle/near-L/far-C/far-skew conditions. Baseline configs/seeds/physical
initial states are copied from finite_acquisition_support_v1.json unchanged;
candidate differs only by acquisition_finite_support=True. Old candidate outcomes
may be referenced as historical comparisons, never counted as new episodes.

Budget and schedule:12,000steps per episode, dt.05s,600s, original40-frame/.005
settling rule, task tolerance.12m,8episodes/96,000maximum total steps. Fixed
shuffle seed20261103,4workers,1numerical thread each. No wall-time truncation,
shape-specific adjustment, extra seeds, retry-until-success or numerical tuning.
Source/protocol/scripts/design/environment/test receipt freeze before running.

Retain the SAME mechanism gate as v1: all8 executions complete; all4 control
trajectories/scientific fields exact against prior control receipts;4 candidate
safe task completions, valid zero epochs, no false HOLD or original hard safety
failure; zero candidate acquisition scaling; no matched increase in post-freeze
scaling; preserve every baseline G500 pass. Full-chain gate additionally requires
all4 candidate original G500 passes and formal TRANSPORT entry. Do not change
quorum4, brake timing, radius, rho, force, contact or completion conditions.

Report every episode including failures/timeouts; endpoint distance, duration,
true rotation, original hard safety, G500 reasons, stage-scaled robot steps,
activation, zero-epoch validity, support admission/fallbacks and reach caps.
Count baseline original-path robot steps from its saved diagnostics rather than
filling absent audit counters with0. These are path-use counts, not counts of
positive object rows. The old CSV stays unchanged and retains its caveat.

Reused development conditions cannot support general reliability or deployment
superiority. Even if these gates pass, keep default-off pending broader original
success preservation, unused-condition validation and deployment-method matched
controls. Arbitrary unseen shape/rotation safety remains uncertified. A negative
result remains in the denominator and guides the next decision without relaxing
the frozen criteria. C activation and rotation must be evaluated explicitly.
