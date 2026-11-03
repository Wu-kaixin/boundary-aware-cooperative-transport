# Independent final review and engineering closure

Review range: `35c853373ac0190c0459f0f0514d9690c118fc14` through
`3c14d05bdf4fafea9377e014d82af48d4a3e4c20`. One fresh reviewer, read-only,
no delegated reviewers or research episodes. Original verdict: **With fixes**.
Reviewer reported47 targeted tests passed/2 deselected. Declined to judge: **none**.

Strengths: explicit default-off and own/one-hop scope, original-row fallback,
freeze-frame guard, noise accounting and scalar motion demand; honest negative
gates, eight-episode denominator and separate historical identities.

## Important finding and one fix pass

The raw-distance prefilter omitted isolated returns before evaluating their
structural half-gap. An observed point outside that cover could be a stricter
constraint. This is a real implementation defect, separate from the disclosed
unknown-boundary limitation. Critical findings:0; Important findings:1.

At position(0,0), same-object scan points(-.001,-.16),(.001,-.16),(.197,0),
(.276,0), rays0,1,5,7, zero noise and the original parameters, the old method
kept only chord h=-.036. The omitted point has half-gap.0395 and h=-.0375.
With chord y=-.163, isolated points(.260,0),(.339,0), and noiseσ=.01, the
actual safety filter returned velocity(.2,.285), optimal/scale1, violating
the omitted point constraint -u_x>=-.184.

Repair: compute all isolated same-object half-gaps before the common guarded
feature cover. Keep actual oversized gaps and unbounded unknown gaps; if such
a potentially competing feature is selected, retain original safety rows.
No new tuning parameter, cap truncation or changed numerical configuration.

[Five new regressions](../../../tests/test_finite_support_review_fix.py) observed
**4 failures/1 pass before repair**, then **75 targeted passes**, followed by
**992 core passes/2 skips**. The feasible noisy-QP witness now returns
(.184,.285). Unsupported competitor cases exercise exact original-row fallback;
the distant noncompeting guard remains excluded. No second review was dispatched.
[Correction design](../../../experiments/ccta2027/finite_acquisition_support_review_fix_v1.md)
and[separate engineering receipt](post_review_checks.json) identify the repaired source.

## Deferred minor

The original CSV converts absent baseline usage counters into0, including
`original_row_robot_steps`. Baseline counters were not measured by the audit;
those zeros **do not mean the baseline used no original constraints**. Do not use
that field for baseline/candidate usage comparisons. Task outcomes, safety,
scaling, G500 and all scientific gates are unaffected. A corrected derivative is
deferred; original CSV, analysis and publication hashes remain unchanged.

## Publication boundary

The eight physical episodes and three earlier old-state probes remain exactly
their original evidence. The repaired runtime has **no new research physical
validation**, remains default-off, and must receive a new frozen study.
Do not interpret the original 4/4 task completions, reduced scaling, or other
metrics as performance of the repaired source. The original candidate could
omit observed competing guards; whether that occurred in its trajectories has
not been established. Its empirical safety interpretation is therefore limited
in addition to the original negative gates and unknown-boundary caveat.

Publication is an honest historical negative study plus a tested engineering
correction, not adoption or a claim that the research objective is complete.

## Recorded rulings

1. Run the fixed matched study despite8 infeasible old states: earlier paths can
   change. Cost if wrong: negative eight-case study; all failures retained.
2. Exclude the freeze frame using both acquisition status and map state: the
   radius is already restored then. Cost if wrong: incorrect margins at contact;
   a dedicated regression covers the boundary.
3. Publish the historical study and separate correction without adding episodes
   to the frozen budget. Cost if wrong: repaired runtime may perform differently;
   default-off and fresh physical validation are required before adoption.
