# Post-study review correction: competing isolated support

One fresh final review of35c8533..3c14d05 identified an Important omission: the
raw-distance cover removes isolated returns before their structural half-gap is
known. A later guarded feature cover cannot recover those excluded candidates.
A valid isolated point can consequently impose a stricter row than the only
retained chord, and the QP can report optimal while violating that omitted row.
This is an implementation defect separate from unknown-boundary limitations.

One fix pass, no new tuning parameters or physical episodes:

1. Write analytical geometry and actual-QP RED regressions from the review's
   zero-noise and noisy examples. Include unsupported potential competitors and
   a noncompeting distant valid point guard.
2. With isolated guards enabled, compute nearest distinct same-object half-gaps
   for ALL unpaired measured returns before the common feature cover. Retain
   actual gap values, including values above the cap; an unknown gap gets an
   unbounded interval, not an invented finite radius.
3. Apply the existing lower/upper-distance, noise, motion and range cover to
   chords and point guards together. If a selected isolated guard has unknown
   gap or exceeds the existing0.04m cap, mark support unavailable and keep the
   original safety rows. Never truncate the radius to make it admissible.
4. Preserve the chord-only option, acquisition scope, disabled behavior,
   radius/noise accounting, original thresholds and numerical configuration.
5. Observe targeted GREEN and run the full core suite. Publish a separate
   engineering identity and a post-study addendum. No second reviewer/re-review.

The completed eight-episode study remains frozen at source dab8a22a... and
run commit5e2a536. Do not modify its protocol, physical design, source snapshot,
raw analysis, published CSV or original outcome. It is evidence for the original
candidate only, not validation of this correction. The corrected candidate stays
default-off and requires a separate future physical freeze. Original scientific
gate failures and C-shape regressions remain open.

The reviewer also found a Minor publication issue: baseline usage counters were
unmeasured but the CSV used0 as the absent-value default. Scientific outcomes and
gates are unaffected; record the caveat and defer a corrected derivative without
rewriting the original CSV/analysis.
