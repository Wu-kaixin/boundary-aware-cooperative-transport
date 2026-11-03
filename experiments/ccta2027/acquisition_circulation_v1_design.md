# Active acquisition circulation v1

Goal: obtain sufficient local reference directions while preserving non-contact
acquisition and the original completion/safety criteria. The exact skew traces
show no capacity exhaustion, only3 qualified captures, near-zero motion despite
nonzero straight-line redeploy commands, and1422 infeasible joint QPs in capture.
Correct map coordinates alone cannot change that acquisition objective.

Chosen architecture: separate the acquisition motion objective from the later
deployment/transport objective. An opt-in boolean `initial_acquisition_circulation`
(default False) replaces only nominal commands while initial_reference_status is
`acquiring`. Require the already opt-in reference-map motion mode, transport and
every-frame sensing/planning; all its existing information and row-mode guards
continue to apply. No numeric parameter is added or retuned.

For a nonempty permitted map view, choose the nearest observed boundary point and
its outward normal. The common counterclockwise tangent is (-n_y,n_x). Derive the
uniform acquisition standoff as the maximum of existing cage_offset, lead_offset
(when present), and the active acquisition r_safe plus the existing press_margin.
Nominal velocity is max_speed times that tangent plus kp_cage times the signed
standoff error in the normal direction. Cap its norm at the existing max_speed.
This removes sticky straight-line redeploy during acquisition and does not need
the true center, shape, pose, contact, task start or all-team sensing. Empty views
retain the original recall/search behavior. The original QP still filters every
command, with unchanged constraints and accounting. Once frozen or unavailable,
the original nominal path and missing-reference guard resume immediately.

This is a single acquisition policy experiment, not a safety certificate. The
uniform circulation direction is a fixed policy choice, not a new measured
world direction. Larger configured offsets remain honored. The source keeps the
best numeric configuration byte-for-byte unchanged. Later contact activation
may change through the resulting initial arrangement and must be measured.

Alternatives considered: increasing capture capacity is contradicted by818/4096
maximum usage; lowering capture support or relaxing safety/velocity thresholds
would change the requested acceptance criteria. Coordinated constraint sharing
remains a possible later mechanism if this acquisition policy still produces
infeasible constraints; this trial does not assume it is solved.

Verification: tests for tangential progress at the standoff, outward correction
when too close, speed cap, rotation/translation covariance, default compatibility,
stage gating, empty-view search, larger offsets and invalid selectors. Execute
affected and full core tests before physical freeze.

Prospective physical screen: the same four original development conditions,
three arms (translation reference, rejected reference-map, and reference-map plus
circulation), same seeds/initial states,12000 frames at0.05s, unchanged metrics.
Twelve newly executed episodes; both existing arms must reproduce their original
25 arrays and scientific fields exactly. Run once under a frozen source/protocol;
all timeouts, failures and scaling remain in the denominator. No retuning or
shape-specific combination after outcomes are seen.

Advance only with12 completed executions,8 exact old-arm reproductions,4 safe
candidate completions and valid zero epochs, no false HOLD or hard safety failure,
zero candidate acquisition scaling, no post-acquisition scaling increase relative
to the original translation reference, and preservation of its per-case G500
passes. Report all endpoint/time/G500 values, capture time, support and unavailable
counts, phase participation and actual pre-capture object movement. This only
tests a development mechanism; further preserved-success and unused-condition
validation, independent deployment contribution and paper work remain required.
