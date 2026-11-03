# Corrected finite support: saved-state audit before new physical validation

Base commit7f7f069; corrected runtime source cce9a4fdc1677a54a7faa9ea6d9baf6c5110387b8acee63e158f6da378952177.
The previous goal turn made progress: the8 original physical episodes completed,
an Important omitted-guard defect was reproduced/fixed, and publication plus
exact-head Windows/Ubuntu CI passed. The broader research goal is still active.

Question: where does the corrected full feature cover change the old candidate's
observed constraints, and can any claimed reconstruction be trusted before new
closed-loop validation? This is posthoc diagnosis of existing development data,
not independent physics, general reliability or a safety certificate.

Freeze this design before execution. Keep all original run receipts, snapshots,
raw arrays, public analyses, gates and numeric parameters unchanged. Use the
root .venv. Do not alter runtime while collecting this diagnostic.

Scope: all4 finite_acquisition_support candidate prefixes in frozen protocol
finite_acquisition_support_v1.json, through the first frozen-reference frame.
First perform20-frame C and near-L engineering preflights in separate directories;
the C prefix covers unavailable observations, while near-L exercises finite rows.
Then replay the complete4 prefixes, each once. All failures retained. Four
subprocesses may run with one numerical thread each. No physical integration.

Load the ORIGINAL frozen runtime dab8a22a... to reconstruct controller state from
saved pre-step agent/cargo poses, previous commands and timestamps. Require exact
commands, all diagnostics, solver statistics, phase guards/response and saved map
points on every frame. Abort and preserve an invalid reconstruction on mismatch.
Instrument filter calls without changing returned commands or controller state.

Evaluate the CORRECTED geometry module separately on the same actual supplied
own/one-hop packets and current safety parameters. Verify old finite geometry
matches original object rows whenever it was applied. Record all supplied calls,
old/new admission, fallback reasons, row changes, full-rho and rho-free residuals
of the old command against new rows, reach caps and new joint-QP feasibility.
Hard agent/wall rows stay exactly those of the reconstructed original call.
These are same-state counterfactual QPs, not the future corrected trajectory.

Independent solver checks: both original residual scaling events (C saved365,
skew saved399), plus the FIRST changed admitted geometry per condition and the
FIRST old-command violation of an admitted corrected row per condition. Record
which deterministic selection rule chose each witness. Compare full-rho and
rho-free feasibility to CLARABEL; no claim that selected witnesses represent all
unseen states. Original-command violation thresholds1e-7 are diagnostic numerical
tolerances, not altered runtime safety/success thresholds.

Report full denominators, admission losses/gains, omission-related row changes,
old-command violations, independently checked witnesses and the original two
residuals. Unknown/oversized gaps causing fallback must not be counted as newly
feasible constraints. Truth, if shown, is observer-only. Saved-state evidence
cannot establish corrected closed-loop success or erase original negative gates.

If exact reconstruction and independent-QP checks succeed without a new
implementation defect, prepare a SEPARATE four-condition/two-arm physical freeze
using the unchanged best numeric parameters and original task/G500/full-chain
criteria. Negative saved-state feasibility is retained; it does not license
tuning or weaken physical gates. Any actual defect must be fixed/tested before
that new freeze. No physical episodes belong to this diagnostic design.
