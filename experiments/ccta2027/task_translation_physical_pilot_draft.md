# Draft next decision: test actual stopping after complete four-case replay

Status: **DRAFT, NOT FROZEN, NOT EXECUTED, NOT ADOPTED**. Written while the
unchanged far-skew completion replay is live. Do not edit the frozen runtime
or launch this pilot until that replay is terminal and all four outcomes have
been inspected. The full objective and best numerical parameter protection
remain in active_goal_20261103.md.

## Why a different question may be necessary

The original offline screen is already failed for L and C. Retain that result.
Its 40-consecutive-estimator-entry rule is not the controller's physical
40-quiet-frames-after-HOLD rule: the original successful L has only 9 consecutive
offline entry frames. The selected translation/all-captured arm has 38/40 entry
frames in that old HOLD window, but only a 17-frame uninterrupted streak.

In the completed C replay the same candidate improves paired MAE from 58.850 to
14.270 mm and supplies four-voter feedback on 11549/11549 evaluation frames,
versus 4020/11549 originally. At the last old control sample, true remaining
error is 57.674 mm and the candidate estimates 44.227 mm, outside the unchanged
20 mm arrival band. Stopping on that old trajectory is therefore not a necessary
sign of accurate feedback. An offline replay cannot show whether the candidate
would provide the additional motion and settle in a new closed loop.

A prospective exploratory physical trial could answer that different question.
It would not retroactively pass the old gate, validate arbitrary rotation, or
promote the candidate. Before deciding, inspect the missing far-skew result and
its false potential stops, error, availability, quorum and actual rotation.
If it adds false goal-stops outside 120 mm or exceeds the original 30 mm MAE /
original+10 mm limits, investigate the mechanism before launching this draft.

## Proposed candidate and controls

Keep translation_all_captured as the single common candidate for every shape.
No per-shape arm selection, retuning, stopping-band relaxation or extra dwell.
The only proposed runtime additions are opt-in model/admission selectors, default
SE(2)/original supporters. Candidate uses the existing initial-acquisition
policy and epoch, every robot's own already-captured zero-epoch template, and
the exact TranslationTaskReferenceTracker already evaluated offline.

All acquisition/noise, control, transport, physics, task and safety numerical
parameters remain byte-for-byte/effectively equal to their protected sources.
The all-captured list must not replace the historical strong-supporter list in
diagnostics; record both the original freeze condition and actual selected
tracker identities. A zero fitted angle and the resulting local goal-error
bound must remain labeled as a translation model assumption. No true geometry,
true centroid, contact truth or true yaw enters the controller.

Use the same four development conditions (circle, L, far C, far skew) under one
predeclared protocol. Proposed primary comparison: four fresh protected-common-
parameter baseline episodes plus four fresh candidate episodes, with identical
per-condition inputs/seeds and unchanged 600 s / 12000-step maximum. The original
initial-task-reference episodes are historical context, with their original
source identity, not extra new controls. Baseline reproduction must check the
old physical arrays and scientific fields, allowing only explicitly listed new
diagnostic/selector fields. Freeze source, configuration, environment, protocol,
resource budget and raw output paths before execution.

## Proposed evidence required before any further promotion

Primary outcomes: all four complete physical tasks within the unchanged 120 mm
task tolerance and settling rule, no false HOLD outside that tolerance, and no
new hard-safety violation. Preserve every old baseline failure and all new
timeouts. Report old-baseline/candidate paired outcomes, not a pooled success
probability. G500 remains a separate metric with every failure and barrier
scaling visible; a success gain is not a dynamic-safety proof.

Secondary outcomes: final true error, first HOLD error, completion time,
pre-epoch motion/contact, estimated displacement MAE and error at first HOLD,
fresh-voter/quorum availability, actual rotation (evaluation only), minimum
spacing/penetration, contact recoveries, barrier scaling and termination reason.
Count an estimator's refusal to stop separately from a completed transport task.

If this development pilot supports the candidate, a single common controller
still needs the wider protected-condition regression and unused-condition
validation before promotion. Deployment CVT/spacing/nearest must then be compared
with that same common controller, parameters, starts and information permissions.
Only their differences can support the independent deployment contribution.
Retain the shared phase supervisor limitation; final paper claims must follow
the resulting evidence and preserve the negative studies.

## Implementation verification before a frozen pilot

Write failing tests for selectors requiring valid initial-task-reference mode,
default compatibility, immutable original freeze/strong-supporter identities,
all-captured admission without a late reset, translation-model labels and
unchanged protected numerical values. After the offline completion is terminal,
implement the minimum opt-in wiring and run the relevant/full regressions.
Do not modify the canonical protected YAML or redefine task success.

This document proposes a future experiment. It is not a report that the missing
case passed, permission to call the old gate PASS, or evidence that the final
research goal is achieved.
