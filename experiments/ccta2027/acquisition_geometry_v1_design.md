# Acquisition geometry consistency: prospective development check

## Intent and scope

The published translation pilot completed four reused development conditions,
but far-skew incurred 240 scaled object-barrier robot-steps during stationary
reference acquisition. Its acquisition prefix exactly reproduced the older
SE(2) candidate. Far-C incurred one acquisition and seven post-freeze scalings.
The nominal cage offset (0.105 m) is inside the active acquisition safety radius
(0.195 m). This contract mismatch is established; its causal importance is a
hypothesis. The user delegated follow-up decisions and requires preservation of
all best numerical parameters, original receipts and negative outcomes.

## Approach

Add one explicit, default-false `initial_reference_safe_geometry` selector.
During stationary acquisition, construct the cage using at least the active
safety radius plus the existing feasibility margin (0.033 m here); preserve any
larger cage/lead offsets. Apply the existing uniform-ring option normally.
Restore the original geometry immediately when acquisition freezes. Unavailable
acquisition retains its existing zero nominal command and enlarged safety radius.
This changes a nominal target construction, not the safety filter, noise model,
numerical parameters, capture/support requirements, zero epoch or deadline.
The selector requires transport with initial-reference acquisition. It does not
claim QP feasibility or a dynamic safety guarantee.

Alternatives considered: reducing the safety radius violates the protected
safety contract; enlarging capture budgets or adjusting gains adds a confound;
changing the post-freeze controller does not isolate the observed acquisition
mismatch. The selected mechanism is the agent's decision under the user's
delegation, not a claim that the user prescribed this formula.

## Frozen physical comparison

After focused and full engineering checks, freeze eight episodes: the current
translation-all-captured controller versus that same controller with the single
selector enabled, for circle, L, far-C and far-skew. Reuse the exact four cases
from task_translation_physical_v2, including seed 20261102, initial signatures,
12000-step / 600-second limits, 40 settling frames and all success/safety gates.
Four workers, one internal thread; fixed shuffle seed 20261103. All new raw
outputs and source/environment snapshots go to a fresh runs/research directory.
Verify each baseline against the corresponding previously executed translation
candidate, including every replay array and scientific JSON field.

The advance gate requires all eight executions, four exact baselines, all four
candidate safe completions with no false HOLD and valid zero epochs, preservation
of each baseline G500 pass, zero acquisition barrier scalings in every candidate,
and no increase in post-acquisition scalings in any matched condition. Report
all total/phase-specific scalings, endpoint error, time, hard safety, G500,
acquisition status/epoch and available feedback errors. An acquisition timeout
is a failure; do not enlarge its 60-second budget. Keep the selector default off
regardless of this small development screen. No additional outcome-driven cases,
per-shape choices, retuning or relaxed gates within this experiment.

## Remaining goal

Wider protected-success retention, unused conditions, rotation applicability,
deployment-only matched comparisons, decentralized information limitations,
locked-environment physical reproduction and the evidence-aligned CCTA2027 draft
remain outside this eight-episode increment. Do not claim the full goal complete.
