# Finite acquisition support: frozen eight-episode physical comparison

Question: does the guarded finite chord/isolated-return candidate remove acquisition
conflicts while preserving the latest observation-reacquisition transport results?
Four reused development conditions: near circle, near L, far C opening away, far
skew box. Two matched arms: original reference_reacquisition and the same config
with acquisition_finite_support=True. Exactly8 new physical episodes, at most
12,000 steps each, dt0.05s,600s budget, original40-frame/.005 settling criteria.
No new seeds, shape-specific adjustment, numerical tuning or extra retries.

All baseline configurations are copied from the frozen reference_reacquisition_v1
protocol unchanged. The new field is omitted in controls so it resolves False.
Candidate differs only in that boolean. Keep all best numerical parameters,
physics, original task tolerance0.12m, G500, safety/phase gates and no attachment.
Fixed shuffled schedule seed20261103; four workers with one numerical thread each.
Freeze source, scripts, design, environment, core-test receipt and original case
receipts before execution. Every failure and budget termination stays in denominator.

The candidate consumes current own/one-hop ray packets only. Chord and isolated
point guards remain empirical; this is not an arbitrary-unknown-shape certificate.
The existing stationary-acquisition model is retained. Use the new rows only while
initial_reference_status is acquiring AND the map model is stationary_acquisition.
On the freeze frame the radius already returns to transport size, so original
rows apply immediately. Missing coverage falls back to original rows. Report
row-reach caps separately; capped demand is not an uncapped barrier proof.

Mechanism gate (all required):8 execution completions; four original controls
exact in all25 replay arrays and scientific records; four candidate safe task
completions, valid original zero epochs, no false HOLD or original hard failures;
candidate acquisition scaling zero; no matched increase in post-acquisition
scaling; preserve every baseline G500 pass. Candidate capture prefixes are expected
to change, so exact prefix reproduction is NOT a criterion for this mechanism.

Complete-chain gate: all mechanism checks plus all four candidate original G500
passes and formal TRANSPORT entries. Do not force phase changes or reduce quorum4.
Report endpoint distance, time/termination, phase response, true rotation, contact
and safety metrics, reference epoch validity and guard use/fallback/locality.

Do not declare default adoption or general reliability on four reused conditions,
even if gates pass. A positive result must be followed by broader original-success
preservation, unused-condition validation and deployment-method matched controls.
A negative result is preserved and changes the next research decision; do not
quietly extend budget or weaken the preregistered mechanism/complete-chain gates.
