# Reference reacquisition implementation plan

> For agentic workers: REQUIRED SUB-SKILL superpowers:executing-plans for inline
> implementation. One fresh final review under superpowers:requesting-code-review
> covers this and the preceding contact-activation diagnosis together.

Goal: let a late robot regain observations without granting missing-reference
transport permission, then test whether the entire far transport failure improves.
Architecture: one default-off selector and narrow override branch; original QP
and all reference/map functions remain owners of their existing constraints.
Tech stack: root .venv, Python/NumPy, existing frozen simulation/evaluators.
Spec: reference_reacquisition_v1_design.md; evidence contact_activation_v1.

Global constraints: canonical config SHA a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112;
no numerical/safety/phase/physics/acceptance change; new runs only; all old source,
receipts and negative conclusions unchanged. User delegated mechanism choices and
normal research-branch push. No external environment, force push or main merge.

Review focus: false-typed selectors; fresh versus unavailable reference; failed
initial capture; transition into TRANSPORT/BRAKE/HOLD; implicit pushing/stale
targets disguised as an acquisition mode. Keep these boundaries in focused tests.

- [x] Add tests/test_reference_reacquisition.py and observe RED. Full-controller
      search/recall must be nonzero only when enabled, frozen and before transport;
      approach likewise, but press/effort/other modes/later phases remain zero.
- [x] Add reference_map_reacquisition bool/guard in src/dbact/control/controller.py
      and candidate-only metadata in src/dbact_sim/experiments/deployment.py.
      Update the two legacy parameter-shape tests to account for the new false
      selector. Focused GREEN then full root pytest; protect canonical hash.
- [ ] Create scripts/reference_reacquisition_study.py with freeze/run/analyze,
      using unchanged original episode/reproduction evaluators. Freeze8 cases,
      execute once, assert exact old4 and unchanged candidate acquisition prefix,
      report mechanism and full-chain gates separately with all negative results.
- [x] Prepare the evidence/README publication checkpoint, one fresh final review,
      resolve Important findings once and verify integrity. Remote publication
      and exact-head CI are recorded after push in the local maintenance receipts.

Commands use .\.venv\Scripts\python.exe explicitly. Tests/experiment logs go in
runs/maintenance/2026-11-03-reference-reacquisition-v1/. Scientific freeze/run data
go in runs/research/2026-11-03-reference-reacquisition-v1/. Helper/script/environment
identity stays fixed during physical execution and analysis; no duplicate starts.

2026-11-03 publication checkpoint: RED16 expected failures, focused GREEN57,
full root pytest928 passed/2 skipped (terminal exit0). Canonical parameters remain
unchanged. The latest user request is to publish current results and README now;
the study helper, executable freeze and all8 physical episodes have NOT run.
Publication covers diagnostic evidence and default-off engineering only. The
prospective study stays unchecked and must retain its original acceptance gates.

Final: fixed missing boundary regression coverage — six new cases reject three
isolated synthetic source faults (1+2+3 failures), then focused63 and full934
passed/2 skipped. The original implementation's RED16/GREEN57/core928 receipts
remain unchanged. One new test fixture initially referenced the wrong statistics
attribute; its failure is retained separately, not attributed to runtime behavior.
No runtime code or numerical configuration changed in this review fix pass.
Final review: one fresh read-only reviewer, c8f93b2; no second review. No Critical
or deferred Minor findings. Important verification gap resolved by executor tests.
Final: Ruling: closed-loop performance remains unjudged — publish default-off at
the user's requested checkpoint, with eight episodes explicitly pending — cost
if wrong: the candidate may still fail transport; no adoption claim is permitted.
Final: Ruling: future observation recovery/contact/dynamic safety remain unjudged
— eight witnesses certify only one-step conditional feasibility — cost if wrong:
later recovery may fail or produce contact/constraint conflict.
Final: Ruling: adoption/generalization/rotation/decentralization/hardware remain
unjudged — keep them outside this research checkpoint's claims — cost if wrong:
unsupported full-system readiness claims, so those validations remain outstanding.
