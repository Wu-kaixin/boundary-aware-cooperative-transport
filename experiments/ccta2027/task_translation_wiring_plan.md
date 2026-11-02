# Optional translation-reference controller wiring: execution ledger

Status: opt-in runtime wiring implemented, defaults unchanged; physical pilot
NOT frozen or executed. All four replay conditions are complete and the original
screen remains FAIL. The new interface is preparation for the separate physical
question in task_translation_physical_pilot_draft.md, not candidate adoption.

The frozen replay process (exec 25427) was observed terminal before src/ changes.
Its completed data and failed final aggregation are separately preserved and
audited by zero-update finalization. Tests live under tests/; the replay's frozen
design, protocol, source snapshot and data were not edited for this interface.

2026-11-03 RED: tests/test_task_translation_wiring.py executed using root .venv:
**7 failed, 1 passed in 2.39 s**. Failure causes are the expected absent selector
fields (KeyError in the protected-value test) and unknown-controller-parameter
rejection in the planned selector cases. The existing BRAKE vector-goal behavior
test passes: positive remaining distance does not request ring retraction, while
arrival/HOLD does. This is not a physical success test.
Log/XML: runs/maintenance/2026-11-03-task-translation-completion-v1/wiring-red.*.

After implementation, the new tests plus task-translation, initial-task-reference
and reference-surface tests pass: **33 passed in 3.44 s** (`wiring-green.*`).
The first full-core invocation had **819 passed, 2 skipped, 65 setup errors**:
pytest could not create its numbered directory in the sandbox default temp.
The original log/XML (`core-wiring.*`) remain unchanged. A fresh invocation uses
`--basetemp=runs/maintenance/2026-11-03-task-translation-completion-v1/core-temp-v2`
and writes `core-wiring-v2.*`; it completed with **884 passed, 2 skipped in
145.38 s**, session 79303 exit 0. Public engineering receipt:
`docs/results/task_translation_completion_v1/engineering_checks.json`.
The new runtime fingerprint is
`9c7927a57d614f6f2d18757c2677c7af8829203882d3b40effd43043f3afa948`.
Published 4e1bb4c passed 876 / 2 skipped on both CI platforms; that older CI does
not validate this newly wired runtime.

MAS without hardware also passed **106 / 106**. Repository health, 12 original
replay product hashes, the frozen replay snapshot, publication artifact hashes
and the new engineering source identity all passed their checks. The separate
receipt is `docs/results/task_translation_completion_v1/prepublication_checks.json`.

## Implemented minimal interface

- initial_reference_motion_model: default 'se2'; only 'se2' / 'translation'.
- initial_reference_use_all_captured: default False; require a real boolean.
- Non-default choices require initial_task_reference and its existing acquisition
  constraints. Default settings preserve existing methods and all numeric values.
- Keep original freeze policy, epoch and strong-supporter diagnostics. Initialize
  the selected trackers from the already-frozen original buffers only. Empty
  captures remain uninitialized forever; later scans cannot supply a new zero.
- Select TranslationTaskReferenceTracker only for the opt-in translation model.
  Keep explicit model-assumption and uncertified labels, including uninitialized
  tracker records. Missing current scans must revoke fresh navigation/stop votes.
- Extend the existing protected-receipt test to remove/assert these two new
  default selectors before comparing every original parameter. Do not modify
  the receipt or canonical YAML.
- Do not add BRAKE phase rollback, alter gains/arrival bands, or change physical
  task/safety evaluation. Existing vector-goal control already permits continuing
  motion while BRAKE and has not reached common HOLD.

Ruling: implement an explicit default-off interface after examining all four
replay outcomes, without promoting the candidate — more accurate readings ask
for further motion on old L/C paths, and the existing vector-goal BRAKE law can
request it; only a separately frozen physical trial can answer whether that
motion occurs. Cost if wrong: an unsuccessful exploratory trial, retained as a
failure; no change to the protected default or old screening conclusion.

Ruling: preserve the first full-suite temp-directory failure and use a fresh
workspace-contained temp directory for the rerun — the observed errors occur
before test bodies and do not establish algorithm failures. Cost if wrong: the
new run will retain and expose any remaining real test failure.

Ruling: preserve the existing environment and lock file during publication —
inspection found installed joblib 1.5.1 / PyYAML 6.0.2 versus locked 1.6.0 / 6.0.3.
No dependency installation or silent lock rewrite accompanies these results.
Cost: exact lock-environment reproducibility remains a declared future check.

Any exploratory pilot needs its own frozen source, protocol, parameter comparison,
resource budget and complete outcome reporting. No physical performance claim
follows from the engineering tests.

Final whole-increment review: 4e1bb4c..917b5517, no findings requiring action.
`docs/results/task_translation_completion_v1/final_review.md` records its scope.
Implementation task complete; physical pilot and candidate adoption remain open.
