# Development snapshot — 22 October 2026

Baseline: `e4b0a19`, the locally and remotely verified tip of
`paper/boundary-aware-safe-deployment` at push preparation. Today's work is
collected on `codex/reliability-theory-manuscript`.

## Reliability and experiment evidence

[Revision report](RELIABILITY_REVISION_2026-10-22.md) and
[curated results](results/reliability_2026-10-22/README.md) retain the failure
classification, all rejected development comparisons, the final default
comparison, source snapshots, integrity hashes, and the prospective validation
freeze. The default remains at 5/9 CVT development completions; this is not an
independent generalization result or demonstrated reliability gain. The 832-case
prospective campaign has not been executed.

## Literature and research direction

[Literature review](literature_review_2026-10-22/README.md) contains the first-pass
reading record for 70 PDFs (871 pages) and one historical audit document, the
capability matrix, project audit, theory checks, and Markdown manuscript draft.
First-pass reading does not certify every proof, figure, or publication version.
Song/Bai/Wakamiya is the team's prior work and must be cited as such.

The current research hypothesis is task-directed preparation under incomplete
local boundary observations: diagnose whether a bounded pushing configuration
supports a specified task, gather additional evidence or reconfigure when
needed, and issue a justified start decision. It is not an established novel
capability or an implemented joint algorithm. Initial scope is a stationary
planar rigid object, short directional pushing, and endpoint/stop criteria under
explicit sensing, communication, and physical assumptions.

The next decisive evidence should use matched configurations with similar
spacing/coverage but different task outcomes, then test whether diagnosis-led
changes outperform equal-budget alternatives. Report false starts, unconfirmed
cases, preparation cost, and all execution outcomes. Accurate state observation
is not assumed necessary for every possible transport strategy. Supplementary
work includes sequential caging, Caging in Time, task-oriented contact
optimization, and object-agnostic contact-configuration regulation; their
relationship to the candidate contribution still needs full comparison.

## Theory and manuscript status

[Theory supplement](../theory/static_deployment/applicability_and_residual.md)
records the scope and residual argument. Static approach guarantees do not imply
safe moving contact or full transport success. The TeX/PDF draft authored earlier
today is included as an existing historical artifact; the subsequent literature
and positioning stage did not edit TeX or launch a large experiment campaign.

## Verification and archive boundaries

- Pre-push verification: 593 tests passed and two skipped on the first run;
  36 temporary-directory setup errors were rerun with a workspace-local base
  directory and all passed. Combined coverage: 629 passed, two skipped.
- The smoke protocol dry-run expanded to five cases successfully.
- All 20 curated evidence payloads matched their recorded SHA-256 values in
  the Git staging area. CRLF is retained for byte-identified evidence.
- Git-tracked changes and all non-ignored new project files are included.
  Approximately 2.1 GB of raw reliability run products remain under ignored
  `runs/reliability/`; source archives and compact results are committed.
  Downloaded reference PDFs/DOCX and extraction caches remain under ignored
  `tmp/literature/`. Virtual environments and test caches are also local only.
