# DBACT branch integration audit — 2026-09-01

This is the decision record for the nine-branch consolidation into
`DBACT-research-v3` and `DBACT-publication-v3`.  It is deliberately a selective
integration, not an all-merge.  The row-level disposition of every commit that
is unique to the reviewed source tips is in
`INTEGRATION_COMMIT_DECISIONS_2026-09-01.csv`.

## 1. Precedence and non-negotiable invariants

`Claude-boundary-aware-closed-loop-v2` at `57d9d3b6819df0a85cd80b4d70caea8a060b2889`
is the scientific source of truth.  In conflicts, the following remain
authoritative:

1. the symmetric `arrival_tolerance` gate and progress-gain correction;
2. the unscreened 180-case shape-matrix evidence and its denominator;
3. `docs/CONDITIONAL_GUARANTEE_V2.md` and its four-way claim ledger;
4. the SE(2) error audit and default-off yaw estimator;
5. the re-derived ISSf margin, including its unsatisfied general-yaw regime.

No source branch may reintroduce an “arbitrary shape”, unconditional finite-time,
formal-caging, or achieved-perception-bound claim.  Publication code may read
research traces and committed results, but may not change controller, contract,
certificate, or result data.

## 2. Frozen pre-integration branch heads

Before either integration branch was created, lightweight archive tags with the
prefix `archive/pre-integration-2026-09-01/` were made for all nine source heads.

| Source branch | Frozen SHA |
| --- | --- |
| `main` | `72e5ef0c7940fbe9e4682b1b8527f99e59dccbcb` |
| `A-boundary-aware` | `1751eea4fc3ec5f9b554ea70863d9ece10622045` |
| `A-boundary-aware-closed-loop-v2` | `3faf1192911c297bb263f6677e195a803c3f1dc3` |
| `A-boundary-aware-closed-loop-v3` | `c038829487ad7b6a15046dc79b9dfc95b3bfe83c` |
| `Claude-Codex-boundary-aware-closed-loop-v3` | `093e1630afa3a36d8ad3773d532fed1a9c68fc96` |
| `Claude-boundary-aware-closed-loop-v1` | `92ee6f68fc6584c9a288dcf0bad556915e7a1c34` |
| `Claude-boundary-aware-closed-loop-v2` | `57d9d3b6819df0a85cd80b4d70caea8a060b2889` |
| `Codex-boundary-aware-closed-loop-v1` | `e4f32214f6ea3ffec5123f2d3cefe81aba2cab5c` |
| `Codex-boundary-aware-closed-loop-v2` | `ffd272e1c75e6c6c4b6931ffb2f12684529dcfd0` |

`A-boundary-aware` and `Claude-boundary-aware-closed-loop-v1` have no commits
unique to their tips relative to the scientific source; both are already
ancestors of it.

## 3. What was integrated

### Research branch

`DBACT-research-v3` starts exactly at `57d9d3b`.  No raw source-only scientific
commit was replayed.  This is intentional: the valid Codex theorem, matrix,
SE(2), degradation, robustness, distance, and publication-analysis content had
already been re-derived over the Claude-v1 API in the `6774d62..57d9d3b`
canonical stack.  Replaying the older commits would duplicate implementations
and restore conclusions that the v2 ledger later narrowed or refuted.

The three rewritten Hybrid history commits are retained by tree-equivalent
canonical SHAs:

| Historical SHA | Canonical SHA | Disposition |
| --- | --- | --- |
| `ab8f750` | `6774d62` | identical tree; certificate commit message corrected |
| `742b251` | `27fca06` | same scale-relative matrix port after history rewrite |
| `d5ce40a` | `592f68e` | same decisive-matrix result after history rewrite |

### Publication branch

`DBACT-publication-v3` is based on `DBACT-research-v3` and selectively takes two
Hybrid commits:

| Source SHA | Integrated SHA | Content |
| --- | --- | --- |
| `3fb7f240906390ea98cf34f59f2ae568d4acd1bd` | `d18dbf1` | immutable trace schema, renderer, HUD/overlays, offline animation, Figures A–G, tests |
| `093e1630afa3a36d8ad3773d532fed1a9c68fc96` | `c88cb79` | predeclared success/failure selection, manifest and hashes, paired Figure H, replay/package tests |

Those commits consolidate the useful `Codex-v2` V2-2 through V2-8 publication
capabilities without importing that branch's controller or stale README/results.
The V2-9 binaries were not copied: they were produced by a different controller
history and do not establish provenance for the current source of truth.

The first full package build exposed a missing runtime contract: MP4 was
mandatory, but no dependency file declared an encoder.  Publication commit
`1c9afd9` adds `imageio-ffmpeg>=0.6`, performs the encoder check before paying
for a rerun, documents the dependency, and tests the preflight.

## 4. Conflicts and rejected whole-branch merges

The two Hybrid commits cherry-picked without textual conflicts.  Their code is
additive and does not touch the scientific core.

The `main` performance line was probed against `57d9d3b` rather than guessed
through.  Applying `398ef0b` produced conflicts in `README.md`, both dependency
files, boundary density, cargo, controller, local CVT, metrics, and run-sim, plus
a modify/delete conflict for `src/dbact/distributed_cbf.py` (that alternate
module does not exist in the scientific source).  `edce854` is layered on
`398ef0b`, so it is not independently portable.  These optimizations are
deferred until they can be re-derived on the canonical controller and pass
trajectory-equivalence tests; resolving those conflicts by choosing one side
would be a scientific change, not integration hygiene.

The raw A/Codex-v1 closed-loop and guarantee commits are rejected as a unit for
the same reason.  Their useful mechanisms are superseded by the v2 ports, while
their broad guarantee, achieved-error-bound, and stale README wording conflicts
with the current claim ledger.  Merge commits are archived but not replayed,
because they add no content beyond their parents.

## 5. Verification and provenance

### Scientific baseline

On Windows 11, CPython 3.13.13:

- `python -m pytest -q`: **449 passed, 3 skipped** in 35.06 s on `57d9d3b`;
- `python scripts/verify_refactor.py --stage all --steps 900`: **S1–S7 PASS**;
- representative S7: `J=2.2476959803`, efficiency `0.9990753511`, minimum
  inter-agent distance `0.3406943602`, zero fallback/infeasible solves.

The current environment collects fewer tests than the “463 tests” text in the
historical commit message.  This audit records executed results, not inherited
commit-message counts.

### Publication pipeline

After the two Hybrid ports, the full suite produced **471 passed, 3 skipped**.
After the MP4 preflight fix and this audit were applied, the final publication
tip produced **472 passed, 3 skipped** in 36.91 s; its focused
publication/visualization set produced **9 passed**.  The final publication tip
also passed `verify_refactor` S1–S7 with the same S7 values reported above.

The selection-only build reproduced the frozen denominator and predeclared
cases:

- 54/180 unconditional successes;
- 41/149 runtime-eligible successes;
- representative: `u_shape__a0.40__seed000`;
- high-concavity stall: `star10__a0.40__seed001`.

Frozen sources:

| Item | Provenance |
| --- | --- |
| Matrix research SHA | `742b251477095b6eec0428c6d4a6ae43e2a5ac67` (maps to canonical `27fca06`) |
| `episodes.csv` SHA-256 | `931eb6889673497816ee22e146ab29c70a5064c58342556e38b58f5c9721a96f` |
| matrix manifest SHA-256 | `10a6a0c819ea2c01996405a9712df9e237a618e5ae119f4053e0f0dd086fdf42` |
| matrix config SHA-256 | `d268c46ea87dde60e37f40d7d9830be51a270f6c1761d4f718fd77beedd35962` |

A current-controller full build correctly failed closed.  The formerly selected
success changed from `success=true`, 288 frames, `J=1.3032826232` to
`success=false`, 259 frames, `J=1.0528567978` after the arrival/progress changes.
The high-concavity stall reproduced every checked field exactly.  Re-running
with `--allow-numeric-drift-preview` generated 23 hash-indexed artifacts and a
manifest with `status=preview_only`, `publication_eligible=false`; those local
preview artifacts are not committed as evidence.

Therefore the frozen 180-case matrix remains valid historical evidence for the
controller SHA recorded in its manifest, but it is not a current-controller
result.  A publication-ready showcase requires a complete unscreened 180-case
rerun at the final `DBACT-research-v3` SHA with an environment fingerprint,
followed by selection from that new frozen table.  Two representative reruns
cannot replace that denominator.

## 6. Branch and PR policy

- Push all nine archive tags before moving any integration ref.
- Push `DBACT-research-v3` and `DBACT-publication-v3` as new branches; do not
  rewrite or delete old branches.
- Open `DBACT-research-v3 -> main` as a pull request with the final head SHA,
  tests, verify report, and this decision record.
- Do not merge `DBACT-publication-v3` into `main` with the research PR.  It stays
  downstream until the current-SHA matrix rerun removes the documented
  publication blocker.

