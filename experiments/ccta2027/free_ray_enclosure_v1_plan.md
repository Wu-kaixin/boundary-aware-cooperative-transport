# Free-ray enclosure implementation plan

> For agentic workers: use superpowers:executing-plans task by task. Root .venv
> only; preserve original data. The user delegated routine design decisions.

**Goal:** determine whether permitted free-ray information can constrain the
unknown center without geometry truth or a shape prior.

**Architecture:** a pure geometry component returns a conservative conditional
convex set or the original workspace. A separate audit reconstructs frozen
sensor inputs and scores all candidates, without advancing physics.

**Tech stack:** existing Python / NumPy / Shapely in root .venv.

**Spec:** free_ray_enclosure_v1_design.md. Full objective: active_goal_20261103.md.

## Global constraints and review focus

- Keep canonical parameters, source snapshots, raw runs and old conclusions.
- No gap interpolation, simulator-normal leakage, center truth or task-start input.
- Every feasible face remains; empty/inconsistent evidence cannot shrink the set.
- Centroid containment uses a convex hull, including for nonconvex objects.
- Control-frame/time alignment and zero-epoch verification must be explicit.
- All-team input is a diagnostic bound, never labeled one-hop execution.

## Tasks

- [x] Write tests in tests/test_free_ray_enclosure.py: closed measured ray loop,
  dangling rays, absent hits, contradictory hits, all feasible cells retained,
  concave component convex hull, rigid-coordinate covariance, invalid arrays.
- [x] Run the failing tests with .\.venv\Scripts\python.exe.
- [x] Implement src/dbact/sensing/free_space.py: free_ray_enclosure(starts, ends,
  target_points, workspace_vertices, margin) -> record with vertices, status,
  face counts and area; retain all feasible faces and use the workspace if none remain.
- [x] Run component tests: 18 component tests pass; 39 tests pass with initial
  task-reference and protected surface-reference regressions.
- [x] Obtain one final independent read-only mathematical/code review of the full change.
- [x] Implement scripts/audit_free_ray_enclosure.py with frozen protocol and fresh
  runs/research/ output, independent expected case identities, sanitized raw
  packets, original scan-count checks, and complete per-agent results.
- [x] Freeze then run four recorded acquisition snapshots; report every failure
  and source/input hash. Do not modify the frozen script while executing.
- [x] Use measured local closure, containment and transform ambiguity to decide
  the next runtime mechanism; keep the full goal active until all requirements
  in the completion ledger have direct verification.

## Execution ledger

2026-11-03: geometry tests failed first because the module did not exist, then
12 passed. Packet-sanitization tests failed first because the new function did
not exist, then all 18 passed. Combined targeted suite: 39 passed. Frozen source
hash: 6892bbbd247118891622c46ed0c112ffe31714b17435af730b6f246089475112.

Ruling: one independent review at the end of this complete change, rather than
an extra component review before the observational audit — follows the inline
executing-plans workflow; the audit cannot change control or physics — an error
found in review could invalidate this audit, in which case its receipts remain
preserved and a new declared audit would be required.

2026-11-03: all four frozen snapshots completed, 147.38 s, no physics advanced;
64 original scan-count checks and old stop-bound recomputation passed. Local
contraction 0/128; all-team contraction 2/4; observed object/center exclusions 0/132.
The four-panel PNG was visually inspected. Report/README keep the candidate
unwired and the entire goal active; acquisition quality and task observability
remain separate next steps.

Final review: fresh-context reviewer checked the complete 7da54cc..3a9dcda change,
owning scanner/controller logic, source and original product identities, all
aggregates and the figure. No Critical, Important or actionable Minor findings;
no invalidation or fix required. Full core suite: 872 passed, 2 skipped in
130.39 s. Repository health: PASS at committed 3a9dcda (health-tracked.json;
1255 tracked files, 423 parsed Python files). The reviewer cited the earlier
health receipt; the executor independently checked this later tracked-HEAD receipt.

Final: Ruling: closed-loop reliability, dynamic safety and preservation of all
transport successes remain outside this zero-physics diagnostic — the component
is unwired and cannot establish them — treating it as a repair would leave the
original false-stop/stall failures unresolved.

Final: Ruling: no unconditional Gaussian-tail or hardware missed-detection claim —
the mathematical result is conditional and the driver contract requires complete
raw scans — deployment without those conditions could exclude the real object.

Final: Ruling: temporal accumulation, relaying, dropout and active observation
require separately frozen work — a single snapshot does not execute them —
assuming them here would overstate available local information.

Final: Ruling: deployment-method superiority and CCTA2027 readiness remain open
in the full-goal ledger — no new comparison or manuscript revision was executed —
marking them complete would misrepresent the research evidence.

Final: Ruling: retain the recorded PyYAML environment identity and defer its
reconciliation to a separately declared check — changing dependencies during
this frozen study would invalidate its identity — locked-environment physical
reproduction is still unverified.
