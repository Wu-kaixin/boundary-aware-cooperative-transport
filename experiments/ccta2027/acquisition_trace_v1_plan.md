# Acquisition failure trace implementation plan

> For agentic workers: use superpowers:executing-plans, inline implementation,
> then one fresh final review under superpowers:requesting-code-review.

Goal: identify why skew capture fails and where the original QP becomes infeasible.
Architecture: frozen forced-pose controller replay, observer-only instrumentation,
then compact analysis and figures. Stack: root .venv, existing NumPy/matplotlib.
Spec: acquisition_trace_v1_design.md. Base:58b03f1424f463ed888ed85cc93ffd7fd6acda1e.

Constraints: unchanged runtime/configuration/old evidence; no new physical episodes;
new output directories only; truth for diagnostics only. User delegated research
choices and normal research-branch publication, so no additional design approval.

Review focus: frame index alignment; frozen imports; observer side effects;
capacity rejection accounting; individual versus joint QP feasibility.

- [x] Implement scripts/trace_initial_acquisition.py; snapshot and run both20-frame
      preflights, then exact791/1201-frame traces with terminal receipts.
- [x] Analyze all capture/support and scaled-QP events in a separate helper;
      publish compact tables and a readable geometry/support figure with hashes.
- [x] State which hypotheses the evidence supports or refutes, and define the
      next mechanism without changing numeric parameters or relabeling old runs.
- [x] Review once, resolve Important findings, check publication integrity and
      prepare the existing research-branch publication. Post-commit push and
      exact-head CI receipts are maintained with the joint circulation increment.

Both preflights exact20/20. Full baseline session29611 and candidate23044 terminal
exit0:791+1201=1992 exactframes. Analyzer46121 terminalexit0. Capacity rejection0,
peak818/4096, candidate support3; acquisition1422scaled and deadline-first-frame4
additional. All4 selected witnesses independently agree under5 constraint sets.
Both bodies remain truly stationary throughout these prefixes. Figure inspected.

Ruling: capacity is not the cause; support and motion stall at a crowded observed
edge. Proceed under separate acquisition_circulation_v1 design, retaining all old
failures and numerical parameters. One final review will cover both increments;
no separate diagnostic-only reviewer or premature push is needed.

Joint final review of58b03f1..5d127c5 found no unresolved Critical/Important/Minor
issues. The one minor caption ambiguity was clarified without changing historical
figure products. See docs/results/acquisition_circulation_v1/final_review.md and
the corresponding maintenance github_sync.json/github_ci.json after publication.
