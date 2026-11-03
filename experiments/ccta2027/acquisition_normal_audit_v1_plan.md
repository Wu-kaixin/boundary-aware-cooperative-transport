# Acquisition normal audit execution ledger

Base: 9bbe274504167c4f7a9aab34bedefa9c40e54a0a. Binding frozen design:
acquisition_normal_audit_v1_design.md. This separate ledger can be updated without
changing the design snapshots already recorded by running diagnostics.

The previous increment published the physical study and verified exact-head
two-platform CI. This increment is retrospective diagnosis only. Runtime,
physics, numerical parameters and original gates remain unchanged.

- [x] 20-frame preflight completed exit 0, exact commands/diagnostics/phase/map.
- [x] C 375-frame and skew 409-frame full traces completed exit 0, all 26
  acquisition scaled events retained; all concern agent_05.
- [x] Four candidate phase monitors reconstructed exactly over 2,589 frames;
  every transport_active count verified against previous-step diagnostics.
- [x] C 1,326 and skew 483 normal-fusion checks, maximum residual 5.55e-16;
  2,437/40 source returns from 794/30 scans reconstructed. Correlated records,
  not independent reliability samples.
- [x] All selected cells project beyond their nearest true finite edge;
  true margin positive in all 26 states, original h negative in all 26.
  Observer-only true incident-plane correction leaves 25 negative h values;
  no counterfactual QP or physical run is implied.
- [x] C never meets quorum 4 before BRAKE at 41.25 s; first quorum at 46.05 s.
  Maximum active 5 occurs only in BRAKE, not an eligible missed transition.
- [x] Publication completed exit 0, PNG visually inspected and readable;
  README explains mixed native frame numbering and contrary evidence.
- [x] Publication integrity PASS: 7 documents / 293 local links, all products
  and 26 CSV rows, phase CSV, old physical products, source/environment and
  protected config verified. Receipt publication_integrity_v1.json.
- [x] Repository health PASS on 5bfdbefd23f0effc415ff1fdc4f1c4cd4deb223b:
  1,480 files / 471 Python parses / 46 legacy evidence hashes, pip/fsck PASS.
- [x] One fresh final reviewer (gpt-6-astra/high), no Critical/Important/Minor
  findings. Independent hashes, all26 CSV rows, lineage and all2,589 phase
  responses checked; no fix pass or second review required.
- Publication is ready. Final remote-head/readme equality and two-platform CI
  will be recorded after the last documentation commit in local github_sync.json
  and github_ci.json under this increment's maintenance directory, avoiding
  another commit solely to record that commit's own CI result.

Raw outputs and frozen helper/design copies are in
runs/research/2026-11-03-acquisition-normal-audit-v1/; logs in the same-named
runs/maintenance/ directory. New physical episodes: 0. The design's original
unchecked execution list remains frozen; this ledger records current status.

Next research direction: finite observed boundary support with explicit unknown
continuation, sensing uncertainty and conservative missing-motion bounds. No new
candidate is implemented or adopted. Preserve the original four-agent quorum
and braking rules; skew's six post-freeze events remain outside this audit.
Any repair needs a separate design and frozen physical study.

The latest user request is GitHub publication, README and remaining problems.
Complete this deliverable before further runtime work. The broader research
goal remains active and incomplete; research-branch publication is authorized.

One fresh final reviewer completed this increment's review, per executing-plans.
The completed previous publication was not reviewed again.

Final: Ruling: prior physical efficacy remains the separately reviewed four reused
conditions; zero new episodes here — avoids conflating evidence — cost if wrong:
unsupported general success rates or causal claims.

Final: Ruling: no finite-boundary repair safety is established — require a separate
design/validation with unseen continuation and motion bounds — cost if wrong:
omitted obstacle constraints and unsafe behavior.

Final: Ruling: six post-freeze skew events remain unresolved — the audited prefix
does not cover them — cost if wrong: false complete-chain acceptance.

Final: Ruling: retain old regression identity and verify publication-head CI on
both platforms — this audit did not rerun the core suite locally — cost if wrong:
undetected integration or platform regressions.

Final: Ruling: no locked-environment equivalence claim — current dependency
differences remain disclosed — cost if wrong: unsupported reproducibility.

Final: no deferred minors in this increment. The earlier physical-study suggestion
is unchanged. Final review text: docs/results/acquisition_normal_audit_v1/final_review.md.
