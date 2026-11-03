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
- [ ] Repository health, one fresh final review and normal publication.

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

One fresh final reviewer will cover this new increment when ready, per
executing-plans. The completed previous publication is not reviewed again.
