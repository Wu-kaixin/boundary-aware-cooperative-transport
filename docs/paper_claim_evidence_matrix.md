# Current manuscript evidence (22 October 2026)

The older matrix below is historical. The authored draft uses
[v2 audited results](CCTA2027_EXPERIMENTS.md),
[its current claim/source map](../manuscript/ccta2027/sources.md), and
[the static applicability supplement](../theory/static_deployment/applicability_and_residual.md).
Development repair replays are separate from the frozen v2 scientific tables.

---

# Claim–evidence matrix

The table below is the immutable October 7 evidence snapshot. Current repairs,
the separately frozen CCTA revision and its verdicts are documented in
[CCTA2027_EXPERIMENTS.md](CCTA2027_EXPERIMENTS.md); populations are not pooled.

Updated 7 October 2026 from executed runs. Batches are not pooled. EMPIRICAL_ONLY except the stated static-oracle scope.

| Claim | Code | Evidence | Verdict |
|---|---|---|---|
| Faster controller CONTACT_READY than nearest observed target | `boundary_density.py`, `local_cvt.py`, `deployment_variants.py` | Development 9 pairs: mean 4.383 s vs 8.983 s, 8 faster, median difference −0.75 s. Frozen test 10 complete pairs: mean difference −4.37 s, median −0.77 s, bootstrap 95% [−10.02, −0.32] s. Two test pairs censored. | SUPPORTED against this control. Small median. Not a powered general law. |
| Faster preparation than an information-matched dispersion baseline | `deployment_variants.py` local spacing | Development mean 4.383 s vs 4.306 s. Test: CVT slower on 9/11 pairs, mean difference +0.69 s, interval includes 0. | NOT_SUPPORTED |
| Quorum physical-readiness latch is reached sooner | `run_deployment_pairs.py` physical streak | Test: 13/36 episodes ever latch; 3 complete CVT–nearest pairs; interval includes large positive and negative values. Development latch fired once, on the C seed 8 overrun. | INCONCLUSIVE. Definition not relaxed. |
| Unknown-boundary transport is safer | transport `safety_filter.py` | Moving-body status remains UNRESOLVED. Static 65 mm rule is not the transport contact rule. | INCONCLUSIVE for transport |
| Static local path clearance can be repaired without an oracle map | `local_boundary_features.py`, `paper_evaluation.py` | Original local: 8/9 OBSERVED_VIOLATION, L seed 2 path minimum 48.33 mm. Segment candidate: 9/9 CERTIFIED_SAFE, minima 65.85–75.92 mm, displacement 0.133–0.284 m. Oracle: 9/9 CERTIFIED_SAFE. | EMPIRICAL on this static set. Less motion. No unknown-region certificate. Not used in transport. |
| Static oracle safety | `theorem_mode.py`, `qp2d.py` | 9 oracle reproductions certified by the path evaluator. Theorem text stays in `theory/static_deployment/`. | Limited to the oracle statement. Float QP is numerical. |
| Better transport of unknown geometry | `phase.py`, `transport_control.py` | Development task completion 4/9 CVT, 6/9 nearest, 6/9 spacing. Frozen test 1/12, 1/12, 2/12. Progress-error interval includes 0. | NOT_SUPPORTED |
| C seed 8 watchdog is a missing pusher quorum, not absent force | `phase.py` transport quorum | CVT max `transport_active` 3, quorum 4, true progress 3.095 m vs target 1.304 m, forward force remains positive. Nearest and spacing reach quorum and HOLD. Recovery releases the latch without creating a fourth pusher. | SUPPORTED for this case. Downstream, not a CVT credit. |
| Exploration density is necessary for the development result | `boundary_density.py` `explore_gain` | `no_exploration` matches full CVT trajectory hashes. Gain is already 0. | Null ablation. OPEN for a nonzero gain. |
| Gate 6 v2 60-pair study | missing paths | Not found on reachable branches, tags, or worktrees. | MISSING. Seeds 12–31 stay quarantined. |

Source and case hashes are in each run `manifest.json` and `case_result.json`. Tables and figures: `docs/results/evidence_closure/paper_evidence/`. Raw replays stay under `runs/evidence_closure/` and are not required to reinterpret the summary.
