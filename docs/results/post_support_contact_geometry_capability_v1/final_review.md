# Fresh-context final review: packet geometry and directional capability

Reviewed increment: `dea31ff8b344db35c98e0860bd2282e802977188..ee89aed7e99e33d45eba14e97f2a98d373763244`.

This is the one final review for this increment. I read AGENTS.md, the Serena manual, the requesting-code-review reviewer instructions, both designs, the execution ledger, all four added scripts, the three added tests, the changed narrative documents, and the relevant owning geometry/admission/replay code. Large evidence files were parsed programmatically rather than dumped into context. No subagent was dispatched. The checkout/index/HEAD and original data were left unchanged; only this report was written. Every Python invocation used the root `.venv\Scripts\python.exe` with `-B`.

## Strengths

- Capture is observational: the factory wrapper saves the actual packets and pre-command pose and returns the same original prepared problem. The predecessor wrapper still performs the original exact command/diagnostic/statistics/phase/map comparisons; complete selected-event output equality is also asserted. The frame counter agrees with the predecessor's one prepare call per agent per frame. The factory is restored in `finally`.
- Packet permissions and geometric feature construction match their owners. Own/current-reference admission precedes one-hop selection; the captured sources are restricted to the receiving agent and its actual permitted neighbors. Chords precede isolated-point guards, isolated radii use same-object returns across permitted packets, and coverage uses the configured motion bound while demand uses the received-speed bound. The runtime builder remains responsible for unsupported-gap and range guards before attributed rows are accepted.
- Pre-command evaluation geometry is separated from the subsequent recorded force interval and never enters the optimization systems. Raw packets and all captured source identities remain local, while the derived complete directional systems and certificates are published.
- The directional analysis preserves all original non-object rows and original speed limits. Only the declared uncapped comparison replaces object RHS values. The dual sign is correct for `A u >= b`: `V ||g + A.T lambda|| - b.lambda` is an upper bound for nonnegative multipliers. The normalized separator is likewise algebraically valid. Solver status alone is never accepted as a capability or infeasibility certificate.
- Evidence retains every denominator and guard. The 128-state table contains all 16 nonadmissions, all 3,813 object rows are retained, and the 112 admitted states each have exactly three systems. Failures and initial environment errors remain visible. The results explicitly distinguish instantaneous velocity feasibility from contact force, simultaneous team feasibility, and closed-loop success.
- Future unverified solves fail closed for publication: the analyzer defaults to INCONCLUSIVE, the verifier rejects unknown/unverified statuses, and the publisher requires zero inconclusive systems. The fixed cohort/count assertions prevent a changed cohort from being silently published as this experiment. These are intentionally cohort-specific research scripts rather than general-purpose analysis APIs.

## Independent checks performed

- `study.check()` passed before and after the independent analysis: runtime hash `e6b7456f5f0a4d71ddd4b33159c058e215115aae35c01b6c5c9eae7e68f32767`, protected configuration hash `a5d5c8e200fa502557d4da9a9db2a646d027abababd59661bcdac21a1b43b112`, and all 22 frozen helpers unchanged.
- Rechecked analysis/outcome hashes, captured source snapshots, goal-direction replay hashes, and every product named by the public publication manifest. Public directional certificates exactly equal the raw directional states. All public table sizes and the 17-state subset membership match; state classifications, guard labels and selected numerical columns match their source states.
- Independently checked all 128 saved positions, cargo centers/angles and timestamps against their replay arrays. Source IDs are unique and remain within own/one-hop permissions. Recomputed polygon distances/closest points with scalar segment projection and ray crossing. All evaluation distances and margins match within `1e-12`.
- Recomputed the complete ordered feature lists and owning-builder normals/margins; independently recomputed retained-feature endpoint distances, structural allowances, normals and residual/term-removal minima. All 3,813 retained rows match, including 3,378 chords and 435 isolated-point guards. Nonadmissions are 7 no-current-map-motion, 4 outside-range, 4 isolated-gap-exceeds-cap, and 1 no-current-own-returns.
- All 17 positive-force states have positive pre-command physical-floor margin, range 0.01705903670680836 to 0.06624074067344704 m. Their limiting sources are 10 own and 7 one-hop, all chords; 11 limiting finite-support h values are negative. Removing structural/noise/both terms on the fixed rows leaves 14/10/6 old-command conflicts. The structural and normal-alignment ranges match the report.
- Independently recomputed the public primal and dual/separator algebra for all 336 systems and checked every matrix/RHS/speed against its original saved counterpart. Maximum primal-dual gap is `1.659502985029704e-8`; worst primal row residual is `-5.551115123125783e-17`; largest speed excess is `5.551115123125783e-17`. The least positive reported goal speed is 0.053024692068682044 m/s, well away from the classification threshold.
- All 20 reverse certificates remain strict after simultaneously relaxing speed and each row by `1e-7`; the least negative relaxed upper bound is `-0.021381277903266507`. All 6 infeasibility separators remain strict under the same relaxation; the least negative relaxed separator is `-0.021942954206906742`. These are system counts, not 26 different states.
- Recomputed subgroup counts: all admitted old/capped/uncapped positive-reverse-infeasible counts are 112/0/0, 99/11/2, 99/9/4. Positive-force counts are 17/0/0, 6/10/1, 6/8/3. Positive-force declared-pusher counts are 11/0/0, 2/9/0, 2/7/2. The three positive-capability capped states with negative selected velocity are exactly circle 162 agent_09, L 124 agent_12, and skew-box 891 agent_06; the first two are declared pushers.
- Executed the three added test functions directly against the current analyzer, without pytest cache or temporary output; all passed. Also exercised empty-row positive capability and a zero-boundary case, which correctly returned BOUNDARY_OR_UNCERTAIN. This direct invocation is an additional check, not a replacement claimed for a full pytest run.
- Parsed and hash-checked the original test XML receipts: actual RED 3 failures; GREEN 3 passed; initial full run 1,011 passed / 2 skipped / 76 setup errors; unchanged repository-basetemp rerun 1,087 passed / 2 skipped / zero failures or errors. All 76 original error messages are the same pytest numbered-temp-directory creation failure. The health receipt hash also matches its published record.
- I did not rerun the 1,749-frame capture, the full suite, any writer main function, or any physical episode. I did not stop, restart, or otherwise interact with the parent's active full audit.

## Issues

### Critical

None found.

### Important

None found.

### Minor

1. **Qualify the root summary's causal ranking by its fixed-state scope.**
   - Location: `README.md:5` ("进一步确认主要瓶颈在约束本身"), reinforced by `README.md:9` ("约束兼容是主要问题，优化目标另有次要问题").
   - The complete result document carefully restricts the inference, and paragraph 9 provides the sampled-state context. However, the first root summary puts an unqualified ranking directly beside the 0/4 closed-loop outcome. A reader scanning that headline can infer that the principal cause of all closed-loop failures has been established, which the fixed old-control states and unfinished full audit do not establish. Similarly, a forward-feasible but reverse-selected velocity motivates further objective/nominal-velocity investigation; it does not independently establish a controller-objective defect or its closed-loop importance.
   - Suggested correction: qualify both ranking sentences with "在这组固定状态中", and say that constraints eliminate forward capability in 11/17 states while three remaining states require separate examination of nominal velocity/objective selection. Keep the existing statement that full closed-loop attribution remains unfinished.
   - This is documentation precision, not a numerical or implementation defect, and is nonblocking for the evidence increment.

## Recommendations

Make the small summary qualification before public dissemination if practical. Preserve the current certificate tables and failure receipts. Future mechanism work should continue under a separately declared design and the protected numerical parameters, as already stated in this increment; this review authorizes no candidate activation or safety-margin removal.

## Declined to judge

- Final results or success of the still-running full physical constraint audit: outside this increment and explicitly unfinished; its process was not polled or modified here.
- Whether the finite-support candidate should be enabled or adopted: the increment is observational and leaves it disabled; no new closed-loop adoption evidence is supplied.
- Causal completeness or the primary cause of all four candidate task failures: the sampled original-control states establish local constraint incompatibility, not the complete candidate trajectory mechanism.
- Whether changing nominal velocities or the optimization objective improves physical transport: positive-capability/negative-selection cases motivate this question but do not execute such a change.
- Sufficiency of the empirical three-sigma, structural-gap, and received-motion assumptions for unknown or unseen boundaries: no new deterministic world-safety guarantee is part of this diagnostic.
- Safety or benefit of deleting margins, dropping rows, changing admission rules or tuning numeric parameters: term omission here is fixed-row algebra only; no such runtime modification was made.
- Simultaneous multi-agent/contact-force feasibility and effectiveness of the per-agent maximizing velocities: the solved systems are instantaneous per-agent velocity problems.
- Population reliability, statistical significance, held-out generalization or independent-trial conclusions: the same correlated development trajectories/events are deliberately reused.
- Full deployment contribution, C-shape formal activation, acquisition/reference-bias repair, unattached transport/stop acceptance and CCTA2027 paper readiness: these broader research goals remain outside this diagnostic increment.
- Correctness of prior increments beyond the current capture/geometry/matrix/time interfaces inspected here: those increments have their own completed reviews; this review does not reopen them.
- Exhaustive robustness for arbitrary future cohorts, malformed packets, degenerate geometry or solver backends: this analyzer is expressly scoped to the frozen cohort; I checked its current numerical results and fail-closed publication behavior, not a general library contract.
- Current cross-platform CI results for this new head: local receipts were checked, but no new remote CI claim is made by this review.
- A fresh full-suite run or fresh exact-controller replay: explicitly prohibited for this review; preserved receipts and read-only source/data checks were used instead.
- Completing or revalidating the full paper manuscript and publication-venue requirements: neither the manuscript nor current venue rules were changed or assessed here.

## Assessment

**Ready to merge/publish this fixed-state diagnostic increment: Yes, with the nonblocking summary wording recommendation above.**

The implementation matches the two declared designs, preserves the protected runtime/configuration and original data, and the published numerical claims are supported by complete independently recomputed witnesses. This verdict applies to the diagnostic evidence increment only; it does not establish full research-goal completion, candidate acceptance, or completed closed-loop failure attribution.
