# Evidence closure progress

## Stage 0 — initial checkpoint
- Branch: research/simulation-evidence-closure, base 9693dfd6b5ef72af354b8f5dd3d8e1cc245b931a; clean starting worktree.
- Remote main and paper branch checked with git ls-remote; match supplied snapshot. No push.
- No applicable AGENTS.md found in workspace/root search. No referenced RAS PDFs located.
- Python D:/miniconda/envs/dbact/python.exe: 3.10.20, NumPy 2.2.6, SciPy 1.15.3, Shapely 2.1.2.
- Archived sources and readiness/error calculations audited; initial storyline and claim matrix saved before repairs.
- Gate6 v2 absent from reachable path history and searched worktrees. Search encountered inaccessible pytest caches; missing evidence remains a limitation. Seeds 12..31 excluded from held-out claims.
- Shell sandbox startup fails; approved elevated execution works. No research thresholds changed.
- Next: independent evaluation, compatibility schema, observer invariance test, full-horizon baseline reproductions.

## Stage 1 — instrumentation checkpoint
- Added evaluator-only full linear hold/solid polygon distances, versioned aliases and explicit unresolved states; original archive untouched.
- Added physical substep copies, true/estimated progress, force decomposition, contact identities, phase guards, controller diagnostics, endpoint errors and censored events.
- Corrected README/report generator descriptions of archived lower-bound failures and signed progress error.
- Validation: 40 targeted tests passed (paper evaluator, contact dynamics, QP, theorem mode), including 12-frame instrumented/uninstrumented exact trajectory equality. Static and transport five-frame paired smoke runs completed. These are smoke checks, not formal experimental evidence.
- Windows default GBK caused a read-only test failure; explicit UTF-8 fixed it. Run commands use PYTHONUTF8=1.
- Full 600-frame static and 3000-frame transport reproduction starts next. No thresholds or horizons changed. task_completed remains unresolved pending protocol definition.

## Stages 2–3 — measured checkpoint
- Full original reproduction completed: 9 static local/oracle pairs (600 frames), 9 transport pairs (3000-frame watchdog). Source/config hashes and trajectories under runs/evidence_closure/baseline_*.
- Independent path evaluation observed below-65-mm clearance in 8/9 local runs. L seed2: first control frame25, agent_14; path minimum 48.3328895 mm over full episode. At first violation sample-row residual remained positive. This is now new independent evidence, not reinterpretation of a bound as collision.
- Candidate static fresh segments: all 9 complete 600 frames, no observed threshold violation; minimum across cases 65.8548473 mm. Mean robot displacement ranges .133–.284 m. Unknown regions and distribution shift remain unproved. Deployment quality/stagnation still to analyze.
- Full regression at candidate checkpoint: 555 passed, 2 skipped. Phase recovery regressions: 16 passed.
- C seed8 original: CONTACT_READY at 4.15s, TRANSPORT/BRAKE/HOLD never entered by 150s despite true motion; final phase guard estimated progress 3.25569m for target1.30439m, active pushers0. Final speed0 alone does not satisfy original settled condition. Absolute progress error1.79097m retained.
- Diagnostic v1 contained invalid estimate_yaw config and all10 cases failed before simulation. Errors retained in study_v1/diagnostic. v2 uses correct estimate_object_yaw; complete configs prevalidated. This is implementation failure, not algorithm failure.
- v3 adds a separate estimated-progress phase-recovery diagnostic using unchanged brake threshold; default off. Main comparison continues with original downstream.
- Three-method development comparison and single-factor diagnostic experiments running. No independent test cases consumed yet.

## Stages 4–5 — frozen test and evidence report
- Development batch finished 27/27 after the earlier partial checkpoint. No completed case was rerun.
- `no_exploration` matches full CVT trajectory hashes because density explore_gain is already 0. Recorded as a null ablation.
- Frozen protocol `provisional-test-frozen-v1` executed with workers 4. Source hash matched the freeze. 36/36 episodes recorded in about 459 s. 33 COMPLETED. 3 ERROR on l_shape near scale 1.2 seed 910072: initial clearance 0.0702 m inside robot radius 0.13 m, all three methods. Failures kept. Seeds were not replaced.
- Test controller-readiness difference versus nearest target, 10 complete pairs: mean -4.37 s, median -0.77 s, bootstrap 95 percent [-10.02, -0.32]. Versus local spacing the interval includes 0 and CVT is slower on 9/11 pairs.
- Task completion 1/12, 1/12, 2/12. Transport improvement is not supported. Physical-readiness latch remains inconclusive (3 complete pairs).
- Evidence tables and figures: docs/results/evidence_closure/paper_evidence/. Storyline, claim matrix, mechanism report, and professor summary updated. Not pushed.

