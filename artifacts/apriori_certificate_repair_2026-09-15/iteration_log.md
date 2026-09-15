# Iteration log

## Worktree

Created `D:/boundary-aware-cooperative-transport-certificate-repair` from `7b9fb36` as `feat/apriori-certificate-repair`. Original apriori worktree not reset or merged.

## Certificate repair

- Reproduced clipped-edge remainder bug (L seed 2, 16/16 cells, 1.40 m vs 0.0196 m chord).
- Analytic M2; h_max=0.004 panels; robot-centred (ξ−p)δm; discrete φ_max; centroid projection; unclamped K0 dumps; labels.
- Tests: 55 passed, 2 skipped.

## Priors (frozen before closed loop)

Repaired B_J,prior still beats M_plane u_max²: rectangle 0.1435 < 0.1803, L 0.1883 < 0.2295, C 0.1978 < 0.2295.

## Experiments

- 8-frame serial 27.8 s vs parallel 17.4 s, metrics identical.
- Stage C: L2 K0 yes; C5 K0 no (228–232, agent 14; 2 hard + 3 margin).
- Stage D seeds 2/5/8: 8/9 K0; only C5 fails. No aborts.
- Stage E seeds 11/17/23: L11 and L17 fail K0 (margin-only); all rectangles and C-shapes and L23 pass.

## Route B

‖Π_F(0)‖ ≤ u_max is a priori but geometric-order; it cancels the prior's advantage. Tighter bound needs an unproved h-invariant. Stopped as PARTIAL with that blocking condition.
