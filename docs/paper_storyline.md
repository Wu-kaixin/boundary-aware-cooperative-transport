# Preparation for cooperative transport from local boundary observations

Status: provisional storyline, updated 7 October 2026 after the development batch and frozen test v1. EMPIRICAL_ONLY outside the delimited static oracle theorem. This text is not a professor-approved manuscript scope.

## Problem
A team that does not know an object's geometry must turn local boundary measurements into a configuration for cooperative transport. A short controller phase time is not successful transport. Contact forces can keep pushing after the goal, a phase latch can refuse to brake, and estimated displacement can differ from true displacement. The question is whether a boundary-based configuration demand shortens preparation under matched information and dynamics, and whether safety and task outcome survive that choice.

## Key idea
Local ranges and neighbor communication build a distributed boundary measure. Arc-length weights represent observed extent. Direction-dependent offsets separate pushing and resisting sides. A limited-range CVT turns that measure into configuration targets before CONTACT_READY. Sensing, the transport safety layer, contact physics, and the original downstream phase law stay common in the main comparison. True geometry is used for simulated sensing, contact, and independent evaluation. Oracle-map runs are a separate reference.

## Candidate contribution
The candidate is faster controller preparation, not a proved improvement of unknown-geometry transport.

On the nine development pairs (L, rectangle, C; seeds 2, 5, 8), boundary CVT reaches CONTACT_READY in a conditional mean of 4.383 s versus 8.983 s for nearest observed target. Eight pairs are faster and one is slower. The paired median difference is −0.75 s and the mean difference is −4.6 s. Local spacing, an engineering baseline with the same information permissions, has conditional mean 4.306 s. CVT is not faster than that baseline on this sample. Task completion, defined as the original settling rule plus Euclidean endpoint error within the existing tolerance, is 4/9 for CVT, 6/9 for nearest target, and 6/9 for local spacing.

The frozen test has 12 conditions and 36 episodes. One L-shape near, larger-scale condition is an invalid initial state for every method (signed clearance 0.070 m inside the robot radius). Those three episodes stay in the denominator and were not replaced. Among the 10 pairs in which both CVT and nearest target reached CONTACT_READY, the difference CVT minus nearest has mean −4.37 s, median −0.77 s, and a 95% paired bootstrap interval of [−10.02, −0.32] s (seed 20261007, 10 000 resamples). Eight of those ten pairs favor CVT. Two of twelve pairs are censored, including the invalid initialization. Against local spacing, CVT is slower on 9 of 11 comparable pairs; the mean difference is +0.69 s and the interval [−1.54, +2.64] s includes zero.

Physical readiness, defined as four simultaneous positive-force contacts held for the existing dwell, is observed in 13 of 36 test episodes and in only 3 complete CVT-versus-nearest pairs. That comparison is inconclusive. It was not relaxed to create a pass.

Transport is not supported as an improvement. Frozen-test task completion is 1/12 for CVT, 1/12 for nearest target, and 2/12 for local spacing. Wilson 95% intervals are approximately [0.01, 0.35], [0.01, 0.35], and [0.05, 0.45]. Settling is more common (9/12, 9/12, 8/12) because many settled endpoints remain outside the 0.12 m tolerance. The paired absolute-progress difference has interval [−1.07, +0.63] m. Search layouts and the larger joint scale/distance settings account for most task failures of every method. G500 passes are 4/12, 0/12, and 1/12. G500 is not used as a substitute for task completion.

## Evidence required, and what this round measured
Static local safety was evaluated on the full linear hold. The original local controller has observed path clearance below 65 mm in 8/9 episodes. The lowest path clearance is 48.33 mm on L seed 2. A fresh scan-segment candidate in static theorem-local mode is certified safe on 9/9 by the same evaluator, with path minima 65.85–75.92 mm, while mean robot displacement falls to 0.13–0.28 m from about 0.8–1.0 m. This is an empirical static repair. It does not certify unknown regions, and it was not installed in the transport safety layer. Moving penalty contact remains unresolved as a continuous safety claim.

C-shape seed 8 shows a concrete downstream break. CVT becomes CONTACT_READY at 4.15 s, never records four simultaneous push-side robots, and stays in CONTACT_READY until the 150 s watchdog. True progress reaches 3.095 m against a 1.304 m target while mean forward force after readiness is 5.75 N. The paired nearest-target and spacing runs meet the pusher quorum and finish in HOLD. A diagnostic phase recovery that consults estimated progress can release this latch. That change is downstream and is not credited to CVT. Details are in `docs/failure_mechanism_report.md`.

The density exploration ablation is null: `explore_gain` is already 0, and the three seed-2 trajectories match full CVT. Arc-weight and direction-offset ablations do change those three trajectories, with shape-dependent signs.

Gate 6 v2 and the RAS PDFs were not found. Seeds 12–31 remain quarantined. The frozen test does not cross direction or partial visibility as factors. No threshold, horizon, or failure was removed to improve a rate.

## Verdicts
Preparation versus nearest observed target: SUPPORTED for controller CONTACT_READY on the observed frozen-test pairs, with a small median and two censored pairs. Preparation versus local spacing: NOT_SUPPORTED. Effective physical contact under the quorum latch: INCONCLUSIVE. Static local path safety of the segment candidate: EMPIRICAL improvement on 9 episodes, with less motion and no unknown-map certificate. Transport task completion: NOT_SUPPORTED.

A manuscript can center on preparation for cooperative transport and must report the spacing baseline, the transport failures, and the C-shape phase latch as limits. A claim of improved unknown-geometry transport is not supported. The title remains a decision for Professors Ogura and Bai.
