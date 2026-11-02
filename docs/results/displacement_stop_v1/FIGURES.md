# Figure, data and claim mapping

All plots use saved data, Matplotlib and the frozen experiment identities. PDF
and SVG are vector outputs; PNG is a preview. The two closed-loop figures have
7.16-inch design width. Four figure families were visually checked at their
exported resolution. They are research figures, not simulated success artwork.
The manuscript and its title were not edited in this investigation.

| Scientific question | Figure and required fields | Statistical unit | Supported claim | Boundary | Placement |
|---|---|---|---|---|---|
| Why does historical C-away retain about 0.5 m estimated goal error? | [Signed accounting](signed_accounting.pdf): registration/fusion increments, gain, active coordinate-median voters, physical displacement | One selected 600 s development episode | The recorded shared estimate and signed drift can be exactly reconstructed | An accounting identity is not a causal ablation; truth is evaluation-only | Mechanism supplement |
| Does a fixed observed reference reduce drift on exactly the same inputs? | [Estimation comparison](estimation_comparison.pdf): verified reconstructed local scans, original and candidate readouts, physical displacement, physical time | One paired trajectory; 11,400 dependent frames in the 30–600 s window | Within-trajectory MAE decreases from 281.9 to 12.0 mm | No new closed-loop completion or independent reliability evidence | Mechanism panel |
| Does estimation improvement improve complete transport? | [Endpoint comparison](endpoint_comparison.pdf): all 36 episode endpoints, 150/600 s horizons, original completion and hard-limit checks | One reused seed per each of nine paired conditions | Candidate adds one completion, while three conditions remain unfinished | No seed-level confidence interval or general superiority claim; G500 remains separate | Main comparison, with limitations |
| Where do successful and failed paths actually go? | [Complete trajectories](complete_trajectories.pdf): physical center paths and goal circle from saved replay | Same 36 episodes | All paths, including C-away regression, remain visible | Paths show physical outcomes, not information available to the controller or time-aligned movies | Main or supplement |

No smoothing, interpolation, missing-value imputation or outlier exclusion is
applied. All saved trajectory vertices are used. Matplotlib's ordinary path
drawing is not an event detector; exact peaks and numerical checks come from the
full arrays. Nine condition panels include every planned episode, irrespective
of outcome. Endpoint panels use the actual 150 s state or the final state if
execution already terminated. Later completion is never assigned retroactively
to the 150 s panel. At 600 s, unfinished observations are right-censored; their
duration is not a completion time.

The endpoint figure's filled markers denote completed tasks, open markers denote
unfinished tasks, and the dotted line is the original 0.12 m endpoint tolerance.
Being below that line is not sufficient for completion: HOLD and physical
settling are still required. A black cross would denote a hard-limit violation;
none occurred under the four explicitly reported checks. Barrier scaling and
G500 failures are nevertheless preserved in `all_episodes.csv` and the README.

Colors and markers are consistent across the two closed-loop figures: blue
circles for CVT, orange squares for spacing, green triangles for vector-CVT and
purple diamonds for observed-reference CVT. Trajectory styles distinguish the
original controls. In the single-trajectory comparison, black denotes evaluation
truth; green dashed and purple denote the same two vector feedback mechanisms.
No confidence interval is plotted because these reused development conditions
do not provide independent replicate seeds.

## English caption drafts

**Signed accounting.** Signed decomposition of shared displacement error in the
recorded C-open-away vector-CVT episode. Registration, fusion, gain and changes
of coordinate-median voters reproduce the longitudinal error to numerical
precision. The final shared displacement underestimates longitudinal motion by
0.483 m. These terms provide exact bookkeeping along one closed-loop trajectory,
not independent causal effects or a guarantee of physical estimation accuracy.

**Estimation comparison.** Same-input replay of the recorded C-open-away
vector-CVT episode. Black curves are physical displacement used only for
evaluation. Green dashed curves show the original shared accumulated-map readout;
purple curves show the persistent observed-reference readout on verified
reconstructed local observations. The frozen 30–600 s window has mean vector
errors of 281.9 and 12.0 mm, respectively. These are within-trajectory descriptive
errors, not independent experimental samples or closed-loop completion results.

**Endpoint comparison.** Complete fixed-budget comparison over nine paired
shape–orientation conditions and four methods (36 episodes; one reused
development seed per condition). Filled markers indicate task completion under
the unchanged HOLD, settling and endpoint criteria. The dotted line shows the
0.12 m endpoint tolerance, which alone is insufficient for completion. The
observed-reference method completes the perpendicular ellipse by 483.85 s,
increasing the 600 s count from five to six; all methods remain at five at 150 s.
C-open-away worsens despite improved displacement accuracy. No independent
reliability inference is made. G500 results are reported separately.

**Complete trajectories.** Physical object-center trajectories for all 36
episodes, relative to their own starts, with goal displacement (1.2, 0) m and the
unchanged 0.12 m endpoint circle. Lines end at actual execution termination or
the fixed 600 s cutoff; markers denote the last recorded position. Equal aspect
ratios preserve geometry, but axis limits vary by panel. The observations include
failed and prematurely settled tasks. Near-target position alone does not imply
a completed stopping sequence.

## Provenance and generation

- [manifest.json](manifest.json) records the SHA-256 of each of the 23 copied
  derivatives and its original in-repository path. These are intentional small
  publication copies, not additional experimental samples.
- [analysis.json](analysis.json) records protocol, source/input identities and
  analysis/helper hashes; [all_episodes.csv](all_episodes.csv) contains all 36
  outcomes, stopping/safety fields and G500 reasons.
- [historical_reconstruction.json](historical_reconstruction.json),
  [signed_accounting.json](signed_accounting.json) and
  [estimation_figure_provenance.json](estimation_figure_provenance.json) identify
  the diagnostic inputs and processing.
- Generators: `scripts/analyze_displacement_audit.py`,
  `scripts/plot_displacement_replay.py`, and
  `scripts/analyze_displacement_stop_study.py`. Arrays are read with pickle
  disabled. Scripts refuse to replace completed output folders or figures.
- Large reconstructed scans, full time series and all new raw trajectories stay
  inside the repository's local `artifacts/research/` and `runs/research/` trees.
  No new video or new paper build was produced in this diagnostic comparison.
