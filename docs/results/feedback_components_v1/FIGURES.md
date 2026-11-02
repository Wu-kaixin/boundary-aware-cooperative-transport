# Figure–data–claim map

These figures come from saved simulation data. New factorial comparisons use
36 executions over nine paired conditions at reused development seed 2. The
27 original CVT/spacing/vector-CVT references remain separately identified
historical executions in the complete CSV. No figure fabricates a successful
trajectory, fills a missing observation, or treats frames as independent trials.

| Scientific question | Figure / required fields | Unit | Supported claim | Limitation | Use |
|---|---|---|---|---|---|
| When did the previous C-away candidate lose useful contact? | `contact_loss_timing`: original phase, actual contact count, enacted pushers, estimated speed and physical error | Two selected historical episodes | Timing of the observed loss and phase response | First 40 s only; time ordering is not causal attribution | Mechanism supplement |
| Which feedback component changes complete transport? | `endpoint_factorial`: original completion, all endpoints, safety checks, 150/600 s views | Nine paired development conditions, four arms each | Conditional position/velocity effects with common admission | Reused seed; no population superiority or reliability inference | Main comparison |
| Where did all four variants actually move? | `complete_trajectories`: saved physical center paths, starts and goal | Same 36 episodes | All favorable and adverse physical paths remain visible | Center paths alone do not prove contact controllability or stopping | Main or supplement |
| How do C-away feedback and enacted pushers evolve? | `c_away_40s`, `c_away_600s`: physical error, actual shared displacement error, designated pusher count | Four paired C-away closed-loop interventions | Whether error reduction coincides with continued pushing and arrival | Different endogenous trajectories; not a same-input accuracy comparison | Mechanism panels |

PDF and SVG are vector exports; PNG files are previews. Design width is 7.16
inches. Colors/markers are consistent within the new factorial figures: gray
circle = gate-only (00), blue square = position-only (10), orange triangle =
velocity-only (01), purple diamond = both (11). Distinct line styles retain
meaning in grayscale. The historical timing figure uses green dashed for the
previous vector method and purple for the same both-reference candidate.

No smoothing or subsampling is applied; all recorded vertices are supplied to
Matplotlib. The 40 s figures are explicitly cropped views, with full C-away
records shown separately. Missing shared readouts appear as gaps. Curves end
at actual termination/cutoff and are not extended as fictitious ongoing runs.
Numerical peak, safety and completion checks use the complete saved arrays.

Filled endpoint markers mean completed under the original HOLD, physical
settling and endpoint rule; open markers mean not completed. The 0.12 m dotted
line is only the endpoint criterion. A black cross marks an original hard-limit
failure (separation, inside-object clearance, penetration budget or solver
failure). Barrier scaling is retained separately in the CSV/G500 results;
absence of a cross does not imply G500 or continuous-time safety acceptance.
Crosses refer to the complete saved episode in both horizon panels; they do not
assert that a violation already occurred by 150 s. Gate-only has two adverse
episodes: parallel ellipse (inside-object and penetration-budget violations)
and perpendicular ellipse (penetration-budget violation).

At the 150 s horizon, the actual state at that time or at earlier termination is
used. Later completion is not retroactively assigned to the earlier horizon.
An unfinished 600 s episode is observation-censored, not a completed task taking
600 s. No confidence interval or frame-based significance test is shown.

## English captions

**Historical contact-loss timing.** First 40 physical seconds of two frozen
C-open-away episodes. The both-reference candidate enters TRANSPORT at 9.0 s,
suspends pushing at 13.9 s and enters BRAKE at 17.9 s after contact recovery
times out. The previous vector method enters TRANSPORT at 32.8 s and brakes at
285.95 s, outside the displayed interval. Geometric contact readiness, designated
pushers and actual contacts are distinct. Curves are unsmoothed; the comparison
establishes event timing rather than attributing the loss to one feedback channel.

**Endpoint factorial.** Common-admission 2x2 feedback comparison over nine paired
shape–orientation conditions. Gate-only (00) uses the original map position and
velocity on the reference admission rule; position-only (10), velocity-only (01)
and both (11) replace the indicated channels. All four retain the same physical,
safety and task criteria. Filled markers indicate complete tasks; the dotted
0.12 m line alone is insufficient for completion. Results at 150 and 600 s come
from the same executions. These reused development conditions do not support a
population reliability claim. G500 and hard-limit outcomes are reported separately.

**Complete trajectories.** Physical object-center paths for all 36 factorial
executions, relative to their starts. The goal displacement is (1.2, 0) m and the
dotted goal circle has the original 0.12 m tolerance. Final markers indicate the
last recorded position, not necessarily successful termination. Equal aspect
ratios preserve geometry; axis limits vary by condition. No unsuccessful path
is omitted. Contact feasibility and stopping must be evaluated from their
separate records, not inferred from path appearance.

**C-away feedback and pushing.** Physical goal error, actual selected shared
displacement error and enacted pusher count for the four C-open-away
interventions. The 40 s view resolves initial transitions; the 600 s view retains
the entire observed outcome. Shared-feedback absence is shown as a gap. The
controller sees only permitted boundary messages; physical displacement is used
here solely for independent evaluation. Different arms generate different
trajectories, so their error curves are not a same-input estimator comparison.

## Generation and provenance

- `scripts/diagnose_feedback_contact_loss.py` generated the historical diagnosis;
  corrected plot-only exports use its saved `timeseries.json`. The executed
  diagnostic version is retained in the local audit directory.
- `scripts/analyze_feedback_components.py` generates the complete table,
  conditional factorial differences and all new comparison figures.
- `analysis.json` records protocol, raw products, historical analysis and
  analysis/helper hashes. `manifest.json` maps every copied figure/data derivative
  to its in-repository source and SHA-256.
- `all_63_episodes.csv` identifies new versus historical executions;
  `factorial_effects.csv` uses only the 36 new common-admission arms.
- The detailed pause/resume predicate audit stays local; its source-linked
  summary is `recovery_loop_summary.json`. Its shadow eligibility is not actual
  contact confirmation or proof that resuming is safe.

This study produces no new manuscript build or video. The manuscript title
remains unchanged; earlier frozen videos retain their original identities.
