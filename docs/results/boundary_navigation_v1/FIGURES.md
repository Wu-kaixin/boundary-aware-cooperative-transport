# Figure plan frozen before complete batch results

| Question | Figure | Unit and fields | Scope |
|---|---|---|---|
| Does any method complete more paired tasks without losing old successes? | Endpoint scatter at 150/600 s | Nine geometry/direction conditions, one reused development seed; all 54 episodes | Descriptive comparison, no independent reliability estimate |
| Does improvement arise from different travel, correction or termination? | Full center trajectories, all methods and conditions | Saved centers and times; actual final point; 150 s marker; target tolerance | Does not establish formal caging or contact stability |
| Does estimated point-goal arrival agree with physical position? | Estimated/true 2D goal-error traces for the two vector arms | Median of logged local votes and corresponding pre-physics center; all recorded samples | Median of local votes is not identical to the supervisor median; frames are not independent replicates |

## English captions

**Endpoint comparison.** Task endpoint errors for six controllers under nine geometry-direction conditions with the same development seed and paired initial states. The panels use observations at 150 and 600 s, or the original terminal state for episodes that ended earlier. Filled markers indicate an already terminated, settled task within the unchanged 0.12 m Euclidean tolerance; open markers indicate failure or censoring. A transient visit to the tolerance region is not completion. No confidence interval is shown because each condition contains one development initial state.

Completion markers encode the endpoint/settling definition, not passage of every
safety check. In particular, the filled vector-spacing L-reference point has a
recorded penetration-budget violation. All three such violations and barrier
scaling counts are retained in the accompanying result table.

**Complete trajectories.** Object-center displacement trajectories from initialization to termination or the 600 s observation cutoff. All methods and all conditions are included. Markers indicate actual final states, crosses mark the 150 s observation for still-running episodes, and dashed circles denote the unchanged physical goal tolerance. The traces are not smoothed, extrapolated or constructed from summary scores.

**Estimated and physical goal errors.** Time-aligned point-goal error for vector-feedback CVT and spacing. Solid curves use evaluation-only pre-control object centers; dashed curves use the coordinate median of logged local one-hop displacement votes from that control decision. This descriptive aggregate is not the common supervisor's exact median. Feedback and evaluation remain separate; agreement in this plot is not an observability, accuracy or contact certificate.
