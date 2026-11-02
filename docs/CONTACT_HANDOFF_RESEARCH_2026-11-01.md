# Contact handoffs: question, information boundary and decision record

This is a development investigation, not independent validation. Historical runs
and their original success definitions remain unchanged. The user authorized a
separate 600 s physical-time observation limit on 2026-11-01. The primary comparison
still uses 150 s. A resource cutoff is not proof that a task can never complete.
An already terminated HOLD outside the target tolerance is a different outcome.

## What the existing evidence says

The nine-condition, seed-2 census completed 5/9 tasks with boundary CVT and 5/9
with local spacing. Equal counts do not establish equivalence or CVT superiority.
The initial joint dynamic-contact candidate completed 2/9, versus 5/9 for the
original controller and 4/9 with the isolated previous-frame feedback repair.
Repairing an implementation path does not establish that its old feedback policy
is beneficial. These are development cases, not nine independent random seeds.

The subsequent read-only audit found command differences before the first
relocation in all nine conditions. Of 124 relocation starts, only six followed a
previous-frame push flag and fifteen followed a recorded physical contact. This
does not support attributing the entire regression to departing pushers. Event
windows are descriptive; the four-arm pressure/relocation ablation is needed to
separate effects. Raw counts are checkable in the mission's offline analysis.

The bottleneck is not merely whether a geometric normal cone spans a direction.
For unilateral contacts, normal magnitudes must satisfy f_i >= 0. A direction-only
fit ||sum_i (-n_i) a_i - d|| does not establish achievable force, friction,
torque balance, contact persistence or collision-free access to that contact.
Normalizing its largest coefficient to one can remove useful pushing demand.
The executed object's nominal ground-friction breakaway is approximately
0.6 * 4.1175 kg * 9.81 m/s^2 = 24.24 N; an aligned but weak contact set can stall.
Actual forces are evaluation-only and are not inputs to the candidate.

## Mechanisms worth separating

| Route | Question | Smallest meaningful evidence | Limitation |
|---|---|---|---|
| Pressure allocation only | Does reweighting existing contacts help without moving robots? | Same initial state, original vs allocation-only | A velocity press weight is not a force allocation |
| Relocation only | Does changing contact geometry help while keeping old press membership? | Same initial state, original vs relocation-only | Arrival at an observed patch is not contact confirmation |
| Confirmed handoff with rollback | Can an incoming contact acquire useful local motion response before an incumbent unloads? | Explicit one-hop handshake, gradual unload, rollback and paired full tasks | Local motion does not identify one robot's causal force contribution |
| Later force-capacity estimation | Can permitted measurements distinguish insufficient force from bad directions? | Separate sensor/identification assumptions and budget | New force sensing or attachment cannot be silently added |

The current task implements and tests one bounded handoff candidate. No method is
called superior unless its frozen task comparison supports that statement. No
baseline is weakened to manufacture an advantage.

## Closest original work and title implications

- [Collective transport via sequential caging](https://arxiv.org/html/2106.03132v1)
  already connects unknown-shape deployment, push-role assignment and transport,
  with simulation and robot experiments. Its distributed allocation and waypoint
  coordination matter when comparing information assumptions. Merely combining
  enclosure and transport is not a new contribution.
- [Cooperative Transportation Without Prior Object Knowledge via Adaptive
  Self-Allocation and Coordination](https://arxiv.org/html/2602.19070v2) already
  combines local detection, density-driven CVT allocation and CBF separation.
  A CVT/CBF combination alone cannot establish novelty. The actual boundary-map,
  contact-dynamics and endpoint/stopping evidence must be compared explicitly.
- [Collaborative Planar Pushing of Polytopic Objects with Multiple Robots in
  Complex Scenes](https://arxiv.org/html/2405.07908v2) studies force-feasible contact
  modes, switching and tracking. Its formulation assumes object geometry and
  physical properties; its execution uses centralized perception. Reconfiguration
  itself is already prior art. A useful distinction would be a demonstrated
  handoff with less information, not the word “handoff” in a title.

The original-paper sections on assumptions, control and experiments were
consulted, rather than inferring novelty from titles. This is a focused comparison
of three close works, not an exhaustive novelty or title-clearance search.

| Working title | Chinese title | Evidence commitment |
|---|---|---|
| Cooperative Pushing of Unknown Objects Using Local Boundary Maps | 基于局部边界地图的未知物体协同推动 | Concrete current system scope; shared phase supervision must be disclosed |
| From Local Boundary Sensing to Cooperative Pushing: Deployment, Transport, and Stopping | 从局部边界感知到协同推动：部署、运输与停止 | Complete-process evidence, including failures; no general guarantee |
| Boundary-Aware Deployment and Contact Coordination for Cooperative Object Transport | 面向物体协同搬运的边界感知部署与接触协调 | Requires actual contact-coordination evidence, not only geometry snapshots |
| Contact-Preserving Handoffs for Cooperative Pushing with Local Boundary Observations | 基于局部边界观测的保持接触交接与协同推动 | Conditional future title: handoff retention and task benefit must first be demonstrated |

For the present system, the first title is more specific than “Boundary-Aware
Deployment for Cooperative Transport of Unknown Objects”. A title does not resolve
overlap in research contributions. “Fully decentralized”, “guaranteed”, “arbitrary
shapes” and “robust completion” are unsupported here. The local map/control paths
and the all-team estimated progress/phase supervisor are distinct information
layers and must both be described.

## Execution record

Mission: `2026-11-01-contact-handoff-v1`. Protocols, source snapshots, all episode
receipts and analyses are linked by that identity. Results and the final decision
are appended only after each frozen batch completes. The original phase/stop and
2D task-completion thresholds are retained. Neither longer observation nor a
successful program exit substitutes for transport completion.

## Completed 150 s findings

The 12-episode ablation finished with zero execution errors. Three conditions
(circle, rectangle-parallel, C-open-away) each used four arms. Feedback repair
alone and relocation alone each completed 1/3; normalized allocation alone and
the v1 joint candidate each completed 0/3. In rectangle-parallel, endpoint errors
were respectively 0.0551, 0.0831, 1.1447 and 1.1449 m. All six reference arms
exactly reproduced their previous replay arrays and old scientific log fields.
The ablation's feedback-repair baseline is not the default baseline of the main
study: this separation is intentional and encoded in the protocols.

The subsequent 27-episode main comparison also finished without execution errors.
Original CVT, spacing and the new handoff candidate each completed 5/9. The same
five conditions completed in every arm; there were no rescued or lost completions.
Against original CVT, candidate endpoint error changed by +1.229 mm for circle,
-58.716 mm for ellipse-parallel, -59.815 mm for rectangle-parallel and -104.561 mm
for C-open-away; the other five endpoint errors were unchanged. These are paired
development observations, not significance or equivalence tests.

The main candidate generated 35 proposals, 10 donor acceptances and seven arrivals
at replacement locations, but zero sustained confirmations, trial unloads or
completed releases. Counts are robot events and are not independent experiments;
donor and incoming abort records are not disjoint episode counts. Consequently,
the endpoint changes cannot be attributed to successful handoffs. The partial
recruitment/engagement actions did change four trajectories. The candidate fails
its predeclared promotion gate and stays disabled by default.

All eighteen original-CVT/spacing main controls match the previous census's 25
replay arrays and every old scientific log field exactly. Recorded hard safety
checks had no new violation. Barrier scaling events are still separately reported;
this is not a general dynamic-contact safety proof. G500 counts were 5/9, 3/9 and
6/9, respectively, and do not replace the common 2D task-completion counts.

## What this version has not solved

The confirmation rule can reject a real, useful side contact. It requires every
incoming contact's force direction to have a nonnegative goal projection. That
is a conservative candidate restriction, not a consequence of unattached contact.
For example, two compressive contact forces proportional to (1,0) and
(-0.2,sqrt(0.96)) sum to (0.8,sqrt(0.96)): the second has a backward component,
yet the pair advances and corrects laterally. This is an algebraic example, not
an experimental measurement. The recorded ellipse-parallel aborts include local
normals with goal-axis components about +0.039 and +0.058, which this rule rejects.
It is inappropriate to infer physical impossibility from that controller choice.

Nor does sustained object motion establish the replacement's individual force.
The observed acceleration depends on the sum of all contact forces and ground
friction. Several allocations can produce the same motion. If an observed gap
has a known error bound epsilon, a sufficient geometric contact test would need
estimated_gap + epsilon <= robot_radius; the current point-estimate test has no
such validated error bound. A “credible” registration label is not that bound.

The next useful question is therefore narrower than another broad parameter sweep:
**can a locally observed side contact demonstrate a useful incremental response
while the original pushers remain active, even if its individual force has a
small backward projection?** A future experiment should isolate a short,
predeclared engagement/probe window, preserve incumbent demand, count its time
and motion cost, and test the response before any unload. This would require a
new candidate/version/budget; it is not silently applied to the present batch.

Separately, circle and both ellipse orientations can terminate in HOLD with large
2D errors. A scalar projected-progress stop cannot by itself establish lateral
arrival. A future estimated 2D recovery/stop policy would be a different mechanism
from contact handoff and must be ablated separately, without using object truth
or changing the independent success criterion. More observation time alone does
not reopen an already terminated HOLD.

## Completed 600 s observation and final decision

All eighteen extended episodes finished execution without errors. Original and
candidate each still complete 5/9; C-open-away is right-censored in both arms at
600 s. Its original/candidate endpoint errors are 0.519419/0.656044 m, compared
with 0.846019/0.741458 m at 150 s. Both move closer over the added observation,
but the candidate's 104.561 mm advantage at 150 s becomes a 136.625 mm disadvantage
at 600 s. Neither finite observation establishes eventual impossibility, and the
150 s advantage must not be generalized across horizons. Early settled off-target
circle/ellipse episodes retain their original termination rules and do not reopen.

The eighteen extended replay prefixes agree exactly with the primary runs.
An initial analysis check incorrectly treated packed `sensed_points` as a regular
frame array. The reader schema concatenates each frame's points and indexes them
by `sensed_counts`; the corrected comparison checks every recorded prefix point
without numerical tolerance. The initial failed check, source version and three
regression tests are retained in the audit area. No scientific trajectory or
expected hash was edited to pass this check.

Decision: the fixed 57-episode investigation is complete, but the objective of
successful transport in all conditions is not achieved. Do not promote this
candidate or claim confirmed-handoff capability. Keep the original default and
the negative results. Before any new broad matrix, a separately frozen short
engagement experiment should establish that a real side contact can survive the
local admission rule and produce measurable incremental corrective response.
Projected-progress stopping and 2D recovery remain a separate research mechanism.

The seven position-arrival events all follow an offer directly: those volunteers
were already within the target neighbourhood. They are not evidence of completed
lateral travel. Six patch/estimate aborts have recorded directions that fail the
candidate's geometric gate; three have a physical-contact report in the preceding
physics step. Those reports precede cargo integration and are not exactly
simultaneous with the next controller decision. They motivate a better admission
test, not a claim that the rejected contact was already sufficient to take over.
