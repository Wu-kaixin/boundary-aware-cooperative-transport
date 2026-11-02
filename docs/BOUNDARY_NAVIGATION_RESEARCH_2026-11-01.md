# Boundary navigation: frozen development comparison

Mission `2026-11-01-boundary-navigation-v1`. This investigation is simulation-only.
The MAS interface and manuscript title remain unchanged. It follows the completed
negative handoff investigation; it does not relabel its zero confirmed handoffs.

## Evidence and question

The original circle, parallel ellipse and perpendicular ellipse terminated in
HOLD after approximately 13.50, 26.60 and 13.40 s, respectively. Their actual final
task-axis displacements were 1.1247, 1.1295 and 1.1184 m for a 1.2 m goal, but their
signed lateral errors were -1.1748, -0.8350 and -0.3462 m. Those are off-target
terminations, not tasks still waiting for more computation. Local maps also
reported substantial lateral displacement; no new truth input is needed merely
to notice a lateral error. In contrast, C-open-away continued to move during the
600 s observation. Time helps its displacement but did not complete the task.

The original controller chooses push membership against the initial task axis,
even when it steers the weights toward a lateral correction. The question here
is whether admitting compressive side contacts and, separately, closing the
estimated two-dimensional goal error improve task completion. This is a new
controller experiment, not an engineering repair proven to preserve behavior.

## Six methods and the changed mechanisms

| Arm | Deployment | Transport intervention |
|---|---|---|
| `cvt` | Original boundary CVT | Original scalar-feedback transport |
| `spacing` | Original local spacing before CONTACT_READY | Same original transport |
| `push_cone` | Existing observed trailing-cone target before CONTACT_READY | Same original transport |
| `adaptive_arc` | Original boundary CVT | Membership uses the existing steered direction; scalar feedback/stop unchanged |
| `vector_cvt` | Boundary CVT | Two-dimensional goal feedback, adaptive membership, corresponding offset direction, recoverable local latch before common HOLD |
| `vector_spacing` | Local spacing before CONTACT_READY | Same vector-feedback intervention |

The existing `push_cone` method is an engineering control, not a newly reproduced
literature baseline. The vector method changes several tightly related control
components, so its net effect cannot be attributed to just one of them. The
`adaptive_arc` arm isolates membership alone. The paired vector arms check whether
the result depends on the deployment method. Neither default transport nor the
failed pressure/handoff candidates is replaced.

## Information permissions and mathematical scope

Each vector controller reads its own map and current permitted one-hop messages.
Only fresh full-vector `credible` registration estimates participate; a direction
that is observable under a degraded scan does not make both displacement axes
observable. A componentwise median produces estimated displacement m_hat_i. The
existing scalar calibration 1.24 is applied to the full displacement vector in
these new arms and explicitly changes lateral feedback; it is not refitted.
Velocity is not rescaled. A missing vote is not a zero displacement.

For known relative target delta = L*d, define

    e_i = L*d - m_hat_i
    r_i = max(0, ||e_i|| - epsilon)
    q_i = e_i / ||e_i||                 (outside the control arrival tube)

Here epsilon is the frozen configuration's existing 0.02 m cross-track deadband.
It is a controller design choice, not the independent 0.12 m physical endpoint
tolerance. The existing progress PI receives virtual progress L-r_i. Membership
is n_i dot q_i <= -push_side_threshold, while actual pressure remains along
the locally observed inward normal -n_i. A negative component along the original
task axis can therefore be admitted during correction without tensile forces or
attachment. No physical contact force is fed into this decision.

If x_dot really pointed toward e, the kinematic function V=||e||^2/2 would have
V_dot=-e dot x_dot < 0. That conditional identity does not prove that these contact
commands produce such a velocity. Friction, torque, missing useful contacts,
geometric reachability, estimator bias and switching can all defeat it. There is
no dynamic-contact, convergence or completion guarantee.

The vector arms retain PI authority when braking below the falling reference;
the historical loss-of-quorum reset is not applied in these arms. A local estimate
entering the tube does not irreversibly latch the local loop before common HOLD.
The shared supervisor uses an all-team estimated-vector median and estimated
speed norm for its guards. Common HOLD remains terminal. This is disclosed shared
supervision, not a claim of fully decentralized end-to-end transport. The core
never reads simulator object position, geometry, contact force, success or goal
error. Physics and independent evaluation retain those truth quantities.

## Fixed experiment and decision rules

The [protocol](../experiments/ccta2027/boundary_navigation_v1.json) freezes all
54 episodes: nine geometry/direction conditions, six methods, reused development
seed 2. Circle, both ellipse directions, both rectangle directions, both L
orientations and both C-opening directions are included. Geometry, equal-area
normalization, actual mass/inertia, 16 robots, initial states, sensor noise,
random streams, contact physics, dt=0.05 s, safety and independent evaluation are
inherited from the original census. No new sensor or attachment is added.

Each episode has at most 12,000 frames (600 s physical time), and retains the
original termination test: common HOLD and physical linear speed <=0.005 m/s for
40 consecutive frames. Independent completion additionally requires final 2D
error <=0.12 m. Task completion, G500 and execution status remain separate.
The 150 s result uses the recorded trajectory at that horizon; a task that only
passes through the target is not counted as stopped completion. Early off-target
HOLD is distinguished from right-censoring at 150/600 s. No episode is restarted
from HOLD based on evaluator truth.

Eight case processes use one internal numerical thread each. The complete batch
has a separate 60-minute wall-clock resource limit. Errors and cutoffs stay in the
planned denominator. One candidate version is run, with no post-result tuning,
seed replacement, success-threshold changes or extension until success.

All conditions share one development seed and were already inspected in previous
work; frames, contacts and direction conditions are not independent random trials.
Report gained and lost completions against both controls, every paired endpoint,
physical safety checks, barrier scalings, termination time and message-use cost.
An exploratory promising gate is net >=2 additional completions against both
controls, <=1 lost prior completion, and no new hard safety violation. Even passing
this gate does not promote a new default or establish statistical superiority.
Independent new initial conditions would require a separately frozen study.

## Reproducibility

- Runtime: `src/dbact/control/boundary_navigation.py` plus opt-in wiring in the controller.
- Freeze: `scripts/freeze_boundary_navigation_study.py` (new-only; refuses refreeze).
- Execution: existing `python -m dbact_sim study`; always use the root `.venv`.
- Analysis: `scripts/analyze_boundary_navigation_study.py`, verified receipt inputs only.
- Local raw data: `runs/research/2026-11-01-boundary-navigation-v1/run/`.
- Local audit/backup: `artifacts/migrations/2026-11-01-boundary-navigation-v1/`.
- Engineering logs: `runs/maintenance/2026-11-01-boundary-navigation-v1/`.

Analysis compares all 18 original-CVT/spacing 150 s replay prefixes exactly,
including packed map-snapshot arrays, and every historical scientific step field.
No tolerance is loosened for the controls. Runtime source, inputs and manuscript
hashes are rechecked after execution. This document describes the frozen design;
completed findings are appended only after the batch and integrity checks finish.

## Completed findings and decision

All 54 episodes executed without errors in 1730.67 s wall time. The 60-minute
resource bound was not reached. Completion at both 150 and 600 s is 5/9 for CVT,
5/9 for spacing, 4/9 for push-cone, 4/9 for adaptive-arc, 5/9 for vector-CVT and
4/9 for vector-spacing. There are no rescued completions. Every candidate except
vector-CVT loses the previously successful rotated-L condition. Vector-CVT keeps
the same five completions but fails the predeclared improvement gate.

The four original failures have lower 600 s endpoint error with vector-CVT:
circle 1.1772 to 0.6908 m, parallel ellipse 0.8379 to 0.2296 m, perpendicular
ellipse 0.3557 to 0.1534 m, and C-away 0.5194 to 0.0237 m. These improvements do
not imply comparable time/resource cost: the original circle/ellipse controls
ended early, while several candidates consumed the full 600 s observation.

C-away exposes a useful distinction. Vector-CVT is within the original physical
target tolerance and below the original settling speed for the final 14.8 s,
yet never enters common HOLD. Its last diagnostic median of local one-hop votes
still implies about 0.501 m goal error and differs from the pre-control physical
displacement by about 0.484 m. This aggregate is not the supervisor's exact median.
The physical near-target/low-speed tail is a post-hoc diagnostic added after
seeing this episode; the task remains unsuccessful under the frozen definition.
Its error drops from 0.8399 m at 150 s to 0.0237 m at 600 s, so time does affect
the physical trajectory even though the completed-task count does not change.

There are adverse safety findings. Adaptive-arc exceeds the existing penetration
budget in parallel ellipse (0.097140 versus 0.089338 m). Vector-spacing exceeds
it in perpendicular ellipse (0.113048 versus 0.096875 m) and L-reference (0.098166
versus 0.097442 m). The latter reaches and settles, so the completion count must
not be presented as a safe-completion count. No tolerance was relaxed, including
for the smallest excess. Vector-CVT has no recorded hard-check violation, but
207 barrier-scaling events versus four in original CVT, and only 1/9 G500 passes
versus 5/9. Raw event totals have different exposure durations and are not rates.

All eighteen baseline 150 s replay prefixes and old scientific step fields match
exactly. The original-CVT C-away full 600 s trajectory also matches the previous
study in all 25 arrays and old step fields. Frozen runtime, protocol and historical
input hashes pass; the manuscript title/files and MAS interface were not changed.

Decision: retain all negative results and keep the new modes experimental/default
off. Do not promote any arm or add more variants to this budget. The next useful
work is a bounded same-input offline comparison of a persistent observed reference
map/keyframe against accumulated motion, followed by an independent contact-response
check. That is a hypothesis, not an already implemented solution. A new method
should first show improved estimation on recorded observations without truth input,
without inventing observations and without per-shape gain fitting. The circle's
residual contact limitation and the ellipse's off-target stop must remain separate
failure explanations; the near-target C case does not prove estimation is the
only bottleneck in all shapes. Only then is another fixed closed-loop batch justified.

The [curated result archive](results/boundary_navigation_v1/README.md) contains the
full paired table, safety records and three vector figure families. No additional
scientific episodes were launched after inspecting these results.

The saved `sensed_points` field is the selected agent's **fused map snapshot**,
not all robots' raw scans or matched normals (see `environment._record`). The
current batch therefore supports the logged-error audit but does not, by itself,
provide a complete raw-input replay for a replacement estimator. Before any
same-input keyframe experiment, validate reconstruction from frozen sensor source,
recorded pre-control poses and random identities, and label it reconstructed.
If that cannot be validated, define a separate minimal capture budget. Do not
treat fused points as raw scans or invent missing observations.
