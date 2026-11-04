# Same-input filtering at actual first pusher suppression

Follow the all-eight onset diagnostic bd86cd750a4734adac81c086ffd247b3133ff64021aa6489a611acbf32be62ac.
Its original selector, fixed before the full scan, selects the first frozen
declared pusher with nominal goal velocity >1e-7 m/s and executed goal velocity
<=1e-7 m/s. The four actual candidate frames are circle110, L96, C515 and skew620.
Do not choose a later, more favorable event. Retain all 16 agents at each event.

The earlier first matched-pair divergence replay covers frames77,71,381,443.
Those earlier events need not be pushers or losses of positive goal velocity.
This follow-up asks the separate, narrower question of what the filter does to
the actual candidate's nominal command at its first preselected pusher event.

Replay each candidate from frame zero through its selected event, without any
physics step. Verify all commands, diagnostics, cumulative solver statistics,
phase guards/response and map snapshots exactly. Use the existing exact replay
function, preserving its source identity, and capture the event's full prepared
constraint system and controller-permitted inputs only.

After exact replay succeeds, prepare an isolated aggregate filter on the SAME
candidate position, nominal command, neighbors, map boundary/normal/velocity
inputs and numerical parameters. Omit only post_support_packets and its mutable
post_support_trace. Keep observation_time and any other input unchanged; require
that stationary finite-support inputs are absent at these frozen-reference
events. Verify every nonobject row and the clipped nominal command is identical.
Solve the aggregate filter once without execution, retaining solver status,
command, all rows and the actual finite system. Bound goal-directed velocity
for BOTH complete systems using the previously verified geometric/CLARABEL
primal/dual or infeasibility certificates. Report inconclusive cases explicitly.

Report all64 agent states, the selected declared-pusher subgroup and admitted
subset, including positive-capability versus must-reverse versus infeasible.
Separate a positive nominal command from the feasible set and from the selected
QP command. An aggregate command on a diverged candidate's inputs is a filter
counterfactual, not a new closed-loop control episode or a full original-arm
controller replay. Goal-directed velocity does not prove contact force, task
success or geometry safety. Do not change a controller or discard a margin.

Snapshot new driver/design plus exact helper sources and input hashes before
replay in a fresh runs/ directory. Preserve failures. Check the original frozen
source/environment/config before and after; leave full audit session56448 alone.
