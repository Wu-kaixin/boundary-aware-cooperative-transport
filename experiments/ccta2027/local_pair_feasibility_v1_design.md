# Local pair feasibility after verified residual diagnosis

2026-11-03. This follows residual_coordination_v1 attempt2, which verified the
two residual conflicts each contain an agent_05/agent_09 pair row and an object
row; without speed caps both are feasible. No runtime parameter may be changed.
The existing corrected-geometry trace contains supported rows for BOTH agents at
both selected states. Freeze this follow-up before solving any coupled system.

Scope: exactly C saved365 and skew saved399, each at full original rho and at
rho0 diagnostic:4 paired systems. No controller replay, physical integration or
new physical episode. Reconstruct original hard agent/wall rows from recorded
pre-step positions, original neighbor rule (require dropout0) and saved safety
parameters. Require agent_05 rows to reproduce the previous witness and both
agents' corrected object rows to match the immutable audit entries. Keep all
non-mutual rows, original max_speed .35 per agent, safety radii, margins and
stationary acquisition motion model unchanged.

The mathematical intervention replaces ONLY the two equally split mutual rows
with their summed relative-motion condition:
2*(p_i-p_j) dot (u_i-u_j) >= -gamma_agent*(||p_i-p_j||^2-d_min^2).
All other hard rows and object demands stay intact; neither robot is treated as
an unconstrained retreating obstacle. This tests redistribution of pairwise
avoidance responsibility, not deletion of separation or increase of speed.

Compare each original independent2D system with the4D paired problem minimizing
distance to the same capped nominal commands. Solve the paired system using
CLARABEL and independently SLSQP, then explicitly verify every retained row,
mutual relative row and each speed norm with1e-7 diagnostic tolerance. A feasible
command is only a same-state existence witness. A failed numerical check is
unresolved, not proof of infeasibility or permission to weaken requirements.

Save all matrices, row identities, original/coupled commands, margins, solver
statuses and hashes. Preserve negative outcomes. This offline pair calculation
does not implement a distributed handshake, avoid conflicting simultaneous pair
assignments, establish multi-step invariance under new observations, or prove
closed-loop safety. If feasible, the next implementation must supply those
protocol rules and retain default-off behavior until separate physical gates pass.
