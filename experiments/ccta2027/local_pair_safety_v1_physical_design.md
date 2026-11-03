# Local pair safety: independent eight-episode development comparison

Freeze this design, protocol, source and environment BEFORE any new physical episode.
Implementation design: local_pair_safety_v1_design.md. Preserve the four corrected
finite_acquisition_support_v2 candidate episodes as immutable originals; new
controls rerun their exact configurations under the new source with the new
selector omitted (default False). New candidates differ only by
acquisition_pair_safety=True. No change to any numeric controller/physics/gate,
initial state, task, seed, geometry, dt0.05s or600s/12000-step maximum.
Arm identities are finite_acquisition_support (new control) and local_pair_candidate;
the control retains its original arm label for exact scientific-field comparison.

Exactly4existing conditions x2arms =8new physical episodes: near_circle,
near_l_reference, far_c_open_away_pos0, far_skew_box_s0_r0_pos0. Conditions and
seeds are reused for mechanism development, not independent generalization.
Randomize the8execution slots once with schedule_seed20261103. Four workers,
one numerical thread each. No early dropping, cherry-picking, new seeds, retuning
or automatic restart. Preserve failures, partial outputs and every original receipt.

Before freeze: assert the original source snapshots/product hashes, canonical
YAML hash and each old effective-controller hash. Removing only the new boolean
from each resolved controller must reproduce the old hash. Assert all physical
parameters, gates, initial positions and budgets exactly match. Record the
existing local dependency/lock differences without changing or relabeling runs.
Require passing current runtime core tests and study/audit tests.

Report each case's safe completion, false HOLD, original hard safety violations,
budget termination, original G500 and reasons, formal TRANSPORT, time, endpoint
error, actual rotation, reference availability, per-stage scaling and original
zero-input/relaxation counters. Also report independent infeasible proposals,
failed peer trials/reasons, accepted pairs/robot steps, source locality and
executed constraints. Distinguish control frames, robot steps and episodes.

Independently reconstruct every recorded accepted pair's non-mutual rows,
mutual sum, separate speed balls, source/neighbor positions and timestamps,
both acknowledgement identities and actual commands from physical replay.
Check the entire constant-velocity hold's minimum pair distance. Recorded
physical end positions must realize those commands, including clipping.
Check all accepted records are paired, disjoint, acquisition-only, full-rho,
uncapped finite support. Count actual scaled statuses against counter increments;
an unexecuted independent proposal cannot be called an executed scaling event.
This audits the received observational rows; it does not prove unobserved
object geometry or an asynchronous/hardware communications protocol.

Mechanism gate: exactly8completed executions;4exact original controls;4candidate
safe complete/valid zero epochs, no false HOLD or original hard failure;
zero candidate acquisition scaling; no matched increase in post-acquisition
scaling; preserve every baseline G500 pass; all execution audits pass.
Full-chain gate additionally requires4/4original G500 and4/4formal TRANSPORT.
Do not soften these gates to adopt the candidate. A pair mechanism pass alone
does not excuse C's rotation/contact regression. Broader old-success preservation,
unused conditions, independent deployment contribution, paper and hardware remain.
