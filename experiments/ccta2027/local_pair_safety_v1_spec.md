# Next mechanism: acquisition-only local pair safety negotiation

Status: implementation draft after four verified same-state paired witnesses.
No runtime candidate or new physical protocol has been implemented/frozen yet.
Evidence: docs/results/residual_coordination_v1/ and local_pair_feasibility_v1_design.md.

Preserve all existing numeric configuration, physical/success/safety parameters,
default behavior and original failed outcomes. The only candidate configuration
addition should be a default-off boolean. Initial scope is stationary acquisition
with currently admitted finite support for both participating robots. Do not
reuse missing/stale packets or change contact/transport/stop behavior in this fix.

Each robot may offer its current local safety rows, capped nominal command,
position, radius/margin/step identity and timestamp to a one-hop peer. Exchange
constraint certificates, not simulator object truth or global optimization data.
The peer's other-neighbor and object constraints must remain present in the
two-robot solve; a neighbor is not an unconstrained obstacle that can simply yield.

Replace only the mutually confirmed equal-share rows by their original relative
separation condition. Keep all other rows and each original speed ball. Validate
both resulting commands and their actually applied values, including clipping.
Maintain the discrete separation condition; never infer a new object-safety
certificate from empirical finite scan support.

Concurrent proposals require deterministic mutual acceptance, freshness checks
and disjoint execution: no robot can commit different commands to two peers in
one step. Reject mismatched timestamps, positions, contracts or absent replies.
On an unavailable/failed proposal, use the original fallback and retain its
original failure classification; do not call that fallback proven safe.

The architecture must negotiate BEFORE an executed scaled fallback is selected.
Preserve raw unexecuted proposal infeasibility and every executed fallback in
separate diagnostics. Do not erase a scaling event after execution or redefine
G500 to pass: final commands need the same physical separation/object/speed
requirements, and an unresolved original fallback remains a gate failure.
Keep raw individual rows, pair acceptance, relative-row certificate, other rows,
commands, residuals and source IDs so the evaluation can independently verify it.

Required engineering checks before physical freeze: a single-robot infeasible
fixture repaired by a genuinely feasible pair without changing any radius/speed;
peer object constraints that make coordination impossible; conflicting/stale
proposals; unchanged default path; no pairing outside stationary acquisition;
no double assignment; executed separation and retained rows; truthful statistics.
Use meaningful RED/GREEN tests, then the whole suite. Do not implement a global
central QP and label it distributed.

Then freeze exactly four original finite-support controls plus four pair-enabled
candidates, same current development conditions/configurations/seeds/initial
states and600s budgets. Require exact original controls, zero final acquisition
scaling, no increased post-freeze scaling, original safe completion and G500
preservation. Report all failures; full-chain acceptance still also requires
all original G500 passes and formal TRANSPORT. Candidate stays default-off until
broader original-success and unused-condition validation. A pair improvement
does not resolve C's pre-brake torque/activation problem automatically.
