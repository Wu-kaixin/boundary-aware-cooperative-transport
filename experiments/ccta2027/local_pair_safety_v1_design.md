# Stationary acquisition local pair negotiation implementation

2026-11-03. Base77de03d150eb83d3ff7131ee65f9b39e87552925. The previous
goal turn published verified residual diagnosis and passed exact-head CI: progress.
Implements local_pair_safety_v1_spec.md; it does not solve C's contact/rotation
problem or establish general reliability, independent deployment benefit or hardware safety.

Preserve the protected YAML, every existing numeric parameter, thresholds,
physical model and original gates. Add only acquisition_pair_safety=False.
Require finite support, qp backend, stationary acquisition and current admitted
packets for both robots; refuse pairing when support reach caps are active,
margin clamping is enabled or the actual dt differs from the certified dt.
The old default path and all raw historical outcomes retain their identities.

Separate stateless constraint preparation from final selection/accounting.
Construct exactly the existing rows and capped nominal, retaining parameter
snapshots. Preparing or checking an unexecuted independent proposal must not
increment execution/fallback counters. Finalize each problem exactly once;
an accepted pair has conditional mutual rows, all other rows unchanged, and
retains the original individual rows/proposal status in separate diagnostics.
Original zero-input checks and G500 definitions remain unchanged. Failed or
unavailable negotiation executes the original solver policy with its original
failure/relaxation/scaling classification.

Each node's offer consists only of its own prepared rows, position, nominal,
current one-hop neighbor IDs/positions, source IDs/timestamps, frame/time,
dt, domain and complete safety contract snapshot. No object truth enters it.
Only mutually visible current one-hop offers with matching contracts are eligible.
An independently infeasible node tries eligible peers in ID order, retaining
all failed trials; the first valid pair becomes its single outgoing proposal.
Use SciPy SLSQP for the4D convex problem with original per-robot speed balls,
full rho and every non-mutual agent/object/wall row. No new dependency or physical
tuning parameter. Numerical acceptance uses the existing planar solver1e-7
tolerance and explicitly checks all retained rows, mutual row and speed bounds.
A solver failure is an unsuccessful proposal, not proof of infeasibility.

The mutual row is 2(pi-pj) dot (ui-uj) >= -gamma_agent (||pi-pj||^2-d_min^2).
Require the original dt*gamma_agent<=1 and nonnegative current separation
barrier. Independently check the minimum pair separation over the whole constant
velocity hold, not only its endpoint. Finite-support object rows remain empirical,
not a newly proven unknown-boundary certificate.

Synchronous one-hop message rounds on the current link graph carry offers,
proposals and acknowledgements. Each endpoint validates both offers and the
same post-clipping command tuple, chooses at most one incident proposal by
lexicographic endpoint/requester key, and acknowledges that digest. Commit only
on matching two-sided acknowledgements. Missing/stale/mismatched replies,
positions/contracts, or overlapping assignments cannot commit. A simulator driver
routes messages; each decision sees only its local incident messages, never a
global optimization or global greedy matching. This assumes intra-step message
rounds; asynchronous network timing is outside this candidate's evidence.

Before acknowledgement, compute exactly the velocities realized by domain
clipping and verify them against all constraints. Reject if clipping invalidates
either robot. Before apply_commands mutates positions, verify committed positions,
dt and commands still match; fail explicitly if an external caller changed them.
Log full offers, independent infeasibility, proposed/applied commands, both acks,
row residuals and observed-source identities for independent execution audit.

Engineering tests cover real conflicting rows, peer impossibility, third-neighbor
and wall preservation, speed limits, stale/asymmetric links, conflicting proposals,
missing acknowledgements, clipped/tampered commands, default behavior and
outside-acquisition rejection. Add tests before runtime changes, run the whole
suite, then freeze a separate physical protocol before any new episode.

Physical follow-up remains the draft's exactly four finite-support controls and
four pair-enabled candidates, same original development conditions/seeds/initial
states and600s budgets. Require exact controls, zero executed acquisition scaling,
no increased post-freeze scaling, baseline G500 preservation and safe completion.
Full-chain acceptance additionally requires all4 original G500 passes and formal
TRANSPORT. Report every failure, proposal and intervention; no adoption based on
these reused development conditions. Preserve C degradation even if pairs improve.
