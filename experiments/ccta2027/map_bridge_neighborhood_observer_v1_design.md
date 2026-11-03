# Whole captured neighborhood screen before runtime integration

The selected-agent probe restored all17 uncapped full-rho systems using current
finite scan support and the capped received-speed norm. With the original full
isotropic speed cap it restored16/17. The actual configured cap was0.2m/s;
map_bridge_finite_observer_v1_motion_cap_erratum.json corrects a0.6 prose typo
without changing that probe or its inputs/computation.

Before wiring this proposal, inspect ALL16robots at ALL17post-acquisition saved
frames:272robot states, including the17known failures and255other states. Use
the unchanged saved-state observer helper. Admission additionally requires the
original map_reference_status to be fresh_reference, ensuring a current local
motion source; unavailable readings do not become an assumed stationary bound.
Retain existing own/current-one-hop freshness and coverage guards.

Compare original and proposed full-rho systems, capped and uncapped, with both
full_bound and received_speed motion models. No selection by robot identity,
truth clearance, desired outcome or constraint feasibility. Record every guard
fallback and every newly infeasible originally feasible state. Verify bounded
feasibility with exact2D and CLARABEL; retain original constraints and candidate
matrices for failures. This is a broader saved-state screen, not independent
trials, a safety guarantee, a trajectory improvement or new physics.

If the observer introduces conflicts on contact or other peers, characterize
them before choosing any runtime intervention. Do not discard those states or
claim selected-agent feasibility as a whole-system repair.
