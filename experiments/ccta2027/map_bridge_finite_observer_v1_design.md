# Current-scan finite support after acquisition: saved-state probe

This follows the completed map_bridge_conflicts_v1 analysis, not a revision of
its hypotheses or output. All17 post-acquisition scaled states use an infinite
plane whose associated true finite edge projection is outside its endpoints.
Original joint feasibility is0/17 even with zero motion and no rho; simultaneous
true point/normal substitutions restore1/17 uncapped full-rho systems. The true
nearest boundary row restores17/17, but is forbidden as a controller input.

Probe all17 states with their actual fresh own/current-one-hop ScanSupport
packets. Require a current nonempty own packet. Use the existing chord/isolated
point guards and keep every competing feature; any unsupported case is a
fallback, never counted as successful constraint removal. Acquisition402 is
excluded from this POST-acquisition proposal and remains an unresolved case.

Keep the original physical radius, rho, gamma, speed bound and all agent/wall
rows. The transport safety radius does not contain the acquisition noise margin:
use h = lower_distance - original r_safe, where lower_distance already subtracts
each chord's structural half-gap and existing three-sigma range-noise allowance.
Do not double-count or omit that allowance. Finite scan interpolation is empirical
for unknown boundary continuation; three sigma is not a deterministic noise bound.

The feature-switch cover ALWAYS uses the original configured boundary speed cap
(0.6 m/s in these states), not a reduced measured speed. Compare three RHS models
on exactly the same resulting geometry, with original reach capping and without:

1. full_bound: isotropic original configured speed cap in every object row.
2. received_speed: isotropic norm of the original received translation after its
   original cap (or maximum capped norm of received per-point velocities). This
   is conditional on the empirical motion estimator, not a certified true-speed
   bound. It is at least every directional projection of the same bounded input.
3. zero_motion: diagnostic-only lower-demand comparison; NEVER a new runtime
   stationary assumption after acquisition.

Verify every bounded joint/group problem with the exact2D and CLARABEL solvers.
Report all denominators, guard failures, raw/capped demands, required speeds and
original/changed input identities. No controller, numeric configuration, physical
episode, adoption or independent deployment-contribution claim in this probe.
Consider runtime integration only after these results and the contact/force
implications are inspected; any physical experiment requires a separate freeze.
