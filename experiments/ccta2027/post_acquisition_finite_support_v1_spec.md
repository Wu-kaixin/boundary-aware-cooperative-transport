# Post-acquisition current finite support: empirical runtime candidate

Motivation: exact517frame replay captured all18 map-bridge scaling states.
The17 post-acquisition states project beyond a finite true edge, and replacing
only point/normal errors does not repair the joint system. A truth-free current
scan observer with original hard rows/rho/speed, explicit range-noise allowance,
and isotropic received-speed norm repairs17/17. Across all272robot states at
those frames,181 pass information/geometry admission, and all181 full uncapped
systems are feasible; none of the originally feasible admitted states regresses.
Using the full configured0.2m/s bound repairs16/17 but introduces6 uncapped
peer conflicts. These are selected saved states, not trajectory validation.

Implement a default-false post_acquisition_finite_support option. Require the
existing reference-map bridge, acquisition finite support and qp stack; reject
theorem mode. Keep all numeric configuration, physical/contact/stop contracts,
stage thresholds, original default behavior and all old evidence unchanged.

Admission uses only initial_reference_status=frozen, this agent's current
map_reference_status=fresh_reference, a fresh nonempty own ScanSupport packet,
and fresh packets from the current one-hop roster. No global geometry, true
distance/pose/velocity, remote/stale vote or feasibility-based admission. Preserve
all unsupported/isolated-return guards and original aggregate fallback.

Use the existing finite chord/isolated-point cover. Its reach ALWAYS uses the
original configured object_velocity_bound. In transport the physical r_safe
does not contain acquisition noise: h=distance-half_gap-noise_allowance-r_safe.
Add a default-true noise_in_radius argument to the shared support helper to
preserve acquisition behavior exactly; post-acquisition passes False. No new
noise gain or safety-radius parameter. The geometry remains empirical for
unknown unsampled continuations and unbounded stochastic noise.

Every admitted unit row receives the norm of the current originally bounded
translation, or the maximum capped norm over original per-point velocities.
This is at least every projected component of the same received inputs. It is
NOT a certified actual object-speed bound; original estimation error remains
an empirical assumption. Preserve rho, gamma, speed, reach capping, hard rows,
relaxation accounting and the original navigation/stop/force algorithms.
Record source roster/timestamps, cover and demand speeds, noise convention,
guard/fallback reason, row count and cap count. New trace only when opted in.

Test independent world geometry/noise arithmetic, freshness/locality, nonfinite
motion rejection, received per-point cap, original acquisition behavior,
unchanged hard rows/fallback, invalid combinations/defaultFalse, and controller
wiring. Independently compare the integrated geometry against all181 admitted
frozen observer states before physics. Physics gets a separate four-condition
two-arm freeze, equal seeds/initial states/600s budgets and exact original-control
reproduction. No reuse of saved-state feasibility as a physical success count.

Mechanism gate: all8complete, four map-bridge controls exactly reproduced;
candidate safe completion4/4, no falseHOLD/hard violation/cutoff, matchedG500
preserved, acquisition and post-acquisition scaling never increase, skew
post-acquisition scaling strictly improves, C maximum rotation does not increase
versus concurrent control, and input/constraint audits pass. Full gate additionally
requires zero candidate scaling, G5004/4, formal TRANSPORT in all four cases,
and zero violations of the full uncapped admitted object rows by executed commands.
Report every endpoint/time/rotation regression even if it is not a gate. Adoption
requires the full gate and later broader validation; this candidate stays off.

Publication checkpoint: the user requested uploading the current results and
updating README before further research. The runtime and saved-state audit are
complete, but the independent eight-episode protocol is NOT frozen or executed.
The gate evaluator is implemented; the prospective trajectory auditor still
needs preflight on a newly frozen candidate run. No physical success is claimed.
