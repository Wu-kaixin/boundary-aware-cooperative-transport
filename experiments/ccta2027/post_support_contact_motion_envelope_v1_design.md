# Signed-motion envelope on the fixed contact-event cohort

Follow the completed post-hoc row decomposition without changing its input,
128-state cohort or17positive-force/11pusher subsets. The zero-motion residual
remains negative on all17original pushing commands. That does not rule out a
negative signed normal motion component, so check the entire configured speed
ball before attributing the conflict solely to geometry.

For the previously selected limiting row n and fixed original command u,

    r(v) = n.u + gamma h - rho - n.v,
    ||v|| <= B,
    r(v) <= n.u + gamma h - rho + B||n||.

Use the original configured object_velocity_bound B (0.2m/s), with no tuning.
If this upper bound is below the existing -1e-7 residual tolerance, that ONE
finite-support row rejects the original command for EVERY signed motion vector
in this configured ball. The vector v=-B n/||n|| attains the row upper bound;
verify it numerically. A nonnegative bound is INCONCLUSIVE about the complete
multi-row system: do not call it a feasible controller or a rescued state.

Report every admitted/nonadmitted state and the fixed subgroup counts. Retain
the row, original command, h, gamma, rho, B and attaining vector for each witness.
Check the witness against the prior128-state raw matrix, rather than relying only
on a scalar exported minimum. No re-solving, changed physical command or new
episode is involved. This is a conditional algebraic obstruction to recovering
the SAME original command, not proof that no other safe pushing command exists.
The configured motion ball is not a certificate of true object motion; nor does
this analysis isolate half-gap, noise and normal error within the geometry term.

Snapshot this design and its script, bind both predecessor analyses by hash,
and save a fresh outcome without altering any previous artifact or running audit.
