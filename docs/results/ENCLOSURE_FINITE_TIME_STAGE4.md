# Stage 4: operational enclosure and conditional finite time

Date: 2026-08-09
Branch: `Codex-boundary-aware-closed-loop-v1`

## Operational enclosure result

The simulation no longer treats `coverage >= 0.65` as an enclosure event. Every frame receives an independent truth-audit certificate with six simultaneous checks:

1. exterior-only strict boundary coverage;
2. conservative sampled upper bound on the maximum cyclic uncovered boundary arc;
3. every robot centre outside the object;
4. inter-agent distance at least `d_min`;
5. cage-offset facing-edge clearance at least `d_min`; and
6. a sufficient engaged-agent quorum.

The certificate always serialises:

```json
{
  "certificate_type": "operational_boundary_enclosure",
  "formal_caging": false,
  "formal_caging_nonclaim": "no configuration-space escape proof is implemented"
}
```

The seed-0 0.10 m regression first passed this certificate at frame 240 and maintained it through TRANSPORT/BRAKE/HOLD. At the final frame it measured strict coverage 1.0, maximum uncovered arc 0 m, 10 engaged robots, minimum robot-centre-to-object clearance 0.1791 m and minimum inter-agent distance 0.3521 m. The episode reached HOLD at frame 339 with fallback/infeasible/rho-relaxation all zero.

## Conditional analytic bound

`derive_conditional_finite_time_bound` implements the following sufficient result:

\[
T_{total}\le T_{search}+T_{map}+T_{enclose}+T_{transport}+T_{hold},
\]

with

\[
T_{enclose}\le\frac{\log(E_0/E_{tol})}{\lambda_e},
\]

provided the closed-loop enclosure error satisfies the independently certified Dini inequality `D+E <= -lambda_e E`, and

\[
T_{transport}\le
\frac{\max(0,L-e_{brake})}{v_{progress,min}}+
\frac{\log(e_{brake,0}/e_{hold})}{\lambda_b},
\]

provided `dot J >= v_progress,min > 0` outside BRAKE and `D+|e_J| <= -lambda_b|e_J|` during BRAKE.

Each phase is ceiling-rounded independently to control frames. The output is labelled `provable_sufficient_conditional` and `empirical=false`.

## Current theorem status

The research configuration deliberately declares zero placeholders for:

- `enclosure_contraction_rate_hz`;
- `transport_progress_rate_mps`; and
- `brake_contraction_rate_hz`.

Therefore its analytic finite-time certificate is currently **ineligible**. This is fail-closed: the observed 240/282/325/339 event frames are empirical witnesses, not substituted rates or a theorem.

The CLI `scripts/derive_finite_time_bound.py` accepts independently proved constants and produces a JSON bound. A smoke example with explicitly hypothetical rates returned 623 frames, but that number is not claimed for the current controller because those rates have not been proved across the admissible domain.

## Claim separation

- **Mathematically proved:** complete-workspace paired-lane search under the existing finite-ray feature and lane-spacing premises; algebraic phase bound conditional on the stated contraction/progress inequalities.
- **Conditionally guaranteed:** operational enclosure and transport only for shapes/episodes satisfying geometry, map, wrench, error, positive-rate and corridor premises.
- **Empirically validated:** seed-0 operational enclosure maintained through a 0.10 m closed-loop episode.
- **Unsupported:** global Local-CVT convergence rate for every admissible nonconvex shape, positive progress lower bound over the full domain, BRAKE contraction over the full domain, and formal caging.

No empirical upper confidence bound is reported at this stage; it requires the multi-seed shape matrix in Stage 5 and will remain separate from the analytic bound.
