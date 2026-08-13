# Robust safety closure — stage 2

Date: 2026-08-09  
Branch: `Codex-boundary-aware-closed-loop-v1`  
Baseline: `configs/sim/research/adaptive_progress_closed_loop.yaml`  
Seed: 0 (causal robustness ablation; not a cross-seed probability claim)

## Outcome

The research controller now keeps planning geometry and hard safety geometry separate. Planning uses the persistent gossiped map. The object CBF uses only the robot's latest raw scan and a nearest-feature polyline-distance barrier. The CBF is evaluated every physics frame; perception and planning remain multi-rate.

All five in-domain perturbation variants completed `SEARCH -> MAP -> ENCLOSE -> TRANSPORT -> BRAKE -> HOLD` with:

- hard-QP fallback = 0;
- infeasible = 0;
- rho relaxation = 0;
- zero-input barrier-certificate failure = 0;
- measured boundary point error below `epsilon_b = 0.023 m`;
- measured boundary-velocity/CBF-direction error below `rho = 0.35 m/s`;
- no direct cargo position or velocity actuation.

| Variant | HOLD frame | J (m) | Efficiency | Rotation (deg) | Max penetration (m) | Point error max (m) | CBF velocity error max (m/s) | Delivery | Audited fps |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| nominal | 597 | 0.164 | 0.867 | -7.23 | 0.0123 | 0.0098 | 0.3175 | 1.000 | 20.9 |
| range noise 5 mm | 447 | 0.122 | 0.940 | -2.15 | 0.0097 | 0.0203 | 0.1660 | 1.000 | 30.7 |
| perception/planning every 5 frames | 502 | 0.143 | 0.779 | -5.60 | 0.0130 | 0.0080 | 0.3262 | 1.000 | 37.7 |
| communication dropout 10% | 414 | 0.131 | 0.834 | -3.92 | 0.0130 | 0.0100 | 0.2268 | 0.9002 | 30.7 |
| combined 5 mm / 4-frame / 10% | 484 | 0.139 | 0.916 | -8.39 | 0.0107 | 0.0178 | 0.2603 | 0.8993 | 34.9 |

The audited rate includes online simulator-truth error measurement. The safety QP still executes on every physics frame.

## Root causes closed

1. Historical gossiped tangent planes rotated away from the cargo and created false object constraints. Hard safety now consumes only current local scans.
2. The old margin-free tier subtracted `rho` after capping the complete RHS, which could turn a positive recovery row negative. Full-margin and barrier-only rows are now constructed independently.
3. Multiple tangent-plane outside half-spaces incorrectly intersected the free space near arbitrary polygon corners. The research controller now uses the closest valid polyline feature and a radial distance gradient.
4. Range error is no longer merely declared. `epsilon_b` inflates the point-distance barrier and appears in the C1 contact-margin contract.
5. Estimated translational velocity omitted cargo rotation. The evaluator now measures error against true boundary-point velocity and its CBF radial projection.
6. `rho >= max_speed` made the full ISSf row impossible at `h=0`. Controller construction now rejects that contract; the research configuration reserves 0.45 m/s safety authority for `rho=0.35 m/s`.
7. Local progress estimates diverged, leaving agents in different phases. A finite-hop component consensus fuses median progress and velocity without cargo truth.
8. The TRANSPORT PI integral survived the switch to BRAKE and could keep pushing. The integrator is reset on that mode transition.
9. Online truth auditing mutated observation normals through an aliased NumPy array. Auditing is now observational; audit-on and audit-off trajectories are identical.

## Claim boundary

Conditionally guaranteed in this stage: the implemented point/polyline CBF construction under local boundary observability, `boundary_point_error <= 0.023 m`, CBF boundary-velocity projection error `<= 0.35 m/s`, sufficient control authority, the C1 contact band, and the existing communication/connectivity assumptions.

Empirically validated here: one seed over five actual perturbation variants. This is a causal robustness ablation only.

Not claimed: a probability of success over arbitrary shapes/seeds, a formal caging proof, or an enclosure/transport finite-time theorem. The 10 mm range-noise stress case exceeded `epsilon_b`, timed out, and was rejected; it is not counted as a successful survivor.
