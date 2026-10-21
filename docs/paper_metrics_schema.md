# Paper metrics schema v1

Legacy JSON remains immutable. New runs add schema_version=paper-metrics-v1.
minimum_true_clearance and object_safety_violation_frames retain their legacy lower-bound meanings. New aliases are clearance_lower_bound_min and lower_bound_check_failed_frames. upgrade_legacy_static never infers actual collision from either.

Static evaluation occurs after command commitment and before application. Shapely/GEOS computes distance from the entire straight hold path to the fixed solid polygon. An intersecting or interior path has solid distance zero. This differs from signed penetration depth and is explicitly named clearance_path_min. Values below r_safe minus 1e-9 are OBSERVED_VIOLATION; above r_safe plus tolerance are CERTIFIED_SAFE under the exact fixed-polygon/linear-hold model with floating-point evaluation. The tolerance band, invalid geometry and incomplete runs are UNRESOLVED. This label is not an interval-arithmetic proof or a local-map certificate. actual_violation_count counts robot-hold intervals, not collision incidents. Aborted cases never receive full-episode certification.

The legacy signed-distance sampler retains 21 endpoint-inclusive samples per hold; sample_h_seconds is dt/20. Its speed-times-spacing bound is separately logged. first_violation_trace copies the implicated robot, command, geometry witness, observed map, active constraint counts and solver residual. Robot pair distances remain a separate metric.

Transport logs preserve phase events with null/right-censoring at the observation horizon. Physical readiness is a diagnostic: the existing contact quorum and dwell applied to contacts with positive projected actual force, sampled at control frames. It does not imply adequate net force. Signed progress, absolute progress, Euclidean endpoint error, cross-track error, settled and G500 are separate. New runs use the already frozen completion definition: settled plus final Euclidean endpoint error within the existing task tolerance. Both direct paired episodes and the study runner now compute it consistently; historical null values are not rewritten. No threshold was altered. The phase clock is the controller's existing frame clock; callback rows describe post-physics state for the frame.

Penalty substeps record pre-integration pose, robot positions, signed clearance, penetration, force and torque. The existing engine holds robots at their post-command positions throughout its cargo substeps; this temporal discretization is disclosed, not silently changed. Logs do not certify continuous moving-body collision freedom. A static 65 mm clearance threshold is not applied to deliberate transport contact. Peak wrench capacity is not sustained force.

Truth access: dbact_sim.paper_evaluation is evaluator-only; no controller import or command veto. Optional physics logging only copies state. Instrumented/uninstrumented fixed-seed trajectory equality is a regression test.

The repaired Pymunk reference exposes the same contact-report interface, with
unique robot identities and control-interval average force/torque. It accumulates
solver impulses across substeps and checks linear/angular momentum conservation
before the separate floor-friction update. Robots integrate from the preceding
endpoint to the newly commanded endpoint. The penalty engine retains its declared
post-command hold and start-of-hold force report. These temporal/contact models
are different; the engine reference is not a pure single-factor material-law
ablation. Penalty spring capacity bounds are marked inapplicable for rigid Pymunk
contacts. Impulses are copied inside the documented
[Pymunk arbiter callback](https://www.pymunk.org/en/latest/pymunk.html#pymunk.Arbiter.total_impulse).
