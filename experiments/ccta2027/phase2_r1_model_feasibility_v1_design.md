# Phase 2 R1: local model feasibility audit

The user resumed the active goal after the route-evaluation report. Execute the
specific next step proposed there: model/information-contract review and minimal
candidate design only. Zero new physical episodes, zero controller replay,
at most 7200 seconds offline wall budget, at most 5 GiB new output. No runtime,
physics, safety-definition, parameter, protocol, environment or manuscript change.
Stop after this assessment for the next user decision.

Question: can the current finite observations and neighbor messages justify a
contact-compatible gap/normal/relative-motion model, and where must it abstain?
The route evaluation and earlier geometry results have already been observed.
This is exploratory diagnosis of reused development evidence, not independent
validation or a preregistered claim of improvement.

Read-only cohorts, without reselection:

- all 128 original control-event states from the saved contact-packet-geometry
  capture and contact-compatibility-v2 evidence (112 admitted, including the
  already defined 17 positive-force states);
- all 64 candidate first-pusher-loss-v3 states, four preselected pushers labeled.
  Preserve nonadmitted states and keep the cohorts separate.

Before computing new results, fix these outputs:

1. Reconstruct every admitted row from saved packet points, chords, half-gaps,
   source, noise allowance and timestamp; match A/h and saved cap RHS exactly.
   Attribute geometric distance, structural allowance, noise, rho/gamma and
   motion demand/gamma. No new counterfactual controller or parameter sweep.
2. Under the explicitly hypothetical two-sided distance interval [d-E,d+E],
   E=structural+noise, report the robust clearance/contact band width
   (robot_radius-r_safe)-2E and its rho/gamma-reserved variant. These are algebraic
   diagnostics of that assumed interval, NOT proof E bounds the real boundary,
   actual contact, force or task success. Count rows and state-level 'all rows'
   and 'any row' summaries separately; do not promote either to a control gate.
3. In the 128-state cohort, use saved pre-command truth geometry only as offline
   labels. Compare feature distances/lower bounds with true signed distance and
   nearest-feature normal; distinguish current geometry from subsequent saved
   positive force. Do not interpret a remote feature's normal mismatch alone as
   a modelling error, and do not infer causality from correlated rows.
4. Construct deterministic algebraic counterexamples: two polygons with equal
   noiseless finite-ray returns but a hidden protrusion, and Gaussian 3-sigma
   allowance versus bounded-noise semantics. Use geometric/ray functions only;
   never instantiate or advance a simulation environment. The polygon example
   tests the generic finite-ray/half-gap claim, not the full observation history
   or all one-hop packet unions in the saved cohorts.
5. Audit motion information, object identification, pose/frame/timing assumptions,
   and the exact discrete contact/safety-check execution order in owning source.
   State which conditions a candidate would need and which are unavailable.

Use one internal numerical thread and sequential calculations. Keep original
receipts and every new failure. Hash inputs, sources and output, record timing.
Independent adversarial source review and scalar/analytic output checks must
distinguish model premises from numerical correctness.

Deliver: information contract, mathematical assumptions and falsifiers, observed
diagnostics with full denominators, a minimal conditional candidate specification
and difference table, and a proceed/adjust/stop recommendation with next budget.
No successful full-task claim follows from this stage.
