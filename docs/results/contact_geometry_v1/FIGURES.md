# Figure provenance and draft captions

All new-batch figures use the verified 12-case table and saved full trajectories. Three selected conditions, one reused seed; no population intervals. Matplotlib PDF/SVG with embedded TrueType fonts, 7.16-inch width; full samples, no smoothing, downsampling or path simplification.

| Scientific question | Figure | Unit and boundary |
|---|---|---|
| Does either factor change task outcome? | complete_trajectories.pdf | One trajectory per condition-arm; endpoint x means incomplete, not missing; same goal tolerance. No reliability claim. |
| Does better displacement/contact feedback lead to progress? | task_and_contact_timeseries.pdf | Physical time, goal error, selected displacement error, contact count. Frames are not independent trials. The existing contact report samples the first contact substep after the kinematic robot update and before cargo integration; each hold-interval report is plotted at the right endpoint of that interval. It is not a fresh post-integration contact measurement. Estimates use pre-control object state. |
| What can old contact positions support? | historical_contact_geometry.pdf | Three fixed historical reference cases at 600 s; true geometry is evaluator-only; no controller truth access. |

**Caption 1.** Complete object trajectories for the frozen geometry-readout factorial (three development conditions, seed 2). The four arms differ only in the observed radial-reference readout and nominal local-patch distance. Crosses mark incomplete tasks; the dotted circle is the unchanged physical target tolerance. Historical successes are not pooled with these selected failures. In the ellipse and C cases, radial/reference and combined/patch trajectories coincide exactly because the radial model is rejected; overlapping curves represent separate verified executions.

The full-sample time-series figure is a supplementary diagnostic; dense contact traces preserve every recorded excursion and are not treated as independent samples.

**Caption 2.** Physical goal error, controller-selected displacement error, and physical contact count over the complete observation horizon. No temporal smoothing or endpoint extension is applied. Error statistics across closed-loop arms depend on their trajectories; same-input readout comparisons are reported separately. Physical target entry does not by itself satisfy the original HOLD-and-settling termination.

**Caption 3.** Historical reference contact geometry at 600 s. Orange discs geometrically overlap the contact band, blue arrows show inward normals with fixed length, and black arrows point toward the task goal. These are independent geometric annotations, not measured force magnitudes or a contact certificate. The sampled static wrench analysis does not establish overall transport impossibility or feasible repositioning.

Regeneration: root .venv Python, scripts/analyze_contact_geometry.py --out artifacts/research/2026-11-02-contact-geometry-v1/<fresh-output>. Historical capacity: scripts/diagnose_force_capacity.py with its --out option. The exact local rendering helper is retained in artifacts/migrations/2026-11-02-contact-geometry-v1/execution_helpers/render_capacity.py. Raw data, execution source snapshots and helper hashes remain in the mission audit; no fabricated frames are used.
