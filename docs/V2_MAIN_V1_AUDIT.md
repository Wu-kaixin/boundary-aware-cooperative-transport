# main versus Codex closed-loop v1 infrastructure audit

This audit compares `origin/main` with
`origin/Codex-boundary-aware-closed-loop-v1`. The v1 research implementation is
the authority whenever infrastructure and algorithmic behaviour are coupled.

## Decision summary

No source file is copied wholesale from `main`. The useful ideas from main are
already present in stronger form in v1, while the remaining main modules belong
to an older and incompatible controller/dynamics stack.

| Area | main | v1 decision |
|---|---|---|
| CLI | Basic config/output/animation/live flags and a worker override | Keep the v1 seed-aware CLI, provenance headline, headless figure switch, MP4 support, and failure reporting. Rebuild the v2 view/export flags around the offline trace rather than transplanting main's older parser. |
| Sensor abstraction | `raycast_sensor.py` aliases the legacy `LocalBoundarySensor` | Keep v1 `dbact.perception.RayCastBoundarySensor`; replacing it would change controller observations and invalidate research results. |
| Rigid-body abstraction | Optional PyMunk `RigidBodyWorld` with a separate state synchronisation path | Keep v1 `dbact.transport_dynamics.build_engine`, contact status, penalty/scripted engines, and their validated metrics. The main module does not expose v1's measured force/torque/contact contract. |
| Simulation environment | Shorter environment coupled to the old controller and logs | Keep v1's provenance, admissibility certificates, online truth audit, termination semantics, metrics, and safety time series. Only rendering-facing trace extraction will be added. |
| Scenarios/config | Older schema and paper matrix | Keep the migrated v1 schema and validation. No old config is allowed to overwrite v1 research parameters or thresholds. |
| Visualization | Simpler environment-bound Matplotlib functions | Keep v1's richer phase and MP4 behaviour as compatibility requirements, then replace the coupling with a trace-driven visualization package. |
| Reusable utilities | No isolated utility is both newer and independent of old types | Nothing to transplant. Selective non-migration is safer and smaller than adapting obsolete modules. |

## Protected v1 surfaces

The following remain authoritative and are outside the visualization refactor:

- `src/dbact/controller.py` and its closed-loop phase/progress logic;
- `src/dbact/contracts.py`, `guarantees.py`, `safety_filter.py`, and `qp2d.py`;
- `src/dbact/perception.py` and local boundary-map semantics;
- `src/dbact/contact_dynamics.py` and `transport_dynamics.py`;
- success/safety thresholds in existing configs;
- simulation validation, metrics, experiment scripts, and saved truth audits.

The v2 renderer is therefore implemented as a read-only consumer of saved
simulation observations. It must not call the controller or physics engine and
must not make simulator truth available to either one.
