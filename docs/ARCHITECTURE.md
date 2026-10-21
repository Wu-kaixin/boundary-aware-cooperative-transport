# Architecture

Current structure: 21 October 2026, branch `paper/boundary-aware-safe-deployment`.
See [the migration record](REFACTOR_2026-10-21.md) for preservation and verification.

## Responsibilities

| Layer | Canonical implementation | Responsibility |
| --- | --- | --- |
| Sensing | `dbact.sensing.sensor`, `.boundary_map`, `.features` | Occlusion-respecting ranges, per-agent maps and empirical local segments |
| Deployment | `dbact.deployment.density`, `.cvt`, `.variants` | Boundary measure, limited-range CVT and pre-CONTACT_READY interventions |
| Static reference | `dbact.deployment.theorem` | Delimited static oracle/local runtime and hold diagnostics |
| Safety | `dbact.safety.filter`, `.qp2d` | Hard constraint construction and planar minimum-norm solve |
| Control | `dbact.control.controller`, `.agent_control`, `.phase`, `.transport` | Wiring, phases, single-agent interface and transport commands |
| Shared model | `dbact.types`, `.geometry`, `.cargo`, `.*_dynamics` | Data, geometry, rigid-body and contact models |
| Contracts | `dbact.contracts`, `.provenance` | Declared backend, safety/success rules and reproducibility |
| Simulation | `dbact_sim.environment`, `.scenarios` | Truth geometry for sensing/physics and state evolution |
| Evaluation | `dbact_sim.paper_evaluation` | Independent truth-only metrics; no feedback into control |
| Experiments | `dbact_sim.experiments` | Protocols, paired episodes, snapshots, receipts and statistics |
| Integration | `mas_adapter` | MAS and RoboMaster bridge; separate from paper reproduction |

Old top-level module paths are compatibility aliases to the same Python module objects.
Classes, mutable module state, exceptions and monkeypatches retain identity across paths.
Internal imports use canonical paths. Moving modules changes no control law. The pre-existing
uncommitted gap-guard and push-cone changes were retained.

## Data and information flow

```text
Simulation truth --> ray sensor --> local observations --> local map
       |                                  |
       |                           density --> local CVT --> phase / nominal command
       |                                  |                         |
       |                             local boundary rows --> hard safety QP
       |                                                            |
       +---------------------- physics <--------------------- command
       |
       +---------------------- independent evaluator --> recorded metrics
```

The core never imports `dbact_sim`, experiment scripts or `mas_adapter`.
`tests/test_architecture.py` enforces this dependency direction across the core tree.
The simulator may pass truth into the sensor and contact engine. Local control receives
observations rather than truth polygons. Static oracle mode explicitly has different
information permissions and must be reported separately.

The dependency graph is not a strict pipeline: safety and deployment share geometry and
contracts; static hold diagnostics use safety helpers. The theorem runtime imports controller
diagnostics lazily to avoid an import cycle. Subpackage initializers contain documentation;
`dbact.__init__` preserves the public API.

## Experimental workflow

`python -m dbact_sim` and the installed `dbact` command load the selected workflow:

| Command | Implementation | Outputs |
| --- | --- | --- |
| `simulate` | `dbact_sim.run_sim` | State, summary and optional figures |
| `deployment` | `experiments.deployment` | Matched Gate 5/6 pairs and replay products |
| `study` | `experiments.protocol`, `.study` | Resolved cases, snapshots, receipts and summary |
| `evidence` | `experiments.evidence` | Episode CSV, paired CSV, intervals and figures |

Cases may contain a complete `config`, a repository-relative `config_ref`, or use the existing
shape/gate factory. Historical complete-case protocols remain readable. Only a selected split
executes. Identifiers, gates, variants, shapes, seeds and positive frame limits are checked
before output creation. `--dry-run` resolves cases without executing them.

Studies hash Python sources under `src/`, scenario files under `configs/`, and environment
specifications. Resolved inputs, protocol, runtime versions, source hashes and Git state are
recorded. Each case owns one directory. Windows-safe process workers limit nested BLAS/CVT
threads. JSON receipts use atomic replacement. Missing, partial or modified products cannot
be silently resumed. A frozen source mismatch requires a new study. The v2 runner rejects old
receipt identities instead of rewriting them.

Execution errors remain episodes in the denominator and make the command return nonzero.
A measured task failure is a valid observation. CONTACT_READY, physical contact, settling,
G500 and task completion retain separate meanings.

## Evidence ownership

`experiments/ccta2027/` holds entry points and conference metadata; `manuscript/ccta2027/`
maps the paper outline to evidence; `theory/static_deployment/` holds delimited derivations;
`docs/results/` holds curated archives; `runs/` holds local products. Large historical protocols
stay at their original paths so hashes and citations remain valid. See [CCTA2027.md](CCTA2027.md).
