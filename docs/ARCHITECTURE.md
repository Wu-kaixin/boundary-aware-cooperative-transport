# Architecture

Current as of the C1–C3 + S1–S7 rebuild; see
[REFACTOR_2026-08-08.md](REFACTOR_2026-08-08.md) for results and for what was
withdrawn.

```text
src/dbact        core algorithm modules
src/dbact_sim    standalone simulation and visualization
src/mas_adapter  integration layer for MAS-public / RoboMaster S1
```

## Contract layer

Asserted before any controller is constructed, so a configuration that cannot
produce the experiment it describes fails immediately rather than after a run.

- `contracts.py` — C1 contact/safety, C2 solver provenance, C3 success criterion,
  plus the coverage contract `R_l <= R_comm/2`.
- `provenance.py` — git SHA, config hash, and BLAKE2-derived frame seeds, so a
  run is reproducible and attributable. Python's built-in `hash` is deliberately
  not used as a random source: it is salted per process.

## Core modules

| module | stage | role |
|---|---|---|
| `cargo.py` | — | planar **rigid body**: position, angle, twist, mass, inertia. Carries no task direction. |
| `perception.py` | S2 | ray-cast scanner, nearest hit per ray, local PCA normals, residual-based confidence |
| `boundary_map.py` | S3 | per-agent **voxel** map; fusion is idempotent under relay |
| `boundary_density.py` | S4 | boundary measure; `offset` and `distance_field` models; arc allocation for transport |
| `local_cvt.py` | S5 | strict-disk limited-range CVT, truncated cost, cell mass |
| `safety_filter.py` | S1 | one hard QP: inter-robot responsibility rows + nonsmooth object rows |
| `qp2d.py` | S1 | exact planar minimum-norm solver, plus a cvxpy cross-check |
| `contact_dynamics.py` | S6 | penalty contact, Coulomb contact friction, Coulomb **floor** friction |
| `transport_dynamics.py` | S6 | engine registry: `penalty` \| `pymunk` \| `scripted` |
| `controller.py` | S7 | wiring, mode arbitration, transport gating |
| `metrics.py` | — | strict coverage, penetration report, directional progress |

## Data flow

Per robot, per step, using only its own observations and its communication
neighbours:

```text
AgentState + Cargo polygons
  -> RayCastBoundarySensor          (nearest hit per ray, so occlusion is respected)
  -> LocalBoundaryMap               (own scan, then one hop of neighbour relay)
  -> BoundaryAwareDensity           (arc length x confidence x age x gap)
  -> LocalCVT                       (centroid, cell mass, unheld mass)
  -> mode arbitration               (explore | approach | redeploy | cage | push)
  -> SafetyFilter                   (hard QP, no slack)
  -> ControlCommand
```

The object-boundary rows given to the safety filter come from the robot's **own
map**, never from the simulator. That is what makes the normal-estimate error a
quantity with consequences rather than a number in a table.

## Modes

| mode | entered when | target |
|---|---|---|
| `explore` | map empty | deterministic sweep + neighbour repulsion |
| `approach` | map non-empty but cell essentially empty | nearest unheld cage target |
| `redeploy` | cell's boundary already held by neighbours, and not itself in contact | nearest unheld cage target beyond `R_l` |
| `cage` | otherwise | density centroid of the local Voronoi cell |
| `push` | in contact, enough neighbours in contact, observed normal opposes the goal | centroid + inward press along `-n̂` |

`approach` and `redeploy` exist because move-to-centroid on a limited-range cell
has local equilibria that the coverage law cannot escape on its own; both use only
local information. `redeploy` also supplies the exit mechanism the recruitment
argument needs.

## Removed

- `local_sensing.py` — see-through sampler with ground-truth normals; replaced by
  `perception.py`. Its behaviour survives as `LegacyProximitySampler`, used only
  as the B0 perception baseline in audits.
- `local_cbf_qp.py` — soft filter with a slack penalty and a silent projection
  fallback; replaced by `safety_filter.py` + `qp2d.py`.
