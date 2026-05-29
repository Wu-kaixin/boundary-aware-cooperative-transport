# Stage 1 Results: Unknown Polygon Caging Baseline

## Environment

* OS: Windows
* Conda environment: `dbact`
* Python: 3.10.20
* Project: `boundary-aware-cooperative-transport`
* Controller mode: caging-only
* Transport bias: disabled
* Cargo prior in controller: disabled
* CBF scope: agent-agent only
* Cargo representation: polygon vertices
* Evaluation metrics:

  * `final_coverage`
  * `recruited_agents`
  * `min_inter_agent_distance`
  * `mean_path_length`

## Goal

The goal of Stage 1 is to validate that the proposed controller can form a caging structure around an unknown cargo represented as an arbitrary polygon.

The controller must not directly use:

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
```

The controller only uses:

```text
local BoundaryObservation
neighbor agent states
boundary-aware density
local CVT
agent-agent CBF
```

Cargo geometry is used only by the simulator-side local sensing module to generate local boundary observations and by the offline evaluation module to compute metrics.

---

## Original Baseline Results

| Scenario                          |        Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --------------------------------- | ----------------: | -----: | ----: | -------------: | ---------------: | -----------------------: |
| `baseline_unknown_polygon_caging` | arbitrary polygon |     12 |   900 |         0.7625 |           6 / 12 |                   0.3446 |
| `one_rectangle_polygon_caging`    | rectangle polygon |     12 |   900 |         0.7000 |           6 / 12 |                   0.3446 |
| `one_nonconvex_polygon_caging`    | nonconvex polygon |     14 |  1000 |        0.90625 |           9 / 14 |                   0.3393 |

## Tight Baseline Results

| Scenario                                |        Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --------------------------------------- | ----------------: | -----: | ----: | -------------: | ---------------: | -----------------------: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon |     12 |   900 |        0.95625 |          11 / 12 |                   0.3450 |
| `one_rectangle_polygon_caging_tight`    | rectangle polygon |     12 |   900 |        0.99375 |           9 / 12 |                   0.3450 |
| `one_nonconvex_polygon_caging_tight`    | nonconvex polygon |     14 |  1000 |         0.9750 |          13 / 14 |                   0.3370 |

---

## Parameter Change from Original to Tight Baseline

The tight baseline improves caging compactness by reducing the cage offset and narrowing the density field.

```yaml
controller:
  sensor_range: 1.55
  comm_range: 2.40
  cage_offset: 0.28
  sigma: 0.34
  d_min: 0.30
  max_speed: 0.30
  kp_explore: 0.20
  kp_cage: 1.20
  kp_transport: 0.0
  grid_resolution: 52
  map_ttl: 8.0
  cbf_gamma: 6.0

transport:
  contact_radius: 0.50
  coverage_threshold: 0.60
  min_contact_agents: 4
  speed: 0.0
  boundary_samples: 240
```

Compared with the original baseline, the tight baseline changes:

```text
cage_offset: 0.36 -> 0.28
sigma: 0.42 -> 0.34
kp_cage: 1.10 -> 1.20
kp_explore: 0.25 -> 0.20
```

These changes make the agents converge closer to the cargo boundary while keeping the inter-agent CBF safety constraint active.

---

## Key Results

The tight baseline significantly improves both boundary coverage and actual robot recruitment.

| Scenario          | Coverage Improvement | Recruitment Improvement |
| ----------------- | -------------------: | ----------------------: |
| arbitrary polygon |    0.7625 -> 0.95625 |       6 / 12 -> 11 / 12 |
| rectangle polygon |    0.7000 -> 0.99375 |        6 / 12 -> 9 / 12 |
| nonconvex polygon |    0.90625 -> 0.9750 |       9 / 14 -> 13 / 14 |

The minimum inter-agent distance remains above 0.33 m in all tight scenarios, which indicates that the agent-agent CBF safety filter remains effective.

---

## Main Claim

Stage 1 validates an unknown-cargo caging baseline for arbitrary polygonal objects.

The caging behavior is achieved without direct use of cargo center, radius, polygon vertices, or closest-boundary queries inside the controller.

The successful results show that the system can generate stable caging formations around:

```text
arbitrary polygon cargo
rectangle polygon cargo
nonconvex polygon cargo
```

using only local boundary observations, boundary-aware density, local CVT, and agent-agent CBF safety filtering.

---

## Current Limitation

Although the tight baseline improves caging compactness, the current implementation is still a simulation-level prototype.

Current limitations:

```text
1. Cargo geometry is still used by the simulator-side local sensor.
2. The controller has not yet been connected to MAS / RoboMaster.
3. Cargo physical contact dynamics are simplified.
4. The current stage only demonstrates caging, not real cooperative transportation.
5. No cargo-CBF is used; CBF is applied only between agents.
```

These limitations are acceptable for Stage 1 because the goal is to validate unknown polygon caging before MAS integration.

---

## Next Stage

Stage 2 will focus on MAS integration.

Planned steps:

```text
1. Connect DBACT controller to MAS as dtransport controller.
2. Use virtual object mode first.
3. Verify WorldState -> DBACTController -> ControlCommand pipeline.
4. Test with mock robots.
5. Then move to RoboMaster S1 physical experiments.
```
