# Roadmap

## Paper-v1 focus (active)

Branch: `paper-v1-boundary-aware`

Prioritize algorithm + simulation + paper formulation. Freeze Motive / RoboMaster / MAS depth work; keep hardware-agnostic interfaces only.

### Phase 1 — algorithm core

- [x] Ray-cast local sensing + PCA normal estimation (no GT outward normals).
- [x] LocalBoundaryMap voxel dedup, confidence fusion, age decay.
- [x] Boundary-measure-induced density (`Δs`, confidence, decay, gap weight).
- [x] Strict limited Local CVT on `D ∩ B(p_i, R_ℓ)`.
- [x] Distributed responsibility-splitting CBF-QP + object-boundary CBF.

### Phase 2 — physics transport

- [x] PyMunk planar rigid-body world (`dbact_sim/rigid_body_world.py`).
- [x] Switchable transport backend: `scripted` | `pymunk`.
- [x] Optional transport bias from task velocity using local measurements only.
- [x] Optional deps: `pip install -e ".[sim,qp,analysis,dev]"`.

### Phase 3 — experiment matrix

- [x] Baselines B0 ARM / B1 oracle / B2 no-CBF / B3 full DBACT (`controller.method`).
- [x] Extended metrics: `T_enclosure`, `d_min_obs`, `R_CBF`, `T_solve`, `P_success`.
- [x] Paper configs under `configs/sim/paper/`.
- [x] Multi-seed batch runner `scripts/run_paper_matrix.py`.

## Stage 1: Simulation Baseline

- [x] Arbitrary polygon cargo model.
- [x] Local boundary sensing.
- [x] Boundary-aware density field.
- [x] Local CVT approximation.
- [x] Local CBF-style safety filter.
- [x] Simplified caging / pushing dynamics.
- [x] Unknown polygon caging baseline.
- [x] Tight caging configs with improved recruited-agent counts.

## Stage 2: MAS Virtual-Object Integration

- [x] Add root MAS adapter under `src/mas_adapter`.
- [x] Add `dtransport` configs under `configs/mas`.
- [x] Vendor MAS platform under `platforms/mas_public`.
- [x] Register `controller.type: dtransport` in MAS config loading.
- [x] Register `DecentralizedTransportController` in MAS `ControllerModule`.
- [x] Validate `compute(WorldState) -> ControlCommand`.
- [x] Validate root mock MAS pipeline with CSV and trajectory outputs.

## Stage 3: MAS Dry-Run

- [x] Add controller-level MAS dtransport dry-run.
- [x] Add world-bound checks.
- [x] Add trajectory plotting.
- [x] Add automatic dry-run robot-state initialization.
- [x] Add clamp-to-world-bounds mode.
- [x] Add ControllerModule-level dry-run.
- [x] Document Stage 3 dry-run scope and remaining gaps.

## Stage 4–6: Hardware (frozen for paper-v1)

OptiTrack validation, hardware dry-run, and physical experiments remain future physical validation. Do not block the paper-v1 algorithm track on these items.

- [ ] Validate logger with real Motive / NatNet robot rigid bodies.
- [ ] Confirm rigid-body names/IDs, axes, yaw, velocity estimates, and world bounds.
- [ ] Add cargo/object OptiTrack observation path.
- [ ] Hardware dry-run with robot output disabled.
- [ ] Physical caging / transport demos.

## Research and Engineering Backlog

- [x] Formal QP path for distributed CBF (CVXPY) with projection fallback.
- [x] Gap-weighted density term (heuristic uncovered-gap score).
- [x] Physics-based contact transport (PyMunk backend).
- [x] Paper baselines / ablations / scalability suite skeleton.
- [ ] Larger multi-seed paper tables (N=8/32/64, random polygons ×50).
- [ ] Estimate object pose and boundary from local memory.
- [ ] Add nonholonomic robot model.
