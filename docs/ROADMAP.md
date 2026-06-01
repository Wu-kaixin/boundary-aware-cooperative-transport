# Roadmap

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

## Stage 4: OptiTrack Read-Only Bridge

- [x] Inspect MAS OptiTrack to `WorldState` chain.
- [x] Add read-only OptiTrack / mock NatNet `WorldState` CSV logger.
- [x] Validate logger with `MockNatNetAdapter`.
- [ ] Validate logger with real Motive / NatNet robot rigid bodies.
- [ ] Confirm rigid-body names/IDs, axes, yaw, velocity estimates, and world bounds.
- [ ] Add cargo/object OptiTrack observation path.

## Stage 5: Hardware Dry-Run

- [ ] Run `OptiTrack -> WORLD_STATE -> ControllerModule -> ControlCommand` with robot output disabled.
- [ ] Log and plot real OptiTrack-driven command outputs.
- [ ] Verify stop behavior and shutdown command records.
- [ ] Tune low-speed safety limits for RoboMaster S1.

## Stage 6: Physical Experiments

- [ ] Low-speed caging-only RoboMaster S1 experiment with virtual or known cargo polygon.
- [ ] Add real cargo boundary observation or marker-derived boundary samples.
- [ ] Run circle / rectangle / L-shape / nonconvex benchmark experiments.
- [ ] Compare against baseline CVT + fixed circular AOI.
- [ ] Ablation: no CBF, no boundary density, no communication.
- [ ] Paper-quality real unknown-object caging / transport demo.

## Research and Engineering Backlog

- [ ] Replace the half-plane projection filter with a formal QP solver.
- [ ] Add boundary gap detection and explicit adaptive recruitment.
- [ ] Estimate object pose and boundary from local memory.
- [ ] Add nonholonomic robot model.
- [ ] Add contact force and friction model.
