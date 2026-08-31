# Roadmap

> **Superseded for the research track.** The stage list below records the
> engineering history of the MAS/hardware bridge and is still current for that.
> The research pipeline was rebuilt on branch `A-boundary-aware` around a
> contract layer (C1–C3) plus stages S1–S7; see
> [REFACTOR_2026-08-08.md](REFACTOR_2026-08-08.md) for the current state,
> including which earlier results are withdrawn and why.
>
> In particular: every coverage number produced before the object-boundary CBF
> existed is void, because robots standing *inside* the cargo counted as covering
> its boundary. Directed transport is currently **not** achieved — the closed loop
> produces positive, direction-efficient progress that stalls at a static caging
> equilibrium below the task threshold. The three claimed contributions do not
> depend on transport and are all supported.

## Stage 1: Simulation Baseline

- [x] Arbitrary polygon cargo model.
- [x] Local boundary sensing. *(replaced: ray casting with occlusion, `perception.py`)*
- [x] Boundary-aware density field. *(extended: two models, `density_mode`)*
- [x] Local CVT approximation. *(replaced: strict disk + truncated cost)*
- [x] Local CBF-style safety filter. *(replaced: hard QP, no slack, object rows)*
- [x] Simplified caging / pushing dynamics. *(replaced: rigid body + contact only)*
- [x] Unknown polygon caging baseline.
- [~] Tight caging configs with improved recruited-agent counts. *(the "improved"
      counts were measured under the void metric; re-run required)*

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
