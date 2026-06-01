# Stage 3: MAS Dry-Run / Virtual Object Experiment

Stage 3 validates the vendored MAS platform integration without physical robots, real OptiTrack data, RoboMaster SDK runtime, or live network orchestration.

The purpose is to reduce risk before hardware experiments. At this stage the project verifies that MAS-style `WorldState` data can flow through the DBACT `dtransport` controller and produce bounded MAS `ControlCommand` output.

## Completed Scope

Stage 3 covers two dry-run chains.

Controller-level dry-run:

```text
synthetic WorldState
  -> DecentralizedTransportController
  -> ControlCommand
  -> integrated mock robot states
  -> states.csv / commands.csv / events.csv / trajectory.png
```

ControllerModule-level dry-run:

```text
synthetic WorldState
  -> ControllerModule helper methods
  -> _world_state_for_control_frame()
  -> DecentralizedTransportController.compute()
  -> _apply_gimbal_control()
  -> _normalize_command_for_mode()
  -> ControlCommand
  -> states.csv / commands.csv / events.csv
```

## Key Files

```text
platforms/mas_public/apps/dbact/run_dtransport_dry_run.py
platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py
platforms/mas_public/src/controller/decentralized_transport_controller.py
platforms/mas_public/src/controller/controller_module.py
platforms/mas_public/configs/controller.yaml
platforms/mas_public/configs/controllers/dtransport.yaml
```

## Run Commands

From the MAS platform directory:

```powershell
cd platforms\mas_public
```

Controller-level dry-run:

```powershell
python apps\dbact\run_dtransport_dry_run.py `
  --steps 80 `
  --dt 0.05 `
  --print-every 20 `
  --output data\dry_runs\stage3_final_dtransport `
  --clamp-to-world-bounds
```

ControllerModule-level dry-run:

```powershell
python apps\dbact\run_controller_module_dtransport_dry_run.py `
  --steps 80 `
  --dt 0.05 `
  --print-every 20 `
  --output data\dry_runs\stage3_final_controller_module
```

Expected command output includes nonzero `dbact_cage` commands with `robot_mode=free`.

## Outputs

Controller-level dry-run writes:

```text
data/dry_runs/<name>/
|-- states.csv
|-- commands.csv
|-- events.csv
`-- trajectory.png
```

ControllerModule-level dry-run writes:

```text
data/dry_runs/<name>/
|-- states.csv
|-- commands.csv
`-- events.csv
```

## What This Stage Validates

- The MAS platform can select `controller.type: dtransport`.
- `ControllerModule` can construct `DecentralizedTransportController`.
- Synthetic MAS `WorldState` can be converted into DBACT agent states.
- The dtransport controller can produce MAS `ControlCommand`.
- Controller command normalization, gimbal helper logic, and safety limits are exercised.
- Dry-run state, command, event, and trajectory outputs can be inspected before hardware tests.

## What This Stage Does Not Validate

- Real Motive / NatNet streaming.
- Real OptiTrack coordinate frame correctness.
- Real ZeroMQ runtime timing under all modules.
- RoboMaster command execution.
- Physical safety behavior.
- Real cargo/object observation.

Those belong to Stage 4 and later hardware experiments.
