# Stage 4: OptiTrack Read-Only Integration

Stage 4 connects real Motive / OptiTrack data to the MAS platform without sending commands to RoboMaster robots.

The rule for this stage is simple:

```text
read OptiTrack data
  -> build WorldState
  -> write CSV / print diagnostics
  -> do not start ControllerModule
  -> do not start RobotModule
  -> do not publish ControlCommand
```

## Current Status

Validated so far:

- NatNet Python SDK files are restored under `platforms/mas_public/third_party/natnet_client`.
- `NatNetClient` can be imported.
- `NatNetAdapter` is selected instead of `MockNatNetAdapter`.
- Motive connection succeeds through `127.0.0.1` Unicast.
- Python receives continuous MoCap frames.
- Read-only logger writes `WorldState` CSV headers.
- Raw rigid body diagnostic option `--print-raw-bodies` is available.

Current blocker:

- Motive has not yet created or streamed robot rigid bodies.
- The logger receives MoCap frames but reports `raw_bodies=0` and `robots=0`.

## Key File

```text
platforms/mas_public/apps/dbact/log_optitrack_world_state.py
```

This script builds the same `WorldState` structure used by MAS, but it never publishes robot commands.

## Mock Validation

Run from the MAS platform root:

```powershell
conda activate dbact
cd E:\DBACT\boundary-aware-cooperative-transport\platforms\mas_public

python apps\dbact\log_optitrack_world_state.py `
  --mock `
  --frames 50 `
  --hz 100 `
  --print-every 10 `
  --output data\optitrack_readonly\mock_world_states.csv
```

Expected result:

```text
adapter=MockNatNetAdapter
robots=3
rows > 0
```

## Real Motive / NatNet Validation

First create and enable rigid bodies in Motive:

```text
Rigid Body 001 -> robot_1
Rigid Body 002 -> robot_2
Rigid Body 003 -> robot_3
```

Then run:

```powershell
conda activate dbact
cd E:\DBACT\boundary-aware-cooperative-transport\platforms\mas_public

python apps\dbact\log_optitrack_world_state.py `
  --frames 300 `
  --hz 100 `
  --print-every 30 `
  --print-raw-bodies `
  --output data\optitrack_readonly\real_world_states.csv
```

Expected result after Motive rigid bodies are correct:

```text
raw_bodies > 0
robots > 0
robot_1 / robot_2 / robot_3 rows appear in CSV
```

## What To Check In The CSV

Main fields:

```text
time
frame_id
robot_id
tracked
x, y, z
roll, pitch, yaw
vx, vy, vz, wz
robot_timestamp
```

Before enabling any robot command, confirm:

- each physical robot maps to the intended `robot_id`;
- moving one robot changes only the matching row;
- x/y/z axes match the MAS control frame expectation;
- yaw sign and units are correct;
- velocity estimates are finite and stable;
- positions are inside `configs/system.yaml` world bounds.

## Next Step After Read-Only Success

Once real `WorldState` logs are correct:

```text
OptiTrack WorldState
  -> ControllerModule dry-run
  -> DecentralizedTransportController.compute()
  -> ControlCommand log only
  -> no RoboMaster output yet
```

Only after that should low-speed RoboMaster S1 experiments begin.
