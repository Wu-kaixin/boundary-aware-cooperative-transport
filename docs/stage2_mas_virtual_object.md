# Stage 2: MAS Virtual Object Integration

## Goal

Stage 2 connects the DBACT controller to a MAS-style control pipeline before running on the real MAS / RoboMaster system.

The current goal is not physical robot deployment yet. The goal is to validate the following pipeline:

```text
mock WorldState
  -> DecentralizedTransportController
  -> ObjectObserver virtual object
  -> DBACTController
  -> planar velocity commands
  -> integrated mock WorldState
  -> states.csv / commands.csv / mock_trajectory.png
```

This stage verifies that the DBACT controller can be wrapped as a MAS-compatible controller and can consume MAS-like robot states to generate velocity commands.

---

## Branch

```text
stage2-mas-virtual-object
```

---

## Completed Steps

### Stage 2.1: MAS adapter check

Confirmed that the repository contains:

```text
src/mas_adapter/decentralized_transport_controller.py
src/mas_adapter/object_observer.py
configs/mas/controller.yaml
configs/mas/dtransport.yaml
```

The adapter can be imported outside MAS-public because MAS-specific imports are protected by fallback definitions.

---

### Stage 2.2: Adapter import/init test

Added:

```text
tests/test_mas_adapter_import.py
```

Verified that `DecentralizedTransportController` can be imported and initialized without MAS-public.

Result:

```text
pytest: 4 passed
```

---

### Stage 2.3: Mock WorldState pipeline test

Added:

```text
tests/test_mas_adapter_mock_pipeline.py
```

Added method:

```python
DecentralizedTransportController.compute_planar_velocities(world_state)
```

This method does not require MAS message classes. It returns:

```python
dict[str, np.ndarray]
```

mapping each robot id to a world-frame planar velocity.

Verified:

```text
mock WorldState -> DBACTController -> planar velocity commands
```

Result:

```text
pytest: 6 passed
```

---

### Stage 2.4: Manual mock pipeline script

Added:

```text
scripts/run_mock_mas_pipeline.py
```

The script prints per-agent velocity commands:

```text
agent_00: vx=..., vy=..., speed=...
agent_01: vx=..., vy=..., speed=...
...
```

---

### Stage 2.5: YAML-based mock config

Added:

```text
configs/mas/dtransport_mock.yaml
```

The script now reads:

```text
configs/mas/controller.yaml
configs/mas/dtransport_mock.yaml
```

instead of hardcoding controller parameters.

`dtransport_mock.yaml` enables:

```yaml
virtual_object:
  enabled: true
```

The original `configs/mas/dtransport.yaml` remains conservative for future real MAS / RoboMaster experiments.

---

### Stage 2.6: Multi-step mock integration

Updated:

```text
scripts/run_mock_mas_pipeline.py
```

The script now supports:

```powershell
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

The script repeatedly computes velocity commands and integrates mock robot states.

Validated:

```text
WorldState(t)
  -> planar velocity commands
  -> WorldState(t + dt)
```

---

### Stage 2.7: CSV output

The mock pipeline now writes:

```text
runs/mock_mas_pipeline/states.csv
runs/mock_mas_pipeline/commands.csv
```

`states.csv` records:

```text
step,time,robot_id,x,y,vx,vy,yaw,tracked
```

`commands.csv` records:

```text
step,time,robot_id,cmd_vx,cmd_vy,cmd_speed
```

---

### Stage 2.8: Trajectory plot output

The mock pipeline now writes:

```text
runs/mock_mas_pipeline/mock_trajectory.png
```

This provides a quick visual check of the mock robot trajectories and the virtual polygon cargo.

---

## Current Validation Command

```powershell
pytest
```

Expected result:

```text
6 passed
```

Run the mock MAS pipeline:

```powershell
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

Expected outputs:

```text
runs/mock_mas_pipeline/states.csv
runs/mock_mas_pipeline/commands.csv
runs/mock_mas_pipeline/mock_trajectory.png
```

---

## Current Mock Pipeline Result

Latest manual run produced:

```text
states.csv
commands.csv
mock_trajectory.png
```

The maximum velocity remains bounded by the mock controller speed limit.

Example:

```text
max_last_step_speed=0.1513
```

---

## Important Design Notes

### 1. Virtual object mode

The current pipeline uses:

```yaml
virtual_object:
  enabled: true
```

This is intentional. Stage 2 validates the software interface before object perception is connected.

### 2. No physical robot yet

Stage 2 has not connected to RoboMaster or OptiTrack yet.

Current scope:

```text
mock WorldState only
virtual polygon cargo only
planar velocity commands only
```

### 3. MAS message classes are not required yet

The current mock pipeline uses:

```python
compute_planar_velocities(world_state)
```

instead of:

```python
compute(world_state)
```

because `compute(world_state)` is reserved for actual MAS-public integration with real `ControlCommand` and `RobotCommand` classes.

### 4. Stage 1 constraints remain active

The controller remains caging-only:

```text
kp_transport = 0.0
```

The controller should not directly use:

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
```

Cargo geometry is used by virtual object / sensing and offline debugging, not as a direct control prior.

---

## Next Steps

### Stage 2.10: Prepare MAS-public integration plan

Next, map the current DBACT adapter to MAS-public files:

```text
src/mas_adapter/decentralized_transport_controller.py
  -> MAS-public/src/controller/decentralized_transport_controller.py

configs/mas/dtransport.yaml
  -> MAS-public/configs/controllers/dtransport.yaml

configs/mas/controller.yaml
  -> MAS-public/configs/controller.yaml or experiment-specific config
```

### Stage 2.11: Register dtransport controller in MAS-public

Modify MAS-public controller loader so that:

```yaml
controller:
  type: dtransport
```

creates:

```python
DecentralizedTransportController
```

### Stage 2.12: Run MAS-public in virtual object mode

Use `virtual_object.enabled=true` first.

Do not connect RoboMaster yet.

The next target pipeline is:

```text
MAS WorldState
  -> DecentralizedTransportController.compute()
  -> MAS ControlCommand
```
