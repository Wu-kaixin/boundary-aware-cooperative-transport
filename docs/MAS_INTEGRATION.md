# MAS Integration Guide

This guide explains how to connect DBACT to the uploaded MAS-public platform.

## 1. Copy files

From this repository:

```text
src/mas_adapter/decentralized_transport_controller.py
src/mas_adapter/object_observer.py
configs/mas/dtransport.yaml
```

Copy to MAS-public as:

```text
MAS-public/src/controller/decentralized_transport_controller.py
MAS-public/src/controller/object_observer.py
MAS-public/configs/controllers/dtransport.yaml
```

If you install this package with `pip install -e .`, you can also import `dbact` directly instead of copying the full package.

## 2. Modify `src/common/config_loader.py`

Change:

```python
CONTROLLER_TYPES = {"manual", "point", "cvt"}
```

To:

```python
CONTROLLER_TYPES = {"manual", "point", "cvt", "dtransport"}
```

Also allow bool validation for `dtransport` if MAS validation only expects `point` and `cvt`.

## 3. Modify `src/controller/controller_module.py`

Add import:

```python
from src.controller.decentralized_transport_controller import DecentralizedTransportController
```

In the controller factory, add:

```python
if controller_type == "dtransport":
    return DecentralizedTransportController(
        self.controller_config,
        self.robot_ids,
        self.system_config["world"],
        limits_config,
    )
```

The exact factory method name may differ. Search for where `CVTController` is created.

## 4. Modify MAS config

Set `MAS-public/configs/controller.yaml`:

```yaml
controller:
  type: dtransport
  robot_mode: free
```

## 5. Object observation

MAS WorldState normally contains robot states only. DBACT needs object boundary observations.

For first lab test, use virtual object mode in `dtransport.yaml`:

```yaml
virtual_object:
  enabled: true
```

For real experiment, replace `ObjectObserver.observe()` with one of:

- OptiTrack rigid body for cargo marker points;
- camera/LiDAR segmentation;
- AprilTag/ArUco polygon reconstruction;
- tactile contact boundary estimator.

## 6. Safety

Start with very low speeds:

```yaml
max_speed: 0.08
kp_cage: 0.25
kp_transport: 0.03
```

Validate with mock robot first, then with real RoboMaster S1.
