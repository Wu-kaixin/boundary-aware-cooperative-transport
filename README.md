# DBACT: Decentralized Boundary-Aware Cooperative Transportation

> 面向未知任意形状物体的去中心化边界感知协同搬运算法  
> Decentralized Boundary-Aware Cooperative Transportation of Arbitrarily Shaped Objects Without Prior Object Knowledge

DBACT 是一个面向多机器人协同搬运任务的研究型算法框架。项目目标是在不知道物体中心、半径、形状、质量和所需机器人数量的情况下，让多个移动机器人仅依靠局部感知、局部通信和局部控制，自适应地发现物体边界、围合任意形状物体，并通过 caging / pushing 的方式完成协同搬运。

本仓库建议作为以下两个已有来源的扩展与整合版本：

1. `Cooperative-Transport-Multi-Agent-System`：作为仿真算法 baseline，提供动态密度函数、CVT、CBF-QP、轨迹记录和可视化基础。
2. `MAS-public` / `Robot.zip` 中的 MAS 平台：作为 RoboMaster S1 + OptiTrack/Motive 的闭环实验平台，提供 `WorldState -> Controller -> ControlCommand -> Robot` 的实机运行链路。

当前 README 面向研究开发版本，重点说明算法设计、代码结构、仿真复现、MAS 接入方式和未来实验路线。

---

## 1. Motivation

多机器人协同搬运通常依赖以下先验条件：

- 物体位置已知；
- 物体形状或尺寸已知；
- 机器人数量或分组关系已知；
- 搬运接触点或围合结构预先设计；
- 控制器可以访问全局状态和全局任务分配。

这些假设会限制算法在真实场景中的应用。真实场景中，机器人可能只在局部传感范围内看到物体的一部分边界，物体可能是圆形、矩形、L 型、非凸多边形或其他不规则形状，并且物体出现的位置、尺度和所需机器人数量都不应预先给定。

DBACT 的核心思想是：

> 不再把机器人吸引到“已知物体中心”或“已知圆形 AOI 区域”，而是把机器人吸引到“局部检测到的物体边界外侧 cage candidate points”，然后通过局部 CVT / 局部 CBF-QP 形成稳定围合结构，并进一步推动物体完成搬运。

---

## 2. Core Contributions

本项目计划实现以下核心能力：

### 2.1 Boundary-Aware Density Field

原始 baseline 通过检测圆形 AOI 与机器人传感范围的交集生成动态密度点。DBACT 将其改为边界驱动密度场：

1. 每个机器人只检测自身传感范围内的物体边界点；
2. 对每个边界点估计外法向；
3. 在物体外侧生成 cage target point；
4. 对 cage target point 叠加 Gaussian kernel，形成局部边界密度场。

形式上，可以写成：

```text
q_target = b + d_cage * n_out
rho(q) = rho0 + Σ exp(-||q - q_target||² / (2σ²))
```

其中：

- `b` 是局部检测到的边界点；
- `n_out` 是边界外法向；
- `d_cage` 是机器人相对物体边界的期望围合距离；
- `sigma` 控制密度影响范围；
- `rho(q)` 是供局部 CVT 使用的目标密度。

### 2.2 Adaptive Recruitment

机器人数量不再通过固定编号或预设队伍决定，而由以下因素自然决定：

- 物体边界长度；
- 未覆盖边界 gap；
- 局部密度强度；
- 当前邻域机器人数量；
- 通信范围内的 object token 传播。

边界越长、空缺越大、检测机器人越多，密度场就会自然吸引更多机器人加入。

### 2.3 Local CVT / Limited Voronoi

每个机器人只基于以下局部信息计算目标位置：

```text
self_state
neighbor_states within comm_range
local_boundary_points within sensor_range
received_object_tokens within TTL
```

控制器不直接读取全局 polygon、全局物体中心或全局机器人状态。这样可以让算法逻辑更接近真正去中心化。

### 2.4 Local CBF-QP Safety Control

每个机器人只对局部邻居构造 CBF 约束：

```text
h_ij = ||p_i - p_j||² - d_min² >= 0
```

并通过小规模 QP 求解安全控制输入：

```text
minimize    ||u_i - u_nom_i||² + λ * slack²
subject to  local inter-robot CBF constraints
            object boundary distance/contact constraints
            velocity limits
```

这比全局 pairwise CBF 更适合扩展到更多机器人。

### 2.5 Arbitrary-Shape Caging + Pushing Transport

目标物体不再限定为圆形。仿真阶段支持：

- circle；
- rectangle；
- L-shape；
- non-convex polygon；
- random polygon；
- multiple objects。

在物体被足够数量机器人围合后，控制器生成共同运输速度 `v_transport`，机器人在保持边界围合和安全距离的同时推动物体。

---

## 3. Relationship to Existing Sources

### 3.1 Baseline: Cooperative-Transport-Multi-Agent-System

该项目已经实现：

- 2D 工作空间；
- 多智能体位置更新；
- 动态密度函数；
- Voronoi / CVT；
- CBF-QP 安全控制；
- 轨迹 CSV 保存；
- density surface 和 Voronoi 可视化。

但它仍然存在几个需要重构的地方：

- `main.py` 中固定了两个圆形 AOI 的中心、半径和移动方向；
- `density_func.py` 中存在固定机器人编号分配，例如 `aoi1_agent_indices`；
- `controller.py` 使用全局 pairwise CBF-QP；
- 物体运动是外部强制移动，不是真正由机器人接触/推动产生；
- 控制器可以间接使用全局 AOI 信息，还不是严格的局部边界感知。

因此，该代码适合作为 baseline 和可视化基础，但 DBACT 需要重写 sensing、density、allocation、local CVT、local CBF 和 transport dynamics。

### 3.2 Real-Robot Platform: MAS-public

MAS 平台适合作为实机闭环验证平台。它的核心运行链路是：

```text
Motive GUI
  -> NatNet
  -> OptiTrack Module
  -> WorldState
  -> Controller Module
  -> ControlCommand
  -> Robot Module
  -> RoboMaster S1
```

MAS 已经提供：

- `manual`、`point`、`cvt` 控制器；
- ZeroMQ 消息通信；
- RoboMaster S1 速度控制；
- OptiTrack 状态获取；
- tracking validation；
- command limiter；
- watchdog stop；
- world-bound stop；
- experiment logging；
- CSV recording；
- offline plotting；
- mock robot / mock OptiTrack 测试脚本。

DBACT 不应该直接破坏已有 `cvt_controller.py`，而应该新增一个独立控制器，例如：

```text
src/controller/decentralized_transport_controller.py
src/controller/decentralized_transport_utils.py
configs/controllers/dtransport.yaml
```

---

## 4. Proposed Repository Structure

推荐把仓库整理成如下结构：

```text
boundary-aware-cooperative-transport/
├── README.md
├── requirements.txt
├── pyproject.toml                         # optional
├── configs/
│   ├── sim/
│   │   ├── circle.yaml
│   │   ├── rectangle.yaml
│   │   ├── l_shape.yaml
│   │   └── nonconvex.yaml
│   └── mas/
│       └── dtransport.yaml
├── src/
│   ├── dbact/
│   │   ├── __init__.py
│   │   ├── cargo.py                       # arbitrary-shape cargo model
│   │   ├── local_sensing.py               # ray casting / local boundary observation
│   │   ├── boundary_map.py                # local boundary memory and object tokens
│   │   ├── boundary_density.py            # boundary-aware Gaussian density field
│   │   ├── local_cvt.py                   # local / limited / weighted CVT
│   │   ├── local_cbf_qp.py                # per-agent CBF-QP
│   │   ├── transport_dynamics.py          # simplified caging/pushing object dynamics
│   │   ├── controller.py                  # DBACT high-level controller
│   │   └── metrics.py                     # success rate, coverage, safety, energy
│   ├── sim/
│   │   ├── run_sim.py                     # simulation entry
│   │   ├── environment.py
│   │   ├── visualization.py
│   │   └── scenarios.py
│   └── mas_adapter/
│       ├── decentralized_transport_controller.py
│       └── object_observer.py
├── third_party/
│   ├── cooperative_transport_baseline/     # optional: original baseline reference
│   └── mas_public/                         # optional: MAS submodule/reference
├── scripts/
│   ├── reproduce_baseline.py
│   ├── run_shape_benchmark.py
│   ├── plot_metrics.py
│   └── export_video.py
├── data/
│   ├── experiments/
│   └── baseline/
├── figures/
├── tests/
│   ├── test_boundary_sensing.py
│   ├── test_local_cvt.py
│   ├── test_local_cbf_qp.py
│   └── test_transport_controller.py
└── docs/
    ├── algorithm.md
    ├── mas_integration.md
    └── hardware_setup.md
```

---

## 5. Algorithm Overview

### 5.1 State and Assumptions

每个机器人 `i` 的状态：

```text
p_i = [x_i, y_i]^T
θ_i = yaw_i
v_i = [vx_i, vy_i]^T
```

机器人只允许访问：

```text
local observation O_i(t)
neighbor states N_i(t) = {j | ||p_i - p_j|| <= comm_range}
local boundary points B_i(t)
local object tokens T_i(t)
```

机器人不允许直接访问：

```text
complete object polygon
object center
object radius
global robot assignment
global QP over all robots
predefined team size
```

仿真环境可以保存完整 polygon 用于碰撞检测和评价指标，但控制器不能直接读取完整 polygon。

### 5.2 Local Boundary Sensing

仿真阶段可以用 Shapely 实现 ray casting：

```python
def detect_boundary_points(robot_pos, cargo_polygons, sensor_range, n_rays=64):
    """Return boundary points visible within local sensing range."""
    points = []
    for ray in rays_from(robot_pos, n_rays, sensor_range):
        for cargo in cargo_polygons:
            hit = ray.intersection(cargo.polygon.boundary)
            if not hit.is_empty:
                points.append(nearest_hit(robot_pos, hit))
    return points
```

实机阶段可以先用 OptiTrack 中的物体 marker 模拟局部边界，后续再替换为深度相机、LiDAR、触碰传感或视觉分割。

### 5.3 Boundary Target Generation

对每个检测到的边界点 `b_k`：

```text
n_k = estimated outward normal
q_k = b_k + d_cage * n_k
```

`q_k` 是机器人期望围合位置，不是物体中心。

### 5.4 Boundary-Aware Density

局部密度函数：

```text
rho_i(q, t) = rho0 + Σ_k w_k exp(-||q - q_k||² / (2σ²))
```

其中权重可以由以下因素决定：

```text
w_k = boundary_weight + gap_weight * uncovered_gap_score + confidence_weight
```

未覆盖边界附近权重更高，从而吸引更多机器人补齐空缺。

### 5.5 Local CVT Target

每个机器人只在局部区域内计算目标点：

```python
local_agents = [self] + neighbors_within_comm_range
local_samples = sample_points_from_local_density(rho_i)
local_voronoi = compute_limited_voronoi(local_agents, local_samples)
c_i = weighted_centroid(local_voronoi[self_id], rho_i)
```

名义控制输入：

```text
u_cage_i = k_cage * (c_i - p_i)
```

如果机器人没有检测到物体，则执行探索控制：

```text
u_explore_i = k_explore * (coverage_centroid_i - p_i)
```

### 5.6 Transport Direction

运输方向可以有三种模式：

```text
fixed_direction:       给定方向，例如 [0, 1]
goal_position:         给定目标点，例如 [x_goal, y_goal]
operator_guided:       外部遥控/人工指定方向
```

围合形成后加入运输分量：

```text
u_transport_i = k_transport * v_transport
```

总名义输入：

```text
u_nom_i = u_explore_i + u_cage_i + u_transport_i + u_spacing_i
```

### 5.7 Local CBF-QP

每个机器人独立求解：

```text
minimize    ||u_i - u_nom_i||² + λs²
subject to  h_ij(p_i, p_j) >= 0, j in local_neighbors
            h_obj(p_i) >= 0 or contact-distance constraint
            ||u_i|| <= u_max
            s >= 0
```

机器人间安全约束：

```text
h_ij = ||p_i - p_j||² - d_min²
```

物体边界约束可分两类：

```text
non-contact phase: keep outside object body
contact phase: keep near desired cage/contact distance
```

### 5.8 Simplified Object Transport Dynamics

第一版不需要复杂摩擦模型，可以使用简化模型：

```python
if boundary_coverage_ratio > threshold and enough_contact_robots:
    cargo.velocity = mean(projected_push_velocities)
    cargo.pose += cargo.velocity * dt
else:
    cargo.velocity = 0
```

后续再升级到 quasi-static pushing、接触力估计和摩擦锥约束。

---

## 6. How to Build from Existing Code

### 6.1 Refactor from Cooperative-Transport-Multi-Agent-System

建议先复制 baseline 的核心思想，而不是直接在原文件中硬改。

需要保留或参考：

```text
src/voronoi.py              # Voronoi / CVT 思路
src/density_func.py         # density-driven target generation 思路
src/controller.py           # CBF-QP 思路
src/plotting_3Ddens.py      # 可视化思路
src/main.py                 # 仿真循环和 CSV 保存思路
```

需要替换：

```text
固定圆形 AOI                  -> Cargo polygon / arbitrary shape
固定 aoi center / radius       -> local boundary observation
固定 AOI1_AGENT_INDICES        -> adaptive recruitment
全局 pairwise QP               -> local per-agent QP
外部强制移动 AOI               -> object dynamics driven by robots
全局密度点                     -> boundary-aware density points
```

### 6.2 Add DBACT Simulation Modules

建议按以下顺序实现：

```text
1. cargo.py
2. local_sensing.py
3. boundary_density.py
4. local_cvt.py
5. local_cbf_qp.py
6. transport_dynamics.py
7. controller.py
8. sim/run_sim.py
9. metrics.py
```

第一阶段先不要接入 MAS。先在纯仿真中跑通圆形、矩形、L 型和非凸多边形。

### 6.3 Integrate with MAS Platform

在 MAS 中新增控制器，不要覆盖已有 `CVTController`。

需要新增：

```text
MAS-public/src/controller/decentralized_transport_controller.py
MAS-public/src/controller/decentralized_transport_utils.py
MAS-public/configs/controllers/dtransport.yaml
```

需要修改：

#### `src/common/config_loader.py`

```python
CONTROLLER_TYPES = {"manual", "point", "cvt", "dtransport"}
```

#### `src/controller/controller_module.py`

```python
from src.controller.decentralized_transport_controller import DecentralizedTransportController
```

在 `_build_controller()` 中增加：

```python
if controller_type == "dtransport":
    return DecentralizedTransportController(
        self.controller_config,
        self.robot_ids,
        self.system_config["world"],
        limits_config,
    )
```

#### `configs/controller.yaml`

```yaml
controller:
  type: dtransport
  robot_mode: free
```

#### `configs/controllers/dtransport.yaml`

```yaml
sensor_range: 0.35
comm_range: 0.8
robot_radius: 0.15
cage_offset: 0.25
d_min: 0.35

kp_explore: 0.4
kp_cage: 1.0
kp_transport: 0.25
kp_spacing: 0.4

max_vx: 0.25
max_vy: 0.25
max_wz: 0.5

density:
  sigma: 0.25
  boundary_weight: 1.0
  gap_weight: 1.5
  confidence_weight: 0.5

cbf:
  gamma: 5.0
  slack_weight: 100.0
  solver: scipy

object_observer:
  mode: optitrack_mock_boundary
  marker_frame: cargo
  boundary_sample_count: 64

transport:
  mode: fixed_direction
  direction: [0.0, 1.0]
  coverage_threshold: 0.65
  min_contact_robots: 3
```

### 6.4 MAS Controller Skeleton

```python
from __future__ import annotations

import math
from typing import Any

import numpy as np

from src.common.math_utils import clamp
from src.common.messages import ControlCommand, RobotCommand, RobotState, WorldState
from src.common.time_utils import now_s
from src.controller.base_controller import BaseController


class DecentralizedTransportController(BaseController):
    """Boundary-aware decentralized cooperative transport controller."""

    def __init__(self, config: dict[str, Any], robot_ids: list[str], world_config: dict[str, Any], limits_config: dict[str, Any] | None = None):
        super().__init__(config, robot_ids)
        params = config.get("controller_params", {}).get("dtransport", {})
        chassis_limits = (limits_config or {}).get("chassis", {})

        self.world_config = world_config
        self.sensor_range = float(params.get("sensor_range", 0.35))
        self.comm_range = float(params.get("comm_range", 0.8))
        self.robot_radius = float(params.get("robot_radius", 0.15))
        self.cage_offset = float(params.get("cage_offset", 0.25))
        self.d_min = float(params.get("d_min", 0.35))
        self.kp_cage = float(params.get("kp_cage", 1.0))
        self.kp_transport = float(params.get("kp_transport", 0.25))
        self.max_vx = float(chassis_limits.get("max_vx", params.get("max_vx", 0.25)))
        self.max_vy = float(chassis_limits.get("max_vy", params.get("max_vy", 0.25)))
        self.max_wz = float(chassis_limits.get("max_wz", params.get("max_wz", 0.5)))

    def compute(self, world_state: WorldState | None) -> ControlCommand:
        if world_state is None:
            return self._zero_command("dtransport_zero")

        state_by_id = {robot.robot_id: robot for robot in world_state.robots}
        commands = []

        for robot_id in self.robot_ids:
            state = state_by_id.get(robot_id)
            if state is None or not state.tracked:
                commands.append(self._robot_zero(robot_id, "dtransport_untracked"))
                continue

            neighbors = self._local_neighbors(state, state_by_id)
            boundary_points = self._observe_local_boundary(state)
            u_nom_world = self._nominal_control(state, neighbors, boundary_points)
            u_safe_world = self._local_cbf_qp(state, neighbors, u_nom_world)
            commands.append(self._world_velocity_to_robot_command(state, u_safe_world, "dtransport"))

        return ControlCommand(timestamp=now_s(), robot_mode=self.robot_mode, commands=commands)

    def _local_neighbors(self, state: RobotState, state_by_id: dict[str, RobotState]) -> list[RobotState]:
        neighbors = []
        for other_id, other in state_by_id.items():
            if other_id == state.robot_id or not other.tracked:
                continue
            dist = math.hypot(other.x - state.x, other.y - state.y)
            if dist <= self.comm_range:
                neighbors.append(other)
        return neighbors

    def _observe_local_boundary(self, state: RobotState):
        # In simulation: ray casting against polygon.
        # In MAS first version: use OptiTrack/object marker to generate mock local boundary.
        return []

    def _nominal_control(self, state: RobotState, neighbors: list[RobotState], boundary_points) -> np.ndarray:
        # TODO: boundary density + local CVT centroid.
        return np.array([0.0, 0.0], dtype=float)

    def _local_cbf_qp(self, state: RobotState, neighbors: list[RobotState], u_nom_world: np.ndarray) -> np.ndarray:
        # TODO: solve local CBF-QP. First version can use clipping + repulsive fallback.
        return u_nom_world

    def _world_velocity_to_robot_command(self, state: RobotState, u_world: np.ndarray, mode: str) -> RobotCommand:
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        vx_body = cos_yaw * float(u_world[0]) + sin_yaw * float(u_world[1])
        vy_body = -sin_yaw * float(u_world[0]) + cos_yaw * float(u_world[1])
        return RobotCommand(
            robot_id=state.robot_id,
            chassis_vx=clamp(vx_body, -self.max_vx, self.max_vx),
            chassis_vy=clamp(vy_body, -self.max_vy, self.max_vy),
            chassis_wz=0.0,
            gimbal_yaw_speed=0.0,
            gimbal_pitch_speed=0.0,
            controller_mode=mode,
        )

    def _zero_command(self, mode: str) -> ControlCommand:
        return ControlCommand(timestamp=now_s(), robot_mode=self.robot_mode, commands=[self._robot_zero(robot_id, mode) for robot_id in self.robot_ids])

    @staticmethod
    def _robot_zero(robot_id: str, mode: str) -> RobotCommand:
        return RobotCommand(robot_id, 0.0, 0.0, 0.0, 0.0, 0.0, mode)
```

---

## 7. Installation

### 7.1 Simulation Environment

Recommended Python version: `3.10` or `3.11`.

```bash
git clone https://github.com/<your-name>/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport

python -m venv .venv
source .venv/bin/activate  # Linux/macOS
# .\.venv\Scripts\Activate  # Windows PowerShell

python -m pip install --upgrade pip
pip install -r requirements.txt
```

Suggested `requirements.txt`:

```text
numpy>=1.26
scipy>=1.13
matplotlib>=3.9
pandas>=2.2
shapely>=2.0
cvxpy>=1.4
osqp>=0.6
pyyaml>=6.0
pytest>=8.0
```

If you keep compatibility with the original baseline, you may also need:

```text
geovoronoi>=0.2.0
cvxopt>=1.3.3
```

### 7.2 MAS Environment

For MAS real-robot experiments, follow MAS platform setup:

```bash
cd MAS-public
python -m venv .venv
.\.venv\Scripts\Activate
python -m pip install --upgrade pip
pip install -r requirements.txt
python -m pytest -q
```

Real hardware requires:

- RoboMaster S1 robots;
- OptiTrack cameras;
- Motive;
- NatNet SDK Python client;
- same LAN connection between computer and RoboMaster S1;
- physical emergency stop or manual safety method.

---

## 8. Running Simulations

### 8.1 Reproduce Baseline

```bash
python scripts/reproduce_baseline.py
```

Expected outputs:

```text
data/baseline/agent_positions.csv
data/baseline/coverage_rates.csv
figures/baseline/FIG_001.png ... FIG_130.png
figures/baseline/coverage_rate_curve.png
```

### 8.2 Run DBACT on Arbitrary Shapes

```bash
python -m src.sim.run_sim --config configs/sim/circle.yaml
python -m src.sim.run_sim --config configs/sim/rectangle.yaml
python -m src.sim.run_sim --config configs/sim/l_shape.yaml
python -m src.sim.run_sim --config configs/sim/nonconvex.yaml
```

Example scenario config:

```yaml
world:
  x_min: 0.0
  x_max: 8.0
  y_min: 0.0
  y_max: 8.0
  dt: 0.05
  steps: 2000

robots:
  count: 12
  init_mode: grid
  robot_radius: 0.15

sensing:
  sensor_range: 0.35
  comm_range: 0.8
  n_rays: 64

cargo:
  shape: l_shape
  pose: [3.0, 3.0, 0.0]
  mass: 1.0
  target_direction: [0.0, 1.0]

controller:
  cage_offset: 0.25
  d_min: 0.35
  kp_explore: 0.4
  kp_cage: 1.0
  kp_transport: 0.25
  max_speed: 0.25

cbf:
  gamma: 5.0
  slack_weight: 100.0

output:
  save_frames: true
  save_csv: true
  output_dir: data/experiments/l_shape
```

---

## 9. Running with MAS

### 9.1 Mock Closed-Loop Test

Before using real robots, run mock tests:

```bash
cd MAS-public
python apps/manual_tests/mock_optitrack.py
python apps/manual_tests/mock_robot.py
python apps/run_controller.py
```

Set:

```yaml
controller:
  type: dtransport
  robot_mode: free
```

### 9.2 Real-Robot Test

Recommended startup order:

```bash
python apps/run_optitrack.py
python apps/run_robot_comm.py
python apps/run_controller.py
```

Or use supervisor:

```bash
python apps/run_supervisor.py
```

Safety checklist:

- reduce `max_vx`, `max_vy`, `max_wz` for the first test;
- enable world bounds stop;
- enable watchdog stop;
- verify all robot IDs and rigid-body mappings;
- test with `manual` or `point` controller before `dtransport`;
- keep physical emergency stop ready.

---

## 10. Evaluation Metrics

Recommended metrics:

```text
transport_success_rate     # whether cargo reaches target region
object_displacement_error  # final cargo pose error
completion_time            # time to finish transport
boundary_coverage_ratio    # visible/covered boundary ratio
largest_boundary_gap       # maximum uncovered boundary segment
number_of_recruited_robots # adaptive team size
minimum_inter_robot_dist   # safety
collision_count            # robot-robot or robot-object invalid collision
control_energy             # Σ ||u_i||² dt
trajectory_smoothness      # oscillation / acceleration proxy
communication_load         # object token messages per second
robustness_to_failure      # success under robot dropout
```

Suggested benchmark table:

| Scenario | Shape | Robots | Sensor Range | Comm Range | Success | Time | Boundary Coverage | Min Distance | Energy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| S1 | Circle | 12 | 0.35 | 0.80 | TBD | TBD | TBD | TBD | TBD |
| S2 | Rectangle | 12 | 0.35 | 0.80 | TBD | TBD | TBD | TBD | TBD |
| S3 | L-shape | 12 | 0.35 | 0.80 | TBD | TBD | TBD | TBD | TBD |
| S4 | Non-convex | 12 | 0.35 | 0.80 | TBD | TBD | TBD | TBD | TBD |

---

## 11. Development Roadmap

### Stage 0: Baseline Reproduction

- [ ] Run original `Cooperative-Transport-Multi-Agent-System`.
- [ ] Save `FIG_1` to `FIG_130`.
- [ ] Save trajectory CSV and coverage curve.
- [ ] Document limitations of fixed circular AOIs and global assignment.

### Stage 1: Arbitrary Shape Simulation

- [ ] Implement `Cargo` polygon model.
- [ ] Add circle, rectangle, L-shape, non-convex polygon scenarios.
- [ ] Replace circular AOI logic with polygon boundary logic.
- [ ] Add shape benchmark script.

### Stage 2: Local Boundary Sensing

- [ ] Implement ray casting boundary detection.
- [ ] Ensure controller cannot access full polygon.
- [ ] Add local boundary memory and object token.
- [ ] Visualize detected boundary points and cage target points.

### Stage 3: Boundary-Aware Density + Local CVT

- [ ] Generate density from cage target points.
- [ ] Implement local / limited CVT.
- [ ] Add gap-aware weight.
- [ ] Compare with original density-based baseline.

### Stage 4: Local CBF-QP

- [ ] Replace global pairwise QP with per-agent local QP.
- [ ] Add robot-robot safety constraints.
- [ ] Add object boundary/contact constraints.
- [ ] Add slack variable for infeasible QP.

### Stage 5: Transport Dynamics

- [ ] Add simplified pushing dynamics.
- [ ] Move object based on robot contact velocities.
- [ ] Add success condition and object pose logging.
- [ ] Test multi-object transportation.

### Stage 6: MAS Integration

- [ ] Add `dtransport` controller to MAS.
- [ ] Run mock OptiTrack + mock robot closed-loop test.
- [ ] Use OptiTrack marker to simulate object boundary.
- [ ] Run low-speed RoboMaster S1 experiment.
- [ ] Record and plot real trajectories.

---

## 12. Troubleshooting

### `ModuleNotFoundError: shapely`

```bash
pip install shapely
```

### `geovoronoi` install fails

If only running DBACT refactored version, prefer grid-based or scipy-based local Voronoi and avoid depending on `geovoronoi`. If reproducing the original baseline, install:

```bash
pip install geovoronoi==0.2.0
```

### QP infeasible

Try:

- reduce robot speed;
- increase `d_min` carefully;
- add slack variable;
- reduce `kp_cage`;
- enlarge `comm_range`;
- lower `gamma` if constraints become too aggressive.

### Robots oscillate near boundary

Try:

- increase density smoothing `sigma`;
- add low-pass filter to target points;
- reduce `kp_cage`;
- increase damping term;
- add hysteresis to recruitment state.

### MAS robot moves in wrong direction

Check:

- coordinate transform setting;
- Motive Z-up / control frame conversion;
- body-frame vs world-frame velocity conversion;
- robot yaw angle sign;
- `robot_command_transform` in `configs/system.yaml`.

---

## 13. Citation

If this project is used in academic work, please cite the baseline paper and this repository:

```bibtex
@misc{dbact2026,
  title        = {DBACT: Decentralized Boundary-Aware Cooperative Transportation},
  author       = {Kaixin Wu},
  year         = {2026},
  howpublished = {GitHub repository},
  note         = {Research prototype for boundary-aware cooperative transportation of arbitrarily shaped objects}
}
```

Also cite the related baseline:

```bibtex
@inproceedings{song2026cooperative,
  title     = {Cooperative Transportation Without Prior Object Knowledge via Adaptive Self-Allocation and Coordination},
  author    = {Song, Jie and Bai, Yang and Wakamiya, Naoki},
  year      = {2026}
}
```

---

## 14. License

This repository is intended for academic research and experimental validation. Please check the licenses of all third-party sources before redistribution, especially if copying code from the baseline project or MAS platform.

---

## 15. Short Summary

DBACT extends density-driven CVT/CBF cooperative transportation from circular-object simulation toward arbitrary-shape, local-boundary-aware, decentralized caging and pushing. The recommended development path is:

```text
Cooperative-Transport baseline
  -> arbitrary polygon simulation
  -> local boundary sensing
  -> boundary-aware density field
  -> local CVT + local CBF-QP
  -> simplified contact-based transport dynamics
  -> MAS dtransport controller
  -> RoboMaster S1 + OptiTrack validation
```
