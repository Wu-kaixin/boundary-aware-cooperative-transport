# DBACT: Decentralized Boundary-Aware Cooperative Transportation

> 面向未知任意形状物体的去中心化边界感知协同搬运算法  
> Decentralized Boundary-Aware Cooperative Transportation of Arbitrarily Shaped Objects Without Prior Object Knowledge

DBACT 是一个面向多机器人协同搬运任务的研究型算法框架。它面向未知位置、未知尺寸、未知任意形状物体，让多个机器人仅依靠局部感知、局部通信和局部控制，自适应地发现物体边界、围合物体，并通过 caging / pushing 完成协同搬运。

本仓库为DBACT的项目框架，包含：

- 可运行的 Python 仿真代码；
- 任意多边形物体模型；
- 局部边界感知；
- boundary-aware density field；
- local / limited CVT；
- local CBF safety filter；
- 简化 caging-pushing transport dynamics；
- MAS / RoboMaster S1 平台接入适配层；
- 配置文件、测试文件、文档和 GitHub Actions。

---

## Quick Start

```bash
# 1. clone repository
cd boundary-aware-cooperative-transport

# 2. create environment
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

# 3. install dependencies
pip install -e .[dev]

# 4. run a demo simulation
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape

# 5. run tests
pytest
```

仿真输出会保存在：

```text
runs/l_shape/
├── trajectories.csv
├── metrics.json
├── final_snapshot.png
└── trajectory.png
```

---

## Repository Structure

```text
boundary-aware-cooperative-transport/
├── README.md
├── LICENSE
├── pyproject.toml
├── requirements.txt
├── .gitignore
├── configs/
│   ├── sim/
│   │   ├── circle.yaml
│   │   ├── rectangle.yaml
│   │   ├── l_shape.yaml
│   │   ├── nonconvex.yaml
│   │   └── multi_object.yaml
│   └── mas/
│       ├── controller.yaml
│       └── dtransport.yaml
├── src/
│   ├── dbact/
│   │   ├── __init__.py
│   │   ├── types.py
│   │   ├── geometry.py
│   │   ├── cargo.py
│   │   ├── local_sensing.py
│   │   ├── boundary_map.py
│   │   ├── boundary_density.py
│   │   ├── local_cvt.py
│   │   ├── local_cbf_qp.py
│   │   ├── transport_dynamics.py
│   │   ├── controller.py
│   │   └── metrics.py
│   ├── dbact_sim/
│   │   ├── __init__.py
│   │   ├── scenarios.py
│   │   ├── environment.py
│   │   ├── visualization.py
│   │   └── run_sim.py
│   └── mas_adapter/
│       ├── __init__.py
│       ├── decentralized_transport_controller.py
│       └── object_observer.py
├── scripts/
│   ├── run_all_scenarios.py
│   └── make_repo_tree.py
├── tests/
├── docs/
│   ├── ARCHITECTURE.md
│   ├── ALGORITHM.md
│   ├── MAS_INTEGRATION.md
│   └── ROADMAP.md
└── .github/workflows/tests.yml
```

---

## Algorithm Pipeline

```text
local sensing
  -> boundary point detection
  -> cage target generation
  -> boundary-aware density field
  -> local / limited CVT
  -> local CBF safety filter
  -> caging + pushing transport
```

### 1. Local Boundary Sensing

每个机器人只在 `sensor_range` 内观测物体边界点。仿真中由 `LocalBoundarySensor` 从 polygon boundary 采样得到局部边界观测。

### 2. Boundary-Aware Density Field

对每个边界点 `b` 和外法向 `n_out`，生成 cage target：

```text
q_target = b + d_cage * n_out
```

然后叠加 Gaussian kernel 得到局部密度场：

```text
rho(q) = rho0 + sum_i w_i exp(-||q - q_i||^2 / (2 sigma^2))
```

### 3. Local / Limited CVT

每个机器人只使用自己和通信范围内邻居的位置，在局部采样点上近似计算 weighted Voronoi centroid。

### 4. Local CBF Safety Filter

每个机器人只对通信范围内邻居构造安全约束：

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

当前实现使用半平面投影形式作为轻量级 CBF-QP safety filter，便于无 cvxpy 环境直接运行。后续可替换为标准 QP 求解器。

### 5. Caging / Pushing Transport Dynamics

仿真环境用简化模型判断边界覆盖率。当物体被足够多机器人围合后，物体沿配置文件中的 `transport_direction` 运动。该模块用于算法验证，不等价于完整刚体接触动力学。

---

## Run Examples

```bash
python -m dbact_sim.run_sim --config configs/sim/circle.yaml --steps 300 --output runs/circle
python -m dbact_sim.run_sim --config configs/sim/rectangle.yaml --steps 400 --output runs/rectangle
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 500 --output runs/l_shape
python -m dbact_sim.run_sim --config configs/sim/nonconvex.yaml --steps 500 --output runs/nonconvex
python -m dbact_sim.run_sim --config configs/sim/multi_object.yaml --steps 600 --output runs/multi_object
```

或者运行全部场景：

```bash
python scripts/run_all_scenarios.py
```

---

## MAS / RoboMaster S1 Integration

本仓库没有直接修改 MAS-public 源码，而是在 `src/mas_adapter/` 中提供适配层。推荐做法是：

1. 保留 MAS-public 原项目；
2. 将 `src/mas_adapter/decentralized_transport_controller.py` 复制或软链接到 MAS-public 的 `src/controller/`；
3. 将 `configs/mas/dtransport.yaml` 复制到 MAS-public 的 `configs/controllers/dtransport.yaml`；
4. 在 MAS-public 的 `config_loader.py` 中加入 `dtransport` 类型；
5. 在 `controller_module.py` 中注册 `DecentralizedTransportController`；
6. 将 MAS 的 `configs/controller.yaml` 中 controller type 改为 `dtransport`。

详细操作见：[`docs/MAS_INTEGRATION.md`](docs/MAS_INTEGRATION.md)。

---

## Current Status

当前版本是完整的研究型项目骨架：

- 仿真代码可以运行；
- 算法模块已拆分；
- 任意形状 polygon 支持已具备；
- MAS 接入层已给出；
- real-world object sensing、真实接触动力学和实机闭环仍需要后续继续实现。

---

## Citation / Acknowledgement

本项目设计参考以下方向：

- Cooperative Transportation Without Prior Object Knowledge via Adaptive Self-Allocation and Coordination
- CVT-based multi-agent coverage control
- Control Barrier Function based safety-critical control
- MAS-public RoboMaster S1 multi-agent platform

---

## License

MIT License.
