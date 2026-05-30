# DBACT: Decentralized Boundary-Aware Cooperative Transportation

> **Decentralized Boundary-Aware Cooperative Transportation of Arbitrarily Shaped Objects Without Prior Object Knowledge** > 面向未知任意形状物体的去中心化边界感知协同搬运算法

DBACT is a research-grade algorithmic framework designed for multi-robot cooperative transportation tasks involving objects of unknown positions, sizes, and arbitrary geometry. Relying **strictly** on local sensing, local communication, and local control, robots adaptively discover object boundaries, encircle the cargo, and achieve cooperative transportation via caging and pushing dynamics.

---

## 🌟 Key Features

* **Zero Prior Knowledge:** No pre-loaded maps, shapes, or size profiles of the cargo are required.
* **Decentralized Coordination:** Scalable swarm control driven entirely by local peer-to-peer communication and limited-range sensing.
* **Boundary-Aware Density Fields:** Adaptive target generation utilizing Gaussian kernels along the object's outer normals.
* **Safety-Critical Execution:** Lightweight Control Barrier Function (CBF) safety filter ensures collision-free multi-agent interaction.
* **Hardware-Ready Architecture:** Includes an out-of-the-box adapter layer tailored for the RoboMaster S1 MAS (Multi-Agent System) platform.

---

## 🛠️ Repository Architecture

```text
boundary-aware-cooperative-transport/
├── configs/                  # Configuration files
│   ├── sim/                  # Simulation scenario profiles (Circle, L-shape, etc.)
│   └── mas/                  # MAS platform integration configurations
├── src/                      # Source code
│   ├── dbact/                # Core algorithmic modules
│   │   ├── geometry.py       # Polygon and vector geometry helpers
│   │   ├── local_sensing.py  # Local boundary sensor emulation
│   │   ├── local_cvt.py      # Constrained/Limited Centroidal Voronoi Tessellation
│   │   └── local_cbf_qp.py   # Analytical/QP Control Barrier Function safety filter
│   ├── dbact_sim/            # Lightweight simulation engine
│   └── mas_adapter/          # Bridge adapters for the RoboMaster S1 MAS platform
├── scripts/                  # Batch execution and automation scripts
├── tests/                    # Unit testing suite (PyTest)
└── docs/                     # In-depth technical documentation

```

---

## ⚡ Quick Start

### 1. Environment Setup

Clone the repository and set up a virtual environment:

```bash
git clone https://github.com/yourusername/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport

python -m venv .venv
source .venv/bin/activate  # On Windows use: .venv\Scripts\activate

```

### 2. Install Dependencies

Install the package in editable mode along with development dependencies:

```bash
pip install -e .[dev]

```

### 3. Run a Demo Simulation

Execute a benchmark simulation with an asymmetrical L-shaped object:

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape

```

Outputs will be saved dynamically to `runs/l_shape/`:

* `trajectories.csv` & `metrics.json` (Numerical logs)
* `trajectory.png` & `final_snapshot.png` (Visual summaries)

### 4. Run Tests

Verify the installation by running the test suite:

```bash
pytest

```

---

## 🧬 Algorithmic Pipeline

```text
[ Local Sensing ] ──> [ Boundary Point Detection ] ──> [ Cage Target Generation ]
                                                                   │
                                                                   ▼
[ Collision-Free Output ] <── [ Local CBF Filter ] <── [ Local/Limited CVT ]
           │
           ▼
[ Caging & Pushing Transport ]

```

### Deep Dive into Core Modules

1. **Local Boundary Sensing:** Robots detect raw boundary points only within their predefined `sensor_range`. The `LocalBoundarySensor` samples these points directly from the target polygon dynamically.
2. **Boundary-Aware Density Field:** For every detected boundary point $b$ and its corresponding outer normal $n_{\text{out}}$, a caging target is generated via:

$$q_{\text{target}} = b + d_{\text{cage}} \cdot n_{\text{out}}$$



A continuous local density field is then constructed by overlaying Gaussian kernels:

$$\rho(q) = \rho_0 + \sum_i w_i \exp\left(-\frac{\|q - q_i\|^2}{2\sigma^2}\right)$$


3. **Local/Limited CVT:** Robots compute a weighted Centroidal Voronoi Tessellation (CVT) over locally sampled points, restricted by their communication topologies.
4. **Local CBF Safety Filter:** Inter-robot collision avoidance constraints are strictly enforced via a half-plane projection implementation of Control Barrier Functions ($h_{ij} = \|p_i - p_j\|^2 - d_{\text{min}}^2 \ge 0$), keeping the system lightweight and independent of heavy QP solvers like `cvxpy`.
5. **Transport Dynamics:** A stylized caging-pushing dynamic evaluates boundary coverage. Once encirclement criteria are satisfied, the object is transported cooperatively along the designated vector.

---

## 📊 Evaluation Scenarios

Run the provided benchmarks to test the system's robustness across various geometries:

| Command | Target Geometry | Challenge Highlight |
| --- | --- | --- |
| `python -m dbact_sim.run_sim --config configs/sim/circle.yaml` | **Circle** | Baseline uniformity test |
| `python -m dbact_sim.run_sim --config configs/sim/rectangle.yaml` | **Rectangle** | High-curvature corner negotiation |
| `python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml` | **L-Shape** | Asymmetric center-of-mass balancing |
| `python -m dbact_sim.run_sim --config configs/sim/nonconvex.yaml` | **Non-Convex** | Local minima avoidance inside cavities |
| `python -m dbact_sim.run_sim --config configs/sim/multi_object.yaml` | **Multi-Object** | Dynamic cluster splitting and allocation |

> 💡 **Tip:** To evaluate all scenarios in a single batch run, execute: `python scripts/run_all_scenarios.py`

---

## 🤖 RoboMaster S1 / MAS Integration

The framework keeps the upstream `MAS-public` repository untouched by operating entirely through an adapter layer located in `src/mas_adapter/`.

To deploy DBACT onto your MAS platform workspace, follow these integration steps:

1. Copy or symlink `src/mas_adapter/decentralized_transport_controller.py` into your MAS-public directory at `src/controller/`.
2. Copy `configs/mas/dtransport.yaml` into your MAS-public directory at `configs/controllers/dtransport.yaml`.
3. Register the `dtransport` type inside the MAS `config_loader.py`.
4. Register `DecentralizedTransportController` within your `controller_module.py`.
5. Update your primary system config (`configs/controller.yaml`) by changing the controller type to `dtransport`.

For a comprehensive guide, please refer to [docs/MAS_INTEGRATION.md](https://www.google.com/search?q=docs/MAS_INTEGRATION.md).

---

## 📈 Project Status & Roadmap

This repository contains the complete, verified algorithmic framework for decentralized multi-agent coordination.

* [x] Fully functional Python simulation environment
* [x] Modular boundary-aware density field & CVT control loop
* [x] Analytical CBF safety filtering
* [x] RoboMaster S1 hardware adapter hooks
* [ ] Real-world point cloud noise filtering modules
* [ ] High-fidelity rigid-body contact dynamics integration
* [ ] Closed-loop physical fleet deployment tests

---

## 📄 Citation & Acknowledgements

This framework builds upon foundational theories in multi-agent robotics. If you use this software in your research, please cite our project alongside the following foundational methodologies:

* **CVT-based multi-agent coverage control** (Centroidal Voronoi Tessellations)
* **Control Barrier Function** (CBF) safety-critical control laws
* **Adaptive Self-Allocation paradigms** for cooperative manipulation without prior shape knowledge
* The **MAS-public** RoboMaster S1 multi-agent development ecosystem

---

## ⚖️ License

Distributed under the **MIT License**. See `LICENSE` for more information.
