<div align="center">

# DBACT: Decentralized Boundary-Aware Cooperative Transport

可復現的去中心化多機器人圍捕與搬運實驗，包含指標、報告、MAS dry-run 與可視化。

[English](README.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)
![Tests](https://img.shields.io/badge/Tests-16%20passed-brightgreen.svg)
![Version](https://img.shields.io/badge/Version-0.1.0-informational.svg)
![Visualization](https://img.shields.io/badge/Visualization-Matplotlib-orange.svg)
![Platform](https://img.shields.io/badge/Platform-MAS%20%7C%20RoboMaster%20S1-lightgrey.svg)

</div>

DBACT 是 **Decentralized Boundary-Aware Cooperative Transport** 的研究原型。它研究多台移動機器人如何在不知道物體完整形狀、中心、半徑與所需隊伍規模的情況下，形成有用的圍捕與搬運結構。

本倉庫整合了獨立仿真系統、邊界感知局部控制、指標、可在 GitHub 顯示的可視化素材、MAS-compatible 控制器適配、OptiTrack 只讀工具，以及保守的 RoboMaster S1 命令 smoke test。

> 本倉庫是研究原型，不是已完成的實體搬運產品。仿真與 dry-run 路徑已可運行；完整實體搬運仍是分階段驗證目標。

---

## 視覺展示

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

> 從仿真 replay 生成並已提交的 GIF。不同於本地 `runs/*.gif`，此檔案位於 `docs/assets/`，可直接在 GitHub 顯示。

![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png)

> 論文風格畫面，同時展示未知 cargo、局部 CVT / Voronoi 結構、機器人安全區域與邊界感知密度曲面。

---

## 媒體展示

以下內嵌媒體都指向 `docs/assets/` 下已提交的檔案，因此不依賴本地 run artifact 也能在 GitHub 顯示。

| 動畫 | 密度場 + 局部 CVT |
| --- | --- |
| <img src="docs/assets/dbact-moving-cargo.gif" alt="DBACT moving cargo animation" width="100%"> | <img src="docs/assets/dbact-density-cvt-frame.png" alt="DBACT density and local CVT frame" width="100%"> |

| 軌跡 | 覆蓋率曲線 |
| --- | --- |
| <img src="docs/assets/dbact-trajectory.png" alt="DBACT trajectory" width="100%"> | <img src="docs/assets/dbact-coverage-curve.png" alt="DBACT coverage curve" width="100%"> |

| 最終截圖 | 素材清單 |
| --- | --- |
| <img src="docs/assets/dbact-final-snapshot.png" alt="DBACT final snapshot" width="100%"> | [`docs/assets/README.md`](docs/assets/README.md) |

仿真仍會在 `runs/` 或 `platforms/mas_public/data/` 下生成 PNG、GIF、CSV 與 MP4，但這些檔案預設被 Git 忽略。若要在 GitHub 顯示，請將精選素材複製到 `docs/assets/`，或透過 GitHub Releases 發布較大的影片。

---

## 專案快照

| 項目 | 說明 |
| --- | --- |
| 專案名稱 | DBACT: Decentralized Boundary-Aware Cooperative Transport |
| 目的 | 測試未知形狀物體周圍的局部邊界感知圍捕與搬運。 |
| 核心技術 | Python 3.9+, NumPy, PyYAML, Matplotlib, pytest |
| 主要場景 | `paper_like_irregular_moving_cargo.yaml`, `l_shape.yaml`, `nonconvex.yaml`, `multi_object.yaml` |
| 輸出類型 | CSV 軌跡、覆蓋率指標、JSON 摘要、PNG 圖表、GIF 動畫 |
| 集成路徑 | DBACT simulation -> MAS dry-run -> OptiTrack read-only -> RoboMaster S1 smoke test |
| 當前狀態 | 仿真與 dry-run 可用；完整實體實驗尚未完成 |

---

## 核心特性

- **去中心化邊界感知控制**：機器人使用局部邊界觀測與鄰居狀態，而不是全域物體幾何。
- **未知物體圍捕**：控制器避免直接使用 `cargo.center`、`cargo.radius`、`cargo.vertices` 與 closest-boundary query。
- **局部 CVT 分配**：每台機器人使用自己與附近鄰居計算局部 weighted centroid。
- **CBF-style 安全過濾**：機器人間距約束與速度限制讓圍捕過程更保守。
- **可視化優先流程**：仿真輸出軌跡、覆蓋率曲線、最終截圖、論文風格幀圖與可選 GIF。
- **面向硬體的分階段驗證**：MAS adapter、OptiTrack 只讀 logging 與 RoboMaster S1 smoke test 為真實實驗做準備。

---

## 結果與可視化

### Stage 1 未知多邊形圍捕

Stage 1 驗證了：即使控制器內部不直接讀取完整 cargo 幾何，也可以對任意多邊形 cargo 形成圍捕。

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

### Tight Baseline 結果

Tight baseline 透過降低 cage offset 並縮窄 density field 來提升圍捕緊湊度。

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

### 移動不規則 Cargo Demo

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

**解讀**

- Tight caging 在 Stage 1 benchmark 中提升邊界覆蓋率，同時將最小機器人間距保持在 0.33 m 以上。
- Moving-cargo demo 顯示了圍捕與類搬運位移，但物理接觸動力學仍是簡化模型。
- 目前結果是仿真與 MAS dry-run 證據，不是完整真實世界搬運宣稱。

---

## 快速上手

### 1. 克隆

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. 建立環境

Conda：

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .[dev]
```

Windows PowerShell virtual environment：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
python -m pip install -e ".[dev]"
```

macOS / Linux virtual environment：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ".[dev]"
```

### 3. 一行 Smoke Experiment

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

重要輸出：

- `runs/paper_like_irregular_moving_cargo/animation.gif`
- `runs/paper_like_irregular_moving_cargo/trajectory.png`
- `runs/paper_like_irregular_moving_cargo/final_snapshot.png`
- `runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png`
- `runs/paper_like_irregular_moving_cargo/metrics.json`
- `runs/paper_like_irregular_moving_cargo/figures/FIG_520.png`

---

## 運作原理

1. **讀取場景配置**
   YAML 定義工作空間、cargo 幾何、機器人初始狀態、感測範圍、通訊範圍、搬運方向與控制器參數。

2. **產生局部邊界觀測**
   仿真器使用 cargo 幾何產生局部邊界觀測，但控制器不直接讀取完整 cargo 形狀。

3. **建立 cage target**
   每個觀測到的邊界點 `b` 沿外法向偏移：

```text
q_target = b + d_cage * n_out
```

4. **建立邊界感知密度**
   cage target 變成 Gaussian density peak，吸引機器人前往有效的邊界外側位置。

5. **執行局部 CVT 分配**
   每台機器人使用自己與通訊範圍內鄰居計算局部 weighted centroid。

6. **套用安全過濾**
   CBF-style filter 維持機器人間距高於設定下限：

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **保存 replay、指標與圖表**
   仿真輸出 CSV log、metrics、final snapshot、trajectory plot、coverage curve、paper-style frame 與可選動畫。

---

## 倉庫結構

```text
boundary-aware-cooperative-transport/
|-- configs/                         # 仿真與 MAS 配置
|   |-- sim/
|   `-- mas/
|-- src/
|   |-- dbact/                       # 核心控制、感測、密度、CVT、安全、指標
|   |-- dbact_sim/                   # 仿真環境、場景、可視化、CLI
|   `-- mas_adapter/                 # MAS-compatible controller adapter
|-- scripts/                         # 批量運行、mock MAS pipeline、RoboMaster S1 smoke tests
|-- docs/                            # 架構、演算法筆記、報告、分階段驗證
|   |-- assets/                      # GitHub 可顯示的 README 媒體素材
|   |-- ARCHITECTURE.md
|   |-- ALGORITHM.md
|   |-- MAS_INTEGRATION.md
|   `-- stage1_results.md
|-- platforms/mas_public/            # Vendored MAS platform code
|-- runs/                            # 本地生成 runs，Git 忽略
|-- tests/                           # 單元測試與 smoke tests
|-- README.md
|-- README.en.md
|-- README.zh-TW.md
`-- README.ja.md
```

---

## 常用命令

運行標準場景：

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

運行 L-shape 場景：

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

運行 mock MAS pipeline：

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

運行 MAS dry-run：

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

運行 OptiTrack 只讀檢查：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

運行 RoboMaster S1 mock command smoke test：

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

運行測試：

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## 當前研究方向

下一步的重點是分階段驗證，而不是擴張無關大系統。優先項目包括：

- 保持 `docs/assets/` 作為穩定的 GitHub media surface；
- 增加 side-by-side 場景對比圖；
- 改進 moving-cargo transport metrics 與 dashboard 摘要；
- 以真實 boundary-observation pipeline 取代 virtual-object 假設；
- 驗證 Motive rigid body，直到機器人姿態穩定；
- 只在 read-only logging 與 dry-run 通過後，進行低速 caging-only 實體實驗。

---

## 安全說明

- 啟用控制器輸出前，先執行只讀 OptiTrack logging。
- 逐台確認 robot ID 與 rigid-body mapping。
- 第一次實體運行使用極低速度限制。
- 硬體測試期間保持物理急停可用。
- 每次運行後檢查 command 與 state log。

---

## 貢獻與授權

歡迎透過 Issues 和 Pull Requests 貢獻。新場景、更清晰的可視化、更強的指標，以及更好的分階段硬體驗證尤其有幫助。

本專案採用 [MIT License](LICENSE)。
