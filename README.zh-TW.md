<div align="center">

# DBACT：邊界感知協同搬運

**面向未知形狀物體的去中心化邊界感知多機器人圍捕與搬運。**

<p>
  <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License: MIT">
  <img src="https://img.shields.io/badge/python-%3E%3D3.9-3776AB.svg" alt="Python >= 3.9">
  <img src="https://img.shields.io/badge/tests-pytest%20passing-brightgreen.svg" alt="pytest passing">
  <img src="https://img.shields.io/badge/status-research%20prototype-orange.svg" alt="Research prototype">
</p>

[English](README.en.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

</div>

---

## 專案概覽

DBACT 是一套多機器人去中心化協同搬運研究軟體棧。它處理的問題是：機器人不預先知道物體完整幾何、中心、半徑，也沒有固定分工，仍然要依靠局部觀測形成穩定圍捕並推進搬運。

系統使用局部邊界觀測、鄰居通訊、邊界感知密度、局部 CVT 目標分配與安全過濾，讓多台移動機器人自組織地靠近物體邊界外側的有效位置。倉庫也包含仿真工具、MAS 控制器適配、OptiTrack 只讀記錄流程與 RoboMaster S1 命令 smoke test。

**核心技術棧**：Python 3.9+, NumPy, Matplotlib, PyYAML, pytest, local CVT, CBF-style safety filtering, MAS adapter, OptiTrack read-only bridge, RoboMaster S1 smoke tests。

---

## 視覺展示

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

<p align="center">
  <sub>移動不規則物體場景：機器人透過局部感知與局部協調形成邊界感知圍捕結構，並推動物體前進。</sub>
</p>

| 密度場與局部 CVT | 運動軌跡 |
| --- | --- |
| ![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png) | ![DBACT trajectory](docs/assets/dbact-trajectory.png) |

| 覆蓋率曲線 | 最終狀態截圖 |
| --- | --- |
| ![DBACT coverage curve](docs/assets/dbact-coverage-curve.png) | ![DBACT final snapshot](docs/assets/dbact-final-snapshot.png) |

---

## 核心特性

- **適合未知幾何物體**：控制器不直接讀取物體中心、半徑、頂點或最近邊界查詢。
- **仿真輸出完整**：每次運行可產生軌跡圖、覆蓋率曲線、最終截圖、論文風格幀圖、CSV 記錄與可選 GIF。
- **邊界感知協調**：局部觀測會轉換成 cage target，再形成吸引機器人的邊界密度場。
- **局部 CVT 分配**：每台機器人只使用自己和通訊範圍內鄰居的位置計算目標。
- **面向硬體驗證**：提供 MAS adapter、OptiTrack 只讀 logging，以及 RoboMaster S1 低速命令測試路徑。

---

## 實驗結果與可視化

### Stage 1：未知多邊形圍捕

Stage 1 驗證了核心主張：即使控制器不直接使用完整物體幾何，也可以只依靠局部邊界觀測、邊界感知密度、局部 CVT 和機器人間安全過濾形成穩定圍捕。

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

### Tight Baseline

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

### 移動不規則物體 Demo

| 指標 | 數值 |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

**結論**：Tight caging 設定可以顯著提升邊界覆蓋率，同時在 Stage 1 報告的基準測試中保持最小機器人間距大於 0.33 m。

---

## 快速上手

### 1. 克隆倉庫

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. 建立環境

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
```

也可以使用標準虛擬環境：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Windows PowerShell：

```powershell
.\.venv\Scripts\Activate
```

### 3. 安裝依賴

```bash
pip install -r requirements.txt
pip install -e .[dev]
```

### 4. 一行命令跑通 Demo

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

預期輸出：

```text
runs/paper_like_irregular_moving_cargo/
|-- animation.gif
|-- final_snapshot.png
|-- trajectory.png
|-- coverage_rate_curve.png
|-- trajectories.csv
|-- coverage_rates.csv
`-- figures/
    |-- FIG_0.png
    |-- FIG_130.png
    |-- FIG_260.png
    |-- FIG_390.png
    `-- FIG_520.png
```

### 5. 批量運行標準場景

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

### 6. 驗證

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform 測試：

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## 倉庫結構

```text
boundary-aware-cooperative-transport/
|-- README.md                         # 語言入口與視覺預覽
|-- README.en.md                      # 英文 README
|-- README.zh-TW.md                   # 繁體中文 README
|-- README.ja.md                      # 日文 README
|-- configs/
|   |-- sim/                          # 仿真場景配置
|   `-- mas/                          # MAS 控制器配置
|-- docs/
|   |-- assets/                       # 可在 GitHub 顯示的 README 圖片與 GIF
|   |-- ARCHITECTURE.md               # 架構說明
|   |-- ALGORITHM.md                  # 演算法說明
|   `-- stage1_results.md             # Stage 1 實驗結果
|-- src/
|   |-- dbact/                        # 核心演算法模組
|   |-- dbact_sim/                    # 仿真環境與可視化
|   `-- mas_adapter/                  # MAS 適配層
|-- scripts/                          # 批量運行、MAS mock pipeline、S1 smoke test
|-- tests/                            # 單元測試與 smoke tests
|-- runs/                             # 本地生成輸出，預設不提交 Git
`-- platforms/mas_public/             # Vendored MAS platform 程式碼
```

---

## 運作原理

1. **載入場景**：`dbact_sim.run_sim` 讀取 YAML，初始化世界、機器人、物體、控制參數與輸出目錄。
2. **局部邊界觀測**：每台機器人只接收自身感測範圍內的邊界觀測。
3. **產生 cage target**：每個邊界點沿外法向偏移指定圍捕距離。

```text
q_target = b + d_cage * n_out
```

4. **建立邊界感知密度場**：cage target 形成高斯密度峰，引導局部 CVT。
5. **分配局部 CVT 目標**：機器人只使用自己與通訊範圍內鄰居的位置。
6. **安全過濾**：CBF-style filter 維持機器人間最小安全距離。

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **輸出資料和圖表**：仿真保存 CSV、截圖、軌跡圖、覆蓋率曲線、論文風格圖和可選 GIF。
8. **走向硬體驗證**：MAS adapter 將 DBACT 包裝成 `WorldState -> ControlCommand` 控制器。

---

## MAS 與硬體驗證

Mock MAS pipeline：

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

MAS dry-run：

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

OptiTrack 只讀檢查：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

安全建議：

- 先執行只讀 OptiTrack logging，再發布控制命令。
- 逐台確認 robot ID 與 rigid body 映射。
- 第一次物理運行請使用極低速度限制。
- 硬體測試期間保持物理急停可用。
- 每次運行後檢查 command、state 與事件日誌。

---

## 文件

| 文件 | 內容 |
| --- | --- |
| `docs/ARCHITECTURE.md` | Package layout 與資料流。 |
| `docs/ALGORITHM.md` | 邊界目標、局部 CVT、安全過濾與搬運模型。 |
| `docs/MAS_INTEGRATION.md` | MAS 集成指南。 |
| `docs/ROADMAP.md` | 開發路線圖。 |
| `docs/stage1_results.md` | Stage 1 圍捕結果。 |
| `docs/assets/README.md` | README 視覺素材清單與來源映射。 |

---

## 貢獻

歡迎提交 issue、復現報告、場景配置、可視化圖表、文件修訂與平台集成改進。

提交 PR 前建議運行：

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

---

## 授權

本專案採用 MIT License。詳見 [LICENSE](LICENSE)。
