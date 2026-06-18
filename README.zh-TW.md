# DBACT：去中心化的邊界感知協同運輸系統

[English](README.en.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

## 專案背景

本倉庫是一套多機器人去中心化協同搬運的研究型軟體棧。

這個專案關注一個實際的機器人問題：

> 當控制器不知道物體完整形狀、中心、半徑，也不知道需要多少台機器人時，多台機器人能否形成有用的圍捕與搬運結構？

目前目標比完整實體搬運系統更窄，也更務實：

> 建立可復現的 DBACT 仿真系統，接入 MAS 風格控制器介面，產出可視化實驗證據，並準備通往 RoboMaster S1 + OptiTrack 驗證的安全路徑。

研究脈絡如下：

```text
Cooperative-Transport-Multi-Agent-System
        -> DBACT: Decentralized Boundary-Aware Cooperative Transportation
        -> MAS adapter / OptiTrack read-only / RoboMaster S1 smoke path
```

本倉庫應被視為仍在推進中的研究原型。仿真與 dry-run 路徑已可運行；完整實體搬運實驗仍是分階段驗證目標。

## 當前研究決策

目前專案方向是：

- 保持 `main` 作為主要維護分支。
- 將仿真與 MAS dry-run 作為主要驗證面。
- 暫不宣稱已完成完整實體搬運實驗。
- 在只讀 OptiTrack logging、mock pipeline 與低速命令 smoke test 通過前，不發送真實機器人運動命令。
- 將可視化作為一級輸出，因為本專案需要讓圍捕、覆蓋率、密度場與安全行為一眼可理解。

因此當前行動項是：

```text
維持清楚的 simulation-to-hardware 路徑：
local boundary simulation
  -> visual result generation
  -> MAS-compatible dry-run
  -> OptiTrack read-only validation
  -> low-speed RoboMaster S1 smoke testing
  -> future closed-loop physical transport
```

## 當前專案範圍

本倉庫目前聚焦於：

```text
unknown polygon caging
+ local boundary sensing
+ boundary-aware density
+ local CVT target allocation
+ local CBF-style safety filtering
+ simplified caging / pushing transport dynamics
+ simulation metrics and visualizations
+ MAS-compatible ControlCommand generation
+ OptiTrack read-only logging path
+ RoboMaster S1 command smoke path
```

當前已驗證階段暫不包含：

```text
完整實體搬運驗證
任意感測器的真實 cargo perception
力控制接觸動力學
每條路徑上的 paper-grade QP solver
大規模硬體部署
全自動 process-launcher 實驗
```

## 控制器與仿真模型

DBACT 將每台機器人建模為局部決策者。每個 agent 具有：

- 位置與速度；
- 感測範圍；
- 通訊範圍；
- 安全半徑或最小機器人間距；
- 局部邊界觀測；
- 局部鄰居狀態。

控制器刻意避免直接使用：

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
global robot assignment
global all-pairs QP
predefined team size
```

仿真器可以使用真實 cargo 幾何來產生局部感測觀測與離線指標，但控制器路徑保持為基於邊界觀測。

## DBACT 邊界感知流程

DBACT 的核心想法是：

> 讓機器人移向有用的邊界外側 cage target，而不是移向已知物體中心。

目前 DBACT 流程：

1. 產生局部邊界觀測；
2. 估計邊界外法向；
3. 在物體外側建立 cage target；
4. 建立邊界感知高斯密度場；
5. 使用附近機器人計算局部 weighted CVT centroid；
6. 套用局部 CBF-style 安全過濾；
7. 輸出機器人速度命令；
8. 匯出軌跡、覆蓋率曲線、圖表與可選動畫。

cage target 規則是：

```text
q_target = b + d_cage * n_out
```

機器人間安全約束是：

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

主要實作：

```text
src/dbact/controller.py
src/dbact/local_sensing.py
src/dbact/boundary_density.py
src/dbact/local_cvt.py
src/dbact/local_cbf_qp.py
src/dbact/transport_dynamics.py
src/dbact_sim/run_sim.py
src/mas_adapter/decentralized_transport_controller.py
```

## 可視化優先級

本專案的價值高度依賴可視化。

視覺輸出應該讓仿真結果一眼可理解：

- 機器人軌跡；
- 未知 cargo 形狀；
- cage target 區域；
- 局部 CVT / Voronoi 結構；
- 邊界感知密度曲面；
- 邊界覆蓋率曲線；
- 最小距離與安全行為；
- MAS dry-run 軌跡證據。

精選 README 素材存放在 `docs/assets/`，因此可以在 GitHub 正常顯示。

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

| 密度場 + 局部 CVT | 軌跡 |
| --- | --- |
| ![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png) | ![DBACT trajectory](docs/assets/dbact-trajectory.png) |

| 覆蓋率曲線 | 最終截圖 |
| --- | --- |
| ![DBACT coverage curve](docs/assets/dbact-coverage-curve.png) | ![DBACT final snapshot](docs/assets/dbact-final-snapshot.png) |

若新的圖片、GIF 或影片需要在 GitHub 顯示，請從 `runs/` 複製到 `docs/assets/`，並在 README 中引用穩定路徑。`runs/` 下的生成檔案仍預設不提交 Git。

## 實驗結果

Stage 1 驗證了未知多邊形 cargo 圍捕，而且控制器內部不直接讀取完整 cargo 幾何。

原始 baseline 結果：

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

Tight baseline 結果：

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

移動不規則 cargo demo：

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

Tight caging 設定在 Stage 1 benchmark 中提升了邊界覆蓋率，同時將最小機器人間距保持在 0.33 m 以上。

## 倉庫結構

```text
boundary-aware-cooperative-transport/
|-- configs/
|   |-- sim/
|   `-- mas/
|-- docs/
|   |-- assets/
|   |-- ARCHITECTURE.md
|   |-- ALGORITHM.md
|   |-- MAS_INTEGRATION.md
|   |-- ROADMAP.md
|   `-- stage1_results.md
|-- src/
|   |-- dbact/
|   |-- dbact_sim/
|   `-- mas_adapter/
|-- scripts/
|   |-- run_all_scenarios.py
|   |-- run_mock_mas_pipeline.py
|   `-- run_seven_s1_cvt_test.py
|-- tests/
|-- platforms/
|   `-- mas_public/
|-- README.md
|-- README.en.md
|-- README.zh-TW.md
|-- README.ja.md
|-- requirements.txt
|-- pyproject.toml
`-- LICENSE
```

## Conda 環境設定

已知可用的本地環境是名為 `dbact` 的 Conda 環境。

建立並安裝：

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .[dev]
```

Vendored MAS platform：

```bash
conda activate dbact
cd platforms/mas_public
python -m pip install -r requirements.txt
```

## 運行仿真

移動不規則 cargo demo：

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

L-shape 場景：

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

批量運行：

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

## 輸出

每次仿真會保存：

- `trajectories.csv`;
- `agent_positions.csv`;
- `coverage_rates.csv`;
- 啟用 metrics 匯出時的 `metrics.json`;
- `final_snapshot.png`;
- `trajectory.png`;
- `coverage_rate_curve.png`;
- `figures/FIG_*.png`;
- 可選的 `animation.gif`。

生成輸出預設不提交 Git。精選 README 視覺素材放在 `docs/assets/`。

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

ControllerModule-level dry-run：

```bash
cd platforms/mas_public
python apps/dbact/run_controller_module_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/controller_module_dtransport
```

OptiTrack 只讀檢查：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

RoboMaster S1 mock 命令 smoke test：

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

安全規則：

- 啟用控制器輸出前，先執行只讀 OptiTrack logging。
- 逐台確認 robot ID 與 rigid-body 映射。
- 第一次實體運行使用極低速度限制。
- 硬體測試期間保持物理急停可用。
- 每次運行後檢查 command 與 state log。

## 報告與文件

現有文件：

```text
docs/ARCHITECTURE.md
docs/ALGORITHM.md
docs/MAS_INTEGRATION.md
docs/ROADMAP.md
docs/stage1_results.md
docs/stage2_mas_virtual_object.md
docs/stage3_mas_dry_run.md
docs/stage4_optitrack_readonly.md
docs/daily_health_2026-06-18.md
docs/assets/README.md
```

這些報告應視為分階段研究證據，而不是最終實體實驗宣稱。

## 測試

根層測試：

```bash
python -m pytest -q tests
```

編譯檢查：

```bash
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform 測試：

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

最新本地健康報告：

```text
docs/daily_health_2026-06-18.md
```

## 近期計畫

建議下一步：

1. 保持 `docs/assets/` 作為 GitHub 穩定可視化展示面。
2. 增加更多場景的 side-by-side 對比圖。
3. 改進 moving-cargo transport metrics 與 summary dashboard。
4. 用真實邊界觀測 pipeline 取代 virtual object 假設。
5. 驗證 Motive rigid body，直到機器人姿態穩定。
6. 在只讀與 dry-run 檢查後，執行低速 caging-only 實體實驗。

## 研究解讀

正向結果表示 DBACT-style 局部邊界感知與局部分配可以為未知形狀物體產生有用的圍捕行為。

較弱或負面的結果同樣有價值，因為它能指出下一步該改善 local sensing、density shaping、CVT target allocation、safety filtering 或 physical contact modeling。

因此，本倉庫的設計目標是在宣稱完整真實世界協同搬運之前，先建立可靠仿真證據與安全硬體驗證基礎。

## 授權

本專案採用 MIT License。詳見 [LICENSE](LICENSE)。
