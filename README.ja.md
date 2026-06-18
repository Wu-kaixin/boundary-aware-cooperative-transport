# DBACT: 境界認識型協調搬送

[English](README.en.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

## プロジェクト背景

このリポジトリは、複数移動ロボットによる分散協調搬送のための研究用ソフトウェアスタックです。

このプロジェクトは、次の実践的なロボティクス課題を扱います。

> 物体の完全な形状、中心、半径、必要なロボット数がコントローラに与えられていない場合でも、複数ロボットは有用な囲い込み・搬送構造を形成できるか？

現在の目的は、完全な実機搬送システムよりも狭く、より実験的です。

> 再現可能な DBACT シミュレーションスタックを構築し、MAS 形式のコントローラインターフェースへ接続し、視覚的な実験証拠を生成し、RoboMaster S1 + OptiTrack 検証へ向けた安全な道筋を整備する。

研究の流れは次の通りです。

```text
Cooperative-Transport-Multi-Agent-System
        -> DBACT: Decentralized Boundary-Aware Cooperative Transportation
        -> MAS adapter / OptiTrack read-only / RoboMaster S1 smoke path
```

このリポジトリは、進行中の研究プロトタイプとして読むべきものです。シミュレーションと dry-run は動作していますが、完全な実機搬送実験はまだ段階的な検証目標です。

## 現在の研究判断

現在の方針は次の通りです。

- `main` を保守対象ブランチとして維持する。
- シミュレーションと MAS dry-run を主要な検証面とする。
- 完了済みの実機搬送実験とは主張しない。
- 読み取り専用 OptiTrack logging、mock pipeline、低速 command smoke test が通る前に実ロボットへ運動命令を送らない。
- 囲い込み、被覆率、密度場、安全挙動を理解しやすくするため、可視化を第一級の出力として扱う。

したがって、現在の action item は次の通りです。

```text
明確な simulation-to-hardware 経路を維持する：
local boundary simulation
  -> visual result generation
  -> MAS-compatible dry-run
  -> OptiTrack read-only validation
  -> low-speed RoboMaster S1 smoke testing
  -> future closed-loop physical transport
```

## 現在のスコープ

このリポジトリは現在、次に集中しています。

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

現在の検証済み段階では範囲外のもの：

```text
完全な実機搬送検証
任意センサからの実 cargo perception
力制御を含む接触ダイナミクス
すべての経路での paper-grade QP solver
大規模ハードウェア展開
完全自動 process-launcher 実験
```

## コントローラとシミュレーションモデル

DBACT は各ロボットを局所的な意思決定者として扱います。各 agent は次を持ちます。

- 位置と速度；
- センサ範囲；
- 通信範囲；
- 安全半径または最小ロボット間距離；
- 局所境界観測；
- 局所近傍状態。

コントローラは意図的に次へ直接アクセスしません。

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
global robot assignment
global all-pairs QP
predefined team size
```

シミュレータは局所センサ観測とオフライン評価指標を生成するために真の cargo 幾何を使えますが、コントローラ経路は境界観測ベースに保たれます。

## DBACT 境界認識パイプライン

DBACT の中心的な考え方は次です。

> ロボットを既知の物体中心へ向けるのではなく、有用な境界近傍の cage target へ向ける。

現在の DBACT flow：

1. 局所境界観測を生成する；
2. 境界の外向き法線を推定する；
3. 物体外側に cage target を作る；
4. 境界認識型ガウス密度場を構築する；
5. 近傍ロボットを使って局所 weighted CVT centroid を計算する；
6. 局所 CBF-style safety filtering を適用する；
7. ロボット速度命令を出力する；
8. 軌跡、被覆率カーブ、図、任意のアニメーションを出力する。

cage target の規則：

```text
q_target = b + d_cage * n_out
```

ロボット間安全制約：

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

主な実装：

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

## 可視化の優先度

このプロジェクトの価値は可視化に強く依存します。

視覚出力は、シミュレーションを一目で理解できる必要があります。

- ロボット軌跡；
- 未知 cargo 形状；
- cage target 領域；
- 局所 CVT / Voronoi 構造；
- 境界認識密度曲面；
- 境界被覆率カーブ；
- 最小距離と安全挙動；
- MAS dry-run の軌跡証拠。

選定済み README assets は `docs/assets/` に保存されているため、GitHub 上で正しく表示されます。

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

| Density + Local CVT | Trajectory |
| --- | --- |
| ![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png) | ![DBACT trajectory](docs/assets/dbact-trajectory.png) |

| Coverage Curve | Final Snapshot |
| --- | --- |
| ![DBACT coverage curve](docs/assets/dbact-coverage-curve.png) | ![DBACT final snapshot](docs/assets/dbact-final-snapshot.png) |

新しい画像、GIF、動画を GitHub で表示したい場合は、`runs/` から `docs/assets/` へコピーし、README から安定したパスを参照してください。`runs/` の生成物は引き続き Git では無視されます。

## 実験結果

Stage 1 は、コントローラ内で完全な cargo 幾何へ直接アクセスせずに、未知多角形 cargo の囲い込みを検証します。

Original baseline results：

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

Tight baseline results：

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

Moving irregular cargo demo：

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

Tight caging 設定は、報告された Stage 1 benchmark において境界被覆率を改善し、最小ロボット間距離を 0.33 m 以上に保ちます。

## リポジトリ構成

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

## Conda セットアップ

既知の動作環境は `dbact` という名前の Conda 環境です。

作成とインストール：

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

## シミュレーション実行

Moving irregular cargo demo：

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

L-shape scenario：

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Batch run：

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

## 出力

各シミュレーション run は次を保存します。

- `trajectories.csv`;
- `agent_positions.csv`;
- `coverage_rates.csv`;
- metrics export が有効な場合の `metrics.json`;
- `final_snapshot.png`;
- `trajectory.png`;
- `coverage_rate_curve.png`;
- `figures/FIG_*.png`;
- 任意の `animation.gif`。

生成出力は Git では無視されます。選定済み README visuals は `docs/assets/` に追跡されています。

## MAS とハードウェア検証

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

OptiTrack read-only check：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

RoboMaster S1 mock command smoke test：

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

安全規則：

- コントローラ出力を有効にする前に、読み取り専用 OptiTrack logging を実行する。
- robot ID と rigid-body mapping を 1 台ずつ確認する。
- 最初の実機 run では非常に低い速度制限を使う。
- ハードウェアテスト中は物理的な emergency stop を利用可能にする。
- 各 run 後に command と state log を確認する。

## レポートとドキュメント

既存ドキュメント：

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

これらの report は段階的な研究証拠として読むべきであり、最終的な実機実験の主張ではありません。

## テスト

Root tests：

```bash
python -m pytest -q tests
```

Compile check：

```bash
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform tests：

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

最新のローカル health report：

```text
docs/daily_health_2026-06-18.md
```

## 近い計画

推奨される次の実装目標：

1. `docs/assets/` を GitHub 用の安定した可視化表示面として維持する。
2. より多くのシナリオで side-by-side comparison figures を追加する。
3. moving-cargo transport metrics と summary dashboards を改善する。
4. virtual object assumption を実際の boundary-observation pipeline に置き換える。
5. Motive rigid bodies を検証し、ロボット姿勢が安定するまで確認する。
6. read-only と dry-run checks の後に低速 caging-only 実機実験を行う。

## 研究解釈

良い結果は、DBACT-style の局所境界認識と局所割り当てが、未知形状物体に対して有用な囲い込み挙動を生成できることを意味します。

弱い、または否定的な結果も有用です。local sensing、density shaping、CVT target allocation、safety filtering、physical contact modeling のどこを改善すべきかを示すからです。

したがって、このリポジトリは、完全な実世界協調搬送を主張する前に、信頼できるシミュレーション証拠と安全なハードウェア検証基盤を作るために設計されています。

## ライセンス

このプロジェクトは MIT License で公開されています。詳細は [LICENSE](LICENSE) を参照してください。
