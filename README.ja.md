<div align="center">

# DBACT: Decentralized Boundary-Aware Cooperative Transport

再現可能な分散型マルチロボット囲い込み・搬送実験。指標、レポート、MAS dry-run、可視化を含みます。

[English](README.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)
![Tests](https://img.shields.io/badge/Tests-16%20passed-brightgreen.svg)
![Version](https://img.shields.io/badge/Version-0.1.0-informational.svg)
![Visualization](https://img.shields.io/badge/Visualization-Matplotlib-orange.svg)
![Platform](https://img.shields.io/badge/Platform-MAS%20%7C%20RoboMaster%20S1-lightgrey.svg)

</div>

DBACT は **Decentralized Boundary-Aware Cooperative Transport** の研究プロトタイプです。物体の完全な形状、中心、半径、必要なチーム規模がコントローラに与えられていない場合に、複数の移動ロボットが有用な囲い込み・搬送構造を形成できるかを調べます。

このリポジトリは、独立したシミュレーションスタック、境界認識型の局所制御、指標、GitHub で表示可能な可視化素材、MAS-compatible controller adapter、OptiTrack read-only tooling、保守的な RoboMaster S1 command smoke tests を統合しています。

> このリポジトリは研究プロトタイプであり、完成済みの実機搬送製品ではありません。シミュレーションと dry-run は動作していますが、完全な実機搬送は段階的な検証目標です。

---

## ビジュアルショーケース

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

> シミュレーション replay から生成され、リポジトリに追跡されている GIF です。ローカルの `runs/*.gif` とは異なり、`docs/assets/` にコミットされているため GitHub で直接表示されます。

![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png)

> 未知 cargo、局所 CVT / Voronoi 構造、ロボット安全領域、境界認識密度曲面を示す論文風フレームです。

---

## メディアギャラリー

以下のインラインメディアはすべて `docs/assets/` 下のコミット済みファイルを参照しているため、ローカル run artifact がなくても GitHub で表示できます。

| Animation | Density + Local CVT |
| --- | --- |
| <img src="docs/assets/dbact-moving-cargo.gif" alt="DBACT moving cargo animation" width="100%"> | <img src="docs/assets/dbact-density-cvt-frame.png" alt="DBACT density and local CVT frame" width="100%"> |

| Trajectory | Coverage Curve |
| --- | --- |
| <img src="docs/assets/dbact-trajectory.png" alt="DBACT trajectory" width="100%"> | <img src="docs/assets/dbact-coverage-curve.png" alt="DBACT coverage curve" width="100%"> |

| Final Snapshot | Asset Manifest |
| --- | --- |
| <img src="docs/assets/dbact-final-snapshot.png" alt="DBACT final snapshot" width="100%"> | [`docs/assets/README.md`](docs/assets/README.md) |

生成された PNG、GIF、CSV、MP4 は通常 `runs/` または `platforms/mas_public/data/` に保存され、Git では意図的に無視されます。GitHub 表示用には、選定した図を `docs/assets/` にコピーするか、大きな動画を GitHub Releases で公開してください。

---

## プロジェクト概要

| 項目 | 詳細 |
| --- | --- |
| Project name | DBACT: Decentralized Boundary-Aware Cooperative Transport |
| Purpose | 未知形状物体に対する局所境界認識型の囲い込み・搬送を検証する。 |
| Core stack | Python 3.9+, NumPy, PyYAML, Matplotlib, pytest |
| Main scenarios | `paper_like_irregular_moving_cargo.yaml`, `l_shape.yaml`, `nonconvex.yaml`, `multi_object.yaml` |
| Output types | CSV trajectories, coverage metrics, JSON summaries, PNG figures, GIF animations |
| Integration path | DBACT simulation -> MAS dry-run -> OptiTrack read-only -> RoboMaster S1 smoke test |
| Current status | シミュレーションと dry-run は動作中。完全な実機実験は未完了。 |

---

## 特徴

- **分散型境界認識制御**：ロボットは全体物体形状ではなく、局所境界観測と近傍状態を使います。
- **未知物体の囲い込み**：コントローラは `cargo.center`、`cargo.radius`、`cargo.vertices`、closest-boundary query を直接使いません。
- **局所 CVT 割り当て**：各ロボットは自分自身と近傍ロボットから局所 weighted centroid を計算します。
- **CBF-style safety filtering**：ロボット間距離制約と速度制限により、囲い込み過程を保守的にします。
- **可視化優先ワークフロー**：軌跡、被覆率カーブ、最終スナップショット、論文風フレーム、任意の GIF を出力します。
- **ハードウェア検証への段階的経路**：MAS adapter、OptiTrack read-only logging、RoboMaster S1 smoke tests を含みます。

---

## 結果と可視化

### Stage 1 未知多角形囲い込み

Stage 1 は、コントローラが完全な cargo 幾何へ直接アクセスしなくても、任意多角形 cargo の囲い込みを形成できることを検証します。

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

### Tight Baseline 結果

Tight baseline は cage offset を小さくし、density field を狭めることで囲い込みのコンパクトさを改善します。

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

### Moving Irregular Cargo Demo

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

**解釈**

- Tight caging は Stage 1 benchmark で境界被覆率を改善し、最小ロボット間距離を 0.33 m 以上に保ちます。
- Moving-cargo demo は囲い込みと搬送に似た変位を示しますが、物理接触ダイナミクスはまだ簡略化されています。
- 現在の結果はシミュレーションと MAS dry-run の証拠であり、完全な実世界搬送の主張ではありません。

---

## クイックスタート

### 1. クローン

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. 環境作成

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

### 3. One-line Smoke Experiment

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

重要な出力：

- `runs/paper_like_irregular_moving_cargo/animation.gif`
- `runs/paper_like_irregular_moving_cargo/trajectory.png`
- `runs/paper_like_irregular_moving_cargo/final_snapshot.png`
- `runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png`
- `runs/paper_like_irregular_moving_cargo/metrics.json`
- `runs/paper_like_irregular_moving_cargo/figures/FIG_520.png`

---

## 仕組み

1. **シナリオ設定を読み込む**
   YAML は領域サイズ、cargo 幾何、ロボット初期状態、センサ範囲、通信範囲、搬送方向、制御パラメータを定義します。

2. **局所境界観測を生成する**
   シミュレータは cargo 幾何から局所境界観測を生成しますが、コントローラは完全な cargo 形状を直接消費しません。

3. **cage target を作る**
   観測された各境界点 `b` を外向きにずらします。

```text
q_target = b + d_cage * n_out
```

4. **境界認識密度を構築する**
   cage target は Gaussian density peak となり、有用な境界近傍位置へロボットを引き寄せます。

5. **局所 CVT 割り当てを実行する**
   各ロボットは自分自身と通信範囲内の近傍ロボットから局所 weighted centroid を計算します。

6. **安全フィルタを適用する**
   CBF-style filter はロボット間距離を設定下限より大きく保ちます。

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **Replay、指標、図を保存する**
   simulation run は CSV log、metrics、final snapshot、trajectory plot、coverage curve、paper-style frame、任意の animation を出力します。

---

## リポジトリ構成

```text
boundary-aware-cooperative-transport/
|-- configs/                         # Simulation and MAS configuration
|   |-- sim/
|   `-- mas/
|-- src/
|   |-- dbact/                       # Core controller, sensing, density, CVT, safety, metrics
|   |-- dbact_sim/                   # Simulation environment, scenarios, visualization, CLI
|   `-- mas_adapter/                 # MAS-compatible controller adapter
|-- scripts/                         # Batch runs, mock MAS pipeline, RoboMaster S1 smoke tests
|-- docs/                            # Architecture, algorithm notes, reports, staged validation
|   |-- assets/                      # Tracked GitHub-renderable README media
|   |-- ARCHITECTURE.md
|   |-- ALGORITHM.md
|   |-- MAS_INTEGRATION.md
|   `-- stage1_results.md
|-- platforms/mas_public/            # Vendored MAS platform code
|-- runs/                            # Local generated runs, ignored by Git
|-- tests/                           # Unit and smoke tests
|-- README.md
|-- README.en.md
|-- README.zh-TW.md
`-- README.ja.md
```

---

## 便利なコマンド

標準シナリオを実行：

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

L-shape scenario を実行：

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

mock MAS pipeline を実行：

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

MAS dry-run を実行：

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

OptiTrack read-only check を実行：

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

RoboMaster S1 mock command smoke test を実行：

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

テストを実行：

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## 現在の研究方向

次に有用なのは広い機能追加ではなく、段階的な検証です。優先項目：

- `docs/assets/` を安定した GitHub media surface として維持する；
- side-by-side scenario comparison figures を追加する；
- moving-cargo transport metrics と dashboard summary を改善する；
- virtual-object assumption を実際の boundary-observation pipeline に置き換える；
- robot pose が安定するまで Motive rigid body を検証する；
- read-only logging と dry-run 通過後のみ、低速 caging-only 実機実験を行う。

---

## 安全メモ

- コントローラ出力を有効化する前に read-only OptiTrack logging を行う。
- robot ID と rigid-body mapping を 1 台ずつ確認する。
- 最初の実機 run では非常に低い速度制限を使う。
- ハードウェアテスト中は物理 emergency stop を用意する。
- 各 run 後に command と state log を確認する。

---

## コントリビューションとライセンス

Issues と Pull Requests を歓迎します。新しいシナリオ、より明確な可視化、強い指標、より良い段階的ハードウェア検証は特に有用です。

このプロジェクトは [MIT License](LICENSE) で公開されています。
