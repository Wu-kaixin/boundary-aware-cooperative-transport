<div align="center">

# DBACT: 境界認識型協調搬送

**未知形状物体に対する分散型の境界認識マルチロボット囲い込み・搬送。**

<p>
  <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License: MIT">
  <img src="https://img.shields.io/badge/python-%3E%3D3.9-3776AB.svg" alt="Python >= 3.9">
  <img src="https://img.shields.io/badge/tests-pytest%20passing-brightgreen.svg" alt="pytest passing">
  <img src="https://img.shields.io/badge/status-research%20prototype-orange.svg" alt="Research prototype">
</p>

[English](README.en.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

</div>

---

## 概要

DBACT は、複数の移動ロボットによる分散協調搬送のための研究用ソフトウェアスタックです。対象物の完全な形状、中心、半径、事前のチーム割り当てを仮定しない状況を扱います。

ロボットは局所的な境界観測、近傍通信、境界認識密度、局所 CVT 目標割り当て、安全フィルタリングを用いて、未知の荷物の周囲に囲い込み構造を形成します。このリポジトリには、シミュレーション、MAS 互換コントローラアダプタ、OptiTrack 読み取り専用ツール、RoboMaster S1 コマンド smoke test も含まれます。

**主要技術**: Python 3.9+, NumPy, Matplotlib, PyYAML, pytest, local CVT, CBF-style safety filtering, MAS adapter, OptiTrack read-only bridge, RoboMaster S1 smoke tests。

---

## ビジュアルショーケース

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

<p align="center">
  <sub>移動する不規則形状の荷物に対して、ロボットが局所観測と局所協調により境界認識型の囲い込み構造を形成します。</sub>
</p>

| 密度場と局所 CVT | 軌跡 |
| --- | --- |
| ![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png) | ![DBACT trajectory](docs/assets/dbact-trajectory.png) |

| 被覆率カーブ | 最終スナップショット |
| --- | --- |
| ![DBACT coverage curve](docs/assets/dbact-coverage-curve.png) | ![DBACT final snapshot](docs/assets/dbact-final-snapshot.png) |

---

## 特長

- **未知形状物体に対応**: コントローラは荷物の中心、半径、頂点、最近傍境界クエリを直接使用しません。
- **豊富なシミュレーション出力**: 軌跡、被覆率カーブ、最終スナップショット、論文風フレーム、CSV ログ、任意の GIF を生成できます。
- **境界認識型の協調**: 局所観測を cage target に変換し、ロボットを有用な境界位置へ導く密度場を構築します。
- **局所 CVT 割り当て**: 各ロボットは自分自身と通信範囲内の近傍ロボットのみを使って目標を計算します。
- **実機検証への段階的経路**: MAS adapter、OptiTrack 読み取り専用 logging、RoboMaster S1 低速コマンドテストを備えています。

---

## 実験結果と可視化

### Stage 1: 未知多角形の囲い込み

Stage 1 は、コントローラ内部で完全な荷物形状を直接使わずに、局所境界観測、境界認識密度、局所 CVT、ロボット間安全フィルタリングだけで囲い込みを形成できることを検証します。

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

### 移動する不規則形状荷物の Demo

| 指標 | 値 |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

**結論**: tight caging 設定は境界被覆率を改善しつつ、報告された Stage 1 ベンチマークで最小ロボット間距離を 0.33 m 以上に保ちます。

---

## クイックスタート

### 1. クローン

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. 環境作成

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
```

標準の仮想環境を使う場合:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

Windows PowerShell:

```powershell
.\.venv\Scripts\Activate
```

### 3. インストール

```bash
pip install -r requirements.txt
pip install -e .[dev]
```

### 4. Demo を 1 コマンドで実行

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

出力例:

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

### 5. 標準シナリオを一括実行

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

### 6. 検証

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform tests:

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## リポジトリ構成

```text
boundary-aware-cooperative-transport/
|-- README.md                         # 言語選択とビジュアルプレビュー
|-- README.en.md                      # 英語 README
|-- README.zh-TW.md                   # 繁体字中国語 README
|-- README.ja.md                      # 日本語 README
|-- configs/
|   |-- sim/                          # シミュレーションシナリオ
|   `-- mas/                          # MAS コントローラ設定
|-- docs/
|   |-- assets/                       # GitHub で表示される README 画像と GIF
|   |-- ARCHITECTURE.md               # アーキテクチャ説明
|   |-- ALGORITHM.md                  # アルゴリズム説明
|   `-- stage1_results.md             # Stage 1 実験結果
|-- src/
|   |-- dbact/                        # コアアルゴリズム
|   |-- dbact_sim/                    # シミュレーション環境と可視化
|   `-- mas_adapter/                  # MAS adapter
|-- scripts/                          # 一括実行、MAS mock pipeline、S1 smoke tests
|-- tests/                            # 単体テストと smoke tests
|-- runs/                             # ローカル生成出力、Git では無視
`-- platforms/mas_public/             # Vendored MAS platform code
```

---

## 動作原理

1. **シナリオを読み込む**: `dbact_sim.run_sim` が YAML を読み、世界、ロボット、荷物、制御パラメータ、出力先を初期化します。
2. **局所境界を観測する**: 各ロボットはセンサ範囲内の境界観測だけを受け取ります。
3. **cage target を生成する**: 各境界点を外向き法線方向に指定距離だけずらします。

```text
q_target = b + d_cage * n_out
```

4. **境界認識密度場を作る**: cage target をガウス密度ピークにして局所 CVT を誘導します。
5. **局所 CVT 目標を割り当てる**: 各ロボットは自分自身と通信範囲内の近傍だけを使います。
6. **安全フィルタを適用する**: CBF-style filter により最小ロボット間距離を維持します。

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **データと図を出力する**: CSV、スナップショット、軌跡図、被覆率カーブ、論文風図、任意の GIF を保存します。
8. **実機検証へ進む**: MAS adapter は DBACT を `WorldState -> ControlCommand` コントローラとして包みます。

---

## MAS と実機検証

Mock MAS pipeline:

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

MAS dry-run:

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

OptiTrack read-only check:

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

安全上の注意:

- 制御コマンドを出す前に、OptiTrack の読み取り専用 logging を実行してください。
- robot ID と rigid body の対応を 1 台ずつ確認してください。
- 最初の実機走行では非常に低い速度制限を使ってください。
- 実機テスト中は物理的な緊急停止手段を用意してください。
- 各 run の後に command、state、event log を確認してください。

---

## ドキュメント

| ファイル | 内容 |
| --- | --- |
| `docs/ARCHITECTURE.md` | パッケージ構成とデータフロー。 |
| `docs/ALGORITHM.md` | 境界目標、局所 CVT、安全フィルタ、搬送モデル。 |
| `docs/MAS_INTEGRATION.md` | MAS integration guide。 |
| `docs/ROADMAP.md` | 開発ロードマップ。 |
| `docs/stage1_results.md` | Stage 1 caging results。 |
| `docs/assets/README.md` | README 用アセット一覧と出典。 |

---

## コントリビューション

Issue、再現レポート、シナリオ設定、可視化、ドキュメント修正、プラットフォーム統合の改善を歓迎します。

Pull request の前に、次を実行してください。

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

---

## ライセンス

このプロジェクトは MIT License で公開されています。詳細は [LICENSE](LICENSE) を参照してください。
