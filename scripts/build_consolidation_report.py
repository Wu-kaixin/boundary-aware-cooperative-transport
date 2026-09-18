"""Publish compact evidence from fresh runs; never infer success from exit code.

Run after the consolidation gates. Raw trajectories remain in ignored runs/.
"""
from __future__ import annotations

import hashlib
import json
import platform
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/results/consolidation"
ASSETS = ROOT / "docs/assets"


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()



def compare_trajectories(serial, parallel, frames=600):
    expected = {"J": frames, "E": frames, "H_star": frames + 1, "positions": frames + 1}
    delta = {}
    for key, length in expected.items():
        a, b = np.asarray(serial[key]), np.asarray(parallel[key])
        if a.shape != b.shape or a.ndim == 0 or len(a) != length:
            return {"pass": False, "reason": f"{key}: incompatible or incomplete horizon"}
        if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
            return {"pass": False, "reason": f"{key}: nonfinite measurements"}
        delta[key] = float(np.max(np.abs(a - b)))
    if not np.array_equal(serial["vertices"], parallel["vertices"]):
        return {"pass": False, "reason": "different object geometry"}
    return {"max_absolute_difference": delta, "frames": frames,
            "pass": all(v <= 1e-12 for v in delta.values())}


def serial_verdict(checks):
    if set(checks) != {"l_shape", "rectangle"}:
        return "INCOMPLETE"
    return "PASS" if all(v["pass"] for v in checks.values()) else "FAIL"


def d10_block(diagnosis, ab):
    total = sum(diagnosis["aggregate"]["segments_total"].values())
    converged = diagnosis["aggregate"]["segments_total"]["F"]
    lines = ["", "**D10：当前重跑结果**", "",
             f"发现至接触就绪共 {total} 帧，其中 ENCLOSURE_CONVERGENCE 为 {converged} 帧。",
             "", "![当前 D10 阶段分布](docs/assets/d10-post-detection-stages.png)",
             "![当前 D10 覆盖率](docs/assets/d10-coverage-and-gap.png)", "",
             "| 探索增益 | 种子数 | 接触就绪帧（均值；观测 n） | 缩放障壁事件总数 |",
             "| --- | ---: | ---: | ---: |"]
    for gain, rows in ab["arms"].items():
        times = [r["T_contact_ready"] for r in rows if r["T_contact_ready"] is not None]
        mean = f"{np.mean(times):.6g}; n={len(times)}" if times else "未到达; n=0"
        lines.append(f"| {gain} | {len(rows)} | {mean} | {sum(r['barrier_scalings'] for r in rows)} |")
    lines += ["", "![当前 D10 门槛对照](docs/assets/d10-gate-tradeoff.png)",
              "历史机制探索与旧数值见 [历史说明](docs/HISTORICAL_CLOSED_LOOP_README.md)，不用于解释本次图表。"]
    return lines

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = {"report_runtime": {"python": platform.python_version(), "numpy": np.__version__,
                                   "scipy": scipy.__version__}, "files": {}, "figures": {},
                "scope": "oracle theorem validation from this branch; Gate 5/6 tables only if paired runs exist; no local-map safety theorem"}

    def remember(path):
        key = path.relative_to(ROOT).as_posix()
        manifest["files"][key] = digest(path)
        return key

    def copy(source, target, inputs=()):
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        remember(source)
        remember(target)
        if inputs:
            manifest["figures"][target.relative_to(ROOT).as_posix()] = {
                "source": source.relative_to(ROOT).as_posix(),
                "inputs": {remember(p): digest(p) for p in inputs},
            }

    lines = ["由已记录运行的 JSON/CSV 生成；所有种子与失败均保留。报告重建不代表重新执行仿真。", "",
             "| 运行 | 帧数 | 终止 | G500 | 失败原因 |",
             "| --- | ---: | --- | --- | --- |"]
    near = []
    for name, seed, asset in [("near", 2, "closed_loop_d_seed2.gif"),
                               ("near", 4, "closed_loop_d_seed4.gif"),
                               ("near", 8, "closed_loop_d_seed8.gif"),
                               ("search", 7, "search_d_seed7.gif")]:
        folder = ROOT / "runs/readme" / (f"near_seed_{seed}" if name == "near" else "search_seed7")
        summary = read(folder / "summary.json")
        cargo = next(iter(summary["cargoes"].values()))
        gate = cargo["g500"]
        lines.append(f"| {name} seed {seed} | {summary['steps']} | {summary['timing']['terminated_by']} | "
                     f"{'PASS' if gate['success'] else 'FAIL'} | {'; '.join(gate['failure_reasons']) or '—'} |")
        near.append({"run": name, "seed": seed, "summary": summary})
        copy(folder / "summary.json", OUT / f"{name}_seed{seed}.json")
        copy(folder / "closed_loop.gif", ASSETS / asset,
             [folder / "replay.npz", folder / "summary.json"])
    manifest["closed_loop_commits"] = sorted({r["summary"]["provenance"]["git_sha"] for r in near})

    sweep_path = ROOT / "runs/readme/d_sweep/g500_sweep.json"
    sweep = read(sweep_path)
    copy(sweep_path, OUT / "g500_sweep.json")
    lines += ["", f"近场扫描种子 `{sweep['seeds']}`：**{sweep['g500_pass']}/{sweep['g500_total']} 通过 G500**。",
              "", "| 指标 | 均值 ± 总体标准差 | 最小–最大 |", "| --- | ---: | ---: |"]
    keep = ("J", "efficiency", "direction_error_deg", "max_cross_track", "max_strict_coverage",
            "min_inter_agent_distance", "min_signed_clearance", "contact_ready_frame", "hold_frame")
    for key in keep:
        stat = sweep["distributions"].get(key)
        if stat:
            lines.append(f"| {key} | {stat['mean']:.6g} ± {stat['sd']:.6g} | {stat['min']:.6g}–{stat['max']:.6g} |")
    for folder, file in [("d10_diag", "diagnosis.json"), ("d10_ab", "ab.json"), ("d10_enc", "gate.json")]:
        copy(ROOT / "runs/readme" / folder / file, OUT / f"{folder}.json")
    for folder, file, asset, data in [
        ("d10_diag", "figA_segments.png", "d10-post-detection-stages.png", "diagnosis.json"),
        ("d10_diag", "figB_coverage.png", "d10-coverage-and-gap.png", "diagnosis.json"),
        ("d10_enc", "figF_gate_tradeoff.png", "d10-gate-tradeoff.png", "gate.json"),
    ]:
        source = ROOT / "runs/readme" / folder
        copy(source / file, ASSETS / asset, [source / data])

    lines += d10_block(read(OUT / "d10_diag.json"), read(OUT / "d10_ab.json"))

    matrix = ROOT / "runs/paper/jeh_matrix"
    summary = read(matrix / "summary.json")
    copy(matrix / "summary.json", OUT / "jeh_summary.json")
    copy(matrix / "summary.csv", OUT / "jeh_summary.csv")
    copy(matrix / "prior_screen.json", OUT / "prior_screen.json")
    manifest["static_commits"] = sorted({r["code_sha"] for r in summary["cases"]})
    safety = []
    for row in summary["cases"]:
        path = matrix / "runs" / row["shape"] / f"n{row['grid_resolution']}" / f"seed_{row['seed']}" / "step_records.json"
        records = read(path)
        remember(path)
        safety.append({"shape": row["shape"], "seed": row["seed"],
                       "min_true_hold_clearance_lower_bound": min(r["hold_min_object_clearance"] for r in records),
                       "min_robot_distance": min(r["hold_min_pair"] for r in records),
                       "min_object_safety_margin": min(r["hold_object_clearance_margin"] for r in records),
                       "solver_status_counts": row["solver_status_counts"],
                       "K0_entire_horizon": row["theorem_conditions_hold_full_horizon"]})
    (OUT / "static_safety.json").write_text(json.dumps(safety, indent=2), encoding="utf-8")
    lines += ["", "**静态 oracle-map 理论验证**（3 种形状 × 9 个种子 × 600 帧）。",
              "下表是已声明的严格先验量；图中 J/E/H 为数值观测器估计。",
              "", "| 形状 | B_E | B_J_prior | B_J_geom | 完成 | 全程 K0 |",
              "| --- | ---: | ---: | ---: | ---: | ---: |"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    checks = {}
    for col, shape in enumerate(("l_shape", "rectangle", "c_shape")):
        rows = [r for r in summary["cases"] if r["shape"] == shape]
        first = rows[0]
        lines.append(f"| {shape} | {first['B_E_rigorous']:.10g} | {first['B_J_prior_certificate']:.10g} | "
                     f"{first['B_J_geom']:.10g} | {sum(r['complete_30s'] for r in rows)}/{len(rows)} | "
                     f"{sum(r['theorem_conditions_hold_full_horizon'] for r in rows)}/{len(rows)} |")
        case = matrix / "runs" / shape / "n20/seed_2"
        remember(case / "trajectory.npz")
        data = np.load(case / "trajectory.npz", allow_pickle=False)
        p = data["positions"]
        vertices = data["vertices"]
        ax = axes[0, col]
        ax.fill(vertices[:, 0], vertices[:, 1], color="0.75")
        for i in range(p.shape[1]):
            ax.plot(p[:, i, 0], p[:, i, 1], lw=0.7, alpha=0.65)
        ax.scatter(p[0, :, 0], p[0, :, 1], facecolors="none", edgecolors="C0", s=22, label="initial")
        ax.scatter(p[-1, :, 0], p[-1, :, 1], c="C3", s=18, label="final")
        ax.set(title=f"{shape}, seed 2", xlabel="x (m)", ylabel="y (m)", aspect="equal")
        ax.legend(fontsize=7)
        ax = axes[1, col]
        for key in ("J", "E", "H_star"):
            ax.plot(np.arange(len(data[key])) * 0.05, data[key], label=key)
        ax.set(xlabel="Time (s)", ylabel="Numerical observer metric")
        ax.legend(fontsize=7)
        if shape != "c_shape":
            serial = ROOT / "runs/paper/jeh_serial_pair/runs" / shape / "n20/seed_2/trajectory.npz"
            if serial.exists():
                other = np.load(serial, allow_pickle=False)
                remember(serial)
                checks[shape] = compare_trajectories(other, data)
    fig.suptitle("Static oracle-map theory validation — numerical observer trajectories")
    fig.tight_layout()
    figure = ASSETS / "static-deployment-comparison.png"
    fig.savefig(figure, dpi=160)
    plt.close(fig)
    remember(figure)
    manifest["figures"][figure.relative_to(ROOT).as_posix()] = {
        "inputs": {p: h for p, h in manifest["files"].items() if p.endswith("trajectory.npz")},
        "seed": 2, "label": "static oracle-map theory validation"}
    (OUT / "serial_parallel_consistency.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
    verdict = serial_verdict(checks)
    serial_line = (f"串行/并行完整 600 帧对照（L 形与矩形，seed 2，容差 1e-12）：**{verdict}**。")
    lines += ["", serial_line, "",
              "![静态部署对照](docs/assets/static-deployment-comparison.png)", "",
              "完整 QP 状态、K0 与最小净空：[静态安全](docs/results/consolidation/static_safety.json)。"]

    for gate, folder in [(5, "local_boundary_pairs"), (6, "deployment_transport_pairs")]:
        source = ROOT / "runs/paper" / folder
        if not (source / "summary.json").exists():
            title = "局部边界接口" if gate == 5 else "部署对搬运的作用"
            lines += ["", f"**Gate {gate}：{title}**",
                      "该门的成对实验仍在执行，暂不宣称成功率。"]
            continue
        pair_summary = read(source / "summary.json")
        copy(source / "summary.json", OUT / f"gate{gate}_pairs.json")
        copy(source / "manifest.json", OUT / f"gate{gate}_manifest.json")
        manifest[f"gate{gate}_commit"] = pair_summary["manifest"]["commit"]
        all_rows = [r for pair in pair_summary["pairs"] for r in pair]
        lines += ["", f"**Gate {gate}：{'局部边界接口' if gate == 5 else '部署对搬运的作用'}**",
                  "成对形状：L、矩形、C；种子 2、5、8。每一对共享同一初值。"]
        if gate == 5:
            lines += ["仅经验测量。真值几何只给外部评估器，不进入局部控制，也不做真值净空否决。",
                      "", "| 形状 | 地图 | 完成 | 终态 H（均值） | 终态质心残差²（均值） | 最小安全裕量 | 终态地图覆盖（均值） | QP 干预（均值） |",
                      "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
            for shape in ("l_shape", "rectangle", "c_shape"):
                for variant in ("oracle", "local"):
                    rs = [r for r in all_rows if r["shape"] == shape and r["variant"] == variant]
                    mean = lambda key: float(np.mean([r[key] for r in rs if r[key] is not None]))
                    lines.append(f"| {shape} | {variant} | {sum(r['frames_completed'] == r['frames_requested'] and r['abort'] is None for r in rs)}/{len(rs)} | "
                                 f"{mean('H_final'):.5g} | {mean('centroid_residual_squared_final'):.5g} | "
                                 f"{min(r['minimum_safety_margin'] for r in rs if r['minimum_safety_margin'] is not None):.5g} | "
                                 f"{mean('final_union_map_coverage'):.4f} | {mean('qp_intervention_fraction'):.4f} |")
            unsafe = sum(r["object_safety_violation_frames"] > 0 for r in all_rows if r["variant"] == "local")
            lines += ["", f"局部运行中观测到物体安全违反的次数：**{unsafe}/9**。"
                      "对照完成并不构成未知边界安全定理。"]
        else:
            lines += ["仅替换 CONTACT_READY 之前的部署目标；感知、安全层与后续搬运保持相同。",
                      "", "| 形状 | 部署 | G500 通过 | 收定 | 接触就绪秒（均值；观测 n） | 接触数（均值） | 终态距离误差（均值） | QP 干预（均值） |",
                      "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
            for shape in ("l_shape", "rectangle", "c_shape"):
                for variant in ("boundary_cvt", "nearest_observed_target"):
                    rs = [r for r in all_rows if r["shape"] == shape and r["variant"] == variant]
                    times = [r['contact_ready_seconds'] for r in rs if r['contact_ready_seconds'] is not None]
                    times_text = f"{np.mean(times):.4g}; n={len(times)}" if times else "未到达; n=0"
                    lines.append(f"| {shape} | {variant} | {sum(r['g500_success'] for r in rs)}/{len(rs)} | "
                                 f"{sum(r['termination']['settled'] for r in rs)}/{len(rs)} | {times_text} | "
                                 f"{np.mean([r['mean_contacts'] for r in rs]):.4g} | {np.mean([r['final_distance_error'] for r in rs]):.4g} | "
                                 f"{np.mean([r['qp_intervention_fraction'] for r in rs]):.4g} |")
            lines += ["", "完整成对 JSON 含接触方位分布、观测力/力矩、法向瞬时扳手容量、"
                      "方向误差、横向偏移与安全最小值。"
                      "该容量是瞬时接触模型界，不是动力学保证。"
                      "这些小样本对照不构成普遍的搬运优越性。"]
        lines += ["", f"[Gate {gate} 逐种子结果与定义](docs/results/consolidation/gate{gate}_pairs.json)。"]
    lines += ["", "[实验清单与源哈希](docs/results/consolidation/manifest.json)。",
              "[精简结果目录](docs/results/consolidation/)。"]
    queue = ROOT / "runs/consolidation/readme_manifest.json"
    if queue.exists():
        manifest["execution"] = read(queue)
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    block = "\n".join(lines)
    (OUT / "results.md").write_text(block + "\n", encoding="utf-8")
    path = ROOT / "README.md"
    text = path.read_text(encoding="utf-8")
    start, end = "<!-- CONSOLIDATION_RESULTS_START -->", "<!-- CONSOLIDATION_RESULTS_END -->"
    before, rest = text.split(start, 1)
    _, after = rest.split(end, 1)
    path.write_text(before + start + "\n" + block + "\n" + end + after, encoding="utf-8")


if __name__ == "__main__":
    main()
