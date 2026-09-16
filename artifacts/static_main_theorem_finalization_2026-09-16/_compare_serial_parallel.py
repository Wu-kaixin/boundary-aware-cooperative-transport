import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def load_metrics(case_dir: Path) -> dict[str, list[float]]:
    path = case_dir / "metrics.csv"
    cols = {"J_k": [], "E_k": [], "H_star": [], "dissipation_slack_u_command": [], "in_K0": []}
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            cols["J_k"].append(float(row["J_k"]))
            cols["E_k"].append(float(row["E_k"]))
            cols["H_star"].append(float(row["H_star"]))
            cols["dissipation_slack_u_command"].append(float(row["dissipation_slack_u_command"]))
            cols["in_K0"].append(int(row["in_K0"]))
    return cols


def max_abs(a: list[float], b: list[float]) -> float:
    n = min(len(a), len(b))
    return max(abs(a[i] - b[i]) for i in range(n)) if n else float("nan")


def compare(a: Path, b: Path) -> dict:
    ma, mb = load_metrics(a), load_metrics(b)
    return {
        "n_a": len(ma["J_k"]),
        "n_b": len(mb["J_k"]),
        "max_abs_J": max_abs(ma["J_k"], mb["J_k"]),
        "max_abs_E": max_abs(ma["E_k"], mb["E_k"]),
        "max_abs_H": max_abs(ma["H_star"], mb["H_star"]),
        "max_abs_slack": max_abs(ma["dissipation_slack_u_command"], mb["dissipation_slack_u_command"]),
        "K0_a": all(ma["in_K0"]),
        "K0_b": all(mb["in_K0"]),
    }


serial = ROOT / "jeh_serial_pair" / "runs"
par = ROOT / "jeh_parallel_pair" / "runs"
full = ROOT / "jeh_matrix" / "runs"

report = {
    "tol_J": 1e-12,
    "tol_E": 1e-12,
    "tol_H": 1e-12,
    "reason": (
        "Same seed, same frozen controller, Windows spawn ProcessPool vs in-process serial. "
        "Real mass-weighted J_k, E_k and observer H_star. 80-frame representative hold; "
        "the 600-frame matrix is compared on the overlapping prefix."
    ),
    "pairs": {},
}
all_ok = True
for shape in ("l_shape", "rectangle"):
    sdir = serial / shape / "n20" / "seed_2"
    pdir = par / shape / "n20" / "seed_2"
    fdir = full / shape / "n20" / "seed_2"
    sp = compare(sdir, pdir)
    sf = compare(sdir, fdir)
    report["pairs"][shape] = {"serial_vs_spawn_pool": sp, "serial_80_vs_matrix_prefix": sf}
    for name, block in (("spawn", sp), ("matrix_prefix", sf)):
        ok = (
            block["max_abs_J"] <= 1e-12
            and block["max_abs_E"] <= 1e-12
            and block["max_abs_H"] <= 1e-12
            and block["K0_a"]
            and block["K0_b"]
        )
        block["pass"] = ok
        all_ok = all_ok and ok

report["pass"] = all_ok
(ROOT / "serial_parallel_consistency.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
