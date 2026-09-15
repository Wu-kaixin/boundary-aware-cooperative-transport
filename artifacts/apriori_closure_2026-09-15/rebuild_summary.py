import csv
import json
from pathlib import Path

root = Path("artifacts/apriori_closure_2026-09-15")
rows = []
for p in sorted(root.joinpath("runs").rglob("summary.json")):
    s = json.loads(p.read_text(encoding="utf-8"))
    parts = p.parts
    s["shape"] = parts[parts.index("runs") + 1]
    s["seed"] = int(parts[-2].split("_")[1])
    rows.append(s)

keys = [
    "shape",
    "seed",
    "grid_resolution",
    "complete_30s",
    "theorem_conditions_hold_full_horizon",
    "full_horizon_J_bound_status",
    "condition_failure_frames",
    "zero_input_with_rho_failure_count",
    "J_bar",
    "E_bar",
    "gradient_residual_bar",
    "centroid2_bar",
    "B_E_rigorous",
    "B_J_prior_certificate",
    "B_J_prior_P0H_rigorousE",
    "B_J_geom",
    "prior_beats_geom_certificate",
    "wall_seconds",
]
with (root / "summary.csv").open("w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow({k: r.get(k) for k in keys})

print("n=", len(rows))
for r in rows:
    print(
        f"{r['shape']:10} seed{r['seed']} K0={r.get('theorem_conditions_hold_full_horizon')} "
        f"beat={r.get('prior_beats_geom_certificate')} "
        f"prior={float(r.get('B_J_prior_P0H_rigorousE')):.4f} "
        f"geom={float(r.get('B_J_geom')):.4f} zr={r.get('zero_input_with_rho_failure_count')}"
    )
