"""Regenerate closure figures from summary.csv."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src"))

from run_apriori_centroid_bound import make_figures  # noqa: E402


def main() -> None:
    out = ROOT / "artifacts" / "apriori_closure_2026-09-15"
    rows = []
    with (out / "summary.csv").open(encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            r = dict(raw)
            for k, v in list(r.items()):
                if v == "":
                    r[k] = None
                    continue
                if v in ("True", "False"):
                    r[k] = v == "True"
                    continue
                try:
                    r[k] = int(v)
                except ValueError:
                    try:
                        r[k] = float(v)
                    except ValueError:
                        pass
            shape = str(r["shape"])
            seed = int(r["seed"])
            grid = int(r["grid_resolution"])
            r["metrics_csv"] = str(
                out / "runs" / shape / f"n{grid}" / f"seed_{seed}" / "metrics.csv"
            )
            r.setdefault("B_E_diameter", r.get("B_E_rigorous"))
            r.setdefault("posthoc_J_bound", None)
            r.setdefault("B_J_prior_numerical_a_priori", r.get("B_J_prior_P0H_rigorousE"))
            rows.append(r)
    figs = make_figures(rows, out)
    (out / "figures_written.json").write_text(json.dumps(figs, indent=2), encoding="utf-8")
    print("wrote", figs)


if __name__ == "__main__":
    main()
