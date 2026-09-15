from pathlib import Path

import csv
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path("artifacts/apriori_certificate_repair_2026-09-15")
figdir = root / "figures"
figdir.mkdir(exist_ok=True)


def load(path: Path):
    with path.open(encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


rows = load(root / "summary.csv") + load(root / "stage_e_seeds_11_17_23" / "summary.csv")
cert = {
    "l_shape": 0.18826500477830582,
    "rectangle": 0.14354783808912766,
    "c_shape": 0.19775039877477252,
}
fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.0), constrained_layout=True)
for ax, shape in zip(axes, ("l_shape", "rectangle", "c_shape")):
    sub = sorted((r for r in rows if r["shape"] == shape), key=lambda r: int(r["seed"]))
    x = list(range(len(sub)))
    labels = [f"s{r['seed']}" for r in sub]
    jbar = [float(r["J_bar"]) for r in sub]
    geom = [float(r["B_J_geom"]) for r in sub]
    ax.scatter(x, jbar, label="measured J", zorder=3)
    ax.hlines(cert[shape], -0.4, len(sub) - 0.6, colors="C1", linestyles="--", label="B_J,prior (R^2 M)")
    ax.scatter(x, geom, marker="_", s=160, label="geom M u_max^2")
    for i, r in enumerate(sub):
        if r["theorem_conditions_hold_full_horizon"] != "True":
            ax.axvspan(i - 0.4, i + 0.4, color="orange", alpha=0.3)
    ax.set_xticks(x, labels)
    ax.set_title(shape)
    ax.set_yscale("log")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7)
fig.suptitle("Measured J vs repaired prior vs geometry. Orange = K0 failed (theorem inapplicable).")
fig.savefig(figdir / "J_bounds_comparison.png", dpi=150)
fig.savefig(figdir / "J_bounds_comparison.pdf")
print("wrote", figdir)
