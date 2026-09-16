import json
from pathlib import Path

root = Path("artifacts/static_main_theorem_closure_2026-09-16/barrier_diagnosis")
for name in ["c_shape_seed5", "l_shape_seed11", "l_shape_seed17"]:
    s = json.loads((root / name / "summary.json").read_text(encoding="utf-8"))
    ts = json.loads((root / name / "target_series.json").read_text(encoding="utf-8"))
    fails = s["observed_k0_fail_frames"]
    print("====", name, "fails", fails)
    lo, hi = min(fails) - 4, max(fails) + 3
    for row in ts:
        if row["frame"] < lo or row["frame"] > hi:
            continue
        feat = row.get("feature") or {}
        dec = row.get("decompose") or {}
        hold = row.get("hold_true") or {}
        print(
            row["frame"],
            "K0",
            row["in_K0"],
            "h_agg",
            row.get("h_aggregate"),
            "h_true",
            row.get("h_true"),
            "kind",
            feat.get("kind"),
            "edge",
            feat.get("edge_index"),
            "Fhard",
            row.get("zero_in_F_hard"),
            "Frho",
            row.get("zero_in_F_rho"),
            "nbar",
            row.get("n_bar"),
            "motion",
            dec.get("motion_term"),
            "update",
            dec.get("barrier_update_term"),
            "switch",
            dec.get("face_switch"),
            "hold_sd",
            hold.get("sampled_min_sd"),
            "feature_kind_trace",
            (row.get("decompose") or {}).get("match"),
        )
