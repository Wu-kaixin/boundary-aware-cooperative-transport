import json
from pathlib import Path

p = Path(__file__).with_name("jeh_matrix") / "summary.json"
d = json.loads(p.read_text(encoding="utf-8"))
rows = d["cases"]
out_rows = []
for r in rows:
    st = r.get("solver_status_counts") or {}
    out_rows.append(
        {
            "shape": r.get("shape"),
            "seed": r.get("seed"),
            "complete_30s": r.get("complete_30s"),
            "full_horizon_K0": r.get("theorem_conditions_hold_full_horizon"),
            "abort": r.get("abort"),
            "J_bar": r.get("J_bar"),
            "E_bar": r.get("E_bar"),
            "H0": r.get("H0"),
            "H_final": r.get("H_final"),
            "zero_input_with_rho_failure_count": r.get("zero_input_with_rho_failure_count"),
            "dissipation_u_command_max_slack": r.get("dissipation_u_command_max_slack"),
            "n_optimal": st.get("optimal", 0),
            "n_qp": r.get("n_qp_solves"),
            "n_k0_fail_frames": len(r.get("condition_failure_frames") or []),
            "posthoc_J_bound": r.get("posthoc_J_bound"),
            "B_J_prior": r.get("B_J_prior_certificate"),
            "B_J_geom": r.get("B_J_geom"),
            "J_bar_below_prior": (r.get("J_bar") is not None and r.get("B_J_prior_certificate") is not None and r["J_bar"] <= r["B_J_prior_certificate"]),
        }
    )
summary = {
    "n": len(out_rows),
    "all_complete": all(x["complete_30s"] for x in out_rows),
    "all_K0": all(x["full_horizon_K0"] for x in out_rows),
    "any_abort": any(x["abort"] for x in out_rows),
    "all_qp_optimal": all(x["n_optimal"] == x["n_qp"] for x in out_rows),
    "max_slack": max(x["dissipation_u_command_max_slack"] or 0.0 for x in out_rows),
    "cases": out_rows,
}
dest = Path(__file__).with_name("jeh_table.json")
dest.write_text(json.dumps(summary, indent=2), encoding="utf-8")
print("n", summary["n"], "all_complete", summary["all_complete"], "all_K0", summary["all_K0"], "any_abort", summary["any_abort"], "all_qp_optimal", summary["all_qp_optimal"], "max_slack", summary["max_slack"])
for x in out_rows:
    print(x["shape"], x["seed"], "J", f"{x['J_bar']:.6g}", "E", f"{x['E_bar']:.3g}", "K0", x["full_horizon_K0"])
