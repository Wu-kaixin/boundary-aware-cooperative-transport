import json
from pathlib import Path

root = Path("artifacts/apriori_certificate_repair_2026-09-15")
cases = [
    (root / "stage_e_seeds_11_17_23/runs/l_shape/n20/seed_11", 11, [238, 239, 240]),
    (root / "stage_e_seeds_11_17_23/runs/l_shape/n20/seed_17", 17, [318, 319, 320, 321, 322]),
]
out = {}
for run, seed, frames in cases:
    recs = []
    for fr in frames:
        dump = json.loads((run / "constraint_dumps" / f"frame_{fr:04d}.json").read_text(encoding="utf-8"))
        bad = dump["k0_bad_agents"]
        for ag in dump["agents"]:
            if ag["agent"] not in bad:
                continue
            kinds = ag["row_kinds"]
            b_orig = ag["b_original"]
            b_eff = ag["b_effective"]
            pos = [i for i, b in enumerate(b_orig) if b > 1e-15]
            recs.append(
                {
                    "frame": fr,
                    "agent": ag["agent"],
                    "status": ag.get("status"),
                    "zero_input_feasible": ag["zero_input_feasible"],
                    "zero_input_feasible_with_rho": ag["zero_input_feasible_with_rho"],
                    "original_zero_input_feasible_with_rho": ag["original_zero_input_feasible_with_rho"],
                    "empty_feasible_set": ag["empty_feasible_set"],
                    "h_object": ag.get("h_object"),
                    "u": ag.get("u"),
                    "u_nom": ag.get("u_nom"),
                    "positive_rows": [
                        {
                            "i": i,
                            "kind": kinds[i],
                            "b_original": b_orig[i],
                            "b_effective": b_eff[i],
                        }
                        for i in pos
                    ],
                }
            )
    out[f"l_shape_seed{seed}"] = recs
    print("seed", seed, "n", len(recs))
    for r in recs:
        print(
            r["frame"],
            "ag",
            r["agent"],
            "zf",
            r["zero_input_feasible"],
            "zr",
            r["zero_input_feasible_with_rho"],
            "empty",
            r["empty_feasible_set"],
            "rows",
            r["positive_rows"],
            "h",
            r["h_object"],
        )

(root / "l11_l17_k0_extract.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
print("wrote extract")
