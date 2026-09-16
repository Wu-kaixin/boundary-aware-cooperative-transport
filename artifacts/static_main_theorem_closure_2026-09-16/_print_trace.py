import json
from pathlib import Path

path = Path("artifacts/static_main_theorem_closure_2026-09-16/barrier_diagnosis/c_shape_seed5/window_frames.json")
data = json.loads(path.read_text(encoding="utf-8"))
for pack in data:
    if pack["frame"] not in (237, 241):
        continue
    ag = pack["agents"]["14"]
    tr = (ag.get("explain") or {}).get("trace") or {}
    print("frame", pack["frame"])
    print("  mode", tr.get("mode"))
    print("  feature_kind", tr.get("feature_kind"))
    print("  n_bar", tr.get("n_bar"))
    print("  h", tr.get("h"))
    print("  h_bar", tr.get("h_bar"))
    print("  h_true_in_trace", tr.get("h_true"))
    print("  reflex", tr.get("reflex"))
    print("  two_rows", tr.get("two_rows"))
    print("  n_near", tr.get("n_near"))
    print("  n_same_face", tr.get("n_same_face"))
    print("  object_rows_qp", ag.get("h_object_qp"))
    print("  zero_rho", ag.get("zero_input_feasible_with_rho"))
