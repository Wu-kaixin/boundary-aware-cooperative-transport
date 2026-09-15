"""Export a reviewable delivery bundle for feat/sampled-theorem-mode.

Writes a self-contained patch (all source, config, script, and test changes vs
the frozen baseline 98ba28e), a manifest with the patch sha256 and the pytest
result, an updated assumption->code table, and reproduce commands for every
audit (unit tests, validation, N=16 static, discrete dissipation, J/E bounds,
multi-shape).
"""
from __future__ import annotations

import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(
    r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11\delivery"
)
FROZEN_BASELINE = "98ba28e835a8c45fa8375b2a5ae6130c17e547ad"

# Every path that carries a change relative to the frozen baseline.  Tracked
# modifications (controller/safety/environment) show up in ``git diff HEAD``
# directly; untracked new files are surfaced with an intent-to-add and reset
# afterwards so the working tree is left untouched.
PATHS = [
    # Modified tracked sources.
    "src/dbact/controller.py",
    "src/dbact/safety_filter.py",
    "src/dbact_sim/environment.py",
    # New sampled-theorem-mode runtime.
    "src/dbact/theorem_mode.py",
    # Configs (all theorem static-oracle shapes).
    "configs/sim/theorem",
    # Audit / validation / diagnosis / export scripts.
    "scripts/audit_n16_static_performance.py",
    "scripts/audit_discrete_dissipation.py",
    "scripts/audit_JE_and_bounds.py",
    "scripts/audit_projection_margin_band.py",
    "scripts/run_multi_shape_static.py",
    "scripts/finalize_advisor_delivery.py",
    "scripts/validate_theorem_mode.py",
    "scripts/diagnose_seed8_fork.py",
    "scripts/export_delivery_bundle.py",
    # Tests + branch note.
    "tests/test_theorem_mode.py",
    "BRANCH_NOTE_feat_sampled_theorem_mode.md",
]


def run(cmd: list[str]) -> bytes:
    return subprocess.check_output(cmd, cwd=str(ROOT))


def assumption_code_table() -> list[dict]:
    """Map each theorem-mode assumption to the code that enforces / evaluates it."""
    return [
        {
            "assumption": "Sample-and-hold execution without clipping (U applied as P + Delta U)",
            "status": "implemented",
            "code": [
                "src/dbact/theorem_mode.py::apply_theorem_commands",
                "src/dbact/theorem_mode.py::theorem_step",
                "configs/sim/theorem/*.yaml:theorem_disable_clipping=true",
            ],
        },
        {
            "assumption": "Abort on illegal state instead of silent fallback",
            "status": "implemented",
            "code": [
                "src/dbact/theorem_mode.py::TheoremModeAbort",
                "src/dbact/theorem_mode.py::_abort",
                "configs/sim/theorem/*.yaml:theorem_forbid_fallback=true",
            ],
        },
        {
            "assumption": "Wall half-plane rows in the hold QP (domain containment)",
            "status": "implemented",
            "code": [
                "src/dbact/theorem_mode.py::wall_halfplanes",
                "src/dbact/theorem_mode.py::point_in_domain",
                "src/dbact/safety_filter.py (wall rows + rho margin)",
            ],
        },
        {
            "assumption": "Hold-segment pairwise and object clearance over the whole step",
            "status": "implemented",
            "code": [
                "src/dbact/theorem_mode.py::hold_segment_min_distance",
                "src/dbact/theorem_mode.py::team_hold_min_distance",
                "src/dbact/theorem_mode.py::hold_segment_object_clearance",
                "src/dbact/theorem_mode.py::omitted_neighbor_certificate",
            ],
        },
        {
            "assumption": "Assumption 2 -- true object motion bound (frozen cargo => v_obj = 0)",
            "status": "satisfied_by_construction",
            "code": [
                "src/dbact/theorem_mode.py::freeze_cargoes",
                "configs/sim/theorem/*.yaml:cargoes[].movable=false",
                "configs/sim/theorem/*.yaml:max_object_speed=0.0",
            ],
        },
        {
            "assumption": "Assumption 3 -- same Gaussian offset kernel (uniform cage, no lead/explore)",
            "status": "structurally_aligned",
            "code": [
                "configs/sim/theorem/*.yaml:density_mode=offset, cage_offset, lead_offset=null,"
                " gap_gain=0, explore_gain=0",
                "src/dbact/theorem_mode.py::oracle_boundary_view",
                "src/dbact/theorem_mode.py::reference_offset_note",
            ],
        },
        {
            "assumption": "Assumption 5 -- local Lipschitz continuous-time feedback",
            "status": "not_claimed",
            "code": ["hard Voronoi/grid membership retained; sampled model used instead"],
        },
        {
            "assumption": "Theorem parameter guard (gamma_agent*Delta<=1, theorem params valid)",
            "status": "checked_at_runtime",
            "code": ["src/dbact/theorem_mode.py::assert_theorem_params"],
        },
        {
            "assumption": "Fine-grid mass certificate (29): eta_m floor >= m_-",
            "status": "fails_as_documented",
            "code": [
                "scripts/audit_n16_static_performance.py::theoretical_constants (eta_m_floor vs m_minus)"
            ],
        },
        {
            "assumption": "Discrete descent inequality on the hold map (three command stages)",
            "status": "numerically_consistent_not_certified",
            "code": [
                "scripts/audit_JE_and_bounds.py::three_command_hyp (same-observer H,g,m on refinement)",
                "scripts/audit_discrete_dissipation.py::dissipation_rhs",
            ],
            "note": (
                "strict_numerical_certificate always false "
                "(reason=no_rigorous_quadrature_error_bound); layers coarse/refined/"
                "unresolved_quadrature_error recorded per stage."
            ),
        },
        {
            "assumption": (
                "Finite-time J bound with per-robot effective gain lambda_i and "
                "qualifying step set K_0 (full-horizon only)"
            ),
            "status": "conditional_full_horizon_else_inapplicable",
            "code": [
                "scripts/finalize_advisor_delivery.py::finite_time_bound",
                "scripts/finalize_advisor_delivery.py::main (c_shape seed 5 inapplicable)",
            ],
            "note": (
                "a_i=2/lambda_i-Delta>=a=2/k_c-Delta>0; geometric J ceiling "
                "M_total*u_max^2; c_shape seed 5 full-horizon bound inapplicable "
                "(frames 241-246, agent 14)."
            ),
        },
        {
            "assumption": "J/E post-hoc bound J_bar <= 2H0/(aKD)+4E_bar/a^2 (E_bar post-hoc)",
            "status": "post_hoc_only",
            "code": [
                "scripts/audit_JE_and_bounds.py::compute_bounds",
                "scripts/audit_JE_and_bounds.py::a_priori_conclusion (a_priori_error_bound_sufficient=false)",
            ],
        },
        {
            "assumption": "Theorem 1 coarse asymptotic bound as a quantitative certificate",
            "status": "not_established",
            "code": ["reported as diagnostic scale only; assumptions incomplete"],
        },
    ]


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    sha = run(["git", "rev-parse", "HEAD"]).decode().strip()
    branch = run(["git", "branch", "--show-current"]).decode().strip()

    existing = [p for p in PATHS if (ROOT / p).exists()]
    missing = [p for p in PATHS if not (ROOT / p).exists()]
    subprocess.check_call(["git", "add", "-N", *existing], cwd=str(ROOT))
    try:
        patch = run(["git", "diff", "HEAD", "--", *existing])
    finally:
        subprocess.check_call(["git", "reset", "-q", "HEAD", "--", *existing], cwd=str(ROOT))

    patch_path = OUT / "feat_sampled_theorem_mode_vs_98ba28e.patch"
    patch_path.write_bytes(patch)
    patch_sha = hashlib.sha256(patch).hexdigest()

    status = run(["git", "status", "--porcelain"]).decode("utf-8", errors="replace")
    (OUT / "git_status_porcelain.txt").write_text(status, encoding="utf-8")

    assumption_table = assumption_code_table()
    (OUT / "assumption_code_table.json").write_text(
        json.dumps(assumption_table, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "src")
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    test_out = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/test_theorem_mode.py", "-q"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=env,
    )
    (OUT / "pytest_theorem_mode.txt").write_text(
        test_out.stdout + test_out.stderr, encoding="utf-8"
    )

    import numpy
    import scipy
    import yaml

    try:
        import matplotlib

        mpl = matplotlib.__version__
    except Exception:
        mpl = None

    art = (
        r"E:\boundary-aware-cooperative-transport\artifacts\theorem_audit_theorem_mode_2026-09-11"
    )
    snapshot = {
        "frozen_baseline": FROZEN_BASELINE,
        "branch": branch,
        "head": sha,
        "worktree": str(ROOT),
        "os": platform.platform(),
        "python": sys.version,
        "packages": {
            "numpy": numpy.__version__,
            "scipy": scipy.__version__,
            "matplotlib": mpl,
            "PyYAML": yaml.__version__,
        },
        "patch_file": patch_path.name,
        "patch_bytes": len(patch),
        "patch_sha256": patch_sha,
        "paths_included": existing,
        "paths_missing": missing,
        "pytest_returncode": test_out.returncode,
        "assumption_code_table": "assumption_code_table.json",
        "two_theorem_modes": {
            "sampled_execution_mode": {
                "where": "feat/sampled-theorem-mode: src/dbact/theorem_mode.py + controller/safety/environment patches",
                "meaning": (
                    "Static sample-and-hold cover: wall rows, no clipping, mode freeze, "
                    "abort-on-illegal, hold-segment pair/object checks"
                ),
            },
            "certificate_mode_on_research_v3": {
                "where": "DBACT-research-v3 @ 795105f: density/CVT fail-closed gates + theory/certificates",
                "meaning": (
                    "Proof-certificate wiring for density/locality constants; does NOT "
                    "implement the sampled execution fix"
                ),
            },
            "warning": "Same flag name theorem_mode, different responsibility. Do not merge blindly.",
        },
        "reproduce": {
            "env": "OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 PYTHONPATH=src",
            "unit_tests": "python -m pytest tests/test_theorem_mode.py -q",
            "validation": "python scripts/validate_theorem_mode.py --out <out>/validation --frames 80",
            "n16_static_audit": (
                "python scripts/audit_n16_static_performance.py "
                "--config configs/sim/theorem/static_l_shape_n16_oracle.yaml "
                "--seeds 2 5 8 --frames 600 --observer-stride 5 --out <out>/n16_static_audit"
            ),
            "discrete_dissipation": (
                "python scripts/audit_discrete_dissipation.py "
                "--config configs/sim/theorem/static_l_shape_n16_oracle.yaml "
                "--seeds 2 5 8 --frames 600 --ntheta 128 --nradial 12 "
                "--out <out>/n16_discrete_dissipation"
            ),
            "je_bounds_audit": (
                "python scripts/audit_JE_and_bounds.py "
                "--config configs/sim/theorem/static_l_shape_n16_oracle.yaml "
                "--dissipation-dir <out>/n16_discrete_dissipation "
                "--static-audit-dir <out>/n16_static_audit --seeds 2 5 8 "
                "--out <out>/n16_JE_bounds"
            ),
            "multi_shape": (
                "python scripts/run_multi_shape_static.py "
                "--seeds 2 5 8 --frames 600 --ntheta 128 --nradial 12 "
                "--out <out>/n16_multi_shape"
            ),
            "finalize_advisor_delivery": (
                "python scripts/finalize_advisor_delivery.py "
                "--seeds 2 5 8 --ntheta 128 --nradial 12 "
                "--out <out>/final_advisor_delivery  "
                "# offline; reuses nine-case step_records, fixed three_command_hyp "
                "(same-observer H,g,m), per-robot lambda finite-time J bound, "
                "nine_case_summary.json + figures + CSV"
            ),
            "compile_report": (
                "cd <out>/final_advisor_delivery/latex && "
                "tectonic main.tex   # or pdflatex/latexmk/xelatex; produces main.pdf"
            ),
            "artifact_root": art,
        },
    }
    (OUT / "delivery_manifest.json").write_text(
        json.dumps(snapshot, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(snapshot, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
