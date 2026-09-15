# Reproducible entry for the 2026-09-15 a priori closure package (PARTIAL).
#
# Requires: conda/venv with repo requirements; cwd = repo root.
# Writes under artifacts/apriori_closure_2026-09-15/ (does not overwrite priors
# if --skip-existing is passed).

python scripts/run_apriori_centroid_bound.py ^
  --out artifacts/apriori_closure_2026-09-15 ^
  --shapes l_shape rectangle c_shape ^
  --seeds 2 5 8 ^
  --grids 20 ^
  --frames 600 ^
  --integration-method edge_green ^
  --edge-n-gon 256 ^
  --edge-panels 48 ^
  --density-mesh 80 ^
  --restrict-site-grid 4 ^
  --restrict-local-mesh 12

python -m pytest tests/test_edge_green_and_restrict.py tests/test_apriori_centroid_bound.py tests/test_local_cvt.py -q
