@echo off
REM One-click repair experiment. Comments must be REM on Windows cmd.
setlocal EnableExtensions
set ROOT=%~dp0..\..
cd /d "%ROOT%"
if errorlevel 1 exit /b 1

set PYTHONUNBUFFERED=1
set OUT=artifacts\apriori_certificate_repair_2026-09-15

python scripts\run_apriori_centroid_bound.py --out %OUT% --shapes l_shape rectangle c_shape --seeds 2 5 8 --grids 20 --frames 600 --integration-method edge_green --edge-n-gon 256 --edge-h-max 0.004 --resume
if errorlevel 1 exit /b 1

python -m pytest tests\test_edge_green_and_restrict.py tests\test_apriori_centroid_bound.py tests\test_local_cvt.py tests\test_safety_filter.py -q
if errorlevel 1 exit /b 1
exit /b 0
