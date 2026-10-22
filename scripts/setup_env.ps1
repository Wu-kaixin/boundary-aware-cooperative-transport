param(
    [string]$Python,
    [switch]$UpdateDependencies
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $projectRoot
try {
    $env:PYTHONUTF8 = '1'
    $env:PYTHONIOENCODING = 'utf-8'
    $env:MPLBACKEND = 'Agg'
    $venvPython = Join-Path $projectRoot '.venv\Scripts\python.exe'
    if (-not (Test-Path -LiteralPath $venvPython)) {
        $candidates = @()
        if ($Python) { $candidates += $Python }
        else {
            $candidates += Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
            $command = Get-Command python -ErrorAction SilentlyContinue
            if ($command) { $candidates += $command.Source }
            $launcher = Get-Command py -ErrorAction SilentlyContinue
            if ($launcher) {
                $launchedPython = & $launcher.Source -3.12 -c 'import sys; print(sys.executable)' 2>$null
                if ($LASTEXITCODE -eq 0) { $candidates += $launchedPython }
            }
        }
        $selectedPython = $null
        foreach ($candidate in $candidates) {
            if (-not (Test-Path -LiteralPath $candidate)) { continue }
            & $candidate -c 'import pathlib,sys; base=pathlib.Path(sys.base_prefix); sys.exit(0 if sys.version_info >= (3,12) and not (base / "conda-meta").exists() else 1)' 2>$null
            if ($LASTEXITCODE -eq 0) { $selectedPython = $candidate; break }
        }
        if (-not $selectedPython) { throw 'Provide standalone Python 3.12+ using -Python C:\path\to\python.exe.' }
        & $selectedPython -m venv .venv
        if ($LASTEXITCODE -ne 0) { throw 'Could not create the project .venv.' }
    }
    & $venvPython -c 'import pathlib,sys; expected=pathlib.Path.cwd()/".venv"; assert pathlib.Path(sys.prefix).resolve()==expected.resolve(); assert not (pathlib.Path(sys.base_prefix)/"conda-meta").exists(); print(sys.executable); print(sys.version)'
    if ($LASTEXITCODE -ne 0) { throw 'The existing .venv is invalid; rebuild it with standalone Python.' }
    $lockFile = Join-Path $projectRoot 'requirements.lock.txt'
    if ((Test-Path -LiteralPath $lockFile) -and -not $UpdateDependencies) {
        & $venvPython -m pip install -r $lockFile
    } else {
        & $venvPython -m pip install -r requirements.txt
    }
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
    & $venvPython -m pip install --no-deps -e .
    if ($LASTEXITCODE -ne 0) { throw 'Editable project installation failed.' }
    & $venvPython -m pip check
    if ($LASTEXITCODE -ne 0) { throw 'Dependency consistency check failed.' }
} finally {
    Pop-Location
}
