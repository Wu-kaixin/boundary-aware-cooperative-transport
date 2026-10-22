# Project working conventions

- Use this repository's root `.venv` for every Python command, package install,
  test and experiment. On Windows call `.\.venv\Scripts\python.exe` explicitly;
  on POSIX call `.venv/bin/python`. Use `scripts/setup_env.ps1` to rebuild it.
- One project has one environment. The included MAS platform uses the same root
  environment. Do not use Conda or import another project's `site-packages`.
- Create `.venv` on the current machine using a standalone Python interpreter;
  copied virtual environments are historical artifacts, not runnable installs.
- Keep runtime code in `src/`, executable helpers in `scripts/`, configurations in
  `configs/`, experiment designs in `experiments/`, and regression tests in `tests/`.
- Keep original run receipts, source snapshots, raw data and historical conclusions
  unchanged. New experiments use a new directory under `runs/`. Never relabel old
  results as validation of the current source or environment.
- Local reference originals and extraction data live in `data/literature/`;
  literature notes live in `docs/literature_review_2026-10-22/`. Do not commit
  downloaded full-text references, local environments or large raw run archives.
- Before removing a migration source, verify every file against its SHA-256
  mapping, including conflicting originals and hidden environment/Git files.
