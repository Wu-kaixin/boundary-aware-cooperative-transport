# Codex closed-loop v2 baseline

The v2 branch was created directly from
`origin/Codex-boundary-aware-closed-loop-v1` at commit `e4f3221`.
No controller, contract, safety-filter, dynamics, scenario, or metric code was
changed for this baseline.

## Reproduction environment

- Date: 2026-08-12
- Platform: Windows / Python 3.12
- Test command:
  `python -m pytest -q --basetemp=.pytest-tmp-v2-baseline -p no:cacheprovider`
- Result: `259 passed in 27.70s`

The first test attempt used the system pytest temporary directory and produced
three setup errors because that directory was not writable in the execution
sandbox. Re-running with an explicit workspace-local temporary directory
produced the clean result above; no assertion failed in either attempt.

## Fixed 500-frame performance witnesses

Both measurements used seed 0 and timed only `SimulationEnvironment.run(500)`.
Figure and animation rendering were excluded.

| Scenario | Wall time | Simulation FPS | J | Success | Solver fallback / infeasible |
|---|---:|---:|---:|---|---:|
| `configs/sim/v2/l_shape_closed_loop_500.yaml` | 24.857 s | 20.115 | 0.0443 m | false | 0 / 0 |
| `configs/sim/v3/arbitrary_shape_full_workspace_500.yaml` | 10.345 s | 48.334 | 0.7115 m | false | 49 / 49 |

The second run reached full strict coverage and recorded HOLD at frame 462, but
the v1 success contract still classifies it as a failure because solver fallback
and infeasibility events are not hidden. These baseline outcomes are retained as
regression witnesses rather than relabelled for presentation.

## Pre-existing untracked work

The starting `main` worktree contained untracked v2 experiment files. They are
not part of this baseline or its commits. One untracked file conflicted with a
path tracked by v1 and was preserved byte-for-byte under
`.pre-v2-untracked-backup-20260812/` before switching branches.
