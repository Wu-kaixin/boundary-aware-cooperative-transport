# Traceable publication showcase

This package presents one successful and one failed Claude v2 episode without
changing the controller or selecting cases by appearance.  The source is the
complete frozen `12 shapes x 5 seeds x 3 distances = 180` matrix in
`docs/results/v2_shape_matrix/episodes.csv`.

## Predeclared selection

- Representative success: the robust medoid of runtime-eligible, non-convex,
  `failure_class=SUCCESS` cases at the median distance factor (`alpha=0.4`).
- High-concavity failure: the median-progress member of the maximum-concavity,
  runtime-eligible `TRANSPORT_STALL` group at the same distance factor.
- Every case remains in the denominator.  The showcase must report the frozen
  unconditional result, `54/180`, next to the two examples.

With the frozen table these rules select:

- `u_shape__a0.40__seed000` as the representative success;
- `star10__a0.40__seed001` as the high-concavity transport stall.

## Build

Install the publication dependency set first (`pip install -e .[publication]`),
or use `requirements.txt` / `environment.yml`.  The full build checks for its
H.264 encoder before running either selected episode.

Selection and provenance only:

```bash
python scripts/build_publication_package.py \
  --select-only \
  --output runs/publication_showcase
```

Full rerun, validation, MP4, per-case Figures A--G, and paired Figure H:

```bash
python scripts/build_publication_package.py \
  --output runs/publication_showcase \
  --frame-stride 10 \
  --paper-formats png,pdf,svg
```

The build fails if the rerun differs from the frozen CSV in verdict, phase,
frame count, solver status, displacement, efficiency, coverage, cross-track, or
safety metrics.  `publication_manifest.json` records source hashes, selection
rankings, rerun validation, render rates, and the SHA-256/size of every output.

Simulation FPS, offline rendering FPS, and playback FPS are separate fields.
Ground-truth-only debug layers are not used in the paper or demo outputs.

## Preview-only recovery

The frozen matrix manifest predates environment fingerprinting: it records the
git SHA and config hash, but not the OS, Python, NumPy, or SciPy versions.  A
long-horizon stalled trajectory can therefore keep the same categorical verdict
while drifting in continuous endpoint metrics on another environment.  The
default build fails closed in that situation.

For visual inspection only, already-rendered current-environment traces can be
indexed without rerunning:

```bash
python scripts/build_publication_package.py \
  --reuse-existing \
  --allow-numeric-drift-preview \
  --output runs/publication_showcase
```

That manifest is marked `preview_only`, includes every mismatch, and sets
`publication_eligible=false`.  It must not be used as evidence for the frozen
continuous metrics.  The publication-ready recovery is to rerun the complete
matrix in a fully locked environment, record the environment fingerprint, and
archive the selected immutable traces/replays with their hashes.
