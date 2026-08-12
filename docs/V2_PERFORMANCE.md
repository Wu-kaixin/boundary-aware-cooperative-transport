# V2 visualization pipeline performance report

Measured on 2026-08-12 on the development Windows/Python 3.12 environment.
The fixed witness was
`configs/sim/v3/arbitrary_shape_full_workspace_500.yaml`, seed 0, 500 physics
steps. The safety QP continued to run on every physics step for every agent.

## Simulation and observer

Command:

```text
python scripts/profile_visualization_pipeline.py \
  --config configs/sim/v3/arbitrary_shape_full_workspace_500.yaml \
  --steps 500 --seed 0 --repeats 2
```

| Measurement | Result |
|---|---:|
| Headless simulation FPS, median | 41.13 |
| Sparse visualization observer FPS, median | 40.80 |
| Observer overhead | 0.82% |
| Headless/observer numeric identity | exact |
| Artist + canvas rendering FPS (51 frames) | 10.51 |

The two headless samples were 43.20 and 39.07 FPS; observer samples were 43.77
and 37.82 FPS. Scheduling noise is larger than the observer cost, so the median
overhead is reported rather than claiming precision the run does not support.
A separate one-command `--no-render` 500-frame run measured 47.91 FPS. Both
measurements remain above the requested 20 control frame/s threshold without
reducing safety update frequency.

The observer-on run was compared element-for-element with its same-seed headless
run: every agent position, every cargo vertex and the complete safety solver
statistics were identical.

## Offline encoding

The same saved 501-frame trace was rendered with a stride of 10 at 938 x 532:

| Output | Rendered frames | Before overlay cache | After overlay cache |
|---|---:|---:|---:|
| H.264 MP4 | 51 | 4.18 FPS | 4.69 FPS |
| Pillow GIF | 51 | 4.29 FPS | 4.08 FPS |

The MP4 path improved by about 12%. The GIF difference is within single-run
encoder variability and should not be read as a simulation regression. Both
files decoded successfully; the GIF contained 51 frames and ffmpeg decoded the
MP4 with no errors. Simulation FPS, artist/canvas FPS, encoder FPS, and playback
FPS are deliberately separate quantities.

The renderer optimization caches display-only sensor segments and the ordered
fused-boundary polyline for each sparse overlay snapshot, and precomputes HUD
phase history. Polygon, robot, trail, sensor, map, cage, contact and push artists
are reused rather than re-created per frame.

## Profiled bottlenecks

A 150-step cProfile sample attributed cumulative time as follows:

| Component | Cumulative time | Calls |
|---|---:|---:|
| Controller step | 0.997 s | 150 |
| Safety velocity filter | 0.639 s | 2,700 |
| 2-D QP solve | 0.415 s | 2,700 |
| Contact dynamics | 0.191 s | 150 |
| Perception / ray returns | 0.185 / 0.171 s | 900 |
| Per-step metrics record | 0.160 s | 151 |

The actual simulation bottleneck is the validated controller/safety path,
especially per-agent QP filtering, followed by contact dynamics and perception.
V2 does not lower their frequency or alter their thresholds to manufacture a
higher frame rate. The raw profiler output is written to
`runs/v2_performance/profile.json` by the command above.
