# Protected best-known common transport configuration

`best_known_transport_v1.yaml` saves the complete resolved controller parameters,
original physics and near-L scene from the 0.45 motion-prior candidate in
`experiments/ccta2027/transport_validation_v1_new_starts.json`, seed 20261102.
All controller values, including previously implicit defaults, are explicit.
The source case is `near_l_reference_boundary_cvt_candidate`.

It is the best known common safety/completion configuration on the evaluated
conditions: 33/33 old near conditions and 11/12 later starts safely completed.
Those are historical development results, not a global optimum or population
reliability estimate. Other geometries and initial states remain in the original
protocol; this single YAML is a runnable near-L example, not all scenes combined.

The global parameter defaults remain compatible with historical configurations.
Failed/new reference candidates remain off. New numerical tuning must be a
separate declared experiment, never an edit to this protected baseline.

Run from the repository root with a **new** output directory:

```powershell
.\.venv\Scripts\python.exe -m dbact_sim simulate --config configs/research/best_known_transport_v1.yaml --seed 20261102 --steps 12000 --output runs/manual-best-known-example --no-figures
```

The simulation CLI above is a fixed-frame example. Formal settling/completion
evaluation uses the study runner and the original 40-frame/.005 m/s rule;
see `experiments/ccta2027/protected_baseline_v1_reproduction.json` and
[the full replay verification](../../docs/results/reference_surface_v1/README.md).
