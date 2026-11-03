# Declared arm-label comparison correction; no physical changes

During the frozen eight-episode run, all four controls completed. Their25 replay
arrays and config/paired_metrics/physics_substeps/summary scientific files match
the old bridge candidates exactly. The old shared reproduction helper compares
result.arm as a scientific field, but this protocol explicitly names the control
map_bridge_control rather than map_bridge_candidate. Its original comparison
therefore reportsFalse solely for the predeclared arm label. Preserve that result
in control_reproduction_precheck_v1.json; all other fields matched.

Use an explicitly identified analysis adapter. Check matching condition/case IDs
and the exact old/new arm pair, then compare a temporary result copy with only
that arm label normalized. Record both actual arm labels in execution_metadata.
Retain all original array/JSON/result comparisons; an endpoint, command, safety
or other scientific change must still fail. Reject unexpected identities.
Do not edit original data, the physical protocol, frozen helpers, runtime,
budgets, gates, parameters or running workers. No physical rerun is authorized
by this clerical analysis correction. Snapshot the adapter, this addendum and
tests separately and bind its outcome to the resulting full analysis hash.

## Exact packet array serialization correction

A real saved 12-point ScanSupport packet exposed a second analysis-only defect:
`recorded(asdict(packet)) == recorded(asdict(packet))` raises NumPy's ambiguous
truth-value error because `recorded` leaves arrays unchanged and `asdict` copies
them. The comparison cannot audit admitted packets even when they are identical.
The frozen auditor is preserved. Its separately identified adapter installs the
existing `trace_boundary_map.plain` converter for the duration of the audit and
restores the original in `finally`. Lists compare exactly, with no tolerance or
omitted values; a one-ULP point change must still fail. Every original assertion
remains active. The adapter snapshots and hashes this serializer as well.

Before the physical analysis, exercise admitted packets on a synthetic controller
replay driven by the first 80 saved near-circle control poses. Generate fresh
candidate commands/diagnostics/maps, without integrating physics, and require the
original auditor to reproduce the array-comparison failure and the adapter to
verify the same fixture. This is an engineering preflight with zero new physical
episodes; its generated commands are not a candidate physical trajectory or a
task outcome. Preserve its original failure and all source/protocol identities.
