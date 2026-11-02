# Boundary map provenance investigation v1

Base: fa8b6194d65d8027303c97cdb207ecb6cf02742a. The preceding goal turn made
progress: published complete QP diagnostics and verified both platform CI jobs.

Question: identify which operations created and retained the known far-C
geometry-candidate ghost near agent_05, frame917. This is an adaptive debugging
investigation of an already selected negative case, not a new performance trial.

Replay that original case through frame933 with its frozen source/environment,
seed and saved pre-physics inputs. No physics, numerical change or control
intervention. Record copied map points/keys/timestamps/weights/normals before
registration, after rigid shift before rekey, after registration/rekey, after
carving and after fusion. Also retain the own/one-hop fresh packets, registration
increments and fusion-drift terms. Truth distances are observer-only diagnostics.

Validate every frame's commands, all logged diagnostics, solver statistics,
phase/guards/response and packed map snapshot exactly against the original.
Stop and preserve failure if any mismatch occurs. First preflight20 frames in
a separate new output, then run the fixed933-frame trace once. Save script,
input receipt, source and environment identities before execution.

Trace the selected point's history backward across key-preserving fusion and
rigid shifts/rekey; account for any cell collisions rather than silently picking
one parent. Separate observed position error, registration transport, refreshed
observation support and carving failures. An operation-level explanation may
motivate a parameter-preserving repair; it cannot establish that the repair or
the original rejected geometry candidate succeeds. Wider safety, local teamwork,
deployment contribution and the paper remain active goal requirements.
