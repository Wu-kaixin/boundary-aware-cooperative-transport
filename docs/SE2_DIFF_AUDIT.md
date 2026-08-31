# Diff audit — every deleted line under `src/`, and why

The v2 acceptance rule is that the `v1 → v2` diff under `src/dbact` and `src/dbact_sim`
contains no deleted lines, **except** for additions and the SE(2)-directed modification,
and that the modification is justified segment by segment. This file is that
justification.

There are **28** deleted lines in total: 23 for the SE(2) work (T2, §§1–3 below) and 6 more
for the three degradation mechanisms the robustness ablation needed (T5, §4). Every one is a
signature change or a single expression replaced in place; none removes a behaviour, and each
is covered by a test asserting the pre-existing path is bit-identical.

Baseline: `origin/Claude-boundary-aware-closed-loop-v1` at `92ee6f6`.

```bash
git diff origin/Claude-boundary-aware-closed-loop-v1 -- src/dbact src/dbact_sim --numstat
```

| file | added | deleted | classification |
| --- | --- | --- | --- |
| `src/dbact/geometry.py` | 94 | 0 | pure addition (v2 certificate helpers) |
| `src/dbact/guarantees.py` | 911 | 0 | new file (T1) |
| `src/dbact/metrics.py` | 139 | 0 | pure addition (T1 audit measures) |
| `src/dbact/error_audit.py` | 347 | 0 | new file (T2) |
| `src/dbact_sim/environment.py` | 77 | 0 | pure addition (T2 audit hook) |
| `src/dbact/boundary_map.py` | 250 | 11 | **SE(2) modification** (T2) |
| `src/dbact/safety_filter.py` | 79 | 7 | **SE(2) modification** (T2) |
| `src/dbact/controller.py` | 111 | 11 | **5 SE(2) (T2) + 6 degradation (T5)** |

Five of the eight files have zero deleted lines. The 28 deletions in the other three are
enumerated below. There are no others.

---

## `src/dbact/boundary_map.py` — 11 deletions

### 1. The "yaw is not estimated" paragraph (7 lines)

```
-Only translation is estimated. Yaw is not, and that is a stated limitation rather
-than an approximation: the object-boundary rows and the transport controller both
-use the estimate, so claiming SE(2) here without an error bound would put an
-unmeasured quantity inside a safety constraint.
-The same translation, accumulated, is the only thing any robot knows about how
-far the cargo has travelled. It is what the transport controller closes its loop
-on, so nothing in the control path reads a simulator pose.
```

**Why.** The statement became false: yaw *is* now estimated when `estimate_yaw` is on.
The replacement paragraph keeps the reasoning verbatim in substance — it is still the
reason the flag defaults to off — and adds the SE(2) derivation and the pointer to
`dbact.error_audit`, which is the error bound the old paragraph said did not exist. The
last three lines are restated unchanged in meaning: the accumulated translation is still
the only thing any robot knows about travel, and nothing in the control path reads a
simulator pose.

**Not a behaviour change.** Docstring only.

### 2. `RegistrationResult`'s one-line docstring

```
-    """One object's estimated frame-to-frame translation, with its diagnostics."""
```

**Why.** The dataclass now carries `rotation`, `reference` and `yaw_clamped`, so
"translation" undersells it. Replaced by a docstring that says so and states the
invariant that keeps v1 intact: `rotation` is exactly `0.0` and `reference` the origin
when the estimator is off.

**Not a behaviour change.** The three new fields all have defaults, so every existing
construction site is unaffected.

### 3. The residual-trim line

```
-                n, residual = n[keep], residual[keep]
```

**Why.** Became `n, residual, b = n[keep], residual[keep], b[keep]`. The yaw column of
the design matrix is built from the lever arm `b - c`, so `b` has to be trimmed by the
same mask. Trimming two of the three arrays would pair each residual with a *different*
cell's lever arm — a silent correspondence error rather than a visible one.

**Behaviour with the estimator off:** identical. `b` is trimmed but the translation-only
solve never reads it.

### 4. The `_register_object` return statement

```
-        return RegistrationResult(object_id, translation, len(residual), rms, conditioning, clamped)
```

**Why.** `_register_object` now ends by dispatching to one of two solvers:
`_solve_translation`, which is v1's arithmetic moved verbatim into its own method, and
`_solve_se2`. The dispatch is what makes "the estimator is off" mean *v1's code path*
rather than *the SE(2) path with a zero in it*.

**Behaviour with the estimator off:** identical. `_solve_translation` contains the same
normal-matrix construction, the same Tikhonov damping, the same `eigvalsh` conditioning,
the same speed clamp and the same RMS. The only addition is that it passes `reference`
through to the result, which no v1 caller reads.

---

## `src/dbact/controller.py` — 5 deletions

All five are the single call site and signature of `_object_rows_from_map`, which gained
a fourth return value.

```
-            points, normals, v_obj = self._object_rows_from_map(agent.agent_id, agent.position, view)
-    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
-            return np.empty((0, 2)), np.empty((0, 2)), np.zeros(2)
-            velocity = self.maps[agent_id].object_velocity(str(view.object_ids[nearest]))
-        return view.points, view.normals, velocity
```

**Why.** The barrier needs the velocity of each row's *own* boundary point, so the
method that assembles the rows is the method that has to produce them. Passing the twist
to the filter instead and reconstructing the field there would put the map's reference
point inside the safety filter, which has no business knowing about maps.

**Behaviour with the estimator off:** identical. The fourth value is `None`, which is the
sentinel that makes `_object_rows` take v1's `normals @ v_obj` branch. `object_velocity`
is still read exactly once, from the same map, for the same nearest object.

The guard is `if local_map.estimate_yaw:` rather than a check on whether the estimated
rate is non-zero, deliberately: a rate that happens to be zero on one frame must not
silently switch the code path being exercised.

---

## `src/dbact/safety_filter.py` — 7 deletions

### 1. The right-hand-side velocity term

```
-        demand = normals @ v_obj - self.params.gamma_obj * h
```

**Why.** This is the substantive change, and the whole item. `h_k = n_k^T (p_i - b_k) -
r_safe` differentiates to `n_k^T (u_i - v_{b_k})`, and `v_{b_k}` is the velocity of the
*material point* at `b_k`, which on a rigid body is `v_c + omega R90 (b_k - c)`. Feeding
the body's translational velocity to every row understates the demand by
`omega |b_k - c|` on the arcs turning towards the robot — largest where the object is
widest, which is where the pushing robots stand.

**Behaviour with `point_velocities=None`:** identical. `normal_velocity` is literally
`normals @ v_obj` on that branch, and
`tests/test_se2_registration.py::test_object_rows_are_identical_when_point_velocities_are_none`
asserts approximate equality of all four returned arrays against a uniform field, which
is the algebraic check that the two branches agree.

### 2–6. `_aggregate_face` signature and returns (5 lines)

```
-        self, position: np.ndarray, points: np.ndarray, normals: np.ndarray, distance: np.ndarray
-    ) -> tuple[np.ndarray, np.ndarray]:
-            return normals, np.empty(0)
-            return np.empty((0, 2)), np.empty(0)
-        return n_bar.reshape(1, 2), np.array([h_bar])
```

**Why.** Aggregate mode collapses a face to one row, so one row needs one velocity. The
method that decides which cells form the face is the only place that knows which
velocities to combine.

The aggregated velocity is the **maximum** of `n_bar^T v_k` over the face, not the
weighted mean. The aggregated row stands for the whole face, so it must demand at least
as much retreat as the fastest-approaching point on that face requires; a weighted mean
lets a fast arc be averaged away by the slow cells beside it, which is the aggregate
under-reporting the disturbance it exists to summarise. On a purely translating object
every `v_k` is equal and the maximum *is* the mean, so the conservatism appears only
where rotation is present.

**Behaviour with the estimator off:** identical. `n_bar` and `h_bar` are computed by the
same three lines as in v1; the third return value is `None`.

### 7. The `_aggregate_face` call site

Folded into the change above: the call now passes and receives the velocity.

---

## 4. `src/dbact/controller.py` — 6 further deletions, for T5's degradation mechanisms

The robustness ablation needed three mechanisms v1 did not have. All three default to exact
no-ops, and `tests/test_degradation.py::test_explicit_no_op_values_reproduce_the_baseline_exactly`
asserts that literally — identical agent positions, identical cargo pose, identical map
sizes, identical solver counters — rather than to a tolerance.

### 1. The scan call

```
-        scans = [self.sensor.sense_view(agent, cargoes, timestamp) for agent in agents]
```

**Why.** `perception_every = k` has to be able to *not* sense. The replacement wraps the same
expression in the stride test and supplies empty views otherwise. Frame 0 always senses, so a
run never starts blind.

**Behaviour at the default:** `stride = 1` makes `frame % 1 == 0` true on every frame, so the
same list comprehension runs with the same arguments every frame.

### 2–3. The two `dt` arguments to registration and fusion

```
-            local_map.register(scans[i], dt)
-            local_map.update(batch, timestamp, agent_codes=codes, dt=dt)
```

**Why.** Both now take `interval`, the time accumulated since the sensor last fired. A 4 Hz
sensor in a 20 Hz world that divided by `dt` would report a fifth of the object's true speed,
and the transport controller closes its loop on that estimate — so the arm would be measuring
a broken estimator rather than a slow sensor.

**Behaviour at the default:** `interval == dt` exactly on every frame, because the accumulator
is incremented by `dt` and reset on every sensing frame, and every frame is a sensing frame.
`tests/test_degradation.py::test_perception_stride_gives_registration_the_whole_elapsed_interval`
pins the accumulator's value frame by frame.

### 4–5. The nominal-command call

```
-            u_nom, mode, cell_mass, push_side, effort = self._nominal_command(
-                i, agents, neighbors[i], view, contact_ready, dt
```

**Why.** `planning_every = k` holds the previous nominal command on the frames it does not
plan. The safety filter still runs every frame, so a held command is re-projected against the
*current* barrier rows; holding the filtered output instead would let a stale command drive
through a constraint that became active after it was computed.

**Behaviour at the default:** `plan_stride = 1` makes `planning` true on every frame, so
`_nominal_command` is called with the same arguments as before and the hold dict is written
but never read.

### 6. The neighbour-set construction

```
-        return [list(np.flatnonzero(row <= self.params.comm_range)) for row in d]
```

**Why.** `communication_dropout_prob` masks individual directed links. Applying it here rather
than at each consumer is deliberate: one change degrades the scan relay, the object-token
flood, the progress consensus and the local contact-ready quorum together, which is what a
dropped packet actually costs.

**Behaviour at the default:** the `if dropout > 0.0` branch is skipped entirely, so `within`
is exactly `d <= comm_range` and the returned lists are the same. The RNG is not even
constructed, so the default path draws no random numbers and cannot perturb any other
stream's state.

The loss is **directed** — robot i losing j does not imply j loses i — and keyed on the frame
index, so the pattern moves rather than latching one subgraph. A latched subgraph is a
partitioned team, which is a different experiment from a lossy link;
`test_dropout_pattern_changes_between_frames` and `test_dropout_is_directional` pin both.

---

## What survived, and was checked

The v1 constructions the brief required to stay alive are all present and exercised by
tests:

| construction | where | test |
| --- | --- | --- |
| aggregate object row, `n_bar = normalize(sum g_k n_k)` | `_aggregate_face` | `test_aggregate_face_returns_no_velocity_when_none_is_supplied` |
| face filtering on `object_row_face_cosine` | `_object_rows` | unchanged; `tests/test_safety_filter.py` |
| right-hand-side reachability cap | `_cap_to_reachable` | `test_per_row_velocity_is_clamped_by_the_issf_disturbance_bound` |
| witness `w = normalize(sum n_k)` | `_cap_to_reachable` | unchanged, not touched by this diff |
| inner limit, tangential window | `_object_rows` | unchanged |
| two-tier then scaled-barrier solve | `_solve` | unchanged |

`src/dbact/phase.py` is byte-identical to v1:

```bash
git diff origin/Claude-boundary-aware-closed-loop-v1 HEAD -- src/dbact/phase.py
```

returns empty, and the seven-phase enum is unchanged with no MAP phase.
