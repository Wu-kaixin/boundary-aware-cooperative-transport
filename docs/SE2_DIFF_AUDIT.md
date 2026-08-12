# SE(2) diff audit — every deleted line under `src/`, and why

The v2 acceptance rule is that the `v1 → v2` diff under `src/dbact` and `src/dbact_sim`
contains no deleted lines, **except** for additions and the SE(2)-directed modification,
and that the modification is justified segment by segment. This file is that
justification.

Baseline: `origin/Claude-boundary-aware-closed-loop-v1` at `92ee6f6`.

```bash
git diff origin/Claude-boundary-aware-closed-loop-v1 -- src/dbact src/dbact_sim --numstat
```

| file | added | deleted | classification |
| --- | --- | --- | --- |
| `src/dbact/geometry.py` | 94 | 0 | pure addition (v2 certificate helpers) |
| `src/dbact/guarantees.py` | 911 | 0 | new file (T1) |
| `src/dbact/metrics.py` | 139 | 0 | pure addition (T1 audit measures) |
| `src/dbact_sim/environment.py` | 77 | 0 | pure addition (T2 audit hook) |
| `src/dbact/boundary_map.py` | 250 | 11 | **SE(2) modification** |
| `src/dbact/controller.py` | 31 | 5 | **SE(2) modification** |
| `src/dbact/safety_filter.py` | 79 | 7 | **SE(2) modification** |

Four of the seven files have zero deleted lines. The 23 deletions in the other three are
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
