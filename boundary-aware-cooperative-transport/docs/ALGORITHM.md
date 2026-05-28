# Algorithm Notes

## Boundary-aware cage target

For every local boundary observation `(b, n_out)`, DBACT creates a cage target outside the object:

```text
q_target = b + d_cage * n_out
```

The density field is a sum of Gaussian kernels centered at those targets.

## Local CVT

Each robot computes a local weighted Voronoi centroid using only itself and neighbors within communication range. This is a grid approximation of CVT, not a global Voronoi computation.

## Local CBF safety filter

The inter-agent safety constraint is:

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

The current implementation uses iterative half-plane projection. It is fast and dependency-light. For a paper-grade controller, replace it with a standard QP solver.

## Transport model

The simulator moves an object only if boundary coverage and the number of contact agents exceed thresholds. This validates the coordination logic but does not replace physical contact dynamics.
