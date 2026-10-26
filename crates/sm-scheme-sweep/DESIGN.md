# sm-scheme-sweep

The `"sweep"` volume scheme of smart-mesh: a 1-to-1 sweep of a source face along columns to a
target face (translation, twist, arbitrary linking surfaces, rotation about an outside axis).

## Positioning

Similar to Cubit's 1-to-1 `sweep` volume scheme and to the layer placement of Roca, Sarrate and
Huerta (an affine map fitted on the boundary, the residual interpolated by Shepard weights),
this scheme differs in four ways: it is constraint-first inside the `sm-scheme-api` driver (it
imposes `QUAD | BLOCK_STRUCTURED` on the linking faces, declares roles, registers a copy
constraint for the target, adds the column interval equalities to the global integer solve, and
generates as a pure function of the solved plan); its cells have exact shapes (`Quad -> Hex`,
`Tri -> 6-node Wedge`, `Polygon(n >= 5) -> prismatic Polyhedron`); twist and rotation need no
special code, because the layers are affine images of the source and placement is exact; and
its output is bit-identical for 1 and N threads, with canonical node keys. This gives a
polygonal source a polyhedral mesh for the in-house Sn/MOC/Monte Carlo code (Cubit cannot) and
makes twisted and rotational sweeps analytically testable. Gaps against Cubit in W3a: no
projected (non-copy) target (`target_mode: "project"` is rejected with `BadParams`), no pole
columns on a rotation axis and no 360 degree revolve, no many-to-one or multisweep, no smoothing.

## Pipeline

Planning (all three phases share one analysis, `scheme::analyse`):

1. *Columns* (`discover_columns`). For each source vertex follow the single edge that is not an
   edge of the source, the target or an interface face, until a target vertex is reached. At a
   vertex with several candidates (an interior vertex of a composite piece) the edge that
   continues straight on is taken (cosine > 0.9). It handles one volume, a composite piece (the
   layer count is the sum of the step counts) and a source with several loops (holes: each
   loop contributes its own columns). `NotApplicable` (with a reason) when a source vertex has
   no column, source and target share a vertex, the vertex counts differ (many-to-one), two
   columns end at one target vertex, or a linking face has fewer than four *coedges* (a
   cylinder face has three distinct edges, its seam is used twice). Without `source`/`target`
   the face pairs are tried lowest ids first.
2. *Target transform* (`fit_target_transform`). See below. `target_mode: "auto"` registers the
   copy only when the fit is accepted; `"copy"` registers it with the best-effort transform.
3. *Requirements*: roles `SweepSource/SweepTarget/SweepLinking`, `require_caps(linking, QUAD |
   BLOCK_STRUCTURED, Hard)`, the `CopyConstraint(source -> target, T, SweepTarget)` with the
   vertex map of the columns, and the `Columns` as plan data.
4. *Intervals*: `equal` rows between corresponding edges of all columns, step by step, and, with
   `layers: n`, one `linear` row `sum(first column edges) = n`.

Generation:

1. Source nodes (nodes and references of the source patch) in `NodeKey` order. A vertex node's
   column is the edge-node chain of its path; an edge node's column is walked across the linking
   faces with `GenCtx::strip_from` (one face after another through composite interfaces); an
   interior node's column is `(volume, slot * (L-1) + l - 1)` for layers `0 < l < L` and
   `(target, k.index)` at the top (the copy's node).
2. Positions of boundary column nodes come from the earlier waves (`GenCtx::point`).
3. `place_layers` places the interior nodes (below).
4. Orientation per source element from the sign of `n . up` at layer 0 (`n` the Newell normal
   of the bottom ring, `up` the mean displacement to layer 1), then cells in source-element
   order, then layer order. Template order of `sm-mesh-api`: bottom ring counter-clockwise
   seen from the sweep direction, the top ring above it. A polygon with five or more sides is a
   polyhedron with the faces reversed bottom loop, top loop, then `n` side quads, all outward.
5. Quality gate, then the patch: created nodes (interior layers), references, elements. The
   patch is assembled directly in CSR form (no per-element allocation).

### Target transform

`fit_target_transform(src, dst, n_s, n_t, tol)` with outward normals of the solid on both
faces (from the volume's face-use side, so a `Right` use is negated):

* If all `dst_i - src_i` agree within `tol` the result is `Transform::translation(mean)`: a
  translation comes out bit-exact (and axial cylinders, whose vertex sets are one or two
  points, stay full rank by this path).
* Otherwise both vertex sets are coplanar, so one virtual pair `c_s + l n_s -> c_t - l n_t`
  (`c` the centroids, `l` the mean vertex distance from `c_s`) is added. The fit is then full
  rank and encodes that the map takes the outward source normal to the inward target normal;
  a rotation by `phi` about `z` comes out as `Rz(phi)`, a mirror image is rejected (`det < 0`),
  and an inconsistent normal pair raises the residual.
* Accepted only if the largest vertex residual is `<= tol = 1e-9 * (source bbox diagonal)` (a
  constant: the model tolerance is not reachable through `PlanCtx`) and `det > 0`.

### `fit_affine`

Centred least squares: with `B = [b_i - mean(b)]`, `C = [c_i - mean(c)]` (3 x n),

    M = C Bᵀ (B Bᵀ)⁺ + Σ_dropped v vᵀ,   t = mean(c) - M mean(b)

where `B Bᵀ = V diag(λ) Vᵀ` (`sym3_eigen`), `(B Bᵀ)⁺ = Σ_{λ_k >= 1e-10 λ_max} v_k v_kᵀ / λ_k`,
and the dropped (null) eigen-directions are completed with the identity. For a planar point
set the null direction is the plane normal, so `M` is the exact in-plane map and keeps
`det > 0`. At least 4 points and moment-matrix rank >= 2 are required.

### Layer placement

For each layer `l = 1..L-1` with boundary nodes `B` sorted by `NodeKey` and `c_j[l]` their
column nodes:

* *Translation fast path*: if all `c_j - b_j` agree within `1e-12 * (boundary bbox diagonal)`,
  then `x -> x + d` with `d` the mean (the first difference itself if all are bit-identical).
  Lattices stay bit-exact. No residual is added.
* *Affine*: `M, t = fit_affine(b, c)`, residuals `r_j = c_j - (M b_j + t)`, interior node
  `x -> M x + t + Σ_j w_j(x) r_j`.
* *Interpolate*: `x -> x + Σ_j w_j(x) (c_j - b_j)`.
* Shepard weights `w_j = d_j^-p / Σ d^-p` over the `support` nearest boundary nodes (`p = 3`,
  16 by default; `support: 0` is all); ties in distance break by `NodeKey`. Weights are sparse,
  computed once per interior node with a uniform-grid k-nearest search, and reused for every
  layer. A node coincident with a boundary node takes weight 1 on it. Weights sum to one, so a
  uniform residual shifts every interior node by exactly that residual.
* `Placement::Auto`: translation, then affine (interpolation if the fit is impossible); if the
  Jacobian gate rejects the result the whole sweep is placed again with interpolation, and the
  failure of the first attempt is reported if that fails as well.

Boundary nodes are never recomputed: the cells reference the nodes the linking faces and the
target copy already own.

### Quality gate

After placement every cell must pass: Hex 8 and Wedge 6 corner scaled Jacobians
`det(e_a, e_b, e_c) / (|e_a| |e_b| |e_c|)` must exceed `min_scaled_jacobian` (default 0,
strict; 1 for an orthogonal corner); a polyhedron needs positive signed volume and positive
face-pyramid volumes from its vertex centroid. A failure is `FailureKind::Quality` naming the
layer, the source element index, the Jacobian and the `NodeKey`s of the cell, and carries the
same keys in `GenerateFailure::nodes`, so the driver can reject `(volume, sweep)` and fall back
and diagnostics can locate the cell. The gate is checked in parallel over elements with
`find_first`, so the reported cell is the first in element/layer order for any thread count.
The wedge corner definition is the plain normalized triple product, which differs from
`sm-quality`'s wedge metric (weighted to an equilateral base); hexes agree with `sm-quality`.

## Determinism and parallelism

Node tables are sorted by `NodeKey`; sums run in sorted order; weights, layer fits, orientation
and the gate are `rayon` maps with ordered collects (above `PAR_MIN = 1024` items, or `PAR_MIN / 8`
source elements for the gate; below that they run serially, with the same result). No hash-map iteration, no randomness, no time.
Tests compare `Sequential`, `ScopedThreads(4)`, a 1-thread and a 4-thread rayon pool, on
40 000-cell meshes, by bit patterns.

## Test formulas

* Inscribed sector prism: `V = (L/2)(r_out² - r_in²) h sin(sweep/L)`, within the relative
  deficit `1 - sin(x)/x <= x²/6`, `x = sweep/L`, of `sweep/2 (r_out² - r_in²) h`.
* Inscribed ring: `V = (n/2)(r_out² - r_in²) h sin(2π/n)`; cylinder: `(n/2) r² h sin(2π/n)`.
* Twisted prism: `V = side² h (2 + cos φ) / 3`, checked by exact 2x2x2 Gauss integration of the
  trilinear Jacobian (exact for trilinear hexes), and node `l` of source node `x` at
  `((1-t) I + t Rz(φ)) x`, `z = h t`, `t = l/L`.

## Recorded numbers

Twisted prism `(side 2, h 3, φ = π/2)`, 4x4 source quads, 6 layers: minimum scaled Jacobian
**0.832050** (all corner Jacobians positive; the fit residual is zero to 1e-12).

Performance (release build, 4 rayon threads, this VM; `tests/perf.rs`, `--ignored`):

| case | cells | generate | commit (reference builder) | commit share |
|---|---|---|---|---|
| brick 100x100x100 (translation path) | 10⁶ | 0.79 s | 15.8 s | 95 % |
| twisted prism 100x100x100 (affine + Shepard) | 10⁶ | 6.0-6.9 s | 15.4-16.4 s | 70-73 % |

`generate` is the whole `Driver::generate` (curves, source, linking faces, target copy and the
volume), against the reference of 4.7 s driver-only for the complete mesh. In the twisted case
about 3.3 s of the 6 s are 396 calls to `Patch::strip_from` on Submap-meshed (grid-less) linking
faces (about 7.5 ms each: the edge index of the 10⁴-quad patch is rebuilt per call); placement,
gate and emission together take about 0.15 s. See the API change request in `W3_REPORT.md`.
The bulk mesh commit is W3b work and dominates both cases.

## Limits and deferred

`target_mode: "project"` (stretch, not implemented), full 360 degree revolve and pole columns
(need a topology decision on periodic revolve), many-to-one and multisweep (W4), smoothing,
auto selection and a registry fallback for sweep (composer's choice), a model-tolerance-based
copy tolerance (needs a `PlanCtx` accessor). Polyhedral side faces are non-planar when the
swept polygon twists; `validate` checks declared planar faces only, exporters may report face
warp.
