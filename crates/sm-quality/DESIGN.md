# sm-quality: design

## Positioning

**Similar to** Sandia's Verdict library (the metric engine behind Cubit, VTK's `vtkMeshQuality` and Exodus tooling), **differs in** operating on smart-mesh's face-first `MeshRead` contract (mixed tri/quad/tet/hex/wedge/pyramid *and* general polygons and polyhedra, dimension-2 surface meshes included) with parallel kernels whose reductions are bit-identical for any thread count, typed `Unsupported` errors instead of silent sentinels for undefined metrics, and a validity classifier instead of a bare sign test, **giving** one call that grades a whole mixed mesh (including OpenFOAM-style polyhedral cells) reproducibly across machines and thread counts, which is what a reproducible timeline of meshing operations needs (the same mesh hashes to the same quality report).

## Confidence in the formulas

The Verdict manual and source could not be fetched in the authoring environment (outbound network was limited to package registries). Every formula below was **re-derived or reconstructed from memory of Verdict and Knupp's papers, then checked analytically** (ideal elements hit the ideal values; distorted elements hit closed forms derived by hand; see `tests/`). Treat as **verified by derivation, not diffed against Verdict**:

* High confidence (derived and self-consistent): volume, Jacobian, scaled Jacobian (corner definition), condition, Frobenius aspect, shape, aspect ratio of tri/tet, radius ratio, edge ratio, angles, stretch, diagonal, skew, taper, relative size, distortion, Oddy, tet `beta`/`gamma`.
* Medium: the normalisation factors that make the ideal element score 1 for scaled Jacobian of tri (`2/sqrt3`), tet (`sqrt2`), wedge (`2/sqrt3`), pyramid (`sqrt2`); whether the hex scaled Jacobian/Jacobian include the centre sample (they do here); Verdict's wedge reference element (equilateral base, height = edge); shear (taken as the scaled Jacobian floored at 0); collapse ratio.
* Low: the **acceptable ranges** in the tables (advisory, from memory of the manual), and quad **warpage** orientation: this crate returns `min cos^3` of opposite corner normals (1 = flat, as the Verdict source computes it as far as recalled); some documentation lists the complement (0 = flat). Callers that need the other convention can use `1 - w` themselves.

Numbers that differ from a real Verdict build by a constant factor or a centre sample are the first suspects; the `Metric::ideal` table and `tests/` pin the conventions used here so a later cross-check against Verdict output is a one-line diff per metric.

## Layers

| Layer | Item | Notes |
|---|---|---|
| element | `eval(metric, shape, corners, &Context) -> Option<f64>` | pure, allocation free, `[f64; 3]` points; `None` = metric undefined for the shape |
| mesh | `compute`, `compute_many`, `report` | `&dyn MeshRead`; one parallel pass per metric into a pre-sized column (SoA) |
| mesh | `validity` | `Valid / Degenerate / Inverted / Skipped` per cell |
| reduce | `summarize`, `histogram`, `histogram_auto` | min/max/argmin/argmax/mean/std, fixed-chunk deterministic reductions |

Files: `vec.rs` (3-vector helpers, sentinels), `metric.rs` (catalogue, applicability, ideal values, dispatch), `planar.rs` (tri/quad/polygon), `solid.rs` (tet/hex/wedge/pyramid), `faces.rs` (loop-based: fan volume, dihedral angles, sphericity, star-shapedness), `polyhedron.rs` (gather outward loops from the mesh), `driver.rs`, `stats.rs`.

## Conventions

* **Corner Jacobians.** For a 3-D template corner `[n, a, b, c]` the edge vectors are `A = [p_a - p_n, p_b - p_n, p_c - p_n]`, ordered so `det A > 0` for a right-handed element (tables in `solid.rs`, unit-tested on the ideal elements: all corners are `+1`). For a 2-D corner `k` of a polygon, `a = p_{k+1} - p_k`, `b = p_{k-1} - p_k`, `J_k = (a x b) . n` with `n` the reference normal.
* **Weighted matrices.** `T = A W^-1`, where the columns of `W` are the corner edge vectors of the ideal element (hex/quad: identity; regular tet; wedge: equilateral base with height = edge length; square pyramid with all eight edges equal, base corners only). Then `||T||_F`, `det T`, `||T^-1||_F = ||adj T||_F / |det T|`.
* **Volume** is always the mesh geometry rule `smartmesh.fan_vertex_mean.v1` (fan polyhedron, `V = 1/3 sum A_f . (apex_f - o)`), so metrics agree with `sm_mesh_api::cell_volume` (tested against it, 1e-12). It equals the Verdict volume for planar-faced cells. In 2-D, "volume" is the *signed* area `n . A` (positive when the cycle is counter-clockwise about `n`).
* **Reference normal (2-D cells).** `Options::normals`: `Own` (default; the Newell normal of the cell, so a planar cell is never inverted but a reflex corner has a negative Jacobian), `Uniform(n)`, `PerCell(&[n])`, or `Field(&dyn FaceNormals)` (the oriented normal of the model face the cell is classified on, at the cell's vertex mean: the shell orientation rule of `sm-mesh-api`).
* **Degenerate input** follows Verdict: metrics where large is bad (`AspectRatio`, `EdgeRatio`, `Condition`, aspect Frobenius, `RadiusRatio`, `Taper`, `Oddy`, `Skew`, `MaxEdgeRatio`) return at most `DBL_MAX = 1e30` for degenerate or inverted elements; metrics where small is bad (`Shape`, `RelativeSizeSquared`, `Shear`, `Diagonal`, `Sphericity`, `Warpage` of a collapsed corner) return `0`; signed metrics (`Jacobian`, `ScaledJacobian`, `Volume`, `Distortion`) return their signed value (negative = inverted). Non-finite coordinates give `NaN`. The threshold for "degenerate denominator" is `TINY = 1e-30` (absolute; the kernels are scale-free except for this guard).
* **Undefined metrics.** Element level: `eval` returns `None`. Mesh level: cells whose shape does not support the metric get `NaN`, which summaries count in `undefined` and exclude from min/max/mean; if **no** cell of the mesh supports the metric, `compute*` returns `QualityError::Unsupported`. (A mixed mesh therefore reports `Skew` for its hexes and `NaN` elsewhere; a pure tet mesh asking for `Skew` is an error.)
* **Inactive cells** (`cell_active == false`, refined parents) are skipped by default (`NaN`, `Skipped`, excluded from the reference size); `Options::with_inactive(true)` includes them.
* **High-order cells** are evaluated on their corner (vertex) skeleton only. Curved-element Jacobian checks are future work.
* **Reference size** for relative-size metrics: `ReferenceSize::MeanAll` (default; Verdict's average element size: mean of the finite positive volumes of the counted cells), `MeanPerShape`, or `Fixed(v)`. `RelativeSizeSquared = min(V/V_ref, V_ref/V)^2`, `0` for `V <= 0`.
* **Determinism.** Per-cell values are written to their own slot (no cross-thread accumulation). Reductions use chunks of `CHUNK = 4096` values, independent of the thread count: each chunk is reduced sequentially with Neumaier compensated sums, chunk results are collected in order and merged sequentially. Min/max use `f64::total_cmp` and the lowest index on ties (so `-0.0 < +0.0` is deterministic). Std-dev is two-pass (mean, then `sum (x-mean)^2`), population form. Histogram bins are integer counts with a fixed `[lo, hi]`. `tests/determinism.rs` hashes every column, summary, histogram and the validity counts for 1, 2, 3 and 4 threads, repeated runs and concurrent runs (testkit `determinism::check`).

## Element metrics: formulas, ideal values, ranges

`ℓ` edge lengths, `A` area, `V` volume. "Accept" is the advisory Verdict acceptable range (low confidence, see above). Ideal = value on the unit square/cube, equilateral triangle, regular tetrahedron, regular wedge (unit edges), regular pyramid; pinned by `Metric::ideal` and `tests/elements.rs`.

### Triangle (`Tri`)

Corner `k`: `J_k = (a x b) . n`. Weight `W = [[1, 1/2], [0, sqrt3/2]]`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume (area) | `n . (sum (p_i - p_0) x (p_{i+1} - p_0)) / 2` | `sqrt3/4` (unit edge) | > 0 |
| Jacobian | `min_k J_k` (= `2A`) | n/a | > 0 |
| ScaledJacobian | `(2/sqrt3) min_k J_k / (|a||b|)` | 1 | [0.5, 1.15] |
| AspectRatio | `ℓmax (ℓ0+ℓ1+ℓ2) / (4 sqrt3 A)` | 1 | [1, 1.3] |
| AspectFrobenius = Condition | `(ℓ0²+ℓ1²+ℓ2²) / (4 sqrt3 A)` (= `||T||_F ||T^-1||_F / 2`) | 1 | [1, 1.3] |
| Shape | `4 sqrt3 A / sum ℓ²` (= 1/Condition) | 1 | [0.25, 1] |
| RadiusRatio | `R / (2r) = ℓ0 ℓ1 ℓ2 (ℓ0+ℓ1+ℓ2) / (16 A²)` | 1 | [1, 3] |
| EdgeRatio | `ℓmax / ℓmin` | 1 | [1, 1.3] |
| MinAngle / MaxAngle | interior angles, degrees, `atan2((a x b).n, a.b)` in `[0, 360)` | 60 | [30, 60] / [60, 90] |
| Distortion | 1 if `A > 0`, -1 if `A < 0`, else 0 (the Jacobian of a linear triangle is constant) | 1 | [0.5, 1] |
| RelativeSizeSquared, ShapeAndSize | `min(τ,1/τ)²`, `· Shape` with `τ = A / A_ref` | 1 | [0.25, 1], [0.2, 1] |

### Quadrilateral (`Quad`)

Corners `k = 0..3`, `J_k` as above (`n` = Newell normal or reference normal). Principal axes `X1 = (p1-p0)+(p2-p3)`, `X2 = (p2-p1)+(p3-p0)`, cross derivative `X12 = p0-p1+p2-p3`. Weight: identity.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume (area) | signed Newell area (`= ¼ sum J_k`) | 1 | > 0 |
| Jacobian | `min_k J_k` | n/a | > 0 |
| ScaledJacobian | `min_k J_k / (|a_k||b_k|)` (sine of the corner angle) | 1 | [0.5, 1] |
| AspectRatio | `ℓmax P / (4A)`, `P` perimeter | 1 | [1, 4] |
| MaxEdgeRatio | `max(|X1|/|X2|, |X2|/|X1|)` | 1 | [1, 1.3] |
| EdgeRatio | `ℓmax/ℓmin` (extra) | 1 | |
| Skew | `|X1 . X2| / (|X1||X2|)` | 0 | [0, 0.5] |
| Taper | `|X12| / min(|X1|, |X2|)` | 0 | [0, 0.7] |
| Warpage | `min((N0.N2)³, (N1.N3)³)`, `N_k` = unit `a_k x b_k` | 1 (see note) | [0.9, 1] |
| Stretch | `sqrt2 ℓmin / dmax` (`dmax` longest diagonal) | 1 | [0.25, 1] |
| Condition | `max_k (|a|²+|b|²) / (2 J_k)` | 1 | [1, 4] |
| MaxAspectFrobenius / MeanAspectFrobenius | max / mean over corners of `(|a|²+|b|²)/(2J_k)` (equal to Condition for the max) | 1 | [1, 4] |
| Shape | `min_k 2 J_k / (|a|²+|b|²)` | 1 | [0.3, 1] |
| Shear | `max(0, ScaledJacobian)` | 1 | [0.3, 1] |
| Oddy | `max_k ((G11-G22)² + 4 G12²) / (2 det G)`, `G = [a b]^T [a b]` | 0 | [0, 0.5] |
| Distortion | `min detJ / |mean detJ|` over the 2×2 Gauss points of the bilinear map | 1 | [0.5, 1] |
| MinAngle / MaxAngle | corner angles incl. reflex (`> 180`) | 90 | [45, 90] / [90, 135] |
| RelativeSizeSquared, ShapeAndSize, ShearAndSize | as the triangle, `· Shape` / `· Shear` | 1 | [0.3, 1], [0.2, 1], [0.2, 1] |

Warpage note: Verdict-style value (cosine cubed, 1 = flat); `1 - w` gives the "0 = flat" convention.

### Polygon (`Polygon`, 5+ vertices; generalization)

`Volume` (signed area), `Jacobian`, `ScaledJacobian` (min over corners of `J_k/(|a||b|)`, negative at a reflex corner), `EdgeRatio`, `MinAngle`, `MaxAngle` as the quad, and `AspectRatio = ℓmax P / (4 tan(π/n) A)` (the triangle and quad formulas are the `n = 3, 4` cases; a regular n-gon scores 1). No Verdict counterpart; ideal-element metrics (Condition, Shape, ...) are not defined for polygons.

### Tetrahedron (`Tet`)

`a, b, c = p1-p0, p2-p0, p3-p0`, `det = a . (b x c) = 6V`. `T = A W^-1` with the regular-tetrahedron `W` (corner 0 suffices: a tet is affine). Face areas `A_i`, `ΣA = sum A_i`. Circumradius `R = |a²(b×c) + b²(c×a) + c²(a×b)| / (2 det)`, inradius `r = det / (2 ΣA)`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume | fan rule (`= det/6`) | `sqrt2/12` (unit edge) | > 0 |
| Jacobian | `det` | n/a | > 0 |
| ScaledJacobian | `sqrt2 det / max over the 4 corners of ℓℓℓ` | 1 | [0.5, 1] |
| AspectRatio (β) | `ℓmax ΣA / (6 sqrt6 V)` | 1 | [1, 3] |
| AspectFrobenius | `||T||_F² / (3 det(T)^(2/3))` (= 1/Shape) | 1 | [1, 1.3] |
| AspectGamma | `(Σℓ²/6)^(3/2) sqrt2 / (12 V)` | 1 | [1, 3] |
| RadiusRatio | `R / (3r)` | 1 | [1, 3] |
| CollapseRatio | `min_i h_i / ℓmax(face opposite i)`, `h_i = det/(2 A_opp)` | `sqrt(2/3)` | [0.1, ∞) |
| Condition | `||T||_F ||T^-1||_F / 3` | 1 | [1, 3] |
| Shape | `3 det(T)^(2/3) / ||T||_F²` | 1 | [0.3, 1] |
| EdgeRatio | `ℓmax/ℓmin` | 1 | [1, 3] |
| MinAngle / MaxAngle | min / max interior **dihedral** angle (degrees) | 70.5288 | [40, 90] |
| Distortion | sign of `det` (constant Jacobian) | 1 | [0.5, 1] |
| RelativeSizeSquared, ShapeAndSize | | 1 | [0.3, 1], [0.2, 1] |

Right-corner tet (`(0,0,0),(1,0,0),(0,1,0),(0,0,1)`) closed forms used in tests: `ScaledJacobian = 1/sqrt2`, `AspectRatio = RadiusRatio = (1+sqrt3)/2`, `Shape = 2^(1/3)·2/3`, `Condition = sqrt(3/2)`, dihedrals `{90°, acos(1/sqrt3) = 54.7356°}`, `CollapseRatio = 1/sqrt6`.

### Hexahedron (`Hex`)

Corner table `[n, a, b, c]` (verified: all `+1` on the unit cube) and principal axes `X1 = Σ edges along ξ` (4 edges), `X2`, `X3`; cross derivatives `X12`, `X13`, `X23` (sums of the second differences along the three face pairs); diagonals `0-6, 1-7, 2-4, 3-5`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume | fan rule | 1 | > 0 |
| Jacobian | `min(min_k det A_k, X1.(X2×X3)/64)` | n/a | > 0 |
| ScaledJacobian | `min(min_k det A_k/(|a||b||c|), X1.(X2×X3)/(|X1||X2||X3|))` | 1 | [0.5, 1] |
| Condition | `max_k ||A_k||_F ||A_k^-1||_F / 3` | 1 | [1, 3] |
| MaxAspectFrobenius / MeanAspectFrobenius | max / mean over corners of `||A||_F² / (3 det A^(2/3))` | 1 | [1, 3] |
| Shape | `min_k 3 det A_k^(2/3) / ||A_k||_F²` | 1 | [0.3, 1] |
| Oddy | `max_k (||G||_F² - tr(G)²/3) / det(A)^(4/3)`, `G = A^T A` | 0 | [0, 0.5] |
| Skew | `max_{i<j} |X_i.X_j|/(|X_i||X_j|)` | 0 | [0, 0.5] |
| Taper | `max(|X12|/min(|X1|,|X2|), |X13|/min(|X1|,|X3|), |X23|/min(|X2|,|X3|))` | 0 | [0, 0.4] |
| Stretch | `sqrt3 ℓmin / dmax` | 1 | [0.25, 1] |
| Diagonal | `dmin / dmax` | 1 | [0.65, 1] |
| Dimension | `V^(1/3)` | n/a | |
| MaxEdgeRatio | `max_{i,j} |X_i|/|X_j|` | 1 | [1, 1.3] |
| EdgeRatio | `ℓmax/ℓmin` over 12 edges | 1 | [1, 1.3] |
| Shear | `max(0, min_k det A_k/(|a||b||c|))` (corners only) | 1 | [0.3, 1] |
| Distortion | `min detJ / |mean detJ|` over the 2×2×2 Gauss points | 1 | [0.5, 1] |
| Min/MaxAngle | min/max interior dihedral angle over the 12 edges | 90 | |
| RelativeSizeSquared, ShapeAndSize, ShearAndSize | | 1 | [0.5, 1], [0.2, 1], [0.2, 1] |

Closed forms used in tests: brick `1×2×4` (`EdgeRatio = 4`, `Stretch = sqrt(3/21)`, `Condition = sqrt21·sqrt(1+1/4+1/16)/3`, `Oddy = 126/16`, `Shape = 12/21`); 45° sheared cube (`Skew = ScaledJacobian = 1/sqrt2`, dihedrals 45°/135°); cube with top face scaled by `t` (`Taper = 2(1-t)/min(2(1+t), 4)`, frustum volume `(1+t+t²)/3`).

### Wedge (`Wedge`) and Pyramid (`Pyramid`)

Same corner/weighted-matrix machinery with the corner tables and ideal `W` stated above (pyramid: the four base corners only; the apex has no canonical frame, and a collapsed apex shows up as a zero base-corner determinant). Defined: `Volume`, `Jacobian`, `ScaledJacobian` (`× 2/sqrt3` for the wedge, `× sqrt2` for the pyramid so the ideal element scores 1; values above 1 occur for pyramids taller than the ideal), `Condition`, `Shape`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `EdgeRatio`, `Min/MaxAngle` (dihedral), `RelativeSizeSquared`, `ShapeAndSize`; wedge only: `Distortion` (6 Gauss points: 3 triangle points × 2). Verdict's published ranges for these are not recalled; none are claimed.

### Polyhedron (`Polyhedron`; generalization) and 3-D extras

Outward loops are taken from the signed cell-face list (`Right` side loops reversed), vertices deduplicated locally (`SmallVec`, no heap allocation up to 32 vertices / 16 faces).

| Metric | Definition |
|---|---|
| Volume | fan rule (identical to `sm_mesh_api::cell_volume`) |
| EdgeRatio | longest / shortest edge over all face edges |
| MinAngle / MaxAngle | interior dihedral over every edge shared by two faces of the cell: `180° - atan2((n_f × n_g).e, n_f . n_g)` (> 180° at a reflex edge); needs a closed cell |
| Sphericity (also tet/hex/wedge/pyramid) | Wadell `π^(1/3) (6V)^(2/3) / S`, `S` fan-rule surface area; 1 for a sphere, `(π/6)^(1/3) = 0.806` for a cube |
| Starness (also templates) | `min_f cos∠(A_f, apex_f - c)`, `c` = vertex mean; > 0 iff every face is visible from `c` (star-shaped) |

Verdict-only metrics (skew, taper, Jacobians, ...) return `Unsupported` for polyhedra. Corner-based Jacobians have no canonical frame on a general polyhedron; star-shapedness about the vertex mean is the validity criterion instead (see below).

## Validity

`validity(mesh, opts) -> Validity { status[], n_valid, n_degenerate, n_inverted, n_skipped }`, `tol = Options::degenerate_tol` (default `1e-12`, relative):

* template cells and polygons: `s = ScaledJacobian`; `s < -tol` → `Inverted`; `|s| <= tol` or non-finite → `Degenerate`; otherwise `Valid`, except that a negative `Volume` still gives `Inverted`. (Any corner with a negative Jacobian inverts the cell, including the hex centre sample and a reflex 2-D corner.)
* polyhedra: `V < -tol S^(3/2)` or `Starness < -tol` → `Inverted`; `V <= tol S^(3/2)` or `Starness <= tol` → `Degenerate`; else `Valid`.
* inactive cells → `Skipped`.

`Validity::inverted()` lists inverted cells in ascending order; `all_valid()` is true iff nothing is inverted or degenerate.

## Performance

SoA input (`coords()` slices read once; per cell one `cell_shape` and one `cell_corners` virtual call, then a `SmallVec<[V; 8]>` gather reused across the chunk), SoA output columns, `rayon` over fixed chunks. Release numbers on a 4-core container (criterion, `benches/metrics.rs`): hex scaled Jacobian 160 ns/element (kernel), 4.6 ms for 64 000 hexes (all four cores); five metrics over 64 000 hexes 40 ms; validity 10.8 ms; `summarize` of 10^6 values 2.5 ms.

## Tests

`elements.rs` (every applicable metric on every ideal shape equals `Metric::ideal`), `distorted.rs` (closed forms above: rectangle, sheared and trapezoid quads, warped quad, reflex quad, right triangle, n-gons, right-corner tet, brick, sheared and tapered hex, right wedge, tall pyramid, relative size, Verdict sentinels, inverted elements), `mesh.rs` (sm-testkit fixtures vs exact volumes and the mesh geometry rule, polyhedral prism, mixed shapes, inactive cells, 2-D meshes with each normal source, mirrored/collapsed cells), `stats.rs` (closed-form sequences, ties, signed zero, non-finite values, histogram, thread parity), `properties.rs` (proptest with the testkit's pinned seed: rigid-motion invariance of every metric, uniform scaling, mirroring with `CellShape::reflect_permutation`, range bounds), `determinism.rs` (1/2/3/4 threads, repeats, concurrent callers, all fixtures and an 8000-cell perturbed block).
