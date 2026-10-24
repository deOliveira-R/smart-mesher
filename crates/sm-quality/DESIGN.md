# sm-quality: design

## Positioning

**Similar to** Sandia's Verdict library (the metric engine behind Cubit, VTK's `vtkMeshQuality` and Exodus tooling), **differs in** operating on smart-mesh's face-first `MeshRead` contract (mixed tri/quad/tet/hex/wedge/pyramid *and* general polygons and polyhedra, dimension-2 surface meshes included) with parallel kernels whose reductions are bit-identical for any thread count, typed `Unsupported` errors instead of silent sentinels for undefined metrics, and a validity classifier instead of a bare sign test, **giving** one call that grades a whole mixed mesh (including OpenFOAM-style polyhedral cells) reproducibly across machines and thread counts, which is what a reproducible timeline of meshing operations needs (the same mesh hashes to the same quality report).

## Confidence in the formulas

The first version was reconstructed from memory of Verdict and Knupp's papers and checked analytically. A review then diffed every kernel against the Verdict C++ sources (`V_TriMetric.cpp`, `V_QuadMetric.cpp`, `V_TetMetric.cpp`, `V_HexMetric.cpp`, `V_WedgeMetric.cpp`, `V_PyramidMetric.cpp`) and the 2007 user manual (`VerdictUserManual2007`, in the `sandialabs/verdict` repository under `docs/`; it is not part of this repo), and fix round 0 aligned the crate with it. Status per group:

* **Verified against the Verdict sources** (by reading the C++ and, in fix round 1, by a differential run against a real Verdict build; pinned by `tests/verdict_ref.rs`): the metrics in the tables below **except** the ones listed under "Not Verdict-identical" next to this list. That covers hex `MaxAspectFrobenius`/`MeanAspectFrobenius`/`Condition`/`Oddy` (centre sample included, `DBL_MAX` for a non-positive determinant)/`Dimension`/relative size (mean corner determinant), hex `Distortion` on valid hexes (Gauss points and nodes); wedge `Shape`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `Condition`, `Distortion` (up to the constants in "Normalisations"); pyramid `ScaledJacobian`; pyramid `Shape` (since fix round 1: apex height along the corner-0 base normal, as Verdict; the largest-edge bug is not replicated); quad `Distortion` on planar convex quads; collapsed-quad routing (including `Shear`/`ShearAndSize` since round 1); quad warpage (`min cos^3`, 1 = flat; the manual's table lists ideal 0, the code is authoritative); the volume rule (Verdict triangulates hex/pyramid faces about the face mean, which is `smartmesh.fan_vertex_mean.v1`, so `Volume` equals Verdict's even for non-planar faces). `MeanAspectFrobenius` returns `DBL_MAX` as soon as any corner is degenerate (quad, hex, wedge), as Verdict does; it never averages the sentinel.
* **Not Verdict-identical** (deliberate or documented; full list under "Deviations from Verdict"): tet `MinAngle` (true minimum dihedral), tet `CollapseRatio` (Verdict's index slip not replicated), hex `EdgeRatio` (all 12 edges), pyramid `Shape` where the longest edge is one of edges 0-1/1-2 (about 25% of random pyramids), collapsed-quad `Distortion`, quad `MinAngle`/`MaxAngle` on tangled (self-intersecting, bowtie) quads (the crate uses `atan2` in `[0, 360)`, Verdict unsigned `acos`; simple reflex quads match), the tet inverted-element `DBL_MAX` sentinels (`AspectRatio`, `AspectGamma`, `RadiusRatio`, `AspectFrobenius`, `Condition`; Verdict returns finite or signed values), hex/quad/wedge behaviour on inverted or zero-volume elements, the degenerate-input residue listed under "Deviations from Verdict", and the quad metrics on non-planar quads (`MinAngle` can differ; `MaxAspectFrobenius`, `MeanAspectFrobenius`, `Distortion` do).
* **Normalisations:** the factors that make the ideal element score 1 (tri `2/sqrt3`, tet `sqrt2`, wedge `2/sqrt3`, pyramid `sqrt2` on top of the tet value) follow Verdict. Verdict divides the wedge Frobenius aspect by the rounded literal 1.16477; the crate uses the exact value `4 / (3 * 1.5^(1/3)) = 1.1647740...` so the ideal wedge scores exactly 1. Every wedge `MaxAspectFrobenius`/`MeanAspectFrobenius`/`Condition` value therefore differs from Verdict by a constant factor of 3.4e-6 (confirmed against a Verdict build). Likewise wedge `Distortion` uses the exact factor 1/2 where Verdict uses the literals 0.433013/0.866025 (a 1.2e-6 relative difference).
* **Advisory ranges.** The "Accept" columns are the manual's acceptable ranges where the manual gives one (the entries corrected in round 0: quad `AspectRatio`, quad Max/Mean `AspectFrobenius`, quad `ScaledJacobian`, quad `Warpage`, hex `Taper`, hex `EdgeRatio` (none), tet `MinAngle`, tri `ShapeAndSize`). All ranges were checked against the manual tables in fix round 1; wedge ranges in the Verdict sources are `Jacobian [0, DBL_MAX]`, `Distortion [0.5, 1]`, `Shape [0.3, 1]`. There is no thresholds API yet (see Known gaps).
* **smart-mesh extensions** (not Verdict metrics, no ranges claimed): see "Extensions" below.

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
* **Volume** is always the mesh geometry rule `smartmesh.fan_vertex_mean.v1` (fan polyhedron, `V = 1/3 sum A_f . (apex_f - o)`), so the `Volume` metric agrees with `sm_mesh_api::cell_volume` to rounding (tested against it, 1e-12). It equals the Verdict volume, also for non-planar faces (both triangulate faces about the face mean). In 2-D, "volume" is the *signed* area `n . A` (positive when the cycle is counter-clockwise about `n`).
* **Reference normal (2-D cells).** A zero or non-finite `Uniform` normal, or a zero/non-finite `PerCell` normal on a 2-D cell, is rejected with `QualityError::BadArgument`; a zero/non-finite `Field` normal is treated as absent (falls back to `Own`). `Options::normals`: `Own` (default; the Newell normal of the cell, so a planar cell is never inverted but a reflex corner has a negative Jacobian), `Uniform(n)`, `PerCell(&[n])`, or `Field(&dyn FaceNormals)` (the oriented normal of the model face the cell is classified on, at the cell's vertex mean: the shell orientation rule of `sm-mesh-api`).
* **Degenerate input** follows Verdict except for the deliberate deviations listed in "Deviations from Verdict" below: metrics where large is bad (`AspectRatio`, `EdgeRatio`, `Condition`, aspect Frobenius, `RadiusRatio`, `Taper`, `Oddy`, `Skew`, `MaxEdgeRatio`) return at most `DBL_MAX = 1e30` for degenerate or inverted elements; metrics where small is bad (`Shape`, `RelativeSizeSquared`, `Shear`, `Diagonal`, `Sphericity`, `Warpage` of a collapsed corner) return `0`; signed metrics (`Jacobian`, `ScaledJacobian`, `Volume`, `Distortion`) return their signed value (negative = inverted). Non-finite coordinates give `NaN`. The threshold for "degenerate denominator" is `TINY = 1e-30`, applied **relative to the element size**: `eval` (and the polyhedron path) first divides the coordinates by `2^k`, the largest power of two not above the largest coordinate extent from the first corner (`vec::Scale`), so the characteristic length is in `[1, 2)` and the kernels' absolute guard acts on `length^d` of order 1. The division is exact, so dimensionless metrics are bit-identical for an element and any power-of-two multiple of it (and agree to rounding for other scales), and a unit cube scaled by `1e-12` or `1e12` scores `Condition = ScaledJacobian = 1`. `Volume`, `Jacobian` (`length^d`) and `Dimension` (`length`) are scaled back by the same exact factor; the reference size of the relative-size metrics is scaled down likewise. Coincident or non-finite points leave the scale at 1. This is not Verdict's `VERDICT_DBL_MIN` (2.22e-15, `verdict.h`), which Verdict applies after its own per-element scaling in some kernels only, so degenerate-input values can differ (see the deviations list). `tests/properties.rs` checks scaling over `1e-12 .. 1e12`.
* **Undefined metrics.** Element level: `eval` returns `None`. Mesh level: cells whose shape does not support the metric get `NaN`, which summaries count in `undefined` and exclude from min/max/mean; if **no** cell of the mesh supports the metric, `compute*` returns `QualityError::Unsupported`. (A mixed mesh therefore reports `Skew` for its hexes and `NaN` elsewhere; a pure tet mesh asking for `Skew` is an error.)
* **Inactive cells** (`cell_active == false`, refined parents) are skipped by default (`NaN`, `Skipped`, excluded from the reference size); `Options::with_inactive(true)` includes them.
* **High-order cells** are evaluated on their corner (vertex) skeleton only. Curved-element Jacobian checks are future work.
* **Reference size** for relative-size metrics: `ReferenceSize::MeanAll` (default; Verdict's average element size: mean of the finite positive volumes of the counted cells), `MeanPerShape`, or `Fixed(v)`. `RelativeSizeSquared = min(V/V_ref, V_ref/V)^2`, `0` for `V <= 0`.
* **Determinism.** Per-cell values are written to their own slot (no cross-thread accumulation). Reductions use chunks of `CHUNK = 4096` values, independent of the thread count: each chunk is reduced sequentially with Neumaier compensated sums, chunk results are collected in order and merged sequentially. Min/max use `f64::total_cmp` and the lowest index on ties (so `-0.0 < +0.0` is deterministic). Std-dev is two-pass (mean, then `sum (x-mean)^2`), population form. Histogram bins are integer counts with a fixed `[lo, hi]`. `tests/determinism.rs` hashes every column, summary, histogram and the validity counts for 1, 2, 3 and 4 threads, repeated runs and concurrent runs (testkit `determinism::check`).

## Element metrics: formulas, ideal values, ranges

`ℓ` edge lengths, `A` area, `V` volume. "Accept" is the Verdict manual's acceptable range where the manual gives one (all ranges were checked against the 2007 manual tables). Ideal = value on the unit square/cube, equilateral triangle, regular tetrahedron, regular wedge (unit edges), regular pyramid; pinned by `Metric::ideal` and `tests/elements.rs`.

### Triangle (`Tri`)

Corner `k`: `J_k = (a x b) . n`. Weight `W = [[1, 1/2], [0, sqrt3/2]]`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume (area) | `n . (sum (p_i - p_0) x (p_{i+1} - p_0)) / 2` | `sqrt3/4` (unit edge) | > 0 |
| Jacobian | `min_k J_k` (= `2A`; smart-mesh extension, Verdict has no triangle Jacobian) | n/a | > 0 |
| ScaledJacobian | `(2/sqrt3) min_k J_k / (|a||b|)` | 1 | [0.5, 2sqrt3/3 = 1.1547] |
| AspectRatio | `ℓmax (ℓ0+ℓ1+ℓ2) / (4 sqrt3 A)` | 1 | [1, 1.3] |
| AspectFrobenius = Condition | `(ℓ0²+ℓ1²+ℓ2²) / (4 sqrt3 A)` (= `||T||_F ||T^-1||_F / 2`) | 1 | [1, 1.3] |
| Shape | `4 sqrt3 A / sum ℓ²` (= 1/Condition) | 1 | [0.25, 1] |
| RadiusRatio | `R / (2r) = ℓ0 ℓ1 ℓ2 (ℓ0+ℓ1+ℓ2) / (16 A²)` | 1 | [1, 3] |
| EdgeRatio | `ℓmax / ℓmin` | 1 | [1, 1.3] |
| MinAngle / MaxAngle | interior angles, degrees, `atan2((a x b).n, a.b)` in `[0, 360)` | 60 | [30, 60] / [60, 90] |
| Distortion | 1 if `A > 0`, -1 if `A < 0`, else 0 (the Jacobian of a linear triangle is constant) | 1 | [0.5, 1] |
| RelativeSizeSquared, ShapeAndSize | `min(τ,1/τ)²`, `· Shape` with `τ = A / A_ref` | 1 | [0.25, 1], [0.25, 1] |

### Quadrilateral (`Quad`)

Corners `k = 0..3`, `J_k` as above (`n` = Newell normal or reference normal). Principal axes `X1 = (p1-p0)+(p2-p3)`, `X2 = (p2-p1)+(p3-p0)`, cross derivative `X12 = p0-p1+p2-p3`. Weight: identity.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume (area) | signed Newell area (`= ¼ sum J_k`) | 1 | > 0 |
| Jacobian | `min_k J_k` | n/a | > 0 |
| ScaledJacobian | `min_k J_k / (|a_k||b_k|)` (sine of the corner angle); collapsed quad (`p3 == p2`): triangle formula | 1 | [0.3, 1] |
| AspectRatio | `ℓmax P / (4A)`, `P` perimeter | 1 | [1, 1.3] |
| MaxEdgeRatio | `max(|X1|/|X2|, |X2|/|X1|)` | 1 | [1, 1.3] |
| EdgeRatio | `ℓmax/ℓmin` (Verdict `quad_edge_ratio`) | 1 | [1, 1.3] |
| Skew | `|X1 . X2| / (|X1||X2|)` (0 if an axis has zero length, as Verdict) | 0 | [0, 0.5] |
| Taper | `|X12| / min(|X1|, |X2|)` | 0 | [0, 0.7] |
| Warpage | `min((N0.N2)³, (N1.N3)³)`, `N_k` = unit `a_k x b_k` | 1 (see note) | [0.3, 1] |
| Stretch | `sqrt2 ℓmin / dmax` (`dmax` longest diagonal) | 1 | [0.25, 1] |
| Condition | `max_k (|a|²+|b|²) / (2 J_k)` | 1 | [1, 4] |
| MaxAspectFrobenius / MeanAspectFrobenius | max / mean over corners of `(|a|²+|b|²)/(2J_k)` (equal to Condition for the max) | 1 | [1, 1.3] |
| Shape | `min_k 2 J_k / (|a|²+|b|²)` | 1 | [0.3, 1] |
| Shear | `max(0, ScaledJacobian)`; a collapsed quad uses the triangle `ScaledJacobian` | 1 | [0.3, 1] |
| Oddy | `max_k ((G11-G22)² + 4 G12²) / (2 det G)`, `G = [a b]^T [a b]` | 0 | [0, 0.5] |
| Distortion | `min detJ / |mean detJ|`, min over the 2×2 Gauss points **and the four nodes** (`detJ_k = J_k/4`), mean over the Gauss points; collapsed quad: triangle value (1; Verdict returns 2, see deviations). Planar convex quads match Verdict; non-planar, reflex and zero-area quads do not (see deviations) | 1 | [0.5, 1] |
| MinAngle / MaxAngle | corner angles incl. reflex (`> 180`) | 90 | [45, 90] / [90, 135] |
| RelativeSizeSquared, ShapeAndSize, ShearAndSize | as the triangle, `· Shape` / `· Shear` | 1 | [0.3, 1], [0.2, 1], [0.2, 1] |

Warpage note: cosine cubed, 1 = flat, as computed by the Verdict source. The manual lists the ideal as 0; the code is authoritative. `1 - w` gives the "0 = flat" convention.

### Polygon (`Polygon`, 5+ vertices; generalization)

`Volume` (signed area), `Jacobian`, `ScaledJacobian` (min over corners of `J_k/(|a||b|)`, negative at a reflex corner), `EdgeRatio`, `MinAngle`, `MaxAngle` as the quad, and `AspectRatio = ℓmax P / (4 tan(π/n) A)` (the triangle and quad formulas are the `n = 3, 4` cases; a regular n-gon scores 1). No Verdict counterpart; ideal-element metrics (Condition, Shape, ...) are not defined for polygons.

### Tetrahedron (`Tet`)

`a, b, c = p1-p0, p2-p0, p3-p0`, `det = a . (b x c) = 6V`. `T = A W^-1` with the regular-tetrahedron `W` (corner 0 suffices: a tet is affine). Face areas `A_i`, `ΣA = sum A_i`. Circumradius `R = |a²(b×c) + b²(c×a) + c²(a×b)| / (2 det)`, inradius `r = det / (2 ΣA)`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume | fan rule (`= det/6`) | `sqrt2/12` (unit edge) | > 0 |
| Jacobian | `det` | n/a | > 0 |
| ScaledJacobian | `sqrt2 det / max over the 4 corners of ℓℓℓ` | 1 | [0.5, 1] |
| AspectRatio (Verdict `tet_aspect_ratio`; Verdict's aspect "beta" is `RadiusRatio`) | `ℓmax ΣA / (6 sqrt6 V)` | 1 | [1, 3] |
| AspectFrobenius | `||T||_F² / (3 det(T)^(2/3))` (= 1/Shape) | 1 | [1, 1.3] |
| AspectGamma | `(Σℓ²/6)^(3/2) sqrt2 / (12 V)` | 1 | [1, 3] |
| RadiusRatio | `R / (3r)` | 1 | [1, 3] |
| CollapseRatio | `min_i h_i / ℓmax(face opposite i)`, `h_i = det/(2 A_opp)` | `sqrt(2/3)` | [0.1, ∞) |
| Condition | `||T||_F ||T^-1||_F / 3` | 1 | [1, 3] |
| Shape | `3 det(T)^(2/3) / ||T||_F²` | 1 | [0.3, 1] |
| EdgeRatio | `ℓmax/ℓmin` | 1 | [1, 3] |
| MinAngle / MaxAngle | min / max interior **dihedral** angle (degrees). `MinAngle` is the true minimum dihedral, **not** Verdict's value (see deviations); `MaxAngle` is an extension. Edges next to a zero-area face are skipped; if no valid edge remains (collapsed tet) the result is `MinAngle = 0`, `MaxAngle = 180` | 70.5288 | MinAngle [40, 70.53] |
| Distortion | sign of `det` (constant Jacobian; Verdict returns a constant 1, see deviations) | 1 | [0.5, 1] |
| RelativeSizeSquared, ShapeAndSize | | 1 | [0.3, 1], [0.2, 1] |

Right-corner tet (`(0,0,0),(1,0,0),(0,1,0),(0,0,1)`) closed forms used in tests: `ScaledJacobian = 1/sqrt2`, `AspectRatio = RadiusRatio = (1+sqrt3)/2`, `Shape = 2^(1/3)·2/3`, `Condition = sqrt(3/2)`, dihedrals `{90°, acos(1/sqrt3) = 54.7356°}`, `CollapseRatio = 1/sqrt6`.

### Hexahedron (`Hex`)

Corner table `[n, a, b, c]` (verified: all `+1` on the unit cube) and principal axes `X1 = Σ edges along ξ` (4 edges), `X2`, `X3`; cross derivatives `X12`, `X13`, `X23` (sums of the second differences along the three face pairs); diagonals `0-6, 1-7, 2-4, 3-5`.

| Metric | Formula | Ideal | Accept |
|---|---|---|---|
| Volume | fan rule | 1 | > 0 |
| Jacobian | `min(min_k det A_k, X1.(X2×X3)/64)` | n/a | > 0 |
| ScaledJacobian | `min(min_k det A_k/(|a||b||c|), X1.(X2×X3)/(|X1||X2||X3|))` | 1 | [0.5, 1] |
| Condition | `max_k ||A_k||_F ||adj A_k||_F / (3 det A_k)` | 1 | [1, 3] |
| MaxAspectFrobenius / MeanAspectFrobenius | max / mean over the 8 corners of the same per-corner `condition_comp / 3` (so `MaxAspectFrobenius = Condition`, as Verdict) | 1 | [1, 3] |
| Shape | `min_k 3 det A_k^(2/3) / ||A_k||_F²` | 1 | [0.3, 1] |
| Oddy | `max` over the **centre sample** (principal axes `X1,X2,X3`) and the 8 corners of `(||G||_F² - tr(G)²/3) / det(A)^(4/3)`, `G = A^T A`; `DBL_MAX` if any `det <= TINY` (an inverted hex is never "perfect") | 0 | [0, 0.5] |
| Skew | `max_{i<j} |X_i.X_j|/(|X_i||X_j|)` | 0 | [0, 0.5] |
| Taper | `max(|X12|/min(|X1|,|X2|), |X13|/min(|X1|,|X3|), |X23|/min(|X2|,|X3|))` | 0 | [0, 0.5] |
| Stretch | `sqrt3 ℓmin / dmax` | 1 | [0.25, 1] |
| Diagonal | `dmin / dmax` | 1 | [0.65, 1] |
| Dimension | Pronto characteristic length `sqrt(0.5 V² / sum_i |dV/dx_i|²)` (nodal volume gradients, 2×2×2 Gauss) | `sqrt3/3` (unit cube) | |
| MaxEdgeRatio | `max_{i,j} |X_i|/|X_j|` | 1 | [1, 1.3] |
| EdgeRatio | `ℓmax/ℓmin` over all 12 edges. Verdict's maximum covers edges 0-5 only (index bug), so a 1x1x3 brick reads 1 in Verdict and 3 here | 1 | none in the manual |
| Shear | `max(0, min_k det A_k/(|a||b||c|))` (corners only) | 1 | [0.3, 1] |
| Distortion | `min detJ / |mean detJ|`, min over the 2×2×2 Gauss points **and the 8 nodes** (`detJ_k = det A_k / 8`), mean over the Gauss points. Divides by `|mean|`: a mirrored cube reads -1 (Verdict +1); zero volume reads 0 (Verdict `DBL_MAX`) | 1 | [0.5, 1] |
| Min/MaxAngle | min/max interior dihedral angle over the 12 edges | 90 | |
| RelativeSizeSquared, ShapeAndSize, ShearAndSize | `τ = (sum of the 8 corner det A_k) / (8 V_ref)`, i.e. the **mean corner determinant**, not the cell volume | 1 | [0.5, 1], [0.2, 1], [0.2, 1] |

Closed forms used in tests: brick `1×2×4` (`EdgeRatio = 4`, `Stretch = sqrt(3/21)`, `Condition = sqrt21·sqrt(1+1/4+1/16)/3`, `Oddy = 126/16`, `Shape = 12/21`, `Dimension = 8/sqrt84`); brick `1×1×2` (`MaxAspectFrobenius = Condition = sqrt(3/2)`); 45° sheared cube (`Skew = ScaledJacobian = 1/sqrt2`, dihedrals 45°/135°); cube with top face scaled by `t` (`Taper = 2(1-t)/min(2(1+t), 4)`, frustum volume `(1+t+t²)/3`).

### Wedge (`Wedge`) and Pyramid (`Pyramid`)

Corner tables as in `solid.rs` (wedge: six corners; pyramid: the four base-corner frames are used for `Jacobian` and the smart-mesh extensions).

| Shape | Metric | Formula | Ideal | Accept |
|---|---|---|---|---|
| Wedge | Volume, Jacobian | fan rule; min corner determinant | n/a | Jacobian [0, DBL_MAX] |
| Wedge | ScaledJacobian | `(2/sqrt3) min_k det/(|v1||v2||v3|)` | 1 | |
| Wedge | Shape | `min(1, min_k 3 (det_k 2/sqrt3)^(2/3) / (|v1|²+|v2|²+|v3|²))` on raw corner edge vectors; 0 if any `det <= TINY` | 1 | [0.3, 1] |
| Wedge | MaxAspectFrobenius, MeanAspectFrobenius, Condition | tet-style Frobenius aspect (regular-tet weight) of the six corner tets `0123, 1204, 2015, 3540, 4351, 5432`, divided by the exact ideal-wedge value `4/(3 * 1.5^(1/3))` (Verdict: the literal 1.16477), floored at 1; max / mean (sum/6); `DBL_MAX` if any corner sub-tet is inverted (Verdict squares the determinant and stays finite); `Condition = MaxAspectFrobenius` | 1 | |
| Wedge | Distortion | `min_k det_k / (2 |V|)` over the six corner determinants (exact factor 1/2; Verdict 0.433013/0.866025) | 1 | [0.5, 1] |
| Pyramid | Volume, Jacobian | fan rule; min over the four base-corner frames (= the four sub-tets) | n/a | |
| Pyramid | ScaledJacobian | `sqrt2 * min` of the tet scaled Jacobian over sub-tets `0124, 0234, 0134, 1234` (the apex corner is included through the tet's longest-edge product), floored at 0, folded (`x > 1` becomes `2 - x`): values in `[0, 1]`. An inverted pyramid reads 0 (validity uses the unfloored value) | 1 | |
| Pyramid | Shape | `quad_shape(base) * cos(apex offset angle) * min(d/L, L/d)`, `d` the apex height over the base plane measured along the corner-0 normal `(p1-p0) x (p3-p0)` (Verdict's `distance_point_to_pyramid_base`), `quad_shape` with the base's own Newell normal, `L = (sqrt2/2) * longest edge`; 0 if `d <= 0` or `cos <= 0`. Verdict's `largest_pyramid_edge` seeds its maximum with `min(l0, l1)`, dropping the longer of edges 0-1 and 1-2; this bug is not replicated (see deviations) | 1 | |

## Extensions

The following are smart-mesh extensions that have no Verdict counterpart for the shape; they use the weighted-Knupp machinery (`T = A W^-1`, ideal `W`: regular tet, equilateral-base wedge with height = edge, square pyramid with equal edges) or generic geometry, and no acceptable ranges are claimed: triangle `Jacobian` (`2A`); pyramid `Condition`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `EdgeRatio`, `RelativeSizeSquared`, `ShapeAndSize`, `Min/MaxAngle`; wedge `RelativeSizeSquared`, `ShapeAndSize`; tet `MaxAngle`; hex and wedge `Min/MaxAngle`; `Sphericity`; `Starness`; polygon and polyhedron metrics (below).

## Deviations from Verdict

Deliberate, and kept because the crate's choice is the safer or more informative one:

* **Signed values on inverted elements.** Tet `AspectRatio`, `AspectGamma`, `RadiusRatio`, `AspectFrobenius`, `Condition` return `DBL_MAX` for `det <= TINY`; Verdict uses `|det|`/`det^2` (finite positive) or a signed determinant (negative `Condition`). Tri/tet `Distortion` returns the sign (+1/0/-1); Verdict returns a constant 1.0 for linear elements. Quad `Max/MeanAspectFrobenius` use the signed corner `J` (`DBL_MAX` at a reflex or inverted corner); Verdict uses `|a x b|`, which would score an inverted quad as fine. Wedge `Distortion` divides by `|V|` so an inverted wedge reads negative instead of positive.
* **Zero-length inputs.** Tet `ScaledJacobian` (zero edge product) and hex `ScaledJacobian` (zero edge/axis) return 0; Verdict returns `DBL_MAX`. Hex `Diagonal` with a zero maximal diagonal returns 0; Verdict returns `DBL_MAX` via `safe_ratio`. Tri `MinAngle`/`MaxAngle` with a zero-length side and quad `MinAngle`/`MaxAngle` with a zero edge use `atan2` (0 for the zero vector); Verdict returns 0 (tri) and 360/0 (quad). Hex `Skew` returns `DBL_MAX` for a zero axis (as Verdict); quad `Skew` returns 0 (as Verdict).
* **Quad `MinAngle`/`MaxAngle` on tangled quads.** Verdict uses unsigned `acos` angles (and `360 - max` when a signed corner area is negative); the crate computes each corner angle with `atan2` in `[0, 360)`. They agree on simple reflex quads and differ on tangled (self-intersecting, bowtie) quads.
* **Tet `CollapseRatio`.** Verdict's face-edge maxima use `l13` for face 012 and `l12` for face 031 (an index slip in `tet_collapse_ratio`); the crate uses the geometrically correct edges of each face. They agree on symmetric tets and differ on generic ones.
* **Tet `MinAngle`** is the true minimum dihedral. Verdict's `tet_minimum_angle` builds its face normals (`ab x bc`, `ab x ad`, `ad x cd`, `bc x cd`) with inconsistent orientation, so some edges report `180 - theta`. Tet `(0,0,0),(1,0,0),(0.2,0.3,0),(0.4,0.1,0.9)`: crate 24.69 (checked independently), Verdict 60.37; Verdict even returns values no tet can reach (70.89 > 70.53). About 1 in 4 random tets differ. An inverted tet reads reflex in the crate (270) where Verdict gives 54.7.
* **Hex `EdgeRatio`.** Verdict's `hex_edge_ratio` computes `M2` over all 12 edges and then overwrites it with `max(Mab, Mcd, Mef)`, so its maximum covers edges 0-5 only while its minimum covers all 12 (an index bug). A 1x1x3 brick reads 1 in Verdict, 3 in the crate (the geometrically correct value); about half of random hexes differ.
* **Pyramid `Shape` largest edge.** Verdict's `largest_pyramid_edge` starts from `fmin(l0, l1)`, dropping the longer of edges 0-1 and 1-2 (a Verdict bug). Base `(0,0,0),(3,0,0),(2.5,1,0),(0.5,1,0)`, apex `(1.5,0.5,0.8)`: Verdict 0.3311, crate 0.2208. Not replicated. This affects about 25% of random pyramids (those whose longest edge is edge 0-1 or 1-2, which `largest_pyramid_edge` drops through its `fmin(l0, l1)` seed). All other parts of pyramid `Shape` match (apex height along the corner-0 normal, base shape with its own normal).
* **Collapsed-quad `Distortion`.** Verdict's `quad_distortion` returns 2.0 for a collapsed quad (its collapsed branch divides `2A` by `A`); the crate routes to the triangle value, 1.0. The other collapsed-quad routings (`Jacobian`, `ScaledJacobian`, `Shear`, `ShearAndSize`, `Condition`, `MinAngle`, `MaxAngle`) match Verdict.
* **Hex `Distortion`** divides by `|mean detJ|`, so a mirrored cube reads -1 where Verdict (signed Gauss volume) reads +1; zero volume reads 0 where Verdict returns `DBL_MAX`. **Quad `Distortion`** also uses `|mean|` where Verdict uses the unsigned sum of `|N|`; a zero-area quad reads 0 (Verdict -2) and a reflex quad -2 (Verdict -1.464, which takes the area from `|J|` and evaluates against node normals). Wedge `Distortion` was already listed above.
* **Wedge `Max/MeanAspectFrobenius`/`Condition` on an inverted corner sub-tet** return `DBL_MAX` (the corner uses the signed determinant). Verdict's `tet_aspect_frobenius` uses `det^2`, so a mirrored wedge scores about 1.0 there.
* **Non-planar (warped) quads.** `MinAngle` can differ from Verdict on a non-planar quad (`MaxAngle` matched on the probes), and `MaxAspectFrobenius`, `MeanAspectFrobenius` and `Distortion` differ on every non-planar quad (planar ones match). The crate projects onto the reference normal: signed `J = (a x b) . n` and `atan2` against `n`. Verdict uses `|a x b|` (unsigned, unprojected) and true 3-D `acos` corner angles, and for `Distortion` switches to a shell model with a thickness and two through-thickness Gauss points when the element is not flat. Quad `(0,0,0),(1,0,0),(1,1,0.5),(0,1,0)`: `MinAngle` Verdict 78.463, crate 78.023; `Distortion` Verdict 0.9047, crate 0.8889; differences up to about 1% on the warped cases tested. This is the common case in surface meshes.
* **More degenerate-input conventions.** Tet `CollapseRatio` with a zero-length edge: crate 0, Verdict `DBL_MAX`. Wedge `ScaledJacobian` with a zero-length edge: crate 0, Verdict 0.577 (its NaN corners are silently skipped by `fmin`).
* **Remaining degenerate-input residue** (values on exactly degenerate elements; not pinned to Verdict): coincident hex `Dimension` is 0 (Verdict `NaN`); coincident quad `AspectRatio` is `+1e30` (Verdict `-1e30`); degenerate pyramid `ScaledJacobian` is 0 (Verdict `-1.41e30`); zero wedge `ScaledJacobian` is 0 (Verdict `-1e30`). Probed with `eval()` on all-coincident inputs and on partly collapsed variants (flat pyramid, flat wedge, quad with 3 coincident corners, hex with collapsed top).
* **Wedge Frobenius normaliser** is the exact ideal value, not the rounded 1.16477 (see Confidence); wedge `Distortion` likewise uses the exact 1/2.
* **Summary statistics** (`summarize`) treat the `DBL_MAX = 1e30` sentinel as a finite value: one degenerate cell turns `mean` and `std_dev` into about `1e30/N`. Use `min`/`max`/`argmax` to find degenerate cells, or filter the column (`x < DBL_MAX`) before summarising.
* **Collapsed quads** (`p3 == p2`) are routed to the triangle formulas for `Jacobian`, `ScaledJacobian`, `Shear` (`max(0, tri SJ)`), `ShearAndSize`, `Condition`, `MinAngle`, `MaxAngle`, `Distortion`; as Verdict does for all of these except `Distortion` (see above); other quad metrics (`Shape`, `AspectRatio`, ...) are evaluated on the degenerate quad as is.

### Polyhedron (`Polyhedron`; generalization) and 3-D extras

Outward loops are taken from the signed cell-face list (`Right` side loops reversed), vertices deduplicated locally (`SmallVec`, no heap allocation up to 32 vertices / 16 faces).

| Metric | Definition |
|---|---|
| Volume | fan rule (identical to `sm_mesh_api::cell_volume`) |
| EdgeRatio | longest / shortest edge over all face edges |
| MinAngle / MaxAngle | interior dihedral over every edge shared by two faces of the cell: `180° - atan2((n_f × n_g).e, n_f . n_g)` (> 180° at a reflex edge); needs a closed cell. Edges next to a zero-area face are skipped; a cell with no valid edge (flat hex, collapsed tet) reads `MinAngle = 0`, `MaxAngle = 180` so it shows up as a worst cell |
| Sphericity (also tet/hex/wedge/pyramid) | Wadell `π^(1/3) (6V)^(2/3) / S`, `S` fan-rule surface area; 1 for a sphere, `(π/6)^(1/3) = 0.806` for a cube |
| Starness (also templates) | `min_f cos∠(A_f, apex_f - c)`, `c` = vertex mean; > 0 iff every face is visible from `c` (star-shaped) |

Verdict-only metrics (skew, taper, Jacobians, ...) return `Unsupported` for polyhedra. Corner-based Jacobians have no canonical frame on a general polyhedron; star-shapedness about the vertex mean is the validity criterion instead (see below).

## Validity

`validity(mesh, opts) -> Validity { status[], n_valid, n_degenerate, n_inverted, n_skipped }`, `tol = Options::degenerate_tol` (default `1e-12`, relative):

* template cells and polygons: `s = ScaledJacobian` (pyramids: the unfloored, unfolded value, since the reported one is floored at 0); `s < -tol` → `Inverted`; `|s| <= tol` or non-finite → `Degenerate`; otherwise `Valid`, except that a negative `Volume` still gives `Inverted`. (Any corner with a negative Jacobian inverts the cell, including the hex centre sample and a reflex 2-D corner.)
* polyhedra: `V < -tol S^(3/2)` or `Starness < -tol` → `Inverted`; `V <= tol S^(3/2)` or `Starness <= tol` → `Degenerate`; else `Valid`.
* inactive cells → `Skipped`.

`Validity::inverted()` lists inverted cells in ascending order; `all_valid()` is true iff nothing is inverted or degenerate.

## Known gaps

* **Missing Verdict metrics:** `*_equiangle_skew` (all shapes), `tri_normalized_inradius`, `quad_radius_ratio`, `tet_mean_ratio`, `tet_normalized_inradius`, `tet_inradius`, `tet_squish_index`, `tet_equivolume_skew`, `hex_nodal_jacobian_ratio`, hex/tet `timestep`, `wedge_max_stretch` (`Stretch` is not applied to wedges). PARITY lists "full" as P1.
* **No thresholds API.** The "Accept" columns are documentation only; PARITY asks for thresholds.
* High-order cells are evaluated on the corner skeleton; no curved-element Jacobian sampling.
* **Degenerate threshold** is `TINY = 1e-30` on power-of-two-normalised coordinates, not Verdict's `VERDICT_DBL_MIN`; values on exactly degenerate elements are not all Verdict-identical (see the deviations list).

## Performance

SoA input (`coords()` slices read once; per cell one `cell_shape` and one `cell_corners` virtual call, then a `SmallVec<[V; 8]>` gather reused across the chunk), SoA output columns, `rayon` over fixed chunks. Polyhedron cells gather their face loops once per cell (validity: once for volume, surface area and starness), the dihedral twin search uses a sorted directed-edge map (`O(E log E)`), and the shapes present in the mesh are scanned once per `compute*` call. `prepare()` skips the Volume column for `ReferenceSize::Fixed`, and the `SurfaceNormals::Field` lookup (classification, carrier normal) runs for 2-D cells only. Release numbers on a 4-core container (measured before round 0) (criterion, `benches/metrics.rs`): hex scaled Jacobian 160 ns/element (kernel), 4.6 ms for 64 000 hexes (all four cores); five metrics over 64 000 hexes 40 ms; validity 10.8 ms; `summarize` of 10^6 values 2.5 ms.

## Tests

`elements.rs` (every applicable metric on every ideal shape equals `Metric::ideal`), `distorted.rs` (closed forms above: rectangle, sheared and trapezoid quads, warped quad, reflex quad, right triangle, n-gons, right-corner tet, brick, sheared and tapered hex, right wedge, tall pyramid, relative size, Verdict sentinels, inverted elements), `mesh.rs` (sm-testkit fixtures vs exact volumes and the mesh geometry rule, polyhedral prism, mixed shapes, inactive cells, 2-D meshes with each normal source, mirrored/collapsed cells), `stats.rs` (closed-form sequences, ties, signed zero, non-finite values, histogram, thread parity), `properties.rs` (proptest with the testkit's pinned seed: rigid-motion invariance of every metric, uniform scaling, mirroring with `CellShape::reflect_permutation`, range bounds), `determinism.rs` (1/2/3/4 threads, repeats, concurrent callers, all fixtures and an 8000-cell perturbed block).
