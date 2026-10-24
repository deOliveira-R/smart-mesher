# W1 report: sm-quality

## Summary

`sm-quality` implements a large subset of the Verdict metric set (gaps listed in `DESIGN.md`) for every `CellShape` of `sm-mesh-api` (tri, quad, tet, hex, wedge, pyramid; polygons and polyhedra as documented generalizations), element-level (`eval`, pure, allocation free) and mesh-level (`compute`, `compute_many`, `report`, `validity`) through `&dyn MeshRead`, with deterministic parallel reductions (`summarize`, `histogram`). 30 metrics, shape-applicability table (`Metric::applies`), ideal values (`Metric::ideal`). Formulas, ranges and conventions are in `DESIGN.md`.

Implemented per the task: scaled Jacobian, Jacobian, aspect ratio (tri/quad perimeter form, tet `beta` and `gamma`, Frobenius, radius ratio), skew, taper, warpage, stretch, condition number, shape, relative size, shape-and-size, min/max angle (interior angles in 2-D, dihedral in 3-D), edge ratio, max edge ratio, volume/area, plus Oddy, shear, shear-and-size, diagonal, dimension, distortion, collapse ratio, sphericity and star-shapedness; validity (valid / degenerate / inverted / skipped); summary statistics and histograms.

**Verification status:** the first version was reconstructed from memory. A review then diffed it against the Verdict C++ sources and the 2007 manual, fix round 0 corrected the mismatches, and a differential run against a real Verdict build drove fix round 1 (below). Not everything is Verdict-identical: tet `MinAngle`, hex `EdgeRatio`, pyramid `Shape` when the longest edge is edge 0-1 or 1-2, collapsed-quad `Distortion`, hex/quad/wedge behaviour on inverted or zero-volume elements, and the quad `MinAngle`/`MaxAngle`/`Max,MeanAspectFrobenius`/`Distortion` on non-planar quads differ from Verdict, deliberately and documented in `DESIGN.md` ("Deviations from Verdict"). `tests/verdict_ref.rs` pins Verdict values for distorted elements (hand-computed in round 0; probe values from the Verdict build in round 1). Crate-only metrics are marked as extensions in `Metric`'s rustdoc and `DESIGN.md`.

## Fix round 0 (review findings)

Fixed to match Verdict:

* Hex `MaxAspectFrobenius`/`MeanAspectFrobenius` (per-corner `||A|| ||adj A|| / det / 3`, so `MaxAspectFrobenius = Condition`; 1x1x2 brick 1.2247), hex `Oddy` (centre sample; `DBL_MAX` for `det <= TINY`, so a mirrored cube is no longer "perfect"), hex `Dimension` (Pronto length, `sqrt3/3` on the unit cube), hex `RelativeSizeSquared`/`ShapeAndSize`/`ShearAndSize` (mean corner determinant).
* Quad and hex `Distortion` sample the nodes as well as the Gauss points (trapezoid 2/3); wedge `Distortion` = min corner det / (2V).
* Wedge `Shape`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `Condition` (right-isosceles wedge: 0.8254, 1.1357, 1.0978, 1.1357).
* Pyramid `ScaledJacobian` (min over four sub-tets, floored, folded to [0, 1]; tall pyramid 0.419) and `Shape` (0.75); `validity` classifies pyramids on the unfloored value so inversion is still detected.
* Collapsed quads (`p3 == p2`) route to the triangle formulas for Jacobian, ScaledJacobian, Condition, Min/MaxAngle, Distortion; quad `Skew` of a zero axis is 0.
* `histogram_auto` on a constant column of large magnitude (e.g. all `DBL_MAX`) no longer fails.
* Performance: polyhedron face loops gathered once per cell, sorted directed-edge map for dihedral twins (was `O(F^2 n^2)`), shape scan once per call.
* `DESIGN.md`: ranges corrected against the manual (quad AspectRatio [1,1.3], quad Max/MeanAF [1,1.3], quad SJ [0.3,1], quad Warpage [0.3,1], hex Taper [0,0.5], hex EdgeRatio none, tet MinAngle [40,70.53], tri ShapeAndSize [0.25,1]), the "low confidence" caveats removed for verified parts, extensions and deviations documented.

Documented as deliberate deviations rather than changed (see `DESIGN.md`): signed/inverted conventions of tet metrics, tri/tet `Distortion` sign, quad Max/MeanAF signed `J`, zero-length-edge conventions (tet/hex `ScaledJacobian`, hex `Diagonal`, tri/quad angles), quad `MaxAngle` on non-convex quads, tet `CollapseRatio` (Verdict's index slip not replicated), the `DBL_MAX` sentinel in summary statistics.

Not done: the missing Verdict metrics (equiangle skew, quad radius ratio, tet mean ratio / inradius / squish / equivolume skew, hex nodal Jacobian ratio, timestep, wedge max stretch), and a thresholds API; both are listed in Known gaps. The Verdict manual is not in this repo (it lives in `sandialabs/verdict` under `docs/`); in round 1 the ranges were checked against it by the reviewer and the `(unchecked)` tags were removed.

## Fix round 1 (differential review against a real Verdict build)

Fixed:

* `MeanAspectFrobenius` (quad, hex, wedge, also the tet/pyramid generic path) returns `DBL_MAX` when any corner is degenerate or inverted instead of averaging the 1e30 sentinel into e.g. 5e29.
* Collapsed quad (`p3 == p2`): `Shear` and `ShearAndSize` route to the triangle (`max(0, tri SJ)`); probe 0.9238 / 0.3608.
* Pyramid `Shape`: apex height along the corner-0 base normal `(p1-p0) x (p3-p0)` (Verdict's `distance_point_to_pyramid_base`), base quad shape with its own normal; non-planar-base probe now 0.8844 (was 0.8189). Verdict's `largest_pyramid_edge` bug (seed `fmin(l0, l1)`) is documented, not replicated (probe: Verdict 0.3311, crate 0.2208).
* `prepare()` rejects zero or non-finite `Uniform` normals and zero/non-finite `PerCell` normals of 2-D cells with `BadArgument`; a zero/non-finite `Field` normal is treated as absent (falls back to `Own`).
* Hex axes are computed lazily (only for the metrics that use them).
* Docs: `TINY` is not Verdict's `VERDICT_DBL_MIN` (2.22e-15, scaled per element); wedge Frobenius normaliser corrected to 1.1647740 (3.4e-6 relative to Verdict's literal) and wedge `Distortion` exact 1/2 (1.2e-6); `(unchecked)` tags removed; manual reference fixed; quad `EdgeRatio` is a Verdict metric ([1, 1.3]) and triangle `Jacobian` is an extension; `Metric` rustdoc updated.

Documented as deliberate deviations (new in `DESIGN.md`): tet `MinAngle` (true minimum dihedral; inverted tet reads reflex), hex `EdgeRatio` (all 12 edges), collapsed-quad `Distortion` (1 vs Verdict 2), hex/quad `Distortion` `|mean|` denominator and zero-volume value, quad reflex/zero-area `Distortion`, wedge Frobenius family on an inverted corner, non-planar quad `MinAngle`/`MaxAngle`/`Max,MeanAspectFrobenius`/`Distortion` (projection onto the reference normal, up to about 1%), tet `CollapseRatio` zero edge and wedge `ScaledJacobian` zero edge. The "Verified against the Verdict sources" claim in `DESIGN.md` was narrowed accordingly.

Skipped, with reasons:

* Scale-relative degenerate guard (SHOULD 1): documented only. Changing the guard touches every kernel and cannot be validated without the Verdict differential fixture.
* Real Verdict differential fixture generator (SHOULD 2/10): no Verdict build is available in this environment; instead `tests/verdict_ref.rs` gained pinned cases for every mismatch class (mean-AF sentinel, collapsed-quad shear, non-planar-base pyramid, pyramid edge quirk, hex edge ratio, perturbed tet `MinAngle`, collapsed-quad `Distortion`, mirrored/flat hex `Distortion`, mirrored wedge AF, warped quad), using the probe values from the review. Verdict-side numbers are quoted in comments; where the crate deliberately differs the test pins the crate value.
* `From<Point3>` convenience (SHOULD 11, optional): not added.
* Not matched (documented instead): non-planar quad formulations, quad reflex/zero-area `Distortion`, tet `MinAngle`, hex `EdgeRatio`, collapsed-quad `Distortion`.

## Test summary

All pass with default threads and with `RAYON_NUM_THREADS=1`: `cargo test -p sm-quality` (elements 2, distorted 18, verdict_ref 20, mesh 12, properties 5, stats 8, determinism 3; counts as of round 1), `cargo clippy -p sm-quality --all-targets --all-features -- -D warnings` clean, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-quality --no-deps` clean, `cargo build` clean. Analytic checks: every applicable metric on every ideal shape equals its ideal value (~70 checks); closed forms on distorted elements; fixture volumes vs exact values and vs `sm_mesh_api::cell_volume` (1e-12). Property tests use `sm-testkit` strategies with the pinned seed. Determinism test: bit hashes of all columns/summaries/histograms/validity for 1, 2, 3, 4 threads, repeated, concurrent, over all testkit fixtures and a perturbed 20^3 block. Bench: `benches/metrics.rs` (hex kernel ~160 ns, 64k-hex scaled Jacobian 4.6 ms on 4 cores).

## Deviations from the contracts' intent

* None of the frozen crates was touched. `Cargo.toml` of the crate gained `rayon`, `smallvec` and dev-deps `sm-testkit`, `proptest`, `criterion` (all on the allowed list); `sm-core` is a dependency (`Point3` in the `FaceNormals` call of `SurfaceNormals::Field`).
* `Cargo.lock` changes with the new dependencies; it is deliberately not committed (the orchestrator owns it and regenerates it at merge).
* Delivery branch: the task text names `claude/w1-sm-quality`, the session harness designates `claude/sm-quality-metrics-vmofo0`; the same commit is pushed to both (merge by SHA).
* `Volume` is always the mesh geometry rule (fan from the vertex mean), not Verdict's per-shape volume; they agree for planar-faced cells.
* 2-D cells need a reference normal to be *signed*; the contract provides none on `MeshRead`, so `Options::normals` takes `Own` / `Uniform` / `PerCell` / `Field(&dyn FaceNormals)` (the contract's shell-orientation trait).
* Undefined metrics are `NaN` per cell (excluded from stats) and `Unsupported` only when no cell supports the metric, rather than an error for any mixed-mesh mismatch.
* Pyramid `Jacobian` uses the four base-corner frames (equal to Verdict's four sub-tets); `ScaledJacobian` and `Shape` follow Verdict. Pyramid `Condition`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `EdgeRatio`, `RelativeSizeSquared`, `ShapeAndSize` and wedge `RelativeSizeSquared`/`ShapeAndSize` are smart-mesh extensions.

## Known gaps

* There is no reproducible Verdict differential pipeline in the repo; `tests/verdict_ref.rs` holds hand-computed values plus the probe values quoted from the reviewer's Verdict build (round 1). A generated fixture is still wanted.
* Missing Verdict metrics and thresholds API: see `DESIGN.md` Known gaps.
* High-order cells are evaluated on their corner skeleton only; no curved-element Jacobian sampling.
* Hex centre sample: Jacobian, scaled Jacobian and Oddy include it; shear and condition use corners only.
* No OpenFOAM `checkMesh`-style face metrics (non-orthogonality, face skewness, face pyramids): they are per-face rather than per-cell and were left out of this task's cell-metric scope.
* Polyhedral dihedral angles need closed cells and allocate (SmallVec, inline for <= 16 faces / 32 vertices) only beyond those sizes.
* `compute_many` makes one parallel pass per metric (gather repeated); a fused pass would save ~2x on long metric lists.
* Faces of 3-D cells (tri/quad faces) are not evaluated with the 2-D metrics.

## api_change_requests

None blocking. Optional, all additive (open tier):

1. **sm-mesh-api**, `MeshRead::cell_corners_into(&self, c: CellIx, out: &mut [VertexIx; 8]) -> usize` (provided, default via `cell_corners`) or exposing the corner column in `TopologyColumns`; why: removes a virtual call plus a slice-length branch per cell in the quality hot loop; proposal is optional, the current cost is small.
2. **sm-mesh-api**, a provided `MeshRead::cell_normal(&self, c: CellIx) -> Option<Vector3>` for dimension-2 meshes (reference orientation of a shell cell) or a documented way to obtain the `FaceNormals` of a mesh; why: today every caller of signed 2-D metrics must supply the normal source separately.
