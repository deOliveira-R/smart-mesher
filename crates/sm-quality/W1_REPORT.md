# W1 report: sm-quality

## Summary

`sm-quality` implements a large subset of the Verdict metric set (gaps listed in `DESIGN.md`) for every `CellShape` of `sm-mesh-api` (tri, quad, tet, hex, wedge, pyramid; polygons and polyhedra as documented generalizations), element-level (`eval`, pure, allocation free) and mesh-level (`compute`, `compute_many`, `report`, `validity`) through `&dyn MeshRead`, with deterministic parallel reductions (`summarize`, `histogram`). 30 metrics, shape-applicability table (`Metric::applies`), ideal values (`Metric::ideal`). Formulas, ranges and conventions are in `DESIGN.md`.

Implemented per the task: scaled Jacobian, Jacobian, aspect ratio (tri/quad perimeter form, tet `beta` and `gamma`, Frobenius, radius ratio), skew, taper, warpage, stretch, condition number, shape, relative size, shape-and-size, min/max angle (interior angles in 2-D, dihedral in 3-D), edge ratio, max edge ratio, volume/area, plus Oddy, shear, shear-and-size, diagonal, dimension, distortion, collapse ratio, sphericity and star-shapedness; validity (valid / degenerate / inverted / skipped); summary statistics and histograms.

**Verification status:** the first version was reconstructed from memory. A review then diffed it against the Verdict C++ sources and the 2007 manual, and fix round 0 (below) corrected the mismatches. `tests/verdict_ref.rs` pins Verdict reference values for distorted elements. Remaining deviations from Verdict are deliberate and listed in `DESIGN.md` ("Deviations from Verdict"); crate-only metrics are marked as extensions in `Metric`'s rustdoc and `DESIGN.md`.

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

Not done: the missing Verdict metrics (equiangle skew, quad radius ratio, tet mean ratio / inradius / squish / equivolume skew, hex nodal Jacobian ratio, timestep, wedge max stretch), and a thresholds API; both are listed in Known gaps. The Verdict manual was not present in the worktree (`docs/VerdictUserManual2007` does not exist here), so the range corrections and reference numbers are taken from the review text, not re-read by me.

## Test summary

All pass with default threads and with `RAYON_NUM_THREADS=1`: `cargo test -p sm-quality` (elements 2, distorted 18, verdict_ref 10, mesh 11, properties 5, stats 8, determinism 3), `cargo clippy -p sm-quality --all-targets --all-features -- -D warnings` clean, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-quality --no-deps` clean, `cargo build` clean. Analytic checks: every applicable metric on every ideal shape equals its ideal value (~70 checks); closed forms on distorted elements; fixture volumes vs exact values and vs `sm_mesh_api::cell_volume` (1e-12). Property tests use `sm-testkit` strategies with the pinned seed. Determinism test: bit hashes of all columns/summaries/histograms/validity for 1, 2, 3, 4 threads, repeated, concurrent, over all testkit fixtures and a perturbed 20^3 block. Bench: `benches/metrics.rs` (hex kernel ~160 ns, 64k-hex scaled Jacobian 4.6 ms on 4 cores).

## Deviations from the contracts' intent

* None of the frozen crates was touched. `Cargo.toml` of the crate gained `rayon`, `smallvec` and dev-deps `sm-testkit`, `proptest`, `criterion` (all on the allowed list); `sm-core` is a dependency (`Point3` in the `FaceNormals` call of `SurfaceNormals::Field`).
* `Cargo.lock` changes with the new dependencies; it is deliberately not committed (the orchestrator owns it and regenerates it at merge).
* Delivery branch: the task text names `claude/w1-sm-quality`, the session harness designates `claude/sm-quality-metrics-vmofo0`; the same commit is pushed to both (merge by SHA).
* `Volume` is always the mesh geometry rule (fan from the vertex mean), not Verdict's per-shape volume; they agree for planar-faced cells.
* 2-D cells need a reference normal to be *signed*; the contract provides none on `MeshRead`, so `Options::normals` takes `Own` / `Uniform` / `PerCell` / `Field(&dyn FaceNormals)` (the contract's shell-orientation trait).
* Undefined metrics are `NaN` per cell (excluded from stats) and `Unsupported` only when no cell supports the metric, rather than an error for any mixed-mesh mismatch.
* Pyramid `Jacobian` uses the four base-corner frames (equal to Verdict's four sub-tets); `ScaledJacobian` and `Shape` follow Verdict. Pyramid `Condition`, `MaxAspectFrobenius`, `MeanAspectFrobenius`, `EdgeRatio`, `RelativeSizeSquared`, `ShapeAndSize` and wedge `RelativeSizeSquared`/`ShapeAndSize` are smart-mesh extensions.

## Known gaps

* No run against a real Verdict build; `tests/verdict_ref.rs` holds values computed by hand from the reviewed C++ formulas. Ranges not listed as corrected are marked "(unchecked)" in `DESIGN.md`.
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
