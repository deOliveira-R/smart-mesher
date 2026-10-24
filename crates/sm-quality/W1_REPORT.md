# W1 report: sm-quality

## Summary

`sm-quality` implements the Verdict metric set for every `CellShape` of `sm-mesh-api` (tri, quad, tet, hex, wedge, pyramid; polygons and polyhedra as documented generalizations), element-level (`eval`, pure, allocation free) and mesh-level (`compute`, `compute_many`, `report`, `validity`) through `&dyn MeshRead`, with deterministic parallel reductions (`summarize`, `histogram`). 30 metrics, shape-applicability table (`Metric::applies`), ideal values (`Metric::ideal`). Formulas, ranges and conventions are in `DESIGN.md`.

Implemented per the task: scaled Jacobian, Jacobian, aspect ratio (tri/quad perimeter form, tet `beta` and `gamma`, Frobenius, radius ratio), skew, taper, warpage, stretch, condition number, shape, relative size, shape-and-size, min/max angle (interior angles in 2-D, dihedral in 3-D), edge ratio, max edge ratio, volume/area, plus Oddy, shear, shear-and-size, diagonal, dimension, distortion, collapse ratio, sphericity and star-shapedness; validity (valid / degenerate / inverted / skipped); summary statistics and histograms.

**Important caveat:** the Verdict sources and manual could not be fetched (network limited to package registries), so formulas are re-derived or reconstructed from memory and verified analytically, not diffed against Verdict. `DESIGN.md` marks the confidence of each group; the least certain are the advisory acceptable ranges and the quad warpage orientation (here `min cos^3`, 1 = flat).

## Test summary

All pass with default threads and with `RAYON_NUM_THREADS=1`: `cargo test -p sm-quality` (elements 2, distorted 18, mesh 11, properties 5, stats 7, determinism 3), `cargo clippy -p sm-quality --all-targets --all-features -- -D warnings` clean, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-quality --no-deps` clean, `cargo build` clean. Analytic checks: every applicable metric on every ideal shape equals its ideal value (~70 checks); closed forms on distorted elements; fixture volumes vs exact values and vs `sm_mesh_api::cell_volume` (1e-12). Property tests use `sm-testkit` strategies with the pinned seed. Determinism test: bit hashes of all columns/summaries/histograms/validity for 1, 2, 3, 4 threads, repeated, concurrent, over all testkit fixtures and a perturbed 20^3 block. Bench: `benches/metrics.rs` (hex kernel ~160 ns, 64k-hex scaled Jacobian 4.6 ms on 4 cores).

## Deviations from the contracts' intent

* None of the frozen crates was touched. `Cargo.toml` of the crate gained `rayon`, `smallvec` and dev-deps `sm-testkit`, `proptest`, `criterion` (all on the allowed list); `sm-core` is a dependency (`Point3` in the `FaceNormals` call of `SurfaceNormals::Field`).
* `Cargo.lock` changes with the new dependencies; it is deliberately not committed (the orchestrator owns it and regenerates it at merge).
* Delivery branch: the task text names `claude/w1-sm-quality`, the session harness designates `claude/sm-quality-metrics-vmofo0`; the same commit is pushed to both (merge by SHA).
* `Volume` is always the mesh geometry rule (fan from the vertex mean), not Verdict's per-shape volume; they agree for planar-faced cells.
* 2-D cells need a reference normal to be *signed*; the contract provides none on `MeshRead`, so `Options::normals` takes `Own` / `Uniform` / `PerCell` / `Field(&dyn FaceNormals)` (the contract's shell-orientation trait).
* Undefined metrics are `NaN` per cell (excluded from stats) and `Unsupported` only when no cell supports the metric, rather than an error for any mixed-mesh mismatch.
* Pyramid metrics ignore the apex corner (no canonical frame); wedge/pyramid reference elements are my reconstruction.

## Known gaps

* No comparison against real Verdict output (see caveat); acceptable ranges are advisory and unverified.
* High-order cells are evaluated on their corner skeleton only; no curved-element Jacobian sampling.
* Hex/hex-like metrics at the centre point: Jacobian and scaled Jacobian include it, shear, Oddy and condition use corners only (my reading of Verdict).
* No OpenFOAM `checkMesh`-style face metrics (non-orthogonality, face skewness, face pyramids): they are per-face rather than per-cell and were left out of this task's cell-metric scope.
* Polyhedral dihedral angles need closed cells and allocate (SmallVec, inline for <= 16 faces / 32 vertices) only beyond those sizes.
* `compute_many` makes one parallel pass per metric (gather repeated); a fused pass would save ~2x on long metric lists.
* Faces of 3-D cells (tri/quad faces) are not evaluated with the 2-D metrics.

## api_change_requests

None blocking. Optional, all additive (open tier):

1. **sm-mesh-api**, `MeshRead::cell_corners_into(&self, c: CellIx, out: &mut [VertexIx; 8]) -> usize` (provided, default via `cell_corners`) or exposing the corner column in `TopologyColumns`; why: removes a virtual call plus a slice-length branch per cell in the quality hot loop; proposal is optional, the current cost is small.
2. **sm-mesh-api**, a provided `MeshRead::cell_normal(&self, c: CellIx) -> Option<Vector3>` for dimension-2 meshes (reference orientation of a shell cell) or a documented way to obtain the `FaceNormals` of a mesh; why: today every caller of signed 2-D metrics must supply the normal source separately.
