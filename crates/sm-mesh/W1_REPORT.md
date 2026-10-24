# W1 report: sm-mesh

Pushed to `claude/w1-sm-mesh-icdwyq`, the session's designated branch (the task text names `claude/w1-sm-mesh`; the harness only permits the designated branch, so that name was not pushed and the orchestrator can create it from the SHA). The orchestrator merges by SHA; the final HEAD SHA is in the hand-off message.

## Summary

`crates/sm-mesh` is the production implementation of `sm-mesh-api`:

* `Mesh`: finished face-first store in flat SoA/CSR columns (`MeshRead`, `MeshEdit`), implicit consecutive ids, compact classification (8 bytes per entity plus a sparse parameter map), `columns()` as whole slices, dimension 2 and 3, high order, face curves, sets, tags, lineage, geometry cache. About 263 bytes per cell on a structured hex block.
* `MeshBuilder`: `MeshBuild` over CSR columns with a lazy open-addressing face index, plus a bulk path (`add_cells_bulk`, `add_uniform_cells`, `add_vertices_with`, `classify_*_with`) that builds the face table by a deterministic parallel sort. Both paths end in one parallel, deterministic `finish` (canonical order `(left, right, loop)`).
* Parallel edge table (identical to `EdgeTable::derive`), adjacency tables (`vertex_cells`, `vertex_faces`, `cell_neighbors`, `edge_faces`, `vertex_edges`) as cached `Csr`.
* `Catalog` (`MeshCatalog`/`CatalogEdit`): meshes, level maps, extrusion maps, instanced views.
* `RawColumns`/`as_bytes`: zero-copy byte views of the primitive columns.
* `generate::{hex_block, hex_block_bulk}`: structured block generators (same classification as `reference::hex_block`) for tests and benchmarks.

Build time, Apple M1 Max (10 threads), release, `examples/timing.rs` with `WITH_REFERENCE=1`, median of 3 runs each (run-to-run spread under 5%):

| hex block | bulk | incremental | reference |
|---|---|---|---|
| 32^3 (33 k) | 17 ms | 22 ms | 108 ms |
| 64^3 (262 k) | 88 ms | 200 ms | 910 ms |
| 100^3 (10^6) | 0.30 s | 0.83 s | 3.6 s |

Bulk 64^3 on 1/2/4/10 threads: 277/153/101/88 ms. At 10^6 cells on 1 thread: bulk 1.06 s, incremental 1.03 s, reference 3.5 s. Single-threaded, bulk is no faster than incremental (the sequential sort/scatter stages dominate), so bulk only pays off with more threads. The 10^6 bulk mesh is bit-identical for 1 and N threads. (The earlier 4 vCPU shared-machine figures, e.g. 1.0-1.4 s bulk and 2.1 s incremental at 10^6, are superseded.)

## Test summary

Definition of done, all run on this branch:

* `cargo build` (workspace): ok.
* `cargo test -p sm-mesh`: 124 passed; `--all-features`: 130 passed; `RAYON_NUM_THREADS=1 cargo test -p sm-mesh --all-features`: 130 passed.
* `cargo clippy -p sm-mesh --all-targets --all-features -- -D warnings`: clean.
* `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-mesh --no-deps`: clean. `#![forbid(unsafe_code)]`; no `unwrap`/`expect` in library code (one `unreachable!` on a constant that is provably in range: the "unset" classification sentinel).

What is tested:

* **Contract suite against the new store**: all 87 tests of `sm-mesh-api/tests` (81 without `serde`) copied into `tests/contract_*.rs` and run unchanged except for a shim that maps `ReferenceBuilder`/`ReferenceMesh`/`ReferenceCatalog`/`hex_block` to `sm-mesh` (counting-allocator zero-allocation tests, orientation, sets, instances/flatten/unfold, level and extrusion maps, hashes, validation negatives, surface meshes, high order).
* **Differential, every sm-testkit mesh fixture** (`tests/differential.rs`): replayed through `add_face`+`add_cell` and through `add_polyhedron_with_corners`; every accessor, id, `index_of`, class sets, edge table, sets, tags, `connectivity_hash`, `topology_hash`, `geometry_hash`, quantized hash, `face_remap`, `validate` identical to the reference. The harness is checked first by replaying into the reference itself.
* **State-machine differential** (`tests/builder_ops.rs`, 600 cases): random valid and invalid builder call sequences; every return value and error, the `finish` outcome and the allocator state agree with the reference.
* **Bulk equals incremental equals reference** (`tests/bulk.rs`, `properties.rs`): hex blocks, random mixed hex/wedge meshes, random Tri/Quad surface meshes; same provisional face numbering, same errors for the lowest offending cell, builder unchanged on error.
* **Property tests with the testkit checkers** (`mesh_valid`, `orientation_rule`, `closed_cells`, `positive_volumes`, `watertight`, `classification_complete`, Euler, `conformity`) plus exact volume and boundary area of random blocks.
* **Fault injection** (`properties.rs`): fourteen fault classes (classification, lineage, open cells, face curves, high order); reference, `Validation::Fast` and `Validation::Full` give the same outcome, including the complete violation lists. Mutating a fast check makes the property fail (checked by hand).
* **Edge table and adjacency** (`tests/edges_adjacency.rs`): parallel derivation equals `EdgeTable::derive` (results, errors, allocator state, explicit classes); adjacency tables equal brute force; Euler characteristic of a block.
* **Determinism** (`tests/determinism.rs`): `sm_testkit::determinism::check` over 1 vs N threads, repeats and concurrent builds, for bulk and incremental blocks, all fixtures in both feeding modes, face remap, adjacency, and a failing build (the reported error is the lowest one).
* `tests/misc.rs`: `Send + Sync`, zero-copy pointer identity and byte widths, memory density, id ranges, bulk edge cases, `Fast` vs `Full`.

## Fix round fix0 (review findings)

Must fix:
1. `fast_checks` now returns `false` whenever the mesh holds high-order cells or face curves, so the full validator judges them (the builder cannot range-check `NodeRef::Vertex` tails of a face curve; `Validation::Fast` used to finish `Ok` with PointIx 999 in an 8-point mesh). Builder API unchanged, so errors still equal the reference's. Regression: `tests/curves.rs` (explicit case plus a 200-case property over random curve tails, Fast and Full against the reference; fails without the fix). Cost: meshes with curves skip the fast path (they are rare and small).
2. `add_cells_bulk` / `add_uniform_cells` report the incremental error of the lowest offending cell. Each cell is checked in the order of `add_cell_from_vertices` (vertex count, dimension, per-face `check_loop`, corners). The lowest invalid cell is found first (parallel `position_first`, no per-cell `format!`); face clashes are then resolved only over the cells below it (or including it when only its corners are bad), and the lowest cell wins; for one cell a loop error beats a clash. A corner-invalid cell that also clashes or repeats a face is replayed incrementally on a scratch builder (cold path, exact error). Tests: `tests/bulk.rs` cases (a)-(d) plus reversed orders, and a 400-case property against the reference cell by cell (dimension 3 and 2, duplicate and unknown vertices, clashes, mixed shapes).

Should fix, done:
1. `builder_ops` now also runs `add_extra_node`, `set_high_order`, `set_face_curve`, `add_set`, `set_label`, `add_tag` and a `MeshBuilder::surface()` variant; the fault-injection property has face-curve and high-order faults; `tests/curves.rs` covers classified hexes with curves that reach `finish`. The state machine alone did not catch the face-curve bug (a random sequence rarely gets a classified, closed mesh with a bad curve), which is why `curves.rs` and the faults exist. Bulk methods are compared with the reference in `bulk_matches_the_reference_cell_by_cell`.
2. and 7. `tests/determinism.rs`: bulk 24x20x18 (8640 cells, 51840 records, more than one run-resolution chunk), incremental with edges 20x18x16 (5760 cells), a shuffled 22x20x18 lattice with random vertex numbering and cell order through bulk and incremental at 1/3/10 threads (bit-identical and equal to each other), and a 8640-cell bulk error test where the lowest of three defects must win at 1/3/8 threads. The reference topology/geometry hash comparison from the review was ad hoc and is not repeated in-repo.
3. `class::group` finds a chunk's slot through a `BTreeMap<EntityId, usize>` (`O(log k)`), not a linear scan.
4. `truncate_faces` deletes the truncated slots from the face index (backward-shift delete in tag order) instead of dropping the index; a failed `add_polyhedron` no longer costs `O(F)` on the next lookup. Test: `rolled_back_polyhedra_keep_the_face_index_consistent`.
5. `IdRange::id` panics for `ix >= n` like indexing the reference id column (`MeshRead::id` out of range: test `id_of_an_out_of_range_index_panics_like_the_reference`).
6. `add_vertices_with` allows the same total as `add_vertex` (one more vertex); the per-cell `format!` in bulk validation is gone (see Must fix 2).
8. Chosen: document the coupling (rustdoc on `MeshBuilder` and `Validation::Fast`) and keep `Fast` as the default; the curves bug is closed by routing all curve/high-order meshes to the full validator.
10. 10^6 bulk time is 0.30 s (10 threads, Apple M1 Max) in both this report and `DESIGN.md`; see Performance.

Skipped: 9 (retire `tests/contract_*.rs`): the contract suite is not generic over an implementation yet, so the copies stay; this is already in the deviations list.

## Deviations from the contracts' intent

1. **`finish` validates in `Validation::Fast` mode by default.** The reference runs the whole sequential `validate` (2.3 s at 10^6 cells, more than the entire bulk build). `Fast` checks in parallel only the rules builder input can break (face kind vs class, dimension order for vertices/faces/edges, lineage, open cells); on any hit it runs the contract `validate` and returns its report, so errors equal the reference's. Evidence that nothing is missed, independent of how the checks were designed: `tests/builder_ops.rs` compares the `finish` outcome of 600 random call sequences against the reference builder (a hole in `fast_checks` would show as `Ok` here and `Err(Invalid)` there), the fault-injection property compares `Fast`, `Full` and the reference on eleven fault classes, and every contract negative test (validate_hash, surface, highorder, final_fixes) runs through `Fast`. `Validation::Full` (`MeshBuilder::with_validation`) runs `validate` always. Coupling: a new `validate` rule that user input can break must be mirrored in `fast_checks` (see request 3).
2. **No `unsafe`, so the typed index columns have no byte view.** `RawColumns` covers coordinates, offsets, `face_right`, `cell_parent`; `face_vertices`, `face_left` and `cell_faces` (`VertexIx`, `CellIx`, `SignedFace`, all `repr(transparent)` over `u32`) need request 1.
3. **Extra public API beyond the traits**: `Validation`, bulk methods, `Csr` and the five adjacency methods, `memory_bytes`, `generate`, `RawColumns`/`as_bytes`, `Catalog` (the reference calls it `ReferenceCatalog`).
4. The builder's lazy face index and vertex grid sit behind `Mutex` (uncontended) so `&MeshBuilder` is `Sync` for the parallel `finish`.
5. `Mesh` has no serialized form and no import from raw columns. The `serde` cargo feature only enables `sm-mesh-api/serde` so the contract's serde round-trip tests run.
6. Ids are stored as a first id and a count per kind (consecutive by contract), not as a column.
7. Dependencies: `rayon`, `smallvec`, `indexmap`, `bytemuck` (all on the allowed list); dev: `proptest`, `criterion` (default features off), `serde`/`serde_json` (contract serde tests), `sm-testkit`. `Cargo.lock` changed (rayon, criterion and their trees were not in it); the orchestrator regenerates it at merge.
8. The contract test files were copied into this crate (maintenance duplicate until the contract tests become generic over an implementation).
9. Branch name: pushed only to the designated `claude/w1-sm-mesh-icdwyq`, not to the task's `claude/w1-sm-mesh` (see the top).

## Known gaps

* The bulk path takes template shapes only; polyhedra use the incremental path.
* `MeshEdit::set_class` on a finished mesh rebuilds the reverse class index (`O(n)`); `set_classes` rebuilds once.
* Parallel scaling is 2.2x on 4 threads at 64^3: the remaining sequential parts are the inverse-permutation scatter, the merge of the per-chunk class groups and the record scatter in the edge table. `vertex_cells` sorts all `(vertex, cell)` pairs (190 ms at 10^6 cells); a counting sort would be faster.
* Timings come from a shared 4-vCPU container and vary by up to 2x between runs; the table shows criterion medians. Meshes beyond 10^6 cells and the 2^31 limits were not exercised here.
* Not implemented because the contract defers them: non-conformal interface records, partition-local views (ghosts), refinement edits.
* Memory figure (263 B/cell) is for a classified structured hex block without edge table; the edge table adds about 50 B/cell.

## api_change_requests

1. **crate `sm-mesh-api`**: `bytemuck::Pod` + `Zeroable` for the index types and `SignedFace`, behind an optional `bytemuck` feature.
   *Why*: zero-copy export of `face_vertices`, `face_left`, `cell_faces`, `corners` and any other index column; the orphan rule prevents implementing it downstream, and `#![forbid(unsafe_code)]` rules out casting.
   *Proposed*: in `index.rs`, `#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]` on `VertexIx`, `EdgeIx`, `FaceIx`, `CellIx`, `PointIx` (inside `dense_index!`) and `SignedFace` (all `Copy`, `repr(transparent)` over `u32`); Cargo: `bytemuck = ["dep:bytemuck"]`, `bytemuck = { version = "1", features = ["derive"], optional = true }`.
2. **crate `sm-mesh-api`**: provided methods on `MeshBuild` so generic meshers can reach the fast path.
   *Why*: `add_cells_bulk` is an inherent method of `MeshBuilder`; code written against the trait cannot use it, and an implementation cannot size its tables up front.
   *Proposed*: `fn add_template_cells(&mut self, shapes: &[CellShape], offsets: &[u64], verts: &[VertexIx]) -> Result<core::ops::Range<usize>>` defaulting to a loop of `add_cell_from_vertices` (same observable result and errors), and `fn reserve(&mut self, vertices: usize, cells: usize) {}` defaulting to a no-op.
3. **crate `sm-mesh-api`** (low priority): make the classification rules of `validate` callable on their own.
   *Why*: `Validation::Fast` re-implements the face-kind, dimension-order and lineage rules; a public function removes the duplicate.
   *Proposed*: `pub fn validate_classification(m: &dyn MeshRead) -> Vec<Violation>` (the rules of the private `validate_classification`, plus the lineage rule), and let `validate` call it.
4. **crate `sm-core`** (nice to have): store `EntityId` as `NonZeroU64` (the kind byte is never 0), so `Option<EntityId>` is 8 bytes and builders need no sentinel id (this crate uses a `MeshCell` id with serial 0 as the "unset" marker of its classification columns).

## fix1 (review findings)

* **Fixed: `FaceIndex::grow` broke creation order for coincident faces.** The rehash walked old slots in slot order, so a probe chain wrapped past the table end was reinserted tail-first. It now starts at an empty old slot and goes round cyclically, which keeps every chain intact. Regression test `tests/face_index.rs` (baffle pairs, several grows, `find_face`/`find_face_where` against the reference before and after `truncate_faces` and a re-add, then `add_polyhedron` onto a baffle pair); it fails on the old code and passes now.
* `MeshRead::id(Edge, ix)` without an edge table now indexes an empty slice and panics, like the reference.
* `cell_problem`: dropped the unreachable unknown-vertex part of the Corners check (commented why).
* New `tests/flatten_diff.rs`: property differential of `unfold` (decks of 0 to 2 mirrors, welded and unwelded) and catalog `flatten` against the reference on decorated meshes (sets, side sets, labels, cell/face tags, face classes, reclassification through `on_merge`); results, errors, `FlattenMap` and allocator state agree.
* DESIGN.md notes that single-threaded bulk is no faster than incremental; bulk pays off with threads. The sequential sort/scatter stages were not changed. (Numbers re-measured in polish0, see the Performance table.)
* `replay_error` now documents that it relies on `add_cells_bulk` requiring an empty builder (O(cv) cost only on the error path).
* `Cargo.lock` is the only file changed outside `crates/sm-mesh` in the original W1 commit; the orchestrator regenerates it at merge. This fix touches nothing outside the crate.
* Not done: retiring `tests/contract_*.rs` copies; this waits for contract tests generic over an implementation (deviation 8).

## polish0

* `FaceIndex::grow`: documented the load-factor invariant (at most 1/2, so a non-empty table always has an empty slot) and added `debug_assert`s; the `unwrap_or(0)` fallback only covers the initial empty table.
* Timings re-measured on an Apple M1 Max, 10 threads (and 1 thread), release, median of 3; DESIGN.md and this report now use the same table. Old numbers came from a 4 vCPU shared machine and are superseded. The reviewer's figures (0.85 s incremental, 0.29 s bulk, 3.5 s reference at 10^6 on 10 threads) agree with the new ones (0.83 s, 0.30 s, 3.6 s).
* Deviation 8 stays accurate: `tests/contract_*.rs` still duplicates the `sm-mesh-api` contract tests (87 with `serde`, 81 without), copied unchanged behind a shim, until the contract offers tests generic over an implementation. Left as is, per review.

## Timing note: `Validation::Fast` finish (release, 100^3 hexes via `examples/timing`)

`fast_checks` computes the per-face loop flags once, in parallel, with the allocation-free
`sm_mesh_api::face_loop_ok`, then runs `validate_classification_with` and `validate_records_with`
concurrently (`rayon::join`). Before: bulk build + finish about 560 ms (two serial passes, each
sorting a copy of every loop); origin/main (single parallel pass) 261-298 ms; now about 315 ms
(classification 77 ms, records 17 ms, loop flags 5 ms on the finished mesh). The classification
reverse-index build groups runs of equal classes instead of one map lookup per entity.
