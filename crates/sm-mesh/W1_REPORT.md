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

Build time, 4 vCPU shared machine, release (criterion medians; `examples/timing.rs` for the 10^6 case):

| hex block | bulk | incremental | reference |
|---|---|---|---|
| 32^3 (33 k) | 37 ms | 47 ms | 191 ms |
| 64^3 (262 k) | 305 ms | 514 ms | 1.70 s |
| 100^3 (10^6) | 1.0-1.4 s | 2.1 s | 7.1 s |

Bulk 64^3 on 1/2/4 threads: 511/335/229 ms. The 10^6 bulk mesh is bit-identical for 1 and N threads.

## Test summary

Definition of done, all run on this branch:

* `cargo build` (workspace): ok.
* `cargo test -p sm-mesh`: 109 passed; `--all-features`: 115 passed; `RAYON_NUM_THREADS=1 cargo test -p sm-mesh --all-features`: 115 passed.
* `cargo clippy -p sm-mesh --all-targets --all-features -- -D warnings`: clean.
* `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-mesh --no-deps`: clean. `#![forbid(unsafe_code)]`; no `unwrap`/`expect` in library code (one `unreachable!` on a constant that is provably in range: the "unset" classification sentinel).

What is tested:

* **Contract suite against the new store**: all 87 tests of `sm-mesh-api/tests` (81 without `serde`) copied into `tests/contract_*.rs` and run unchanged except for a shim that maps `ReferenceBuilder`/`ReferenceMesh`/`ReferenceCatalog`/`hex_block` to `sm-mesh` (counting-allocator zero-allocation tests, orientation, sets, instances/flatten/unfold, level and extrusion maps, hashes, validation negatives, surface meshes, high order).
* **Differential, every sm-testkit mesh fixture** (`tests/differential.rs`): replayed through `add_face`+`add_cell` and through `add_polyhedron_with_corners`; every accessor, id, `index_of`, class sets, edge table, sets, tags, `connectivity_hash`, `topology_hash`, `geometry_hash`, quantized hash, `face_remap`, `validate` identical to the reference. The harness is checked first by replaying into the reference itself.
* **State-machine differential** (`tests/builder_ops.rs`, 600 cases): random valid and invalid builder call sequences; every return value and error, the `finish` outcome and the allocator state agree with the reference.
* **Bulk equals incremental equals reference** (`tests/bulk.rs`, `properties.rs`): hex blocks, random mixed hex/wedge meshes, random Tri/Quad surface meshes; same provisional face numbering, same errors for the lowest offending cell, builder unchanged on error.
* **Property tests with the testkit checkers** (`mesh_valid`, `orientation_rule`, `closed_cells`, `positive_volumes`, `watertight`, `classification_complete`, Euler, `conformity`) plus exact volume and boundary area of random blocks.
* **Fault injection** (`properties.rs`): eleven classes of classification/lineage/open-cell faults; reference, `Validation::Fast` and `Validation::Full` give the same outcome, including the complete violation lists. Mutating a fast check makes the property fail (checked by hand).
* **Edge table and adjacency** (`tests/edges_adjacency.rs`): parallel derivation equals `EdgeTable::derive` (results, errors, allocator state, explicit classes); adjacency tables equal brute force; Euler characteristic of a block.
* **Determinism** (`tests/determinism.rs`): `sm_testkit::determinism::check` over 1 vs N threads, repeats and concurrent builds, for bulk and incremental blocks, all fixtures in both feeding modes, face remap, adjacency, and a failing build (the reported error is the lowest one).
* `tests/misc.rs`: `Send + Sync`, zero-copy pointer identity and byte widths, memory density, id ranges, bulk edge cases, `Fast` vs `Full`.

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
