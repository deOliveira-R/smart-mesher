# sm-mesh: design

`sm-mesh` is the production implementation of the `sm-mesh-api` contract (`MeshRead`, `MeshBuild`, `MeshEdit`, `MeshCatalog`/`CatalogEdit`, `columns()`, classification, sets and tags, hierarchy and extrusion maps, dimension-2 and dimension-3 meshes, high order). It is a drop-in for the in-memory reference implementation (`sm_mesh_api::reference`): same canonical result, same errors, same ids, same hashes, built several times faster and held in flat columns.

## Positioning

Similar to OpenFOAM's `polyMesh` and CGNS NGON/NFACE (face-first storage: face -> vertex loop, face -> (owner, neighbour), cell -> signed faces) and to the `sm-mesh-api` reference implementation, differs in how it is built and stored: flat structure-of-arrays/CSR columns with implicit ids and a compact classification column, a face table constructed by a deterministic parallel sort instead of one hash insertion per face, a parallel `finish` with fixed tie-breaks and error selection, a parallel edge table, adjacency tables (vertex -> cells, cell -> neighbours, edge -> faces) built on demand and cached, and a validation mode that only re-checks what user input can break, giving a 10^6-hexahedron mesh in 1.0-1.4 s on four cores (7.1 s for the reference), bit-identical for 1 and N threads, in about 260 bytes per cell, with primitive columns exposed as byte slices for zero-copy export.

## Layout

All columns are `Vec`s of fixed-width elements; offsets are `u64` (the on-disk width), indices `u32`.

| Column | Type | Notes |
|---|---|---|
| `x`, `y`, `z` | `f64` | vertices first, then high-order extra nodes |
| `face_off`, `face_verts` | `u64`, `VertexIx` | CSR face loops, right-hand rule, start at the smallest vertex |
| `face_left`, `face_right`, `face_kind` | `CellIx`, `u32`, `FaceKind` | `face_right = NO_CELL` for one-cell faces |
| `cell_off`, `cell_faces` | `u64`, `SignedFace` | CSR, ascending face index, `face << 1 \| side` |
| `cell_shape`, `corner_off`, `corners` | `CellShape`, `u64`, `VertexIx` | template corners for every template-shaped cell (and every dim-2 cell) |
| `cell_parent`, `cell_active` | `u32`, `bool` | lineage |
| classification | `Vec<EntityId>` + sparse `BTreeMap<u32, Param>` per vertex/face/cell | the model entity densely (8 B), the parametric position only where there is one |
| reverse class index | `BTreeMap<EntityId, ClassSet>` | rebuilt in parallel chunks after classification edits |
| ids | first id + count per kind | ids are consecutive serials, so the id column is implicit and `index_of` is `O(1)` |
| edge table | contract `EdgeTable` | optional, derived in parallel |
| sparse side tables | `BTreeMap` | high-order cells, face curves; sets, tags, geometry cache |

Memory for a structured hexahedral block is about 263 bytes per cell (3.03 faces, 1.03 vertices per cell; classification included), `Mesh::memory_bytes`. `MeshRead::columns()` returns the topology columns as whole slices, `Mesh::raw_columns()` the primitive ones (`RawColumns`, `as_bytes`) for zero-copy writers. The typed index columns (`VertexIx`, `CellIx`, `SignedFace` are `repr(transparent)` over `u32`) have no `bytemuck::Pod` impl and cannot get one here (orphan rule); this crate is `#![forbid(unsafe_code)]` and files the request in `W1_REPORT.md`.

## Construction

Two ways in, one `finish`.

**Incremental** (`MeshBuilder`, the `MeshBuild` trait). Vertices, faces and cells are appended to CSR columns. Face deduplication uses a lazily synchronised open-addressing index keyed by a commutative hash of the vertex *set* (order independent, so any rotation or reversal of a loop finds its face); equal-hash faces probe in creation order, so `find_face` returns the same "first face in creation order" as the reference, and `find_face_where` walks coincident baffle faces in order. `truncate_faces` (the rollback of a failed `add_polyhedron`) resets the index. `add_cell_from_vertices` is overridden: it builds the template faces on the stack and flags the cell as built from its template, which lets `finish` skip re-deriving corners that hold by construction. `find_vertex` uses a hash grid (lookups only, the answer is the lowest matching index).

**Bulk** (`MeshBuilder::add_cells_bulk`, `add_uniform_cells`, `add_vertices_with`, `classify_*_with`). For template meshes the face table is built without per-face hashing: every (cell, local face) becomes a record keyed by its sorted vertex set, a parallel sort groups equal keys, each run is resolved to one or two faces with the same rules as the incremental builder (a twisted duplicate is a separate face, a third cell is `NonManifoldFace`, two cells that traverse a face the same way are `InconsistentOrientation`), provisional face numbers are ranks of the first appearance (a bitmap and a prefix popcount), and loops, cell lists and incidence are filled in parallel into disjoint slices (`split_at_mut`, no `unsafe`). The outcome equals adding the cells one by one: same provisional numbering, same cell face lists, same error for the lowest offending cell, builder unchanged on `Err`. Differential tests pin this.

**`finish`** (parallel, deterministic). Stages: completeness of vertex and cell classes (`position_first`); per-face facts (left/right cell, reversal, rotation to the smallest vertex, kind); the canonical sort `(left, right-or-none, loop)` with the builder index as final tie-break (a strict total order, so `par_sort_unstable_by` is deterministic); duplicate-face check; final loops written into disjoint CSR slices; face classes (derived from the cells for unset interior-like faces); cells in chunks of 4096 (sorted signed faces, shape detection, corner verification or derivation by the contract's own `derive_corners`, 2-D cycles by `polygon_cycle`), merged in chunk order; ids; high-order and face-curve renumbering; sets and tags remapped to the final face order; edge table; class index; validation. Every parallel stage that can fail records `(item index, error)` and the lowest index wins, so the reported error never depends on the thread count. `finish` runs on a clone of the `SerialAlloc` and commits it only on success (failure atomicity of the contract).

## Validation modes

`Validation::Fast` (default) checks in parallel exactly the `validate` rules that builder input can break: classification kind of every face (`accepts_facet`), dimension order of face vertices and of edges, lineage (an active child of an active parent), and closedness of cells built from loops. Everything else `validate` tests (canonical order, orientation, incidence, shape tags and corners, reverse class index, edge table, set and tag shape, high-order records) is established by construction or checked at the call that stores it. If a fast check fails, `finish` calls the contract's `validate` and returns its exact report (`MeshError::Invalid`), so errors equal the reference's. `Validation::Full` always runs the sequential contract validator (2.3 s at 10^6 cells, more than the whole bulk build). A property test injects eleven classes of faults and demands that reference, `Fast` and `Full` produce the same outcome (including the violation lists); mutating a fast check makes it fail.

## Edge table

Parallel and identical to `EdgeTable::derive` (test: fixtures and blocks, with and without explicit classes, errors, allocator state). Every face-edge incidence is a record `(low<<32 | high, face<<1 | forward, loop position)`; a parallel sort groups them, run starts give the edges in lexicographic vertex-pair order, each edge is classified from its own run (lowest-dimension face entity if unique, else the shared cell entity when all faces are interior, explicit classes first), the lowest unclassified edge is reported, ids are drawn last, and `face_edges` is scattered by loop position.

## Adjacency

`Mesh::vertex_cells`, `vertex_faces`, `cell_neighbors`, `edge_faces`, `vertex_edges` return a `Csr` with ascending, duplicate-free rows, built on first use in parallel (pairs `row<<32 | value` sorted and deduplicated; offsets by `partition_point` per row) and cached in `OnceLock`s that never take part in equality, are dropped by `clone`, and are reset when the edge table changes. Topology is immutable after `finish` (`MeshEdit` is topology-preserving), so the other caches never go stale.

## Catalog

`Catalog` implements `MeshCatalog`/`CatalogEdit` over `Box<dyn MeshRead>` with ordered maps keyed by `(fine, coarse)` and `(base, extruded)`; `maps_from` is a range scan. Level maps and extrusion maps validate against the meshes exactly as in the contract (`check_meshes`, stale or foreign pairs refused).

## Determinism

No `HashMap` iteration reaches an output (the face index and vertex grid are used for lookups only), every sort has a total order, every reduction is a `min` by item index or runs in chunk order, no randomness. Tests: 1 thread vs N threads vs repeated runs vs concurrent builds (`sm_testkit::determinism::check`) on the bulk block, the incremental block with edges, all fixtures in both feeding modes, the face remap, the adjacency tables and a failing build.

## Tests

* `tests/contract_*.rs`: the 87 tests of the `sm-mesh-api` suite (81 without the `serde` feature), copied and run unchanged against this store (a shim maps `ReferenceBuilder`/`ReferenceMesh`/`ReferenceCatalog`/`hex_block` to `sm-mesh`), including the zero-allocation accessor tests and (feature `serde`) the serde round trips.
* `tests/differential.rs`: every `sm-testkit` fixture replayed through `add_face`/`add_cell` and through `add_polyhedron_with_corners`, compared on every accessor, id, `index_of`, class set, hash (connectivity, topology, geometry, quantized), `face_remap`, validity.
* `tests/builder_ops.rs` (600 cases): random sequences of builder calls (valid and invalid) applied to the reference and to `MeshBuilder`; every return value, error, the `finish` outcome and the allocator state agree.
* `tests/bulk.rs`, `tests/edges_adjacency.rs`, `tests/determinism.rs`, `tests/properties.rs` (random blocks with the testkit checkers and analytic volume/area, random hex/wedge meshes, random surface meshes, fault injection).

## Performance (4 vCPU, release; see `benches/build.rs`, `examples/timing.rs`)

| structured hex block | cells | bulk | incremental | reference |
|---|---|---|---|---|
| 32^3 | 33 k | 37 ms | 47 ms | 191 ms |
| 64^3 | 262 k | 305 ms | 514 ms | 1.70 s |
| 100^3 | 1 M | 1.0-1.4 s | 2.1 s | 7.1 s |

Bulk 64^3 on 1, 2, 4 threads: 511, 335, 229 ms. Queries on 64^3: `vertex_cells` 59 ms, `cell_neighbors` 18 ms; edge table of a 32^3 block 44 ms. Single-threaded, bulk (1.01 s at 10^6 cells) is not faster than incremental (0.90 s): the sequential sort/scatter stages dominate, so bulk only pays off with more threads. Timings on this shared machine vary by up to 2x between runs; the table shows the criterion medians.

## Limits

* The bulk path takes template shapes only (polyhedra go through the incremental path).
* A single `set_class` on a finished mesh rebuilds the reverse class index (`O(n)`); `set_classes` rebuilds once.
* High-order dimension-3 meshes need the edge table (contract rule); it is derived in `finish`.
* `Mesh` has no serialized form and no import from raw columns (the loader belongs to `sm-meshio`, and needs `finish_with_ids` or the Pod request first).
* Faces are limited to 2^31 (contract), records of the bulk path to 2^31, vertices to `u32::MAX - 1`.
