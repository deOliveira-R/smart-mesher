# W1 report: sm-meshio (mesh export)

status: **done** (all items of the definition of done pass; see "Known gaps" for scope left out).

## Summary

* `.smsh` v1.0 writer and reader (`smsh` module): JSON header plus 64-byte-aligned little-endian
  arrays, per-array CRC-32, face-first topology (signed `cell_faces`, `face_left/right/kind`),
  template corners, ids, classification (with parametric coordinates), optional edge table,
  high-order cell and face-curve records, cached geometry, named sets (node, face, side,
  shell-side, point, cell) with integer labels, typed tags, lineage, level maps, extrusion maps,
  deck metadata (built from `sm_model_api::Deck`), provenance hash, and the three mesh-API content
  hashes. Spec: `SMSH_FORMAT.md` (versioned). Reader: zero-copy typed views (`SmshFile`), full
  owned decode (`SmshMesh`), optional `mmap` feature (`SmshMap`).
* `readers/smsh.py`: 81-line standard-library reference reader with CRC verification.
* VTU writer (`vtu` module): tet/hex/wedge/pyramid, `VTK_POLYHEDRON` with `faces`/`faceoffsets`,
  tri/quad/polygon for dimension-2 meshes, raw or base64 appended data, cell data `region`,
  `class_entity`, `set:*`, `tag:*`, `active`, point data `class_dim`, `class_entity`, `set:*`,
  `tag:*`.
* Everything reads through `&dyn MeshRead` only; output is byte-identical across runs and
  thread counts.
* `DESIGN.md` (positioning paragraph and design), criterion bench `benches/export.rs`.

Release timings (criterion, 64,000-cell hex block, this container): `.smsh` write 43 ms (of
which about 25 ms is the three `sm-mesh-api` hashes; `without_hashes()` skips them),
parse + CRC verify + full decode 14 ms, VTU raw 4.2 ms, VTU base64 5.6 ms.

## Test summary

`cargo test -p sm-meshio` (also with `RAYON_NUM_THREADS=1`; all pass):

* default features: 40 tests = 4 unit, 11 `.smsh` (`tests/smsh.rs`), 14 hostile-input and
  stale-map (`tests/corrupt.rs`), 8 VTU, 2 Python-reader, 1 property test;
* `--all-features`: 41 tests, the same plus the `mmap` test in `tests/smsh.rs` (12 `.smsh`).

Clippy (`--all-targets --all-features -D warnings`, and default features) and
`RUSTDOCFLAGS="-D warnings" cargo doc` are clean.

* **Corpus** (all 10 `sm-testkit` mesh fixtures, hex block, a decorated block with edge table,
  labelled cell/face/side/node/point sets, tags, geometry cache and parametric classification, a
  HEX27 mesh with extra nodes, a dimension-2 strip, a lineage pair): every array of every mesh
  round-trips bit-exactly through writer -> parser -> decode, including ids, classification,
  sets, tags, hashes.
* **Analytic**: cell volumes recomputed from the decoded file only (Newell area vectors and the
  stored signs) match the fixtures' exact volumes to 1e-12; random hex blocks (seeded
  `sm-testkit` proptest strategy) reproduce `prod(n h)`.
* **Layout/corruption**: alignment, no overlap, no trailing bytes, flipped payload byte, truncated
  file, bad magic, newer major, unaligned input buffer, mmap path (feature).
* **Python**: `python3 -I readers/smsh.py` on every corpus file agrees with the Rust side (counts,
  per-cell vertex sets walked through the CSR arrays, ids, classification, set labels, last
  coordinate bits) and rejects a corrupted file. Skipped (with a message) if no interpreter runs.
* **VTU, independent decoder**: `tests/vtu_check.py` decodes the raw and base64 files with the
  standard library; points (bit-exact), types, connectivity, offsets, `faces`, `faceoffsets`,
  region/class/set/tag arrays match values computed independently from the mesh. XML escaping of
  a hostile set name.
* **VTU, VTK itself**: if the interpreter in `SM_PYTHON` has `vtkmodules`, the files are also
  loaded with `vtkXMLUnstructuredGridReader`: cell types equal, every 3D cell has outward faces
  in VTK's convention, and VTK's signed cell volumes equal the fan-rule volumes (this includes
  the polyhedra, except the non-planar `dual_patch` prisms, where VTK's face triangulation and
  the fan rule legitimately differ). I ran this with VTK 9.7.1 in a venv (all green); without VTK the test skips
  that part. **This check found a bug in my first version**: I had permuted the wedge nodes
  (`0,2,1,3,5,4`) from memory of another format; VTK reports volume -0.5 for that order and +0.5
  for the template order, so the permutation was removed.
* **Determinism**: for the whole corpus, both formats and both VTU encodings, output hash is
  equal over repeated runs, 1 thread, 4 threads, and 4 concurrent 1-thread calls
  (`sm_testkit::determinism::check`); a 24,000-cell block (larger than every chunk size)
  compares full bytes for 1 vs 6/7 threads.

## Polish round 0 (review items)

* Decoder range-checks every index column (face cells, cell parents, cell corners / faces, edge
  table, set words by kind, high-order and face-curve records, tag lengths, `n_points`, level
  map and extrusion columns by the role of the file's mesh) and rejects `cell_shape` /
  `face_kind` codes outside the table; one test per class in `tests/corrupt.rs`.
* Writer verifies level and extrusion maps against the mesh (connectivity hash and sizes) and
  returns `Inconsistent` for a stale map instead of panicking; tested with stale meshes whose id
  matches. The extrusion check relies on the hash because `ExtrusionMap` has no length accessor
  (api_change_request 3 below).
* VTU packs connectivity, offsets, faces and face offsets straight into byte buffers; tag
  columns of the wrong length are `Inconsistent`.
* VTU tests: hand-written oracle (wedge, pyramid, polyhedron); the VTK check fails loudly when
  `SM_PYTHON` is set and `vtkmodules` cannot be imported.
* `SmshMesh` holds `sm_mesh_api::EdgeTable` and `MeshSet` (the redundant role string is gone).
* `DeckRecord` stays a file-format schema, not the serde form of `sm_model_api::Deck`; the
  reason is documented in `DESIGN.md` and `SMSH_FORMAT.md` 2.7.

## Deviations from the contracts' intent

1. **Reader returns `SmshMesh` (owned arrays and API value types), not a `MeshRead`.** The mesh
   API has no `finish_with_ids` (W0a decision 5) and mesh import is deferred (ir.md 5.3), so a
   file cannot be rebuilt with its original ids without a contract change. Round-trip tests
   compare arrays to the source mesh instead.
2. **`DeckRecord` is defined here**, built from `sm_model_api::Deck` (new dependency on that
   frozen crate, read-only use), because `Deck` carries no name and no association with mesh
   sets. The record names sets of the file; creating those sets from model boundary faces
   (lowering through classification) is not done here.
3. **One mesh per file.** `MeshCatalog` hierarchies are stored as maps attached to each mesh
   file, referencing the other mesh by id and connectivity hash; a multi-mesh archive is not
   defined.
4. **Hashes are optional** in the header (`without_hashes()`), because the `sm-mesh-api` hashes
   dominate write time.
5. **VTU `class_entity` is the entity's serial only**; the kind is lost (the volume cells are all
   `TopoVolume`/`PartitionCell`, but a mixed mesh would collide). `class_dim` carries the
   dimension for points. Use `.smsh` for the full id.
6. **Cargo.lock is not committed.** My dependencies change it; the orchestrator regenerates it at
   merge (W0a decision 8). A `--locked` build of this branch alone fails until then.
7. `crates/sm-meshio/Cargo.toml` adds `sm-model-api`, `rayon`, `bytemuck`, `serde`,
   `serde_json` (`float_roundtrip`), optional `memmap2` (feature `mmap`; the only `unsafe`, one
   read-only `Mmap::map`), and dev-deps `sm-testkit`, `criterion` (`cargo_bench_support`),
   `proptest`, `serde_json`, `rayon`. All are on the allowed list. CRC-32 and base64 are
   implemented locally (about 60 lines, tested against known vectors) to avoid more dependencies.

## Known gaps

* VTU writes high-order cells as their linear corners (extra nodes are written as unused
  points). No VTK Lagrange/quadratic cell types, no VTKHDF, no `.pvtu`, no compression.
* VTU has no boundary/face-set output: face and side sets exist in `.smsh` only. A boundary skin
  dataset (surface cells with set ids) is the natural follow-up.
* `.smsh` does not persist instanced views/prototypes (`InstancedView`, only flattened meshes),
  non-conformal interface records and overlap tables (no API type yet), partition/ghost columns,
  or the owner geometry of high-order nodes beyond their classification (`point_class`).
* Writers materialise one owned buffer per non-`f64` array (about the size of the mesh) before
  writing; there is no streaming path for meshes near memory limits.
* Big-endian hosts are untested (all scalar output goes through explicit LE conversion).
* Exodus II, CGNS, Gmsh, DAGMC, OpenFOAM exporters and the loss report (ir.md 2.F) are later work.

## api_change_requests

None blocking. Non-blocking, additive suggestions (each "Open tier" under W0g):

1. crate `sm-mesh-api`: stable on-disk codes. What: `CellShape::code()/from_code()`,
   `FaceKind::code()/from_code()`, `Basis::from_code()` (only `Basis::code` exists). Why: the
   code tables of `.smsh` (shape, face kind, basis) are duplicated in this crate and in the test
   oracle; a second exporter (Exodus, CGNS) would triple them. Proposed:
   `impl CellShape { pub const fn code(self) -> u8; pub const fn from_code(c: u8) -> Option<Self> }`
   and the same for `FaceKind`, `Basis::from_code(u8) -> Option<Basis>`.
2. crate `sm-mesh-api`: `finish_with_ids` (already deferred by W0a decision 5). What/why: needed
   for a `.smsh` importer that preserves ids, classification and set ids so a loaded mesh hashes
   identically to the saved one. Proposed signature left to the orchestrator; the reader here is
   ready (`SmshMesh` holds every column).
3. crate `sm-mesh-api`: `ExtrusionMap::n_extruded()` / `n_base()` accessors. Why: the writer
   must know how many `cell_of` entries to store when the file's mesh is the base mesh; it
   currently derives the count from `cell_count * n_layers`. Proposed:
   `pub fn n_extruded(&self) -> usize; pub fn n_base(&self) -> usize`.
4. crate `sm-model-api` (or `sm-fem`): a lowering helper from `Deck` boundary records to named
   mesh face sets (through classification), producing the `face_set` / `mate_set` names that
   `DeckRecord` references. Not needed by this crate to compile or test.
