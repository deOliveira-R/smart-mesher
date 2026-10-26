# sm-meshio: design

## Positioning

`sm-meshio` is similar to **meshio** (Python) and the writers inside Gmsh, Cubit and MFEM: it
lowers an in-memory mesh to files that other tools read. It differs in four ways. (1) It exports
from the face-first IR of `sm-mesh-api` through the read trait only, so polyhedra, mixed cells,
dimension-2 meshes, classification, named sets with integer ids, tags, lineage, hierarchy and
extrusion maps, deck transformations and provenance all have a defined place, instead of being
squeezed through a cell-type list. (2) Its native `.smsh` container is a JSON header plus
64-byte-aligned little-endian arrays (the safetensors idea): memory-mappable with zero copies,
readable in about 80 lines of Python, checksummed per array, with no HDF5 dependency, where
Exodus II / CGNS / `.cub5` need a C library and have no polyhedral orientation, ownership or
hierarchy records. (3) Output is a pure function of the mesh: byte-identical across runs and
thread counts, while the writers are data-parallel (rayon over fixed-size chunks). (4) Every
claim about a foreign format is tested against that format's own reader (VTK's XML reader for
VTU), not just against our reader. This gives a transport code a lossless, fast mesh file today
(polyhedral cells, oriented cell->face signs, left/right, regions, boundary labels), and
ParaView a faithful picture of the same mesh, with Exodus / CGNS / Gmsh exporters left to slot
in next to them.

The OpenFOAM `polyMesh` writer (`foam`) is similar to OpenFOAM's own converters (`gmshToFoam`,
`fluent3DMeshToFoam`, `ideasUnvToFoam`), which rebuild `owner` / `neighbour` by matching the
faces of cell-shape lists. It differs because it derives polyMesh directly from the face-first IR,
whose left / right / outward-loop rule already is OpenFOAM's owner / neighbour / normal convention:
there is no face matching, no flipping, no cell renumbering, polyhedra and true 6-node prisms stay
exact, and patches and cellZones come from the mesh's own sets and classification. This gives an
export in which a `checkMesh` failure points at the exporter (our validator already approved the
mesh), a loss report saying what OpenFOAM could not carry, and an in-process parser and checker
(`foam::parse_polymesh`, `foam::check_polymesh`) that find exporter bugs without OpenFOAM.

## Layout

| module | role |
|---|---|
| `smsh::format` | header schema (serde), code tables, `DeckRecord` (from `sm_model_api::Deck`) |
| `smsh::writer` | `SmshWriter`: builder over `&dyn MeshRead`, `to_bytes` / `write_to` / `write_file` |
| `smsh::reader` | `SmshFile` (zero-copy typed views, CRC check), `SmshMesh` (full owned decode), `SmshMap` (feature `mmap`) |
| `vtu` | `write_vtu` / `vtu_bytes`, `VtuOptions`, `VtuEncoding` |
| `report` | `LossReport` / `LossItem` / `LossSeverity` shared by every exporter, `vtu_loss_report` |
| `foam::build` | `build_polymesh`: face reordering, patch and zone assignment, `FoamReport` |
| `foam::write`, `foam::tar` | `PolyMesh::files` / `write_dir` / `write_tar`, `write_polymesh` |
| `foam::parse`, `foam::check` | `parse_polymesh` / `read_polymesh_dir`, `check_polymesh` |
| `le`, `crc` | little-endian packing, CRC-32 (slicing-by-8) |

Decoded edge tables and sets use the contract types (`sm_mesh_api::EdgeTable`, `MeshSet`).
`DeckRecord` is deliberately not the serde form of `sm_model_api::Deck`: the file format is
specified independently of any Rust type's serde shape (see `SMSH_FORMAT.md` 2.7), so a model
refactor cannot silently change what is on disk.

The byte-level spec is `SMSH_FORMAT.md`. The reference reader is `readers/smsh.py`.

## `.smsh`

* **Container.** Magic, `u64` header length, JSON header padded so the data region starts at a
  multiple of 64, then arrays at 64-byte-aligned offsets. Offsets are relative to the data
  region, so the header can be written after the arrays' sizes are known without a fix-up pass.
* **Content.** The face-first tables verbatim (`face_offsets/vertices/left/right/kind`,
  signed `cell_faces`), plus template corners and shape tags (so tet/hex readers need no face
  walk), lineage columns, ids, classification as indices into a table of the model entities in
  use (with optional parameter arrays), the optional edge table, sparse high-order cell and face
  curve records, the cached geometry, sets and tags with names and labels, hierarchy maps,
  decks, provenance and the three mesh-API content hashes. Nothing is derived by the writer
  except the entity table and the CSR layout of already-sorted data.
* **Writer.** Reads `MeshRead::columns()` slices (no per-element dynamic dispatch for the big
  tables). Each array is packed by `pack` (fixed 16K-element tasks into pre-sized buffers) or
  `csr` (blocks of 4096 rows fill disjoint windows of a pre-sized buffer), so there is no
  per-row allocation and the result does not depend on the schedule. Coordinates are borrowed as
  bytes on little-endian hosts. CRC-32 runs per array in parallel; the three mesh hashes run side
  by side (`without_hashes()` skips them, they cost more than the rest of the write).
* **Reader.** `parse` validates the table (bounds, alignment, sizes, uniqueness) without
  touching array data. Typed views are `Cow`: borrowed when aligned on a little-endian host,
  decoded otherwise. `decode` validates CSR offsets and index ranges and rebuilds API value types
  (`Classification`, `SetMembers`, `TagColumn`, `HighOrderCell`, `GeometryCache`).
  There is no `finish_with_ids` in the mesh API (W0a decision 5), so the reader returns an
  owned `SmshMesh` rather than a `ReferenceMesh`; mesh import is deferred (ir.md 5.3).
* **Why not HDF5 / Exodus now.** The array schema does not depend on the container; an HDF5 or
  Zarr mapping can reuse the names, dtypes and shapes. The container is what makes the
  format trivially readable from C++/Fortran/Python and mmap-fast.

## VTU

* Template cells keep their node order (VTK's order for tet, hex, wedge and pyramid equals the
  template order of `sm-mesh-api`; verified against VTK's signed cell volumes and outward face
  tables). Polyhedra are `VTK_POLYHEDRON` with sorted distinct vertices as connectivity and the
  `faces` stream of outward loops (a `Right`-side face is reversed) and `faceoffsets`; the
  streams are omitted when there is no polyhedron. Dimension-2 meshes give triangles, quads and
  polygons.
* Arrays are built per block of 4096 cells in parallel and concatenated in block order; index
  arrays are `Int32` unless a count exceeds 2^31, then `Int64`.
* Binary data is appended, either raw or base64, uncompressed, `UInt64` block headers. In base64
  mode the header and the payload of each block are encoded separately (VTK's convention).
* Cell data: `region`, `class_entity`, `set:<name>`, `tag:<name>`, `active`. Point data:
  `class_dim`, `class_entity`, `set:<name>`, `tag:<name>`. `class_entity` is the serial of the
  classified model entity (the "color"); `region` is the label of the first labelled cell set.

## `.smsh` 1.1

`VERSION_MINOR = 1`; the reader accepts 1.0 and 1.1 (`tests/data/v10_hex_block.smsh`, written by
the unmodified 1.0 writer, pins that). 1.1 adds two optional header items and no arrays:

* `element_types: [{set, element_type, shape}]`, always written (possibly `[]`), ascending by set
  id. `SmshWriter::element_type(set, name)` declares the type of a cell set; the writer checks the
  set, the shape of every member, the high-order node count for `HEX20`-style names, one record
  per set and agreement of a cell that is in two typed sets. Spellings are decision 12's
  (`POLYHEDRON` / `POLYGON`); `NFACED` / `NSIDED` are exporter mappings and are `Unsupported`
  here. The decoder re-checks every record against the `cell_shape` array.
* `mesh.stats`, derived counts (cells by shape, faces by kind, `max_cell_faces`,
  `max_face_vertices`) for consumers that size ray-tracing or transport scratch from the header.
  `SmshFile::decode` recomputes them and refuses a file that disagrees. The writer and the decoder
  share one function (`MeshStats::from_arrays`), integer counting over fixed chunks.

No other polyhedral additions are needed: the `cell_faces` CSR, `cell_shape` code 4 and the
zero-length `cell_corners` rows already hold any closed polyhedron exactly.

## OpenFOAM

`foam::build_polymesh` reads the mesh through `MeshRead` only. Internal faces are those with a
right cell; if they are already in `(owner, neighbour)` order (always, for canonical meshes) they
are taken as is, else sorted by `(owner, neighbour, source index)`. Patch faces follow, each patch
in ascending source order, then the loop CSR is filled in 4096-face windows in parallel. The rules
for patches, zones, names and losses are in `FOAM_FORMAT.md`. `PolyMesh` keeps its own face CSR
(about 12 M `u32` per million hexahedra); `PolyMesh::write_tar` exists for `Write`-based
consumers, while the integration plan writes the case directory.

## Determinism

No hash-ordered iteration. Parallelism is only over fixed-size index chunks writing disjoint
output windows, and per-array independent jobs collected in order (`par_iter().collect()` keeps
order). Tests compare 1-thread and N-thread output by hash (`sm_testkit::determinism::check`)
for the whole corpus, both formats, and a block larger than every chunk size. The OpenFOAM files,
the tar stream, the index maps and the report, and `.smsh` 1.1 bytes with element types are
checked the same way (`tests/determinism.rs`), including a 64,000-cell block.

## Limits (see W1_REPORT.md)

High-order geometry is not exported to VTU (corners only; `vtu_loss_report` says so); VTU has no
face/side-set or boundary skin output; Exodus lives in `sm-exodus`, CGNS / Gmsh / DAGMC / Fluent
are W3b; `.smsh` import returns arrays, not a `MeshRead`. OpenFOAM: ASCII only, no binary, no
cyclic / AMI patches, dimension-3 meshes without inactive cells only (see `FOAM_FORMAT.md`).
