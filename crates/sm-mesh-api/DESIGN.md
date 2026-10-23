# sm-mesh-api: design (candidate "tables")

Style: MOAB/STK-like **typed columnar tables**. A mesh is one SoA table per dimension (vertex, edge,
face, cell) joined by CSR relations. Extra data are typed tag columns and sets with a selector
algebra. A small serializable **schema** declares what an instance must contain; the **instance**
is the tables. Consumers see object-safe read traits; producers see builder/edit traits.

## Model

| Table | Columns |
|---|---|
| vertex (nodes) | `id`, `x,y,z` (f64 SoA), classification (+ parametric coords) |
| edge (optional) | `id`, `[v0<v1]`, classification. Derived, sorted by vertex pair |
| face | `id`, `face_vertices` CSR (loop), `face_left`, `face_right` (none sentinel), classification, sparse curvature side table |
| cell | `id`, shape tag, `degree`, `basis`, `cell_faces` CSR of `SignedFace`, `cell_corners` CSR (templated shapes), `cell_nodes` CSR (degree > 1), classification |
| tags | `TagColumn { def, data: I64 / F64 / Id }`, one row per entity of `def.dim`, width >= 1 |
| sets | `EntitySet { dim, sorted members }`, evaluated/combined by `Selector` |

Ids are `sm_core::EntityId`; the dense `u32` index is the position in the id-sorted column
(canonical order = ascending id), so `index_of` is a binary search. CSR offsets are `u64`.

## Key types

- `MeshTopology`, `MeshGeometry`, `MeshClassification`, `MeshTags`, blanket `MeshRead`: read-only,
  object safe (`&dyn MeshRead`). Provided methods: `next_cell` (mesh walking), `cell_neighbors`,
  `outward_area_vector` (sweeps), `face_area_vector`, `cell_volume_centroid` (fan rule).
- `MeshBuilder` (producers: add vertices, shaped cells, polyhedra, classification, tags, `finish`)
  and `MeshEdit` (modifiers: move points, reclassify, tags/sets, `apply_transform`).
- `CellShape` (`Tet/Hex/Wedge/Pyramid/Polyhedron`), `FaceShape` (`Tri/Quad/Polygon(n)`),
  `CellSpec { shape, degree, basis }`, `Basis`.
- `Classification { entity: EntityId, param: Param }`, `ClassTable` (column + deterministic reverse index).
- `MeshSchema { version, geometry_rule, tags, sets }`, `TagDef`, `SetDef`.
- `Selector`: `All, Nothing, Set, Shape, ClassifiedOn, Boundary, Interior, TagEq, Union, Intersect,
  Difference, Complement`.
- `Hierarchy<M>` + `LevelMap` (fine to coarse, derived inverse CSR, `restrict_sum`, `prolong_copy`,
  `compose`); `InstancedMesh<M>` + `Instance` + `flatten_into(builder, FlattenOptions)`.
- `validate()` (contract checker over any `MeshRead`) and `canonical_hash()` (FNV-1a over canonical order).
- `reference::{Mesh, RefBuilder}`: the in-memory implementor.

## How each requirement is met

- **Face-first, oriented**: cells are added with *outward* loops; `finish` merges faces by sorted
  vertex key (`BTreeMap`) and stores `cell_faces` (signed) and `face_left/right`. Sign `Plus` = cell is
  `left` (CGNS NFACE `+`). Tet/hex/wedge/pyramid add templates (`local_faces`) and `cell_corners` on top
  of the same tables; faces are `Tri`/`Quad` fast paths via `face_shape`.
- **Orientation rule** (`orient`): loop right-hand normal; left = lower cell id; normal points left to
  right; boundary `right = None`, normal outward. The builder visits cells in ascending id, so the first
  user of a face is `left`; a second user must present the reversed loop or `finish` fails with
  `InconsistentOrientation`. A third user is `NonManifoldFace`. Loops start at their smallest vertex.
- **Classification**: every dimension has a `ClassTable`; rule "entity dim <= model dim" is enforced on
  set; vertices carry `Param::Curve(u)` / `Surface([u,v])` (interpreted by the target's kind).
  `entities_on(dim, model)` is the reverse query. Faces/edges are derived, so the builder classifies
  them by vertex set (`classify_face`, `classify_edge`) or `MeshEdit::set_classification` afterwards.
- **High order**: per-cell `(shape, degree, basis)`; `cell_nodes` lists all nodes (corners first) with
  `CellShape::num_nodes`; high-order nodes live in the vertex table so they can be classified.
  Curved polyhedral faces use a sparse `FaceCurve` table.
- **Hierarchy / instancing**: `Hierarchy` keeps independent meshes plus validated `LevelMap`s.
  `flatten_into` replays instances (ascending instance id) through any `MeshBuilder`, so the orientation
  rule is re-applied by construction; `weld_exact` merges bit-identical vertices so coincident faces
  across a mirror plane become one interior face. Returns instance cell ranges, vertex maps and a loss
  report (sets, non-cell tags, curved faces).
- **Improper transforms**: polyhedra loops are reversed (XOR with the cell's sign); templated cells get
  corners permuted by `mirror_permutation` so Jacobians stay positive and local face order still follows
  the template. `MeshEdit::apply_transform` does the same in place (left/right and signs unchanged).
  High-order cells under a mirror return `Unsupported` rather than being silently wrong.
- **Read vs build**: consumers take `&dyn MeshRead`; nothing in it mutates. Topology is immutable after
  `finish`; topological changes are IR to IR passes through a builder.
- **Determinism**: only `Vec`, `BTreeMap`, `IndexMap` with insertion order; all lists ascending; face
  ids by first appearance in `(cell, local face)` order; edges sorted by vertex pair; `canonical_hash`
  tested across runs and threads.

## Tradeoffs

- Schema/instance split is light: the schema only declares tags/sets/rule; shapes and relations are
  fixed by the contract. Cheap to evolve (`version`), but not a general table description language.
- Edge/face tags and sets cannot be given in the builder (those entities do not exist until `finish`);
  use `MeshEdit`. Face/edge classification in the builder is keyed by vertex set for the same reason.
- Cell ids equal insertion order and ids are `serial = index` in the reference; real id assignment
  (canonical sort per op, inactive parents after refinement) is left to the engine. No `active` column yet.
- `SignedFace` packs sign in a `u32` (max 2^30 faces in the builder); vertex/cell indices are `u32`.
- Fan-rule geometry is computed on demand (no cache columns); a caching implementor can override.
- Mixed face-key dedupe assumes one face per vertex set; two distinct polygons on the same vertex set
  are rejected as inconsistent.
- Tags are I64/F64/Id only; `Selector::TagEq` tests the first integer component.
- `Classification` is not serde-derivable because the frozen `EntityId` has no serde impl (persist raw).

## Open

- 2D/1D meshes, mixed-material cells, interface records and overlap tables (separate crate/module).
- Backend profiles (mmap / Zarr / HDF5) and partition/ghost columns: schema hooks only.
- Wedge/pyramid high-order node ordering conventions per `Basis` are not yet specified.
