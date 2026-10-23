# sm-mesh-api: face-first candidate

## Model

OpenFOAM/CGNS-style **face-first** core. The backbone is three CSR/column tables:

- `face -> vertex loop` (right-hand rule gives the normal),
- `face_left`, `face_right` (`right = None` on the boundary),
- `cell -> [CellFace { face, side }]` where `side` is the CGNS NFACE sign (`Left` = outward, `Right` = inward).

Everything else is derived or a side table: cell shape tags (derived from face sizes and vertex count, re-checked by `validate`), cell ordered-vertex lists (only for fast-path cells built from a vertex list), the optional `EdgeTable` (derived, with its own classification column), classification, sparse high-order records, sets, tag columns.

**Orientation rule** (ir.md section 6): the face loop's right-hand normal points left -> right; `left` is the lower cell id; boundary faces have normal outward. It is enforced in one place: `MeshBuild::finish`. Builders accept cells whose `side` is relative to the face's *current* loop; `finish` reverses the loop where the lower cell sees it inward, and rejects two cells that disagree (`InconsistentOrientation`), more than two cells, and dangling faces. No geometry is needed, so it works for polyhedra and for improper transforms.

**Ids and canonical order.** Stable `EntityId`s (kinds `MeshVertex/Face/Cell/Edge`, `Group` for sets) come from a shared `SerialAlloc`, so ids stay unique across hierarchy levels and instances. In a finished mesh dense index order equals ascending id order; "lower cell id" = "lower cell index". `finish` sorts faces by `(left, right-or-none, loop)`, starts loops at the smallest vertex, and sorts cell->face lists. Vertex and cell order are the mesher's contract (it owns the canonical sort of its op). `content_hash` is bit-exact FNV-1a over everything stored; tests show it is independent of face creation order.

## Key types

| Item | Role |
|---|---|
| `MeshRead` (object safe) | consumer queries: slices for CSR, `across(CellFace)` for O(1) mesh walking, `classified_on`, sets, tags, edges |
| `MeshBuild` | mesher API: `add_vertex/add_face/find_face/add_cell`, plus provided `add_polyhedron` (outward loops, faces deduplicated by vertex cycle) and `add_cell_from_vertices(shape, verts)`; `finish(&mut SerialAlloc)` |
| `MeshEdit: MeshRead` | topology-preserving edits (move points, reclassify, high order, sets, tags, edges) |
| `CellShape` / `FaceShape` | tags plus templates (`local_faces`, `reflect_permutation`) for Tet/Hex/Wedge/Pyramid and Tri/Quad |
| `Classification { entity, param }`, `Param`, `ClassSet` | every vertex/face/cell lies on a `Topo*`/`PartitionCell` id with optional (u) or (u,v); reverse index per model entity |
| `HighOrderCell`, `Basis`, `CellGeometry`, `NodeRef` | sparse (degree, basis, node list); coords SoA = vertices then extra nodes |
| `MeshCatalog` / `CatalogEdit`, `CellMap` | meshes by level; fine->coarse total map with inverse CSR; link checks sizes, level+1, equal classification |
| `InstancedView`, `Instance`, `flatten`, `FlattenMap` | prototype + placements; deterministic expansion (ascending instance id) with `locate(cell) -> (instance, local cell)` |
| `append_transformed`, `unfold` | generic over any `MeshBuild`; the improper-transform path |
| `validate`, `validate_geometry`, `geometry::*` | topology/orientation/closedness/template/classification checks; normals left->right and positive volumes; rule `smartmesh.fan_vertex_mean.v1` |
| `reference::{ReferenceMesh, ReferenceBuilder, ReferenceCatalog}` | in-memory oracle; `finish` runs `validate` |

## Requirements

- **Face-first, stored orientation**: tables above; `validate` checks that `cell_faces` sides and `face_left/right` agree both ways.
- **Polyhedra general, fast paths**: `add_polyhedron` is the general entry; shape tag is derived; `add_cell_from_vertices` stores template-ordered vertices which `validate` checks against the outward loops. Tri/Quad are face tags (`face_shape`).
- **Classification**: required at `finish` (`Unclassified` error); boundary faces must be on a `TopoFace`; cells on a volume/partition cell; reverse queries via `classified_on`. Boundary sets are therefore classification, not a second source of truth. Sets/tags are annotations.
- **Geometry / high order**: f64 SoA; per-cell (shape, degree, basis) via `cell_geometry`; default linear.
- **Hierarchy**: catalog with levels and `CellMap`; instanced view and `flatten`; `weld_tol` makes abutting instances conforming.
- **Mirror unfold**: `append_transformed` rebuilds each cell from its *outward* loops. If `det < 0` every loop is reversed and fast-path vertex order is permuted (`reflect_permutation`); `finish` then re-applies the left/right rule, so shared plane faces get left = original cell, normal = mirror direction. Tested, including a negative control (inside-out hex fails `validate_geometry`).
- **Read vs build/edit separation**: three traits; consumers never see builder methods.
- **Determinism**: `BTreeMap`/sorted vectors only, no hash iteration; canonical face order; ids from a counter; `content_hash` tests.

## Tradeoffs

- Cell->face signs are redundant with `face_left/right` (kept, per the "stored oriented" requirement, to avoid a branch in sweeps); `validate` guards against drift.
- Ordered vertices exist only for cells built from a vertex list; a hex built from face loops is tagged `Hex` but has no template order. Cheap and honest, but a FE consumer must check `cell_ordered_vertices`.
- Face dedup in the builder is by vertex set + cyclic order; twisted duplicates are separate faces.
- `cell_vertices_sorted`, `boundary_faces`, geometry helpers allocate: cold-path conveniences, not sweep inner loops.
- The reference `find_vertex` is O(n); a real backend needs a deterministic grid.

## Open issues

- 2D/1D/RZ meshes (ir.md section 7) are not modelled; faces need >= 3 vertices.
- High-order nodes cannot be mirrored (reference-element permutation tables per basis not defined); `append_transformed` errors. Face-level curvature tables and the addendum's sparse curvature table are not separate yet.
- `flatten`/`unfold` do not carry sets/tags; interface records (non-conformal L7) and deck-pairing lowering are not in this crate.
- Mirror image classification is the caller's `remap_class`; deriving unfolded model entities belongs to the model layer.
- Refinement/coarsening (parents inactive, lineage) is not part of the traits yet; ids are append-only via `SerialAlloc`.
- Cached face/cell geometry arrays keyed by rule id are not stored; only the functions are provided.
