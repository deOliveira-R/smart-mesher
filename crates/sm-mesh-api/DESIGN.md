# sm-mesh-api: final design (frozen contract)

Base: the face-first candidate, plus the grafts and defect fixes of `docs/design/w0/mesh-api-recommendation.md`, as modified by `docs/design/w0/decisions.md`. This crate is a frozen contract after merge: only the orchestrator edits it.

## Model

OpenFOAM/CGNS-style **face-first** core. Three CSR/column tables are the backbone (offsets are `u64`, the on-disk width):

- `face -> vertex loop` (right-hand rule gives the normal),
- `face_left` (always present), `face_right` (`NO_CELL` if none), `face_kind`,
- `cell -> [SignedFace]`, a packed `u32` (`face << 1 | side`, at most 2^31 faces). The sign is the CGNS NFACE sign: `+` (Left) = the stored loop is outward for this cell.

Everything else is derived or a side table: the cell shape tag, template **corners** for every template-shaped cell (Tet/Hex/Wedge/Pyramid; supplied by the mesher, or derived at `finish` from the outward loops), the optional `EdgeTable` (fully classified when present), classification with reverse index (vertices, edges, faces, cells), sparse high-order cell records, sparse face-curve records, lineage columns (`cell_parent`, `cell_active`), named sets, typed tag columns, and an optional geometry cache.

**Orientation rule** (ir.md section 6, binding): the loop normal points from `left` to `right`; `left` is the lower cell id; one-cell faces have the normal outward. Enforced in one place, `MeshBuild::finish`, independent of insertion order: it reverses a loop when the lower cell sees it inward, rejects two cells that disagree, more than two cells, and dangling faces. No geometry is needed, so it works for polyhedra and improper transforms. Canonical order (`validate` checks it): ids ascend with the dense index; loops start at the smallest vertex; faces sorted by `(left, right-or-none, loop)`; cell->face lists ascend. Exporters permute (OpenFOAM: interior by `(owner, neighbour)`, then patches by classification; CGNS ParentElements = `(left, right)`, NFACE = signed face ids) and never flip.

**Face kinds.** `Interior` (two cells), `Boundary` (one cell, classified on a `TopoFace`), `NonConformal` and `PartitionCut` (one cell, reserved; classified on a volume/partition cell or `TopoFace`). Defaults are derived; `set_face_kind` marks the other two. `is_boundary_face` is true only for `Boundary`.

**Ids.** One `SerialAlloc` is shared by all meshes of a model. `finish` draws the mesh id, then vertex, face, cell, set and edge ids. Dense order equals id order, ids never collide across meshes.

**Classification is mandatory** for vertices, boundary faces and cells. `finish` fills an unset interior / non-conformal / partition-cut face (and an unset edge strictly inside the volume) from the cells around it when they share one classification, and errors otherwise. `Param` must match the entity kind (`None` always fits; `Curve` on `TopoEdge`, `Surface` on `TopoFace`, `Volume` on `TopoVolume`); a mismatch is rejected, never rewritten. `validate` also enforces `dim(sub-entity) <= dim(entity)` (vertex <= edge <= face), edges included, and that the reverse index matches the columns.

## Traits

| Item | Role |
|---|---|
| `MeshRead` (object safe, `Send + Sync`) | kind-indexed `count/id/index_of/classification`, required `columns()` (whole slices, no dyn dispatch in hot loops), `across(SignedFace)` O(1) walking, checked `other_cell`, `face_kind`, `cell_corners`, `high_order_cell`, `face_curve`, `cell_parent/cell_active`, `classified_on`, `edges`, `sets`, `tags`, `geometry_rule`, `geometry_cache` |
| `MeshBuild` | mesher/importer API. Two-phase `add_cell` (all checks before any change), `CellAdded {cell, faces, reused}`, `Finished {mesh, face_remap}`, atomic provided `add_polyhedron[_with_corners]` and `add_cell_from_vertices`, `set_class` (edges by vertex pair, resolved at finish), `set_face_kind`, `set_cell_parent`, `set_cell_active`, `set_high_order`, `set_face_curve`, sets, typed tags, quantized `find_vertex` |
| `MeshEdit: MeshRead` | topology-preserving: `set_point`, `add_extra_node`, `set_class` (all four kinds), `set_face_kind`, `set_high_order`, `set_face_curve`, sets, tags, `ensure_edges`, `drop_edges` (removes edge tags), `set_geometry_cache` |
| `geometry` | allocation-free free functions over `&dyn MeshRead` (below) |
| `LevelMap`, `MeshCatalog`, `CatalogEdit` | fine->coarse maps (`restrict_sum`, `prolong_copy`, `fines_of`, `compose`, optional validated `face_map`), catalog with `maps_from/maps_to` |
| `InstancedView`, `flatten`, `unfold`, `append_transformed`, `finalize_welds` | instancing, mirror unfold, welding |
| `validate`, `validate_geometry`, `connectivity_hash`, `topology_hash`, `geometry_hash` | checks with stable `ViolationCode`s; content hashes |
| `select` | `Selector` algebra and sorted-slice set operations as free functions |
| `reference::{ReferenceBuilder, ReferenceMesh, ReferenceCatalog, hex_block, hex_block_with_edges}` | in-memory oracle |

## Geometry rule `smartmesh.fan_vertex_mean.v1`

Face: apex = vertex mean, `A = 1/2 sum (p_i - apex) x (p_{i+1} - apex)`, centre = area-weighted fan centroid. Cell: the fan polyhedron; `V = 1/3 sum_f s_f A_f . (apex_f - o)` equals the sum of side tetrahedra `(o, apex_f, p_i, p_{i+1})` exactly (the cross term vanishes because each triangle's area vector is orthogonal to its edges from the apex), `o` = first vertex of the first face; centroid from the same tetrahedra. No `Vec`, no heap allocation (a counting-allocator test enforces zero allocations for `face_area_vector`, `cell_volume`, `across` over `&dyn MeshRead`).

## High order

`smartmesh.nodes.v1` = Gmsh ordering for template shapes up to degree 2: corners, then local edges (`CellShape::local_edges`), then quadrilateral faces (`local_quads`, Lagrange only), then the hex centre. `node_count(shape, degree, basis)` is checked everywhere (HEX27 vs HEX20 via `Basis::Serendipity`; TET10; WEDGE18/15; PYRAMID14/13); everything else is `Unsupported`, including mirroring high-order cells. Polyhedra cannot carry cell records; they curve through `face_curve` (loop vertices first, then edge nodes, then the Lagrange-quad centre), which `finish` permutes when it re-orients a loop. `MeshEdit::add_extra_node` lets a finished mesh be curved. Node ids are `PointIx`; the builder uses `NodeRef` because extra-node indices are not final until `finish`.

## Hierarchy

A coarse mesh is an independent mesh over the same model; no classification equality is required (CMFD nodes may sit on `PartitionCell`s). `LevelMap` stores both mesh ids and connectivity hashes and refuses a stale or foreign pair (`check_meshes`, `link`). The face map is validated against both meshes. A catalog may hold several maps per fine mesh.

## Instancing, welding, unfold

`flatten` places instances in ascending id order. Welding is global on a quantized grid (`BTreeMap`, lowest index wins) but **across instances only** and **only on source boundary vertices**, and a vertex can only be merged into a *boundary* vertex (one on a one-cell face) of an earlier instance, never an interior one. Source faces map 1:1 into the output, so a topological baffle (two one-cell faces on the same vertices) stays two faces, also under `NonConformal` marks; a face is shared only with an earlier instance and only if its free side matches. A face shared with an earlier instance becomes interior and is reclassified through `FlattenOptions::on_merge` (default: the cells' common classification, else `ClassConflict`), recorded in `FlattenMap::merged_faces`; welded vertices are resolved by `finalize_welds` (interior => shared cell classification; still on the boundary => equal classes keep, otherwise `on_merge`/error). `remap_class(instance, class)` applies to vertices, faces and cells. **Caller's responsibility:** `on_merge` is not consulted when both sides of a welded boundary vertex already agree, so a vertex that stays on the union boundary keeps a classification on a vanished symmetry entity (the `TopoEdge` where the symmetry face meets the top face, `TopoVertex` on the quarter-unfold axis). Map these with `remap_class` or reclassify afterwards; the default output carries stale model references. `unfold(src, deck, ..)` builds the images of all subsets of the deck (identity implicit): a 2-mirror deck on a hex gives 18 vertices / 4 interior faces. Sets, tags, the edge table and face curves are not carried and are reported as `LossNote`s.

## Hashes

`connectivity_hash`: ids, counts, loops, left/right, face kinds, signed cell faces, shapes, corners, lineage (keys the geometry cache and level maps). `topology_hash`: connectivity plus class entities, the edge table and its classification, high-order and face-curve node indices, set members, integer tag values, real tag names. `geometry_hash`: coordinate bits (vertices and extra nodes), all `Param` bits, real tag values, rule id. The geometry cache is derived and excluded.

## Tradeoffs and limits

- Cell->face signs duplicate `face_left/right` (kept so sweeps avoid a branch); `validate` guards against drift.
- A template cell built from loops gets *a* valid corner order (first match of a fixed search); symmetric shapes have several.
- Face dedup in the builder is by vertex set + cyclic order; twisted duplicates are separate faces.
- 2D/1D/RZ meshes are not modelled; faces need >= 3 vertices.
- Inactive parents keep their faces; consumers filter by `cell_active` (the faces of a refined parent are not marked inactive).
- Non-conformal interface records, partition-local views (ghosts) and refinement edits belong to later crates; only `FaceKind` slots and lineage columns are reserved.
- No `rayon` here: all algorithms are sequential and deterministic (`BTreeMap`/sorted vectors, no hash-order dependence), so 1 thread and N threads are trivially identical.

## Failure atomicity

`MeshBuild::finish`, `MeshEdit::ensure_edges`, `MeshEdit::add_set`, `flatten` and `unfold` leave the `SerialAlloc` unchanged on `Err`: ids are what the model history records, so a failed attempt must not shift the ids of a later successful one (op decision 4, strict Diverged rule). The reference builder runs `finish` against a clone and copies it back on `Ok`; `EdgeTable::derive` draws edge ids only after every check; `flatten`/`unfold` work on a clone. `append_transformed` is **not** atomic (it takes `&mut B`); discard the builder after an error.

## Edits, edge classes, caches

- `MeshEdit` checks the rules local to one entity (parameter fits, `Boundary` face stays on a `TopoFace`, cell on a volume, edge tag needs a table). Rules relating entities (dimension order) are not checked: run `validate` after classification edits. `set_classes` is the batch form (one index rebuild, all-or-nothing).
- `EdgeTable::derive`: explicit class, else the entity shared by all incident faces, else (all faces interior) the shared cell entity, else `UnclassifiedEdge(a, b)` naming the vertex pair.
- `MeshBuild::set_face_kind(f, Boundary)` clears a NonConformal/PartitionCut mark; welding a seam clears it automatically.
- `GeometryCache` carries `connectivity_hash` and `coordinate_hash`; `validate` and `set_geometry_cache` check both. (`geometry_hash` is not used for this: it also covers tags and parameters, which do not affect the cache.)
- `MeshRead::face_active` is false for the faces of a refined parent; filter with it before integrating over `classified_on(TopoFace)` or boundary faces.
- Hash change: `topology_hash` now prefixes the set and tag lists with their counts.

## Features

`serde` derives (de)serialization on the plain value types (`EntityId` as `Kind#serial`): indices, `SignedFace` (raw `u32`), `Classification`, `HighOrderCell`, sets and tags, `EdgeTable`, `GeometryCache`, `Instance`/`InstancedView`, `Violation`/`ViolationCode`, `LossNote`, and `SerialAlloc` (also `state()`, `next_serial()`, `from_state()` for snapshots). `schemars` implies `serde`, passes through to `sm-core` and derives `JsonSchema` on the same types. Not serialized: `ReferenceMesh`, builders, `ClassSet` (derived), callbacks and `*Map` results.

## Deviations from `mesh-api-recommendation.md` section 5

1. `GeometryCache` and `LevelMap` are keyed on `connectivity_hash`, not `topology_hash`, so reclassifying or tagging does not invalidate them. The cache also keys on `coordinate_hash`.
2. `MeshBuild::set_high_order` / `set_face_curve` take `NodeRef` (vertex or extra), because extra-node indices are not final until `finish`.
3. `CellAdded` carries `reused`, so meshers and `append_transformed` need no second `find_face`.
4. `on_merge` is a `MergeRule` taking `&MergeCtx` (kind, existing, incoming, cells) rather than separate arguments.
5. `ensure_edges` takes explicit classes by vertex pair, plus derivation from faces.
6. `FlattenMap.instance_vertices` added next to `vertex_origin` for per-instance vertex lookup.
7. An extra `connectivity_hash` (and `coordinate_hash`) beyond the two recommended hashes.
8. Face limit: `MAX_FACES = 2^31` faces are addressable and enforced exactly.
9. `MeshRead::tags()` is required and `tag(kind, name)` is a provided lookup over it (the recommendation has `tag()` required).
10. `MeshRead::cell_parent` is required (orchestrator decision 4); `cell_active` and the provided `face_active` complete the lineage view.
11. `MeshEdit::set_class` takes a `MeshEntity` (kind plus index in one value) rather than `(kind, ix)`.
12. Extra trait methods beyond the recommendation: `MeshEdit::{set_classes, set_face_kind, drop_edges}`; `MeshBuild::{set_face_kind, set_cell_parent, set_cell_active, truncate_faces, face_cell, vertex_cell, vertex_class, face_class, cell_class, face_loop, face_degree, find_face, find_vertex, cell_faces, add_extra_node}`; `MeshRead::face_active`.
13. `EdgeTable::face_off` is `Vec<u64>` (same width as the on-disk `TopologyColumns` offsets), so more than 2^32 face-edge incidences cannot wrap.
14. `SerialAlloc` deserializes through `from_state` (`serde(try_from)`), so a bad snapshot is rejected at load time.
