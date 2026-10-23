# sm-mesh-api: design (candidate: Hasse / DMPlex style)

## Model

One **point space** stratified by dimension: vertex (0), edge (1), face (2), cell (3). A `Point` is
`(Stratum, u32 index)`; points order by `(stratum, index)`, which is the canonical order. Stable
identity is a separate column: `entity_id(p) -> EntityId` (`MeshVertex/Edge/Face/Cell`, serial =
`base + index`), so the dense index is a derived column as ir.md §7.2 requires.

Topology is two CSR relations, each with per-incidence **orientation** (`Orient::{Fwd, Rev}`):

| Relation | Content |
|---|---|
| `cone(cell)` | oriented faces. `Fwd` = normal points out of the cell (cell is `left`), `Rev` = cell is `right` (CGNS NFACE sign) |
| `cone(face)` | ordered vertex loop. It **skips the edge stratum**: polygons of any size need no edge decomposition |
| `cone(edge)` | two vertices (low, high) |
| `support` | the transpose: `support(face)` = `[left, right?]`, `support(vertex/edge)` = faces |
| `face_edges(f)` | edges in loop order with orientation (derived, always built by the reference) |

Departure from pure DMPlex: faces cone straight to vertices (face-first, polyhedra-first) and
edges are a derived stratum. Reasoning: the transport/MC consumers read `face_to_vertex` and
`face_left/right`; a face->edge->vertex hop would cost them and gives nothing for non-planar polygons.

**Labels** (`MeshLabels`) are secondary indexes `name -> Point -> i32` (DMLabel). Sets, material
ids and boundary sets are label strata. No separate set type.

## Orientation rule (declared, validated)

Face loop right-hand normal; `left` = lower cell `EntityId`; normal points left -> right; boundary
faces have `right = None` and the normal points outward. Enforced in two places:
`MeshEdit::add_cell` (callers give only **outward-wound loops**; cells are appended in increasing id,
so a pre-existing face makes the new cell `right`, a new face makes it `left`, and a same-side or
different-cycle loop is a typed error) and `validate()` (implementation-independent: side order,
cone signs vs sides, per-cell closedness of directed loop edges, shape tags, positive volume,
classification order, dangling labels).

## Key types

* Read traits (object safe, slices/Vecs only): `MeshTopology`, `MeshGeometry`, `MeshClassification`,
  `MeshLabels`; `MeshView` is the blanket union. `MeshMetrics` adds derived face area vector /
  centroid and cell volume / centroid under the default `smartmesh.fan_vertex_mean.v1` rule.
* Edit trait: `MeshEdit` (`add_vertex/add_cell/add_node/set_cell_order/classify/classify_edge/
  set_label/finish`). Consumers and builders are separate types: `reference::MeshBuilder` ->
  `reference::Mesh`.
* `CellShape {Tet,Hex,Wedge,Pyramid,Polyhedron}` is a tag checked against `CellShape::detect`
  (vertex count + face-size multiset); `FaceShape` is derived from loop length.
* `Classification {entity: EntityId, param: Param}`; `points_on(model)` is the inverse index.
* `CellGeometry {degree, basis}` per cell; extra high-order nodes via `NodeRef` (vertex or extra),
  `cell_nodes` is empty for degree 1.
* `MeshCollection` + `FineToCoarse` (hierarchy); `InstancedView` + `Instance` + `FlattenMap`.
* `content_hash` / `validate` are free functions over `&dyn MeshView`, so they check any implementor.

## Requirements

* **Face-first, oriented**: above. **Polyhedra general, fast paths**: tags are validated, standard
  shapes have fixed counts (`CellShape::counts`), cones are contiguous slices.
* **Classification**: every stratum, edges via `classify_edge(a, b)` resolved at `finish`.
  `validate` requires `dim(model) >= stratum dim` and `dim(class(lower)) <= dim(class(upper))`.
* **Geometry**: SoA coordinates (`coords_soa`), versioned `geometry_rule()`.
* **Hierarchy**: `FineToCoarse` holds a cell map (and optional face map, `NONE` = interior to a
  coarse cell) and is checked against both meshes: lengths, ranges, and that a mapped coarse face
  separates exactly the mapped coarse cells.
* **Instancing + flatten**: `flatten` replays instances in order through any `MeshEdit`, re-emitting
  outward loops. For an **improper** transform it reverses every loop (the transformed copy would
  otherwise wind inward); the builder then re-applies the left/right rule, so the unfolded seam is
  one interior face with `left` = lower id. `weld_tol` shares vertices fixed by the transform (mirror
  plane / axis) so copies stitch. `FlattenMap` records instance -> cell range and cell origin.
* **Read vs edit**: read traits take `&self` and never allocate on hot paths (`cone`, `support`,
  `face_sides`); edit lives only on `MeshEdit`.
* **Determinism**: no hash containers anywhere; faces numbered by first use in ascending cell order,
  loops start at the smallest vertex, edges sorted by vertex pair, supports ascending, labels in
  `BTreeMap`; `content_hash` (FNV-1a over canonical columns) is tested equal across rebuilds and flatten.

## Tradeoffs / open issues

* `Point` is a unified type, so stratum mistakes are runtime (`debug` panics / `InvalidPoint`), not
  compile-time. Typed wrappers can be layered on top.
* CSR slices in the contract rule out lazy or computed topologies (e.g. a virtual instanced view);
  those must go through `flatten`.
* Append-only builder: no deletion / refinement / deactivation yet (ids are never reused; inactive
  parents need an `active` column). Face standard-shape local ordering is not canonicalised;
  FE per-cell permutation words are a derived cache, not in this crate.
* High-order: node ordering per `(shape, degree, basis)` is opaque here; curved faces on polyhedra
  (sparse side table) and flatten of high-order cells are not implemented.
* Edge labels are unsupported (edges are derived); edge classification is by endpoints.
* Not covered: interface records / overlap tables, partitions/ghost columns, units, 2D meshes,
  on-disk codecs.
