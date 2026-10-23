# sm-mesh-api: final design recommendation

## 1. Scores

Each cell shows judge 0 / judge 1 → mean.

| Criterion | hasse (DMPlex points) | tables (MOAB/STK columns) | facefirst |
|---|---|---|---|
| Expressiveness | 6 / 5 → 5.5 | 6 / 5 → 5.5 | 6 / 5 → 5.5 |
| Ergonomics | 6 / 6 → 6.0 | 6 / 6 → 6.0 | 7 / 6 → **6.5** |
| Performance | 6 / 6 → **6.0** | 4 / 5 → 4.5 | 6 / 6 → **6.0** |
| Determinism | 7 / 9 → 8.0 | 9 / 8 → **8.5** | 7 / 7 → 7.0 |
| Extensibility | 5 / 5 → 5.0 | 6 / 5 → **5.5** | 5 / 5 → 5.0 |
| Correctness | 6 / 6 → 6.0 | 7 / 6 → 6.5 | 8 / 6 → **7.0** |
| **Total (of 60)** | 36 / 37 → 36.5 | 38 / 35 → 36.5 | 39 / 35 → **37.0** |

The numbers are almost tied. The choice below rests on structure, not on the totals.

## 2. Winner: `facefirst` as the base, with large grafts

Why facefirst:
1. **Its storage matches what was asked for and what the transport consumer reads first.** The requirement says "face-first", and addendum §7.7 item 2 lists the first columns the in-house transport code reads. facefirst stores exactly those as primary data: face→vertex loops, `face_left`/`face_right`, and cell→face with a sign. hasse puts everything into one untyped `Point` space, so a stratum mistake shows up as a runtime panic. tables is close to facefirst but makes faces implicit in the builder.
2. **The orientation rule is enforced in one place, and that place does not depend on insertion order.** `MeshBuild::finish` takes each cell's side relative to the face's current loop and flips loops so that left is the lower id. In hasse and tables the rule holds only because cells are appended in id order, so it is an accident of the builder. Both judges flagged that renumbering breaks it.
3. **Importers can feed it directly.** `add_face` + `add_cell(&[CellFace])` takes OpenFOAM and CGNS NGON/NFACE data with no re-winding. Meshers use `add_polyhedron` or `add_cell_from_vertices`.
4. **Ids are unique across meshes.** One `SerialAlloc` is shared by every mesh of a model, and dense order equals id order. That is the only design where ids do not collide between hierarchy levels or flattened instances. Both other candidates collide (`MeshCell#0` appears in several meshes).
5. **Mirror unfold is correct by construction.** Each cell is rebuilt from its outward loops; an improper transform reverses the loops and applies `reflect_permutation` to template cells. There is a negative-control test. Classification is mandatory at finish, and boundary sets are derived from classification, so there is one source of truth.

## 3. Ideas to graft

**From tables**
- **Packed signed face.** Use `SignedFace(u32)` (bit 0 = sign) for cell→face, replacing the facefirst `CellFace { face, side }`. It halves storage, and the sign equals the CGNS NFACE sign. Keep `across()` and `side()` accessors on top of it.
- **Reverse classification index.** Adopt the `ClassTable`-style index per dimension: `entities_on(dim, model) -> &[u32]`, sorted. It must cover edges too.
- **Level-map operations.** Merge `LevelMap::{restrict_sum, prolong_copy, compose}` and the derived inverse CSR `fines_of` into the facefirst `CellMap`.
- **Flatten loss report.** Put `loss: Vec<LossNote>` in `FlattenMap` (dropped tags, sets, curved faces). Add per-instance vertex maps.
- **Face curvature side table.** Add `face_curve(f) -> Option<FaceCurve>` on the read trait (addendum §7.5) and `set_face_curve` on **MeshBuild and MeshEdit**, not as an inherent method.
- **Typed tag columns.** Use typed columns (`I64` and `F64` at least) keyed by `(kind, name)` in an `IndexMap`. Put the Selector algebra in a free-function module `select`, outside the traits.
- **Stable violation codes.** `validate` should return a `Violation { code, entity, detail }` with stable codes.
- **Template-cell tests.** Keep the tables test pattern: each template shape (tet, hex, wedge, pyramid) gets an analytic volume and is also mirrored in place.

**From hasse**
- **Two-phase `add_cell`** (plan, then commit), so an error leaves the builder unchanged.
- **Face map in the hierarchy.** Optional `face_map` in the level map, validated against `face_cells` of both meshes. CMFD surface currents need it.
- **Extra `validate` checks.** Directed loop edges must cancel per cell (closedness and watertightness), and the cone sign must agree with the stored sides.
- **Welding with a tolerance:** `weld_tol: Option<f64>`. Do not use the bit-exact weld from tables (it fails for oblique mirror planes).
- **Versioned geometry rule id** on the read trait (`smartmesh.fan_vertex_mean.v1`).

**Own additions, beyond all three candidates**
- **Bulk column access.** `MeshRead::columns() -> TopologyColumns<'_>` exposes the raw §7.7 arrays as whole slices, so hot loops can avoid dyn dispatch.
- **Geometry cache.** Add a `GeometryCache` (face area vector, centroid and apex; cell volume and centroid; keyed by rule id and topology hash) and `MeshRead::geometry_cache() -> Option<&GeometryCache>` (addendum §7.5 and §7.7 item 5).
- **Face kind.** `FaceKind { Interior, Boundary, NonConformal, PartitionCut }` keeps a slot for hanging-node faces and partition cuts. Without it, those faces would have to be falsely classified as model boundary.

## 4. Defects to fix (must fix before freeze)

1. **Geometry allocates in hot paths** (all three candidates). Rewrite the fan rule with no `Vec`: one pass over the loop for the vertex mean, one for the triangle sums. Compute volume as `V = 1/3 Σ_f s_f A_f · c_f` (addendum §7.5), which needs no cell-vertex set. Centroid uses the first face vertex as origin. Add a test with a counting allocator that requires 0 allocations for `face_area_vector`, `cell_volume` and `across` over `&dyn MeshRead`.
2. **Welded seam faces keep a stale classification** (all three). When flatten or unfold turns a boundary face into an interior face, it must reclassify it through `FlattenOptions::on_merge`. The default is the shared cell classification when both cells agree, and an error otherwise. `remap_class` must also apply to welded vertices. Record these events in `FlattenMap.merged_faces`.
3. **Welding is wrong for multi-mirror and translated copies** (hasse and tables). Weld globally on quantized coordinates within tolerance, deterministically: a `BTreeMap` on a quantized key, lowest id wins. Weld only across instances and only on boundary vertices, never inside one prototype (baffles). Tests: a quarter unfold must give 18 vertices / 4 interior faces, and two translated copies must give 1 interior face.
4. **`face_other_cell(f, c)` / `across` return the left cell for a non-incident cell.** `other_cell` should return `Result<Option<CellIx>, NotIncident>`. `across(SignedFace)` stays branch-free and gets a `debug_assert`.
5. **`finish` does not return a face remap.** It should return `Finished { mesh, face_remap: Vec<FaceIx> }` (provisional → final). The add-cell calls should return the faces they created or reused (`CellAdded`), so meshers do not need to call `find_face` again.
6. **`content_hash` leaves out data** (facefirst and hasse). Hash everything stored: Param bits, set members, tag values, high-order records, face curves, the edge table and its classification. Split it into `topology_hash` and `geometry_hash` (addendum §7.2: two checksums).
7. **High order is under-specified.**
   - Node count must be checked against `(shape, degree, basis)`, and `degree >= 1` required.
   - Add a `Basis::Serendipity` variant (HEX20 vs HEX27).
   - Name a node-ordering convention in the contract (see Q3).
   - `set_high_order` on a `Polyhedron` must be rejected; polyhedra curve only through `face_curve`.
   - `MeshEdit::add_extra_node` is needed so a finished mesh can be curved.
8. **Edges are second-class.** Add an `edges` field to `ClassSet`. Edge classification must be settable in the builder (by vertex pair, resolved at finish). `validate` must check the dimension order for edges (an edge on `TopoFace` with a vertex on `TopoVolume` must be rejected). `drop_edges` must also remove edge tags.
9. **A `Param` that does not match the target kind is silently rewritten** (tables). Reject it. Add `Param::Volume([f64; 3])`.
10. **Non-conformal faces are forced onto a `TopoFace`** (facefirst `validate.rs:176`). Allow a degree-1 face that is not classified on a `TopoFace` only when its `FaceKind` is `NonConformal` or `PartitionCut`.
11. **Hierarchy limitations** (facefirst).
    - `MeshCatalog::coarse_of` returns only one link. Replace it with `maps_from(fine) -> Vec<(EntityId, &LevelMap)>` plus `maps_to(coarse)`.
    - Remove the reference-only rule that a coarse cell's classification must equal its fine cells' (CMFD nodes span several volumes and may sit on a `PartitionCell`).
    - `LevelMap` stores both mesh ids and both topology hashes and rejects a stale use.
12. **Mesh id is user-supplied.** Draw it from `SerialAlloc` (facefirst `reference/builder.rs:43`). `catalog.views()` must be in id order, not insertion order.
13. **Reference performance.** `find_vertex` is O(n); use a quantized `BTreeMap`. Face dedup should use a `SmallVec<[u32; 8]>` sorted key, not a `Vec` per face.
14. **Doc and test fixes.**
    - Face limit is 2^31, consistent everywhere.
    - Fix the "panics on foreign index" docs.
    - Fix the existing tests that set degree 2 with 8 nodes or 2 nodes.
    - Document the OpenFOAM face permutation as an exporter step.
15. **Interior-face classification burden.** `finish` auto-classifies an interior face or edge when all incident cells share one classification, and errors otherwise. Classification stays mandatory.

## 5. Proposed public API (concise)

```rust
// index.rs
pub struct VertexIx(pub u32); pub struct EdgeIx(pub u32);
pub struct FaceIx(pub u32);   pub struct CellIx(pub u32);
pub struct PointIx(pub u32);              // coords index; [0, n_vertices) are vertices, the rest are HO nodes
pub enum Side { Left, Right }             // Left = this cell is the face's left cell, normal outward (+)
#[repr(transparent)] pub struct SignedFace(u32); // bit0 = side; max 2^31 faces
impl SignedFace { pub const fn new(f: FaceIx, s: Side) -> Self; pub const fn face(self) -> FaceIx;
                  pub const fn side(self) -> Side; pub const fn sign(self) -> f64; }
pub enum MeshEntityKind { Vertex, Edge, Face, Cell }
pub enum FaceKind { Interior, Boundary, NonConformal, PartitionCut }
pub const NO_CELL: u32 = u32::MAX;

// classify.rs
pub enum Param { None, Curve(f64), Surface([f64; 2]), Volume([f64; 3]) }
pub struct Classification { pub entity: EntityId, pub param: Param }  // Topo* or PartitionCell
pub struct ClassSet { pub vertices: Vec<VertexIx>, pub edges: Vec<EdgeIx>,
                      pub faces: Vec<FaceIx>, pub cells: Vec<CellIx> }

// shape.rs / highorder.rs
pub enum CellShape { Tet, Hex, Wedge, Pyramid, Polyhedron }
pub enum FaceShape { Tri, Quad, Polygon }
impl CellShape { pub fn local_faces(self) -> &'static [&'static [u8]];
                 pub fn reflect_permutation(self) -> &'static [u8];
                 pub fn detect(n_verts: usize, face_sizes: &[usize]) -> CellShape; }
pub enum Basis { LagrangeEquispaced, LagrangeGll, Serendipity }
pub struct CellGeometry { pub shape: CellShape, pub degree: u8, pub basis: Basis }
pub fn node_count(shape: CellShape, degree: u8, basis: Basis) -> Option<usize>;
pub struct HighOrderCell { pub degree: u8, pub basis: Basis, pub nodes: Vec<PointIx> } // smartmesh.nodes.v1 order
pub struct FaceCurve<'a> { pub degree: u8, pub basis: Basis, pub nodes: &'a [PointIx] } // loop vertices first

// read.rs — object safe, Send + Sync
pub struct Coords<'a> { pub x: &'a [f64], pub y: &'a [f64], pub z: &'a [f64] }
pub struct TopologyColumns<'a> {           // addendum §7.7 item 2, canonical order
    pub face_vertex_offsets: &'a [u64], pub face_vertices: &'a [VertexIx],
    pub face_left: &'a [CellIx], pub face_right: &'a [u32 /* NO_CELL */],
    pub cell_face_offsets: &'a [u64], pub cell_faces: &'a [SignedFace],
}
pub trait MeshRead: Send + Sync {
    fn mesh_id(&self) -> EntityId;
    fn count(&self, k: MeshEntityKind) -> usize;
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId;      // ids ascend with dense index
    fn index_of(&self, id: EntityId) -> Option<u32>;
    fn coords(&self) -> Coords<'_>;
    fn columns(&self) -> TopologyColumns<'_>;
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx];       // right-hand loop, starts at min vertex
    fn face_left(&self, f: FaceIx) -> CellIx;                // lower cell id
    fn face_right(&self, f: FaceIx) -> Option<CellIx>;
    fn face_kind(&self, f: FaceIx) -> FaceKind;
    fn cell_faces(&self, c: CellIx) -> &[SignedFace];        // ascending face index
    fn cell_shape(&self, c: CellIx) -> CellShape;
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]>; // template order; Some for every template-shaped cell
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell>;
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>>;
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification;
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet>;
    fn classified_entities(&self) -> &[EntityId];            // ascending
    fn edges(&self) -> Option<&EdgeTable>;
    fn sets(&self) -> &[MeshSet];                            // ascending id
    fn tag(&self, k: MeshEntityKind, name: &str) -> Option<&TagColumn>;
    fn geometry_rule(&self) -> &str { FAN_VERTEX_MEAN_V1 }
    fn geometry_cache(&self) -> Option<&GeometryCache> { None }
    fn cell_active(&self, _c: CellIx) -> bool { true }       // slot reserved for inactive parents (§7.2)
    // provided, all allocation-free:
    fn across(&self, sf: SignedFace) -> Option<CellIx>;
    fn other_cell(&self, f: FaceIx, c: CellIx) -> Result<Option<CellIx>, MeshError>;
    fn is_boundary_face(&self, f: FaceIx) -> bool;
    fn cell_geometry(&self, c: CellIx) -> CellGeometry;
}
pub struct GeometryCache { pub rule: String, pub topology_hash: u64,
    pub face_area: Vec<Vector3>, pub face_centroid: Vec<Point3>, pub face_apex: Vec<Point3>,
    pub cell_volume: Vec<f64>, pub cell_centroid: Vec<Point3> }

// geometry.rs — free functions, zero allocation
pub fn face_area_vector(m: &dyn MeshRead, f: FaceIx) -> Vector3;
pub fn face_centroid(m: &dyn MeshRead, f: FaceIx) -> Point3;
pub fn cell_volume_centroid(m: &dyn MeshRead, c: CellIx) -> (f64, Point3);
pub fn compute_geometry_cache(m: &dyn MeshRead) -> GeometryCache;

// build.rs
pub struct SerialAlloc { /* BTreeMap<EntityKind,u64> */ }
impl SerialAlloc { pub fn take(&mut self, k: EntityKind, n: usize) -> Result<u64>;
                   pub fn one(&mut self, k: EntityKind) -> Result<EntityId>; }
pub struct CellAdded { pub cell: CellIx, pub faces: SmallVec<[SignedFace; 6]> }
pub struct Finished<M> { pub mesh: M, pub face_remap: Vec<FaceIx> }
pub trait MeshBuild {
    type Output: MeshRead;
    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx>;
    fn add_extra_node(&mut self, p: Point3) -> Result<PointIx>;
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx>;
    fn add_face(&mut self, lp: &[VertexIx]) -> Result<FaceIx>;
    fn find_face(&self, lp: &[VertexIx]) -> Option<(FaceIx, LoopRelation)>;
    fn add_cell(&mut self, faces: &[SignedFace], corners: Option<&[VertexIx]>) -> Result<CellIx>; // two-phase
    fn cell_faces(&self, c: CellIx) -> &[SignedFace];
    fn set_class(&mut self, e: MeshEntityRef, c: Classification) -> Result<()>; // edges by vertex pair
    fn set_high_order(&mut self, c: CellIx, h: HighOrderCell) -> Result<()>;     // count-checked
    fn set_face_curve(&mut self, f: FaceIx, degree: u8, basis: Basis, nodes: &[PointIx]) -> Result<()>;
    fn add_set(&mut self, name: &str, members: SetMembers) -> Result<()>;
    fn add_tag(&mut self, name: &str, k: MeshEntityKind, values: TagValues) -> Result<()>;
    fn finish(self, alloc: &mut SerialAlloc) -> Result<Finished<Self::Output>>;
    // provided:
    fn add_polyhedron(&mut self, outward: &[&[VertexIx]]) -> Result<CellAdded>;
    fn add_cell_from_vertices(&mut self, s: CellShape, v: &[VertexIx]) -> Result<CellAdded>;
}
pub enum MeshEntityRef { Vertex(VertexIx), Edge(VertexIx, VertexIx), Face(FaceIx), Cell(CellIx) }
pub trait MeshEdit: MeshRead {           // topology-preserving
    fn set_point(&mut self, i: PointIx, p: Point3) -> Result<()>;
    fn add_extra_node(&mut self, p: Point3) -> Result<PointIx>;
    fn set_class(&mut self, k: MeshEntityKind, ix: u32, c: Classification) -> Result<()>;
    fn set_high_order(&mut self, c: CellIx, h: Option<HighOrderCell>) -> Result<()>;
    fn set_face_curve(&mut self, f: FaceIx, c: Option<(u8, Basis, Vec<PointIx>)>) -> Result<()>;
    fn add_set(&mut self, name: &str, m: SetMembers, alloc: &mut SerialAlloc) -> Result<EntityId>;
    fn set_tag(&mut self, name: &str, k: MeshEntityKind, v: TagValues) -> Result<()>;
    fn ensure_edges(&mut self, alloc: &mut SerialAlloc) -> Result<()>;
    fn set_geometry_cache(&mut self, g: Option<GeometryCache>) -> Result<()>;
}

// hierarchy.rs
pub struct LevelMap { /* fine, coarse: EntityId; fine/coarse topology_hash; coarse_of: Vec<CellIx>;
                         inverse CSR; face_map: Option<Vec<u32>> */ }
impl LevelMap { pub fn new(fine: &dyn MeshRead, coarse: &dyn MeshRead, coarse_of: Vec<CellIx>,
                           face_map: Option<Vec<u32>>) -> Result<Self>;
    pub fn coarse_of(&self, f: CellIx) -> CellIx; pub fn fines_of(&self, c: CellIx) -> &[CellIx];
    pub fn restrict_sum(&self, v: &[f64]) -> Result<Vec<f64>>; pub fn prolong_copy(&self, v: &[f64]) -> Result<Vec<f64>>;
    pub fn compose(&self, next: &LevelMap) -> Result<LevelMap>; }
pub trait MeshCatalog {
    fn mesh_ids(&self) -> &[EntityId];                       // ascending
    fn mesh(&self, id: EntityId) -> Option<&dyn MeshRead>;
    fn maps_from(&self, fine: EntityId) -> Vec<(EntityId, &LevelMap)>;
    fn maps_to(&self, coarse: EntityId) -> Vec<(EntityId, &LevelMap)>;
    fn views(&self) -> &[InstancedView];                     // ascending id
}
pub trait CatalogEdit: MeshCatalog { fn insert_mesh(&mut self, m: Box<dyn MeshRead>) -> Result<()>;
    fn link(&mut self, map: LevelMap) -> Result<()>; fn insert_view(&mut self, v: InstancedView) -> Result<()>; }

// instance.rs
pub struct Instance { pub id: EntityId, pub prototype: EntityId, pub transform: Transform }
pub struct InstancedView { pub id: EntityId, pub instances: Vec<Instance> }
pub struct FlattenOptions<'a> { pub weld_tol: Option<f64>,
    pub remap_class: Option<&'a dyn Fn(EntityId /*instance*/, Classification) -> Classification>,
    pub on_merge: Option<&'a dyn Fn(Classification, Classification, Classification) -> Result<Classification>> }
pub struct FlattenMap { pub cells: Vec<(EntityId, Range<u32>)>, pub vertex_origin: Vec<(EntityId, VertexIx)>,
    pub face_origin: Vec<(EntityId, FaceIx)>, pub merged_faces: Vec<FaceIx>, pub loss: Vec<LossNote> }
impl FlattenMap { pub fn locate(&self, c: CellIx) -> Option<(EntityId, CellIx)>; }
pub fn append_transformed<B: MeshBuild>(b: &mut B, src: &dyn MeshRead, t: &Transform, o: &FlattenOptions) -> Result<AppendMap>;
pub fn flatten<B: MeshBuild>(cat: &dyn MeshCatalog, view: EntityId, b: B, o: &FlattenOptions,
                             alloc: &mut SerialAlloc) -> Result<(B::Output, FlattenMap)>;
pub fn unfold<B: MeshBuild>(src: &dyn MeshRead, deck: &[Transform], b: B, o: &FlattenOptions,
                            alloc: &mut SerialAlloc) -> Result<(B::Output, FlattenMap)>;

// validate.rs
pub fn validate(m: &dyn MeshRead) -> Vec<Violation>;           // topology, orientation, closedness, canonical order, classification incl. edges/Param, HO counts
pub fn validate_geometry(m: &dyn MeshRead, tol: &GeomTol) -> Vec<Violation>; // positive volume / side-tets, planarity
pub fn topology_hash(m: &dyn MeshRead) -> u64; pub fn geometry_hash(m: &dyn MeshRead) -> u64;
pub mod reference { pub struct ReferenceBuilder; pub struct ReferenceMesh; pub struct ReferenceCatalog;
                    pub fn hex_block(n: [u32; 3], h: [f64; 3], alloc: &mut SerialAlloc) -> Result<ReferenceMesh>; }
pub mod select;   // Selector algebra over &dyn MeshRead (free functions)
```

**Required tests:**
- the five requested tests, kept;
- plus tests from tables: template volumes, each also mirrored;
- a quarter unfold (18 vertices / 4 interior faces) and two translated copies welded (1 interior face);
- a check that welded seam faces are reclassified;
- a zero-allocation hot loop measured with a counting allocator;
- a high-order node-count rejection;
- `face_remap` correctness;
- a hash that changes when only Param, tag values or set members change;
- the determinism test from `CLAUDE.md`.

## 6. Open questions for the orchestrator

1. **Partitions and ghosts.** The global rule ("left = lower id, always present") cannot describe a ghost cell's outer face once its left cell is off-part. My recommendation: partition-local views are **not** `MeshRead`. They get a separate trait in a later partition crate, with `CellRef::{Local, Ghost, Remote(EntityId)}`. v1 only reserves `FaceKind::PartitionCut`, and `validate` accepts it. Please confirm.
2. **Non-conformal interfaces.** Should a minimal L7 record type (`InterfaceRecord { cell_in, face_in, cell_out, face_out, orientation, area_vector }`, DUNE style, addendum §7.6) go into sm-mesh-api now, or stay deferred to a later crate, with only `FaceKind::NonConformal` reserved? I recommend deferring and reserving only the face kind.
3. **High-order node order** (`smartmesh.nodes.v1`). Pick one convention: gmsh MSH4, VTK Lagrange, or Basix. I recommend gmsh ordering for degree ≤ 2 template shapes, with everything else `Unsupported` in v1. Mirroring high-order cells also stays `Unsupported` in v1.
4. **Inactive parents and lineage** (addendum §7.2). v1 has only the `cell_active` default slot. Should real in-mesh parent/active columns and topology-changing edits be added before freeze, or come with the refinement crate?
5. **Preserving ids.** Should the builder accept explicit ids (re-import, restart, part rebuild) through `finish_with_ids`? The ids would still have to ascend with dense index.
6. **Bulk columns.** `columns()` is required in my proposal, which rules out lazy or virtual implementors. Should it instead be optional (`Option<TopologyColumns>`)?
7. **serde.** The sm-core types (`EntityId`, `Point3`, `Transform`) do not derive `Serialize`. Should serde be added to sm-core (a frozen contract), or should the mesh API persist raw `u64`/`f64` only?
8. **Cargo.lock.** tables committed a lock update; hasse and facefirst reverted theirs. The merge must add the thiserror 2, smallvec 1 and indexmap 2 entries to `Cargo.lock`.
9. **Canonical face order.** Keep `(left, right, loop)` and let the OpenFOAM exporter permute the faces? The alternative, interior faces first and then boundary faces grouped by classification, would make that export trivial. But reclassifying a face would then break the order. I recommend keeping `(left, right, loop)`.