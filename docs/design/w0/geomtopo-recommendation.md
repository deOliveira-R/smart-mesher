# sm-geom-api + sm-topo-api: final synthesis (geomtopo panel)

## 1. Score table

Two judges scored each candidate on six criteria. Each cell shows judge 0 / judge 1. Σ is the sum over both judges (maximum 120).

| Candidate | Expressiveness | Ergonomics | Performance | Determinism | Extensibility | Correctness | Σ | Mean |
|---|---|---|---|---|---|---|---|---|
| occt-like (`e24b66f`) | 6/7 | 7/6 | **4/4** | 9/9 | 7/7 | 8/7 | **81** | 6.75 |
| radial-edge (`ddd7098`) | 6/7 | 6/6 | 6/5 | 9/8 | 7/6 | 7/7 | 80 | 6.67 |
| cellcomplex (`2d306cf`) | 6/6 | 6/7 | **6/7** | 9/9 | 7/6 | 5/6 | 80 | 6.67 |

The scores are a tie:
- All three candidates compile.
- All three have the same webcut gap and the same overlay gap.
- All three score 8 or 9 on determinism.

So the winner is decided by the binding constraints in section 2, not by points.

## 2. Winner: cellcomplex as the base, with large grafts

Three constraints decided it, in this order.

1. **The ir.md §6 orientation rule.** The rule: `left` is the lower cell id, the normal points from left to right, and a boundary face has `right = none` with an outward normal.
   - cellcomplex's `face_sides → {left, right: Region}` *is* this rule. Its `merge_faces` enforces lower-to-higher orientation and logs the flip as `Modified`.
   - occt derives the sides from FaceUse senses (`front/back`).
   - radial-edge uses `Negative/Positive` sides plus a redundant `UseSense`.
   - Both of those can be mapped onto the rule, but neither enforces it.
   - The semantics panel, the tracker's "mesh walking" (§5.2) and the OpenFOAM/CGNS exporters all depend on this rule.
2. **The CLAUDE.md performance rule (CSR layouts, no per-element allocation).**
   - cellcomplex is the only candidate whose core relation returns borrowed slices, in canonical order: `cells(dim) -> &[EntityId]` and `boundary/coboundary -> &[Incidence]`.
   - occt returns an owned `Vec` on every adjacency step and removes duplicates with `Vec::contains`. A judge measured it as quadratic: 26.8 s to build 2000 bricks.
   - radial-edge is in between.
3. **The ARCHITECTURE §9 Q2 proposal** ("global default, per-entity overrides on imported geometry"). cellcomplex already implements exactly this. The other two candidates take the two extremes (per-entity everywhere, or global only).

cellcomplex also has a structural advantage. It uses one oriented incidence relation with a single tag, `Side{Left, Right, Both}`. That one tag covers:
- coedge sense
- FaceUse sense after a merge
- periodic seams (the same edge listed once as Left and once as Right)
- closed edges
- baffles

Loops, shells and FaceUses are derived views, implemented once for any `dyn TopologyView`. Overlays and other implementors get them for free, and sm-core needs no new entity kinds.

cellcomplex's low correctness score comes from three things:
- Mostly the overlay (defect D4). Every candidate has to redo the overlay anyway.
- A wrong doc comment on `make_face`.
- The orientation rule being enforced only in `merge_faces`.

The last two are fixed below.

**Geometry base.** Keep cellcomplex's trait shape:
- jets up to second derivatives
- `Domain1/Domain2` with periods and `Singularity` poles
- `project(p, hint)`
- `implicit()` on every analytic surface
- `ray_hits` appending into a caller-owned buffer

The serialization model (closed serde enums) and the `RayHit` crossing type come from the other two candidates (section 3).

**Judge finding rejected.** One judge flagged reusing `EntityKind::PartitionCell` for overlay cells as misuse. It is not misuse:
- ir.md §5 defines the Decomposition layer as "virtual, reversible cuts/composites … into meshable pieces".
- concepts.yaml says `P.decomposition: virtual cuts/composites`.
- sm-core documents `PartitionCell` as a virtual decomposition piece.

The overlay *is* that layer. The only real cost is that one kind covers all ranks, so `dim_of` is needed (open question Q5).

## 3. Ideas to graft

### From radial-edge

- **Radial ring:** `radial_uses(edge) -> &[EdgeUse]`, the uses around a non-manifold edge in canonical angular order.
  - It is a derived index that depends on geometry, rebuilt once per operation rather than on every edit.
  - Ties break by id, and the angle key is rounded to a fixed step that the contract states.
  - It also gives the "neighbouring face across this edge inside volume V" helper: `face_across_edge(face, edge, volume)`.
- **`CoedgeKind::Pole`** for sphere and cone apexes. These are degenerate coedges with no edge, left out of Euler counts and rings.
- **Retired records keep a state:** `EntityState::{Live, Split(op), Merged(op), Deleted(op)}`, so old ids always resolve.
- **`LineageEvent::Merged { gap }`** as an audit trail of the gap discarded by a merge.
- **`ShellKind::{Outer, Void, Internal}`.** Baffles and wires go in an `Internal` shell, never in the closed `Outer` shell, and validate enforces this.
- **`RayHit { t, point, normal, uv: Option<Uv>, crossing: Crossing::{AlongNormal, AgainstNormal, Tangent} }`.** A double root is reported once, as `Tangent`.
- **`TopologyView: Send + Sync`** as a supertrait.
- **Pole limit normals** in the style of `SurfaceDomain`.

### From occt-like

- **Closed serde enums `CurveDef` / `SurfaceDef`** behind the object-safe traits, plus a `GeomStore` that hands out geometry by handle, so many faces can share one surface.
  - `SurfaceDef` gets an **`Implicit(ImplicitDef)`** variant, so a general quadric (GQ) and the torus quartic can be stored by handle.
  - This fixes the cellcomplex defect where a `HalfSpace` could not reference a general quadric.
- **A parameter-space curve (pcurve) on each coedge:** `PCurve` with variants `Line2`, `Polyline2` and `Nurbs2`, each with `eval`, `d1` and `range`. It is stored on the face-loop incidence and **hashed by its contents**.
- **Location / Instance / ShapeRef** with parent chaining, plus `Location::orient()` for improper (mirroring) transforms. This serves the §5.1 mirror unfold and lattices.
- **Merge resolution rule:** the lowest id survives, and candidates are processed in ascending id order (see the tolerance clause in 5.6).
- **`topology_hash` rounds coordinates and tolerances** before hashing, and covers lineage and coincidence records.
- **Tolerances only grow,** only inside a logged operation (`Modified`), and never above `linear_max`.
- **The `WithOverlay` delegation pattern,** used in a test to prove the host hash is unchanged.
- **The small reference NURBS evaluator (de Boor),** pending open question Q4.

### From cellcomplex (kept)

- Paged CSR rows in canonical order: each cyclic loop starts at its minimum `(edge, side)`.
- Atomic edits, tested by checking the hash is unchanged after a failed edit.
- A documented order in which each operator creates ids.
- `OpId` supplied by the caller.
- `validate()` and `validate_geometry()`.
- `ParametricSurface::implicit()`, so one object serves the boundary representation, the CSG form and the default ray hook.

### Removed

- radial-edge's exact predicates `orient2d/orient3d` move to **sm-math**, which is the L0 crate in ARCHITECTURE. They are not sm-geom-api's job.
- radial-edge's `FaceUseRec.sense` field is dropped, because it can be derived from the side.

## 4. Must-fix defects

These bind the implementing agent. Most of them affect all three candidates.

| # | Defect | Fix in the final contract |
|---|---|---|
| D1 | No `split_volume`, so webcut volume lineage (`1 → 1a, 1b`) cannot be expressed. All six judges hit this in scenario 4. | Add `split_volume(vol, cut_faces, assign) -> [EntityId; 2]`: it retires the parent and creates two children with `Split` lineage. Add a generic `record_split(parent, children)` for composite operations. It is allowed only inside an operation, and the parent must be retired in the same operation. |
| D2 | No operator to add an inner loop, so a smaller face cannot be imprinted onto a larger one. A periodic face cannot be cut by a closed edge (radial judge0_6). `split_face` handles only single-loop faces. | Add `make_inner_loop(face, loop) -> loop index` (the MEKR Euler operator) and `kill_inner_loop` (KEMR). Change to `split_face(face, cut: FaceCut)`, where `FaceCut = Chord{edge} \| ClosedLoop{edges}`. `ClosedLoop` covers the annulus split of a periodic face. Multi-loop faces are supported: inner loops go to the children by an assignment the caller supplies, or by a point-in-face test inside the operation. |
| D3 | The reference store rebuilds its derived indices in O(N) after every edit, so building a model is quadratic. | Maintain the coboundary rows **incrementally** on each primitive edit. Rebuild only the geometric index (radial ring order), only for changed edges, at `end_op`. Contract target: building N bricks is O(N log N), and a 10,000-brick test finishes in under 2 s in release mode. |
| D4 | In all three candidates the overlay is only a tag table:<br>• virtual faces have no loops, sides or geometry carrier<br>• there is no `remove`<br>• ids come from `len()`<br>• composites across volumes are accepted<br>• the interior edges stay visible | The overlay **is** a `TopologyView`:<br>• Composite faces get a derived boundary (interior edges and vertices hidden), sides, and `Carrier::Composite(&[host faces])`.<br>• `composite_faces` rejects members that are not adjacent, are on different sides, are in different volumes, or share an interior edge with a face outside the composite.<br>• `partition_*` takes real cut geometry.<br>• Ids are never reused: the overlay keeps an increasing counter, and `remove(id)` exists.<br>• Each overlay stores `host_revision` and refuses queries once the host has changed.<br>• It holds the host by `Arc` with a revision check instead of a `&'h dyn` borrow, so it can be saved. |
| D5 | Coedge keys are positions in a loop, so they shift when an edge is split. A seam vertex appears twice in a loop, so a `(face, vertex)` key collides. | Use a stable **`CoedgeKey { face, edge, side }`**. An edge appears at most once per side in a face's loops, and a seam appears once as Left and once as Right. Name a vertex use with **`CornerKey { face, incoming: CoedgeKey }`**. When the edge splits, lineage maps `edge → [e1, e2]` and the scheme layer re-keys through the successors. This is documented in the contract. |
| D6 | Tracking gaps:<br>• no point-in-face classification<br>• no helper giving the side a ray enters<br>• no CSG cell type<br>• volumes cannot carry a CSG expression | In sm-geom-api: add `CsgCell`, an intersection of signed `HalfSpace`s with `contains` and `ray_hits` (optionally a union of cells as a region).<br>In sm-topo-api:<br>• `Carrier::Csg(EntityId)` for volumes<br>• `classify_point_on_face(face, uv) -> In/On/Out`, a signature in the geometry query trait (5.3)<br>• the helper `entered_region(face, &RayHit) -> Region`, which combines the hit's crossing, the surface sense and the side |
| D7 | The edit trait cannot insert geometry (radial-edge). There is no snap or move step, which leaves a hole in radial-edge's global-tolerance plan. | Add `TopologyEdit::shapes_mut() -> &mut dyn GeomStoreMut`, plus `set_vertex_point`, `replace_curve` and `replace_surface`, each logged as `Modified`. |
| D8 | No rollback for a whole operation, and `end_op` does not validate. | `abort_op()` restores the state from `begin_op`, using a snapshot or an undo log. `end_op() -> Result<OpSummary>` runs the cheap invariant checks and aborts if any fails. |
| D9 | Deleting a volume does not cascade, so every "keep one chamber" repeats a three-pass manual cleanup. | `kill_cascade(cell)` removes the cell and every lower cell left orphaned, in a fixed top-down order, with a `Deleted` lineage record for each. |
| D10 | `merge_faces` compares edge sets instead of checking that the loops correspond under `relative`. It drops the absorbed face's embedded edges and ignores that face's tolerance (occt, cellcomplex). | Check loop correspondence, carry over the embedded edges, set the survivor's tolerance to the maximum of the two, and record the measured gap. |
| D11 | Coincidence records are lost when a face splits (cellcomplex `retire()`). Status changes are not tied to an operation (occt). | Records move to the split children. Every status change writes a `Modified` lineage entry under the current `OpId`. |
| D12 | The §6 orientation rule is enforced only on merge. `face_sides` silently overwrites when two volumes claim the same side. The `make_face` doc contradicts the right-hand rule. | `validate` checks that every boundary face has its volume on the left (normal outward), and treats two claims on one side as an error. Correct the doc: the loops run counter-clockwise seen from the **right** side, so the normal points from left to right. Every edit that leaves a side empty (`remove_face_use`, `kill`) re-applies the rule and logs a `reverse_face` as `Modified`. |
| D13 | Lineage `parents` and `descendants` are linear or quadratic scans. `coincidence_between` is a linear scan. | Index lineage in both directions (`BTreeMap<EntityId, SmallVec<[usize; 2]>>`). Keep coincidence pairs sorted by `(min, max)` and binary-search them. |
| D14 | The hash uses exact float bits, so the doc's "equal on every platform" claim is false. Pcurves are hashed only as `is_some()`. | `canonical_hash` rounds coordinates and tolerances to a fixed step (the step is open question Q2) and hashes pcurve contents. |
| D15 | serde derives are incomplete (radial-edge). Error styles are inconsistent (`Option` in some constructors, `Result` in others). | Every public data type derives serde behind the feature flag. Every constructor returns `Result<_, GeomError>`. |
| D16 | `ray_hits` misses grazing (tangent) roots on the torus in every candidate. | The quartic solver also brackets the roots of its derivative and reports a double root as `Tangent`. This needs a test against an analytic result. |
| D17 | Cut faces record no provenance (scenario 5). | Add a lineage kind `Generated { from_surface: EntityId }` for faces created on a tool surface, so "the new cut face" can be found by a query. |

## 5. Proposed final public API

### 5.1 sm-geom-api

```rust
pub struct Interval { pub lo: f64, pub hi: f64 }
pub struct Domain1 { pub range: Interval, pub period: Option<f64> }
pub enum Singularity { None, Pole { at: f64 } }              // a collapsed domain edge
pub struct Domain2 { pub u: Domain1, pub v: Domain1,
                     pub u_lo: Singularity, pub u_hi: Singularity,
                     pub v_lo: Singularity, pub v_hi: Singularity }
pub struct CurveJet { pub p: Point3, pub d1: Vector3, pub d2: Vector3 }
pub struct SurfaceJet { pub p: Point3, pub du: Vector3, pub dv: Vector3,
                        pub duu: Vector3, pub duv: Vector3, pub dvv: Vector3 }

pub trait ParametricCurve: Send + Sync + Debug {
    fn domain(&self) -> Domain1;
    fn jet(&self, t: f64) -> CurveJet;
    fn eval(&self, t: f64) -> Point3 { self.jet(t).p }
    fn project(&self, p: Point3, hint: Option<f64>) -> Result<CurveProjection, GeomError>;
    fn length(&self, range: Interval, tol: f64) -> Result<f64, GeomError>;
}
pub trait ParametricSurface: Send + Sync + Debug {
    fn domain(&self) -> Domain2;
    fn jet(&self, uv: Uv) -> SurfaceJet;
    fn eval(&self, uv: Uv) -> Point3 { self.jet(uv).p }
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError>; // Su x Sv; limit at poles
    fn project(&self, p: Point3, hint: Option<Uv>) -> Result<SurfaceProjection, GeomError>;
    fn implicit(&self) -> Option<ImplicitDef> { None }
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError>;
}
pub trait ImplicitSurface: Send + Sync + Debug {           // f < 0 inside, gradient = outward normal
    fn value(&self, p: Point3) -> f64;
    fn gradient(&self, p: Point3) -> Vector3;
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError>;
}
pub enum Crossing { AlongNormal, AgainstNormal, Tangent }
pub struct RayHit { pub t: f64, pub point: Point3, pub normal: Vector3,
                    pub uv: Option<Uv>, pub crossing: Crossing }   // ascending t

// Closed, serializable data
pub enum CurveDef { Line(Line), Circle(Circle), Ellipse(Ellipse), Nurbs(NurbsCurve) }
pub enum SurfaceDef { Plane(Plane), Cylinder(Cylinder), Cone(Cone), Sphere(Sphere),
                      Torus(Torus), Nurbs(NurbsSurface), Implicit(ImplicitDef) }
pub enum ImplicitDef { Quadric(Quadric), Torus(ImplicitTorus) } // GQ incl. improper transforms
pub enum PCurve { Line2(Line2), Polyline2(Vec<Uv>), Nurbs2(NurbsCurve2) } // eval, d1, range
pub struct HalfSpace { pub surface: SurfaceId, pub sign: Sign }
pub struct CsgCell { pub halfspaces: SmallVec<[HalfSpace; 4]> }   // an intersection
impl CsgCell { fn contains(&self, s: &dyn GeomStore, p: Point3, tol: f64) -> Result<Containment, GeomError>;
               fn ray_hits(&self, s: &dyn GeomStore, ray: &Ray, out: &mut Vec<RayHit>) -> Result<(), GeomError>; }

pub trait GeomStore: Send + Sync {
    fn curve(&self, id: EntityId) -> Option<&dyn ParametricCurve>;
    fn surface(&self, id: EntityId) -> Option<&dyn ParametricSurface>;
    fn implicit(&self, id: EntityId) -> Option<&dyn ImplicitSurface>; // every surface that has one
    fn curve_def(&self, id: EntityId) -> Option<&CurveDef>;
    fn surface_def(&self, id: EntityId) -> Option<&SurfaceDef>;
}
pub trait GeomStoreMut: GeomStore {
    fn add_curve(&mut self, def: CurveDef) -> Result<EntityId, GeomError>;   // deterministic serial
    fn add_surface(&mut self, def: SurfaceDef) -> Result<EntityId, GeomError>;
}
pub struct ModelTolerance { pub linear: f64, pub angular: f64, pub parametric: f64,
                            pub linear_max: f64 }
impl ModelTolerance { fn for_extent(extent: f64) -> Self;
                      fn coincident(&self, d: f64, ta: f64, tb: f64) -> bool; } // d <= ta+tb && ta+tb <= linear_max
```

### 5.2 sm-topo-api: read side

```rust
pub enum Dim { Vertex, Edge, Face, Volume, Body }
pub enum Side { Left, Right, Both }          // Both = closed edge / baffle
pub enum Sense { Forward, Reversed }
pub enum Region { Volume(EntityId), Exterior }
pub struct Incidence { pub cell: EntityId, pub side: Side, pub ring: u16 }  // ring = loop/shell index
pub enum Carrier { None, Point(Point3), Curve(CurveCarrier), Surface(SurfaceCarrier),
                   Csg(EntityId), Composite(SmallVec<[EntityId; 4]>) }
pub enum ShellKind { Outer, Void, Internal }
pub enum EntityState { Live, Split(OpId), Merged(OpId), Deleted(OpId) }
pub enum Coincidence { Merged, ImprintedNotMerged, CoincidentUnimprinted, Overlapping }
pub struct FacePair { pub a: EntityId, pub b: EntityId, pub status: Coincidence, pub op: OpId }
pub enum LineageKind { Created, Split { children: SmallVec<[EntityId; 2]> },
                       Merged { into: EntityId, gap: f64 }, Deleted, Modified(ModKind),
                       Generated { from_surface: EntityId } }
pub struct CoedgeKey { pub face: EntityId, pub edge: EntityId, pub side: Side }
pub struct CornerKey { pub face: EntityId, pub incoming: CoedgeKey }

pub trait TopologyView: Send + Sync {
    fn tolerance(&self) -> &ModelTolerance;
    fn shapes(&self) -> &dyn GeomStore;
    fn dim_of(&self, id: EntityId) -> Option<Dim>;
    fn state(&self, id: EntityId) -> Option<EntityState>;
    fn cells(&self, dim: Dim) -> &[EntityId];                  // live, ASCENDING (contract)
    fn boundary(&self, id: EntityId) -> &[Incidence];          // canonical CSR row
    fn coboundary(&self, id: EntityId) -> &[Incidence];        // ascending by cell
    fn pcurve(&self, k: CoedgeKey) -> Option<&PCurve>;
    fn shell_kind(&self, volume: EntityId, ring: u16) -> Option<ShellKind>;
    fn radial_uses(&self, edge: EntityId) -> &[EdgeUse];       // angular order, derived
    fn carrier(&self, id: EntityId) -> Carrier;
    fn tolerance_override(&self, id: EntityId) -> Option<f64>;
    fn tolerance_of(&self, id: EntityId) -> f64 { /* override or global linear */ }
    fn location(&self, inst: EntityId) -> Option<Location>;    // Instance kind
    fn coincidences(&self) -> &[FacePair];                     // sorted (min, max)
    fn lineage(&self) -> &LineageLog;                          // indexed both ways
    fn revision(&self) -> OpId;
    fn canonical_hash(&self) -> u64;                           // quantized
}

// Blanket-implemented over every TopologyView (including dyn and overlays)
pub struct FaceSides { pub left: Region, pub right: Region }   // normal points left -> right
pub struct FaceUse { pub volume: EntityId, pub shell: u16, pub sense: Sense }
pub trait TopologyViewExt: TopologyView {
    fn volumes(&self) -> &[EntityId];                    // = cells(Dim::Volume)
    fn faces(&self) -> &[EntityId];
    fn face_sides(&self, f: EntityId) -> Option<FaceSides>;
    fn face_uses_of(&self, f: EntityId) -> SmallVec<[FaceUse; 2]>;   // ascending by volume
    fn face_use_in(&self, f: EntityId, v: EntityId) -> SmallVec<[Sense; 2]>; // 2 entries = baffle
    fn face_class(&self, f: EntityId) -> Option<FaceClass>;  // Free|Boundary|Interface|Baffle
    fn volume_faces(&self, v: EntityId) -> SmallVec<[EntityId; 8]>; // ascending, deduplicated
    fn boundary_faces(&self) -> impl Iterator<Item = EntityId> + '_;
    fn loops(&self, f: EntityId) -> impl Iterator<Item = Loop<'_>>;       // rows, no allocation
    fn coedges(&self, f: EntityId) -> impl Iterator<Item = Coedge> + '_;  // with CoedgeKey
    fn loop_vertices(&self, f: EntityId, ring: u16) -> impl Iterator<Item = EntityId> + '_;
    fn face_outline(&self, f: EntityId) -> Vec<Point3>;   // outer-loop corner points in loop order
    fn face_across_edge(&self, f: EntityId, e: EntityId, v: EntityId) -> Option<EntityId>;
    fn entered_region(&self, f: EntityId, hit: &RayHit) -> Option<Region>;
    fn coincidence(&self, a: EntityId, b: EntityId) -> Option<&FacePair>; // binary search
}
```

### 5.3 Geometric model queries (a separate trait, with signatures in sm-topo-api)

```rust
pub struct Moments { pub measure: f64, pub first: Vector3, pub second: [[f64; 3]; 3] } // ∫1, ∫x, ∫x xᵀ about ORIGIN
impl Moments { fn centroid(&self) -> Point3; fn central_second(&self) -> [[f64; 3]; 3]; } // parallel axis
pub struct FaceMoments { pub area: f64, pub centroid: Point3,
                         pub vector_area: Vector3,      // ∫n dA, oriented left -> right
                         pub unit_normal: Option<Vector3> } // Some only when planar
pub trait ModelGeometryQuery {
    fn volume_moments(&self, v: EntityId) -> Result<Moments, QueryError>;
    fn face_moments(&self, f: EntityId) -> Result<FaceMoments, QueryError>;
    fn classify_point_on_face(&self, f: EntityId, uv: Uv) -> Result<PointClass, QueryError>;
}
```

The reference `CellComplex` implements this trait exactly for planar faces (by the divergence theorem) and for the cylinder fixture. Every other case returns `Unsupported`. The production implementation belongs in sm-geom/sm-model.

### 5.4 sm-topo-api: edit side

```rust
pub trait TopologyEdit: TopologyView {
    fn begin_op(&mut self, op: OpId) -> Result<(), TopoError>;
    fn end_op(&mut self) -> Result<OpSummary, TopoError>;     // validates; aborts on failure
    fn abort_op(&mut self);
    fn shapes_mut(&mut self) -> &mut dyn GeomStoreMut;
    fn make_vertex(&mut self, p: Point3) -> Result<EntityId, TopoError>;
    fn make_edge(&mut self, a: EntityId, b: EntityId, c: Option<CurveCarrier>) -> Result<EntityId, TopoError>;
    fn make_face(&mut self, loops: &[&[CoedgeSpec]], s: Option<SurfaceCarrier>) -> Result<EntityId, TopoError>;
    fn make_volume(&mut self, shells: &[ShellSpec]) -> Result<EntityId, TopoError>; // ShellSpec carries ShellKind
    fn make_body(&mut self, members: &[EntityId]) -> Result<EntityId, TopoError>;
    fn make_instance(&mut self, proto: EntityId, loc: Location) -> Result<EntityId, TopoError>;
    fn add_face_use(&mut self, v: EntityId, shell: u16, u: FaceUseSpec) -> Result<(), TopoError>;
    fn remove_face_use(&mut self, v: EntityId, f: EntityId, side: Side) -> Result<(), TopoError>;
    fn embed(&mut self, parent: EntityId, child: EntityId) -> Result<(), TopoError>;
    fn split_edge(&mut self, e: EntityId, at: EntityId, t: f64) -> Result<[EntityId; 2], TopoError>;
    fn split_face(&mut self, f: EntityId, cut: FaceCut, inner: InnerLoopAssign) -> Result<[EntityId; 2], TopoError>;
    fn split_volume(&mut self, v: EntityId, cut: &[EntityId], assign: ShellAssign) -> Result<[EntityId; 2], TopoError>;
    fn make_inner_loop(&mut self, f: EntityId, lp: &[CoedgeSpec]) -> Result<u16, TopoError>;
    fn kill_inner_loop(&mut self, f: EntityId, ring: u16) -> Result<(), TopoError>;
    fn merge_vertices(&mut self, keep: EntityId, drop: EntityId, max_gap: Option<f64>) -> Result<(), TopoError>;
    fn merge_edges(&mut self, keep: EntityId, drop: EntityId, rel: Sense) -> Result<(), TopoError>;
    fn merge_faces(&mut self, keep: EntityId, drop: EntityId, rel: Sense) -> Result<(), TopoError>;
    fn reverse_face(&mut self, f: EntityId) -> Result<(), TopoError>;
    fn kill(&mut self, cell: EntityId) -> Result<(), TopoError>;
    fn kill_cascade(&mut self, cell: EntityId) -> Result<SmallVec<[EntityId; 16]>, TopoError>;
    fn set_vertex_point(&mut self, v: EntityId, p: Point3) -> Result<(), TopoError>;
    fn set_carrier(&mut self, cell: EntityId, c: Carrier) -> Result<(), TopoError>;
    fn set_pcurve(&mut self, k: CoedgeKey, pc: Option<PCurve>) -> Result<(), TopoError>;
    fn raise_tolerance(&mut self, cell: EntityId, tol: f64) -> Result<(), TopoError>; // grow-only
    fn record_coincidence(&mut self, a: EntityId, b: EntityId, s: Coincidence) -> Result<(), TopoError>;
    fn record_split(&mut self, parent: EntityId, children: &[EntityId]) -> Result<(), TopoError>;
    fn record_generated(&mut self, cell: EntityId, from_surface: EntityId) -> Result<(), TopoError>;
}

pub trait VirtualOverlay: TopologyView {                 // decomposition layer; PartitionCell ids
    fn host_revision(&self) -> OpId;
    fn resolve(&self, virt: EntityId) -> &[EntityId];    // host members
    fn virtual_of(&self, host: EntityId) -> Option<EntityId>;
}
pub struct Overlay { /* Arc<dyn TopologyView> host, monotonic serials */ }
impl Overlay { fn composite_faces(&mut self, f: &[EntityId]) -> Result<EntityId, TopoError>;
               fn partition_face(&mut self, f: EntityId, cut: VirtualCut) -> Result<SmallVec<[EntityId; 2]>, TopoError>;
               fn remove(&mut self, virt: EntityId) -> Result<(), TopoError>; }

pub fn validate(v: &dyn TopologyView) -> Vec<TopoIssue>;
pub fn validate_geometry(v: &dyn TopologyView) -> Vec<TopoIssue>;
pub fn euler_characteristic(v: &dyn TopologyView) -> i64;
```

**Reference fixtures (all required):**
- a brick
- two bricks imprinted and then merged: one shared face with sides `{left: A, right: B}` and opposite FaceUse senses
- a smaller brick imprinted onto a larger one (exercises `make_inner_loop`)
- a face split with lineage
- a webcut brick: `volume → [1a, 1b]` with `Split` lineage
- a cylinder with a periodic seam, with pcurves at u=0 and u=2π
- a sphere with poles
- a baffle in an `Internal` shell, plus a wire
- a mirrored instance using `Location::orient`
- an overlay composite: host hash unchanged, interior edge hidden
- determinism: build twice, then build on four threads, and compare ids, lineage and hash
- a 10,000-brick performance test

**Ids.** The trait boundary uses the raw `sm_core::EntityId`, because the semantics panel uses it and the incidence relation spans all ranks. The extension trait also offers typed wrappers (`#[repr(transparent)] VolumeId(EntityId)` and so on) at no runtime cost, with `From` conversions in both directions.

### 5.5 Cross-check: required topology queries (union of all three panels)

All three requested `face_sides` shapes map onto `FaceSides { left, right }`, which is the ir.md §6 rule.

| Requested | Final | Mapping / note |
|---|---|---|
| `volumes()`, `faces()`, ascending (all three panels) | `cells(Dim::Volume)` and `cells(Dim::Face)` return `&[EntityId]` | Ascending order is a stated contract. Call `.to_vec()` if a `Vec` is needed. |
| tables: `face_sides -> SmallVec<[FaceSide{volume, sense}; 2]>` | `face_uses_of(f) -> SmallVec<[FaceUse; 2]>`, ascending by volume | `Forward` (normal points out of the volume) means the volume is on `left`. |
| | | Boundary face: 1 entry, `Forward`. |
| | | Interface: 2 entries, A `Forward` (lower id) and B `Reversed`. |
| | | Baffle: 2 entries with the same volume, one `Forward` and one `Reversed`. |
| grouptheory: `FaceSides{fwd, rev}` | `face_sides` | `fwd` is `left` and `rev` is `right`. A boundary face has `right = Exterior` (`rev = None`) and an outward normal. ✓ |
| selections: `FaceSides{left, right}` | `face_sides` | Identical. `Region::Exterior` stands for `None`. ✓ |
| `volume_faces`, ascending (grouptheory, selections) | `volume_faces(v)` | Ascending, no duplicates. A baffle appears once; use `face_use_in` to get its sides. |
| `boundary_faces()` | `boundary_faces()` | |
| tables: `face_outline -> Vec<Point3>` | `face_outline(f)` | Outer-loop corner points in loop order. On a seam the repeated vertex appears twice; pole coedges are skipped. |
| `volume_moments` / `volume_mass_properties` | `ModelGeometryQuery::volume_moments` | **Not topology:** exact integration over trimmed surfaces needs a geometry kernel. The second moment is raw, about the origin. selections' *central* moment is `central_second()` = ∫xxᵀ − m·c·cᵀ. |
| `face_centroid`, `face_area`, `face_normal`, `face_properties` | `face_moments` | On a curved face, `face_normal` is defined as the vector area ∫n dA, oriented left to right (outward on boundary faces). `unit_normal` is `Some` only for planar faces. |

Every requested query is supported. The geometric ones go through `ModelGeometryQuery` rather than `TopologyView`, for the reason given in the table.

### 5.6 Tolerance-model decision (contract clause)

The model uses a **global `ModelTolerance` plus sparse per-entity overrides**, stored as a `BTreeMap<EntityId, f64>`. This adopts the ARCHITECTURE §9 Q2 proposal.

1. **Native construction never creates overrides.** Overrides come only from import or heal operations, or from a merge given an explicit `max_gap` (a Cubit-style merge tolerance).
2. **Coincidence test:** two entities coincide when `d <= tol(a) + tol(b)` and `tol(a) + tol(b) <= linear_max`.
3. **Determinism.** Coincidence is not transitive, so the order of resolution is fixed:
   - candidate pairs are processed in ascending `(min id, max id)` order
   - the lowest id always survives
   - merge groups are resolved in that single pass

   Traversal order therefore cannot change the result. This answers radial-edge's objection to per-entity tolerances.
4. **Tolerances only grow,** only inside a logged operation (`Modified`), and never above `linear_max`. On a merge the survivor's tolerance becomes `max(own, d + other)`, and the gap is recorded in `Merged { gap }`.
5. `canonical_hash` rounds coordinates and tolerances before hashing.
6. Orientation and sign decisions use exact predicates (from sm-math), never tolerances.
7. **Rejected alternatives:**
   - Global tolerance only cannot represent gaps in imported STEP files without healing every model. Radial-edge judge 1 found its imported-STEP scenario impossible.
   - A tolerance on every entity puts floats into every predicate and lets tolerances creep upward during booleans.

## 6. Open questions for the orchestrator

- **Q1. Merge tolerance.** Should `merge_*` take an operation-level `max_gap`, or must loose merges always go through a heal/snap operation first? Proposal: yes for `merge_vertices`, with edge and face merges inheriting it.
- **Q2. Hash rounding.** What step should the cross-platform hash round to: one relative to `ModelTolerance::linear`, or a fixed 2⁻⁴⁰ of the model extent? CI depends on this.
- **Q3. Cargo.lock.** The candidates disagree: occt left it uncommitted, radial-edge and cellcomplex committed it. Proposal: the orchestrator owns it and regenerates it at merge.
- **Q4. NURBS evaluator placement.** Keep occt's small reference de Boor evaluator in sm-geom-api, so `GeomStore` can hand out `&dyn` for every `SurfaceDef`? Or defer it to sm-geom, as cellcomplex and radial-edge do? Proposal: keep it in the API crate, because it is small and makes the contract complete.
- **Q5. Overlay ids.** One `PartitionCell` kind covers all ranks, so `dim_of` is needed. Accept that, or add per-rank virtual kinds to sm-core?
- **Q6. Coedge and shell ids.** Should coedges and shells get persistent ids (new sm-core kinds) for P2 naming, or are `CoedgeKey{face, edge, side}` plus lineage enough? Proposal: they are enough for P0–P1.
- **Q7. Home of the geometry queries.** Should the `ModelGeometryQuery` signatures live in sm-topo-api (proposed) or in a separate `sm-model-api`?
- **Q8. Instancing.** Does instancing (`Location`, `make_instance`) stay in sm-topo-api or move to the semantics/instancing layer (ir.md §5 puts instancing under Model)? Proposal: the transform machinery stays in sm-topo-api and lattice semantics go elsewhere.
- **Q9. Transform operations.** `sm-core::Transform` has no compose or inverse, and occt had to reimplement them. Should they be added to sm-core or to sm-math?
- **Q10. Surface coverage for P0.**
  - Is a double-nappe parametric cone needed, or are a single-nappe parametric cone plus a double-nappe implicit cone enough? All candidates do the latter.
  - Is a ring torus only (R > r) acceptable for P0?

A copy of this report is saved at /private/tmp/claude-501/-Users-rodrigo-git-smart-mesh/c40a3162-2e83-405b-a924-4e8e94047b55/scratchpad/geomtopo-synthesis.md