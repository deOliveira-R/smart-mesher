# sm-model-api: final design synthesis (semantics panel)

This synthesis recommends `tables` as the base design, with grafts from the other two. Tables and grouptheory tie on judge points, so the choice is argued from structure in §2.

Candidates: `w0/semantics/tables` (1c6451c), `w0/semantics/grouptheory` (c7a6f25) and `w0/semantics/selections` (b04ba1a). The candidate reports say all three compile, pass their own tests and are clippy-clean. I did not re-run any of them for this synthesis. A copy of this document is at /private/tmp/claude-501/-Users-rodrigo-git-smart-mesh/c40a3162-2e83-405b-a924-4e8e94047b55/scratchpad/sm-model-api-synthesis.md

## 1. Score table

| Approach | Judge | Expr | Ergo | Perf | Determ | Ext | Correct | Sum |
|---|---|---|---|---|---|---|---|---|
| tables | J0 (workflows) | 7 | 6 | 6 | 9 | 6 | 6 | 40 |
| tables | J1 (lattice/decomp/interface/tracking) | 6 | 7 | 6 | 9 | 7 | 6 | 41 |
| **tables total** | | 13 | 13 | 12 | 18 | 13 | 12 | **81** |
| grouptheory | J0 | 7 | 6 | 5 | 9 | 7 | 5 | 39 |
| grouptheory | J1 | 8 | 6 | 6 | 9 | 7 | 6 | 42 |
| **grouptheory total** | | 15 | 12 | 11 | 18 | 14 | 11 | **81** |
| selections | J0 | 7 | 6 | 5 | 9 | 6 | 6 | 39 |
| selections | J1 | 6 | 6 | 4 | 8 | 6 | 6 | 36 |
| **selections total** | | 13 | 12 | 9 | 17 | 12 | 12 | **75** |

## 2. Winner: **tables** as the base, with major grafts from grouptheory

**Why tables over grouptheory.** Grouptheory has a root-cause defect, not a local one. It requires every deck boundary (a face identification) to also be a generator of a finite group that can be unfolded. Three symptoms follow:
- A reflective unit cell with mirrors at x=0 and x=1 fails validation with "deck group does not close".
- A plain periodic pair must carry an unfold window (`copies>=2`).
- Two translation decks with the same vector but different windows silently merge.

Requirement (2) and ir.md §5 make the boundary kind a property of each face, whether or not anyone unfolds. Fixing this in grouptheory means restructuring its model, group and validate core.

Tables' defects are all local: missing checks, one panic, missing helpers, and a rotation limited to 2π/order (one field change fixes that). Tables also already has the pieces the final design keeps:
- deck parameters as the source of truth, with the matrix derived from them;
- one table keyed by face that holds membership, side and mate;
- cuts as data in, data out (`CutOutcome`);
- pieces with no color field;
- serde on most types.

**Why not selections as base.**
1. **Scope violation.** `Selector<K>` is a stored geometric query meant to be re-resolved later. ARCHITECTURE.md §3 says selectors are P2 and must not be built ahead of time.
2. **Id contract violation.**
   - Flattened instance ids are `id_base + emission index`. They collide with authored `Instance` ids, they shift when a slot is voided, and `id_base=0` gives `Instance#0`.
   - Piece ids restart after `clear_decomposition`, and `unsplit` mints a new id.
   - All of these break sm-core's rule that ids are never reused.
3. **Weakest results.** It has the lowest performance scores and the most confirmed bugs. For example, a rotation order-3 spec over an order-6 pairing validates cleanly and then unfolds a hex with gaps.

Its idea of the domain as a view survives only as a special case (see §3).

## 3. Ideas to graft

| From | Idea | Fixes |
|---|---|---|
| grouptheory | Rotation angle as a rational `Turns{num, den}`: exact order, and k·2π/n is expressible | tables' 2π/order-only rotation |
| grouptheory | Deck sign rule `n_B = −g(n_A)` (for a mirror, `g(n) = −n`) with its own `IssueCode::DeckOrientation`, plus a check that the two sides' volumes end up on opposite sides after gluing | tables s3b: a same-side pair passed with zero issues |
| grouptheory | Two-phase cut: `plan_cut` (pure) → kernel returns `CutOutcome` → `Model::apply_cut` | one cut path; this layer stays read-only on topology |
| grouptheory | Unfold as a view of `CopyRef = (base, elem)`, the glue rule A@e ~ B@(e∘g⁻¹), exact `refold()`, and `Moments::transformed` | copies need no minted ids (selections bug); unfold can be tested analytically |
| grouptheory | A fixed internal `GROUP_TOL` for group identity, independent of the user tolerance | stable group closure |
| grouptheory | Documented recolor order: inner first, outer last; per-volume beats per-color | instancing semantics |
| selections | Explicit `DeckSide {A, B, SelfPaired}` per face (A = mesh here, B = copy the mesh under g) | the meshing-intent query becomes one call |
| selections | Staged verification `Rejected{stage, residual}` (per-color invariants → volumes → faces with side colors, boundary color and kind); `declare_detected` refuses unverified candidates unless forced | detection reporting |
| selections | Binary cut tree per volume for decomposition, which tiles the volume by construction | tables' all-or-nothing decomposition |
| selections | Coloring fingerprint as a checkable invariant (kept private) | proof that decomposition does not change coloring |
| selections | `Placement` carries `orientation_reversed` | one rule for the improper-copy flip (§5) |
| selections | Domain-as-view only as a special case: if no volume straddles the cut, the kernel returns a `CutOutcome` with empty lineage and only the cap faces, so there is no second code path | pre-split imported geometry |
| tables | `Candidate::chamber()` turns a detected candidate into a `CutSpec` | detect → cut workflow |
| tables | `FlatModel::fingerprint()` | determinism tests |

## 4. Must-fix defects

### A. Recurring gaps (hit by judges on every candidate)

1. **No user-chosen numeric color label.** "Boundary color 7", a Cubit block or sideset id, or an MCNP material number can only live in the color's name.
   - Fix: `ColorRow{label: u32, name, domain}`, with the label unique per domain.
   - Add lookups `color_by_label` and `color_by_name`.
2. **No O(1) deck lookup for ray tracking.** ir.md §5.2 says the tracker reflects or wraps on a deck hit.
   - Every candidate re-derives the matrix on each call (sin_cos, 3×3 inverse) and returns a nested `Option<Result>`. Selections is O(n) on side B.
   - Fix: a `DeckLookup` built once, as a dense table indexed by the face's position in `faces()`. Entries are `DeckHit{mate, to_mate: Transform, improper}`.
3. **Deck and interface geometry checks fail in one direction or the other.**
   - Tables compares corner outlines, so it rejects a valid pair of curved faces whose seams sit at different angles.
   - Grouptheory and selections compare centroid, area and normal, so they accept faces of different shape with equal values.
   - Fix: integrated face moments (`FaceMoments`, §5). They do not depend on where the seam is, the second moment makes them sensitive to shape, and they transform exactly under improper transforms.
4. **Detection misses symmetry when the inertia tensor is degenerate.** Hex and C4 assemblies, the main use case, are exactly this case.
   - The candidate directions come from too few sources, and nothing marks the result as incomplete.
   - Every passing order is reported separately (C2, C3 and C6 on the same axis).
   - Tolerances do not scale with the model.
   - Fix, candidate directions: add planar-face normals, face-centroid offsets and bisectors of same-color pairs. Inside a degenerate eigenspace, take in-plane directions from pairs of same-color barycentres about the axis.
   - Fix, reporting: only the maximal cyclic order per axis, plus a minimal generator set.
   - Fix, tolerance: relative, scaled by the model's radius of gyration.
   - Fix, completeness: add `complete: bool`, false when an eigenspace was degenerate and no in-plane direction source existed.
5. **Non-conformal interface validation is empty or wrong in all three.** Tables cannot put an unpaired face on side B and allows only 1:1 pairs. Selections' check always fails for two coincident faces that are not shared.
   - Fix: an interface is two M:N face sets with an optional transform.
   - Check that side A and side B face each other: summed vector areas are opposed and bounding boxes overlap within tolerance.
   - Check that every face has a counterpart.
   - Warn when the two sides' volume colors are equal, unless the interface is declared as explicit intent.
   - Reject an interface declared on a merged face (`partner == self`).
6. **No handle for the part of a face that straddles a decomposition cut.** Fix: `PieceFace::Part{face, piece}` is the key the mesh classifier uses, and cut faces are `PieceFace::Cut{cut, side}`.
7. **Sweep source and target are not checked to be opposite.** Fix: reject a source and target that share an edge. This needs the `face_edges` query (§5).
8. **Serde is missing on result types** (detection, unfold, report, decomposition). Fix: every public data type gets `cfg_attr(serde)`.
9. **Cargo.lock was edited outside the crate** (new smallvec and thiserror entries). The orchestrator owns the lockfile.

### B. Tables defects to fix during the port

10. **A panic is reachable from input** (`detect.rs:460 vmap[&fs.volume]`). Fix: typed errors, and no indexing on keys derived from input.
11. **A material interface gets no kind.** An interior face between two colors has no kind. Fix: `face_kind` derives `Interface(Conformal)` for interior faces whose side colors differ. The validator requires an explicit kind only on boundary faces.
12. **`collect_group` drops Color, Body and nested Group members.** Fix: expand them recursively to volumes and faces, with cycle detection.
13. **`cut_and_keep_chamber` ignores `Keep` and does not check caps against the plane.** The caller also has to build the lineage by hand.
    - Fix: `apply_cut` verifies that caps lie on the planned planes, using face moments (`nᵀ S n = d²·A`).
    - It also verifies that the kept volumes lie in the chosen half-space.
    - Lineage comes from the kernel only.
14. **`apply_lineage` errors when a deck mate splits.** Fix: when both mates split, re-pair the children by matching face moments under the deck transform, and keep `face_boundary` reciprocal by construction.
15. **Unfold is a view only, and `fold()` just returns the base.** Fix: `UnfoldedModel` gets real queries.
    - It answers `color_of(CopyRef)`, `kind_of(face CopyRef)` and `sense(CopyRef)`.
    - It exposes `origin_of(CopyRef)`, the pre-cut entity found through the recorded cut lineage.
    - `refold()` returns the exact `Symmetry`.
    - It does not materialize topology; the topology engine does that.
16. **Pub fields let invariants be bypassed.** Fix: private tables with read accessors, and mutation only through methods that keep the invariants.
17. **Moments and outlines are recomputed inside O(n²) loops.** Fix: cache them once per operation in a `GeomCache`, and bin candidates by quantized centroid in a BTreeMap so matching is O(n log n).
18. **Translation unfold silently assumes translations commute with the point group.** Fix: the validator raises `IssueCode::TranslationNotNormalized`. Grouptheory documented this check but never implemented it.
19. **Each `FlatEntry` clones a full `Placed` (240 B), and lookup by path is linear.** Fix: a placement arena (`Vec<Placed>`) with entries holding `placement: u32`, plus a `BTreeMap<InstancePath, Range>` index.
20. **Overrides at an outer level cannot reach an inner slot** (for example, absorbers only in assembly #3). Fix: overrides keyed by an `InstancePath` prefix.
21. **After undo, `Model != before`,** because piece ids come from a never-reused allocator. Fix: undo restores the parent piece with its original id. Parents stay inactive in the cut tree, as ir.md §6 does for refinement. Re-splitting mints new ids, which is correct.
22. **Splitting one volume forces pieces on every volume.** Fix: a volume with no cut tree is implicitly one piece, with the deterministic virtual id `PieceRef::Whole(volume)`.

### C. Grouptheory defects to fix during the graft

23. **Point-group closure dedups with a linear scan** (O(n²) up to 4096 elements; 126 ms for one box), and a generator set that never closes is not rejected up front.
    - Fix: dedup through a BTreeMap keyed by the quantized matrix, plus a check of neighbouring cells.
    - `declare_symmetry` rejects a point group that does not close immediately, with `ModelError::GroupNotFinite`.
24. **`apply_cut` after `adopt_detected` keeps generators that were never cut along.** The result is an overlapping 8-copy cover, and `Detected` provenance is overwritten.
    - Fix: after a cut, the symmetry generators are exactly the decks produced by cuts or declarations.
    - Provenance becomes an append-only `Vec<Provenance>`.
    - An unused generator is an Error.

## 5. Proposed final public API

- **Dependencies:** `sm-core`, `thiserror`, `smallvec`, `indexmap`, and `serde` behind a feature.
- **Determinism:** all maps are `BTreeMap` or `IndexMap`, and all outputs are in ascending id order.
- **Serde:** every public data type derives `cfg_attr(feature="serde", Serialize, Deserialize)`.

### 5.1 TopologyView (input to the sm-topo-api read trait)

```rust
/// ir.md §6 orientation: left = lower volume id; normal (right-hand rule on loop) points left→right;
/// boundary face: right = None, normal outward.
pub struct FaceSides { pub left: EntityId, pub right: Option<EntityId> }

/// Raw moments about the ORIGIN (additive across volumes for per-color sums; transform without a centroid shift).
pub struct Moments { pub volume: f64, pub first: Vector3 /* ∫x dV */, pub second: [[f64;3];3] /* ∫x xᵀ dV */ }

/// Integrated face moments, exact for analytic surfaces; seam-invariant and shape-sensitive.
pub struct FaceMoments {
    pub area: f64,             // ∫ dA
    pub first: Vector3,        // ∫ x dA
    pub second: [[f64;3];3],   // ∫ x xᵀ dA
    pub vector_area: Vector3,  // ∫ n dA, n = loop normal (left→right)
}

pub trait TopologyView {
    // incidence
    fn volumes(&self) -> &[EntityId];                    // active TopoVolume, ascending
    fn faces(&self) -> &[EntityId];                      // active TopoFace, ascending
    fn volume_faces(&self, v: EntityId) -> &[EntityId];  // ascending; index-backed (CSR); unknown → &[]
    fn face_sides(&self, f: EntityId) -> Option<FaceSides>;
    fn face_edges(&self, f: EntityId) -> &[EntityId];    // ascending; only for sweep source/target opposition
    // geometry
    fn volume_moments(&self, v: EntityId) -> Option<Moments>;
    fn face_moments(&self, f: EntityId) -> Option<FaceMoments>;
}
```

Contract: no panic on unknown ids, deterministic results, no mutation.
- **Slices instead of `Vec`:** every judge flagged per-call allocation. This assumes sm-topo keeps sorted CSR id tables (Q3).
- **`face_moments` replaces outline, centroid, area and normal.** Let n̂ = vector_area/area and d = n̂·first/area. The face is planar exactly when `n̂ᵀ·second·n̂ = d²·area`, and then d is the plane offset.
- **Per-volume `Sense` is derived:** Forward exactly when the volume is `left`.
- **Seven queries.** `volume_faces` (required, not a default that scans every face) and `face_edges` are deliberate additions to what the candidates used.

### 5.2 Core types

```rust
pub struct Tolerance { pub linear: f64, pub angular: f64, pub relative: f64 }
pub enum Domain { Volume, Face }
pub struct ColorRow { pub id: EntityId /*Color*/, pub label: u32, pub name: String, pub domain: Domain }
pub struct GroupRow { pub id: EntityId /*Group*/, pub name: String, pub members: BTreeSet<EntityId> } // any kind; expanded recursively

pub struct Plane { pub normal: Vector3 /*unit*/, pub offset: f64 }
pub struct Axis  { pub point: Point3, pub dir: Vector3 /*unit*/ }
pub struct Turns { pub num: i32, pub den: u32 }   // angle = 2π·num/den, reduced
pub enum Deck { Mirror { plane: Plane }, Translation { vector: Vector3 }, Rotation { axis: Axis, angle: Turns } }
impl Deck { pub fn transform(&self) -> Transform; pub fn is_improper(&self) -> bool; }

pub enum InterfaceIntent { Conformal, NonConformal { transform: Option<Transform> } }
pub enum BoundaryKind { Deck(Deck), Response, Interface(InterfaceIntent) }
pub struct BoundaryRow { pub id: EntityId /*Boundary*/, pub name: String, pub kind: BoundaryKind }

pub enum DeckSide { A, B, SelfPaired }
pub enum FaceRole { Deck { side: DeckSide, mate: Option<EntityId> /*None iff SelfPaired*/ }, Response, Interface { side: DeckSide /*A|B*/ } }
pub struct FaceBoundary { pub boundary: EntityId, pub role: FaceRole }  // single source of truth, keyed by face
```

### 5.3 Model: private tables, mutators that keep invariants

```rust
pub struct Model { /* colors, volume_color, face_color, groups, boundaries, face_boundary, symmetry, instancing, decomposition, ids, provenance */ }
impl Model {
    // coloring
    pub fn add_color(&mut self, label: u32, name: &str, d: Domain) -> Result<EntityId, ModelError>;
    pub fn color_volumes(&mut self, c: EntityId, vols: &[EntityId]) -> Result<(), ModelError>; // reassigns; partition kept
    pub fn color_faces(&mut self, c: EntityId, faces: &[EntityId]) -> Result<(), ModelError>;
    pub fn uncolor_faces(&mut self, faces: &[EntityId]);
    pub fn remove_color(&mut self, c: EntityId) -> Result<(), ModelError>;   // Err if still used by volumes
    pub fn color_by_label(&self, d: Domain, label: u32) -> Option<EntityId>;
    pub fn color_by_name(&self, name: &str) -> Option<EntityId>;
    pub fn volume_color(&self, v: EntityId) -> Option<EntityId>;
    pub fn face_color(&self, f: EntityId) -> Option<EntityId>;
    pub fn add_group(&mut self, name: &str, members: &[EntityId]) -> Result<EntityId, ModelError>;
    pub fn group_volumes(&self, g: EntityId, t: &impl TopologyView) -> Result<Vec<EntityId>, ModelError>;
    pub fn group_faces(&self, g: EntityId, t: &impl TopologyView) -> Result<Vec<EntityId>, ModelError>;
    // boundary kind
    pub fn add_boundary(&mut self, name: &str, kind: BoundaryKind) -> Result<EntityId, ModelError>;
    pub fn assign_self_paired(&mut self, b: EntityId, faces: &[EntityId]) -> Result<(), ModelError>;      // mirror
    pub fn assign_pairs(&mut self, b: EntityId, pairs: &[(EntityId, EntityId)]) -> Result<(), ModelError>; // deck A↔B, reciprocal
    pub fn assign_response(&mut self, b: EntityId, faces: &[EntityId]) -> Result<(), ModelError>;
    pub fn assign_interface(&mut self, b: EntityId, a: &[EntityId], bside: &[EntityId]) -> Result<(), ModelError>; // M:N
    pub fn unassign(&mut self, faces: &[EntityId]);      // also clears mate's role for pairs
    pub fn face_kind(&self, f: EntityId, t: &impl TopologyView) -> Option<FaceKindView>; // derives Interface(Conformal) on color seams
    pub fn deck_faces(&self) -> impl Iterator<Item = (EntityId, &FaceBoundary)>;
    pub fn deck_lookup(&self, t: &impl TopologyView) -> DeckLookup;   // build once
}
pub struct DeckHit { pub mate: EntityId, pub to_mate: Transform, pub improper: bool } // SelfPaired: mate = face
pub struct DeckLookup { /* dense Vec<Option<DeckHit>> by face ordinal */ }
impl DeckLookup { pub fn hit(&self, face: EntityId) -> Option<&DeckHit>; }
```

### 5.4 Symmetry: declare, detect, cut, unfold

```rust
pub enum Provenance { Declared, Imported, Detected(Verification), Cut { spec: CutSpec } }
pub struct Symmetry { pub generators: Vec<EntityId> /*deck Boundary ids*/, pub domain: BTreeSet<EntityId> /*volumes*/ }
impl Model {
    pub fn declare_symmetry(&mut self, gens: &[EntityId], t: &impl TopologyView) -> Result<(), ModelError>; // point part must close
    pub fn symmetry(&self) -> Option<&Symmetry>;
    pub fn provenance(&self) -> &[Provenance];   // append-only
}
// Cut-and-keep (two-phase; kernel does geometry)
pub enum Keep { Positive, Negative }
pub struct CutSpec { pub deck: Deck, pub keep: Keep, pub origin: Point3 } // origin = slab base for Translation
pub struct CutPlan { pub half_spaces: SmallVec<[Plane; 2]>, pub spec: CutSpec }
pub fn plan_cut(spec: &CutSpec) -> CutPlan;
pub struct Lineage { pub old: EntityId, pub new: SmallVec<[EntityId; 2]> } // empty = removed (Q4)
pub struct CapFace { pub face: EntityId, pub plane_ix: u8 }
pub struct CutOutcome { pub lineage: Vec<Lineage>, pub caps: Vec<CapFace> }
impl Model {
    /// Transactional re-key; re-pairs split deck mates by face moments; verifies caps on planes and kept side;
    /// tags caps (mirror SelfPaired, slab/wedge A/B); applies cap_color; sets domain; appends Provenance::Cut.
    pub fn apply_cut(&mut self, plan: &CutPlan, out: &CutOutcome, cap_color: Option<EntityId>,
                     t: &impl TopologyView) -> Result<(), ModelError>;
}
// Detection
pub struct ColorInvariant { pub color: EntityId, pub volume: f64, pub barycentre: Point3, pub inertia: [[f64;3];3] }
pub enum CandidateKind { Mirror(Plane), Rotation { axis: Axis, order: u32 /*maximal*/ } }
pub enum Stage { Invariants, Volumes, Faces }
pub enum Verification { Unverified, Verified { max_dev: f64 }, Rejected { stage: Stage, residual: f64, reason: String } }
pub struct Candidate { pub kind: CandidateKind, pub verification: Verification }
impl Candidate { pub fn chamber(&self, keep: Keep) -> CutSpec; }
pub struct DetectionResult { pub invariants: Vec<ColorInvariant>, pub candidates: Vec<Candidate>, pub group_order: u32, pub complete: bool }
pub fn detect_symmetry(m: &Model, t: &impl TopologyView, tol: Tolerance) -> Result<DetectionResult, ModelError>;
pub fn verify_transform(m: &Model, t: &impl TopologyView, x: &Transform, tol: Tolerance) -> Result<Verification, ModelError>;
// Unfold (view; mints no ids)
pub struct UnfoldSpec { pub translation_repeats: BTreeMap<EntityId /*deck*/, u32> }
pub struct CopyRef { pub base: EntityId, pub elem: u32 }  // elem 0 = identity
pub struct Glue { pub a: CopyRef, pub b: CopyRef }
pub struct UnfoldedModel { /* elements: Vec<Placement>, glued, exposed, lineage ref */ }
impl UnfoldedModel {
    pub fn elements(&self) -> &[Placement];
    pub fn volumes(&self) -> impl Iterator<Item = CopyRef>;
    pub fn color_of(&self, c: CopyRef) -> Option<EntityId>;
    pub fn kind_of(&self, face: CopyRef) -> Option<FaceKindView>;  // glued deck faces → interior
    pub fn sense(&self, face: CopyRef) -> Sense;
    pub fn origin_of(&self, c: CopyRef) -> Option<EntityId>;       // pre-cut entity via recorded lineage
    pub fn glued(&self) -> &[Glue]; pub fn exposed(&self) -> &[CopyRef];
    pub fn refold(&self) -> &Symmetry;                             // exact
}
pub fn unfold(m: &Model, t: &impl TopologyView, spec: &UnfoldSpec) -> Result<UnfoldedModel, ModelError>;
```

**Orientation rule for improper copies (the mesh layer inherits this contract):**
1. A copy under an improper g reverses the loop of each image face, so the stored normal again follows the right-hand rule.
2. left/right is then re-derived from the copy's volume ids (ir.md §6).
3. `Sense` is never stored as "flipped relative to the base", and the swap at the domain boundary is not mixed into it.
4. `Placement.orientation_reversed = g.is_improper()` records the flip.

### 5.5 Instancing (shares Placement and group action with unfold)

```rust
pub struct Placement { pub transform: Transform, pub orientation_reversed: bool }
pub enum PathStep { Instance(EntityId), Slot { lattice: EntityId, ix: SlotIx } }
pub type InstancePath = SmallVec<[PathStep; 4]>;   // stable id of a flattened copy; no minted ids
pub struct SlotIx { pub i: i32, pub j: i32, pub k: i32 }
pub enum HexOrient { XRows, YRows }
pub enum Layout { Cartesian { n: [u32;3], pitch: Vector3 }, Hex { rings: u32, pitch: f64, orient: HexOrient, nz: u32, dz: f64 } }
pub enum Fill { Prototype(EntityId), Lattice(EntityId), Void }
pub struct ColorMap { pub volume: BTreeMap<EntityId, EntityId>, pub face: BTreeMap<EntityId, EntityId> } // typed by domain
pub struct Slot { pub fill: Fill, pub recolor: ColorMap }
pub struct Lattice { pub layout: Layout, pub origin: Transform, pub outer: Fill, pub slots: BTreeMap<SlotIx, Slot> } // sparse
pub struct Prototype { pub volumes: BTreeSet<EntityId> }
pub struct InstanceRow { pub of: Fill, pub at: Transform, pub recolor: ColorMap }
pub struct PathOverride { pub prefix: InstancePath, pub recolor: ColorMap }
impl Model {
    pub fn add_prototype(&mut self, vols: &[EntityId]) -> Result<EntityId, ModelError>;
    pub fn add_lattice(&mut self, l: Lattice) -> Result<EntityId, ModelError>;   // cycle-checked
    pub fn add_instance(&mut self, i: InstanceRow) -> Result<EntityId, ModelError>;
    pub fn add_path_override(&mut self, o: PathOverride) -> Result<(), ModelError>;
}
pub struct FlatEntry { pub path: u32, pub placement: u32, pub volume: EntityId, pub color: EntityId }
pub struct FlatModel { pub placements: Vec<Placement>, pub paths: Vec<InstancePath>, pub entries: Vec<FlatEntry> /* + path index */ }
impl FlatModel { pub fn find(&self, p: &InstancePath) -> Option<std::ops::Range<usize>>; pub fn fingerprint(&self) -> u64; }
pub fn flatten(m: &Model) -> Result<FlatModel, ModelError>; // canonical: instance id, slot (k,j,i), volume id
```

- **Recolor order:** prototype → inner slot → outer slot → instance → path override. The outer level wins, and a per-volume mapping beats a per-color mapping.
- **Domain check:** a volume color may only map to a volume color, and the validator checks this.

### 5.6 Decomposition (part of Model, with serde)

```rust
pub enum PieceRef { Whole(EntityId /*volume*/), Cell(EntityId /*PartitionCell*/) }
pub enum CutGeometry { Plane(Plane), Surface(EntityId /*Surface*/), Faces(Vec<EntityId>) }  // no "Unspecified"
pub struct CutId(pub u32);   // local until a PartitionFace kind exists (Q1)
pub enum PieceFace { Whole(EntityId), Part { face: EntityId, piece: EntityId }, Cut { cut: CutId, side: Keep } }
pub enum PieceShape { Any, Sweep { source: Vec<PieceFace>, target: Vec<PieceFace> }, Map }
pub struct Piece { pub id: EntityId, pub volume: EntityId, pub faces: Vec<PieceFace>, pub shape: PieceShape } // no color field
impl Model {
    pub fn split(&mut self, piece: PieceRef, cut: CutGeometry, neg: &[EntityId], pos: &[EntityId],
                 t: &impl TopologyView) -> Result<[EntityId; 2], ModelError>;  // faces must bound the volume
    pub fn composite(&mut self, pieces: &[PieceRef]) -> Result<EntityId, ModelError>; // same color only
    pub fn set_shape(&mut self, piece: EntityId, s: PieceShape) -> Result<(), ModelError>;
    pub fn unsplit(&mut self, piece: EntityId) -> Result<PieceRef, ModelError>;     // restores parent's original id
    pub fn pieces_of_volume(&self, v: EntityId) -> SmallVec<[PieceRef; 4]>;           // indexed
    pub fn pieces_of_face(&self, f: EntityId) -> SmallVec<[PieceRef; 2]>;             // indexed
    pub fn piece_volume(&self, p: PieceRef) -> EntityId;
    pub fn piece_color(&self, p: PieceRef) -> Option<EntityId>;                       // reads through the volume
}
```

### 5.7 Validator

```rust
pub enum Severity { Error, Warning }
pub enum IssueCode {
  ColoringNotPartition, ColorDomainMismatch, LabelDuplicate, BoundaryFaceNoKind, BoundaryFaceUncolored /*warn*/,
  KindOnInteriorFace, DeckMateNotReciprocal, DeckGeometry, DeckOrientation, MirrorOffPlane, TranslationNotNormalized,
  GeneratorUnused, DeckNotInSymmetry /*warn*/, GroupNotFinite, InterfaceNotFacing, InterfaceUnmatched, InterfaceSameColor /*warn*/,
  CapOffPlane, InstanceCycle, RecolorDomainMismatch, RecolorUnknownKey, PieceFacesNotOfVolume, PieceTilingBroken,
  SweepSourceTargetAdjacent, DecompositionChangedColoring, UnknownEntity,
}
pub struct Issue { pub code: IssueCode, pub severity: Severity, pub entities: SmallVec<[EntityId; 2]>, pub detail: String }
pub struct Report { pub issues: Vec<Issue> }   // sorted by (code, entities)
impl Report { pub fn is_ok(&self) -> bool; pub fn has(&self, c: IssueCode) -> bool; }
pub fn validate(m: &Model, t: &impl TopologyView, tol: Tolerance) -> Report;
pub fn validate_unfolded(u: &UnfoldedModel, m: &Model, t: &impl TopologyView, tol: Tolerance) -> Report;
```

Deck checks are scoped to each deck, which fixes grouptheory's report-wide gate. All checks share one `GeomCache` per call.

### 5.8 Tests required beyond the task's scenarios

- Reflective cell with mirrors at both x=0 and x=1: valid; unfold needs a window.
- Curved periodic disks whose seams sit at different angles: valid.
- Faces with equal area and centroid but different shape: `DeckGeometry`.
- Translation pair with both volumes on the same side: `DeckOrientation`.
- Hex prism rotated 15°: 7 mirrors, group order 24, `complete=true`.
- Mirror cut through an existing periodic pair: children are re-paired.
- Non-conformal interface with 1 face against 2.
- Path override in one assembly only.
- `unsplit` restores the original piece id.
- Flatten fingerprint is identical across runs.

## 6. Open questions for the orchestrator

1. **api_change_request: sm-core `EntityKind`**
   - What: split `Instance` into `Prototype`, `Instance` and `Lattice`, and add `PartitionFace` (a virtual cut face). Possibly add `Symmetry`.
   - Why: every judge wanted ids that say definition vs placement. Cut faces also need persistent ids for mesh classification, instead of a local `CutId`.
   - Proposed signature: `Prototype = 36, Lattice = 37, PartitionFace = 49`.
2. **api_change_request: sm-core `Transform`**
   - What: add compose, inverse and constructors. sm-core today has only `apply_point`, `apply_vector`, `det` and `is_improper`.
   - Why: all three candidates rewrote these locally, and unfold, lattices and the deck lookup all need them.
   - Proposed signature: `compose(&self,&Self)->Self; inverse(&self)->Option<Self>; reflection(n:Vector3,d:f64)->Self; rotation(p:Point3,axis:Vector3,angle:f64)->Self; translation(v:Vector3)->Self; const IDENTITY`.
3. **sm-topo-api shape**
   - Is it OK to return `&[EntityId]` slices? That implies sorted CSR id tables.
   - Should there be a dense `face_ordinal(f) -> Option<u32>`, so `DeckLookup` is truly O(1)?
   - Can `face_moments` be computed for analytic and NURBS faces (quadrature within tolerance is acceptable)?
   - Is `face_edges` acceptable as an addition?
4. **Lineage type.** ARCHITECTURE says sm-naming records lineage. Should sm-model-api use an `sm-naming-api` `Lineage`, or keep its own and convert? Either way, the topology cut op must return `CutOutcome{lineage, caps}`, which is a new contract on the op layer.
5. **Imprint/merge status.** Should interface validation read topology's per-pair status (merged / imprinted / coincident-unimprinted) through a query such as `coincident_faces(f)`, rather than inferring coincidence from moments?
6. **Translation symmetry** stays declared, not detected; none of the candidates detects it. Confirm.
7. **Mixed-material cells** stay out, and volume coloring is an exact partition (ir.md §7). Confirm for P0-P1.
8. **Determinism test.** API crates cannot depend on rayon, so the 1-thread vs N-thread test is vacuous here. Options:
   - keep a two-run test plus a hash comparison across spawned threads; or
   - allow rayon as a dependency.

   `sm-testkit` does not exist in the tree yet, so its property tests cannot be used.
9. **Cargo.lock.** The new deps (smallvec, thiserror) change the root lockfile. The orchestrator should land that change on merge.
10. **Boundary color policy.** Should an uncolored boundary face be an Error or a Warning? Proposed: Warning. Cap faces take the `cap_color` passed to `apply_cut`.