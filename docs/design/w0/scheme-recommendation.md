# sm-scheme-api: final design recommendation

Inputs: three panel candidates and two judge reports. Short commit ids below:
- `constraint`: `ddd5598` = `ddd559894ed57107994d2e644c60a925f2f14f65`
- `pipeline`: `b7bf13e` = `b7bf13ee30be85e34e4f038c4881649bfb3edb3f`
- `transaction`: `40b5c3f` = `40b5c3fd1f650bbe57929d3184103bdb4786f155`

Paths such as `ddd5598:src/driver.rs:689` mean `crates/sm-scheme-api/src/driver.rs`, line 689, at that commit.

## 1. Verdict and scores

**Base: `constraint`.** Both judges picked it independently.

| Candidate | Judge 0 | Judge 1 | Mean |
|---|---|---|---|
| constraint (constraint system first) | 7.6 | 8.0 | **7.8** |
| pipeline (plan/apply with capabilities) | 7.3 | 7.0 | 7.15 |
| transaction (multi-entity jobs + ledger) | 6.8 | 6.0 | 6.4 |

Judge 0 also scored each brief scenario out of 10. Items (a), (c), (d), (e) and (g) ran on the sm-topo-api brick; (b) and (f) only compiled.

| Scenario | constraint | pipeline | transaction |
|---|---|---|---|
| (a) vertex types + corner override | 8 | 8 | 6 |
| (b) many-to-one sweep over 3 volumes | 5 | **8** | 7 |
| (c) pave → trimesh fallback | 7 | **8** | 7 |
| (d) deck pairs × sweep | 7 | 7 | 7 |
| (e) auto selection with reasons | **8** | 6 | 6 |
| (f) decomposition pieces + `PieceShape` | **8** | 7 | – |
| (g) atomic groups + conversion | **9** | 6 | 8 |

Deck × sweep matrix as judge 0 measured it. The middle row matters for periodic pin cells.

| Deck pair | constraint | pipeline | transaction |
|---|---|---|---|
| Linking faces (A=−x, B=+x) | OK | OK | OK |
| A = sweep source, B = sweep target | `CopyConflict` | OK (verify_copy) | OK (deck_checks) |
| A = sweep target, B = sweep source | dependency cycle | `NoScheme` | job dependency cycle |

The base fails both bad rows. Defect D2 (§4) fixes both, and none of the three candidates handles the bottom row today.

## 2. Base candidate

`constraint` at **`ddd559894ed57107994d2e644c60a925f2f14f65`** (`crates/sm-scheme-api/`). W1 starts from that tree, then applies §3 and §4 to reach the §5 API.

Why this base:
1. **Its planning order matches `docs/research/cubit-schemes.md`.**
   1. Schemes state requirements (child impositions, roles, copies, corner rules).
   2. A **driver-owned** `CornerPicker` produces one shared `FaceLayout` per face (corner types plus `LogicalSide`s).
   3. Only then do schemes write interval equations, against that layout.

   In pipeline and transaction every scheme picks its own corners, so two schemes can disagree about the same face.
2. **One inspectable constraint store.** Every constraint is a `Record { id, origin }` (`ddd5598:src/problem.rs`). The whole `MeshingProblem` sits inside a fully serde `Plan`, so a plan can live in an Op or in git and can be diffed. `Infeasible { constraints, entities }` and `ImpositionConflict { child, imposed_by, required, conflict }` say exactly what conflicts with what. In the other two candidates the plan is only partly serde (pipeline) or not serde at all (transaction).
3. **Cubit's ownership model.** Linking faces are meshed by map/submap, and the sweep reads them through `NodeGrid::line_from`. Canonical `NodeKey`s mean a face scheme knows its boundary nodes without asking the curve scheme (`GenCtx::edge_nodes`/`coedge_nodes`/`side_nodes`). Commit numbers vertices by key, so 1 thread and N threads give bit-identical meshes.
4. **Exact copies.** Copies map node keys through vertex and edge correspondences, and orientation comes from the real face normals (`det(M)·(M n_s)·n_t`, `ddd5598:src/copy.rs:167`). Both other candidates use `reverse = !improper`, which judge 1 flags as plausibly wrong for face uses with mixed senses.
5. **The model layer is first class.** `PieceResolver` (default `WholeFaces`) makes volume targets decomposition pieces. The driver turns `PieceShape::Sweep`/`Map` into a `Selection::PieceShape` choice. Groups (`Target::Group` plus `MeshConstraint::Atomic`) are keyed on the group, not on a member. It also has the strongest determinism test of the three: plan JSON plus both mesh hashes, compared across `Sequential`, `ScopedThreads(4)` and 4 concurrent threads.

What the base gets wrong is mostly in the driver (child claims, copy merging, dimension-locked waves, fallback reach). §3 and §4 cover it.

## 3. Grafts

Each graft says what changes, where it goes in the base, and where it comes from. The judges' two lists overlapped by about 80% and are merged here into one list. Where the judges disagreed, the decision is stated.

**G1. Child claims and footprint scheduling** (one graft, not two).
- `Requirements::claim(child)` is how a volume or group scheme declares that it writes a lower-dimensional child itself: the target of a many-to-one sweep, or linking faces when a sweep wants them. Source: pipeline `Plan::claims` / `ClaimedSurface` (`b7bf13e:src/plan.rs`, `src/patch.rs`).
- A claimed child gets `TargetPlan { selection: Selection::Claimed { by } }` and is not resolved on its own.
- `check_patch` lets a patch own nodes of its claimed children (fixes D1).
- Each job's `Footprint { reads, writes }` is **derived**, never written by hand:
  - writes = the job's own target plus its claims;
  - reads = its boundary children that it does not claim, its copy source, its `Precedes` predecessors, and the claimer of any child it reads.
- Waves come from Kahn levelling over footprints. This replaces `ddd5598:src/driver.rs:640-680 waves()`, which only levels within one dimension.
- Write conflicts, missing writers and cycles are typed errors that name the constraint ids involved.
- Source for the levelling: transaction `Footprint`, `Footprint::independent` and `MeshProblem::schedule` (`40b5c3f:src/job.rs`, `src/driver.rs:360`).

**G2. Capability impositions.**
- Add `Caps` (TRI, QUAD, STRUCTURED, TET, HEX, WEDGE, PYRAMID, POLY, BOUNDARY_LAYER, MESHES_CHILDREN, INHERIT) and `Scheme::caps(&Params) -> Caps`.
- An imposition carries `Need::{Schemes(names), Caps(caps), Forbid(caps)}` with a firmness.
- `INHERIT` resolves through the copy source. A deck copy of a mapped face therefore satisfies a sweep's `QUAD | STRUCTURED` requirement, and a new structured scheme needs no edits in the crates that impose requirements (about 25 parallel scheme crates).
- Name lists stay available for the rare "exactly this algorithm" case.
- Deck faces get `Forbid(BOUNDARY_LAYER)` (Hard, origin `Deck`), which replaces `MeshConstraint::NoBoundaryLayer`.
- Run the check in `resolve()` **and** for copy targets (fixes D3).
- Source: pipeline `Caps` (`b7bf13e:src/caps.rs`), `Requirement::{Caps, Forbid}` (`src/plan.rs`), `resolve_caps`/`check_reqs` (`src/driver.rs`).

**G3. Registry fallback chains and auto lists.**
- `SchemeRegistry::{set_fallback, fallbacks, set_auto, auto}`.
- Resolution order: intent chain → registry fallbacks of the last intent choice (`Selection::Registry { rank }`) → `PieceShape` → imposed set or auto.
- `MeshIntent.defaults.auto`, when it is non-empty, overrides the registry's auto list.
- Source: `b7bf13e:src/registry.rs`.

**G4. Group fallbacks.** Add `MeshGroup.fallbacks: Vec<SchemeChoice>`; `resolve` uses it instead of `Vec::new()` (`ddd5598:src/driver.rs:273`). Source: transaction `GroupIntent.scheme: SchemeIntent` with `SchemeChoice::Chain` (`40b5c3f:src/intent.rs`).

**G5. Attempt history and a wider `FailureKind`.**
- `FailureKind::{NotApplicable, Rejected, Constraint, Geometry, Quality, Unsupported, Internal}`, with `allows_fallback()`. Only `Internal` is fatal.
- `Attempt { target, scheme, phase, kind, message }`, collected in `Plan::attempts`, `Meshed`, `NoApplicableScheme` and `ReplanLimit`.
- The winner's `Applicability` reason goes into `TargetPlan::reason`.
- Source: `b7bf13e:src/error.rs` (`FailureKind`, `Phase`, `Attempt`).

**G6. Linear interval constraints.**
- Replace `SumEqual` with `Linear { terms: Vec<(EntityId, i32)>, rel: Rel::{Eq, Le, Ge}, rhs: i64 }`.
- Keep `IntervalSink::equal`/`even` as helpers.
- Submap and many-to-one path sums need coefficients and inequalities.
- Source: `b7bf13e:src/interval.rs` (`Term`, `Relation`, `Linear`).

**G7. Scheme-private plan data.**
- `Requirements::set_data(Params)` stores data in `TargetPlan::data`, and `IntervalCtx` and `GenCtx` read it back.
- Then the sweep does not recompute `SweepTopo` in every phase.
- Source: pipeline `Plan::data` (`b7bf13e:src/plan.rs`).

**G8. Narrow inputs for unstructured and curve schemes.**
- `GenCtx::loops(face) -> Vec<BoundaryLoop>`, where each node is a `NodeKey`. Pave and tri* get exactly the boundary they need.
- `GenCtx::curve_patch(edge, fractions)` builds a valid curve patch from `n−1` fractions, so bias, dualbias and curvature schemes never compute positions or keys themselves.
- Source: pipeline `BoundaryLoop`/`BoundaryUse` (`b7bf13e:src/scheme.rs:72-92`) and `CurvePatch { fractions }` (`src/patch.rs:44`).

**G9. Patch with a local node table and a dense `PatchSet`.**
- Elements become `u32` indices into the patch's own table: indices `0..nodes.len()` are new nodes, and the rest index into `refs: Vec<NodeKey>`.
- `PatchSet` stores points and params as dense SoA in commit order, plus a lookup-only `NodeKey → u32` index (never iterated). Commit becomes O(1) per reference instead of a `BTreeMap` lookup.
- `NodeKey` stays the canonical name.
- A `PatchBuilder` dedups refs.
- Source: transaction `MeshPatch`/`PatchBuilder` (`40b5c3f:src/patch.rs:90`) and pipeline `CommittedMesh` (`b7bf13e:src/mesh.rs`).

**G10. Standalone transforms.**
- `MeshIntent::transforms: Vec<TransformIntent>` lists THex, HTet and QTri over arbitrary volume targets, not only groups.
- Each becomes `Job::Transform(id)`, with reads = writes = its targets, scheduled after every writer of those targets.
- `MeshTransform::apply` rewrites copies of the targets' patches. The driver swaps them in only if all succeed.
- `MeshGroup.conversion` stays as shorthand.
- Source: transaction `TransformIntent` + `JobKind::Transform` (`40b5c3f:src/intent.rs:298`, `src/job.rs:92`).

**G11. Post-commit copy verification.**
- When a copy relation cannot be generated, because its target is claimed or written by another job, it is demoted to `CopyMode::Verify`.
- After generation the driver compares the two meshes by key correspondence and returns `CopyMismatch { constraint }` (fallback kind `Constraint`).
- Source: transaction `verify_copy` + `deck_checks` (`40b5c3f:src/copy.rs:142`, `src/driver.rs`). Unlike transaction, match by key correspondence, not by quantized centroids.

**G12. Object-safe executor.**
- `Executor::run(&self, n, job: &(dyn Fn(usize) + Sync))`. The driver collects results into pre-allocated slots and reads them in index order.
- The mesher crate can then pass a `&dyn Executor` backed by rayon.
- Source: transaction `Executor` (`40b5c3f:src/driver.rs`).

**Decisions where the judges disagreed:**
- **Params: keep `serde_json::Value`** (judge 1), and do not adopt pipeline's `ParamValue` tree (judge 0's graft).
  - `sm_op_api::codec::Value` *is* `serde_json::Value`, restricted to TOML (`crates/sm-op-api/src/codec.rs:14`), and W0a op-api decision 2 allows `serde_json` + `toml`. A second value model would have to be converted at every Op boundary.
  - Graft only the validation: `Params::check()` runs the value through `sm_op_api::codec::to_params` (TOML-representable, no null, finite floats) when the intent is loaded.
  - `Params::empty()` becomes `{}` (D9).
  - Entity references are `EntityId` strings (`Kind#serial`) read by `Params::get::<EntityId>`.
- **Fallback replay: `Rejections` stays out of the intent** (judge 1), so it does not take transaction's `MeshIntent::excluded` (judge 0).
  - `Rejections` becomes serde and is stored in `Plan::rejections` and `Meshed`.
  - Replay is `(intent, rejections)` through `plan_with`. Storing the plan reproduces the mesh exactly, and the user's Op payload never holds solver state.
- **No `ShapeView` trait** (against judge 1's graft). `&dyn TopologyView` is already object-safe, and sm-topo-api's `CellComplex` fixtures let a scheme crate test without a kernel. A second query trait would duplicate a frozen contract. Graft its *queries* as free helpers in `geom` instead: `face_loops`, `corner_angle`, `snap_to_face`, `volume_face_uses`.

Rejected outright: transaction's per-scheme corner picking, pipeline's f64 applicability score (keep `u32`), and transaction's sweep that writes its own linking faces (it breaks the map/submap linking-face model).

## 4. Must-fix defects in the base

Line numbers refer to `ddd5598`.

- **D1. A volume cannot write a child face.** `check_patch` (`src/driver.rs:689-705`) only accepts nodes owned by the volume or group members. A many-to-one sweep, or any sweep that meshes its own target, therefore needs a helper surface scheme that reads the sweep's parameters back. Fix with G1 claims. Test: a 3-volume many-to-one sweep that writes its target face directly.
- **D2. Copy constraints are treated as directed and unique.**
  - Middle row of the matrix: `CopyConflict` at `src/driver.rs:404-409` when a deck copy and a sweep-target copy name the same pair.
  - Bottom row: a cycle in `waves()` when they point in opposite directions.
  - Fix: treat copy constraints as **edges of an equivalence relation** between faces (or curves), each carrying its transform.
    1. The driver forms classes with union-find.
    2. It picks one **root** per class: the member meshed by a scheme. Preference order: a sweep source, then a face with a Hard non-copy intent, then the lowest id.
    3. It derives every other member's transform by composing along the class tree.
    4. A redundant edge is kept only as a check: its transform must equal the composed one within `ModelTolerance::linear` (on the image of the face's vertices). If it does not, the result is `CopyConflict { first, second, reason }`.
    5. A member that is claimed by another job becomes `CopyMode::Verify` (G11).
  - Result for the bottom row: B (sweep source) is the root, A = copy(B, T⁻¹), then the volume runs. The deck relation holds by construction because A = T⁻¹(B) ⇔ B = T(A).
  - Tests: both bad rows of the matrix. Also cover the sweep stub's mean-shift translation, which differs from the deck vector in the last ulp.
- **D3. Copy targets bypass impositions.** `copy_plan` (`src/driver.rs:198-199, 210-211, 627-636`) is called instead of `resolve()`, so a sweep's imposition on a linking face that is also a deck copy is never checked. It works today only because copies happen to keep the grid. Fix: run the imposition and caps check for copy targets, with `caps = INHERIT` resolved through the class root.
- **D4. Copies added after the closure are never closed.** `copy_closure` runs once (`src/driver.rs:193`), before face and curve `requirements`. A copy contributed by a surface or curve scheme therefore gets no vertex map, no interval equality and no conflict check. Fix: run the closure after each dimension's requirements, so it covers copies from every dimension.
- **D5. Requirements on non-children are silently ignored.** `Requirements` accepts impositions, roles and copies on any target, including ones already resolved (a face imposing on a volume, or a volume on a sibling volume), and those are dropped without notice. Fix: a requirement's child must be a strict sub-entity of the target (or of a group member) inside the region, and not already resolved; otherwise return `BadRequirement { by, child, reason }`. Make `Requirements::new` and `IntervalSink::new` `pub(crate)` so origins cannot be forged.
- **D6. Dimension-locked waves.** `waves()` rejects any dependency on a higher-rank target (`src/driver.rs:662`), so "face after volume" cannot be expressed. Example: a deck copy whose source face is claimed by a sweep. Fixed by G1.
- **D7. Fallback reaches only the scheme that failed to generate.**
  - **Interval infeasibility** is a hard error (`src/driver.rs:240`), even though `Infeasible.constraints` points at the origins. Fix: the driver fills `Infeasible::blame`. Among the involved `Origin::Scheme { scheme, target }` whose selection is not Hard intent, it blames the lowest-dimension one; ties go to the larger `ConstraintId` (the later contribution). For example, pave's `EvenSum` loses to the sweep's equalities. `Driver::mesh` rejects the blamed pair and re-plans. None of the three candidates does this.
  - **A child with no applicable scheme under an imposition** is fatal today. Fix: blame the imposing parent unless the parent's choice is Hard. Example: sweep → tetmesh when a linking face cannot be submapped.
  - Planning errors therefore enter the re-plan loop too, through `SchemeError::blame()`.
- **D8. Diagnostics are lost.**
  - The winner's reason is dropped (`src/driver.rs:316, 349-361`).
  - Auto selection among imposed schemes discards its score (`Selection::Imposed`).
  - Applicable candidates that lose are not recorded.
  - `ReplanLimit` carries no history.
  - Fixed by G5: every candidate evaluated becomes an `Attempt`.
- **D9. Params are not TOML-safe.** `Params::empty()` is JSON `null` (`src/intent.rs`), and `TargetPlan.params` serializes it inside `Plan`. Fix: `{}`, `Params::check()` on load, and a serde test that round-trips `Plan` through `sm_op_api::codec`.
- **D10. `Executor::map` is generic** (`src/exec.rs:9`), so it is not object-safe. Fixed by G12.
- **D11. Hot paths go through `BTreeMap`.**
  - `PatchSet` is a `BTreeMap<NodeKey, (Point3, Param)>`, and commit does one map lookup per element-node reference (`src/driver.rs:540-561`).
  - Elements carry 16-byte `NodeKey`s.
  - `MeshingProblem::copy_into` and `roles_of` are linear scans.
  - `NodeGrid::line_from` is a linear search (`src/patch.rs:114`).
  - Fix: G9, plus sorted-index lookups for copies and roles, plus `NodeGrid::position(key)` from a sorted boundary index.
- **D12. Contract text is ambiguous.**
  - `NodeKey` docs and DESIGN say an edge owns `(e, 1..n)`, but `check_patch` requires `1..=n−1` (`src/driver.rs:713-718`). Write `1..=n−1` everywhere.
  - Copied nodes get `Param::None` (`src/copy.rs`). Keep this for v1 and document it; a later pass may re-project.
  - Remove the dead parameters `let _ = input` (`check_patch`) and `let _ = plan` (`commit`).
  - `Elements::get` and `NodeGrid::at` panic out of range. Return `Option`, because they read patches that schemes produced, and CLAUDE.md forbids panics that inputs can reach.
- **D13. No polyhedral or volume-copy elements.** `commit` rejects `CellShape::Polyhedron` (`src/driver.rs:556`), but the catalogue has surface and volume `polyhedron`. Fix: `Elements::push_polyhedron(face_loops)` (face CSR), committed through `MeshBuild::add_face` + `add_cell(&[SignedFace], None)`. Volume copies stay out of v1 (§6 Q3).

## 5. Final proposed public API

This is the base plus §3 and §4. Rustdoc is omitted, but every item gets it. All maps are `BTreeMap` or sorted `Vec`.

```rust
// ---- target.rs ----
pub enum SchemeDim { Curve, Surface, Volume }                    // Ord; rank() -> u8
pub struct GroupId(pub u32);
pub struct TransformId(pub u32);
pub enum Target { Curve(EntityId), Surface(EntityId), Volume(EntityId), Group(GroupId) }
impl Target {
    pub const fn dim(&self) -> SchemeDim;
    pub const fn entity(&self) -> Option<EntityId>;
    pub fn piece(&self) -> Option<PieceRef>;
    pub const fn of_piece(p: PieceRef) -> Target;
}
pub fn piece_of(id: EntityId) -> Option<PieceRef>;
/// Topology vertex: index 0. Edge with n intervals: 1..=n-1 in the edge's own direction.
/// Face/volume: generator-defined. A copy keeps the source index.
pub struct NodeKey { pub owner: EntityId, pub index: u32 }
impl NodeKey { pub const fn vertex(v: EntityId) -> Self; pub const fn new(owner: EntityId, index: u32) -> Self; }
pub enum Job { Generate(Target), Transform(TransformId) }        // Ord

// ---- firm.rs ----
pub enum Firmness { Default, Soft, Hard }
pub struct Firm<T> { pub value: T, pub firmness: Firmness }      // hard/soft/auto/is_hard

// ---- caps.rs (G2) ----
pub struct Caps(pub u32);
impl Caps {
    pub const NONE: Caps; pub const TRI: Caps; pub const QUAD: Caps; pub const STRUCTURED: Caps;
    pub const TET: Caps; pub const HEX: Caps; pub const WEDGE: Caps; pub const PYRAMID: Caps;
    pub const POLY: Caps; pub const BOUNDARY_LAYER: Caps; pub const MESHES_CHILDREN: Caps;
    pub const INHERIT: Caps;
    pub const fn contains(self, o: Caps) -> bool;
    pub const fn intersects(self, o: Caps) -> bool;
}                                                                // BitOr, BitOrAssign

// ---- intent.rs ----
pub struct SchemeName(pub String);                               // COPY = "copy" reserved
pub struct Params(pub serde_json::Value);                        // == sm_op_api::codec::Value
impl Params {
    pub fn empty() -> Self;                                      // {}
    pub fn check(&self) -> Result<()>;                           // TOML-representable, finite
    pub fn get<T: DeserializeOwned>(&self, scheme: &str, key: &str) -> Result<Option<T>>;
}
pub struct SchemeChoice { pub name: SchemeName, pub params: Params, pub firmness: Firmness }
pub enum IntervalRequest { Count(u32), Size(f64) }
pub struct BoundaryLayerIntent { pub first: f64, pub growth: f64, pub layers: u32 }
pub struct EntityIntent {
    pub scheme: Option<SchemeChoice>, pub fallbacks: Vec<SchemeChoice>,
    pub intervals: Option<Firm<IntervalRequest>>, pub boundary_layer: Option<BoundaryLayerIntent>,
}
pub struct CornerIntent { pub corner: CornerKey, pub vertex_type: Firm<VertexType> }
pub struct MeshGroup {
    pub id: GroupId, pub members: Vec<EntityId>,
    pub scheme: Option<SchemeChoice>, pub fallbacks: Vec<SchemeChoice>,   // G4
    pub conversion: Option<SchemeChoice>,
}
pub struct TransformIntent { pub id: TransformId, pub targets: Vec<EntityId>, pub transform: SchemeChoice } // G10
pub enum SizingSpec { Constant { size: f64 }, Function { name: String, params: Params } }
pub struct AutoPolicy { pub curve: Vec<SchemeName>, pub surface: Vec<SchemeName>, pub volume: Vec<SchemeName> }
pub struct Defaults { pub sizing: Option<SizingSpec>, pub auto: AutoPolicy }
pub struct MeshIntent {
    pub entities: BTreeMap<EntityId, EntityIntent>, pub corners: Vec<CornerIntent>,
    pub groups: Vec<MeshGroup>, pub transforms: Vec<TransformIntent>, pub defaults: Defaults,
}
impl MeshIntent {
    pub fn entity_mut(&mut self, id: EntityId) -> &mut EntityIntent;
    pub fn set_scheme(&mut self, id: EntityId, c: SchemeChoice);
    pub fn set_intervals(&mut self, id: EntityId, r: Firm<IntervalRequest>);
    pub fn canonicalize(&mut self);
    pub fn check(&self) -> Result<()>;                           // params, group/transform ids
}

// ---- vertex.rs ----
pub enum VertexType { End, Side, Corner, Reversal }              // element_count, nominal_angle,
                                                                 // submap_weight, nearest
pub enum VertexRule { Free, Map, Submap, Ends(u8) }
pub struct CornerAssignment { pub corner: CornerKey, pub vertex_type: VertexType, pub firmness: Firmness, pub angle: f64 }
pub enum SideDir { PlusI, PlusJ, MinusI, MinusJ }                // from cumulative submap turns
pub struct LogicalSide { pub coedges: Vec<CoedgeKey>, pub dir: SideDir }
pub struct FaceLayout { pub face: EntityId, pub rule: VertexRule, pub corners: Vec<CornerAssignment>, pub sides: Vec<LogicalSide> }

// ---- problem.rs ----
pub struct ConstraintId(pub u32);
pub enum Origin { Intent, Sizing, Scheme { scheme: SchemeName, target: Target }, Deck { boundary: EntityId }, Piece { piece: EntityId }, Driver }
pub struct Record<T> { pub id: ConstraintId, pub origin: Origin, pub item: T }
pub enum Rel { Eq, Le, Ge }
pub enum IntervalConstraint {                                    // G6
    Fixed { edge: EntityId, count: u32, firmness: Firmness },
    Goal { edge: EntityId, goal: f64 },
    Range { edge: EntityId, min: u32, max: Option<u32> },
    Linear { terms: Vec<(EntityId, i32)>, rel: Rel, rhs: i64 },
    EvenSum { edges: Vec<EntityId> },
}
impl IntervalConstraint { pub fn sum_equal(lhs: &[EntityId], rhs: &[EntityId]) -> Self; pub fn edges(&self) -> Vec<EntityId>; }
pub enum Need { Schemes(Vec<SchemeName>), Caps(Caps), Forbid(Caps) }   // G2
pub struct Imposition { pub child: Target, pub need: Need, pub firmness: Firmness, pub by: Target }
pub struct Claim { pub child: Target, pub by: Target }          // G1
pub enum Role { SweepSource { volume: Target }, SweepTarget { volume: Target }, SweepLinking { volume: Target } }
pub enum CopyReason { Deck { boundary: EntityId }, SweepTarget { volume: Target }, User, Closure { parent: ConstraintId } }
pub struct CopyConstraint { pub source: Target, pub target: Target, pub transform: Transform, pub vertex_map: Vec<(EntityId, EntityId)>, pub reason: CopyReason }
pub enum MeshConstraint {
    PlaneLock { face: EntityId, normal: Vector3, offset: f64 },
    Precedes { before: Target, after: Target },
    Atomic { group: GroupId, members: Vec<Target> },
    BoundaryLayer { face: EntityId, volume: Target, spec: BoundaryLayerIntent },
}
pub enum Selection {
    Intent { rank: u32 }, Registry { rank: u32 }, PieceShape, Imposed { score: u32 },
    Auto { score: u32 }, Copy { class: u32 }, Claimed { by: Target },
}
pub struct TargetPlan {
    pub target: Target, pub scheme: SchemeName, pub params: Params, pub data: Params,  // G7
    pub caps: Caps, pub selection: Selection, pub reason: String,                    // G5
}
pub struct MeshingProblem {
    pub targets: Vec<TargetPlan>, pub vertices: Vec<EntityId>,
    pub impositions: Vec<Record<Imposition>>, pub claims: Vec<Record<Claim>>,
    pub roles: Vec<Record<(Target, Role)>>, pub copies: Vec<Record<CopyConstraint>>,
    pub vertex_rules: Vec<Record<(EntityId, VertexRule)>>,
    pub vertex_types: Vec<Record<(CornerKey, Firm<VertexType>)>>,
    pub intervals: Vec<Record<IntervalConstraint>>, pub constraints: Vec<Record<MeshConstraint>>,
}
impl MeshingProblem {
    pub fn target(&self, t: Target) -> Option<&TargetPlan>;
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_;
    pub fn record_origin(&self, id: ConstraintId) -> Option<&Origin>;
}
pub struct Requirements<'a> { /* pub(crate) */ }
impl Requirements<'_> {
    pub fn impose(&mut self, child: Target, allowed: &[&str]) -> ConstraintId;      // Need::Schemes, Hard
    pub fn require_caps(&mut self, child: Target, caps: Caps, f: Firmness) -> ConstraintId;
    pub fn forbid(&mut self, child: Target, caps: Caps) -> ConstraintId;
    pub fn claim(&mut self, child: Target) -> ConstraintId;
    pub fn role(&mut self, child: Target, role: Role) -> ConstraintId;
    pub fn copy(&mut self, c: CopyConstraint) -> ConstraintId;
    pub fn vertex_rule(&mut self, face: EntityId, rule: VertexRule) -> ConstraintId;
    pub fn vertex_type(&mut self, corner: CornerKey, t: Firm<VertexType>) -> ConstraintId;
    pub fn constraint(&mut self, c: MeshConstraint) -> ConstraintId;
    pub fn set_data(&mut self, data: Params);
}
pub struct IntervalSink<'a> { /* pub(crate) */ }
impl IntervalSink<'_> {
    pub fn equal(&mut self, lhs: &[EntityId], rhs: &[EntityId]) -> ConstraintId;
    pub fn linear(&mut self, terms: &[(EntityId, i32)], rel: Rel, rhs: i64) -> ConstraintId;
    pub fn even(&mut self, edges: &[EntityId]) -> ConstraintId;
    pub fn fix(&mut self, edge: EntityId, count: u32, f: Firmness) -> ConstraintId;
    pub fn range(&mut self, edge: EntityId, min: u32, max: Option<u32>) -> ConstraintId;
}

// ---- plan.rs ----
pub enum CopyMode { Generate, Verify }                           // G11
pub struct CopyMember { pub target: Target, pub from_root: Transform, pub mode: CopyMode, pub constraints: Vec<ConstraintId> }
pub struct CopyClass { pub root: Target, pub members: Vec<CopyMember> }
pub struct Footprint { pub job: Job, pub reads: Vec<Target>, pub writes: Vec<Target> }  // G1, derived
pub struct TransformPlan { pub id: TransformId, pub targets: Vec<Target>, pub transform: SchemeChoice }
pub struct Plan {
    pub problem: MeshingProblem, pub intervals: Vec<(EntityId, u32)>, pub layouts: Vec<FaceLayout>,
    pub copies: Vec<CopyClass>, pub transforms: Vec<TransformPlan>,
    pub footprints: Vec<Footprint>, pub waves: Vec<Vec<Job>>,
    pub rejections: Rejections, pub attempts: Vec<Attempt>,
}
impl Plan {
    pub fn intervals(&self, edge: EntityId) -> Option<u32>;
    pub fn layout(&self, face: EntityId) -> Option<&FaceLayout>;
    pub fn target(&self, t: Target) -> Option<&TargetPlan>;
    pub fn copy_of(&self, t: Target) -> Option<(&CopyClass, &CopyMember)>;
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_;
    pub fn claimed_by(&self, child: Target) -> Option<Target>;
}

// ---- patch.rs (G9, D13) ----
pub enum ElemShape { Segment, Tri, Quad, Polygon, Cell(CellShape), Polyhedron }
pub struct Elements { /* shapes, offsets, nodes: Vec<u32>, poly face CSR */ }
impl Elements {
    pub fn len(&self) -> usize; pub fn is_empty(&self) -> bool;
    pub fn get(&self, i: usize) -> Option<(ElemShape, &[u32])>;
    pub fn polyhedron_faces(&self, i: usize) -> Option<impl Iterator<Item = &[u32]>>;
    pub fn iter(&self) -> impl Iterator<Item = (ElemShape, &[u32])> + '_;
}
pub struct PatchNode { pub key: NodeKey, pub point: Point3, pub param: Param }
pub struct NodeGrid { pub ni: u32, pub nj: u32, pub nodes: Vec<NodeKey> }
impl NodeGrid {
    pub fn at(&self, i: u32, j: u32) -> Option<NodeKey>;
    pub fn position(&self, key: NodeKey) -> Option<(u32, u32)>;
    pub fn line_from(&self, key: NodeKey) -> Option<Vec<NodeKey>>;
}
/// Element index i < nodes.len() names nodes[i]; otherwise refs[i - nodes.len()].
pub struct Patch { pub target: Target, pub nodes: Vec<PatchNode>, pub refs: Vec<NodeKey>,
                   pub elements: Elements, pub grid: Option<NodeGrid>, pub owners: Vec<EntityId> }
impl Patch { pub fn new(target: Target) -> Self; pub fn key(&self, local: u32) -> Option<NodeKey>; }
pub struct PatchBuilder { /* dedups refs */ }
impl PatchBuilder {
    pub fn new(target: Target) -> Self;
    pub fn node(&mut self, key: NodeKey, point: Point3, param: Param) -> u32;
    pub fn refer(&mut self, key: NodeKey) -> u32;
    pub fn push(&mut self, shape: ElemShape, local: &[u32]);
    pub fn push_polyhedron(&mut self, faces: &[&[u32]]);
    pub fn owner(&mut self, piece: EntityId);
    pub fn grid(&mut self, g: NodeGrid);
    pub fn finish(self) -> Patch;
}
pub struct PatchSet { /* dense SoA in commit order + lookup-only key index */ }
impl PatchSet {
    pub fn get(&self, t: Target) -> Option<&Patch>;
    pub fn index_of(&self, k: NodeKey) -> Option<u32>;
    pub fn point(&self, k: NodeKey) -> Option<Point3>;
    pub fn node(&self, k: NodeKey) -> Option<(Point3, Param)>;
    pub fn patches(&self) -> impl Iterator<Item = &Patch>;       // target order
    pub fn nodes(&self) -> impl Iterator<Item = (NodeKey, Point3, Param)> + '_; // commit order
    pub fn node_count(&self) -> usize;
}

// ---- scheme.rs ----
pub enum Applicability { Applicable { score: u32, reason: String }, NotApplicable { reason: String } }
pub trait PieceResolver: Send + Sync {
    fn faces(&self, topo: &dyn TopologyView, model: &Model, piece: PieceRef) -> Result<Vec<EntityId>>;
    fn face_of(&self, topo: &dyn TopologyView, model: &Model, pf: &PieceFace) -> Option<EntityId>;
    fn classify(&self, owner: EntityId) -> Option<EntityId>;
}
pub struct WholeFaces;
pub struct MeshInput<'a> { pub topo: &'a dyn TopologyView, pub model: &'a Model, pub pieces: &'a dyn PieceResolver, pub selection: &'a [EntityId] }
pub struct PlanCtx<'a> { pub input: &'a MeshInput<'a>, pub intent: &'a MeshIntent }
pub struct IntervalCtx<'a> { pub input: &'a MeshInput<'a>, pub layouts: &'a [FaceLayout], pub roles: &'a [(Target, Role)], pub targets: &'a [TargetPlan] }
pub struct GenCtx<'a> { pub input: &'a MeshInput<'a>, pub plan: &'a Plan, pub patches: &'a PatchSet, pub size: Option<&'a dyn SizeField> }
pub struct BoundaryUse { pub coedge: CoedgeKey, pub first: u32, pub segments: u32 }
pub struct BoundaryLoop { pub ring: u16, pub nodes: Vec<NodeKey>, pub uses: Vec<BoundaryUse> }
impl GenCtx<'_> {
    pub fn intervals(&self, edge: EntityId) -> Option<u32>;
    pub fn data(&self, t: Target) -> Option<&Params>;
    pub fn edge_nodes(&self, edge: EntityId) -> Option<Vec<NodeKey>>;
    pub fn coedge_nodes(&self, k: CoedgeKey) -> Option<Vec<NodeKey>>;
    pub fn side_nodes(&self, face: EntityId, side: usize) -> Option<Vec<NodeKey>>;
    pub fn loops(&self, face: EntityId) -> Result<Vec<BoundaryLoop>>;                  // G8
    pub fn grid(&self, face: EntityId) -> Option<&NodeGrid>;
    pub fn point(&self, k: NodeKey) -> Option<Point3>;
    pub fn curve_patch(&self, edge: EntityId, fractions: &[f64]) -> Result<Patch, GenerateFailure>; // G8
}
pub trait Scheme: Send + Sync {
    fn name(&self) -> &str;
    fn dim(&self) -> SchemeDim;
    fn caps(&self, params: &Params) -> Caps;
    fn supports_groups(&self) -> bool { false }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability;
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, params: &Params, out: &mut Requirements<'_>) -> Result<()> { Ok(()) }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, params: &Params, out: &mut IntervalSink<'_>) -> Result<()> { Ok(()) }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, params: &Params) -> Result<Patch, GenerateFailure>;
}
pub trait MeshTransform: Send + Sync {
    fn name(&self) -> &str;
    fn caps(&self, params: &Params) -> Caps;
    /// Rewrites `patches` (one per target, target order) in place; the driver commits all or none.
    fn apply(&self, cx: &GenCtx<'_>, targets: &[Target], params: &Params, patches: &mut [Patch]) -> Result<(), GenerateFailure>;
}
pub struct SchemeRegistry { /* BTreeMaps */ }
impl SchemeRegistry {
    pub fn new() -> Self;
    pub fn register(&mut self, s: Box<dyn Scheme>) -> Result<()>;
    pub fn register_transform(&mut self, t: Box<dyn MeshTransform>) -> Result<()>;
    pub fn get(&self, dim: SchemeDim, name: &str) -> Option<&dyn Scheme>;
    pub fn transform(&self, name: &str) -> Option<&dyn MeshTransform>;
    pub fn of_dim(&self, dim: SchemeDim) -> impl Iterator<Item = &dyn Scheme>;
    pub fn set_fallback(&mut self, dim: SchemeDim, name: &str, chain: &[&str]) -> Result<()>; // G3
    pub fn fallbacks(&self, dim: SchemeDim, name: &str) -> &[SchemeName];
    pub fn set_auto(&mut self, dim: SchemeDim, names: &[&str]) -> Result<()>;
    pub fn auto(&self, dim: SchemeDim) -> &[SchemeName];
}

// ---- solve.rs / sizing.rs / exec.rs ----
pub trait IntervalSolver: Send + Sync {
    fn solve(&self, edges: &[EntityId], constraints: &[Record<IntervalConstraint>]) -> Result<Vec<(EntityId, u32)>, Infeasible>;
}
pub trait CornerPicker: Send + Sync {
    fn pick(&self, topo: &dyn TopologyView, face: EntityId, rule: VertexRule,
            requests: &[(CornerKey, Firm<VertexType>)], ids: &[ConstraintId]) -> Result<FaceLayout>;
}
pub struct ReferenceIntervalSolver { pub max_steps: usize }
pub struct ReferenceCornerPicker;
pub struct SymMat3(pub [f64; 6]);
pub trait SizeField: Send + Sync { fn size(&self, p: Point3) -> f64; fn metric(&self, p: Point3) -> SymMat3 { /* iso */ } }
pub struct ConstantSize(pub f64);
pub fn builtin_field(spec: &SizingSpec) -> Option<Box<dyn SizeField>>;
pub trait Executor: Sync { fn run(&self, n: usize, job: &(dyn Fn(usize) + Sync)); }      // G12
pub struct Sequential;
pub struct ScopedThreads(pub usize);

// ---- copy.rs / geom.rs (free helpers) ----
pub fn derive_vertex_map(t: &dyn TopologyView, c: &CopyConstraint) -> Result<Vec<(EntityId, EntityId)>>;
pub fn edge_map(t: &dyn TopologyView, c: &CopyConstraint) -> Result<Vec<(EntityId, EntityId, bool)>>;
pub fn verify_copy(t: &dyn TopologyView, ps: &PatchSet, class: &CopyClass, m: &CopyMember, tol: f64) -> Result<()>;
// geom: vertex_point, edge_point, edge_tangent, edge_length, face_normal, corners,
//       face_loops, corner_angle, snap_to_face, volume_face_uses

// ---- error.rs ----
pub enum FailureKind { NotApplicable, Rejected, Constraint, Geometry, Quality, Unsupported, Internal }
impl FailureKind { pub const fn allows_fallback(self) -> bool; }             // all but Internal
pub enum Phase { Resolve, Requirements, Corners, Intervals, Schedule, Generate, Transform, Verify, Commit }
pub struct Attempt { pub target: Target, pub scheme: SchemeName, pub phase: Phase, pub kind: FailureKind, pub message: String }
pub struct GenerateFailure { pub target: Target, pub scheme: SchemeName, pub kind: FailureKind, pub reason: String }
pub struct Infeasible { pub reason: String, pub constraints: Vec<ConstraintId>, pub entities: Vec<EntityId>,
                        pub blame: Option<(Target, SchemeName)> }                       // D7
pub enum SchemeError {
    UnknownScheme { name: SchemeName, target: Target },
    BadParams { scheme: SchemeName, reason: String },
    NotApplicable { target: Target, scheme: SchemeName, reason: String },
    NoApplicableScheme { target: Target, tried: Vec<Attempt>, imposed_by: Vec<(Target, SchemeName)> },
    ImpositionConflict { child: Target, imposed_by: Target, required: Need, conflict: String },
    BadRequirement { by: Target, child: Target, reason: String },                       // D5
    CopyConflict { target: Target, first: ConstraintId, second: ConstraintId, reason: String },
    CopyMismatch { constraint: ConstraintId, reason: String },                          // G11
    Correspondence { source_target: Target, target: Target, reason: String },
    BoundaryLayerOnDeck { face: EntityId, boundary: EntityId },
    Schedule { jobs: Vec<Job>, constraints: Vec<ConstraintId>, reason: String },        // cycle, write conflict
    Infeasible(Infeasible), Generate(GenerateFailure),
    InvalidPatch { target: Target, reason: String }, Unclassifiable { owner: EntityId },
    Input(String), ReplanLimit { attempts: Vec<Attempt> },
    Topology(sm_topo_api::QueryError), Geometry(sm_geom_api::GeomError), Mesh(sm_mesh_api::MeshError),
}
impl SchemeError { pub fn blame(&self) -> Option<(Target, SchemeName)>; }       // what to reject on re-plan
pub type Result<T, E = SchemeError> = core::result::Result<T, E>;

// ---- driver.rs ----
pub struct DriverOptions { pub max_replans: usize, pub classify_edges: bool, pub copy_tolerance: Option<f64> }
pub struct Rejections(pub BTreeMap<(Target, SchemeName), Attempt>);           // serde
pub struct Meshed<M> { pub plan: Plan, pub patches: PatchSet, pub finished: Finished<M>, pub attempts: Vec<Attempt> }
pub struct Driver<'r> {
    pub registry: &'r SchemeRegistry, pub solver: &'r dyn IntervalSolver,
    pub picker: &'r dyn CornerPicker, pub size: Option<&'r dyn SizeField>, pub options: DriverOptions,
}
impl<'r> Driver<'r> {
    pub fn new(registry: &'r SchemeRegistry, solver: &'r dyn IntervalSolver, picker: &'r dyn CornerPicker) -> Self;
    pub fn plan(&self, input: &MeshInput<'_>, intent: &MeshIntent) -> Result<Plan>;
    pub fn plan_with(&self, input: &MeshInput<'_>, intent: &MeshIntent, rej: &Rejections) -> Result<Plan>;
    pub fn generate(&self, input: &MeshInput<'_>, plan: &Plan, exec: &dyn Executor) -> Result<PatchSet>;
    pub fn commit<B: MeshBuild>(&self, input: &MeshInput<'_>, ps: &PatchSet, b: B, alloc: &mut SerialAlloc) -> Result<Finished<B::Output>>;
    /// plan → generate → verify → commit; on `err.blame()` reject the pair and re-plan.
    pub fn mesh<B: MeshBuild>(&self, input: &MeshInput<'_>, intent: &MeshIntent, exec: &dyn Executor, b: B, alloc: &mut SerialAlloc) -> Result<Meshed<B::Output>>;
    pub fn mesh_with<B: MeshBuild>(&self, input: &MeshInput<'_>, intent: &MeshIntent, rej: &Rejections, exec: &dyn Executor, b: B, alloc: &mut SerialAlloc) -> Result<Meshed<B::Output>>;
}
```

Contract notes that go into DESIGN.md:
- **Resolution order per target:**
  1. claim → `Claimed`;
  2. copy class member → `Copy`;
  3. intent chain;
  4. registry fallbacks;
  5. `PieceShape`;
  6. imposed set or auto. Ties go to the higher score, then auto-list order, then name.

  Every candidate evaluated becomes an `Attempt`.
- **Planning order:**
  1. region and deck constraints;
  2. volumes, then groups;
  3. closure;
  4. faces;
  5. closure;
  6. curves;
  7. closure;
  8. copy classes;
  9. caps check on copy members;
  10. corners;
  11. sizing and scheme intervals, then solve;
  12. footprints and waves.
- **Generate:** each wave runs on the executor; patches are checked and merged in `Job` order. Then transforms run in their waves, then copy verification, then commit. Commit takes nodes in `PatchSet` order, then cells, then face and edge classes, then `finish`. `SerialAlloc` is unchanged on any error.
- **Determinism:** unchanged from the base, plus copy-class roots and blame choices follow the fixed rules in D2 and D7. Footprint levelling breaks ties in `Job` order. The test matrix keeps plan JSON plus `topology_hash`/`geometry_hash` across `Sequential`, `ScopedThreads(4)` and concurrent threads, and adds the deck × sweep rows and a many-to-one sweep.

## 6. Open questions and recommended answers

1. **Cut faces of pieces (`PieceFace::Cut`/`CutPart`).** *Answer:* the contract does not change. `PieceResolver` is the seam, and `WholeFaces` keeps refusing cut faces with a typed error. An overlay resolver arrives with the decomposition overlay. Two pieces are blocked on other crates: classifying onto `PartitionFace`/`PartitionEdge`/`PartitionVertex`, and realizing cuts as topology (see the API change requests).
2. **Submap side directions.** *Answer:* driver-owned, in `FaceLayout`. `LogicalSide::dir` is computed by the picker from cumulative turns (End +1, Side 0, Corner −1, Reversal −2, mod 4). Every submap, triprimitive and sweep crate reads the same layout. Many-to-one sweep layer surfaces stay scheme-private (`TargetPlan::data`).
3. **Rotational sweeps, morph copies, volume copies.** *Answer:* copies stay rigid (one `sm_core::Transform`; rotation is already one). Morph or least-squares smoothing is a sweep generation detail, not a copy. Volume copies (lattice fills) belong to model-layer instancing plus sm-mesh-api flatten, not to schemes, so they stay out of v1.
4. **Re-plan granularity.** *Answer:* keep the full deterministic re-plan. Planning is cheap next to generation, and `plan_with(rejections)` already gives the API an incremental re-solve could reuse later without a break.
5. **Boundary layers.**
   - *Answer:* the intent becomes `MeshConstraint::BoundaryLayer { face, volume, spec }`.
   - Only a volume scheme with `Caps::BOUNDARY_LAYER` may honour it. Otherwise the result is a soft rejection (`Attempt`) or, for a Hard intent, `NotApplicable`.
   - A deck's Hard `Forbid(BOUNDARY_LAYER)` wins with `BoundaryLayerOnDeck`.
   - No boundary-layer scheme ships in W1.
6. **Deck pair where B is the sweep source.** *Answer:* copy equivalence classes (D2). The root is the face a surface scheme meshes (the sweep source B), and A is a copy of B under T⁻¹. When A is instead claimed by a volume, the deck relation becomes `CopyMode::Verify`. With footprint scheduling, any face that copies from a claimed face waits for the claimer. Checked against the levelling rules: no cycle remains in either bad row.
7. **Capabilities or names in impositions?** *Answer:* capabilities by default (`require_caps`/`forbid`). Names remain for "exactly this algorithm". A scheme crate must never list sibling scheme names to express a property.
8. **Params value model and fallback replay.** Decided in §3: `serde_json::Value` checked through the op codec; `Rejections` stored in the `Plan`, not in the intent.
9. **Surface-only meshes.** `sm-mesh-api` faces need cells, so a 2D mesh cannot be committed today. *Answer:* out of scope for sm-scheme-api v1. If a 2D target appears in `docs/PARITY.md` scope, it is an sm-mesh-api change (a 2D cell mode), and the patch contract already carries surface elements.
10. **A rayon executor.** *Answer:* lives in the mesher crate and implements the object-safe `Executor` (decisions W0c 8: no rayon in API crates).

### API change requests (non-blocking for W1 start)

- **sm-mesh-api:** `Classification::dim` should accept `PartitionFace` (2), `PartitionEdge` (1) and `PartitionVertex` (0). Today `classify.rs:66-74` accepts only `PartitionCell`, although the sm-core kinds exist since W0c. This is needed to classify mesh entities on virtual cuts.
- **sm-model-api / sm-topo-api:** a way to realize a piece's cut faces as topology ids, so a resolver can return them. Proposed: `Decomposition::piece_face_cells(&self, pf: &PieceFace, topo: &dyn TopologyView) -> Option<Vec<EntityId>>`, backed by an overlay `Model::realize_cut(cut: CutId, faces: &[EntityId])`.
- **sm-topo-api / sm-model-api:** an optional `schemars` feature deriving `JsonSchema` on `CoedgeKey`, `CornerKey`, `Side`, `PieceRef`, `PieceFace`, `CutId` and `Keep`, so that `MeshIntent` and `Plan` can export schemas for `TypedOp`/MCP. Neither crate has the feature today.
