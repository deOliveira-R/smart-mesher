# sm-scheme-api: design (panel candidate `pipeline`)

The contract between the meshing driver and every meshing scheme, plus the sizing hooks.
Approach: a **plan/apply pipeline with capabilities**. Depends on `sm-core`, `sm-geom-api`,
`sm-topo-api`, `sm-model-api`, `sm-mesh-api` (all with `serde`), `serde`, `smallvec` and
`thiserror`. There is no rayon (W0c decision 8): parallelism is a property of the driver, behind
an `Executor` trait.

## 1. Positioning

> Similar to **Cubit's scheme / interval-assignment model** (per-entity schemes with
> firmness, surface vertex types, a global interval solve before meshing, auto selection, sweep
> source/target/linking roles, fallback chains) and to **Gmsh's transfinite / recombine
> pipeline** (constraints are declared on entities, propagated to curves, then the mesh is
> generated bottom-up 1D, 2D, 3D). It differs in:
> - **(A) Explicit phases.** Every scheme is `applicability -> plan -> apply`. `plan` runs
>   top-down and returns data (`Plan`): requirements on children, claimed children, ordering,
>   interval constraints, vertex types. `apply` runs bottom-up and returns a local patch. Cubit
>   and Gmsh mix these steps inside each mesher.
> - **(B) Capabilities, not names.** Parents require properties of child meshes (`Caps::QUAD |
>   Caps::STRUCTURED`) instead of scheme lists, so a deck copy of a mapped face satisfies a
>   sweep, and a new structured scheme needs no edits elsewhere.
> - **(C) Topology-aware decks.** Periodic/rotational pairs from the model layer become
>   `CopyOf` requirements, interval equalities per curve pair and `Forbid(BOUNDARY_LAYER)`.
>   The driver forces the built-in copy schemes and checks the result node by node.
>   (Gmsh `Periodic` is a per-entity mesh constraint with no link to physics boundary kinds.)
> - **(D) Deterministic parallel commit.** Patches reference committed nodes plus their own new
>   nodes, and they are committed in job order, so the numbering is identical for any thread
>   count.
> - **(E) Serializable intent and plans.** Scheme choice, parameters, sizes, vertex types and
>   whole plans are serde data. They can live in an op and in git, and a plan is a "mesh
>   preview" you can diff.
> - **(F) Pieces as first-class targets.** `ShapeKey` names topology cells, decomposition
>   pieces and piece faces, and `PieceShape::Sweep` feeds sweep roles directly.
>
> This gives: schemes that can be tested on a synthetic `ShapeView` without a kernel;
> requirement conflicts reported with both sources before any meshing; fallback that is always
> consistent (re-plan rather than patching a half-meshed model); and bit-identical meshes on 1
> or N threads.

## 2. The pipeline (`Driver::run`)

1. **Closure** of the targets through `ShapeView` (volume -> face uses -> loops -> curves ->
   vertices).
2. **Decks** (`ShapeView::deck_links` from `Model::deck_faces`). For a pair `B = T(A)`:
   `CopyOf{A, T}` on B, `Forbid(BOUNDARY_LAYER)` on A and B, and for every curve correspondence
   (`ShapeView::corresponding_curves`) an `Equal` interval constraint plus `CopyOf` on the
   B-curve. For a mirror: `Forbid(BOUNDARY_LAYER)` and `OnPlane` (a smoothing constraint).
3. **Plan top-down**: atomic groups (`MeshIntent::groups`), then dimension 3, 2, 1, keys
   ascending, copies after their sources. For each entity: candidates
   (`SchemeRegistry::candidates`: the explicit chain, each entry followed by its registry fallback
   chain, then auto unless `Hard`); hard requirement filter (`Caps`, `Forbid`, `Scheme`;
   `INHERIT` is resolved against the copy source's job); the `applicability` gate (auto
   candidates are ranked by score, with soft-requirement penalties); then `plan`. An accepted
   plan owns its targets and claims, pushes its `requires` onto the children and its interval
   constraints into the global problem. Every rejection is kept as an `Attempt`.
4. **Sizes**: each curve gets its own `SizeSpec`, else one from a parent face, else from a
   parent volume, else the default (`Goal = length / size`, or a `Count` with firmness), plus
   `Min 1`.
5. **Solve** once with the `IntervalSolver` (production: `sm-sizing` IIA; reference:
   `SimpleSolver`).
6. **Apply bottom-up** (dimension 1, 2, 3). Inside a dimension, jobs are levelled by
   `Plan::after`. Each level runs on the `Executor`; the jobs read only the committed mesh. The
   level is committed in job order. Every commit validates the whole patch first, so it is
   all-or-nothing. After dimension 3, every `CopyOf` face is verified against its source.
7. **Fallback**: an `apply` error with `allows_fallback()` (everything except `Internal`)
   excludes `(entity, scheme)` and **re-plans from scratch**, bounded by `max_replans`.
8. **Emit** (`emit`): nodes with classification (the `Param` is kept when it fits the entity
   kind), cells classified on their volume or piece, and each new face classified on the
   topology face whose patch element it matches. Interior faces are left to `MeshBuild::finish`.
   Nothing reaches the builder until all of the above has succeeded, and `finish` leaves the
   `SerialAlloc` unchanged on error.

`MeshTransform` (THex, QTri, HTet) runs on an existing `CommittedMesh` through
`Driver::transform`. It replaces the cells of all its targets and any listed surfaces in one
commit (`CommittedMesh::apply_transform`). A volume job that is only partly covered is refused.

## 3. Brief -> types

| Brief item | Where |
|---|---|
| Scheme list (curve / surface / volume) | `CurveScheme`, `SurfaceScheme`, `VolumeScheme` (shared `SchemeCore`); conversions are `MeshTransform`; copy is built in at dimensions 1 and 2 (`CurveCopy`, `SurfaceCopy`) |
| Sizing layer | `SizeSpec` (intent), `IntervalConstraint::{Count, Goal, Min}` from sizes, the `SizeField` trait (`size_at`, `metric_at`), `ConstantSize` |
| Vertex types and corner picking | `VertexType{End, Side, Corner, Reversal}` keyed by `sm_topo_api::CornerKey` (a vertex use in one face), `CornerAssignment{firmness}`, the `CornerPicker` service trait (`AnglePicker` reference), `ShapeView::corner_angle`, `interior_angle`; plans report the types they use (`Plan::vertex_types`) |
| Hard/soft counts, map/submap side sums, pave even sum, sweep matching | `IntervalConstraint::{Count{firmness}, Linear (sides_equal), Even, Min, Goal}` with `SourcedConstraint{by, scheme, why}`; one `IntervalProblem` per run; `IntervalSolver` |
| Schemes imposing on children | `Plan::requires: Vec<ChildRequirement{target, req, by}>`, with `Requirement::{Caps, Forbid, Scheme, Role, CopyOf, OnPlane}`. A map volume would require `Caps{QUAD\|STRUCTURED, Hard}` on its faces; the sweep stub requires `Role(Source)` + `QUAD` on the source and `Role(Linking)` + `QUAD\|STRUCTURED` on linking faces |
| Volume scheme meshes its own target | `Plan::claims` (not scheduled separately) + `VolumePatch::surfaces: Vec<ClaimedSurface>` |
| Fallback chains and auto selection | `SchemeRegistry::{set_fallback, set_auto, candidates}`, `Applicability{score, reason}`, `SchemeChoice::{Auto, Chain{firmness}}`, re-plan on `SchemeError::allows_fallback`, `Attempt{phase, kind}` history |
| Atomic multi-volume (multisweep, merged volumes) | `GroupIntent{targets, scheme}` -> one plan with `targets.len() > 1`, one job, one `VolumePatch` with `CellList::target` per cell, one commit; transforms through `MeshTransform` + `apply_transform` |
| Deck copy constraints, no boundary layers on deck faces | Section 2 step 2; `Caps::BOUNDARY_LAYER`; `DeckLink`; `verify_copy`; proper vs improper transforms handled in `SurfaceCopy` (loops reversed and the grid transposed for proper transforms, by the sign rule `n_B = -g(n_A)`) |
| Pieces from the Decomposition layer | `ShapeKey::{Piece(PieceRef), PieceFace(PieceFace)}` with `canonical()` (`Whole` = the cell); `TopoShape` reads `Piece` rows (whole-face pieces) and `PieceShape::Sweep` -> `ShapeView::sweep_hint`; cells classify on the `PartitionCell` |
| Inputs | `ShapeView` (object safe), implemented by `TopoShape` over `&dyn TopologyView` (complex or overlay) + its `GeomStore` + optional `&Model` |
| Output | `CommittedMesh` (scratch) -> `emit<B: MeshBuild>` with classification onto model / partition entities |
| Determinism / parallelism | Section 4 |
| Serializable intent | `MeshIntent{entities: Vec<(ShapeKey, EntityIntent)>, groups, default_size}`, `SchemeSpec{scheme, params}`, `Params` = an ordered tree of TOML-safe `ParamValue`s (`check()` rejects non-finite floats); `Plan`, `IntervalProblem` and `Attempt` are serde too |
| Typed errors with a structured failure | `SchemeError` + `FailureKind` (`allows_fallback`), `DriveError::{NoScheme{attempts}, Conflict, Intervals, Failed, Cycle, BadPatch, Mesh, TooManyReplans, Registry}`, `IntervalError` |

## 4. Determinism and parallelism

- Every map is a `BTreeMap` / `BTreeSet`, and `ShapeKey: Ord` is the processing order. Candidate
  lists keep their declared order (score ties are broken by list position). Registration order
  never matters.
- `apply` is a pure function of `(plan, committed mesh, intervals, shape)`. Jobs at one level
  only read. Patches number their new nodes locally (`PatchRef::New`), and the commit assigns
  `NodeId`s in job order. The global numbering is therefore a function of the plan only.
- `Executor::map` returns results in index order. `Sequential` and `Threads(n)` (scoped `std`
  threads) ship here; `sm-engine` can add a rayon executor without changing output.
- Interval solvers must be deterministic; `SimpleSolver` is union-find plus an ordered greedy
  repair.
- `tests/determinism.rs`: two runs, `Sequential` vs `Threads(1|4|64)`, and four concurrent runs
  all give equal `CommittedMesh::fingerprint`, `topology_hash` and `geometry_hash`.
- Granularity: curves are all in one level; faces wait only for their copy sources; volumes are
  independent unless grouped. Fallback re-plans serially (rare path).

## 5. What the stub test shows (`tests/`)

`common/mod.rs` has `curve.equal`, `surface.map` (corner picking through the picker, 4 `End`
vertices, two `sides_equal` constraints, a structured `Grid`, TFI interior projected onto the
surface), `volume.sweep` (1-to-1: roles from params, then the piece hint, then the first valid
pair; it claims the target, requires the source and linking capabilities, adds linking-curve
equalities, reads layer columns from the linking grids and translates the interior), and
`test.fail`. These run against `sm_topo_api::fixtures::brick` and write into
`sm_mesh_api::reference::ReferenceBuilder`; `validate()` is empty. Checked: 48 hexes from size
0.25 plus a hard 3 on one z-edge (propagated by the sweep equalities), total volume 1, 80
boundary faces on `TopoFace`s, `Param::Curve` on edge nodes, auto selection, fallback after an
`apply` failure, a hard vertex type disabling map, a hard interval conflict, a periodic deck
forcing `surface.copy` / `curve.copy` (the copy still satisfies the sweep's `STRUCTURED`
requirement, and nodes match under the translation), a mirror getting `OnPlane`, a `Whole`
piece target, JSON round trips, all-or-nothing transforms, and the reference solver on sums
and parity.

## 6. Deviations and limits (honest list)

- `serde` is a **required** dependency, not a feature: the brief requires the intent to be
  serializable, so the sibling crates' `serde` features are switched on. `schemars` is not
  derived, because `sm_topo_api::CornerKey` has no `JsonSchema`.
- Fallback **re-plans from scratch** instead of re-using committed lower-dimensional meshes.
  This is correct by construction but costs a full re-mesh per fallback. Re-using unchanged
  jobs, keyed by plan hash, is an optimization for the real driver.
- `CellList` holds template shapes only (Tet, Hex, Wedge, Pyramid). Polyhedral cells (the
  polyhedron scheme, tet dual) need a face-loop CSR in `VolumePatch` and `add_polyhedron` in
  `emit`; this is the next addition.
- `SurfaceCopy` matches boundary nodes by a linear scan, O(boundary^2), and `verify_copy` by a
  sorted sweep. Both are fine for a reference, not for very large faces.
- `TopoShape` handles pieces whose faces are all `PieceFace::Whole`. Cut and partial piece
  faces return `Unsupported` until the model and the overlay share ids (API request 2).
  Composite pieces are not handled either (a model follow-up is already tracked).
- `SimpleSolver` is not an optimizer: it only raises counts, and it handles sums and parity
  greedily. The real IIA belongs in `sm-sizing`.
- Mesh output is 3D only, because `sm-mesh-api` models 3D cells and faces with at least 3
  vertices. Curve-only and surface-only runs fill `CommittedMesh`, but `emit` writes only cells
  and their faces.
- `Cargo.lock` gains this package's entry. The orchestrator owns the lock (Q3) and should
  regenerate it.
- `sm-testkit` does not exist yet, so its property tests are not used.

## 7. Open questions

1. **Corners mid-curve** (submap, Cubit "a corner may lie inside a curve"): should a vertex type
   be allowed at a curve parameter (an `IntervalConstraint` split into two variables), or should
   the overlay always partition the edge first (`Overlay::partition_edge`)? The second option
   fits the current key model.
2. **Multi-curve linking paths** in many-to-one / many-to-many sweeps need per-path sums. The
   `Linear` constraint can express them; the open question is whether the sweep or a shared
   `sm-sizing` helper should derive them.
3. **Extensible capabilities**: `Caps` is a closed `u32` bit set. Should plugins be able to
   register named capabilities?
4. **Boundary layers**: a BL is an additional pass over a face set (a transform) or a
   `Requirement` on a face. Deck faces forbid it either way. This depends on the boundary-layer
   research (the Cubit page returned 404).
5. **Quality gate**: should the driver run `sm-quality` on each patch and turn violations into
   `FailureKind::Quality` (which triggers fallback), or should schemes do it themselves?
6. **Where `OnPlane` and other smoothing constraints live** once `sm-smooth` exists. They need
   to be carried from the plan to the committed mesh (per-node constraint table).

## 8. API change requests (frozen crates)

1. **sm-mesh-api**: `Classification::dim` / `is_valid` accept `PartitionCell` but not
   `PartitionFace` / `PartitionEdge` / `PartitionVertex`. Mesh entities on an overlay's virtual
   faces, edges and vertices (pieces meshed through `sm_topo_api::Overlay`) therefore cannot be
   classified on them. A face on a cut between two pieces can only be classified on the parent
   `TopoVolume`, and the cut's identity is lost (its two cells sit on different
   `PartitionCell`s, so `finish` cannot derive it either). Proposed:
   `PartitionVertex => Some(0)`, `PartitionEdge => Some(1)`, `PartitionFace => Some(2)` in
   `Classification::dim`, with `Param::None` the only fitting parameter.
2. **sm-model-api**: piece faces `PieceFace::{Part, Cut, CutPart}` have no link to topology
   (the `CutId` is local and no `PartitionFace` is minted), so a driver cannot find their loops
   or geometry. Proposed: when a split is realized in the overlay, record the virtual ids,
   `Model::realize_cut(&mut self, cut: CutId, faces: &[EntityId]) -> Result<()>` plus
   `Decomposition::piece_face_cells(&self, pf: &PieceFace) -> Option<&[EntityId]>` (overlay
   `PartitionFace` / `TopoFace` ids). `TopoShape` then maps piece faces to overlay faces.
