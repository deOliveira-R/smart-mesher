# sm-scheme-api: design (panel candidate `constraint`)

The contract between the meshing driver and every meshing scheme (curve, surface, volume) and the sizing layer. **Approach: constraint system first.** Schemes are mostly declarative. Before any mesh exists, each scheme contributes constraints to one global `MeshingProblem`: vertex types, interval equations, child-scheme impositions, roles, and copy/deck equalities. The driver solves the whole problem (corner picking, then interval matching) into a serializable `Plan`. Generation is then a pure function of the plan: every scheme returns a `Patch`, and patches are committed in a fixed order into `sm-mesh-api`'s `MeshBuild`.

Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-model-api`, `sm-mesh-api` (all with `serde`), plus `serde`, `serde_json`, `thiserror` and `smallvec`. No rayon (decisions.md W0c 8): parallelism goes through the `Executor` trait, and a rayon executor belongs in the mesher crate.

## Positioning

> Like **Cubit's scheme + interval-assignment (IIA) model**, it has per-entity schemes with firmness, auto selection, vertex types per surface, and a global integer interval solve. Like **Gmsh's transfinite/recombine pipeline**, constraints (transfinite curves/surfaces, periodic copies) are declared first and structured meshes are generated from them. It differs in four ways:
> - **(A) One explicit, inspectable constraint store.** Every constraint has an id and an origin (intent, sizing, scheme + target, deck boundary, piece hint, driver), and the whole store is serialized in the `Plan`. In Cubit these constraints are implicit state in the session; in Gmsh they are side effects of API calls.
> - **(B) Impositions are data.** A parent scheme imposes allowed schemes and roles on its children (map volume → map surfaces; sweep → source/target/linking). A conflict with a hard user choice is a typed error that names both sides.
> - **(C) Generation is pure and runs in parallel.** Generated nodes have canonical names (`NodeKey`), so any wave can run concurrently, and commit in key order gives bit-identical meshes on 1 or N threads. Cubit and Gmsh mesh entity by entity and write straight into the mesh.
> - **(D) Model-layer semantics feed the solve directly.** Deck pairs (periodic, rotational) become copy constraints with interval equalities. Deck faces get `NoBoundaryLayer`, and mirror faces get `PlaneLock`. Decomposition pieces (`PieceRef`, `PieceShape::Sweep`) are first-class volume targets.
>
> This gives reproducible meshes (the plan and the mesh are functions of the intent and the model, so they can live in an Op and in git), explainable failures (an infeasibility lists its constraints and entities), safe fallback (re-plan without the failed scheme), and exact periodic or mirrored meshes by construction.

## Pipeline

```
MeshIntent (serde) + MeshInput {TopologyView, Model, PieceResolver, selection}
  │ Driver::plan
  ├─ region: pieces → faces (resolver) → edges → vertices          (ascending ids)
  ├─ deck constraints from Model::face_kind (copy / NoBoundaryLayer / PlaneLock)
  ├─ volumes, then groups: resolve scheme → Scheme::requirements    (impose, role, copy, rules)
  ├─ user copies; copy closure (vertex maps, edge copies, interval equalities, conflicts)
  ├─ faces, then curves: resolve (impositions ∩, intent chain, auto) → requirements
  ├─ CornerPicker per face with a VertexRule → FaceLayout (types + logical sides)
  ├─ sizing/intent goals + Scheme::interval_constraints → IntervalSolver
  └─ waves (dimension, then copy/precedence depth)                  → Plan (serde)
  │ Driver::generate (Executor per wave; merge in target order)    → PatchSet
  │ Driver::commit (nodes by key → cells by target → face/edge classes → finish)
  └ Driver::mesh = plan → generate → commit, re-planning on fallback failures
```

## The brief, item by item

| Brief item | Types |
|---|---|
| Scheme catalogue (curve: equal/bias/dualbias/curvature/stretch/pinpoint/copy; surface: map/submap/pave/tri*/circle/hole/polyhedron/copy/mirror; volume: map/submap/sweep 1-1, many-1, many-many, rotational/tetmesh/copy/polyhedron) | One object-safe `Scheme` trait (`name`, `dim: SchemeDim`, `applicability`, `requirements`, `interval_constraints`, `generate`), registered in `SchemeRegistry` by `(dim, name)`. Parameters are `Params` (a JSON object, the same value model as `sm-op-api`). `copy` is reserved and executed by the driver at curve and surface level (user copy, mirror = copy with a reflection, deck copies, sweep targets). Conversions (thex/htet/qtri) are `MeshTransform`s over a group patch. |
| Per-surface vertex types End/Side/Corner/Reversal and corner picking | `VertexType` (`element_count`, `nominal_angle`, `submap_weight`) is keyed by `sm_topo_api::CornerKey` (face + incoming coedge), never by the vertex. A request is a `Firm<VertexType>` (hard/soft/default). Schemes ask for a `VertexRule` (`Map`, `Submap`, `Ends(n)`, `Free`). Corner picking is a shared service (`CornerPicker`; reference: angle-based) and returns a `FaceLayout` with `CornerAssignment`s and `LogicalSide`s (coedge runs; a side may span several curves). |
| Interval constraints: hard/soft counts, map/submap side sums, even sum for pave, sweep source/target matching, a global solve | `IntervalConstraint::{Fixed{firmness}, Goal, Range, SumEqual, EvenSum}` (with multiplicity), written through `IntervalSink` after corners are solved. Sizing and intent add `Goal`/`Fixed` (requests cascade volume → face → edge). Copies add single-edge `SumEqual`. One `IntervalSolver` call covers the whole region; the reference solver uses union-find classes named by the lowest edge id, then cheapest-step repair. |
| Schemes imposing child schemes/constraints (map volume forces map surfaces; sweep forces source/target/linking roles) | `Requirements::impose(child, allowed)`, `role(child, Role::{SweepSource, SweepTarget, SweepLinking})`, `copy(..)`, `vertex_rule`, `vertex_type`, `constraint(Precedes/PlaneLock/...)`. Resolution intersects every imposition. A hard intent outside the set gives `ImpositionConflict`; a soft one is overridden and recorded in `TargetPlan::rejected`. |
| Fallback chains and auto selection (applicability with score/reason) | `EntityIntent { scheme, fallbacks }` is tried in order, then the `PieceShape` hint, then the imposed set or the `AutoPolicy` candidates by `Applicability::Applicable{score, reason}` (ties: policy order, then name). `Selection` records why a scheme won. A generation failure with `FailureKind::Fallback` makes `Driver::mesh` reject `(target, scheme)` (`Rejections`) and **re-plan**, because a different scheme changes the constraints. |
| Atomic multi-volume transforms (multisweep, merged volumes meshed together, all-or-nothing commit) | `MeshGroup { members, scheme, conversion }` becomes `Target::Group` plus `MeshConstraint::Atomic`. The scheme must `supports_groups()`, produces one patch (`Patch::owners` per cell), and the optional `MeshTransform` rewrites it. Commit is all-or-nothing by construction: nothing is written before every patch exists, and `MeshBuild::finish` leaves `SerialAlloc` untouched on error. |
| Deck copy constraints (periodic/mirror faces get identical meshes; no boundary layers on deck faces; ir.md §5.1) | For each `FaceKindView::Deck` face the driver adds `NoBoundaryLayer`. A translation/rotation A side gets `CopyConstraint { source A, target B, transform: deck.transform(), reason: Deck }`, and the copy closure adds the edge copies and `SumEqual`s. A mirror face gets `PlaneLock`. A boundary-layer intent on a deck face is `BoundaryLayerOnDeck`. The copy generator maps keys through the vertex/edge correspondence and reverses loops when `det(M)·M n_s · n_t < 0`. |
| Meshing Decomposition pieces, not only raw topology | Volume targets are pieces: a `TopoVolume` id = `PieceRef::Whole`, a `PartitionCell` id = `PieceRef::Cell` (`Target::piece`, `piece_of`). `PieceResolver` maps pieces to faces of the view and node owners to classification entities. The default `WholeFaces` handles whole volumes and cells bounded by whole faces. `PieceShape::Sweep{source, target}` / `Map` become `Selection::PieceShape` choices. Cells are classified on the `PartitionCell`. |
| Inputs: TopologyView + geometry queries + Model/Decomposition | `MeshInput { topo: &dyn TopologyView, model: &Model, pieces: &dyn PieceResolver, selection }`. `geom` has the shared queries (vertex points, edge evaluation in the edge's own direction, lengths, oriented face normals, corner angles). |
| Output through `MeshBuild` with classification onto model/decomposition entities | `Driver::commit<B: MeshBuild>`: vertices are added in `NodeKey` order and classified on their owner (`Param::Curve`/`Surface` kept when it fits). Cells are classified on their piece, surface elements on their face (via `find_face`), and curve segments on their edge (`classify_edges`). |
| Deterministic, parallel, serializable intent, typed errors with a structured failure | See below. `MeshIntent`, `Plan` and `MeshingProblem` are serde (ids as `Kind#serial`). `SchemeError` uses `thiserror`, with `Infeasible { reason, constraints, entities }` and `GenerateFailure { target, scheme, kind, reason }`. |
| Sizing layer | `SizeField` (`size`, `metric` → `SymMat3`), `SizingSpec::{Constant, Function{name, params}}`, `ConstantSize`, `builtin_field`. The driver turns sizes into `Goal = length / size(midpoint)`. Unstructured schemes read `GenCtx::size`. |

## Contracts schemes rely on

- **Node keys.** `NodeKey { owner, index }`. A topology vertex is `(v, 0)`. An edge with `n` intervals owns `(e, 1..n)` **in the edge's own direction**, so `GenCtx::edge_nodes`/`coedge_nodes`/`side_nodes` give every boundary node without asking the curve scheme. Faces and volumes number their own nodes. **A copy keeps the source index**: `(S, k)` maps to `(T, k)`, which the sweep stub uses to find the top of each column.
- **Patch.** `Patch { target, nodes, elements (CSR: shapes + offsets + keys), grid, owners }`. Surface elements follow the face normal, and cells use the `sm-mesh-api` template order. `check_patch` enforces that nodes are owned by the target, that a curve has exactly indices `1..n`, that keys are unique, that every referenced node is known, and that parameters fit their owner.
- **`NodeGrid`** (map/submap): row `j = 0` runs along logical side 0, column `ni-1` along side 1, and row `nj-1` / column `0` along sides 2/3 reversed. `line_from(boundary key)` walks across the grid in any orientation; the sweep uses it on linking faces, including faces that are deck copies, since copies keep the grid.
- **Purity.** `generate` reads only `GenCtx` (input, plan, earlier waves, size field) and returns its patch.

## Determinism and parallelism

- Every map is a `BTreeMap`/`BTreeSet` or a sorted `Vec`. Targets, faces, edges and vertices are processed in ascending id. Constraint ids follow contribution order, which is itself deterministic. Plan tables are sorted vectors (`binary_search` lookups), so the JSON form is canonical.
- Union-find classes in the solver are named by the lowest edge id, never by the root. Repair ties break by class index, then by step sign.
- **Waves.** Targets of one wave are independent: same dimension, same copy/precedence depth. The `Executor` runs `f(i)` for the wave and returns results **in index order**. The driver checks each patch and merges it in target order. The first error in target order is reported, so error reporting is deterministic too.
- **Commit** is sequential and keyed: mesh vertex numbering is the sorted `NodeKey` order, independent of which thread produced what. Ids come from `SerialAlloc` in `finish`.
- `tests/determinism.rs` compares `topology_hash`, `geometry_hash` and the plan JSON across two sequential runs, `ScopedThreads(4)`, and four concurrent `std::thread` runs.

## Reference implementations and stubs

In the crate: `ReferenceIntervalSolver`, `ReferenceCornerPicker`, `WholeFaces`, `Sequential`, `ScopedThreads`, `ConstantSize`, and the copy generator. In `tests/stub_schemes`: `equal` (curve, uniform in parameter), `map` (`VertexRule::Map`, opposite-side `SumEqual`s, Coons interior projected on the surface, `NodeGrid`) and `sweep` (1-to-1; one helper resolves source/target from params, the piece hint or the lowest opposite pair; imposes map/submap on linking faces; assigns roles; copies source → target; equalizes linking edges; builds hex/wedge columns with an orientation test).

Tests, on the reference `CellComplex` brick written into `ReferenceBuilder`:
- 2×3×4 sweep: interval propagation, roles, impositions, copy, four End corners per face; 24 cells, `validate` clean, each cell volume 0.125 and the total 3.0 to 1e-12, per-`TopoFace` face counts 12/12/8/8/6/6, every vertex on the 0.5 grid.
- Auto selection of the sweep.
- Hard pave on a linking face → `ImpositionConflict`; a soft one is overridden.
- Conflicting hard counts → `Infeasible` naming both edges.
- Solver sums and parity, including an infeasible odd loop.
- A fallback re-plan away from an always-failing high-score scheme.
- A translation deck pair meshed as an exact copy, plus `BoundaryLayerOnDeck`.
- Intent and plan serde round trips.
- Determinism.

## Deviations and limits (honest list)

- `Cargo.lock` is not committed (the orchestrator owns it; adding this member adds one `[[package]]` entry).
- `serde` is a hard dependency (intent has to be serializable). There is no `schemars` feature, because `CornerKey`, `PieceRef` and `PieceFace` have no `JsonSchema` (see the API requests).
- The reference interval solver is not an optimal IIA: it is greedy repair with a step limit. It is exact for the structured equations used here. An ILP or network-flow solver can plug in behind `IntervalSolver`.
- Copy generation covers curves and surfaces. Volume copies and polyhedral patch cells (which need face lists) are not in the patch contract yet. Copied nodes drop their parameter (`Param::None`).
- The face-copy vertex correspondence is geometric (`transform(p)` within the summed vertex tolerances) unless `vertex_map` is given. A closed edge matches by its ends only.
- A planar surface mesh without cells cannot be committed: `sm-mesh-api` faces need cells. Surface-only meshing is reported as `InvalidPatch`.
- `Elements::get` and `NodeGrid::at` panic out of range (a documented caller contract). `NodeGrid::line_from` is a linear search.

## Open questions

1. **Cut faces of pieces.** `PieceFace::Cut`/`CutPart` have only a model-local `CutId`. Mesh nodes on a virtual cut need an `EntityId` to classify onto (or a `Classification` variant), and the topology overlay must realize the cut as faces. A resolver for this needs the decomposition overlay. `WholeFaces` refuses such pieces with a clear error.
2. **Submap/many-to-one sweep data.** Should `FaceLayout` carry submap side *directions* (±i/±j per side) for the scheme, or should submap compute them? Many-to-one sweeps need source-to-target layer surfaces, which are a separate derived structure.
3. **Rotational sweeps and least-squares transforms.** The copy constraint carries one `Transform`. Morphing copies (Cubit's least-squares / morph smoothing) would need a `Placement` enum instead.
4. **Re-plan granularity.** A fallback re-plans everything. That is deterministic but repeats corner and interval solves; an incremental re-solve could reuse unaffected classes.
5. **Boundary layers.** The intent type exists and decks forbid it, but there is no boundary-layer constraint or scheme yet (Cubit's boundary-layer page 404'd in the research).

## API change requests (non-blocking)

- `sm-model-api` and `sm-topo-api`: a `schemars` feature deriving `JsonSchema` on `PieceRef`, `PieceFace`, `CutId`, `Keep`, `CoedgeKey`, `CornerKey`, `Side`. This lets `MeshIntent`/`Plan` derive it for `TypedOp` and MCP schemas.
- `sm-mesh-api`: classification onto decomposition faces/edges (`PartitionFace`/`PartitionEdge` in `Classification::dim`, or a `CutFace` entity), so mesh entities on virtual cuts can be classified.
- `sm-model-api` / `sm-topo-api`: a way to realize a decomposition piece as faces of a `TopologyView` (overlay operations for virtual volume splits), so `PieceResolver::faces` works for cut pieces.
