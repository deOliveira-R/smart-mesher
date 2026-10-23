# sm-scheme-api: final design

The contract between the meshing driver and every meshing scheme (curve, surface, volume) and the sizing layer. This is the frozen W0 contract. It is the `constraint` panel candidate (`ddd5598`) with the grafts G1 to G12 and the fixes D1 to D13 of `docs/design/w0/scheme-recommendation.md` applied.

**Approach: constraint system first.** Schemes are mostly declarative. Before any mesh exists, each scheme contributes constraints to one global `MeshingProblem`: capability impositions on children, claims, roles, corner rules, vertex types, copy relations and linear interval rows. The driver solves the whole problem (copy classes, corner picking, interval matching) into a serializable `Plan` and derives a schedule. Generation is then a pure function of the plan: every job returns a `Patch`, and patches are merged and committed in a fixed order into `sm-mesh-api`'s `MeshBuild`.

Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-model-api`, `sm-mesh-api` (all with `serde`), `sm-op-api` (only for the value codec that checks `Params`), plus `serde`, `serde_json`, `thiserror`, `smallvec`. No rayon (decisions.md W0c 8): parallelism goes through the object-safe `Executor` trait, and a rayon executor belongs in the mesher crate.

## Positioning

> Like **Cubit's scheme + interval-assignment (IIA) model**, it has per-entity schemes with firmness, auto selection, vertex types per surface, and a global integer interval solve. Like **Gmsh's transfinite/recombine pipeline**, constraints (transfinite curves/surfaces, periodic copies) are declared first and structured meshes are generated from them. It differs in four ways:
> - **(A) One explicit, inspectable constraint store.** Every constraint has an id and an origin (intent, sizing, scheme + target, deck boundary, piece hint, driver), and the whole store, the solution, the schedule and every rejected candidate are serialized in the `Plan`. In Cubit these constraints are implicit session state; in Gmsh they are side effects of API calls.
> - **(B) Impositions are data, phrased over capabilities.** A parent scheme requires `Caps::QUAD | Caps::STRUCTURED` of its linking faces, not a list of scheme names, so a new structured scheme needs no edits in the crates that impose. A conflict with a hard user choice is a typed error that names both sides.
> - **(C) Generation is pure and runs in parallel.** Generated nodes have canonical names (`NodeKey`), jobs of one wave run concurrently, and merge and commit are in a fixed order, so 1 thread and N threads give bit-identical meshes. Cubit and Gmsh mesh entity by entity and write straight into the mesh.
> - **(D) Model-layer semantics feed the solve directly.** Deck pairs (periodic, rotational) become copy constraints that join equivalence classes with the sweep's own copies. Deck faces get a hard `Forbid(BOUNDARY_LAYER)`, and mirror faces get `PlaneLock`. Decomposition pieces (`PieceRef`, `PieceShape::Sweep`) are first-class volume targets.
>
> This gives reproducible meshes (the plan and the mesh are functions of the intent, the rejections and the model, so they can live in an Op and in git), explainable failures (every infeasibility lists its constraints and entities, every passed-over scheme an `Attempt`), safe fallback (blame, reject, re-plan), and exact periodic or mirrored meshes by construction.

## Pipeline

```
MeshIntent (serde) + Rejections + MeshInput {TopologyView, Model, PieceResolver, selection}
  | Driver::plan_with
  |  1. region: pieces -> faces (resolver) -> edges -> vertices                 (ascending ids)
  |  2. deck constraints from Model::face_kind (copy, Forbid(BOUNDARY_LAYER), PlaneLock), boundary-layer requests
  |  3. volumes, then groups: resolve scheme -> Scheme::requirements            (impose, claim, role, copy, rules, data)
  |  4. user copies; copy closure (vertex maps, implied edge copies); copy classes (root + members)
  |  5. faces: claimed -> Claimed, class member -> Copy, else resolve + requirements;
  |     closure and classes again whenever a requirement added copies
  |  6. curves: the same
  |  7. final classes; copy-member caps check; Selection::Copy{class} and inherited caps
  |  8. CornerPicker per face with a VertexRule -> FaceLayout (types + logical sides with directions)
  |  9. sizing/intent goals, curve-copy equalities, Scheme::interval_constraints -> IntervalSolver
  | 10. footprints per job; Kahn levels -> waves                                -> Plan (serde)
  | Driver::generate (Executor per wave; patches checked, claims split, merged in job order;
  |                    transforms; copy verification)                           -> PatchSet
  | Driver::commit (nodes in patch-set order -> cells -> face and edge classes -> finish)
  \ Driver::mesh = plan -> generate -> commit; on err.blame() reject (target, scheme) and re-plan
```

## Resolution of one target

1. **Claimed** by a parent (`Requirements::claim`): `Selection::Claimed { by }`, no scheme of its own. A hard intent for another scheme is an `ImpositionConflict`; a soft one is recorded as an `Attempt`. Impositions on a claimed child (by other targets) are checked after planning against `(claimer scheme, claimer caps restricted to the surface bits)`: a hard one that fails is an `ImpositionConflict`, a soft one an `Attempt`.
2. **Non-root member of a copy class**: `Selection::Copy { class }`, scheme `copy`, `caps = INHERIT` resolved through the root.
3. Otherwise the candidates are tried in this order, and every candidate that does not win becomes an `Attempt` (phase `Resolve`):
   1. the intent chain `[scheme, fallbacks...]` (`Selection::Intent { rank }`; a group uses `MeshGroup::scheme` and `MeshGroup::fallbacks`);
   2. the registry fallbacks of the last intent choice (`SchemeRegistry::set_fallback`, `Selection::Registry { rank }`);
   3. the decomposition hint (skipped past a hard intent, which is never replaced): `PieceShape::Sweep` becomes a `sweep` choice with source and target faces, `PieceShape::Map` a `map` choice (`Selection::PieceShape`);
   4. the best applicable scheme among the auto list: `MeshIntent::defaults.auto` if non-empty, else `SchemeRegistry::auto`, else every registered scheme of the dimension. When impositions exist the remaining registered schemes follow the list, so a narrow auto list never hides what a parent requires. Higher score wins, ties go to list order, then name. `Selection::Imposed { score }` if any imposition applied, else `Selection::Auto { score }`.
4. A candidate is skipped when it is not registered for the target's dimension (an unknown *intent* choice is `UnknownScheme`) or cannot mesh groups, is in `Rejections`, violates an imposition (`Need::Schemes`, `Need::Caps` against `Scheme::caps`, `Need::Forbid`), or reports `NotApplicable`. A **hard** intent choice that violates an imposition is an `ImpositionConflict` (or `NotApplicable` when the imposition is a boundary-layer request from the intent); a soft or default one is overridden. Imposition firmness: a `Hard` imposition always blocks; a `Soft`/`Default` one is a preference. A hard intent ignores soft impositions (recorded as an `Attempt`); otherwise candidates violating one are passed over, and if that leaves nothing applicable, resolution runs again with the soft impositions dropped. Impositions from the user (decks, boundary-layer requests) are `Hard`.
5. The winner's `Applicability` reason is stored in `TargetPlan::reason`.

Impositions on the members of a class that the root generates apply to the root too: a sweep's requirement on a linking face that is a deck copy constrains the scheme chosen for the deck's other face (defect D3). `Caps::INHERIT` is resolved through the class root, so a copy of a mapped face satisfies `QUAD | STRUCTURED`.

## Constraints and requirements

`Requirements` is the writer a scheme gets while planning. Everything is recorded with the scheme and target as origin, and a requirement is refused with `SchemeError::BadRequirement` unless its child is a strict sub-entity of the planned target (or of a group member) inside the region, and not resolved yet (defect D5). `Requirements::new` and `IntervalSink::new` are crate-private, so origins cannot be forged.

| Call | Meaning |
|---|---|
| `impose(child, names)` | `Need::Schemes`, hard. Only for "exactly this algorithm". |
| `require_caps(child, caps, firmness)` / `forbid(child, caps)` | `Need::Caps` / `Need::Forbid`. The normal way. |
| `claim(child)` | This target writes `child` itself (many-to-one target, interface faces). The child is not resolved or generated on its own. |
| `role(child, Role::{SweepSource, SweepTarget, SweepLinking})` | Roles read by the interval phase and by root preference. |
| `copy(CopyConstraint)` | An edge of a copy class. `source` and `target` are strict sub-entities and `target` is unresolved. |
| `vertex_rule`, `vertex_type` | Corner rules and requests on faces. |
| `constraint(MeshConstraint)` | `Precedes`, `PlaneLock`, `BoundaryLayer`. |
| `set_data(Params)` | Scheme-private analysis, stored in `TargetPlan::data` and read back in the interval and generation phases through `IntervalCtx::data` and `GenCtx::data`. |

`IntervalConstraint::{Fixed, Goal, Range, Linear { terms, rel, rhs }, EvenSum}`. `Linear` has integer coefficients and `Rel::{Eq, Le, Ge}`: map and submap side sums, submap direction sums (`SideDir`), sweep layer counts, many-to-one path sums and copy equalities are rows of this form. `IntervalSink::{equal, linear, even, fix, range}`.

### Corners

`VertexType::{End, Side, Corner, Reversal}` is stored per `CornerKey` (face + incoming coedge), never on the vertex. Schemes ask for a `VertexRule::{Free, Map, Submap, Ends(n)}`. The driver-owned `CornerPicker` returns one `FaceLayout` per face: the corner types and the logical sides. Each `LogicalSide` carries a `SideDir` computed from the cumulative quarter turns (End +1, Side 0, Corner -1, Reversal -2), so submap, triprimitive and sweep crates all read the same directions. Intent overrides (`CornerIntent`) are hard requests; the picker reports a violated rule or sum as `Infeasible` naming the rule and the requests.

## Copy classes

Copy constraints (user copies, deck pairs, sweep targets, edge copies implied by face copies) are **edges of an equivalence relation**, not directed, unique relations (defect D2).

- The driver forms classes with union-find. The **root** is the member meshed by a scheme. Preference, first to last: not a `copy`-intent target; already planned with a scheme of its own; a `Role::SweepSource`; claimed by a parent; a hard non-copy intent; the lowest target. A sweep source beats a claimed face, so a deck pair between a group sweep's source and its claimed target is rooted at the source and the target is only verified.
- A spanning tree from the root composes transforms and vertex correspondences. Each member gets `from_root`, a root-to-member `vertex_map`, the constraint ids on its path and a `CopyMode`.
- Every redundant constraint is checked: its vertex correspondence must agree with the composed one, and its transform must place the root's vertices where the composed one does, within `DriverOptions::copy_tolerance` (default `ModelTolerance::linear`). Otherwise `CopyConflict { target, first, second, reason }`. A member that already has a scheme of its own is a `CopyConflict` too.
- `CopyMode::Generate`: the driver generates the member as the image of the root. `CopyMode::Verify`: another job writes it (it is claimed). After generation the driver checks, by key correspondence and not by quantized centroids, that the member's nodes are at `from_root(p)` within tolerance and that its elements are the image of the root's. A mismatch is `CopyMismatch { constraint }`, blamed on the claimer.
- A copy generated from a **claimed** root runs after the claimer: "face after volume" is a plain dependency (defect D6).
- Closure runs again whenever a face or curve requirement adds a copy (defect D4): vertex maps, implied edge copies, interval equalities (origin `Driver`), class membership.
- The copy generator maps `NodeKey`s through the vertex and edge correspondences (nodes owned by the source keep their index, nodes of reversed edges use `n - i`), places points with `from_root`, drops parameters (`Param::None`, kept for v1) and reverses surface elements when `det(M) (M n_s) . n_t < 0`, using the real face normals.
- Volume copies are not in v1 (instancing and flatten belong to the model layer and `sm-mesh-api`).

## Footprints, claims and waves

Every job (`Job::Generate(target)` or `Job::Transform(id)`) has a **derived** `Footprint { reads, writes }`:

- writes = the job's own target plus the children it claims;
- reads = its boundary children that it does not claim (edges of a face, faces of a piece or group), its class root if it is a generated member, its `Precedes` predecessors; for a transform its targets.

A job depends on the writer of everything it reads: the claimer for a claimed child, the group job for a grouped volume, the target's own job otherwise. Waves are Kahn levels over these dependencies; ties are broken in `Job` order. Two writers of one target, a read with no writer, and a cycle are `SchemeError::Schedule { jobs, constraints, reason }` naming the claims, copies or precedences involved.

**Claimed children and classification.** A claimer's patch may contain nodes owned by the children it claims and elements whose `owners[i]` names a claimed face (Tri, Quad or Polygon in a volume patch; a Segment for a claimed edge). The driver splits these into patches of their own with `target = child`, merges the child patches first (the remainder references their nodes), and from then on the child behaves like any generated target: it is classified on its face by `commit`, it can be read by `GenCtx::grid`/`point`, and it can be a copy root or a verified member.

## Patches

A `Patch` has a local node table. Element entries are `u32`: `0..nodes.len()` name the nodes the patch creates (owned by the target, a group member or a claimed child), the rest index `refs`, the canonical `NodeKey`s of earlier jobs. `PatchBuilder` deduplicates references and renumbers at `finish`. `Elements` is CSR over the flat index array, with a face CSR for polyhedra (`push_polyhedron`, committed through `MeshBuild::add_polyhedron`). `Elements::get`, `NodeGrid::at` and `NodeGrid::position` return `Option`.

`PatchSet` keeps points and parameters as dense arrays in commit order (vertex nodes first, then each patch's nodes in merge order) plus a lookup-only `NodeKey -> u32` hash index that is never iterated. Commit resolves each patch's table once, so it does one lookup per node or reference of a patch, not one per element corner.

`check_patch` enforces: the right target; node owners; unique keys that fit their owner; references that exist and are unique; curve patches own exactly `1..=n-1` of their edge; element shapes with the right node counts and dimension; owners empty or one per element; surface elements in a volume patch only for claimed faces.

**Node keys.** `NodeKey { owner, index }`. A topology vertex is `(v, 0)`. An edge with `n` intervals owns `(e, 1..=n-1)` **in the edge's own direction**, so `GenCtx::edge_nodes`/`coedge_nodes`/`side_nodes`/`loops` give every boundary node without asking the curve scheme. Faces and volumes number their own nodes. A copy keeps the source index.

**Narrow inputs.** `GenCtx::loops(face)` returns closed `BoundaryLoop`s of node keys with the coedge stretches (pave, tri*, polygon schemes). `GenCtx::curve_patch(edge, fractions)` builds a valid curve patch from `n - 1` increasing fractions, so bias, dualbias and curvature schemes never compute keys or positions. `geom` has `face_loops`, `corner_angle`, `snap_to_face`, `volume_face_uses` next to the older point, tangent, length, normal and corner helpers. There is no second query trait: `&dyn TopologyView` is object-safe and `CellComplex` fixtures serve scheme tests.

## Failure, blame and re-plan

`FailureKind::{NotApplicable, Rejected, Constraint, Geometry, Quality, Unsupported, Internal}`; only `Internal` is fatal (`allows_fallback`). `Attempt { target, scheme, phase, kind, message }` with `Phase::{Resolve, Requirements, Corners, Intervals, Schedule, Generate, Transform, Verify, Commit}` is collected in `Plan::attempts`, `Meshed::attempts`, `NoApplicableScheme::tried` and `ReplanLimit::attempts`.

`SchemeError::blame()` names the `(target, scheme)` a different choice could avoid:

- **Interval or corner infeasibility** (`Infeasible::blame`, filled by the driver): among the involved constraints whose origin is a scheme whose selection is not a hard intent, the lowest-dimension target, ties to the later constraint. Pave's even-sum loses to a sweep's equalities.
- **A child with no applicable scheme under an imposition** (`NoApplicableScheme::imposed_by`): the imposing parents that can be re-planned (planned, not hard, not `copy`, not claimed). A hard parent is never blamed.
- **A generation failure** with a fallback-eligible kind blames the failing scheme; `CopyMismatch` blames the member's claimer.

`Driver::mesh` inserts the blamed pair into `Rejections` and re-plans from scratch, up to `max_replans`. A different scheme changes the constraints, planning is cheap next to generation, and `plan_with(intent, rejections)` is the replay entry point. `Rejections` serializes as a list of `Attempt`s and lives in `Plan::rejections`, not in the intent. Planning errors enter the loop too; errors without blame (hard conflicts, bad requirements, schedule errors) are returned as they are. At generation time a failing scheme is rejected whatever its firmness; a hard choice then ends in `NoApplicableScheme` carrying the failure in its attempts.

## Atomicity

Nothing is written before every patch exists, and `commit` builds a fresh builder: any error before `MeshBuild::finish` drops it, and `finish` leaves `SerialAlloc` unchanged on error. A failed group, transform or verification therefore leaves the allocator untouched, and a later success is bit-identical to a run on a fresh allocator (tested with `topology_hash`, `geometry_hash` and `SerialAlloc::state`).

## Transforms

`MeshTransform::apply(cx, targets, params, patches)` rewrites one patch per target, in target order. `MeshIntent::transforms` lists standalone conversions (THex, HTet, QTri) over arbitrary volume targets; `MeshGroup::conversion` is shorthand for a transform over the group (driver id `TransformId::GROUP_BASE | group id`). A transform is a job whose reads and writes are its targets: it runs after every writer of them, and transforms over the same targets run in id order. The rewritten patches are checked and swapped in after the wave; any failure fails the mesh.

## Model layer

- **Decks.** Each `FaceKindView::Deck` face gets a hard, deck-origin `Forbid(BOUNDARY_LAYER)` imposition (a `BoundaryLayerIntent` on it is `BoundaryLayerOnDeck`). The A side of a translation or rotation pair gets a `CopyConstraint` to B with `deck.transform()`. A mirror face gets `PlaneLock`.
- **Boundary layers.** The intent on a face becomes `MeshConstraint::BoundaryLayer { face, volume, spec }` per adjacent volume target plus a hard `Need::Caps(BOUNDARY_LAYER)` on it (a user request is never silently dropped). Only a volume scheme with that capability can honour it: others are passed over, a hard choice is `NotApplicable`, and nothing applicable is `NoApplicableScheme`. No boundary-layer scheme ships in W1.
- **Pieces.** Volume targets are pieces (`Target::piece`, `piece_of`): a `TopoVolume` id is `PieceRef::Whole`, a `PartitionCell` id `PieceRef::Cell`. `PieceResolver` maps pieces to faces of the view and node owners to classification entities. The default `WholeFaces` handles whole volumes and cells whose faces are all whole model faces, including composites of whole volumes (internal faces vanish, so a composite of merged bricks is meshed as one piece). It refuses cut faces with a clear error; a decomposition-overlay resolver is the seam for them.

## Determinism and parallelism

- Every map is a `BTreeMap`/`BTreeSet` or a sorted `Vec`; the only hash map is the lookup-only `NodeKey` index of `PatchSet`. Targets, faces, edges and vertices are processed in ascending id. Constraint ids follow contribution order. Plan tables are sorted vectors, so the JSON form is canonical.
- Union-find classes in the interval solver are named by the lowest edge id. Copy-class roots follow the fixed preference above, the spanning tree is a BFS in constraint order, blame follows the fixed rule, and levelling breaks ties in `Job` order.
- A wave's jobs write into pre-allocated slots; the driver reads them in job order and merges, so the first error in job order is reported and the output is independent of the executor.
- `tests/determinism.rs` compares `topology_hash`, `geometry_hash` and the plan JSON across two sequential runs, `ScopedThreads(4)` and four concurrent `std` threads, for a deck-periodic sweep, a pave-to-tri re-plan and a claiming group sweep. This crate has no rayon (the executor is a trait), so there is no `RAYON_NUM_THREADS` variant to run here; the mesher crate repeats the check with its rayon executor.

## Reference implementations and stubs

In the crate: `ReferenceIntervalSolver` (union-find classes, greedy repair of `Linear` rows with `Eq`/`Le`/`Ge`, parity), `ReferenceCornerPicker`, `WholeFaces`, `Sequential`, `ScopedThreads`, `ConstantSize`.

In `tests/stub_schemes`: `equal` (via `curve_patch`), `map`, `submap` (direction sums), `pave` (even loop sum, quad fan), `tri` (triangle fan), `sweep` (columns may pass through interior vertices: a composite piece), `msweep` (a group of stacked volumes; claims interfaces and the target; path sum), plan-level `mapvol`, `tetvol`, `polyvol` (one polyhedral cell), transforms `noop`, `lift`, `boom`, and the fixtures `brick_setup` and `row_setup` (merged bricks).

Tests (all through the sm-topo-api `CellComplex` fixtures and the sm-mesh-api `ReferenceBuilder`):

| File | Covers |
|---|---|
| `brick.rs` | 2x3x4 sweep with roles and capability impositions (24 cells, `validate` clean, analytic volumes, classification counts); equal-curve nodes; auto selection with score, reason and outscored losers; imposition conflicts; infeasible hard counts with blame; solver parity; generation failure and replay; translation deck pair with identical face meshes; boundary layers |
| `corners.rs` | vertex types per face use (map with a mid-edge vertex), submap with hard corner overrides and side directions, an infeasible corner sum |
| `pave.rs` | even-sum repair, hard odd counts to `Infeasible` with pave blamed, fallback to `tri` through the intent chain and the registry chain, auto lists |
| `groups.rs` | many-to-one style multisweep over three volumes (path sum matching, claimed faces, footprints), verification mismatch, atomic failure (allocator state and hashes), standalone transforms, a composite decomposition piece |
| `copies.rs` | deck x sweep matrix (source/target in both directions), conflicting redundant constraints, impositions through `copy`, copies contributed by a face scheme, a deck copy of a claimed face scheduled after the group |
| `contracts.rs` | `BadRequirement`, schedule errors, blame of a soft parent, polyhedral commit, patch and grid API, linear rows, geometry helpers, executors |
| `intent_serde.rs` | intent and plan round trips through JSON and the Op codec (TOML-safe), parameter checks |
| `determinism.rs` | see above |

## Deviations and limits (honest list)

- **Additions to the section 5 listing** (nothing removed): `CopyMember::vertex_map` (the composed root-to-member correspondence, needed by the generator and the verifier); `Plan::sizing` (so `generate(input, plan, exec)` can rebuild the size field without the intent); `Plan::class_of`, `MeshingProblem::{claim_of, planned_targets}`; `NodeGrid::new`; `Patch::local_len`; `PatchSet::merge_order`; `Elements::indices`; `Caps::without` and `BitAnd`; `SideDir::{turned, reversed, along_i}`; `TransformId::GROUP_BASE`; `copy::{compose, inverse, compose_maps, invert_map, edge_map_for, target_vertices}`; `SchemeError::BadRequirement` also covers requirements on resolved targets.
- `Meshed::attempts` is the history of failures that caused re-plans followed by the passed-over candidates of the final plan; `Plan::attempts` has only the latter (plus candidates skipped because of rejections).
- `NodeGrid::position` searches the boundary ring first and then the interior instead of keeping a sorted boundary index, so `NodeGrid` stays constructible by struct literal. `line_from` is O(ni + nj).
- `MeshingProblem` and `Plan` keep lazily built lookup tables that are skipped by serde and equality. Mutating the public tables by hand after a lookup leaves them stale; the driver never does.
- With impositions present, an auto list does not hide the registered schemes that are not in it (a narrow list must not make a parent's requirement unsatisfiable).
- Generation-time blame does not look at firmness (planning-time blame never blames a hard intent). A hard choice that fails generation is rejected and the re-plan ends in `NoApplicableScheme` with the failure in its attempts.
- Planning builds copy classes from scratch after every requirement that adds copies; this is O(copies) each time. Fine for decks and sweeps, to be revisited if a scheme adds thousands of copies one by one.
- A claimed *edge* child is split out of its claimer's patch like a face, but the curve rule (`1..=n-1`) is not re-checked on the split patch.
- Volume copies, a boundary-layer scheme and surface-only meshes (an `sm-mesh-api` face needs a cell) are out of v1. Copied nodes get `Param::None`; a later pass may re-project.
- The reference interval solver is not an optimal IIA: greedy repair with a step limit. It is exact for the structured rows used here; an ILP or network-flow solver can plug in behind `IntervalSolver`.
- The face-copy vertex correspondence is geometric (`transform(p)` within the summed vertex tolerances) unless `vertex_map` is given. A closed edge matches by its ends only.
- Test stubs are stubs: `sweep` finds columns by straight continuation and handles prismatic stacks only; `submap` meshes layouts with exactly four logical sides (the corner-override test generates a face whose straight-through vertex sits inside a logical side); the many-to-one test is a multisweep through three stacked volumes with one source and one target (it exercises claims, interface faces, the path sum and verification, not several sources into one target face).
- `Cargo.lock` gained the `sm-op-api` edge for this crate (written by cargo).

## Answers to the open questions (recommendation section 6)

1. **Cut faces of pieces.** The contract does not change. `PieceResolver` is the seam and `WholeFaces` refuses cut faces with a typed error; an overlay resolver arrives with the decomposition overlay.
2. **Submap side directions.** Driver-owned, in `FaceLayout` (`LogicalSide::dir`). Many-to-one layer surfaces stay scheme-private (`TargetPlan::data`).
3. **Rotational sweeps, morph copies, volume copies.** Copies stay rigid (one `sm_core::Transform`; a rotation is one). Morphing is a sweep generation detail. Volume copies are model-layer instancing, not schemes.
4. **Re-plan granularity.** Full deterministic re-plan; `plan_with(rejections)` leaves room for an incremental re-solve behind the same API.
5. **Boundary layers.** `MeshConstraint::BoundaryLayer`; only `Caps::BOUNDARY_LAYER` schemes may honour it; deck faces forbid it.
6. **Deck pair where B is the sweep source.** Copy classes (D2); tested in both directions.
7. **Capabilities or names.** Capabilities by default; names for "exactly this algorithm". A scheme crate must never list sibling scheme names to express a property.
8. **Params and replay.** `serde_json::Value` checked through the Op codec; `Rejections` stored in the `Plan`.
9. **Surface-only meshes.** An `sm-mesh-api` change if `docs/PARITY.md` ever needs them; the patch contract already carries surface elements.
10. **A rayon executor.** Lives in the mesher crate and implements `Executor`.

## API change requests (non-blocking)

- `sm-mesh-api`: `Classification::dim` should accept `PartitionFace` (2), `PartitionEdge` (1) and `PartitionVertex` (0), so mesh entities on virtual cuts can be classified.
- `sm-model-api` / `sm-topo-api`: a way to realize a piece's cut faces as topology ids, so a resolver can return them. Proposed: `Decomposition::piece_face_cells(&self, pf: &PieceFace, topo: &dyn TopologyView) -> Option<Vec<EntityId>>`, backed by an overlay `Model::realize_cut(cut: CutId, faces: &[EntityId])`.
- `sm-topo-api` / `sm-model-api`: an optional `schemars` feature deriving `JsonSchema` on `CoedgeKey`, `CornerKey`, `Side`, `PieceRef`, `PieceFace`, `CutId` and `Keep`, so that `MeshIntent` and `Plan` can export schemas for `TypedOp`/MCP.
- `sm-core`: `Transform::inverse()` and composition (`then`). This crate carries its own `copy::{compose, inverse}`; a shared one would remove the duplicate.
