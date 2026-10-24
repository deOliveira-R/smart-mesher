# sm-scheme-api: final design

The contract between the meshing driver and every meshing scheme (curve, surface, volume) and the sizing layer. This is the frozen W0 contract. It is the `constraint` panel candidate (`ddd5598`) with the grafts G1 to G12 and the fixes D1 to D13 of `docs/design/w0/scheme-recommendation.md` applied.

**Approach: constraint system first.** Schemes are mostly declarative. Before any mesh exists, each scheme contributes constraints to one global `MeshingProblem`: capability impositions on children, claims, roles, corner rules, vertex types, copy relations and linear interval rows. The driver solves the whole problem (copy classes, corner picking, interval matching) into a serializable `Plan` and derives a schedule. Generation is then a pure function of the plan: every job returns a `Patch`, and patches are merged and committed in a fixed order into `sm-mesh-api`'s `MeshBuild`.

Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-model-api`, `sm-mesh-api` (all with `serde`), `sm-op-api` (only for the value codec that checks `Params`), plus `serde`, `serde_json`, `thiserror`, `smallvec`. No rayon (decisions.md W0c 8): parallelism goes through the object-safe `Executor` trait, and a rayon executor belongs in the mesher crate.

## Positioning

> Like **Cubit's scheme + interval-assignment (IIA) model**, it has per-entity schemes with firmness, auto selection, vertex types per surface, and a global integer interval solve. Like **Gmsh's transfinite/recombine pipeline**, constraints (transfinite curves/surfaces, periodic copies) are declared first and structured meshes are generated from them. It differs in four ways:
> - **(A) One explicit, inspectable constraint store.** Every constraint has an id and an origin (intent, sizing, scheme + target, deck boundary, piece hint, driver), and the whole store, the solution, the schedule and every rejected candidate are serialized in the `Plan`. In Cubit these constraints are implicit session state; in Gmsh they are side effects of API calls.
> - **(B) Impositions are data, phrased over capabilities.** A parent scheme requires `Caps::QUAD | Caps::BLOCK_STRUCTURED` of its linking faces (W0f gaps3; `STRUCTURED` alone before), not a list of scheme names, so a new structured scheme needs no edits in the crates that impose. A conflict with a hard user choice is a typed error that names both sides.
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
  | Driver::commit (nodes in patch-set order -> cells -> face and edge classes -> finish;
  |              a dimension-2 builder takes surface elements as polygon cells)
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

Every job (`Job::Generate(target)` or `Job::Transform(id)`) has a **derived** `Footprint { reads, writes }` (W0f gaps 2: a generate job's dependency on a read target goes to the last transform over it, if any):

- writes = the job's own target plus the children it claims;
- reads = its boundary children that it does not claim (edges of a face, faces of a piece or group, and the embedded edges of a face or piece), its class root if it is a generated member, its `Precedes` predecessors; for a transform its targets.

A job depends on the writer of everything it reads: the claimer for a claimed child, the group job for a grouped volume, the target's own job otherwise. Waves are Kahn levels over these dependencies; ties are broken in `Job` order. Two writers of one target, a read with no writer, and a cycle are `SchemeError::Schedule { jobs, constraints, reason }` naming the claims, copies or precedences involved.

**Claimed children and classification.** A claimer's patch may contain nodes owned by the children it claims and elements whose `owners[i]` names a claimed face (Tri, Quad or Polygon in a volume patch; a Segment for a claimed edge). The driver splits these into patches of their own with `target = child`, merges the child patches first (the remainder references their nodes), and from then on the child behaves like any generated target: it is classified on its face by `commit`, it can be read by `GenCtx::grid`/`point`, and it can be a copy root or a verified member.

## Patches

A `Patch` has a local node table. Element entries are `u32`: `0..nodes.len()` name the nodes the patch creates (owned by the target, a group member or a claimed child), the rest index `refs`, the canonical `NodeKey`s of earlier jobs. `PatchBuilder` deduplicates references and renumbers at `finish`. `Elements` is CSR over the flat index array, with a face CSR for polyhedra (`push_polyhedron`, committed through `MeshBuild::add_polyhedron`). `Elements::get`, `NodeGrid::at` and `NodeGrid::position` return `Option`.

`PatchSet` keeps points and parameters as dense arrays in commit order (vertex nodes first, then each patch's nodes in merge order) plus a lookup-only `NodeKey -> u32` hash index that is never iterated. Commit resolves each patch's table once, so it does one lookup per node or reference of a patch, not one per element corner.

`check_patch` enforces: the right target; node owners; unique keys that fit their owner; references that exist and are unique; curve patches own exactly `1..=n-1` of their edge, **and so does every curve the target claims** (W0f: a claimed edge is split out and committed as a curve, so its numbering must hold for the copy generator's reversed mapping `n - k`); element shapes with the right node counts and dimension; owners empty or one per element; surface elements in a volume patch only for claimed faces. `split_claims` maps patch-local indices through a dense table (nodes are already in order), not a per-node `BTreeMap`.

**Node keys.** `NodeKey { owner, index }`. A topology vertex is `(v, 0)`. An edge with `n` intervals owns `(e, 1..=n-1)` **in the edge's own direction**, so `GenCtx::edge_nodes`/`coedge_nodes`/`side_nodes`/`loops` give every boundary node without asking the curve scheme. Faces and volumes number their own nodes. A copy keeps the source index.

**Surface element orientation (W0f).** Surface elements follow their face's normal (the topology orientation rule, left to right) or, for a copy, the root's orientation relative to the root's normal: the copy generator reverses elements when `det(M) (M n_s) . n_t < 0`, and `verify_copy` now compares elements with their cyclic orientation (rotated to the smallest key, the root's reversed when the copy flips), so a claimer that writes a verified member against its normal is a `CopyMismatch`. The `msweep` stub writes claimed planes along each plane face's normal.

**Narrow inputs.** `GenCtx::loops(face)` returns closed `BoundaryLoop`s of node keys with the coedge stretches (pave, tri*, polygon schemes). `GenCtx::curve_patch(edge, fractions)` builds a valid curve patch from `n - 1` increasing fractions, so bias, dualbias and curvature schemes never compute keys or positions. `geom` has `face_loops`, `corner_angle`, `snap_to_face`, `volume_face_uses` next to the older point, tangent, length, normal and corner helpers. There is no second query trait: `&dyn TopologyView` is object-safe and `CellComplex` fixtures serve scheme tests.

## Surface-only meshes (W0f, W0e decision 5)

`MeshInput::selection` holds either pieces or faces of the view (`TopoFace` / `PartitionFace`), never both (`SchemeError::Input`). A face selection plans faces, curves and vertices only. `Driver::commit` into a dimension-2 builder (`ReferenceBuilder::surface()`) adds every surface element as a polygon cell (`MeshBuild::add_polygon`) classified on its face, oriented along the face normal (each face patch is reversed as a whole when its first element runs against the normal, so schemes need not agree on an orientation), and classifies every curve segment's facet on its edge; interior facets derive their class from the cells. A volume patch cannot go into a dimension-2 builder (`InvalidPatch`). Faces whose normals disagree across a shared edge cannot form one oriented surface; `finish` reports the facet. Tested in `surface_only.rs`: the six faces of a brick as one closed quad shell (`V - E + F = 2`, analytic area, cells along the outward normals, identical across executors), a single open face, mixed and mismatched selections.

## W0f fixes (before plans are persisted)

- **Firm sizes keep their firmness.** `IntervalRequest::Size(h)` with `Hard` or `Soft` becomes `Fixed { count: round(L / h).max(1), firmness }`; `Default` stays a real `Goal { L / h }`. (It used to become a goal and drop the firmness.)
- **Size-field goals integrate.** With a `SizeField` and no request, an edge's goal is `integral ds / h(x)` (`geom::edge_size_goal`: composite 5-point Gauss-Legendre over the curve parameter, panels doubled until two estimates agree to `1e-10` relative). It used to sample `h` once at the midpoint. Test: `h(x) = 0.01 + x` on a unit edge gives `ln 101`.
- **Boundary layers off faces are refused.** `MeshIntent::check` and the planner reject a boundary layer on anything but a `TopoFace` / `PartitionFace` (`SchemeError::Input`) until curve boundary layers exist; it used to be dropped silently. `BoundaryLayerIntent` gained `into: Vec<EntityId>` (empty = every adjacent volume; each entry must be an adjacent volume piece) and `params: Params`, both `serde(default)`, so later options are not breaking; it is no longer `Copy` (`BoundaryLayerIntent::new(first, growth, layers)`).
- **`#[non_exhaustive]`**: `Selection` and `MeshConstraint` (a seeded selection, curve boundary layers later).
- **Schemas.** The `schemars` feature (enables the upstream `schemars` features) derives `JsonSchema` on `SchemeName`, `Params` (object schema), `SchemeChoice`, `Firmness`, `Firm`, `IntervalRequest`, `BoundaryLayerIntent`, `EntityIntent`, `CornerIntent`, `MeshGroup`, `TransformIntent`, `SizingSpec`, `AutoPolicy`, `Defaults`, `MeshIntent` (and `VertexType`, `GroupId`, `TransformId` inside them). `schema_for!(MeshIntent)` is tested.

## W0f gaps 2

- **Embedded conformity (contract-owned planner behaviour).** `MeshInput::embedded_edges(parent)` / `embedded_vertices(parent)` read the `Incidence::EMBEDDED` edge and vertex members of a face or piece in the view. `Planner::region` adds the embedded edges of every selected face and piece to the region's edges (each gets a curve target, an interval variable and node keys `(e, 1..n-1)`) and their ends and the embedded vertices to its vertices (vertex nodes). `children_of` lists them as curve children of the face, of the piece and of a group's members, so `Requirements` accepts them; the schedule makes the parent's job read them, so their keys exist when it runs. `GenCtx::embedded(parent)` returns one node chain per embedded edge (edge direction) and a one-node chain per embedded vertex that ends no embedded edge. A scheme meshing a face or piece with embedded members must conform (reference every chain node, every chain segment an element edge) or report `NotApplicable`; `check_patch` rejects a face, volume or group patch (and its claimed faces) that does not reference every embedded node (`InvalidPatch` "does not conform to its embedded members"), replacing the old silent non-conforming mesh. Embedded faces of a volume are not covered. Tested in `gaps2.rs` with a conforming two-polygon scheme (`splitpoly`) on a face split by an embedded diagonal, the stub `tri` refused, and an embedded point that needs a vertex node.
- **Transforms over faces and curves.** `TransformIntent::targets` may name faces and curves of the region besides pieces (`plan_transforms`). In the scheduler the last transform over a target (by id) is the writer its readers depend on (a volume reading a smoothed face, a face reading a transformed edge, a copy member reading its transformed root); transforms over one target keep their id-order chain and still depend on the target's generator. `GenCtx::patches` therefore shows later waves the transformed patch, and a generated copy of a transformed root is the image of the transformed patch. `Plan` and `Footprint` are unchanged. A transform covering a target and one of its readers is a dependency cycle (`Schedule`). Needed for smoothing a surface before a sweep and for transforms on surface-only meshes (W0e decision 5).
- **User interval rows.** `MeshIntent::intervals: Vec<IntervalConstraint>` (`serde(default)`; `IntervalConstraint` and `Rel` derive `JsonSchema` under `schemars`): matched, relative and chain-sum intervals (PARITY C P1). `IntervalConstraint::check` (called by `MeshIntent::check`) requires edge kinds, counts and minimums `>= 1`, `min <= max`, finite positive goals, non-empty rows without zero coefficients and `|rhs| <= 2^53`; `IntervalConstraint::canonicalize` (called by `MeshIntent::canonicalize` and the planner) sorts and merges `Linear` terms and sorts `EvenSum` edges. The planner adds each row with `Origin::Intent` after the per-edge goals, refusing an edge outside the region (`Input`). Not adopted: an optional `IntervalSink::goal`.
- **Commit node correspondence (contract).** `Driver::commit` adds nodes in `PatchSet::nodes()` order, so `VertexIx(i) == PatchSet::index_of(key) == i` (finishing does not renumber vertices); a non-empty builder is refused (`Input`). `Meshed::vertex_of(key)` reads it. There is no `CommitMap`: a cell's target is its classification.

## W0f gaps3: block-structured faces

- `Caps::BLOCK_STRUCTURED = Caps(1 << 10)`: all-quad, valence-4 interior, so every quad strip
  can be walked boundary to boundary. `Caps::STRUCTURED` is narrowed to "exactly one
  `NodeGrid`" (a single-block mapped mesh). Map produces `QUAD | STRUCTURED | BLOCK_STRUCTURED`,
  submap `QUAD | BLOCK_STRUCTURED`. `Patch::grid` is the single-block structured view (map); a
  submap patch need not carry one (the submap promise of "grid blocks" is dropped).
- `Patch::strip_from(key) -> Option<Vec<NodeKey>>`: the node line of the quad strip that starts
  at boundary node `key`, `key` first. A grid answers through `NodeGrid::line_from` (fast path);
  otherwise the element table is walked (interior edge from `key`, then at each node the edge
  sharing no quad with the incoming one) until a boundary node; ties (re-entrant corner,
  irregular node) go to the smallest neighbour key. `None` for a non-quad patch, a key off the
  boundary, a corner without an interior edge, or a walk that does not close. `GenCtx::strip_from(face,
  key)` and `GenCtx::surface_patch(face)` expose it to generators.
- The planner's claimer mask (`child_caps`) includes `BLOCK_STRUCTURED`, so a claimed linking
  face written by its claimer still satisfies the imposition.
- Stubs: the sweep imposes `QUAD | BLOCK_STRUCTURED` on linking faces and walks them with
  `GenCtx::strip_from`; the stub submap attaches no grid. Tests (`gaps3.rs`): an L-shaped
  submapped face (two blocks, no grid) walked from every kind of boundary node, the grid fast
  path agreeing with the walk, and a sweep through four hard-submapped (grid-less) linking faces.

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

`MeshTransform::apply(cx, targets, params, patches)` rewrites one patch per target, in target order; it is restricted to **topology-preserving patch edits** (smoothing, snapping, reclassification), because the rewritten patch is re-checked by `check_patch`, which rejects nodes its target does not own. `MeshIntent::transforms` lists such transforms over volume targets (pieces); `MeshGroup::conversion` is shorthand for a transform over the group (driver id `TransformId::GROUP_BASE | group id`) and is **deprecated** (W0f). **Element conversions** (THex, HTet, QTri, TQuad) cannot work as `MeshTransform`s: they create new nodes on shared faces and edges (owned by neighbours) and change element counts. They are IR -> IR mesh passes: read a finished mesh through `sm_mesh_api::MeshRead`, build the converted mesh through `MeshBuild` (classification carried over, new nodes classified on the entity of the split face/edge/cell), and re-verify the deck pairs on the result (the conversion must split paired deck faces identically). They live in the mesh-ops crates, not in this contract. A transform is a job whose reads and writes are its targets: it runs after every writer of them, and transforms over the same targets run in id order. The rewritten patches are checked and swapped in after the wave; any failure fails the mesh.

## Model layer

- **Decks.** Each `FaceKindView::Deck` face gets a hard, deck-origin `Forbid(BOUNDARY_LAYER)` imposition (a `BoundaryLayerIntent` on it is `BoundaryLayerOnDeck`). The A side of a translation or rotation pair gets a `CopyConstraint` to B with `deck.transform()`. A mirror face gets `PlaneLock`.
- **Boundary layers.** The intent on a face becomes `MeshConstraint::BoundaryLayer { face, volume, spec }` per adjacent volume target plus a hard `Need::Caps(BOUNDARY_LAYER)` on it (a user request is never silently dropped). Only a volume scheme with that capability can honour it: others are passed over, a hard choice is `NotApplicable`, and nothing applicable is `NoApplicableScheme`. No boundary-layer scheme ships in W1.
- **Pieces.** Volume targets are pieces (`Target::piece`, `piece_of`): a `TopoVolume` id is `PieceRef::Whole`, a `PartitionCell` id `PieceRef::Cell`. `PieceResolver` maps pieces to faces of the view and node owners to classification entities. `WholeFaces` handles whole volumes and cells whose faces are all whole model faces, including composites of whole volumes (internal faces vanish, so a composite of merged bricks is meshed as one piece), and refuses cut faces with a clear error. **`DecompositionFaces` (W0f)** resolves every piece face through `Decomposition::piece_face_cells`, so pieces split with `Model::split_realized` mesh through the decomposition overlay (`MeshInput::topo` = the `Overlay`): the cut face, the partitioned halves of model faces and the virtual edges and vertices are ordinary faces, edges and vertices of the view, and `PieceShape::Sweep` hints resolve through `face_of`. Both resolvers classify node owners on themselves for every model and decomposition kind (`Topo*` and `Partition*`, W0e decision 1), so a split piece's mesh lies on its `PartitionCell`, its cut-face nodes and faces on the `PartitionFace`, and so on. Tested end to end in `split_piece.rs` (both pieces: interior faces on the cut face; one piece alone: boundary faces on it).

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
| `many_to_one.rs` | several sources into one claimed target (see above) |
| `split_piece.rs` | realized decomposition split meshed end to end through the overlay, classification on `Partition*`, determinism |
| `surface_only.rs` | dimension-2 commits, claimed-edge curve rule, sizing-function error |
| `determinism.rs` | see above |

## Deviations and limits (honest list)

- **Additions to the section 5 listing** (nothing removed): `CopyMember::vertex_map` (the composed root-to-member correspondence, needed by the generator and the verifier); `Plan::sizing` (so `generate(input, plan, exec)` can rebuild the size field without the intent); `Plan::class_of`, `MeshingProblem::{claim_of, planned_targets}`; `NodeGrid::new`; `Patch::local_len`; `PatchSet::merge_order`; `Elements::indices`; `Caps::without` and `BitAnd`; `SideDir::{turned, reversed, along_i}`; `TransformId::GROUP_BASE`; `copy::{compose_maps, invert_map, edge_map_for, target_vertices}`; `SchemeError::BadRequirement` also covers requirements on resolved targets. W0f additions: `DecompositionFaces`, `sizing::field_for`.
- **Removed in W0f (W0e decision 4):** `copy::{compose, inverse}`. Copy transforms compose with `sm_core::Transform::{compose, inverse}` (same convention: `a.compose(b)` applies `b` first).
- `Meshed::attempts` is the history of failures that caused re-plans followed by the passed-over candidates of the final plan; `Plan::attempts` has only the latter (plus candidates skipped because of rejections).
- `NodeGrid::position` searches the boundary ring first and then the interior instead of keeping a sorted boundary index, so `NodeGrid` stays constructible by struct literal. `line_from` is O(ni + nj).
- `MeshingProblem` and `Plan` keep lazily built lookup tables that are skipped by serde and equality. Mutating the public tables by hand after a lookup leaves them stale; the driver never does.
- With impositions present, an auto list does not hide the registered schemes that are not in it (a narrow list must not make a parent's requirement unsatisfiable).
- Generation-time blame does not look at firmness (planning-time blame never blames a hard intent). A hard choice that fails generation is rejected and the re-plan ends in `NoApplicableScheme` with the failure in its attempts.
- Planning builds copy classes from scratch after every requirement that adds copies; this is O(copies) each time. Fine for decks and sweeps, to be revisited if a scheme adds thousands of copies one by one.
- Volume copies and a boundary-layer scheme are out of v1. Copied nodes get `Param::None`; a later pass may re-project.
- The reference interval solver is not an optimal IIA: greedy repair with a step limit. It is exact for the structured rows used here; an ILP or network-flow solver can plug in behind `IntervalSolver`.
- The face-copy vertex correspondence is geometric (`transform(p)` within the summed vertex tolerances) unless `vertex_map` is given. A closed edge matches by its ends only. The edge correspondence is one-to-one: two source edges with one image (two edges sharing their ends against a single target edge) are a `Correspondence` error (W0f).
- Sizing: `SizingSpec::Function` without `Driver::size` is `SchemeError::Input("sizing function '..' needs a SizeField")` in planning and generation (`sizing::field_for`), never a silent fallback to one interval per edge; a non-positive constant size is an error too (W0f).
- `NodeGrid` of a copy member follows the image of the root's layout (keys mapped, `ni`/`nj` and order kept), not the member's own logical side 0; documented on `NodeGrid`. The existing consumer (`line_from`) is key-based.
- Test stubs are stubs: `sweep` finds columns by straight continuation and handles prismatic stacks only; `submap` meshes layouts with exactly four logical sides (the corner-override test generates a face whose straight-through vertex sits inside a logical side). `groups.rs` has a multisweep through three stacked volumes (claims, interface faces, the path sum, verification). **`many_to_one.rs` (W0f, W0e decision 6)** is the real many-to-one sweep: two source faces of different densities (2 x 2 and 1 x 2 quads) swept into one six-sided target face, which the `m2one` test scheme claims and writes as the projection of both sources (boundary columns from vertical edges and linking-face grids, interior columns, including the edge where the sources meet, through the volume onto target-face nodes); 18 hexes, analytic volume, 6 target quads, `validate` clean, deterministic.
- `Cargo.lock` gained the `sm-op-api` edge for this crate (written by cargo).

## Answers to the open questions (recommendation section 6)

1. **Cut faces of pieces.** The contract does not change. `PieceResolver` is the seam: `WholeFaces` refuses cut faces with a typed error, `DecompositionFaces` (W0f) resolves them through the overlay.
2. **Submap side directions.** Driver-owned, in `FaceLayout` (`LogicalSide::dir`). Many-to-one layer surfaces stay scheme-private (`TargetPlan::data`).
3. **Rotational sweeps, morph copies, volume copies.** Copies stay rigid (one `sm_core::Transform`; a rotation is one). Morphing is a sweep generation detail. Volume copies are model-layer instancing, not schemes.
4. **Re-plan granularity.** Full deterministic re-plan; `plan_with(rejections)` leaves room for an incremental re-solve behind the same API.
5. **Boundary layers.** `MeshConstraint::BoundaryLayer`; only `Caps::BOUNDARY_LAYER` schemes may honour it; deck faces forbid it.
6. **Deck pair where B is the sweep source.** Copy classes (D2); tested in both directions.
7. **Capabilities or names.** Capabilities by default; names for "exactly this algorithm". A scheme crate must never list sibling scheme names to express a property.
8. **Params and replay.** `serde_json::Value` checked through the Op codec; `Rejections` stored in the `Plan`.
9. **Surface-only meshes.** Done in W0f: dimension-2 meshes in `sm-mesh-api`, face selections and dimension-2 commits here.
10. **A rayon executor.** Lives in the mesher crate and implements `Executor`.

## API change requests

All four requests were accepted (W0e decisions 1 to 4) and applied in W0f: `Classification::dim` accepts the `Partition*` kinds; `Model::split_realized` + `Decomposition::piece_face_cells` realize and resolve cut faces (the overlay mints the cut cells, the piece ids come from the model); `schemars` features on `sm-topo-api` and `sm-model-api` derive `JsonSchema` on `Side`, `CoedgeKey`, `CornerKey`, `PieceRef`, `PieceFace`, `CutId`, `Keep`; the duplicate transform algebra is gone. This crate does not yet derive `JsonSchema` on `MeshIntent` / `Plan` themselves (no `schemars` feature here); that is the MCP layer's step now that the leaf types have schemas.

## W0f gaps4

- **`Requirements::reads(other) -> ConstraintId`.** A scheme may declare that its generation
  reads the patch of a target that is not one of its sub-entities (curve `copy` by percent or
  auto, PARITY C P1, as an ordinary scheme). It records a scheme-origin
  `MeshConstraint::Precedes { before: other, after: self }`, so `GenCtx::patches.get(other)` is
  guaranteed during generation. Refused (`REFUSED`, `BadRequirement` when `requirements`
  returns) when `other` is not a target of the meshed region, is the requester, one of its
  ancestors (the requester is in `other`'s sub-entities) or a child the requester claimed.
  `constraint(Precedes)` keeps its old child-only rule. The planner's `Scope` (crate-private)
  now carries region membership and a sub-entity predicate (`children_in`, factored out of
  `children_of`). No public type changes.
- **Blamed read cycles.** A schedule `dependency cycle` whose involved constraints include a
  scheme-origin `Precedes` from a blameable `(target, scheme)` (not hard, not `copy`, not
  already rejected) is reported as `Infeasible { reason: "dependency cycle through a scheme
  read", blame: Some(reader) }` (the later such constraint), so the re-plan loop rejects that
  scheme and falls back; otherwise it stays `SchemeError::Schedule`. Tested: a sibling read
  orders the waves and finds the source patch; self, parent and out-of-region reads are refused;
  a soft two-curve read cycle falls back, a hard one stays a schedule error.

## W0g freeze rules

- **Context structs are `#[non_exhaustive]` with constructors**, so scheme-crate tests and
  sm-engine can build them while fields can still be added (a `control` field on `GenCtx` is
  then additive): `MeshInput::new(topo, model, pieces, selection)`, `PlanCtx::new(input,
  intent)`, `IntervalCtx::new(input, layouts, roles, targets)`, `GenCtx::new(input, plan,
  patches, size)`; `DriverOptions::new()` / `Default` with `with_max_replans`,
  `with_classify_edges`, `with_copy_tolerance`.
- **Intent structs are `#[non_exhaustive]`**: `MeshIntent`, `EntityIntent`, `Defaults`,
  `AutoPolicy`, `MeshGroup`, `TransformIntent`, `BoundaryLayerIntent`, `SchemeChoice`,
  `CornerIntent`. Constructors: `SchemeChoice::new` / `with_params`,
  `BoundaryLayerIntent::new`, `MeshIntent::new()`, `EntityIntent::new()` (+ `with_scheme`,
  `with_fallbacks`, `with_intervals`, `with_boundary_layer`), `Defaults::new()` (+
  `with_sizing`, `with_auto`), `AutoPolicy::new(curve, surface, volume)`, `MeshGroup::new(id,
  members)` (+ `with_scheme`, `with_fallbacks`), `TransformIntent::new(id, targets,
  transform)`, `CornerIntent::new(corner, vertex_type)`. Fields stay `pub`.
- **Every field added to an intent or plan struct after the freeze must be `#[serde(default)]`**
  (and omitted when default where the struct already does that), so stored intents and plans
  still read. Existing optional fields keep their `serde(default)`.
- **Implementer-built results are `#[non_exhaustive]` with constructors** (they are serialized
  into `Plan`): `GenerateFailure::new` / `fallback` / `fatal`, `Infeasible::new(reason,
  constraints, entities)` (sorts and dedups; the driver fills `blame`) and `Attempt::new(target,
  scheme, phase, kind, message)`.
- **W0g r2: the other implementer-built values are `#[non_exhaustive]` too.** `Patch` (the
  return value of `Scheme::generate`) is built with `Patch::new(target)` plus `pub` field
  assignment, or with `PatchBuilder`. `PatchNode::new(key, point, param)`. `FaceLayout` (the
  return value of `CornerPicker::pick`, serialized into `Plan`) is built with
  `FaceLayout::new(face, rule, corners, sides)`, `LogicalSide::new(coedges, dir)` and (W0g r3)
  `CornerAssignment::new(corner, vertex_type, firmness, angle)`, which every external
  `CornerPicker` needs to fill `FaceLayout::corners`. Fields stay `pub`. **Any field added later
  to `FaceLayout`, `LogicalSide` or `CornerAssignment` must be `#[serde(default)]`**, so stored
  plans still read.
- **W0g r3: `CopyConstraint` is `#[non_exhaustive]`.** Implementers pass it by value to
  `Requirements::copy` and it is persisted in `Plan.problem.copies`. Build it with
  `CopyConstraint::new(source, target, transform, reason)` (empty `vertex_map`: derived from
  `transform`) plus `with_vertex_map(pairs)` (sorted by source). Fields stay `pub`. **Any field
  added later must be `#[serde(default)]`.**
- **`#[non_exhaustive]` enums**: `FailureKind`, `Phase`, `SchemeError` (now like every other
  contract error enum), `Target`, `Job`, `IntervalRequest`, `SizingSpec`, `Origin`,
  `IntervalConstraint`, `Need`, `Role`, `CopyReason`, `Applicability`, `ElemShape`,
  `VertexRule`. Serde shapes are unchanged. `FailureKind::Cancelled` can come later as an
  additive variant. Of the struct variants only `Applicability::Applicable` / `NotApplicable`
  are `#[non_exhaustive]` (build them with `Applicability::yes` / `no`): the others
  (`IntervalConstraint::*`, `Role::*`, `Origin::*`, `CopyReason::*`, `SchemeError::*`) are
  built by scheme implementations and users with literal syntax and have no constructors, so
  marking them would break every implementer for no expected growth; a new field there would be
  a new variant instead.
