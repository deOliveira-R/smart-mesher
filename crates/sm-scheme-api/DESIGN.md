# sm-scheme-api: design (panel candidate "transaction")

The contract between the meshing driver and every meshing scheme (curve, surface, volume, transform)
and the sizing layer. Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-model-api`,
`sm-mesh-api`, `thiserror`, `smallvec`; `serde` behind the `serde` feature. No rayon (W0c
decision 8): parallelism goes through an `Executor` trait.

## 1. Positioning

> Similar to **Cubit's scheme / interval-assignment model** (per-entity schemes with
> firmness, vertex types, a global integer interval solve before meshing, auto selection, schemes
> that set up their children) and to **Gmsh's transfinite/recombine pipeline** (constraints such as
> `Transfinite Curve/Surface/Volume` are declared first, the mesher then runs bottom-up by
> dimension). It differs in that **the unit of work is a transaction over a set of entities**, a
> `MeshJob` with a declared read/write `Footprint` that produces a `MeshPatch` delta committed
> all-or-nothing in a fixed order, and in that nodes are **named by their owning entity**
> (`NodeKey { owner, local }`), so separately meshed entities are conformal by construction and
> classification comes for free. Copy/mirror (deck pairs) are first-class job kinds, not a
> post-processing step. This gives: parallel meshing of independent entities with bit-identical
> results for any thread count; atomic multisweep / merged-volume / THex-style operations; a
> failed scheme that leaves no trace (no partial mesh, no shifted ids) so the driver can fall
> back; and an intent that is plain serde data, so it lives in an Op and in git.

Cubit runs the dimension hierarchy entity by entity and mutates the mesh as it goes; a multi-volume
operation is a special call. Gmsh's transfinite constraints are global state on the model. Here,
both are the same thing: a job whose footprint covers several entities.

## 2. The pipeline

```
MeshIntent (serde) ──► MeshProblem::build ──► solve (IntervalSolver) ──► schedule ──► execute(Executor) ──► Ledger ──► write_mesh(MeshBuild)
                         plans, impositions,      one global IIA            waves by       waves; commit in      3D cells +
                         constraints, jobs                                  footprints     job-id order          classification
```

1. **Build** (`MeshProblem::build`). Closure of the requested targets (volume → faces → edges →
   vertices; pieces through `Model::decomposition().piece_faces`). Volume and group jobs are
   planned first: they claim what they write (a sweep writes its target and linking faces) and
   impose requirements on the rest. Model deck rules are added (§3.6). Remaining faces get surface
   jobs (or copy jobs), remaining edges curve jobs. User intervals and sizes become goals. Vertex
   types are merged by firmness. Jobs get ids by sorting on (transform last, first write target).
2. **Solve**: all constraints go to one `IntervalSolver` over all edges of the closure.
3. **Schedule**: job *j* depends on the writer of each target it reads; waves are dependency
   levels (Kahn with an ordered ready set). Two writers of one target, a read with no writer
   (except model vertices, which the driver seeds) and cycles are errors.
4. **Execute**: each wave runs through the `Executor` against the ledger as of the previous wave;
   results come back in index order and are committed in job-id order.
5. **Write**: `Ledger::write_mesh` adds vertices in `NodeKey` order, cells per target, and
   classifies faces from committed face meshes.
6. **Fallback** (`run`): an execution failure with `Retry::NextInChain` records the scheme in
   `MeshIntent::excluded` for that target and starts over (re-plan + re-solve, because the next
   scheme brings different constraints). Plan-time failures fall through the candidate list
   directly. The final intent, exclusions included, is returned so a replay is exact.

## 3. Brief → types

| Brief item | Types / functions |
|---|---|
| Scheme families (curve, surface, volume, transform) | one `Scheme` trait with `dim() -> SchemeDim {Curve, Surface, Volume, Transform}`; `SchemeRegistry` (unique names, `BTreeMap`) |
| Vertex types end/side/corner/reversal, per surface | `VertexType` (+ `quarter_turns`, `from_angle`), `VertexTypeSetting { ty, firmness }` keyed by `CornerKey` (a vertex *use*, so a vertex can differ per face and a seam vertex twice in a loop); intent holds hard ones, `JobPlan::vertex_types` soft picks; merged by firmness, equal-firmness disagreement is `DriverError::Conflict` |
| Corner picking, logical sides | `vertex::{loop_corners, logical_sides, turn_sum, chain_nodes}`: shared interpretation of types. Picking itself is an `sm-sizing` service; schemes may also pick (the stub map does, from angles) |
| Interval constraints: hard/soft counts, side sums, even sums, sweep matching | `IntervalConstraint { kind: Linear{terms, rel, rhs} \| Even{edges} \| Goal{edge, count}, firmness, origin }`, `Origin` (user, default size, scheme, deck) for diagnostics |
| Global interval matching | `IntervalSolver` trait (implemented in `sm-sizing`), `IntervalSolution`; `ReferenceIntervalSolver` (union-find + rounding + parity repair + full check) as oracle |
| Schemes imposing on lower dims (map volume → map faces; sweep roles) | `JobPlan::impositions: Vec<Imposition { target, requirement, firmness }>`, `Requirement::{Scheme, Role(SweepRole), NoBoundaryLayer, OnPlane, CopyOf}`; `JobPlan::roles` kept on the job; a scheme may also *write* children itself (sweep: target + linking) by listing them in `footprint.writes` |
| Fallback chains, auto selection | `SchemeChoice::{Fixed, Chain, Auto{candidates}}` with `Firmness`; `Applicability::{Applicable{score, reason}, NotApplicable{reason}}`; auto = score desc, then name; `Attempt` log; `run` loop |
| Atomic multi-volume (multisweep, merged clusters, THex) | `GroupIntent { kind: Multisweep \| Cluster, members, scheme }` → one job over all members, one patch; `TransformIntent` → `JobKind::Transform`, runs after the writers of every target it rewrites and replaces them in one commit |
| All-or-nothing commit | `Ledger::commit` checks the whole patch first; the run writes to `MeshBuild` only at the end, so a failed run never touches the database or its `SerialAlloc` |
| Deck copy constraints, no boundary layers on deck faces | driver reads `Model::face_boundary` / `BoundaryKind::Deck`: every deck face gets `NoBoundaryLayer` (Hard), mirror faces `OnPlane`, side B of a translation/rotation pair `CopyOf { source, transform }` plus Hard `Origin::Deck` equalities between mapped edges; B becomes a `JobKind::Copy(CopySpec)` job, or, if another job writes it (a sweep), a post-commit `verify_copy` check (`DriverError::DeckMismatch`) |
| Meshing pieces from the Decomposition layer | `Target::{Topo, Piece(PieceRef), PieceFace}`; `Target::piece_face` normalizes; closure of a `Cell` piece goes through `piece_faces`; a sweep reads `PieceShape::Sweep { source, target }` for its source/target; a `Whole` piece is meshed end to end in the tests (`whole_piece_target_meshes_like_its_volume`); cells on a `PartitionCell` are classified on it; faces between two pieces of one volume are classified on the volume (§5) |
| Inputs | `PlanCtx`/`ExecCtx`: `&dyn TopologyView` (an `Overlay` works the same way), `&Model`, `&MeshIntent`, `Option<&dyn SizeField>`, plus `&Ledger`, `&IntervalSolution`, `&VertexTypes` at execution |
| Output with classification | `MeshPatch` (columnar) → `Ledger` → `Ledger::write_mesh(&mut impl MeshBuild, &Model)`; node classification from `NodeKey::owner`, `Param::Curve(t)` kept on edge nodes |
| Sizing layer | `SizeField { size(p), metric(p) }`, `IntervalSpec::{Count, Size}`, `MeshIntent::default_size` |
| Serializable intent | `MeshIntent`, `TargetIntent`, `SchemeIntent`, `SchemeSpec { name, params: Params(BTreeMap<String, ParamValue>) }`, `GroupIntent`, `TransformIntent`, vertex types, exclusions: all serde (feature), non-string-key maps as pair lists |
| Typed errors with structured fallback | `SchemeFailure { scheme, targets, kind: FailureKind, retry: Retry::{NextInChain, Fatal}, message }`, `PatchError`, `DriverError` (thiserror) |

### Node naming (the conformity rule)

`NodeKey { owner: Target, local: u32 }`. A model vertex has one node (`local 0`, seeded by the
driver from the vertex carrier). An edge with *n* intervals owns interior nodes `1..n-1` counted
in the **edge's own direction**; any face reads them through `vertex::coedge_nodes`, which reverses
for a `Right` coedge. Face and volume interiors are numbered by their writer. Because every node
has exactly one owner and every owner exactly one writer job, two faces sharing an edge share its
nodes without welding, and the writer knows each node's classification without geometry.

### Commit rules (`Ledger::commit`)

Every part target and every new node's owner lies in `footprint.writes`; every non-vertex write
target gets a part; nothing is written twice (except by transform jobs, which replace); node keys
are unique, new nodes are new and finite, referenced nodes are committed; CSR offsets and indices
are well formed; cells have their shape's corner count; an edge mesh runs from its start vertex
node to its end vertex node. Checked before any mutation.

## 4. Determinism and parallelism

- **Ordering.** Every map is a `BTreeMap`/`BTreeSet`; `Target` and `NodeKey` have a total order;
  job ids sort jobs by (transform last, first write target); waves list ids ascending; the solver
  contract requires determinism; auto selection breaks score ties by name. No randomness.
- **Parallelism.** Jobs in one wave have disjoint writes and read only earlier waves, so they are
  independent by construction (`Footprint::independent`). The executor may run them in any order
  on any number of threads; it returns results by index and the driver commits in id order, so
  the ledger is identical for 1 and N threads. A scheme may parallelize internally only if its
  output is bit-identical (the contract on `Scheme::execute`).
- **Ids.** The ledger mints no mesh ids. `write_mesh` adds vertices in `NodeKey` order and cells in
  target order, then the caller's `MeshBuild::finish` assigns ids from the shared `SerialAlloc`
  (unchanged on error). A failed or fallen-back attempt therefore never shifts ids.
- **Tested** (`tests/brick.rs::determinism_runs_and_threads`): two sequential runs, a
  thread-per-job executor, and a run on a fresh topology built on another thread give equal
  `Ledger::hash`, equal problems and schedules, and equal `topology_hash`/`geometry_hash` of the
  finished mesh. API crates use `std::thread`, not rayon (decision 8).

## 5. The stub implementors (`tests/common/mod.rs`) and what the tests show

- `equal` (curve): equal parameter spacing (equal arc length on lines), `Param::Curve(t)`.
- `map` (surface): vertex types from intent or corner angles (soft), 4 logical sides, two Hard
  side-sum equalities, transfinite interpolation from the committed side nodes.
- `sweep` (volume, 1-to-1 translational): source/target from params or `PieceShape::Sweep`
  (the latter for `Cell` pieces, which the stub then rejects as `NotApplicable`: it meshes
  topology volumes and `Whole` pieces only, since `Cell` pieces with cut faces need overlay or
  `PartitionFace` ids, see §6);
  plans Hard layer equalities over the linking edges and target-edge = source-edge equalities,
  soft `End` types on the linking faces, a `Role(Source)` imposition; reads the source face and all
  edges, writes the volume, the target face and the linking faces; emits hexes.
- `broken` (surface, always fails) and `noop-transform` (transform) exercise fallback and
  transforms.

On the `sm-topo-api` brick fixture (2 × 1 × 1, size 0.5): 14 jobs in 3 waves (12 curves | map |
sweep), counts 4/2/2, 16 hexes of volume 0.125 each summing to 2.0 exactly, 45 vertices,
`sm_mesh_api::validate` clean, boundary faces classified on the right `TopoFace`s (4/4/8/8/8/8).
Also tested: auto fallback after an execution failure, chain fallback, hard interval conflict,
unknown/not-applicable schemes, a `Whole` piece target meshed and classified on its volume, a deck pair meshed as a copy job, a deck pair written by the sweep
and verified, a transform replacing the volume's cells, commit-rule rejections that leave the
ledger unchanged, schedule conflicts and cycles, and the reference solver's classes/goals/parity.

## 6. Deviations and limits (honest list)

- **One `Scheme` trait instead of `CurveScheme`/`SurfaceScheme`/`VolumeScheme`/`MeshTransform`**
  (ARCHITECTURE §5 sketch). The transaction model needs one plan/execute shape for jobs of any
  size; `SchemeDim` keeps the dimension. Schemes do not get a pre-discretized boundary loop as
  input: they read committed edge meshes from the ledger by `NodeKey`, which is what makes jobs
  composable (a sweep writes faces a map would otherwise write).
- **The reference driver, solver and copy execution live in this crate** (like `CellComplex` and
  `ReferenceBuilder` in theirs), so the contract is executable. `mesh-driver` may replace any
  phase; the phases are public.
- **Surface-only results do not reach `MeshBuild`**: it stores 3D cells only (a face without a
  cell is `DanglingFace`). Edge and face meshes stay in the `Ledger` (readable, hashable).
- **Interval variables are edges** (`TopoEdge` or overlay `PartitionEdge`). Edges of
  `PieceFace::Part`/`Cut` faces have no topology id, so pieces with virtual cut faces need the
  overlay (or `PartitionFace` ids from the model) to take part in the interval solve.
- **Cut faces** are normalized to one target per cut (`Keep::Negative`), and nodes on them have no
  classifiable owner (classified via the cell's volume at write time).
- **Copy jobs** handle faces and their edges; the deck verification compares quantized element
  centroids (a value right at a rounding boundary can give a false mismatch). Copy orientation
  follows the model's sign rule `n_B = -g(n_A)`.
- **Transforms** run after all writers and nobody may read their output except `write_mesh`.
- **Ledger storage** is `BTreeMap`-keyed (O(log n) per node at commit). Patches are columnar (CSR,
  `u32` local indices); the ledger layout is the place to optimize later (per-target arenas).
- `schemars` is not derived: `sm-model-api` and `sm-topo-api` have no `schemars` feature yet
  (see API requests).
- `Cargo.lock`: the build updates it; it is left out of the commit (the orchestrator owns it).
- `sm-testkit` does not exist yet; its property tests are not used.

## 7. Open questions

1. **Sweep vs separately meshed linking faces.** Here the sweep writes its linking faces (as
   Cubit does). If linking faces are meshed by their own map jobs, the sweep needs a structured
   grid record on face meshes (`(i, j)` per node) to find layer *k*. Add an optional
   `FaceMesh::grid` later?
2. **Many-to-one / many-to-many sweeps and multisweep groups** fit `GroupIntent` and multi-target
   jobs; the source-precedence rule (meshed, merged, specified) belongs to the sweep scheme's plan.
3. **Fallback granularity.** Re-planning the whole problem is simple and deterministic but costs a
   full re-solve. Incremental re-plan of the affected interval classes is possible once the solver
   reports classes.
4. **Soft linear constraints.** The reference solver treats every linear/parity constraint as
   hard and uses firmness only for goals. `sm-sizing` should define what a soft equality means
   (penalty weight?).
5. **Boundary layers** are `JobKind::Transform` jobs that must honour `Requirement::NoBoundaryLayer`;
   whether they also shift the interval solve (first-layer thickness as a constraint) is open.
6. **Curved faces.** The stub map interpolates in space; real map/submap must project onto the
   carrier (`ParametricSurface::project`) and fill `Param::Surface`.
7. **Polyhedral cells** (`CellShape::Polyhedron`) need face loops in the patch, not only corners;
   a `cell_faces` CSR column can be added to `MeshPatch` without changing jobs.
