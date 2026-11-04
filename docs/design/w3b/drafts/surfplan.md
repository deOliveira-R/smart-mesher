> Planning draft (2026-11-04), kept for reference by `../plan.md`. Where it differs from `../plan.md` or `../rulings.md`, those win.

### W3b-SURFPLAN. `sm-mesher` (L3, exists): surface-only `mesh.run` and the `mesh.plan` explain query

- **Deps:** no new internal crate. Existing normal deps stay (sm-core, sm-topo-api, sm-model-api, sm-mesh-api, sm-mesh, sm-scheme-api, sm-op-api, sm-layer-api). sm-mesher still names no scheme crate and no sizing crate. Dev-only: sm-testkit and `sm-scheme-api[stubs]` as today. Tests use the stub registry (`map`, `submap`, `pave`, `tri`, `sweep`, ...). Acceptance against the real scheme crates is wave integration (W3b-INT, `sm-cli` tests), not this task.
- **Contract items used by name (batch already running, not requested here):** `Driver::mesh_from_plan` (plan once), `SchemeNote` / `Patch::notes` (sm-diag), the firmness cascade, scheme-derived interval counts, bulk mesh commit, `MeshEntry.derived_from`. This task starts after that batch is merged. Where a signature is needed, take it from main, not from this text.
- **Already on main (do not rebuild):** the driver commits a face-only selection into a dimension-2 builder (`MeshBuilder::surface()`; `MeshInput::selection` of `TopoFace`/`PartitionFace` ids; `Driver::commit` dim-2 branch; tests `sm-scheme-api/tests/surface_only.rs`). `sm_mesh_api::validate` and `cell_volume` already work for `MeshDim::Two`. `ElementType::{Quad4, Tri3, Polygon}` exist in sm-fem. The gap is `mesh.run` (volumes only) and the absence of any query that says which scheme ran.

#### Scope A. `mesh.run` accepts faces (Cubit `mesh surface`)

- `MeshRun.targets` takes `TopoVolume` ids **or** `TopoFace` ids (`x-entity-kind = ["TopoVolume","TopoFace"]`), never both. A mix is `OpError::Invalid("mesh.run takes volumes or faces, not both; run it twice")`. The op name, the `mesh` output role and the argument name are unchanged, so recorded scripts and W2/W3a hashes are unchanged.
- Face path: same sequence as the volume path (guards, `Driver::plan`, budget check, then `mesh_from_plan` with `MeshBuilder::surface()`, `insert_mesh`). The entry is `MeshEntry::new(.., selection = the faces, ..)`, so `mesh.delete {targets: [faces]}`, staleness and `mesh_of(face)` work with no layer change.
- Guards (typed `OpError::Invalid`, state untouched, message names the ids and the fix):
  1. A face already in a mesh entry, or whose coboundary volume is meshed: "TopoFace#5 is already meshed (Mesh#1)" / "bounds TopoVolume#1, already meshed; mesh.delete first".
  2. A face that shares an edge with a face meshed in another entry: refused ("mesh them together"). Sequential meshing against neighbours stays a backlog item; the message says so.
  3. The volume path gets the symmetric guard: a volume with a face that is in a surface mesh is refused ("TopoFace#5 is in Mesh#2; mesh.delete first").
  4. Faces of a dead or hidden host cell are refused by the existing `require` / `live_targets` rule (extend its kind list only).
- Cell budget: `estimate_cells` gets a face branch, `sum over faces of (geometric mean of the solved counts of the face's edges)^2`. Same doc caveat (guard against runaway requests, not a forecast). Volume estimate unchanged.
- Scheme choice for faces is whatever the registry's surface auto list and the intent give (`mesh.scheme` on a face already works). This task does not change auto scoring (that is W3b-AUTO).
- Output element types: QUAD4 for quad schemes, TRI3 for tri schemes, `Polygon` for mixed or poly. No new `ElementType`. A `fem.block_element_type` of `Quad4`/`Tri3` on a surface mesh is covered by a test, not changed.

#### Scope B. `mesh.plan` query (explain)

- `mesh.plan {mesh?: Mesh, targets?: [TopoVolume|TopoFace], attempts?: bool = true}`; exactly one of `mesh` and `targets` (else `Invalid`). Registered as a query of `MeshPlugin` next to `mesh.info`; read-only, no mutation, no id minted.
  - `mesh`: report **what ran**, from `MeshEntry.plan` (`status: "ran"`, `stale` copied). A mesh with no stored plan (imported) is `Invalid("mesh has no plan")`.
  - `targets`: report **what will run**: build the same `MeshInput`, intent, size field and driver as `mesh.run` and call `Driver::plan` (`status: "will_run"`). No generation. A planning failure is returned as `status: "infeasible"` with the `SchemeError` text and its blame, not as a query error, so an agent can read it. Guard conflicts of Scope A are returned in `blockers: [String]` (the same texts), not as errors.
- Output `PlanReport` (serde + schemars, `deny_unknown_fields` on input, stable field order):
  - `status`, `mesh?`, `stale`, `dim` (2 or 3), `selection`, `estimated_cells` (will_run only), `waves` (count), `plan_hash` (16 hex digits of a stable hash of the serialized plan, fixed byte order), `blockers`.
  - `entities: [EntityPlan]`, ascending by target (curves, surfaces, volumes, groups):
    `{ target, kind, scheme, how, score?, reason, params, caps: ["tri"|"quad"|"structured"|...], claimed_by?, copy_of?, elements?, cells?, alternatives }`.
    - `how` is a stable string from `Selection` (`intent#rank`, `registry#rank`, `piece_shape`, `imposed`, `auto`, `claimed`, `copy`), and `score` is its score when it has one.
    - `reason` is `TargetPlan.reason` verbatim (the winner's applicability text).
    - `alternatives` are the `plan.attempts` and `plan.rejections` rows of that target: `{scheme, phase, kind, message}`, in evaluation order (what lost, and why).
    - `elements` / `cells` for `ran` only: cells per shape classified on the entity (`MeshRead::classified_on`), `{"Quad": 6}`. For `will_run`, `elements` is from `caps` (`"QUAD4"`, `"TRI3"`, `"mixed"`) and `cells` is absent.
  - `intervals: [{edge, count, why, requests}]`, ascending by edge. `why` is one of `hard_intent`, `scheme` (names scheme and target from `Origin::Scheme`), `deck`, `sizing_goal` (with the goal), `default`, `derived` (solved from linear rows, listing the constraint ids). It is computed in sm-mesher from the public `MeshingProblem.intervals` records and their `Origin`. When two sources disagree on the solved count, the report lists both and marks the winner (so "why 3 and not 5" is answerable).
  - `notes: [SchemeNote]` for `ran` when the entry carries them (see ACR-1), else empty.
- No clock, no thread dependence: the report is a pure function of `(state, request)` and byte-identical for 1 and N threads.

#### Public API (rustdoc on all)
- `pub struct MeshPlanQuery { pub mesh: Option<EntityId>, pub targets: Option<Vec<EntityId>>, pub attempts: bool }` (`TypedQuery`, NAME `"mesh.plan"`).
- `pub struct PlanReport`, `pub struct EntityPlan`, `pub struct IntervalWhy`, `pub enum PlanStatus { Ran, WillRun, Infeasible }`, `pub enum PlanDim`.
- `pub fn explain_plan(plan: &Plan, view: &dyn TopologyView, mesh: Option<&dyn MeshRead>) -> PlanReport`: pure, reusable by sm-diag suggestions and the CLI pretty printer.
- `MeshRun` keeps its shape (field `targets` only).

#### Acceptance (analytic; brick fixtures through the `common` harness, stub schemes, size 1)
1. Brick 2x3x4, `mesh.size 1`, `mesh.run {targets:[TopoFace#5]}` (the 2x3 face): 6 QUAD cells, 12 vertices, one `Mesh`, `dim == 2`, `selection == [TopoFace#5]`, total `mesh.info.volume` (area) 6 within 1e-12, `sm_mesh_api::validate` empty, boundary facets `2*(2+3) = 10`, no volume mesh cell, `MeshVolume` counts untouched. Every cell is classified on `TopoFace#5`.
2. All six faces in one run: 52 quads (`2*(6+8+12)`), closed surface, `V - F + C = 2`, area 52 within 1e-12, `validate` empty. Face normals agree across shared edges (the driver's orientation rule).
3. A tri scheme on one face (`mesh.scheme {targets:[face], scheme:"tri"}`): TRI cells only, area equals the face area to 1e-12 and cell shapes are `Tri`. `mesh.info.cell_shapes == {"Tri": n}`.
4. Guards: faces + volume mixed is `Invalid`; a face twice meshed, a face of a meshed volume, a volume with a meshed face, and an edge-neighbour face in another entry are each `Invalid` with the documented text. After every refusal `state.hash()`, layer hashes, entity count and `peek_next` of every Mesh kind are identical to before (atomicity, as `infeasible_hard_intent_fails_atomically`).
5. `mesh.delete {targets:[face]}` retires every id (live entity counts return to the pre-run values). Deleting the face's volume marks the entry `stale`. A hard `mesh.intervals {count:3}` on an edge of the face is honored exactly (3 intervals on that edge, the opposite edge too by the map rows).
6. `mesh.plan {mesh}` after acceptance-1: `status ran`; the face entity row has `scheme == "map"`, `how == "auto"` or the intent rank, `elements == {"Quad": 6}`, the 4 edge rows have `scheme == "equal"`, intervals `[2,3,2,3]` with `why = sizing_goal` (or `hard_intent` when set by `mesh.intervals`), `plan_hash` equal to `plan_hash` of `mesh.plan {targets}` taken before the run on the same state (plan and replay agree).
7. `mesh.plan {targets:[TopoVolume#1]}` with a hard sweep scheme: volume row `how == "intent#0"`; claimed children carry `claimed_by`; copy-class members carry `copy_of`. With `AlwaysFail` registered and the fallback chain, the failed scheme appears in `alternatives` with `phase == Generate`, and the recovered plan's `rejections` row is reported on the mesh (reuse the `failing_scheme_replans...` fixture).
8. Infeasible intent (3 vs 5 on opposite edges) via `mesh.plan {targets}` returns `status infeasible`, the `SchemeError` text and the blamed constraint origin; it does not error, and the state hash is unchanged (query).
9. A `mesh.plan` of a mesh with an edge count forced by two sources lists both requests and marks the winner (`hard_intent` over `sizing_goal`).
10. Blockers: `mesh.plan {targets:[meshed face]}` returns `status will_run`, `blockers` non-empty with the guard text, while `mesh.run` on the same input fails identically.
11. Determinism: `sm_testkit::determinism::check` over (acceptance 1, 3, 6) comparing layer hashes, 1 vs N threads, and the serialized `PlanReport` bytes. Cold replay of the recorded script (size, scheme, run on faces) from the root reproduces the state hash.
12. Schema: `mesh.run` and `mesh.plan` schemas list `TopoFace` in `x-entity-kind`; CLI/MCP discovery shows both (golden in W3b-INT, not here). Property test (`sm_testkit::strategies::config(n)`, brick specs 1..6 integer sides, size 1): for a random face the surface mesh area equals the face area, cells equal `a*b`, and `explain_plan` interval counts equal the face edge lengths.

#### Must NOT
- Edit another crate, a frozen contract, the root Cargo.toml, CI or docs outside `crates/sm-mesher/`; add a dependency on a scheme crate, sm-fem, sm-diag or sm-meshio.
- Change planner scoring, the Surface auto list, or W2/W3a hashes (no auto selection here).
- Plan a second time inside `mesh.run` (use `mesh_from_plan`); mutate state in a query; iterate a `HashMap` into output; use time or unseeded randomness; use `unwrap` on input paths (typed `OpError`, thiserror for new errors).
- Silently let a face mesh coexist with a volume mesh on the same face (no duplicated classification).
- Gate on wall-clock; performance is recorded by a release bench (`mesh.run` on 10^6 faces cells, plan-once vs double plan), not asserted.
- Treat an external tool as the judge of the mesh: `sm_mesh_api::validate` decides.
- Write `DESIGN.md` update (guards, explain schema, `why` rules) and `W3B_REPORT.md`.

#### Deferred
Sequential meshing against meshed neighbours (face next to a meshed face, volume over meshed faces); `mesh.plan` caching in `DerivedCache`; `mesh.plan` for partial selections of a decomposition's pieces beyond whole/cell ids (works, untested); a CLI pretty table (W3b-INT); explain of generation-time scheme notes before `mesh.run` (only `ran` has them).
