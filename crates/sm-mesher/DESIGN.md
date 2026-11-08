# sm-mesher

The intent and meshes plugin and the mesh-driver node (W2 task T7).

## Positioning

Similar to Cubit's meshing command layer (`size`, `interval`, `scheme`, `mesh volume`), this
crate records meshing requests with Cubit's firmness ladder and then meshes bottom-up. It differs
in that requests are **ops on a replayable timeline**, not mutable attributes of live geometry:
`mesh.size` / `mesh.intervals` / `mesh.scheme` write a hashed `intent` layer, and `mesh.run` is a
pure function of `(model, intent, injected schemes)` that adopts every mesh entity id into the
shared entity table. That gives a meshing session that is atomic per op (a failed `mesh.run`
changes nothing, not even an id counter), undoable, forkable and bit-reproducible with 1 or N
threads, which Cubit's journal replay is not. The crate is only a thin node: planning, matching,
generation and commit are the `sm-scheme-api` `Driver`, and schemes, solver, picker and size
fields arrive through the `MesherConfig` service, so this crate names no scheme crate.

## Contents

| Item | Role |
|---|---|
| `MeshPlugin` | registers `IntentLayer`, ops and the query. Does **not** register `MeshLayer` (that is `sm_layer_api::CoreLayers`). Install after `CoreLayers`. |
| `MesherConfig` | the service (`Services::with_versioned("mesher", v, cfg)`): `Arc<SchemeRegistry>`, `IntervalSolver`, `CornerPicker`, optional `SizeFieldFn` and `AutoSizeFn`. Defaults to the reference solver and picker. |
| `IntentLayer` | `intent` layer, `DEPENDS_ON [model]`, wraps `sm_scheme_api::MeshIntent`. |
| `RayonExec` | `Executor` over the global rayon pool, one job per task. |
| `mesh.size`, `mesh.intervals`, `mesh.scheme`, `mesh.default_size` | edit the intent. |
| `mesh.run`, `mesh.delete` | the driver node; `insert_mesh` / `retire_mesh` of `sm-layer-api`. |
| `mesh.info` | query: counts by shape, topology hash, quantized geometry hash, total volume, `stale`. |
| `mesh.plan` | query: explain what ran (`mesh`) or will run (`targets`): scheme per entity and why, interval counts and their source, `plan_hash`, `blockers`. `explain_plan` is the pure core. |

## IntentLayer

* Keyed by entity id in a `BTreeMap`, so requests on different entities commute: the state hash
  does not depend on the order they were made in. A repeated request on one entity **replaces**
  the old one (`Option` slots, not a list); it never accumulates.
* Digest: `Digest::value(&intent)`, the canonical encoding. Floats in it are user input and are
  unquantized. `mesh.size {auto}` stores a *computed* float (the hook's result), so state hashes
  are bit-identical on one build but may differ across platforms if the hook is not exactly
  reproducible. The plan accepts this; the hook is injected and versioned through the service.
* `on_retired` drops rows on retired entities, corner overrides on retired faces, retired group
  members and transform targets (empty groups and transforms go) and interval rows naming a
  retired edge. `references_any` is the exact predicate of "would change" (tested against
  `on_retired` over all 512 subsets of a sample).
* Not implemented: `on_succession` (a split volume loses its requests; the user re-requests).

## Ops

### Firmness (Cubit's ladder, with our defaults)

| Op | Default firmness |
|---|---|
| `mesh.size` | `default`: the size is a real goal `length / size` that the interval solver approaches while matching opposite sides. `soft` / `hard` freeze the count `max(1, round(length / size))`. |
| `mesh.intervals` | `hard` on a curve; `soft` on a face or volume (applies to the owned curves) |
| `mesh.scheme` | `hard` (the user named it); fallbacks are `soft` and carry no parameters |

*Deviation from Cubit's manual*: a count derived from a size is *soft* in Cubit. We default to a
`default`-firmness goal instead, which lets the solver minimize deviation instead of picking an
arbitrary soft count; `firmness: "soft"` gives the Cubit reading. In the code `Default < Soft`,
so for the "firmer request wins" check any `default`-firmness `mesh.size` (implicit or an explicit `"default"`) is *compared* at the
`soft` level (it is still stored as `default`): a later `mesh.size` replaces an earlier soft
`mesh.intervals` on the same entity (the most recent command wins at the same level), and a
`hard` request is still kept. A request never overwrites a firmer one on the same entity: the op keeps the
firmer request and emits a `mesh.firmness_kept` warning. Requests cascade volume, face, edge
inside the planner.

*Cascade across entities (ruling P1).* `MesherConfig::cascade` is `RequestCascade::Firmness`
(Cubit's rule): where requests from an edge, a face and a volume reach the same edge, the firmer
wins; at equal firmness the latest command wins. The stored intent stays order-free: the planner
needs only "firmer wins, then edge over face over volume, then lowest id", and `mesh.size` /
`mesh.intervals` resolve the "latest command" part when they run, by writing the new request on
each edge where an equally firm, different request from another entity would otherwise beat it
(an identical request writes nothing). The plan therefore does not depend on the command history
and survives a round trip through serde. `RequestCascade::Level` (the original rule: an edge
request beats a face request beats a volume request whatever their firmness) stays available via
`MesherConfig::with_cascade`. The default was flipped to `Firmness` after an A/B of the W2
milestone, `p0_rows` and `milestone_w3a`: every intent layer hash and every `mesh.run` topology
hash and cell count was identical under both.

Limits of "order-free" (known, latent). (1) The written rows live on the edges that exist at
command time. If a later topology op (such as webcut) deletes and replaces those edges, the rows
are dropped (intent_layer retain) and the planner's `(firmness, intervals_seq)` comparison
decides again, so the outcome depends on command order. A serde-loaded intent, where every seq is
0, would then plan face-over-volume where a live session plans latest-wins. No production path
deserializes `IntentLayer` today. (2) `resolve_latest_wins` compares firmness as stored, so a
default-firmness `mesh.size` (which `set_intervals` treats as competing at soft) never overrides
an older soft request on another entity; this matches the planner but not the "any interval
command raises the curve to at least soft" note.

`auto` (1..=10) is resolved at apply time through the `AutoSizeFn` hook and recorded as a plain
size, so replay never runs the hook again.

### mesh.run

1. Resolve targets: live `TopoVolume`s **or** `TopoFace`s, sorted, de-duplicated; a mix is
   `Invalid("mesh.run takes volumes or faces, not both; run it twice")`. Run the guards (below).
2. Clone the read layers (`Arc` clones) and the allocator; build the size field from
   `defaults.sizing` through the hook if there is one.
3. `Driver::plan` once, then `Driver::mesh_from_plan(&input, &intent, plan, &RayonExec,
   builder, &mut alloc_copy)` with `MeshBuilder::new()` (volumes) or `MeshBuilder::surface()`
   (faces). Re-plans around failing schemes happen inside the driver.
4. `insert_mesh(ctx, entry, &alloc0, alloc, "mesh")` adopts the minted ids (one range per kind,
   `Mesh` first) and inserts the entry; its selection is the targets. Output role `mesh`
   (cardinality one). The op name, role and argument are the same for faces, so recorded
   scripts and hashes are unchanged.

#### Surface-only runs (faces)

A face run (Cubit `mesh surface`) makes a dimension-2 mesh: QUAD4 for quad schemes, TRI3 for tri
schemes, polygons for mixed. `mesh.delete {targets: [faces]}`, staleness and `mesh_of(face)` work
through the entry selection with no layer change. The scheme of a face is whatever the registry
and the intent give (`mesh.scheme` on a face works as for a volume).

Guards (all `OpError::Invalid`, state untouched; `mesh.run` fails with the first, `mesh.plan`
lists all as `blockers`):

| Case | Text |
|---|---|
| face already in an entry | `TopoFace#5 is already meshed (Mesh#1); mesh.delete first` |
| face of a meshed volume | `TopoFace#5 bounds TopoVolume#1, already meshed (Mesh#1); mesh.delete first` |
| face sharing an edge with a face meshed in another entry (selected, or of a meshed volume), not in `targets` (Q13) | `TopoFace#1 shares TopoEdge#n with TopoFace#5, meshed in Mesh#1; mesh them together in one mesh.run, or mesh.delete Mesh#1 first` |
| volume with a face in a surface mesh | `TopoVolume#1 has a face in a surface mesh: TopoFace#5 is in Mesh#1; mesh.delete first` |

Sequential meshing against meshed neighbours stays a backlog item. Faces of a dead host cell are
refused by `live_targets`.

**A request on the volume does not reach a face run.** The planner puts a volume's size or count
on the edges of the volumes in the run, so with faces only it is ignored (the edges fall back to
the built-in default goal, one interval per unit size). Set `mesh.size` / `mesh.intervals` on the
faces or their edges. `mesh.run` emits the warning `mesh.volume_request_ignored` (not hashed)
when a target face has no request of its own but a bounding volume has one, and `mesh.plan`
shows `why: default` for such edges.

Cell budget for faces: the driver's estimate counts volumes, so for a face run the driver budget
is off and `mesh.run` checks `sum over faces of (geometric mean of the face's edge counts)^2`
against `max_cells` after the first plan (same `OverBudget` text). Re-plans after a failure are
not budget-checked for faces (the driver's budget is the only hook; a face branch in
`Driver::estimate_cells` would remove this gap).

Atomicity: the driver builds into a fresh builder and `finish` leaves the allocator unchanged on
error, so a failed run adopts nothing; the engine discards the scratch state.

The whole intent is passed to the driver. Entries outside the region are ignored by the planner
(tested with two bricks), so no filtering is needed.

The stored plan is exactly the driver's: failures that caused a re-plan are in
`plan.rejections`, candidates passed over by the final plan in `plan.attempts`. The plan is not
hashed.

#### Scheme notes (ruling INT-7)

Schemes attach located diagnostics (`SchemeNote`: position, nearby model entities, metric, value,
threshold, suggestion) to their `Patch`. After the run, `mesh.run` stores `PatchSet::notes()`
(target order, then emission order) in `MeshEntry::notes`, so `mesh.quality` can read them later.

`MeshEntry::notes` is an *unhashed* per-entry record (like `plan` and `derived_from`): no layer
digest reads it, so no state or layer hash moves, with or without notes. It is a pure function of
the hashed inputs. `Plan` no longer carries notes: a plan reproduces its mesh exactly, and the
mesh derived by `mesh.to_poly` does not inherit the notes (they are positioned on the source
cells; the derived entry starts with none). Notes are not errors: a quality-floor violation
never fails `mesh.run`. `mesh.quality` publishes them as `scheme_notes` (capped at
`MAX_SCHEME_NOTES`) and `n_scheme_notes` (all of them); both are absent when there are none.
`mesh.plan` reports the same notes (`notes`, `ran` only) from `MeshEntry::notes`; a plan made
for `targets` has none, because notes exist only after generation.

### mesh.run cell budget

Before generating, `mesh.run` plans once (`Driver::plan`) and estimates the cells as
`volumes * mean(edge intervals)^3`. Above `MesherConfig::max_cells` (default
`DEFAULT_MAX_CELLS` = 20 million) it fails with `OpError::Invalid` and changes nothing. This
guards against e.g. `size 1e-6` on a 10-unit brick (about 10^21 cells). The estimate is crude
(anisotropic meshes are estimated by their mean count; exact for a structured brick, but only
edge counts are seen, so a volume with few long edges, e.g. a cylinder with one seam edge on its
curved face, can be off by a large factor either way); it only catches absurd requests. A plan
error is returned as such, before the budget check. The plan is made once: the driver checks the
budget on it inside `Driver::mesh_from_plan` and on every re-plan (volumes); for faces see
"Surface-only runs".

### mesh.delete

`meshes` xor `targets` (volumes or faces); `retire_mesh` per mesh, sorted. Any error fails the
op.

### mesh.plan (explain)

`mesh.plan {mesh | targets, attempts = true}` (exactly one of `mesh` and `targets`) is a
read-only query registered next to `mesh.info`; it mints no id and changes no state.

* `mesh`: what **ran**, from `MeshEntry::plan` (`status: ran`, `stale` copied). A mesh without a
  plan (imported) is `Invalid("mesh has no plan")`.
* `targets`: what **will run**: the same input, intent, size field and driver as `mesh.run`
  (`make_driver`, shared code) and one `Driver::plan` call, no generation (`status: will_run`).
  A planning failure is `status: infeasible` with `failure` (the `SchemeError` text, for an
  infeasible interval system its reason, constraint ids, entities and blame), not a query
  error. The guards above come back as `blockers` (same texts) and an over-budget estimate as
  one more blocker; `mesh.run` on the same input fails with `blockers[0]`.

`PlanReport` fields: `status`, `mesh`, `stale`, `dim` (2 or 3), `selection`, `estimated_cells`
(`will_run`), `waves`, `plan_hash` (FNV-1a of the JSON serialization of the plan, 16 hex
digits; a `ran` report of a plan made with no re-plan equals the `will_run` hash taken before
the run on the same state), `blockers`, `failure`, `entities`, `intervals`, `notes`.

* `entities`: one row per `plan.problem.targets` entry (curves, surfaces, volumes, groups,
  ascending): `scheme`, `how` (`intent#rank`, `registry#rank`, `piece_shape`, `imposed`, `auto`,
  `copy`, `claimed`), `score`, `reason` (verbatim), `params`, `caps`, `claimed_by`, `copy_of`
  (the class root), and `alternatives`: the candidates that lost, as `{scheme, phase, kind,
  message}`: failures that forced a re-plan (`plan.rejections`) first, then what the final plan
  passed over (`plan.attempts`), duplicates dropped. `elements` is `{"Quad": 6}` (cells
  classified on the entity) for `ran` and the caps' element type (`"QUAD4"`, `"TRI3"`,
  `"HEX8"`, ..., `"mixed"`) for `will_run`; `cells` is the classified count (`ran`).
* `intervals`: one row per solved edge, ascending: `count`, `why`, and every request that reaches
  the edge as `requests` (`origin`, `kind`, `value`, `agrees` with the solved count, `winner`).
  `why` is computed here from the public `MeshingProblem.intervals` records: the strongest
  direct request (a fixed count or goal) that agrees with the solved count decides, by rank
  `hard_intent` (a hard user count or size) < `deck` < `scheme` (scheme requirement or
  preference; names scheme and target) < `soft_intent` (a soft user count or size) <
  `sizing_goal` (`mesh.size` default firmness, or the default sizing; carries `goal`) <
  `default` (no request reached the edge: the built-in goal of 1). If no direct request agrees
  and linear or even-sum rows mention the edge, the count is `derived` (the rows' ids in
  `constraints`, marked as winners); the disagreeing goal is still listed. `soft_intent` and
  `piece` are additions to the draft's list: a soft user count is neither a hard one nor a size
  goal.

Determinism: the report is a pure function of `(state, request)`; maps are `BTreeMap`, rows are
in plan order, and nothing reads a clock or the thread count.

## Determinism

* `RayonExec` only decides *where* a job runs; the driver gives each job its own slot and merges
  in job order. Tests compare the state hash and the exact (bit-for-bit) `meshes` layer hash on
  `determinism::check(2, 4, ..)` (1 thread against 4, twice), 4 concurrent 1-thread runs, and
  the default pool.
* `mesh.info` volume is summed in fixed chunks of 4096 cells, then in chunk order.
* Maps are `BTreeMap`; targets are sorted before use.

## Benchmark: commit throughput (plan section 5, risk 3)

`cargo bench -p sm-mesher --bench commit` (`SM_BENCH_N=50,100` selects sizes). Stub schemes
(`sweep` + `map` + `equal` of `sm_scheme_api::stubs`), 4-core VM, release profile.

| Phase (100^3 = 10^6 hexes, 1 030 301 vertices) | Time | Cells/s |
|---|---|---|
| plan | 0.11 ms | n/a (size independent) |
| generate (rayon) | 1.62 s | 0.62 M |
| commit into `MeshBuilder` | 2.74 s | 0.37 M |
| plan + generate + commit (`Driver::mesh`) | 4.72 s | 0.21 M |
| `mesh.run` + `mesh.delete` op pair (adoption of 2.4 M ids, layer insert, state hash, retire) | 10.6 s | 0.094 M |

Reading: commit is the largest driver phase (58% of generate+commit) and the op layer adds about
6 s on top of the driver at 10^6 cells (id adoption/retirement and the quantized digest of the
mesh), so plan section 5 risk 3 is **partly triggered**: a 10^6-cell run is seconds, not
milliseconds. It is workable for W2; a bulk commit path (`MeshBuild::add_template_cells`) would
cut the 2.7 s, and the op overhead needs a profile before W3. No ACR is raised now.

### Bulk mesh commit: measurement (w3b-acr)

`cargo test --release -p sm-mesher --test bulk_perf -- --ignored --nocapture` (record only, no
gate; `SM_BULK_N` sets the brick edge). Loaded shared machine, so timings vary by about 2x
between runs. Before any change: `mesh.run` 11.9-16.4 s, `mesh.delete` 5.6-12 s at 10^6 cells.
Instrumenting `retire_mesh` (5.06 M ids): `mesh_ids` 0.10 s, liveness check 0.27-0.41 s, layer
removal ~0, and the per-id `ApplyCtx::retire` loop 7-11 s, i.e. nearly all of `mesh.delete`.
`insert_mesh` (id check, adoption by `adopt_range`, layer insert) is 0.23 s: adoption is already
bulk. The remainder of `mesh.run` over the driver is the state hash after the op.
`ApplyCtx::retire` costs a SHA-256 for the multiset accumulator, a `BTreeSet` insert and a push
to `Delta.removed` per id; `State::retire` is crate-private to sm-op-api, so a bulk retire cannot
be built from sm-layer-api. It needs an `ApplyCtx::retire_range` (see the wave report).

After (w3b-0, P4): `ApplyCtx::retire_range(first, n)` retires a consecutive run with one liveness
range query, a bulk-built `BTreeSet` and one `Delta.removed` reserve; the accumulator and state
hash are bit-identical to per-id `retire` (test `retire_range_equals_per_id_retire` in
sm-op-api). `mesh.delete` coalesces the sorted mesh ids into runs (`retire_mesh_bulk` in
`mesh_ops.rs`; same contract as `sm_layer_api::retire_mesh`). Same test, same loaded machine,
n=100: `mesh.delete` 4.5 s (was 5.6-12 s), `mesh.run` 11.6 s (unchanged). The rest of
`mesh.delete` is one SHA-256 per id for the multiset accumulator (5.06 M); making that parallel
needs `rayon` in sm-op-api (the sum is order-independent, so the result would stay identical).
Hashes unchanged.

The stub schemes are reference code; the numbers bound the driver and commit path, they do not
predict production schemes.

## Auto selection (W3b-AUTO, ruling Q5)

Automatic scheme choice is a **pluggable strategy that is consulted once and stored**.

* `AutoStrategy { name, description, choose(&AutoInput) -> Vec<SchemeAssignment> }`; strategies
  live in `MesherConfig::strategies()` (`with_strategy` replaces by name). `DefaultStrategy`
  (`"default"`) is registered by `MesherConfig::new`.
* `mesh.auto {strategy = "default", targets?}` runs the strategy at apply and stores itself as
  `mesh.auto {strategy, assignments: [{target, scheme, params}]}` through
  `ApplyCtx::set_canonical_record`. A record that already holds `assignments` applies them and
  never looks up a strategy, so replay calls no strategy and swapping or deleting a strategy
  cannot move a recorded hash. The effect equals one hard `mesh.scheme` per assignment (same
  intent, same hash). Targets that already have an intent scheme are never in the answer.
* `DefaultStrategy` lists: surface `[map, submap, pave, trimesh]`, volume `[map, sweep,
  tetmesh]`, curve `[equal]` (names not registered are dropped). It installs them as
  `defaults.auto` on a copy of the intent, plans, and meshes into a scratch mesh through the
  driver's replan loop, so a candidate that only fails at generation is rejected and replaced
  (fallbacks). The winners are the plan rows with `Selection::Auto` for faces and volumes; faces
  a parent imposes or claims, copies and curves are not assigned. List order breaks equal
  scores (the planner's rule).
* `replan_budget(n) = 16 + 2 n` is the driver's `max_replans` for `mesh.run`, `mesh.plan` and the
  strategy (each target that must fall back costs one replan). Runs that used to fit in 16
  are unaffected.
* Plan-time blame (W3c). `Driver::plan` can fail before any generation with
  `NoApplicableScheme` when a face is imposed incompatibly (tri by a hard `tetmesh` neighbour,
  quad by an auto `map`). `scratch_plan` (so `mesh.auto` and the census) plans through
  `plan_with_blame`: on that error it rejects the entries of `imposed_by` (sorted, never
  `copy`, never already rejected) one at a time. A rejection is kept once the replan gets past
  the target; if the same target fails again, or the rejected scheme's own volume has no
  scheme left, it is dropped and the next entry is tried, so the culprit need not be the entry
  with the lowest id. Every try, kept or dropped, counts against the same `replan_budget` as
  generation-time rejections (`mesh_from_plan` gets the remainder; a budget of 0 returns the
  planner's own error, and running out afterwards is `ReplanLimit`). Kept rejections travel in
  `plan.rejections`. `mesh.run` and `mesh.plan` do not replan (`mesh.auto` stores explicit
  assignments, and hard assignments are never blamed). Only inputs that failed to plan before
  change; no existing hash moves.
* B6 outcome (ruling C2): two merged bricks with a hard `tetmesh` on the left. The right
  volume's `map` is rejected and it falls to `sweep` from the shared triangle face: cells
  `{Tet: 160, Wedge: 32}`, volume 16, conformal, validator empty. An all-tet fill needs an
  explicit `tetmesh` (or a future `tet` strategy).
* Warning `mesh.auto_tet` per volume the strategy gave a tet mesher, listing why the earlier
  candidates were passed over and ending with the fix. Query `mesh.auto_strategies`.
* `standard_schemes` lists are untouched (strict), so a run without `mesh.auto` has the same
  plan and hash as before. `sm_mesher::census` compares today's plan with the strategy's,
  per volume, for the SERVER-HOST census (it reads the state and writes nothing).
* Cost: the strategy meshes the targets once in scratch, so `mesh.auto` costs about one
  `mesh.run`.

## Known gaps

* No `on_succession` for the intent layer.
* Scheme `params` that name entities (sweep `source`/`target` faces, ...) are opaque JSON owned
  by the scheme, so `IntentLayer::on_retired` does not prune them. A scheme on volume 2 whose
  params name faces of a deleted volume 1 survives, and `mesh.run` then fails with "not
  applicable". Fix by re-issuing `mesh.scheme`; a typed-reference convention for params is a
  W3 item.
* `fallbacks` carry names only (no parameters).
* Whole volumes (`PieceRef::Whole`) and whole faces only; no decomposition pieces.
* No sequential meshing against meshed neighbours (a face next to a meshed face, a volume over
  meshed faces): refused by the guards.
* `mesh.plan` is not cached in the `DerivedCache`; it plans on every call. Generation-time
  scheme notes are only available for `ran` reports.
* Neighbour guard is per face adjacency to *meshed* volumes; it does not look through
  coincident-face pairs.
* No partial success (`mesh.run` is all or nothing, as the research doc decides for W2).
* No cancellation inside the driver; `ctx.step` is checked before and after it.
