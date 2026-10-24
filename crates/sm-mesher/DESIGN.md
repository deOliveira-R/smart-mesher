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
`default`-firmness goal instead. It is the same "this is not a requirement" level, and it lets the
solver minimize deviation instead of picking an arbitrary soft count; `firmness: "soft"` gives
the Cubit reading. A request never overwrites a firmer one on the same entity: the op keeps the
firmer request and emits a `mesh.firmness_kept` warning. Requests cascade volume, face, edge
inside the planner (an edge's own request wins).

`auto` (1..=10) is resolved at apply time through the `AutoSizeFn` hook and recorded as a plain
size, so replay never runs the hook again.

### mesh.run

1. Resolve targets (live `TopoVolume`s, sorted, de-duplicated). Refuse a volume that already has
   a mesh, or that shares a face with a meshed volume outside `targets` (`mesh.delete first`).
2. Clone the read layers (`Arc` clones) and the allocator; build the size field from
   `defaults.sizing` through the hook if there is one.
3. `Driver::mesh(&input, &intent, &RayonExec, MeshBuilder::new(), &mut alloc_copy)`. Re-plans
   around failing schemes happen inside the driver.
4. `insert_mesh(ctx, entry, &alloc0, alloc, "mesh")` adopts the minted ids (one range per kind,
   `Mesh` first) and inserts the entry. Output role `mesh` (cardinality one).

Atomicity: the driver builds into a fresh builder and `finish` leaves the allocator unchanged on
error, so a failed run adopts nothing; the engine discards the scratch state.

The whole intent is passed to the driver. Entries outside the region are ignored by the planner
(tested with two bricks), so no filtering is needed.

The stored plan is exactly the driver's: failures that caused a re-plan are in
`plan.rejections`, candidates passed over by the final plan in `plan.attempts`. The plan is not
hashed.

### mesh.delete

`meshes` xor `targets` (volumes); `retire_mesh` per mesh, sorted. Any error fails the op.

## Determinism

* `RayonExec` only decides *where* a job runs; the driver gives each job its own slot and merges
  in job order. Tests compare the state hash and the exact (bit-for-bit) `meshes` layer hash on
  pools of 1, 2 and 4 threads, plus concurrent runs, plus the default pool.
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

The stub schemes are reference code; the numbers bound the driver and commit path, they do not
predict production schemes.

## Known gaps

* No `on_succession` for the intent layer.
* `fallbacks` carry names only (no parameters).
* Whole volumes only (`PieceRef::Whole`); no decomposition pieces, no surface-only `mesh.run`.
* Neighbour guard is per face adjacency to *meshed* volumes; it does not look through
  coincident-face pairs.
* No partial success (`mesh.run` is all or nothing, as the research doc decides for W2).
* No cancellation inside the driver; `ctx.step` is checked before and after it.
