# W4-0.4 CSGBRIEF: decisions for the OpenMC CSG to B-rep import

Binding text: `plan.md` section 3.3 (W4-0.4). Source: `drafts/csgbrep.md` CSGBRIEF-0, minus the crate skeleton and plugin registration (W4-0.2). Rulings D9 and D10 (`rulings.md`) and `drafts/regions.md` R1, R2 apply. All evidence is at commit `69ba521`, and every code cite was re-verified at `846c2ee` unless a line says otherwise. This task changes no code and no hash.

## Summary

| # | Question | Decision |
|---|---|---|
| 1 | One op or a batch | D1 option (a): one op, `model.import_openmc`. One history node (ruling D10). No frozen-crate change. |
| 2 | Materials input | A second `Source` field of the same type. No ACR. |
| 3 | `model.copy`, `fem.block_add`, `fem.sideset_add` | The import calls their `TypedOp::apply` with its own context. The layer bookkeeping is not skipped. **W4-CSG-LIBFN is dropped.** Three constraints on the import are listed in section 3.3. |
| 4 | D9 lattice tags | One `LatticeFrame` per deck lattice, named `lat<id>`, nested by fill, written in one `model_txn` after the last geometry edit, through ACR-REGIONS. |

## 1. D1 option (a): `model.import_openmc` is one op

Decision: (a). The op follows `model.import_step`. It calls library functions inside one `apply`, so the timeline holds one node (ruling D10).

Evidence:
- Option (b), a `core.batch` of ordinary ops, is unavailable to a new crate. `ApplyCtx::apply_nested` is `pub(crate)` (`crates/sm-op-api/src/op.rs:623`), and `core.batch` refuses nesting and has no control flow (`crates/sm-op-api/src/batch.rs:13-23`, cited in `drafts/csgbrep.md` finding 12). Option (c) would edit a frozen crate. Ruling D10 settles that W4 takes (a); W5 regeneration re-runs the import with an override map.
- Precedent: `ImportStep::apply` builds all its cells inside one `apply` with `IMPL_VERSION = 2` (`crates/sm-cadio/src/op.rs:72-140`; the `TypedOp` impl starts at `:109`).
- The kernel entry points the partition builder (D2) needs are public library functions generic over `TopologyEdit`: `sm_boolean::webcut` (`crates/sm-boolean/src/webcut.rs:112`), `boolean` (`boolean.rs:155`), `merge` (`merge.rs:571`), `merge_check` (`merge.rs:510`).
- The transaction wrappers those functions need are public in `sm-layer-api`: `topo_txn` (`crates/sm-layer-api/src/model.rs:264`), `commit_topology` (`:442`), `model_txn` (`:521`); re-exported at `crates/sm-layer-api/src/lib.rs:33`.

No contract change under (a). Wiring (workspace member, plugin registration) stays in W4-0.2.

## 2. Materials: a second `Source` field of the same type, no ACR

Decision: `model.import_openmc { geometry: Source, materials: Option<Source>, options }`. Both fields have the type `sm_cadio::Source` (`{sha256}` or `{inline}`).

Evidence:
- `sm_csgio::openmc::import` already takes the two documents separately as strings: `pub fn import(geometry_xml: &str, materials_xml: Option<&str>)` (`crates/sm-csgio/src/openmc/mod.rs:148`). `geometry.xml` and `materials.xml` are two files in an OpenMC deck, so two `Source` values map one to one. No archive type is needed.
- `Source` is public with `Serialize`, `Deserialize` and `JsonSchema` (`crates/sm-cadio/src/source.rs:36-41`; re-exported `crates/sm-cadio/src/lib.rs:46`). The `{sha256}` form reads through the `StepInputs` service, which is already injected (`crates/sm-server/src/standard.rs:192`). A second hash is a second lookup in the same store, so replay stays content-addressed, with no new service.
- Hash rule for the op record: `materials` is omitted when absent (`skip_serializing_if = "Option::is_none"`, as `ImportStep` does for its optional fields).

One implementation point, not a contract change: `sm_cadio::source::load` is `pub(crate)` (`crates/sm-cadio/src/source.rs:58`). `sm-csgbrep` must resolve a `Source` itself. It can do so from public items only: `Source`, `InlineSource`, `Sha256Source`, `MAX_INLINE_BYTES`, `StepInputs`, `InputStore::get`, `sha256_hex` (`crates/sm-cadio/src/lib.rs:40,46`, `crates/sm-cadio/src/inputs.rs:18,43`). That is about 25 lines (decode with the size cap, check the hex form, look up, re-hash, compare), with the error codes renamed `csg.*`. Alternative if the duplication is unwanted: the sm-cadio owner makes `load` public (one word). W4-CSG-CORE chooses; neither needs an ACR. This task recommends the 25-line copy, so no crate outside the owner's gets touched and `step.*` codes are not reused for a deck.

If a later need makes one `Source` hold both files (an archive), that is a new ACR on `Source`. W4 does not need it, and the fallback ACR text is therefore not written.

## 3. `model.copy`, `fem.block_add`, `fem.sideset_add`

### 3.1 Answer

Yes. The import calls `TypedOp::apply` of `ModelCopy`, `BlockAdd` and `SidesetAdd` with its own `&mut ApplyCtx`. It skips none of the layer bookkeeping, because that bookkeeping lives inside those `apply` bodies, and they are the same code the registered ops run. **W4-CSG-LIBFN is dropped.** No public library function has to be added in `sm-model` or `sm-fem`.

### 3.2 Evidence (code)

- The three ops are public structs with public fields and public `TypedOp` impls: `ModelCopy` (`crates/sm-model/src/copy.rs:87`, re-exported `crates/sm-model/src/lib.rs:15`), `BlockAdd` and `SidesetAdd` (`crates/sm-fem/src/ops.rs:67`, `:254`, re-exported `crates/sm-fem/src/lib.rs:27`). `TypedOp::apply` is a public trait method taking `&mut ApplyCtx<'_>` (trait `crates/sm-op-api/src/op.rs:41-67`, `apply` at `:66`), and `ApplyCtx` is a public struct (`:247`). The `pub(crate)` item is only `apply_nested` (`:623`), which the import does not need.
- `model.copy` (`copy.rs:265-341`): reads the closure through `ctx.layer::<ModelLayer>()`, builds the copies inside `topo_txn` (`:317`), and finishes with `commit_topology(ctx, &txn, &[("volumes", ..), ("bodies", ..)])` (`:340`). `commit_topology` (`model.rs:442`) runs `Model::apply_lineage` (re-keys colours, groups, tags), adopts ids, records the roles and `record_successors`. That is the whole of the bookkeeping. By design copies carry no names, colours, groups or boundary kinds (`copy.rs:8-10`), so there is nothing else to skip.
- `fem.block_add` (`ops.rs:88`): `model_txn` (the model-only transaction, `model.rs:521`) does colour creation and `color_volumes`; `block-move` warnings go through `ctx.warn`. `fem.sideset_add` (`ops.rs:274`): `model_txn` for the face colour and `ctx.layer_mut::<FemLayer>()` for the side preference. Both go through public `ApplyCtx` methods only.
- `sm-boolean/DESIGN.md:59-63` says the same of the boolean ops: `topo_txn` + `commit_topology`, `model_txn` for lineage-driven colours and groups. The import's own boolean calls use the same pair.

### 3.3 Evidence (spike) and the three constraints

Spike (throw-away, outside the repo, not committed; scratchpad `spike/`): a test op `spike.import` registered next to `ModelPlugin` and `FemPlugin` ran, inside one `apply`, `CreateBrick`, `ModelCopy { count: 2, Move }`, two `BlockAdd` and one `SidesetAdd` through `TypedOp::apply(&op, ctx)`. Result: the session accepted it as one node; the delta held 87 created ids, touched layers `fem` and `model`, no diagnostics; the volumes carried their block colours (`TopoVolume#1` in `fuel`, `#2` and `#3` in `clad`). So the calls compose and the colours land.

The spike also showed three constraints the W4-CSG-CORE worker must handle:

1. **Disambiguate the call.** `Op` has a blanket impl for every `TypedOp` (`op.rs:69-86`), so `op.apply(ctx)` is ambiguous when both traits are in scope. Write `TypedOp::apply(&BlockAdd { .. }, ctx)`.
2. **Output roles from nested calls leak into the import's delta, and no public method reads or clears them.** The nested ops record their roles into `ctx` (`volumes`, `bodies`, `block`, `sideset`), and `outputs` is private (`op.rs:247-262`). The engine checks them against the import's declared `OUTPUTS` (`engine.rs:100`, `check_outputs` at `op.rs:654`): in the spike, a declared list that omitted a leaked role failed with `Invalid("undeclared output role ...")`. With an empty `OUTPUTS` (accepts anything, `op.rs:658-659`) the run passed and `block` held both colours, so repeated `BlockAdd` calls accumulate (their own `Cardinality::One` is checked only for the op run alone, and the import must declare `Many`). Rules for the import:
   - `OUTPUTS` is a static const, so the import declares a **fixed, named role list**, never one role per region name (`check_outputs` refuses any undeclared role). Per-region information goes into the tag table and the `info` notes, not into roles.
   - The import's own result roles have **distinct names** that cannot clash with a nested call's: `deck_volumes` (the final live volumes, ascending) and `deck_bodies` (the final live bodies, ascending), both `Many`, plus the roles `block` and `sideset` that its `BlockAdd` and `SidesetAdd` calls record (`Many`).
   - The nested `ModelCopy` records `volumes` and `bodies`, which leak into the import's roles. They must be declared too (`Many`), and the docs state that they are **intermediate copies**: later cuts and merges of the import may retire ids they hold. Consumers read `deck_volumes` and `deck_bodies`, never `volumes` or `bodies`. If the engine's checks reject retired ids in a leaked role (the spike did not retire any), W4-CSG-CORE falls back to LIBFN (section 3.4), whose `copy_entities` returns the ids and records no role.
   - The geometry calls (`webcut`, `merge`) go through the library functions with the import's own `topo_txn` + `commit_topology`, which name only the roles the import chooses. Only `model.copy`, `fem.block_add` and `fem.sideset_add` need the `TypedOp::apply` route.
3. **Cross-layer reactions run once, at the end of the import.** `ApplyCtx::flush_reactions` is private (`op.rs:332`); `apply_nested` calls it after each sub-op, a direct call does not, so `on_succession` and `on_retired` of other layers see the merged maps when the import ends (`finish`, `op.rs:314`). This is harmless for the import's own work (it creates its volumes and refers to them by ids it tracks), but the import must not read, between two of its steps, state that another layer derives from retirements (for example the `fem` layer's side preferences on a volume the import itself retired). Applying the import to a non-empty model is allowed; ids of the earlier model are never retired by it, since the import only cuts and copies bodies it created.

Additionally, `ModelCopy::apply` returns no ids. The import finds the copies as the live volumes that were not present before the call (`ctx.layer::<ModelLayer>()?.topo().cells(Dim::Volume)`), which is deterministic because copy ids are ascending (`copy.rs:5`). The per-call cap is `MAX_COPIES = 4096` (`copy.rs:25`); larger lattices use several calls.

### 3.4 Fallback

If constraint 2 or 3 turns out unworkable in W4-CSG-CORE, the order of fallbacks is: (1) the owners of `sm-model` / `sm-fem` add the small library functions below in a new W4-CSG-LIBFN (not frozen crates, each wraps the existing op body so op and function share one code path); (2) D1 option (b) or (c), which the orchestrator decides. The functions, in case they are needed:

```rust
// sm-model (crates/sm-model/src/copy.rs), re-exported at the crate root
pub struct CopyResult { pub volumes: Vec<EntityId>, pub bodies: Vec<EntityId> }
pub fn copy_entities(ctx: &mut ApplyCtx<'_>, entities: &[EntityId], count: u32, xform: Option<&CopyXform>) -> Result<CopyResult, OpError>;

// sm-fem (crates/sm-fem/src/ops.rs), re-exported at the crate root
pub fn block_add(ctx: &mut ApplyCtx<'_>, block: u32, volumes: &[EntityId], name: Option<&str>) -> Result<EntityId /* colour */, OpError>;
pub fn sideset_add(ctx: &mut ApplyCtx<'_>, sideset: u32, faces: &[EntityId], wrt: Option<EntityId>, name: Option<&str>) -> Result<EntityId /* colour */, OpError>;
```

This is **not** requested now: the answer is "LIBFN dropped". These signatures exist so that the orchestrator can reinstate LIBFN without a new design round.

## 4. `ApplyCtx` methods the import calls, all public

All are `pub fn` on the public `ApplyCtx` (`crates/sm-op-api/src/op.rs`), exported at `crates/sm-op-api/src/lib.rs:38`.

| Method | Line | Use in the import |
|---|---|---|
| `layer::<L>()` | 575 | read `ModelLayer` (tolerance, topology), `FemLayer` presence |
| `layer_mut::<L>()` | 582 | `FemLayer::set_side_wrt` (through `SidesetAdd`), model access |
| `service::<S>()` | 597 | `StepInputs` for `{sha256}` sources |
| `state()` | 348 | read-only state (`peek_next`, liveness) |
| `require(id)` | 550 | check ids named in options |
| `resolve_kind(id, kind)` | 565 | check an entity kind |
| `step(done, total, msg)` | 605 | progress and cancellation per universe or lattice |
| `sub(lo, hi)` | 615 | progress slices per phase |
| `warn(code, msg, ents)` | 496 | `import.reflective_curved`, overlap and gap reports, `import.lattice_multi_placement`, `import.lattice_rotated` |
| `info(code, msg, ents)` | 506 | counts, the partition report, `import.lattice_multi_placement` (same parent) |
| `adopt_as(role, ids)` | 438 | used inside `topo_txn`/`commit_topology`/`model_txn` helpers |
| `record_role(role, ids)` | 454 | the import's fixed role list (`deck_volumes`, `deck_bodies`, section 3.3 point 2) |
| `record_successors(old, new)` | 480 | used inside `commit_topology` |
| `retire(id)` | 515 | used inside `commit_topology` and `model_txn` |
| `set_canonical_record(rec)` | 301 | only if the import re-encodes its params with defaults filled (as `ImportStep` options may) |

Not called: `alloc`, `alloc_as`, `adopt`, `adopt_range` (ids are minted by the topology store and adopted by `commit_topology`), `retire_range`, `derived`, `registry`, `scope`. `apply_nested` (`pub(crate)`, `:623`) is not needed, and the import neither asks for it nor works around it.

## 5. D9 tags: how the import writes them through ACR-REGIONS

Context: ACR-REGIONS (W4-0.6) adds `Model::define_lattice_frame(name, LatticeFrame)`, `Model::set_region_tag(volume, RegionTag)` and the lineage rule (`drafts/regions.md` R1; `plan.md` W4-0.6). `model.import_openmc` is the writer named in R2.

Decisions:

1. **One frame per deck lattice, named `lat<id>`** (the deck's lattice id; `[A-Za-z0-9_]+` is met). Written with `define_lattice_frame`. The frame's `Layout::Cartesian { n: dims, pitch }` copies the deck `Lattice` (`crates/sm-csgio/src/ir.rs:190-203`), with `SlotIx { i, j, k }` equal to the deck's `[i, j, k]`. The IR already fixes the index order (`i` along +x, `j` along +y, `k` along +z, `universes` `i` fastest; `ir.rs:9-12`), so no row reversal appears in the tags; the reader's reversal of OpenMC's top-to-bottom rows is done in the IR.
2. **Frame origin.** R1 says slot `(0,0,0)` is centred at `origin` (`Layout::offset`, `crates/sm-model-api/src/instancing.rs:209-212`). The IR stores the lower-left corner, so `origin = lower_left + pitch / 2` per axis, mapped to world by the placement of the lattice (the `xf` of `Fill::Lattice`). A lattice axis with one element and infinite pitch (`ir.rs:13-14`) gets a finite layout pitch equal to the build-box extent on that axis, and the origin is the box centre on that axis. This keeps `LatticeFrame::locate` and `layout.contains` total.
3. **Nesting by fill.** `parent` is the frame of the nearest enclosing lattice on the fill path: a lattice reached through `Fill::Universe` inside an element of lattice M gets `parent = Some("lat<M>")`; a lattice at the root gets `None`. The walk is in ascending cell-id and ascending slot order, so names and parents are deterministic. If one lattice id is placed more than once, the first placement in that order defines `origin` and `parent`. The import then emits `import.lattice_multi_placement` naming the lattice: a **warning** when the placements have different parent frames (the frame cannot describe both), and an **info** when the parent is the same (a core deck places one assembly lattice in many slots of one parent lattice; the slot tags stay exact because they carry the deck's own indices and not a position, `RegionTag.slots`, R1). Only the geometric re-derivation by `model.lattice_tag` is unreliable for the later placements.
   Rotation: a rotated `xf` on a `Fill::Lattice`, or on an enclosing `Fill::Universe` placement, is not representable in `LatticeFrame` (it has an `origin` only). Tags then come from the deck indices, and the import emits the warning `import.lattice_rotated` naming the lattice: `model.lattice_tag` cannot re-derive these tags from geometry.
4. **Tags.** Each volume that the import creates inside lattice element `[i,j,k]` gets a `RegionTag` whose `slots` hold one entry per enclosing frame, outer to inner (`lat<outer>` then `lat<inner>`), `ring` and `sector` unset (those are written by `model.subdivide_pin`). A volume outside every lattice (reflector, root-level cells) gets no tag, as R3 specifies ("simply has no such part").
5. **When.** After the last cut, merge and copy of the import, in one `model_txn`, so no lineage rewrite follows and no volume is tagged twice. The frame table is written first; then the tags, ascending volume id. `model_txn` mints no ids, so it records no roles.
6. **Acceptance for W4-CSG-CORE and CSG-ORACLE** (as in `drafts/regions.md` R2): the tags equal `model.lattice_tag` on the same volumes for a single-placement lattice, and equal OpenMC's lattice indices at the oracle points. Hex lattices (`Layout::Hex`) are CSGBREP-HEX in W4c (ruling D8); until then a deck with a hex lattice stays a typed `Unsupported`.

## 6. W4-CSG-LIBFN

LIBFN dropped. Reason: section 3. `ModelCopy`, `BlockAdd` and `SidesetAdd` are callable through `TypedOp::apply` with the import's own context, and the spike shows the colours, lineage and layer touches land. Section 3.4 keeps the signatures should W4-CSG-CORE find constraint 2 or 3 unworkable.

## 7. Acceptance checklist

1. Each decision carries file:line evidence (sections 1 to 3, 5).
2. LIBFN: "LIBFN dropped" with the reason (sections 3 and 6); fallback signatures kept in 3.4.
3. Every `ApplyCtx` method the import calls is named, with line numbers, and all are public (section 4).
4. No code file changes: the only changed file is `docs/design/w4/csgbrief.md`.

Hash impact: none.
