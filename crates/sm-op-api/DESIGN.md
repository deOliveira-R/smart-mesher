# sm-op-api: design (candidate "deltas")

## Model

Ops are the source of truth; every applied op also yields a reversible **Delta**, persisted next to it. Moving around the timeline DAG applies/reverts deltas; no op code runs and no snapshots are needed. Deltas are a *cache*: if one is missing, unparsable, or fails its state-hash check, the workspace replays ops from the root and rewrites the deltas.

```
OpRecord{kind,version,params} --Registry::apply(State)--> AppliedOp{ Delta, created ids }
Node{parent, op, meta}  --hash--> timeline/ops/<hash>.toml      (immutable, content addressed)
Delta{state_before/after, counters, [LayerPatch{layer, forward, inverse}]} -> timeline/deltas/<hash>.toml
```

## Key types

- `Value`/`to_value`/`from_value`: the canonical data model (TOML minus datetime, no null, sorted keys). Own serde serializer/deserializer, because only serde/thiserror/indexmap/schemars are allowed dependencies (no toml/json/sha crates). `toml` module: canonical emitter and a parser for the same subset. `hash` module: SHA-256, `Hash`, `StateHasher` (canonical feeding of `Value`s).
- `Op` (trait: `KIND`, `VERSION`, `Serialize + Deserialize + JsonSchema`, `apply(&self, &mut ApplyCtx)`), `OpRecord` (its serialized form).
- `Layer` (typed store with `type Patch`; `apply(&patch) -> inverse patch`, `hash_into`), `DynLayer` (erased, patches as `Value`), `MapLayer<T>` (reference layer: `EntityId -> T`), `LayerId` (well-known names: model, partition, intent, meshes, derived).
- `State`: `name -> layer`, `IdCounters` (per-kind serial), `Arc<DerivedCache>`. Per-layer content hashes are cached and invalidated by patches; `State::hash()` combines layers + counters.
- `Registry` + `Concept`: each concept node calls `register_layer`/`register_op`/`register_migration`; the api crate never names concrete layers or ops.
- `ApplyCtx`: the only mutation path (`alloc`, `patch`, `layer`, `checkpoint`).
- `Node`, `TimelineStore` (+ `MemStore`, which holds the exact file layout), `Workspace<S>` (apply, create_branch, checkout, replay_from_root).

## How requirements are met

- **Every change is an Op, versioned, deterministic.** `Op::VERSION` is stored in the record; `Registry::register_migration(kind, from, f)` upgrades params one version at a time; newer-than-known versions are refused.
- **Timeline DAG, content addressed, TOML per node, refs/HEAD.** Node hash = SHA-256 over the canonical TOML text (prefix-tagged). Canonical form: sorted keys, scalars before `[tables]`, fixed escaping, shortest round-trip floats, non-finite floats rejected. Metadata (author/time/message) is in the hash but time is caller-supplied (no clock in the api). `MemStore::files()` is the `timeline/{ops,deltas,refs/heads}/` + `HEAD` layout.
- **Checkout/branch.** `checkout` finds the common ancestor, reverts deltas up, applies deltas down, then verifies one end-to-end state hash. `create_branch` = `git checkout -b`. Detached HEAD advances with new nodes. Forking is just applying on an older node.
- **Stable references.** `ApplyCtx::alloc(kind)` takes the next serial from per-kind counters held in the state (serial 0 unused, never reused within a lineage). Ids depend only on op history and on the op's own (data-defined) allocation order; counters live in deltas, so revert/replay reproduce them. Ops carry ids as `Eid` (serde/schemars wrapper of `EntityId`, since sm-core has no serde).
- **Schema generation.** `Registry::catalog()` / `op_schema(kind)` return schemars output as `Value` (`to_json()` for text), with rustdoc as descriptions. One source for MCP tools, help and Python stubs.
- **Transactional apply.** `Registry::apply` records an inverse for each patch/counter as it goes; on `Err`, cancellation or panic it applies them in reverse before returning (panic is re-raised). Layers must be atomic per patch. `Workspace::apply_record` persists only after success; a store failure reverts the delta.
- **Progress/cancel.** `ApplyOptions{progress: &dyn ProgressSink, cancel: CancelToken}`; ops call `ctx.checkpoint(fraction, msg)`, which errors with `Cancelled` and triggers rollback.
- **Composable State.** Layers are independent stores registered by name; patches are per layer; the state hash is the hash of per-layer hashes, so registration order is irrelevant.
- **Derived caches by content hash.** `DerivedCache::get_or_compute(kind, key, f)` with `State::derived_key(&[LayerId])` hashing exactly the input layers. Not in state hash or deltas; no invalidation logic needed.

## Trade-offs

- Delta persistence doubles on-disk metadata and costs an extra serialize per op (patches as `Value`); in exchange checkout/cold start never re-run expensive ops (meshing) and need no snapshot policy. Large layer payloads (whole meshes) should use coarse patches (replace entry) or content-addressed blobs referenced from patches; a blob store is not in this crate.
- Reversibility is a layer-author duty (`apply` must return a correct inverse). It is enforced indirectly by hash verification in checkout and by tests; a generic `MapLayer` covers the common case.
- Ids are counters, not hash-derived: deterministic and compact, but two branches reuse the same serials for different entities (fine because branches never mix; merging would need remapping).
- A failed delta walk mid-way is repaired by full replay, which is O(history). Snapshots could bound that; deliberately omitted.
- Linear DAG nodes have a single parent (merges are out of scope for P0-P1).
- State hash covers whatever layers feed it; platform-independent hashes need layers to quantize floats (the api canonicalizes NaN and rejects non-finite floats in params only).
- `Value` ints are `i64`; `u64` above `i64::MAX` is an error (entity kinds < 128 keep ids in range).
