# sm-op-api: registry design

Style: **open trait-object ops registered by name**, typed payloads with `schemars` schemas,
layer stores registered dynamically. The api crate knows no concrete op or layer; each concept
node ships a `Plugin` that registers its own.

## Model

- **Op** (`Op` object-safe trait): `name`, `version`, `encode() -> Value`, `apply(&self, &mut ApplyCtx)`.
  Authors implement `TypedOp` (`Serialize + DeserializeOwned + JsonSchema`, `NAME`, `VERSION`, `apply`);
  a blanket impl gives `Op`. `Registry::register::<T>()` stores an `OpType` (name, version, schema, decode).
  `register_dyn` accepts hand-written `OpType`s.
- **State**: `BTreeMap<name, Arc<dyn Layer>>` + live-entity set + per-kind next-serial counters.
  Layers implement `LayerType` (`NAME`, `ROLE`, `digest`), registered with `register_layer::<L>()`;
  `registry.new_state()` instantiates every registered layer. Ops reach layers by type
  (`ctx.layer_mut::<ModelLayer>()`). Conventional names (`model/partition/intent/meshes/derived`)
  are constants in `layer::names`; only `derived` (`DerivedCache`) is built in.
- **Engine / ApplyCtx**: `Engine::apply` clones the state (Arc-cheap, layers copy on first write),
  runs the op on the clone, and swaps it in only on `Ok`. `ApplyCtx` is the only mutation handle:
  `alloc/retire/require` (entities), `layer/layer_mut`, `cache_get/cache_put`, `report`, `check_cancelled`.
- **Timeline**: `Node { parent?, op: OpRecord{name,version,params}, meta, state }`; node hash =
  SHA-256 of its canonical TOML. `Timeline` holds nodes, `refs/heads/*` and `HEAD` (`Head::Ref` or detached).
  `Session` = Engine + Timeline + current State + snapshots. `fs::save/load` writes
  `ops/<hash>.toml`, `refs/heads/<name>`, `HEAD`.

## Requirement map

| Requirement | Where |
|---|---|
| Op serializable, versioned | `OpRecord`, `Op::version`, `Registry::register_migration` (v -> v+1 chain on decode; newer-than-known is a typed error) |
| Deterministic apply | `apply` sees only `(self, ApplyCtx)`; no clock/RNG in the api; ops needing RNG carry a seed in params; layers hash via `Digest` (supports quantized floats) |
| Timeline DAG, content addressed, TOML per node, refs/HEAD | `Node`, `Timeline`, `Head`, `fs` |
| Checkout/branch, replay from root or nearest snapshot | `Session::{checkout, branch, state_at}`; snapshots every N depth (`set_snapshot_interval`); each replayed node's recorded `state` hash is verified (`TimelineError::Diverged`) |
| Stable refs between ops | `ctx.alloc(kind)` takes the next serial of that kind from counters inside the state, so ids are a pure function of history; `EntityRef` (integer in params) + `resolve(ctx)` checks liveness; serials are never reused, even after `retire` |
| JSON schema for MCP/help/stubs | `Registry::schema`, `schema_json`, `catalog()` (sorted `OpInfo`); doc comments become descriptions |
| Transactional apply | `Engine::apply` scratch clone; `Session` records a node only after success; failing op leaves state, counters, timeline untouched |
| Progress / cancellation | `Progress` trait (`report`, `is_cancelled`), `CancelToken`, `RecordingProgress`, `ctx.check_cancelled()?`; `Cancelled` is just an `OpError`, so partial work is discarded |
| Composable layered State, plugin-registered | `Plugin`, `LayerType`, `Registry::install` |
| Derived caches keyed by content hash | `DerivedCache` layer (role `Derived`, excluded from `State::hash`), `State::cache_key(kind, input layers, params)` hashes input layer digests |

Live and replay share one path: `Session::apply` encodes the op and applies the *decoded* record,
so what ran is exactly what a node stores.

## Canonical serialization and the dependency ceiling

`toml`, `serde_json` and `sha2` are outside the allowed set, so the crate carries small
replacements: `Value` (sorted-table tree) with serde `Serializer`/`Deserializer`, a TOML subset
writer/parser (`toml.rs`, standard TOML output, `None` fields omitted, floats via shortest
round-trip, `nan`/`inf` supported), and a FIPS 180-4 SHA-256 (`hash.rs`, byte-identical to `sha2`).
Limits: integers are `i64` (a `u64 > i64::MAX` in params is an error; `EntityId` kinds < 128 fit),
no null, no datetimes. Swapping in `toml`/`sha2` later should not change hashes provided the
emitted text stays identical, which is worth a golden test at that time.

## Tradeoffs

- Open registry vs. a closed enum: plugins add ops without touching the api, at the cost of
  dynamic dispatch, runtime "unknown op" errors, and no exhaustive matching. Params are decoded per
  replay; fine for op-rate workloads.
- `Layer` hashing is a full digest per call; `State::hash()` is O(state). Large layers should keep
  an incremental hash internally (the `digest` hook allows it). Snapshot interval bounds replay cost.
- Entity table lives in `State`, not a layer, so existence checks and id allocation are uniform
  across layers (one source of liveness); layers store payload keyed by `EntityId`.
- Node hash includes `state`, so a node commits to its result; a cross-platform float difference
  in a layer digest would surface as `Diverged` (hence quantized digests).
- No merge, no op editing/regeneration (P2 naming problem is out of scope per ARCHITECTURE.md §3).
- Panics inside `apply` propagate (state is safe because the scratch clone is dropped); no `catch_unwind`.
