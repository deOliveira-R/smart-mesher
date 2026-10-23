# sm-op-api: final design (frozen contract)

Open trait-object ops registered by name, typed payloads with `schemars` schemas, layer stores
registered dynamically. The api crate knows no concrete op or layer; each concept node ships a
`Plugin`. The crate does **no file I/O**: persistence is `Timeline::to_files` / `from_files`
(path -> text) and lives in `sm-engine`.

Basis: `docs/design/w0/op-api-recommendation.md` (registry base, grafts G1-G11, defects D1-D16)
as amended by `docs/design/w0/decisions.md`.

## Model

- **Op** (`Op`, object safe): `name`, `version`, `encode() -> Value`, `apply(&self, &mut ApplyCtx)`.
  Authors implement `TypedOp` (`Serialize + DeserializeOwned + JsonSchema`); `Registry::register::<T>()`
  stores name, version, schema and decoder. Params must serialize to an object. Entity ids appear in
  params as `EntityId` (`"TopoVolume#3"`, schema pattern `^[A-Za-z]+#[0-9]+$`) from `sm-core`.
- **Params value** (`codec`): `serde_json::Value` restricted to what TOML can hold: no `null`
  (absent `Option` fields are omitted), integers within `i64`, finite floats, `-0.0` folded to `+0.0`.
  `to_params` encodes through `toml::Value` so `NaN`/inf are caught instead of silently becoming null.
- **State**: `BTreeMap<name, Slot{Arc<dyn Layer>, OnceLock<Hash256>}>` plus an `Arc`'d entity table.
  Cloning is cheap (copy-on-write). `State::hash()` covers the allocation state and every
  non-empty authoritative layer, in name order. Layer content hashes are memoized and cleared on
  `layer_mut`, so a trivial op costs O(touched layers). Empty layers are skipped, so installing a new
  concept does not change old hashes. Floats must be quantized in `LayerType::digest`
  (`Digest::f64_quantized`) so the hash is platform independent.
- **Entity table**: per kind `next` serial and a set of retired serials (live = `1..next` minus retired).
  Alloc is a counter bump (a million mesh cells cost nothing); retire feeds an order-independent
  multiset hash (sum mod 2^256 of `sha256(serial)`). Serials are never reused. `alloc_sorted` assigns a
  contiguous range in sorted-key order (addendum §7.2). Ids are a pure function of the op prefix, so
  sibling branches reuse serials; UI selections must be scoped to a node.
- **LayerType**: `NAME`, `ROLE`, `DEPENDS_ON`, `digest`, `is_empty`, `on_retired`, and the optional
  persisted-snapshot hook `snapshot() -> Result<Option<Vec<u8>>>` / `restore(&[u8])` (default: none).
  `State::snapshot()` / `Registry::restore()` use the hook so a storage backend can persist big mesh
  layers instead of replaying. `MapLayer<T>` is the reference store for concept layers.
- **Derived cache** (`DerivedCache`): owned by the session, shared with every state and op through
  `ApplyCtx::derived()`; bounded LRU by a byte estimate; keyed by `State::cache_key(kind, input layer
  names, params)`, which hashes the *exact (unquantized) content hashes* of the named layers (the CI state hash is quantized; cache keys must not be), so an entry is reused exactly
  when its inputs are unchanged, across checkout, failure and cancellation. Never part of the state
  hash. A hit must never change op output. Type confusion under one key is an error.
- **Engine / ApplyCtx**: `Engine::apply` runs the op on a clone and swaps it in only on `Ok`, after the
  `on_retired` reactions. `ApplyCtx` is the only mutation handle: `alloc`, `alloc_as(role, kind)`
  (named outputs in `Delta.outputs`), `alloc_sorted`, `retire`, `require`/`resolve`/`resolve_kind`,
  `layer`/`layer_mut`, `derived`, `service::<S>()` (geometry kernel, mesher; injected at `Engine::new`),
  `step(done, total, msg)` (progress + cancellation in one call), `scope()`/`sub(lo, hi)` (copyable
  `Sync` progress scopes for scoped worker threads).
- **Cross-layer reactions**: after the op body the engine calls `on_retired(&removed)` on every
  non-empty authoritative layer in the topological order of `DEPENDS_ON` (upstream first, ties by name).
  Plugins mirror the edges of `docs/graph/concepts.yaml`; the api crate cannot read yaml.
  `Registry::layer_order()` is strict (unknown dependency or cycle is an error); `install` rejects cycles.
- **core.batch**: one node, atomic, ops applied in sequence. A string `"$i.role"` / `"$i.role[k]"` in a
  later op's params is replaced by an id allocated by op `i` with `alloc_as`. No control flow.
  Format rules: (1) a string starting `$$` is the literal with one `$` removed. (2) Nesting a
  `core.batch` is rejected. (3) The stored node holds each sub-op in canonical form (decoded by
  the registry and re-encoded at the current version, so unknown fields, nulls and int-for-float
  are gone), with the original `$i.role` references and `$$` escapes restored where the field
  survives. References in fields the op does not know are dropped with the field. If the
  canonical record no longer holds the substituted value at a reference path (a migration
  reshaped the params), that sub-op is stored with concrete ids instead, which replays exactly.
  The raw client record is never stored, so typed and JSON clients produce the same node hash.
- **Retire reactions**: `LayerType::references_any(&ids)` (default `true`) lets a layer say through
  `&self` that it holds none of the retired ids; such layers are neither copied-on-write nor listed
  in `Delta::touched_layers`. Override it for large layers. Known limit: `Delta::created` lists
  every allocated id (8 bytes each); range compression is deferred because it changes the frozen
  `Delta` shape.
- **Registry**: ordered maps behind `Arc`; `Clone`; `install(plugin)` stages a clone and commits only
  on success (atomic); registration order never affects the catalog, layer order or new-state hash.
  `register_layer_with(factory)`, `register_migration(op, from, fn)` (op must exist, no duplicates),
  versioned decode with a v -> v+1 migration chain. `catalog()` returns a per-op `Result<OpInfo>`
  (schema title = op name, `tool_name()` mangles `.` to `__`, output schema = `Delta`).

## Timeline

- **Node** = `NodeBody{parent, op: OpRecord, meta}` (hashed) + `Check{state}` (not hashed).
  Node hash = `SHA-256(domain ‖ canonical_bytes(body))` (`canonical_bytes`: type-tagged, length-framed,
  sorted keys). It never depends on TOML text, CRLF, key order or the op's result, so the same op appended
  on another platform is the same node. `from_files` recomputes the hash of every parsed file and
  compares it to the file name (`HashMismatch`). A golden-hash test pins the format.
- **Result check**: `Check.state` is the quantized `State::hash()` after the op. Replay compares it;
  a mismatch is a strict `Diverged` error by default (`ReplayPolicy::Warn` collects instead).
  `Timeline::insert` of the same body with another check is also `Diverged`.
- **Timeline** stores `depth` and a `children` index at insert time, sorted refs, no HEAD.
- **Files** (`to_files`): `ops/<hash>.toml` (immutable, content-addressed) and `refs/heads/<name>` (one
  file per branch). Independent branches add disjoint files; nothing existing is rewritten. Node text is
  written by a small emitter (`text.rs`) that never produces multi-line strings, so autocrlf cannot alter
  values; reading uses the `toml` crate. Ref names `a` and `a/b` cannot coexist (file vs directory).
- **HEAD** is per-checkout state, held by `Session` as `Head` and persisted by `sm-engine` in gitignored
  `.cache/HEAD`. A missing HEAD means `Timeline::default_head()` (`main`, else first branch, else unborn
  `main`), never a silent empty detached state. Needs an ARCHITECTURE §4 amendment (orchestrator).
- **Session**: engine + timeline + head + current state + bounded caches (recent states, default 8, and
  depth-interval snapshots, at most 64). `apply` = `prepare` + `commit`; `PrepareBase::prepare` runs
  off-thread on an owned clone, `commit` fails with `StaleBase` if the position moved. The live path is
  the replay path (encode, decode, apply the decoded record). `checkout`, `state_at`, `open`,
  `reset_branch` take a `Control` (progress, cancellation). `state_at_from_root` replays without caches.
  `undo` moves the attached branch ref to the parent (git reset); the undone node stays in the timeline
  (checkout-able, branch-able); `redo` moves it back; a new op clears the redo stack. `reflog()` is
  in memory; `sm-engine` persists it in `.cache/`.

## Determinism

Everything observable is ordered (`BTreeMap`, sorted keys, fixed chunking in the sample op). The test
suite replays twice, compares 1 vs N worker threads (state hash identical) and checks that an op with
injected nondeterminism raises `Diverged`.

## Requirement map (tests in `tests/api.rs`)

| Requirement | Test |
|---|---|
| op then op referencing its id; replay equal hashes | `create_then_reference_and_replay_twice_...` |
| fork from node 3 of 10, switch back | `fork_from_node_3_of_10_and_switch_back` |
| plugin adds op + layer without api edits | `plugin_registers_its_own_op_and_layer_...` |
| JSON schema of all ops | `json_schema_of_all_registered_ops` |
| progress + cancellation, state untouched | `long_op_...`, `cancelled_long_op_...` |
| failing op leaves state untouched | `failing_op_leaves_state_untouched` |
| to_files/from_files round trip | `to_files_from_files_round_trip_...`, `hashes_do_not_depend_on_text_format` |
| disjoint file sets, no HEAD | `two_branches_created_independently_...` |
| derived cache by layer content hash | `derived_cache_is_keyed_by_layer_content_hashes` |
| undo moves ref, node checkout-able | `undo_moves_the_ref_...` |
| `on_retired` in DAG order | `on_retired_runs_in_declared_dag_order_...` |
| `Diverged` on nondeterminism | `diverged_is_raised_when_...` |
| persisted-snapshot hook | `persisted_snapshot_hook_...` |

## Deviations from the recommendation and decisions

- No `Id` newtype (decision 3: ops use `EntityId`); `resolve`/`resolve_kind` are `ApplyCtx` methods.
- `Delta.outputs` uses `Vec<EntityId>` (no `smallvec`, not an allowed dependency).
- `reset_branch` takes a `Control`; `undo`/`redo` use cached states or a quiet replay (no `Control`).
- Decision 2 drops the hand-written TOML *parser* (done: parsing uses `toml`). A ~110-line *emitter*
  (`text.rs`) is kept because `toml::to_string` writes strings with newlines as multi-line literals,
  which a CRLF checkout (autocrlf) would alter.
- `OpType::schema` returns `Result<Schema, String>`; `register_dyn` takes `Arc<dyn OpType>`;
  `to_files` returns `Result`; `Node.check` is a `Check { state }` struct.
- No `rayon` (not an allowed dependency): the thread test uses `std::thread::scope` and the thread
  count is an op parameter instead of `RAYON_NUM_THREADS`.
- Layer dependency validation: `install` rejects cycles but tolerates not-yet-registered
  dependencies (plugins install in any order); `Registry::layer_order()` is the strict check and
  `Engine::new` runs it (fallible) and caches the order, so a bad registry fails at startup.
- Op names must match `^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)+$` without `__` (`RegistryError::BadOpName`); client records are decoded and re-encoded canonically before hashing; `core.batch` strings starting `$$` are literal `$...`; `on_retired` runs after each batch sub-op.
- The "no git conflict" test simulates a git merge as a union of file maps; it does not run real git.
- ARCHITECTURE §4 (HEAD location) still needs the orchestrator's amendment.
