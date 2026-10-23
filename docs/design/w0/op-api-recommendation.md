# sm-op-api: final design recommendation

## 1. Scores

The scores are judge averages (J0 / J1, then the mean of the two).

| Dimension | closed-enum | registry | deltas |
|---|---|---|---|
| Expressiveness | 6 / 5 → 5.5 | 7 / 6 → **6.5** | 7 / 6 → **6.5** |
| Ergonomics | 7 / 6 → **6.5** | 7 / 5 → 6.0 | 7 / 5 → 6.0 |
| Performance | 5 / 4 → **4.5** | 5 / 4 → **4.5** | 4 / 4 → 4.0 |
| Determinism | 9 / 8 → **8.5** | 8 / 8 → 8.0 | 7 / 8 → 7.5 |
| Extensibility | 4 / 4 → 4.0 | 8 / 8 → **8.0** | 5 / 7 → 6.0 |
| Correctness | 7 / 7 → **7.0** | 7 / 7 → **7.0** | 6 / 6 → 6.0 |
| **Mean** | 6.00 | **6.67** | 6.00 |

## 2. Winner: `w0/op-api/registry` (b17bf76)

1. **It is the only candidate that meets the composable-State requirement.** That requirement says each concept node registers its own ops and layers without the api crate knowing them. Both judges confirmed that `Plugin` + `register_layer::<L>()` + `register::<T>()` adds a scheme-tet layer and op with no api edits.
   - closed-enum fails structurally: it has four fixed slots, versions per slot, and needs a central composite.
   - deltas meets it but is less safe: installing a concept changes every state hash, and installs are not atomic.
2. **Its transaction model is simple and needs no per-layer cooperation.** It applies the op to a copy-on-write `State` clone and swaps it in only on `Ok`. That rollback covers layers, the entity table, the counters, and (today) the cache. deltas instead depends on every layer returning a correct inverse and does not check it after rollback; J1 s1b shows the leak.
3. **The live path is the replay path.** `Session::apply` encodes the op, decodes it, then applies the decoded record. Each node records its result hash, and a mismatch raises `Diverged`. That gives the strongest detection of nondeterminism.
4. **Versioning is right.** Each op has its own version and a v→v+1 migration chain on decode. closed-enum has one version per layer and makes old nodes fatal on mismatch.
5. **Ids are right.** The entity table and per-kind counters live in `State`, serials are never reused, and ids are a pure function of the op prefix.

**Deltas is not the winner.** Persisted reversible deltas are 3.8–32 MB TOML per mesh op committed to git. That conflicts with the ARCHITECTURE §4 rule to commit ops, not derived data, and with the array backends in addendum §7.1. Most of the delta machinery is cost without a requirement behind it. Fast checkout is better served by a snapshot and recent-state cache.

## 3. Ideas to graft from the runners-up

**From closed-enum:**
- **G1. Readable string ids.** Replace `EntityRef(u64)` with an `Id` newtype that serializes as `"TopoVolume#3"` and carries a JSON-schema `pattern`. This fixes three defects at once: raw u64 ids are unreadable in TOML, JSON clients lose precision above 2^53, and the schema says uint64 while the codec rejects values above i64::MAX.
- **G2. Hash a canonical binary encoding, not TOML text.**
  - Node hash = `SHA-256(len‖domain‖canonical_bytes(NodeBody))`.
  - On load, recompute the hash from the parsed value and check it against the file name.
  - Hashes then no longer depend on TOML formatting, so CRLF/autocrlf no longer breaks loading. Swapping the text codec later (e.g. to the real `toml` crate) cannot change hashes.
  - Normalize `-0.0` to `+0.0` and reject NaN **in the `Value` encoder**, so the file text and the hash agree. closed-enum normalized only in the hash.
  - Keep a golden-hash test.
- **G3. `ApplyCtx::step(done, total, msg) -> Result<(), OpError>`**, which reports progress and checks cancellation in one call.
- **G4. `Session::state_at_from_root(h)`**, to validate snapshots and test determinism.
- **G5. A pure text codec.** `Timeline::to_files() -> BTreeMap<String, String>` and `Timeline::from_files(&BTreeMap<..>)` hold the exact on-disk layout as path → text, an idea both closed-enum and deltas' `MemStore` had. `fs::save/load` become thin wrappers over them, and tests can corrupt files directly.

**From deltas:**
- **G6. Memoize layer hashes.** Each layer slot memoizes its content hash in a `OnceLock<Hash256>`, cleared on `layer_mut`. State hash = hash over (name, layer hash). A trivial op then costs O(touched layers), not O(state). `Delta.touched_layers` is already the right input for this.
- **G7. Empty layers do not affect the state hash.** Add `fn is_empty(&self) -> bool` to `LayerType` and skip empty layers in `State::hash`. Installing a new concept then does not change the hash of existing histories.
- **G8. Atomic `Registry::install`.** Register into a staging copy and merge only on `Ok`.
- **G9. A registration-order-independence test.**
- **G10. Progress can be called from worker threads.** Require `Progress: Sync` so that `&ApplyCtx` can call `step` from scoped rayon workers.
- **G11. A reference `MapLayer<T>`** (ordered `BTreeMap<Id, T>`) as the template for concept stores. Drop its patch/inverse machinery; it only needs `digest`.

**Not grafted:** persisted deltas, `Layer::Patch` inverses and `catch_unwind`. The scratch clone already makes panics safe.

## 4. Defects to fix in the registry design

**D1. HEAD in the shared tree (blocking).** `fs::save` writes `timeline/HEAD` into the tracked tree. Every concurrent branch then conflicts in git on HEAD; J1 reproduced this with real git. HEAD is per-working-copy state, so write it to `.cache/HEAD` (gitignored) and write only `ops/` and `refs/heads/` to the shared tree. A missing HEAD means "attach to `main` if it exists"; it must never silently become `Detached(None)` (fs.rs:109). **This requires an ARCHITECTURE §4 edit**; see Q1.

**D2. The node hash includes the result state hash (blocking).** With this rule, the same op appended on macOS and on Linux gets different node hashes whenever float bits differ, and loading a project on the other platform raises `Diverged`.
- Fix: the node body is {parent, op, meta}. The recorded result goes in an unhashed `[check]` table holding the quantized, platform-independent state hash.
- Replay compares that check hash. A mismatch is `Diverged` by default, with a `ReplayPolicy::Warn` option.

**D3. Hashing cost is O(state) on every op and every replay step.**
- Apply G6 for layers.
- Wrap the entity table in `Arc` (copy-on-write; today it is deep-copied twice per op) and hash it with an incremental multiset hash: the sum mod 2^256 of `sha256(id)`. That is O(1) per alloc/retire and independent of order.
- Hash `Digest::value` over canonical bytes, not TOML text.

**D4. Long ops lock the session.**
- `Session::apply_with(&mut self)` holds the session for the whole op. Split it into `prepare(&self, op, ctl) -> Prepared` (runs off-thread on a cloned `State`) and `commit(&mut self, Prepared, Meta)`. `commit` fails with `StaleBase` if the position moved.
- Replay paths (`state_at`, `with_timeline`, `checkout`) also take `&Control`, so they can report progress and be cancelled.

**D5. The derived cache lives inside State.** Today it is copied on write, discarded on failure, cancel or checkout, writable only by ops, and unbounded. Move it out of `State`:
- `Arc<DerivedCache>` is owned by the Session and shared with every state, using interior mutability. It holds a bounded LRU sized by a byte estimate.
- It is reachable from `ApplyCtx::derived()`, so meshing can reuse cached tracking structures, and from `Session::derived()`, so readers and UI can fill it.
- This is safe because entries are pure functions of their content key, and the contract states that a cache hit must not change op output.
- Typed dependencies: `state.cache_key(kind, &[L::NAME…])` returns `Err(MissingLayer)` on an unknown name instead of `None` or a silent 0.
- Replacing an entry with a different type under the same key is an error.

**D6. Checkout and snapshot cost.**
- Keep the state of the position being left, plus branch tips, in a bounded LRU of recent states (default 8). Keep the depth-interval snapshots, but bound them.
- Store `depth` and a `children` index in `Timeline` at insert time. This removes `path_to_root` from every apply (today O(n²) over a session).
- Load in topological order using indegree counts, not repeated passes.

**D7. Dangling references across layers.** An op in an upstream layer cannot clean downstream layers it does not know about. For example, model `Delete` leaves intent sizes and partitions pointing at the deleted id.
- Add `LayerType::on_retired(&mut self, ids: &[EntityId])`. After the op body, the engine calls it on every non-empty layer whose role is authoritative, in name order. The default does nothing.
- This is generic, deterministic and part of the same transaction.

**D8. Compound edits.** Add a built-in `core.batch { ops: Vec<OpRecord> }`: one node, atomic, applied in sequence. Later ops in the batch refer to ids from earlier ones through named outputs (D9) as `"$0.volume"`. Keep it minimal: no control flow.

**D9. Unlabeled outputs.**
- `Delta.created` is a positional list. Add `ctx.alloc_as(role, kind)`, which fills `Delta.outputs: BTreeMap<String, SmallVec<[Id; 1]>>`.
- Add `ctx.alloc_sorted(kind, keys)` for addendum §7.2: sort by (primary parent, ordinal), then allocate a contiguous range.

**D10. Kind checks and services.**
- `Id::resolve_kind(ctx, EntityKind)` checks the kind as well as liveness.
- Add a generic `ctx.state().contains(id)` / `kind_of(id)`, so a cross-concept op does not need the other concept's concrete layer type for an existence check.
- Add `ApplyCtx::service::<S>()` for a geometry kernel or mesher, injected at `Engine::new` through `Services: TypeMap`. Without it, Webcut and meshing need globals; deltas' judge flagged this as d.

**D11. Registry hygiene.**
- `register_migration` should return `Result`, check that the op exists and reject duplicates. Use `Box<dyn Fn(Value) -> Result<Value> + Send + Sync>` instead of a bare fn pointer.
- Add `register_layer_with(factory)` for layers that need configuration (drop the hard `Default` requirement).
- Use `#[serde(deny_unknown_fields)]` on the node, meta and check tables.

**D12. MCP catalog fixes.**
- Set the schema `title` to the op NAME.
- Add `OpInfo::tool_name()` to mangle `.` → `__` for tool-name patterns.
- Have `catalog()` return per-op `Result` so one bad schema does not kill the whole list.
- Add an optional output schema (the Delta and outputs shape).

**D13. Undo/redo and branch refs.**
- Add `Session::reset_branch(name, h)`.
- `undo()` moves the attached branch ref to its parent and pushes the old tip onto a session-local redo stack. `redo()` pops it.
- Add a local reflog in `.cache/`.
- Re-applying the same op and meta reproduces the same node.
- Document that sibling branches reuse serials (they are a pure function of the prefix, per ARCHITECTURE §3a), so UI selections are scoped to a node.

**D14. Control.** Add a `Control { progress: &dyn Progress, cancel: &CancelToken }` bundle. `CancelToken` no longer swallows progress reports, and `ctx.sub(range)` scopes progress for nested stages.

**D15. Write order and atomicity.**
- Insert the node into the timeline only after every ref name has been validated. That part already holds; keep it under test.
- `fs` writes to a temp file and renames, writes nodes before refs, and never writes HEAD to the shared tree.

**D16. Value codec.** Keep it free of null for TOML, but convert the schemars JSON through `schema.as_value()` (serde_json, which schemars already pulls in), never through `Value`. That avoids deltas' `"default": {}` bug.

## 5. Proposed public API

```rust
// ---- ids
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Id(pub EntityId);            // serde: "TopoVolume#3"; JsonSchema: string + pattern
impl Id {
    pub fn resolve(self, ctx: &ApplyCtx) -> Result<EntityId, OpError>;
    pub fn resolve_kind(self, ctx: &ApplyCtx, k: EntityKind) -> Result<EntityId, OpError>;
}

// ---- ops
pub trait TypedOp: Serialize + DeserializeOwned + JsonSchema + Debug + Send + Sync + 'static {
    const NAME: &'static str;           // "model.create_brick"
    const VERSION: u32 = 1;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
}
pub trait Op: Debug + Send + Sync + 'static {   // blanket impl for TypedOp
    fn name(&self) -> &'static str; fn version(&self) -> u32;
    fn encode(&self) -> Result<Value, CodecError>;
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError>;
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpRecord { pub name: String, pub version: u32, pub params: Value }

pub struct ApplyCtx<'a> { /* scratch State, &Control, &Services, Arc<DerivedCache>, journal */ }
impl ApplyCtx<'_> {
    pub fn state(&self) -> &State;
    pub fn layer<L: LayerType>(&self) -> Result<&L, OpError>;
    pub fn layer_mut<L: LayerType>(&mut self) -> Result<&mut L, OpError>;
    pub fn alloc(&mut self, k: EntityKind) -> Result<EntityId, OpError>;
    pub fn alloc_as(&mut self, role: &str, k: EntityKind) -> Result<EntityId, OpError>;
    pub fn alloc_sorted<K: Ord>(&mut self, k: EntityKind, keys: &[K]) -> Result<Vec<EntityId>, OpError>;
    pub fn retire(&mut self, id: EntityId) -> Result<(), OpError>;
    pub fn require(&self, id: EntityId) -> Result<(), OpError>;
    pub fn derived(&self) -> &DerivedCache;
    pub fn service<S: Any + Send + Sync>(&self) -> Result<&S, OpError>;
    pub fn step(&self, done: u64, total: u64, msg: &str) -> Result<(), OpError>; // progress + cancel
    pub fn sub(&self, lo: f32, hi: f32) -> ApplyCtxScope<'_>;
}
pub struct Delta {
    pub created: Vec<EntityId>, pub removed: Vec<EntityId>,
    pub outputs: BTreeMap<String, SmallVec<[EntityId; 1]>>, pub touched_layers: Vec<&'static str>,
}

// ---- layers / state
pub enum LayerRole { Authoritative, Derived }
pub trait LayerType: Clone + Send + Sync + 'static {
    const NAME: &'static str; const ROLE: LayerRole = LayerRole::Authoritative;
    fn digest(&self, d: &mut Digest);            // must quantize floats for the CI hash
    fn is_empty(&self) -> bool;                  // empty layers don't enter State::hash
    fn on_retired(&mut self, _ids: &[EntityId]) {}
}
pub struct Digest; impl Digest { bytes, str, u64, i64, f64_quantized, hash, value<T: Serialize> }
#[derive(Clone)] pub struct State { /* BTreeMap<&str, Slot{Arc<dyn Layer>, OnceLock<Hash256>}>, Arc<Entities>, counters */ }
impl State {
    pub fn layer<L: LayerType>(&self) -> Option<&L>;
    pub fn contains(&self, id: EntityId) -> bool; pub fn kind_of(&self, id: EntityId) -> Option<EntityKind>;
    pub fn peek_next(&self, k: EntityKind) -> Option<EntityId>;
    pub fn hash(&self) -> Hash256;               // O(touched); quantized-digest based
    pub fn layer_hash(&self, name: &str) -> Result<Hash256, OpError>;
    pub fn cache_key(&self, kind: &str, inputs: &[&str], params: &Value) -> Result<Hash256, OpError>;
}
pub struct DerivedCache;                          // Session-owned, Sync, bounded LRU
impl DerivedCache {
    pub fn get<T: Any + Send + Sync>(&self, kind: &str, key: Hash256) -> Option<Arc<T>>;
    pub fn get_or_compute<T, E>(&self, kind: &str, key: Hash256, f: impl FnOnce() -> Result<T, E>) -> Result<Arc<T>, E>;
}

// ---- registry
pub trait Plugin { fn id(&self) -> &str; fn register(&self, r: &mut Registry) -> Result<(), RegistryError>; }
impl Registry {
    pub fn new() -> Self;                        // includes core.batch
    pub fn install(&mut self, p: &dyn Plugin) -> Result<&mut Self, RegistryError>; // atomic
    pub fn register<T: TypedOp>(&mut self) -> Result<&mut Self, RegistryError>;
    pub fn register_layer<L: LayerType + Default>(&mut self) -> Result<&mut Self, RegistryError>;
    pub fn register_layer_with<L: LayerType>(&mut self, f: impl Fn() -> L + Send + Sync + 'static) -> Result<&mut Self, RegistryError>;
    pub fn register_migration(&mut self, op: &str, from: u32, m: impl Fn(Value) -> Result<Value, CodecError> + Send + Sync + 'static) -> Result<&mut Self, RegistryError>;
    pub fn new_state(&self) -> State;
    pub fn catalog(&self) -> Vec<Result<OpInfo, RegistryError>>;   // sorted
    pub fn decode(&self, rec: &OpRecord) -> Result<Box<dyn Op>, RegistryError>;
}
pub struct OpInfo { pub name: String, pub version: u32, pub schema: schemars::Schema }
impl OpInfo { pub fn tool_name(&self) -> String; }

// ---- engine / control
pub trait Progress: Send + Sync { fn report(&self, fraction: f32, msg: &str); }
#[derive(Clone, Default)] pub struct CancelToken(Arc<AtomicBool>);
pub struct Control<'a> { pub progress: &'a dyn Progress, pub cancel: &'a CancelToken }
impl Engine {
    pub fn new(reg: Arc<Registry>, services: Services, cache: Arc<DerivedCache>) -> Self;
    pub fn apply(&self, s: &mut State, op: &dyn Op, ctl: &Control) -> Result<Delta, ApplyError>; // transactional
    pub fn apply_record(&self, s: &mut State, r: &OpRecord, ctl: &Control) -> Result<Delta, ApplyError>;
}

// ---- timeline
pub struct Meta { pub author: String, pub time: String /* caller-supplied */, pub message: String }
pub struct NodeBody { pub parent: Option<Hash256>, pub op: OpRecord, pub meta: Meta } // hashed
pub struct Node { pub body: NodeBody, pub check: Hash256 /* quantized result hash, unhashed */ }
pub enum Head { Ref(String), Detached(Option<Hash256>) }
impl Timeline {
    pub fn insert(&mut self, n: Node) -> Result<Hash256, TimelineError>;
    pub fn get(&self, h: Hash256) -> Option<&Node>; pub fn depth(&self, h: Hash256) -> Option<usize>;
    pub fn children(&self, h: Hash256) -> &[Hash256]; pub fn refs(&self) -> &BTreeMap<String, Hash256>;
    pub fn set_ref / create_ref / delete_ref;
    pub fn to_files(&self) -> BTreeMap<String, String>;          // ops/<h>.toml, refs/heads/*  (no HEAD)
    pub fn from_files(f: &BTreeMap<String, String>) -> Result<Self, TimelineError>; // verifies hashes
}
pub mod fs { pub fn save(dir: &Path, t: &Timeline, head: &Head) -> io::Result<()>;  // HEAD → .cache/HEAD
             pub fn load(dir: &Path) -> Result<(Timeline, Head), TimelineError>; }

// ---- session
pub enum Rev { Empty, Node(Hash256), Ref(String) }
pub struct Prepared { /* base, record, State, Delta */ }
impl Session {
    pub fn new(engine: Engine) -> Self;
    pub fn open(engine: Engine, t: Timeline, head: Head, ctl: &Control) -> Result<Self, TimelineError>;
    pub fn state(&self) -> &State; pub fn position(&self) -> Option<Hash256>;
    pub fn timeline(&self) -> &Timeline; pub fn derived(&self) -> &Arc<DerivedCache>;
    pub fn apply(&mut self, op: &dyn Op, meta: Meta, ctl: &Control) -> Result<(Hash256, Delta), TimelineError>;
    pub fn prepare(&self, op: &dyn Op, ctl: &Control) -> Result<Prepared, TimelineError>; // off-thread
    pub fn commit(&mut self, p: Prepared, meta: Meta) -> Result<(Hash256, Delta), TimelineError>;
    pub fn checkout(&mut self, rev: Rev, ctl: &Control) -> Result<(), TimelineError>;
    pub fn branch(&mut self, name: &str) -> Result<(), TimelineError>;
    pub fn reset_branch(&mut self, name: &str, to: Hash256) -> Result<(), TimelineError>;
    pub fn undo(&mut self) -> Result<(), TimelineError>; pub fn redo(&mut self) -> Result<(), TimelineError>;
    pub fn state_at(&mut self, h: Option<Hash256>, ctl: &Control) -> Result<State, TimelineError>;
    pub fn state_at_from_root(&self, h: Hash256, ctl: &Control) -> Result<State, TimelineError>;
    pub fn set_cache_policy(&mut self, snapshot_every: usize, recent_states: usize);
}
// hashing: Hash256, sha256 (in-crate, FIPS vectors), canonical_bytes, content_hash(domain, &T)
```

**Required tests:** the six from the brief, plus:
- golden node hash;
- CRLF-insensitive load;
- the git-merge simulation with no HEAD conflict;
- installing a new concept leaves old hashes unchanged;
- `prepare`/`commit` with a stale base;
- cache survives checkout and cancel;
- `on_retired` cleans downstream layers;
- `core.batch` with an intra-batch reference;
- undo/redo staying attached to the branch;
- registration-order independence;
- the 1-thread vs N-thread hash test from CLAUDE.md, run over a sample op that uses rayon inside `step`.

## 6. Open questions for the orchestrator

1. **HEAD location.** Can ARCHITECTURE §4 be amended to move `timeline/HEAD` into the gitignored `.cache/` (or `timeline/HEAD` plus a `.gitignore` entry)? Without this the "concurrent branches never conflict" claim is false.
2. **Dependencies.**
   - `schemars` already pulls in `serde_json` transitively. Can `serde_json` be allowed directly?
   - Can `toml` be allowed as the text codec? With G2 the hash no longer depends on the text, so the roughly 1,000-line hand-written TOML parser could be dropped without changing any hash.
   - Keep the in-crate SHA-256, or allow `sha2`? I recommend keeping the in-crate one.
3. **sm-core change request.** Add optional `serde` + `schemars` derives to `EntityId`, `Point3`, `Vector3` and `Transform`, with `EntityId` serializing as `"Kind#serial"`. Without it, every op mirrors these types and `Id` stays a wrapper (orphan rule).
4. **Node identity across platforms.** Confirm D2: the node hash excludes the result, and the recorded result check uses the quantized state hash. Strict `Diverged` by default, or warn?
5. **Cross-layer consistency.** Is the `on_retired` hook (D7) enough for P0–P1, or do concept nodes need ordered reactions (e.g. partition before intent)? The current proposal runs them in layer-name order. The alternative is the order of the concept DAG in `docs/graph/concepts.yaml`.
6. **Undo semantics.** Should undo move the branch ref (git-reset style, with the reflog in `.cache`), or append inverse ops as new nodes, which keeps undo in the shared history? I recommend moving the ref.
7. **Where file I/O lives.** Should `fs::save/load`, atomic writes and `project.toml` live in sm-op-api, or should they move to sm-engine, leaving only `to_files`/`from_files` here?
8. **Large mesh layers.** Mesh layers can be very large (addendum §7.1 array backends). Should `LayerType` gain an optional persisted-snapshot hook (`Serialize + DeserializeOwned` into `.cache`) now, or should it wait for the storage API?