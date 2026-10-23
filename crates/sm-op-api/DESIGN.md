# sm-op-api: design (candidate: closed-enum)

## Model

Every state change is an **op**: a value of a closed, serde-tagged enum. Each layer owns one such
enum plus one store; the api crate composes four of them and knows none of their variants.

```
StackOp<M,P,I,X> = { layer = "model"|"partition"|"intent"|"meshes", args = <that layer's enum> }
ModelOp          = #[serde(tag = "kind")] CreateBrick{..} | Translate{..} | Delete{..}   (per layer crate)
```

`State<S>` holds one `Arc`'d store per layer plus the `IdAllocator` and a shared `DerivedCache`.
`State::apply(&self, &Op, &dyn Control) -> Result<(State, Delta), OpError>` is pure: it clones
the (pointer-sized) state, runs the layer's `apply`, and returns the new state; `self` is never
touched. A **layer** implements `Layer` on its store type: `NAME`, `OP_VERSION`, `type Op`, and
`fn apply(&self, &Op, &mut ApplyCtx) -> Result<Self, OpError>`. Layer dependencies follow the
acyclic concept graph: `ApplyCtx::upstream` gives read-only access to the layers above
(partition sees model; intent sees model+partition; meshes sees all three), by type.

The timeline is `Node = hash(NodeBody{format, parent?, op_version, meta{author,message,time}, op})`.
`Timeline<O>` keeps nodes, branch refs and HEAD (`Branch(name)` or `Detached(hash)`).
`Session<S>` is the reference in-memory engine: current state, timeline, snapshots every N depth.

## Key types

| Type | Role |
|---|---|
| `Layer` | store + closed `Op` enum + pure `apply` (what a concept node registers) |
| `StackOp`, `Stack4`, `OpOf<S>` | composition of four layer enums / stores |
| `State<S>` | layer stores + ids + derived cache; `apply`, `state_hash`, `derived_or_compute` |
| `ApplyCtx`, `Delta`, `Upstream` | id allocation, upstream reads, progress/cancel, created/modified/removed |
| `Id`, `IdAllocator` | serde wrapper of `sm_core::EntityId` (`"TopoVolume#3"`); per-kind never-reused counters |
| `Control`, `CancelToken`, `NoControl` | progress sink + cancellation, never affects output |
| `Node`, `NodeBody`, `Timeline`, `Head` | content-addressed DAG, refs, HEAD, file layout helpers |
| `Session` | apply / checkout / branch / `state_at` with snapshots; `from_timeline` replays |
| `Hash256`, `canonical_bytes`, `content_hash` | in-crate SHA-256 and canonical serde encoding |
| `DerivedCache` | `(kind, content hash) -> Arc<dyn Any>` |

## How each requirement is met

- **Serializable, versioned, deterministic ops.** Closed enums with serde; `op_version` (from
  `Layer::OP_VERSION`) is stored in each node and checked on replay (`SessionError::OpVersion`).
  A schema change bumps the version; migration is an explicit timeline rewrite (new hashes), not
  silent.
- **Content addressing and canonical serialization.** The hash is `SHA-256(domain || canonical_bytes)`
  where `canonical_bytes` is our own serde encoding (type tags, LE lengths, `f64::to_bits`, `-0.0`
  normalised, NaN rejected). It does not depend on TOML text, so reformatting or reordering a
  file keeps the hash (tested), and on load the hash is recomputed and must match the file name.
  A golden-hash test pins the encoding. `std` hashers are never used.
- **TOML per node, refs/HEAD like git.** `timeline::{node_path, ref_path, HEAD_PATH}` and
  `Timeline::{to_files, from_files}` produce/consume `timeline/ops/<hash>.toml`,
  `timeline/refs/heads/<name>`, `timeline/HEAD` (`ref: refs/heads/main`). The text codec is
  injected, so the library has no TOML dependency (`toml` is a dev-dependency used by the tests,
  including a real on-disk round trip). Actual file I/O belongs to `sm-engine`.
- **Checkout / branch / fork.** `Session::checkout(rev)` (branch, hash, prefix, `HEAD`),
  `branch(name)`, `state_at(node)` = replay from the nearest ancestor snapshot, falling back to
  the root. Applying at a detached or old HEAD forks; identical parent+op+meta is the same node.
- **Stable references between ops.** `IdAllocator` is part of the state and committed only on
  success, so ids are a pure function of the op prefix: replay and forks reassign identical ids.
  `Delta::created` lists new ids in allocation order, and later ops name them by their string.
- **JSON schema.** Every op enum derives `schemars::JsonSchema`; `op_schema::<S>()` yields the
  schema of the whole composed op (MCP tools, help, Python stubs from one source).
- **Transactional apply.** Pure `apply` plus clone-then-swap: errors return before anything is
  committed; the timeline node is inserted only after the state change succeeded (tests: failing
  mid-op, cancelled op, allocator not advanced).
- **Progress / cancellation.** `ApplyCtx::step(done,total,msg)` reports and returns
  `OpError::Cancelled` when the `Control` says so; `Session::apply_with` takes the control.
- **Composable layers without the api crate knowing them.** The api fixes only the four layer
  *slots* of `ir.md` section 5; concrete stores and ops come from the layer crates via type
  parameters (`Stack4<M,P,I,X>`). `sample` shows the pattern.
- **Derived caches by content hash.** `State::derived_or_compute(kind, &["meshes", ..], f)` keys
  by the hashes of the named layers; the cache is shared by all states of a session and is outside
  the state hash.

## Tradeoffs

- **Closed enums**: exhaustive matches, one schema, easy diffs, `#[serde(tag)]` TOML that humans
  read. Cost: a new op means editing the owning layer's enum (a recompile of its dependents); no
  runtime plugin ops. The slot set (four layers) is fixed here; adding a fifth is an api change
  (consistent with the orchestrator-gated contract rule).
- **Pure `apply(&self) -> Self`** makes transactions free but needs cheap clones: layer stores
  should hold big tables behind `Arc`/persistent structures. The sample stores clone `BTreeMap`s.
- **State hash** serializes whole layers (O(size)); hashing per op is fine for tests, and an
  incremental/per-chunk hash is a later optimisation. It hashes exact bits; the cross-platform
  quantised CI hash (ARCHITECTURE section 3) is a layer-level concern (`Serialize` can quantise).
- **Single parent** per node; merges are not modelled (not needed in the architecture).
- **Id serialization is a string** (diff-friendly) rather than raw `u64`; sm-core should gain
  serde/schemars derives so the wrapper can go away (api change request).

## Not done / for sm-engine

File I/O and `project.toml`; persisting snapshots to `.cache`; migration tooling between op
versions; undo/redo as HEAD moves; batch/macro ops with intra-batch references.
