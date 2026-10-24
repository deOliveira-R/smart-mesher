# sm-engine: design

## Positioning

`sm-engine` is similar to **git's storage model** (immutable content-addressed objects, one small
file per ref, a per-checkout HEAD and reflog) and to **Cubit's journal/undo** (every modeling
action is recorded and replayable). It differs in what is versioned and when it is saved: the
project stores the **op DAG itself**, each node carrying the hash of the state it produced, and it
saves **after every action** with crash-safe incremental writes, not on an explicit "save" or by
appending to one linear journal file. That gives:

- fork, checkout, undo and replay that reproduce state hashes bit-for-bit, verified on every open;
- project files that git merges without conflicts when independent branches are added (node files
  are never rewritten, each branch owns one ref file; the only possible text conflict is two
  writers moving the same branch);
- one JSON facade (`Workspace`) over registered plugins, so a server, an MCP adapter or Python
  bindings drive the same operations without knowing any op type.

## Scope

The contract crate `sm-op-api` owns ops, layers, state, hashing, the timeline DAG and `Session`
(replay, undo, branches). It performs no I/O. `sm-engine` adds, per W0a decision 7:

| Piece | Module |
|---|---|
| Project directory format, `project.toml` | `project`, `fs` |
| Atomic writes, write lock, crash injection | `fsio` |
| `.cache/`: HEAD, reflog, redo stack, snapshots | `cache` (private) |
| Diagnostics policy, clocks | `policy` |
| Facade: plugins, apply/query/checkout/fork/undo, autosave, open | `workspace` |

## On-disk format (format version 1)

```
<project>/
  project.toml                 format, name, [units], [tolerances], [metadata], unknown keys kept
  .gitignore                   "/.cache/" (written at create)
  timeline/ops/<hash>.toml     node files, text exactly as Timeline::node_file
  timeline/refs/heads/<name>   "<hash>\n"; nested names are directories
  .cache/                      gitignored, per checkout; never required
    HEAD                       one line of JSON: {"ref":"main"} | {"node":"<hash>"} | "empty"
    reflog.jsonl               append-only, one ReflogEntry per line (newest 10 000 kept at open)
    redo.json                  redo stack, bottom first
    snapshots/<node>.snap      state snapshot (binary container, SHA-256 trailer)
    lock                       advisory lock held while the project is open for writing
```

`project.toml` is validated so that it survives a CRLF checkout (no control characters in
strings) and has no timestamps, so it does not change when nothing changed. Unknown top-level keys
are kept in `ProjectConfig::extra` and written back; a `format` newer than 1 is refused
(`unsupported-format`). Node and ref text come from `Timeline::node_file` / `ref_file`, so the
files equal `Timeline::to_files`, and `Timeline::from_files` re-hashes every node against its
file name on load.

HEAD is per checkout (W0a decision 1), so it lives in `.cache/`. ARCHITECTURE §4 still shows
`timeline/HEAD` and needs the orchestrator's amendment (see the report).

## Atomic writes and the crash argument

Every file is written by `atomic_write`: temp file in the same directory (`<name>.<pid>-<n>.lock`),
`fsync` (with `Durability::Full`), `rename` over the target, directory `fsync`. A reader or a
crash sees the old file or the complete new one. Temp names end in `.lock`, which no ref name may
(`valid_ref_name`) and no node file name matches (`<64 hex>.toml`), so they can never be loaded as
data; `open` deletes them (the write lock makes that safe).

`Workspace::save` (also run after each mutating call when autosave is on) writes in this order:

1. node files for the nodes recorded or re-baselined since the last save (immutable, so a crash
   cannot damage an existing node);
2. refs that clash with a ref being written (file versus directory, or case-only) are deleted;
3. new and moved ref files; then **HEAD**; then the remaining deleted refs;
4. redo stack; reflog append.

Consequences, each pinned by a test: a ref never points at a node that is not on disk; a branch is
never lost to a rename or delete (the new name is written before the old is removed); HEAD never
names a branch that is already gone; after a crash at any point the project opens, every branch tip
replays to its recorded state hash, and the workspace accepts further ops. The one exception is
documented: renaming a branch across a file/directory shape (`a` to `a/b`) must delete `a` first,
so a crash in between loses that name (its nodes stay).

Incrementality: autosave touches one node file, one ref file, and appends one reflog line,
independent of timeline length (`autosave_writes_only_what_changed` counts the I/O points).
The dirty node set is explicit (`apply` and re-baseline add to it); refs are diffed against what
is on disk, which costs O(branches).

`FaultInjector` makes this testable: each atomic write has three points (half-written temp,
complete temp, renamed) and each delete one. `tests/crash.rs` runs a scenario once to count the
points, then once per point, killing the writer there. Writes run sequentially while an injector is
installed, so numbering is deterministic. Mutation checks confirm the sweeps catch refs-before-nodes
and HEAD-after-deletes orderings.

`Durability::Fast` skips `fsync` (process crashes stay safe, power loss may leave a ref ahead of its
node). The default is `Full`.

## Open

1. read and validate `project.toml`; take the lock (not for read-only); sweep temp files;
2. read `timeline/` (files in parallel with rayon, results collected in path order so the first
   error reported never depends on scheduling), `Timeline::from_files`;
3. HEAD from `.cache/HEAD`. A damaged or dangling HEAD (a branch deleted by a pull, a node that no
   longer exists) falls back to `Timeline::default_head` with a warning. An unborn branch is kept
   only for an empty project or when the redo stack shows the branch was undone to its root;
4. reflog (a torn tail is tolerated and the file rewritten), redo stack (dropped with a warning if
   it no longer chains to the position);
5. snapshot seed: the nearest ancestor of HEAD that has a snapshot file; it is restored with
   `Registry::restore`, must match both its own stored hash and the node's recorded `check.state` (our
   check, before `Session::seed_state`, so it holds under every replay policy), otherwise it is deleted and a warning is recorded, never used;
6. `Session::open_with` replays from the seed, strictly by default (`Diverged` / `Upgraded` stop
   the open). `ReplayPolicy::Warn` / `Rebaseline` are available through the builder; re-baselined
   node files are rewritten.

A missing or garbage `.cache/` is always survivable: nothing there is authoritative.

## Branches, undo, redo

`fork(name, at)` = `create_branch_at` + checkout of the new branch (rolled back if the replay
fails or is cancelled). `delete_branch`, `rename_branch`, `reset_branch`, `checkout(Rev)`, `undo`,
`redo` delegate to `Session` and then save. Undo moves the branch ref (git reset) and keeps the
node, the redo stack is persisted, so undo/redo survive a restart. The facade refuses to create or
rename onto the name of the unborn branch HEAD is attached to (the session would otherwise keep an
empty position under a born branch; see the report).

## Diagnostics policy

`DiagnosticPolicy { verbosity, stop_on_warning, suppress }`. Diagnostics are not part of any hash,
so policy never changes node or state hashes. `apply` runs `Session::prepare`, looks at the
`Delta`, and only then `commit`s: with `stop_on_warning` a non-suppressed warning fails the call
with `stopped-on-warning`, nothing is recorded, the state and id counters are untouched. Per call,
`accept_warnings` overrides the stop. `verbosity` and `suppress` only filter what the
`ApplyReport` shows (`hidden_diagnostics` counts the rest).

## Facade

`Workspace::builder().plugin(..).services(..).policy(..).clock(..).durability(..)` then
`memory()`, `create(path, config)` or `open(path)`. JSON entry points: `apply_json(op, params,
&ApplyOptions)` (params canonicalized, migrated, re-encoded before hashing, so JSON and typed
clients record the same node), `query`, `query_at(rev)`, `catalog_json`, `status`, `log`;
`Status`, `ApplyReport`, `LogEntry`, `ApplyOptions` are `Serialize` (and `ApplyOptions`
`Deserialize`) for a server. Long ops can run off-thread: `prepare_base()` then
`commit_prepared()`. Errors carry stable kebab-case `code()`s. If saving fails after an op was
recorded, the error is `not-persisted { node }`: the op is in memory, `dirty()` is true, and
`save()` retries from where it stopped.

## Determinism

No `HashMap` iteration influences output; everything is `BTreeMap`/sorted. Parallelism is limited
to reading and writing independent files; results are collected in index order and errors are the
first in index order. `tests/determinism.rs` runs a 200-op, three-branch session under
`sm_testkit::determinism::check` (repeat, 1 vs 4 rayon threads, concurrent runs) and compares every
state hash and a hash of all project files; a fixed clock makes node hashes reproducible too.
Wall-clock time only enters through the `Clock` (node `time`), and temp-file names (never observable
after success) use pid and a counter.

## Performance

Criterion bench `benches/engine.rs` (4-core sandbox, noisy disk, so treat I/O figures as
indicative):

| Bench | Time |
|---|---|
| `apply` in memory (typed / JSON) | 12 µs / 11 µs |
| JSON query | 0.44 µs |
| `apply` with autosave, no fsync | 0.33 ms |
| `apply` with autosave, full fsync | 2.0 ms |
| bulk save 5 000 nodes, tmpfs, 1 vs 4 threads | 97 ms vs 57 ms |
| `load_timeline` / `open` with replay, 5 000 nodes | 116 ms / 136 ms |
| `open` with snapshot at the tip, 5 000 nodes | 124 ms |

Open time is dominated by reading and hashing node files (`Timeline::from_files` verifies every
hash single-threaded); snapshots remove the replay but not that. One file per node is the known
scaling limit (see the report).

## Tests (`cargo test -p sm-engine`)

| File | What it pins |
|---|---|
| `tests/timeline.rs` | fork/checkout/replay bit-identical; cold derived cache; JSON = typed node; plugin order independence; branch rules; off-thread prepare |
| `tests/persistence.rs` | layout, open round trip, incremental autosave, undo/redo/branch persistence, snapshots (seed, prune, stale), damaged cache, lock, read-only, project.toml, tampering and re-baseline, junk files, HEAD fallback |
| `tests/crash.rs` | exhaustive crash-point sweeps for apply, fork, rename/delete, undo/redo, attached-branch rename, shape change, project creation |
| `tests/git.rs` | real `git`: commit, clone (also `autocrlf=true`), merge of independent branches, same-branch conflict confined to one ref file, older checkout under a stale cache |
| `tests/policy.rs` | verbosity, suppression, stop-on-warning |
| `tests/determinism.rs` | bit-identical across runs and 1/N threads; scheduling-independent errors |
| `tests/property.rs` | random op/fork/checkout/undo/redo/branch sequences: disk = memory = twin, with mid-sequence restarts |
| unit tests | snapshot codec (every truncation and bit flip), atomic write at every crash point, lock, RFC 3339, policy, config |

The demo plugin set (a counter layer, ops and queries split over two plugins) lives in
`tests/common/mod.rs` and is used only by tests and the bench.
