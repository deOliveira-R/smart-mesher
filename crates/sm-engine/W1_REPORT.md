# W1 report: sm-engine

## Summary

`sm-engine` implements the session engine on top of the frozen `sm-op-api`: the project on-disk
format with crash-safe incremental autosave, open with snapshot seeds, persistence of undo/redo,
branches and HEAD, the diagnostics policy, and a JSON `Workspace` facade over registered plugins.
Design, format, the crash argument and performance numbers are in `DESIGN.md`.

Delivered:

- **Format**: `project.toml`, `timeline/ops/<hash>.toml`, `timeline/refs/heads/<name>`, gitignored
  `.cache/` (HEAD, reflog, redo stack, snapshots, lock). Node and ref text is exactly
  `Timeline::node_file` / `ref_file`.
- **Atomic writes** (`fsio`): temp file, fsync, rename, directory sync; ordered saves (nodes, refs,
  HEAD, deletions, cache); advisory write lock; stray temp files swept at open.
- **`fs::{save_timeline, load_timeline, read_timeline_files}`** (W0a decision 7), parallel file I/O
  with order-independent results and errors.
- **Incremental autosave** after every mutating call: one node file, one ref file, one reflog line.
- **Open** with the snapshot hook (`Registry::restore` + `Session::open_with` seeds), reflog and redo
  restoration, `Strict`/`Warn`/`Rebaseline` replay policies, tolerant handling of any damaged cache (snapshot seeds are validated against the node's recorded check, so a stale snapshot is dropped with a warning and cannot block `Warn`/`Rebaseline`).
- **Undo/redo persistence**, **branch create / delete / rename / reset / checkout / fork**, detached HEAD.
- **Diagnostics policy**: verbosity, suppression, stop-on-warning (nothing recorded), per-call override.
- **Facade** `Workspace`: plugins (layers, ops, queries), services, `apply` / `apply_json` / `query` /
  `query_at` / `checkout` / `fork` / `undo` / `redo` / `status` / `log` / `catalog_json`, off-thread
  `prepare_base` + `commit_prepared`, read-only mode, `FaultInjector` for crash tests.
- Demo plugin set (counter layer, 8 ops, 2 queries over two plugins) in `tests/common`, test-only.

## Test summary

`cargo test -p sm-engine`: **72 tests, all passing**; also with `RAYON_NUM_THREADS=1`.
`cargo build`, `cargo clippy -p sm-engine --all-targets --all-features -- -D warnings` and
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-engine --no-deps` are clean; library code has no
`unwrap`/`expect`/`panic`. `cargo bench -p sm-engine` runs (criterion, `benches/engine.rs`).

| Suite | Tests | Notes |
|---|---|---|
| unit | 13 | snapshot codec survives every truncation and every single-bit flip; atomic write at each crash point |
| `timeline` | 10 | fork at node 3 of 10 and back, every node by checkout and by cold replay from the root, derived cache disabled, JSON vs typed node identity |
| `seed_check` | 5 | seed validated against the node's recorded check (dropped, survivable under `Warn`/`Rebaseline`), prune keeps the fresh snapshot, reflog bad line keeps newer entries, op impl-version change invalidates a snapshot (older valid one used, else root; file kept) |
| `persistence` | 20 | layout, incremental I/O count, snapshots skip replay (counted), stale snapshot rejected, lock, tamper to `Diverged`, re-baseline rewrite |
| `crash` | 9 | exhaustive: killed at every write/delete point of apply, fork, rename/delete, undo/redo, attached-branch rename, shape change, project creation |
| `git` | 5 | real git: commit, clone, clone with `autocrlf=true`, merge of independent branches, conflict confined to one ref file, older checkout under stale cache |
| `policy` | 6 | |
| `determinism` | 3 | `sm_testkit::determinism::check` (repeat, 1 vs 4 rayon threads, concurrent), file-tree hash, scheduling-independent errors |
| `property` | 1 | random sequences, disk = memory = twin, mid-sequence restart; also run with 800 cases |

Findings from the tests that changed the code: the property test found that a HEAD naming a deleted
branch opened as an empty unborn state (fixed: fallback with a warning, unborn kept only after an
undo-to-root); the same test found the `Session::rename_branch` hazard below (guarded in the facade);
a flaky `Locked` came from a forked `git` child briefly holding the lock descriptor (the lock now
waits up to 100 ms before reporting contention). The crash sweeps were mutation-checked: swapping
refs before nodes, or HEAD after deletions, makes them fail.

Numbers (4-core sandbox, noisy disk): apply in memory 12 µs; with autosave 0.33 ms (2.0 ms with
fsync); open of 5 000 nodes 136 ms; bulk save of 5 000 nodes on tmpfs 97 ms on 1 thread vs 57 ms on 4.
On this sandbox's disk, timings varied 10x between runs, so the parallel-write speedup is only
demonstrated on tmpfs.

## Deviations from the contracts' intent

- The facade is called `Workspace` (not "Engine") to avoid clashing with `sm_op_api::Engine`.
- Open under `Warn`/`Rebaseline` cannot pass the policy to `Session::open_with` (it replays strictly),
  so the engine opens at the empty state, sets the policy and checks out HEAD. This adds one
  `checkout` entry to the reflog per such open (api_change_request 1).
- The facade rejects `create_branch` / `fork` / `rename_branch` onto the name of the unborn branch
  HEAD is attached to (api_change_request 2); the contract would accept it and leave an inconsistent
  session.
- ARCHITECTURE §4 lists `timeline/HEAD`; per W0a decision 1 HEAD is `.cache/HEAD`. The orchestrator
  still has to amend that section. Also new there: `.gitignore` written at create, `.cache/` contents.
- `project.toml` fields (`format`, `name`, `description`, `units`, `tolerances`, `metadata`, kept
  unknown keys) are my reading of ARCHITECTURE §4 ("format version, units, default tolerances,
  metadata"); they are not in any frozen contract and can be extended additively.
- Snapshot files use an engine-defined binary container (magic, node and state hash, JSON kinds,
  layer blobs, SHA-256 trailer). The contract only fixes `LayerType::snapshot/restore` bytes.
- `project.toml` is written last by `create` / `save_as`, so a crash during creation leaves a
  directory that is not a project (and `create` then refuses the non-empty directory).
- Dependencies: `serde`, `serde_json`, `toml` (project.toml; already in the tree via `sm-op-api`,
  decision 2), `rayon`, `thiserror`. Dev: `sm-testkit` (determinism harness), `proptest`, `criterion`
  0.7 without default features, `schemars` (derive `JsonSchema` on test ops; already in the tree),
  `sm-core` (ids in tests). No other crates; temp dirs are hand-rolled to avoid `tempfile`.
- `Cargo.lock` is committed (the new direct dependencies are locked in a separate commit).

## Known gaps

- **One file per node.** 10 000 nodes are 10 000 files; open reads and hashes all of them (136 ms
  per 5 000 here). A pack format would need a contract decision (the node-file layout is frozen).
- **Single writer.** One engine per project (advisory lock). A `Workspace` does not notice external
  changes (`git checkout` while open); there is no `reload()`.
- **File/directory ref shape change is not crash-atomic** (`a` to `a/b`): a crash can lose that branch
  name, never its nodes. Case-only renames are ordered correctly; the full gate was also run on macOS
  (case-insensitive APFS) in review and passed. Windows was not exercised (CI is Linux only).
- **No garbage collection** of abandoned nodes; only snapshots are pruned (deepest 4 kept).
- Reflog is capped only when read (newest 10 000 at open, file rewritten when it exceeds twice that);
  the in-memory reflog of a very long session is unbounded.
- Snapshots are written when the recorded node's depth is a multiple of `snapshot_every`, or on
  `snapshot_now()`; checkout, undo and redo do not write them. Layers without the snapshot hook make
  snapshotting a warning, not an error.
- Progress/cancellation covers `open_with`, `apply_json_with`, `checkout_with`, `fork_with`; saving has
  no progress reporting.
- No test under a real power cut: durability relies on fsync ordering, tested only by process-level
  crash injection.

## api_change_requests

All additive; none blocks W1.

1. **Crate** `sm-op-api` (`Session`). **What:** replay policy at open. **Why:** `Session::open_with`
   always replays strictly, so `Warn` / `Rebaseline` cannot apply to the replay of HEAD during open;
   the engine detours through an empty open plus `checkout`, which adds a spurious reflog entry.
   **Proposed:** `Session::open_with_policy(engine, timeline, head, seeds, reflog, policy: ReplayPolicy, ctl) -> Result<Session, TimelineError>`
   (`open_with` keeps its meaning, strict).
2. **Crate** `sm-op-api` (`Session`). **What:** guard against creating the branch HEAD is attached to
   while it is unborn. **Why:** undo to the root leaves `Head::Ref("b")` with no ref and an empty
   position; `create_branch_at("b", n)` or `rename_branch(x, "b")` then makes `b` exist while
   `position()` stays `None`, so the in-memory state disagrees with what a reopen produces. Found by
   the property test. **Proposed:** `create_branch_at` and `rename_branch` return
   `TimelineError::HeadOnBranch(name)` (or move the position and state) in that case. The engine
   guards it today.
3. **Crate** `sm-op-api` (`State`). **What:** a way to ask whether a state can be snapshotted.
   **Why:** `State::snapshot` reports "layer has no snapshot support" only as a `CodecError` string, so
   the engine cannot tell it from corruption. **Proposed:** `State::can_snapshot(&self) -> bool`, or a
   `CodecError` kind.
4. **Crate** `sm-op-api` (`Session`). **What:** `trim_reflog(&mut self, keep: usize)`. **Why:** the
   docs say the engine owns the reflog and may truncate it, but the session exposes no way to do so,
   so a long-lived session's in-memory reflog only grows. **Proposed:** drop all but the newest
   `keep` entries.
5. **Crate** docs (orchestrator-owned). **What:** amend ARCHITECTURE §4 for `.cache/HEAD`,
   `.gitignore`, `.cache/` contents, and the `project.toml` keys listed in `DESIGN.md`.

## Delivery

Branch `claude/w1-sm-engine-u0sels` (the branch this session was assigned) and
`claude/w1-sm-engine` (the name the task asked for) point at the same commit; the SHA is what to merge.

## Fix round 0

- Seeds are checked against `check.state` of the node before use; a mismatch warns and deletes the
  snapshot, so `Warn` and `Rebaseline` open (tests in `tests/seed_check.rs`). `snapshot_now` refuses
  (with a warning) when the live state differs from the node's recorded state.
- `snapshot_now` protects the snapshot it just wrote from pruning; the reflog reader skips an
  unparsable line instead of truncating; seed search lists `.cache/snapshots` once;
  `save_timeline` uses a set; `save_as` unbinds the project (and drops the lock) if any step,
  including the final `project.toml` write, fails.

## Polish round 0

- Snapshots record the `op:`/`layer:` implementation versions they depend on (container
  `SMSNAP02`); at open a snapshot whose versions differ from the registry is ignored (kept on
  disk, warning) and the search falls back to an older valid one or the root. DESIGN.md now states
  the real guarantee: hashes verified from the nearest valid snapshot (or root) on every open,
  snapshots trusted up to their node.
- `save_as` removes what it wrote on failure (deletes the directory, or empties it if it
  pre-existed) so a retry to the same path works; tested at every injected failure point.
- `read_reflog` docs match the skip-bad-line behaviour. DESIGN.md documents how `project.toml`
  tolerances map onto `sm_model_api::Tolerance` (relative stays at the default).
