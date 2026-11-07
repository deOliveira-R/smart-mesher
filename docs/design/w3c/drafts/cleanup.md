# W3c-CLEANUP: triage of the small backlog into three bounded tasks

> Planning draft (W3c), corrected by the planning critique. Changes against the original draft are marked **Critique delta**. `docs/design/w3c/plan.md` wins over this file.
>
> **Feeds plan tasks:** W3c-CLEAN-SERVER (T1 items 1, 2), W3c-CLEAN-CADIO (T1 item 5, micro task), W3c-CLEAN-RT (T2), W3c-CLEAN-DOCS (T3, inside W3c-INT). T1 item 3 moves to W3c-EXPORT-A (`drafts/export-opts.md`); T1 item 4 is dropped (EXPORT-A/B own it).

Original size: M overall (T1 S/M, T2 S, T3 S); no L work. Original owners: W3c-CLEAN-SERVER: sm-server (micro edits in sm-cli, sm-cadio); W3c-CLEAN-RT: sm-rt-capi (+ `tools/w3b_driver.c`, one test file in sm-rt); W3c-CLEAN-DOCS: orchestrator. Depends: none between the three tasks; all can run in wave 1 of W3c. T1 and the R11 FOAM-WARP session.export option both edit `crates/sm-server/src/host.rs`: the orchestrator should merge one first (FOAM-WARP is a separate draft). T3 is last, because it records the T1 and T2 results and the CI figures.

**Critique delta (ordering).** CLEAN-SERVER is sub-wave 2, after EXPORT-A merges (both edit sm-server). CLEAN-RT is sub-wave 1. CLEAN-CADIO is sub-wave 2 (micro, any time). CLEAN-DOCS is part of W3c-INT.

Base: main at d7ad022. I read the code and docs only; nothing was built or run.

## Goal
Close the small W3b leftovers that are cheap and low risk, and defer or drop the rest with a stated trigger. No frozen contract changes. No existing state hash changes: W2 milestone/fork, w3a A/B and W3b h_model/h_auto/h_mesh stay as they are.

Four backlog items are already done or void, and the backlog and report should say so:
- `with_unchecked_merges` is already behind a `test-hooks` feature (commit 2d1e1ea, POLY2). See `crates/sm-poly/src/lib.rs:26-27`, `src/agglom.rs:213-215` and `Cargo.toml:13-16,32`.
- The dev-only cycle sm-cadio to sm-server is already recorded in `crates/sm-cadio/DESIGN.md:301-308`.
- CLAUDE.md already allows test-only `unsafe` for counting allocators (the "Exception: test code" bullet).
- The playwright-core version (1.49.1) is already in the report (w3b-report.md line 38) and in `crates/sm-view/web/package.json:17`. Only the Chrome version is missing.

## Findings and disposition, item by item

Legend: K = keep in W3c, D = defer to W4, X = drop. "Frozen" says whether a frozen contract is touched. "Hash" says whether a state hash moves.

### W3b-INT acceptance items
| Item | Disposition | Owner | Frozen / Hash | Acceptance line |
|---|---|---|---|---|
| w3b-report numbering, CI result, viewer versions | K, T3 | orchestrator | no / no | The deviations are numbered D1..Dn in one scheme. Today the list items are 1, 3, 5 (line 60 onwards) while the labels are D1, D2, and so on. The CI rc and duration (59-70 min) are in the CI section. The Chrome version is recorded (playwright-core 1.49.1 is already there). **Critique delta:** the CI section records GitHub Actions as **red** with its causes (`flat_golden.rs:55` bundle hash on Linux; python job `test_errors.py [<lambda>5]`), not "rc=0"; the rc=0 figure is only the local macOS `tools/ci.sh`. |
| `tools/w3b_driver.c` chord tolerance | K, T2 | tools/w3b_driver.c, owned by sm-rt-capi | no / no | The driver has `1e-12*(1+chord)*max(1,0.05/mc)` at line 187, and the header comment at line 8 repeats it. R7 says `1e-12*max(1,0.05/min\|d_k\|)`. Keep the factor, since it only widens the tolerance, and rewrite the comment to say so with a one-line reference to R7. Worst/tol is 5.3e-15 against at least 1e-12, so the margin is large either way. Alternatively drop the factor if the milestone still passes; I recommend keeping it and documenting it. **Critique delta:** `tools/` is outside every crate; the orchestrator grants CLEAN-RT explicit permission for this one file. |
| `ridx()` complement guard (`w3b_driver.c:79-83`) | K, T2 | same | no / no | `ridx` returns `m->comp`, which is -1 (SMRT_NONE) when the file has no complement, so a segment with region SMRT_COMPLEMENT would index `sig[-1]`. Add `MUST(m->comp >= 0)` in that branch. Check this by a driver run against a stubs or no-complement `.smrt` that fails cleanly with a message. If that fixture is awkward, a code-read check plus the milestone still passing is enough. |
| `smrt_regions` set-name ambiguity (`crates/sm-server/src/mesh_export.rs:183-205`, `crates/sm-rt/src/build.rs:112-118`) | K, T1 | sm-server | no / no | `ByCellSets` resolves by `set_by_name`, which returns the first match (`sm-mesh-api/src/read.rs:429`). `sm_fem::lower` rejects block names that collide with mesh sets (`lower.rs:111`), and blocks are emitted before sidesets (`lower.rs:214` and `:288`). So today the block wins, but only by ordering. Add a test in `crates/sm-server/tests/export_wire.rs` where a block and a deck sideset share a name. Expected result: the `.smrt` regions are the block cells (cell count per region equals the block size), or the export falls back with the `smrt-region-names` note. Add a comment in `smrt_regions` stating the invariant. If the test shows a mis-pick, make `smrt_regions` fall back with a note on duplicate names across `lowered.sets`. **Critique delta:** moved to W3c-EXPORT-A (it rewrites `smrt_regions`). |
| PARITY Transforms row wording (`docs/PARITY.md:31`) | K, T3 | orchestrator | no / no | The M1 text says "volumes and the merged interfaces checked", but M1 checks only volume counts and total volume. Reword to "volume counts and total volume checked", plus a pointer to the M4/M6 rows if they cover interfaces. |
| R10 follow-up: the fallback note must reach CLI and MCP users | K, T1 | sm-server, with a micro edit in sm-cli | no / no | The note `smrt-region-names: ...` goes into `ExportResult.notes` (`host.rs:notes_of`). The human CLI render `crates/sm-cli/src/render.rs:134-152` prints no notes, only `--json` shows them. Print each note as `warning: <code>: <detail>` on stderr in `render::export`. The MCP `session_export` tool should return the raw result with the notes, which I could not confirm from `crates/sm-mcp/src/tools.rs` and `lib.rs`. A CLI test exports a block-less model to `.smrt` and asserts the stderr line and the JSON `notes[0].code == "smrt-region-names"`. A similar MCP test runs through the in-process server. Stdout is unchanged, so the CLI/MCP goldens stay unless one already covers a note. **Critique delta:** dropped here; duplicates W3c-EXPORT-B (CLI, MCP) and EXPORT-A (warning severity), which use a different, agreed output format (`warning:` only for warnings, `note:` for the rest). |

### W3b report open items
| Item | Disposition | Owner | Frozen / Hash | Acceptance line |
|---|---|---|---|---|
| `SMRT_NODE_UNKNOWN` added without an ABI bump (`smrt.h:48,104`, `types.rs:11,315`, `convert.rs:441`; added in 8be661f, version still 10100) | K, T2 | sm-rt-capi | no (it is `sm-rt-capi`, not `*-api`) / no | Bump to 10200, or 10101 (see Q1), in `include/smrt.h` and `src/types.rs`. `tests/abi.rs:376` and `tests/header.rs:154-155,203` compare the two, so they catch drift. DESIGN.md line 108 gets one line on the rule: a new enum value or constant bumps the minor version. Add an assertion that the header macro equals `smrt_version()`. The C and Python drivers never read the number, so they are unaffected. **Critique delta:** 10200, by implementer decision; not a user question. |
| Builder complement name has no length prefix, `LeafInput::content_hash` has one (`sm-rt-api/src/input.rs:265`, `build.rs:140-146`) | X | none | n/a | R2 kept it deliberately so that complement leaf keys do not move. The doc comment at `input.rs:265` already says so. Re-open only if keys are ever re-based. |
| sm-rt delta.rs acc04 should bracket `next_segment` (`crates/sm-rt/tests/delta.rs:353-357`, also `:388-390`) | K, T2 | sm-rt (tests only) | no / no | Today only `probe` is bracketed (`bvh_descents` before and after). Bracket `next_segment` too and assert that the descents of a delta segment starting at an owning node equal 0 (or equal the pinned count). Test-only, so the library hashes are unchanged. |
| sm-server 503 refusal on the accept thread (`crates/sm-server/src/transport.rs:454-462`) | K, T1 | sm-server | no / no | The refusal writes with a 1 s write timeout, then waits up to 200 ms for the read, on the accept thread, so each excess connection stalls accepts for up to 1.2 s. Move the refusal to a short-lived thread spawned in the existing `conns` scope. Count refusals in an atomic and cap them at, say, 8 in flight, closing silently above that, so a flood cannot spawn unbounded threads. Test: hold `MAX_LIVE_CONNECTIONS` idle connections, send one excess connection that never reads, then connect a valid client and require that it is accepted well under 1.2 s. Use a work counter or a generous bound, not a tight wall-clock gate (CLAUDE.md prefers counters). The 503 body and the existing refusal tests stay unchanged. |
| `OPENING` thread-local test (`crates/sm-server/src/standard.rs:119-148`) | K, T1 | sm-server | no / no | The existing test (`export_wire.rs:369-406`) shows installs run on the opening thread. It does not show that a second thread does not bind another opener's store. Add a test: two `Server::open` calls on two threads with different project dirs, each followed by `stage_input` and a check that the bytes land under that thread's own project dir and not the other's. Also assert that `OPENING` is `None` again after a failed `open`. |
| DESIGN.md dev-only cycle sm-cadio to sm-server | X (already done) | none | n/a | Already in `crates/sm-cadio/DESIGN.md:301-308`. Just delete the line from the backlog. |
| sm-mesher: volume `mesh.size` / `mesh.intervals` do not reach a face-only `mesh.run` | X, pending Q2 | none | n/a | This is documented (`DESIGN.md:129-134`) and already warned (`mesh.volume_request_ignored`, `mesh_ops.rs:322-331`). Keep as the current behaviour. See Q2, because "Cubit parity" is asserted but not verified here. |
| sm-mesher: re-plan after a failed face run is not budget-checked (`mesh_ops.rs:273-290,340-348`, `DESIGN.md:135-137`) | D (W4) | sm-mesher, with an sm-scheme-api ACR | would need an ACR to sm-scheme-api / no | The only clean fix is a face branch in `Driver::estimate_cells`, which is frozen. The replan count is already capped (`replan_budget`), so the exposure is bounded. A post-hoc check on `out.plan` in sm-mesher would only fail after the work is done. Trigger: a face run observed to blow up on a replan. Pin the gap with a test only if it is cheap. |
| sm-boolean `merge.rs:390` O(F^2) face-box scan (`face_pairs`) | D (W4) | sm-boolean | no / no (the output is sorted by `(a,b)` at line 408, so a faster scan must give identical pairs) | It is already a sweep on x with early break (`merge.rs:387-405`); it is quadratic only when many faces share an x range. There is no measurement at 10^5 faces. First task in W4: a benchmark at 10^5 faces. Acceptance then: identical `face_pairs` output to the current scan on the existing merge tests. |
| sm-poly `with_unchecked_merges` behind `test-hooks` | X (already done) | none | n/a | See Goal. Delete from the backlog. |
| CAD-ATTR: an explicit `attributes: false` serializes into the op record | K, T1 | sm-cadio (one attribute) | no (state hash) / the recorded op JSON changes for explicit `false` only | `crates/sm-cadio/src/op.rs:97-98` serializes `Some(false)`, while `DESIGN.md:277` says "absent or false: record JSON exactly as before". The state hash is already equal (`tests/attributes.rs:95-110`). Fix in code: `skip_serializing_if` that skips `None` and `Some(false)`. Add a test that `serde_json::to_value` of `Some(false)` has no `attributes` key. The only visible change is the record of scripts that wrote `attributes: false`; none are committed (grep of crates and tools). If the user prefers no code change, fix the DESIGN wording instead (Q3). **Critique delta:** becomes W3c-CLEAN-CADIO. First check whether node or record hashes cover the op payload; if they do, old project files that recorded `Some(false)` would fail the strict replay check after the change. Default: fix the DESIGN.md wording (no code change) unless the check shows the code change safe. Not a user question. |
| CLAUDE.md test-only unsafe | X (already done) | none | n/a | The CLAUDE.md "Exception: test code" bullet exists. |
| Process note about Sonnet co-author lines | X | none | n/a | Accurate as kept. |

### Older W3b-0 backlog items
| Item | Disposition | Owner | Frozen / Hash | Acceptance line |
|---|---|---|---|---|
| `retire_mesh_bulk` into `sm_layer_api::retire_mesh` | D (W4) | sm-layer-api (orchestrator) | frozen crate / no (state hash and `Delta::removed` already proven equal by `sm-op-api/tests/w3b_ops.rs:85`) | Only `mesh.delete` (sm-mesher, `mesh_ops.rs:439`) calls the bulk path. `retire_mesh` has test-only callers (`sm-layer-api/tests/sketches.rs`, `sm-fem/tests/common/mod.rs:244`). Moving it gains nothing in production. Trigger: a second production caller. Acceptance then: `retire_mesh` uses `retire_range`, and the existing sketches tests and the hash equality test pass. |
| `mesh.delete` parallel sum (about 4.5 s at 10^6 cells) | D (W4) | sm-op-api | frozen / must be bit-identical (an order-independent combine) | Needs rayon in sm-op-api and a proof that the combine is commutative. Performance only. Trigger: a requirement on 10^6-cell delete time. |
| `RegionInfo.tags` not carried in cache images, hashed without a presence marker | D (W4) | sm-rt / sm-rt-api | frozen / would move leaf keys | No producer sets tags yet. Trigger: the first producer, for example DAGMC property import. Fix together with a cache-image version bump. |
| P1 limits tests (`sm-mesher/DESIGN.md:79-87`) | D (W4) | sm-mesher | no / no | Both limits are latent: no production path deserializes `IntentLayer`, and webcut after `mesh.size` is not a milestone path. Trigger: any code that loads `IntentLayer` through serde (the original plan said "before SUBMAP", which is already merged without it). Acceptance then: a test with a serde round-trip intent after a webcut that replaces edges, checking plan equality with the live session. |
| DAGMC reference h5m volume 177 note | X | none | n/a | Informational, already in the backlog. Keep it as a one-line note in the EXPORT docs. |

### The `$TMPDIR/smrt-capi-*` leak (do not delete)
The leak helper is `crates/sm-rt-capi/tests/cutil/mod.rs:41-45`. `ScratchDir::drop` removes the directory unless the thread is panicking (lines 33-39). The directory name is `smrt-capi-<pid>-<name>`, so names are never reused and dirs from failed runs stay forever. Counts on this machine: 1309 dirs. They are mostly from 1 to 5 Nov, with 640 on 5 Nov alone.

Do the current tests still leak? Yes, in a small way. The newest run (7 Nov 03:35) left 5 empty directories: `-98638-raii`, `-98638-dag`, `-98638-chords`, `-98796-fortran`, `-98796-layout`. Empty means the contents were removed but the directory itself was not, or the directory was never populated. A normal `remove_dir_all` would remove both. I could not determine the cause by reading (a test killed mid-run or an aborted test binary are candidates; scratch uses are in `tests/handover.rs`, `tests/header.rs` and `cutil/mod.rs:103`). T2 should find it, with a before/after count of dirs for one `cargo test -p sm-rt-capi` run. Acceptance for T2 is below.

## Tasks

### Task T1: W3c-CLEAN-SERVER
- **Owner:** sm-server (`crates/sm-server/`). Micro edits: `crates/sm-cli/src/render.rs` (print export notes) and `crates/sm-cadio/src/op.rs` (the `attributes` serialization rule). The micro edits follow the EXPORT-WIRE precedent of one-line edits in sm-cli. I recommend one worker for all three, sequentially. If the orchestrator wants strictly one crate per task, split the sm-cadio line out as a 3-line task of its own.
  - **Critique delta:** CLEAN-SERVER owns `crates/sm-server/` only (no sm-cli edit: item 4 is dropped). The sm-cadio item is its own micro task, W3c-CLEAN-CADIO.
- **Scope:**
  1. 503 refusal off the accept thread, with an in-flight cap (`transport.rs:454-462`).
  2. Two-thread `OPENING` isolation test (`standard.rs:119-148`).
  3. `smrt_regions` duplicate-name test and invariant comment. **Critique delta:** moved to W3c-EXPORT-A.
  4. Export notes printed by the human CLI (R10 surfacing), plus an MCP check. **Critique delta:** dropped (EXPORT-A/B).
  5. `attributes: false` record normalisation in sm-cadio. **Critique delta:** W3c-CLEAN-CADIO, DESIGN.md wording fix by default (see the table).
- **Contract changes / ACRs:** none. `ExportNote`, `ExportResult` and the protocol are not touched.
- **Acceptance:**
  1. A client that never reads, held as the 17th (excess) connection, no longer delays the accept of a later valid connection. The test asserts accept of the valid client in under 1 s (the old stall was up to 1.2 s per excess connection) and uses a refusal counter, not tight timing. The 503 body is byte-identical to today.
  2. A flood of N > cap excess connections never creates more than cap refusal threads at once (an atomic high-water mark asserted in a test hook or via a counter).
  3. Two threads opening two projects concurrently each stage input into their own project directory (check the file location), and `OPENING` is `None` on each thread after `open` returns, and after an `open` that returns `Err`.
  4. A block and a sideset with the same name: the `.smrt` exports with regions equal to the block cells, with a per-region cell count equal to the block size; or it exports with the `smrt-region-names` fallback note. Either way no mis-pick, and the test pins which. **Critique delta:** moved to EXPORT-A (`drafts/export-opts.md` acceptance 4(e)).
  5. `smesh export x.smrt` on a model with no blocks prints a stderr line containing `smrt-region-names`, and `--json` has `notes[0].code == "smrt-region-names"`. A model with all blocks prints no note. The MCP `session_export` result contains the same note (add a test; if it is missing, one-line fix in sm-mcp as a micro edit). **Critique delta:** dropped (EXPORT-A acceptance 4, EXPORT-B acceptance 5-7 cover it).
  6. `serde_json::to_value(ImportStep{attributes: Some(false), ..})` has no `attributes` key; `Some(true)` still serializes. `crates/sm-cadio/tests/attributes.rs` stays green. **Critique delta:** CLEAN-CADIO; applies only if the code change is shown safe (no node or record hash covers the payload, or a replay test of a recorded `Some(false)` passes); otherwise the acceptance is "`DESIGN.md:277` matches the code" and `tests/attributes.rs` stays green.
  7. Determinism: the existing server and cadio determinism tests pass, and `cargo build`, `cargo test -p sm-server -p sm-cli -p sm-cadio -p sm-mcp`, and `cargo clippy -p <each> -- -D warnings` are clean. **Critique delta:** CLEAN-SERVER: `-p sm-server`; CLEAN-CADIO: `-p sm-cadio`.
- **Gates / dependencies:** none. Sequence with FOAM-WARP (R11) on `host.rs` if both touch it. **Critique delta:** after EXPORT-A merges.
- **Hash impact:** none on any state hash or milestone hash. `.smrt` and export bytes are unchanged. The only change in recorded data is the op JSON for imports with an explicit `attributes: false`.
- **Size:** S to M. **Critique delta:** CLEAN-SERVER S; CLEAN-CADIO XS.
- **Risks:** a refusal thread cap that is too low turns a 503 into a silent close for clients that expect a 503 (keep the cap generous). Timing tests are flaky unless they use counters; follow CLAUDE.md. The CLI change adds stderr output, which could break a CLI golden that exports with notes (run the CLI golden test first and bless only with a ruling).

### Task T2: W3c-CLEAN-RT
- **Owner:** sm-rt-capi (`crates/sm-rt-capi/` and `tools/w3b_driver.c`). One test file in sm-rt: `crates/sm-rt/tests/delta.rs`. Both are in the same family, one worker. **Critique delta:** `tools/w3b_driver.c` by explicit orchestrator permission (workers do not edit `tools/` otherwise).
- **Scope:**
  1. ABI version bump with a recorded rule.
  2. `w3b_driver.c`: the tolerance comment and the `ridx` guard.
  3. acc04 bracketing of `next_segment` (test only).
  4. Find and fix the `smrt-capi-*` leak in the tests.
- **Contract changes / ACRs:** none. `smrt.h` is in sm-rt-capi's own tree, not a frozen `*-api` crate. No `sm-rt-api` change.
- **Acceptance:**
  1. `SMRT_ABI_VERSION` is bumped (value per Q1; **Critique delta:** 10200) in `include/smrt.h` and `src/types.rs`; `tests/abi.rs` and `tests/header.rs` pass; DESIGN.md states the rule (a new constant or enum value bumps the minor version).
  2. `tools/w3b_driver.c`: `ridx` fails with a message when a complement segment arrives and `m->comp < 0`. The comment at line 8 names the extra `(1 + chord)` factor and R7. The milestone C row (M6) passes unchanged with identical output (the reported 10000/10000 chord lines and the 4 sigma flux check).
  3. In `crates/sm-rt/tests/delta.rs`, `acc04_hinted_location_equals_locate` brackets `next_segment` with the counters as well, and asserts the descents of that call are 0 for a start point inside an owning node (or the pinned count if the run shows otherwise, reported in the commit message). No sm-rt library code changes, so sm-rt hashes are untouched.
  4. A single `cargo test -p sm-rt-capi` run leaves zero new `$TMPDIR/smrt-capi-*` directories (count before and after, in the commit message). The fix is either a per-process root removed at the end, or the specific cause found (see the leak section). Existing dirs are not deleted by the task.
  5. `cargo build`, `cargo test -p sm-rt-capi -p sm-rt`, the stubs configuration in `tools/ci.sh` (own target dir `target/capi-stubs`), and clippy with `-D warnings` pass.
- **Gates / dependencies:** none. **Critique delta:** `crates/sm-rt/tests/flat_golden.rs` is not CLEAN-RT's: it is edited by the orchestrator in sub-wave 0 per the user's ruling (plan C11), so CLEAN-RT branches after that commit.
- **Hash impact:** none. `.smrt` bytes unchanged, no state hash moves. `smrt_version()` returns a new number, which only the ABI tests read.
- **Size:** S.
- **Risks:** the leak cause may not be reproducible by reading alone (empty directories point to an aborted or killed process); budget a quick experiment. A bumped ABI number might be compared in sm-py or an out-of-tree consumer: I found no such use (grep covers `crates`, `tools`).

### Task T3: W3c-CLEAN-DOCS
- **Owner:** orchestrator (docs are outside every crate; CLAUDE.md forbids workers from editing them).
- **Scope:**
  1. `docs/design/w3b/w3b-report.md`: number the deviations D1..Dn consistently (the list items are 1, 3, 5 with D1, D2 labels), add the CI result (rc=0, 59-70 min) and the Chrome version used by the viewer smoke (the playwright-core 1.49.1 line is already there).
     - **Critique delta:** record GitHub Actions CI as red, with its causes (`flat_golden` bundle hash on Linux; python `test_errors.py [<lambda>5]`) and the runs (the last 4 on main, including 42412194333). "rc=0, 59-70 min" is labelled as the local macOS `tools/ci.sh` only. Number every deviation (the list mixes D1, D2 labels with unlabelled items).
  2. `docs/PARITY.md:31` Transforms row wording.
  3. `docs/design/w3/backlog.md`: rewrite the "From W3b-INT acceptance" and W3b report open-item sections to the dispositions above. Delete the four already-done items. Move the deferred items under a "W4" heading with their triggers.
  4. Record the tmp-dir leak result after T2.
  5. **Critique delta (added):** correct the W3b report repro recipe for the round pin (`model.subtract` without `keep`, then imprint and merge; from `drafts/cyl-pin.md` "Doc fix"); amend the W3b plan.md B6 wording if the user accepts the wedge outcome (plan C2); record the W3c deferrals (H1 analytic, P2 tet twin, `--model` periodic faces, XDG tet walk) in the wave summary.
- **Contract changes / ACRs:** none.
- **Acceptance:**
  1. Every item in the task list above appears exactly once in the backlog as done, kept in W3c with its task id, deferred to W4 with a trigger, or dropped with a reason.
  2. PARITY row M1 text matches what the milestone test asserts (volume counts and total volume); the row names no check the test does not make.
  3. The report names the Chrome version and the CI duration, and the deviation labels run D1.. in order with no gaps. **Critique delta:** and states the GitHub Actions result as red with its causes.
  4. `git diff` touches only `docs/`.
- **Gates / dependencies:** after T1 and T2 so that the backlog states their outcomes. The CI figures are already known.
- **Hash impact:** none.
- **Size:** S.
- **Risks:** none beyond staleness; keep it to one commit.

## Contract changes / ACRs (summary)
None in W3c-CLEANUP. The two items that need ACRs (`retire_mesh` into sm-layer-api, and face-aware `Driver::estimate_cells` in sm-scheme-api) are deferred to W4 with triggers, so no frozen crate is opened for cleanup. `mesh.delete` parallel sum would be an sm-op-api change and is also deferred.

## Hash impact (summary)
No state hash of any existing milestone moves. No change to pave, tri, sweep or `.smrt` bytes. The one observable data change is the recorded JSON of `model.import_step` with an explicit `attributes: false` (T1). **Critique delta:** only if CLEAN-CADIO shows the code change safe; the default is a doc-only fix.

## Size
M overall; no single task is larger than M.

## Risks (overall)
- The R10 warning surfacing (T1 item 4) is a visible CLI behaviour change on stderr. Check CLI goldens first. **Critique delta:** now EXPORT-B's risk.
- The tmp-dir leak cause is unconfirmed (5 empty dirs from the latest run, 1309 old ones).
- Deferred items are only safe while their triggers stay unmet: `RegionInfo.tags` (no producer), P1 limits (no serde-loaded intent), face-run replan budget (needs a runaway case), merge `face_pairs` (no 10^5-face measurement). Each trigger is written into the backlog by T3.

## Questions
- Q1 ABI version after adding SMRT_NODE_UNKNOWN: 10200 or 10101? **Critique delta:** implementer decision, 10200; dropped from the user list.
- Q2 Volume mesh.size and mesh.intervals do not reach a face-only mesh.run. The backlog calls this Cubit parity. Is that the behaviour you want, or should a face run inherit its bounding volume's size request? (rec: Keep the current behaviour (drop the item). It is documented and already raises the mesh.volume_request_ignored warning. If you know Cubit's mesh surface applies the parent volume's size, say so and I would schedule it in W4 with the planner change and a hash A/B, because it moves face-run plans.) → plan C14.
- Q3 For an explicit attributes:false, normalise in code or fix the DESIGN.md wording? **Critique delta:** orchestrator/implementer call (CLEAN-CADIO checks hash coverage of the op payload; DESIGN.md wording fix unless shown safe); dropped from the user list.
