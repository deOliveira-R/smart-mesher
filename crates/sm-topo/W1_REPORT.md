# W1 report: sm-topo

Status: **done** (definition of done passes; see "Test summary"). Branch: `claude/w1-sm-topo-ciy1fj`
(the harness-designated branch). The task text names `claude/w1-sm-topo`; the same commit is pushed
to both names, and the orchestrator merges by SHA.

## Summary

`TopoStore` is the production implementation of `sm-topo-api`'s `TopologyView` and `TopologyEdit`:
the same operators, ids, lineage, `OpInvalid` payloads and canonical hashes as the reference
`CellComplex`, on a structure-of-arrays store.

- **Storage.** Dense per-rank columns (`state`, `born`, `flags`, `carrier`, tolerance override),
  boundary, coboundary and aux rows in paged CSR arenas edited in place (`splice`), sparse data
  dense by slot (pcurves per face, shell kinds per volume, coincidence records per face, radial
  rings per edge). No map keyed on cell ids, no per-cell heap allocation (rows live in the arenas).
- **Edits and journal.** Row edits are journaled as deltas; rollback replays them. Long volume/body
  rows (>= 24 entries) are rewritten by position (binary search + `memmove`) instead of rebuilt,
  re-sorted and relinked.
- **End of operation.** Orientation rule, shell compaction, local checks, radial rebuild (rayon) and
  summary as in the reference, with incremental shell closure (edge-centric, from recorded dirty
  edges) and incremental row checks (entries inserted in the operation) so the cost does not grow
  with the size of the touched volume.
- **Hash.** `canonical_visit` / `canonical_hash` are specialised overrides emitting the identical
  word stream (asserted word by word in the differential tests).
- **Parallelism.** Radial-ring rebuild at `end_op` and `validate_par` use rayon with ordered,
  slot-addressed write-back; output is identical for 1 and N threads.
- **Verification.** The reference tests (all eight files of `sm-topo-api/tests`, hash goldens
  included) run unchanged against the store; the sm-testkit topology fixtures and checkers, the
  fixture catalogue, merge/unmerge, pole splits, overlays over both hosts, a seeded random
  driver (about 44 operator kinds, valid and invalid) and property tests run in lock step against
  the reference and compare every result, every cell row, lineage, validators and the canonical
  word stream. `paranoid` mode additionally asserts each incremental shortcut against the full
  reference algorithm on every operation.

### Performance (release, 4 cores; `benches/scaling.rs`, `tests/scale.rs`)

Workload: the top face of a brick refined by `n` splits (edge subdivision + chord, breadth first),
so one volume ends with `n + 6` faces.

BENCH_TABLE

At 10^5 faces the reference does not finish in reasonable time (it is quadratic: it was 5 s at
4 000 per-op splits and the cost per split grows with the volume row), so it is benchmarked only
up to 4 000; the store runs to 10^5.

## Test summary

`cargo test -p sm-topo` (debug) and `RAYON_NUM_THREADS=1 cargo test -p sm-topo`: **83 passed, 0
failed, 2 ignored** (timing and 10^5-size reports, run with `--release --ignored`).

| File | Tests | What |
|---|---|---|
| `src/rows.rs` | 2 | arena splice against a `Vec` model (4 000 random edits), block reuse |
| `ref_reference.rs` | 29 | copy of `sm-topo-api/tests/reference.rs`: brick, imprint+merge, webcut lineage, cylinder seam, sphere poles, baffle, fin, CSG, atomicity, tolerances, gaps, hash quantization, perf, semantics queries |
| `ref_shells.rs` / `ref_bridge_chain.rs` / `ref_gaps3.rs` / `ref_gaps4.rs` | 6 / 4 / 4 / 6 | shell compaction, bridge + chain cuts, `canonical_visit` + `record_merged` + `live_cover`, unmerge, degenerate splits, meridian cut with analytic areas |
| `ref_freeze.rs` | 5 | pcurve word and hash goldens (frozen values) |
| `ref_overlay.rs` / `ref_overlay_volume.rs` | 6 / 4 | decomposition overlay over `TopoStore` hosts |
| `differential.rs` | 8 | testkit fixtures rebuilt and compared with the testkit's own complexes + its checkers; fixture catalogue; merge/unmerge; sphere pole splits + `ChordAt`; overlay over `Arc<TopoStore>` and `Arc<CellComplex>`; polyline/NURBS pcurve words; determinism |
| `random.rs` | 2 | 8 seeds x 60 ops and 2 seeds x 150 ops, 1-3 operators per op, random aborts, failing `end_op`s (OpInvalid payloads compared), row thresholds forced to 6 so the positional/incremental paths run on small models |
| `scale.rs` | 2 (+2 ignored) | refinement (300 and 700 splits, one op and per op) equal to the reference, at forced and at production thresholds; timing reports |
| `props.rs` | 2 | proptest (fixed seed, 24 cases): random bricks vs closed forms and the testkit complex; random refinement + webcut vs reference |
| `determinism.rs` | 3 | same history twice, 1 vs 4 rayon workers, concurrent builds: canonical hash, every radial ring, both validators; `validate_par` == `validate` |

The random driver's tally (valid/invalid calls per operator) is printed with
`cargo test -p sm-topo --test random -- --nocapture`; it asserts that 12 key operators succeed at
least once (brick, split_edge, split_face, merge_vertices, unmerge_face, kill_cascade, reverse,
remove_face_use, make_inner_loop, imprint, sheet, tetra) and that the positional parent rewrite
ran. A typical run: about 20 % of `end_op`s fail validation and roll back (compared exactly), every
unmerge/merge/bridge/pole/embed/tolerance operator succeeds several times.

Commands (all clean): `cargo build`; `cargo test -p sm-topo`; `RAYON_NUM_THREADS=1 cargo test -p
sm-topo`; `cargo clippy -p sm-topo --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-topo --no-deps`; `cargo bench -p sm-topo --bench
scaling`.

## Deviations from the contracts' intent

1. **`canonical_visit` / `canonical_hash` are overridden.** `sm-topo-api/DESIGN.md` describes
   `canonical_hash` as the FNV fold of the provided `canonical_visit`. The store overrides both with
   the same traversal over its columns (`hash.rs`). Not a behaviour change: the word stream is
   asserted equal word by word to the reference's on every state the differential tests build
   (including polyline and NURBS pcurves with weights), and the goldens pass. Reason: the hash is
   O(cells) and the op layer hashes after every operation.
2. **Operator algorithms are the reference's.** The operator logic was ported step by step so ids,
   lineage order and error behaviour are identical; the new code is the storage, the primitives,
   the journal, the positional row edits and the incremental end-of-op checks. A bug that exists in
   the reference operator logic would exist here (differential tests cannot see it); none was found
   while porting (the reference suites and analytic checks pass).
3. **`end_op` re-examines fewer faces for the orientation rule** (only those whose volume uses
   actually changed, plus touched faces) than the reference (every face of every rewritten
   volume row). A face whose uses did not change cannot flip, and every committed state is
   normalized, so the results are identical; confirmed by the lineage comparison (flips are logged,
   in order) on the random driver.
4. **`end_op` local checks are incremental** (see DESIGN.md "End of operation"). They rely on the
   invariant that every committed state has closed shells and valid rows, which `end_op` itself
   enforces; `paranoid` mode asserts equality with the full algorithm, and `OpInvalid` issue lists
   are compared exactly in the random driver.
5. **Hidden API.** `TopoStore::set_paranoid`, `set_row_thresholds` (both `#[doc(hidden)]`, for
   tests), `counters()` (diagnostics), `validate_par()` (additive, same result as `validate`).
6. **`cells(dim)`** is a lazily rebuilt cache (`OnceLock`): exact at all times as the contract
   requires, but a read after more than 32 retirements in one operation costs O(slots).
7. **Dependencies added:** `rayon` (parallel radial rebuild, `validate_par`), `smallvec`.
   Dev: `proptest`, `criterion` (with `default-features = false`), `serde_json` (the copied
   reference tests), `sm-testkit`. `sm-math` (from the skeleton) is unused and was removed.
   `Cargo.lock` is committed with the additions (as the sm-topo-api branch did); the orchestrator
   owns and may regenerate it (Q3).

## Known gaps

- Long volume rows still pay an O(n) `memmove` per edit (16 bytes per face use, 8 for the aux row);
  this shows as mild super-linearity above 10^5 faces (17 us/split at 16k, ~45-55 us at 100k). A
  chunked row would remove it; not worth the complexity for a contract that returns `&[Incidence]`.
- `validate_par` (and `validate`) walks whole shells; there is no incremental full validation.
- The canonical hash is the contract's byte-serial FNV fold: ~65 ms at 10^5 faces, dominated by the
  fold itself; faster only if the hash definition changes.
- `LineageLog` is the contract's type with `BTreeMap` indexes; its cost per record is outside this
  crate.
- Row and arena offsets are `u32` (about 4 billion entries per rank table); beyond that edits fail
  with `IdExhausted`.
- `tests/ref_*.rs` are copies of the reference tests with `CellComplex` rebound to `TopoStore` and
  the `cfg(feature = "serde"/"schemars")` tests (which only test `sm-topo-api` derives) removed.
  They must be re-copied when the originals change (see api_change_request 3).
- Benchmarks cover refinement of one big volume (where the reference is quadratic) and an assembly of
  many small solids (where rows are short and the win is smaller); no real-model import benchmark
  exists yet.

## api_change_requests

All additive (open tier of the freeze policy).

1. **crate `sm-topo-api`** (`validate.rs`). *What:* make the local `end_op` checks public.
   *Why:* `check_cells` is `pub(crate)`, so every implementor must copy ~150 lines (ported verbatim
   into `sm-topo/src/check.rs`) and keep them in step by hand; any rule added to `validate` silently
   drifts. *Proposed:* `pub fn check_cells(view: &dyn TopologyView, ids: &[EntityId]) -> Vec<TopoIssue>`
   plus a variant that lets an implementor with its own shell-closure bookkeeping supply it:
   `pub fn check_cells_with(view: &dyn TopologyView, ids: &[EntityId], open_shell: &dyn Fn(EntityId, u16) -> Option<(EntityId, i64)>) -> Vec<TopoIssue>`
   (`open_shell(volume, ring)` = lowest edge with non-zero net use and that net use, if the shell is open).
2. **crate `sm-topo-api`** (`types.rs` / `complex`). *What:* export the canonical row forms.
   *Why:* every implementor must reproduce `normalize` (loop rotation, volume sort and
   `Left + Right -> Both` fusion) and `canonical_rotate` exactly, or boundary rows, and therefore
   hashes, diverge. *Proposed:* `pub fn normalize_row(dim: Dim, row: &mut Vec<Incidence>)` and
   `pub fn canonical_rotate(ring: &mut [Incidence])`.
3. **crate `sm-testkit`.** *What:* a generic conformance suite for `TopologyEdit` implementors.
   *Why:* I copied eight test files (4 000 lines) and rebound a type name; they will drift, and the
   next implementor would copy them again. *Proposed:* the reference scenarios exposed as
   `pub fn run_topology_conformance<E: TopologyEdit + Default + Send + Sync>(name: &str)` (and the
   twin comparison helpers `same_state` / a seeded random driver as `pub mod differential`).
4. **crate `sm-topo-api`** (`lineage.rs`). *What:* a cheaper index in `LineageLog`.
   *Why:* `by_subject` / `by_target` are `BTreeMap<EntityId, SmallVec<[usize; 2]>>`: O(log n)
   per record and a heap allocation per distinct entity, now the dominant per-record cost once rows
   are in-place. *Proposed:* index by dense `(kind, serial)` (`Vec<SmallVec<..>>` per kind) behind the
   same public methods; no signature change.
5. **crate `sm-topo-api`** (`view.rs`, doc only). *What:* state that implementors may override
   `canonical_visit` / `canonical_hash` provided the word stream is identical, and add a public
   `pub fn canonical_words(view: &dyn TopologyView) -> Vec<HashWord>` for conformance tests.
   *Why:* deviation 1 above; the word-by-word comparison currently needs a hand-written collector.
