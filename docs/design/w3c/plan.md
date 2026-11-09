# W3c plan: round pin cells through `mesh.auto`, export options and warnings, CSG importers, benchmark pinning, Linux hashes, cleanup

Base: `main` at `d7ad022` (W3b-INT merged in cd0ca67, follow-ups 2dc9a97, user rulings R9-R11). Rulings: `docs/design/w3b/rulings.md` (Q1-Q14, R1-R11 and the O-answers stay binding; O7 puts the MCNP/OpenMC importers in W3c). Inputs, all committed next to this file: the per-task planning drafts `docs/design/w3c/drafts/<task>.md` (`cyl-pin.md`, `auto-b6.md`, `csg-import.md`, `export-opts.md`, `bench-arm64.md`, `linux-hash.md`, `cleanup.md`). The drafts were corrected after a planning critique; every correction is marked **Critique delta** in the draft itself, so a worker never needs the critique. The W3b plan (`docs/design/w3b/plan.md`) is the structural model.

**Rulings:** `docs/design/w3c/rulings.md` (C1-C14, 2026-11-07) override this plan where they differ. In short: MCNP is not a target (W3c-CSG-MCNP dropped, no `mcnp` module); OpenMC is validated against OpenMC itself (conda env `openmc`); W3c-STRATEGIES dropped; region names per C9; a wave merges only with GitHub CI green on its PR.

**Citation convention.** Paths are relative to `docs/design/w3c/`. "`drafts/<task>.md` acceptance N" is item N of that file's numbered **Acceptance** list; "Scope N" / "Scope X" is item N or lettered part X of its **Scope** list; "finding N" / "FN" is its **Findings** item. A cited item includes the **Critique delta** written next to it in the draft. Where this plan changes a cited item further, the change is written next to the citation ("plan delta"). `plan.md` wins over the drafts, and `rulings.md` wins over both.

**Facts the plan relies on (checked against `d7ad022` by the critique, not re-checked here):** `scratch_plan` (`crates/sm-mesher/src/auto.rs:188`) and `mesh.run` (`mesh_ops.rs:337`) call `driver.plan(..)?` once; the sweep linking check is only `coedge_count < 4` (`discover.rs:~319`) and tet refuses shells with `uses.len() < 4` (`scheme.rs:174`); `blameable_imposers` sorts by target so `blame()` takes the lowest target id (`planner.rs:1550-1573`, `error.rs:350`); `standard_schemes` sets the auto lists to `[map]` (`standard.rs:37-48`); the MCP `session_export` schema lists 4 formats and no options (`tools.rs:146,213`) and `render::export` ignores notes (`render.rs:133`); the XDG pin `83efb306e152` (Dockerfile and `docker.rs:19`) does not exist upstream and `image_digest` is never assigned; `sm-rt-bench` does not depend on sm-mesher, sm-server or any sm-scheme-*; `pin_assembly` is `sm_testkit::topo::assembly::pin_assembly`. **GitHub CI is red on main** (the last 4 runs, including 42412194333): the gate fails at `crates/sm-rt/tests/flat_golden.rs:55` (bundle hash on Linux) and masks every crate after sm-rt; the python job fails at `test_errors.py::test_python_side_validation_raises_before_any_call[<lambda>5] DID NOT RAISE InvalidParamsError`.

Milestone (§5): a round pin cell (cylinder in a 1.26 × 1.26 × 1.0 brick) and its 3×3 lattice are built with `model.copy`, meshed with `mesh.auto {strategy: "default"}` plus `mesh.run` (through pin all-hex; enclosed pin all-tet if C1 is yes), exported to `.smrt` with block-named regions and no warning, recorded and replayed; CLI, MCP, in-process and Python give the same state hashes; the milestone identity hashes are compared between macOS arm64 and Linux arm64 (container) and pinned for Linux x86_64 CI; the DAGMC benchmark images build from pinned inputs and run a smoke row with a recorded image identity.

---

## 1. Ownership

| Task | Crate / dir | Sub-wave | Size | Owns |
|---|---|---|---|---|
| W3c-0 | orchestrator | 0 | S | `*-api`, `sm-protocol`, `sm-testkit`, root `Cargo.toml`/`Cargo.lock`, `tools/`, CI, docs; `crates/sm-rt/tests/flat_golden.rs` (C11 only) |
| W3c-PY-CI | `sm-py` | 0 | XS | `crates/sm-py/` (micro: the red CI test) |
| W3c-MESHER-REPLAN | `sm-mesher` (+ one test row) | 1 | S | `crates/sm-mesher/`; the `b6` row of `crates/sm-server/tests/auto_select.rs` |
| W3c-CYL-SCHEMES | `sm-scheme-sweep`, `sm-scheme-tet` | 1 | S | `crates/sm-scheme-sweep/`, `crates/sm-scheme-tet/` (explicit two-crate ownership, one worker) |
| W3c-EXPORT-A | `sm-fem`, `sm-server` | 1 | S | `crates/sm-fem/`, `crates/sm-server/` |
| W3c-EXPORT-B | `sm-cli`, `sm-mcp` | 1 | S | `crates/sm-cli/` (src and existing tests), `crates/sm-mcp/` |
| W3c-EXPORT-C | `sm-py` | 1 | S | `crates/sm-py/` |
| W3c-CLEAN-RT | `sm-rt-capi` (+ one test file, one tool) | 1 | S | `crates/sm-rt-capi/`, `crates/sm-rt/tests/delta.rs`, `tools/w3b_driver.c` (explicit permission) |
| W3c-BENCH-PIN | `sm-rt-bench` | 1 | S/M | `crates/sm-rt-bench/` (incl. `docker/`, `drivers/`) |
| W3c-CSG-CORE | `sm-csgio` (new) | 1 | M | `crates/sm-csgio/` (core, pre-declared format modules) |
| W3c-LINUX-HASH | orchestrator | 1 | S | `tools/platform/`, `docs/design/w3c/platform-hashes.md` (read-only against crates) |
| W3c-CYL-PIN | `sm-server` tests | 2 | S | rows `b5` and the new pin/lattice rows of `crates/sm-server/tests/auto_select.rs` |
| W3c-CLEAN-SERVER | `sm-server` | 2 | S | `crates/sm-server/` |
| W3c-CLEAN-CADIO | `sm-cadio` | 2 | XS | `crates/sm-cadio/` (micro) |
| W3c-CSG-OPENMC | `sm-csgio` | 2 | M | `crates/sm-csgio/src/openmc/`, its tests |
| W3c-CSG-MCNP (stretch) | `sm-csgio` | 3 | M | `crates/sm-csgio/src/mcnp/`, its tests |
| W3c-BENCH-WORKLOADS (stretch) | `sm-rt-bench` | 2 | M | `crates/sm-rt-bench/` |
| W3c-BENCH-XDG (stretch / W4) | `sm-rt-bench` | 3 | M | `crates/sm-rt-bench/` |
| W3c-STRATEGIES (only if C5 = yes) | `sm-mesher` (+ test rows) | 3 | S | `crates/sm-mesher/`; b3/b4/strategy rows of `auto_select.rs` |
| W3c-INT | integration agent | INT | M | see §5 (`tools/w3c_pin.smesh`, `crates/sm-cli/tests/milestone_w3c_pin.rs`, `crates/sm-cli/tests/platform_pins.rs`, `tools/platform/expected.txt`, docs) |

Rules for every task (CLAUDE.md applies): own only the listed paths; frozen contracts change only through W3c-0 ACRs, else `status: blocked_on_api`; workers do not edit `tools/` (one granted exception: CLEAN-RT's `tools/w3b_driver.c`), docs outside their crate, or the new sm-cli test files that belong to INT; no `HashMap` order in output; determinism test (two runs, `RAYON_NUM_THREADS=1` vs default); property tests with `sm_testkit::strategies::config(n)`; rustdoc on every public item; no wall-clock gates (work counters only); no `unwrap` on input paths. **Hash rule for W3c:** a moved existing hash (W2 milestone/fork, w3a A/B, W3b `h_model`/`h_auto`/`h_mesh`, `SUBMAP_GRADED`, `p0_rows`, any golden) stops the task and goes to the user; it is never re-blessed by a worker (Q5 requires explicit-scheme bit-identity; C3).

---

## 2. Rulings cited in this plan (from `docs/design/w3b/rulings.md`)

| Id | Ruling | Consequence in W3c |
|---|---|---|
| Q2 | Cross sections never enter smart-mesh or sm-rt; region names are material names. | CSG import carries material names only (`m<id>`, `void`). |
| Q4 | CSG cells must be disjoint; MCNP/OpenMC importers reject overlapping decks. | CSG-CORE `ImportError::Overlap`; gaps are warnings (C9). |
| Q5 | `mesh.auto` records explicit assignments; replay never re-runs a strategy; explicit-scheme runs stay bit-identical; every changed case is reported. | MESHER-REPLAN and CYL-SCHEMES census is blocking; moved hash = stop (C3). |
| Q9 | Published benchmark from one bare-metal Linux x86_64 machine; macOS is smoke only. | BENCH-PIN firm; competitor workloads and XDG stretch (C13). |
| O7 | MCNP/OpenMC importers go to W3c. | CSG-CORE, CSG-OPENMC firm; CSG-MCNP stretch, gated on C7. |
| R9 | `.smrt`: one region per material block. | CSG one region per material; EXPORT-A keeps block-named regions. |
| R10 | Volumes in no block: fall back to volume-id names with a warning; a clean export means a fully defined model. | ACR-EXP2 (note severity), EXPORT-A/B/C surface the warning (C10). |
| R11 | `session.export` option for `split_warped_faces` (like P5). | ACR-EXP1, EXPORT-A/B/C. `warp_tolerance` stays library-only. |

---

## 3. W3c-0 (orchestrator, on main before workers branch)

### 3.1 Order
1. **0.1 CI unmasking.** Add `--no-fail-fast` to `cargo test --workspace` in `tools/ci.sh`, so one failing binary no longer hides the crates after it (`drafts/linux-hash.md` F3, Q4 with its critique delta).
2. **0.2 `flat_golden` ruling.** Ask C11. On yes: key the four bundle-hash constants of `crates/sm-rt/tests/flat_golden.rs` per os-arch (macOS values kept; Linux values from the CI log or the LINUX-HASH container run; structure hashes required equal on all platforms), with both tuples pasted into `platform-hashes.md` (`drafts/linux-hash.md` acceptance 4 and Q3 deltas). On no: the gate stays red and the report says so; CLEAN-RT does not touch the file either way.
3. **0.3 W3c-PY-CI** (sm-py micro task, a worker): find the root cause of `test_errors.py::test_python_side_validation_raises_before_any_call[<lambda>5] DID NOT RAISE InvalidParamsError` first (the critique's guess, unverified: a new op keyword after the `ops.pyi` regeneration), then fix it in `crates/sm-py/`. Acceptance: `crates/sm-py/ci/py.sh` green locally and the GitHub python job green on the merge; the commit message states the cause. If the cause is outside sm-py, stop and report.
4. **0.4 ACR-EXP1 + ACR-EXP2** (§3.2) with every mechanical literal fix-up (`split_warped_faces: false` at the sites listed in `drafts/export-opts.md` Findings "R11 path") in **one commit**, plus the two protocol golden insertions (`drafts/export-opts.md` acceptance 1). ACR-EXP2 lands only if C10's third part is approved; if not, EXPORT-A/B/C are re-planned before they branch.
5. **0.5 Cargo.lock:** add `roxmltree` (exact version at least two weeks old) for sm-csgio (`drafts/csg-import.md` Q-XML delta), and the `tools/ci.sh` line for `cargo test -p sm-csgio` if the workspace glob does not already cover it.
6. **0.6 LINUX-HASH container run** starts here (read-only; it can run while sub-wave 1 works).

Workers of sub-wave 1 branch from the commit that contains 0.1-0.5.

### 3.2 Contract changes (the only ACRs in W3c)
- **ACR-EXP1** (`sm-protocol`, additive): `Request::Export.split_warped_faces: bool`, `#[serde(default, skip_serializing_if = "is_false")]`, OpenFOAM only. Exactly `drafts/export-opts.md` Scope 1 "ACR-EXP1".
- **ACR-EXP2** (`sm-protocol`, additive, needs C10): `#[non_exhaustive] enum NoteSeverity { Info (default), Warning }`, `ExportNote.severity` skipped when `Info`, `ExportNote::warning`, `ExportNote::is_warning`, re-exported. Exactly `drafts/export-opts.md` Scope 1 "ACR-EXP2". Existing notes serialise byte for byte as before.
- No other frozen crate changes. MESHER-REPLAN, CYL-SCHEMES, CSG-*, BENCH-*, CLEAN-* need none (each draft's "Contract changes" section). Optional and not scheduled: `Driver::plan_replanning` in sm-scheme-api (`drafts/cyl-pin.md` Contract changes).

---

## 4. Tasks

### 4.1 Sub-wave 1 (disjoint crates, in parallel)

### W3c-MESHER-REPLAN. `sm-mesher` (L3): plan-time replan in `mesh.auto` (B6, round pin prerequisite)
- **Owner:** `crates/sm-mesher/`; by explicit permission the `b6` row of `crates/sm-server/tests/auto_select.rs` (test file only, no sm-server src).
- **Scope:** `drafts/auto-b6.md` Scope "B6" with its critique delta, which supersedes `drafts/cyl-pin.md` Scope 2. Summary: `pub(crate) fn plan_with_blame(driver, input, intent, budget)` looping over `driver.plan_with(&mi, intent, &rej)`; on `NoApplicableScheme`, reject the first `imposed_by` entry (in its sorted order) that is not hard, not `copy`, not already rejected, and on a repeat failure on the same face move to the next entry instead of stopping at `.first()`; insert `Attempt::new(t, s, Phase::Resolve, FailureKind::NotApplicable, e.to_string())`; plan-time and generation-time rejections share one budget (`max_replans`); `mesh_from_plan` receives them through `plan.rejections`. Called from `scratch_plan` only. **Plan delta:** `mesh_ops.rs`, `plan_query.rs`, `census.rs` are not edited (`mesh.run` gets no replan: `mesh.auto` stores explicit assignments, unassigned volumes reach only `[map]`, and blame skips hard assignments). DESIGN.md "Auto selection": plan-time blame and the B6 outcome. The strategies part of the draft is not in this task (W3c-STRATEGIES, C5).
- **Contract changes / ACRs:** none (`Driver::plan_with`, `Rejections`, `SchemeError::blame`, `NoApplicableScheme.imposed_by`, `Attempt::new` are public).
- **Acceptance:** `drafts/auto-b6.md` acceptance 1 (with the added second-entry test: the blamed imposer must be the second `imposed_by` entry), 2, 3, 4 (no existing test changes), 6 (B6 rewritten; counts re-observed on the final code; outcome per C2), 11 (rows 6 only), 12 (census; moved hash = stop, to the user), 13.
- **Gates:** census first: record `milestone`, `milestone_w3a`, `milestone_w3b` (`h_model`, `h_auto`, `h_mesh`, `SUBMAP_GRADED`) and `p0_rows` on `d7ad022`, then after the change; any difference stops the task (C3). Blocks: CYL-PIN.
- **Hash impact:** none expected (the loop runs only after a failed first plan). Inputs that failed may now succeed (B6), reported by the census.

### W3c-CYL-SCHEMES. `sm-scheme-sweep` + `sm-scheme-tet` (L3): strip check and three-face shells
- **Owner:** `crates/sm-scheme-sweep/` and `crates/sm-scheme-tet/` (one worker, explicit two-crate ownership).
- **Scope:** `drafts/cyl-pin.md` Scope 1 (sweep F1: reject a pair unless every linking face has exactly one loop and at least 2 column coedges, seam counted twice; reasons `linking face {l} has inner loops (not a strip)` and `linking face {l} has {n} column coedges`; not the stricter all-edges rule; DESIGN.md pipeline item 1) and Scope 3 (tet F4: replace `uses.len() < 4` by the closed-shell check; the 3+-face rule vs seam-only rule is decided by the tet spike, acceptance 10).
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/cyl-pin.md` acceptance 9 **first** (census, blocking: `milestone`, `milestone_w3a`, `milestone_w3b` incl. `SUBMAP_GRADED`, `p0_rows`, all sm-scheme-* and sm-mesher tests, before and after; the 18 W3b milestone `sweep` volumes are the population at risk because `find_columns` takes the first passing pair), then 1 (strip check unit tests), 10 (tet spike), 11 (clippy, rustdoc, DESIGN.md in sweep and tet). Plan delta: the commit lists every error-text change (an explicit `sweep` on an invalid pair now fails at plan time, not at generation).
- **Gates:** none to start; a moved hash in the census stops the task and goes to the user (C3), never a re-bless. Blocks: CYL-PIN.
- **Hash impact:** none intended; F1 can move a hash only if a previously chosen pair is now rejected and its mesh worked, which the census detects.

### W3c-EXPORT-A. `sm-fem` + `sm-server` (L3/L5): `split_warped_faces` wiring and the R10 warning
- **Owner:** `crates/sm-fem/`, `crates/sm-server/`.
- **Scope:** `drafts/export-opts.md` Scope 2, including the critique delta that moves the `smrt_regions` duplicate-name test and invariant comment here (`drafts/cleanup.md` T1 item 3).
- **Contract changes / ACRs:** consumes ACR-EXP1, ACR-EXP2 (W3c-0 item 0.4).
- **Acceptance:** `drafts/export-opts.md` acceptance 2, 3, 4 (with the 4(d) delta: in-process build comparison only, no committed sha256 of `.smrt` bytes; and 4(e), the block/sideset same-name test), 10.
- **Gates:** after W3c-0 item 0.4. Blocks: EXPORT-C acceptance 8(c)/(d), EXPORT acceptance 6 (combine), CLEAN-SERVER.
- **Hash impact:** none on state hashes; default exports byte-identical; only opt-in `split_warped_faces` output differs; only the smrt fallback note gains `"severity":"warning"`.

### W3c-EXPORT-B. `sm-cli` + `sm-mcp` (L6/L7): flag, stderr notes, MCP schema and warning lines
- **Owner:** `crates/sm-cli/` (src and existing tests; not the INT test files), `crates/sm-mcp/`.
- **Scope:** `drafts/export-opts.md` Scope 3 (CLI `--split-warped-faces`; `render::export` prints notes to stderr per C10; MCP `session_export` schema brought up to the protocol, a gap fixed here, not a question; leading `warning:` text lines in `tool_result`).
- **Contract changes / ACRs:** consumes ACR-EXP1, ACR-EXP2.
- **Acceptance:** `drafts/export-opts.md` acceptance 5, 7, 9 (sm-mcp session goldens regenerated with `SM_MCP_BLESS=1`, diff limited to the `session_export` entry; plan delta: the orchestrator reviews that diff at merge, `tools.standard.txt` unchanged). Acceptance 6 runs at combine (orchestrator), after A and B merge.
- **Gates:** after W3c-0 item 0.4; check first that no `milestone_w3b.rs` helper asserts empty stderr on a successful export (`drafts/export-opts.md` Gates).
- **Hash impact:** none; stdout unchanged; new stderr lines.

### W3c-EXPORT-C. `sm-py` (L6): `Session.export` options and `ExportWarning`
- **Owner:** `crates/sm-py/`.
- **Scope:** `drafts/export-opts.md` Scope 4 (`ExportWarning` per C10; no `ops.pyi` regeneration).
- **Contract changes / ACRs:** none in Rust; reads `severity` from the wire.
- **Acceptance:** `drafts/export-opts.md` acceptance 8 (a), (b), (e) in the worker; (c) and (d) need EXPORT-A merged and run at combine; acceptance 9 first sentence (`gen_stubs.py --check` with an empty `ops.pyi` diff).
- **Gates:** after W3c-0 items 0.3 (PY-CI, same crate) and 0.4.
- **Hash impact:** none.

### W3c-CLEAN-RT. `sm-rt-capi` (L4) + one sm-rt test + `tools/w3b_driver.c`
- **Owner:** `crates/sm-rt-capi/`, `crates/sm-rt/tests/delta.rs`, `tools/w3b_driver.c` (explicit orchestrator permission for this one tools file).
- **Scope:** `drafts/cleanup.md` T2 Scope 1-4 (ABI 10200 with the rule in DESIGN.md; driver tolerance comment and `ridx` guard; acc04 brackets `next_segment`; find and fix the `smrt-capi-*` tmp-dir leak). Not `flat_golden.rs` (W3c-0 item 0.2).
- **Contract changes / ACRs:** none (`smrt.h` is sm-rt-capi's own).
- **Acceptance:** `drafts/cleanup.md` T2 acceptance 1-5.
- **Gates:** branches after W3c-0 item 0.2.
- **Hash impact:** none; `.smrt` bytes unchanged; `smrt_version()` returns 10200.

### W3c-BENCH-PIN. `sm-rt-bench` (L4): pinning, image identity, Docker hygiene, small fixes
- **Owner:** `crates/sm-rt-bench/` including `docker/` and `drivers/`.
- **Scope:** `drafts/bench-arm64.md` Scope A (pins.env single source; `XDG_COMMIT=fec80980ce29` and `MOAB_VERSION=5.6.0`, a mechanical correction because the old pin never existed, also fixed in the xdg Dockerfile so the lint passes; per-architecture conda locks and `docker-lock`; apt versions recorded, not frozen, unless C13 says otherwise; `ImageIdentity` and `images` filled, out of `result_hash`; `--platform` explicit, mixed-arch refused), Scope D (Docker VM block in `bench-env.json`; `--cpus`; smoke threads 1 and 4, worker call), Scope E (P1 `tiny` weld or "tiny is not a benchmark scale", worker call; doc updates).
- **Contract changes / ACRs:** none (`smbench-result v1` additive fields).
- **Acceptance:** `drafts/bench-arm64.md` acceptance 1, 2 and 3 for the two DAGMC images (xdg: the pin lint of 1(a)/(b) only), 8 (sm-rt part and the competitor `w-track` part), 9, 10, 11, 12.
- **Gates:** Docker Desktop and network; D items report `partial` without Docker, never numbers.
- **Hash impact:** none on state hashes; only `tiny/P1/*` lines of `expected_sha256.txt` may change (orchestrator blesses those lines only).

### W3c-CSG-CORE. `sm-csgio` (new crate, L3): IR, DNF, instantiation, overlap rejection, `smcsg`
- **Owner:** `crates/sm-csgio/` (new; picked up by the `crates/*` glob).
- **Scope:** `drafts/csg-import.md` Scope 1-7 with the critique deltas: canonical order of the flat leaf (store surfaces sorted by definition bytes, flat cells by (name, DNF)) before `content_hash`; `roxmltree` as the XML crate (W3c-0 item 0.5); `pub mod openmc;` and `pub mod mcnp;` pre-declared in `lib.rs` and the `--format` dispatch pre-wired in `src/bin/smcsg.rs` (both returning `Unsupported`), so the format tasks touch only their own directories; dev-dependencies `sm-model` and `sm-testkit`.
- **Contract changes / ACRs:** none (`drafts/csg-import.md` Contract changes).
- **Acceptance:** `drafts/csg-import.md` acceptance 3, 4 and 5 against a hand-built IR of Deck A (no parser yet), 12 (core errors: cyclic universe, unknown universe, `TooManyTerms`, `max_cells`), 21, 22 (canonical order makes it hold), 23 (core parts); the `smcsg` exit codes 0/1/2 on hand-built worlds. Names and gaps per C9 (defaults: the material `name` when non-empty and unique, else `material_<id>`; `void`; gaps as warnings; `--strict-gaps`).
- **Gates:** after W3c-0 item 0.5. Core frozen after merge. Blocks: CSG-OPENMC, CSG-MCNP.
- **Hash impact:** none (new crate, no ops, no protocol surface).

### W3c-LINUX-HASH. orchestrator: Linux arm64 container recipe and hash comparison
- **Owner:** orchestrator; `tools/platform/` (new files), `docs/design/w3c/platform-hashes.md`. No crate source.
- **Scope:** `drafts/linux-hash.md` S1 (container recipe, pinned digests read, never invented), S2 (`hashes.sh`, `compare.py`, the `flat_golden` and `smrt-bench gen --print` runs), S3 (the note; `expected.txt` waits for INT and C12). S4 (`platform_pins.rs`) is INT's.
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/linux-hash.md` acceptance 1-6 and 8 (acceptance 4 with both `(structure hash, bundle hash)` tuples, macOS and Linux, pasted into the note; acceptance 6, the curved probe, is information only). Divergences follow the draft's "Decision path" under C12.
- **Gates:** Docker; read-only against `d7ad022` and then against the INT commit (INT reruns it with the W3c pin script).
- **Hash impact:** none.

### 4.2 Sub-wave 2

### W3c-CYL-PIN. `sm-server` tests: round pin, lattice, B5
- **Owner:** `crates/sm-server/tests/auto_select.rs`, rows `b5` and new pin/lattice rows only (the `b6` row is MESHER-REPLAN's).
- **Scope:** `drafts/cyl-pin.md` Scope 4 (b5 only) and the pin/lattice tests. No library code: if any acceptance below needs a source edit (in particular in `crates/sm-mesher/src/mesh_ops.rs`), the worker stops and reports.
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/cyl-pin.md` acceptance 2 (through pin all-hex), 3 (enclosed pin all-tet, if C1 is yes; run with brick-first and with cylinder-first creation order, both passing), 4 (3×3 lattices, through and enclosed), 5 (B5 pass, the tetmesh workaround kept as a second assertion), 7 (decided: reads `MeshEntry::plan.rejections` through `mesh.info`, or is dropped if `mesh.info` does not expose it, said in the commit), 8 (determinism, CLI/MCP/in-process agree). Cell counts (220, 2542, 432, 6647, 154) are re-observed on the merged code and pinned as observed; volumes are checked analytically as in the draft.
- **Gates:** after MESHER-REPLAN and CYL-SCHEMES merge.
- **Hash impact:** none on existing hashes (new rows; b5 pinned outcome changes from limit to pass).

### W3c-CLEAN-SERVER. `sm-server` (L5): 503 refusal off the accept thread, `OPENING` isolation test
- **Owner:** `crates/sm-server/`.
- **Scope:** `drafts/cleanup.md` T1 Scope 1, 2 (items 3 and 4 moved or dropped per their critique deltas; item 5 is CLEAN-CADIO).
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/cleanup.md` T1 acceptance 1, 2, 3, 7 (`-p sm-server`).
- **Gates:** after EXPORT-A merges (same crate).
- **Hash impact:** none.

### W3c-CLEAN-CADIO. `sm-cadio` (micro): `attributes: false` record
- **Owner:** `crates/sm-cadio/`.
- **Scope:** `drafts/cleanup.md` table row "CAD-ATTR" with its critique delta: first check whether node or record hashes cover the op payload (a recorded `Some(false)` must still replay under the strict check); default is to fix the `DESIGN.md:277` wording; the `skip_serializing_if` code change only if shown safe.
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/cleanup.md` T1 acceptance 6 as amended by its critique delta; `cargo test -p sm-cadio`, clippy.
- **Gates:** none (any time in sub-wave 2).
- **Hash impact:** none on state hashes; record JSON only if the code change is taken.

### W3c-CSG-OPENMC. `sm-csgio` (L3): OpenMC `geometry.xml` / `materials.xml`
- **Owner:** `crates/sm-csgio/src/openmc/` and its tests.
- **Scope:** `drafts/csg-import.md` Scope 8-12 with the critique deltas (roxmltree; any convention not confirmed by the manual or a user deck returns `Unsupported`, and the test asserts it). Subset per C6, names per C9.
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/csg-import.md` acceptance 13 (Deck O chords; the MCNP pairing runs in whichever format task merges second), 14, 15, 16, 17 (multi-row lattices and multi-axis rotations `Unsupported` unless confirmed), 18, 19, 20 (OpenMC half, `sm_testkit::topo::assembly::pin_assembly`), 21-23 for its decks.
- **Gates:** after CSG-CORE; conventions wait for C7 (until answered, unconfirmed conventions are `Unsupported`).
- **Hash impact:** none.

### 4.3 Stretch (scheduled only if sub-waves 1-2 land with time left, or on the named answer)

### W3c-BENCH-WORKLOADS (stretch). `sm-rt-bench`: W-fire and W-flight in the DAGMC drivers
- **Owner:** `crates/sm-rt-bench/`.
- **Scope:** `drafts/bench-arm64.md` Scope C for `dagmc_driver.cpp` (both DAGMC images) and the Rust runner side; flights may use an independent RNG explicitly seeded from `--seed` instead of a bit-identical SplitMix64 port (critique delta; the worker states which in DESIGN.md).
- **Contract changes / ACRs:** none.
- **Acceptance:** `drafts/bench-arm64.md` acceptance 4, 5, 6 and 8 for the DAGMC rows (xdg rows `partial`), 7 for the DAGMC `w-track`/`w-flight` rows only.
- **Gates:** after BENCH-PIN; C13.
- **Hash impact:** none on state hashes.

### W3c-CSG-MCNP (stretch, last sub-wave). `sm-csgio`: MCNP decks
- **Owner:** `crates/sm-csgio/src/mcnp/` and its tests.
- **Scope:** `drafts/csg-import.md` Scope 13-17 with the critique deltas (TX/TY/TZ, TRC and `*TR` may be cut to `Unsupported`; unconfirmed conventions `Unsupported`). Uses the corrected Deck C.
- **Acceptance:** `drafts/csg-import.md` acceptance 1, 2 (as restated), 6-12 (with the `Unsupported` assertions where conventions are unconfirmed or features cut), 13 pairing, 20 (MCNP half), 21-23 for its decks.
- **Gates:** after CSG-CORE; C6 and C7 answered (reference decks in hand).
- **Hash impact:** none.

### W3c-BENCH-XDG (stretch or W4). `sm-rt-bench`: XDG image and `ray_fire` driver
- **Scope:** `drafts/bench-arm64.md` Scope B without `smtets_to_moab` and the `walk` mode (tet walk is W4). Acceptance 2 and 3 for `xdg`, the `xdg` rows of 4-6. **Gates:** after BENCH-WORKLOADS; C13. **Hash impact:** none.

### W3c-STRATEGIES (only if C5 = yes). `sm-mesher`: built-in `hex_only` and `tet`
- **Scope:** `drafts/auto-b6.md` Scope "Strategies"; acceptance 5, 7, 8, 9, 10 (8 and 9 first confirm the planner-append behaviour flagged in the draft's Risks). **Owner:** `crates/sm-mesher/` plus the b3/b4/strategy rows of `auto_select.rs`. **Gates:** after MESHER-REPLAN and CYL-PIN merge. **Hash impact:** none on existing hashes (`mesh.auto_strategies` is unhashed).

---

## 5. W3c-INT. Integration and milestone

Integration agent after the sub-wave 2 merges (stretch tasks that merged are included; others are listed as not done). Owner micro tasks for any crate edit.
1. **Round-pin milestone** (C4: new files, the W3b milestone and its hashes untouched): `tools/w3c_pin.smesh` and `crates/sm-cli/tests/milestone_w3c_pin.rs`. The through pin (H = 1.0) and its 3×3 lattice via `model.copy` (18 volumes; hex count as pinned by CYL-PIN; total volume 9 × 1.5876 = 14.2884 to 1e-9); `mesh.auto {strategy: "default"}` with explicit assignments in history (replay calls no strategy); `mesh.run`; `validate` empty and conformal interfaces; blocks per material, `session.export pin.smrt` with block-named regions and **no** `smrt-region-names` warning (R9, R10), loaded by `sm_rt::read_smrt` with the region names equal to the blocks; one `.smrt` export with a volume left out of the blocks shows the warning in CLI stderr, MCP text and Python `ExportWarning`; OpenFOAM export with and without `split_warped_faces` (R11) through CLI and Python. If C1 is yes, an enclosed-pin (H = 0.8) row, all-tet, in the same test. `script.record` and replay equal; CLI, MCP, in-process and Python state hashes equal; two runs and `RAYON_NUM_THREADS=1` vs default equal.
2. **Linux arm64 hash comparison:** rerun W3c-LINUX-HASH `hashes.sh` on macOS and in the arm64 container on the INT commit, now including `w3c_pin.smesh` (the first hashed curved script); write the table into `docs/design/w3c/platform-hashes.md` (smoke). If C12 is yes: `tools/platform/expected.txt` and `crates/sm-cli/tests/platform_pins.rs` (`drafts/linux-hash.md` S4, acceptance 7), so Linux x86_64 CI enforces the identity hashes. A divergence follows the draft's "Decision path" and C12; nothing is pinned per platform for a state hash without a ruling.
3. **CI:** after W3c-0 items 0.1-0.3 the GitHub gate and python job must be green on the INT commit; if not, the report records them as red with their causes (never as rc = 0).
4. **Benchmark smoke** on this Mac: `smrt-bench docker-check` and a P1 `reduced` run with `--trackers smrt,dagmc-moab,dagmc-dd --threads 1,4` (plus fire/flight rows if BENCH-WORKLOADS merged), `report` under the `SMOKE RESULT` banner with image identities and the Docker VM block; nothing under `results/` committed; no ratios.
5. **Combine-time tests:** `drafts/export-opts.md` acceptance 6 and 8(c)/(d); the CSG MCNP-vs-OpenMC pairing (acceptance 13) if both merged.
6. **Goldens:** only the sm-mcp `session_export` entry (EXPORT-B) and the two sm-protocol insertions (W3c-0) may change; any other moved golden or hash stops integration and goes to the user.
7. **W3c-CLEAN-DOCS** (`drafts/cleanup.md` T3 with its deltas): W3b report deviations numbered D1..Dn, GitHub CI recorded as red with causes for the W3b period, Chrome version; PARITY.md:31 wording; backlog dispositions with W4 triggers; tmp-dir leak result; the round-pin repro recipe (`model.subtract` without `keep`, then imprint and merge); the W3b plan B6 wording per C2; W3c deferrals (H1 analytic, P2 tet twin, `--model` periodic faces, XDG tet walk, O-grid all-hex pins, hex-tet transition cells).
8. **PARITY.md:** B Import "MCNP/OpenMC CSG" (with the supported subset; MCNP only if merged), C auto selection (round pin cell, B5, B6), E OpenFOAM `split_warped_faces` option and export warnings; each ticked only with a test name.
9. **Report** `docs/design/w3c/w3c-report.md`: deviations, census results (every changed case), platform table, benchmark smoke, stretch items done or carried to W4.

---

## 6. Dependencies (summary)

```
W3c-0: 0.1 --no-fail-fast, 0.2 flat_golden (C11), 0.3 PY-CI, 0.4 ACR-EXP1/EXP2 + fix-ups (C10), 0.5 roxmltree, 0.6 LINUX-HASH start
  ├─ sub-wave 1: MESHER-REPLAN ∥ CYL-SCHEMES ∥ EXPORT-A ∥ EXPORT-B ∥ EXPORT-C ∥ CLEAN-RT ∥ BENCH-PIN ∥ CSG-CORE ∥ LINUX-HASH (orchestrator)
  ├─ sub-wave 2: CYL-PIN (after MESHER-REPLAN + CYL-SCHEMES), CLEAN-SERVER (after EXPORT-A), CLEAN-CADIO,
  │              CSG-OPENMC (after CSG-CORE; conventions after C7), stretch BENCH-WORKLOADS (after BENCH-PIN; C13)
  ├─ sub-wave 3 (stretch): CSG-MCNP (after CSG-CORE; C6, C7), BENCH-XDG (after BENCH-WORKLOADS; C13), STRATEGIES (C5 = yes)
  └─ W3c-INT
```
Gates inside sub-waves (the dependent part waits, the rest proceeds): EXPORT-C 8(c)/(d) and EXPORT acceptance 6 at combine after EXPORT-A (and B); CYL-PIN 3 (enclosed pin) on C1; MESHER-REPLAN 6 outcome on C2. Census results in MESHER-REPLAN and CYL-SCHEMES are blocking (C3). Merge order in sm-mesher/sm-server tests: MESHER-REPLAN before CYL-PIN; EXPORT-A before CLEAN-SERVER.

Not in W3c (recorded in the report): `mesh.run`/`mesh.plan` plan-time replan; O-grid all-hex enclosed pins and hex-tet transition cells (W4, C1); CSG-IMAGE and the deck-to-`.smesh` B-rep route (W4, C8); XDG tet walk, H1 analytic row, P2 tet twin, `--model` periodic faces (W4 / other owners); face-run size inheritance (C14); the deferred backlog items of `drafts/cleanup.md` with their triggers.

---

## 7. Open questions for the user

C1. **Enclosed pin (H=0.8) all-tet for now**, with O-grid all-hex and hex-tet transition cells deferred to W4? *Rec: yes; the through pin is all-hex.*

C2. **B6 outcome with `default`**: accept a conformal wedge fill of the right volume (`{Tet:160, Wedge:32}`), not `tetmesh` as plan.md says? (Merges CYL q2 and Q-B6-1.) *Rec: accept, and amend the B6 wording.*

C3. **A census hash move is a stop.** If the sweep strip check or the replan moves any existing milestone hash (W2, W3a, W3b h_*, SUBMAP_GRADED), it is reported to you and not blessed. Agreed? *Rec: yes.*

C4. **Round-pin milestone**: add a new `tools/w3c_pin.smesh` and test, leaving the W3b milestone and its hashes as they are? *Rec: yes.*

C5. **Built-in strategies** `hex_only` and `tet` in W3c? *Rec: defer to W4. The benchmark argument does not hold; B3 and B4 stay covered by stub tests and explicit scripts.*

C6. **CSG subset** (Q-SUBSET): the proposed MCNP and OpenMC subset, with hex lattices, periodic boundaries, `like n but` and DAGMC universes refused. Does anything on the refused list appear in your decks? *Rec: accept; MCNP is stretch in W3c.*

C7. **Reference decks** (Q-REFDECKS): can you supply 2-3 small MCNP and OpenMC lattice and rotation decks with known positions? *Rec: yes, before the format tasks start. Until then, unconfirmed conventions return `Unsupported`.*

C8. **CSG output** (Q-OUTPUT): in-process plus the `smcsg` CLI only in W3c, with no CSG section in `.smrt`? B-rep import (deck to `.smesh` script) goes to W4. *Rec: yes.*

C9. **CSG names and gaps**: regions named `m<id>`, void cells named `void`, a user override map (R9: one region per material), and gaps reported as warnings only (`--strict-gaps` to fail)? *Rec: yes to both.*

C10. **R10 surfacing**:
- Should Python raise an `ExportWarning` (UserWarning) as well as filling `ExportResult.warnings`? *Rec: yes.*
- Should the CLI print every export note on stderr, or warnings only? *Rec: all notes as `note:`/`warning:` lines; stdout unchanged.*
- Approve ACR-EXP2 (note severity)? *Rec: approve.*

C11. **`flat_golden` on Linux**: its comment says moved values are never re-blessed. May the bundle-hash constants be keyed per os-arch, keeping the macOS values and requiring equal structure hashes? *Rec: yes.*

C12. **Platform pins and divergence rule**: pin the milestone identity hashes in `tools/platform/expected.txt`, enforced by Linux CI. If a quantized hash diverges: fix the near-tie decision locally first; use a portable libm only if that fails; raw-bit hashes stay per platform. *Rec: yes to both.*

C13. **Benchmark scope in W3c**: pinning, identity and Docker hygiene (plus DAGMC W-fire/W-flight if time allows); the XDG image and tet walk go to W4. Also: record apt versions instead of freezing via a snapshot mirror? *Rec: yes to both.*

C14. **Cubit parity**: should a face-only `mesh.run` inherit the parent volume's `mesh.size`/`mesh.intervals`? *Rec: keep the current behaviour and the warning unless you know Cubit inherits; inheriting would be a W4 change with a hash A/B.*

**Answered without a user question** (by a ruling, or as an orchestrator or implementer call; recorded in the drafts' critique deltas): CYL q2 (fold B6: B6 goes to MESHER-REPLAN) and q4 (tet shell rule: the tet spike decides); Q-B6-3 (`mesh.run` replan: no) and Q-B6-4 (test-file ownership: the b6 row only); Q-XML (roxmltree); EXPORT Q2 (MCP schema: fixed in EXPORT-B) and Q5 (`warp_tolerance`: answered by R11); Q-xdg-pin (mechanical correction), Q-tiny (weld, else document), Q-threads (1 and 4), Q-deferred-rows (wave summary); LINUX-HASH Q4 (`--no-fail-fast`: W3c-0); CLEANUP Q1 (ABI 10200) and Q3 (`attributes: false`: DESIGN wording unless the code change is shown safe).
