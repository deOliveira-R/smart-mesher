# W4 plan: polyhedral meshes for the transport code, ray tracing that beats DAGMC/XDG, then parity

Base: `main` at `d2f4803` (W3c-INT merged in f2e831d, report follow-ups 63ec23b, backlog d2f4803). Inputs, all committed next to this file: the area drafts `docs/design/w4/drafts/<key>.md` (`poly.md`, `rt.md`, `hexpin.md`, `vtopo.md`, `csgbrep.md`, `meshops.md`, `export.md`). Each draft was corrected by an Opus critique, and every correction is marked **Critique delta** inside the draft, so a worker never needs the critique. The W3c plan (`docs/design/w3c/plan.md`) is the structural model.

**Goals (user).** (1) Polyhedral meshes for the user's own neutron-transport code. (2) Ray tracing (`sm-rt`, `.smrt`) that beats DAGMC and XDG. Standing principle: Cubit is a bar to surpass, not to match; we pick the best design we can find. MCNP is not a target. OpenMC is the CSG format and the oracle (conda env `/Users/rodrigo/opt/miniforge/envs/openmc`, OpenMC 0.16.0).

**Rulings:** `docs/design/w3b/rulings.md` and `docs/design/w3c/rulings.md` stay binding where §2 lists them. The answers to §9 (D1-D10) will go into `docs/design/w4/rulings.md` and override this plan where they differ.

**Citation convention.** Paths are relative to `docs/design/w4/`. "`drafts/<key>.md` §T acceptance N" is item N of the **Acceptance** list under the task heading T in that draft (T is the draft's own name: `W4-POLY-VORO`, `T1 VTOPO-OPS`, `MOD-HTET`, `CSGBREP-CORE`, ...). Lettered items keep their letter (`§W4-POLY-MEASURE acceptance (b)`). "F3", "finding 7", "D2" refer to that draft's Findings and Design items. A cited item includes the **Critique delta** written next to it. Where this plan changes a cited item, the change is written next to the citation as "plan delta". `plan.md` wins over the drafts; the rulings win over both.

**Facts the plan relies on** (checked at `d2f4803` by the drafts' critiques, spot-checked here): `tools/platform/expected.txt` has 63 lines and no poly or sm-rt structure entry; `sm-rt-bench` depends on neither sm-poly nor sm-mesher (`crates/sm-rt-bench/Cargo.toml:18-27`); `standard_schemes` registers eight scheme crates (`crates/sm-server/src/standard.rs:25-32`) and fails if an auto list names an unregistered scheme (`standard.rs:240-241`); `mesh.to_poly` refuses any non-tet cell (`crates/sm-poly/src/prep.rs:237-238`); `ApplyCtx::apply_nested` is `pub(crate)` (`crates/sm-op-api/src/op.rs:619-648`); `MeshLayer` stores no level map (`crates/sm-layer-api/src/mesh.rs:16-41`); `ModelLayer` stores no overlay (`crates/sm-layer-api/src/model.rs:44-51`); the XDG pin `83efb306e152` is 5 commits behind upstream `a5d67e6` (`drafts/rt.md` finding 2).

**Milestone (§5).** An OpenMC pin-cell deck becomes B-rep volumes in one op, is meshed all-tet and dualized to a convex circumcentric polyhedral mesh where the strict gate allows (barycentric otherwise), and is exported to `.smsh` and `.smrt`; the chain is checked against OpenMC's own cell lookups and against closed-form polygon volumes. The round pin is all-hex in both forms (through and enclosed) with an O-grid. The benchmark has XDG rows (fire, tet walk), a region-jump row and a polyhedral row, a pre-registered verdict in code, and a smoke run on arm64; the published x86_64 table follows when the machine exists (D6).

---

## 1. Ownership

One worker per row. Rows that share a crate run one after the other, in the order given in §4. "Files" means file-level ownership inside a crate (W3c precedent: CYL-PIN owned rows of one test file).

| Task | Crate / dir | Sub-wave | Size | Owns |
|---|---|---|---|---|
| W4-0 | orchestrator | 0 | M | `*-api`, `sm-protocol`, `sm-testkit`, root `Cargo.toml`/`Cargo.lock`, `crates/sm-server/src/standard.rs` (registration lines only), new crate skeletons, `tools/`, CI, docs |
| W4-POLY-MEASURE | `sm-quality`, `sm-poly` | a1 | S/M | `crates/sm-quality/`, `crates/sm-poly/` |
| W4-POLY-VOLDIAG | `sm-diag` | a1 | S | `crates/sm-diag/` |
| W4-RT-XDG | `sm-rt-bench` | a1 | L | `crates/sm-rt-bench/` except `src/result.rs`, `src/report.rs` and their tests |
| W4-RT-VERDICT | `sm-rt-bench` (files) | a1 | M | `crates/sm-rt-bench/src/result.rs`, `src/report.rs`, their tests, new `crates/sm-rt-bench/VERDICT.md` |
| W4-RT-WORLD | `sm-rt` | a1 | L | `crates/sm-rt/` |
| W4-HEX-STRAT | `sm-mesher` | a1 | S | `crates/sm-mesher/` |
| W4-HEX-DECOMP | `sm-decomp` (new) | a1 | M | `crates/sm-decomp/` |
| W4-HEX-OGRID-K | `sm-scheme-ogrid` (new) | a1 | M | `crates/sm-scheme-ogrid/` |
| W4-CSG-CLASSIFY | `sm-csgbrep` (new) | a1 | S | `crates/sm-csgbrep/src/quadric.rs` and its tests |
| W4-CSG-PROBE | `sm-boolean` (one test file) | a1 | S | `crates/sm-boolean/tests/webcut_probe.rs` (new) |
| W4-CSG-LIBFN (if needed) | `sm-model`, `sm-fem` | a1 | S | library entry points named by W4-0 item 0.4 only |
| W4-CLI-DIAG | `sm-cli` | a1 | S | `crates/sm-cli/` (src and existing tests) |
| W4-EXO-VERIFY | `sm-exodus` | a1 | S | `crates/sm-exodus/` (tests and docs unless a bug is found) |
| W4-DAGMC-MODEL (if D7) | `sm-server`, `sm-meshio/src/dagmc` | a1 | M/L | `crates/sm-server/`, `crates/sm-meshio/src/dagmc/` |
| W4-POLY-VORO | `sm-poly` | a2 | L | `crates/sm-poly/` |
| W4-HEX-QUERY | `sm-mesher` | a2 | S | `crates/sm-mesher/` |
| W4-HEX-OGRID-S | `sm-scheme-ogrid` | a2 | M | `crates/sm-scheme-ogrid/` |
| W4-CSG-CORE | `sm-csgbrep` | a2 | L | `crates/sm-csgbrep/` |
| W4-RT-ROWS | `sm-rt-bench` | a2 | M | `crates/sm-rt-bench/` |
| W4-RT-CELLINDEX | `sm-rt` | a2 | L | `crates/sm-rt/` |
| W4-HEX-PIN-INT | tests + tools (new files) | a3 | M | `crates/sm-cli/tests/milestone_w4_pin.rs`, `tools/w4_pin_ogrid.smesh`, `tools/w4_pin_enclosed_hex.smesh` (explicit permission) |
| W4-MESHER-LISTS | `sm-mesher` | a3 | S | `crates/sm-mesher/` |
| W4-CSG-CHECK, then W4-CSG-ORACLE | `sm-csgbrep` (+ fixtures) | a3 | M + M | `crates/sm-csgbrep/`; ORACLE also `crates/sm-csgio/tests/fixtures/openmc/` (explicit permission) |
| W4-POLY-MIXED (if D1) | `sm-poly` | a3 | M | `crates/sm-poly/` |
| W4-BOOL-TOOLS | `sm-boolean` | b1 | M/L | `crates/sm-boolean/` |
| W4-BOOL-HEAL-A | `sm-boolean` | b2 | S + M | `crates/sm-boolean/` |
| W4-VTOPO-OPS | `sm-model` | b | M | `crates/sm-model/` |
| W4-STL-IN | `sm-cadio` | b | M | `crates/sm-cadio/` |
| W4-MOD-REFINE-U | `sm-modify` (new) | b | M/L | `crates/sm-modify/` |
| W4-POLY-AGG (if D3) | `sm-poly` | b | M | `crates/sm-poly/` |
| W4-POLY-SCHEME, then W4-SIZE-IIA-OP | `sm-mesher` | b | S/M + S | `crates/sm-mesher/` |
| W4-SIZE-IIA | `sm-sizing`, `sm-scheme-curve` | b | S + S | `crates/sm-sizing/`, `crates/sm-scheme-curve/` |
| W4-EXPORT-SELECT | `sm-meshio`, `sm-fem`, `sm-server` | b | M | `crates/sm-meshio/` (writers' filter), `crates/sm-fem/`, `crates/sm-server/` |
| W4-CSG-CLI | `sm-cli`, `sm-mcp`, `sm-py` | b | S | `crates/sm-cli/`, `crates/sm-mcp/`, `crates/sm-py/` |
| W4-SIZE-CURV | `sm-sizing`, `sm-mesher` | c | M/L | `crates/sm-sizing/`, `crates/sm-mesher/`, one line of `standard.rs:184` (explicit) |
| W4-VTOPO-MESH | `sm-mesher`, `sm-scheme-map`, `sm-scheme-pave` | c | M | those crates |
| W4-EXPORT-FLUENT, then W4-EXPORT-CGNS | `sm-meshio` (+ wiring) | c | M + L | `crates/sm-meshio/`; wiring lines in `sm-server`, `sm-cli` |
| W4-STL-WIRE | `sm-server`, `sm-cli` | c | S | `crates/sm-server/`, `crates/sm-cli/` |
| W4-HEX-MSWEEP | `sm-scheme-sweep` | c | L | `crates/sm-scheme-sweep/` |
| W4-HEX-PYR | `sm-scheme-tet` | c | L | `crates/sm-scheme-tet/` |
| W4-MOD-HTET, then W4-MOD-REFINE-L | `sm-modify` | c | M + L | `crates/sm-modify/` |
| W4-HEAL-TOL, then W4-STEP-REPAIR | `sm-boolean`, then `sm-cadio` | c | M + S/M | `crates/sm-boolean/`, then `crates/sm-cadio/` |
| W4-EXPORT-ABAQUS (last) | `sm-meshio` | c | M | `crates/sm-meshio/src/abaqus/` |
| W4-INT | integration agent | INT | M | §5 |

Rules for every task (CLAUDE.md applies): own only the listed paths; frozen contracts (`*-api`, `sm-testkit`, `sm-protocol`, the sm-csgio core per `crates/sm-csgio/DESIGN.md:3-4`) change only through a W4-0 ACR, else `status: blocked_on_api` with an `api_change_request`; workers do not edit `tools/` (named exceptions only), docs outside their crate, or the INT test files; determinism test (two runs, `RAYON_NUM_THREADS=1` vs default); property tests with `sm_testkit::strategies::config(n)`; rustdoc on every public item; no wall-clock gates (work counters only); no `unwrap` on input paths; no `HashMap` order in output. Hash rule: §7.

Hygiene ride-alongs (`drafts/export.md` §HYGIENE-*): HYGIENE-CLI goes with W4-CLI-DIAG, HYGIENE-RTBENCH with W4-RT-XDG, HYGIENE-CSGIO with W4-CSG-ORACLE, HYGIENE-MESHER with W4-HEX-STRAT, HYGIENE-SERVER with W4-DAGMC-MODEL (or W4-EXPORT-SELECT if D7(b) is no), HYGIENE-PY with W4-CSG-CLI, HYGIENE-RTCAPI and HYGIENE-TOOLS in W4-0.

---

## 2. Standing rulings that still bind

| Id | Ruling | Consequence in W4 |
|---|---|---|
| W3b Q2 | Cross sections never enter smart-mesh or sm-rt; regions are material names. | CSG import carries material names only (`CsgIr::dropped`). |
| W3b Q3 | Hex lattices in sm-rt: stretch, pointy-top only. | CSGBREP-HEX stays trigger-based (D8); the import draft's "both orientations" needs D8's answer. |
| W3b Q4 | CSG cells must be disjoint. | CSG-CHECK default overlap policy is `error` (`drafts/csgbrep.md` D5). |
| W3b Q5 | `mesh.auto` records explicit assignments; replay never re-runs a strategy; explicit-scheme runs stay bit-identical. | New strategies (`hex_only`, `tet`) and lists cannot change old histories; decomposition is explicit ops (`drafts/hexpin.md` D2). |
| W3b Q9 | The published benchmark comes from one bare-metal Linux x86_64 machine arranged by the user; macOS is smoke only. | RT-VERDICT banner rule; D6. |
| W3b R1 | Region-jump tolerance: jump vs walk agreement by the exit-normal rule. | RT-ROWS jump row acceptance. |
| W3b R9, R10 | `.smrt`: one region per material block; volumes in no block fall back with a warning. | CSG import makes one block per region name; per-element regions only on D9. |
| W3c standing principle | Cubit is a bar to surpass; legacy conventions are not references. | Applies to every design choice here. |
| W3c C1 | Enclosed pin all-tet for now; O-grid all-hex and hex-tet transitions in W4. | HEX-DECOMP/QUERY/OGRID/PIN-INT (a1-a3), HEX-PYR (c). |
| W3c C3 | A moved existing hash stops the task and goes to the user. | §7. |
| W3c C5 | `hex_only` and `tet` wait for W4. | W4-HEX-STRAT (a1). |
| W3c C6 | OpenMC only; MCNP is not a target. | No MCNP anywhere; Abaqus is not justified by MCNP6 UM (`drafts/export.md` D1). |
| W3c C7 | OpenMC itself is the oracle. | CSG-ORACLE; VOLFIT ray-sampled volume check. |
| W3c C8 | B-rep import of a CSG model goes to W4. | W4-CSG-* tasks. |
| W3c C9 | Region names: material `name` if non-empty and unique, else `material_<id>`; `void`; override map. | CSG-CORE blocks. |
| W3c C11, C12 | `flat_golden` bundle hashes per os-arch; milestone identity hashes pinned in `tools/platform/expected.txt`, enforced on Linux CI. | New milestone hashes pinned by INT; facet/tess bytes per os-arch. |
| W3c C13 | apt versions recorded, not frozen. | XDG image recipe. |
| W3c C15 | XDG: clone only `xdg-org/xdg`; pin the newest upstream commit that builds and runs. | W4-RT-XDG step 0. |
| W3c process note | A wave merges only with GitHub CI green on its PR as well as the local run. | §7, applied to every sub-wave merge. |

---

## 3. W4-0 (orchestrator)

### 3.1 Order (before W4a workers branch)
1. **0.1 Census baseline.** Record every hash in the §7 population at `d2f4803` into `docs/design/w4/census-base.txt` (macOS arm64; Linux values from the last green CI run). Workers compare against this file.
2. **0.2 New crates.** Skeletons (empty `lib.rs` with crate rustdoc, plugin or `register` stub that registers nothing), root `Cargo.toml` members and the `tools/ci.sh` line for: `sm-decomp`, `sm-scheme-ogrid`, `sm-csgbrep`, `sm-modify`. Register `sm_scheme_ogrid::register` in `standard_schemes` (`standard.rs:25-32`; the stub registers nothing, so no list changes) and the `sm-csgbrep` and `sm-modify` plugins in `standard()`. Check: `smesh ops` lists the plugins, every existing hash in `census-base.txt` is equal.
3. **0.3 ACR-POLY2** (§3.2).
4. **0.4 CSGBRIEF-0** (`drafts/csgbrep.md` §CSGBRIEF-0): record D1 option (a), one op `model.import_openmc`, unless D10 says otherwise; materials as a second `Source` field of the same type (no ACR); decide whether `model.copy` and `fem.block_add`/`fem.sideset_add` need public library functions (`crates/sm-model/src/copy.rs`, `crates/sm-fem/src/ops.rs:84,270`). If yes, W4-CSG-LIBFN runs in a1 with exactly those functions; if the import can call `TypedOp::apply` with its own context without skipping layer bookkeeping, LIBFN is dropped.
5. **0.5 ACR-DAGMC-SRC** (§3.2), only if D7(b) is yes.
6. **0.6 Orchestrator rulings** recorded in `docs/design/w4/rulings.md` (not user questions):
   - `sm-rt-bench` may depend on `sm-poly` (library) for the L2b polyhedral twin. The conforming P1 tet twin is generated offline with the `smesh` CLI into the case directory and its SHA-256 pinned per os-arch; no `sm-mesher` dependency (`drafts/rt.md` risk 8).
   - Healing lives in `sm-boolean`, tolerance first; collapse is virtual (`drafts/vtopo.md` D3, D4).
   - Pre-registered win criteria and the default-build headline with a labelled `target-cpu=native` row are adopted (`drafts/rt.md` Q2, Q5).
   - `renumber` is not in the export selection (`drafts/export.md` D3).
   - GPU smoothing and quality: not in W4, with the trigger of `drafts/export.md` D5 (GPU-DECIDE), unless D10 says otherwise.
7. **0.7 Doc and tools fixes** (HYGIENE-TOOLS and HYGIENE-RTCAPI): `tools/platform/collect.py:58`, macOS `rustc -V`, mypy/ruff for `tools/w3c_driver.py`; mark `docs/design/w3/backlog.md:9,13,24` done and `docs/PARITY.md:59` partly done; the "NetCDF-4" doc at `crates/sm-protocol/src/request.rs:18` and the options doc at `:231-232`; `crates/sm-rt-capi/DESIGN.md:111` width; `docs/research/dagmc-internals.md:16` after the XDG pin moves (done at the RT-XDG merge).
8. **0.8 Bench script** `tools/bench/x86_64.sh` and `docs/design/w4/bench-x86.md` (`drafts/rt.md` §W4-RT-X86 scope, orchestrator part). Can run in parallel with a1.

### 3.2 Contract changes (all additive; done by the orchestrator, never by a worker)

Before W4a (W4-0):
- **ACR-POLY2** (`sm-rt-api`): `TrackerInfo::convex_cells: usize`, filled by sm-rt; `SmrtInfo` in the C ABI unchanged (`drafts/poly.md` §ACR-POLY2). Hash impact: none; `rt.info` output grows.
- **ACR-DAGMC-SRC** (`sm-protocol`, if D7(b)): `Request::Export.source: ExportSource { Mesh (default), Model }`, `#[serde(default, skip_serializing_if = "is_default")]`, valid for `dagmc` only, plus a `chord_tol` key in the DAGMC options map. Existing requests serialise byte for byte as before; the two protocol golden insertions are the only golden change.
- **Doc-only** (`sm-protocol`, `sm-rt-api`): `request.rs:18`, `:231-232`; `Lattice::element_center`/`locate_rect` (`crates/sm-rt-api/src/world.rs:74-80`, `:99-107`) honour their own "z pitch ignored for 2D lattices" doc when `dims[2] == 1` (no caller in the repo, no hash; `drafts/rt.md` finding 8).

Before W4b (W4-0b):
- **ACR-LEVELMAP** (`sm-layer-api`): merges ACR-MOD1 (`drafts/meshops.md` §MOD-0) and ACR-POLY-LEVELMAP (`drafts/poly.md` §W4-POLY-AGG). `MeshLayer` implements the existing `MeshCatalog`/`CatalogEdit::link` (`crates/sm-mesh-api/src/hierarchy.rs:481-515`) with a `BTreeMap<(fine, coarse), LevelMap>`; `on_retired` drops links naming a retired mesh; the digest appends a link section only when at least one link exists. Acceptance: `drafts/meshops.md` §MOD-0 acceptance 1-5; every hash in `census-base.txt` equal.
- **ACR-V1** (`sm-layer-api`): one `Option<Arc<Overlay>>` in `ModelLayer`, shared with the decomposition; `topo()` returns the overlay view when present, new `host()` returns the bare host; the ~5 editing/exporting call sites in `sm-boolean`, `sm-model`, `sm-cadio` switch to `host()` in the same commit; digest contribution only when `Some`, counters included (`drafts/vtopo.md` §W4-0 ACR-V1, D1). Gate: the full workspace tests, not only the pins (`drafts/vtopo.md` R2).
- **ACR-V3** (`sm-topo-api`): `VirtualOp::CompositeEdges` and `VirtualOp::CollapseEdge` with a recorded gap and per-cell view tolerance (`drafts/vtopo.md` §W4-0 ACR-V3, D3). `TopologyEdit::collapse_edge` is not added.

Before W4c (W4-0c):
- **ACR-V2a** (`sm-topo-api`, `sm-scheme-api`): `TopologyView::composite_surface(face)` for single-handle non-periodic composites, `GenCtx::loop_uvs`/`face_normal` using it, and the planner mapping `PartitionFace` to `Target::Surface` (`planner.rs:291`) (`drafts/vtopo.md` §W4-0 ACR-V2, V2a).
- **ACR-EXPFMT** (`sm-protocol`): `ExportFormat::{Fluent, Cgns}` (and `Abaqus` when that task starts); conda-forge `cgns` added to the conformance env (`tools/`).

On demand, not scheduled: ACR-V2b (periodic pcurve synthesis, with T2b), ACR-V4 (T11), a `GenCtx` read of quad-meshed bounding faces if the HEX-PYR spike needs it (`drafts/hexpin.md` R3), the sm-csgio IR `HexLattice` (on D8), the union copy for true many-to-one sweep (`drafts/hexpin.md` R2), the volume copy scheme (`drafts/meshops.md` X-2). Each is an `api_change_request` reviewed by the orchestrator, and each must leave every existing hash equal.

`sm-testkit`: no change needed in W4. Fixtures the tasks need (BCC lattice, notch, step, fillet) are built inside the tasks' own tests.

---

## 4. Tasks

### 4.1 W4a: the two goals (polyhedral transport meshes, beating DAGMC/XDG)

Selection rule: a task is in W4a when it produces or improves a polyhedral transport mesh, an `.smrt` that the transport code consumes, or a benchmark row against DAGMC/XDG; plus three small deferrals from W3c (CLI-DIAG, EXO-VERIFY, STRAT).

#### Step a1 (in parallel, after W4-0)

### W4-POLY-MEASURE. `sm-quality` + `sm-poly`: convexity, fast-path share, dual eligibility
- **Owner:** `crates/sm-quality/`, `crates/sm-poly/` (one worker, two crates).
- **Scope:** `drafts/poly.md` §W4-POLY-MEASURE scope 1-5 (`cell_convexity`, `fast_path_eligible`, `PolyStats` fields, the dual-eligibility counts on a tet mesh, the measurement table, the list of pins). Plan delta: the table also has a column for cells that are convex but over the 64-face cap, so the orchestrator can decide the `CONVEX_CHECK_MAX` lever (`drafts/poly.md` Priority).
- **Acceptance:** `drafts/poly.md` §W4-POLY-MEASURE acceptance (a), (b) (with ACR-POLY2's `convex_cells` once landed), (c), (d).
- **Gates:** W4-0 item 0.3. The kill criterion of the draft is the orchestrator's: the table is reviewed before VORO branches.
- **Hash impact:** none on state hashes; `mesh.poly_info`/`mesh.quality` JSON gain fields; the metric stays out of profile `rules`. A `mesh.quality` schema golden, if one exists, is reported, not re-blessed.

### W4-POLY-VOLDIAG. `sm-diag`: per-region mesh volume against the model
- **Owner:** `crates/sm-diag/`.
- **Scope / acceptance:** `drafts/poly.md` §W4-POLY-VOLDIAG (query `mesh.volume_check`; cylinder `0.5 n r^2 sin(2 pi / n) H` to 1e-12; brick deficit 0; tet and dual agree).
- **Hash impact:** none (query).

### W4-RT-XDG. `sm-rt-bench`: XDG pin, driver and the tet walk (L1, L2)
- **Owner:** `crates/sm-rt-bench/` except `src/result.rs`, `src/report.rs` and their tests.
- **Scope:** `drafts/rt.md` §W4-RT-XDG-PIN (as step 0, with one `const XDG_PIN` used by the tests at `docker.rs:686`, `:704-707`), §W4-RT-XDG-DRIVER, and §W4-RT-TETS scope (a) and (b) on H1. TETS (c), the conforming P1 twin, moves to W4-RT-ROWS. HYGIENE-RTBENCH rides along.
- **Acceptance:** `drafts/rt.md` §W4-RT-XDG-DRIVER acceptance 1-7; §W4-RT-TETS acceptance 1-4 (H1 only).
- **Gates:** Docker Desktop and network; anything not built is `Partial` with the command, never a number. Smoke run `run --cases P1,H1 --scale reduced --trackers smrt,dagmc-dd,xdg`.
- **Hash impact:** new `expected_sha256.txt` lines only; existing lines byte equal.

### W4-RT-VERDICT. `sm-rt-bench` (files): bare-metal banner and pre-registered verdict
- **Owner:** `crates/sm-rt-bench/src/result.rs`, `src/report.rs`, their tests, `crates/sm-rt-bench/VERDICT.md` (new; RT-XDG links it from `DESIGN.md`). The script is orchestrator work (W4-0 item 0.8).
- **Scope:** `drafts/rt.md` §W4-RT-X86 crate part (`bench-env.json` fields, `EnvRecord::smoke` gains virt and governor) and §W4-RT-PUBLISH verdict code with the separate `report --sessions` aggregator.
- **Acceptance:** `drafts/rt.md` §W4-RT-X86 acceptance (banner unit test over four fake environments, dry run on arm64) and §W4-RT-PUBLISH acceptance (golden verdict test on fabricated rows).
- **Gates:** none. Merge before W4-RT-ROWS branches.
- **Hash impact:** none.

### W4-RT-WORLD. `sm-rt`: 2D-lattice z pitch, then CSG prototypes under `Lattice`/`Union`
- **Owner:** `crates/sm-rt/`.
- **Scope:** `drafts/rt.md` §W4-RT-LATZ, then §W4-RT-CSG-WORLD (sm-rt part only; the P2 `analytic-hierarchy` bench row is W4-RT-ROWS).
- **Acceptance:** §W4-RT-LATZ acceptance; §W4-RT-CSG-WORLD acceptance except the bench row (`pins_touched`, `structure_bytes` are checked in an sm-rt test on the P2 world).
- **Gates:** LATZ greps first for any pinned hash of a bridged 2D-lattice world; one found means stop (C3).
- **Hash impact:** none intended; `tests/flat_golden.rs` and the W3b `h_*` hashes stay equal.

### W4-HEX-STRAT. `sm-mesher`: `hex_only`, `tet`, per-volume mesh volume
- **Owner:** `crates/sm-mesher/`. HYGIENE-MESHER rides along.
- **Scope:** `drafts/hexpin.md` §W4-HEX-STRAT. Plan delta: `hex_only` lists `[map, submap, pave]` / `[map, sweep]` here; `ogrid` joins its list in W4-MESHER-LISTS after OGRID-S merges (an auto list that names an unregistered scheme fails, `standard.rs:240-241`).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-STRAT acceptance 1-6 (acceptance 2: `hex_only` equals `default` everywhere until LISTS).
- **Hash impact:** none on state; `mesh.info` gains `volume_by_owner`; the MCP golden diff, if any, is reviewed by the orchestrator.

### W4-HEX-DECOMP. `sm-decomp` (new): cut planner for round cavities
- **Owner:** `crates/sm-decomp/`.
- **Scope:** `drafts/hexpin.md` §W4-HEX-DECOMP (step 0 re-runs the spike at `d2f4803`; ordered `CutStep` list with `{step, role}` references; cylinder plus a plane at every cavity end face).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-DECOMP acceptance 1-5.
- **Hash impact:** none.

### W4-HEX-OGRID-K. `sm-scheme-ogrid` (new): pure O-grid and polar kernels
- **Owner:** `crates/sm-scheme-ogrid/`.
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-OGRID-K, acceptance 1-8 (both `ogrid` and `polar` modes; `collar` deferred).
- **Hash impact:** none.

### W4-CSG-CLASSIFY. `sm-csgbrep`: OpenMC quadric to parametric surface
- **Owner:** `crates/sm-csgbrep/src/quadric.rs` and its tests.
- **Scope / acceptance:** `drafts/csgbrep.md` §QUADRIC-CLASSIFY.
- **Hash impact:** none.

### W4-CSG-PROBE. `sm-boolean` (one new test file): what the kernel cuts today
- **Owner:** `crates/sm-boolean/tests/webcut_probe.rs`, plus a short report in the commit message.
- **Scope / acceptance:** `drafts/csgbrep.md` §WEBCUT-PROBE items 1-4. Its item 4 result picks the D2 partition or the per-cell boolean fallback for CSG-CORE.
- **Hash impact:** none.

### W4-CSG-LIBFN (only if W4-0 item 0.4 asks for it). `sm-model` + `sm-fem`
- **Scope:** the public library functions named in 0.4 (copy with pitch and count; block and sideset add), each wrapping today's op body so the op and the function share one code path.
- **Acceptance:** the ops' outputs and every test in both crates are unchanged; the new functions have rustdoc and one test each.
- **Hash impact:** none (the op bodies call the same code).

### W4-CLI-DIAG. `sm-cli`: per-op diagnostics in `smesh run` text mode
- **Owner:** `crates/sm-cli/`. HYGIENE-CLI rides along.
- **Scope / acceptance:** `drafts/export.md` §CLI-DIAG and §HYGIENE-* HYGIENE-CLI.
- **Hash impact:** none (render and tests only).

### W4-EXO-VERIFY. `sm-exodus`: does a real reader take our NFACED?
- **Owner:** `crates/sm-exodus/`.
- **Scope / acceptance:** `drafts/export.md` §EXPORT-EXO-VERIFY (libMesh `meshtool-opt` from the moose env first; SEACAS if D2 says it exists).
- **Hash impact:** none. The result feeds D2's export order.

### W4-DAGMC-MODEL (if D7(b)). `sm-server` + `sm-meshio/src/dagmc`: DAGMC h5m from the B-rep
- **Owner:** `crates/sm-server/`, `crates/sm-meshio/src/dagmc/`. HYGIENE-SERVER rides along.
- **Scope / acceptance:** `drafts/export.md` §EXPORT-DAGMC-MODEL (structural equality with MOAB's `facets_to_h5m` output on the bench's pin twin facets is the strongest check).
- **Gates:** after ACR-DAGMC-SRC.
- **Hash impact:** new path; existing `model_from_mesh` bytes unchanged; new hashes per os-arch.

#### Step a2 (each after the a1 tasks it names)

### W4-POLY-VORO. `sm-poly`: circumcentric (Voronoi) dual, strict and opt-in
- **Owner:** `crates/sm-poly/`.
- **Scope:** `drafts/poly.md` §W4-POLY-VORO. Plan delta: the task also checks whether the `.smsh` export fills the geometry cache `gc_*` for poly meshes and fills it if not, and stores the generator point as a cell tag (`drafts/poly.md` "`.smsh` for the transport code"), subject to D2.
- **Acceptance:** `drafts/poly.md` §W4-POLY-VORO acceptance 1-8.
- **Gates:** after MEASURE merges and the orchestrator has read its table (kill criterion). The eligibility rates of real meshes decide the follow-up (VORO-CLIP or CONFORM, §8) before VORO merges.
- **Hash impact:** none for existing hashes (`dual` omitted when default); new pins by INT.

### W4-HEX-QUERY. `sm-mesher`: `mesh.decompose_plan`
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-QUERY. **Gates:** after STRAT and DECOMP merge. **Hash impact:** none.

### W4-HEX-OGRID-S. `sm-scheme-ogrid`: scheme adapter
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-OGRID-S acceptance 1-4 (run with an explicit `mesh.scheme`, since no list names `ogrid` yet). **Gates:** after OGRID-K. **Hash impact:** none until a list names it.

### W4-CSG-CORE. `sm-csgbrep`: `model.import_openmc`, partition builder
- **Owner:** `crates/sm-csgbrep/`.
- **Scope:** `drafts/csgbrep.md` §CSGBREP-CORE on the plane-and-cylinder path (D1 (a), D2 or the PROBE fallback, D4 rectangular lattices, D6 blocks and sidesets).
- **Acceptance:** `drafts/csgbrep.md` §CSGBREP-CORE acceptance (all six bullets; sphere, cone and torus decks return typed `Unsupported` until W4-BOOL-TOOLS lands).
- **Gates:** after CLASSIFY, PROBE and W4-0 item 0.4 (and LIBFN if scheduled).
- **Hash impact:** new op, new hashes only.

### W4-RT-ROWS. `sm-rt-bench`: jump row, polyhedral row, conforming P1 twin, P2 hierarchy row
- **Owner:** `crates/sm-rt-bench/`.
- **Scope:** `drafts/rt.md` §W4-RT-JUMP-ROW, §W4-RT-POLY-ROW (L2b, via the `sm-poly` dependency ruled in 0.6), §W4-RT-TETS scope (c) (P1 only, generated with `smesh` and SHA-pinned), and the P2 `analytic-hierarchy` row of §W4-RT-CSG-WORLD.
- **Acceptance:** those sections' acceptance lists; the polyhedral rows labelled "different mesh, same geometry" and kept out of the k-of-n verdict (D6(b)).
- **Gates:** after RT-XDG, RT-VERDICT and RT-WORLD merge.
- **Hash impact:** new case files and `expected_sha256.txt` lines only.

### W4-RT-CELLINDEX. `sm-rt`: disjoint kd tiling for analytic leaves
- **Owner:** `crates/sm-rt/`.
- **Scope / acceptance:** `drafts/rt.md` §W4-RT-CELLBVH (renamed W4-RT-CELLINDEX), acceptance 1-6. This also covers `drafts/csgbrep.md` §CSGBREP-TRACK-PERF, which is dropped as a duplicate.
- **Gates:** after RT-WORLD (same crate). The trigger already fires on the P2 `analytic` row (`drafts/rt.md` finding 7).
- **Hash impact:** none; any move of `flat_golden` or W3b `h_*` stops the task.

#### Step a3

### W4-MESHER-LISTS. `sm-mesher`: `ogrid` into the strategy lists, VOLFIT
- **Owner:** `crates/sm-mesher/`.
- **Scope:** (1) `ogrid` into `hex_only`'s surface list (`drafts/hexpin.md` D3). (2) If D5 is yes: `ogrid` into `default`, as **one** commit that re-blesses exactly the R1 list of `drafts/hexpin.md` (`tools/platform/expected.txt:54-63`, `milestone_w3c_pin.rs:342-343`, `auto_select.rs:296`, the `w3c_pin_leftout` comparison), with the census proving that nothing else moved; the orchestrator updates `expected.txt`. (3) If D4 picks (b): `drafts/hexpin.md` §W4-HEX-VOLFIT (plan delta: it tests on the W3c through pin and does not wait for PIN-INT).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-STRAT acceptance 2 restated (`hex_only` = `default` except disk/annulus faces); §W4-HEX-VOLFIT acceptance if scheduled; the census.
- **Gates:** after OGRID-S and QUERY merge.
- **Hash impact:** none, unless D5 is yes (then exactly the R1 list, signed off by the user).

### W4-HEX-PIN-INT. tests and scripts: the W4 pin milestone
- **Owner:** `crates/sm-cli/tests/milestone_w4_pin.rs`, `tools/w4_pin_ogrid.smesh`, `tools/w4_pin_enclosed_hex.smesh` (explicit permission for these new `tools/` files).
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-PIN-INT acceptance 1-5 (fuel volume exact against the polygon, or `pi r^2 h` with VOLFIT). Pinning in `expected.txt` is done by INT.
- **Gates:** after MESHER-LISTS.
- **Hash impact:** new files only.

### W4-CSG-CHECK, then W4-CSG-ORACLE. `sm-csgbrep` (+ OpenMC fixtures)
- **Owner:** `crates/sm-csgbrep/`; ORACLE also `crates/sm-csgio/tests/fixtures/openmc/` (explicit permission; HYGIENE-CSGIO rides along).
- **Scope / acceptance:** `drafts/csgbrep.md` §CSGBREP-CHECK (sphere-lens row waits for W4-BOOL-TOOLS; the planar rows run now) and §CSGBREP-ORACLE. Plan delta: ORACLE's polyhedral row meshes the deck with `mesh.auto {strategy: "tet"}` (STRAT, a1), because `mesh.to_poly` refuses the all-hex through pin (`prep.rs:237-238`); with POLY-MIXED merged it also runs on the hex mesh.
- **Gates:** after CORE.
- **Hash impact:** new fixtures only; existing `oracle.json` files unchanged.

### W4-POLY-MIXED (if D1 accepts mixed cells). `sm-poly`: `mesh.to_poly` on mixed meshes
- **Owner:** `crates/sm-poly/`.
- **Scope:** `drafts/meshops.md` X-1: dualize the tet region, keep non-tet cells as polyhedra, cut each interface triangle into its dual sub-faces. Plan delta: this supersedes `drafts/poly.md`'s placement of POLY-MIXED in W5+; the transition rule needed is exactly this sub-face cut, not a general conformize.
- **Acceptance (plan):** (1) a W3c B6 `{Tet, Wedge}` mesh and a hex pin next to a tet moderator dualize; every cell passes `validate_geometry`; the volume sum equals the source to 1e-12; interface faces are shared. (2) An all-tet input gives today's output byte for byte (state hash pinned before the change). (3) Determinism.
- **Gates:** after VORO (same crate).
- **Hash impact:** none on existing hashes (all-tet path unchanged).

### 4.2 W4b: front door, hierarchy, interval matching

W4-0b (ACR-LEVELMAP, ACR-V1, ACR-V3) lands first.

### W4-BOOL-TOOLS (b1), then W4-BOOL-HEAL-A (b2). `sm-boolean`
- **BOOL-TOOLS scope / acceptance:** `drafts/csgbrep.md` §WEBCUT-TOOLS (size from the PROBE report). Unblocks the sphere/cone/torus decks of CSG-CORE and the lens row of CSG-CHECK (a small follow-up in `sm-csgbrep` by the CSG worker).
- **BOOL-HEAL-A scope / acceptance:** `drafts/vtopo.md` §T0 REGULARIZE-OP and §T3 VTOPO-DETECT (`model.heal_check`), acceptance 1-6. Thresholds are tuned on the D8 corpus if it exists.
- **Hash impact:** none on existing hashes.

### W4-VTOPO-OPS. `sm-model`: virtual-topology ops on the layer's overlay
- **Scope / acceptance:** `drafts/vtopo.md` §T1 VTOPO-OPS, acceptance 1-6. **Gates:** after ACR-V1, ACR-V3. **Hash impact:** none on existing.

### W4-STL-IN. `sm-cadio`: STL reader, writer, faceted body
- **Scope / acceptance:** `drafts/vtopo.md` §T7 STL-IN, acceptance 1-6 (if D7(a) is yes). **Hash impact:** none on existing.

### W4-MOD-REFINE-U. `sm-modify`: uniform `mesh.refine` with `LevelMap`
- **Scope / acceptance:** `drafts/meshops.md` §MOD-REFINE-U, acceptance 1-7. It builds the D1 pass core (`drafts/meshops.md` ordering delta). **Gates:** after ACR-LEVELMAP. The `.smsh` level-map passthrough in `sm-fem` (`drafts/meshops.md` §MOD-0 follow-up) is done by the W4-EXPORT-SELECT worker, who owns `sm-fem` in W4b. **Hash impact:** none on existing.

### W4-POLY-AGG (if D3 is yes). `sm-poly`: `mesh.agglomerate` with `LevelMap`
- **Scope / acceptance:** `drafts/poly.md` §W4-POLY-AGG. Plan delta: it includes the CMFD coarse level of `drafts/meshops.md` Q7 (one coarse cell per pin or per model volume, by region and a target count) as a mode. **Gates:** after ACR-LEVELMAP and the last a3 `sm-poly` task. **Hash impact:** new op only.

### W4-POLY-SCHEME, then W4-SIZE-IIA-OP. `sm-mesher`
- **POLY-SCHEME:** `drafts/poly.md` §W4-POLY-SCHEME (`mesh.scheme polyhedron`, with `dual`).
- **SIZE-IIA-OP:** the relative-interval op of `drafts/export.md` §SIZE-INTERVALS (`mesh.interval_relation`).
- **Hash impact:** new scheme and op only.

### W4-SIZE-IIA. `sm-sizing` + `sm-scheme-curve`: preference handling
- **Scope / acceptance:** `drafts/export.md` §SIZE-INTERVALS, the solver and curve-scheme parts. **Gates:** census first; the curve-scheme switch from `fix(.., Soft)` to `prefer` is the step most likely to move a count, and any move stops it (C3). **Hash impact:** none intended.

### W4-EXPORT-SELECT. `sm-meshio` + `sm-fem` + `sm-server`: one `select` object for all exporters
- **Scope / acceptance:** `drafts/export.md` §EXPORT-SELECT, plus the `mesh.smsh` level-map passthrough (`drafts/meshops.md` §MOD-0 follow-up; acceptance: a refined mesh exported with `.smsh` reads back with its level map). **Hash impact:** none (absent `select` reproduces current bytes).

### W4-CSG-CLI. `sm-cli` + `sm-mcp` + `sm-py`
- **Scope / acceptance:** `drafts/csgbrep.md` §CSGBREP-CLI. HYGIENE-PY rides along. **Gates:** after CSG-CHECK. **Hash impact:** none.

### 4.3 W4c: last sub-wave (a task not started when W4-INT begins moves to W5 without replanning)

W4-0c (ACR-V2a, ACR-EXPFMT) lands first.

- **W4-SIZE-CURV** (`sm-sizing`, `sm-mesher`, one line of `standard.rs:184`): `drafts/export.md` §SIZE-CURV. Then the `sm-mesher` worker does **W4-VTOPO-MESH**: `drafts/vtopo.md` §T2 VTOPO-MESH (planar composites; after T1 and ACR-V2a).
- **W4-EXPORT-FLUENT, then W4-EXPORT-CGNS** (`sm-meshio`, wiring in `sm-server`/`sm-cli`): `drafts/export.md` §EXPORT-FLUENT, §EXPORT-CGNS (CGNS starts with the two-cell HDF5 spike). These also cover `drafts/poly.md` §W4-POLY-FLUENT, dropped as a duplicate; "done" is judged by the OpenFOAM and CGNS-library oracles, and the Fluent licence stays a stated limit. Order may flip by D2.
- **W4-STL-WIRE** (`sm-server`, `sm-cli`): `session.export format: "stl"` (`drafts/vtopo.md` T7b), after EXPORT-SELECT.
- **W4-HEX-MSWEEP** (`sm-scheme-sweep`): `drafts/hexpin.md` §W4-HEX-MSWEEP acceptance 1, 2, 4 (many-to-many; true many-to-one is W5).
- **W4-HEX-PYR** (`sm-scheme-tet`): `drafts/hexpin.md` §W4-HEX-PYR, half-day spike first (R3); if the spike needs a `GenCtx` ACR, the task returns `blocked_on_api`. Priority rises if D1 says the transport code takes pyramids.
- **W4-MOD-HTET, then W4-MOD-REFINE-L** (`sm-modify`): `drafts/meshops.md` §MOD-HTET (with MOD-CONV2's THex/TQuad if time remains) and §MOD-REFINE-L.
- **W4-HEAL-TOL, then W4-STEP-REPAIR**: `drafts/vtopo.md` §T5 HEAL-TOL (`sm-boolean`), then §T5b STEP-REPAIR (`sm-cadio`). Moves to W4b if D8's corpus shows open shells block the user's STEP files.
- **W4-EXPORT-ABAQUS** (`sm-meshio`, last): `drafts/export.md` §EXPORT-ABAQUS.

---

## 5. W4-INT. Integration and milestone

Integration agent after the W4c merges (tasks not merged are listed as carried). Owner micro tasks for any crate edit.
1. **Polyhedral milestone:** `tools/w4_poly.smesh` and `crates/sm-cli/tests/milestone_w4_poly.rs`. The OpenMC `pin` deck (fixture bytes, content-addressed) → `model.import_openmc` → blocks per C9 → `mesh.intervals` on the circles → `mesh.auto {strategy: "tet"}` → `mesh.run` → `mesh.to_poly {dual: "circumcentric"}` if the eligibility report says the mesh qualifies, else barycentric (the test asserts which, from the report) → `session.export` `.smsh` (with `gc_*` per D2) and `.smrt`. Checks: per-block volumes equal the inscribed-polygon closed form to 1e-9 (`drafts/csgbrep.md` D7 layer 2); `.smrt` regions equal the block names with no warning; membership at the 600 oracle points equals OpenMC; `mesh.validate` 0 violations; CLI, MCP, in-process and Python hashes equal; two runs and `RAYON_NUM_THREADS=1` equal; replay without the strategy.
2. **Pin milestone:** PIN-INT's tests run in the combine; both milestone hash sets pinned in `tools/platform/expected.txt` for macOS arm64 and Linux CI (C12).
3. **Benchmark smoke** on this Mac under `SMOKE RESULT`: P1 and H1 `reduced`, trackers `smrt,dagmc-moab,dagmc-dd,xdg`, plus `walk`, `jump`, polyhedral and P2 `analytic-hierarchy` rows; image identities recorded; nothing under `results/` committed; no ratios. If D6's machine exists, the x86_64 script's first session runs and its table is attached to the report, verdicts computed by the pre-registered code.
4. **MEASURE table and eligibility rates** in the report, with the orchestrator's decision on VORO-CLIP / CONFORM and on `CONVEX_CHECK_MAX`.
5. **Census:** every hash in `census-base.txt` compared; any difference not covered by a D5 sign-off stops integration and goes to the user.
6. **Goldens:** only catalog goldens that list new ops/schemas, the protocol insertions of the W4-0 ACRs and new pins may change; each diff is listed in the report.
7. **CI:** GitHub CI green on the INT PR (gate and python job) as well as `tools/ci.sh` locally.
8. **PARITY.md:** tick each row only with a test name (polyhedron scheme, CSG to B-rep, multisweep many-to-many, THex/QTri/HTet if merged, refinement, export options, Fluent/CGNS if merged, STL import, virtual topology rows if merged, curvature sizing).
9. **Report** `docs/design/w4/w4-report.md`: deviations D1..Dn, census, platform table, benchmark smoke, the answers to §9 as applied, carried tasks with their triggers.

---

## 6. Dependencies

```
W4-0: 0.1 census base, 0.2 crate skeletons, 0.3 ACR-POLY2, 0.4 CSGBRIEF-0, 0.5 ACR-DAGMC-SRC (D7b), 0.6 rulings, 0.7 doc fixes, 0.8 bench script
 ├─ a1: POLY-MEASURE ∥ POLY-VOLDIAG ∥ RT-XDG ∥ RT-VERDICT ∥ RT-WORLD ∥ HEX-STRAT ∥ HEX-DECOMP ∥ HEX-OGRID-K
 │      ∥ CSG-CLASSIFY ∥ CSG-PROBE ∥ CSG-LIBFN (if 0.4) ∥ CLI-DIAG ∥ EXO-VERIFY ∥ DAGMC-MODEL (D7b)
 ├─ a2: POLY-VORO (MEASURE + table review) ; HEX-QUERY (STRAT, DECOMP) ; HEX-OGRID-S (OGRID-K)
 │      CSG-CORE (CLASSIFY, PROBE, 0.4/LIBFN) ; RT-ROWS (RT-XDG, RT-VERDICT, RT-WORLD) ; RT-CELLINDEX (RT-WORLD)
 ├─ a3: MESHER-LISTS (OGRID-S, QUERY; D4, D5) → HEX-PIN-INT ; CSG-CHECK → CSG-ORACLE (CORE; STRAT) ; POLY-MIXED (VORO; D1)
 ├─ W4-0b: ACR-LEVELMAP, ACR-V1, ACR-V3
 ├─ b:  BOOL-TOOLS → BOOL-HEAL-A ; VTOPO-OPS (V1, V3) ; STL-IN (D7a) ; MOD-REFINE-U (LEVELMAP) ; POLY-AGG (LEVELMAP, a3 sm-poly; D3)
 │      POLY-SCHEME → SIZE-IIA-OP ; SIZE-IIA ; EXPORT-SELECT ; CSG-CLI (CSG-CHECK)
 ├─ W4-0c: ACR-V2a, ACR-EXPFMT
 ├─ c:  SIZE-CURV → VTOPO-MESH (VTOPO-OPS, V2a) ; EXPORT-FLUENT → EXPORT-CGNS → EXPORT-ABAQUS ; STL-WIRE (STL-IN, SELECT)
 │      HEX-MSWEEP ; HEX-PYR (spike) ; MOD-HTET → MOD-REFINE-L ; HEAL-TOL → STEP-REPAIR
 └─ W4-INT
```
Cross-area links that are easy to miss: CSG-ORACLE's polyhedral row needs the `tet` strategy (STRAT) or POLY-MIXED; RT-ROWS' polyhedral twin needs the `sm-poly` dependency ruling (0.6); the CSG sphere-lens row needs BOOL-TOOLS; POLY-AGG and MOD-REFINE-U share ACR-LEVELMAP; VTOPO-MESH needs ACR-V2a; `smtets_to_moab` lives only in RT-XDG (`drafts/export.md` defers to it).

---

## 7. Hash census and merge rules

- **C3 carries over.** Any move of an existing hash stops the task and goes to the user; nothing is re-blessed by a worker. The population is `docs/design/w4/census-base.txt` (W4-0 item 0.1): the 63 lines of `tools/platform/expected.txt` (W2, W3a, W3b, `w3c_pin`), the `milestone_w3c_pin` asserted counts (432 hexes, 733 tets, two `mesh.auto_tet` warnings), `SUBMAP_GRADED`, `p0_rows`, `crates/sm-rt/tests/flat_golden.rs` (per os-arch, C11), the existing lines of `crates/sm-rt-bench/expected_sha256.txt`, the export byte-identity test `each_format_is_byte_identical_in_process_cli_and_mcp` and the format goldens, the `tests/dagmc.rs` goldens, the sm-poly counts (375 at `crates/sm-poly/tests/acceptance.rs:93`) and `acc7_determinism`, the conformance fixtures `polydual` and `w3a_poly`, the sm-csgio `oracle.json` files and `content_hash` goldens.
- **Planned moves:** only D5 if the user says yes, as one signed commit (W4-MESHER-LISTS).
- **Allowed changes that are not hash moves:** catalog/schema goldens that gain entries for new ops, reviewed by the orchestrator at merge; protocol golden insertions of the W4-0 ACRs; new pins added by INT (macOS arm64 and Linux CI; facet and tessellation bytes per os-arch).
- **Opt-in rule.** Every geometry-changing feature in W4 is opt-in: new op arguments are omitted from canonical params when they hold the default (`drafts/poly.md` hash exposure), so a script that does not use them hashes as before.
- **Merge rule.** A sub-wave (a, b, c) and W4-INT merge only with GitHub CI green on the PR (gate and python job) as well as the local `tools/ci.sh` (W3c process note). A red CI is recorded with its cause, never reported as green.

---

## 8. What moves out of the scheduled sub-waves, and why

**Trigger-based (in W4 if the trigger fires before W4-INT, else W5):**
- `drafts/poly.md` §W4-POLY-VORO-CLIP and §POLY-CONFORM (`sm-scheme-tet`): trigger is MEASURE's eligibility rates; refusals dominated by outside circumcentres → VORO-CLIP; by non-Delaunay interior faces → CONFORM (scope approved under D10).
- `CONVEX_CHECK_MAX` raise in sm-rt (`drafts/poly.md` Priority): MEASURE shows many convex cells over 64 faces.
- `drafts/poly.md` §W4-POLY-SHAPES: VORO shows hex-lattice outputs matter (the `.smsh` rule change is then a format question for the user).
- `drafts/vtopo.md` §T2b, §T4, §T6, §T11, §T8a: the D8 corpus shows periodic composites, short edges, holes or blends dominate; T8a when the user brings DAGMC models (D7(c)).
- `drafts/csgbrep.md` §CSGBREP-HEX: D8 says decks use `hex_lattice`.
- `drafts/rt.md` real x86_64 table, three sessions and the `xl` tier: D6's machine exists. The local real-model table additionally needs sm-cadio pcurves for periodic faces.

**W5+ (not scheduled), with the reason:**
- POLY-RVD, POLY-POWER (`drafts/poly.md`): new constructions beyond W4's dual; VORO first.
- POLY-BULK (`drafts/poly.md`, an ACR): no 10^6-cell poly requirement yet.
- Sweep-order data for Sn (`drafts/poly.md`): the transport code has not asked.
- HEX-VSUBMAP, the general multisweep decomposer, sweep columns with intermediate vertices, true many-to-one sweep (union-copy ACR), `collar`, sphere O-grid (`drafts/hexpin.md`): pins and assemblies are sweepable after DECOMP; none is on the user's path.
- MOD-ELEVATE (`drafts/meshops.md`): no goal needs high order; the walker and the transport code take linear cells (D10).
- MOD-BL, MOD-PILLOW, MOD-COARSEN (`drafts/meshops.md`): no CFD workflow, no hex defect, coarsening covered by POLY-AGG.
- X-2 volume copy scheme (`drafts/meshops.md`, an ACR): no case is slow today; the 3x3 lattice meshes now.
- T9 STL-BREP, T10 IGES, T12 DEFEAT-BLEND, T8b foreign `.h5m` (`drafts/vtopo.md`): mesh-based geometry stays deferred (PARITY:91); IGES has no user file; blends need SSI maturity; T8b needs D7(c).
- Nastran and the P2 formats, proximity and skeleton sizing, periodic intervals (`drafts/export.md`): no reader or measured need.
- GPU smoothing and quality (`drafts/export.md` D5): bit-identical output cannot hold on Metal (no `f64`) and no bottleneck is measured (D10).
- Model instancing for lattices, edit-one-cell-and-regenerate, DAGMC universe import, periodic boundaries, ellipsoid/paraboloid faces as rational NURBS (`drafts/csgbrep.md`): each needs a contract or a W5 editing model.
- OpenMC plugin for end-to-end runs (`drafts/rt.md` Q4), XDG's GPRT backend, a libMesh backend: W-flight is the W4 transport result.

---

## 9. Open questions for the user

D1. **What does your transport code need from a cell?** Its discretization (FV diffusion, DG on polyhedra, characteristics/MOC, Monte Carlo with mesh tallies) decides whether convex cells, orthogonal faces and one face per neighbour pair are hard requirements. And which cells does it accept: only duals, or any polyhedron including hexes, wedges and pyramids in one mesh; and for pins, an all-quad O-grid or a polar rings-by-sectors section with a central n-gon prism? *Rec:* target planar convex cells (what ray tracing wants too) and accept any polyhedron; build both pin modes, `ogrid` in `hex_only`. *Changes:* if mixed meshes are accepted, W4-POLY-MIXED runs in a3 and HEX-PYR rises in W4c; if only duals, MIXED and PYR go to W5; if orthogonality is a hard requirement, VORO-CLIP/CONFORM become scheduled rather than trigger tasks.

D2. **Which file does the transport code read?** `.smsh` (then should every poly export fill the geometry cache `gc_*`?), Exodus, Fluent `.msh`, CGNS or OpenFOAM. Also: do you have SEACAS (`exodiff`, `epu`) or a Griffin build? *Rec:* `.smsh` with `gc_*` always filled for poly meshes (no format bump); Fluent then CGNS as interchange; Abaqus last. *Changes:* the W4c export order, and whether VORO fills `gc_*`; if the code reads Exodus, EXO-VERIFY's result decides whether NFACED needs work in W4.

D3. **Does the transport code use coarse-mesh acceleration (CMFD or multigrid)?** If so, it needs a nested coarse mesh (for example one coarse cell per pin or per volume) with a fine→coarse cell and face map. *Rec:* yes if CMFD is planned: ACR-LEVELMAP in W4-0b and W4-POLY-AGG in W4b with a per-pin mode; refinement levels are stored the same way, as linked independent meshes (in-place parent marking is refused by sm-rt, Gmsh and OpenFOAM today), and local refinement turns coarse neighbours into polyhedra rather than using transition templates. *Changes:* if no, POLY-AGG moves to W5; ACR-LEVELMAP still lands for MOD-REFINE-U.

D4. **Volume of round regions with linear cells.** At about 8 circle intervals the mesh under-fills a fuel disk by about 10% (2.6% at 16, 0.6% at 32), in the transport mesh and in the `.smrt` alike. Options: (a) accept it and refine the circle; (b) an opt-in op that moves circle nodes to the equal-area radius so each material volume is exact at any n (still conformal; nodes leave the CAD circle); (c) a volume-matched tessellation option in `sm-tess` for unstructured meshes; (d) wait for high-order elements. *Rec:* (b) now as opt-in (W4-MESHER-LISTS), VOLDIAG to report the deficit everywhere, (c) as a W5 task. *Changes:* VOLFIT in a3 and the pin milestone checks `pi r^2 h` exactly; with (a) it checks the polygon value only.

D5. **Does `ogrid` enter the `default` strategy?** If yes, the through pin, its lattice, B5's 154-hex assertion and the leftout fixture change on purpose (`tools/platform/expected.txt:54-63` and three tests), in one signed commit; W2/W3a/W3b hashes stay. *Rec:* yes; the pin is your own example and its mesh is what W4 improves. *Changes:* if no, only `hex_only` uses `ogrid` and `default` stays as in W3c.

D6. **Benchmark machine and the polyhedral row.** (a) Which bare-metal x86_64 machine, and who arranges it? *Rec:* rented bare metal, at least 16 physical cores, SMT off, 64 GB RAM, one day per session, three sessions; we deliver a one-command script. (b) Should the polyhedral row (our walk on the dual of the same tets; XDG cannot read polyhedra) appear in the "beats XDG" report? *Rec:* yes, as a separately labelled table, never in the k-of-n verdict. *Changes:* without (a) W4 publishes only arm64 smoke and correctness; the x86_64 table, three sessions and the `xl` tier move to W5.

D7. **Formats in and out beyond STEP.** (a) STL import and export (as a faceted B-rep) under the "no mesh import" decision of PARITY.md:91, with mesh-as-geometry still deferred? (b) DAGMC `.h5m` straight from the B-rep with no volume mesh, as `source: "model"` on `session.export`? (c) Do you have existing DAGMC `.h5m` models to run through `sm-rt`; if so, extend our pure-Rust HDF5 reader to MOAB's files or link `libhdf5`? (d) IGES? *Rec:* (a) yes; (b) yes, STEP to DAGMC without Cubit or MOAB is a direct surpass item; (c) only if you have models, pure Rust; (d) W5+. *Changes:* (a) STL-IN in W4b; (b) ACR-DAGMC-SRC and W4-DAGMC-MODEL in a1; (c) T8a/T8b scheduled; (d) T10 scheduled.

D8. **Your own models.** (a) Two or three dirty STEP (or STL) files as the `heal_check` corpus, with sha256 committed and files not? (b) Two real OpenMC decks (a pin cell and an assembly or core slice) for the CSG acceptance; do they use `hex_lattice`, and if so in which orientation (W3b Q3 allowed pointy-top only for sm-rt)? (c) May one local model appear in a separate "not reproducible" benchmark table later? *Rec:* yes to (a) and (b) now; (c) yes, after the first table. *Changes:* (a) sets T3 thresholds and the order of HEAL-TOL/STEP-REPAIR, T6 and T11; (b) adds your decks to CSG-ORACLE and, with hex lattices, schedules CSGBREP-HEX (with the sm-csgio IR ACR) in W4c.

D9. **Region granularity in `.smrt`.** Today all fuel pins form one region, "fuel" (one region per block, R9). Per-pin tallies or depletion need one region per lattice element (for example `fuel[3,5]`). *Rec:* yes if your code tallies per pin; it is a small sm-rt/sm-meshio change, and the CSG import keeps one volume per placed cell either way. *Changes:* adds a per-element region option to the `.smrt` export in W4b (owner sm-meshio/sm-rt, small); otherwise stays as in R9.

D10. **Defaults we will apply unless you object** (one answer covers all): GPU smoothing/quality not in W4 (trigger: a measured pass above a threshold you name, plus an approved GPU determinism rule); high-order elements and boundary layers in W5; the OpenMC end-to-end plugin in W5; the circumcentric dual stays opt-in through W4 and its default is decided at W4-INT from the numbers; `hex_only` fails with suggested cuts instead of cutting the model itself; the OpenMC import is one history node (like STEP import), revisited with W5 editing; true many-to-one sweep (a frozen-contract ACR) in W5; `sm-scheme-tet` may gain a "Delaunay output" option if MEASURE shows it is needed; a Fluent licence is not required for W4's Fluent writer (OpenFOAM is the oracle). *Rec:* confirm. *Changes:* any item you reject is re-planned before its sub-wave branches.

**Answered without a user question** (rulings, orchestrator calls, or no real alternative; recorded in the drafts' critique deltas or §3.1 item 0.6): overlap default `error` (W3b Q4); the import makes geometry, blocks and sidesets only; pre-registered win criteria and default-build headline; healing home, tolerance-first healing and virtual collapse; `renumber` dropped from export selection; sagitta-primary curvature sizing with angle as a cap; hygiene ownership by crate; mesh copy via a volume copy scheme rather than a post-hoc op (and in W5); `sm-rt-bench` dependencies (0.6); XDG pin bump (C15).
