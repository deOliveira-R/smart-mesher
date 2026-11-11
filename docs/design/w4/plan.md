# W4 plan: polyhedral meshes for the transport code, ray tracing that beats DAGMC/XDG, then parity

> **Rulings:** `rulings.md` (D1-D10, 2026-11-09) overrides this plan and the drafts where they differ. This revision (re-plan at `9cac369`) applies them; where a draft still says otherwise, the plan and the rulings win.

Base: `main` at `9cac369` (W3c-INT merged in f2e831d, report follow-ups 63ec23b, backlog d2f4803, W4 plan cecd3b1, rulings 9cac369; the code is identical to `d2f4803`, so the drafts' line numbers hold). Inputs, all committed next to this file: the area drafts `docs/design/w4/drafts/<key>.md` (`poly.md`, `rt.md`, `hexpin.md`, `vtopo.md`, `csgbrep.md`, `meshops.md`, `export.md`) and `drafts/regions.md` (D9, written at the re-plan). Each of the first seven drafts was corrected by an Opus critique, and every correction is marked **Critique delta** inside the draft, so a worker never needs the critique. The W3c plan (`docs/design/w3c/plan.md`) is the structural model.

**Goals (user).** (1) Polyhedral meshes for the user's own neutron-transport code, which reads `.smsh` first (D2). (2) Ray tracing (`sm-rt`, `.smrt`) that beats DAGMC and XDG. Standing principle: Cubit is a bar to surpass, not to match; we pick the best design we can find. MCNP is not a target. OpenMC is the CSG format and the oracle (conda env `/Users/rodrigo/opt/miniforge/envs/openmc`, OpenMC 0.16.0).

**Rulings:** `docs/design/w3b/rulings.md` and `docs/design/w3c/rulings.md` stay binding where §2 lists them. The answers to the plan's questions (D1-D10) are in `docs/design/w4/rulings.md`; §9 points there.

**Citation convention.** Paths are relative to `docs/design/w4/`. "`drafts/<key>.md` §T acceptance N" is item N of the **Acceptance** list under the task heading T in that draft (T is the draft's own name: `W4-POLY-VORO`, `T1 VTOPO-OPS`, `MOD-HTET`, `CSGBREP-CORE`, ...). Lettered items keep their letter (`§W4-POLY-MEASURE acceptance (b)`). "F3", "finding 7", "D2" refer to that draft's Findings and Design items; "ruling D2" is `rulings.md`. A cited item includes the **Critique delta** written next to it. Where this plan changes a cited item, the change is written next to the citation as "plan delta". `plan.md` wins over the drafts; the rulings win over both.

**Facts the plan relies on** (checked at `d2f4803` by the drafts' critiques, spot-checked here and at `9cac369`): `tools/platform/expected.txt` has 62 data lines plus 1 comment line (not 63) and no poly or sm-rt structure entry; `sm-rt-bench` depends on neither sm-poly nor sm-mesher (`crates/sm-rt-bench/Cargo.toml:18-27`); `standard_schemes` registers eight scheme crates (`crates/sm-server/src/standard.rs:25-32`) and fails if an auto list names an unregistered scheme (`standard.rs:240-241`); `mesh.to_poly` refuses any non-tet cell (`crates/sm-poly/src/prep.rs:237-238`); `ApplyCtx::apply_nested` is `pub(crate)` (`crates/sm-op-api/src/op.rs:619-648`); `MeshLayer` stores no level map (`crates/sm-layer-api/src/mesh.rs:16-41`); `ModelLayer` stores no overlay (`crates/sm-layer-api/src/model.rs:44-51`); the XDG pin `83efb306e152` is 5 commits behind upstream `a5d67e6` (`drafts/rt.md` finding 2). Added at the re-plan: the workspace members are the glob `crates/*` (root `Cargo.toml:3`); sm-rt refuses every hex lattice today (`crates/sm-rt/src/world.rs:985`, `crates/sm-rt-api/src/stub.rs:1352`), so no pinned hash involves one; `Request::Export` is destructured field by field in `crates/sm-server/src/server.rs:254` and built by a struct literal in `crates/sm-cli/src/lib.rs:637`; `.smrt` images already carry region tags when present (`crates/sm-rt/src/image.rs:20`, `:201-212`) and the C ABI reads them (`crates/sm-rt-capi/include/smrt.h:252-256`).

**Milestone (§5).** An OpenMC pin-cell deck becomes B-rep volumes in one op, is meshed all-tet and dualized to a convex circumcentric polyhedral mesh where the strict gate allows (barycentric otherwise), and is exported to `.smsh` (with the geometry cache `gc_*`, ruling D2) and `.smrt`; the chain is checked against OpenMC's own cell lookups and against closed-form polygon volumes. The round pin is all-hex in both forms (through and enclosed) with an O-grid, and with VOLFIT its fuel volume is `pi r^2 h` exactly (ruling D4). A subdivided lattice exports one region per lattice position x ring x sector to `.smsh` and `.smrt` (ruling D9). The benchmark has XDG rows (fire, tet walk), a region-jump row and a polyhedral row (a separately labelled table, ruling D6), a pre-registered verdict in code, and a smoke run on arm64; the x86_64 script is delivered after the W4a ray-tracing rows and the cell index merge, and the user runs it (ruling D6).

---

## 1. Ownership

One worker per row. Rows that share a crate run one after the other, in the order given in §4. "Files" means file-level ownership inside a crate (W3c precedent: CYL-PIN owned rows of one test file).

| Task | Crate / dir | Sub-wave | Size | Owns |
|---|---|---|---|---|
| W4-0.1 .. W4-0.7 | orchestrator workers | 0 | S-M each | §3.3 (disjoint paths per task) |
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
| W4-REGIONS-CUT | `sm-boolean` (files) | a1 | S/M | `crates/sm-boolean/src/subdivide.rs`, `tests/subdivide.rs` (new), the op registration line in `src/lib.rs` |
| W4-CSG-LIBFN (dropped: csgbrief.md §6; fallback signatures in §3.4) | `sm-model`, `sm-fem` | a1 | S | library entry points named by W4-0.4 only |
| W4-CLI-DIAG | `sm-cli` | a1 | S | `crates/sm-cli/` (src and existing tests) |
| W4-EXO-VERIFY | `sm-exodus` | a1 | S | `crates/sm-exodus/` (tests and docs unless a bug is found) |
| W4-DAGMC-MODEL | `sm-server`, `sm-meshio/src/dagmc` | a1 | M/L | `crates/sm-server/`, `crates/sm-meshio/src/dagmc/` |
| W4-REFCORPUS | tools + doc (new) | a1 | M | `tools/refcorpus/` (new), `docs/design/w4/refcorpus.md` (new) (explicit permission) |
| W4-A1-TIDY | tools + `sm-exodus` + `sm-decomp` | a2 | S | `tools/refcorpus/`, `crates/sm-exodus/`, `crates/sm-decomp/`, `tools/ci.sh` (refcorpus checks only, explicit) |
| W4-POLY-VORO | `sm-poly` | a2 | L | `crates/sm-poly/` (and the `.smsh` `gc_*` fill in `crates/sm-meshio/src/smsh/` if missing, explicit permission); ratified at a2 combine: also `crates/sm-quality/src/convexity.rs` (face planes from stored loop order plus a side sign, matching sm-rt's arithmetic; census unmoved) and `crates/sm-meshio/tests/common/mod.rs` |
| W4-HEX-QUERY | `sm-mesher` | a2 | S | `crates/sm-mesher/` |
| W4-HEX-OGRID-S | `sm-scheme-ogrid` | a2 | M | `crates/sm-scheme-ogrid/` |
| W4-CSG-CORE | `sm-csgbrep` | a2 | L | `crates/sm-csgbrep/` |
| W4-RT-ROWS | `sm-rt-bench` | a2 | M | `crates/sm-rt-bench/` |
| W4-RT-CELLINDEX | `sm-rt` | a2 | L | `crates/sm-rt/` |
| W4-REGIONS-SPLIT | `sm-model`, `sm-fem`, `sm-server` (files) | a2 | M/L | `crates/sm-model/` (new `src/region.rs`, the `lattice` key in `src/copy.rs`), `crates/sm-fem/src/lower.rs`, new `crates/sm-fem/src/region.rs`, `crates/sm-server/src/mesh_export.rs` (ratified at a2 combine: also `crates/sm-server/src/host.rs`, so `smsh` takes `options`), one section of `crates/sm-meshio/SMSH_FORMAT.md` (explicit) |
| W4-HEX-PIN-INT | tests + tools (new files) | a3 | M | `crates/sm-cli/tests/milestone_w4_pin.rs`, `tools/w4_pin_ogrid.smesh`, `tools/w4_pin_enclosed_hex.smesh` (explicit permission) |
| W4-MESHER-LISTS | `sm-mesher` | a3 | S/M | `crates/sm-mesher/`; the D5 re-bless lines of `crates/sm-cli/tests/milestone_w3c_pin.rs` (explicit, in the one D5 commit) |
| W4-CSG-CHECK, then W4-CSG-ORACLE | `sm-csgbrep` (+ fixtures) | a3 | M + M | `crates/sm-csgbrep/`; ORACLE also `crates/sm-csgio/tests/fixtures/openmc/` (explicit permission) |
| W4-POLY-MIXED | `sm-poly` | a3 | M | `crates/sm-poly/` |
| W4-A2-RT | `sm-rt` | a3 | S/M | `crates/sm-rt/` |
| W4-A2-RTBENCH | `sm-rt-bench` | a3 | S | `crates/sm-rt-bench/` |
| W4-BENCH-X86 | orchestrator worker | after a2 | S/M | `tools/bench/` (new), `docs/design/w4/bench-x86.md` (new) |
| W4-BOOL-TOOLS | `sm-boolean` | b1 | M/L | `crates/sm-boolean/` |
| W4-BOOL-HEAL-A | `sm-boolean` | b2 | S + M | `crates/sm-boolean/` |
| W4-VTOPO-OPS | `sm-model` | b | M | `crates/sm-model/` |
| W4-MOD-REFINE-U | `sm-modify` (new) | b | M/L | `crates/sm-modify/` |
| W4-POLY-AGG | `sm-poly` | b | M | `crates/sm-poly/` |
| W4-POLY-SCHEME, then W4-SIZE-IIA-OP | `sm-mesher` | b | S/M + S | `crates/sm-mesher/` |
| W4-SIZE-IIA | `sm-sizing`, `sm-scheme-curve` | b | S + S | `crates/sm-sizing/`, `crates/sm-scheme-curve/` |
| W4-EXPORT-SELECT | `sm-meshio`, `sm-fem`, `sm-server` | b | M | `crates/sm-meshio/` (writers' filter), `crates/sm-fem/`, `crates/sm-server/` |
| W4-CSG-CLI | `sm-cli`, `sm-mcp`, `sm-py` | b | S | `crates/sm-cli/`, `crates/sm-mcp/`, `crates/sm-py/` |
| W4-RT-HEX | `sm-rt` | b | M | `crates/sm-rt/` |
| W4-SIZE-CURV | `sm-sizing`, `sm-mesher` | c | M/L | `crates/sm-sizing/`, `crates/sm-mesher/`, one line of `standard.rs:184` (explicit) |
| W4-VTOPO-MESH | `sm-mesher`, `sm-scheme-map`, `sm-scheme-pave` | c | M | those crates |
| W4-EXPORT-FLUENT, then W4-EXPORT-CGNS | `sm-meshio` (+ wiring) | c | M + L | `crates/sm-meshio/`; wiring lines in `sm-server`, `sm-cli` |
| W4-CSG-HEX | `sm-csgio`, then `sm-csgbrep` | c | M/L | `crates/sm-csgio/` (reader and the IR under ACR-CSG-HEXIR), then `crates/sm-csgbrep/` |
| W4-HEX-MSWEEP | `sm-scheme-sweep` | c | L | `crates/sm-scheme-sweep/` |
| W4-HEX-PYR | `sm-scheme-tet` | c | L | `crates/sm-scheme-tet/` |
| W4-MOD-HTET, then W4-MOD-REFINE-L | `sm-modify` | c | M + L | `crates/sm-modify/` |
| W4-HEAL-TOL, then W4-STEP-REPAIR | `sm-boolean`, then `sm-cadio` | c | M + S/M | `crates/sm-boolean/`, then `crates/sm-cadio/` |
| W4-EXPORT-ABAQUS (last) | `sm-meshio` | c | M | `crates/sm-meshio/src/abaqus/` |
| W4-INT | integration agent | INT | M | §5 |

Rules for every task (CLAUDE.md applies): own only the listed paths; frozen contracts (`*-api`, `sm-testkit`, `sm-protocol`, the sm-csgio core per `crates/sm-csgio/DESIGN.md:3-4`) change only through a W4-0 ACR, else `status: blocked_on_api` with an `api_change_request`; workers do not edit `tools/` (named exceptions only), docs outside their crate, or the INT test files; determinism test (two runs, `RAYON_NUM_THREADS=1` vs default); property tests with `sm_testkit::strategies::config(n)`; rustdoc on every public item; no wall-clock gates (work counters only); no `unwrap` on input paths; no `HashMap` order in output. Hash rule: §7.

Hygiene ride-alongs (`drafts/export.md` §HYGIENE-*): HYGIENE-CLI goes with W4-CLI-DIAG, HYGIENE-RTBENCH with W4-RT-XDG, HYGIENE-CSGIO with W4-CSG-ORACLE, HYGIENE-MESHER with W4-HEX-STRAT, HYGIENE-SERVER with W4-DAGMC-MODEL (ruling D7(b)), HYGIENE-PY with W4-CSG-CLI, HYGIENE-RTCAPI and HYGIENE-TOOLS in W4-0.7.

Same-crate order in W4a that the table implies: `sm-boolean` PROBE and REGIONS-CUT own disjoint files (both a1); `sm-model`/`sm-fem` LIBFN (a1, if scheduled) before REGIONS-SPLIT (a2); `sm-server` DAGMC-MODEL (a1) before REGIONS-SPLIT (a2); `sm-rt` RT-WORLD (a1) before RT-CELLINDEX (a2) before RT-HEX (b).

---

## 2. Standing rulings that still bind

| Id | Ruling | Consequence in W4 |
|---|---|---|
| W3b Q2 | Cross sections never enter smart-mesh or sm-rt; regions are material names. | CSG import carries material names only (`CsgIr::dropped`). Region names under D9 start with the material (block) name. |
| W3b Q3 | Hex lattices in sm-rt: stretch, pointy-top only. | **Overridden by ruling D8:** sm-rt gains both orientations (ACR-RT-HEXY in W4-0.3, W4-RT-HEX in W4b), and W4-CSG-HEX (W4c) confirms both against OpenMC. |
| W3b Q4 | CSG cells must be disjoint. | CSG-CHECK default overlap policy is `error` (`drafts/csgbrep.md` D5). |
| W3b Q5 | `mesh.auto` records explicit assignments; replay never re-runs a strategy; explicit-scheme runs stay bit-identical. | New strategies (`hex_only`, `tet`) and lists cannot change old histories; decomposition is explicit ops (`drafts/hexpin.md` D2). |
| W3b Q9 | The published benchmark comes from one bare-metal Linux x86_64 machine arranged by the user; macOS is smoke only. | Ruling D6: the user's work machine; script W4-BENCH-X86. |
| W3b R1 | Region-jump tolerance: jump vs walk agreement by the exit-normal rule. | RT-ROWS jump row acceptance. |
| W3b R9, R10 | `.smrt`: one region per material block; volumes in no block fall back with a warning. | Still the default. Ruling D9 adds the opt-in split by lattice position, ring and sector (W4-REGIONS-*); a split request never falls back silently (`regions-need-blocks`). |
| W3c standing principle | Cubit is a bar to surpass; legacy conventions are not references. | Applies to every design choice here. |
| W3c C1 | Enclosed pin all-tet for now; O-grid all-hex and hex-tet transitions in W4. | HEX-DECOMP/QUERY/OGRID/PIN-INT (a1-a3), HEX-PYR (c). |
| W3c C3 | A moved existing hash stops the task and goes to the user. | §7. The only planned move is ruling D5. |
| W3c C5 | `hex_only` and `tet` wait for W4. | W4-HEX-STRAT (a1). |
| W3c C6 | OpenMC only; MCNP is not a target. | No MCNP anywhere; Abaqus is not justified by MCNP6 UM (`drafts/export.md` D1). |
| W3c C7 | OpenMC itself is the oracle. | CSG-ORACLE; VOLFIT ray-sampled volume check; both hex orientations (RT-HEX, CSG-HEX). |
| W3c C8 | B-rep import of a CSG model goes to W4. | W4-CSG-* tasks. |
| W3c C9 | Region names: material `name` if non-empty and unique, else `material_<id>`; `void`; override map. | CSG-CORE blocks; the D9 grammar starts with these names. |
| W3c C11, C12 | `flat_golden` bundle hashes per os-arch; milestone identity hashes pinned in `tools/platform/expected.txt`, enforced on Linux CI. | New milestone hashes pinned by INT; facet/tess bytes per os-arch. |
| W3c C13 | apt versions recorded, not frozen. | XDG image recipe. |
| W3c C15 | XDG: clone only `xdg-org/xdg`; pin the newest upstream commit that builds and runs. | W4-RT-XDG step 0. |
| W3c process note | A wave merges only with GitHub CI green on its PR as well as the local run. | §7, applied to every sub-wave merge (restated in `rulings.md` Process). |

---

## 3. W4-0 (orchestrator)

### 3.1 Order (before W4a workers branch)

W4-0 runs as seven tasks for workers acting for the orchestrator (§3.3). Their owner paths are disjoint, so all seven run in parallel from `9cac369`; the orchestrator merges them in any order, runs the census (W4-0.1's script) after the last merge, and only then lets W4a branch. W4-0.1 is the census baseline; no other W4-0 task depends on it, and each keeps every existing hash equal by its own acceptance.

| Key | What | Plan item it replaces |
|---|---|---|
| W4-0.1 | Census baseline script and `census-base.txt` | 0.1 |
| W4-0.2 | New crate skeletons and registration | 0.2 |
| W4-0.3 | `sm-rt-api` ACRs: ACR-POLY2, ACR-RT-HEXY, ACR-RT-REGIONSETS, the 2D-lattice doc fix | 0.3 and the sm-rt-api doc-only item |
| W4-0.4 | CSGBRIEF-0 decisions | 0.4 |
| W4-0.5 | `sm-protocol` ACR-DAGMC-SRC and the protocol doc fixes | 0.5 and the sm-protocol doc-only items |
| W4-0.6 | `sm-model-api` ACR-REGIONS (ruling D9) | new |
| W4-0.7 | Orchestrator rulings text, doc and tools fixes (HYGIENE-TOOLS, HYGIENE-RTCAPI) | 0.6 and 0.7 |

The old item 0.8 (bench script) is not in W4-0 (ruling D6): it is W4-BENCH-X86, after W4-RT-ROWS and W4-RT-CELLINDEX merge (§4.1). `docs/research/dagmc-internals.md:16` is fixed at the RT-XDG merge, as before.

### 3.2 Contract changes (all additive; done by the orchestrator's W4-0 workers, never by a W4a-c worker)

Before W4a (W4-0):
- **ACR-POLY2** (`sm-rt-api`, W4-0.3): `TrackerInfo::convex_cells: usize`, filled by sm-rt; `SmrtInfo` in the C ABI unchanged (`drafts/poly.md` §ACR-POLY2). Hash impact: none; `rt.info` output grows.
- **ACR-RT-HEXY** (`sm-rt-api`, W4-0.3; ruling D8 overrides W3b Q3): a new variant `LatticeKind::HexYRows { pitch, z_pitch }` with centre `(x, y) = pitch * (sqrt(3)/2 q, r + q/2)` (the `sm-model-api` `HexOrient::YRows` convention, `instancing.rs:102-111`); the existing `LatticeKind::Hex` is documented as x-rows (its formula already is). Additive because `LatticeKind` is `#[non_exhaustive]`; content-hash tag 3, so tags 1 and 2 hash as before. No consumer yet (sm-rt refuses every hex lattice until W4-RT-HEX).
- **ACR-RT-REGIONSETS** (`sm-rt-api` + the sm-rt build arm, W4-0.3; ruling D9): `RegionRule::ByTaggedCellSets(Vec<RegionSet>)`, `RegionSet { set, name, tags: Vec<(String, i64)> }` (`#[non_exhaustive]`, constructor). sm-rt treats it as `ByCellSets` with the region named `name` and tagged `tags` (`drafts/regions.md` R3).
- **ACR-DAGMC-SRC** (`sm-protocol`, W4-0.5; ruling D7(b)): `Request::Export.source: ExportSource { Mesh (default), Model }`, `#[serde(default, skip_serializing_if = "is_default")]`; the `chord_tol` key of the DAGMC options map is sm-server's (DAGMC-MODEL), not the protocol's. Existing requests serialise byte for byte as before; the golden changes are exactly the insertions and the schema diff that W4-0.5 lists.
- **ACR-REGIONS** (`sm-model-api`, W4-0.6; ruling D9): `lattice_frames` and `region_tags` tables in `Model` (`drafts/regions.md` R1), hashed and serialized only when non-empty.
- **Doc-only** (`sm-protocol` in W4-0.5, `sm-rt-api` in W4-0.3): `request.rs:18`, `:27`, `:231-232`; `Lattice::element_center`/`locate_rect` (`crates/sm-rt-api/src/world.rs:74-80`, `:99-107`) keep using `pitch[2]` for 2D lattices (centre z is `origin.z + 0.5 * pitch[2]`, ruling OR-a): the doc that says "z pitch ignored" is corrected and the rule is pinned by a test (no behaviour change, no hash; `drafts/rt.md` finding 8).

Before W4b (W4-0b):
- **ACR-LEVELMAP** (`sm-layer-api`; ruling D3): merges ACR-MOD1 (`drafts/meshops.md` §MOD-0) and ACR-POLY-LEVELMAP (`drafts/poly.md` §W4-POLY-AGG). `MeshLayer` implements the existing `MeshCatalog`/`CatalogEdit::link` (`crates/sm-mesh-api/src/hierarchy.rs:481-515`) with a `BTreeMap<(fine, coarse), LevelMap>`; `on_retired` drops links naming a retired mesh; the digest appends a link section only when at least one link exists. Refinement levels and agglomeration levels are stored the same way (linked independent meshes). Acceptance: `drafts/meshops.md` §MOD-0 acceptance 1-5; every hash in `census-base.txt` equal.
- **ACR-V1** (`sm-layer-api`): one `Option<Arc<Overlay>>` in `ModelLayer`, shared with the decomposition; `topo()` returns the overlay view when present, new `host()` returns the bare host; the ~5 editing/exporting call sites in `sm-boolean`, `sm-model`, `sm-cadio` switch to `host()` in the same commit; digest contribution only when `Some`, counters included (`drafts/vtopo.md` §W4-0 ACR-V1, D1). Gate: the full workspace tests, not only the pins (`drafts/vtopo.md` R2).
- **ACR-V3** (`sm-topo-api`): `VirtualOp::CompositeEdges` and `VirtualOp::CollapseEdge` with a recorded gap and per-cell view tolerance (`drafts/vtopo.md` §W4-0 ACR-V3, D3). `TopologyEdit::collapse_edge` is not added.

Before W4c (W4-0c):
- **ACR-V2a** (`sm-topo-api`, `sm-scheme-api`): `TopologyView::composite_surface(face)` for single-handle non-periodic composites, `GenCtx::loop_uvs`/`face_normal` using it, and the planner mapping `PartitionFace` to `Target::Surface` (`planner.rs:291`) (`drafts/vtopo.md` §W4-0 ACR-V2, V2a).
- **ACR-EXPFMT** (`sm-protocol`): `ExportFormat::{Fluent, Cgns}` (and `Abaqus` when that task starts); conda-forge `cgns` added to the conformance env (`tools/`).
- **ACR-CSG-HEXIR** (sm-csgio core, frozen per `crates/sm-csgio/DESIGN.md:3-4`; ruling D8): an IR `HexLattice` (both orientations, rings in OpenMC order, homogeneous `outer`) next to the rectangular `Lattice` (`crates/sm-csgio/src/ir.rs:188-200`); `content_hash` unchanged for every existing fixture (golden test). Scheduled now, no longer on demand.

On demand, not scheduled: ACR-V2b (periodic pcurve synthesis, with T2b), ACR-V4 (T11), a `GenCtx` read of quad-meshed bounding faces if the HEX-PYR spike needs it (`drafts/hexpin.md` R3), the union copy for true many-to-one sweep (`drafts/hexpin.md` R2), the volume copy scheme (`drafts/meshops.md` X-2). Each is an `api_change_request` reviewed by the orchestrator, and each must leave every existing hash equal.

`sm-testkit`: no change needed in W4. Fixtures the tasks need (BCC lattice, notch, step, fillet) are built inside the tasks' own tests.

### 3.3 W4-0 task specs

Common to every W4-0 task: start from `9cac369` (`git checkout --detach 9cac369`, commit, then `git branch w4-0/<key>-<n>`); touch only the owner paths; workers act for the orchestrator and may edit a frozen contract only for the ACR their task names, additively; `cargo build --workspace`, `cargo test` of every touched crate and of every crate that depends on a touched contract, `cargo clippy -- -D warnings` and `RUSTDOCFLAGS="-D warnings" cargo doc` on those crates, and finally `tools/ci.sh` green; rustdoc on every new public item. **Hash rule:** no existing hash may move: `tools/platform/expected.txt` and every golden file stay byte-identical except the insertions the task lists, and no existing test's asserted value is edited; the orchestrator compares the census of the merged result with `docs/design/w4/census-base.txt` (W4-0.1) and rejects any difference the task did not list. A `Cargo.lock` change is allowed only for the task's own new dev-dependency (for example `sm-testkit` and `proptest` in `sm-model-api`, which has neither today); the orchestrator resolves lockfile conflicts mechanically at merge. Report the commit SHA, the list of changed files, and every golden diff.

#### W4-0.1 CENSUS. Census baseline (independent; nobody waits on it)
- **Owner:** `tools/census/` (new), `docs/design/w4/census-base.txt` (new).
- **Spec:** write `tools/census/census.sh` (bash; a `python3 -I` helper in the same directory is allowed) that measures, at the checked-out commit, every value of the §7 population and prints one line per value, sorted, as `<group>|<key>|<os-arch or any>|<value>`, with a header of `#` lines (commit, `uname -sm`, `rustc -V`, date not included so two runs are byte-identical). Values are measured by running code (tests with `--nocapture`, `tools/platform/hashes.sh`, the CLI on the committed scripts, sha256 of committed golden and fixture files), never copied from test source. Groups: `platform` (the 62 data lines of `tools/platform/expected.txt`, recomputed, plus its sha256), `w3c_pin` (432 hexes, 733 tets, two `mesh.auto_tet` warnings), `submap_graded`, `p0_rows`, `flat_golden` (per os-arch tuples), `rtbench` (each existing line of `crates/sm-rt-bench/expected_sha256.txt`), `export` (the format hashes of `each_format_is_byte_identical_in_process_cli_and_mcp` and the sha256 of the format golden files), `dagmc` (the `crates/sm-meshio/tests/dagmc.rs` goldens), `poly` (375 cells at `crates/sm-poly/tests/acceptance.rs:93` and the `acc7_determinism` hash), `conformance` (sha256 of the `polydual` and `w3a_poly` fixtures), `csgio` (sha256 of every `oracle.json` and the `content_hash` goldens), and `golden-reviewed` (sha256 of every file under `crates/*/golden/` and `crates/*/tests/golden/`: these may change only by the reviewed insertions of §7, so they are listed separately). Linux rows (`flat_golden`, platform) come from the log of the last green GitHub CI run on `main` (`gh run list`, `gh run view --log`); the run id is recorded in the header, and rows that the log does not carry are listed as `missing` with the reason. Add `--compare <file>`: prints keys whose value differs or that are missing on either side and exits 1 if any.
- **Acceptance:** (1) `tools/census/census.sh > a.txt; tools/census/census.sh > b.txt; cmp a.txt b.txt` succeeds, and `RAYON_NUM_THREADS=1 tools/census/census.sh | cmp - a.txt` succeeds; (2) `docs/design/w4/census-base.txt` equals `a.txt` produced at `69ba521` (docs-only over `9cac369`; header comments may differ, and the two sm-protocol golden rows were refreshed after W4-0.5) on macOS arm64 plus the Linux rows; (3) `tools/census/census.sh --compare docs/design/w4/census-base.txt` exits 0 at the task commit; editing one value in a copy of the file makes it exit 1 and print that key; (4) every group named above has at least one line, and the `platform` group has exactly 62 data lines plus the file's sha256 row; (5) `shellcheck` and `ruff` (for the helper) are clean; (6) no file outside the owner paths changes.
- **Hash impact:** none.

#### W4-0.2 SKEL. New crate skeletons and registration
- **Owner:** `crates/sm-decomp/`, `crates/sm-scheme-ogrid/`, `crates/sm-csgbrep/`, `crates/sm-modify/` (new), `Cargo.lock`, `crates/sm-server/Cargo.toml`, `crates/sm-server/src/standard.rs` (registration lines only), `tools/ci.sh` (only if a per-crate line is needed; the workspace members are the glob `crates/*`, root `Cargo.toml:3`, so the root manifest needs no edit).
- **Spec:** each crate gets `Cargo.toml` (edition 2024, workspace package fields like its neighbours), `lib.rs` with crate rustdoc stating its purpose and its W4 task, `DESIGN.md` with a one-paragraph scope; `sm-scheme-ogrid` exports `pub fn register(&mut SchemeRegistry) -> sm_scheme_api::Result<()>` that registers nothing; `sm-csgbrep` and `sm-modify` export a `Plugin` with ids `csgbrep` and `modify` that registers no op; `sm-decomp` is a library (no plugin). Register `sm_scheme_ogrid::register` in `standard_schemes` (`standard.rs:25-32`, the array grows to 9; no auto list changes) and the two plugins in `standard()`. Each crate has one determinism-trivial test (`register` succeeds twice into fresh registries and the registry's op/scheme listing is unchanged).
- **Acceptance:** (1) plugin ids are not listable, so the check is that the op and query lists and the sm-server, sm-cli and sm-mcp goldens are unchanged; (2) `cargo test -p sm-server -p sm-mcp -p sm-cli` green with every golden byte-identical, except a listing golden that prints plugin ids or scheme crates, which may gain exactly those lines (listed in the report); (3) if registering an empty plugin or scheme crate changes any state hash or any `tools/platform/expected.txt` value, do not register it: leave the crate unregistered, record that in its `DESIGN.md`, and report it (the first task that adds an op registers it); (4) `cargo tree -p sm-csgbrep` shows no dependency on sm-server (layering: L3 under the server); (5) `tools/ci.sh` green.
- **Hash impact:** none.

#### W4-0.3 RTAPI. `sm-rt-api` ACRs
- **Owner:** `crates/sm-rt-api/`; in `crates/sm-rt/` only the `TrackerInfo` fill sites (`src/image.rs:535`, `src/world.rs:447`, `src/analytic/tracker.rs:218`, the mesh tracker's info construction), the `rt.info` output in `src/plugin.rs:229`, the `RegionRule` arm in `src/build.rs:97-134`, a read accessor for the convex-cell flags in `src/core.rs` if one is needed, and the region section of `crates/sm-rt/DESIGN.md`.
- **Spec:** (a) ACR-POLY2: add `pub convex_cells: usize` to `TrackerInfo` (`crates/sm-rt-api/src/tracker.rs:99-130`, `#[non_exhaustive]`, default 0 in `TrackerInfo::new`); sm-rt fills it from its convex-cell flags (`C_CONVEX`, `core.rs:21`) for mesh trackers and 0 for analytic and world trackers whose leaves have no such flag; `rt.info` prints it. (b) ACR-RT-HEXY: add `LatticeKind::HexYRows { pitch: f64, z_pitch: f64 }`; `Lattice::element_center` for it is `origin + (pitch * sqrt(3)/2 q, pitch * (r + q/2), k z_pitch)`; `shape_issues` checks it like `Hex`; the content hash writes tag 3 then the two floats; the doc of `Hex` says "rows along x (pointy-top), `HexOrient::XRows` in sm-model-api" and the doc of `HexYRows` "rows along y (flat-top)". The stub (`stub.rs:1352`) and sm-rt (`world.rs:985`) keep refusing all hex kinds with `Unsupported`. (c) ACR-RT-REGIONSETS: add `RegionSet { set: String, name: String, tags: Vec<(String, i64)> }` (`#[non_exhaustive]`, `RegionSet::new(set, name)`, `with_tag`) and `RegionRule::ByTaggedCellSets(Vec<RegionSet>)`; its content hash uses a new tag and writes set, name and tags in order; in `build.rs`, the arm does what `ByCellSets` does (`:112-133`) but names the region `name` and attaches `tags` with `RegionInfo::with_tag`. Check `docs/design/w3/backlog.md:98` ("`RegionInfo.tags` not in cache images and hashed without a presence marker"): the image part is done (`image.rs:20`); if the structure hash still hashes tags without a presence marker, add the marker only for regions that have tags, so no tag-free image or hash changes; write the finding in `crates/sm-rt/DESIGN.md`'s region section. (d) Doc fix, no behaviour change (ruling OR-a): `Lattice::element_center` and `locate_rect` (`world.rs:74-80`, `:99-107`) keep using `pitch[2]` for a 2D lattice (`dims[2] == 1`): the element centre z is `origin.z + 0.5 * pitch[2]`, and `bridge.rs:325` and sm-rt's `Lat::center` (`world.rs:194-199`) rely on it. Correct the docs that say "z pitch ignored" (`Rect` field doc `world.rs:39`; the comment at `:438`) to state this rule.
- **Acceptance:** (1) a unit test builds `TrackerInfo::new` and reads `convex_cells == 0`; a mesh tracker over a 2x2x2 brick of hexes reports `convex_cells == cells`; (2) `element_center` of `HexYRows` at `(1, 0, 0)` with pitch 2 is `(sqrt(3), 1, 0)` and at `(0, 1, 0)` is `(0, 2, 0)` to 1e-15; `Hex` is unchanged at the same indices; the world content hash of an existing Rect and Hex world equals its value before the change (pinned in the test from the pre-change build); a `HexYRows` world differs from the same `Hex` world; (3) a mesh tracker built with `ByTaggedCellSets` over the two block sets of a two-brick mesh has two regions named by `name`, with exactly the given tags, round-tripped through `write_smrt`/`read_smrt`; the same mesh with `ByCellSets` gives byte-identical `.smrt` bytes before and after the change; (4) a test pins the actual rule: for `dims[2] == 1` and a finite `pitch[2]` of 1 and of 3, `element_center([0, 0, 0]).z == origin.z + 0.5 * pitch[2]`, and `locate_rect` uses `pitch[2]` (no behaviour change, no hash moves); (5) `cargo test -p sm-rt-api -p sm-rt -p sm-rt-capi -p sm-server -p sm-rt-bench` green; `crates/sm-rt/tests/flat_golden.rs`, the W3b `h_*` hashes and `expected_sha256.txt` unchanged; (6) an `rt.info` output golden, if one exists, gains only the `convex_cells` field (reported).
- **Hash impact:** none on existing hashes; `rt.info` output grows.

#### W4-0.4 CSGBRIEF. Decisions for the CSG import
- **Owner:** `docs/design/w4/csgbrief.md` (new).
- **Spec:** `drafts/csgbrep.md` §CSGBRIEF-0, minus the crate skeleton and plugin registration (W4-0.2). Record: D1 option (a), one op `model.import_openmc` (ruling D10: the import is one history node); materials as a second `Source` field of the same type (no ACR) or, if not, the exact `api_change_request`; for `model.copy` (`crates/sm-model/src/copy.rs`) and `fem.block_add`/`fem.sideset_add` (`crates/sm-fem/src/ops.rs:84,270`), whether the import can call their `TypedOp::apply` with its own context without skipping the layer bookkeeping (`model_txn` lineage, colours, groups, `sm-boolean/DESIGN.md:59-63`), or needs public library functions, named with signatures. Also record how the import writes D9 tags: one frame per deck lattice through ACR-REGIONS (`drafts/regions.md` R2), names `lat<id>`, nesting by fill. The decision is by reading code and, where needed, a throw-away spike outside the repo (not committed).
- **Acceptance:** the file states each decision with file:line evidence; it lists W4-CSG-LIBFN's functions with signatures, or says "LIBFN dropped" with the reason; it names every `ApplyCtx` method the import would call and shows each is public (`crates/sm-op-api/src/op.rs`); no code file changes.
- **Hash impact:** none.

#### W4-0.5 PROTO. `sm-protocol` ACR-DAGMC-SRC and the protocol doc fixes
- **Owner:** `crates/sm-protocol/` (src, golden, tests); in `crates/sm-server/src/server.rs` the `Request::Export` arm (`:254`) only; in `crates/sm-cli/src/lib.rs` the `Request::Export` literal (`:637`) only; the MCP and Python goldens or stubs that embed the `session.export` schema (`crates/sm-mcp/tests/golden/`, sm-py stubs if generated), for the new optional field only.
- **Spec:** add `pub enum ExportSource { #[default] Mesh, Model }` (`serde(rename_all = "lowercase")`, `JsonSchema`, `#[non_exhaustive]`) and the field `source: ExportSource` on `Request::Export` with `#[serde(default, skip_serializing_if = "is_default")]` and rustdoc ("`model`: DAGMC only; facets from the B-rep, no volume mesh"). In sm-server, `source: Model` returns the typed error `unsupported-source` ("`source: model` is not available yet") for every format until W4-DAGMC-MODEL replaces it, so it is never silently ignored; sm-cli sets `source: ExportSource::Mesh`. Doc fixes in `request.rs`: `:18` (Exodus is classic NetCDF-3 64-bit offset or NetCDF-4, whichever the writer emits; check `crates/sm-exodus`), `:27` and `:231-232` (replace the per-format option lists by "validated by the exporter; see `sm-server` `mesh_export.rs` for the keys of each format", so the doc cannot go stale when exporters gain keys).
- **Acceptance:** (1) every request in `crates/sm-protocol/golden/requests.json` serialises byte for byte as before; the golden gains exactly two entries (`source` absent, `source: "model"`), and `rejected.json` gains one (`source: "volume"` is rejected); (2) `catalog.json` and any MCP/Python schema golden differ only by the `source` property and the `ExportSource` definition (diff listed in the report); (3) a server test sends `source: "model"` and gets `unsupported-source`; (4) `cargo test -p sm-protocol -p sm-server -p sm-cli -p sm-mcp -p sm-py` green; `each_format_is_byte_identical_in_process_cli_and_mcp` unchanged.
- **Ruling OR-b:** the mechanical `source: Default::default()` / `ExportSource::Mesh` additions in existing test literals of `sm-server/tests` and `sm-cli/tests` are accepted (no asserted value changes). MCP and Python exposure of `source` is deferred to W4-DAGMC-MODEL.
- **Hash impact:** none.

#### W4-0.6 MODELAPI. `sm-model-api` ACR-REGIONS (ruling D9)
- **Owner:** `crates/sm-model-api/`.
- **Spec:** `drafts/regions.md` R1, exactly: `LatticeFrame { layout: Layout, origin: Point3, parent: Option<String> }`, `RegionTag { slots: BTreeMap<String, SlotIx>, ring: Option<SubIndex>, sector: Option<SubIndex> }`, `SubIndex { index: u32, count: u32 }` (all `#[non_exhaustive]` with constructors, serde and schemars behind the crate's features); `Model` fields `lattice_frames: BTreeMap<String, LatticeFrame>` and `region_tags: BTreeMap<EntityId, RegionTag>` with `#[serde(default, skip_serializing_if = "BTreeMap::is_empty")]` (also in `ModelData`); methods `define_lattice_frame`, `lattice_frame`, `lattice_frames`, `set_region_tag`, `clear_region_tag`, `region_tag`, `region_tags`, and `LatticeFrame::locate(Point3) -> Option<SlotIx>`; `canonical_visit` emits tag 14 (frames: name, layout, origin quantized as lengths, parent) and tag 15 (region tags) **only when the table is non-empty**, after the decomposition table (tag 13 is already used by instancing clusters in `sm-model-api` `canonical.rs`; ruling OR-c); `apply_lineage` maps tags one-to-many like `volume_color` and drops the tag of a volume that inherits two different tags; `check_integrity` checks frame names (`[A-Za-z0-9_]+`), parents (exist, acyclic), slots (frame exists, `layout.contains`), `count > index`, keys are `TopoVolume`.
- **Acceptance:** (1) an empty `Model`'s `canonical_visit` word stream and its serde JSON are byte-identical to the pre-change build (pinned in a test from the pre-change output); `tools/platform/expected.txt` values and every workspace test unchanged (`tools/ci.sh` green); (2) `locate` for a Cartesian 3x3x1 frame, pitch 1.26, origin 0: `(1.26, 2.52, 0)` gives `(1, 2, 0)`, `(1.89, 0, 0)` (a boundary) gives the documented tie result, `(-0.7, 0, 0)` gives `None`; for both `HexOrient` values with 3 rings, every slot's `Layout::offset` locates back to that slot and a point at 0.49 pitch towards each neighbour stays in the slot; (3) a split lineage (one volume to three) copies the tag to all three; a merge of two volumes with different tags leaves no tag; a merge of two equal tags keeps it; (4) `check_integrity` rejects an unknown frame, a parent cycle, a slot outside the layout, `index >= count`, and a bad frame name, each with a typed error; (5) serde round trip of a model with frames and tags is identity and passes `check_integrity`; (6) the word stream of a model with one frame differs from the empty model's (the table is hashed); (7) proptest with `sm_testkit::strategies::config(64)`: random tag tables survive serde round trip and `canonical_visit` is independent of insertion order.
- **Hash impact:** none on existing hashes (tables empty unless a W4 op writes them).

#### W4-0.7 DOCS. Orchestrator rulings, doc and tools fixes
- **Owner:** `docs/design/w4/rulings.md` (append a section "Orchestrator rulings" only), `tools/platform/collect.py`, `tools/w3c_driver.py`, `docs/design/w3/backlog.md`, `docs/PARITY.md`, `crates/sm-rt-capi/DESIGN.md`.
- **Spec:** (a) append to `rulings.md` the old item 0.6 as orchestrator rulings: `sm-rt-bench` may depend on `sm-poly` (library) for the L2b polyhedral twin; the conforming P1 tet twin is generated offline with the `smesh` CLI into the case directory and its SHA-256 pinned per os-arch, no `sm-mesher` dependency (`drafts/rt.md` risk 8); healing lives in `sm-boolean`, tolerance first, collapse virtual (`drafts/vtopo.md` D3, D4); pre-registered win criteria and the default-build headline with a labelled `target-cpu=native` row (`drafts/rt.md` Q2, Q5); `renumber` is not in the export selection (`drafts/export.md` D3); GPU smoothing and quality not in W4, with the trigger of `drafts/export.md` D5 (ruling D10). (b) HYGIENE-TOOLS: `tools/platform/collect.py:58`, macOS `rustc -V` recorded, `mypy` and `ruff` clean for `tools/w3c_driver.py` (`drafts/export.md` §HYGIENE-*). (c) Mark `docs/design/w3/backlog.md:9,13,24` done and `docs/PARITY.md:59` partly done, each with the commit or test that did it; mark backlog `:102` "fired (ruling D9, W4-REGIONS)". (d) HYGIENE-RTCAPI: `crates/sm-rt-capi/DESIGN.md:111` width.
- **Acceptance:** `ruff check tools/platform/collect.py tools/w3c_driver.py` and `mypy tools/w3c_driver.py` clean; `python3 -I tools/platform/collect.py --help` (or its existing smoke) runs; `tools/ci.sh` green; the user's D1-D10 text in `rulings.md` is byte-identical (only the appended section is new); no code file changes.
- **Hash impact:** none.

---

## 4. Tasks

### 4.1 W4a: the two goals (polyhedral transport meshes, beating DAGMC/XDG)

Selection rule: a task is in W4a when it produces or improves a polyhedral transport mesh, an `.smrt` or `.smsh` that the transport code consumes, or a benchmark row against DAGMC/XDG; plus three small deferrals from W3c (CLI-DIAG, EXO-VERIFY, STRAT) and the reference corpus that feeds acceptance (ruling D8).

#### Step a1 (in parallel, after W4-0)

### W4-POLY-MEASURE. `sm-quality` + `sm-poly`: convexity, fast-path share, dual eligibility
- **Owner:** `crates/sm-quality/`, `crates/sm-poly/` (one worker, two crates).
- **Scope:** `drafts/poly.md` §W4-POLY-MEASURE scope 1-5 (`cell_convexity`, `fast_path_eligible`, `PolyStats` fields, the dual-eligibility counts on a tet mesh, the measurement table, the list of pins). Plan delta: the table also has a column for cells that are convex but over the 64-face cap, so the orchestrator can decide the `CONVEX_CHECK_MAX` lever (`drafts/poly.md` Priority). Plan delta (ruling D8): the table includes the W4-REFCORPUS models that mesh today, when the corpus has landed.
- **Acceptance:** `drafts/poly.md` §W4-POLY-MEASURE acceptance (a), (b) (with ACR-POLY2's `convex_cells`), (c), (d).
- **Gates:** W4-0.3. The kill criterion of the draft is the orchestrator's: the table is reviewed before VORO branches.
- **Hash impact:** none on state hashes; `mesh.poly_info`/`mesh.quality` JSON gain fields; the metric stays out of profile `rules`. A `mesh.quality` schema golden, if one exists, is reported, not re-blessed.

### W4-POLY-VOLDIAG. `sm-diag`: per-region mesh volume against the model
- **Owner:** `crates/sm-diag/`.
- **Scope / acceptance:** `drafts/poly.md` §W4-POLY-VOLDIAG (query `mesh.volume_check`; cylinder `0.5 n r^2 sin(2 pi / n) H` to 1e-12; brick deficit 0; tet and dual agree). Ruling D4: it reports the deficit everywhere, per volume (so per ring and sector once W4-REGIONS-CUT volumes exist).
- **Hash impact:** none (query).

### W4-RT-XDG. `sm-rt-bench`: XDG pin, driver and the tet walk (L1, L2)
- **Owner:** `crates/sm-rt-bench/` except `src/result.rs`, `src/report.rs` and their tests.
- **Scope:** `drafts/rt.md` §W4-RT-XDG-PIN (as step 0, with one `const XDG_PIN` used by the tests at `docker.rs:686`, `:704-707`), §W4-RT-XDG-DRIVER, and §W4-RT-TETS scope (a) and (b) on H1. TETS (c), the conforming P1 twin, moves to W4-RT-ROWS. HYGIENE-RTBENCH rides along.
- **Acceptance:** `drafts/rt.md` §W4-RT-XDG-DRIVER acceptance 1-7; §W4-RT-TETS acceptance 1-4 (H1 only).
- **Gates:** Docker Desktop and network; anything not built is `Partial` with the command, never a number. Smoke run `run --cases P1,H1 --scale reduced --trackers smrt,dagmc-dd,xdg`.
- **Hash impact:** new `expected_sha256.txt` lines only; existing lines byte equal.
- **Plan delta (a1 review, ruling OR-d):** acceptance 6 (`docker-check` Ready for the host arch) is met only on x86_64. The XDG image is x86_64-only (upstream `vec3da.h` includes `immintrin.h`), so on arm64 `docker-check` reports `Failed` with the reason and the rows are `Partial`; a Rosetta amd64 smoke run is allowed; real numbers come from W4-BENCH-X86. Done in a1: the rows are `partial` with the command. The follow-ups for the same crate are under W4-RT-ROWS.

### W4-RT-VERDICT. `sm-rt-bench` (files): bare-metal banner and pre-registered verdict
- **Owner:** `crates/sm-rt-bench/src/result.rs`, `src/report.rs`, their tests, `crates/sm-rt-bench/VERDICT.md` (new; RT-XDG links it from `DESIGN.md`). The script is W4-BENCH-X86 (after a2, ruling D6).
- **Scope:** `drafts/rt.md` §W4-RT-X86 crate part (`bench-env.json` fields, `EnvRecord::smoke` gains virt and governor) and §W4-RT-PUBLISH verdict code with the separate `report --sessions` aggregator. Plan delta (ruling D6): the polyhedral rows render as a separately labelled table that the k-of-n verdict never reads; a golden test proves a polyhedral row cannot change a verdict.
- **Acceptance:** `drafts/rt.md` §W4-RT-X86 acceptance (banner unit test over four fake environments, dry run on arm64) and §W4-RT-PUBLISH acceptance (golden verdict test on fabricated rows), plus the polyhedral-row exclusion test.
- **Gates:** none. Merge before W4-RT-ROWS branches.
- **Hash impact:** none.

### W4-RT-WORLD. `sm-rt`: CSG prototypes under `Lattice`/`Union`
- **Owner:** `crates/sm-rt/`.
- **Scope:** ruling OR-a supersedes `drafts/rt.md` §W4-RT-LATZ: the 2D-lattice z rule stays (centre z = `origin.z + 0.5 * pitch[2]`, `PITCH_2D` stays), so LATZ is not built and its pitch-0 acceptance and hash note do not apply. Only §W4-RT-CSG-WORLD is built here (sm-rt part only; the P2 `analytic-hierarchy` bench row is W4-RT-ROWS). Hex lattices are W4-RT-HEX (b), not this task.
- **Acceptance:** §W4-RT-CSG-WORLD acceptance except the bench row (`pins_touched`, `structure_bytes` are checked in an sm-rt test on the P2 world).
- **Gates:** none beyond C3 (no behaviour change for 2D lattices).
- **Hash impact:** none intended; `tests/flat_golden.rs` and the W3b `h_*` hashes stay equal.

### W4-HEX-STRAT. `sm-mesher`: `hex_only`, `tet`, per-volume mesh volume
- **Owner:** `crates/sm-mesher/`. HYGIENE-MESHER rides along.
- **Scope:** `drafts/hexpin.md` §W4-HEX-STRAT. Plan delta: `hex_only` lists `[map, submap, pave]` / `[map, sweep]` here; `ogrid` joins its list in W4-MESHER-LISTS after OGRID-S merges (an auto list that names an unregistered scheme fails, `standard.rs:240-241`).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-STRAT acceptance 1-6 (acceptance 2: `hex_only` equals `default` everywhere until LISTS).
- **Hash impact:** none on state; `mesh.info` gains `volume_by_owner`; the MCP golden diff, if any, is reviewed by the orchestrator.
- **Plan delta (a1 review):** acceptance 4 is restated as: faces follow tetmesh's boundary scheme (on a brick the `tet` strategy records only `{tetmesh, Volume#1}`, and the surface list `[trimesh]` does not apply to faces tetmesh imposes); `tet` gives all-Tet cells, 0 `mesh.auto_tet` warnings and the exact brick volume (1e-12). The same restatement is in `drafts/hexpin.md`. Done by W4-HEX-STRAT-FIX (ba231e9): the `tet` description and rustdoc state the face scheme, the stub-test faces assertion requires at least one face, a real-geometry tet brick test checks all-Tet, 0 warnings and volume 24 to 1e-12, and replay swaps in a counting `hex_only` via `MesherConfig::with_strategy` (0 calls, equal hashes). Only the optional `volume_by_owner` caching remains, on W4-MESHER-LISTS.

### W4-HEX-DECOMP. `sm-decomp` (new): cut planner for round cavities
- **Owner:** `crates/sm-decomp/`.
- **Scope:** `drafts/hexpin.md` §W4-HEX-DECOMP (step 0 re-runs the spike at `9cac369`; ordered `CutStep` list with `{step, role}` references; cylinder plus a plane at every cavity end face).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-DECOMP acceptance 1-5.
- **Hash impact:** none.

### W4-HEX-OGRID-K. `sm-scheme-ogrid` (new): pure O-grid and polar kernels
- **Owner:** `crates/sm-scheme-ogrid/`.
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-OGRID-K, acceptance 1-8 (both `ogrid` and `polar` modes, ruling D1; `collar` deferred).
- **Hash impact:** none.

### W4-CSG-CLASSIFY. `sm-csgbrep`: OpenMC quadric to parametric surface
- **Owner:** `crates/sm-csgbrep/src/quadric.rs` and its tests.
- **Scope / acceptance:** `drafts/csgbrep.md` §QUADRIC-CLASSIFY.
- **Hash impact:** none.
- **Plan delta (a1 review):** the quadric scale and tolerance items listed under W4-CSG-CORE (power-of-two normalisation, `k_zero` and radius/offset floors, the cosmetic `line` text) are carried by CORE, which consumes the module.

### W4-CSG-PROBE. `sm-boolean` (one new test file): what the kernel cuts today
- **Owner:** `crates/sm-boolean/tests/webcut_probe.rs`, plus a short report in the commit message.
- **Scope / acceptance:** `drafts/csgbrep.md` §WEBCUT-PROBE items 1-4. Its item 4 result picks the D2 partition or the per-cell boolean fallback for CSG-CORE.
- **Hash impact:** none.
- **Plan delta (a1 review):** pin the measured details in `tests/webcut_probe.rs` (the W4-CSG-CORE worker does it, explicit permission for that file): `d2_chain` asserts `on_x1 == 1` and the face count (11 faces in total); `d2_cut_one_piece` asserts the neighbour's body still bounds `b.volume`; for `keep=Negative/Positive` (tilted_cylinder, plane_cut) the discarded side is gone (role empty or one volume in the model).

### W4-REGIONS-CUT. `sm-boolean` (files): `model.subdivide_pin` (ruling D9)
- **Owner:** `crates/sm-boolean/src/subdivide.rs` (new), `crates/sm-boolean/tests/subdivide.rs` (new), the op registration line in `crates/sm-boolean/src/lib.rs`.
- **Scope / acceptance:** `drafts/regions.md` §W4-REGIONS-CUT, acceptance 1-6 (rings by equal area, equal radius or radii; sectors of count 1 or even; ring and sector tags through ACR-REGIONS; lattice slots kept).
- **Gates:** W4-0.6.
- **Hash impact:** new op only.
- **Plan delta (a1 review):** kernel finding. `webcut_cylinder` / `subtract` fail with `result volume invalid` for a 0.475-radius cylinder cut at 0.418 or 0.476 with the default tool frame; turning the frame's x reference by 1 rad (SEAM_TURN) avoids it. The cause is investigated in W4-BOOL-TOOLS (b1).

### W4-CSG-LIBFN (dropped by csgbrief.md §6; kept as the fallback in §3.4). `sm-model` + `sm-fem`
- **Scope:** the public library functions named in W4-0.4 (copy with pitch and count; block and sideset add), each wrapping today's op body so the op and the function share one code path.
- **Acceptance:** the ops' outputs and every test in both crates are unchanged; the new functions have rustdoc and one test each.
- **Hash impact:** none (the op bodies call the same code).

### W4-CLI-DIAG. `sm-cli`: per-op diagnostics in `smesh run` text mode
- **Owner:** `crates/sm-cli/`. HYGIENE-CLI rides along.
- **Scope / acceptance:** `drafts/export.md` §CLI-DIAG and §HYGIENE-* HYGIENE-CLI.
- **Hash impact:** none (render and tests only).

### W4-EXO-VERIFY. `sm-exodus`: does a real reader take our NFACED?
- **Owner:** `crates/sm-exodus/`.
- **Scope / acceptance:** `drafts/export.md` §EXPORT-EXO-VERIFY (libMesh `meshtool-opt` from the moose env). Plan delta (ruling D2): `.smsh` is the transport code's format, so the result is recorded for interchange (MOOSE/Griffin through Exodus) and does not change the W4 export order.
- **Hash impact:** none.

### W4-DAGMC-MODEL. `sm-server` + `sm-meshio/src/dagmc`: DAGMC h5m from the B-rep (ruling D7(b))
- **Owner:** `crates/sm-server/`, `crates/sm-meshio/src/dagmc/`. HYGIENE-SERVER rides along.
- **Scope / acceptance:** `drafts/export.md` §EXPORT-DAGMC-MODEL (structural equality with MOAB's `facets_to_h5m` output on the bench's pin twin facets is the strongest check). Plan delta: it replaces W4-0.5's `unsupported-source` error by the model path for `dagmc` (other formats keep the error) and adds the `chord_tol` key to the DAGMC options map. The CLI flag for `source` is W4-CSG-CLI's (`sm-cli` is CLI-DIAG's in a1); here the path is tested through the protocol (in-process, MCP and Python). Plan delta (ruling D7(c)): no foreign `.h5m` is read; the files we check are the ones we and MOAB write.
- **Plan delta (ruling OR-b):** it also exposes `source` through MCP (the `session.export` tool arguments) and Python (sm-py), with a test on each, since W4-0.5 only adds it to the protocol, server and CLI.
- **Gates:** after W4-0.5.
- **Hash impact:** new path; existing `model_from_mesh` bytes unchanged; new hashes per os-arch.

### W4-REFCORPUS. `tools/refcorpus/` + `docs/design/w4/refcorpus.md`: reference models from public sources (ruling D8)
- **Owner:** `tools/refcorpus/` (new), `docs/design/w4/refcorpus.md` (new). Explicit permission for these new `tools/` files.
- **Scope:** scripts that build a reference corpus from public sources; the scripts, a manifest and sha256 values are committed, the models are not. Default output `${SM_REFCORPUS_DIR:-$HOME/.cache/smart-mesh/refcorpus}`. Sources and entries:
  1. **OpenMC decks we write** (conda env `openmc`, Python with `openmc.reset_auto_ids()` and fixed ids, so the XML is byte-reproducible): a PWR pin cell; a 17x17 PWR assembly with guide tubes (mixed universes); a 3x3 mini-assembly; an axially layered pin (z planes); hex assemblies in **both** OpenMC hex orientations (7 rings, pins of one universe, plus one mixed); a TRISO-like sphere lattice and a deck with cones and tori (for W4-BOOL-TOOLS).
  2. **BEAVRS** (MIT CRPG public benchmark): its OpenMC model builder, pinned to a commit; extract a pin cell, one assembly and a 2D core slice as OpenMC XML.
  3. **INL Virtual Test Bed** (`github.com/idaholab/virtual_test_bed`, pinned commit): the OpenMC inputs and the MOOSE reactor-module mesh inputs of models with hex lattices (HTGR, SFR, microreactor); list each chosen file with its repository path.
  4. **FreeCAD** (via the FreeCAD MCP or `freecadcmd`): clean STEP (pin, cladded pin with plenum, assembly can, a grid-spacer-like part) and deliberately dirty STEP, one defect per file with the defect stated in the manifest: notch of `1e-4` (small edges and faces), step of `1e-4` (sliver faces), two bodies with a `1e-5` gap, an open shell (one face removed), a tiny through hole, a fillet (blend candidate), and a tolerance-mismatched edge. The STEP header (`FILE_NAME` timestamp, author, system) is normalised to fixed strings before hashing, so sha256 is reproducible; the FreeCAD version is recorded.
  5. **MOOSE** (optional, only if a `moose` env is present): run reactor-module generators of a VTB model with `--mesh-only` to Exodus as reference meshes; recorded as optional and per os-arch.
- **Manifest:** `tools/refcorpus/manifest.toml`, one entry per output: `id`, `category` (`openmc`, `beavrs`, `vtb`, `step-clean`, `step-dirty`, `moose`), `source` (URL and pinned commit or our script), `licence` (copied from the source repository), `outputs` with `path` and `sha256`, `expected` (for dirty STEP: the `model.heal_check` codes it should raise, `drafts/vtopo.md` §T3; for OpenMC decks: `accepted` or the typed refusal code of today's `sm-csgio` import, e.g. hex lattices refused until W4-CSG-HEX), `optional`.
- **Acceptance:** (1) `python3 -I tools/refcorpus/build.py` builds every non-optional entry on the Mac, and `python3 -I tools/refcorpus/build.py --check` recomputes and matches every sha256 (exit 0); wiping the output directory and building again gives the same sha256 values; (2) at least: 6 OpenMC decks we write (both hex orientations included), 3 BEAVRS extracts, 3 VTB models with hex lattices, 3 clean and 7 dirty STEP files; (3) `--check` on a corrupted file exits 1 naming it; (4) for every OpenMC deck the manifest's `expected` equals what `sm-csgio` does today (run through the existing import, recorded by the script); (5) the script refuses to write into the repository, and `git status` after a build shows no new file outside `tools/refcorpus/`; no committed file is over 100 kB; (6) `refcorpus.md` lists each source, its licence, its pinned commit, what each entry feeds (W4-BOOL-HEAL-A thresholds, W4-CSG-ORACLE decks, the benchmark cases, W4-POLY-MEASURE) and how to rebuild; (7) `ruff` and `mypy` clean; network is used only in `build.py fetch` steps, which are separable (`--offline` builds from the cache).
- **Hash impact:** none (no crate changes).

#### Step a2 (each after the a1 tasks it names)

### W4-A1-TIDY. `tools/refcorpus/`, `crates/sm-exodus/`, `crates/sm-decomp/`: a1 review follow-ups
- **Owner:** `tools/refcorpus/`, `crates/sm-exodus/`, `crates/sm-decomp/`, and `tools/ci.sh` (explicit permission, only to add the refcorpus checks below).
- **Scope:** (1) REFCORPUS: the `gen_openmc.py` `deck_hex` orientation self-check (`assert geom.find(inside)`) is vacuous (true when the point lands in the outer water universe); assert that the leaf cell at the corner is the fuel pin and that a point at 7p along the prism-vertex axis is inside the prism (inside only when prism and lattice orientations match), or reword the `refcorpus.md` bullet. (2) `build --offline` with an empty cache must raise `CorpusError` ('not in cache; run fetch') and exit 2, not a `FileNotFoundError` traceback and exit 1 (recipes `copy` and `beavrs`). (3) Add `ruff check`, `mypy` and the unittest for `tools/refcorpus` to `tools/ci.sh` (as W4-0.7 did for `w3c_driver.py`). (4) The stale `gen_beavrs.py` docstring: it says PYTHONPATH is set, the code reads `SM_RC_BEAVRS_SRC` (`python -I` ignores PYTHONPATH). (5) EXO-VERIFY: `tests/libmesh.rs` hard-codes `MOOSE_ENV=/Users/rodrigo/opt/miniforge/envs/moose`; fall back to `$CONDA_PREFIX` or read only `SM_LIBMESH_BIN`. Record that each test skips green when `meshtool-opt` is missing, so the libMesh result is pinned only on the user's Mac. (6) `sm-decomp`: `plan.rs:397` `.expect("a run")` becomes a let-else (unreachable, but it is library code); assert `mesh_volume`, not only the model-measure total, in the `coaxial_blind_hole_and_enclosed_cavity` test; run `cargo fmt -p sm-decomp`.
- **Gates:** none (a1 merged). Runs in a2 in parallel with the other a2 tasks.
- **Acceptance:** each item has its test or check; `ruff`, `mypy` and the unittest pass where the tools exist; the `manifest.toml` sha256 values are unchanged.
- **Hash impact:** none.

### W4-POLY-VORO. `sm-poly`: circumcentric (Voronoi) dual, strict and opt-in
- **Owner:** `crates/sm-poly/`; if the `.smsh` writer does not fill `gc_*` for poly meshes, also `crates/sm-meshio/src/smsh/` for that fill (explicit permission).
- **Scope:** `drafts/poly.md` §W4-POLY-VORO. Plan delta (ruling D2): every polyhedral `.smsh` export fills the geometry cache `gc_*` (no format bump), and the generator point is stored as a cell tag (`drafts/poly.md` "`.smsh` for the transport code").
- **Acceptance:** `drafts/poly.md` §W4-POLY-VORO acceptance 1-8, plus: a poly mesh exported to `.smsh` has `gc_*` filled and the cached cell volumes and centroids equal `validate_geometry`'s to 1e-12 relative; an existing non-poly `.smsh` export is byte-identical.
- **Gates:** after MEASURE merges and the orchestrator has read its table (kill criterion). The eligibility rates of real meshes decide the follow-up (VORO-CLIP or CONFORM, §8; trigger-based per ruling D1) before VORO merges.
- **Hash impact:** none for existing hashes (`dual` omitted when default; non-poly `.smsh` bytes unchanged); new pins by INT.

### W4-HEX-QUERY. `sm-mesher`: `mesh.decompose_plan`
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-QUERY. **Gates:** after STRAT and DECOMP merge. **Hash impact:** none. Plan delta (a1 review): runs alongside W4-A1-TIDY, which only edits `sm-decomp` internals (a let-else, no API change); HEX-QUERY only consumes `sm-decomp`.

### W4-HEX-OGRID-S. `sm-scheme-ogrid`: scheme adapter
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-OGRID-S acceptance 1-4 (run with an explicit `mesh.scheme`, since no list names `ogrid` yet). **Gates:** after OGRID-K. **Hash impact:** none until a list names it.

### W4-CSG-CORE. `sm-csgbrep`: `model.import_openmc`, partition builder
- **Owner:** `crates/sm-csgbrep/`.
- **Scope:** `drafts/csgbrep.md` §CSGBREP-CORE on the plane-and-cylinder path (D1 (a), D2 or the PROBE fallback, D4 rectangular lattices, D6 blocks and sidesets). Plan delta (ruling D9): the import defines one lattice frame per deck lattice and tags each placed element's volumes with the deck's indices (`drafts/regions.md` R2, W4-0.4's naming).
- **Acceptance:** `drafts/csgbrep.md` §CSGBREP-CORE acceptance (all six bullets; sphere, cone and torus decks return typed `Unsupported` until W4-BOOL-TOOLS lands), plus: on the 3x3 lattice fixture, every element volume's tag equals the deck index and equals what `model.lattice_tag` would assign from its centroid (a test that computes the centroid rule itself until REGIONS-SPLIT merges).
- **Gates:** after CLASSIFY, PROBE and W4-0.4 (and LIBFN if scheduled).
- **Hash impact:** new op, new hashes only.
- **Plan delta (a1 review, from CLASSIFY):** in `crates/sm-csgbrep/src/quadric.rs`, divide all 10 coefficients by a power of two near `max|coeff|` before Jacobi (exact, no bits lost); today the `off <= 1e-34*diag` stop test under/overflows at extreme magnitudes (a cylinder scaled by 1e-200 is refused as an 'empty quadric', a sphere scaled by 1e200 as a 'single point'; fail-safe but the wrong message, and the module doc says the tolerance is relative). Document in the `classify_with_tol` rustdoc: `k_zero` is relative only to the translation terms (for a quadric centred at the world origin `kp == k`, so a cone with round-off 1e-15 in k is refused as a hyperboloid), and the radius/offset ratio has an effective floor of about 3e-5 (r=1e-3 at offset 1e4 is refused as a 'degenerate cylinder'). Fix the stray space in the error text when `line` is None (`surface 7 : ellipsoid`). Explicit permission for `quadric.rs` and its tests, and for `crates/sm-boolean/tests/webcut_probe.rs` (the CSG-PROBE pins above).

### W4-RT-ROWS. `sm-rt-bench`: jump row, polyhedral row, conforming P1 twin, P2 hierarchy row
- **Owner:** `crates/sm-rt-bench/`.
- **Scope:** `drafts/rt.md` §W4-RT-JUMP-ROW, §W4-RT-POLY-ROW (L2b, via the `sm-poly` dependency ruled in W4-0.7), §W4-RT-TETS scope (c) (P1 only, generated with `smesh` and SHA-pinned), and the P2 `analytic-hierarchy` row of §W4-RT-CSG-WORLD. Plan delta (ruling D8): a case built from a W4-REFCORPUS OpenMC assembly deck (17x17) is added at `reduced` scale, with its deck sha256 from the manifest.
- **Acceptance:** those sections' acceptance lists; the polyhedral rows labelled "different mesh, same geometry" and kept out of the k-of-n verdict (ruling D6).
- **Gates:** after RT-XDG, RT-VERDICT and RT-WORLD merge.
- **Hash impact:** new case files and `expected_sha256.txt` lines only.
- **Plan delta (a1 review):** same-crate follow-ups. (1) Wire `report --sessions` in `crates/sm-rt-bench/src/cli.rs` (`render_sessions` exists; the error text and W4-BENCH-X86 name the flag). (2) `smrt::walk_adapter` requires `cell_count == nt`, or `walk()` rejects negative mapped values (today `cell_map` is sized `cell_count().max(nt)` and unmapped cells keep -1). (3) `xdg_driver` fire-locate handles `find_volume == ID_NONE` as one lost ray (-2), like the hinted path, instead of calling `ray_fire(ID_NONE)` (under THROW the whole row becomes Partial). (4) Fix the wording 'with the compiler error' in `pins.env` and `DESIGN.md`, or add the last error lines of the build output to the `Failed` message. (5) The H1 walk comparison at >= 1e5 rays at full scale: the only judged output has 10,000 rays (reduced); run at full scale, on x86 via W4-BENCH-X86 if arm64 cannot. (6) The `SMRT_XDG_SCALE` default `tiny` makes `saved_xdg_walk_output_agrees_with_the_sm_rt_walk` fail on reduced output with 'track counts differ 10000 vs 200': fix the default or the message. (7) `verdicts()` uses `dedup_by_key(threads)` and silently keeps the first row at the same thread count: key cells by precision too, or state the rule in `VERDICT.md`. (8) `VERDICT.md`: 'Anything not proven is smoke' overstates the `/proc/cpuinfo` fallback (a VM hiding the `hypervisor` flag reads as `none` when `systemd-detect-virt` is missing); reword, or record the fallback source in `virt`. (9) Consider failing or recording when XDG prints a warning on stderr during load (TETS acceptance 2 is not asserted); `ImageStatus::NoDriver` stays against the draft as a recorded deviation. `result.rs` and `report.rs` are W4-RT-VERDICT's; items 1, 7 and 8 get explicit permission for them in this task.

### W4-RT-CELLINDEX. `sm-rt`: disjoint kd tiling for analytic leaves
- **Owner:** `crates/sm-rt/`.
- **Scope / acceptance:** `drafts/rt.md` §W4-RT-CELLBVH (renamed W4-RT-CELLINDEX), acceptance 1-6. This also covers `drafts/csgbrep.md` §CSGBREP-TRACK-PERF, which is dropped as a duplicate.
- **Gates:** after RT-WORLD (same crate). The trigger already fires on the P2 `analytic` row (`drafts/rt.md` finding 7).
- **Hash impact:** none; any move of `flat_golden` or W3b `h_*` stops the task.
- **Plan delta (a1 review):** (1) enforce the draft's `sm-csgio openmc_cross` acceptance on the hierarchical build: in `bridge_vs_deck` (`crates/sm-csgio/tests/openmc_cross.rs`) change `chords_to` to take `&dyn Tracker` and build the bridge with `w.world_tracker(...)` (or run both builds); the a1 worker measured a worst chord difference of 4.37e-14 for n=1,2 at 10,000 rays. Explicit permission for that one test file. (2) Remove the duplicated 'hex lattices' entry from the deferred list in `crates/sm-rt/DESIGN.md` (the word appears 4 times; keep 'hex lattices (W4-RT-HEX)').

### W4-REGIONS-SPLIT. `sm-model` + `sm-fem` + `sm-server` (files): lattice frames, tagging, the export split (ruling D9)
- **Owner:** `crates/sm-model/` (new `src/region.rs` with `model.lattice_define`, `model.lattice_tag`, the query `model.region_tags`; the optional `lattice` key of `model.copy` in `src/copy.rs`), `crates/sm-fem/src/lower.rs` and new `crates/sm-fem/src/region.rs`, `crates/sm-server/src/mesh_export.rs`, one new section of `crates/sm-meshio/SMSH_FORMAT.md` (explicit permission).
- **Scope / acceptance:** `drafts/regions.md` R2, R3 and §W4-REGIONS-SPLIT acceptance 1-7 (the `regions: {split: [...]}` option on `smsh` and `smrt`; `region/` cell sets in `.smsh`; `ByTaggedCellSets` with integer tags in `.smrt`; the 3x3 subdivided lattice with exactly 117 regions; the no-split byte identity; the 17x17 x 10 rings x 8 sectors scale check).
- **Gates:** W4-0.3, W4-0.6; after W4-REGIONS-CUT, W4-DAGMC-MODEL (`sm-server`) and W4-CSG-LIBFN if scheduled (`sm-model`, `sm-fem`). Must merge before W4-EXPORT-SELECT (b) branches (same files).
- **Hash impact:** none on existing hashes (opt-in option and new ops; the model tables stay empty unless used). New pins by INT.
- **Plan delta (a1 review):** (1) add a `model.region_clear` op (`crates/sm-model/src/region.rs`) to drop lattice frames and region tags. (2) `Delta.outputs` volumes are id-ordered, so consumers read indices from the region tags, never from output order. (3) moved to W4-BOOL-TOOLS, which owns `crates/sm-boolean/`.


#### Step a3

### W4-MESHER-LISTS. `sm-mesher`: `ogrid` into the strategy lists (ruling D5), VOLFIT (ruling D4)
- **Owner:** `crates/sm-mesher/`; in the one D5 commit, also the re-bless lines named below (explicit permission).
- **Scope:** (1) `ogrid` into `hex_only`'s surface list (`drafts/hexpin.md` D3). (2) Ruling D5: `ogrid` into `default`, as **one** commit that re-blesses exactly the R1 list of `drafts/hexpin.md` (`tools/platform/expected.txt:54-63`, `milestone_w3c_pin.rs:342-343`, `auto_select.rs:296`, the `w3c_pin_leftout` comparison) and lists every moved line in its message, with the census (`tools/census/census.sh --compare docs/design/w4/census-base.txt`) proving that nothing else moved; W2, W3a and W3b hashes stay; the orchestrator updates `expected.txt`. (3) Ruling D4: `drafts/hexpin.md` §W4-HEX-VOLFIT (`mesh.preserve_volume`, opt-in; plan delta: it tests on the W3c through pin and does not wait for PIN-INT). Plan delta (ruling D9): VOLFIT moves every selected circle to its own equal-area radius, so concentric ring cuts and sector cuts from W4-REGIONS-CUT keep each region's volume exact (`drafts/regions.md` R4).
- **Acceptance:** `drafts/hexpin.md` §W4-HEX-STRAT acceptance 2 restated (`hex_only` = `default` except disk/annulus faces); §W4-HEX-VOLFIT acceptance; plus a pin subdivided into 3 equal-area rings x 4 sectors with `n = 16` on every circle: after VOLFIT every region's mesh volume is `pi r^2 h / 12` to 1e-12 relative; the census diff equals exactly the D5 list.
- **Carries (a2 review):** the W4-HEX-QUERY documentation items (`mesh.decompose_plan` docs; see `a2-followups.json` key `W4-HEX-QUERY`).
- **Gates:** after OGRID-S and QUERY merge; item (3)'s ring check after REGIONS-CUT.
- **Hash impact:** exactly the D5 list (signed off by the user in ruling D5); nothing else.
- **Plan delta (a1 review):** one optional W4-HEX-STRAT leftover (the rest was done by HEX-STRAT-FIX, ba231e9): cache the last owner and its running sum per chunk in `volume_by_owner` (not a hot path).

### W4-HEX-PIN-INT. tests and scripts: the W4 pin milestone
- **Owner:** `crates/sm-cli/tests/milestone_w4_pin.rs`, `tools/w4_pin_ogrid.smesh`, `tools/w4_pin_enclosed_hex.smesh` (explicit permission for these new `tools/` files).
- **Scope / acceptance:** `drafts/hexpin.md` §W4-HEX-PIN-INT acceptance 1-5. Plan delta (ruling D4): the scripts run VOLFIT, and the fuel volume per pin is `pi r^2 h` to 1e-12 relative (moderator `(1.26^2 - pi r^2) h`); a second assertion on the same mesh without VOLFIT checks the polygon value `h (n/2) r^2 sin(2 pi / n)`. Pinning in `expected.txt` is done by INT.
- **Carries (a2 review):** the wire-level `mesh.decompose_plan` equality check (CLI, MCP and Python return the same plan as the server) from W4-HEX-QUERY.
- **Gates:** after MESHER-LISTS.
- **Hash impact:** new files only.

### W4-CSG-CHECK, then W4-CSG-ORACLE. `sm-csgbrep` (+ OpenMC fixtures)
- **Owner:** `crates/sm-csgbrep/`; ORACLE also `crates/sm-csgio/tests/fixtures/openmc/` (explicit permission; HYGIENE-CSGIO rides along).
- **Scope / acceptance:** `drafts/csgbrep.md` §CSGBREP-CHECK (sphere-lens row waits for W4-BOOL-TOOLS; the planar rows run now) and §CSGBREP-ORACLE. Plan delta: ORACLE's polyhedral row meshes the deck with `mesh.auto {strategy: "tet"}` (STRAT, a1), and with POLY-MIXED merged it also runs on the hex mesh. Plan delta (ruling D8): ORACLE adds the W4-REFCORPUS rectangular decks (pin, 3x3, 17x17 assembly, BEAVRS assembly) as fixtures (decks are small text; the committed `oracle.json` is generated at 600 points by `gen.py`). Plan delta (ruling D9): for the lattice decks, the lattice index of every oracle point from OpenMC's `find_cell` path equals the region tag of the B-rep volume that contains it.
- **Carries (a2 review):** the W4-CSG-CORE limits on the sagitta bounding box and on the unowned-remainder cut.
- **Gates:** after CORE; the REFCORPUS decks after W4-REFCORPUS.
- **Hash impact:** new fixtures only; existing `oracle.json` files unchanged.

- **Plan delta (a3 review), open items:** the 1e-3 gap and 1e-4 sliver rows of CSG-CHECK are not met (kernel limit, routed to W4-BOOL-TOOLS). ORACLE's polyhedral row ran on the tet mesh only; the hex-mesh run (POLY-MIXED is merged) and the lattice2d fixture (trimesh fails with 'points coincide' on its outer ring; `mesh.scheme(sweep)` fails on `quadric_cyl` with 'source and target share a vertex') go to a mesher follow-up. The placed-surface tolerance dedup in `deck.rs` is O(n^2) and first-match: bucket it before core-size decks (about 1e5 surfaces) land.
### W4-POLY-MIXED. `sm-poly`: `mesh.to_poly` on mixed meshes (ruling D1)
- **Owner:** `crates/sm-poly/`.
- **Scope:** `drafts/meshops.md` X-1: dualize the tet region, keep non-tet cells (hexes, wedges, pyramids) as polyhedra, cut each interface triangle into its dual sub-faces. Plan delta: this supersedes `drafts/poly.md`'s placement of POLY-MIXED in W5+; the transition rule needed is exactly this sub-face cut, not a general conformize.
- **Acceptance (plan):** (1) a W3c B6 `{Tet, Wedge}` mesh and a hex pin next to a tet moderator dualize; every cell passes `validate_geometry`; the volume sum equals the source to 1e-12; interface faces are shared. (2) An all-tet input gives today's output byte for byte (state hash pinned before the change). (3) Determinism. (4) The `.smsh` export of a mixed result fills `gc_*` (ruling D2).
- **Carries (a2 review):** the open W4-POLY-VORO items (acceptance 2 and 3 totals, a final validity check on circumcentric output, agglomerate plus generator tags).
- **Gates:** after VORO (same crate).
- **Hash impact:** none on existing hashes (all-tet path unchanged).

- **Plan delta (a3 review):** acceptance 1's hex pin next to a tet moderator cannot be built by `mesh.run` today (`mesh.auto_tet` turns the fuel into tets too); the synthetic fixtures stand in, recorded in `crates/sm-poly/DESIGN.md`. A positive real-mesh test (W3c B6 `TWO_BRICKS_HARD_TET` through `mesh.auto` then `mesh.to_poly`: {Tet 160, Wedge 32} -> 90 polyhedra, volume 16, 0 violations) goes to the W4 INT tests (sm-server or sm-cli).
#### After W4a's RT rows and cell index (ruling D6)

### W4-A2-RT. `sm-rt`: cell-index follow-ups (a2 review)
- **Owner:** `crates/sm-rt/`.
- **Scope:** `a2-followups.json` key `W4-RT-CELLINDEX` items 1-6: `DESIGN.md` Cell index fixes (the stray `);.`, P2 is `pin_assembly(3)`: 81 pins, 162 cells, 324 surfaces; the stale line 597 now points to the Cell index section, 64-cell threshold) and the recorded P2 reduced and full counters; an ignored or release-only acceptance-4 test at P2 full (n = 17); `memory_bytes` counts the index; a total listed-cells budget in `CellIndex::split`; torus events (either `push_hits` with `max_dist` and a `t <= t_exit` filter, or a torus model in the cellindex ray test). Item 7 (`on_face_any`, `enter_from_outside`, `Wrap`, `locate_candidates`) is W4-RT-HEX's.
- **Gates:** after RT-CELLINDEX. **Hash impact:** none; any move of `flat_golden` or W3b `h_*` stops the task.

### W4-A2-RTBENCH. `sm-rt-bench`: bench follow-ups (a2 review)
- **Owner:** `crates/sm-rt-bench/`.
- **Scope:** `a2-followups.json` key `W4-RT-ROWS`: the L3 labelling or removal of the group-reference verdict cells, the duplicate H1 `w-flight` row, the measured jump-vs-walk max |delta d| recorded, `PairEdges` serial overflow, A17 competitor skip, and the uncompiled `xdg_driver.cpp` change (compiled first by W4-BENCH-X86).
- **Gates:** after RT-ROWS. **Hash impact:** none.

### W4-BENCH-X86. orchestrator worker: the x86_64 benchmark script
- **Owner:** `tools/bench/` (new), `docs/design/w4/bench-x86.md` (new).
- **Scope:** `drafts/rt.md` §W4-RT-X86 scope, orchestrator part: `tools/bench/x86_64.sh`, one command, Docker images, no network after the image pull; self-check refuses a published table off bare-metal Linux x86_64 or without the `performance` governor; three sessions on different days, then `report --sessions` (W4-RT-VERDICT). The polyhedral rows are a separately labelled table, never in the k-of-n verdict. The user runs it on their work machine.
- **Acceptance:** `drafts/rt.md` §W4-RT-X86 acceptance (dry mode on the arm64 Mac exits with "not x86_64"); `shellcheck` clean; the doc states the exact commands for the user's three sessions and what to send back.
- **Gates:** after W4-RT-ROWS and W4-RT-CELLINDEX merge.
- **Hash impact:** none.

### 4.2 W4b: front door, hierarchy, interval matching

W4-0b (ACR-LEVELMAP, ACR-V1, ACR-V3) lands first.

### W4-BOOL-TOOLS (b1), then W4-BOOL-HEAL-A (b2). `sm-boolean`
- **BOOL-TOOLS scope / acceptance:** `drafts/csgbrep.md` §WEBCUT-TOOLS (size from the PROBE report). Unblocks the sphere/cone/torus decks of CSG-CORE and the lens row of CSG-CHECK (a small follow-up in `sm-csgbrep` by the CSG worker).
- **BOOL-HEAL-A scope / acceptance:** `drafts/vtopo.md` §T0 REGULARIZE-OP and §T3 VTOPO-DETECT (`model.heal_check`), acceptance 1-6. Plan delta (ruling D8): thresholds are tuned on the W4-REFCORPUS dirty STEP files, and for each one the codes `model.heal_check` returns equal the manifest's `expected` codes.
- **Hash impact:** none on existing hashes.
- **Plan delta (a1 review):** find the cause of the REGIONS-CUT kernel finding: `webcut_cylinder` / `subtract` fail with `result volume invalid` for a 0.475-radius cylinder cut at 0.418 or 0.476 with the default tool frame, and turning the frame's x reference by 1 rad (SEAM_TURN) avoids it. A coincidence between the tool cylinder's seam and the target's faces is the first suspect. The fix or a documented limit is part of the acceptance, with a test at both cut positions. Also: `subdivide::guard` copies the private `ops::guard_volumes`; make that `pub(crate)` and drop the copy (no behaviour change).

- **Plan delta (a3 review):** two more kernel limits for the same investigation. (1) A plane webcut of a 10-cube fails with `ResultInvalid{rule: "webcut"}` within 1e-3 relative of a face (x = 0.01, 0.001, 9.99; 0.02 works); W4-CSG-CHECK's 1e-3 gap and 1e-4 sliver rows stay `#[ignore]`d and unmet until this is fixed, then the CSG worker un-ignores them. (2) The tall W4-REFCORPUS PWR pin decks are refused by the kernel (W4-CSG-ORACLE pins them as typed refusals). Also: `sm_boolean::locate_point` gives a false `In` for thin annuli.

### W4-VTOPO-OPS. `sm-model`: virtual-topology ops on the layer's overlay
- **Scope / acceptance:** `drafts/vtopo.md` §T1 VTOPO-OPS, acceptance 1-6. **Gates:** after ACR-V1, ACR-V3, and after W4-REGIONS-SPLIT (same crate). **Hash impact:** none on existing.

### W4-MOD-REFINE-U. `sm-modify`: uniform `mesh.refine` with `LevelMap`
- **Scope / acceptance:** `drafts/meshops.md` §MOD-REFINE-U, acceptance 1-7. It builds the D1 pass core (`drafts/meshops.md` ordering delta). **Gates:** after ACR-LEVELMAP. The `.smsh` level-map passthrough in `sm-fem` (`drafts/meshops.md` §MOD-0 follow-up) is done by the W4-EXPORT-SELECT worker, who owns `sm-fem` in W4b. **Hash impact:** none on existing.

### W4-POLY-AGG. `sm-poly`: `mesh.agglomerate` with `LevelMap` (ruling D3)
- **Scope / acceptance:** `drafts/poly.md` §W4-POLY-AGG. Plan delta: it includes the CMFD coarse level of `drafts/meshops.md` Q7 as a **per-pin mode** (one coarse cell per pin, per model volume or per D9 region, by region and a target count), since the transport code will use CMFD/multigrid (ruling D3). Acceptance adds: on the W4 pin lattice, per-pin mode gives exactly one coarse cell per pin region, and the `LevelMap` maps every fine cell to the coarse cell of its pin. **Gates:** after ACR-LEVELMAP and the last a3 `sm-poly` task. **Hash impact:** new op only.

### W4-POLY-SCHEME, then W4-SIZE-IIA-OP. `sm-mesher`
- **POLY-SCHEME:** `drafts/poly.md` §W4-POLY-SCHEME (`mesh.scheme polyhedron`, with `dual`).
- **SIZE-IIA-OP:** the relative-interval op of `drafts/export.md` §SIZE-INTERVALS (`mesh.interval_relation`).
- **Hash impact:** new scheme and op only.

### W4-SIZE-IIA. `sm-sizing` + `sm-scheme-curve`: preference handling
- **Scope / acceptance:** `drafts/export.md` §SIZE-INTERVALS, the solver and curve-scheme parts. **Gates:** census first; the curve-scheme switch from `fix(.., Soft)` to `prefer` is the step most likely to move a count, and any move stops it (C3). **Hash impact:** none intended.

### W4-EXPORT-SELECT. `sm-meshio` + `sm-fem` + `sm-server`: one `select` object for all exporters
- **Scope / acceptance:** `drafts/export.md` §EXPORT-SELECT, plus the `mesh.smsh` level-map passthrough (`drafts/meshops.md` §MOD-0 follow-up; acceptance: a refined mesh exported with `.smsh` reads back with its level map). Plan delta (ruling D9): `select` composes with the `regions` option (a selected subset exports only the regions it contains; the region names do not change). **Gates:** after W4-REGIONS-SPLIT. **Hash impact:** none (absent `select` reproduces current bytes).

### W4-CSG-CLI. `sm-cli` + `sm-mcp` + `sm-py`
- **Scope / acceptance:** `drafts/csgbrep.md` §CSGBREP-CLI. HYGIENE-PY rides along. Plan delta: the CLI also gains the `session.export` `source` flag (`--source model`, ruling D7(b)) and the `regions` option pass-through (ruling D9), each with one test. **Gates:** after CSG-CHECK. **Hash impact:** none.

### W4-RT-HEX. `sm-rt`: hex lattices in both orientations (ruling D8, overrides W3b Q3)
- **Owner:** `crates/sm-rt/`.
- **Scope:** hex lattice support in the world tracker for `LatticeKind::Hex` (x-rows) and `LatticeKind::HexYRows` (ACR-RT-HEXY, W4-0.3): element locate in axial coordinates with cube rounding, a hex DDA stepping element to element across the six faces, the element fit check (each fill inside its hexagonal prism), z stacking with `z_pitch`, and `outer` still refused. The W3b "flat-top through a 30 degree rotated placement" workaround is no longer needed and is not built.
- **Acceptance:** (1) for both kinds, chords along the three lattice directions and along 1 000 seeded random directions cross elements at the analytic distances (hexagon of flat-to-flat pitch `p`) to 1e-12; (2) OpenMC oracle (C7): a test fixture generated with OpenMC 0.16.0 (`gen_hex.py` in `crates/sm-rt/tests/fixtures/openmc_hex/`, committed with its `oracle.json`) for a 3-ring hex lattice of pins in OpenMC orientation `x` and in orientation `y`; at 600 seeded points (5e-3 nudge as in `gen.py`) the element index and the region from the sm-rt world equal OpenMC's `find_cell` after the documented index map from OpenMC's (ring, position) or axial indices to `(q, r)`; the map and which OpenMC orientation equals which `LatticeKind` are written in `crates/sm-rt/DESIGN.md`; (3) a vertex-tie rule (three elements meet) is documented and tested; (4) `flat_golden`, W3b `h_*` and every existing world hash unchanged; (5) determinism.
- **Gates:** after W4-RT-CELLINDEX (same crate).
- **Hash impact:** none on existing hashes (hex worlds were refused before).
- **Plan delta (a2 review):** RT-HEX also takes the RT-CELLINDEX leftovers that need hex-aware code: `on_face_any` (still O(surfaces); accepted and documented in a2), `enter_from_outside` and `Wrap`, and the `locate_candidates` hot spot: in P2 full it is about 977 candidates per segment, because the shared top and bottom z-planes' surface-to-cells CSR lists every cell in `across`; give those planes a per-tile list or skip them in the CSR.


### 4.3 W4c: last sub-wave (a task not started when W4-INT begins moves to W5 without replanning)

W4-0c (ACR-V2a, ACR-EXPFMT, ACR-CSG-HEXIR) lands first.

- **W4-SIZE-CURV** (`sm-sizing`, `sm-mesher`, one line of `standard.rs:184`): `drafts/export.md` §SIZE-CURV. Then the `sm-mesher` worker does **W4-VTOPO-MESH**: `drafts/vtopo.md` §T2 VTOPO-MESH (planar composites; after T1 and ACR-V2a).
- **W4-EXPORT-FLUENT, then W4-EXPORT-CGNS** (`sm-meshio`, wiring in `sm-server`/`sm-cli`): `drafts/export.md` §EXPORT-FLUENT, §EXPORT-CGNS (CGNS starts with the two-cell HDF5 spike). These also cover `drafts/poly.md` §W4-POLY-FLUENT, dropped as a duplicate; "done" is judged by the OpenFOAM and CGNS-library oracles; no Fluent licence (ruling D10). Interchange formats only (ruling D2): the transport code reads `.smsh`.
- **W4-CSG-HEX** (`sm-csgio` reader and IR under ACR-CSG-HEXIR, then `sm-csgbrep`; ruling D8): `drafts/csgbrep.md` §CSGBREP-HEX, scheduled now (no longer on request). Plan delta: both OpenMC orientations, each confirmed against OpenMC (C7) with new `gen.py` fixtures at 600 points; the B-rep import builds hexagonal prisms (`model.create_prism`) and defines a hex lattice frame per deck lattice with the matching `HexOrient` (ruling D9 tags); the tracker import (`ImportedCsg`) and the B-rep agree on every oracle point; a W4-REFCORPUS VTB hex deck imports or fails with a typed error naming the unsupported feature. Acceptance otherwise as the draft (127 prisms for 7 rings, `sqrt(3)/2 p^2` each to 1e-9, `content_hash` unchanged for every existing fixture).
- **W4-HEX-MSWEEP** (`sm-scheme-sweep`): `drafts/hexpin.md` §W4-HEX-MSWEEP acceptance 1, 2, 4 (many-to-many; true many-to-one is W5, ruling D10).
- **W4-HEX-PYR** (`sm-scheme-tet`): `drafts/hexpin.md` §W4-HEX-PYR, half-day spike first (R3); if the spike needs a `GenCtx` ACR, the task returns `blocked_on_api`. Ruling D1: the transport code accepts pyramids, so the task stays in W4c as planned.
- **W4-MOD-HTET, then W4-MOD-REFINE-L** (`sm-modify`): `drafts/meshops.md` §MOD-HTET (with MOD-CONV2's THex/TQuad if time remains) and §MOD-REFINE-L.
- **W4-HEAL-TOL, then W4-STEP-REPAIR**: `drafts/vtopo.md` §T5 HEAL-TOL (`sm-boolean`), then §T5b STEP-REPAIR (`sm-cadio`). Moves to W4b if the W4-REFCORPUS dirty STEP files show open shells block import (ruling D8).
- **W4-EXPORT-ABAQUS** (`sm-meshio`, last): `drafts/export.md` §EXPORT-ABAQUS.

---

## 5. W4-INT. Integration and milestone

Integration agent after the W4c merges (tasks not merged are listed as carried). Owner micro tasks for any crate edit.
1. **Polyhedral milestone:** `tools/w4_poly.smesh` and `crates/sm-cli/tests/milestone_w4_poly.rs`. The OpenMC `pin` deck (fixture bytes, content-addressed) → `model.import_openmc` → blocks per C9 → `mesh.intervals` on the circles → `mesh.auto {strategy: "tet"}` → `mesh.run` → `mesh.to_poly {dual: "circumcentric"}` if the eligibility report says the mesh qualifies, else barycentric (the test asserts which, from the report) → `session.export` `.smsh` (with `gc_*`, ruling D2) and `.smrt`. Checks: per-block volumes equal the inscribed-polygon closed form to 1e-9 (`drafts/csgbrep.md` D7 layer 2); `.smrt` regions equal the block names with no warning; membership at the 600 oracle points equals OpenMC; `mesh.validate` 0 violations; the `.smsh` `gc_*` volumes equal `validate_geometry`'s; CLI, MCP, in-process and Python hashes equal; two runs and `RAYON_NUM_THREADS=1` equal; replay without the strategy.
2. **Pin milestone:** PIN-INT's tests run in the combine (fuel `pi r^2 h` with VOLFIT, ruling D4); both milestone hash sets pinned in `tools/platform/expected.txt` for macOS arm64 and Linux CI (C12).
3. **Regions milestone (ruling D9):** `tools/w4_regions.smesh` and `crates/sm-cli/tests/milestone_w4_regions.rs`: the 3x3 pin lattice of `drafts/regions.md` §W4-REGIONS-SPLIT acceptance 3, with `hex_only` (ogrid on the ring faces where it applies) and VOLFIT, exported to `.smsh` and `.smrt` with `split: ["lattice", "ring", "sector"]`: 117 regions, every fuel region `pi r^2 h / 12` to 1e-12, `.smrt` tags read back through the C ABI (`smrt_region_tag`) in the existing C driver path; and the OpenMC 3x3 deck through `model.import_openmc` gives the same region names for the lattice part. Hashes pinned (C12).
4. **Benchmark smoke** on this Mac under `SMOKE RESULT`: P1 and H1 `reduced`, trackers `smrt,dagmc-moab,dagmc-dd,xdg`, plus `walk`, `jump`, polyhedral (separate table) and P2 `analytic-hierarchy` rows; image identities recorded; nothing under `results/` committed; no ratios. If the user has run W4-BENCH-X86's sessions by then, their table is attached to the report with verdicts computed by the pre-registered code; otherwise the report says the x86_64 table is pending the user's run (ruling D6).
5. **MEASURE table and eligibility rates** in the report, with the orchestrator's decision on VORO-CLIP / CONFORM (trigger-based, ruling D1), on `CONVEX_CHECK_MAX`, and on the circumcentric default (ruling D10).
6. **Census:** `tools/census/census.sh --compare docs/design/w4/census-base.txt`; every difference must be in the D5 list; anything else stops integration and goes to the user.
7. **Goldens:** only catalog goldens that list new ops/schemas, the protocol insertions of the W4-0 ACRs and new pins may change; each diff is listed in the report.
8. **CI:** GitHub CI green on the INT PR (gate and python job) as well as `tools/ci.sh` locally.
9. **PARITY.md:** tick each row only with a test name (polyhedron scheme, CSG to B-rep, hex lattices, multisweep many-to-many, THex/QTri/HTet if merged, refinement, export options, region split, DAGMC from B-rep, Fluent/CGNS if merged, virtual topology rows if merged, curvature sizing).
10. **Report** `docs/design/w4/w4-report.md`: deviations D1..Dn, census, platform table, benchmark smoke, the rulings D1-D10 as applied, the reference corpus summary, carried tasks with their triggers.

---

## 6. Dependencies

```
W4-0: 0.1 census ∥ 0.2 skeletons ∥ 0.3 sm-rt-api ACRs ∥ 0.4 CSGBRIEF ∥ 0.5 sm-protocol ACR ∥ 0.6 sm-model-api ACR ∥ 0.7 docs/tools
 ├─ a1: POLY-MEASURE ∥ POLY-VOLDIAG ∥ RT-XDG ∥ RT-VERDICT ∥ RT-WORLD ∥ HEX-STRAT ∥ HEX-DECOMP ∥ HEX-OGRID-K
 │      ∥ CSG-CLASSIFY ∥ CSG-PROBE ∥ REGIONS-CUT ∥ CLI-DIAG ∥ EXO-VERIFY ∥ DAGMC-MODEL ∥ REFCORPUS
 ├─ a2: POLY-VORO (MEASURE + table review) ; HEX-QUERY (STRAT, DECOMP) ; HEX-OGRID-S (OGRID-K)
 │      CSG-CORE (CLASSIFY, PROBE, 0.4/LIBFN) ; RT-ROWS (RT-XDG, RT-VERDICT, RT-WORLD) ; RT-CELLINDEX (RT-WORLD)
 │      REGIONS-SPLIT (REGIONS-CUT, DAGMC-MODEL, LIBFN) ; A1-TIDY (REFCORPUS, EXO-VERIFY, HEX-DECOMP)
 ├─ a3: MESHER-LISTS (OGRID-S, QUERY; D4, D5; ring check after REGIONS-CUT) → HEX-PIN-INT
 │      CSG-CHECK → CSG-ORACLE (CORE; STRAT; REFCORPUS) ; POLY-MIXED (VORO)
 │      A2-RT (RT-CELLINDEX) ; A2-RTBENCH (RT-ROWS, RT-CELLINDEX)
 ├─ BENCH-X86 (RT-ROWS, RT-CELLINDEX)
 ├─ W4-0b: ACR-LEVELMAP, ACR-V1, ACR-V3
 ├─ b:  BOOL-TOOLS → BOOL-HEAL-A (REFCORPUS) ; VTOPO-OPS (V1, V3, REGIONS-SPLIT) ; MOD-REFINE-U (LEVELMAP)
 │      POLY-AGG (LEVELMAP, a3 sm-poly) ; POLY-SCHEME → SIZE-IIA-OP ; SIZE-IIA ; EXPORT-SELECT (REGIONS-SPLIT)
 │      CSG-CLI (CSG-CHECK) ; RT-HEX (RT-CELLINDEX, A2-RT)
 ├─ W4-0c: ACR-V2a, ACR-EXPFMT, ACR-CSG-HEXIR
 ├─ c:  SIZE-CURV → VTOPO-MESH (VTOPO-OPS, V2a) ; EXPORT-FLUENT → EXPORT-CGNS → EXPORT-ABAQUS
 │      CSG-HEX (CSG-ORACLE, HEXIR, RT-HEX) ; HEX-MSWEEP ; HEX-PYR (spike) ; MOD-HTET → MOD-REFINE-L ; HEAL-TOL → STEP-REPAIR
 └─ W4-INT
```
Cross-area links that are easy to miss: CSG-ORACLE's polyhedral row needs the `tet` strategy (STRAT) or POLY-MIXED; RT-ROWS' polyhedral twin needs the `sm-poly` dependency ruling (W4-0.7); the CSG sphere-lens row needs BOOL-TOOLS; POLY-AGG and MOD-REFINE-U share ACR-LEVELMAP; VTOPO-MESH needs ACR-V2a; `smtets_to_moab` lives only in RT-XDG (`drafts/export.md` defers to it); REGIONS-SPLIT, CSG-CORE and CSG-HEX all write the ACR-REGIONS tables, so the naming of W4-0.4 binds all three; CSG-HEX compares against RT-HEX's orientation map.

---

## 7. Hash census and merge rules

- **C3 carries over.** Any move of an existing hash stops the task and goes to the user; nothing is re-blessed by a worker. The population is `docs/design/w4/census-base.txt` (W4-0.1, measured at `69ba521` by `tools/census/census.sh`; `69ba521` is docs-only over `9cac369` and the values are identical): the 62 data lines (plus 1 comment line) of `tools/platform/expected.txt` (W2, W3a, W3b, `w3c_pin`), the `milestone_w3c_pin` asserted counts (432 hexes, 733 tets, two `mesh.auto_tet` warnings), `SUBMAP_GRADED`, `p0_rows`, `crates/sm-rt/tests/flat_golden.rs` (per os-arch, C11), the existing lines of `crates/sm-rt-bench/expected_sha256.txt`, the export byte-identity test `each_format_is_byte_identical_in_process_cli_and_mcp` and the format goldens, the `tests/dagmc.rs` goldens, the sm-poly counts (375 at `crates/sm-poly/tests/acceptance.rs:93`) and `acc7_determinism`, the conformance fixtures `polydual` and `w3a_poly`, the sm-csgio `oracle.json` files and `content_hash` goldens.
- **Combine reports.** Every future combine (sub-wave or INT) reports the output of `tools/census/census.sh --compare docs/design/w4/census-base.txt` (the count of differing keys and each one), not only the `smesh state` hashes.
- **Backlog (CI):** CI should print the platform values (the recomputed `tools/platform/hashes.sh` lines) in its log, so the Linux census rows can be compared by value and not only by pass/fail.
- **Planned moves:** only ruling D5, as one commit in W4-MESHER-LISTS that lists every moved line: the through pin, its lattice, B5 and the leftout fixture (`tools/platform/expected.txt:54-63` and three tests). W2, W3a and W3b hashes stay. No other move is planned in W4.
- **Allowed changes that are not hash moves:** catalog/schema goldens that gain entries for new ops, reviewed by the orchestrator at merge; protocol golden insertions of the W4-0 ACRs; new pins added by INT (macOS arm64 and Linux CI; facet and tessellation bytes per os-arch).
- **Opt-in rule.** Every geometry-changing feature in W4 is opt-in: new op arguments are omitted from canonical params when they hold the default (`drafts/poly.md` hash exposure), so a script that does not use them hashes as before. New model and layer tables (ACR-REGIONS, ACR-LEVELMAP, ACR-V1) enter digests only when non-empty. Exception by ruling: D5.
- **Merge rule.** A sub-wave (0, a, b, c) and W4-INT merge only with GitHub CI green on the PR (gate and python job) as well as the local `tools/ci.sh` (rulings Process). A red CI is recorded with its cause, never reported as green.

---

## 8. What moves out of the scheduled sub-waves, and why

**Trigger-based (in W4 if the trigger fires before W4-INT, else W5):**
- `drafts/poly.md` §W4-POLY-VORO-CLIP and §POLY-CONFORM (`sm-scheme-tet`): ruling D1 keeps them trigger-based (orthogonality is a goal, not a hard requirement); trigger is MEASURE's eligibility rates: refusals dominated by outside circumcentres → VORO-CLIP; by non-Delaunay interior faces → CONFORM (scope approved under ruling D10).
- `CONVEX_CHECK_MAX` raise in sm-rt (`drafts/poly.md` Priority): MEASURE shows many convex cells over 64 faces.
- `drafts/poly.md` §W4-POLY-SHAPES: VORO shows hex-lattice outputs matter (the `.smsh` rule change is then a format question for the user).
- `drafts/vtopo.md` §T2b, §T4, §T6, §T11: the W4-REFCORPUS dirty STEP and VTB models show periodic composites, short edges, holes or blends dominate.
- `drafts/rt.md` `xl` tier: after the user's first x86_64 session (W4-BENCH-X86) shows the machine holds it.

**W5+ (not scheduled), with the reason:**
- POLY-RVD, POLY-POWER (`drafts/poly.md`): new constructions beyond W4's dual; VORO first.
- POLY-BULK (`drafts/poly.md`, an ACR): no 10^6-cell poly requirement yet.
- Sweep-order data for Sn (`drafts/poly.md`): the transport code has not asked.
- Volume-matched tessellation in `sm-tess` (ruling D4 option (c)): W5 task; VOLFIT covers the structured pins in W4.
- Importing other formats into `.smsh` (ruling D2): W5, no trigger yet.
- STL import and export: `drafts/vtopo.md` §T7 STL-IN and T7b STL-WIRE move to W5 (ruling D7(a)): STL loses material identity and is already faceted. Trigger: a concrete need for STL input.
- T8a H5M-READ-OWN and T8b foreign `.h5m` (`drafts/vtopo.md`): no existing `.h5m` models (ruling D7(c)); we generate our own with OpenMC, DAGMC tools and our exporter.
- T10 IGES (ruling D7(d)): W5+. T9 STL-BREP, T12 DEFEAT-BLEND (`drafts/vtopo.md`): mesh-based geometry stays deferred (PARITY:91); blends need SSI maturity.
- A local-model benchmark table: not planned, since there is no shareable local model (ruling D8); the public corpus is the evidence.
- HEX-VSUBMAP, the general multisweep decomposer, sweep columns with intermediate vertices, true many-to-one sweep (union-copy ACR, ruling D10), `collar`, sphere O-grid (`drafts/hexpin.md`): pins and assemblies are sweepable after DECOMP; none is on the user's path.
- MOD-ELEVATE, MOD-BL (`drafts/meshops.md`): high order and boundary layers in W5 (ruling D10); the walker and the transport code take linear cells.
- MOD-PILLOW, MOD-COARSEN (`drafts/meshops.md`): no hex defect; coarsening covered by POLY-AGG.
- X-2 volume copy scheme (`drafts/meshops.md`, an ACR): no case is slow today; the 3x3 lattice meshes now.
- Nastran and the P2 formats, proximity and skeleton sizing, periodic intervals (`drafts/export.md`): no reader or measured need.
- GPU smoothing and quality (`drafts/export.md` D5): bit-identical output cannot hold on Metal (no `f64`) and no bottleneck is measured (ruling D10).
- Model instancing for lattices (memory sharing; the D9 provenance tables do not need it), edit-one-cell-and-regenerate, DAGMC universe import, periodic boundaries, ellipsoid/paraboloid faces as rational NURBS (`drafts/csgbrep.md`): each needs a contract or a W5 editing model.
- Region split for DAGMC and the other exporters (`drafts/regions.md` R3): `.smsh` and `.smrt` only in W4.
- OpenMC plugin for end-to-end runs (`drafts/rt.md` Q4; ruling D10), XDG's GPRT backend, a libMesh backend: W-flight is the W4 transport result.

---

## 9. Answered

The plan's open questions D1-D10 were answered by the user on 2026-11-09; the answers are in `rulings.md` and are applied throughout this revision (each place cites "ruling Dn"). Questions answered without the user (orchestrator calls) are recorded by W4-0.7 in `rulings.md` ("Orchestrator rulings") and in the drafts' critique deltas: overlap default `error` (W3b Q4); the import makes geometry, blocks and sidesets only; pre-registered win criteria and default-build headline; healing home, tolerance-first healing and virtual collapse; `renumber` dropped from export selection; sagitta-primary curvature sizing with angle as a cap; hygiene ownership by crate; mesh copy via a volume copy scheme rather than a post-hoc op (and in W5); `sm-rt-bench` dependencies; XDG pin bump (C15); D9's storage and naming (`drafts/regions.md`).
