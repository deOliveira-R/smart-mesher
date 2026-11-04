# W3b plan: transport handover, delta and analytic tracking, auto meshing, exporters, Python, viewer

Base: `main` at `f9f6246` (the post-W3a contract batch 55a20a9 and the follow-ups foam2d, tet, server, cadio, poly are merged). Rulings: `docs/design/w3b/rulings.md` (Q1-Q14 binding; P1-P5 proposals awaiting the user's OK). Inputs: the W3b drafts, the three rt-api panel designs and the planning critique (scratchpad `w3b/plan-result.json`); the rulings override them. The W3a plan (`docs/design/w3/plan.md`) is the structural model; decision 12 and the W3a rulings stay binding unless a W3b ruling says otherwise.

Milestone (§4): a pin-cell / small-lattice model is built with the new primitives and `model.copy`, auto meshed with the `default` strategy (the history holds explicit scheme assignments), surface-meshed and explained with `mesh.plan`, exported to DAGMC h5m and Gmsh msh4 (and the model to STEP through `model.export`, registered by W3b-MODEL-EXPORT or W3b-CAD-ATTR once O5 is answered), written as the tracking handover file and opened by a C test program through `libsmrt` that runs surface and delta flights with its own cross sections, recorded as a script and replayed, driven from Python, and shown in the web viewer. CLI, MCP, in-process and Python give the same state hashes. Which session surface writes the handover file is O1; until O1 is answered the milestone uses the fallback in §5.1 (the INT harness calls `sm_rt::write_smrt_file`).

---

## 1. Ownership

| Task | Crate / dir | Layer | Wave | Owns |
|---|---|---|---|---|
| W3b-0 | orchestrator | C, all | 0 | `*-api`, `sm-testkit`, root `Cargo.toml`/`Cargo.lock`, CI, skeletons, docs |
| W3b-SERVER0 | `sm-server` | L5 | 1 | `crates/sm-server/` (small: per-session pending store) |
| W3b-PRIM | `sm-model` | L2 | 1 | `crates/sm-model/` |
| W3b-MODEL-EXPORT | `sm-cadio` | L2 | 1 | `crates/sm-cadio/` |
| W3b-SCRIPT | `sm-lang` | L4 | 1 | `crates/sm-lang/` |
| W3b-EXPORT | `sm-meshio` | L2 | 1 | `crates/sm-meshio/`, `tools/conformance/check_{h5m,msh4}.py`, `tools/conformance/dagmc/` |
| W3b-SURFPLAN | `sm-mesher` | L3 | 1 | `crates/sm-mesher/` |
| W3b-VIEW | `sm-view` | L3 | 1 | `crates/sm-view/` (incl. `web/`) |
| W3b-PY | `sm-py` | L6 | 1 | `crates/sm-py/` |
| W3b-RT-WORLD | `sm-rt` | L3 | 1 | `crates/sm-rt/` |
| W3b-SOS | `sm-math` | L0 | 1 | `crates/sm-math/` |
| W3b-QUALITY | `sm-quality` | L2 | 1 | `crates/sm-quality/` |
| W3b-SMOOTH | `sm-smooth` | L3 | 1 | `crates/sm-smooth/` |
| W3b-PAVE | `sm-scheme-pave` | L3 | 1 | `crates/sm-scheme-pave/` |
| W3b-SWEEP | `sm-scheme-sweep` | L3 | 1 | `crates/sm-scheme-sweep/` |
| W3b-BOOL | `sm-boolean` | L2 | 1 | `crates/sm-boolean/` |
| W3b-RTBENCH-A | `sm-rt-bench` | L4 | 1 | `crates/sm-rt-bench/` (stub-only parts) |
| W3b-RT-DELTA | `sm-rt` | L3 | 2 | `crates/sm-rt/src/delta/` + registration lines |
| W3b-RT-ANALYTIC | `sm-rt` | L3 | 2 | `crates/sm-rt/src/analytic/` + registration lines |
| W3b-RT-CAPI | `sm-rt-capi` | L4 | 2 | `crates/sm-rt-capi/` |
| W3b-AUTO | `sm-mesher` | L3 | 2 | `crates/sm-mesher/` |
| W3b-EXPORT-WIRE | `sm-server` (+ micro edits) | L5 | 2 | `crates/sm-server/`; one-line edits in `sm-cli`, `sm-mcp` |
| W3b-DIAG | `sm-diag` | L3 | 2 | `crates/sm-diag/` |
| W3b-CAD-ATTR | `sm-cadio` | L2 | 2 | `crates/sm-cadio/` |
| W3b-FOAM-WARP | `sm-meshio` | L2 | 2 | `crates/sm-meshio/` |
| W3b-SUBMAP | `sm-scheme-submap` | L3 | 2 | `crates/sm-scheme-submap/` |
| W3b-RT-DELTA-WORLD | `sm-rt` | L3 | 3 | `crates/sm-rt/` |
| W3b-RT-BRIDGE | `sm-rt` | L3 | 3 | `crates/sm-rt/` (after DELTA-WORLD) |
| W3b-SERVER-HOST | `sm-server` (+ micro edits) | L5 | 3 | `crates/sm-server/`; one-line edits in `sm-cli`, `sm-mcp` |
| W3b-RTBENCH-B | `sm-rt-bench` | L4 | 3 | `crates/sm-rt-bench/` |
| W3b-INT | integration agent | L3-L7 | INT | see §4 |

Rules for every task (CLAUDE.md applies): own only the listed directory; frozen contracts change only through W3b-0 ACRs, else `status: blocked_on_api`; no `HashMap` order in output; determinism test (two runs, `RAYON_NUM_THREADS=1` vs default); property tests with `sm_testkit::strategies::config(n)`; rustdoc on every public item; no wall-clock gates (work counters only); no `unwrap` on input paths; no MCP/CLI golden edits (W3b-INT regenerates them).

---

## 2. Rulings (from `rulings.md`; ids are cited in the task blocks)

| Id | Ruling | Consequence in this plan |
|---|---|---|
| Q1 | rt-api base fd4a235 plus grafts: opaque resumable cursor, C ABI from 30e2742, deduplicated material sets, `block(node)`, face_in/out, safety distance plus `REGION_JUMP`. Losers archived as tags. | W3b-0 merges `w3b-panel/rt-1` and applies the grafts before freezing (§3.2 RT-API). |
| Q2 | Cross sections never enter smart-mesh or sm-rt. smart-mesh exports a file (geometry, connectivity, region material names); the transport code opens it through sm-rt (C ABI) and pairs names with its own cross sections. sm-rt reports the materials of each spatial region; the caller passes one majorant per region; a collision with caller σ_t above it is an error. `BuildOptions` stays `Copy`. | Dropped: table bound to `structure_hash`, `from_flat`, surface-mode σ_t(E) sampling, the `(set, band)` `MajorantTable`, `mean_total`, `ModePolicy::Auto`. New: per-node majorant slice passed per segment call, `check_collision`. Handover file is a deliverable (RT-WORLD reader, RT-CAPI `smrt_open`); its format and the surface that writes it are open (O1). If O1 picks `session.export`, a conditional sm-protocol ACR adds `ExportFormat::Smrt` (§3.2 Protocol); until then the milestone uses the §5.1 fallback. |
| Q3 | Hex lattices stretch, pointy-top only; flat-top by a 30° rotated placement; no HexY. ANALYTIC acceptance 9 is stretch. Hex assemblies mesh with the existing sweep. | No HexY ACR. Hex DDA is RT-WORLD stretch S1. |
| Q4 | CSG cells must be disjoint. First-wins and the stub's Union priority rule are removed; stub tests that rely on them change; golden changes need a bless ruling. A Check-stage op samples points and rays for overlaps. MCNP/OpenMC importers reject overlapping decks. | §3.2 RT-API (Q4 part); `overlap-check` node; library `check_overlaps` in RT-ANALYTIC, session op `rt.check_overlaps` registered by RT-BRIDGE (the first task that can build CSG leaves from a session); O4, O6, O7. |
| Q5 | `mesh.auto {strategy}` with registered strategies; `default` = surfaces map/submap/pave/trimesh, volumes map/sweep/tetmesh with `mesh.auto_tet` warning, triangle sources pave. Strategies only choose schemes and record explicit assignments into history; replay never re-runs a strategy; explicit-scheme runs stay bit-identical; every changed case is reported. | sm-op-api canonical-record ACR (§3.2); AUTO acceptance A7-A9; `scheme-auto` node. |
| Q6 | DAGMC h5m: download a small MOAB-written two-volume h5m with graveyard (licence-checked), commit with its h5dump; `mat:<set name>`, `mat:Graveyard`, graveyard opt-in; msh4 ASCII default, binary option; polyhedra to msh4 an error by default. | W3b-0 item 0.9; EXPORT decisions. |
| Q7 | Python: package `smartmesh`, Python ≥ 3.10 abi3, not on PyPI, no zero-copy views. CLAUDE.md allows PyO3 macro-generated `unsafe` in `crates/sm-py` (ddc2122). | PY task; no further CLAUDE.md edit. |
| Q8 | Web viewer: WebGL2, renderer-neutral buffer contract, client in `crates/sm-view/web/`, read-only plus script console, Node CI job non-blocking. | VIEW task; W3b-0 CI item. |
| Q9 | Published benchmark table from one bare-metal Linux x86_64 machine arranged by the user; macOS is smoke only. Synthetic fusion-like model in the repo; `--model <path>` takes a local STEP file, never committed. | RTBENCH-A/B; O11, O12. |
| Q10 | STEP export is the Model-stage op `model.export` (via `sm_cadio::write_step`). Mesh export wiring (h5m, msh4) is one Export-stage unit (protocol, server, cli, lang). Two tasks. | MODEL-EXPORT (library, report, tests and, once O5 is answered, the registration in sm-cadio; CAD-ATTR carries the registration if O5 is answered after MODEL-EXPORT merges) and EXPORT-WIRE; protocol `ExportFormat::{Dagmc, Gmsh}` only; O5. |
| Q11 | New `model.create_sphere_part` (`create_sphere` unchanged); copies carry no names/colors/groups/boundary kinds; prism/pyramid radius is the circumradius, vertex 0 on +x; `move_to` anchors on the selection centroid. | PRIM. |
| Q12 | Script record: `Check::Tips` default; `Forks::Head` on a non-main head leaves a documented extra `main`; no messages or authors. | SCRIPT. |
| Q13 | Surface-only `mesh.run` next to a face meshed in another entry is refused with an error naming the fix. | SURFPLAN guard 2. |
| Q14 | Ratify the provisional transport quality thresholds; `mesh.smooth {set}` reuses `flag_cells` set names; tet size floor stays a reported preference; STEP names become groups first, colors only from solid-attached colors; overlay `MeshRead` stays deferred. | QUALITY, SMOOTH, CAD-ATTR; overlay MeshRead not scheduled. |
| P1 | Firmness cascade at command time; flip `mesh.run` only after a milestone A/B with unchanged hashes. | **Pending.** Planned in W3b-0 item 0.8; nothing starts until OK. |
| P2 | Re-bless the `mesh.quality` schema golden once for `scheme_notes`/`n_scheme_notes`. | **Pending.** No bless until OK. |
| P3 | Scheme notes to an unhashed `MeshEntry::notes`; until then `mesh.to_poly` clears `plan.notes`. | **Pending** (both parts). SURFPLAN reads `entry.plan.notes` meanwhile. |
| P4 | sm-op-api `ApplyCtx::retire_range` with hash = per-id retire test. | **Pending.** |
| P5 | sm-protocol `session.export` gets `split_small_determinant` (default true) and empty/wedge patch naming. | **Pending.** |

---

## 3. W3b-0 (orchestrator, on main before workers start)

### 3.1 Order
1. **0.1 Merge fd4a235** (`w3b-panel/rt-1`) onto main as the rt-api base (Q1). It is additive (new `segments.rs`, `NodeId`, `MaterialSetId`, caps 128/256, `-12`, stub); `BuildOptions` untouched.
2. **0.2 Contract batch** (§3.2), one commit per crate, then the whole workspace `cargo test` plus the W2/W3a milestone hashes (must be unchanged except where §3.2 says "needs bless").
3. **0.3 sm-rt-capi `-12` mapping** in the same merge batch as rt-api (the capi status-code test fails otherwise): `SMRT_ERR_MAJORANT = -12` in the header and the code table; the new `LostCause::{SenseMismatch, Overlap}` values in the C lost-cause enum.
4. **0.4 Skeletons**: `crates/sm-py` (L6; `crate-type = ["cdylib","rlib"]`, feature `python = ["dep:pyo3","dep:numpy"]`, root pins pyo3/numpy abi3-py310), `crates/sm-view` (L3; `web/` dir), `crates/sm-rt-bench` (L4, `publish = false`, forbids `unsafe`). Compile-only probe `cargo check -p sm-py --features python`: `Server: Send` holds and pyo3 macro `unsafe` passes under the Q7 exception; if `Send` fails, fix in sm-server/sm-op-api before PY starts.
5. **0.5 CI**: a maturin/pytest job calling `crates/sm-py/ci/py.sh`; a Node job for `crates/sm-view/web` (non-blocking, Q8). `tools/ci.sh` stays free of `--all-features`.
6. **0.6 Docs**: ARCHITECTURE.md crate map: sm-py at L6; sm-view at L3 with the client in `crates/sm-view/web/` (replaces `frontend/`), WebGL2 per Q8; sm-rt-bench at L4. This plan's graph delta (concepts.yaml, stages.yaml, .mmd) lands with it.
7. **0.7 Backlog cleanups** (2026-11-04 section), owner per item:
   - sm-scheme-api (orchestrator, frozen): `SchemeNote` `#[non_exhaustive]` plus constructor; `MeshIntent::set_intervals` running counter (O(n²) today).
   - sm-server `host.rs` `notes_of` above `write_dir`'s doc comment: W3b-SERVER0.
   - sm-mesher DESIGN.md "Scheme notes" stale text: W3b-SURFPLAN. sm-diag dev-deps comment: W3b-DIAG.
   - `ExportNote::parse` loses the code when the count is missing (sm-protocol, orchestrator).
   - sm-meshio `tests/foam_2d.rs` comment, FOAM_FORMAT.md §8 list and §11 follow-ups: W3b-EXPORT.
   - sm-scheme-tet (n=64 cliff test release-only, hint-index counter test, record path B SJ 0.1848): orchestrator micro task in W3b-1 (no other tet work in W3b).
   - sm-cadio (refusal tests bound parse work, per-run Newton cost counted, `global_projections <= MAX_GLOBAL`, `work` name clash, no global atomics on the public API): W3b-MODEL-EXPORT.
   - sm-poly (`UnrepairedCell` → `OpError::Failed`; `tests/robust.rs` asserts `region`; post-agglomeration validity check): orchestrator micro task in W3b-1.
   - a268047 trailer: process note only.
8. **0.8 Pending proposals P1-P5**: drafted as ACR texts in `docs/design/w3b/pending.md`, not applied; no task depends on them; no golden is blessed on them.
9. **0.9 DAGMC reference (Q6)**: download a MOAB-written two-volume h5m with graveyard from DAGMC or OpenMC test data, record source URL, commit and licence in `tools/conformance/fixtures/ref/README.md`, commit the file and `h5dump -H` text. If the licence does not allow committing, stop and ask (O15). EXPORT's h5m part starts after this.
10. **0.10 Server thread-local** `PENDING` (`crates/sm-server/src/standard.rs:114`) survived server-0: W3b-SERVER0 fixes it in W3b-1; PY acceptance 7 and 8 gate on it.

### 3.2 Consolidated frozen-contract changes

**RT-API (`sm-rt-api`; fd4a235 plus Q1 grafts, rewritten for Q2 and Q4).** These items are new since main, so fd4a235's own types may still change before the freeze.
- *Majorants (Q2).* Remove `EnergyBands`, `MajorantEntry`, `MajorantTable` (`from_fn`, `(set, band)` key, `efficiency`, `check`), `ModePolicy::Auto` and the table argument of `choose_mode`. The caller passes `majorants: &[f64]` indexed by `NodeId` (length `HierarchyView::node_count()`; finite, ≥ 0, else `InvalidInput`) on every segment call, inside `FlightInput` (below). Leaves use their own entry; internal nodes are not read. sm-rt holds no σ_t and keeps no copy of the slice. Whether a band axis survives is O2; the default is one slice per call, so the caller changes energy by passing another slice.
- *Per-call flight input.* `#[derive(Clone, Copy, Debug)] #[non_exhaustive] pub struct FlightInput<'a> { majorants: &'a [f64], block: &'a [NodeId] }` with `new(majorants)` and `with_block(block)`, taken by `next_segment`, `probe` and `stop_at`. `block` (blockdt: force surface in those nodes; sorted, deduplicated, else `InvalidInput`) is passed per call like `majorants`, so the cursor never holds a borrowed slice and needs no heap copy. The slices may differ between calls; each segment uses the slices of the call that produced it.
- *Collision check (Q2).* `FlightCursor::check_collision(&self, sigma_t: f64) -> Result<()>` takes no slice: it compares against the majorant of the current segment's node, cached in the cursor by the last `next_segment` (`InvalidInput` before the first segment or after the end). `Err(RtError::MajorantViolated { node: NodeId, sigma_t: f64, majorant: f64, pos: Point3, cell: CellId })` when `sigma_t > majorant * (1 + 1e-12)`; code -12; never clamped; state not advanced. The variant (new in fd4a235, fields `set, band, sigma_t, majorant`) is redefined with these fields before freeze.
- *Opaque resumable cursor (Q1).* `pub struct FlightCursor` (private fields, no borrow, no heap at depth ≤ `LEVEL_INLINE`) replaces the lending `Flight<'a>` borrow: `MajorantTracker::begin(&self, st: &TrackState, o: &FlightOptions) -> Result<FlightCursor>`; `FlightCursor::{next_segment(&mut self, t: &dyn MajorantTracker, input: FlightInput<'_>, st: &mut TrackState, s: &mut Scratch) -> Result<Option<FlightSegment>>, probe(..., input: FlightInput<'_>, t: f64) -> Result<Located>, stop_at(..., input: FlightInput<'_>, t: f64) -> Result<Located>, set_policy(ModePolicy) -> Result<()>, check_collision(f64) -> Result<()>, safety_distance(...) -> Result<f64>, distance() -> f64, end() -> Option<&TraceEnd>}`. A thin borrowing `Flight` wrapper may stay for Rust callers. The safety cache lives in the cursor, not in `TrackState` (equality unchanged). The trait-level `SegmentQuery<'a>` loses `band` and `table` and gains `input: FlightInput<'a>` (`policy`, `scale`, `max_dist` stay).
- *fd4a235 types under Q2 and O2.* `FlightOptions` stays `#[derive(Clone, Copy, Debug, PartialEq)]` with no lifetime: fields `max_dist`, `max_steps`, `stop_at_deck`, `policy`, `scale`; **`band` is removed** (one slice per call; if O2 keeps a band axis, the band becomes a per-call field of `FlightInput`, not a flight option). `SegmentScale::{Cell, Region}` stays unchanged (σ-independent; `Region` needs `REGION_JUMP`). `FlightTally` stays: `score_track` uses lengths only and `score_collision(cell, weight, sigma_t)` takes the caller's σ_t, so it holds no cross section and is Q2-clean.
- *Material sets (Q1, Q2).* Sets deduplicated in CSR with `BTreeMap` ids (ascending first appearance); `HierarchyView::{set_regions(MaterialSetId), node(NodeId).set}` report the regions of each node; region names (`RegionInfo.name`) are the material names the caller pairs with its cross sections. `validate_partition` also checks containment (a node's set covers every region below it).
- *Modes (Q1).* `ModePolicy::{Surface, Delta}` plus `FlightInput::block` (above). Hybrid is caller-driven per segment through `set_policy` and `block` (O3). Surface segments: one cell; `sigma_m` is the node majorant as a bound only; the caller samples with its own σ_t(region). `FlightSegment.face_in/face_out` and `end` stay as in fd4a235.
- *Safety and region jump (Q1).* `MajorantTracker::safety_distance` (provided, `Ok(0.0)` valid) and `TrackerCaps::{MAJORANT = 128, REGION_JUMP = 256}` as in fd4a235. Doc clause on `Tracker::fire`: an override that jumps regions returns the same `StepEnd` variant, ids and `to_cell`, distance within `1e-12 (1 + d)`; `step`/`trace` never jump.
- *Counters.* `#[non_exhaustive] #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)] pub struct WorkCounters { steps, locates, segments, surface_segments, delta_segments, node_visits, face_tests, surface_evals, locate_candidates, bvh_descents, lattice_index_locates, slow_locates, nudges, tentative, rejected, safety_skips, jumps: u64 }` with `add`; `Scratch.counters` (kept by `Scratch::clear`), `Scratch::clear_counters`. Replaces `FlightStats`.
- *Partition options.* `#[derive(Clone, Copy, Debug, PartialEq)] #[non_exhaustive] pub struct PartitionOptions { enabled: bool = false, leaf_cells: u32 = 8, max_depth: u32 = 24, region_jump: bool = false }`; `BuildOptions.partition` hashed only when non-default (W3a cache keys unchanged); `with_partition`. **`BuildOptions` stays `Copy`**: `tests/freeze.rs` gains `fn _copy<T: Copy>() {} _copy::<BuildOptions>(); _copy::<PartitionOptions>(); _copy::<BuildInput<'_>>();`.
- *Hierarchical input.* `#[derive(Clone, Copy)] #[non_exhaustive] pub enum LeafInput<'a> { Mesh(MeshInput<'a>), Facets(&'a FacetInput), Csg(CsgLeafInput<'a>) }`; `#[derive(Clone, Copy)] #[non_exhaustive] pub struct HierInput<'a> { spec: &'a WorldSpec, leaves: &'a [LeafInput<'a>], boundaries: &'a BoundarySpec }` with `new` (the derive is required: `BuildInput` is `#[derive(Clone, Copy)]`, input.rs:447, and the freeze test asserts `_copy::<BuildInput<'_>>()`); `BuildInput::Hier(HierInput<'a>)`; `cache_key` mixes `WorldSpec::content_hash` and each leaf key. `BuildInput::World` stays `Unsupported`.
- *CSG leaf (Q4).* `pub struct CsgRegion { terms: Vec<CsgCell> }` (union of intersections; `new`, `everything`, `single`, `content_hash(&dyn GeomStore)`); `pub struct CsgCellDef { region: CsgRegion, name: String, volume: Option<f64> }`; `#[derive(Clone, Copy)] pub struct CsgLeafInput<'a> { geom: &'a dyn GeomStore, cells: &'a [CsgCellDef], bounds: Option<Aabb>, complement: &'a ComplementPolicy }`. No `disjoint` flag and no order semantics: cells must be disjoint, and a point is never resolved by order.
- *Overlap reporting (Q4).* Build and check time: `RtError::Build` issues `csg.overlap` (CSG cells) and `world.overlap` (Union placements), each naming both cells/placements and a point. Track time (`locate`, `step`, `fire`, `next_segment`): new `LostCause::Overlap` (next to `SenseMismatch`); the step ends `Lost`, state unchanged, never retried by nudging; `locate` reports it on the same path as `NoCellContainsPoint`. No new status code.
- *Disjoint placements (Q4).* `world.rs` `ComponentKind::Union` variant doc (line 146, "where placements overlap the lowest index wins") becomes "placements must be disjoint; overlapping placements are a build error", and the `placements` field doc (line 148, "Placements, in priority order") becomes "Placements; order carries no priority". Stub `AnalyticWorld`: shapes must be pairwise nested or disjoint; cell c is shape c minus the shapes strictly inside it; partial overlap is `InvalidInput` at build. Stub docs that encode first-wins and must change: the module doc (stub.rs:13, "the first shape containing a point wins, so list inner shapes first"), the `simple()` doc (stub.rs:359, "priority order, inner first") and the region-across-surface comment (stub.rs:461, "the first *other* shape containing a point"). Stub tests that change: `cell_volumes_are_exact_for_nested_shapes_and_partition_the_domain` (its partly-overlapping sub-case becomes a build error). Re-run and report, change only if they move (`STUB_SHELLS` is nested shells, so these are expected unchanged): `nested_shells_partition_the_outer_chord`, `fire_stops_at_region_boundary_and_peek_agrees`, `dagmc_compat_against_the_analytic_sphere`, `structure_hash_is_stable_and_sensitive`, `build_then_load_gives_the_same_structure_hash`, `bundle_hash_is_equal_for_one_and_four_threads_and_tally_sums_to_path`, `tests/freeze.rs` `non_finite_placement_is_reported`, `sm-rt/tests/chords.rs`, every `STUB_SHELLS` user (`sm-rt-capi/src/ffi/test_hooks.rs`, `sm-rt-capi/tests/abi.rs`, `sm-rt-capi/tests/bundle.rs`, `sm-rt-capi/tests/c/chords.c`, `sm-rt-capi/tests/c/smrt_test.h`), `DESIGN.md` line 70. Any hash or golden that moves: stop, list it, bless ruling (O4).
- *DAGMC-compat and loading (TRIAGE R1-R4, R6).* Provided `Tracker::surface_area(SurfaceId) -> Result<Option<f64>> { Ok(None) }`, `Tracker::surface_faces(SurfaceId) -> &[FaceId] { &[] }`; `RegionInfo.tags: Vec<(String, i64)>`; `LostCause::SenseMismatch` (and `LostCause::Overlap`, Q4 above); `LostInfo.retries: u32`; `TrackerBuilder::load_image(&self, &CacheImage) -> Result<(Box<dyn Tracker>, BuildReport)> { Err(Unsupported) }` (opens a tracker without the build input; the Q2 reader path).
- *Stub.* A stub majorant world with heterogeneous node sets (surface and delta mixed in one flight) and an understated-majorant variant for the -12 test; tests in `tests/segments.rs` rewritten for the per-node slice.

**Protocol (`sm-protocol`).** `ExportFormat::{Dagmc /* .h5m */, Gmsh /* .msh 4.1 */}` (non_exhaustive, additive; no `Step`, Q10); `Request::Export.options: Option<serde_json::Value>` (`#[serde(default)]`, per format, validated by the exporter: gmsh `{binary, polyhedra}`, dagmc `{graveyard}`; no material-name prefix option, names are fixed by Q6); extension inference `.h5m`, `.msh`. **Conditional (O1):** if O1 picks `session.export` as the surface that writes the tracking handover file, W3b-0 adds `ExportFormat::Smrt /* .smrt */` (additive; extension `.smrt`) in a follow-up ACR batch and EXPORT-WIRE (or SERVER-HOST, if EXPORT-WIRE has merged) dispatches it to `Tracker::export_cache` and `sm_rt::write_smrt`; if O1 picks an `rt.*` op, RT-DELTA-WORLD registers it in sm-rt; if O1 picks the CLI only, SERVER-HOST adds the `smesh` subcommand. Not scheduled until O1 is answered; the milestone uses the §5.1 fallback meanwhile. **No server-push messages:** the envelope rejects id-less messages (`lib.rs:12`, `envelope.rs`), and W3b adds none (see O16); live clients poll `session.hash`. `Request::ScriptRecord { forks: RecordForks /*head|refs*/, check: RecordCheck /*none|tips|every, default tips*/, verify: bool, raw: bool }` → `ScriptRecordResult { text, nodes, statements, refs: BTreeMap<String, Hash256>, requires: Vec<String>, notes: Vec<String>, verified: Option<bool> }`; `METHODS` gains `"script.record"` (sorted). `ExportNote::parse` fix (0.7). P5 is pending.

**Op API (`sm-op-api`, Q5).** Plugin ops may supply their canonical stored record: `ApplyCtx::set_canonical_record` becomes `pub` (today `pub(crate)`, used by `core.batch`), documented: "the stored record must replay to the same delta without consulting anything the original apply consulted". Test: an op that stores a rewritten record replays from the stored form; hash equal. Alternative if refused: the host expands `mesh.auto` into a `core.batch` of `mesh.scheme` records before apply (orchestrator decides at W3b-0; AUTO is written for the ACR). P4 is pending.

**Mesh API (`sm-mesh-api`).** Provided `MeshEdit::remove_set(&mut self, id: MeshSetId) -> Result<(), MeshError> { Err(Unsupported) }`, implemented in `sm-mesh` by the orchestrator (for `mesh.unflag_cells`). Requirement on that implementation (DIAG acceptance 2 depends on it): adding a set and then removing it leaves the mesh content, its set list and the state hash equal to before the add (no id counter or tombstone enters the hash); W3b-0 tests it in sm-mesh.

**Layer API (`sm-layer-api`).** `ModelLayer::from_parts(host: TopoHost, model: Model) -> ModelLayer` (backlog; sm-fem tests).

**Scheme API (`sm-scheme-api`).** 0.7 cleanups only. `PlanCtx::tolerance` and `Patch::strip_index` (TRIAGE R6/R7) wait.

**Topo API.** `sample_face` pcurve use (TRIAGE R9) is a behaviour change: not in W3b-0. W3b-SUBMAP item 2 is the first user; scheduled only after an orchestrator regression of the W2/W3a hashes with the change on a branch (O14).

**Testkit fixtures (`sm-testkit`).** `meshes::hex_block_two_volumes(n: [u32;3], h: [f64;3], split: u32) -> MeshFixture` (quad skin, TopoFace/TopoVolume classes, a cell set per volume); `meshes::nested_shell_volumes() -> MeshFixture` (hex cube in a hex shell); `meshes::with_sets(fixture, &[SetSpec]) -> MeshFixture`; `rays::degenerate_grid(n) -> Vec<(Point3, Vector3)>` (rays through lattice vertices and fan edges, for SOS); `topo::tiled_relief(n)` only if not already present. Reuse, no new fixture: `sm_mesh_api::reference::hex_block`, `sm_testkit::topo::assembly::pin_assembly`, `sm_testkit::topo::{torus, cone, cylinder, nested_spheres}`, `meshes::{tet_block_two_volumes, dual_patch, hexagonal_prism, mixed_all_shapes, mixed_transition}`, `stub::AnalyticWorld::with_lattice`.

**Acceptance of W3b-0.** Workspace `cargo build`, `cargo test`, clippy green; W2/W3a milestone hashes and every W3a golden unchanged (else O4 bless list); `tools/graph.py` ok; `cargo check -p sm-py --features python` passes; rt-api `codes_are_frozen` table includes -12 and capi maps it.

---

## 4. Tasks

### 4.1 W3b-1 (disjoint crates, in parallel)

### W3b-SERVER0. `sm-server` (L5): per-session pending input store
- **Scope:** replace `thread_local! PENDING` (`standard.rs:114`) with a store owned by each `Server`/session, so two sessions and any calling thread are independent; `host.rs` `notes_of` doc cleanup (0.7).
- **Acceptance:** 1. Two `Server`s staging different inputs on the same thread and on 8 threads import their own bytes (state hashes equal to serial runs). 2. `Server::open` still binds only its own store (existing test green). 3. A STEP import staged on thread A and applied on thread B succeeds. 4. Determinism and clippy.
- **Depends:** W3b-0. **Blocks:** PY acceptance 7-8.

### W3b-PRIM. `sm-model` (L2): remaining Cubit primitives, transform forms, copy (Q11)
- **Scope:** as the draft, unchanged by rulings except where Q11 confirms it: `model.create_frustum {height, radius, minor?, top?, center?}` (circular/elliptic cone, cylinder twin, apex); `model.create_prism`/`model.create_pyramid {height, sides 3..=1024, radius = circumradius, vertex 0 on +x, minor?, top?}`; `model.create_torus {major > minor > 0}`; **new** `model.create_sphere_part {radius, inner?, octants?: {x?,y?,z?}, center?}` (`create_sphere` unchanged, same hash); `model.move_to {entities, to: [Option<f64>;3] | id, anchor = "centroid" (selection) | "bbox_center"}`; `model.rotate_about` (exactly one of `axis`, `vertices: [a,b]`, `face`); `model.copy {entities, count 1..=4096, xform?}` deep copy with no names, colors, groups, boundary kinds, decks, meshes (`model.copy_attributes` later). Existing structs (`CreateSphere`, `Move`, `Rotate`, ...) gain no fields.
- **Acceptance (on `cell_complex` and `topo_store`; `validate`/`validate_geometry` empty; 1e-12 relative):**
  1. Frustum R=3, T=1, h=4: counts [2,3,3,1,1], V = 52π/3, A = 8π√5 + 10π; inverted identical; T=R equals `create_cylinder` hash; T=0 equals `sm_testkit::topo::cone(2,3)` hash.
  2. Elliptic frustum A=3, B=2, T=1.5, h=4: V = 14π; area vs independent quadrature (1e-9); apex V = πABh/3.
  3. Prism n=6, R=2, h=3: [12,18,8,1,1], V = 18√3, A = 36 + 12√3; pyramid n=4, R=√2, h=3: V = 4, A = 4 + 4√10; truncated top=√2/2: V = 7; sides 2, 1025 and top = radius `Invalid`.
  4. Torus R=5, r=1: [1,2,1,1,1], V = 10π², A = 20π², hash equals `topo::torus(5,1)`; major ≤ minor refused.
  5. Sphere part r=2: hemisphere V = 2πr³/3, A = 3πr²; octant V = πr³/6, A = 5πr²/4; all 26 cut sets clean with the listed counts; 8 octants sum to 4πr³/3; hollow (1,2) full V = 28π/3, A = 20π, Euler 4; hollow octant V = 7π/6, A = 19π/4; `inner ≥ radius` Invalid; `create_sphere` hash unchanged.
  6. `move_to [10, null, null]` moves the selection centroid exactly; vertex target; `bbox_center`; meshed refused.
  7. `rotate_about`: vertex-pair 120° permutes the corner edges (vertex set equal 1e-12); face 90° maps the cube to itself; `axis:"z"` equals `model.rotate [0,0,1]` hash; zero/two/no forms Invalid.
  8. `copy` brick count=3 move [20,0,0]: 4 volumes of 1000, centroids c + 20k x, source hash unchanged; reflect copy positive volume, outward normals; merged pair copies with the shared face shared (11 faces); copy of a meshed volume has no mesh and no colors/groups.
  9. Determinism, cold replay, undo/redo; property tests over valid and invalid sizes (invalid leaves the hash unchanged).
  10. Build, test, clippy, rustdoc; old T1 suites unchanged.
- **Must NOT:** fit carriers by tolerance; touch goldens; copy model records. **Fixtures:** testkit topo set (reuse). **Depends:** none.

### W3b-MODEL-EXPORT. `sm-cadio` (L2): `model.export` STEP (Q10)
- **Scope:** a Model-stage `model.export {bodies?: [TopoBody|TopoVolume], path?, schema: "ap214" (default) | "ap242", inline?}` over the existing `write_step`/`export_step` (`write.rs:318/378`), with a `WriteReport` (entities written, losses: colors and names not yet written, located). Registration form (timeline op vs read-only command returning bytes) is O5: the library, options, report and tests proceed now. **This task owns the registration**: the op (or query) type sits in `crates/sm-cadio/src/op.rs` next to `model.import_step` and is registered in `CadioPlugin::register` (`lib.rs`); sm-server already registers `CadioPlugin` in `standard()`, so no server edit is needed. If O5 is answered before this task merges, it registers in the answered form (acceptance 7); if O5 is answered later, W3b-CAD-ATTR (W3b-2, same crate) carries the registration and acceptance 7. Plus the sm-cadio 0.7 cleanups.
- **Acceptance:** 1. W3a three-body part: export, re-import through `model.import_step`: 3 bodies, volumes [1200, 200, 600] within 1e-9, face counts equal, `validate` empty. 2. `bodies` subset writes only those bodies. 3. Primitives from PRIM (frustum, torus, sphere part) round-trip with volume within 1e-9 once PRIM merges (dev test behind the merge; skipped with a reason otherwise). 4. Byte-identical output across two runs and `RAYON_NUM_THREADS=1` vs default (no timestamps in the header). 5. Refusals: unknown body, empty selection, unsupported carrier → typed errors naming the entity. 6. Cleanups: refusal tests bound parse work by counter; Newton cost counted per run; `global_projections <= MAX_GLOBAL`; no global atomics on the public API. 7. *(gated on O5)* `model.export` in the catalog with its schema; acceptance 1 through the registered op/query on a `standard()` registry; if a timeline op, replay from root reproduces the state hash without consulting the written file.
- **Depends:** none (3 soft on PRIM; 7 on O5).

### W3b-SCRIPT. `sm-lang` (L4): script record (Q12)
- **Scope:** the draft's Scopes A-D: `print_op` (schema-ordered named args, `fmt_num`, lexer escapes, ids as script ids, `Raw` fallback with note, always-on syntax self-check), `core.batch` printing, `record_script(reg, tl, head, RecordOptions)` with `Forks::{Head, Refs}`, `Check::{None, Tips (default), Every}`, migration of old op versions, `requires` for staged inputs, header without timestamps, authors or messages (Q12); `verify_recording`; `Host::export_formats()` (default: the current four) so new formats need no sm-lang edit.
- **Acceptance:** the draft's 1-12 unchanged (value round trip proptest; single-op golden; batch golden; linear history replay with `assert session.hash()`; fork topologies (a)-(i) with the documented extra `main` under `Forks::Head` on a non-main head; head after replay; corpus of `tests/golden/{run,parse}`; styles and `verify_recording` failure naming the statement; requirements and limits; determinism incl. sibling insertion order; export-formats hook; lints).
- **Must NOT:** write hashes, authors, times, messages; emit `undo`/`redo`; change existing goldens. **Depends:** none. Transport (`script.record`) is SERVER-HOST.

### W3b-EXPORT. `sm-meshio` (L2): DAGMC h5m and Gmsh msh4 writers (Q6)
- **Decisions:** pure-Rust private `src/h5/` writer (superblock v2, OHDR v2, compact links, contiguous datasets, no timestamps, children sorted, one layout pass); no `hdf5` crate, no C. `dagmc` and `gmsh` modules consume `&dyn MeshRead` (and `FacetInput` for `model_from_facets`; new L2 → C dep on sm-rt-api, declared in the skeleton). `H5M_FORMAT.md` from the 0.9 reference and MOAB sources (read as spec, cited). Naming: `mat:<set name>`, fixed (Q6; no prefix option); graveyard opt-in, named `mat:Graveyard`; volumes in no set reported `h5m.volume_without_material`. msh4 ASCII default, `binary` option; `PolyPolicy::Error` default (`Decompose`, `Drop` opt-in). Plus the sm-meshio 0.7 cleanups.
- **Acceptance:** the draft's 1-12: two-volume box h5m read back (counts, per-surface areas 1e-12, senses `[fwd, bwd]`); sense test by offset points and divergence volume; closed volumes and `OpenSurface` on a deleted face; quads split / polygon fan area exact (`hex_block_two_volumes`, `hexagonal_prism`); graveyard bbox and group; `model_from_facets` semantics; msh4 ASCII and binary round trip with positive Jacobians and analytic volume per shape; polyhedra policies (default error on `dual_patch`); byte determinism; property tests; corrupted-input tests never panic; conformance scripts Tier A (h5py diff against the reference dump, gmsh/meshio) and Tier B (Docker MOAB/DAGMC `mbconvert`, `pydagmc`, `point_in_volume`, `ray_fire` 1e-9).
- **Fixtures:** `hex_block_two_volumes`, `nested_shell_volumes` (W3b-0); reuse `tet_block_two_volumes`, `dual_patch`, `hexagonal_prism`, `mixed_all_shapes`. **Depends:** W3b-0 item 0.9 for the h5m part (msh4 starts at once). Wiring is EXPORT-WIRE.

### W3b-SURFPLAN. `sm-mesher` (L3): surface-only `mesh.run` and `mesh.plan` (Q13)
- **Scope:** the draft's Scope A (`mesh.run` takes volumes or faces, never both; same plan-once path with `MeshBuilder::surface()`; guards: face already meshed or of a meshed volume, **face sharing an edge with a face meshed in another entry is refused with an error naming the fix ("mesh them together in one mesh.run, or mesh.delete Mesh#n first")** (Q13), volume with a face in a surface mesh; face cell estimate) and Scope B (`mesh.plan {mesh? | targets?, attempts}` read-only, `PlanReport` with `status ran|will_run|infeasible`, per-entity `how/score/reason/alternatives`, interval `why`, `plan_hash`, `blockers`; notes from `entry.plan.notes`; P3 pending). Plus the DESIGN.md "Scheme notes" cleanup (0.7).
- **Acceptance:** the draft's 1-12 (brick face 6 quads / area 6; six faces 52 quads closed; tri face; guards atomic with the Q13 text asserted; delete/stale/hard intervals; `mesh.plan` ran vs will_run `plan_hash` equal; claimed/copy rows and `alternatives` with `AlwaysFail`; infeasible as status not error; two-source interval reporting; blockers; determinism and replay; schema `x-entity-kind` and property test on random faces).
- **Depends:** none (`Driver::mesh_from_plan` is on main). **Serializes before** AUTO on `mesh_ops.rs`.

### W3b-VIEW. `sm-view` (L3): render buffers and the web client (Q8)
- **Scope:** the draft unchanged: `ViewPlugin` queries `view.scene`, `view.mesh_skin` (cell-based exact clip, polyhedral centroid fan, `max_tris` refusal), `view.mesh_scalar`; library `skin()`; `assets()`; client in `crates/sm-view/web/` (TS, raw WebGL2, no runtime deps; model, mesh skin, clip, quality overlay, findings with suggested op text, timeline scrub with `at`, `ws://` and `fixture://` transports, one-line `script.run` console; read-only otherwise). Body colors: model colors are labels without RGB; a deterministic label → palette rule in `view.scene` is stated in DESIGN.md.
- **Acceptance:** the draft's 1-11 (4×2×2 box skin 80 triangles area 40; clip keeps 8 cells area 24; two-volume interface cap; poly skin watertight; surface mesh; scalar equals sm-quality bitwise after f32; typed errors; byte determinism and clip property test; Node unit tests; `assets()` and `dist.sha256`; docs, clippy, `tsc --strict`). Browser e2e skipped with a reason when Chromium is absent.
- **Must NOT:** touch sm-server; WebGPU; runtime JS deps; auto-run ops. **Depends:** none. Serving is SERVER-HOST.

### W3b-PY. `sm-py` (L6): Python bindings (Q7)
- **Scope:** the draft's rulings and Scope 1-10 with Q7 settled: package `smartmesh`, abi3 ≥ 3.10, not on PyPI, no zero-copy views of the live store (snapshot copies; `.smsh` mmap reader is the zero-copy path), PyO3 macro `unsafe` only (CLAUDE.md, ddc2122), `#![deny(unsafe_code)]` with one allow on the `#[pymodule]` item if the compiler needs it. `Host` over `Mutex<Server>`, GIL released, panics → `InternalError`; typed `Session` API generated from the catalog (`ops.pyi` drift check); `Mesh` tables with `.smsh` names; errors by RPC code; `ci/py.sh`.
- **Acceptance:** the draft's 1-11 (slice parity with `smesh run` and MCP; analytic tables of the 10-cube; session vs file byte equality and read-only flags; poly volumes by divergence; error mapping, panic hook; ids and `mypy --strict`; determinism incl. 8 threads; STEP import; `Host::call` equals `InProcess` for every method; wheel install; recorded table bench). 7 and 8 gate on SERVER0.
- **Depends:** W3b-0 skeleton and pins; SERVER0 for 7-8.

### W3b-RT-WORLD. `sm-rt` (L3): world walker, hierarchical input, tracking-file reader
- **Scope:**
  1. `BuildInput::Hier` builder: level stack, `pub(crate) trait Leaf { locate, next_event, cross, cell_count, node_info }`, mesh and facet leaf adapters, rect lattice Amanatides-Woo DDA (`SlotChange` on element walls, O(1) element index), `Union` placements (rigid transforms; **overlapping placements are `RtError::Build` `world.overlap` naming both**, Q4), global `CellId` by prefix sum over instances (no per-instance memory for uniform fills).
     *Overlap detection for `Union` placements (Q4):* broad phase by sort-and-sweep over the placed leaf AABBs; for each pair whose boxes overlap, an exact narrow phase that locates every boundary vertex of each placed leaf in the other leaf (a vertex strictly inside a cell of the other leaf, beyond the build tolerance, is `world.overlap` with that point). AABB overlap alone is not an error, so legitimate non-convex nesting (a pin placed in the hole of a moderator leaf) builds. The narrow phase misses overlaps where no vertex of either leaf lies inside the other (two crossed bars); those are reported at track time as `LostCause::Overlap` (the walker tests every placement whose box contains the point, so a point inside two placements is never resolved by order) and by `rt.check_overlaps` sampling. Cost: narrow phase O(boundary vertices × log cells) per overlapping pair, counted.
  2. `load_image` for the mesh tracker: open a tracker from a `CacheImage` without the input (Q2 reader path; what file the transport code receives is O1, this item works for the `.smrt` image whatever O1 decides). `Tracker::mesh()` returns `Some` through an sm-rt builder entry taking `Arc<dyn MeshRead + Send + Sync>` (no contract change). `surface_area`, `surface_faces`, `RegionInfo.tags`, `SenseMismatch`/`retries`.
  3. Switch exact-degenerate crossings to SOS's allocation-free entry once SOS merges (else leave a TODO and a failing-ignored test).
- **Acceptance:** 1. 4×4×1 lattice of `hex_block(3)` pins vs `stub::AnalyticWorld::with_lattice`: chord sums 1e-12 on 1e5 lines, same `SlotChange` sequence, `fire` skips them; prefix-sum `CellId`s at first, last and a random element. 2. Two-level nested lattice: `levels` (component, instance, index) at every step; path length equals box chord 1e-12. 3. Overlapping `Union` placements: `world.overlap` naming both and a point inside both; a pin placed in the hole of a moderator leaf (AABBs overlap, volumes disjoint) builds and its chords equal the single-mesh chords 1e-12; two crossed bars with no vertex inside the other build, and a ray through their intersection ends `Lost(Overlap)` with state unchanged; non-rigid: `world.nonrigid`. 4. `build → export_cache → write_smrt → read_smrt → load_image`: equal `structure_hash`, bit-identical `trace_bundle` over 1e5 rays; flipped byte `CacheCorrupt`; hostile image no panic. 5. `surface_area` of a box face equals the analytic area 1e-12; `surface_faces` non-empty for every surface. 6. Zero allocation with warm `Scratch` at depth ≤ `LEVEL_INLINE`. 7. Determinism (`trace_bundle` hash 1 vs N threads); W3a hashes for hierarchy-less inputs unchanged. 8. Lints, rustdoc, DESIGN.md "World".
- **Stretch (Q3):** S1 pointy-top hex lattice DDA (cube coordinates; flat-top through a 30° rotated placement, tested); vertex ties to the documented rule.
- **Fixtures:** `sm_mesh_api::reference::hex_block`, `stub::AnalyticWorld::with_lattice`. **Depends:** W3b-0.

### W3b-SOS. `sm-math` (L0): allocation-free SOS (TRIAGE GO 2)
- **Scope:** `orient3d_sos_noalloc` with a fixed-size perturbation table; same results as the allocating version.
- **Acceptance:** 1. Equal signs to `orient3d_sos` on `rays::degenerate_grid(64)` and 1e6 random degenerate tuples. 2. Counting allocator: 0 allocations. 3. Determinism, clippy.

### W3b-QUALITY. `sm-quality` (L2): verdict completeness and transport flag (TRIAGE GO 14, GO 5 part, Q14)
- **Scope:** `*_equiangle_skew`, `tet_mean_ratio`, `tet_inradius`, `tet_squish_index`, `quad_radius_ratio`, `hex_nodal_jacobian_ratio`, `wedge_max_stretch`, FV equiangle skewness and cell aspect ratio (checkMesh definitions); cells with ≥ 2 boundary faces flagged under the Transport profile; Transport thresholds ratified (Q14): drop "provisional" from the profile docs, values unchanged.
- **Acceptance:** each metric on its analytic fixture (regular tet mean ratio 1, unit cube aspect 1, a sheared hex at a closed-form value 1e-12); two-boundary-face count on a 3×3×1 slab equals the analytic count; existing outputs bit-identical (no golden moves); determinism.

### W3b-SMOOTH. `sm-smooth` (L3): `mesh.smooth {set}` (Q14)
- **Scope:** `set: <name>` selects nodes of the cells in a `mesh.flag_cells` set (same names); `NodeSelect::Set`.
- **Acceptance:** 1. Nodes outside the set bit-identical after smoothing. 2. Unknown set: typed error listing set names. 3. Quality inside the set does not decrease on the perturbed-hex fixture. 4. Determinism.

### W3b-PAVE. `sm-scheme-pave` (L3): corner rule and valence cleanup (TRIAGE GO 9)
- **Scope:** acute/obtuse corner rule for three-corner regions; diagonal collapse; valence-2 removal; private CDT stays.
- **Acceptance:** 1. 150 seeded random triangles: failures 0 (was 32), all quads, area exact 1e-12, `validate` empty. 2. Five-lobed star min SJ recorded, not below the W3a 0.151. 3. Existing pave tests and hashes unchanged where the old ladder succeeded (report any moved case; a moved golden is a stop). 4. Determinism. **Blocks:** AUTO's triangle-source case.

### W3b-SWEEP. `sm-scheme-sweep` (L3): auto support (C-SWEEP)
- **Scope:** prefilter before `discover_columns` (equal vertex counts, no shared vertex) with identical results; work counter (test feature); reason text `1-to-1 sweepable from {s} to {t} ({k} valid pairs)`, prefix kept. Source/target stays `find_columns`' lowest-id pair (explicit hashes).
- **Acceptance:** identical plans and hashes on every existing sweep test; counter shows fewer discover calls on a 20-face block; reason text asserted.

### W3b-BOOL. `sm-boolean` (L2): candidate-pair BVH (TRIAGE GO 7)
- **Scope:** BVH over face boxes replaces O(F²) `candidate_pairs`; indexed fragment classification; `subtract` with N targets shares the tool (Arc).
- **Acceptance:** identical outputs and hashes on every W2/W3a boolean test; counter `candidate_tests ≤ c·F log F` on a 1e4-face assembly (c recorded); determinism.

### W3b-RTBENCH-A. `sm-rt-bench` (L4): stub-only benchmark harness (Q9)
- **Scope:** the draft's cases generator (L1 legacy, P1 pin cell, P2 17×17 with `pin_assembly`, H1 homogeneous with inclusions, F1 synthetic fusion-like torus sector), `smrays v1`/`smfacets v1`/`smtets v1`/`smcase.json` formats (σ values in `smcase.json` are the benchmark's own transport workload, never sm-rt input), `--model <path>` for a local STEP file (imported through sm-cadio in the runner; the file and its derived data are never committed; `.gitignore` entry and a test that the results writer refuses paths under the repo for `--model` runs), result schema `smbench-result v1`, agreement checker, fairness rules as code, `self-test`, `report` (refuses mixed machines; marks macOS results "smoke"), Dockerfiles pinned by digest and commit.
- **Acceptance:** draft 1, 3 (facet-twin bound), 4, 7 (determinism of generators), 8 (fairness gates), 11 (lints) against the stub `AnalyticWorld`; a `--model` dry run on a tiny STEP from `sm-testkit/data/step` writes nothing under the repo.
- **Must NOT:** claim speedups; commit numbers not produced by the harness; commit `--model` inputs. **Depends:** W3b-0 skeleton.

### 4.2 W3b-2

### W3b-RT-DELTA. `sm-rt` (L3): majorant partition and delta flights (Q1, Q2)
- **Scope (new module `src/delta/`; shared files get registration lines only):**
  1. kd partition over meshed leaves (disjoint boxes covering the padded model box; candidate cell lists in CSR; split rule independent of any σ: split while distinct regions among candidates > 1 and candidates > `leaf_cells`; 8 planes per axis, cost by volume fraction × (candidates + region spread), ties lowest axis/position; `preassigned` leaves inside one convex cell). Lattice elements and universes are nodes. Material sets deduplicated with `BTreeMap`.
  2. `MajorantTracker` for the mesh tracker (`caps |= MAJORANT`): `begin`/`FlightCursor` per §3.2 (majorants and `block` per call in `FlightInput`; the cursor caches the current segment's node majorant for `check_collision`); segments contiguous, lengths sum to the chord; events as `step` (decks applied in flight; `Respond` left on the face); hinted location from the owning node (preassigned, safety-cached previous cell, candidate list; equals `locate` for every point; `bvh_descents == 0` inside the box); `check_collision` per Q2; zero allocation after warm-up.
  3. `block` nodes and caller-driven `set_policy` (O3); safety distance (exact for convex cells, cached in the cursor); region jump behind `PartitionOptions::region_jump` (per-region face BVH; same end, ids, `to_cell`, distance within 1e-12(1+d); `step`/`trace` never jump).
  4. Cache: partition and candidate lists in the image as `Private(4..8)`; options hashed only when non-default; hierarchy-less images byte-identical.
  5. `examples/bench_delta.rs` records counters per mode (record only).
- **Acceptance (tests own a test-local cross-section model keyed by region name; σ never enters sm-rt; the full statistical runs 5-7 are `#[ignore]`, release, fixed seeds, 4σ; 6b is their small CI form):**
  1. Exact segment sums: 1e5 lines on `hex_block(10)`, `mixed_transition`, a 2-level lattice: Σ(t1 − t0) equals the hull chord 1e-12, contiguity bit-exact, endpoints inside node boxes, surface segments equal `trace` segments; 0 Lost.
  2. Material sets: every leaf's set equals the brute-force set of regions of cells meeting it; `validate_partition` passes (tiling and containment).
  3. Violation: an understated majorant for one node gives `MajorantViolated` with exact node, σ_t, majorant, position, cell; state not advanced; code -12.
  4. Hinted location equals `locate` on 1e6 points (1e5 within 1e-9 of faces/edges/vertices); `bvh_descents == 0`; preassigned leaves 0 tests.
  5. Homogeneous slab (mirror lateral decks, L = 20, pure absorber τ ∈ {0.1, 1, 10}, isotropic source, 2e5 histories): leakage (1/τ)(1/2 − E3(τ)) within 4σ; track-length (surface) and collision (delta) flux within 4σ; majorant scaled ×{1,3,10}: estimates agree, rejection fraction 1 − 1/scale within 4σ.
  6. Two-region beam: transmission exp(−(Σ1 a + Σ2 (L − a))) within 4σ in surface, delta and a mixed per-node `block` policy; the low-Σ leaf is separated by the partition (its set holds one region).
  6b. **CI variants (not `#[ignore]`, debug build, run on every `cargo test`):** (i) acceptance 5 at τ = 1 only, 1e4 histories, seed 1, majorant scale ×3, delta mode: leakage within 10σ of (1/τ)(1/2 − E3(τ)) and the rejection fraction within 10σ of 2/3; (ii) acceptance 6 with Σ1 = 0.5, Σ2 = 2, a = 1, L = 2, 1e4 histories, seed 2, in surface, delta and a mixed `block` policy: transmission within 10σ of exp(−(Σ1 a + Σ2 (L − a))). σ is the binomial standard error of the estimate; the seeds are fixed so the outcome is deterministic.
  7. Heterogeneous `hex_block(8)`, 4 materials by (i+j+k) mod 4, mirror sides, scattering ratios {0.2,0.5,0.7,0.9}: balance identity for track-length, collision and mixed estimators within 4σ; per-cell agreement within 5σ; checker `block` policy agrees with pure surface.
  8. Safety ≤ fired distance along 1e3 directions, equal along −n_min 1e-12; flights identical with the cursor cache on/off (positions, RNG draws); `safety_skips > 0`.
  9. Region jump: 1e6 rays, jump on vs off same end, ids, `to_cell`, distance 1e-12(1+d); fewer `face_tests + node_visits` on `hex_block(40)` (recorded).
  10. Determinism: partition bytes and a 1e5-history batch (per-history RNG, block-ordered reduction) bit-equal for 1 vs N threads and repeated; W3a `trace_bundle` hashes unchanged for hierarchy-less fixtures.
  11. Cache round trip with partition: equal `structure_hash`, bit-identical batch; option change gives `CacheStale`; hostile image no panic.
  12. Errors and allocation: each new error path tested; `next_segment`, `probe`, `fire` allocate nothing warm.
- **Must NOT:** put σ, an energy grid or physics into sm-rt; clamp a violation; locate from the root when a node owns the point; change W3a outputs for hierarchy-less inputs; claim speedups.
- **Depends:** RT-WORLD. Runs in parallel with RT-ANALYTIC (new modules; orchestrator merges ANALYTIC first and resolves `lib.rs`, `plugin.rs`, `builder.rs`, `image.rs`).

### W3b-RT-ANALYTIC. `sm-rt` (L3): analytic CSG leaf and overlap check (Q3, Q4)
- **Scope (new module `src/analytic/`):** `AnalyticTrackerBuilder` for `BuildInput::Hier` with `LeafInput::Csg`; surface kinds Plane, Sphere, Cylinder, Cone, Torus, General quadric with conditioned kernels (perpendicular-offset forms; Newton polish on compensated residuals; tangent roots give no event); cells are disjoint (Q4): `locate` evaluates cells by their own surfaces, never resolved by order; concentric fast path (binary search on r²); `step` with neighbour search via `surface → cells` CSR, full-leaf scan counted as `slow_locates`, nudge recovery; exact `closest` (Unsupported for General); build checks `csg.unbounded`, `csg.degenerate_surface`, `csg.empty_cell`, `csg.unused_surface`, `csg.unknown_surface`.
  *Overlap detection during tracking (Q4), bounded cost:* `step` tests only the cells of the `surface → cells` CSR on the far side of the crossed surface and evaluates **all** of them (no early exit), so two matches among those neighbours end the step `Lost(Overlap)`. A cold `locate` (no hint; already counted in `slow_locates`) tests every candidate cell left after bounds pruning (concentric fast path or the leaf's cell-box BVH) and reports two matches as `Lost(Overlap)`; it is not O(all cells) unless the boxes give no pruning. Neither path searches for overlaps elsewhere: completeness is the job of the sampled check below, not of tracking.
  **Overlap check (Q4), library only in this task:** `pub fn check_overlaps(input: &HierInput, n_points, n_rays, seed) -> Vec<BuildIssue>` (stratified points and seeded rays; every sample tests every candidate cell; reports `csg.overlap` and `csg.gap` with the sample point and the cells). The session op `rt.check_overlaps` is registered by W3b-RT-BRIDGE, the first task that can produce a `HierInput` with CSG leaves from a session (no CSG importer is scheduled, O7). `rt.check`/`rt.info` cover analytic trackers.
- **Acceptance (1e-12 absolute unless stated; fixtures local in `tests/common/csg.rs`):** the draft's 1-6 (sphere, cylinder, cone, torus incl. tangent, general quadric chords; point location), then:
  7. Union cell `(A) | (B)` gives the inclusion-exclusion chords; two overlapping cells in one leaf: `check_overlaps` reports `csg.overlap` with a point inside the true lens, and a `step` that crosses into the lens ends `Lost(Overlap)` with state unchanged (no silent pick); a cold `locate` inside the lens is `Lost(Overlap)`; a hole gives `csg.gap`.
  8. 17×17 pin lattice and 3 levels (pin, assembly, 3×3 core): 17 fuel chords of 0.8 per row, 16 `SlotChange` per assembly row, path = box chord 1e-12, `levels` depth 3, prefix-sum ids.
  9. *(stretch, Q3)* pointy-top hex lattice chords along the three lattice directions; flat-top via a 30° rotated placement.
  10. Stub parity on nested/disjoint `AnalyticWorld` worlds (after the §3.2 Q4 stub change): equal segments, cells, regions, ends, distances 1e-12.
  11. 1e6 lines through the 3-level core: sums equal hull chord 1e-12; reversed lines equal; near-edge lines never Lost.
  12. Reflective box and translation wrap; `Respond` not applied in step.
  13. Errors, determinism (`trace_bundle` and `structure_hash`, handle permutation invariant), zero allocation.
  14. Closest distances to plane/sphere/cylinder/cone/torus 1e-12.
  15. Library `check_overlaps` is deterministic (same findings for 1 vs N threads and repeated runs, seed in the output). The op-level test is RT-BRIDGE acceptance 5.
- **Depends:** RT-WORLD. **Must NOT:** first-wins ordering; duplicate world types; root restarts during a step.

### W3b-RT-CAPI. `sm-rt-capi` (L4): transport handover through the C ABI (Q1, Q2)
- **Scope:** `smrt_open(path)` opens the tracking file through `load_image` (format per O1; `.smrt` image until O1 says otherwise); `smrt_dag_measure_area`, `get_angle(surface)` via `surface_faces`, region tags; materials per spatial region: `smrt_n_nodes`, `smrt_node_info`, `smrt_node_set`, `smrt_n_sets`, `smrt_set_regions`, `smrt_region_name`; two-phase callback-free flight entries (30e2742 shape, adapted to Q2): mirroring the Rust per-call `FlightInput`: `smrt_flight_new(tracker, state, opts)` (no slices; opts as `FlightOptions`, no band), `smrt_flight_next(flight, const double* majorants, size_t n_nodes, const uint32_t* block, size_t n_block, segment_out)` (0 = segment, 1 = done, < 0 error), `smrt_flight_probe(flight, majorants, n_nodes, block, n_block, t, out)`, `smrt_flight_stop_at(flight, majorants, n_nodes, block, n_block, t, out)`, `smrt_flight_check(flight, sigma_t)` (against the majorant cached at the last `smrt_flight_next`; −12 on violation), `smrt_flight_free`; the flight handle stores no caller pointer between calls; every entry in `catch_unwind` with `// SAFETY:` comments. No `smrt_majorant_from_flat` (Q2).
- **Acceptance:** 1. A C test program (built by `build.rs` test harness) opens a `.smrt` written by sm-rt, reads node sets and region names, assigns σ per name, runs 1e4 surface and delta flights on the stub majorant world and the mesh tracker: tallies equal the Rust path bit for bit. 2. Understated majorant → −12 from `smrt_flight_check`, state unchanged. 3. `measure_area` equals the analytic face area; `get_angle` equals the face normal. 4. Null/invalid handles and lengths return typed codes, never crash; a panicking stub returns `SMRT_ERR_PANIC`. 5. Header test lists every code −1..−12 and every lost cause including `SenseMismatch` and `Overlap`. 6. Determinism, clippy.
- **Depends:** RT-WORLD (load_image); delta entries run against the stub and, after RT-DELTA merges, the mesh tracker (acceptance 1 second half gated on RT-DELTA). Merge order in W3b-2: RT-ANALYTIC, then RT-DELTA, then RT-CAPI last (it rebases on RT-DELTA and runs acceptance 1 in full before merging).

### W3b-AUTO. `sm-mesher` (L3): pluggable `mesh.auto {strategy}` (Q5)
- **Scope:**
  - `pub trait AutoStrategy: Send + Sync { fn name(&self) -> &'static str; fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError>; }`; registry of strategies in `MeshPlugin`; `pub struct DefaultStrategy` (`"default"`): candidate lists surface `[map, submap, pave, trimesh]`, volume `[map, sweep, tetmesh]`, curve `[equal]`, triangle sources pave; it uses the planner's scores and fallback (`Driver::plan`, replan loop with `replan_budget = 16 + 2·targets`) to find the schemes, then returns one explicit assignment per target that had no intent scheme.
  - Op `mesh.auto {strategy = "default", targets?: [TopoVolume|TopoFace]}`: runs the strategy at apply and **stores its canonical record as `mesh.auto {strategy, assignments: [{target, scheme, params}]}`** via the W3b-0 sm-op-api hook; replay applies `assignments` only and never calls a strategy. The resulting intent equals what `mesh.scheme` per target would give.
  - Warning `mesh.auto_tet` per volume assigned `tetmesh` by the strategy, listing earlier candidates' reasons and ending with the fix (`mesh.scheme <volume> sweep` or a strategy without tets).
  - Query `mesh.auto_strategies {}` → names and one-line descriptions.
  - `standard_schemes` lists are unchanged (strict) so runs without `mesh.auto` keep their hashes; the integration of the registry with real schemes (D-AUTO) is SERVER-HOST.
- **Acceptance A (stub registry):**
  1. `mesh.auto` on a stub brick: the stored record holds explicit assignments; `mesh.run` afterwards gives the same hash as a script with the equivalent `mesh.scheme` ops.
  2. Unknown strategy, targets of the wrong kind: `Invalid`, state hash unchanged.
  3. Order is the tie-break inside `default` (swapping a test strategy's list swaps the winner).
  4. `applicability` called at most once per (target, candidate) per plan (counting stub).
  5. `replan_budget(0) == 16`, `(40) == 96`; 20-volume fallback model succeeds within budget; fails at a fixed 16 (negative control).
  6. `mesh.auto_tet` emitted only for a tet-fallback volume, text contains the reasons.
  7. **Determinism constraint (Q5):** a counting test strategy: replay from root of a timeline containing `mesh.auto` calls the strategy 0 times and reproduces every state hash.
  8. **Strategy change does not move old hashes (Q5):** record a timeline with test strategy v1, swap the registered implementation for v2 (different choices) and replay: all node hashes equal the v1 recording; a fresh `mesh.auto` under v2 yields different assignments.
  9. Explicit-scheme runs: every existing sm-mesher test and the W2/W3a scripts are bit-identical; two runs and 1 vs N threads equal (state hash and stored record bytes).
- **Acceptance B (SERVER-HOST runs them through `standard()` with real schemes; listed here so the strategy is written for them):** brick 10³ map (`Auto` score 100, no warning); L-slab sweep + submap source, 24 hex, V = 3; triangular prism sweep + pave source, all hex, V = 0.5 (needs PAVE); tetrahedron → tetmesh with `mesh.auto_tet`; plate with hole outcome pinned in the report; auto next to a hard tet neighbour conformal; census report listing every changed case (Q5).
- **Must NOT:** change scores, the planner or scheme crates; change `standard_schemes` strict lists; re-run a strategy on replay. **Depends:** SURFPLAN (serial on `mesh_ops.rs`), W3b-0 op-api hook, PAVE and SWEEP for acceptance B.

### W3b-EXPORT-WIRE. `sm-server` (L5) + one-line edits: mesh export wiring (Q10)
- **Scope:** `ExportFormat::{Dagmc, Gmsh}` dispatch to `sm_meshio::dagmc::export_h5m` and `gmsh::write_msh4` with per-format `options`; extension inference `.h5m`, `.msh`; loss reports into `ExportResult.notes`; `Host::export_formats` in the server host lists them; `smesh export` accepts the formats and `--option k=v`; MCP export tool schema regenerated at INT; `tools/conformance.sh --only dagmc|gmsh` (orchestrator applies the one-line tools edit).
- **Acceptance:** 1. `session.export a.h5m` and `a.msh` on the two-volume brick through in-process, CLI and MCP: identical bytes. 2. Options: msh4 `binary: true`; polyhedra default error surfaces as a typed RPC error with the hint; dagmc graveyard opt-in. 3. Notes carry the `h5m.*`/`msh.*` loss codes. 4. Unknown option keys rejected per format. 5. Determinism.
- **Depends:** EXPORT, SCRIPT (export_formats hook), SERVER0.

### W3b-DIAG. `sm-diag` (L3): suggestions and unflag (TRIAGE GO 5)
- **Scope:** the existing `smooth` suggestion (`suggest.rs`, `OP_SMOOTH`, emitted for spread bad entities with `args: None` today) gets `args: {set: <name>}` (Q14: `mesh.smooth {set}` reuses `flag_cells` set names). No new suggestion kind and no new field. Set name rule: `<name> = diag.<metric>` (fits `flag_cells`' `[A-Za-z0-9_.-]{1,64}`). If the mesh already has a set of that name, only the `smooth` suggestion is emitted; otherwise a two-step pair is emitted in this order: first an `inspect` suggestion with `op: mesh.flag_cells, args: {name: "diag.<metric>", metrics: [<metric>], level: "warn"}`, then `smooth` with `args: {set: "diag.<metric>"}` and a reason saying it needs the first step. Transport two-boundary-face findings located; `mesh.unflag_cells {set}` through `MeshEdit::remove_set`; dev-deps comment (0.7).
- **Acceptance:** 1. Perturbed hex box: the pair is `mesh.flag_cells {name: "diag.scaled_jacobian", ...}` then `mesh.smooth {set: "diag.scaled_jacobian"}`; running both improves min SJ; with the set already present only `smooth` is emitted. 2. Starting from state S: `mesh.flag_cells {name: n}` then `mesh.unflag_cells {set: n}` gives a state whose mesh set list equals S's and whose state hash equals S's hash (relies on the W3b-0 `remove_set` requirement); unknown set: typed error listing the set names, hash unchanged. 3. Existing outputs unchanged except `args` of the `smooth` suggestion and the added `inspect` step on affected cases (no schema golden change without the P2 bless). 4. Determinism.
- **Depends:** QUALITY, SMOOTH, W3b-0 `remove_set`.

### W3b-CAD-ATTR. `sm-cadio` (L2): STEP names and colors (TRIAGE GO 8(b), Q14)
- **Scope:** `model.import_step_attributes` (or an option on `model.import_step`): STEP product/solid names become model groups first; colors only from solid-attached colors (Q14); report what was skipped.
- **Acceptance:** a committed STEP with named, colored solids: groups and colors equal the file; face-only colors ignored with a note; hashes of imports without the option unchanged. **Depends:** MODEL-EXPORT (same crate).

### W3b-FOAM-WARP. `sm-meshio` (L2): warped dual faces (TRIAGE GO 4(b))
- **Scope:** split warped polyhedral faces into planar pieces in the OpenFOAM exporter; INT-9 counts recorded before/after.
- **Acceptance:** split faces planar to 1e-12; cell volumes conserved 1e-12; checkMesh concave count recorded; internal cells untouched. **Depends:** EXPORT (same crate).

### W3b-SUBMAP. `sm-scheme-submap` (L3): periodic and seam faces (TRIAGE GO 10)
- **Scope:** blend through pcurves via `GenCtx::loop_uvs` so cylinder lateral faces mesh with submap; graded cut-line spacing (W3a stretch). Item 2 (closed NURBS faces) waits on O14.
- **Acceptance:** cylinder lateral face r=1, h=2 at size 0.25: all quads, area 4π within the chord error bound, nodes on the surface 1e-12; existing submap hashes unchanged.

### 4.3 W3b-3

### W3b-RT-DELTA-WORLD. `sm-rt` (L3): flights across lattices and `rt.partition`
- **Scope:** delta flights over lattice/universe nodes (elements are nodes; wall crossings inside a segment stream; owning element by index arithmetic); `rt.partition {mesh, leaf_cells?, max_depth?}` read-only query (structure only: nodes, leaves, depth, set count, memory, `structure_hash`).
- **Acceptance:** 1. RT-DELTA 5 and 6 repeated on a 3×1×1 lattice of slab elements agree with the single-mesh results within 4σ. 2. `lattice_index_locates` equals the tentative count, `bvh_descents == 0`. 3. 17×17 assembly: `bvh_descents == 0` in delta; segment and tentative counts recorded. 4. `rt.partition` deterministic, schema in catalog. **Depends:** RT-DELTA, RT-ANALYTIC merged.

### W3b-RT-BRIDGE. `sm-rt` (L3): Model → CSG world
- **Scope:** `csg_world(model view, topo view, opts) -> (CsgLeafInput owners, WorldSpec)`: planes, spheres, cylinders, cones, tori of model faces become surfaces; volumes become disjoint cells named by their color/material name; prototypes and lattices become `WorldSpec` components; anything non-quadric is `Unsupported` with the entity.
- **Acceptance:** 1. `pin_assembly(17, 1.26, 0.4096, 0.475, 100)`: analytic tracker chords through each cylinder `2√(r² − b²)` 1e-11 relative on 1e4 rays. 2. Region names equal the model color names. 3. A NURBS face gives `Unsupported` naming it. 4. `rt.check_overlaps` on the bridged assembly finds no overlap. 5. `rt.check_overlaps` op: in the catalog with its schema; input is the session model through `csg_world` (bodies optional, default all); a model with two overlapping cylinders reports `csg.overlap` with a point and both cells; same findings for 1 vs N threads and repeated runs, seed in the output.
- **Also owns:** registration of the Check-stage `rt.check_overlaps {bodies?, samples?, rays?, seed?}` (kind per O6) over RT-ANALYTIC's library `check_overlaps`. **Depends:** RT-DELTA-WORLD (same crate).

### W3b-SERVER-HOST. `sm-server` (L5) + one-line edits: viewer, script record, auto integration
- **Scope:** HTTP static serving of `sm_view::assets()` under `smesh serve --ws`, multiple connections sharing one session; live updates by client polling of the existing `session.hash` method (no server push: sm-protocol rejects id-less messages and W3b schedules no notification ACR, O16); `script.record` dispatch to `sm_lang::record_script` (+ `smesh record`, MCP `record_script`); D-AUTO: register `DefaultStrategy` with the real schemes and run AUTO acceptance B in `crates/sm-server/tests/auto_select.rs`. (`model.export` is registered in sm-cadio by MODEL-EXPORT or CAD-ATTR, not here; if O1 picks the CLI as the handover writer, the `smesh` subcommand lands here.)
- **Acceptance:** 1. Live smoke: headless client A connects and receives `view.scene`; client B applies a `script.run` on a second connection; A's next `session.hash` poll returns B's new hash and A's re-query of `view.scene` reflects the change. 2. `script.record` on the W3b milestone timeline replays to equal per-ref tips via CLI and MCP. 3. AUTO acceptance B 1-7 with the census report (every changed case listed, Q5). 4. Determinism and op-count snapshots updated (additive only).
- **Depends:** VIEW, SCRIPT, AUTO, EXPORT-WIRE (same crate, serial).

### W3b-RTBENCH-B. `sm-rt-bench` (L4): tracker rows and runs (Q9)
- **Scope:** sm-rt adapters (mesh, analytic via RT-BRIDGE, delta), Docker rows (dagmc-moab, dagmc-dd, xdg) on this Mac as smoke; the published table waits for the user's bare-metal Linux x86_64 machine (O11).
- **Acceptance:** draft 2 (analytic truth), 5 (counters), 6 (W-flight unbiasedness, 4σ), 9-10 (Docker, `partial` with the failing command when unavailable); `report` labels macOS rows "smoke"; no speedup text.
- **Depends:** RT-DELTA-WORLD, RT-BRIDGE, RT-ANALYTIC; EXPORT for stretch B6.

---

## 5. W3b-INT. Integration and milestone

Integration agent after the W3b-3 merges; owner micro tasks for any crate edits.
1. **Milestone script** (`tools/w3b_milestone.smesh` + Python and C drivers): PRIM pin cell and 3×3 lattice via `model.copy`; `mesh.auto {strategy: "default"}` (history shows explicit assignments; replay calls no strategy); `mesh.run` on one face plus `mesh.plan`; export `.h5m` (graveyard), `.msh` (ASCII and binary), `.smsh`; `model.export` STEP re-imported with equal volumes (gated on O5: if O5 is still open at INT, this step is skipped with a printed reason and listed in the report); tracking handover file written and opened by the C driver through `libsmrt`, which reads region names, assigns its own σ, and runs surface and delta flights (slab leakage analytic within 4σ). Writer of the handover file: the surface O1 picks; **fallback while O1 is open:** the INT Rust harness (`tools/w3b_milestone/` driver, no crate edit) builds the mesh tracker from the session's exported `.smsh` through `sm_rt`, calls `Tracker::export_cache` and `sm_rt::write_smrt_file`, and hands the `.smrt` path to the C driver (the same path as RT-WORLD acceptance 4 and RT-CAPI acceptance 1); `script.record --forks refs`, replay equal; the same slice from the installed wheel; CLI, MCP, in-process and Python state hashes equal.
2. **Goldens:** regenerate the MCP tool list and op-count snapshots once, for added ops and queries only. Any other golden that moves (hash, existing output schema) stops the integration and goes to the bless list (O4, O10, O13).
3. **Viewer live smoke** against `smesh serve --ws` (Chromium if present, else the Node fixture path with a printed skip).
4. **Conformance:** `SM_CONFORMANCE=1 tools/conformance.sh --only dagmc|gmsh`.
5. **PARITY.md:** A P1 script record, A P1 Python API, B P1 primitives and transforms/copy, B STEP export, C auto selection rows, E P1 msh4 and DAGMC, F web frontend (read-only); each ticked only with a test name.
6. **Report** `docs/design/w3b/w3b-report.md`: deviations, counters, open items carried to W4.

---

## 6. Dependencies (summary)

```
W3b-0 ─┬─ W3b-1: SERVER0, PRIM, MODEL-EXPORT, SCRIPT, EXPORT(h5m after 0.9), SURFPLAN, VIEW, PY(7-8 after SERVER0),
       │         RT-WORLD, SOS, QUALITY, SMOOTH, PAVE, SWEEP, BOOL, RTBENCH-A, tet/poly cleanups
       ├─ W3b-2: RT-DELTA ∥ RT-ANALYTIC (after RT-WORLD), RT-CAPI (after RT-WORLD; acceptance 1 second half
       │         after RT-DELTA), AUTO (after SURFPLAN), EXPORT-WIRE (after EXPORT, SCRIPT, SERVER0),
       │         DIAG (after QUALITY, SMOOTH), CAD-ATTR (after MODEL-EXPORT; model.export registration if
       │         O5 was answered late), FOAM-WARP (after EXPORT), SUBMAP
       │         merge order in sm-rt/sm-rt-capi: RT-ANALYTIC → RT-DELTA → RT-CAPI
       └─ W3b-3: RT-DELTA-WORLD → RT-BRIDGE (rt.check_overlaps op), SERVER-HOST (after VIEW, SCRIPT, AUTO,
                 EXPORT-WIRE), RTBENCH-B (last) → W3b-INT
```
Gates outside the wave order: `model.export` registration on O5; the handover writer on O1 (milestone fallback in §5.1); RT-DELTA 5-7 statistical runs are `#[ignore]`, 6b runs in CI.
Not scheduled (WAIT, from TRIAGE): shared CDT crate, pole/degenerate faces, tet Steiner points, smooth W4 items, high order and Exodus netCDF-4, mesh booleans and virtual topology, sizing exact limits, remote staging and `.smsh` tracking chunk, tri variants, sm-rt SIMD/mmap/F32, sm-lang user functions, sequential meshing, overlay `MeshRead` (Q14), `PlanCtx::tolerance`/`Patch::strip_index`.

---

## 7. Open items (need a user or orchestrator answer; the plan does not fill them)

- **O1 (Q2) Handover file format.** Which file the transport code opens: the `.smrt` image (made self-contained: geometry, connectivity, region names), `.smsh` plus region material names, or both; and which surface writes it (`session.export` format, an `rt.*` op, or the CLI). RT-WORLD/RT-CAPI work on `.smrt` until answered. Conditional owners: `session.export` → W3b-0 ACR `ExportFormat::Smrt` plus EXPORT-WIRE/SERVER-HOST dispatch; `rt.*` op → RT-DELTA-WORLD; CLI → SERVER-HOST. Milestone fallback while open: the INT harness calls `Tracker::export_cache` and `sm_rt::write_smrt_file` (§5.1).
- **O2 (Q2) Energy dependence of majorants.** One slice per segment call (current plan; `FlightOptions.band` removed) or a band index in the contract (would become a per-call `FlightInput` field).
- **O3 (Q2) Hybrid policy.** fd4a235's `ModePolicy::Auto` needed `mean_total`, which Q2 removes; the plan makes hybrid caller-driven (`set_policy`, `block`). Confirm, or allow a second optional per-node number.
- **O4 (Q4) Bless list.** Any hash or golden moved by removing first-wins and Union priority (stub, sm-rt, sm-rt-capi tests in §3.2) needs an explicit bless ruling.
- **O5 (Q10) `model.export` as a timeline op.** A recorded op that writes a file would rewrite it on replay; confirm the op form or a read-only command returning bytes. Registration owner: MODEL-EXPORT if answered before it merges, else CAD-ATTR; the milestone STEP step is skipped while open.
- **O6 (Q4) Overlap check registration.** Recorded op or read-only query (like `mesh.validate`); registered by RT-BRIDGE.
- **O7 (Q4) Importer rule.** MCNP/OpenMC importers must reject overlapping decks; no importer is scheduled in W3b (owner and wave for the importers).
- **O8 P1-P5** await the user's OK; nothing is blessed or flipped until then.
- **O9 Q5 mechanism** (orchestrator): sm-op-api canonical-record hook (planned) vs host expansion into `core.batch`.
- **O10 (Q5) Census.** Changed cases are reported; if an existing golden moves, a bless ruling is needed.
- **O11 (Q9) Bare-metal Linux x86_64 machine** for the published table: user to arrange; RTBENCH-B publishes only smoke rows until then.
- **O12 (Q9) FreeCAD MCP.** The user offered to generate test STEP geometry via FreeCAD MCP; the plan uses only local files through `--model`. Say if specific geometry should be generated (e.g. a fusion sector with known volumes).
- **O13 Additive goldens.** W3b-INT regenerates the tool-list and op-count goldens for new ops only; confirm that standing practice covers it.
- **O14 `sample_face` pcurves (TRIAGE R9).** Behaviour change in sm-topo-api; needs a W2/W3a hash regression first; any moved hash needs a ruling.
- **O15 (Q6) Licence.** If no licence-compatible MOAB-written h5m can be committed, the user picks the source.
- **O16 Server-push notifications.** sm-protocol rejects id-less messages (`lib.rs:12`, `envelope.rs`), so a pushed `state.changed` needs an ACR (message shape, which methods trigger it, ordering against replies). Not in W3b: the viewer polls `session.hash`. Say if push should be scheduled.
- **O17 (Q4) Union overlap detection is incomplete at build.** The narrow phase catches overlaps with a vertex inside the other placement; edge-only crossings surface at track time (`Lost(Overlap)`) or through `rt.check_overlaps`. Confirm, or require an exact mesh-mesh intersection test at build.
