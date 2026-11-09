# W3 backlog (collected at the end of W2)

Sources: W2 integration (decision 10), the post-integration contract batch, and the W2 polish wave (decision 11). Each item names its owner. "Contract" means a frozen crate, so the orchestrator schedules it.

## Contract changes
- **CR-2 completion** (`sm-model-api`): `validate` still uses `FaceMoments::plane`. Switch it to `TopologyView::face_plane`, then drop sm-model's `Tolerance::with_linear(1e-6)` mirror-deck workaround.
- **Plan once** (`sm-scheme-api`): `Driver::mesh_from_plan`, plus `DriverOptions::max_cells` or a cell estimate on `Plan`. Today `mesh.run` plans twice and keeps its own budget.
- **Planner firmness cascade** (`sm-scheme-api`): today an edge request always beats a face or volume request. Cubit's rule is that firmness decides first, then the latest command wins (decision 9).
- **Scheme-derived counts** (`sm-scheme-api`): `IntervalSink::goal` is averaged with the edge's sizing goal and the Default counts. So bias/stretch schemes that derive n from sizes cannot use it: n comes out as 2 instead of 6. They need a preference that replaces the sizing goal without becoming a soft count.
- **Bulk mesh commit** (`sm-layer-api`, `sm-mesh-api`): bulk adopt/retire and `MeshBuild::add_template_cells`. A 10⁶-cell `mesh.run`+`mesh.delete` takes 10.6 s, against 4.7 s for the driver alone.
- **`ModelLayer` from `(TopoHost, Model)`** (`sm-layer-api`): lets sm-fem test against `sm_testkit::topo::mirror_chamber()` instead of an in-test op.
- **Overlay `MeshRead` for export** (`sm-mesh-api`): sm-fem's `clone_host` is O(mesh) per export and has not been measured at 10⁶ cells.
- **Element type in `.smsh`** (`sm-meshio` format): it is checked at export but not written. Fold this into the polyhedral-format work.

## Features
- **`mesh.plan` / explain query:** report which scheme ran or will run on each entity (`mesh.info` has counts only).
- **`mesh.run` on faces alone** (Cubit `mesh surface`). Needed for QUAD4 and the P0 surface-only map.
- **Tri/tet meshing,** for the P0 TET4 and QUAD4 element types.
- **Script record:** write a session's op history out as a `.smsh` script (PARITY A, P1).
- **`smesh serve --ws`:** forward to sm-server's `ws` feature.
- **Sequential meshing against meshed neighbours:** replaces the `mesh.run` neighbour guard (decision 9).

## Small owner items
- sm-server `write_atomic` uses a fixed `.part` temp name. Use a unique one (pid + counter) so concurrent exports are safe.
- sm-mcp: an engine-backed variant of `respelled_ids_feed_back_as_arguments` (needs a real-kernel dev-dependency).
- sm-mesh `contract_alloc`: the warm-up before the measured pass weakens the zero-allocation contract. Drop it (the per-thread counter alone fixed the flake), or assert that the cold pass allocates nothing too.
- A `cargo fmt` pass over sm-mcp and sm-scheme-api (pre-existing drift; CI does not check fmt).

## Validation, quality diagnostics, exporter conformance (user direction, 2026-10-25)
Principle: our own checks decide whether a mesh is valid and how good it is. External tools (OpenFOAM `checkMesh`, Fluent readers, ParaView/meshio) only judge an exporter. If our validator passes and `checkMesh` fails, the bug is in the OpenFOAM exporter. Exporters own each target's quirks (e.g. OpenFOAM's 7-point `wedge` vs 6-point `prism`, Fluent c0/c1 orientation). Internal cells stay exact. Exports are never tuned to one tool.

- **Validity exists; expose it.** `sm_mesh_api::validate` + `validate_geometry` already cover checkMesh's topology part and more (about 35 stable codes: closedness, incidence, orientation, positive volume, normal direction, planarity, classification). Add a `mesh.validate` op (CLI + MCP) that returns the violations with code, entity id, classification (which surface/volume), and location. Every mesh-producing op must pass it in tests, so "our mesh is invalid" is always our bug.
- **Finite-volume face metrics** (`sm-quality`): non-orthogonality, FV skewness, face warp/flatness, face concavity, neighbour volume ratio, cell determinant, interpolation weight. Write each formula in `DESIGN.md` next to the checkMesh / Fluent definition, so our numbers can be cross-checked against `checkMesh` output, not just its pass/fail.
- **Quality diagnostics op** (`mesh.quality`, CLI + MCP): summary, histogram, worst-N cells with location, aggregation per geometry entity (through classification: "bad cells cluster on surface 12 near curve 7"), threshold profiles (generic, openfoam, fluent, transport), and suggested actions (intervals on curve X, scheme change, smoothing). It can store the offending cells as a group, so follow-up ops (smooth, refine) can target them.
- **Exporter conformance harness** (CI, Docker OpenFOAM v2506): for each fixture, our validator passes → export → `checkMesh -allGeometry -allTopology` (polyMesh) / `fluent3DMeshToFoam` + `checkMesh` (Fluent) / meshio round-trip (VTU, CGNS). Any external failure is an exporter bug. Also compare max non-orthogonality and skewness against our metrics within tolerance. References: `polyDualMesh` (tet → poly), Gmsh (tet quality).

## Exporter format rulings (user, 2026-10-25)
- **OpenFOAM polyMesh export: ASCII only.** No OpenFOAM binary writer; users who want binary run `foamFormatConvert` (OpenFOAM's own tool).
- **Fluent `.msh` export: ASCII first** (spec: `docs/research/fluent-msh.md`). Binary waits until the open binary questions (§14 items 5–8) are settled against Fluent itself.
- **Fluent gap:** the user will try to get Fluent (trial or student licence). Checklist once available: (1) load `docs/research/fluent-msh/*.msh` and the exporter's fixtures, confirm no orientation or volume complaints; (2) save a Fluent-written ASCII and binary `.msh` of a small mixed mesh as golden files (settles binary newline, int width, 39 domain-id, solid zone type); (3) check that a polyhedral (type 7) mesh and a non-conformal interface load.

## Ray tracing and delta tracking (W3b, from docs/research/dagmc-internals.md and delta-tracking.md)

- **One segment iterator, two modes.** Tracking yields segments `{t0, t1, cell_or_node, Σ_M}`. Surface tracking is the special case where a segment is one cell and Σ_M = Σ_t, so nothing is rejected. Delta tracking uses a node's local majorant. Switching mode along a flight is unbiased.
- **Majorant hierarchy.** It must be a disjoint partition traversed by DDA: lattice/universe nodes, plus a kd-tree or octree over meshed regions. A BVH has overlapping boxes, so it serves point location but not majorant segments. Nodes store bounds and a deduplicated material-set id; majorant tables are keyed by (material set, energy band). Additive to `sm-rt-api` (new provided trait or new trait).
- **Hinted point location.** At a tentative collision, location starts from the node that owns the majorant: O(1) lattice index, a binary search on r² for pin rings, or a walk or small BVH inside a meshed leaf. Never from the root. OpenMC's delta-tracking PRs (#3971, #4044) restart at the root, and that is why their fusion case got slower.
- **Region jump.** In a large homogeneous region DAGMC makes one O(log T) ray fire while a mesh walk pays per cell. When the caller does not need per-cell segments (Monte Carlo without mesh tallies), add an exact per-region boundary structure for one-jump crossings. MOC/Sn and mesh tallies keep the cell walk.
- **Safety distance.** The distance from a point to a convex cell's planes is an exact lower bound. Skip the intersection test when the sampled flight is shorter. This is DAGMC's signed-distance-field prototype (Shriwise 2017), but exact and without a grid.
- **Hybrid policy.** Use Serpent's rejection test (`dt` 0.9) at run time, plus a per-node, per-band cost model decided at build time; both depend only on (node, material, energy). A majorant violation is an error, never clamped. Use track-length estimators on surface segments and collision estimators on delta segments; mixing both in one tally needs tests.
- **Benchmark honestly.** Compare segments per track × cost per segment against one ray fire per track. Compare against DAGMC `ray_fire` (MOAB and double-down/Embree) and XDG `walk_elements`; XDG already walks tet/hex meshes, so cell walking alone is not novel. Our claims rest on exact analytic faces on polyhedral cells, no restart from the tree root, native instancing, hinted location, determinism and flat memory.
- **Serpent gaps.** The research could not reach serpent.vtt.fi (HTTP 403). STL point-in-solid, the adaptive search mesh and polyhedral-cell location are unverified. The user can paste the wiki pages "Delta- and surface-tracking", `polymesh.html` and `check_stl.html`.

## From W3a integration (2026-10-31; details in `w3a-report.md`)
- **OpenFOAM INT-8 on one-layer 2D cases** (sm-meshio, sm-fem): every cell of a one-layer hex or wedge mesh is split. Add an export option to disable the split and a 2D fixture.
- **Export notes in `smesh export --json`** (sm-cli/sm-server): then assert the INT-8 split count against `smeshCellMap` in the milestone.
- **Warped dual faces** (sm-meshio): split into planar pieces in the OpenFOAM exporter; measure INT-9 counts.
- **Transport diagnostic** (sm-quality/sm-diag): flag cells with two or more boundary faces.
- **Smooth suggestion** (sm-diag): `mesh.quality` should suggest `smooth` for spread scaled-Jacobian warnings on real meshes.
- **sm-scheme-tet:** `recover_facet` rounding (n=64 cliff); `RecoveryFailed.tri` placeholder; pass-level respace unit test; O(n·hints) restart check.
- **sm-server:** `Server::open` binds another builder's pending store.
- **Contracts:** `SchemeNote` / `Patch::notes` (sm-scheme-api); `MeshEntry.derived_from`.
- sm-cadio wall-clock tests → work counters; sm-poly typed error for unrepaired cells.

## From the post-W3a contract batch and W3a follow-ups (2026-11-04)
Merged: contract batch 55a20a9; follow-ups foam2d, tet, server, cadio, poly (a55b1ed..ee78766).

Rulings proposed for W3b-0:
- Firmness cascade (decision 9): resolve at command time in `mesh.size`/`mesh.intervals` so stored intent is order-free; flip `mesh.run` to Firmness only after a milestone A/B with unchanged hashes.
- Re-bless the `mesh.quality` output schema golden once to publish `scheme_notes` / `n_scheme_notes`.
- Move scheme notes from `Plan` to an unhashed `MeshEntry::notes`; until then clear `plan.notes` in `mesh.to_poly`.
- sm-op-api ACR: `ApplyCtx::retire_range` for bulk delete (10⁶ cells: 7-11 s today); test hash equality with per-id retire.
- sm-protocol ACR: `session.export` gets `split_small_determinant` (default true) and a way to name empty/wedge patches; sm-fem emits them. Add 2D conformance fixtures (incl. the unsplit tri-prism slab).

Cleanups:
- sm-server `host.rs`: move `notes_of` above `write_dir`'s doc comment.
- sm-scheme-api: `SchemeNote` `#[non_exhaustive]` + constructor; `MeshIntent::set_intervals` running counter (O(n²) today).
- sm-mesher DESIGN.md "Scheme notes" text is stale; sm-diag dev-deps comment.
- `ExportNote::parse` loses the code when the count is missing.
- sm-meshio: misleading comment in `tests/foam_2d.rs`; FOAM_FORMAT.md §8 "six above" needs the list; §11 open follow-ups.
- sm-scheme-tet: n=64 cliff test runs only in release (CI is debug); hint-index work-counter test; record path B smoothed tet SJ 0.1848.
- sm-cadio: refusal tests no longer bound parse work; per-run Newton cost not counted; tighten `global_projections <= 16` to MAX_GLOBAL; `sm_cadio::work` module/function name clash; global atomics on the public API.
- sm-poly: map `UnrepairedCell` to `OpError::Failed`, not `Invalid`; `tests/robust.rs` asserts nothing about `region`; agglomeration can make invalid cells after the check.
- Commit a268047 has a Sonnet co-author trailer (process note only).

## Dispositions after W3c-INT (2026-11-08)

Legend: done (with the commit or task), kept (stays as the current behaviour), W4 (with its trigger). Items from the W3b-0, W3b-INT and W3b report open lists that were already done or void are listed once as done.

### From W3b-0 and the W3b report open items
- **Done / void:** sm-poly `with_unchecked_merges` behind `test-hooks` (2d1e1ea, POLY2); dev-only cycle sm-cadio to sm-server in `crates/sm-cadio/DESIGN.md:301-308`; test-only `unsafe` rule in CLAUDE.md; Sonnet co-author process note (accurate, kept).
- **Done in W3c:** ABI 10200 for `SMRT_NODE_UNKNOWN` with the rule in `sm-rt-capi` DESIGN.md (CLEAN-RT); driver tolerance comment and `ridx` guard (CLEAN-RT); acc04 brackets `next_segment` (CLEAN-RT, finished in INT FIX-RT: counters cleared, `bvh_descents == 0` asserted); 503 refusal off the accept thread with an in-flight cap and the two-thread `OPENING` test (CLEAN-SERVER, tightened in INT FIX-SERVER); `smrt_regions` duplicate-name test (EXPORT-A); `attributes: false` record: DESIGN.md wording fixed, code unchanged, because the node hash covers the params (CLEAN-CADIO, f256cad); fallback note reaches CLI, MCP and Python (EXPORT-A/B/C, C10).
- **Kept:** volume `mesh.size` / `mesh.intervals` do not reach a face-only `mesh.run` (C14, with the `mesh.volume_request_ignored` warning). Complement name without a length prefix (R2; re-open only if leaf keys are ever re-based). DAGMC reference h5m volume 177 is in two groups: note for the export docs only.
- **W4 triggers:**
  - Re-plan after a failed face run is not budget-checked (sm-scheme-api ACR for a face branch in `Driver::estimate_cells`). Trigger: a face run seen to blow up on a replan. The replan count is already capped.
  - sm-boolean `merge.rs:390` O(F^2) face-box scan. First task: a benchmark at 10^5 faces; then the output must equal the current `face_pairs`.
  - `retire_mesh_bulk` into `sm_layer_api::retire_mesh` (frozen crate). Trigger: a second production caller.
  - `mesh.delete` parallel sum (4.5 s at 10^6 cells, one SHA-256 per id; needs rayon in sm-op-api and an order-independent combine). Trigger: a requirement on 10^6-cell delete time.
  - `RegionInfo.tags` not in cache images and hashed without a presence marker. Trigger: the first producer (for example DAGMC property import); fix together with a cache-image version bump.
  - P1 limits tests (`sm-mesher/DESIGN.md:79-87`). Trigger: any code that loads `IntentLayer` through serde.

### Remaining W3b report open items (dispositions)
- **Region granularity of the `.smrt`** (per-volume region tag option): kept. Regions are one per block; no code has asked for per-volume regions. W4 trigger: the first request for it.
- **Node 20 run of the sm-view web job:** W4. Trigger: the next edit of the sm-view web job or CI image change.
- **Re-run of the C driver and the Python row on the Linux benchmark machine (O11):** W4. Trigger: access to the Linux benchmark machine. The W3c smoke was on macOS only (Q9).
- **sm-rt 2D lattice z pitch 1e9 sentinel; O(surfaces)/O(cells) analytic void/complement stepping and cold locate (cell-box BVH):** W4. Trigger: a benchmark or user model where analytic stepping or cold locate dominates.
- **Benchmark items:**
  - `smtets_to_moab`: W4, trigger: the XDG tet walk work (the same tet inputs are needed).
  - P1 `facets` row partial at `tiny` (zero-area triangles in the hand-built moderator): kept, the reduced and larger scales are used for the smoke.
  - Linux `smcase.json` hash lines unblessed: W4, trigger: the first Linux benchmark run (Linux mesh hashes are covered by `tools/platform/expected.txt`).
  - Docker digests UNSET: done in W3c (BENCH-PIN; the images record their sha256 id, and FIX-BENCH reads it once).
  - Keep `date_utc` and the sequence counter out of hashed outputs: kept; W4 trigger: the next edit of the bench result writer, which should check both are outside `result_hash`.
- **Not scheduled:** the WAIT list, hex lattices (Q3 stretch) and server push (O16) stay unscheduled; W4 trigger for each is an explicit user request. The MCNP importer is dropped (MCNP is not a target, OpenMC is; the OpenMC import is done, sm-csgio). Overlay `MeshRead` stays in Contract changes above.

### From W3b-INT acceptance (done in W3c-INT)
- Done: `w3b-report.md` deviations numbered D1..D8, GitHub CI recorded as red with causes (local `tools/ci.sh` rc = 0, 59 to 70 minutes, kept apart), Chrome version recorded (the run did not record it; the installed app reads 156.0.8078.80), repro recipe corrected (`model.subtract` without `keep`, then imprint and merge).
- Done: PARITY Transforms row reworded (volume counts and total volume).
- Done: `w3b_driver.c` tolerance comment and `ridx` guard (CLEAN-RT).
- Done: `smrt_regions` set-name test (EXPORT-A); R9 to R11 follow-ups (fallback note surfaced, FOAM-WARP option, `split_warped_faces`).
- Done: W3b plan B6 wording amended per C2.

### From W3c sub-waves 1+2
Done in W3c-INT (merged with this report):
- XDG pin back at `83efb306e152` (C15, PR #3).
- Test tightening: sm-py `test_export.py` marks and `NO_SERVER_A` dropped, gmsh `binary=True` byte-compared with the CLI (FIX-PY); `plan_blame.rs` asserts the `ReplanLimit` text (FIX-MESHER); auto_select plan rows, warning text, `rejections == 0`, three-way hash for the single through pin, the lattice, the enclosed pin and B5, transport stall bound and rendezvous timeout (FIX-SERVER); sm-csgio 1-vs-4-thread findings, 1e-11 tolerance, `translated` fixture, row-swapped and transposed-rotation oracle variants, cli precedence (FIX-CSGIO); delta.rs acc04 (FIX-RT).
- Small fixes: LINUX-HASH (toolchain pin, key counts, `error.*` check, comment) (PLATFORM); sm-csgio `world.rs` debug_assert and DESIGN.md step 5 (FIX-CSGIO); sm-rt-capi DESIGN.md `smrt_version` and `guard` (FIX-RT); image id read once (FIX-BENCH); complement-leak limit and `vols[r0]` bounds check (BENCH-WORKLOADS follow-up, 785e765); MESHER-REPLAN trial error reused (FIX-MESHER); `plan.md:142` wording (C9).
- Temp dirs: capi tests (CLEAN-RT) and the sm-cli `smesh-cli-*` and `smesh-full-*` helpers (FIX-CLI-TMP) use `CARGO_TARGET_TMPDIR`; the PIN, server and bench tests clean up by Drop guards. The root cause of the old empty `smrt-capi-*` dirs is still unknown. The sm-csgio leak could not be reproduced (`gen.py` hardened, not run).
- sm-bench: `cases.rs` `only_tiny_p1_lines_of_the_recorded_hashes_changed` compares a BTreeMap of the shared keys (FIX-BENCH, af85afe); images with no `moab=` line are reported (`moab_mismatch` in `docker.rs`, test `moab_pin_check_reports_a_missing_or_wrong_version`; FIX-BENCH, af85afe).

Carried (new follow-ups from the INT reviews; none blocks W4):
- W4, test hygiene, trigger "next edit of that file":
  - `milestone_w3c_pin.rs`: per-block fuel volume against the polygonal-cylinder area times H (the total volume is the same for any fill); a no-blocks fixture (detail "no blocks") next to the left-out one; the `ENCLOSED_HASHES` lookup uses `enumerate` instead of `position(..).unwrap_or(0)`.
  - `tools/w3c_driver.py`: run mypy and ruff.
  - auto_select through-pin plan rows: assert the `copy_of` values are distinct.
  - sm-bench `absorber_unbiased` and `smrt.rs`: assert `image_id` starts with `sha256:` (the FIX-BENCH report claimed it; the diff has none); in `run_competitor_with` check the sha256 before the platform; tighten the flight `lost` count to the rays that meet the unmapped sphere.
  - sm-mesher `auto.rs`: note that `known` assumes `Driver::plan_with` is pure; optional test that counts `plan_with` calls.
  - sm-py: pytest keeps `$TMPDIR/pytest-of-$USER` (pass `--basetemp` in `py.sh`).
  - sm-cli `target/tmp` collects per-pid dirs; add a Drop guard or fixed names. Other crates (sm-engine, sm-rt, sm-mcp, sm-server, sm-cadio, sm-rt-bench) still use `temp_dir()` in places: move them to `CARGO_TARGET_TMPDIR` when next touched.
  - sm-csgio `gen.py`: SIGTERM exit code 143; `openmc.reset_auto_ids()` before each model so a subset run gives byte-identical `geometry.xml`.
  - `RegistryError::BadOpName` is used for a test-only timeout in `standard.rs`; replace if a generic variant is ever added.
  - `sm-rt-capi` DESIGN.md:111 is 118 columns wide.
- ~~Linux x86_64 evidence~~ done: `platform_pins.rs` passed on the x86_64 GitHub gate (runs 42544784727, 42553841653). A future failure follows C12.
- W4: `smesh run` text mode prints per-op `delta.diagnostics`, so a CLI user sees the `mesh.auto_tet` warnings. Trigger: the first user report of a silent tet fallback (the JSON and MCP paths show them today).
- W4 deferrals recorded in the W3c report: H1 analytic case, P2 tet twin, `--model` periodic faces, XDG tet walk and driver at the newest upstream (C15), O-grid all-hex pins (C1), hex-tet transition cells, `hex_only` / `tet` strategies (C5), CSG B-rep import (C8).

## From W3c-INT review (2026-11-08; small, take with W4)
- `milestone_w3c_pin` P5/P6/P9 skip with only a println when maturin is missing; fail instead when `CI=true`.
- sm-py: pytest's basetemp retention leaves `pytest-of-$USER` in `$TMPDIR`; pass `--basetemp "$TMP/pytest"` in `py.sh`.
- Per-pid `smesh-cli-*`/`smesh-full-*` dirs now collect under `target/tmp`; add a Drop guard.
- sm-csgio `gen.py`: call `openmc.reset_auto_ids()` before each model, so fixtures regenerate byte for byte per model; exit 143 on SIGTERM.
- sm-server auto_select: assert distinct `copy_of` values in the through-pin plan rows.
- sm-rt-bench: check the `sha256:` image id before the platform; assert the id in `absorber_unbiased`; give an exact lost count in `flight_step_into_an_unmapped_region_is_lost_not_a_leak`.
- tools/platform `collect.py:58`: `error.new` should go through `last_line(r)`; record the macOS `rustc -V`; `cargo fmt` `platform_pins.rs`.
- sm-mesher `auto.rs`: note that the `known` shortcut assumes `plan_with` is pure.
- PIN: check the fuel-block mesh volume analytically; add a no-blocks CLI/Python check; mypy/ruff on `tools/w3c_driver.py`.
