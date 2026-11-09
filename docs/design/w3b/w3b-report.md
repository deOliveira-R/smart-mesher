# W3b wave report

Date: 2026-11-05 (integration), W3b-1 merged 0241002, W3b-2 b67f107 and 4286563, W3b-3 40c36c0 (tip a5e6768 plus this integration). Acceptance: the ten milestone rows of `crates/sm-cli/tests/milestone_w3b.rs` against `plan.md` section 5, item 1; items 2 to 6 below. Rulings: `rulings.md` (Q1-Q14, O1-O17, P1-P5) and the orchestrator rulings R1-R8 (`rulings.md` and `notes-w3b3.md`).

## What W3b delivered

Three waves of crates and crate deltas, merged before integration (details in `notes-w3b1.md`, `notes-w3b2.md`, `notes-w3b3.md`):

- **Transport handover.** `sm-rt-api` (fd4a235 plus grafts: resumable cursor, material sets, `block(node)`, safety distance, `REGION_JUMP`), `sm-rt` world walker, majorant partition, delta flights, analytic CSG leaf with overlap check, Model to CSG bridge (`rt.check_overlaps`, `rt.partition`), the `.smrt` image reader and writer; `sm-rt-capi` C ABI v10100 (`smrt_open`, node sets, region names, two-phase flights, `smrt_flight_check`); `sm-rt-bench` harness with tracker rows (published table waits for the Linux machine, O11).
- **Automatic meshing.** `mesh.auto {strategy}` with the `default` strategy (surfaces map/submap/pave/trimesh, volumes map/sweep/tetmesh, `mesh.auto_tet` warning), choices stored as explicit assignments; surface-only `mesh.run` and `mesh.plan` (Q13 guard); SUBMAP on periodic faces and graded neighbours; PAVE corner rule and valence cleanup; SWEEP and TET support; POLY2.
- **Model.** Remaining Cubit primitives (frustum, prism, pyramid, torus, `create_sphere_part`), `model.move_to`, `rotate_about`, `model.copy`; `model.export` STEP as a read-only query; STEP names and colors (CAD-ATTR).
- **Exports.** DAGMC `.h5m` (`mat:<set>`, opt-in graveyard), Gmsh msh4 (ASCII and binary), the `.smrt` handover image through `session.export`; FOAM-WARP (`split_warped_faces`, library only).
- **Diagnostics.** `mesh.quality` transport additions, `mesh.flag_cells` / `mesh.unflag_cells`, `mesh.smooth {set}`.
- **Tooling.** Python package `smartmesh` (abi3 wheel, typed ops), `script.record` / `smesh record` / MCP `record_script`, the read-only WebGL2 viewer (`crates/sm-view/web`) served by `smesh serve --ws` with an Origin rule and a 64-connection cap.

W3b-INT wired nothing new into a crate except D1 below. It added the milestone (script, C driver, Python driver, test), the `--only dagmc|gmsh` conformance hook, the viewer smoke, and the PARITY rows.

## Acceptance results

Milestone (`cargo test -p sm-cli --test milestone_w3b -- --nocapture`; debug build; default threads and `RAYON_NUM_THREADS=1`):

| Row | Result | Evidence |
|---|---|---|
| M1 pin cell, 3x3 lattice by `model.copy` | pass | script asserts 5 volumes per pin cell, 45 in the lattice, 9 fuel and 36 moderator, volume 9 x 1.26^2 (14.2884) |
| M2 `mesh.auto` explicit assignments, replay calls no strategy | pass | the recorded `mesh.auto` holds 63 assignments (18 sweep) and ends with `strategy: "default"`; the record replays to the same hash; the same record with the strategy renamed `no_such_strategy` also reaches `h_mesh`, so no strategy ran |
| M3 face-only `mesh.run` and `mesh.plan` | pass | on a side branch: `will_run` before, `ran` after, 20 quads on one face |
| M4 `.h5m` (graveyard), `.msh` ASCII and binary, `.smsh` | pass | HDF5 signature, `mat:fuel` / `mat:water`, `mat:Graveyard` only with the option (h5m 138602 bytes, 141502 with graveyard); `4.1 0 8` and `4.1 1 8` headers (131028 and 188124 bytes); `.smsh` decodes to 675 cells |
| M5 `model.export` STEP | pass | 318205 bytes; re-import gives 45 volumes, worst relative difference 9e-16; state hash and node unchanged (script asserts) |
| M6 C driver through libsmrt | pass | regions `fuel` and `water` read by name (D3), own sigma per name (1.5, 0.5), 1117 nodes, 3 sets (1 with mixed sigma); (i) 10000 of 10000 lines sum to the box chord for the surface and for the delta flight, worst absolute error 5.3e-15 and 1.3e-14 against a tolerance of at least 1e-12 (66170 delta segments); (ii) 2 of 2 regions within 4 sigma, worst z 0.29 (40000 histories per estimator, fixed seeds); (iii) understated majorant gives -12 and the state is unchanged |
| M7 `script.record --forks refs`, replay equal | pass | `smesh record --verify` and the replay of the recorded script (3 tip checks) pass; the MCP `record_script` text equals the CLI's; repeats SERVER-HOST acceptance 2 on the real milestone |
| M8 the slice from the installed wheel | pass | the wheel (`maturin build`, installed in a venv) runs the script to the same three hashes and the same five files; typed Python ops (`model.copy`, `ops.mesh.auto`, `ops.mesh.run`) reach `h_model` and `h_mesh`; the Python `.smsh` reader sees 675 cells |
| M9 CLI, MCP, in-process, Python equal | pass | `h_model` 66e4ac03a259d7e8, `h_auto` fca39b7908faae61, `h_mesh` 6f6c19f042a4b235 (leading 16 hex digits) and the five files byte-equal in all four |
| M10 default threads and `RAYON_NUM_THREADS=1` | pass | the same hashes, every printed line and the five files |

Other items:

- **Goldens (item 2).** No op or query was added by the integration, so no op-listing golden moved: `cargo test -p sm-mcp` passes against the committed tool list and session goldens (the combine steps of W3b-1 to W3b-3 had already regenerated them, additions only). `crates/sm-py/python/smartmesh/ops.pyi` lagged the merged ops and was regenerated with `crates/sm-py/tools/gen_stubs.py`: 25 added lines and one changed line, all additions (`mesh.auto`, `mesh.unflag_cells`, `mesh.auto_strategies`, `rt.check_overlaps`, `rt.partition`, and the new `attributes` keyword of `model.import_step`). No hash or existing output schema moved. W2 milestone, `p0_rows` and `milestone_w3a` pass unchanged.
- **Viewer live smoke (item 3).** `viewer::viewer_live_smoke` (same file): against `smesh -p <milestone project> serve --ws`, an upgrade with `Origin: http://evil.example` gets 403, so does the served host on another port; the served origin and a client without an `Origin` are accepted and `session.hash` equals `h_mesh`. The page runs in headless Google Chrome (SwiftShader) through `tools/w3b_viewer_smoke.mjs` and playwright-core 1.49.1 (installed on demand into `target/w3b-viewer`); Chrome version: the run did not record it, and the installed app on the same machine reads 156.0.8078.80 (checked 2026-11-08, W3c-INT), so treat that as the likely version: it draws 3708 skin triangles, equal to the server's `view.mesh_skin`, and follows an `undo` and a `redo` of the head within the poll interval; no page error. Without playwright-core or a Chromium the test prints `SKIP browser: <reason>` and runs the protocol path alone (checked with `W3B_PLAYWRIGHT_DIR=/nonexistent`). The Node 20 run of the sm-view web job (W3b-1 hand-off) was **not done**: only Node 25 is installed here.
- **Conformance (item 4).** `SM_CONFORMANCE=1 tools/conformance.sh --only dagmc`: 5 pass, 0 fail (h5py on `two_volume`, `hex_blocks`, the milestone and the milestone with graveyard; MOAB and DAGMC in `smesh-dagmc-conformance` on `two_volume`). `--only gmsh`: 14 pass, 0 fail (Gmsh and meshio on the six `export_conformance` cases in ASCII and binary, and the milestone in both). Full run: 47 pass, 0 fail, `mandatory_ok` (the 28 W3a checks, OpenFOAM v2506 in Docker, and the 19 new ones). `SM_CONFORMANCE_DAGMC_IMAGE=no-such-image` exits 3 with the build command in the message; with `SM_CONFORMANCE_ALLOW_SKIP=1` it exits 0 and reports Tier B as skipped (4 pass, 1 skipped). Tier A ran in the venv from `requirements.txt` on CPython 3.13 (`h5py==3.16.0`, `gmsh==4.15.2` installed as pinned).
- **PARITY (item 5).** Ticked, each with test names: A script record, A Python API, B primitives, B transforms and copy, B STEP export (IGES and facets stay open), C auto selection, E Gmsh msh4 and DAGMC (Exodus netCDF4 and Abaqus stay open), F web frontend (remote backend stays open).
- **`tools/ci.sh`:** see "CI" below.

## Timings (record only; debug build, shared loaded machine)

| What | Time |
|---|---|
| Milestone test, warm (the ten rows, six front-end runs, C build and run) | 23.0 s |
| Same, first run in a clean target (venv, pip, `maturin build`) | 76.0 s |
| CLI `smesh run` of the milestone script: default threads / 1 thread / project | 3.8 s / 2.3 to 2.5 s / 2.7 s |
| MCP / in-process / Python wheel run of the same script | 2.3 s / 2.7 s / 2.8 s |
| Replay of the recorded script | 0.7 s |
| C driver (cargo build of `libsm_rt_capi` already done, compile and run, 10000 lines and 2 x 40000 histories) | 2.2 s |
| `viewer_live_smoke` with Chrome | 23.8 s (5.0 s without the browser path) |
| `submap_graded_input_is_pinned` | 0.1 s |

Reference sizes: 675 hexes, 1024 nodes, `.smrt` 491648 bytes with 1117 nodes, 3 material sets and 2 block-named regions (400192 bytes before D1 without the partition; 518080 bytes with volume-named regions), `.smsh` 240256 bytes. Conformance and `tools/ci.sh` durations are in the hand-off.

## Deviations from the plan text

1. **D1: `session.export` `.smrt` now carries the majorant partition** (`crates/sm-server/src/mesh_export.rs`, `smrt_build_options`; test `crates/sm-server/tests/export_wire.rs` `smrt_loads_to_the_in_process_tracker`). Reason: the exporter built the tracker with default `BuildOptions` (partition off), so `smrt_open` returned a tracker that answered `SMRT_ERR_UNSUPPORTED` to `smrt_n_nodes` and `smrt_node_set`, and no delta flight could run on the file the milestone is supposed to hand over. Effect: **every `.smrt` written by `session.export` changes bytes** (the A/B against a5e6768 will show it); no state hash moves; no committed golden pins those bytes (searched `crates`, `tools`, `docs`; the `.smrt` pins in `sm-rt-capi` and `sm-rt-bench` are for their own cases). The test's reference tracker is built with the same options, and `rt.info` (default build, no partition) and the exported image now have different structure hashes.
2. **D2: The pin cell is a square pin, not a cylinder.** The natural cylinder-in-brick pin cell cannot be meshed by the `default` strategy today: a through-hole makes faces with inner loops (the B5 limit), and a cylinder entirely inside the moderator fails `mesh.auto` with the internal-looking error `face FaceIx(0) has inconsistent orientation between its cells`. The milestone pin is a 0.8 x 0.8 x 1.0 fuel brick in four moderator bricks (pitch 1.26), 5 volumes per cell, 45 in the lattice, all hex (27 map volumes, 18 sweep volumes). Both failures are open items below.
3. **D3: regions of the `.smrt` are named by block (material), resolved (ruling Q2).** First run: `session.export` built regions with `RegionRule::ByVolumeClassification`, so names were `TopoVolume#<n>` and the C driver needed a pseudo-table. Now `smrt_regions` in `crates/sm-server/src/mesh_export.rs` lowers the model blocks onto the mesh (the same sets that become `mat:<name>` in `.h5m`) and builds the tracker with `RegionRule::ByCellSets` over the block names when the blocks cover every cell; otherwise (no blocks, a cell in no block, or a lowering error) it falls back to volume ids and returns the export note `smrt-region-names: ...`. Tests in `crates/sm-server/tests/export_wire.rs`: `smrt_loads_to_the_in_process_tracker` (regions are `fuel`, `water`; structure hash equals an in-process build) and `smrt_regions_fall_back_to_volume_ids_without_blocks`. The C driver reads `fuel` and `water` by name and assigns its own sigma per name (1.5 and 0.5). Effect: `.smrt` bytes change again for models with blocks; no state hash moves (milestone hash 6f6c19f0 unchanged); no committed golden pins those bytes.
4. **D4: C driver build.** `crates/sm-cli` has no dependency on `sm-rt-capi`, so the test runs `cargo build -p sm-rt-capi --target-dir target/w3b-capi` itself and links `tools/w3b_driver.c` against the cdylib with `cc`. No `Cargo.toml` was edited. The wheel is built by `tools/w3b_python.sh` into `target/tmp/w3b-venv` (pip needs the network once; exit 3 and a printed skip if maturin cannot be installed).
5. **D5: `tools/ci.sh` runs the `stubs` configuration of sm-rt-capi in its own target directory** (`target/capi-stubs`). The cdylib is written to `deps/libsm_rt_capi.<ext>` without a hash, so two feature sets in one directory would leave the later tests with the wrong library.
6. **D6: STEP re-import equality is checked to a relative 1e-9** (measured 9e-16), not bit for bit: `model.measure` of the re-imported volumes differs in the last digits.
7. **D7: `session.export` from Python has no `options`**, so the graveyard `.h5m` and the binary `.msh` are written through `smesh export --option` only; the wheel row compares the default exports.
8. **D8: Node 20** was not available; the viewer smoke ran on Node 25.

## Rulings at integration

Carried in: Q1-Q14 (`rulings.md`), O1-O17 as ruled on 2026-11-04, P1-P5 approved, R1-R3 (W3b-2) and R4-R8 (W3b-3). Applied by the integration:

- **O1** (handover file is the `.smrt` written by `session.export`): used as ruled; see D1 and D3 for what the file must carry.
- **O5** (`model.export` is a read-only query): used; the script writes `milestone.step` through the query's `path`, and the node and state hash are asserted unchanged.
- **O13 / R6** (additive goldens): only `ops.pyi` was regenerated, additions only.
- **O4 / O10 bless list:** nothing moved.
- **R3:** flag then unflag restores content and sets, not the state hash; noted in `crates/sm-diag/W3_REPORT.md`.
- **R8:** B5 and B6 stay pinned; the milestone does not touch them.
- **SUBMAP version question (open item of `notes-w3b2.md`): decided, no version bump.** `mesh.run` and the mesher service version stay as they are; the graded-input output is pinned by `milestone_w3b.rs` `submap_graded_input_is_pinned` (an L-slab sweep with a `submap` source whose loop has one edge at 0.2 against 0.5: 36 hexes, topology hash `c99bcaa8d93e9a5e`, geometry hash `43ecc0a4b59b3715`, state hash `7b71fd6dd5c3dd71...`, also at one thread). If a later change moves those hashes the submap output moved again and a bump needs a ruling. The pin was blessed at this tip; it does not by itself prove that the fixture differs under the pre-SUBMAP code (the unit test `graded_cut_line_follows_the_neighbouring_boundary_spacing` in sm-scheme-submap covers that).

## Advisories

- Hashes of the milestone (debug build, arm64 macOS): `h_model` 66e4ac03a259d7e8411d7365470a6917770db7ae5fd57492aba72f04fe1e204e, `h_auto` fca39b7908faae617c67ba6ae5ee63f457225ac56982ddf21c5d955e40f469c4, `h_mesh` 6f6c19f042a4b23529cfa0d05ff6c4bbd212e9862fefebacf18617f682b7ea2f. They are the baseline for the P1 firmness A/B (rulings P1: flip `mesh.run` only with unchanged hashes).
- `mesh.auto` chose `sweep` for 18 volumes (the two full-height side strips of each pin cell, volumes 2, 3, 8, 9, ... of the record) and `map` for the other 27; the cause was not investigated (a guess: the imprint with the neighbours splits the strips' faces, so `map` sees more than six). All 675 cells are hexes either way.
- A face-only `mesh.run` ignores the volume's `mesh.size` (the known sm-mesher gap): the milestone sets `mesh.size` on the face before the face run, otherwise the face is one quad.
- The delta/surface agreement of M6 is statistical (4 sigma, 2 regions, worst z 0.29); the seeds are fixed, so the row is deterministic on a given libm.
- Flux per region uses the code's own sigma per material name (fuel 1.5, water 0.5); nothing about cross sections is known to smart-mesh or sm-rt (Q2).

## Open items carried to W3c / W4

From this integration:

- **mesh.auto on a cylinder-in-brick volume** fails with `face FaceIx(0) has inconsistent orientation between its cells` (an internal-looking error rather than a scheme refusal); the pin-cell variant with a through-hole is the B5 limit. W3c micro task in sm-mesher with `DefaultStrategy` (together with B6 below). Reproduce: `model.create_brick`, `model.create_cylinder` (height smaller than the brick), `model.subtract` (without `keep`), imprint, merge, `mesh.auto`.
- **Region granularity of the `.smrt`**: regions are one per block, so two volumes of the same material share a region (the volume identity is not in the file). If a code needs per-volume regions it needs a region tag option; not requested. A model with some volumes in no block falls back to volume ids (D3 note).
- **FOAM-WARP `split_warped_faces` through `session.export`** needs a protocol option (like P5's `split_small_determinant`); no protocol edit was made. Until then it is library-only (`FoamOptions::split_warped_faces`, default false).
- **Python `Session.export` options** (`graveyard`, `binary`, `polyhedra`) for sm-py.
- Node 20 run of the sm-view web job (W3b-1 hand-off).
- Re-run the C driver and the Python row on the Linux benchmark machine together with O11.

From the three notes files:

- **AUTO:** B6 (auto next to a hard tet neighbour) fails and stays pinned: W3c micro task in sm-mesher's `DefaultStrategy` to respect triangle faces imposed by a neighbour and blame the auto volume so the tet fallback is reached (R8). B5 (plate with a hole) is a known limit. B3 second half and B4 (hex-only) are covered only by sm-mesher stub tests because `standard()` registers only `default`. Census: 7 changed cases, all error to success except one error to a different error.
- **Benchmark (O11, Linux machine):** the DAGMC drivers implement W-track only; W-fire, W-flight, the XDG driver and `smtets_to_moab` are not written; the analytic row has no H1 case and P2 has no tet twin; the P1 `facets` row at `tiny` is partial (zero-area triangles in the hand-built moderator); Linux `smcase.json` hash lines are unblessed; Docker digests are UNSET; `--model` cannot tessellate imported periodic faces yet (sm-cadio leaves pcurves unset); keep `date_utc` and the sequence counter out of the hashed outputs.
- **sm-rt / sm-rt-capi:** RT-DELTA-WORLD `tests/delta.rs` acc04 should bracket `next_segment` as well as `probe`; the 2D lattice z pitch is a 1e9 sentinel and analytic void/complement stepping and cold locate are O(surfaces)/O(cells) (a cell-box BVH is deferred); `SMRT_NODE_UNKNOWN = 5` was added without an ABI version bump; the builder writes a complement region name without a length prefix while `LeafInput::content_hash` prefixes it (R2 left them as is to keep complement leaf keys).
- **sm-server / sm-cli:** the 503 refusal runs on the accept thread (up to about 1.2 s stall per excess connection); assert in a test that the scoped `OPENING` thread-local binds stores only on the opening thread; record the dev-only cycle sm-cadio to sm-server in DESIGN.md.
- **sm-mesher:** volume `mesh.size` / `mesh.intervals` do not reach a face-only `mesh.run` (Cubit parity); a re-plan after a failed face run is not budget-checked.
- **sm-boolean:** `merge.rs:390` has its own O(F^2) face-box scan; the copy-on-write cost at 10^5 faces is open.
- **sm-poly:** move `with_unchecked_merges` behind a `test-hooks` feature.
- **CAD-ATTR:** an explicit `attributes: false` serializes into the op record; normalize `Some(false)` to `None` or fix the DESIGN.md wording.
- **CLAUDE.md:** test-only `unsafe` (counting allocators in sm-math and sm-rt tests) needs a one-line rule from the user or a test-support crate.
- **Process:** some workers' commits carry a `Claude Sonnet 5.5` co-author line (the model that ran); kept, it is accurate.
- **Not scheduled** (`plan.md` section 6): the WAIT list, the MCNP/OpenMC importers (O7, W3c), hex lattices (Q3 stretch), server push (O16), overlay `MeshRead`.

## CI

Two different things were called CI. They are kept apart here.

- **Local `tools/ci.sh` on macOS (with the new sm-rt-capi `stubs` test and clippy): rc = 0, 59 to 70 minutes.** This is the only run that was green.
- **GitHub Actions (Linux x86_64): red** on `main` from the W3b-1 merge (2026-11-05) on, including the W3b integration period (last four runs on main, for example run 42412194333). Causes:
  - `crates/sm-rt/tests/flat_golden.rs:55`: the bundle hash differs on Linux (the structure hash is equal). Addressed in W3c-0 item 0.2 (C11, per-platform bundle constants).
  - sm-py job: `test_errors.py [<lambda>5]` fails. Addressed in W3c-0 item 0.3 (PY-CI). Both causes were confirmed fixed by GitHub run 42463845479 (W3c-0 merge).
  - The W3b wave was merged on the local result alone. From W3c on, a wave merges only with GitHub CI green as well (see `docs/design/w3c/rulings.md`, process note).
