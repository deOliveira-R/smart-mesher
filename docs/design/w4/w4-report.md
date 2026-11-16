# W4 report

W4 took the project from "a kernel, meshers and a ray tracer" to the two goals of `plan.md`: polyhedral meshes for the user's transport code (read from `.smsh`), and a ray tracer (`sm-rt`, `.smrt`) with an honest benchmark against DAGMC and XDG. It ran as W4-0 (contracts), W4a (the two goals), W4b (front door, hierarchy, intervals), W4c (parity), and W4-INT (milestones, measurements, fixes). Plain language below; the plan is `plan.md`, the rulings `rulings.md`, open work `backlog-w5.md`, parity `../../PARITY.md`.

State when this report was written: tree `c5f42c0` (main `b51806f`, PR #16, plus the W4-INT merges) plus the documentation commit. **This report was written without running the full CI, the census or the milestone tests again**: the W4-INT combine runs `tools/ci.sh`, the census and the catalog re-bless once (ruling OR-e). Where a number below comes from a worker or reviewer run, the run is named.

## 1. What was delivered

| Sub-wave | PR | What |
|---|---|---|
| W4-0 | #5 | Plan per rulings, crate skeletons, contract ACRs (POLY2, RT-HEXY, RT-REGIONSETS, DAGMC-SRC, REGIONS), census baseline |
| a1 | #6 | 17 tasks: poly measure and volume diagnostics, ogrid kernels, region cut, DAGMC from the model, RT bench and world, hex strategy and decomposition, CSG classify and probe, CLI diagnostics, exodus check, reference corpus |
| a2 | #7 | Circumcentric dual, `mesh.decompose_plan`, ogrid scheme, `model.import_openmc`, RT rows and cell index, region split |
| a3 | #8 | OpenMC oracle for the B-rep import, `mesh.to_poly` on mixed meshes, RT follow-ups, ogrid in `default`/`hex_only` (D5), `mesh.preserve_volume` (VOLFIT), pin milestone, L3 benchmark rows, x86_64 script |
| W4-0b | #9, #10 | ACR-LEVELMAP, ACR-V1, ACR-V3 |
| b | #11, #12, #13 | Virtual topology ops, kernel and heal, uniform refine, agglomerate, `polyhedron` scheme, interval relations, export select, CSG CLI, RT hex lattices, native webcut, b2 fixes |
| D11 | #14 | `ogrid` for disks of 8 to 14 circle nodes (refinement keeps pin quality) |
| W4-0c | #15 | ACR-V2a, ACR-EXPFMT, ACR-CSG-HEXIR |
| c | #16 | Curvature sizing, composite meshing, Fluent / CGNS / Abaqus writers, hex lattices in the B-rep, many-to-many sweep, pyramid transition, THex/QTri/HTet/TQuad, local refine, `model.heal`, STEP repair, tidy tasks |
| INT | (this PR) | Three milestones, benchmark smoke, MEASURE table, fixes, PARITY, this report |

All 16 W4c tasks merged; every combine reported "Left out: none". No scheduled task was dropped except W4-CSG-LIBFN (not needed, `csgbrief.md` section 6) and RT-LATZ (ruling OR-a).

### W4-INT results

| Task | Result | Branch tip |
|---|---|---|
| W4-INT-POLY | Polyhedral milestone passes; two plan checks not met as written (Dev-10, Dev-11) | `2eee90d` |
| W4-INT-REGIONS | Regions milestone passes: 117 regions, fuel regions `pi r^2 h / 12` to 1e-12, C ABI tags read back | `04b704b` |
| W4-INT-BENCH | Benchmark smoke, MEASURE table, refcorpus STEP rows | `fe57b8b` |
| W4-INT-FIX-IO | STEP repair notes and codes, Abaqus shared fan with inverted-tet refusal, refcorpus manifest rows, pycache removed | `226c754`, `b9da2f1` |
| W4-INT-FIX-KERNEL | Heal `close` parent walk bounded, `VirtualTopology` maps to `Unsupported`, guard tests for heal close and `subdivide_pin` | `7145aa5` |
| W4-INT-FIX-MESH | `refine_local` refuses `cell_set` with `threshold`, `thex`/`tquad` docs qualified, `is_ungraded_curvature` helper and 4/3 `tol` docs, composite tests tightened | `5755054` |

## 2. Deviations (Dev-1 to Dev-16)

"Dev" and not "D" because D1 to D11 are the user's rulings. Each deviation names who accepted it, or says that it needs a decision.

| # | Deviation from the plan | Why | Status |
|---|---|---|---|
| Dev-1 | D5 landed twice. The first D5 commit (86a0118) moved the through pin script from 432 to 891 hexes (the unit through pin: 48 to 99 cells) and quality got worse (scaled Jacobian min 0.707 to 0.539, edge ratio max 1.52 to 3.54). W4-OGRID-SIZE made ogrid's score follow the size field, so the through pin, its lattice and the leftout fixture returned to their pre-D5 values (final D5 rows equal the `212689e` values; the census rows were re-blessed back). D11 (2026-11-14) then moved the default pin on purpose (see section 4) | Ruling D5 asks for a hash move, but not a quality loss | Accepted by the orchestrator; D11 by the user |
| Dev-2 | W4-CSG-LIBFN dropped | `csgbrief.md` section 6: the import needs no new library entry points | Plan |
| Dev-3 | Ownership ratifications: W4-POLY-VORO also owns `crates/sm-quality/src/convexity.rs` and `crates/sm-meshio/tests/common/mod.rs`; W4-REGIONS-SPLIT also owns `crates/sm-server/src/host.rs`. OR-i acknowledges W4-VTOPO-MESH's `snap_to_face` edit in the frozen `sm-scheme-api`, W4-CSG-HEX's public methods on `HexLattice` in `sm-csgio`, and the `Cargo.lock` lines | Each was needed and had census 0 | Accepted (plan section 1 table, OR-i) |
| Dev-4 | RT-LATZ not built | OR-a: the existing 2-D lattice z rule stays, docs corrected, a test pins it | Ruling |
| Dev-5 | XDG rows exist only on x86_64. On arm64 the XDG image does not build (`vec3da.h` includes `immintrin.h`) and the rows are `Partial` | OR-d | Accepted |
| Dev-6 | W4-CSG-HEX builds each hex element as a brick cut by its six walls (`partition::hex_prism`), not with `model.create_prism`. The result is a true hexagonal prism (8 faces, analytic volume `sqrt(3)/2 p^2 h`). Lattice reuse is off for a deck with any hex lattice. Limits OR-h | Kernel limits | Accepted for W4 |
| Dev-7 | W4-STEP-REPAIR has its own planar-cap closer in `sm-cadio` instead of running HEAL-TOL's fixes (`sm-cadio` may not depend on `sm-boolean` without an ACR). Tolerance sewing of displaced faces at import is not done (`step-dirty-tol-edge` is capped, with a note naming the displaced face) | Contract boundary | Accepted; W5 |
| Dev-8 | Abaqus has a separate `pyramids` option (`error` default, `c3d5`, `degenerate_hex`) instead of "pyramids follow the polyhedron policy" | C3D5 support unconfirmed; degenerate C3D8 is the conforming choice next to hexes | **Default needs a user decision** (section 6) |
| Dev-9 | W4-VT-TIDY does not restore the saved overlay when `plan.build` fails after a transform; it documents that the engine discards the scratch state. Tolerance-refusal tests show the state hash unchanged | Equivalent outcome | Accepted |
| Dev-10 | W4-INT-POLY: "per-block volumes equal the inscribed-polygon closed form to 1e-9" is not met on the tet chain. The tet mesher adds Steiner nodes on the cylinders, so the tet surface is not the 36-gon prism: fuel mesh 1.984852, 36-gon prism 2.000415, circle 2.010619. `mesh.preserve_volume` refuses the mesh ("not a sweep of the circle"). Checked instead: model volumes equal `pi r^2 h` to 1e-9, the blocks sum to the box volume to 1e-9, tet, polyhedral, `.smsh` set and `.smrt` region volumes agree to 1e-12, and each disc is bounded above by its circle | The plan check contradicts `drafts/csgbrep.md` D7, which gives the closed form only for swept meshes | Substitute checks recommended for acceptance; needs the orchestrator's ruling (add the two-sided `C h^2` bound of the INT-POLY reviewer, or take the tets from a swept hex mesh) |
| Dev-11 | W4-INT-POLY: membership at the 600 oracle points through the `.smrt` is 594 equal, 6 further out (void, clad and water band between the inscribed mesh and the true circle), 0 further in, 0 outside. Test asserts "never further in" and pins 6 for macOS arm64. Exact 600 of 600 is covered on the B-rep by sm-csgbrep `membership_at_the_600_oracle_points_equals_openmc`. No inscribed mesh at 36 intervals can reach 600 of 600 (the reviewer counted 6 oracle points within the worst-case 36-gon band) | `gen.py` samples uniformly with no exclusion near surfaces | Reading recommended for acceptance: item 1's membership check = B-rep oracle test plus the one-sided `.smrt` check |
| Dev-12 | W4-INT-POLY: "replay without the strategy" is read as replay under an unregistered strategy name (the record holds 4 explicit assignments). The dual is barycentric: the circumcentric gate refuses the mesh (5891 of 58264 faces not Delaunay, 5602 circumcentres outside) and the test asserts that choice and that the refusal leaves the state unchanged | Plan text ambiguous; gate result is data | Accepted |
| Dev-13 | W4-INT-REGIONS: sectors start at `angle0 = 1 rad` (the fuel cylinder and its subtract tool are rotated by the same angle) instead of the default 0, because `SEAM_TURN = 1.0` in `sm-boolean` puts the seam of the cutting cylinders at 1 rad. Frame named `lat20` to match the deck's lattice id. `session.export` in a script takes no `options`, so the split is exported in-process and with `smesh export --regions`. The O-grid applies to no face of a sector-cut disk, so the mesh is mediocre: scaled Jacobian min 0.383 (162 cells), mean 0.82, edge ratio max 4.38. This is not O-grid quality | `SEAM_TURN` moves pinned hashes (C3) | Accepted; backlog (sm-boolean, sm-lang) |
| Dev-14 | The fuel shortfall of the polyhedral milestone is 1.3% below the circle (2.58e-2), 2.5x the 36-gon prism's own, probably from `mesh.size 2.0` leaving coarse triangles on the cylinder sides. It exceeds the `C h^2` cap of the sm-csgbrep `oracle_mesh` test | A showcase mesh that is coarser than it needs to be | Backlog (sm-cli) |
| Dev-15 | The x86_64 benchmark table is pending (D6). The smoke run on arm64 states no ratio | Ruling D6 | Pending the user's run |
| Dev-16 | `grade: "off"` curvature sizing with a tet or tri scheme silently breaks the tolerance (2.44% volume deficit against 0.89%). FIX-MESH added the docs and the `is_ungraded_curvature` helper; nothing yet warns or refuses in `mesh.auto` | Scope of FIX-MESH | Backlog (sm-mesher) |

## 3. Rulings as applied

### Rulings D1 to D11 (user)

| Ruling | Applied how | Evidence |
|---|---|---|
| D1 cells for the transport code | Both pin modes are built. `ogrid` is in `hex_only`. `mesh.to_poly` takes mixed meshes (hex, wedge, pyramid, tet); the hex to tet transition by pyramids exists. VORO-CLIP and CONFORM stay trigger-based (section 6) | sm-poly tests/mixed.rs `acc1_hex_pyramid_tet_with_a_volume_interior_transition`; sm-scheme-tet tests/pyramid.rs `hex_tet_by_pyramids_is_conforming_with_analytic_volumes`; sm-cli milestone_w4_pin.rs `milestone_w4_pin` |
| D2 file format | Every polyhedral `.smsh` export fills the `gc_*` geometry cache (no format bump). Fluent, CGNS and Abaqus writers exist for interchange. Importing other formats into `.smsh` is W5 | sm-poly tests/mixed.rs `acc4_smsh_export_of_a_mixed_result_fills_the_geometry_cache`; milestone_w4_poly.rs checks `gc_cell_volume` equals the recomputed volume to 1e-12 |
| D3 coarse-mesh acceleration | ACR-LEVELMAP landed in W4-0b; `mesh.agglomerate` has a per-pin mode; refinement levels are stored the same way (linked meshes with a level map) | sm-poly tests/agg_op.rs `per_pin_mode_gives_one_coarse_cell_per_pin`; sm-modify tests/op_level.rs `replay_and_fork_give_equal_hashes_and_keep_the_link` |
| D4 round-region volume | Option (b): `mesh.preserve_volume` (VOLFIT) and `mesh.volume_check` (VOLDIAG). Option (c) in `sm-tess` is W5 | sm-mesher tests/volfit.rs `rings_and_sectors_hold_pi_r2_h_over_12_to_1e_12_after_the_op`; milestone_w4_pin.rs and milestone_w4_regions.rs |
| D5 `ogrid` in `default` | Applied, then refined (Dev-1); the final D5 state keeps the through pin hash at its pre-D5 value because ogrid's score follows the size field | census notes in `census-base.txt` (lines 7, 13) |
| D6 benchmark machine | `tools/bench/x86_64.sh` and the runbook `bench-x86.md` delivered; the polyhedral row is a separately labelled table; no ratio in the smoke | `bench-x86.md`; `int/bench-smoke.md` |
| D7 formats | (a) STL is W5. (b) DAGMC `.h5m` from the B-rep is done. (c) No existing `.h5m` models were used. (d) IGES is W5+ | sm-server tests/dagmc_model.rs `model_source_needs_no_mesh_and_is_the_same_through_every_door` |
| D8 models | W4-REFCORPUS builds the public corpus (section 5). CSG-HEX covers both orientations, each confirmed against OpenMC | sm-rt tests/openmc_hex.rs `the_hex_world_equals_openmc_find_cell_for_both_orientations`; sm-csgbrep tests/hex.rs |
| D9 regions | Lattice frames, ring and sector tags, region split in `.smsh` cell sets and `.smrt` regions | milestone_w4_regions.rs (117 regions); sm-server tests/regions_split.rs |
| D10 defaults | Circumcentric stays opt-in (recommendation to keep, section 6); `hex_only` suggests cuts (`mesh.decompose_plan`); OpenMC import is one history node; many-to-one sweep is W5; no GPU, high order and boundary layers in W5; optional Delaunay output in `sm-scheme-tet` not built (section 6); OpenFOAM is the oracle for Fluent (no licence) | sm-cli milestone_w4_pin.rs `decompose_plan_gives_equal_replies_and_hashes_through_cli_and_mcp`; sm-meshio tests/fluent.rs `openfoam_reads_the_file_and_checkmesh_agrees` |
| D11 corner cells | Disks of 8 to 14 circle nodes use `ogrid`; 6-node disks keep pave. Default pin 48 to 72 cells | section 4 |

### Orchestrator rulings OR-a to OR-i

| Ruling | Applied how |
|---|---|
| OR-a (2D lattice z) | RT-LATZ not built; docs corrected; a test pins the rule |
| OR-b (`source` in test literals) | Mechanical additions accepted; MCP and Python exposure of `source` is W5 (backlog, sm-server) |
| OR-c (hash tags) | Lattice frames tag 14, region tags tag 15, emitted only when non-empty |
| OR-d (XDG x86_64 only) | XDG rows `Partial` on arm64; real numbers come from the user's x86_64 run (Dev-5) |
| OR-e (checks in a sub-wave) | Workers ran their own crates; the combines ran `tools/ci.sh` once and re-blessed catalog goldens once each time (section 4) |
| OR-f (what D5 covers) | The D5 and D11 commits moved every test, script and golden that pins the through pin, its lattice, B5 and the leftout fixture; the census diff equalled exactly the listed rows |
| OR-g (catalog descriptions) | Docstring changes of existing ops (for example `mesh.to_poly` "all-tet" to "tet", `session_export` format text) were re-blessed with the list of lines |
| OR-h (CSG-HEX limits) | Accepted for W4 (Dev-6); `csg.unsupported` code and multi-body imprint are W5 |
| OR-i (contract edits) | Acknowledged (Dev-3) |

## 4. Census, goldens and platform

### Census

`tools/census/census.sh --compare docs/design/w4/census-base.txt` (plan section 5 item 6; C3). Last run, at the W4c combine (`3345c39`, main `b51806f`): **0 differing of 487 keys** (`--no-linux`). Every refreshed row of `census-base.txt`, with its ruling:

| Combine | Rows refreshed | Ruling |
|---|---|---|
| W4-0.5 (a9d86ca) | sm-protocol `rejected.json`, `requests.json` golden insertions (ACR-DAGMC-SRC) | additive ACR |
| a1 part 2 (bdeea2a) | sm-mcp `tools.standard.txt` (+`mesh__volume_check`, +`model__subdivide_pin`); session transcripts (`source` property); new sm-diag `mesh.volume_check.*` goldens | OR-e, OR-b |
| a2 (6123200) | `tools.standard.txt` (+`mesh__decompose_plan`, +`model__import_openmc`, +`model__lattice_define`, +`model__lattice_tag`, +`model__region_clear`, +`model__region_tags`) | OR-e |
| a3 (86a0118, e80a9ea) | The D5 rows: `platform` `w3c_pin.branch.main`, `.mem.state`, `.print.h_mesh`, `.state`, `.layer.intent`, `.layer.meshes`, `.print.cells`, `.print.h_auto`, `expected.txt.sha256`; `w3c_pin` `leftout.print.h_mesh`, `leftout.state`, `pin.hexes`. After W4-OGRID-SIZE the rows returned to the pre-D5 values (Dev-1). `w3c_pin.h_model`, `layer.model`, W2, W3a, W3b and the enclosed pin did not move | D5, OR-f |
| a3 (86a0118) | 20 new rows for the 10 new OpenMC fixtures (`csgio` `oracle.<name>.sha256`, `content_hash.<name>`): `disconnected`, `placed_z`, `quadric_cyl`, `ref_assembly17`, `ref_assembly17_h2`, `ref_beavrs_assembly`, `ref_mini3x3`, `ref_mini3x3_h2`, `ref_pin`, `ref_pin_h2` | new pins |
| a3 (7acd147) | `tools.standard.txt` (+`mesh__preserve_volume`) | OR-e |
| b (4b51af3 and combines) | `tools.standard.txt` (+9 virtual-topology ops; +`mesh__agglomerate`, +`mesh__interval_relation`, +`mesh__refine`, +`model__heal_check`, +`model__regularize`) | OR-e |
| D11 (266e4c9) | `platform` `expected.txt.sha256` e6bebc85... to be988e42...; `w3c_pin.branch.main`, `.mem.state`, `.print.h_mesh`, `.state` 6953b909... to b493073c...; `layer.intent` 931ece31... to caa6b4cd...; `layer.meshes` 46943517... to 6b3e12f5...; `print.cells` 432 to 648; `print.h_auto` 1f4b0912... to 619e5665...; `leftout.print.h_mesh` and `leftout.state` 87055d62... to 7c352811...; `pin.hexes` 432 to 648. Unchanged: `w3c_pin.h_model`, `layer.model`, W2, W3a, W3b, the enclosed pin, the 6- and 7-node meshes | D11, OR-f |
| W4-0c | sm-mcp session transcripts `session.out.jsonl`, `session.out.respell.jsonl` (`session_export` format enum and text: fluent, cgns, abaqus) | ACR-EXPFMT, OR-e |
| W4c (6481ccf, 3345c39) | `tools.standard.txt` (+`mesh__htet`, +`mesh__qtri`, +`mesh__thex`, +`mesh__tquad`, +`mesh__refine_local`, +`model__heal`); the two session transcripts (format description); sm-py `ops.pyi` (same ops, `import_step` gains `repair`); sm-protocol catalog goldens unchanged | OR-e, OR-g |

W4-INT census expectation, for the combine to confirm by running `census.sh --compare`: **no difference**. The INT merges add documents, tools scripts and test files, and three fix tasks change library code (sm-cadio repair notes and codes, sm-meshio shared decomposition fan, sm-boolean heal and error mapping, sm-modify and sm-sizing docs and one refusal) that no pinned hash passes through. FIX-IO checked that the Gmsh and Abaqus byte pins and the sm-server export identity test still pass. Any difference there stops the integration (C3).

### Catalog goldens the INT combine should expect

- **Expected to change:** sm-py `ops.pyi` docstrings of `MeshOps.thex` and `MeshOps.tquad` (FIX-MESH qualified the V/4 and A/3 claims; OR-g). The sm-mcp `tools.standard.txt` checks names only, so no entry changes. No op is added, removed or given a new parameter by W4-INT. FIX-KERNEL changes an error mapping, not a catalog entry.
- **Everything else:** no change expected. If the re-bless shows another line, check it against these three fix commits first.

### Platform table

`tools/platform/expected.txt` (63 lines: 62 data lines plus 1 comment, enforced on Linux CI by `crates/sm-cli/tests/platform_pins.rs`; method in `docs/design/w3c/platform-hashes.md`; macOS arm64 equals Linux arm64 and Linux x86_64 CI for every row).

| Group | Data lines | Moved in W4 |
|---|---|---|
| `w2_fork`, `w2_milestone` | 9, 7 | no |
| `w3a_tet`, `w3a_hybrid` | 7, 7 | no |
| `w3b_milestone`, `w3b_submap_graded` | 13, 9 | no |
| `w3c_pin` | 10 | yes, by D11 (rows listed above) |

CI: each sub-wave PR merged with the GitHub gate and the python job green (recorded success on `bc1694a` (#15), `5aae13a` (#14) and `fcf243e` (#13)). The run for `b51806f` (PR #16) had not finished when this report was written; its result is to be read from GitHub, not from here.

**Pins waiting for the combine** (macOS arm64; the Linux values come from CI at the INT PR; C12). The tests assert only the digests marked "asserted".

| Milestone | Value |
|---|---|
| `w4_poly` `h_model` | `b0cddbd5ad1b0a00e5f255b86de5eadee394f36bb813b4b84be97250249629f8` |
| `w4_poly` `h_sized` | `c52e9ec1f1ad9086a9290c96283cf272f50e94fa664e31b3ddfeb4db5294d8ac` |
| `w4_poly` `h_auto` | `7a83b512399b460476937ad63010ddb30ff0b7b0c7ad30613b3a2876d510cc17` |
| `w4_poly` `h_tet` | `108cd1a970d8ea3279859e36b535ed38700a189926738d4e31a00502801102a7` |
| `w4_poly` `h_mesh` | `9c49ff7eca7ce84f9fb5c5ece25ae3c420e64cb767fe14f3b76852fdcb484fe5` |
| `w4_poly` `pin.smsh` | `b51553e31d202ec83c1c46d27d11ec6e6c5fd59078c26461474a2c16043e9dd0` |
| `w4_poly` `pin.smrt` | `74e12fea712d397d72ee7ada4bdb2f27ffdf0193e22a750e8d536cb6ba5e0ee8` |
| `w4_poly` dual / oracle mismatches | barycentric / 6 (asserted) |
| `w4_regions` `h_mesh` (printed) | `9af0db70f61b3da57b720470863b2046df492aebf36f9652c717eea14b42661b` |
| `w4_regions` `.smsh` with split (asserted) | `9353bb26406f74b184894f10aa57c2103a8bbf6ecdbae81111d2eb51cbbc2839` |
| `w4_regions` `.smrt` with split (asserted) | `efef863b96a3e9099fcd6beca3ca19b4a61a79e8c4ee0aa8c969c361cb4192ad` |
| `w4_pin_ogrid` `h_model` / `h_auto` / `h_linear` / `h_mesh` | `fe9830c918b2a8417054afc89f49d402444f0b40b404889a493e1387a807bdaa`, `f07ceda8f5a2b0eee8aa93e604b30e8cdb86bf6856aaba032ef1a5122f252e67`, `48d329888ef0a19b0990f3265dd091d258343c32ed6698dd751fc6f150be1f28`, `f7f9e06a8f3cab9a2c4362649ecdbdc634b3ccb51f7730cc4eb32a743c9fcf6c` |
| `w4_pin_enclosed_hex` `h_model` / `h_auto` / `h_linear` / `h_mesh` | `257f44e510e7913e323ff204a757ecf511b6d555e0b8d23e9bf9e7cf29f53bab`, `0f3d59fac046ba39eaf2193826fa17c5723cd27bc9ec02fd0a0671a12fff51ef`, `cbec6fcc43b9def3af8e15dd4b3df388028ff5c8cb889aae56c3d8fbcaf15852`, `dca55a530f8a7c48c7ae09b89eb0abf9b022378c0043a4355203b1d8b2d4775a` |

## 5. Milestones

1. **Polyhedral (`tools/w4_poly.smesh`, `crates/sm-cli/tests/milestone_w4_poly.rs`).** The OpenMC `pin` deck (fixture bytes pinned by sha256) goes through `model.import_openmc` (4 volumes, blocks fuel, void, clad, water), `mesh.intervals` (36 per circle), `mesh.auto {strategy: "tet"}`, `mesh.run` (29657 tets), `mesh.poly_info`, `mesh.to_poly` (barycentric, chosen from the eligibility report), then `.smsh` and `.smrt`. Met: 0 validation violations on the tet and polyhedral meshes; `.smsh` `gc_cell_volume` equals the recomputed volume to 1e-12 and sums to the box volume to 1e-9; `.smsh` cell sets, `.smrt` regions and block names are the same four names; no export warning; CLI (twice and 1 thread), MCP, in-process, CLI project run and Python (a wheel built and run) agree on every print and on both digests; replay reaches `h_mesh` under an unregistered strategy name. Test time about 156 s (default threads) and 176 s (1 thread). Partly met: per-block volumes and the 600-point membership (Dev-10, Dev-11).
2. **Pin milestone (`milestone_w4_pin`).** Passes unchanged under the D11 72-cell default pin: fuel `pi r^2 h` after VOLFIT for the ogrid and the enclosed-hex scripts. Hashes in section 4.
3. **Regions milestone (`milestone_w4_regions`).** The 3x3 pin lattice, each fuel cut into 3 equal-area rings by 4 sectors, `hex_only` at n = 16, VOLFIT on the 108 curved faces: 1998 hexes, 0 violations, total volume 14.2884, 117 regions in `.smsh` and `.smrt`. Every fuel region holds `pi r^2 h / 12` and every water region `(1.26^2 - pi r^2) h`, to 1e-12 relative. The C driver reads all 117 names and the tags (`lat.lat20.i`, `lat.lat20.j`, `ring`, `ring_count`, `sector`, `sector_count`) through `smrt_region_n_tags` and `smrt_region_tag`, and 36 `smrt_locate` calls land in the computed region. The OpenMC 3x3 deck gives 18 regions with the same names and slot tags as the model path under `split: ["lattice"]`. Two runs, 1 thread and the default thread count, and the CLI project run give identical bytes. Quality: see Dev-13.
4. **Benchmark smoke** (below).
5. **MEASURE table:** `int/measure.md`; decisions in section 6.

### Benchmark smoke (macOS arm64, `int/bench-smoke.md`)

A smoke run, not a published table. Apple M1 Max, load average 114 at the start (four agents building); the Docker VM cannot pin cores. Command: `smrt-bench run --cases P1,H1,P2 --scale reduced --trackers smrt,dagmc-moab,dagmc-dd,xdg --threads 1,4`, 124 rows in three result files. Facts that do not depend on timing:

- `dagmc-moab` and `dagmc-dd` agree with the sm-rt facet twin on 10000 of 10000 rays in all three cases (fire and fire-locate; flight tallies agree at 4 sigma); their region sequences are equal; lost rays 0 and agreement failures 0 over the 124 rows. Image identities (MOAB 5.6.0, DAGMC 3.2.4 with double-down `437ad209...`, conda lock hashes) are recorded in the result files.
- `xdg` is `Partial` on arm64 (OR-d; Dev-5). The XDG rows (fire, tet walk) are x86_64 only.
- sm-rt H1 region `jump` against the cell `walk`: maximum fire distance difference 3.411e-12 cm over 10000 rays; face tests per fire 59.8 (cell walk) against 10.7 (region jump).
- The polyhedral rows are a separate table (D6) and are never read by the verdict.
- W4-RT-TIDY speed check (the item that W4c left open): P2 and H1, 56 cells, three interleaved runs of base `33efd2c` and main `b51806f`; the median change lies between -2.3% and +2.9% with signs scattered both ways and run-to-run spreads up to 17.5%. No slowdown or speed-up is resolvable at this load. A tighter claim needs the idle x86_64 machine.
- No ratio between trackers is stated. **The x86_64 table is pending the user's run** of `tools/bench/x86_64.sh` (three sessions on three UTC days, `bench-x86.md`). The pre-registered verdict (`crates/sm-rt-bench/VERDICT.md`) is computed from those three sessions only. arm64 and x86_64 numbers are never merged.

### Reference corpus (W4-REFCORPUS, `refcorpus.md`)

Scripts and sha256 are committed, the models are not. Sources: our generators (OpenMC 0.16.0, FreeCAD 1.1.4), BEAVRS (MIT CRPG, pinned commit), INL Virtual Test Bed (CC-BY-4.0, pinned commit, copied verbatim and not redistributed), optional MOOSE.

| Group | Entries | What `expected` says now (after the FIX-IO manifest update, `build.py expected`, 14 decks ok) |
|---|---|---|
| OpenMC (ours) | pin, 17x17 assembly, 3x3, layered pin, three hex decks (`y` 7 rings, `x` 7 rings, mixed), TRISO lattice, cone and torus | all accepted by `sm-csgio` |
| BEAVRS | pin, 17x17 assembly (12043 flat cells), 2-D core slice | pin and assembly accepted; the core flattens past 2097152 cells and is refused (lattice-aware paths only) |
| VTB | HTGR assembly (hex lattice, `x`, 12 rings, 50 axial), STARTR OpenMC (hex lattices), SEFOR MOOSE input | HTGR refused first for a distributed-material cell (`unsupported:material-list`); STARTR accepted by the tracker import; its B-rep import fails with `csg.kernel` for 0.005 cm gaps (OR-h) |
| STEP clean | pin, cladded pin, assembly can, grid spacer | all four mesh as tets in seconds (MEASURE rows) |
| STEP dirty | notch, step, gap, open shell, small hole, fillet, tolerance edge | heal check codes as listed in `refcorpus.md`; `model.heal` closes gaps, orientation and planar holes; `import_step repair: "tolerant"` caps open shells |

What the corpus told us: hex lattices are in every public source (so CSG-HEX was needed and delivered); a core-scale path must stay lattice-aware; all three OpenMC pin decks have 0.008 to 0.02 cm gaps and rings, so an all-tet mesh at coarse sizes fails the surface quality gate (about 1e6 tets would be needed).

## 6. Carried tasks and decisions

These are decisions for the orchestrator or the user. The data is from `int/measure.md` (13 real tet meshes: 7 earlier meshes and 6 refcorpus STEP meshes). **Nothing is decided here**; each has a recommendation.

| Item | Trigger | Data | Recommendation |
|---|---|---|---|
| **CONFORM** (Delaunay-friendly tet output in `sm-scheme-tet`; ruling D1, D10) | Non-Delaunay interior faces dominate the circumcentric refusals | Met on 13 of 13: non-Delaunay faces are 4.4% to 7.6% of interior faces; against outside-circumcentre tets the ratio is 4.4x to 306x (1.7x on the 733-tet enclosed pin). Pin milestone mesh: 5891 of 58264 faces, 5602 outside | Do CONFORM first, as the optional Delaunay output D10 already allows (boundary-protected Delaunay plus a restoration after smoothing), then re-run the MEASURE table on its output. If orthogonality stays a goal and not a requirement (D1), defer it to W5 with these numbers as the baseline; the barycentric dual is already valid, volume-conserving and `sm-rt`-correct |
| **VORO-CLIP** | Refusals dominated by outside circumcentres | Met only in the weak sense: every mesh also has outside circumcentres (0.03% to 8.1% of tets, growing with the boundary-to-volume ratio) but they never dominate | Wait for the CONFORM result; do not start VORO-CLIP (size L/XL) before it |
| **`CONVEX_CHECK_MAX`** | Many convex cells over 64 faces | Not met: convex over 64 faces is 0 on all 13 duals; cells over 64 faces (0.07% to 26.4%) are all non-convex hub-split cells; a larger cap recovers no cell | Do not raise it |
| **Circumcentric default** (D10) | W4-INT decides | The strict gate passes 0 of 13 real tet meshes; it passes only analytic fixtures. The barycentric dual's fast-path share is 0.3% to 10% of cells | Keep circumcentric opt-in and barycentric the default. A default of "circumcentric only when the eligibility report says the mesh qualifies, else barycentric" needs a new argument value, not a hash move. For the transport code today, `hex_only` and `ogrid` meshes are the route that is 100% convex |
| **CSG-HEX limits** (OR-h) | W5 | (1) Near-tangent concentric cylinders: typed `csg.kernel`, tracker import still works. (2) A cell abutting a hex lattice in a z plane: typed `csg.unsupported` (`sm-boolean` cannot imprint one face onto a tiling of hexagons) | Schedule a `csg.unsupported` code for (1) and a multi-body imprint task for (2) when a real core deck needs a reflector or plenum slab |
| **Many-to-one sweep** (D10) | W5 | Many-to-many multisweep is done; many-to-one needs a union-copy contract | W5, on request |
| **V2b periodic composites** | Dirty STEP or VTB models show periodic composites dominate (plan section 8) | Refused today with a typed message; the refcorpus dirty STEP files did not show it as dominant | Hold; `drafts/vtopo.md` T2b, T4, T6, T11 stay trigger-based |
| **x86_64 benchmark table** (D6) | The user's three sessions | Pending (Dev-15) | The user runs `tools/bench/x86_64.sh`; then attach the table with the verdict from `VERDICT.md`, and consider the `xl` tier |
| **Ruling needed: Dev-10 and Dev-11** | INT-POLY | See the table in section 2 | Accept the substitute checks, and add the two-sided `C h^2` bound and the band check (backlog, sm-cli) |
| **User decision: Abaqus pyramids default** (Dev-8) | HEX-PYR transition meshes | Default is `error` (typed refusal); `degenerate_hex` is the conforming alternative | Decide when a transition mesh must go to Abaqus |
| **Unmerged INT work** | Combine | `expected.txt` pins, census run, catalog re-bless, GitHub CI on the INT PR | Section 4 |

Open work not tied to a decision is in `backlog-w5.md`, grouped by crate, one line per item with its source.

## 7. Checks run for this documentation change

Only the two documents and `docs/PARITY.md` changed. I checked that every test name ticked in PARITY exists as `fn <name>` in the tree (script `w4int-docs-check.sh`, no missing name apart from a Python test that is a `def`). I did not run cargo, the census, the platform script or CI (ruling OR-e: the combine does). I did not edit any catalog golden, `expected.txt` or `census-base.txt`.
