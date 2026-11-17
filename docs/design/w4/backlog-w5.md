# W5 backlog: open items at the end of W4

One line per item, grouped by crate. Every item was found by a review (W4c review list, W4-INT worker or reviewer results) and is **not fixed** after the W4-INT fix round (combine part A, `44ccab4`). Items that the INT fix tasks closed (FIX-IO `226c754`/`b9da2f1`, FIX-KERNEL `7145aa5`, FIX-MESH `5755054`, C12 and C12-LIBM, the `w4_pin` platform pins, the Linux asserts of the `w4_poly` file digests, the `C h^2` bound, the band check and the dead code in `milestone_w4_poly.rs`) are not listed. "Source" names the review item. Nothing here blocks the W4 milestones.

Contents: crates in alphabetical order, then root and tooling, then decisions that wait for a person, then the trigger-based and scheduled W5 work that comes from `plan.md` section 8.

## sm-boolean

- `model.subdivide_pin` sectors start at angle0 = 1 rad in the regions milestone, because the cutting cylinders' seam sits at `SEAM_TURN = 1.0` (`crates/sm-boolean/src/subdivide.rs:24-32`). With the default `angle0 = 0` the seam falls inside a sector arc and n % S spacing and exact VOLFIT sector volumes are lost. Align the tool seam with a sector boundary (for example at `angle0`). Moving `SEAM_TURN` moves the pinned subdivided-pin hashes, so it needs a user-approved re-bless. Source: W4-INT-REGIONS review.
- Multi-body imprint: `imprint_bodies` cannot imprint one face onto a tiling of hexagons that share edges, so a cell abutting a hex lattice in a z plane (reflector or plenum slab on a 2-D hex lattice) is refused with `csg.unsupported` (ruling OR-h(2)). Source: W4c final.
- Run `cargo fmt -p sm-boolean`: drift in `heal.rs`, `tests/heal.rs`, `webcut.rs:721`, `webcut.rs:1573-1593` and `tests/vtopo_guard.rs` (CI has no fmt step). Source: W4-HEAL-TOL, W4-BOOL-TIDY.
- Tolerance sewing of displaced or unsewn faces (`step-dirty-tol-edge`) is not done anywhere; `model.heal` handles gaps, orientation and planar holes only. Source: W4-STEP-REPAIR, W4-HEAL-TOL.

## sm-cadio

- On `step-dirty-tol-edge` the displaced sheet #153 still gets a `step.repair_failed` note ("not repaired: open shell") next to the corrected cap note. The note could say that the sheet's face was replaced by a cap. Source: W4-INT-FIX-IO review.
- Tolerance sewing at import is W5 (see sm-boolean). Source: W4-STEP-REPAIR.

## sm-cli

- Add a CLI test that `smesh export a.CGNS --memory` (and `a.Msh`) sends `Request::Export{format: Some(Cgns)}` (resp. Gmsh), next to `export_new_formats_are_named_and_msh_stays_gmsh` in `tests/w3_commands.rs`. Source: W4-EXPORT-WIRE.
- The CLI usage text (`lib.rs:652`) lists only `.exo`, but `.e`, `.g` and `.exodus` are also inferred. Source: W4-EXPORT-WIRE.
- `milestone_w4_regions.rs`: `build()` runs six times across four tests (about 76 s at default threads, 167 s at `RAYON_NUM_THREADS=1`). Share one build in a `OnceLock` for G1, G3 and G4. Source: W4-INT-REGIONS review.
- `milestone_w4_regions.rs`: G2 sums volumes from the in-memory mesh; compute them from the decoded `.smsh` so the exported file is what is checked. Source: W4-INT-REGIONS review.
- `milestone_w4_poly.rs` Q4 (band check): the lower bound `r*cos(pi/36)` is tighter than the script's 6+30 interval polygon, whose half-angle is `(2*pi-1)/60 = 0.0881` rad. The margin today is 4.76e-4 but nothing guarantees it, and OR-j's "36-gon band" wording has the same approximation. Use the real half-angle. Source: W4-INT-TESTS review.
- `milestone_w4_poly.rs` Q2 (floor): `h = 2*pi*r/36` underestimates the longest chord, so the floor of the two-sided bound is slightly off. Source: W4-INT-TESTS review.
- `milestone_w4_pin.rs` `hashes()` has an `unwrap_or(0)` fallback that can hide a missing value. Source: W4-INT-TESTS review.
- `milestone_w4_poly.rs` takes 290 to 292 s at one thread in debug, close to the budget of about 300 s; slower CI runners have little headroom. Cut the work or raise the budget. Source: W4-INT-TESTS.
- A `.smesh` script cannot request the region split (`session.export` takes no `options`); the test exports in-process and with `smesh export --regions`. Needs an `sm-lang` change. Source: W4-INT-REGIONS.
- The regions milestone has no Python or MCP leg (the draft does not require one). Source: W4-INT-REGIONS.

## sm-csgbrep / sm-csgio

- `build_hex` groups prisms by a pairwise centre-distance scan, O(prisms^2). Bucket by `(q, r)`. Source: W4-CSG-HEX.
- In a deck with any hex lattice, lattice reuse is off for the whole deck (`!has_hex`), so large mixed decks lose prototype reuse. Source: W4-CSG-HEX.
- The B-rep import of the `openmc-hex-*` decks needs `options.bounds` (`bounds_of` cannot see oblique container planes); without it the typed error is `csg.unbounded`. Source: W4-CSG-HEX.
- A dedicated `csg.unsupported` code "concentric surfaces closer than kernel clearance" for decks like `vtb-startr-openmc` (0.005 cm gaps), which fail the B-rep import with `csg.kernel` today (ruling OR-h(1)). Source: W4c final.
- `build_hex` makes each element a brick cut by its six walls (`partition::hex_prism`), not `model.create_prism` as the plan delta worded it. The result is a true hexagonal prism; recorded as a deviation. Source: W4-CSG-HEX.
- sm-cli has no `sm-boolean` dev-dependency, so exact 600-of-600 OpenMC membership is checked on the B-rep in sm-csgbrep and only one-sided through the `.smrt` (see the report, Dev-18, OR-j). Source: W4-INT-POLY.

## sm-diag, sm-quality

- Nothing open.

## sm-exodus

- Nothing open (the NFACED read-back was verified in W4-EXO-VERIFY).

## sm-meshio

- `FluentError::TooLarge` is public but never built (ids are `u32` already). Remove it or use it. Source: W4-EXPORT-FLUENT.
- Fluent cell zones are always `fluid`. Add a `cell_types` option, keyed like `boundary_types`, so fuel and clad can be `solid` for CHT. Source: W4-EXPORT-FLUENT.
- The `fluent.name_changed` note compares against the OpenFOAM-sanitized name, so "Fuel Pin" is reported as `Fuel_Pin` -> `fuel_pin`. Report the original name. Source: W4-EXPORT-FLUENT.
- The Fluent reversed-file negative test runs only on the hex brick; run it on a polyhedral dual too (mixed and polygon face lines). Source: W4-EXPORT-FLUENT.
- CGNS: `debug_assert` that `tracked_group` never returns an existing child when building a set, BC or section node; add one reserved-name case (sets named `NGON`, `ZoneBC`, `Base`, `Zone`) to the `SM_CONFORMANCE` `cgnscheck` test. Source: W4-EXPORT-CGNS.
- Abaqus: face-set NSET members are collected before the dropped-cell check, so the NSET can disagree with the SURFACE (harmless, all vertices are written). Source: W4-EXPORT-ABAQUS.
- `DESIGN.md` (~line 154): the Gmsh writer now refuses a decomposition tet with `det <= 0` (`GmshError::InvertedDecomposition`); only `ABAQUS_FORMAT.md` says so. Source: W4-INT-FIX-IO review.
- `tests/gmsh.rs`: nothing tests `GmshError::InvertedDecomposition`. Reuse the folded split-face cube of `tests/abaqus.rs` (`a_decomposition_with_an_inverted_tet_is_refused_typed`). Source: W4-INT-FIX-IO review.
- `poly.rs` `fan_cell`: `pts.len() as u32` for the centroid and face-centre node ids is unchecked (the Abaqus `TooLarge` guard covers it in practice), and `det6` is evaluated twice per tet. Source: W4-INT-FIX-IO review.
- The Abaqus `pyramids` option (`error` default, `c3d5`, `degenerate_hex`) replaced the draft's "pyramids follow the polyhedron policy". The default for HEX-PYR transition meshes is open for the user (see the report, decisions). Source: W4-EXPORT-ABAQUS.

## sm-modify

- `mesh.refine_local` is serial and allocates a `Vec` per leaf (`crates/sm-modify/src/local.rs:360-362`): 453 312 cells in 5.6 s (about 12 us per output cell) against 3.5 us for uniform `mesh.refine`. Reuse buffers per leaf or build leaves in parallel and merge in a fixed order. Source: W4-MOD-REFINE-L.
- `lerp()` (`local.rs:167`) checks exact divisibility with `debug_assert` only; use a typed `Inconsistent` error or document the bound. Source: W4-MOD-REFINE-L.
- Add one test that runs `deck::decks()` (or the op) on a permuted-index mesh with a translation deck and shows `deck_mismatch == 0` while plain-index ranks would mismatch (acceptance 8 is split across two tests and neither covers deck root discovery under permutation). Source: W4-MOD-HTET.
- The closure result of a partial `mesh.htet` fails `mesh.to_poly` validation (`class.dim_order`, 2 per cut diagonal); `tests/op_htet.rs::known_limit_to_poly_of_a_closure_result_fails_its_own_validation` pins it. Draft acceptance 6 expected a typed refusal. This is a W4-POLY-MIXED defect (see sm-poly). Source: W4-MOD-HTET.

## sm-mesher

- `grade: "off"` with a tet or tri scheme silently breaks the tolerance (cylinder r=1, h=2, tol 0.01: 2.44% volume deficit and min SJ 0.261 against 0.89% and 0.350 at the default grade). `sm_sizing::is_ungraded_curvature` exists (FIX-MESH) but `mesh.auto` / `mesh.run` do not call it: emit a diagnostic or refuse for a tet or tri scheme with grade off. Source: W4-SIZE-CURV, W4-INT-FIX-MESH.
- Size-field speed on lattice-scale models: `Local::size` projects onto every curved feature whose padded box contains the query, and default-size validation builds the whole-model field twice (op time and `mesh.run`). Add a feature BVH or kd index, or measure on a pin lattice. The torus (R=3, r=0.6, tol 0.01) takes 7.0 s in debug for 15917 tets. Source: W4-SIZE-CURV.
- `crates/sm-mesher/tests/composite_schemes.rs` and `composite_e2e.rs` are now tightened (FIX-MESH). `sm-smooth` (`normals.rs:18`, `prep.rs:627`) still reads `Carrier::Surface` directly, so composite faces get no surface constraint when smoothed. Add the smoke test (`mesh.smooth` with `fix_boundary: false` plus an export, top nodes stay on z=0.5) and file it with the sm-smooth owner if it fails. Source: W4-VTOPO-MESH.
- Rustfmt drift in the new code (`auto.rs:301`, `curvature.rs`, `grade.rs`, `config.rs` and their tests; CI has no fmt step). Source: W4-SIZE-CURV.
- `sm-scheme-ogrid/tests/adapter.rs:421` comment still says "The pave cap at 6 and 7 nodes" after the D11 relabel (a disk's circle node count is always even). Source: W4-VTOPO-MESH review.

## sm-model, sm-topo-api, sm-scheme-api (contracts: orchestrator edits)

- `sm-scheme-api` `geom.rs:186` `face_normal`: the refusal text for any non-planar composite ends with "(composites on one periodic surface are not supported yet)". Say the periodic text only when every member shares one periodic surface handle; for a mixed non-planar composite say only "not planar, no shared surface"; test both texts. Source: W4-VTOPO-MESH.
- `sm-topo-api` `view.rs` `composite_surface` doc has one line well past the width of the rest; `composite_faces.rs` (frozen-crate test) carries reformat-only churn. Source: W4-VTOPO-MESH.
- `delete.rs` returns `OpError::Unsupported("... model.unsplit_realized first")` while `vtopo.rs:98` returns `OpError::Invalid` for the same message. Pick one class. Source: W4-VT-TIDY.
- The probe on 10, 100 and 300 cuts scales as n^1.6 to n^1.9 (about three full replays: `first_halves`, `swaps_by_stepping`, `from_data`, and each `partition_face` step costs more as the fan grows). Reword the ignored probe's doc ("must grow about linearly") or make the cost linear. Source: W4-VT-TIDY.
- Periodic composites (V2b): composites on one periodic surface are refused. Trigger: the W4-REFCORPUS dirty STEP and VTB models show periodic composites dominate (plan section 8). Source: plan section 8.

## sm-poly

- `mesh.to_poly` on the closure result of a partial `mesh.htet` fails its own validation (`class.dim_order`, 2 per cut diagonal): a W4-POLY-MIXED defect, to be fixed here and the `known_limit_` test in sm-modify turned into a positive test. Source: W4-MOD-HTET.
- The circumcentric gate refused every real tet mesh measured (13 of 13, `docs/design/w4/int/measure.md`; the polyhedral milestone mesh: 5891 of 58264 faces not Delaunay, 5602 circumcentres outside). Optional Delaunay output in `sm-scheme-tet` (CONFORM) is the trigger-based follow-up. Source: W4-INT-BENCH, W4-INT-POLY.

## sm-rt

- W4-RT-TIDY rays/s before and after: measured in W4-INT-BENCH (`docs/design/w4/int/bench-smoke.md`, 56 cells, 3 interleaved runs each): median change -2.3% to +2.9%, no slowdown resolvable at host load 11 to 28. A claim tighter than a few percent needs the idle x86_64 machine. Source: W4-RT-TIDY, W4-INT-BENCH.
- Mesh-leaf and nested-lattice hex grazing on CSG-HEX decks is not covered by tests. Source: W4c final.
- `crates/sm-rt/tests/graze_hops.rs:292`: `let _ = NodeId::new(0);` only silences an unused import; drop `NodeId` from the `use` line and delete the line. Source: W4-RT-TIDY.

## sm-rt-bench

- The `docker-check` `Failed` message for an image that does not build carries only the exit code, not the last two compiler lines that `DESIGN.md` promises. Source: W4-INT-BENCH.
- The `report` text prints verdict cells (for example `H1/w-walk/walk`, `incomplete`) under the SMOKE banner; harmless, but the report could suppress them when the banner is `SMOKE`. Source: W4-INT-BENCH.
- Shared tags `smrt-bench-dagmc-moab` and `smrt-bench-dagmc-dd` may be re-tagged by `docker-check`; record image ids at run time only (done in the result files). Source: W4-INT-BENCH.
- Full refcorpus coverage of MEASURE: `beavrs-assembly`, `openmc-pin-layered`, `openmc-assembly-17x17` still have no row (the all-tet strategy refused the OpenMC decks at the quality gate at one size; finer sizes not tried), and `step-dirty-*`, `vtb-*` were not tried. Source: W4-INT-BENCH review.

## sm-scheme-sweep

- Document in `DESIGN.md` (and ideally the error text) that an imprinted line with an interior vertex is not supported: discovery refuses it with the generic "vertex X needs exactly one linking edge, found 0". Source: W4-HEX-MSWEEP.
- Add a test with a non-planar or twisted imprinted pair so the per-pair transforms and the `shared_edge_top` reversed-edge branch (`n - k.index`) are exercised. Source: W4-HEX-MSWEEP.
- True many-to-one sweep (union-copy contract, ruling D10). Source: plan section 8.

## sm-scheme-tet

- `DESIGN.md` (W4-HEX-PYR section) is out of date: the exhausted-heights failure is now `FailureKind::Geometry` (7d02bd5); add the `apex_crowding` guard (clearance 0.5 h), the `tets_positive` guard (1e-9 s^3) and the hex|tet-slab|hex sandwich numbers. Source: W4-HEX-PYR.
- The `apex_crowding` message names "apex of quad {q}" by internal index; name the face (`quad_face[q]`). Source: W4-HEX-PYR.
- `tets_positive` takes its tolerance from the smallest pyramid base for every tet of the volume; with a size field about 1000x finer than the hex quads valid small tets could fall under it. Scale per tet or check only apex tets. Source: W4-HEX-PYR.

## sm-server, sm-protocol (contract)

- `mesh_export::write` (`mesh_export.rs:313`): list `Smsh | Vtu | Exodus | Openfoam | Fluent | Cgns | Abaqus` explicitly in the refusal arm ahead of the mandatory `_`, or reuse `host.rs` `not_implemented()`. Source: W4-EXPORT-WIRE.
- `gmsh_error` and `abaqus_error` map the new `InvertedDecomposition` variants to the generic `export-failed` kind; add an `inverted-decomposition` kind next to `nonconforming-decomposition`. Source: W4-INT-FIX-IO review.
- `host.rs` `parse_format` keeps its own `ALL` list of 10 variants; an `ExportFormat::ALL` or `from_name` in `sm-protocol` (next contract window) would make it one table. Source: W4-EXPORT-WIRE.

## sm-py

- `crates/sm-py/tests/test_export.py` Fluent test ran in the combine's Python job (`py.sh`, rc 0); no open item. The stub drift check passed unchanged at combine part A (no golden was re-blessed).

## Root, CI and tooling

- Add `__pycache__/` to the root `.gitignore`: running `tools/refcorpus/test_refcorpus.py` or `build.py` recreates `tools/refcorpus/__pycache__` (b9da2f1 removed the committed copy). Source: W4-INT-FIX-IO review.
- CI should print the recomputed `tools/platform/hashes.sh` lines in its log so the Linux census rows can be compared by value (plan section 7). Source: plan.
- Docker: `smesh-dagmc-conformance:latest` predates ACR-EXPFMT and has no `cgnscheck`; rebuild or retag `:latest` from `tools/conformance/dagmc` and delete the combine's `:w4c` tag. Source: W4c final.
- `tools/conformance/requirements.txt` pins `vtk==9.7.1`, which has no wheel for python3.15; relax the pin or note it. Source: W4c final.
- `crates/sm-meshio/tests/abaqus.rs`: the meshio oracle now hard-fails when meshio is not importable under `SM_CONFORMANCE=1` (FIX-IO confirmed the refusal, but its host has no meshio). The combine's oracle run (14 passed) predates that change; re-run `SM_CONFORMANCE=1` once where meshio exists. Source: W4-INT-FIX-IO review.
- Raw export bytes differ across platforms: the `w4_regions` `lattice.smsh`/`.smrt` digests differ between Apple libm and glibc although every identity hash agrees, so `milestone_w4_regions.rs` pins them per os-arch (macOS arm64 and Linux x86_64 only; other platforms print and do not check). Remove the per-platform digests once the C12 std-trig follow-up below makes the bytes equal. Source: fix round combine part B (CI runs 44005750263, 44016209978).
- C12 follow-up: `sm-geom`, `sm-model`, `sm-sizing`, `sm-rt`, `sm-lang` and the schemes still call std trig. Add a clippy `disallowed-methods` rule and route them through the portable path where a hash can depend on them. Source: `int/c12.md`.
- `tools/census/census.sh` has a stale comment ("62 recomputed lines" of `expected.txt`; there are 108). Cosmetic. Source: fix round combine part A.
- Rustfmt: CI has no fmt step; the new code of sm-sizing, sm-mesher and sm-boolean drifts. Decide whether to add `cargo fmt --check` to `tools/ci.sh` and format once. Source: W4c review items.

## Scheduled for W5 by rulings (not review items)

From `rulings.md` and `plan.md` section 8: volume-matched tessellation in `sm-tess` (D4 option c); importing other formats into `.smsh` (D2); STL import and export (D7a, trigger: a concrete need for STL input); IGES (D7d); true many-to-one sweep (D10); high-order elements and boundary layers (D10); OpenMC plugin for end-to-end runs (D10); region split for exporters other than `.smsh` and `.smrt` (`drafts/regions.md` R3); model instancing for lattices; DAGMC universe import and periodic boundaries in the OpenMC importer; ellipsoid and paraboloid faces as rational NURBS; GPU smoothing (trigger in rulings); POLY-RVD, POLY-POWER, POLY-BULK; sweep-order data for Sn. Trigger-based items (VORO-CLIP, CONFORM, `CONVEX_CHECK_MAX`, circumcentric default, V2b, T4/T6/T11) are in the report, section "Carried tasks and decisions".
