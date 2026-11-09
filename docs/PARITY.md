# Cubit parity checklist

Source: Coreform Cubit manual v2026.8 (quick reference + section index pages). We match **capabilities**, not syntax.
Each item maps to a crate (see `ARCHITECTURE.md`). Priority: **P0** = first vertical slice, **P1** = core parity, **P2** = full parity.

Gaps still to research (per-scheme manual pages not read yet): exact behavior of each scheme, the virtual-topology command set, healing commands, sizing functions, boundary layers, Sculpt.

## A. Environment & session — `sm-engine`, `sm-lang`, `sm-server`
- [x] P0 Command language (own syntax): entity lists, ranges, `all`, `except`, `in` filters (e.g. "surfaces in volume 2") (W2: sm-lang, tests/fix0.rs, fix1.rs, run.rs)
- [x] P0 Script files (play), comments, variables (replaces APREPRO) (W2: milestone.rs)
- [x] P1 Script record: a session's op history written out as a script that replays to equal state hashes; `script.record` / `smesh record` / MCP `record_script`, `--forks head|refs`, `--check none|tips|every`, `--verify` (W3b: sm-lang tests/script_record.rs `linear_history_replays_to_the_same_state`; sm-server tests/script_record.rs `recorded_script_replays_to_equal_per_ref_tips`; sm-cli milestone_w3b.rs row M7 `milestone_w3b`, `--forks refs` of the milestone replays equal)
- [x] P0 Headless/batch mode (W2: `smesh` CLI, `serve --stdio`, milestone.rs)
- [ ] P1 Groups: add/remove/equals/xor, unite/intersect/subtract groups
- [ ] P1 Entity naming, persistent IDs, ID renumbering controls
- [ ] P1 Undo/redo; **beyond Cubit:** timeline DAG with fork/checkout
- [ ] P1 Listing/query: list entity, measure distance/angle/area/volume, bounding boxes
- [x] P1 Python API: query plus op execution (package `smartmesh`, abi3 wheel, typed ops, `cmd`, mesh tables as numpy) (W3b: sm-py pytest tests/test_slice.py, test_step.py, test_mesh.py, wheel_check.py via crates/sm-py/ci/py.sh; sm-cli milestone_w3b.rs row M8 `milestone_w3b` runs the milestone from the installed wheel and replays its main line with typed ops to equal state hashes)
- [ ] P2 Settings system (warnings, verbosity, stop-on-error, memory limits, parallel meshing toggle)
- [ ] P2 Crash save / autosave (largely free with the op log)

## B. Geometry — `sm-geom`, `sm-topo`, `sm-model`, `sm-cadio`
Primitives (P0 brick, rest P1):
- [x] P0 brick (W2: sm-model, milestone.rs; same hashes on TopoStore and CellComplex)
- [x] P1 cylinder · sphere (W2: sm-model, tests/primitives.rs)
- [x] P1 frustum/cone (elliptic, top radius) · prism (n sides) · pyramid · sphere (with inner radius, octant options) · torus (W3b: sm-model, tests/prim2.rs `frustum_counts_volume_area_and_inversion`, `prism_and_pyramid_analytic`, `torus_matches_the_fixture_and_closed_forms`; tests/sphere_part.rs `all_26_cut_sets_are_clean_with_the_listed_counts`, `every_hollow_cut_set_is_clean_with_analytic_volume`)
Bottom-up (P1):
- [ ] vertex (coords, on curve, intersection) · curve (line, arc, circle, ellipse, spline through/control points, offset, projection) · surface (from curves, planar, net, offset, sweep, skin/loft) · volume (from surfaces, sweep/revolve/extrude surface, thicken sheet)
Booleans (P1):
- [x] unite · subtract · intersect (with `keep`), regularization (W3a: sm-boolean, tests/boolean_planar.rs, quadric_ops.rs, marched.rs)
- [ ] mixed-dimension unite
Transforms (P1):
- [x] move · rotate · scale · reflect (W2: sm-model, tests/transforms.rs; which axis/vertex/normal forms exist not audited)
- [x] move (delta, to location) · rotate (axis, two vertices, surface normal) · copy variants (W3b: sm-model, tests/xform2.rs `move_to_a_point_moves_the_centroid_exactly_and_an_omitted_coordinate_stays`, tests/copy.rs `brick_copy_count_3_move`; sm-cli milestone_w3b.rs row M1 `milestone_w3b`: a pin cell and a 3x3 lattice by two `model.copy` steps, volume counts and total volume checked; conformal interfaces are checked for the round pin by `milestone_w3c_pin` P3, `mesh.validate` with 0 violations; the M6 chord sums equal the box chord, which is indirect evidence only)
Decomposition, i.e. webcut (P1):
- [x] plane by vertices / surface / axis plane + offset · cylinder (and sphere) · imprint/merge options (`nomerge`) · section (keep half) (W3a: sm-boolean, tests/webcut.rs, ops.rs, quadric_ops.rs)
- [ ] tool body · sheet body/surface · extended surface
Imprint & merge (P1). This is how conformal multi-volume meshes work:
- [x] imprint bodies · merge all/tolerant · merge checks (W3a: sm-boolean, tests/imprint.rs, merge.rs, ops_merge.rs; conformal_mesh.rs shows the shared interface meshes conformally)
- [ ] imprint curves on surfaces · overlap detection (beyond the partial-overlap imprint)
Virtual topology (P1):
- [ ] composite surfaces/curves · partition surface/curve · collapse curve/angle · remove virtual
Cleanup / healing / defeaturing (P2):
- [ ] small curve/surface detection, sliver removal, blend removal, hole removal, heal (gaps, tolerances), cleanup angle
Import (P1 STEP, STL; P2 rest):
- [x] STEP AP203/214/242 B-rep, content-addressed `model.import_step`, assemblies (W3a: sm-cadio, tests/acceptance.rs, roundtrip.rs, assembly_bomb.rs; milestone_w3a.rs path A)
- [x] OpenMC CSG (`geometry.xml` and `materials.xml`) into an analytic `sm-rt` tracker, in-process and by `smcsg`: surfaces, regions, universes, rectangular lattices, fill translation and rotation, boundary types, overlap and gap checks; hex lattices, periodic boundaries, DAGMC universes and structured lattice outers are refused with a typed error. OpenMC is the oracle; MCNP is not a target (W3c: sm-csgio, tests/openmc_oracle.rs `rect_lattice_row_order_pitch_and_outer`, `rotation_and_translation_of_a_filled_cell`, `the_fixtures_are_not_a_vacuous_oracle`; tests/openmc_reader.rs `deck_o_chords_are_analytic_and_regions_follow_c9`, `refused_features_name_the_element_and_the_line`; tests/openmc_cross.rs `the_model_bridge_and_the_openmc_deck_agree_on_the_pin_assembly`; tests/cli.rs `exit_1_for_an_overlap_and_for_gaps_only_when_strict`). The result is a tracker, not a B-rep: B-rep import of CSG is open
- [ ] IGES · SAT · STL/facets (as faceted geometry) · MCNP CSG (not a target) · CSG to B-rep · native project (our op log). Mesh-based geometry: deferred with mesh import.
Export (P1):
- [x] STEP: `model.export`, a read-only query (the `path` write happens outside history) (W3b: sm-cadio tests/export.rs, tests/roundtrip.rs; sm-server tests/export_wire.rs `model_export_with_a_path_writes_the_bytes_outside_history`; sm-cli milestone_w3b.rs row M5 `milestone_w3b`: the lattice re-imports with equal volumes, state hash and node unchanged)
- [ ] IGES · facets/STL
Attributes & metadata (P2): names, colors, assemblies/parts, attribute propagation through ops

## C. Meshing — `sm-mesh`, `sm-sizing`, `sm-scheme-*`, `sm-quality`, `sm-smooth`, `sm-modify`
Process: size → scheme → mesh (bottom-up hierarchy) → quality. Mesh attributes live on geometry entities.
Sizing / intervals:
- [x] P0 interval count, size, auto size (W2: sm-sizing + sm-mesher; end to end in sm-cli tests/p0_rows.rs)
- [ ] P1 hard/soft intervals, interval matching (integer program over all curves) · relative intervals · periodic intervals · vertex sizing · bias/dualbias · sizing functions · auto size function · mesh preview
Curve schemes:
- [x] P0 equal (W2: sm-scheme-curve; p0_rows.rs row 4)
- [ ] P1 bias · dualbias · curvature · stretch · pinpoint · copy
Surface schemes:
- [x] P0 map (W2: sm-scheme-map, inside volume runs; `mesh.run` on faces alone is W3)
- [x] P1 submap (W3a: sm-scheme-submap, tests/submap.rs; corner picking not checked, corner types derived from the loop)
- [x] P1 pave (unstructured quad, CDT + matching, any number of loops) (W3a: sm-scheme-pave, tests/driver.rs)
- [x] P1 trimesh/tridelaunay (constrained Delaunay + Ruppert/Chew) · triadvance (advancing front) (W3a: sm-scheme-tri, tests/accept_brick.rs, accept_planar.rs, front.rs)
- [ ] P1 tripave · trimap · triprimitive · circle · hole · pentagon · polyhedron · copy · mirror
Volume schemes:
- [x] P0 map (W2: sm-scheme-mapvol; milestone.rs row 6)
- [x] P1 sweep, 1-to-1 only: translate, twist, linking surfaces, rotation about an outside axis; quad->hex, tri->wedge, polygon->prism (W3a: sm-scheme-sweep, tests/bricks.rs, curved.rs; many-to-1, multisweep, projected target, 360 degree revolve open)
- [x] P1 tetmesh (Delaunay + boundary recovery, interior Steiner points only, quality floor) (W3a: sm-scheme-tet, tests/a02_cube.rs, a04_cavity.rs, a08_quality.rs, adapter.rs; hybrid hex/wedge/tet in sm-cli tests/milestone_w3a.rs)
- [ ] P1 volume submap · sweep many-to-1 / many-to-many multisweep · copy
- [x] P1 auto selection: `mesh.auto {strategy}` with registered strategies; `default` picks map / submap / pave / trimesh for faces and map / sweep / tetmesh for volumes (warning `mesh.auto_tet` on the tet fallback); the choices are stored as explicit scheme assignments, so replay calls no strategy (W3b: sm-mesher tests/auto.rs; sm-server tests/auto_select.rs `b1_brick_auto_equals_the_explicit_map_script`, `b2_l_slab_sweeps_with_a_submap_source`, `b4_tetrahedron_falls_to_tetmesh_with_a_warning`; sm-cli milestone_w3b.rs row M2 `milestone_w3b`: 63 assignments on the lattice, replay under an unregistered strategy name reaches the same hash. W3c: a round pin cell in a brick is all hex when the pin goes through (sweeps of pave sources, `pin_through_is_all_hex_in_both_creation_orders`, `pin_through_plan_rows_are_sweeps_of_pave_sources_with_copy_targets`, lattice `pin_lattices_through_and_enclosed`) and all tet, with a `mesh.auto_tet` warning per volume, when it is enclosed (`pin_enclosed_is_all_tet_in_both_creation_orders`; O-grid all-hex is W4); plate with a hole is a hex sweep (`b5_plate_with_a_hole_meshes_all_hex`); auto next to a hard tet neighbour reaches a conformal tet fill with a wedge layer (`b6_auto_next_to_a_hard_tet_neighbour_falls_back_to_a_conformal_scheme`, `b6_hash_is_deterministic_and_equal_in_process_cli_and_mcp`); the round pin end to end with CLI, MCP, in-process and Python equal: sm-cli milestone_w3c_pin.rs `milestone_w3c_pin`)
- [ ] P2 sphere · tetprimitive · polyhedron · radialmesh (free mesh) · Sculpt (overlay-grid hex, parallel)
Conversion:
- [ ] P1 THex (tet→hex) · QTri (quad→tri, 2/4 split) · HTet (hex→tet) · TQuad (tri→quad)
Element types (via `sm-fem`):
- [x] P0 HEX8 (W2: sm-fem; milestone.rs)
- [x] P0 QUAD4 (W3a: sm-scheme-pave, tests/driver.rs) · TET4 (W3a: sm-scheme-tet, tests/a02_cube.rs) · P1 WEDGE6 (W3a: sm-scheme-sweep, tests/curved.rs)
- [ ] P1 HEX20/27, TET10, TRI6, QUAD8/9, WEDGE15, PYRAMID5/13, BAR2/3, SHELL · P2 TETRA14/15, WEDGE16+, cohesive, superelements
- [x] **beyond Cubit:** native polyhedral volume cells: median dual of a tet mesh (`mesh.to_poly`), agglomeration (W3a: sm-poly, tests/acceptance.rs, agglomerate.rs; milestone_w3a.rs row 12; non-star dual cells are split into wedges, INT-1)
Quality (P0 basic, P1 full):
- [x] Verdict-style metrics per element type (scaled Jacobian, aspect ratio, skew, taper, condition number, shape, etc.), thresholds, quality reports, validity (inverted elements) (W3a: sm-quality, tests/verdict_ref.rs, elements.rs, mesh.rs)
- [x] Finite-volume metrics (non-orthogonality, skewness, cell determinant, face shape) against analytic values (W3a: sm-quality, tests/fv.rs)
- [x] **beyond Cubit:** diagnostics ops `mesh.quality` (histograms, worst cells, smooth suggestion), `mesh.validate`, `mesh.flag_cells` (W3a: sm-diag, tests/quality.rs, validate.rs, flag.rs)
Smoothing (P1):
- [x] laplacian · smart laplacian · centroid area pull · winslow (2-D all-quad only) · condition number / mean-ratio (optimization); classification-constrained and never inverts a cell; `mesh.smooth` op (W3a: sm-smooth, tests/decks.rs, winslow.rs, optimize.rs, never_inverts.rs, op.rs)
- [ ] equipotential · untangle (inverted input is reported, not fixed) · randomize · feature-preserving
Modification (P2):
- [ ] refine (local/uniform, templates) · coarsen · pillow · cleanup · intersection removal · node repositioning · collapse edges · align · create/merge elements · tet mesh matching · remesh · scale · skin · cohesive element insertion
Boundary layers (P1): layered hex/wedge inflation on surfaces/curves
Mesh deletion (P1): delete mesh with propagation · free meshes. **Mesh import: deferred by decision** (we import geometry, export meshes).

## D. FE model — `sm-fem`
- [x] P0 blocks (element blocks) with element type (W2: sm-fem; type checked at export, written to `.smsh` in W3)
- [x] P1 sidesets (surface, wrt volume), several sidesets per face (W3a: sm-fem, tests/ops.rs; milestone_w3a.rs row 9, areas checked)
- [ ] P1 sidesets forward/reverse/both, sheet · nodesets · materials · block attributes · global element IDs
- [ ] P2 non-Exodus BCs (displacement, force, pressure, temperature, heat flux, convection, contact) · BC sets · CFD BC codes

## E. Export — `sm-meshio`
- [x] P0 native `.smsh` · VTU (inspect in ParaView) (W1: sm-meshio, round-trip tested)
- [x] P1 Exodus II: blocks, side sets, node sets, NFACED/NSIDED polyhedra, CDF-1/2/5 (large model), pure-Rust writer, `mesh.exo` (W3a: sm-exodus, tests/exodus_write.rs, sides.rs, netcdf.rs; milestone_w3a.rs row 10; no netCDF-4/HDF5)
- [x] OpenFOAM polyMesh (incl. polyhedral cells, cell zones, patches, split of near-degenerate cells with a cell map) (W3a: sm-meshio, tests/foam.rs, foam_split.rs, foam_parse.rs; milestone_w3a.rs row 11; checkMesh run via tools/conformance, INT-8/INT-9; W3c: option `split_warped_faces` splits warped faces into planar pieces, OpenFOAM only, through `session.export`, `smesh export --option` and Python: sm-meshio tests/foam_warp.rs `warped_faces_are_split_into_planar_pieces_and_volumes_are_conserved`, sm-server tests/export_wire.rs `split_warped_faces_reaches_the_openfoam_exporter`, `split_warped_faces_is_openfoam_only_and_planar_meshes_are_unchanged`, sm-cli tests/commands.rs `export_split_warped_faces_flag_sets_the_request_field`, milestone_w3c_pin.rs row P6)
- [x] **beyond Cubit:** export warnings: a result carries notes with a severity; the CLI prints them on stderr as `note:` / `warning:` lines (stdout unchanged), MCP puts warnings first in the text, Python raises `ExportWarning` and fills `ExportResult.warnings`; the first warning is `smrt-region-names` (a `.smrt` that falls back to volume names) (W3c: sm-cli tests/commands.rs `export_json_stdout_is_the_raw_result_and_warnings_still_reach_stderr`; sm-mcp tests/mcp.rs `export_warning_notes_lead_the_text_and_info_only_results_are_unchanged`; sm-py tests/test_export.py `test_smrt_no_blocks_warns`; milestone_w3c_pin.rs rows P4 and P5 across CLI, MCP, in-process and Python)
- [x] P1 Gmsh msh4 (ASCII and binary) and DAGMC h5m (`mat:<set>` groups, opt-in graveyard) (W3b: sm-meshio tests/gmsh.rs, tests/dagmc.rs; sm-server tests/export_wire.rs `each_format_is_byte_identical_in_process_cli_and_mcp`; `SM_CONFORMANCE=1 tools/conformance.sh --only dagmc|gmsh`: h5py, MOAB and DAGMC read the h5m, Gmsh and meshio read the msh; sm-cli milestone_w3b.rs row M4 `milestone_w3b`)
- [ ] P1 Exodus netCDF4, Abaqus (.inp, partial/explicit)
- [ ] P2 Nastran, CGNS, I-DEAS UNV, Patran, LS-DYNA, Fluent, Sierra, VRML
- [ ] Export options: by block/sideset/nodeset, 2D/3D dimension, renumbering, everything/overwrite

## F. Beyond-Cubit requirements (ours)
- [x] Timeline: undo/redo, checkout any node, fork (W2: milestone.rs row 2; compare branches is P1)
- [x] Git-native project format (W1: sm-engine)
- [x] Web frontend, read-only plus a script console: WebGL2 viewer served by `smesh serve --ws`, live by polling `session.hash`; a browser `Origin` must match the served address, 64 live connections at most (W3b: crates/sm-view/web (`npm test`, `SM_VIEWER_E2E=1 npm run e2e`), sm-server tests/live_viewer.rs, sm-cli tests/serve_ws.rs; sm-cli milestone_w3b.rs `viewer::viewer_live_smoke`: foreign Origin refused with 403, the page in Chrome draws the server's skin and follows undo and redo)
- [ ] Remote backend + headless (W2: protocol server over stdio and headless CLI done; remote staging, auth and server push open)
- [x] MCP server for agent control (W2: sm-mcp, MCP 2025-06-18 stdio; milestone.rs)
- [x] Deterministic replay (bit-identical state hashes) (W1: sm-engine strict replay + 1/N-thread CI)
- [x] Polyhedral-first native format: `.smsh` and VTU carry POLYHEDRON cells (W3a: sm-meshio, tests/smsh.rs, vtu.rs; milestone_w3a.rs row 12)
- [x] Ray tracing / particle tracking on the mesh: cell-to-cell walker on polyhedral meshes, point location, `.smrt` cache, `rt.info`/`rt.check`/`rt.locate` queries, C/Fortran ABI for transport codes (W3a: sm-rt, tests/locate.rs, cache.rs, chords.rs; sm-rt-capi, tests/c_abi.rs; milestone_w3a.rs row 13)
- [ ] GPU-accelerated smoothing/quality (later)
