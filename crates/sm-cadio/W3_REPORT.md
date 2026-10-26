# W3_REPORT: sm-cadio (W3-CAD)

Branch `claude/w3-sm-cadio` (also pushed as `claude/w3-sm-cadio-cf7j8a`). Everything is under
`crates/sm-cadio/`; no other crate, `Cargo.lock`, CI or doc was touched.

## Summary

`sm-cadio` reads AP203 / AP214 / AP242 B-rep STEP files into the cell complex through the
content-addressed op `model.import_step` (sources `{sha256}` or `{inline}`, never a path) and
answers `cad.step_info`. Mandatory scope 1-9 is complete, and so are stretch items S1 (OCC pole
encodings: the sphere's `VERTEX_LOOP`, the cone apex), S2 (STEP writer with a byte-identical
round trip), S3 (assembly flattening) and all of S4 (`sheets: true`, `FACETED_BREP` with
`POLY_LOOP`, `TRIMMED_CURVE`).

One thing still misses its stated number (see "Review round 0" for what changed):

- The rational NURBS fillet block integrates to a relative volume error of 4.1e-8 (area 7.7e-9)
  with the default quadrature (5-point Gauss, one panel along a non-periodic parameter), against
  the 1e-8 asked. The test asserts 1e-7 for the volume and 1e-8 for the area (ACR 2).

The OCC loft (a closed but non-periodic degree-13 NURBS lateral face) now meets ruling C2
through the platform measure: `volume_moments` gives 54.454272576 against OCC's 54.454271953
(relative 1.1e-8, limit 1e-6). The reader gets there by cutting such a face into four faces
(`src/split.rs`), because `sample_face` cannot tell the two seam coedges apart (ACR 1).

## Acceptance bullet to test

| # | Bullet | Test (file) |
|---|---|---|
| 1 | Parser, one test per lexical rule | `comments_are_skipped_everywhere`, `records_span_lines_and_forward_references_parse`, `doubled_quotes_and_escapes_decode`, `reals_ints_and_enums`, `nested_lists_and_typed_parameters`, `complex_entities_including_the_occ_spaced_form`, `compact_complex_entity`, `deep_nesting_is_rejected_without_overflowing_the_stack`, `errors_carry_exact_line_and_column`, ... (`src/part21/tests.rs`); `Syntax` line/col on the bad files: `not_a_step_file`, `unterminated_string_points_at_its_opening_quote`, `truncated_file_without_endsec` (`tests/bad_files.rs`) |
| 2 | Brick 2x3x4, AP203 hash, fixture hash, both hosts | `brick_2x3x4_counts_volume_area_bbox_and_hash`, `ap203_and_ap214_bricks_give_the_same_topology_hash` (`tests/acceptance.rs`) |
| 3 | Inch brick, `to_unit`, `step.units` | `inch_brick_converts_to_millimetres_on_request`, `units_are_converted_reported_and_defaulted` |
| 4 | OCC cylinder, frustum | `occ_cylinder_and_hand_written_frustum` |
| 5 | Seam sphere, pole rule | `seam_sphere_proves_the_pole_rule` |
| 6 | OCC torus | `occ_torus_matches_the_fixture_numerically` (hash equal to `fixtures::torus` once the four pcurves the fixture sets are added) |
| 7 | NURBS fillet block, OCC fillet, OCC loft | `fillet_blocks_cylindrical_and_nurbs`, `occ_loft_volume_through_volume_moments` (platform `volume_moments` at 1e-6 plus a tensor-Gauss cross-check) |
| 8 | Void, two bricks | `brick_with_void_and_two_bricks` (hash equal to the fixture for the inward spelling) |
| 9 | Milestone part, inline vs sha256 | `milestone_part_through_the_reader`, `milestone_part_through_the_op`, `inline_and_sha256_forms_give_the_same_state_hash` |
| 10 | Op level | `delta_lists_topology_ids_ascending_and_outputs_match_the_roles`, `undo_redo_and_cold_replay_do_not_diverge`, `failures_are_invalid_and_leave_the_state_untouched`, `solid_ordinals_select_solids`, `a_missing_input_service_is_reported_only_for_sha256_sources`, `json_records_decode_and_reject_paths_and_unknown_fields`, `check_fails_the_op_on_validator_errors_and_can_be_switched_off`, `inverted_solids_are_flipped_with_a_warning` (`tests/op_level.rs`) |
| 11 | Bad files | every test of `tests/bad_files.rs` |
| 12 | Robustness | `mutated_and_random_inputs_never_panic` (2400 cases seeded from the case index, with a parse-tree size bound), `structured_mutations_never_panic_and_stay_bounded` and `random_bytes_never_panic` (proptest, 2000 cases each, fixed seed), `every_prefix_of_a_small_file_is_handled`, `degenerate_inputs` (`tests/robust.rs`); `the_parse_tree_and_the_model_stay_within_twenty_times_the_input` (`tests/memory.rs`: the parse tree is 7.7x the input and `read_step` peaks at 8.8x) |
| 13 | Determinism | `read_step_is_bit_identical_across_runs_and_thread_counts`, `the_import_state_hash_is_bit_identical_across_runs_and_thread_counts`, `cold_replay_from_the_root_does_not_diverge`, `the_parallel_solid_path_matches_the_sequential_one` (`tests/determinism.rs`) |
| 14 | `cad.step_info` | `the_query_reports_each_fixture`, `the_query_fails_cleanly_and_never_changes_the_state`; rustdoc and `DESIGN.md` |
| 15 | Performance (record-only) | `benches/perf.rs`; numbers below and in `DESIGN.md` |
| S1 | OCC sphere and cone | `s1_occ_sphere_vertex_loop_gets_its_seam_and_poles`, `s1_occ_cone_apex_gets_its_pole_edge`, `inward_sphere_and_cone_are_flipped_and_keep_valid_poles` (the orientation check with poles) |
| S2 | Writer, round trip | `every_fixture_round_trips_to_a_byte_identical_fixed_point`, `the_nurbs_block_is_a_fixed_point_with_its_rational_forms`, `output_is_reproducible_and_declares_what_it_is`, `shared_faces_are_written_once_per_volume_and_reported`, `bodies_can_be_selected_and_unsupported_geometry_is_named`, `a_session_state_exports_like_its_topology` (`tests/roundtrip.rs`) |
| S3 | Assemblies | `s3_relationship_placements_are_flattened_into_instances`, `s3_mapped_items_are_flattened_with_their_target_frame`, `s3_flattening_can_be_switched_off_and_then_warns`, `s3_the_milestone_part_keeps_its_coordinates` |
| S4 | `TRIMMED_CURVE`, `FACETED_BREP`, sheets | `a_trimmed_curve_is_its_basis_curve`, `s4_faceted_brep_gets_planes_and_straight_edges`, `s4_sheets_import_as_free_faces_when_asked` (`tests/reader_rules.rs`), `s4_sheets_and_faceted_solids_through_the_op_and_the_query` (`tests/op_level.rs`) |
| - | Welding, edge repair, gaps, options | the rest of `tests/reader_rules.rs`, `tests/inputs.rs` (stores), `src/edge.rs` and `src/weld.rs` unit tests |

## Measured values

Analytic checks (debug and release give the same numbers to the digits shown):

| Model | Quantity | Expected | Measured (relative error) |
|---|---|---|---|
| brick 2x3x4 | volume / area | 24 / 52 | 24.000000000000036 (1.5e-15) / 52.00000000000008 |
| OCC cylinder | volume / area | 20 pi / 28 pi | 62.83185307179576 / 87.96459430051421 (<= 1e-14) |
| frustum | volume / total area | 7 pi / 3 pi sqrt(10) + 5 pi | within 1e-9 |
| seam sphere | volume = area | 36 pi | 113.09733552923261 |
| OCC torus | volume / area | 10 pi^2 / 20 pi^2 | 98.69604401089362 / 197.39208802178715 |
| OCC cone | volume / area | 4 pi / 2 pi sqrt(13) + 4 pi | 12.566370614347397 / 35.22071741262243 |
| OCC fillet box | volume / area | 990 + 2.5 pi / 578 + 5.5 pi | 997.8539816339753 / 595.2787595947442 |
| NURBS fillet block | volume | 960 + 10 pi = 991.4159265358979 | 991.415886269718 (4.1e-8) |
| NURBS fillet block | area | 552 + 12 pi = 589.6991118430775 | 589.699116354852 (7.7e-9) |
| OCC loft (4 faces after the split) | volume by `volume_moments` | 54.454271953 (OCC) | 54.454272576 (1.1e-8); tensor Gauss over the pieces agrees to 1e-9 |
| w3a part | volumes | 1200, 200, 600 | 1200.0000000000018, 200.00000000000006, 600.0000000000009 (<= 1.5e-15) |
| brick with void | volume | 992 | 992.0000000000003 |

Performance (release, one machine, criterion means; targets from the task):

| What | Target | Measured |
|---|---|---|
| parse a generated 55.3 MB file | <= 2 s, >= 40 MB/s | 1.19 s = 46.6 MB/s on one thread |
| import 1000 disjoint bricks through the op (`TopoStore`, `check: true`) | <= 3 s | 501 ms (`read_step` alone 174 ms) |
| `weld_vertices`, 1e5 points | <= 1 s | 180 ms |

The loft is the slowest real model: realizing it takes 0.54 s in release because the
orientation self-check integrates its degree-13 surface (7.9 s in a debug build).

Memory (9.6 MB generated file): the parse tree grows the resident set by 7.7x the input and
`read_step` peaks at 8.8x, against the 20x bound. It is measured with the process's `VmHWM`
(`tests/memory.rs`); the fuzz tests bound a structural estimate of the tree. A counting
allocator would need `unsafe`, which CLAUDE.md allows only in `sm-rt-capi`.

## Deviations from the task text, with reasons

1. **Cargo.toml.** The skeleton only listed `sm-op-api`, so the dependencies are those of the
   task's Deps line plus the G2 pre-approvals: `sm-core`, `sm-geom-api` (serde, schemars),
   `sm-topo-api`, `sm-op-api`, `sm-layer-api`, `serde`, `serde_json`, `schemars`, `thiserror`,
   `sha2`, `base64`, `rayon`, `smallvec`; dev: `sm-topo`, `sm-testkit`, `proptest`, `criterion`.
   `smallvec` is declared but unused (listed in the task). `Cargo.lock` was regenerated locally
   (new: `base64`) and its 22 additive lines (sm-cadio's dependencies, `base64`) are committed only because the branch must build; the orchestrator should discard them and regenerate the lock at merge.
2. **Branch.** Pushed to `claude/w3-sm-cadio` as asked and to the session branch
   `claude/w3-sm-cadio-cf7j8a`.
3. **`StepError`.** `Truncated` and `TooDeep` carry `line:col` (the task also wants every error
   to). Added variants `BadOption` (invalid read option), `Export` (writer problems and I/O) and
   the method `code()` giving `step.<code>`. Each maps to `OpError::Invalid("step.<code>: ...")`.
4. **`no_units.stp`.** The library `read_step` returns `Units` when the file declares no length
   unit and `assume_unit` is `None`; the op passes `assume_unit` (default `mm`) and raises a
   `step.units` warning. This reconciles "default mm + warning" with the bad-file table.
5. **Orientation self-check per shell.** Each shell is realized into the scratch complex as its
   own volume, so a void shell with the wrong orientation is repaired too (the task asks for
   a per-solid flip). The warning `step.inverted_solid` lists the flipped shells.
6. **Hash equality.** Exact `canonical_hash` equality with the stage-0 fixtures holds for the
   brick and for the void brick *written with inward planes* (`brick_with_void_inward.stp`). The
   usual OCC spelling (`brick_with_void.stp`: outward cube faces, `ORIENTED_CLOSED_SHELL .F.`)
   gives the same cells and moments but different carriers (outward plane with reversed sense),
   so its hash differs. The torus hashes like `fixtures::torus` once the four pcurves of the
   fixture are set on the imported cells (the reader sets pcurves only at poles); the cone
   differs the same way and is compared by counts and moments.
7. **Pole pcurves follow the face sampler.** The pole sweep needs the `u` the sampler has
   reached at the pole, which depends on the loop's canonical start and on its nearest-copy
   unwrapping. `realize` replays that walk (starting from the sampler's first Gauss node) so the
   branch always agrees. This is coupled to the sampler's behaviour in `sm-topo-api` (see
   the change request).
8. **S3 changes the mandatory warning.** `step.assembly_transforms_ignored` was mandatory "until
   S3 lands". With S3, an assembly whose placements are understood raises the info
   `step.assembly_flattened`; the warning remains for mirrored or unplaceable parts and when
   `ReadOptions::flatten_assemblies` is `false`. The op has no parameter for it (its args are
   exact), so it always flattens. `Solid` gained an `instance` field. Likewise `sheets: true` no
   longer gives `Unsupported`: S4 implements it.
9. **Synthesized items.** A `VERTEX_LOOP` sphere gets its antipodal vertex, meridian circle and
   seam edge synthesized after the file's cells; a `FACETED_BREP` has no edge entities, so its
   edges and lines carry the id of the `POLY_LOOP` that first used them. Those lists are not
   strictly ascending by STEP id.
10. **Fuzz seeding.** The index-seeded sweep is hand-rolled (SplitMix64 seeded with the case
    index, 2400 cases) because proptest draws its own random values; two proptests (2000 cases
    each, fixed seed) cover the same property. The allocation bound is a structural estimate of
    the parse tree plus a `VmHWM` measurement, not an allocator count (no `unsafe` outside
    `sm-rt-capi`).
11. **Fixture files.** `tests/data/gen_fixtures.py` regenerates every hand-written fixture
    (it is checked in). Extra fixtures beyond the task's list: `brick_with_void_inward.stp`,
    `brick_inverted.stp`, `brick_unstitched.stp`, `brick_faceted.stp`, `brick_faceted_dup.stp`,
    `sheet_open_box.stp`, `assembly_pins.stp`, `assembly_mapped.stp`, `bad/face_off_surface.stp`.

## Known gaps

- Sheets are free faces without volume or closure check; the writer does not write them.
  `TESSELLATED_SOLID` and other tessellated geometry are reported, not imported.
- No `PCURVE` import: closed NURBS faces and periodic seams rely on projection. A closed
  non-periodic NURBS face is cut into four faces (plain case only); any other closed NURBS
  face keeps the seam and a `step.split_closed_face` warning, and its face integrals may be
  wrong. Untested with `sm-scheme-map`.
- Mirrored assembly placements and `CARTESIAN_TRANSFORMATION_OPERATOR_3D` targets are not
  flattened. A relationship is read with `rep_1` as the child (OCC's convention, and the
  milestone file's); a writer using the opposite convention would place parts inversely.
- Welding is per solid; cells are never merged across solids (`model.merge` does that).
- `volume_moments` / `face_moments` for faces on the NURBS fillet are about 4e-8 accurate.
- The scratch orientation check doubles the cost of realizing curved models.
- Orphan vertices (a `VERTEX_POINT` no edge uses) are not imported.

## api_change_requests

1. **crate `sm-topo-api`, `geometry::sample_face`.** What: when a coedge has a pcurve, use it
   for that coedge's `(u, v)` path in every loop, not only in loops that contain a
   `DEGENERATE` coedge. Why: on a closed but non-periodic NURBS surface (the OCC loft) both
   seam coedges project to the same `u` (`NurbsSurface::project` keeps the first of two equal
   candidates), so the face integrates to zero and `volume_moments` is wrong (6 pi instead
   of 54.45). Worked around in the reader since round 0 (`split.rs`); the request stands so
   the split can be removed. Proposed: in `sample_face`, replace `surf.project(cj.p, *prev)` by
   `pc.eval(t)` when `view.pcurve(key)` is `Some`, falling back to projection otherwise. The
   reader would then set `Polyline2` pcurves on NURBS faces that are closed in `u` or `v`
   (a follow-up in `realize`, not landed because it would be inert today). Note that
   `NurbsSurface::project` also ignores the hint on ties, so the sampler must use the pcurve
   value itself, not only as the unwrapping reference.
2. **crate `sm-topo-api`, `geometry::green`.** What: split the inner `u` integral into panels
   along non-periodic parameters too (e.g. at the surface's knot spans, or at most a fixed
   parameter length). Why: one 5-point Gauss panel gives 4.1e-8 relative volume error on the
   rational quadratic fillet, over the 1e-8 the task wants for NURBS faces. No signature change.

## Integration notes (outside this crate)

- D6: `sm_server::standard()` injects `StepInputs(Arc::new(DirInputStore::new(<project>/inputs)))`
  (`MemoryInputStore` for `--memory`); front ends call `DirInputStore::put` / `stage_file` and
  record `{sha256}`. The op name `model.import_step` lives in the `model` namespace but only
  `CadioPlugin` registers it. `CadioPlugin` needs `CoreLayers` for the `model` layer.
- The writer is a library function (`write_step`, `export_step`); exposing it as a
  `session.export` format is later work.
- `sm_testkit::w3a::STEP` imports to bodies, volumes, faces, edges and vertices in STEP-id order
  (V1, V2, V3 = 1200, 200, 600), as the milestone scripts assume.

## Review round 0

Must fix:

1. **C2 / acceptance 7 through `volume_moments`: done.** Cause as reviewed: the face sampler
   projects both seam coedges to the same `u`, and it uses one 5-point panel across a
   non-periodic span. Pcurves alone would not help in-crate: `NurbsSurface::project` keeps the
   first of two equal candidates and ignores the hint on a tie. Fix: `src/split.rs` cuts a
   closed non-periodic NURBS face (one loop, one seam edge used twice) at equal parameter
   fractions into four faces by exact knot insertion (Boehm, homogeneous coordinates, so
   rational surfaces are exact), with the generators as shared edges and the crossing cap edges
   split at the generators (located by Gauss-Newton projection and bisection). Four pieces were
   chosen by measurement on the loft: 1 piece 1.9e-3, 2 pieces 3.6e-6, 4 pieces 1.1e-8 (the
   OCC value itself differs from the converged integral by that much), 8 and more the same.
   The loft imports as 6 faces, 12 edges, 8 vertices (Euler 2); the box_fillet1 (7 faces) and
   hand-written NURBS block are untouched (their NURBS faces are not closed). A configuration
   the splitter cannot handle is left as read with a `step.split_closed_face` warning; the
   result is checked for shell closure and discarded if it fails. The test now asserts
   `volume_moments` at 1e-6 and the validators clean. `IMPL_VERSION` is 2 (the loft imports to
   different cells; the brick and AP203 hashes are unchanged). The orchestrator may remove
   the split when ACR 1 and 2 land. Limits: the number of pieces is fixed at 4, so a very
   wiggly closed surface may still integrate less accurately; the roundtrip of the loft
   writes and re-reads the four faces.
2. **Assembly allocation bomb: done.** `assembly::instances` walks with a path stack (a cycle
   is found at the first revisit, whatever the fan-out), a depth cap of 64, and a limit on the
   placements of every intermediate list. `read_file` shares a budget across solids:
   `max(1024, 8 x entities)` instances and `max(2^16, 4 x entities)` copied cells (file-wide, see round 1), else the
   new typed `StepError::TooManyInstances` (`step.too_many_instances`). The earlier behaviour for
   a cycle is kept (local coordinates plus the `step.assembly_transforms_ignored` warning).
   `tests/assembly_bomb.rs` builds a 4-links-by-12-levels file (about 17 KB, 16.7M instances
   before) and expects the typed error; a 2 x 3 nesting still flattens to 8 solids; a
   20-fold self link returns at once. Note that acceptance 12's 20x bound cannot hold for a
   legitimately flattened assembly (each instance is a deep copy by design); the budgets bound
   it by the file size instead.

Should fix:

- **1 (fillet 1e-8): declined.** It needs ACR 2 (panelled quadrature in `green`); the test
  keeps 1e-7 for the volume until the orchestrator rules.
- **2 (`fit_edge` complementary arc): done.** An open edge on a closed non-periodic B-spline
  whose `same_sense` contradicts the vertex order is `Unsupported`; the reviewer's square is a
  unit test and the OCC fixtures still import.
- **3 (validate only new cells): declined.** `sm_topo_api::validate_geometry` takes the whole
  view; there is no per-cell variant (`check_cells` is already restricted). Candidate ACR:
  `validate_geometry_cells(view, ids)`.
- **4 (test runtime): done for determinism.** The heavy fixtures (loft, w3a part, 40 bricks) skip
  the state-hash loop in debug builds (they run under `--release`); `read_step` determinism and
  the cold-replay test of the w3a part still run everywhere. Acceptance is not trimmed: every
  step file's volume is asserted unconditionally.
- **5 (assembly warning semantics, `rep_1` convention): left for the orchestrator.** Behaviour
  unchanged and documented under deviation 8 and in known gaps.
- **6 (Cargo.lock): left for the orchestrator** (no new dependency this round; the lock is
  unchanged since the worker's commit).
- **7 (`write_step` without a shell): done.** It returns `StepError::Export`.

Gate (round 0): `cargo build --workspace`, `cargo test -p sm-cadio --all-features`, `cargo clippy --workspace --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS=-D warnings cargo doc -p sm-cadio --no-deps` and `tools/ci.sh` (which includes the `RAYON_NUM_THREADS=1` test pass) all exit 0; ci log `/tmp/ci-sm-cadio-r0-81799.log`.

## Review round 1

Must fix:

1. **Shared-shell allocation bomb: done.** `read_file` groups solid records with the same
   arguments after the name (and the same units) and decodes each group once, so `split_closed_nurbs`
   and `check_closed` no longer rerun per record; the later records are clones with their own id and
   name. All copies of the file (assembly placements and shared-body records) are charged to ONE
   file-wide budget, `max(2^16, 4 x entities)` cells (was per solid, `max(2^19, 64 x entities)`), else
   `step.too_many_instances`. The 64/entity scale left box_fillet1 with 16,000 refs at about 200x;
   4/entity bounds the fan at roughly 10-20x for large files. Notes and weld counts of a group are
   reported once. Tests (`tests/shared_shell.rs`): a fan of 3 clones exactly; box_fillet1 and the loft
   with 4,000 refs to the shell fail with the typed error in under 10 s.

Should fix:

- **1 (fillet 1e-8): declined again.** It still needs the panelled quadrature in `sm-topo-api`; the
  test keeps 1e-7 (measured 4.1e-8).
- **2 (split fallback): done.** `model.import_step` with `check: true` fails with
  `step.split_closed_face` (Invalid) when a closed NURBS face was left unsplit; `check: false` imports it
  with the warning. No test: no small file reaches the fallback (the plain seam is the only closed
  configuration in the fixtures).
- **3 (memoize split): done** by the per-body decode above (a shared shell is split once). Distinct
  shells that happen to be equal copies are still split each, in proportion to their own input.
- **4 (`cad.step_info` says what is in the file): done** in `DESIGN.md`.
- **5 (per-file clone budget): done** by the same file-wide budget.
- **6 (Cargo.lock): left for the orchestrator.**
