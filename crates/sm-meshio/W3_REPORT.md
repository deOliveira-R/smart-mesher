# sm-meshio W3 report (W3-MESHIO)

## Summary

Done: all mandatory scope (A loss report, B OpenFOAM polyMesh ASCII, C `.smsh` 1.1, D VTU
polyhedron check) and every mandatory acceptance bullet, each with a named test. Stretch: (b) the
criterion bench `foam_polymesh` (numbers below); (a) is covered by the existing
`tests/vtu_check.py` (it runs VTK's reader when `vtk` is importable; the corpus now includes
`nonconvex_poly`), but `vtk` is not installed in this VM, so the VTK half was **not run here**;
(c) is a spelling-risk table in `FOAM_FORMAT.md` section 7, not a cross-reference to the v2506
sources (no internet).

**Not done here (orchestrator):** the Docker `checkMesh -allGeometry -allTopology` conformance run
(`tools/conformance.sh`; no Docker in this VM). The exporter, parser, checker and
`examples/foam_fixtures.rs` are in place; `cargo run -p sm-meshio --example foam_fixtures -- OUT`
writes 12 cases (every `all_mesh_fixtures()` entry, `nonconvex_poly`, and the sets-bearing
`hex_block_sets`) with `constant/polyMesh/*`, a minimal `system/controlDict` and `expected.json`.

No new dependencies. No other crate or file outside `crates/sm-meshio/` was edited. No
`api_change_requests` (`[]`).

Definition of done, all green on the final commit: `cargo build --workspace`;
`cargo test -p sm-meshio` and `--all-features`; `RAYON_NUM_THREADS=1 cargo test -p sm-meshio
--all-features`; `cargo clippy -p sm-meshio --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-meshio --no-deps`.

## Acceptance -> tests

| # | bullet | test(s) (file) |
|---|---|---|
| 1 | counts, shape census | `counts_of_the_named_fixtures`, `every_fixture_counts_its_entities`, `shape_counts_follow_cell_shape` (`tests/foam.rs`) |
| 2 | round trip, bit-equal points | `files_parse_back_to_the_same_mesh_bit_for_bit`, `awkward_scalars_survive_the_round_trip`, `long_zones_round_trip_through_the_block_layout` |
| 3 | orientation, volumes, checker clean, `validate` clean | `owner_outward_closure_and_volumes_match_the_fixtures` (independent Newell / divergence-theorem oracle in `tests/common/foam.rs`), `checker_is_clean_on_every_fixture_and_the_input_validates` |
| 4 | ordering, reordering proof | `ordering_rules_on_every_fixture`, `interleaved_boundary_faces_are_moved_behind_the_interior_ones` (natural interleaving plus a hand-built mesh with a boundary face between two interior faces of cell 0), `unsorted_interior_faces_are_sorted_by_owner_then_neighbour` (sort path), `an_interior_face_with_left_not_below_right_is_an_orientation_error` |
| 5 | patches and zones | `face_sets_become_patches_in_set_order`, `cell_sets_become_cell_zones`, `an_unlisted_boundary_face_lands_in_default_faces`, `overlapping_sets_report_in_sets_mode_and_fail_when_explicit`, `classification_gives_one_patch_per_model_face`, `non_conformal_and_partition_cut_faces_go_to_their_own_patches`, `a_planar_mirror_set_becomes_a_symmetry_plane_and_a_bent_one_a_patch`, `a_side_set_becomes_a_face_zone_with_flips`, plus `translation_decks_stay_plain_patches_with_a_loss_item`, `names_are_sanitized_and_collisions_are_errors`, `unsupported_inputs_are_typed_errors`, `options_json_shape`, `high_order_cells_are_reported_not_written` |
| 6 | golden bytes | `single_hex_files_equal_the_hand_written_golden_bytes` (`tests/data/foam/single_hex/*`) |
| 7 | hostile parse | `truncated_files_are_errors` (every prefix up to the last closing bracket of every file), `binary_format_is_unsupported`, `negative_and_huge_counts_are_errors`, `indices_out_of_range_are_errors`, `deep_nesting_and_junk_do_not_recurse_or_panic`, proptests `random_bytes_never_panic`, `mutated_valid_files_never_panic`, `printable_garbage_in_every_slot`, `huge_counts_do_not_allocate_under_an_address_space_limit` (`tests/foam_parse.rs`); spellings: `accepts_comments_and_the_list_spellings`, `read_dir_round_trips_and_write_dir_removes_stale_zone_files` |
| 8 | tar | `tar_holds_the_files_with_valid_headers_and_is_deterministic` (own 30-line untar: checksums, mode 0644, uid/gid/mtime 0, sorted names, ustar prefix split, byte-identical runs; system `tar -tvf` when present) |
| 9 | `.smsh` 1.1 | `v10_golden_still_decodes_and_matches_its_mesh`, `hex8_on_a_1000_cell_block_round_trips`, `polyhedron_on_dual_patch_round_trips`, `element_type_violations_are_typed_errors` (TET4 on hexes, unknown set, duplicate record, missing HO record, `NFACED`), `element_type_set_kind_and_overlap_checks`, `high_order_names_need_the_right_node_count`, `stats_match_the_mesh` (`mixed_transition`: 1/1/1/1 cells, 3 interior, 14 boundary, `max_cell_faces` 6; whole corpus), `python_reader_returns_the_same_element_types_and_stats`, `tampered_stats_are_rejected` and `element_type_records_are_checked_by_the_decoder` (`tests/corrupt.rs`) |
| 10 | loss reports | `vtu_report_for_a_high_order_mesh` (`ho_hex`: `vtu.high_order_dropped` 1, `vtu.extra_points_unused` 19), `vtu_report_for_face_sets_and_edge_tables`, `vtu_report_is_empty_for_plain_meshes`, `report_stays_sorted_merges_and_serializes` (`tests/loss.rs`), `smsh_loss_report_is_empty` (`tests/smsh11.rs`) |
| 11 | VTU polyhedron check | `polyhedra_decode_to_the_fixture_volumes` (`tests/vtu.rs`): `dual_patch`, `hexagonal_prism`, `nonconvex_poly`, `split_face_cube` and a hex stacked on `split_face_cube`, volumes decoded from the raw VTU arrays to relative 1e-12; `faceoffsets` is -1 for the hex; outward loop of the shared face checked. No writer change was needed. |
| 12 | determinism | `foam_output_is_bit_identical_across_runs_and_thread_counts` (every fixture + `nonconvex_poly` + sets block + reversed-face block through `determinism::check(2, 4, ..)`; the 40x40x40 block, 64,000 cells / 68,921 points, by direct hash comparison for 1, 1, 7, 3 threads, **not** through `determinism::check`), `smsh_with_element_types_is_bit_identical` (same split); the hash covers files, tar, index maps and report |
| 13 | build, test, clippy, rustdoc | see "Definition of done" above |

Registered ops: none, so there is no cold-replay `Diverged` check to run.

## Measured values (4-core VM, criterion, `benches/export.rs`, record only, no gate)

64,000 hexahedra (40x40x40), about 8.2 MiB of polyMesh files:

| bench | time |
|---|---|
| `foam_polymesh/build` (reorder, patches, CSR fill) | 6.3 ms |
| `foam_polymesh/files` (format 5 files) | 17.0 ms |
| `foam_polymesh/build_and_files`, 4 threads | 23.6 ms |
| `foam_polymesh/build_and_files`, `RAYON_NUM_THREADS=1` | 55.8 ms |
| `foam_polymesh/parse` (single thread, 84 MiB/s) | 97 ms |

The parser's peak memory is bounded by construction (deviation 7); it was not measured.

## E6: other crates that read `.smsh` bytes

`VERSION_MINOR` is now 1, so every `.smsh` file's header bytes change. I ran `cargo test -p sm-fem
-p sm-server -p sm-cli -p sm-mcp` on the branch: **all pass, no golden or pinned hash moved**.
These tests read or compare `.smsh` bytes and are the ones to watch at merge:
`sm-cli/tests/milestone.rs` (`smsh_rebuild`, header `mesh.hashes` compare),
`sm-cli/tests/p0_rows.rs`, `sm-cli/tests/cli_full.rs`, `sm-cli/tests/commands.rs` (fake server, byte
count of a stub), `sm-fem/tests/acceptance.rs`, `sm-fem/tests/determinism.rs` (compares runs, no
pinned hash), `sm-server/tests/session.rs`, `sm-server/tests/determinism.rs`. State hashes are
unaffected (no op code changed).

## Deviations from the task text (reasons)

1. **Branch.** The session's designated branch is `claude/w3-sm-meshio-6us5uz`; all commits are on
   it. The task text names `claude/w3-sm-meshio`; the final commit is also pushed there.
2. **Errors.** Every `foam::*` function returns `sm_meshio::Result` (`MeshioError`), with
   `MeshioError::Foam(#[from] FoamError)`, because `write_dir` and `read_polymesh_dir` need `Io`
   and `FoamError` (as specified) has none. `FoamError` variants carry payloads (`UnknownSet(String)`,
   `NonFinite{point}`, ...). One variant is added: `IneligibleSet{set, detail}` (an explicit patch or
   face zone naming a set that is not a boundary face / left-side set; no listed variant fits).
3. **`PatchOrigin::Parsed`** is added (what the parser sets: a `boundary` file has no origin), and
   `PolyPatch` equality ignores `origin`, so `parse_polymesh(files()) == build.mesh` holds.
4. **Extra codes.** Loss: `foam.name_sanitized` (info), `foam.high_order_dropped` (degraded).
   Checker: `foam.upper_triangular_tie` (the "warning only" for equal `(owner, neighbour)` pairs of
   the risk list; `FoamIssue` has no severity field, so it is its own code).
5. **Pinned where the spec was silent:** patch order `[named][defaultFaces][nonConformal]
   [partitionCut]`; face zones are written in set id order (not listed order); a parsed mesh's
   `n_cells` is `max(owner, neighbour) + 1`; `vtu_loss_report` takes `opts` for the signature but
   no option changes its five codes (set indicators and tags do not affect them); a `HEX8`-typed
   set whose cells carry high-order records is accepted (the spec only constrains HO names).
6. **E1 vs acceptance 8.** `PolyMesh::write_tar` is built and tested as acceptance 8 requires, but
   under ruling E1 the integration task should call `PolyMesh::files` / `write_dir`, not the tar.
7. **Allocation bound (acceptance 7).** Not metered in process: a counting `GlobalAlloc` needs
   `unsafe`, which CLAUDE.md forbids outside `sm-rt-capi` (I wrote one first; it passed; I removed
   it for that reason). Instead the bound holds by construction (lists grow as tokens are read,
   declared counts are compared at the closing bracket, uniform `N{v}` lists are capped at the
   input length) and is tested by re-running the huge-count cases in a child process under
   `ulimit -v` of 1 GiB (`huge_counts_do_not_allocate_under_an_address_space_limit`). Mutation
   check: replacing the list `Vec::new()` by `Vec::with_capacity(count)` fails that test
   (`memory allocation of 96000000000 bytes failed`). The test needs `sh` and is skipped without it.
8. **Golden bytes (acceptance 6)** were produced by the writer and then verified by hand against
   the outward rule (each of the six loops checked for its normal, owner 0 everywhere, boundary
   with one `defaultFaces` patch). They are a regression pin plus a hand-checked layout, not an
   independent oracle; independence comes from the geometry oracle in acceptance 3.
9. **`tests/corrupt.rs`.** Two assertions of an existing test poked `cell_shape` / `face_kind` to
   other *valid* codes and expected `decode` to succeed; with 1.1 `mesh.stats` it now (correctly)
   fails on the stats mismatch, so the test expects `mesh.stats` there.
10. **Golden `.smsh` v1.0.** `tests/data/v10_hex_block.smsh` was written by the unmodified writer
    in the branch's first commit (a temporary example, deleted before committing); it is the
    `hex_block([3,2,2],[0.5,0.25,1.0])` mesh with provenance `v10-golden`.

## Known gaps

* OpenFOAM `checkMesh` has not been run (Docker). Spellings that might differ are listed in
  `FOAM_FORMAT.md` section 7 (`flipMap` `1/0` vs `true/false`, `inGroups` only for `wall`,
  strictly increasing neighbours for two cells sharing two faces, `symmetryPlane` tolerance).
* Dimension-2 meshes (`empty`), inactive-cell skipping, point zones, cyclic pairs from decks, binary
  OpenFOAM, Fluent / h5m / msh4, `.smsh` tracking chunk and `SmshMesh: MeshRead` stay W3b as planned.
* Planarity of a mirror patch is judged against the plane of its first non-degenerate face.
* `PolyMesh` duplicates the face CSR; memory not measured beyond the 64,000-cell bench.
* The VTK-reader half of the VTU check did not run (no `vtk` module in this VM).
