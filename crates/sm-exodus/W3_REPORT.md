# W3_REPORT: sm-exodus (W3-EXO)

Status: **done**, all mandatory acceptance bullets covered; stretch S1 written (skips here), S2 not
done, S3 proposed below. Branch: `claude/w3-sm-exodus-0jcdub` (the branch the session was assigned;
it replaces `claude/w3-sm-exodus` of the task text).

## Summary

`sm-exodus` writes Exodus II files with its own classic-netCDF encoder (CDF-1/2/5, no C or HDF5).
`write_exodus(mesh, opts, w)` reads `MeshRead` only and produces blocks (`TET4 HEX8 WEDGE6
PYRAMID5 TRI3 QUAD4`, `NSIDED`, `NFACED`), side sets, node sets, `elem_num_map`, QA/info records.
Sides are derived by comparing the face loop with the cell's template faces; polyhedra get one
outward face copy per (cell, face). Output is byte-identical across runs and thread counts.
Public API follows the task; extras (all additive): `exodus_nc` (the `NcFile` of an export),
`BlockPlan`/`SideSetPlan`/`NodeSetPlan`, `NcFile` accessors, `NcData::from_*`, `par_pack` helpers.
Dependencies: sm-core, sm-mesh-api, thiserror, rayon; dev: sm-testkit, proptest, criterion, rayon.
No new external crates (nothing added to `Cargo.lock`; the file was not committed).

## Acceptance -> tests

| # | Test(s) |
|---|---|
| 1 | `tests/netcdf.rs::acc1_hand_assembled_bytes_equal_the_writer` (bytes derived from the BNF, all 3 formats; CDF-1 header 100 bytes, CDF-2 104), `acc1_layout_invariants`; `tests/exodus_write.rs::acc1_header_bytes_single_hex`, `acc1_exodus_layout_is_aligned_and_ascending` |
| 2 | `exodus_write.rs::acc2_golden_cdl_single_hex` (CDL string stored in the test), `acc2_simple_shapes` (tet, wedge, pyramid) |
| 3 | `tests/sides.rs::acc3_{hex,tet,wedge,pyramid}_sides_by_centroid`, `acc3_template_corner_order_is_the_exodus_order` |
| 4 | `sides.rs::acc4_all_fixtures`, `acc4_random_hex_blocks` (proptest, 16 cases, pinned seed) |
| 5 | `exodus_write.rs::acc5_hex_block_sides_and_blocks`, `acc5_mixed_transition_blocks`; extras `mixed_set_splits_by_shape`, `block_selection_options`, `set_ids_and_deck_face_sets` |
| 6 | `acc6_round_trip_every_fixture_and_format`, `int64_ids_round_trip_in_data64`, `qa_info_limits_and_defaults` |
| 7 | `acc7_truncated_octahedron`, `acc7_prism_and_dual_patch`, `acc7_sides_on_polyhedra_are_face_list_positions`, `acc7_mixed_template_and_nfaced_blocks`; 2-D: `quad_mesh_is_two_dimensional`, `polygon_mesh_writes_nsided_with_cycle_positions` |
| 8 | `acc8_determinism_with_and_without_sets` (testkit `determinism::check`, pools of 1 and 4 threads, 3 formats, plain/sets/poly), `acc8_regression_hash_of_a_20_cubed_block` |
| 9 | `netcdf.rs::acc9_limits_without_allocation` |
| 10 | `exodus_write.rs::acc10_errors_never_panic` (EmptyMesh, OverlappingBlocks, BadLabel 0/-4, NameTooLong 300 B, high-order, HDF5 and truncated reads), `netcdf.rs::read_rejects_bad_input_without_panicking`, `model_errors_are_typed`, `inactive_cells_are_skipped_and_counted` |
| 11 | `benches/exodus.rs` (record-only), numbers below |
| 12 | gate commands below |
| S1 | `tests/external.rs` (`ncdump -h` header equality, `scipy.io.netcdf_file` coordinates/connectivity); skip with a message when the tool is missing |

Gate (this VM): `cargo build --workspace`, `cargo test -p sm-exodus --all-features` and with
`RAYON_NUM_THREADS=1` (42 tests, all pass), `cargo clippy -p sm-exodus --all-targets --all-features
-- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-exodus --no-deps`.

## Measured values

* 1e6-hex block (100^3, 1,030,301 nodes) to `io::sink`, 4 vCPU, release: **16.6 ms** (16.5-16.7),
  file 60,728,544 bytes (3.4 GiB/s). Target was under 2 s; record-only.
* 64k-hex block (40^3): 1.68 ms, 3,959,424 bytes.
* Regression constant: FNV-1a 64 of the 20^3 hex block (`hex_block([20;3], [0.05;3])`, default
  options, Offset64) = `0x56a9e4cc49286397`.
* Header sizes of the 1-dim/1-attr/1-var test file: CDF-1 100 bytes, CDF-2 104.

## Deviations (with reasons)

1. **Branch**: `claude/w3-sm-exodus-0jcdub` (session-assigned) instead of `claude/w3-sm-exodus`.
2. **`NcFile<'a>` / `NcData<'a>` carry a lifetime**, because `NcData::Lazy` must borrow the mesh to
   keep peak memory at about one variable; `Box<dyn Fn + 'static>` cannot. Call sites need no
   annotation (`NcFile::new(fmt)`); `NcFile::read` returns `NcFile<'static>`. `NcFile` has a
   hand-written `Debug`.
3. **Hash for acceptance 8** is FNV-1a 64 (`sm_testkit::determinism::Fnv`), not SHA: `sha2` is not on
   the allow-list (it is only a dev-dep of sm-testkit).
4. **NFACED/NSIDED names are from memory** (no SEACAS sources or headers in the VM, no internet).
   I use `num_face` for the total face count (the task text says `num_fa`; both are recall, I trust
   `num_face` slightly more) plus `fa_status|fa_prop1|fa_names`, `entity_type1/2` attributes on
   `ebepecnt`/`fbepecnt`, `num_nod_per_fa<j>` for the face block's node total. Marked
   **experimental** (ruling E4); a single constant change per name if a SEACAS check disagrees.
5. Known mismatches with libexodus values that I kept as the task specifies: `file_size = 1` for
   every format (libexodus writes 0 for classic), `int64_status = 1` (libexodus uses a bitmask, 7
   for all-int64).
6. **Acceptance 3, pyramid**: "s1 y=0 triangle" is loose; the side-1 triangle (0,0,0), (1,0,0),
   (.5,.5,1) only has its base edge on y=0. The test asserts the exact centroid (.5, 1/6, 1/3).
7. **Acceptance 5, mixed_transition**: the order HEX8/WEDGE6/PYRAMID5/TET4 appears when four
   labelled sets 1..4 are attached (tested). Without sets, rule 2 gives `unassigned_` blocks in
   shape-code order TET4, HEX8, WEDGE6, PYRAMID5 (also tested).
8. **Acceptance 7, dual_patch**: it contains seven hexagonal prisms (all `Polyhedron`), so it
   yields NFACED only. Mixed NFACED + template is covered by `acc7_mixed_template_and_nfaced_blocks`
   (hex next to a 7-face polyhedron, built with `sm_testkit::meshes::build_mesh`).
9. **NSIDED side numbering**: position of the edge in the polygon's `cell_corners` cycle (edge `i`
   = node `i` -> `i+1`), not the position in `cell_faces` (that list is ascending by face index,
   not boundary order). Identical to "position in the face list as written" for NSIDED.
10. `NcFormat` and `NcType` are not `#[non_exhaustive]` (closed by the netCDF spec).
11. Info lines over 80 bytes are split into 80-byte records (so a long provenance string cannot
    fail); QA strings over 32 bytes give `NameTooLong`; a title over 80 bytes gives `TooLarge`.
12. Zero-length dimensions cannot exist in classic files, so `num_node_sets`, `num_side_sets`,
    `num_qa_rec` and `num_info` and their variables are omitted when empty (as libexodus does).
13. CDF-1 is stricter than the spec: the file end must fit in `u32::MAX` too.
14. Cold replay "from root with no Diverged" does not apply: the crate registers no ops.
15. A dimension-2 mesh off the z = 0 plane is `Unsupported` (shell surfaces are deferred).

## Known gaps

* No SEACAS/libexodus, ParaView, MOOSE or meshio read-back could be run here (no network, no
  ncdump, no scipy, no Docker image with these). S1 tests are written but skip; they have **not**
  been run against the real tools. The `ncdump -h` comparison encodes my memory of ncdump's CDL
  format and may need a small tweak on the first real run.
* NFACED is unverified against a real reader (see deviation 4); the orchestrator's ParaView/meshio
  step is the external judge, also for the side tables (the in-repo tests compare against node
  lists typed from the manual, but cannot prove the manual says the same).
* S2 (coordinate frames) not implemented. Time steps/results, high order, distribution factors,
  attributes, assemblies, edge blocks, shell sides, netCDF-4, Nemesis: deferred as listed in
  `DESIGN.md`.
* The internal reader rejects record data (`numrecs > 0`) and is not a general Exodus reader.

## api_change_requests

None.

Proposal for the orchestrator (S3, belongs to sm-testkit, not edited): `sm_testkit::meshes::with_sets(
fixture, spec)` that attaches labelled `Cells`/`Vertices`/`Sides` sets to a fixture. `tests/common/mod.rs`
has the small helpers (`add_set`, `cells`, `verts`, `sides`, `face_with_vertices`) that this would
replace; sm-fem/sm-meshio tests would benefit as well.

## Integration notes

Wiring is the integration step's: `sm_exodus::write_exodus(&dyn MeshRead, &ExodusOptions, &mut dyn
Write)` for `ExportFormat::Exodus`; `ExodusOptions::provenance` takes the string passed to
`SmshWriter`. Deck face sets (`Faces`) become side sets by default; the sm-fem lowering produces no
vertex sets today, so the milestone export carries blocks and side sets only.
