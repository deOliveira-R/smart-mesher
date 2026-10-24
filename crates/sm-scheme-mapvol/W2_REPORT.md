# W2 report: sm-scheme-mapvol (T6, volume map)

## Summary

`sm_scheme_mapvol::register` now registers the scheme `"map"` (dimension `Volume`, caps
`HEX | STRUCTURED | BLOCK_STRUCTURED`). It requires six faces, imposes `QUAD | STRUCTURED`
(hard) on each, contributes the three parallel-edge classes to the interval system, reads the six
face grids, orients them into one right-handed `(i, j, k)` lattice by node keys alone and emits
interior nodes by 3-D transfinite interpolation plus one `Hex` per lattice cell. No face mesh is
written (no claims). Design: `DESIGN.md`.

Definition of done, all run on this branch: `cargo build --workspace`;
`cargo test -p sm-scheme-mapvol --all-features` (6 unit + 13 integration tests);
`RAYON_NUM_THREADS=1 cargo test -p sm-scheme-mapvol --all-features`;
`cargo clippy -p sm-scheme-mapvol --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-mapvol --no-deps`.

## Acceptance bullet -> test

| Acceptance | Test (file) |
|---|---|
| 10x10x10 brick, size 1: 1000 hexes, 1331 nodes, nodes on the integer lattice (1e-12), volume 1000 (1e-9), `validate` clean, min scaled Jacobian 1 (1e-12) | `brick_10_cubed_size_1_is_the_integer_lattice` (tests/volmap.rs) |
| 2x3x4 brick with hard counts: 24 hexes | `brick_2x3x4_with_hard_counts_has_24_hexes` |
| Parallelepiped built with `make_*`: TFI exact for affine maps (1e-12) | `parallelepiped_is_affine_exact` (counts 3x4x5, nodes compared at 1e-9 rounding and volume 1e-10; coordinates agree to rounding), `affine_block_with_graded_spacing_is_exact` (src/block_tests.rs, graded spacing) |
| Any face rotation or orientation of the six grids gives the same mesh | `any_face_order_loop_start_and_vertex_order_gives_the_same_mesh` (topology rebuilt with permuted vertex, edge and face creation orders and rotated loop starts), `every_dihedral_face_order_and_vertex_numbering_gives_the_same_cells` (all 8 dihedral maps of a grid, random dihedral per face, shuffled face order, permuted vertex ids) |
| A non-six-faced piece gives `NotApplicable` | `a_piece_without_six_faces_is_not_applicable` (cylinder: `Applicability::NotApplicable` and the driver's `SchemeError::NotApplicable` for a hard request) |
| Byte-identical under `Sequential` and `ScopedThreads(4)` | `sequential_and_scoped_threads_are_bit_identical` (mesh equality and node bit patterns) |
| Determinism | `determinism_over_threads_and_repeats` (`sm_testkit::determinism::check`), `big_block_uses_the_parallel_paths_deterministically` (30^3, above the rayon threshold, against `Sequential`, `ScopedThreads(3)` and a one-thread rayon pool), and the whole suite under `RAYON_NUM_THREADS=1` |
| Scope bullets | requirements/impositions and no claims: `brick_10_cubed_size_1_is_the_integer_lattice` (faces `Selection::Imposed`, caps); interval classes: `the_volume_contributes_one_equality_per_parallel_edge_of_each_direction`, `conflicting_hard_counts_on_parallel_edges_are_infeasible`; auto selection: `auto_selection_prefers_the_volume_map_on_a_brick`; registration: `the_registry_name_is_map_at_volume_only`; single hex: `single_hex` |
| Extra robustness | `a_reflected_block_is_made_right_handed`, `faces_that_do_not_close_a_hex_block_are_refused`, `an_inverted_interior_is_a_quality_failure`, `interior_keys_and_counts` |

## Deviations

* **Dependencies beyond the T6 list:** `sm-mesh-api` (contract crate, already a dependency of
  `sm-scheme-api`; needed for `CellShape::Hex` in `ElemShape::Cell`, not re-exported by
  `sm-scheme-api`) and `rayon` (allowed list). `sm-math` and `sm-geom-api` stay listed from the
  skeleton but are not used. Dev-only: `sm-model-api`, `serde_json` (unused so far, harmless),
  `sm-mesh-api` with `serde`.
* **Interpolation parameters:** arc-length fractions of the four parallel boundary lines
  averaged per direction (tensor parameters), not index fractions `i/n`. For uniform edges the
  two coincide; for graded edges this keeps affine blocks exact (tested). Documented in DESIGN.md.
* **No criterion bench:** T6 does not name one. No timing number is claimed.
* **Cold replay from root:** not applicable, this crate registers no ops or layers.
* **Branch:** the harness told me to develop on `claude/w2-sm-scheme-mapvol-7mzets`, the task
  text names `claude/w2-sm-scheme-mapvol`. I pushed to the harness branch; the orchestrator
  should merge by commit SHA as CLAUDE.md says.
* **Extra check (beyond the plan):** after interpolation every hex must have a positive Jacobian
  at all eight corners, else `FailureKind::Quality` (allows fallback).

## Known gaps

* Curved faces: the stub surface `map` (the only face scheme available here) builds faces by 3-D
  TFI, so a volume with curved quad faces is only covered at the unit level (synthetic grids).
  The real `sm-scheme-map` will provide on-surface nodes. The block assembly only reads node
  keys and positions, so no change is expected, but this is untested end to end.
* A `NotApplicable` raised from the interval phase (a block whose logical sides are not 12 shared
  edges in 3 classes of 4) is implemented but has no end-to-end test: the surface map rule already
  forces four `End` corners per face, so it needs a face with a mid-edge vertex on one side only.
* The `Applicability` test is topological only (6 faces, one loop each, shared edges, Euler
  characteristic). A six-faced genus-0 piece that is not hex-like fails later, in the interval
  phase or `generate`, with a failure that allows fallback.
* No smoothing (map is pure TFI; a `MeshTransform` would own that), no boundary layers, no groups.

## api_change_requests

None.
