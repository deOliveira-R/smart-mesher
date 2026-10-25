# W2 report: sm-scheme-mapvol (T6, volume map)

## Summary

`sm_scheme_mapvol::register` now registers the scheme `"map"` (dimension `Volume`, caps
`HEX | STRUCTURED | BLOCK_STRUCTURED`). It requires six faces, imposes `QUAD | STRUCTURED`
(hard) on each, contributes the three parallel-edge classes to the interval system, reads the six
face grids, orients them into one right-handed `(i, j, k)` lattice by node keys alone and emits
interior nodes by 3-D transfinite interpolation plus one `Hex` per lattice cell. No face mesh is
written (no claims). Design: `DESIGN.md`.

Definition of done, all run on this branch: `cargo build --workspace`;
`cargo test -p sm-scheme-mapvol --all-features` (6 unit + 14 integration tests);
`RAYON_NUM_THREADS=1 cargo test -p sm-scheme-mapvol --all-features`;
`cargo clippy -p sm-scheme-mapvol --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-mapvol --no-deps`.

## Acceptance bullet -> test

| Acceptance | Test (file) |
|---|---|
| 10x10x10 brick, size 1: 1000 hexes, 1331 nodes, nodes on the integer lattice (1e-12), volume 1000 (1e-9), `validate` clean, min scaled Jacobian 1 (1e-12) | `brick_10_cubed_size_1_is_the_integer_lattice` (tests/volmap.rs) |
| 2x3x4 brick with hard counts: 24 hexes | `brick_2x3x4_with_hard_counts_has_24_hexes` |
| Parallelepiped built with `make_*`: TFI exact for affine maps (1e-12) | `parallelepiped_is_affine_exact` (counts 3x4x5; every node and every affine lattice point matched within 1e-12 per component, volume to 1e-12 relative), `affine_block_with_graded_spacing_is_exact` (src/block_tests.rs, graded spacing, same 1e-12 matching) |
| Any face rotation or orientation of the six grids gives the same mesh | `any_face_order_loop_start_and_vertex_order_gives_the_same_mesh` (topology rebuilt with permuted vertex, edge and face creation orders and rotated loop starts), `every_dihedral_face_order_and_vertex_numbering_gives_the_same_cells` (all 8 dihedral maps of a grid, random dihedral per face, shuffled face order, permuted vertex ids) |
| A non-six-faced piece gives `NotApplicable` | `a_piece_without_six_faces_is_not_applicable` (cylinder: `Applicability::NotApplicable` and the driver's `SchemeError::NotApplicable` for a hard request) |
| Byte-identical under `Sequential` and `ScopedThreads(4)` | `sequential_and_scoped_threads_are_bit_identical` (mesh equality and node bit patterns) |
| Determinism | `determinism_over_threads_and_repeats` (`sm_testkit::determinism::check`), `big_block_uses_the_parallel_paths_deterministically` (30^3, above the rayon threshold, against `Sequential`, `ScopedThreads(3)` and a one-thread rayon pool), and the whole suite under `RAYON_NUM_THREADS=1` |
| Scope bullets | requirements/impositions and no claims: `brick_10_cubed_size_1_is_the_integer_lattice` (faces `Selection::Imposed`, caps); interval classes: `the_volume_contributes_one_equality_per_parallel_edge_of_each_direction`, `conflicting_hard_counts_on_parallel_edges_are_infeasible`; auto selection: `auto_selection_prefers_the_volume_map_on_a_brick`; registration: `the_registry_name_is_map_at_volume_only`; single hex: `single_hex` |
| Production mesh store | `brick_through_the_production_mesh_builder_matches_the_reference` (`sm_mesh::MeshBuilder`, same `mesh_hash` as the reference builder) |
| Extra robustness | `a_reflected_block_is_made_right_handed`, `faces_that_do_not_close_a_hex_block_are_refused`, `an_inverted_interior_is_a_quality_failure`, `interior_keys_and_counts` |

## Deviations

* **Dependencies beyond the T6 list:** `sm-mesh-api` (contract crate, already a dependency of
  `sm-scheme-api`; needed for `CellShape::Hex` in `ElemShape::Cell`, not re-exported by
  `sm-scheme-api`) and `rayon` (allowed list). `sm-math` and `sm-geom-api` stay listed from the
  skeleton but are not used. Dev-only: `sm-model-api`, `sm-mesh-api` with `serde`.
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
  Note: the planner propagates an `Err` from `interval_constraints` with `?`, so this path
  **aborts** the run (clear reason, no re-plan); it is a backstop. The fallback-capable paths are
  `applicability` and `generate` (the latter's `Quality`/`Constraint`/`Unsupported` failures,
  `Quality` tested at the unit level).
* `Cargo.lock` gained three lines (this crate's new deps `rayon`, `sm-mesh-api`, `sm-model-api`);
  cargo generated them and they are committed so `--locked` builds work. The orchestrator can
  regenerate the lock at merge if other crates conflict.
* The `Applicability` test is topological only (6 faces, one loop each, shared edges, Euler
  characteristic). A six-faced genus-0 piece that is not hex-like fails later, in the interval
  phase or `generate`, with a failure that allows fallback.
* No smoothing (map is pure TFI; a `MeshTransform` would own that), no boundary layers, no groups.

## api_change_requests

None.

## Polish wave

Branch note: the harness designated `claude/p2-sm-scheme-mapvol-alq1zz`; the task text names `claude/p2-sm-scheme-mapvol`. Pushed to the harness branch only.

| # | Item | Status |
|---|---|---|
| 1, 10 | Cargo.lock stale | obsolete: already regenerated upstream; `cargo build --locked` passes on the starting commit. The dependency cleanup of 7/11 removes one line from the `sm-scheme-mapvol` entry (`sm-math`); that lock line is the only root-file touch, committed so `--locked` CI stays green. |
| 2, 9 | affine exactness tolerance | obsolete for the assertion (the 1e-9 rounding is only in the cell-matching helpers used by the invariance tests; both exactness tests already assert `< 1e-12` on max abs error). Tightened `affine_block_with_graded_spacing_is_exact` from nearest-lattice-point matching to node-by-node comparison with the same-index lattice point (max abs error < 1e-12). `parallelepiped_is_affine_exact` keeps nearest-point matching (mesh node order is the builder's) at 1e-12 per component. |
| 3 | frame handedness from one corner | fixed: orientation from the signed boundary volume. Test `a_strongly_bent_block_is_oriented_by_its_volume_not_by_one_corner` (270 degree bent beam, both senses, two face orders); verified it fails with `Quality` on the old code. Hashes unchanged: all existing tests, including the mesh-hash comparison, pass untouched. |
| 4 | count-mismatch only `is_err()` | fixed: `faces_that_do_not_close_a_hex_block_are_refused` now removes an interior column (corners stay vertices; the previous truncation broke the corners and failed earlier with `Unsupported`) and asserts `FailureKind::Constraint` and the reason. |
| 5a | NotApplicable from parallel_classes | fixed at unit level: `classes_from_sides` extracted; tests in `src/lib.rs` (cube, chained side, unshared side, mismatched chains, crossed directions). |
| 5b | six-faced non-hex-like piece through the driver | fixed: `a_six_faced_piece_that_is_not_hex_like_is_refused_by_the_driver` (triangular bipyramid: passes applicability, refused by the driver on the faces, since triangles cannot be imposed `QUAD`). It does not reach `generate`. |
| 5c | Quality failure to fallback, end to end | fixed: `a_folded_block_fails_with_quality_and_the_driver_falls_back` (170 degree twisted hex; map rejected after Quality; with a `tetvol` fallback both attempts are recorded in order). The stub `tetvol` generate fails by design, so the run does not end in a mesh; a successful fallback mesh needs a real volume fallback scheme. |
| 6 | logical-sides gap | recorded in DESIGN.md "Follow-ups". |
| 7, 11 | unused deps | fixed: `sm-math` dropped; `sm-geom-api` moved to dev-dependencies (tests build `Line`/`Plane`); `serde_json` was already gone. Dev `sm-model-api` stays (tests need `Model`); noted as outside the T6 dev list. |
| 8 | T6 Deps line: `sm-mesh-api`, `rayon` | not done (needs orchestrator): add `sm-mesh-api` to T6 (or re-export `CellShape` from `sm-scheme-api`), and decide whether rayon is accepted for T6 or `generate` stays sequential. Left as is; rayon use is deterministic and tested. |
| 12 | E2E NotApplicable in the interval phase | not done: the stub surface map forces four `End` corners per face, so no driver-level fixture can produce it; covered at unit level by 5a. |

