# W3_REPORT: sm-scheme-submap (W3-SUBMAP)

## Summary

The real surface `submap` scheme (`sm_scheme_submap::{NAME, register, SurfaceSubmap}`) plus the
pure `lattice` and `blocks` modules. A single-loop face with corner sum 4 (L, U, staircase,
five-sided rectangle, plain rectangle) is meshed as a union of mapped rectangular blocks:
conforming, all-quad, valence-4 interior, canonical interior keys (row-major rank), `Patch::grid`
`None`, strips walkable through `Patch::strip_from`. Lattice closure/positivity/simplicity are
checked exactly in integers; a failure is a `Constraint` generation failure ("lattice: ...")
that feeds the driver's fallback chain. All mandatory acceptance bullets pass; the stretch item
(graded cut spacing) was not needed (see measured values). No Docker step.

## Acceptance bullet -> test

All in `tests/submap.rs` unless noted.

| # | Bullet | Test |
|---|---|---|
| 1 | L at size 0.25: layout, rows, 48 quads / 65 nodes, (A,B,I) = (48,32,33), area, Jacobian 1, validate, Euler 1, 2 blocks; both solver/picker pairs | `case1_l_shape_at_size_quarter_is_the_48_quad_lattice`, `both_pickers_give_the_same_layouts_and_meshes` |
| 2 | L with hard counts 5,3,3,2,2,5: polygon, (19,20,10), 19 quads / 30 nodes, 2 blocks, area, Jacobian floor | `case2_l_shape_with_hard_counts_5_3_3_2_2_5` |
| 3 | five-sided: 4 sides (one with two edges), 32 quads / 45 nodes, area 8, Jacobian 1, hard goals bottom = top via the layout | `case3_five_sided_face_is_four_logical_sides_with_exact_rows` |
| 4 | rectangle(3,2) size 0.5: bit-exact lattice, 1 block, 24 quads | `case4_rectangle_is_the_analytic_lattice_in_one_block` |
| 5 | proptest staircases k = 1..4, widths/heights 1..4 (48 cases, ChaCha seed printed by proptest on failure): node set = integer lattice points, quads = A, nodes = I + B, area, Jacobian 1, validate, blocks = k + 1, disjoint, total area A | `staircase_property::random_staircases_mesh_to_the_integer_lattice`, `staircase_property::the_unit_l_staircase_is_the_testkit_l` |
| 6 | lattice unit tests: L and staircases accepted; non-simple U ("lattice:" + "not simple"); clockwise -> positivity; unclosed -> closure; Pick vs brute force on 200 seeded staircases | `lattice::tests::*` (`the_l_is_closed_positive_and_simple`, `a_non_simple_u_is_rejected_with_a_lattice_message`, `clockwise_unclosed_and_degenerate_sequences_are_typed_errors`, `staircases_are_accepted_and_pick_matches_brute_force_counting`), plus `blocks::tests::*` (literal reflex-cut oracle) |
| 7 | broken closure row -> `Infeasible` naming the rows in `ReferenceIntervalSolver` and `IiaSolver`; seam edge on both sides cancels | `case7_a_broken_closure_row_is_infeasible_in_both_solvers_naming_the_rows`, `case7_a_seam_edge_on_both_sides_cancels_without_an_empty_row` |
| 8 | fallback `submap -> pave` (stub Pave, registry without stub Submap), attempt history has the `Constraint` failure; without the chain the error is returned | `case8_a_lattice_failure_falls_back_to_pave_and_is_recorded` |
| 9 | `strip_from` from every non-corner boundary node of the L, length = extent + 1, ends on the boundary, `grid` None | `case9_strips_cross_the_l_boundary_to_boundary` (26 nodes walked) |
| 10 | applicability: rectangle 80 (map 100 beats it), L 80, corner sum != 4 -> 20, hard Reversal no, hole no, `{}` ok / `{"x":1}` rejected, caps without `STRUCTURED` | `case10_applicability_scores`, `a_surface_scheme_does_not_apply_to_a_non_surface_target` |
| 11 | `determinism::check(3, 4, ..)` over `mesh_hash`, `Sequential` vs `ScopedThreads(4)`, six brick faces in one wave, cold replay equal; `RAYON_NUM_THREADS=1` run | `case11_output_is_bit_identical_across_executors_runs_and_threads` (and the whole suite under `RAYON_NUM_THREADS=1`) |
| 12 | build, test, clippy, rustdoc `-D warnings`, DESIGN.md | see "Definition of done" |

Extra: `interior_node_keys_are_canonical_row_major_ranks` (ranks `0..33`, cut nodes on the cut
line), `a_sized_u_face_meshes_with_four_valence_interior_nodes` (a valid U, 3 blocks, 2 cuts).

## Measured values

- Case 2 (hard counts 5,3,3,2,2,5): min scaled Jacobian **0.980581** (same for both
  solver/picker pairs). Pinned at `0.9 x` this (0.8825), above the task floor 0.5.
- Cases 1, 3, 4 and the staircases: scaled Jacobian min = max = 1 within 1e-12; rectangle node
  set bit-exact (`f64::to_bits` equal).
- Wall time of the whole driver run on the L (release, `perf_large_l_shape`, `--ignored`):
  7 500 quads 23 ms, 30 000 quads 80 ms, 120 000 quads 373 ms. Not gated (no perf target in
  the task, no criterion bench named).
- Blocks: L = 2, rectangle = 1, k-step staircase = k + 1, U = 3, plus/H/T = 3.

## Definition of done

- `cargo build --workspace`: ok.
- `cargo test -p sm-scheme-submap --all-features` and with `RAYON_NUM_THREADS=1`: 16 unit + 16
  integration tests pass (1 ignored timing test).
- `cargo clippy -p sm-scheme-submap --all-targets --all-features -- -D warnings`: clean.
- `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-submap --no-deps`: clean.
- `cargo fmt -p sm-scheme-submap` applied (crate-local only).

## Deviations (and why)

1. **`loops2d::staircase` exists on main** (stage-0 items table lists it), so the tests use it
   instead of a test-local builder. Its shape is the task's staircase (last column width 1,
   heights decreasing to the right), exact integer area.
2. **`topo::planar_face(&Region2, z)`** takes a height, not a normal (code on main wins); faces
   are at `z = 0`, normal `+z`.
3. **Lattice point -> node map is a CSR row index (`PointRows`) plus ordinal arrays, not a
   `BTreeMap`.** Same determinism (no hashing, canonical order), no per-point allocation,
   `O(log runs)` lookup. Boundary keys, positions and interior ranks are arrays indexed by the
   row-major ordinal.
4. **Case 8 uses a `Soft` imposition for the fallback run.** A `Hard` imposition does not
   consult the registry chain (driver behaviour, same as the stub pave tests); the "without
   chain" half uses a `Hard` imposition and a registry with no other candidate and gets
   `NoApplicableScheme` whose `tried` attempt carries the full `Constraint` / "lattice: polygon
   is not simple" text. Through the driver the nodes attached with `with_nodes` are not visible
   on the `Attempt` (it has no node field), so that attachment is covered by code review only.
5. **Dependencies:** normal: `sm-core`, `sm-scheme-api`, `thiserror`. `sm-geom-api` and
   `sm-topo-api` from the Deps list were not needed in library code (everything goes through
   `sm-scheme-api::geom` and `GenCtx`), so they are not declared as normal deps. Dev:
   `sm-mesh`, `sm-mesh-api`, `sm-model-api`, `sm-quality`, `sm-sizing`, `sm-scheme-map`,
   `sm-testkit`, `sm-topo-api` (the `CellComplex` type of the fixtures), `sm-scheme-api[stubs]`,
   `proptest`, `serde_json` (all already used by sm-scheme-map's tests). No new external crate
   beyond the G2 allow-list.
6. `Cargo.lock` gained only this crate's dependency entries (generated, no other change).
7. No criterion bench (the task names none). The cold replay in acceptance 11 is a
   `plan_with(.., rejections)` replay, since this crate registers no ops.

## Known gaps

- Parameter-space blending through pcurves, periodic/seam faces, `Reversal`, inner loops and
  auto selection are W3b, as specified (noted in DESIGN.md).
- Graded cut spacing (stretch): not implemented, the measured L-case Jacobians do not need it.
- A face on a strongly curved carrier relies on `snap_to_face` (one `project` per interior
  node and one `face_normal` per quad); not benchmarked beyond planar faces.
- The U fallback test exercises the failure path for a *planar* U; the lattice test of
  acceptance 6 covers the exact non-simple sequence on the pure layer.

## api_change_requests

None. All needed items were on main.
