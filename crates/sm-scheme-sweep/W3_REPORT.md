# W3-SWEEP report: `sm-scheme-sweep`

Status: **all 14 mandatory acceptance bullets covered and passing; stretch (`target_mode:
"project"`, S1) not attempted.** Branch `claude/w3-sm-scheme-sweep-t8t99h` (the session's
designated branch; the task text names `claude/w3-sm-scheme-sweep`, the orchestrator can take
the final commit SHA from this branch).

## Summary

The real `"sweep"` volume scheme (dim `Volume`, caps `HEX | WEDGE | POLY`), registered by
`sm_scheme_sweep::register`. Ported from the stub `Sweep` and generalised: typed
`SweepParams` (`deny_unknown_fields`, `BadParams`), column discovery over single volumes,
composite pieces and multi-loop sources, a target transform fitted by least squares
(`fit_target_transform`), affine plus Shepard layer placement (`fit_affine`, `place_layers`),
exact cell shapes (Hex, 6-node Wedge, prismatic Polyhedron), a corner-Jacobian quality gate
reporting `FailureKind::Quality` with `GenerateFailure::with_nodes`, and rayon parallelism with
ordered collects. Design, formulas and recorded numbers are in `DESIGN.md`.

Dependencies: normal `sm-core`, `sm-math`, `sm-topo-api`, `sm-mesh-api`, `sm-scheme-api`,
`rayon`, `serde`, `serde_json`, `thiserror` (the Deps list); dev `sm-testkit`,
`sm-scheme-api[stubs]`, `sm-mesh-api[serde]`, `sm-model-api`, `sm-quality`, `proptest`,
`criterion`, and the crate itself with feature `test-hooks`. No other W3a crate. `Cargo.lock`
gained only the entry of this crate.

## Acceptance -> tests

All tests are in `crates/sm-scheme-sweep/tests/` (plus one unit test in `src/emit.rs`).

| # | Bullet | Test(s) |
|---|---|---|
| 1 | Brick 4x3x5: 60 hexes, 120 nodes, volume 60, bit-exact lattice, min SJ 1; ported stub tests | `bricks::brick_4x3x5_is_a_bit_exact_lattice`; `port_brick` (9 tests incl. `sweep_brick_is_analytic`, fallback, hard pave), `port_copies` (6), `port_claims` (2): copies of the `sm-scheme-api` tests run against the real scheme |
| 2 | Layer counts from intervals, size, composite 3+4, bias 1.2, `layers` row, conflicting counts | `bricks::a_column_edge_count_of_seven_gives_seven_layers`, `size_two_on_a_height_ten_brick_gives_five_layers`, `a_composite_of_two_bricks_with_columns_3_and_4_gives_seven_layers`, `bias_on_the_column_edges_gives_a_geometric_layer_ratio_and_exact_interior_levels`, `layers_forces_a_linear_row_on_a_two_edge_column`, `conflicting_hard_counts_are_infeasible_and_name_both_edges`; extra: `composite_columns_of_edge_nodes_cross_both_members`, `a_single_biased_column_is_blended_into_the_interior_without_inversions` |
| 3 | Extruded L: 48 hexes, 105 nodes, volume 6, SJ 1, re-entrant column | `fixtures::extruded_l_has_48_hexes_and_105_nodes` (+ `fixture_invariants_are_as_documented`) |
| 4 | Axial cylinder: 64 wedges, 85 nodes, r = 1, volume 6.122934918, target by copy | `curved::axial_cylinder_is_64_wedges_on_the_circle` |
| 5 | Hollow cylinder: 64 hexes, 160 nodes, r = 1 or 2, volume 18.368804 | `curved::hollow_cylinder_seam_strips_give_64_hexes`, `curved::strip_from_crosses_a_seam_face_to_the_opposite_circle` (risk 1) |
| 6 | Rotational 90 / 270 degrees: 108 / 216 hexes, radii, angles, volume, `Rz(sweep)` | `fixtures::rotational_90_degrees_9_layers`, `rotational_270_degrees_18_layers`, `rotational_90_degree_mesh_volume_is_2_3442504` |
| 7 | Twisted prism: exact affine layers, volume 8, Jacobians > 0, SJ recorded | `fixtures::twisted_prism_layers_are_exact_affine_images` |
| 8 | Polyhedral prism: 3 polyhedra V12 E18 F8, volume, validate | `curved::polyhedral_prism_has_three_hexagonal_prisms` |
| 9 | `fit_affine` / `fit_target_transform` | `fit::*` (8 tests) |
| 10 | `place_layers` on synthetic data | `place::*` (7 tests) |
| 11 | Failure modes | `failures::faces_that_touch_are_not_applicable`, `a_volume_without_any_sweepable_face_pair_is_not_applicable_with_a_reason`, `a_non_affine_target_in_auto_mode_names_the_residual`, `emit::tests::only_tri_quad_and_polygon_sources_are_supported`, `an_inverted_layer_is_a_quality_failure_with_nodes_and_the_driver_falls_back`; also `bad_parameters_are_bad_params`, `bad_parameters_surface_from_the_driver_as_bad_params`, `a_min_scaled_jacobian_above_the_mesh_quality_fails_the_gate` |
| 12 | Determinism | `determinism::l_prism_is_bit_identical_across_executors_and_pools`, `twisted_prism_with_shepard_is_bit_identical_across_executors_and_pools`, `harness_check_over_repeated_and_concurrent_runs`; whole suite also run with `RAYON_NUM_THREADS=1` |
| 13 | proptest | `prop::twisted_prism_volume_and_jacobians` (48 cases) |
| 14 | Performance (record-only) | `perf::brick_100_cubed`, `perf::twisted_100_cubed` (`--ignored`), `benches/sweep.rs` |

## Measured values

* Twisted prism (2, 3, pi/2), 4x4 source, 6 layers: min scaled Jacobian **0.832050**.
* Rotational 90 degrees, 9 layers: volume 2.3442504 (the formula value, to 1e-6 in the test and
  to 1e-12 relative against `(L/2)(r_out^2 - r_in^2) h sin(sweep/L)`).
* Performance (release, 4 rayon threads): brick 100^3 (10^6 hexes) `Driver::generate`
  **0.79 s**, bulk commit 15.8 s (**95 %** of generate + commit); twisted prism 100^3
  `Driver::generate` **6.0-6.9 s**, commit 15.4-16.4 s (70-73 %). Reference: 4.7 s driver-only
  for a complete 10^6-cell mesh. In the twisted case 3.3 s are 396 `Patch::strip_from` calls
  (see ACR 1); placement + gate + emission take about 0.15 s.
* Determinism cases: 43 200 hexes (L-prism) and 40 000 hexes (twisted, Shepard path).

## Deviations (with reasons)

1. **Acceptance 3 source scheme.** The frozen stub `Submap` meshes four-sided layouts only
   (`6 logical sides (stub meshes four)` for the L), so the L is meshed by a test-local lattice
   quad scheme (`tests/common/surf.rs::GridFace`); the mesh is the one the bullet describes.
2. **Acceptance 12 mesh.** "40x40 (stub Submap) x 25 layers L-prism" is run twice: an L-prism
   from the test-local lattice scheme (1728 quads x 25 = 43 200 hexes) and a 40x40 stub-Submap
   twisted prism x 25 layers (40 000 hexes, which also drives the Shepard path in parallel).
   "Cold replay from root, no `Diverged`": not applicable, the crate registers no ops.
3. **Acceptance 11, hanging element.** The driver never lets a non-surface element reach a
   surface patch (`check_patch` refuses it; a claimed edge's segments are moved out by
   `split_claims`), so `Unsupported` is unreachable through `Driver::mesh`. It is covered by a
   unit test on the shape check (`emit::tests`), the guard stays in `generate`.
4. **Acceptance 10.** Boundary nodes are inputs, not outputs, of `place_layers`: "placed
   exactly" is tested as an interior node coincident with a boundary node landing on its
   column, and in the mesh tests the cells reference the linking-face and target nodes
   themselves. "interpolate and affine agree when the residual is zero" is tested for pure
   translation fields; for a rotation, interpolation of the displacement field only
   approximates (Shepard reproduces constants only), which is why `Auto` uses the affine part.
5. **Test hook.** The 170 degree bend is a `bend` parameter behind the cargo feature
   `test-hooks` (enabled by the crate's own dev-dependency), so the production schema rejects it.
6. **`Placement::Auto`** retries the whole sweep with interpolation if the gate rejects the
   affine result (not layer by layer).
7. **`target_mode: "copy"`** registers the copy with the best-effort fitted transform when the
   strict fit fails; the copy is then the user's responsibility and the quality gate is the
   only check. `auto` is `NotApplicable` with the residual.
8. **Wedge Jacobian.** The gate's wedge corner Jacobian is the plain normalized triple product;
   `sm-quality`'s wedge metric is weighted to an equilateral base. Hexes agree.
9. **Bad parameters** are reported when the requirements run (applicability returns applicable
   for unparsable parameters), so `Driver::plan` yields `SchemeError::BadParams`.
10. **Translation fast path** uses the first column difference itself when all differences are
    bit-identical, instead of the (inexact) mean.
11. `fit_affine` requires 4 points and rank >= 2 as specified; `fit_target_transform` uses the
    same fit without the point-count floor, so a triangle source plus the virtual pair works.

## Known gaps

* Stretch not done: `target_mode: "project"`, `claim` of the target, acceptance S1.
* Deferred as planned: 360 degree revolve and pole columns, many-to-one/multisweep, smoothing,
  auto selection, model-tolerance copy tolerance.
* The copy tolerance is the constant `1e-9 * bbox diagonal`.
* Polyhedral side faces are non-planar under twist (exporter question).
* The Docker steps (OpenFOAM checkMesh, DAGMC) are not part of this task.

## api_change_requests

1. **sm-scheme-api, `Patch::strip_from` (performance).** What: a way to answer many strips of
   one patch without rebuilding the edge index per call, for example
   `Patch::strip_index(&self) -> StripIndex` with `StripIndex::strip_from(key)`, or a batch
   `Patch::strips_from(keys)`. Why: a block-structured (Submap-style, grid-less) linking face
   rebuilds a `BTreeMap` of all its quads for every boundary node; the twisted 100^3 sweep
   spends 3.3 s of its 6 s in 396 such calls (7.5 ms each). Proposed signature as above;
   semantics identical to `strip_from`. Not blocking.
2. **sm-scheme-api, `PlanCtx` model tolerance** (already queued): a `tolerance()` accessor to
   replace the `1e-9 * bbox diagonal` constant.
