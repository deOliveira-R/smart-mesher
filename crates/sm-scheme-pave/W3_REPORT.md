# W3-PAVE report (`sm-scheme-pave`)

Status: **done** for every mandatory acceptance bullet (1-3, 4a, 5, 6, 7a, 8-10, 11) plus stretch S1
and S2 (S3 measured and reported). One known gap with a typed fallback: acute-corner (triangle)
regions, see "Known gaps".

## Summary

`"pave"` is registered by `sm_scheme_pave::register` (surface scheme, `Caps::QUAD`, score 40). The
pure kernel `pave_region` meshes a planar region (any number of loops) with fixed boundary nodes:
metric-corrected frame, constrained Delaunay triangulation with exact predicates, frontal rows,
Delaunay refinement, perfect matching of the dual (greedy by quality + Edmonds blossom, flip
repair, edge splitting), smart-Laplacian smoothing. The scheme glue takes boundary `(u, v)` from
`GenCtx::loop_uvs`, evaluates the interior with `eval_many`, and checks every quad. Details:
`DESIGN.md`.

Branch: pushed to `claude/w3-sm-scheme-pave-p05xc4` (the branch the session harness designated; the
task text says `claude/w3-sm-scheme-pave`).

## Acceptance bullet to test name

| Bullet | Test (file) |
|---|---|
| 1 kernel: conformity, ownership, area, orientation, Euler, containment | `acceptance_1_conformity_ownership_area_euler_on_the_fixtures` (`src/kernel/tests.rs`), fixtures `square`, `rectangle(3,2)`, `circle(1.5,32)`, `l_shape` at 0.125, 0.25, 0.5 |
| 2 kernel quality | `acceptance_2_quality_floors` |
| 3 even rule, `LoopTooSmall`, `BoundaryIntersects` | `acceptance_3_even_rule_for_every_mandatory_fixture` (also duplicates, touching, a hole poking through, non-finite, wrong hole winding, bad metric) |
| 4a constant metric `[4, 0, 1]` | `acceptance_4a_constant_metric_meshes_like_a_2_by_1_rectangle` |
| 5 planar brick face | `acceptance_5_planar_brick_face` (`tests/driver.rs`) |
| 6 cylinder lateral face | `acceptance_6_cylinder_lateral_face` |
| 7a cylinder cap | `acceptance_7a_cylinder_bottom_cap_is_a_disk_of_quads` |
| 8 even rule through the driver | `acceptance_8_soft_odd_counts_are_repaired_to_an_even_sum_and_mesh`, `acceptance_8_hard_odd_counts_blame_pave_and_fall_back_to_tri`, `acceptance_8_a_loop_with_sum_below_four_is_infeasible` |
| 9 `NotApplicable` reasons | `tests/applicability.rs`: `a_face_with_an_embedded_edge_is_refused`, `a_sphere_face_with_poles_is_refused`, `unknown_and_out_of_range_parameters_are_refused_with_a_reason`, `a_face_without_a_surface_carrier_is_refused`, `a_periodic_face_whose_seam_cannot_be_unrolled_is_refused`; plus `a_plain_face_scores_forty`, `caps_are_quad_only_and_groups_are_not_supported` |
| 10 determinism | `tests/determinism.rs`: `the_kernel_is_bit_identical_across_runs_and_concurrent_callers`, `the_scheme_is_bit_identical_under_both_executors_and_concurrent_callers` (planar, cylinder, cap: `topology_hash` and `geometry_hash` under `Sequential` and `ScopedThreads(4)`), `the_library_has_no_hash_collections_env_clock_or_io`; `acceptance_10_executors_give_bit_identical_meshes` (`tests/driver.rs`); `kernel_is_deterministic_and_reflection_invariant` |
| 11 performance gate | `acceptance_11_performance_100_by_100` (release-gated at 2 s) |
| S1 holes | kernel `stretch_s1_regions_with_holes`; scheme `stretch_s1_rectangle_with_two_holes_meshes_through_the_driver`, `stretch_s1_annulus_face_with_two_circle_edges` |
| S2 graded boundary | `stretch_s2_graded_boundary` |
| S3 varying metric | `stretch_s3_varying_metric_quality_is_reported` (prints, asserts only validity) |

Also covered: size field through `generate` (`a_size_field_steers_the_interior_through_generate`),
kernel unit tests (CDT validity, blossom against brute force on 300 random graphs, metric frame).

## Measured values

Everything below is from a release build unless stated; thresholds are the proposed ones and none was
lowered.

SURVEYS_PLACEHOLDER

## Deviations (with reasons)

- **Branch name**: the harness designated `claude/w3-sm-scheme-pave-p05xc4`; pushed there.
- **Extra `PaveError` variants** (`#[non_exhaustive]` enum): `BadOrientation { ring }` (a hole winding like
  the outer loop), `NonFinite`, `Degenerate` (a quad collapses in the original parameter space
  although it is valid in the corrected frame), `Internal(&str)` (guards). The task listed five.
- **Strategy ladder** (not in the task text): the default placement leaves a triangle dual without a
  perfect matching on coarse unequal boundaries and acute corners, so the kernel retries a fixed ordered list
  of placement variants (`DESIGN.md`), under a work budget. The result is still a pure deterministic
  function of the input.
- **Flip repair** before the plan's edge splitting: any convex edge near an unmatched triangle is flipped
  if that lowers the exposure (the triangulation then need not stay Delaunay).
- **Annulus area** (S1): the mesh tiles the polygonal annulus, so the area is checked against
  `ngon_area(64, 2) - ngon_area(32, 1)` to 1e-12 (and against `pi (4 - 1)` to 3 %), not `pi (4 - 1)` to 1e-12.
- **`PaveError::Internal` maps to `FailureKind::Geometry`** (the ladder guards and the vertex budget are not
  contract violations, and `FailureKind::Internal` would abort the whole run instead of falling back).
- **No criterion bench**: the task names none; the 100 x 100 timing is a release-gated test plus an ignored
  recorded `Driver::mesh` timing (`driver_time_for_a_100_by_100_face_is_recorded`).
- **Item 8 blame**: the planner never blames a scheme whose choice is a *hard* intent, so a loop with sum
  below 4 under a hard `pave` is `Infeasible` with `blame = None`; the blame `(Target::Surface(face), "pave")`
  is asserted for a soft choice. The hard-odd test uses a soft scheme choice as in `sm-scheme-api/tests/pave.rs`.
- **Surface-only runs** (items 5-8, 10) select `[face]` and use `MeshBuilder::surface()`; the stub `sweep`
  is not needed. The test registry is `stubs::Equal` + `stubs::Tri` + the real `pave` (never
  `stubs::registry()`).
- **Cold replay / `Diverged`**: the crate registers no ops, queries, layers or formats, so that clause of the
  conventions does not apply. `sm_testkit::determinism::check` covers the repeat and concurrent-caller cases.

## Known gaps

GAPS_PLACEHOLDER

## api_change_requests

None. Stage-0 items used and found as described: `GenCtx::loop_uvs`, `GenerateFailure::with_nodes`,
`planar_face`, `annulus_face`, sm-quality at L2, the skeleton crate.

## Dependencies

`sm-core`, `sm-math`, `sm-geom-api`, `sm-topo-api`, `sm-scheme-api`, `serde` (derive), `serde_json`,
`thiserror` (the task's list; nothing from the G2 allow-list beyond that). Dev: `sm-testkit`,
`sm-scheme-api[stubs]`, `sm-mesh-api` (its `reference` builder), `sm-model-api`, `sm-quality` (exactly the
task's dev list). No dependency on sm-scheme-map, sm-scheme-curve,
sm-sizing or any other W3a crate.

## Docker

Not needed by this task (no OpenFOAM or DAGMC step).
