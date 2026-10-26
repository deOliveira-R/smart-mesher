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

Kernel fixtures (`acceptance_1/2`, spacings 0.125 / 0.25 / 0.5, min / mean corner-sine scaled Jacobian):

| fixture | 0.125 | 0.25 | 0.5 |
|---|---|---|---|
| square, rectangle(3,2) | 1.000 / 1.000 | 1.000 / 1.000 | 1.000 / 1.000 |
| circle(1.5, 32) | 0.525 / 0.934 | 0.680 / 0.950 | 0.783 / 0.928 |
| l_shape | 1.000 / 1.000 | 1.000 / 1.000 | 1.000 / 1.000 |

All fixtures need 0 splitting rounds. Floors (0.40 / 0.85; l_shape 0.35) are met with margin.

Scheme meshes through `Driver::mesh`:

| case | quads | SJ min / mean |
|---|---|---|
| planar brick +z face, counts (8, 6) | 48 | 1.000 / 1.000 |
| cylinder lateral (72, 72, seam 4) | 288 | 1.000 / 1.000; widest quad 0.088 rad, edge ratio 1.22 |
| cylinder bottom cap (32 nodes) | 99 | 0.696 / 0.932 |
| rectangle with two holes (S1) | 200 | 1.000 / 1.000 |
| annulus face r 1 to 2 (S1) | 276 | 0.574 / 0.928 |

Kernel S1: rectangle_with_holes 3200 / 800 / 200 quads, min 0.633 / 0.726 / 1.000; annulus(1,2,32) 983 / 272 / 128 quads, min 0.610 / 0.832 / 0.995 (mean 0.97 to 1.0). S2: 478 quads, min 0.619, mean 0.954, mean-area ratio left/right 0.1426 (gate 1/3). S3 (metric `[(1+u)^2, 0, 1]`, quality measured in the physical image): coarse (12x6) 50 quads min 0.520 mean 0.889; fine (32x16) 317 quads min 0.491 mean 0.927; medium (20x10) fails with `NoPerfectMatching` (typed fallback).

Performance: `pave_region` on 100 x 100 (400 boundary nodes, 10000 quads): 0.12 s release (gate 2 s). `Driver::mesh` of the same face: 0.10 s (recorded only).

Robustness surveys (ignored tests in `src/kernel/tests.rs`, production 14-entry ladder, release):

| corpus | failures (typed `NoPerfectMatching`) | worst min SJ |
|---|---|---|
| unit rectangle, unequal counts (ratio <= 2), 440 | 0 | 0.345 |
| random rectangles, L, U (uniform rounded spacing), 450 | 2 | 0.369 |
| circles, ellipses, 5-lobed stars, 169 | 0 | 0.151 (15 below 0.2) |
| random triangles, 150 | 32 | 0.356 |
| isosceles triangles (apex 20 to 140 deg), 66 | 21 | 0.455 |

Without the ladder the default placement fails every triangle and about 35 % of the 1275-case mixed corpus. `survey_cover`
(448 cases fail the first strategy; 55 of them no one of 94 candidates recovers) chose the first 14 ladder entries
`[0, 73, 33, 93, 10, 70, 71, 35, 40, 12, 3, 6, 21, 72]` of `candidates()`; the other candidates follow in order (the 14 alone failed all three S3 varying-metric rectangles). The survey table below was measured with the 14-entry ladder, so it is a slight underestimate of the production ladder. Isosceles table (min SJ for 4, 6, 8, 10, 12, 16 leg intervals, `--` = failure):
apex 30: 0.50 0.50 0.50 0.50 0.50 0.48; 60: 0.63 0.75 0.74 0.62 0.62 0.50; 90: 0.57 0.55 -- 0.54 0.55 --; 120: -- -- 0.50 -- -- 0.50; 20 and 140: all fail.

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

- **Acute and very obtuse corners.** Triangle regions fail the default placement and part of them the whole ladder
  (32 of 150 random triangles, 21 of 66 isosceles; corners of 20 or 140 degrees always). A failure is typed
  `Geometry`, so the driver falls back (production: `trimesh`). The fix is a corner rule (W3b). The milestone
  never asks `pave` for a triangle (V2 is a sweep).
- 2 of 450 random rectangles/L/U polygons also fail the ladder (same typed fallback).
- Some curved regions have a poor worst quad (min SJ 0.151 on a 5-lobed star); no valence cleanup yet (W3b).
- A failing large face spends at most the ladder budget (150000 triangle-attempts) before falling back.
- Deferred, as listed in `DESIGN.md`: embedded members, poles, min-weight matching, valence cleanup, Winslow,
  anisotropic metric, intra-face parallelism, cross-field, auto-selection, shared CDT crate.
- `Cargo.lock` changed (the `sm-scheme-pave` dependency list); the orchestrator should accept it at merge.

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
