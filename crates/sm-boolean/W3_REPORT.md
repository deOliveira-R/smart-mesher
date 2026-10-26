# W3 report: sm-boolean

## Summary

Mandatory scope (M0-M2) is implemented: `merge`/`merge_check`, `locate_point`, `imprint_bodies`,
`boolean` (unite/subtract/intersect with regularize through `join_faces`/`join_edges`), `webcut` with
plane, surface and cylinder tools, and `BooleanPlugin` (id `boolean`) with ops `model.unite`,
`model.subtract`, `model.intersect`, `model.imprint`, `model.merge`, `model.webcut_plane`,
`model.webcut_surface`, `model.webcut_cylinder` and the query `model.merge_check`. Planar, quadric
(cylinder, sphere, poles and seams) and marched quadric-quadric cases work on both `CellComplex`
and `TopoStore`. Stretch M3/M4 is **not started**.

Definition of done (run on this branch): `cargo build --workspace`, `cargo test -p sm-boolean
--all-features`, the same with `RAYON_NUM_THREADS=1`, `cargo clippy -p sm-boolean --all-targets
--all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-boolean --no-deps` all pass.

## Acceptance bullet -> test

| # | Test(s) |
|---|---|
| 1 | `merge::merge_two_bricks_cell_complex`, `merge_two_bricks_topo_store`, `merged_bricks_matches_the_testkit_fixture_up_to_record_order`, `ops_merge::merge_op_joins_coincident_faces` (counts [12,20,11,2,2], area 11, degree signature and Merged-record counts equal to `merged_bricks`) |
| 2 | `merge::tolerance_near_then_merge_with_tol`, `merge::skipped_faces_are_reported_and_never_merged`, `ops_merge::merge_op_skips_faces_on_an_interface_record` |
| 3 | `imprint::partial_overlap_imprint_then_merge`, `ops_merge::imprint_op_splits_overlapping_faces_and_merge_joins_the_overlap` |
| 4 | `webcut::cube_cut_at_quarter`, `reversed_normal_flips_the_roles_only`, `nomerge_gives_twelve_faces`, `section_keeps_one_half`, `ops::webcut_plane_every_form_gives_the_same_cut`, `webcut_with_three_vertices_and_reversed_order`, `webcut_surface_with_a_planar_face_matches_the_plane`, `keep_positive_and_nomerge_through_the_op` |
| 5 | `quadric_ops::webcut_cylinder_at_height`, `webcut_sphere_at_equator`, `webcut_cylinder_through_brick` |
| 6 | `quadric_ops::brick_minus_through_cylinder`, `flush_cylinder_gives_the_same_result`, `brick_minus_interior_sphere`, `webcut_prop::three_bricks_minus_the_middle_slab_leave_two_bodies` |
| 7 | `boolean_planar::unite_overlapping_bricks_regularized`, `unite_overlapping_bricks_without_regularize`, `quadric_ops::unite_brick_and_sphere_half`, `unite_brick_and_sphere_half_unregularized`, `ops::colors_and_groups_follow_the_lineage_of_a_unite` |
| 8 | `boolean_planar::intersect_overlapping_bricks`, `disjoint_bricks`, `quadric_ops::intersect_cylinder_inside_brick_returns_the_cylinder` (h=4, r=1, volume 4 pi, cells equal to a lone cylinder) |
| 9 | `marched::inclusion_exclusion_on_crossing_cylinders`, `intersection_edges_lie_on_both_surfaces` |
| 10 | `prop_bricks::inclusion_exclusion_and_symmetry` (300 cases), `webcut_prop::random_planes_split_a_cube` (200), `random_planes_split_a_cylinder` (12) |
| 11 | `boolean_planar::keep_leaves_the_inputs_untouched`, `ops::without_keep_the_inputs_are_retired_and_with_keep_they_are_not` (retired volumes and bodies equal the four inputs), `ops::failing_ops_leave_the_state_hash_unchanged` |
| 12 | `conformal_mesh::merged_cap_gives_a_conformal_mesh` (45 nodes), `unmerged_cap_doubles_the_interface_nodes` (54) |
| 13 | `shared_face::unite_x_y_keeps_the_y_z_interface`, `subtract_x_from_y_keeps_the_y_z_interface` (counts V12 E20 F11, only the Y|Z face row and Z's volume row differ and only in `side`, `Modified(Reversed)` and `Merged` records), `ops::a_color_on_the_outside_volume_survives_the_unite` |
| 14 | `determinism::scenario_is_deterministic_across_runs_threads_and_hosts`, `cold_replay_from_the_root_does_not_diverge` |
| 15 | `ops::failing_ops_leave_the_state_hash_unchanged` (tangent-only, mesh guard), `locate::all_directions_ambiguous_is_a_typed_error` |

## Measured values

- Conformal mesh (bullet 12): 16 hexes, 45 nodes merged, 54 unmerged.
- Partial-overlap imprint + merge: V16 E26 F13, total face area 46.
- Brick `[0,2]^3` union half-sphere: volume `8 + 2*pi/3`, area `24 + pi`, errors below `1e-8` relative.
- Debug-build timings (not optimized; no bench was required): crossing-cylinders marched ops about 2 s per
  test binary, `prop_bricks` (300 cases) about 15 s, determinism scenario about 11 s (it runs the full
  scenario six times across two hosts).
- Marched edges use chord tolerance `100 * linear`.

## Deviations (with reasons)

- **`merged_bricks` record order.** The merged-bricks result matches the testkit fixture structurally; the
  canonical hash differs only in lineage/coincidence record order (asserted structurally, as the brief allows).
- **Z not bit-identical on `Y|Z`.** In bullet 13 Z's cells are unchanged except the `Y|Z` face use:
  the orientation rule (ir.md section 6: two volumes are `left < right`) is re-applied by `end_op`
  because the new volume has a higher id than Z, so the face is reversed. The test asserts that only
  the `Y|Z` face row and Z's volume row differ, both only in `side`. Geometry and ids are unchanged
  and the `Merged` coincidence record is kept; the union **adds a `Modified(Reversed)` lineage
  record on the `Y|Z` face** (correct behaviour, asserted). Judged against decision 12 ("a unite
  whose input shares a merged face with a volume outside the op"): the interface stays two-sided,
  alive and merged, which is what the decision requires; bit-identity of Z's words is not
  achievable under the orientation rule.
- **`webcut imprint=true`** returns `Unsupported` (sheet-tool imprint is W3b/M3).
- **`webcut_cylinder`** with a non-wrapping cylinder patch returns `Unsupported`.
- **Subtract tool copies.** With several targets the tool is deep-copied for all but the last target.
- **Dependencies beyond the brief's list:** `smallvec` (normal), `proptest` (dev-only).
- **Bullet 1 hash.** The op-level test compares bricks from `model.create_brick` with `merged_bricks`
  structurally (counts, per-cell degree signature, Merged records), not by canonical hash, because
  the model layer's ids and bookkeeping differ from the fixture's.
- **Bullet 10 range.** Integer-grid bricks use positions `0..6` and sizes `1..5`.
- **Bullet 15 construction.** The all-8-ambiguous case is triggered by inflating the model tolerance (`linear = 0.49`
  on a unit cube, centre point), not by a ray exactly through an edge: the eight irrational directions never
  hit an edge exactly. It exercises the same all-directions-ambiguous path and typed `Ambiguous` error.
- **Disjoint operands:** unite of bodies that do not touch is `Invalid("bodies do not touch...")`, intersect is
  `EmptyResult`, subtract returns the target unchanged; tangent-only contact is `Unsupported` (`TangentContact`, naming the face or volume and a hint).

## What changed in review (round 0)

- **Fixed: order-dependent unite.** `boolean()` now fuses the running volume with any remaining
  operand that touches it and returns `Invalid("bodies do not touch ...")` only when none does;
  `new_faces` accumulates over all fuses (was replaced). Tests: `unite_fold` (all six orders of the
  A/C/B bricks give volume 3, area 14, [8,12,6,1,1]; role sets disjoint and covering).
- Bullet 13 tests strengthened (see table); the report now states the added `Modified(Reversed)` record.
- New `BooleanError::TangentContact { entity, detail }` for line tangency (the former "dangling
  chain" and "not a periodic seam vertex" messages) and for point-only contact; it maps to
  `OpError::Unsupported`. `ResultInvalid` now names near-tangent operands as a known cause.
  Tests: `tangent`.
- Tests are driven by `sm_testkit::boolean::cases()` (`testkit_cases`), including the previously untested
  `brick_sphere_half` intersect and subtract. `crossing_cylinders` has no closed form and
  `nurbs_cylinder_equiv` is M4; they stay covered by `marched` or not at all.
- Bullets 1, 8 and 11 asserted more strictly (table). A plane on an existing face and a diagonal plane through
  edges and vertices are tested for webcut.
- DESIGN.md records the fold, the near-tangent limit and the performance red flags.
- **Declined:** making a coincident-plane webcut `Invalid` (the behaviour is deliberate for stacks of
  bricks; documented and tested instead). Fixing near-tangent robustness itself (1e-5..1e-3 clearance still
  rolls back; documented as a limit).

## Known gaps

- M3 (`webcut_body`, `webcut_sheet`, `imprint_curves`, `model.unmerge`, extended `webcut_surface`) and M4
  (NURBS faces, NURBS cc) are not started.
- Curves whose `(u, v)` is not affine in the curve parameter get no pcurve (schemes can project).
- Multi-edge cut chains that connect different loops of a holed face are unsupported.
- `cc.rs` is not promoted to sm-geom.
- Docker/OpenFOAM checks: no Docker step is needed by this crate.

## api_change_requests

None.
