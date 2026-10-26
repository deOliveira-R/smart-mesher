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
| 1 | `merge::merge_two_bricks_cell_complex`, `merge_two_bricks_topo_store`, `merged_bricks_matches_the_testkit_fixture_up_to_record_order`, `ops_merge::merge_op_joins_coincident_faces` |
| 2 | `merge::tolerance_near_then_merge_with_tol`, `merge::skipped_faces_are_reported_and_never_merged`, `ops_merge::merge_op_skips_faces_on_an_interface_record` |
| 3 | `imprint::partial_overlap_imprint_then_merge`, `ops_merge::imprint_op_splits_overlapping_faces_and_merge_joins_the_overlap` |
| 4 | `webcut::cube_cut_at_quarter`, `reversed_normal_flips_the_roles_only`, `nomerge_gives_twelve_faces`, `section_keeps_one_half`, `ops::webcut_plane_every_form_gives_the_same_cut`, `webcut_with_three_vertices_and_reversed_order`, `webcut_surface_with_a_planar_face_matches_the_plane`, `keep_positive_and_nomerge_through_the_op` |
| 5 | `quadric_ops::webcut_cylinder_at_height`, `webcut_sphere_at_equator`, `webcut_cylinder_through_brick` |
| 6 | `quadric_ops::brick_minus_through_cylinder`, `flush_cylinder_gives_the_same_result`, `brick_minus_interior_sphere`, `webcut_prop::three_bricks_minus_the_middle_slab_leave_two_bodies` |
| 7 | `boolean_planar::unite_overlapping_bricks_regularized`, `unite_overlapping_bricks_without_regularize`, `quadric_ops::unite_brick_and_sphere_half`, `unite_brick_and_sphere_half_unregularized`, `ops::colors_and_groups_follow_the_lineage_of_a_unite` |
| 8 | `boolean_planar::intersect_overlapping_bricks`, `disjoint_bricks`, `quadric_ops::intersect_cylinder_inside_brick_returns_the_cylinder` |
| 9 | `marched::inclusion_exclusion_on_crossing_cylinders`, `intersection_edges_lie_on_both_surfaces` |
| 10 | `prop_bricks::inclusion_exclusion_and_symmetry` (300 cases), `webcut_prop::random_planes_split_a_cube` (200), `random_planes_split_a_cylinder` (12) |
| 11 | `boolean_planar::keep_leaves_the_inputs_untouched`, `ops::without_keep_the_inputs_are_retired_and_with_keep_they_are_not`, `ops::failing_ops_leave_the_state_hash_unchanged` |
| 12 | `conformal_mesh::merged_cap_gives_a_conformal_mesh` (45 nodes), `unmerged_cap_doubles_the_interface_nodes` (54) |
| 13 | `shared_face::unite_x_y_keeps_the_y_z_interface`, `subtract_x_from_y_keeps_the_y_z_interface`, `ops::a_color_on_the_outside_volume_survives_the_unite` |
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
- **Z not bit-identical on `Y|Z`.** In bullet 13 Z's cells are unchanged but the `Y|Z` face's
  left/right orientation is re-normalized by `end_op` after the union (a lone volume goes left), so Z's canonical words are
  not bit-identical in that one face use. Geometry, ids and lineage are unchanged.
- **`webcut imprint=true`** returns `Unsupported` (sheet-tool imprint is W3b/M3).
- **`webcut_cylinder`** with a non-wrapping cylinder patch returns `Unsupported`.
- **Subtract tool copies.** With several targets the tool is deep-copied for all but the last target.
- **`dev-dependencies`** beyond the brief: `proptest` (bullet 10) and `rayon` as a dev-dep for the
  per-thread-count pools in the determinism test. `indexmap`, `smallvec` as normal deps for ordered maps / small vectors.
- Coplanar bricks that touch only along a face give `Invalid("bodies do not touch...")` for unite with no
  overlap and no shared face; tangent-only contact is `Unsupported`.

## Known gaps

- M3 (`webcut_body`, `webcut_sheet`, `imprint_curves`, `model.unmerge`, extended `webcut_surface`) and M4
  (NURBS faces, NURBS cc) are not started.
- Curves whose `(u, v)` is not affine in the curve parameter get no pcurve (schemes can project).
- Multi-edge cut chains that connect different loops of a holed face are unsupported.
- `cc.rs` is not promoted to sm-geom.
- Docker/OpenFOAM checks: no Docker step is needed by this crate.

## api_change_requests

None.
