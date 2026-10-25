# W2 report: sm-scheme-map (T5)

Status: **done**. All Acceptance bullets of plan §2 T5 are covered by named tests; the
definition-of-done commands pass (see the end).

## Summary

`sm_scheme_map::register` registers the surface scheme `"map"` (`SurfaceMap`): caps
`QUAD | STRUCTURED | BLOCK_STRUCTURED`, `VertexRule::Map`, interval rows for logical sides
0 = 2 and 1 = 3 (coedge sums), and a `generate` that blends the four boundary lines by
transfinite (Coons) interpolation **in the face's `(u, v)` space** (pcurves; seam-safe), evaluates
the surface with `eval_many`, and falls back to 3-D TFI + projection for faces without pcurves
(and to plain 3-D for faces without a carrier). Output: `Patch::grid` plus `Quad`s along the face
normal, with the contract's grid and interior-key numbering. Pure kernel in `tfi.rs`. Design
in `DESIGN.md`.

## Acceptance bullet -> test

| Acceptance (plan T5) | Test |
|---|---|
| Planar 10x6 rectangle, counts (5, 3): 15 quads, interior nodes on the exact TFI lattice (1e-12), area exactly 60 (`sm_quality`) | `tests/map.rs::planar_rectangle_5x3_is_the_exact_tfi_lattice_with_area_60` |
| Trapezoid gives the TFI closed form | `trapezoid_matches_the_bilinear_closed_form` (+ `tfi::tests::bilinear_corners_for_uniform_straight_sides`, `graded_opposite_sides_give_graded_columns`) |
| Quarter-cylinder 4-sided face: nodes on the cylinder (1e-12), grid = uv lattice | `quarter_cylinder_nodes_lie_on_the_cylinder_on_the_uv_lattice`; seam: `full_cylinder_lateral_face_crosses_the_seam_in_parameter_space`; proves the parameter-space path (a projected 3-D blend is off the lattice): `sphere_band_nodes_sit_on_the_parameter_lattice_not_on_projected_chords` |
| Mid-edge vertex (5 vertices, Side corner) meshes as 4 logical sides | `a_mid_edge_vertex_is_a_side_corner_and_the_face_meshes_as_four_sides` (+ `a_five_sided_face_meshes_as_four_logical_sides_once_a_corner_is_demoted`) |
| Face with != 4 sides or inner loops gives `NotApplicable` | `faces_that_are_not_four_sided_or_have_holes_are_not_applicable` (triangle, annulus, sphere, hard request through the driver); params: `parameters_are_rejected_as_not_applicable`; layout-with-other-side-count is a `NotApplicable` generation failure (code path in `interval_constraints`/`generate`, not exercised, see gaps) |
| `NodeGrid::line_from` walks match `Patch::strip_from` | `grid_line_walks_match_the_element_strip_walks` (both with the grid and with `grid = None`, i.e. the element walk) |
| Surface-only commit of all six brick faces (dim-2 builder): V - E + F = 2 | `the_six_brick_faces_close_to_a_surface_with_euler_characteristic_2` (also normals agree with `face_normal`, area 13, per-face grid dims) |
| Determinism | `output_is_bit_identical_across_executors_runs_and_threads` (`sm_testkit::determinism::check`: repeats, `Sequential` vs `ScopedThreads(4)`, concurrent runs; topology, geometry and `mesh_hash` equal); whole suite also run with `RAYON_NUM_THREADS=1` |
| Fold-over is a typed quality failure | `lib.rs::tests::orientation_accepts_a_regular_grid_and_rejects_a_fold` |
| Face without carrier | `faces_without_a_carrier_are_meshed_in_three_dimensions_with_no_parameters` |

## Deviations

- **Extra dependencies** (justified): `sm-mesh-api` (runtime) because `PatchBuilder::node` takes
  `sm_mesh_api::Param` and `sm-scheme-api` does not re-export it; `thiserror` (the `tfi` error
  type). Dev: `serde_json` (params values), `sm-model-api` (`Model::new()` in test fixtures).
  `sm-math` stays in the skeleton's dependency list but is unused.
- `Cargo.lock` changed (new dependency edges of this crate only).
- **Branch name:** the harness pinned `claude/w2-sm-scheme-map-6ury5z`; the task text says
  `claude/w2-sm-scheme-map`. Pushed to the pinned branch; merge by SHA.
- **Determinism test scope:** "cold replay from root with no Diverged" does not apply: a scheme
  crate has no ops or layers (replay belongs to sm-mesher / sm-server). Determinism is shown at
  the mesh level instead (above).
- **No criterion bench:** T5 names no hot path (see DESIGN.md "Cost").
- Blending weights use 3-D chord fractions (not uv fractions) so the result is independent of
  the surface parametrization; documented in DESIGN.md.

## Known gaps

- Faces with a collapsed side (poles) are `NotApplicable` (fewer than 4 real corners).
- The "layout with a side count other than 4" failure path is implemented but untested end to
  end: the reference picker never returns such a layout for `VertexRule::Map`.
- Inside one face the work is sequential; parallelism is across faces (driver waves).
- Non-planar faces without pcurves use 3-D TFI + projection, which can pick a wrong branch on
  periodic surfaces; the topology builders that create such faces should set pcurves.

## api_change_requests

None. Two notes for the orchestrator (not blocking):
1. `sm_scheme_api::stubs::registry()` registers a surface scheme named `"map"` too, so
   `sm_scheme_map::register` fails with "already registered" on it. Ruling (w2-acr): the stubs
   stay as they are; tests that want the real map build their registry from
   `SchemeRegistry::new()` plus the specific stubs (`Equal`, ...) and then `register`, as
   `tests/common` here does. sm-mesher and sm-server tests should do the same.
2. Done (w2-acr): `sm_scheme_api::Param` is re-exported, so `sm-mesh-api` is now only a
   dev-dependency of this crate (its tests use `CellIx`, `validate`, ...).

## Definition of done (run)

`cargo build --workspace`; `cargo test -p sm-scheme-map --all-features` (6 unit + 13
integration); `RAYON_NUM_THREADS=1 cargo test -p sm-scheme-map --all-features`;
`cargo clippy -p sm-scheme-map --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-map --no-deps`: all clean.
