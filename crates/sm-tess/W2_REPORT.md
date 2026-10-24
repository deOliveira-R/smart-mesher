# W2 report: sm-tess (T2)

## Summary

`sm-tess` tessellates B-rep faces for display: `tessellate(view, faces, opts)` returns `f32` positions, `u32` indices, per-face index ranges and per-edge polylines that adjacent faces share by index (watertight by construction). `TessPlugin` registers the read-only query `view.tessellate {entities, chord_tol?, angle_tol?, max_edge?}` returning base64 buffers and ranges. Design, bounds and gaps are in `DESIGN.md`.

Status: complete against the T2 acceptance list. Definition of done run on this branch: `cargo build --workspace`, `cargo test -p sm-tess --all-features`, `RAYON_NUM_THREADS=1 cargo test -p sm-tess --all-features`, `cargo clippy -p sm-tess --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-tess --no-deps` all pass.

Branch: the task text says `claude/w2-sm-tess`; this session was assigned `claude/w2-sm-tess-3y8jzw` and instructed to push only there. The work is on `claude/w2-sm-tess-3y8jzw`; the orchestrator should merge by SHA (last commit subject starts with `w2 sm-tess: done`).

## Acceptance bullets to tests

| Acceptance bullet | Test |
|---|---|
| Brick: 12 triangles, total area exactly 600 | `geometry::brick_has_12_triangles_and_area_exactly_600` |
| Cylinder lateral area within the derived chord-error bound | `geometry::cylinder_lateral_area_within_derived_chord_bound` (bound `2 pi h (r - r cos(theta/2))`, `theta` from the shared circle polyline) |
| Sphere area within the bound | `geometry::sphere_area_within_bound` (bound from the actual triangles' circumradii, derivation in DESIGN.md) |
| Watertight: edge vertices shared bit-exactly by both adjacent faces | `geometry::fixtures_are_watertight_and_consistently_oriented` (all six testkit fixtures: boundary segments present in the right direction in every adjacent face, every interior edge used twice, no repeated directed edge, polyline vertices used by every adjacent face by index), `geometry::shared_edges_are_sampled_once_for_all_neighbours` |
| Triangle normal agrees in sign with the face normal | `geometry::triangle_normals_agree_with_face_normals` (all fixtures incl. the reversed-loop bottom cap), plus the same check in every `shapes::*` test |
| 1 thread and N threads give byte-identical buffers | `geometry::output_is_identical_for_one_and_many_threads_and_repeats`, `query::query_bytes_are_identical_across_threads_runs_and_cold_replay`; whole suite also run with `RAYON_NUM_THREADS=1` |
| `TessPlugin` registers `view.tessellate`; `x-entity-kind` on id fields; no ops | `query::plugin_registers_the_query_with_entity_kinds_in_its_schema` |
| Reply = library result, base64 round trip | `query::reply_buffers_equal_the_library_result`, `query::rfc4648_vectors` (unit) |
| Never mutates state | `query::query_does_not_change_state_and_accepts_volumes_and_edges` |
| Determinism + cold replay from root | `query::query_bytes_are_identical_across_threads_runs_and_cold_replay` (state after `state_at_from_root` equals head; reply bytes equal) |
| Beyond the list: holes, apex pole, unequal rings, two-seam torus | `shapes::{planar_face_with_hole, cone_with_apex_collapses_the_pole_row, frustum_with_unequal_rings_is_merge_triangulated, torus_face_with_two_seams_gets_interior_rows}` |
| Polygon triangulator properties | `earcut::tests::{random_star_polygons, random_star_polygons_with_a_hole, ...}` (proptest) |
| Typed errors, options validation, ordering/duplicates | `geometry::{bad_inputs_are_typed_errors, face_order_and_duplicates_do_not_change_the_output, empty_input_is_an_empty_tessellation}`, `query::{bad_input_is_an_error_not_a_panic, missing_model_layer_is_reported}` |

## Deviations (with reasons)

- **Dependencies.** Dropped `sm-math` and `sm-geom` from the skeleton (unused: evaluation goes through the `GeomStore` trait objects). Added `thiserror`, `serde`, `schemars` (query types, errors) and dev-deps `serde_json`, `proptest`, `sm-testkit`. `base64` is hand-written (30 lines, RFC 4648 vectors tested) because the crate is not in `Cargo.lock` and there is no network. `sm-core` is built with `serde` + `schemars` features (needed for `EntityId` in the query schema). `Cargo.lock` changed only in the `sm-tess` entry, as a consequence of this.
- **Query input** has an extra optional `max_edge` beyond the plan's `{entities, chord_tol?, angle_tol?}`, and `entities` accepts faces, volumes, bodies and edges (volumes/bodies expand to faces, edges give polylines only).
- **Default tolerance** when `chord_tol` is absent: `1e-3` of the scene box diagonal (the plan leaves the default open).
- **Triangulation strategy** is narrower than a general constrained-Delaunay face mesher on purpose (it is the droppable task): structured grids for parameter-rectangle faces and boundary-only ear clipping elsewhere. See the gaps.

## Known gaps

- No per-vertex normals in the output.
- Curved faces that are not parameter rectangles (trimmed or multi-loop curved faces, NURBS patches whose opposite sides are sampled differently) get boundary points only: interior chord error is not controlled (flagged by `boundary_only`).
- Grid faces need matching left and right side samples; otherwise they fall back to the boundary-only path.
- Implicit-only surfaces and periodic faces without pcurves return typed errors.
- Not cached in `DerivedCache`.
- No criterion bench (none named in the task); informal release timings are in `DESIGN.md`.
- Only reference-complex (`CellComplex`) fixtures were exercised; `sm_topo::TopoStore` is not a dev-dependency (not in the Deps list). The code uses only `TopologyView`, so a `TopoStore` run is expected to match (the canonical hash equivalence is sm-topo's contract).

## Fix round 0 (review findings)

- **Grid sag (must-fix 1, 2).** Interior rows now take the larger of the blended count and the count of their own iso-`v` curve (circle rule at the row's distance from the axis; `adaptive_params` for non-revolved surfaces), which fixes the formula error on the inner-seamed torus and equator-crossing sphere bands. On doubly curved surfaces (not plane, cylinder, cone) the chord budget is split: edges adjacent to such a face and the face's rows use `chord_tol / 2` (`sample::grid_opts`, `tess::edge_options`). The edge rule inspects the whole model, so polylines do not depend on the requested face set. Measured max distance to the surface (test, `angle_tol = 3`): sphere 0.71-0.99x `chord_tol`, torus 0.99x for both seams, cone/frustum 0.87x, cylinder lateral under 1x; before: sphere 1.87-1.93x, torus 1.97x / 2.95x. Tests: `geometry::sphere_sag_is_within_chord_tol`, `shapes::{torus_sag_is_within_chord_tol_for_both_seams, cone_and_frustum_sag_is_within_chord_tol}`, lateral sag in `geometry::cylinder_lateral_area_within_derived_chord_bound`. Cost: sphere and torus meshes have about twice as many points per direction (about four times the triangles) for the same `chord_tol`. The `TessOptions` rustdoc, crate docs and DESIGN.md state the bound and that it is derived and measured for the analytic grid surfaces, not a theorem for freeform ones. Limit: a freeform grid face whose side edges are straight lines gets no `v` refinement (extra rows would need vertices on the shared side edges).
- **SHOULD 1 (earcut speed).** Work-queue ear clipping with local re-tests and an `x`-sorted reflex index: the 10 004-vertex plate with hole drops from 2.5-4.3 s to about 0.16 s (release); test `earcut::tests::large_plate_with_a_hole_is_fast_and_correct`.
- **SHOULD 2.** `query::plugin_alone_registers_exactly_one_query_and_no_ops` builds a `Registry` with only `TessPlugin`.
- **SHOULD 3.** `FaceRange::boundary_only` / `FaceRangeOut::boundary_only` (serde default false) flag curved faces triangulated from their boundary only. Test `shapes::curved_face_with_a_hole_is_flagged_boundary_only` (torus with a hole loop).
- **SHOULD 4.** `shapes::reversed_curve_carrier_still_gets_a_grid`: frustum whose seam is carried `Sense::Reversed` (pcurve parameter equals the curve parameter); it must produce the same interior rows and triangle count as the forward fixture.
- **SHOULD 5, 9.** Rows follow the circle rule including `angle_tol`, so turning between neighbouring points of every row is bounded; the rustdoc is narrowed to say that the angle between grid lines across rows is not controlled.
- **SHOULD 6, 8.** `earcut::triangulate` returns an error (`coincide on the 2^30 snapping lattice`) when two distinct mesh vertices of a ring snap together, instead of dropping one and cracking the edge. Test `earcut::tests::distinct_vertices_on_one_lattice_point_are_an_error_not_a_crack`.
- **SHOULD 7.** Dependencies unchanged; the orchestrator should amend the T2 Deps line to include `thiserror`, `serde`, `schemars` (not edited here, outside the crate).
- **SHOULD 10.** Done in DESIGN.md ("Sag bound on grid faces"); the edge-sagitta theorem is now stated as edge-only.

## Fix round 1 (review findings)

- **MUST 1 (frustum sag).** Cause confirmed: rings with different radii got different counts, and the zipper's cross-ring edges (not generators) cut across the slant (2.0-2.5x at ring ratio 20, 1.9-2.2x at half angle 0.05). Fix (option b, at the edge level): `tess::edge_rule` samples every circle next to a cone with the radius of the largest circle on that cone, so all rings have equal counts and parameters and the strips are planar quads; interior rows are never finer than that. No change to `grid_face`'s zipper. Measured now (`angle_tol = 3`, chord 1e-2/1e-3/1e-4): 0.36-0.99x over cone sweep `(a, vb, vt)` = (0.5,4,-), (0.5,6,1.5), (0.5,10,0.5), (0.2,10,0.5), (0.05,10,1), (0.5,10,2), (0.05,10,5), (0.05,10,0.5), (1.2,10,0.2); with `max_edge` 1.0 and 0.7 also within `chord_tol`. Tests: `shapes::cone_and_frustum_sag_is_within_chord_tol` (extended, all nine cases, also structure and normals), `shapes::frustum_rings_get_one_common_point_count` (replaces `frustum_with_unequal_rings_is_merge_triangulated`, which asserted the old unequal counts), `shapes::frustum_rings_share_one_point_count_and_max_edge_cones_stay_in_tolerance`. DESIGN.md, `TessOptions`, `grid_opts`/`is_doubly_curved` and the crate docs are corrected. Cost: the small ring of a frustum has the big ring's point count. Not taken: cone budget halving (unnecessary once the strips are planar).
- **SHOULD 1 (size budget).** Done: `TessError::TooLarge` above 2^24 vertices per tessellation, checked before allocation (edge numbering; grid faces sum rows x points first and reserve from a shared atomic counter). The `u32` overflow in `g_count + steiner.len()` can no longer occur. Test `geometry::tiny_chord_tol_is_too_large_not_an_abort` (unit sphere, `chord_tol = 1e-9`). Documented in `TessOptions`, `TessError::TooLarge`, crate docs and DESIGN.md.
- **SHOULD 2.** Done: f32 rounding term stated in the `TessOptions` rustdoc and DESIGN.md.
- **SHOULD 3.** Done: `shapes::sphere_band_across_the_equator_is_within_chord_tol` (bands `|v| <= 0.4, 0.8, 1.2`; measured 0.97-1.00x; with the per-row own count disabled it fails at 1.005x).
- **SHOULD 4, 5 (Deps line).** Not a crate change. Orchestrator action: amend the T2 Deps line in `docs/design/w2/plan.md` to add `thiserror`, `serde`, `schemars` and drop `sm-math`, `sm-geom`. The crate's `Cargo.toml` is unchanged.
- **SHOULD 6.** Noted in DESIGN.md (an edge between a cylinder or cone and a doubly curved face refines the developable side too).
- **SHOULD 7.** Skipped on purpose: the 0.5 split is the derived bound, and the measured 0.99x is that bound, not an unexplained margin; the only excess comes from `f32` rounding, which is now documented. A factor of 0.45 would add about 5% points per direction and change no guarantee. Reasoning in DESIGN.md.

## api_change_requests

None.
