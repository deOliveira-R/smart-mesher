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
- Curved faces that are not parameter rectangles (trimmed or multi-loop curved faces, NURBS patches whose opposite sides are sampled differently) get boundary points only: interior chord error is not controlled.
- Grid faces need matching left and right side samples; otherwise they fall back to the boundary-only path.
- Implicit-only surfaces and periodic faces without pcurves return typed errors.
- Not cached in `DerivedCache`.
- No criterion bench (none named in the task); informal release timings are in `DESIGN.md`.
- Only reference-complex (`CellComplex`) fixtures were exercised; `sm_topo::TopoStore` is not a dev-dependency (not in the Deps list). The code uses only `TopologyView`, so a `TopoStore` run is expected to match (the canonical hash equivalence is sm-topo's contract).

## api_change_requests

None.
