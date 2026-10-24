# W2 T1 report: sm-model

Status: **done** (all acceptance bullets covered; stretch primitives not done, see gaps).

## Summary

`ModelPlugin` registers 8 ops (`model.create_brick|cylinder|sphere`, `model.move|rotate|scale|reflect`,
`model.delete`) and 3 queries (`topo.related`, `model.list`, `model.measure`). It registers no layers.
All topology edits go through `sm_layer_api::topo_txn` / `commit_topology`; surviving ids of
transforms are reported with the `volumes` role (`ApplyCtx::record_role` through `commit_topology`).
Every op declares `x-entity-kind` on its id fields and `OUTPUTS` (except `model.delete`, which has
no id to report). See `DESIGN.md`.

Definition of done, as run:
`cargo build --workspace`; `cargo test -p sm-model --all-features` (37 tests, one of them a proptest of 48 seeded cases);
`RAYON_NUM_THREADS=1 cargo test -p sm-model --all-features`;
`cargo clippy -p sm-model --all-targets --all-features -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-model --no-deps`: all clean.

## Acceptance bullet -> test

All in `crates/sm-model/tests/` unless noted.

| Acceptance bullet | Test |
|---|---|
| `create_brick(10)`: 8/12/6, vertices bit-exact +-5, volume 1000 (1e-12 rel), `validate` clean, Euler 2 (both hosts) | `primitives::brick_counts_vertices_volume_euler_and_validate` |
| brick canonical hash == `CellComplex` fixture `brick(-5, 10)` | `primitives::brick_hash_equals_the_cell_complex_fixture` (reference and production store) |
| `Delta.created` = the 28 ids ascending; `outputs` = declared roles | `primitives::brick_delta_lists_the_28_ids_ascending_and_the_declared_roles` |
| cylinder volume pi r^2 h | `primitives::cylinder_volume_and_topology` |
| sphere volume 4/3 pi r^3 | `queries::measure_cylinder_and_sphere_match_closed_forms` (closed form, see deviations); topology: `primitives::sphere_volume_and_topology` |
| move (1,2,3): vertices bit-exact | `transforms::move_translates_every_vertex_bit_exactly_and_keeps_ids`, `move_by_body_moves_its_volume` |
| rotate 90 deg about z: vertices within 1e-12, outward normals kept | `transforms::rotate_90_about_z_maps_vertices_and_keeps_outward_normals`, `rotate_about_an_offset_axis` |
| reflect: positive volume, outward normals, `validate` clean | `transforms::reflect_brick_keeps_positive_volume_outward_normals_and_validates`, `reflect_cylinder_and_sphere_keep_pcurves_and_orientation` |
| mirror deck declared beforehand stays consistent (`sm_model_api::validate`) | `transforms::mirror_deck_stays_consistent_through_every_transform` (move, rotate, scale, reflect, diagonal reflect) |
| `model.delete`: ids retired, `topo.related` no longer lists them | `queries::delete_retires_ids_and_later_queries_do_not_list_them` |
| `MeshLayer` entries on the deleted volume become `stale` | `queries::delete_marks_meshes_on_the_volume_stale` |
| model tables stay consistent after delete | `queries::delete_keeps_the_model_tables_consistent` |
| `topo.related` == brute force on every `all_topo_fixtures()` case | `queries::topo_related_matches_brute_force_on_every_fixture` (every source cell x every kind, plus unions); `topo_related_named_relations_on_a_brick`, `topo_related_rejects_bad_input` |
| transform of a meshed volume refused (`Invalid`, "mesh.delete first"); missing layer tolerated | `transforms::transform_of_a_meshed_volume_is_invalid_and_mesh_delete_hint_is_in_the_message`, `transform_without_a_meshes_layer_is_allowed` |
| determinism: two runs, 1 vs N threads, concurrent runs | `replay::scenario_is_deterministic_across_runs_threads_and_hosts` (also run under `RAYON_NUM_THREADS=1`) |
| cold replay from root, no `Diverged`; undo/redo | `replay::undo_redo_and_cold_replay_do_not_diverge` |
| JSON params (`10` vs `10.0`, explicit nulls) record like typed params | `replay::json_params_hash_like_typed_params` |
| `x-entity-kind` on every id field, `OUTPUTS` declared | `queries::every_declared_entity_id_has_a_kind_annotation_and_ops_declare_outputs`, `primitives::schemas_declare_entity_kinds_outputs_and_positional` |
| refusals leave the state untouched | `transforms::invalid_requests_are_refused_without_changing_the_state`, `primitives::invalid_parameters_are_refused_and_leave_the_state_alone`, `transforms::a_shared_face_refuses_a_partial_move_but_allows_the_whole_assembly` |
| random chains of transforms on bricks and cylinders (volume x f^3, outward, valid, both hosts agree) | `props::chains_of_transforms_keep_the_model_valid_and_agree_across_hosts` (seeded via `sm_testkit::strategies::config`) |
| scale of cylinders with pcurves, spheres | `transforms::scale_brick_cylinder_and_sphere_about_a_point` |
| measure: brick, cylinder, sphere, edges, vertices | `queries::measure_brick_edges_faces_volume_and_boxes`, `measure_cylinder_and_sphere_match_closed_forms`, `measure_rejects_unknown_and_non_topology_ids` |

## Deviations (with reasons)

- **Sphere volume goes through `model.measure`, not `volume_moments`.** The default
  `TopologyView::face_moments` returns `Unsupported("face on a surface with poles")` for the sphere
  (see api_change_requests). `model.measure` uses the closed forms for a *whole* sphere and fails
  for a partial pole patch.
- **Reflect does not just call `reverse_face`.** `reverse_face` on a face with a lone volume is undone
  by `end_op`'s orientation normalisation. The bracket in `DESIGN.md` (remove uses, reverse, fix
  carrier sense, re-add uses) is what works; it is checked against both hosts.
- **Extra refusals not in the plan:** a closure cell shared with an unselected volume; a curve or
  surface handle shared outside the selection. Both would tear the model.
- `topo.related` with the target kind equal to a source's kind returns that source (the plan did not
  say); `within` is validated against the topology rather than the entity table (needed for the
  fixture test).
- Cargo: dropped `sm-math` and `sm-geom` (unused), moved `sm-topo` to dev-dependencies (tests only).
  Added `serde`, `schemars`, `thiserror` (regular) and `serde_json`, `proptest`, `sm-mesh-api` (dev;
  `sm-mesh-api` only for `CellShape` in a test op that inserts a mesh).
- `model.scale` is uniform only (the op takes `factor`). Non-uniform maps would need ellipse phase
  and cylinder -> elliptic cylinder handling in the pcurves.
- The `x-entity-kind` annotation of `topo.related.kind` is not applicable (it is a kind, not an id);
  its schema is the full `EntityKind` enum.
- Mirror-deck test uses `Tolerance::with_linear(1e-6)` for `sm_model_api::validate` (see CR-2).
- No criterion benchmark (T1 names none).

## Known gaps

- Stretch primitives (`create_frustum`, `create_torus`, `create_prism`, `create_pyramid`) are not
  implemented.
- Transforms of faces with pcurves on cone, torus, elliptic and implicit surfaces return `Failed`
  (no such face can be created by W2 ops); NURBS pcurves are only carried by an identity parameter
  change.
- `model.measure` bounding boxes of partial patches of curved faces are the boxes of their edges.
- Cost: every transform scans all edges and faces once and the layer clones the host on a shared
  state (known W3 item); not benchmarked.
- Non-meshed-only: meshed volumes are refused (decision W2-2); moving a mesh with its geometry is W3.
- `scale_tolerances` is exercised but there is no test that a merge-grown tolerance override scales.

## api_change_requests (none blocking)

- **CR-1 (`sm-topo-api`, `geometry.rs::sample_face`)**: allow faces that touch poles/apexes. The
  boundary integral `oint H dv` is well defined there because degenerate coedges are skipped and
  contribute `dv = 0`; for the sphere's loop it gives the exact area and volume. Today
  `face_moments`/`volume_moments` refuse any surface with `Domain2::singularities`, so a sphere volume
  has no quadrature in the view. Proposed: drop the early `Unsupported` when every singularity
  lies on a `DEGENERATE` edge of the face. `sm-model::model.measure` then drops its closed-form
  fallback.
- **CR-2 (`sm-topo-api`, `geometry.rs`, `FaceMoments::plane`; symptom in `sm-model-api` `validate`)**: `FaceMoments::plane(planar)` computes
  `(n^T second n - d^2 area) / area`, which cancels catastrophically far from the origin. A 10 x 10
  face at `z = 5` has an rms error of about `1e-7`, above `planar = max(tol.linear, 1e-9 l)`, so a
  correct mirror deck reports `MirrorOffPlane: mirror face is not planar` with the default
  `Tolerance`. Proposed: compute the rms about the area centroid (central second moment), or scale
  `planar` with `|d|`.
- **CR-3 (docs, `sm-topo-api` `TopologyEdit::reverse_face`)**: state that on a face whose only volume
  is the reversed side, `end_op` reverses it back; a mirror needs the sequence in `DESIGN.md`. A
  `TopologyEdit::mirror_face` (reverse loops, keep side tags) would remove the bracket.

- **CR-4 (`sm-op-api`)**: the contract cannot tell "declared no outputs" from "undeclared"
  (`OUTPUTS = &[]` is the default). `model.delete` has nothing to report, so a registry-level check
  that every op declares `OUTPUTS` (plan risk 6, sm-server) would flag it. Proposed: let sm-server's
  check exempt ops whose doc says they produce no ids, or add an explicit marker (for example
  `const NO_OUTPUTS: bool`).

## Notes for the orchestrator

- `Cargo.lock` changed (new `proptest` dev-dependency edge for `sm-model`); it is the only file touched
  outside `crates/sm-model/`.
- Branch: pushed to the session branch `claude/w2-sm-model-wg1gyr` and, as the task asked, to
  `claude/w2-sm-model`.
- `ModelPlugin` needs `CoreLayers` installed first (it reads `ModelLayer`); queries return
  `MissingLayer` otherwise.
