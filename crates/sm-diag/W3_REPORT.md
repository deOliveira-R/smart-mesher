# W3-DIAG report: `sm-quality` FV delta (D1) and `sm-diag` (D2)

Branch: `claude/w3-sm-diag-go88ci` (the session's designated branch; the task text says `claude/w3-sm-diag`).

## Summary

* **D1** (`crates/sm-quality`): `fv.rs` (FV geometry and nine metrics following OpenFOAM `checkMesh` and Fluent), `profile.rs` (`Profile`, `Measure`, `Sense`, `Rule`, `Grade`, `rules`, `Metric::name/from_name`), DESIGN.md section "Finite-volume metrics". Additive only (ruling D0). See `crates/sm-quality/W3_REPORT.md`.
* **D2** (`crates/sm-diag`, L3): `DiagPlugin` (id `diag`) registers queries `mesh.validate` and `mesh.quality` and op `mesh.flag_cells` (role `set`, `Cardinality::Optional`, kind `MeshSet`; stored with `sm_layer_api::add_mesh_set`). It registers no layer. `OpNames` is the service that makes suggestions' `available` follow the op registry. Pure cores: `validate_mesh`, `quality_report`, `flagged_cells`, `build_report`.
* sm-diag depends on no W3a crate, edits no other crate, root manifest, CI or docs, and contains no `unsafe`, no `unwrap` on input paths and no `HashMap`.
* Test commands (all pass, also with `RAYON_NUM_THREADS=1`): `cargo build --workspace`, `cargo test -p sm-quality -p sm-diag --all-features`, `cargo clippy -p sm-quality -p sm-diag --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-quality -p sm-diag --no-deps`. `cargo check --workspace --all-targets` is clean. No Docker step is needed.

## Acceptance bullet to test

| # | Bullet | Test (file) |
|---|---|---|
| 1 | Sheared pair, independent re-derivation | `sheared_pair_matches_analytic_values`, `sheared_pair_matches_an_independent_implementation` (sm-quality `tests/fv.rs`) |
| 2 | Stretched pair | `stretched_pair_matches_analytic_values` (fv.rs) |
| 3 | CellDeterminant | `cell_determinant_matches_analytic_values` (fv.rs) |
| 4 | Face shape | `face_shape_metrics_match_analytic_values` (fv.rs) |
| 5 | Cross-checks | `fv_volumes_equal_cell_volume_on_planar_fixtures`, `names_round_trip`, `boundary_faces_are_nan_for_internal_only_metrics`, `two_dimensional_mesh_is_a_bad_argument` (fv.rs) |
| 6 | `mesh.validate` clean | `every_fixture_validates_clean`, `stub_brick_mesh_run_is_valid_through_the_query` (sm-diag `tests/validate.rs`) |
| 7 | `mesh.validate` on broken meshes | `swapped_left_right_is_reported_at_the_face`, `reversed_loop_is_structural_and_skips_geometry`, `vertex_moved_through_the_opposite_face_gives_an_inverted_cell`, `declassified_vertex_is_reported_with_its_bogus_owner`, `order_is_stable_under_reversal_of_the_input_findings`, `class_retired_and_stale_are_reported` (validate.rs) |
| 8 | `mesh.quality` on the sheared pair | `sheared_pair_passes_openfoam`, `threshold_override_makes_the_pair_fail` (`tests/quality.rs`) |
| 9 | Clusters and suggestions | `hot_edge_ranks_first_with_the_recounted_fraction_and_enrichment`, `availability_follows_the_registry_service`, `smooth_becomes_available_when_the_op_is_registered`, `surface_clusters_suggest_a_scheme_change_or_refinement` (quality.rs) |
| 10 | `mesh.flag_cells` | `flags_exactly_the_failing_cells`, `op_stores_a_live_set_and_is_atomic_and_replayable` (set live, `count_of(MeshSet)` +1, hash changes, duplicate name fails with unchanged hash, `mesh.delete` retires the set, undo/redo), `cold_replay_from_the_root_does_not_diverge`, `empty_selection_warns_and_adds_nothing`, `mesh_defaults_to_the_only_mesh_and_lists_candidates` (`tests/flag.rs`) |
| 11 | JSON stability | `schemas_are_snapshotted`, `quality_reply_is_golden_and_finite`, `validate_reply_is_golden_and_finite`, `registered_by_the_plugin_only` (`tests/golden.rs`, files in `tests/golden/`; bless with `SM_DIAG_BLESS=1`) |
| 12 | Determinism | `quality_and_validate_json_are_bit_identical_across_threads` (20^3 block, 8000 cells, three profiles), `flagged_cells_are_identical_across_threads`, `query_bytes_through_the_engine_are_identical_for_concurrent_callers` (`tests/determinism.rs`); D1: `every_fv_column_is_bit_identical_for_1_to_4_threads_on_all_fixtures`, `perturbed_block_is_bit_identical_across_runs_and_thread_counts`, and the proptest `rigid_motion_and_uniform_scale_leave_every_fv_metric_unchanged` (fv.rs) |
| 13 | Bench (record only) | `benches/quality.rs`, numbers below |

## Measured values

* Analytic checks: every FV value of acceptance 1 to 4 is asserted to 1e-12 absolute (45 degrees, skewness 1 and 2, weight 0.5 and 1/3, orthogonal quality `1/sqrt2` and `1/sqrt5`, volume ratio 2, determinants 1, 0.864, 0.9932 (`108 sqrt3/((12+3 sqrt3)/3)^3`), flatness 0.894427191, warp 26.5650511771 deg, concavity 53.1301023542 and 90 deg).
* Bench, `mesh.quality` core (`quality_report`, generic profile, 8 metrics + validity, nothing bad so no cluster pass) on a 100^3 hex block (1 000 000 cells, 3 030 000 faces), release, 4 cores: **0.87 s** (criterion interval 0.82 to 0.94 s, 10 samples). `validate_mesh` on the same block: **3.8 s** (3.67 to 3.95 s), dominated by `sm_mesh_api::validate`. Recorded, not asserted.
* Hot-spot fixture (3x3x3 block, the two vertices on `TopoEdge#1` moved 0.45 along x, thresholds warn 5 / fail 30 deg on `non_orthogonality`): `TopoEdge#1` ranks first, 2 of 2 touching internal faces bad, enrichment 13.5; suggestion `adjust_intervals` with `current_count = 3`.

## Deviations (with reasons)

1. **Branch name**: `claude/w3-sm-diag-go88ci` (session instruction) instead of `claude/w3-sm-diag`.
2. **Undefined is `null`, but absent over the engine.** The reply types serialize undefined values as `null`; `Engine::run_query` canonicalizes its output and drops `null` entries, so a client sees the key absent (see `threshold: {"fail": 70.0}` in `tests/golden/mesh.quality.reply.json`). This is the engine's codec, not a choice of this crate. The value `1e30` stays a number and no NaN or inf ever appears (tested by scanning the parsed value and the raw text).
3. **Cell `location` fallback**: a cell centroid that is non-finite or outside the bounding box of the cell's vertices (an inverted or collapsed cell) is replaced by the vertex mean. Without it acceptance 7's "location inside the cell's bounding box" fails for tangled cells (centroid = moment / signed volume is unbounded).
4. **Touch set**: a face's touch set is the model entities of its own vertices; a cell's, those of its corners (or its faces' vertices for a polyhedron). The task's "or of its cells" was ambiguous; this is the narrower reading. `n_total` counts entities with a *defined* value touching the model entity (so internal-only metrics are not diluted by boundary faces); `bad` means grade warn or fail. `TopoFace` suggestions use the same thresholds as `TopoEdge` (fraction >= 0.5, enrichment >= 2); the task gives none for faces.
5. **Angles by `atan2`** instead of `acos(clamp(..))` (same value, accurate near 0).
6. **OpenFoam profile**: no `VolumeRatio` rule (the task lists none); `FaceFlatness` warns at 0.8 and has `fail = -inf` (never fails). Wire thresholds report an infinite limit as absent.
7. **`validate` report extras**: `mesh` and `geometry_skipped_reason` fields. `quality.inverted`/`quality.degenerate` are produced only when `geometry` is on and no structural code was found (same gate as `validate_geometry`). `diag.class_retired` is one finding per retired classification entity (located at its first affected entity), not one per affected mesh entity (that would be a million objects for a retired volume).
8. **`count` semantics** of a metric report: all entities of the metric's kind, so `counts + below + above + undefined + n_sentinel == count`. `grade` of a worst entry and the threshold fields are `null` for an ungraded metric; its `sense` is informational (rule's, else a small fixed table).
9. **Registry flag**: `QualityOptions::registry_provided` carries "the `OpNames` service was absent" into the pure core (whose fixed signature only has the `ops` predicate). `OpNames::new` is a convenience constructor.
10. **Dev-dependencies beyond the task's list** (all contract or workspace crates already on main, no W3a crate): `sm-topo-api` (`fixtures::brick`, for the test rig's model entities) and `sm-scheme-api` with `stubs` and `schemars` (the stub `mesh.run` rig, same as sm-fem/sm-mesher tests), plus `criterion` (bench). Normal deps are exactly the task's list; `sm-core` is added with features `serde` and `schemars` (already transitively enabled by `sm-mesh-api`). sm-quality gained no dependency.
11. `Metric::name/from_name` are in `profile.rs` (additive file) rather than `metric.rs`.

## Known gaps

* Stretch items are not done: `grow` and `thresholds` on `mesh.flag_cells`, topology-liveness of classification entities, a testkit `sheared_hex_pair`.
* Deferred to W3b as in the task: dimension-2 shell orientation, equiangle/equivolume skewness, cell aspect ratio in the checkMesh definition, face-pyramid volumes, `mesh.unflag_cells`, `DerivedCache`, per-material profiles, the checkMesh/Fluent conformance harness.
* `quality_report` assumes a structurally valid mesh (every finished mesh is) and may panic on a corrupted one; run `mesh.validate` first on a suspect mesh (which never calls quality code when structural codes exist).
* Generic and Transport thresholds are provisional; the user should ratify Transport before it is advertised. The five FV formulas marked "recalled" in sm-quality's DESIGN.md are for the conformance harness to diff.
* Cluster hot spots rely on classification quality; a surface-dominated hot spot is diluted by interior faces that touch the surface (needs fraction >= 0.5 for a suggestion).

## api_change_requests

None. The frozen contracts were enough (`add_mesh_set`, `MeshLayer`, `QueryCtx::service`, `TypedQuery`, `Registry::op_names`).

## Integration notes for the orchestrator

* sm-server: install `DiagPlugin` after `MeshPlugin` (the plugin, not a `QualityPlugin`) and provide `OpNames(registry.op_names())` as a service; without it every suggestion with an op is `available: false` and says "op registry not provided".
* The integration text still assumes per-violation `class + suggestion` and `QualityPlugin`; it must adopt the shapes in `crates/sm-diag/DESIGN.md`.
* Ruling E6: no `.smsh` bytes, golden `.smsh` files or state hashes of other crates were touched. Adding `DiagPlugin` registers no layer, so no state hash changes.
