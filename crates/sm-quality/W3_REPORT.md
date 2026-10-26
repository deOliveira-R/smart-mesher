# sm-quality W3 report (D1: finite-volume delta)

Owner: the W3-DIAG worker (decision 12 one-crate exception: `crates/sm-quality/` and `crates/sm-diag/`). The full report, with the acceptance table, is in `crates/sm-diag/W3_REPORT.md`; this file covers the sm-quality part.

## Summary

Strictly additive (ruling D0): new files `src/fv.rs` and `src/profile.rs`, new `pub use` lines in `lib.rs`, a "Finite-volume metrics" section in `DESIGN.md`, `tests/fv.rs`. No existing item was renamed or re-signed, no metric value changed, `Cargo.toml` is untouched (no new dependency). `Metric::name()/from_name()` live in `profile.rs`.

New public items: `FvGeometry`, `fv_geometry`, `FvMetric` (+ `ALL`, `entity`, `name`, `from_name`, `unit`, `internal_only`), `FvEntity`, `FvColumn`, `fv_compute`, `face_to_cell`, `Reduce`, `Profile`, `Measure`, `Sense`, `Rule`, `Grade`, `rules`, `Metric::name`, `Metric::from_name`.

## Acceptance (D1 part) to tests (`tests/fv.rs`)

| Acceptance | Test |
|---|---|
| 1 sheared pair (45 deg, skew 1, weight 0.5, OQ 1/sqrt2 and 1/sqrt5, boundary skew 2, independent re-derivation) | `sheared_pair_matches_analytic_values`, `sheared_pair_matches_an_independent_implementation` |
| 2 stretched pair | `stretched_pair_matches_analytic_values` |
| 3 cell determinant | `cell_determinant_matches_analytic_values` |
| 4 saddle, dart, L-hexagon | `face_shape_metrics_match_analytic_values` |
| 5 cross-checks | `fv_volumes_equal_cell_volume_on_planar_fixtures`, `names_round_trip`, `boundary_faces_are_nan_for_internal_only_metrics`, `two_dimensional_mesh_is_a_bad_argument`, `geometry_of_the_wrong_mesh_is_rejected`, `face_to_cell_reduces_over_faces_and_skips_nan`, `rules_grade_with_equality_passing` |
| 12 (D1 half) determinism 1/2/3/4 threads on all fixtures; rigid motion and scale property | `every_fv_column_is_bit_identical_for_1_to_4_threads_on_all_fixtures`, `perturbed_block_is_bit_identical_across_runs_and_thread_counts`, `rigid_motion_and_uniform_scale_leave_every_fv_metric_unchanged` (proptest, testkit `proper_transform`, pinned seed, 24 cases over 12 meshes) |

All analytic values are asserted to 1e-12 absolute (rigid-motion property: 1e-9 relative with a 1e-9 absolute floor).

## Deviations and notes

* Angles use `atan2(|a x b|, a . b)` instead of `acos(clamp(...))` (identical mathematically; `acos` loses about 8 digits near 0, which would make the near-zero non-orthogonality of a regular mesh noisy under rigid motions).
* Degenerate guards follow OpenFOAM's `vSmall`: `NonOrthogonality` of a face with `|d||Sf| = 0` is 90, `FaceWarp` of a zero-area face is 90, `VolumeRatio` is the 1e30 sentinel when the smaller volume is not positive.
* The OpenFoam profile has no `VolumeRatio` rule (the task lists none); `FaceFlatness` there has `warn 0.8` and `fail = -inf`, meaning it never fails (checkMesh only warns). Generic and Transport carry `VolumeRatio` rules.
* Formulas marked "recalled, to be diffed against checkMesh by the conformance harness" in DESIGN.md: skewness limits and boundary variant, flatness fan apex, concavity asin form, determinant normalisation, volume-ratio convention. No external tool was run.
* No `.smsh` bytes, golden files or state hashes of any other crate were touched (ruling E6 does not apply).
