# W3 report: sm-scheme-tet

status: done (all mandatory acceptance items 1-11 covered by named tests; item 12 recorded)

`api_change_requests`: none. No contract crate, no other crate, no root file was edited
(`Cargo.lock` gained the new crate's entries only).

## What exists

* **Scope A (pure core)**: `SurfaceInput`, `TetOptions`, `SizeSpec`, `TetMesh`, `TetStats`,
  `TetError`, `tetrahedralize`, `delaunay`, plus `tet_quality` / `TetQ`. Boundary points and
  triangles are frozen; only interior Steiner points are added; output is canonical
  (tets rotated and sorted, unused Steiner points dropped) and bit-identical across runs and
  thread counts.
* **Scope B (adapter)**: `TetVolume: Scheme` (`"tetmesh"`, Volume, `Caps::TET`, requires
  `Caps::TRI`, score 10), `TetParams{size, radius_edge, optimize, gradation}`, `register`.
  Core errors become `GenerateFailure::fatal(..).with_nodes(..)` naming the `TopoFace` and
  a suggested action. Steiner nodes are owned by the piece (`Param::None`).
* `DESIGN.md` documents the algorithm, the deviations and the calibration/perf records.

## Acceptance bullet to test

| # | bullet | test(s) |
|---|---|---|
| 1 | exact Delaunay, sign conventions, permutation invariance | `a01_delaunay::{sign_conventions_pinned, delaunay_2000_empty_spheres_volume, delaunay_permutation_invariant}`, unit tests in `delaunay`, `tds`, `flips` |
| 2 | cube, exact boundary and volume | `a02_cube::{cube4_exact_boundary_volume_and_valid_mesh, cube4_without_optimization_is_also_exact}` |
| 3 | curved and non-convex | `a03_curved::{icosphere_level3_volume_boundary, torus_nonconvex_genus1}` |
| 4 | cavity (void shell) | `a04_cavity::cube_with_cavity_volume_and_both_shells` |
| 5 | shared interface conforming | `a05_shared::two_cubes_shared_interface_is_conforming` |
| 6 | hard recovery (Schönhardt, noisy sphere) | `a06_hard::{schonhardt_needs_an_interior_steiner_point, noisy_sphere_keeps_all_triangles}`; `stress::fixture_sweep_quick` (all surf3 fixtures; `fixture_sweep_full` ignored); `fuzz_voxel::voxel_fuzz_small` (`voxel_fuzz_large` ignored) |
| 7 | size control and gradation | `a07_size::{constant_size_counts_and_edge_lengths, graded_field_adjacent_ratio}` |
| 8 | quality floors and `sm_quality` cross-check | `a08_quality::{optimize0_radius_edge_and_unrefinable_floors, optimize2_floors_and_monotone, cross_check_with_sm_quality}` |
| 9 | typed errors with locations | `a09_errors::*` (9 tests), `scheme::tests::recovery_failure_names_the_face_and_carries_the_triangle_keys` |
| 10 | adapter through the `Driver` | `adapter::{boundary_equals_patches, brick10_size2_through_the_driver, brick_with_void_has_both_shells, baffle_and_embedded_members_are_not_applicable, unknown_keys_and_bad_values_are_not_applicable, params_change_the_mesh, failure_names_the_face_and_carries_node_keys}` |
| 11 | determinism (runs, 1 vs N threads, triangle order) | `a11_determinism::{tetrahedralize_is_bit_identical_across_runs_and_threads, delaunay_is_bit_identical, stage_ms_is_excluded_from_equality, triangle_order_does_not_matter}`; also `a12_misc` (scale/translation, disjoint solids, budget, smoke bound) |
| 12 | performance record (no gate) | `perf::{record_delaunay_200k, record_ball_pipeline, record_ball_small}`, `adapter::record_adapter_perf` (all `#[ignore]`), `benches/tet.rs` |

## Measured values (release, 4 vCPU)

Quality (default size, optimize 2; floors are the task's, guards are tighter):

| fixture | min dihedral | max dihedral | min mean ratio | mean mean ratio |
|---|---|---|---|---|
| `tri_cube(10)` | 27.8 | 135.6 | 0.621 | 0.872 |
| `icosphere(1,4)` | 24.3 | 139.5 | 0.593 | 0.870 |
| `tri_torus` | 20.1 | 139.8 | 0.562 | 0.831 |

At `optimize = 0` the fraction with R/l <= 2 is 1.0000 / 1.0000 / 0.9988 and unrefinable tets
are 0 / 2 / 0. `sm_quality` agrees to 1e-15 (mean ratio, radius-edge) and 6e-14 deg (dihedral).

Performance (record only): Delaunay 200k points 3.9 s (51k points/s); 1.14M-tet ball in 25.4 s
(44.9k tets/s; stage ms 8 / 1512 / 3025 / 10084 / 10478); adapter, 1.72M tets through the
`Driver` including serial commit, 53.7 s (32k tets/s). Details in `DESIGN.md`.

Robustness: all surf3 fixtures recover; 122 random planar voxel shapes (CAD-like, coplanar
facets) all succeed, 0 failures, mean 0.19 s, worst 9.5 s.

Test runs: release and debug `cargo test -p sm-scheme-tet --all-features` pass; debug with
`RAYON_NUM_THREADS=1` passes (the determinism file alone takes ~6 min in debug).

## Deviations from the task text

1. **Eight auxiliary far points** in `tetrahedralize` (the public `delaunay()` keeps the exact
   hull). Reason and effect: `DESIGN.md`. The output never contains them.
2. **Restart with hints.** Recovery failures are collected, turned into interior hint points
   (inside and, for reflex edges, outside the solid) and the build restarts, at most twelve
   times; the first two attempts are deliberately cheap. This trades a few extra interior
   points for speed on planar inputs (a 3 s case became 30 ms with 17 instead of 5 points).
3. **Zero-volume tets** of the perturbed Delaunay are removed (flip, cavity refill, or
   kernel Steiner point), before recovery and after classification.
4. **Delaunay flips after classification**, and a first refinement insertion with unbounded
   cavity. Without them refinement of cospherical boundaries is orders of magnitude slower.
5. **Pre-refinement `flip_optimize(1e-3)`** runs even at `optimize = 0`, so `optimize = 0` is
   "no quality optimization after refinement", not "no flips ever".
6. **Size clamp**: a user size below `max_c(h_c - (g-1)|p-c|)` is raised to that bound
   (symmetric gradation); a tet is never asked to be smaller than 1.2x its largest boundary
   triangle's circumradius. A user size far below the boundary size is therefore reached only
   at the gradation rate away from the boundary.
7. **Failure kind**: adapter failures use `GenerateFailure::fatal` (kind `Internal`).
8. **A crossing constraint triangle** is reported as `SelfIntersection`, not as the literal
   `RecoveryFailed` of the brief; `RecoveryFailed` is reserved for exhausted recovery. The
   adapter's `explain` maps both to the same actionable message (unit-tested).
9. **`delaunay()` drops flat (zero-volume) tets** of the exact hull; the volume equals the hull
   volume.
10. **`stage_ms`** is `[validate, delaunay, recover+classify+flats, refine, optimize]` and is
    excluded from equality and hashes.
11. **Steiner budget**: core default `8 n + 100_000`; the adapter raises it from a volume
    estimate when a constant `size` parameter is given (not for a context size field; the adapter does not expose `max_steiner`).

## Known gaps

* `Target::PartitionCell` is not exercised by any test (the adapter accepts only `Volume`
  targets with closed shells in the fixtures used).
* No S1 (edge removal with vertex collapse) or S2 (`optimize = 3`); `3` is `BadOption`.
* "Delta-level counts" were interpreted as stable counts: same input gives identical counts.
* Large-scale performance is measured on sphere/brick-bounded cases only. A fine interior
  size on a very coarse boundary is limited by the gradation rate (deviation 6) and by the
  Steiner budget.
* Recovery without boundary Steiner points can fail on adversarial inputs; the failure is
  typed and located, never a panic (0 failures in the fuzz, but no proof of completeness).
* Debug-profile tests are slow (a11 ~6 min single-threaded); release is the intended profile.
