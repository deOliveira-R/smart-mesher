# W3 report: sm-scheme-tet

status: done (all mandatory acceptance items 1-11 covered by named tests; item 12 recorded); review round 0 applied (see below)

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

## What changed in review round 0

Must fix:

1. **False `SelfIntersection` on a valid input.** The shipped `schonhardt()` with z scaled by 0.3
   (an affine image of a valid surface) returned `SelfIntersection{2, 6}`. Root cause: after a
   restart, hint points sit on both sides of a constraint triangle and tets on both sides touch
   the segment being recovered (the segment really passes through both, via other tets, without
   crossing the triangle); `edge_by_cavity` treated any constraint face between two cavity tets
   as a crossing. Now `SelfIntersection` needs an exact proof (`exact::seg_tri_improper`,
   `tris_conflict`); an unproven constraint face between cavity tets stays in the cavity as a
   *slit* (both sides cavity boundary; `wrap_valid` accepts the two-sided front face). The other
   sites were audited the same way: corridor faces in `ensure_edge_once` (SoS walk; an unproven
   hit now falls through to the cavity path), the BFS of `recover_face` (`tri_pierce`, SoS; a
   hit on a constraint face needs `tris_conflict`, else the crossing is not followed) and the
   constraint-edge test (`pierces`, SoS; now the exact closed contact). Regression:
   `a13_schonhardt_family::z_scaled_03` plus xy-stretched, z 0.5, twists 45/55 and an affine
   sweep (4 maps x all fixtures); the baseline failed three of these.
2. **Ruling S4 on the recovery path.** New `guard.rs` (grid over the input triangles; score =
   smallest `dist/h_T`, `h_T` = mean edge length of the triangle, 0.3 if the point is outside
   the solid). Recovery candidates are ranked by it (hint selection, `fill_with_steiner`
   candidates, `kernel_point` with margins 0.4 .. 0.05 h and the old fixed 0.05 h replaced),
   smoothing refuses moves that reduce the margin below the bound or below the current value,
   hints on a triangle are dropped, and the zero-volume repair uses the same ranking. Where S4
   is infeasible the best-scoring point is used instead of failing (refusing would turn S4 into
   a recovery failure). Measured (`a14_s4`): Schönhardt 1 Steiner point at 0.154 h (was 0.064 h,
   3 points; no interior point of this solid reaches 0.3 h, the best is 0.18 h), `cube_with_cavity(4)`
   0.346 h (was 0.267 h by the reviewer's measure), `two_cubes_shared(3)` 0.52 h, `tri_cube(4)`
   0.44 h. The 10 x 10 x 0.02 slab (64 diagonal patterns) is thinner than 0.3 h everywhere
   (best possible about 0.002 h): up to 5 Steiner points at >= 0.0006 h, all inside the slab. A
   ruling that S4 applies to refinement only is therefore not needed, but the bound is a
   preference where the geometry forbids it.

Should fix:

1. **Schönhardt variants.** The z-scaled (0.5, 0.3), xy-stretched and twisted (45, 55 degrees)
   fixtures now succeed with one interior point after one restart (0.00 s). For a tiny surface
   component (<= 64 vertices) the first restart hint is the vertex centroid of the component
   (the point a non-star-shaped pocket needs). Not applied to large surfaces: a trial with ring
   patches hurt `noisy_sphere` (see below). Starring the failing *edge* cavity from a kernel
   point is not possible (the segment must stay an edge); the face-cavity star fill already
   existed.
2. **Recovery performance.** Fixture `schonhardt` 6-30 s -> 0.00 s (release), debug 180 s ->
   0 s; twisted/squashed prisms 10-35 s then failure -> 0.00 s and success. `Vec::contains` on
   `rset`/`r`/`tested`/`touching` replaced by epoch marks. Residual: an adversarial input that
   exhausts all twelve restarts is still slow (flattened 3 : 0.2 : 0.7 noisy sphere, about
   1 minute in release, then a located `RecoveryFailed`; the baseline fails on it as well, and
   the affine test skips that single combination).
3. **Size clamp (deviation 6): declined, needs an orchestrator ruling.** The code was not
   changed (making the user size a pure `min` would change the graded-size behaviour that gives
   acceptance 7's adjacent-tet ratio, and the spec text and the gradation guarantee conflict).
   The rustdoc of `SizeSpec::Constant`/`Field` now states the floor explicitly. Question for the
   orchestrator: keep the floor (current), or honour the user size and accept a size jump at the
   frozen boundary?
4. **PartitionCell.** `adapter::partition_cell_pieces_mesh_conformingly_and_own_their_steiner_nodes`
   meshes both pieces of a realized split through the overlay: cells classified on the
   `PartitionCell`s, volumes 0.8 / 1.2, the cut face shared and interior, Steiner nodes owned by
   the cell with `Param::None`. The Driver commit accepts them; no `blocked_on_api`.
5. **Adapter-level shuffle.** `adapter_result_does_not_depend_on_patch_listing_order` (see
   acceptance 11).
6. **Cleanup.** `_unused` helpers removed (`recover.rs`, `exact.rs`); the restart comment now
   refers to `MAX_RESTARTS` (12). `GenerateFailure::fatal`: **declined** on purpose. The spec
   (W3-TET, generate) prescribes `fatal(..).with_nodes(..)`, and a `Geometry` kind lets the
   Driver fall back and drop the nodes (an `Attempt` carries only the message), the located
   nodes being the point of the failure; `tetmesh` is the last-resort scheme anyway.
7. **Debug-profile tests.** Measured on an idle host (debug, all binaries): a11 157 s, a13 203 s
   (new affine sweep), stress 121 s, fuzz 71 s, a08 40 s, the rest under 10 s; a06 is now
   0 s (was 180 s from the recovery restarts). Heavy fixtures are gated with
   `cfg!(debug_assertions)` in `a11`, `a13`, `stress` (`fixture_sweep_quick`) and `fuzz_voxel`
   (release runs the full sets). a08 is unchanged (its floors are calibrated on the large
   meshes).

Found and fixed on the way: a restart batch could contain the same hint twice (duplicate
points make the Delaunay insertion fail with `replace: cavity faces do not match`).

## Acceptance bullet to test

| # | bullet | test(s) |
|---|---|---|
| 1 | exact Delaunay, sign conventions, permutation invariance | `a01_delaunay::{sign_conventions_pinned, delaunay_2000_empty_spheres_volume, delaunay_permutation_invariant}`, unit tests in `delaunay`, `tds`, `flips` |
| 2 | cube, exact boundary and volume | `a02_cube::{cube4_exact_boundary_volume_and_valid_mesh, cube4_without_optimization_is_also_exact}` |
| 3 | curved and non-convex | `a03_curved::{icosphere_level3_volume_boundary, torus_nonconvex_genus1}` |
| 4 | cavity (void shell) | `a04_cavity::cube_with_cavity_volume_and_both_shells` |
| 5 | shared interface conforming | `a05_shared::two_cubes_shared_interface_is_conforming` |
| 6 | hard recovery (Schönhardt, noisy sphere) | `a06_hard::{schonhardt_needs_an_interior_steiner_point, noisy_sphere_keeps_all_triangles}`; `a13_schonhardt_family::{z_scaled_03, z_scaled_05, xy_stretched, twists, affine_images_of_the_fixtures_are_accepted}` (review round 0: affine images of valid surfaces are never `SelfIntersection`); `stress::fixture_sweep_quick` (all surf3 fixtures; `fixture_sweep_full` ignored); `fuzz_voxel::voxel_fuzz_small` (`voxel_fuzz_large` ignored) |
| S4 | interior Steiner points >= 0.3 h from boundary triangles (ruling S4) | refinement: unchanged (`refine.rs`); recovery and smoothing (review round 0): `a14_s4::{schonhardt_steiner_point_is_deep, feasible_fixtures_keep_the_bound, thin_slab_all_diagonal_patterns}` |
| 7 | size control and gradation | `a07_size::{constant_size_counts_and_edge_lengths, graded_field_adjacent_ratio}` |
| 8 | quality floors and `sm_quality` cross-check | `a08_quality::{optimize0_radius_edge_and_unrefinable_floors, optimize2_floors_and_monotone, cross_check_with_sm_quality}` |
| 9 | typed errors with locations | `a09_errors::*` (9 tests), `scheme::tests::recovery_failure_names_the_face_and_carries_the_triangle_keys` |
| 10 | adapter through the `Driver` | `adapter::{boundary_equals_patches, brick10_size2_through_the_driver, brick_with_void_has_both_shells, baffle_and_embedded_members_are_not_applicable, unknown_keys_and_bad_values_are_not_applicable, params_change_the_mesh, failure_names_the_face_and_carries_node_keys, partition_cell_pieces_mesh_conformingly_and_own_their_steiner_nodes}` (the last: a realized plane split, two `PartitionCell` pieces through the overlay, Steiner nodes owned by the cell, the Driver commits them) |
| 11 | determinism (runs, 1 vs N threads, triangle order, adapter listing order) | `a11_determinism::{tetrahedralize_is_bit_identical_across_runs_and_threads, schonhardt_family_is_bit_identical_across_runs_and_threads, delaunay_is_bit_identical, stage_ms_is_excluded_from_equality, triangle_order_does_not_matter}`; `adapter::adapter_result_does_not_depend_on_patch_listing_order` (the surface scheme lists nodes, elements and corners in scrambled order; tets and Steiner nodes equal as node-key sets); also `a12_misc` (scale/translation, disjoint solids, budget, smoke bound) |
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
`RAYON_NUM_THREADS=1` passes (round 0 gate: debug `cargo test -p sm-scheme-tet` 396 s, with `RAYON_NUM_THREADS=1` 487 s, `tools/ci.sh` rc=0).

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
7. **Failure kind**: adapter failures use `GenerateFailure::fatal` (kind `Internal`), as the
   spec prescribes (see review round 0, should-fix 6).
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

* Ruling S4 is a preference where the geometry forbids it (thin parts); the size floor
  (deviation 6) awaits a ruling.
* No S1 (edge removal with vertex collapse) or S2 (`optimize = 3`); `3` is `BadOption`.
* "Delta-level counts" were interpreted as stable counts: same input gives identical counts.
* Large-scale performance is measured on sphere/brick-bounded cases only. A fine interior
  size on a very coarse boundary is limited by the gradation rate (deviation 6) and by the
  Steiner budget.
* Recovery without boundary Steiner points can fail on adversarial inputs; the failure is
  typed and located, never a panic (0 failures in the fuzz, but no proof of completeness).
* Debug-profile tests are still slow (a11 and a13 two to three minutes each on an idle host);
  release is the intended profile.
