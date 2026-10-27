# W3 report: sm-scheme-tet

status: done with one open item (all mandatory acceptance items 1-11 covered by named tests; item 12 recorded); review rounds 0 and 1 applied (see below). Open: recovery still fails, with a typed `RecoveryFailed`, on a family of valid height-field surfaces (see round 1, must fix 1).

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

## What changed in review round 1

Must fix:

1. **Recovery rejected valid closed surfaces (decision 12).** Case by case, with the review's
   reproductions as tests (`a15_recovery_robust`):
   * (a) height-field box `tri_cube(4)` with relief at z = 1 (seed 5, depth 0.9): was
     `RecoveryFailed{176, [22, 95]}` after 10 s, now meshes in 0.2 s with 23 interior points.
   * (b) two-layer Schönhardt tower: was `RecoveryFailed{6, [2, 3]}`, now 0.00 s, 1 interior point
     (Gmsh: 1).
   * (c) five-ring tower: was `RecoveryFailed{2, [0, 1]}` after 38 s, now 0.01 s, 5 interior
     points (Gmsh: 2).
   * Added families: relief on 4 x 4 for seeds 1..6 and depths 0.3 / 0.6 / 0.9; 6 x 6 and
     8 x 8 at moderate depth; alternating towers (3 and 4 layers, twists 25 / 35 / 45).
     Each test checks the boundary exactly, positive exact orientation, that the input points
     are untouched, and the volume against the divergence-theorem volume of the input to 1e-10
     relative. For comparison, the round 0 code fails 6 x 6 relief at depth 0.6 (seed 2) and
     all four seeds of 8 x 8 at depth 0.6.
   What was wrong, and what changed in `recover.rs` / `fill.rs`:
   * An edge cavity that cannot be refilled has two sides (solid and exterior). The old search
     added at most one interior point and chose it from a few face offsets; it never put a point
     in both wedges and it never used the segment. `fill_with_steiner` now takes *seeds* from the
     caller (points next to the endpoints inside the cone of the incident triangles' inner and
     outer sides, wedge bisectors on both sides, a pocket centroid for tiny components), tries
     one inside + one exterior point, and finishes with *dead-face repair* (add a point behind
     the front face that no vertex can close, retry, up to four times).
   * Interior points added by a restart that a later cavity encloses used to make the refill
     fail ("a vertex is enclosed by the cavity"). They are now droppable (`Dt::droppable_from`).
   * The refill search is fail-first and reports the front faces that invalidated candidate
     tets (*blockers*); the cavity absorbs the tets beyond them instead of growing by uniform
     layers.
   * The two "cheap" attempts are gone. They failed fast, but the hints of a cheap pass enlarged
     the point set and made the later cavities harder (with them, the four-layer alternating
     tower needed many restarts and failed in one configuration; the final code recovers it
     with 9 interior points, which is more than the ideal 2). `MAX_RESTARTS` is now 2 (was 12); twelve full restarts, each repeating the
     expensive failing searches, were part of the 10 s and 38 s spent before failing on
     small inputs.
   * `edge_hints` no longer scans every constraint face per failing edge (should fix 4):
     `Constraints::tris_of_edge` / `tris_of_vertex` are sorted adjacency lookups.
   **Not fixed (honest limit).** Relief on a cube face at depth 0.9 on grids 6 x 6 and finer
   (seeds 1-3 of the sweep), and depth 0.6 on 8 x 8 (3 of 4 seeds), still end in a typed, located
   `RecoveryFailed` after 15-20 s (release). Where it gets stuck (seed 1, 6 x 6, depth 0.9): a
   rim vertex at z = 1 next to a pit vertex of the first interior column (z = 0.22) makes a
   sliver of about 12 degrees between the side wall and the steep surface triangle, and for the
   edge from the rim vertex to the pit the exact fill finds a wall triangle with no valid apex
   in every cavity tried (up to 324 tets; the search is budgeted, so this is "none found", not
   a proof of non-existence); interior points placed behind that triangle do not help, and six
   random relabellings of the points (a different SoS tie-breaking) do not help either. This
   was not compared against Gmsh. A complete method needs either a different decomposition of
   the pit/rim region or boundary Steiner points, which decision 12 rules out; this is the open
   item for the next round. `a15::relief_known_hard_fail_typed` (ignored, release) records that these cases fail
   cleanly. Failing inputs still cost 15-20 s (the first three failing edges per pass get the
   full search, then only a plain refill); that is the remaining performance red flag.

Should fix:

1. **One `h` for ruling S4.** `h_T` (mean edge length of the input triangle) is now used by
   the guard, by refinement and by the test (`DESIGN.md`, "One definition of h"). Refinement
   checks `Guard::margin(p) >= 0.3` against every input triangle, not only the faces of its
   cavity. `a14_s4::feasible_fixtures_keep_the_bound` covers `cube_with_cavity(3)` too: 0.302 h
   (round 0: 0.267 h), `cube_with_cavity(4)` 0.347 h.
2. **Calibration tables refreshed** from this commit (a08, release): `icosphere(1,4)` 25.1 /
   137.9 / 0.558 / 0.871, 0 unrefinable of 27 619 tets at optimize 0; `tri_torus` 21.2 / 137.4 /
   0.543 / 0.832, 0 of 7 503; `tri_cube(10)` unchanged. The floors and guards pass.
3. **Error locations.** An out-of-range vertex index is `TetError::BadIndex{tri, index}`
   (new variant; the adapter maps it to a located failure) and an empty triangle list is
   `BadOption`; neither names a triangle that does not exist or calls it degenerate
   (`a09_errors::degenerate_triangles_and_bad_indices`).
4. **Restart hint generation** uses the adjacency lookups (see must fix 1).
5. **Fewer, deeper points.** Candidates inside the solid are ranked by S4 score before use, the
   seeds come first among equals, and a tiny component gets its centroid first. The twisted
   prisms of 5 and 10 degrees now use one point at 0.289 h and 0.263 h (round 0: three points
   at 0.14 and 0.125 h; the reviewer's estimate of 0.29 h for one point near the centroid
   holds); Schönhardt keeps one point at 0.154 h (`a13::small_twists_use_few_deep_points`,
   `a14::schonhardt_steiner_point_is_deep`). The thin-slab patterns stay at 0.0008 h worst
   (best possible 0.002 h).

## Acceptance bullet to test

| # | bullet | test(s) |
|---|---|---|
| 1 | exact Delaunay, sign conventions, permutation invariance | `a01_delaunay::{sign_conventions_pinned, delaunay_2000_empty_spheres_volume, delaunay_permutation_invariant}`, unit tests in `delaunay`, `tds`, `flips` |
| 2 | cube, exact boundary and volume | `a02_cube::{cube4_exact_boundary_volume_and_valid_mesh, cube4_without_optimization_is_also_exact}` |
| 3 | curved and non-convex | `a03_curved::{icosphere_level3_volume_boundary, torus_nonconvex_genus1}` |
| 4 | cavity (void shell) | `a04_cavity::cube_with_cavity_volume_and_both_shells` |
| 5 | shared interface conforming | `a05_shared::two_cubes_shared_interface_is_conforming` |
| 6 | hard recovery (Schönhardt, noisy sphere) | `a06_hard::{schonhardt_needs_an_interior_steiner_point, noisy_sphere_keeps_all_triangles}`; `a13_schonhardt_family::{z_scaled_03, z_scaled_05, xy_stretched, twists, affine_images_of_the_fixtures_are_accepted}` (review round 0: affine images of valid surfaces are never `SelfIntersection`); `stress::fixture_sweep_quick` (all surf3 fixtures; `fixture_sweep_full` ignored); `fuzz_voxel::voxel_fuzz_small` (`voxel_fuzz_large` ignored) |
| S4 | interior Steiner points >= 0.3 h_T from every input triangle (ruling S4) | refinement (round 1: `Guard::margin` against every input triangle), recovery and smoothing: `a14_s4::{schonhardt_steiner_point_is_deep, feasible_fixtures_keep_the_bound (incl. cube_with_cavity(3)), thin_slab_all_diagonal_patterns}`, `a13_schonhardt_family::small_twists_use_few_deep_points` |
| 6b | recovery of valid surfaces that need interior points (decision 12; review round 1) | `a15_recovery_robust::{relief_cube_seed5_depth09_n4, relief_family_n4, relief_family_n6_n8_moderate, two_layer_schonhardt_tower, five_ring_tower, alternating_towers}`; `relief_known_hard_fail_typed` (ignored) records the open limit |
| 7 | size control and gradation | `a07_size::{constant_size_counts_and_edge_lengths, graded_field_adjacent_ratio}` |
| 8 | quality floors and `sm_quality` cross-check | `a08_quality::{optimize0_radius_edge_and_unrefinable_floors, optimize2_floors_and_monotone, cross_check_with_sm_quality}` |
| 9 | typed errors with locations | `a09_errors::*` (9 tests; bad index and empty input added in round 1), `scheme::tests::recovery_failure_names_the_face_and_carries_the_triangle_keys` |
| 10 | adapter through the `Driver` | `adapter::{boundary_equals_patches, brick10_size2_through_the_driver, brick_with_void_has_both_shells, baffle_and_embedded_members_are_not_applicable, unknown_keys_and_bad_values_are_not_applicable, params_change_the_mesh, failure_names_the_face_and_carries_node_keys, partition_cell_pieces_mesh_conformingly_and_own_their_steiner_nodes}` (the last: a realized plane split, two `PartitionCell` pieces through the overlay, Steiner nodes owned by the cell, the Driver commits them) |
| 11 | determinism (runs, 1 vs N threads, triangle order, adapter listing order) | `a11_determinism::{tetrahedralize_is_bit_identical_across_runs_and_threads, schonhardt_family_is_bit_identical_across_runs_and_threads, delaunay_is_bit_identical, stage_ms_is_excluded_from_equality, triangle_order_does_not_matter}`; `adapter::adapter_result_does_not_depend_on_patch_listing_order` (the surface scheme lists nodes, elements and corners in scrambled order; tets and Steiner nodes equal as node-key sets); also `a12_misc` (scale/translation, disjoint solids, budget, smoke bound) |
| 12 | performance record (no gate) | `perf::{record_delaunay_200k, record_ball_pipeline, record_ball_small}`, `adapter::record_adapter_perf` (all `#[ignore]`), `benches/tet.rs` |

## Measured values (release, 4 vCPU)

Quality (default size, optimize 2; floors are the task's, guards are tighter):

| fixture | min dihedral | max dihedral | min mean ratio | mean mean ratio |
|---|---|---|---|---|
| `tri_cube(10)` | 27.8 | 135.6 | 0.621 | 0.872 |
| `icosphere(1,4)` | 25.1 | 137.9 | 0.558 | 0.871 |
| `tri_torus` | 21.2 | 137.4 | 0.543 | 0.832 |

At `optimize = 0` the fraction with R/l <= 2 is 1.0000 / 1.0000 / 0.9988 and unrefinable tets
are 0 / 0 / 0 (of 5 101 / 27 619 / 7 503 tets). `sm_quality` agrees to 1e-15 (mean ratio, radius-edge) and 6e-14 deg (dihedral).

Performance (record only): Delaunay 200k points 3.9 s (51k points/s); 1.14M-tet ball in 25.4 s
(44.9k tets/s; stage ms 8 / 1512 / 3025 / 10084 / 10478); adapter, 1.72M tets through the
`Driver` including serial commit, 53.7 s (32k tets/s). Details in `DESIGN.md`.

Robustness: all surf3 fixtures recover; 122 random planar voxel shapes (CAD-like, coplanar
facets) all succeed (round 0 numbers: 0 failures, mean 0.19 s, worst 9.5 s). That fuzz covers
planar facets only. Round 1 adds the non-planar families of `a15` (relief boxes, stacked and
alternating twisted prisms); see the open limit above for the part of the relief family that
still fails.

Test runs (round 1): release `cargo test -p sm-scheme-tet --all-features` passes in under 10 s per
binary (a15 0.5 s, stress 4.4 s; `--ignored`: `fixture_sweep_full` 16 s, `voxel_fuzz_large` 13 s,
`relief_known_hard_fail_typed` 44 s). Debug on a busy host (load average 23): a08 77 s, a11 212 s,
fuzz 67 s, a13 1.5 s (was 203 s in round 0: the affine sweep no longer runs the twelve restarts),
a15 5.6 s, the rest under 10 s; the stress sweep took 743 s in that run because of a slow edge
cavity fill in `cube_with_cavity(1)` (43 s, found with the per-fixture timings now printed by
`stress.rs`; fixed by limiting the pair and dead-face searches to edge cavities: 1.8 s).
Earlier rounds: release and debug `cargo test -p sm-scheme-tet --all-features` pass; debug with
`RAYON_NUM_THREADS=1` passes (round 0 gate: debug `cargo test -p sm-scheme-tet` 396 s, with `RAYON_NUM_THREADS=1` 487 s, `tools/ci.sh` rc=0).

## Deviations from the task text

1. **Eight auxiliary far points** in `tetrahedralize` (the public `delaunay()` keeps the exact
   hull). Reason and effect: `DESIGN.md`. The output never contains them.
2. **Restart with hints.** What the in-situ search cannot repair is collected, turned into
   interior hint points (inside and, for reflex edges, outside the solid) and the build restarts,
   at most twice (round 1: was twelve restarts, the first two deliberately cheap).
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
* Recovery without boundary Steiner points can fail on valid inputs; the failure is typed and
  located, never a panic. Known failing family: height-field relief with deep pits next to the
  rim on 6 x 6 and finer grids (round 1, must fix 1). There is no proof of completeness and the
  fuzz covers planar facets only.
* Debug-profile tests are still slow (a11 and a13 two to three minutes each on an idle host);
  release is the intended profile.
