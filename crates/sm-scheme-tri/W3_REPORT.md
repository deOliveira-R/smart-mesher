# W3-TRI report: `sm-scheme-tri`

Status: **done** (mandatory scope 1-11 and stretch S1/S2 pass). Docker is not needed by this
task. Branch: `claude/w3-sm-scheme-tri-rt1y0w` (the session's designated branch) and the same
commits on `claude/w3-sm-scheme-tri` (the name in the task text).

## Summary

`trimesh` and `tridelaunay` (one algorithm: exact constrained Delaunay in a power-of-two-normalised
chart, Ruppert/Chew refinement measured in 3-D, smoothing and flips, a quality gate) and
`triadvance` (advancing front, fallback of `trimesh`). `register` adds the three; it never
registers `tri`, a fallback chain or an auto list (the composer sets
`set_fallback(Surface, "trimesh", &["triadvance"])`). The pure 2-D kernel (`cdt`), the chart
(`chart`), the schemes (`scheme`) and the front (`front`) are public and tested separately.
`DESIGN.md` has the positioning, the pole-fan rationale and the guarantee statement.

Two things differ from the plan text and matter for the reader (details under *Deviations*):
a **conformal chart** for surfaces of revolution (without it the full sphere with 24 seam
intervals refined without end) and **pole rings that reuse the seam's first node**.

## Acceptance bullet -> test

All tests are in `crates/sm-scheme-tri/tests/`.

| # | Bullet | Test(s) |
|---|---|---|
| 1 | cocircular 11x11 lattice, 100 collinear boundary points, 2^+-60 scaling, 3 shuffles | `kernel.rs`: `acc1_cocircular_lattice_is_exact_and_order_independent`, `acc1_square_with_a_hundred_collinear_boundary_points`, `acc1_scaled_by_two_to_the_sixty_both_ways`; extra: `prop_random_lattice_subsets_are_exact_and_order_independent`, `point_cloud_without_segments_is_the_delaunay_triangulation_of_the_hull`, `cocircular_points_on_a_circle_with_a_center` |
| 2 | Delaunay (exact, chart) with `optimize = false` on square, annulus, L, cylinder; with `optimize = true` on planar faces | `accept_planar.rs`: `acc2_delaunay_without_optimize_on_square_annulus_l_shape`, `acc2_delaunay_with_optimize_on_planar_faces`; `accept_curved.rs`: `acc2_delaunay_on_the_cylinder_lateral_face_without_optimize` |
| 3 | boundary edges = input segments, Euler, embedded chain / isolated vertex, area | `accept_planar.rs`: `acc3_conformity_euler_and_area_on_the_loop_fixtures`, `acc3_embedded_chain_and_isolated_vertex_are_mesh_vertices_and_edges`, `acc3_embedded_diagonal_between_two_loop_vertices_shares_their_nodes`; kernel level: `kernel.rs::regions_triangulate_with_holes_and_conform`, `embedded_chain_and_isolated_vertex_are_conformed`, `collinear_overlapping_segments_are_split`, `crossing_segments_are_reported` |
| 4 | 20 degrees on square (N in [180,330], `unfixable` empty), disc, annulus, L, cylinder | `accept_planar.rs`: `acc4_unit_square_ten_intervals`, `acc4_twenty_degrees_on_disc_annulus_and_l_shape`, `acc4_analytic_annulus_face_with_circle_edges`; `accept_curved.rs`: `acc4_acc6_cylinder_lateral_face`; kernel level: `refine.rs` |
| 5 | gate: 10x0.1 rectangle fails `Quality` with angle and worst nodes; single attempt without fallbacks | `accept_planar.rs`: `acc5_gate_fails_on_the_thin_rectangle_with_the_worst_triangle_nodes` |
| 6 | curved faces: on-surface, uv area, 3-D area bound, sphere poles/seam/closed, sphere zone | `accept_curved.rs`: `acc4_acc6_cylinder_lateral_face`, `acc6_sphere_zone_twenty_degrees_and_on_surface`, `acc6_full_sphere_poles_seam_and_closed_surface`; extra `scheme_misc.rs`: `spheres_of_many_resolutions_are_closed_valid_and_pole_fans_have_six_triangles`, `torus_and_cone_faces_mesh_validly` |
| 7 | size fields: graded `h(x) = 0.05 + 0.15 x`; 4:1 metric | `accept_size.rs`: `acc7_graded_size_field_counts_and_edge_lengths`, `acc7_anisotropic_metric_stretches_edges_four_to_one` |
| 8 | `Driver` with `stubs::registry()` + `register`; brick surface; two embedded edges | `accept_brick.rs`: `acc8_brick_surface_with_trimesh_is_closed_valid_and_exact`, `acc8_planar_face_with_two_embedded_edges_conforms`, `registration_never_registers_tri_chains_or_auto_lists` |
| 9 | determinism: `determinism::check`, `Sequential` vs `ScopedThreads(4)`, no `HashMap`/`HashSet` | `determinism.rs`: `acc9_pure_kernel_is_deterministic`, `acc9_driver_sequential_vs_scoped_threads_is_bit_identical`, `s2_front_kernel_is_deterministic`, `acc9_no_hash_containers_in_the_sources`; also run under `RAYON_NUM_THREADS=1` |
| 10 | performance, release, `#[ignore]`, record only | `perf.rs`: `acc10_kernel_one_hundred_thousand_triangles`, `acc10_driver_one_hundred_thousand_triangles` |
| 11 | hygiene | `cargo clippy -p sm-scheme-tri --all-targets --all-features -- -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-scheme-tri --no-deps`, no `unwrap`/`expect` in `src/`, `DESIGN.md` |
| S1 | `triadvance`, `front::advance`, `AdvanceOptions` | `front.rs`, `scheme.rs` (`TriScheme::triadvance`, score 20) |
| S2 | front acceptance, iteration cap, fallback chain naming both attempts | `front.rs`: `front_on_the_regions_meets_the_acceptance_numbers`, `s2_front_scheme_on_square_disc_annulus_and_cylinder`, `s2_iteration_cap_is_enforced_and_reversed_loops_return_stuck`, `s2_fallback_chain_reports_both_attempts_on_the_thin_rectangle`, `s2_front_conforms_to_embedded_chains_and_points`, `s2_front_on_random_star_polygons_is_valid_or_stuck`, `s2_front_scheme_on_sphere_and_torus_is_valid_or_a_typed_failure` |

Other tests: unit tests of the conformal coordinate (`chart::tests::psi_is_monotone_with_an_accurate_inverse`:
Mercator `psi`, inverse to 1e-12) and of the closure check; params (`params_defaults_roundtrip_and_validation`, `bad_parameters_are_not_applicable_and_never_panic`),
failure kinds (`error_kinds_map_to_failure_kinds`, `point_budget_is_a_quality_failure`,
`a_sphere_too_coarse_for_pole_rings_is_unsupported_not_a_panic`), carrier senses
(`reversed_carrier_sense_keeps_the_face_counter_clockwise`,
`a_face_without_a_surface_carrier_uses_its_newell_plane`), `cylinders_with_few_or_many_intervals`,
random star polygons through the kernel (`prop_random_star_polygons_with_interior_points_and_chords`,
150 seeds), a needle corner (`needle_corner_terminates_and_reports_unfixable`), a cross-check of the
angle metric and orientation against `sm-quality` (`crosscheck.rs`), `tridelaunay` == `trimesh`.

## Measured values

Debug build, numbers printed by the tests above.

| Item | Value |
|---|---|
| unit square, 10 intervals/side (driver, optimize) | N = 270 (ideal 231), min angle 39.67 deg, `unfixable` = 0. Kernel only, no optimize: N = 258, 37.73 deg |
| disc n = 64 | N = 922, 35.98 deg |
| annulus r 1/3, 48 + 48 points | N = 4260, 33.82 deg |
| annulus r 1/3, 48 (inner) + 96 (outer) points | N = 6658, 33.21 deg, 5 unfixable (boundary spacing 0.13 / 0.20 against h 0.17) |
| analytic annulus (circle edges, 96 + 48 intervals) | N = 2384, 32.68 deg |
| L-shape / rectangle with holes | 36.80 deg (N 876) / 33.96 deg (N 560) |
| cylinder r 1, h 2 (12 around, 6 along) | N = 184, 31.13 deg (3-D), uv area = 2 pi h to 1e-13, A_3D = 12.4975 (true 12.5664), phi_max = 0.5917, per-triangle ratio A_3D / A_uv >= cos(phi_t / 2) holds for every triangle |
| sphere zone u in [0, pi/2], v in +-pi/4 | N = 518, 31.75 deg, on-surface 1e-12 |
| full sphere r 1, 16 seam intervals | N = 908, A = 12.4767 in [12.2629, 12.5664], R_max = 0.1554, min angle 30.88 deg, 6 triangles at each pole |
| sphere, 24 seam intervals | 1105 points, 29.1 deg (the case that diverged before the conformal chart) |
| torus R 3 r 1 (24 x 12) | valid, Euler 0, >= 20 deg (26.7 deg) |
| graded size field | N = 287 = 1.243 x N_ideal (230.9), 99.8 % of interior edges with len/h in [0.5, 1.5], worst 1.633 |
| anisotropic 4:1 | mean abs dx / mean abs dy = 3.578 (N = 272) |
| gate (10 x 0.1 rectangle) | achieved 5.711 deg < 12 deg, `Quality`, 3 nodes of the worst triangle |
| front, square | N = 230 (231), 37.01 deg |
| front, disc / annulus / cylinder | N = 748 (754), 31.97 deg / N = 3390 (3373), 27.93 deg / N = 140, 27.76 deg |
| front, sphere / torus | 804 triangles, 22.18 deg / 354 triangles, 26.74 deg |
| front, 60 random star polygons | 60 of 60 closed, all valid and conforming |
| performance, release, 1 thread, planar square, 117 634 triangles | kernel 0.98-1.06 s (new 0.002, refine 0.21-0.30 with 58 418 Steiner points, optimize 0.75-0.77); through the driver 2.0-2.2 s. Target < 3 s, not gated |

## Deviations (and why)

1. **Pole ring reuses the seam node.** The plan says "a ring of `m = 6` face-owned nodes". The
   fan must contain the seam edge `(pole, s1)`, which is a boundary edge shared with the face
   itself; so the ring is `s1` plus `m - 1 = 5` face-owned nodes at the same latitude, and the
   radius is the seam's first interval instead of "about `h`". Exactly 6 triangles per pole,
   three distinct keys each. Unsupported cases: pole neighbours on different iso-lines, a ring
   that does not close the revolution, and rings that collide (spheres with very few seam
   intervals), all `FailureKind::Unsupported`.
2. **Conformal chart for surfaces of revolution** (added). With the single constant-metric chart
   of the plan the full sphere refined without end for 24 seam intervals (20 000+ points inside a
   thin latitude band, then 2 000 000, with the 3-D aspect of chart-isotropic triangles equal to
   `cos(v)` whatever their size). The chart is now `z = (s (u - u0), psi(v) - psi0)` for a curved
   surface whose parameter lines are orthogonal with `E`, `G` depending on `v` only (sphere,
   cone, torus); planes and cylinders keep the linear (Cholesky) chart, so everything the plan
   asserts about them is unchanged. Other curved faces keep the plan's approximate linear chart.
3. **`unfixable` tolerance.** A rejected triangle is reported as `unfixable` only if its angle is
   below the limit or its circumradius exceeds the size limit by more than 6 %. Reason: a
   boundary triangle with base `h` has `R` up to `h / sqrt(2) = 0.707 h`, above the 0.67 limit,
   with the circumcenter inside the base's diametral circle, which without a Steiner point on
   the segment cannot be split. Without the tolerance the unit square (acceptance 4: `unfixable`
   empty) reports 12 corner triangles.
4. **Minimum-radius guard** `0.02 h`: a triangle that small is never split. It is the
   termination safeguard around input corners that are too sharp for the angle bound.
5. **Encroachment filter tolerance** `1e-9` relative (a circumcenter on the diametral circle does
   not encroach). It only decides whether to try an insertion; validity is always re-checked
   with exact `orient2d`. All structural predicates are exact.
6. **`Geometry2` has two provided methods** beyond the three of the plan: `chart_metric(q)`
   (first fundamental form of the chart, for circumcenters on non-isometric charts) and
   `metric_map(p)` (matrix `A` with `|A e|^2 = e^T M e`, which makes quality metric-based for
   the anisotropic size field). Both default to `None`, so a three-method implementor compiles
   unchanged.
7. **API shape details.** `FaceChart<'a>` carries a lifetime (it borrows the surface);
   `RefineOptions { min_angle, size_factor, max_points }` and `OptimizeOptions { sweeps }` are the
   `o` arguments; `RefineReport { inserted, unfixable, truncated }`; `TriError` has extra variants
   (`BadInput`, `OutOfRange`, `DuplicatePoint`, `Inconsistent`, `Unsupported`, `Degenerate`) and
   `Quality` also carries the worst triangle's point ids; `TriParams::validate` also bounds
   `min_angle <= 34`. `Cdt::new` rejects duplicate points instead of merging them.
8. **Acceptance 6, 3-D cylinder area.** I could not derive a tight closed-form bound for general
   inscribed triangles; the test asserts `A_3D <= 2 pi h` and `A_3D >= 2 pi h cos(phi_max / 2)`
   with `phi_max` the largest angular span of any triangle, and checks per triangle that
   `A_3D / A_uv >= cos(phi_t / 2)` (true on every triangle of the fixture, equality-free; the
   two extreme configurations I checked by hand give `sin(phi/2) / (phi/2) >= cos(phi/2)`). It is
   a verified bound, not a proof for arbitrary triangle shapes.
9. **No rayon.** One face is meshed sequentially (the driver parallelises faces and its
   `ScopedThreads` executor is what the determinism test varies); there is nothing to reduce in
   parallel, so `rayon` is not a dependency.
10. **Dev-dependencies beyond the plan's list**: `sm-model-api` (`Model::new()` for `MeshInput`)
    and `sm-geom-api` (building embedded edges and a reversed plane in tests); both are contract
    crates (decision 11). Normal dependencies: sm-core, sm-math, sm-geom-api, sm-topo-api,
    sm-scheme-api, serde, serde_json, thiserror. `Cargo.lock` gained this crate's dependency list
    (the only change); regenerate it at merge if it conflicts.
11. **Tests with `stubs::registry()`.** No stub name collides with `trimesh`, `tridelaunay` or
    `triadvance`, so nothing had to be removed. Face-only tests use the real curve schemes from
    `sm-scheme-curve`; the brick test uses the stub `equal`.
12. **No ops registered**, so the cold-replay / `Diverged` check does not apply.
13. **`OpenLoop` criterion.** The node list of a loop does not repeat its first node, so a loop
    closes implicitly. A loop is reported open when the implicit closing step is longer than four
    times the longest other step (plus `1e-9` of the uv extent), i.e. when the last coedge does
    not end where the first starts; `GenCtx::loop_uvs` already refuses inconsistent coedge ends
    (returns `None`), which falls back to projection and then to `Lift`/`Unsupported`. Unit test
    in `chart.rs` (`a_closed_loop_passes_and_an_open_one_is_reported`).
14. **`r < 2h` rule replaced.** The plan says any sphere with `r < 2h` is `Unsupported`. That
    rule is not implemented: the ring radius is the seam's first interval (deviation 1), so the
    check is geometric (ring collision or crossing, neighbours not on one iso-line, ring not
    closing), all `Unsupported`. Spheres of 4 to 32 seam intervals mesh; 2 are `Unsupported`.

## Known gaps

* No off-center insertion (Ungor); boundary encroachment is handled only by rejection. Where the
  fixed boundary spacing is incompatible with `h` the face reports an achieved angle or fails the
  gate (that is the documented guarantee).
* The chart is exact only for planes and cylinders; for revolution surfaces it is conformal (angles
  right to first order, circumcenters accurate); other curved faces and anisotropic metrics on
  curved faces use the constant-metric chart and are approximate.
* The full sphere at 16 seam intervals reaches 30.9 degrees; coarse spheres (4-32 intervals) pass
  the default gate (fan triangles measure 55-58 degrees; see *Polish*); spheres with 2 seam
  intervals are `Unsupported`.
* Hemispheres, rim-winding faces and cone apexes without a collapsed coedge are `Unsupported`
  (they need a model-level partition).
* `triadvance` has no angle guarantee; on the 10 x 0.1 rectangle it fails the gate like `trimesh`
  (5.711 degrees, the boundary forces it). Its first closing choice is greedy; a front that no
  front edge can close is `Stuck` (random star polygons: 60 of 60 closed).
* An embedded node that is a seam key (appears twice in a loop) is attached to the copy nearest to
  the chain; no test covers a chain through a seam.
* The `Attempt` the driver stores for a failed scheme keeps only the message; the structured
  `GenerateFailure::nodes` (worst triangle) is visible when calling `Driver::generate` directly
  (as the acceptance 5 test does) but not in the final `NoApplicableScheme` error. A contract
  note, not an ACR.

## Polish

Round r0 after the Opus review of the W3a pass.

1. **Gate on pole fans** (`scheme.rs`, `gate_fans`): the fan triangles `(b, a, pole)` are now
   checked against the same limit and for a positive normal; they were outside the kernel and
   unchecked. Failure is `Quality` with the three nodes. Measured fan minimum angles 55.0 to
   57.8 degrees for 4 to 32 seam intervals.
2. **Deviation 14** declares the `r < 2h` replacement (also in `DESIGN.md`).
3. **`SIZE_SLACK = 1.06` ratified** by the orchestrator; recorded in `DESIGN.md` (Refinement).
4. **Coarse-sphere note refreshed**: the test `spheres_of_many_resolutions_...` now runs at
   default parameters (no `fail_angle` override) for 4, 5, 6, 8, 12, 16, 24, 32 intervals and
   asserts a minimum angle of at least 12 degrees. Known gaps updated.
5. **`hilbert_order`** breaks ties of the quantized index by exact coordinates (`total_cmp`),
   then id.
6. **Cylinder at defaults**: `acc4_cylinder_with_default_params_reaches_twenty_degrees`
   (`optimize = true`, asserts >= 20 degrees).
7. **Scale test**: `acc1_scaled_by_two_to_the_sixty_both_ways` also runs 3 shuffles at each
   scale and compares triangle sets.

Round 1 (review MUST FIX 1): the angle limit `min(fail_angle, 0.9 * smallest corner)` was 0 on
every face with a pole fan, because `geom::corners` skips the degenerate pole coedge and the seam
then turns straight back at the pole (a 0 degree corner). The fan angle gate and the kernel gate
on spheres therefore could never fire. `smallest_corner_deg` now leaves out turn-back corners
(next real coedge on the same edge), so the 12 degree default applies to kernel triangles and
fans. The sphere test now also asserts that a sphere with `fail_angle` above the achieved fan
angle fails with `Quality`.

Declined: none. (The property-style tests use their own fixed xorshift seeds, not
`sm_testkit::strategies::config`, because they are not proptest strategies; unchanged.)

## api_change_requests

None.