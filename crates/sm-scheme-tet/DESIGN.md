# sm-scheme-tet: tetrahedral volume mesher

Layer L3. Crate `sm-scheme-tet` turns a **closed, oriented triangle surface** into a conforming
tetrahedral mesh with exact predicates, and registers the scheme `tetmesh` with the scheme
framework (`sm_scheme_tet::register`). It registers no ops, queries, layers or formats.

## Positioning

Similar to **TetGen** (Delaunay insertion, constrained boundary recovery, Delaunay refinement,
flip/smooth optimization) and to the Delaunay mode of **Gmsh/HXT** (the comparator of the W3
milestone), it differs in four ways, giving a mesher that never disturbs a surface that other
meshes share:

1. **The boundary is frozen.** TetGen splits boundary facets and edges with Steiner points, HXT
   and fTetWild modify or approximate the surface. We add only *interior* points
   (decision 12), so every neighbouring volume and the triangle patches of the faces stay
   conforming by construction. The price is that recovery can fail on bad surfaces; it then fails
   with a *located, actionable* error (triangle, edge, `TopoFace`, node keys).
2. **Exact predicates with symbolic perturbation.** `sm_math::orient3d/insphere` with
   `sos::{orient3d_sos, insphere_sos}` (perturbation id = point index) make the Delaunay
   tetrahedralization of a point set unique and independent of the insertion order; TetGen uses
   Shewchuk predicates with a bounding tetrahedron and ad-hoc degeneracy handling.
3. **Bit-identical output for any thread count.** Insertion and recovery are sequential;
   rayon is used for quality evaluation, queue seeding, colour-class smoothing and (via
   `sm_math::sum`) reductions, all with fixed-order combination. HXT is faster on many cores but
   not bit reproducible.
4. **Driven by the scheme framework.** Sizes come from the `GenCtx` size field, node keys and
   orientation from the driver, failures are `GenerateFailure`s with node keys for MCP/CLI
   diagnostics.

Against **fTetWild**: fTetWild accepts non-manifold, self-intersecting garbage and returns an
approximation of it; we accept only valid closed surfaces and return the exact surface.
Throughput is TetGen class and single-thread (see "Performance record"); no claim is made against
parallel HXT.

## Contract

```rust
pub fn tetrahedralize(&SurfaceInput, &TetOptions) -> Result<TetMesh, TetError>;
pub fn delaunay(&[[f64; 3]]) -> Result<TetMesh, TetError>;   // convex hull, stage 2 alone
pub struct TetVolume; pub fn register(&mut SchemeRegistry) -> sm_scheme_api::Result<()>;
```

* **Boundary-preservation decision.** Boundary points and triangles are never moved, split or
  added. The first `n_input` output points are bit-identical to the input; every input triangle
  is a face of exactly one output tet (tested). Interior Steiner points are the only additions.
* **Shell and orientation convention.** `SurfaceInput` triangles are wound so the right-hand
  normal points *out of the region to mesh*. Shells are the edge-connected components; the outer
  shell encloses positive volume, cavity shells negative (their normals point into the cavity).
  In the adapter, orientation is read from the topology and never inferred from geometry: a face
  use with `Side::Left` (volume on the left, normal out of the volume) keeps the winding of the
  face patch's triangles, `Side::Right` flips it; void shells follow the same rule.
* **Sign conventions** (pinned by tests against `sm_testkit::meshes::single_tet` and
  `sm_mesh_api::cell_volume`): `ori(a,b,c,d) = -sm_math::orient3d(a,b,c,d)` is `+1` for a
  positive-volume `CellShape::Tet`, i.e. the sign of `det(b-a, c-a, d-a)`. For such a tet
  `sm_math::insphere` is *negative* inside; `in_sphere` hides that. Outward face opposite slot
  `i` is `FACE[i] = [[1,2,3],[0,3,2],[0,1,3],[0,2,1]]`.
* **Determinism.** SoS ids are dense point indices (adapter: ascending `NodeKey`; Steiner points
  `n_input + 8 + k` by creation order, never insertion position). Triangles are canonically
  rotated and sorted before use, so the result does not depend on their listing order. `HashMap`
  is not used at all. Output is canonical: unused Steiner points dropped keeping creation order,
  every tet rotated to the lexicographically smallest even permutation, tets sorted.
  `delaunay()` uses lexicographic coordinate ranks as ids so a permuted input gives the same
  tet set.
* **Errors** (`TetError`, `#[non_exhaustive]`, all carry indices; the crate never panics on
  input): `NotClosed`, `NonManifold`, `DuplicatePoint`, `InvertedShell`, `DegenerateTri`,
  `RecoveryFailed{tri, edge, reason}`, `SelfIntersection`, `SteinerBudget`, `BadOption`,
  `OutOfRange`, `Internal`.

## Pipeline

| # | stage | module | notes |
|---|---|---|---|
| 1 | validate | `validate` | options; finite coordinates within `[2^-100, 2^100]`; duplicate points; degenerate triangles (exact collinearity); every undirected edge shared by exactly two oppositely wound triangles; shell volumes (`InvertedShell` if the total enclosed volume is not positive) |
| 2 | Delaunay | `delaunay`, `tds` | Bowyer-Watson with a ghost vertex, SoS predicates, BRIO rounds (seed from `n`) with a Hilbert key, stochastic visibility walk from the last tet, SoA `Tds` (`verts`, `nbr`, free list). One primitive, `Tds::replace(old, new)`, glues faces by sorted keys and validates face orientation before mutating |
| 3 | recovery | `recover`, `flips`, `fill`, `exact` | see below |
| 4 | classify | `pipeline` | flood fill from the ghost tets toggling at constraint triangles; the side of a tet is read from the *orientation* of its constraint face, contradictions give `InvertedShell`; outside tets dropped; then zero-volume tets removed |
| 5 | refine | `refine` | constrained-cavity circumcenter insertion |
| 6 | optimize | `optimize` | level 0 none, 1 flips, 2 flips + smoothing |
| 7 | canonical output | `canon` | as above |

### Deviation from the task text: auxiliary far points

The task asks for a ghost-vertex Delaunay "with no bounding super-tet, so the hull is exact".
`delaunay()` does exactly that. `tetrahedralize` instead adds **eight auxiliary far points**
(a jittered box 20 spans away) around the input before stage 2. Reason: a surface made of
planar facets (every CAD model) has coplanar boundary points on the *convex hull*; with an exact
hull the hull diagonals chosen by the perturbation differ from the surface triangles and fixing
them needs special hull-edge flips with open rings and ghost bookkeeping. With the far box all
such edges are ordinary interior edges (ring flips, e.g. 4-4), and the ghost vertex only closes
the far box. The far tets are outside the surface, so classification drops them; the output
never contains a far point. This costs nothing in output exactness.

### Boundary recovery (interior Steiner points only)

Everything below works on the triangulation including the far box.

1. **Edges.** For each missing surface edge, trace the segment through the tets it crosses
   (generic position via SoS). Candidate repairs are n-to-m *edge removals* of the edges of the
   crossed faces (Klincsek DP over the ring polygon maximizing the minimum mean ratio, every new
   tet needing strictly positive **exact** orientation) and 2-3 flips of crossed faces. A repair
   is applied only if it reduces the number of tets crossing the segment (2-3 flips may keep it
   equal; the created edge is then tabu). `SelfIntersection` is raised only on an *exact*
   proof (`exact::seg_tri_improper`, `tris_conflict`, closed-contact predicates): the corridor
   walk and the crossing tests of triangle recovery use symbolic perturbation, which on
   degenerate (symmetric, coplanar) input can report a crossing that is a mere touch, so a
   perturbed hit on a constraint face that the exact test does not confirm is not an error (the
   walk stops and the cavity path decides). Affine images of valid surfaces are valid
   (regression: the Schönhardt fixture scaled by 0.3 in z used to fail here).
2. **Edges, fallback.** When no flip helps, the cavity of all tets that touch the open segment
   (exact closed-contact test) is removed and refilled by *backtracking gift wrapping* (`fill`)
   with the segment as a mandatory edge, growing the cavity by up to four layers of neighbours
   (never across a present constraint triangle). Validity uses exact closed-contact predicates
   (`exact`): a new tet may meet the other cavity vertices and the front faces only in common
   sub-simplices; coplanar overlaps count. A constraint triangle that lies between two cavity
   tets without being crossed by the segment (tets that merely touch the segment from both
   sides, as happens with hint points around a triangle) stays in the cavity as a *slit*: both
   of its sides are cavity boundary, so the refill keeps it.
3. **Edges, last resort.** An edge whose cavity cannot be filled (even with one interior point,
   `fill_with_steiner`) fails with `RecoveryFailed` and yields hint points; step 5 restarts.
4. **Triangles** (all three edges present). Collect the tets whose interior the triangle
   crosses, split the cavity boundary by the side of the triangle's plane, fill each side with
   the triangle as a cap (empty-sphere preference, backtracking). A side that cannot be filled
   (Schönhardt-like) gets an interior Steiner point from a kernel search (vertex centroid, face
   centroid, alternating projections onto the inner half spaces) and is starred from it.
5. **Cheap attempts, then hints, then restart.** Each recovery attempt collects *all* its
   failures instead of stopping at the first. The first two attempts are "cheap": the cavity
   fills are budgeted (`120 + 3 faces` validity evaluations), edge cavities grow at most two
   layers, and no per-failure Steiner search runs. A failure instead yields a *hint*: a point in
   the kernel of the failed cavity, or a point just inside (and, for reflex edges, just outside)
   the solid next to the failing triangle. The pipeline then **restarts the whole build** with
   the hints as extra interior points (at most 32 or `n/8` per restart, at most twelve
   restarts); later attempts use the full budgets and `fill_with_steiner` (one interior point
   that the cavity's tets may use). If nothing is left to try, `RecoveryFailed` names the
   triangle and the edge. Restarts cure Steiner-needing configurations (`cube_with_cavity(5)`)
   and keep CAD-like planar surfaces fast: a random voxel surface that took 3 s with a
   full-budget fallback takes 30 ms with two cheap attempts (at the price of a few more
   interior points). Hints are interior Steiner points; boundary points and triangles stay
   frozen.
   **Ruling S4 on this path** (`guard.rs`). Every interior point the recovery or smoothing adds
   is scored by `Guard::score`: the smallest `dist(p, T) / h_T` over all input triangles `T`
   (`h_T` = mean edge length of `T`, capped at 0.3), or 0.3 if the point lies outside the solid
   (side of the nearest triangle, evaluated only for points closer than the bound; outside points are dropped with the exterior). Candidates are
   tried best score first (hint selection, the `fill_with_steiner` candidates, the kernel point,
   whose alternating projection runs with margins 0.4 .. 0.05 h and keeps the deepest visible
   point), so a compliant point is used whenever one exists. Where S4 is infeasible (thin parts:
   Schönhardt fixture best 0.18 h, a 10 x 10 x 0.02 slab best 0.002 h) the best-scoring point is
   used rather than failing; refusing would turn S4 into a recovery failure. Smoothing refuses
   any move that brings a vertex closer (relative to `h_T`) to a triangle than the bound or than
   it already was. Measured Steiner margins: Schönhardt 0.154 h, `cube_with_cavity(4)` 0.346 h,
   `two_cubes_shared(3)` 0.52 h, `tri_cube(4)` 0.44 h (`tests/a14_s4.rs`).
6. **Zero-volume tets.** Degenerate input (cospherical, coplanar grids) makes the perturbed
   Delaunay contain tets of exactly zero volume. After classification each one is removed by an
   edge-removal or 2-3 flip with exact positive results, else by a cavity refill, else by a
   Steiner point in the kernel of its neighbourhood. A first pass (non-strict) also runs right
   after the Delaunay build, before recovery, so crossing tests are never degenerate.
7. **Delaunay flips.** After classification, locally non-Delaunay interior faces are flipped
   (bounded work). This matters for refinement: a mesh left as recovery produced it contains
   huge needle tets whose conflict regions are the whole mesh, and circumcenter insertion
   degrades by orders of magnitude (measured: 1.1M-tet ball, 14 s with, over 500 s without).
   For the same reason the first insertion of a refinement run may carve a cavity of any size
   (a sphere's boundary points are cospherical, so the first interior point conflicts with
   every tet); later insertions are capped at 400 tets.

### Refinement

A priority queue of tets with `R/l_min > radius_edge` or `R > 0.75 h(centroid)`. The key is
`(-badness, sorted vertex tuple)`, never a slot id. The circumcenter is inserted by a
*constrained cavity*: grown from the containing tet through interior faces only, extended until
star-shaped, refused if it lies outside, closer than `0.3 h` to a cavity vertex or boundary
triangle (ruling S4), or would enclose a vertex. On failure the point moves along the segment
centroid to circumcenter at t = 1/2 and 1/4; else the tet is *unrefinable*. Unrefinable tets
(boundary slivers whose circumcenter is outside) are then attacked by quality flips, and the
count that remains is `stats.unrefinable`.

**Size.** `FromBoundary`: `h_v` = mean incident boundary edge length. At a point `p` of a tet,
`hi = min_c (h_c + (g-1)|p-c|)` is the graded upper bound; the user size (`Constant` or `Field`,
combined by `min`) can lower it but never below `lo = max_c (h_c - (g-1)|p-c|)` (symmetric
gradation), so a fine user size beside a coarse frozen boundary grades smoothly instead of
jumping: this is what gives the adjacent-tet size ratio of acceptance 7. A tet is never asked to
be smaller than 1.2x the circumradius of its largest boundary triangle (it cannot be).
Consequence: a user size far below the boundary size takes effect only at the gradation rate
away from the boundary; refine the surface mesh if the interior must be finer.

### Optimization

* **Level 1, flips.** For tets with mean ratio below 0.7: edge removal (DP) and 2-3 flips,
  accepted only if the minimum mean ratio of the affected tets strictly rises; boundary faces and
  edges are never touched (no neighbour / open ring). Exact positivity of every new tet.
* **Level 2, smoothing.** Steiner (interior) vertices only. Laplacian target (mean of the
  neighbours), backtracking line search (alpha = 1, 1/2, ..., 1/16), accepted only if the local
  minimum mean ratio rises and every incident tet stays exactly positive. Greedy colouring in
  vertex-index order; each class is evaluated in parallel (`par_iter().collect()` keeps index
  order) and written back in index order. Passes stop when the average gain drops below 1e-4.
  Sequence: flips, smooth (<= 12), flips, smooth (<= 8), flips.
* The monotonicity requirement (minimum mean ratio never below the `optimize=0` value) holds
  because every accepted operation raises the local minimum.

## Quality definitions and the `sm_quality` cross-check

| quantity | definition here | `sm_quality` equivalent |
|---|---|---|
| mean ratio | `12 (3V)^(2/3) / sum(l^2)` (1 regular, 0 flat) | `Metric::Shape` (equal to 1e-9 after no mapping; measured 7.8e-16) |
| radius-edge | `R / l_min` | `Metric::RadiusRatio = R / (3 r_in)` with `r_in = 3V / sum(face areas)`: `R = 3 RR r_in`, then `/ l_min` (measured 7.8e-16 relative) |
| dihedral | angle between face planes along the six edges, degrees | `Metric::MinAngle` / `MaxAngle` (3-D: dihedral; measured 4e-14 degrees) |

`TetStats` carries `min/max_dihedral_deg`, `max_radius_edge`, `min_mean_ratio`,
`mean_mean_ratio`; `tet_quality` evaluates a single tet. Radius-edge of a near-flat tet is
numerically unreliable (the circumsphere of four almost coplanar points); such tets are removed
by optimization, and `max_radius_edge` can be `inf` at `optimize = 0`.

## Input assumption

Recovery and refinement thresholds are calibrated on inputs whose minimum angle is about 20
degrees (the tri scheme's documented target where the boundary allows it), not on the 25 degrees
of an ideal surface mesher. The adapter never assumes more than validity: any closed oriented
surface is accepted, and failures are reported with their location rather than avoided.

## Calibration (acceptance 8)

One calibration pass; floors frozen at the task's values, with tighter regression guards (marked
"guard") below the measured numbers. Release build, default options, seeds fixed.

| fixture (default size, optimize = 2) | min dihedral | max dihedral | min mean ratio | mean mean ratio | frac R/l <= 2 at optimize 0 | unrefinable at optimize 0 |
|---|---|---|---|---|---|---|
| `tri_cube(10)` | 27.8 | 135.6 | 0.621 | 0.872 | 1.0000 | 0 of 5101 |
| `icosphere(1, 4)` | 24.3 | 139.5 | 0.593 | 0.870 | 1.0000 | 2 of 27523 |
| `tri_torus(2, 0.7, 48, 24)` | 20.1 | 139.8 | 0.562 | 0.831 | 0.9988 | 0 of 7480 |

Mean ratio without optimization (min): 0.183 / 0.040 / 0.000 (cube / sphere / torus); with level 1:
0.537 / 0.406 / 0.477. Frozen gates (task): min dihedral >= 10, max dihedral <= 165, min mean
ratio >= 0.10, mean mean ratio >= 0.75, fraction R/l <= 2 at optimize 0 >= 0.98, unrefinable
<= 0.5 %. Regression guards ("guard", in `tests/a08_quality.rs`): min dihedral >= 15, min mean
ratio >= 0.4, mean mean ratio >= 0.78. The `sm_quality` cross-check agrees to 1e-15 (mean ratio
and radius-edge) and 6e-14 degrees (dihedrals).

## Performance record (acceptance 12, record-only, 4 vCPU)

Release build, 4 vCPU, `tests/perf.rs` and `tests/adapter.rs` (`--ignored`), no gates:

| case | result |
|---|---|
| `delaunay`, 200 000 random points | 1 347 023 tets in 3.9 s = 51 000 points/s |
| `tetrahedralize`, `icosphere(1, 5)` (10 242 boundary points), `h = 0.033`, optimize 2 | 1 139 988 tets (186 076 Steiner, 120 unrefinable) in 25.4 s = 44 900 tets/s; stage ms: validate 8, Delaunay 1512, recovery + classify + flats 3025, refinement 10 084, optimization 10 478; min dihedral 16.4, min mean ratio 0.428 |
| same surface, `h = 0.07` | 182 924 tets in 7.4 s = 24 700 tets/s, 0 unrefinable, min dihedral 22.1 |
| adapter: unit brick through the `Driver`, 1/56 boundary intervals, `size = 0.018` | 1 719 532 tets in 53.7 s = 32 000 tets/s, including the serial commit into the reference builder |

The sphere is the only large core case measured; planar CAD-like surfaces were measured for
robustness (random voxel shapes, mean 0.19 s each in release over 122 cases, the slowest 9.5 s), not for
throughput. The adapter's default Steiner budget is `8 n + 100 000`, raised by an estimate of the
interior point count when a constant `size` parameter is given.

## Stretch and deferred

Not done (stretch): S1 edge removal with interior-vertex collapse as `optimize = 3` (`3` stays
`BadOption`), S2 the 15 degree gate at `optimize = 3` (level 2 already reaches the measured
minima above). Done: S3, a non-ignored smoke bound (`a12_misc::smoke_performance_bound`).

Deferred to W3b or later, as in the task: boundary Steiner points with coordinated surface
re-meshing (needs a driver retry hook), anisotropic metrics, embedded wires and points,
internal baffles, parallel (HXT-style) insertion, sliver exudation with weighted Delaunay,
neighbour adjacency exposure on `TetMesh`, higher-order tets, tet-to-polyhedron dual, the 1M-tet
gate (needs the bulk commit), automatic scheme selection. Also untested here: `PartitionCell`
pieces (the adapter reads `boundary(piece)` for the face sides and owns Steiner nodes by the
piece; only `TopoVolume` pieces have a fixture in the W3a stage 0 set).
