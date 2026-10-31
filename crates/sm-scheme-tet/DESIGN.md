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
   conforming by construction. Recovery is nevertheless complete in practice: where interior
   points do not suffice, temporary boundary Steiner points are inserted and then moved off
   the surface again (Si's surface-preserving recovery, TetGen `-Y`; step 8 below), so the
   output never has one. Invalid input (self-intersections) still fails with a *located,
   actionable* error (triangle, edge, `TopoFace`, node keys).
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
| 3 | recovery | `recover`, `steiner`, `flips`, `fill`, `exact` | see below |
| 4 | classify | `pipeline`, `relocate` | flood fill from the ghost tets toggling at constraint triangles; the side of a tet is read from the *orientation* of its constraint face, contradictions give `InvertedShell`; outside tets dropped; then zero-volume tets removed, unneeded recovery points removed, recovery points closer than ruling S4 allows moved (step 9) |
| 5 | refine | `refine`, `relocate` | constrained-cavity circumcenter insertion; then step 9 again |
| 6 | optimize | `optimize`, `relocate` | level 0 none, 1 flips, 2 flips + smoothing; then step 9 a last time |
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

### Boundary recovery (no Steiner point left on the boundary)

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
   with the segment as a mandatory edge. The search is fail-first (it wraps the front face with
   the fewest valid apexes among the first few) and, when it fails, reports the front faces
   that made candidate tets invalid by crossing or overlapping them (*blockers*). Validity uses
   exact closed-contact predicates (`exact`): a new tet may meet the other cavity vertices and
   the front faces only in common sub-simplices; coplanar overlaps count. A constraint triangle
   that lies between two cavity tets without being crossed by the segment (tets that merely
   touch the segment from both sides, as happens with hint points around a triangle) stays in
   the cavity as a *slit*: both of its sides are cavity boundary, so the refill keeps it.
   Interior points added earlier (hints, earlier Steiner points) that the cavity encloses are
   *droppable*: the refill may swallow them (`Dt::droppable_from`), they do not have to become
   vertices of the new tets.
3. **Edges, interior points.** The order for one edge: (0) the smallest cavity, with interior
   points if it needs them (`fill_with_steiner`); (A) up to eight rounds of absorbing the tet
   beyond each blocker face (never across a constraint triangle) and refilling with the vertices
   at hand, until the cavity would exceed 80 tets; (B) interior points on the grown cavity and
   up to two layers around it. `fill_with_steiner` tries, in this order: the plain fill, one
   point from the *seeds* (points next to both endpoints in the cone of their inner and outer
   sides, bisectors of the solid-side and exterior wedges around the edge, at distances capped
   at half the way to the next input triangle), one point from the cavity centroid and offsets
   of its faces, a solid-side plus an exterior point (a cavity with a slit has two sides), and
   finally *dead-face repair*: the fill reports the shallowest front face that no vertex can
   close, a point just behind that face is added (the placement that lets the search go
   deepest), and the fill is retried, up to four times. After three failed edges in a pass only
   the plain refill is tried for the rest (the pass then just collects hints). An edge that
   fails everything yields hint points; step 5 restarts.
4. **Triangles** (all three edges present). Collect the tets whose interior the triangle
   crosses, split the cavity boundary by the side of the triangle's plane, fill each side with
   the triangle as a cap (empty-sphere preference, backtracking). A side that cannot be filled
   (Schönhardt-like) gets an interior Steiner point from a kernel search (vertex centroid, face
   centroid, alternating projections onto the inner half spaces) and is starred from it.
5. **Hints and restarts.** Each recovery attempt collects *all* its failures instead of
   stopping at the first. A failure yields *hints*: the centroid of the vertices around the
   failing segment, points next to its endpoints (the fan point of a pit or peak vertex: the
   single point from which every incident triangle is visible), the wedge bisectors on the solid
   side and, free of S4 because they are dropped with the exterior, on the exterior side; after
   a few restarts also the chamber points next to its two triangles. The pipeline then
   **restarts the whole build** with the hints as extra interior points (at most 32 or `n/8`
   per restart, at most two restarts). Since review round 2 only a tiny surface component (at
   most 64 vertices: a Schönhardt-like pocket) restarts; everything else that is stuck goes to
   complete recovery (step 8) on the triangulation the pass left. The interior-point searches
   of steps 2 and 3 are budgeted by count: an edge cavity above 80 tets is not searched, and a
   pass stops searching after 2.5 million units of fill predicate work (`Dt::work`, one unit
   per vertex side test or per vertex / front face checked by a validity test); successful
   searches on the test families used at most about 2.2 million, the stuck searches on the hard
   relief cases tens to hundreds of millions. The previous design ran two deliberately cheap attempts first; review
   round 1 removed them: the hints from a cheap pass enlarged the point set and made later
   cavities harder, and failing inputs paid for several full passes. Hints are interior Steiner
   points; boundary points and triangles stay frozen.
   **Ruling S4 on this path** (`guard.rs`). Every interior point the recovery or smoothing adds
   is scored by `Guard::score`: the smallest `dist(p, T) / h_T` over all input triangles `T`
   (`h_T` = mean edge length of `T`, capped at 0.3), or 0.3 if the point lies outside the solid
   (side of the nearest triangle, evaluated only for points closer than the bound; outside points are dropped with the exterior). Candidates are
   tried best score first (hint selection, the `fill_with_steiner` candidates, the kernel point,
   whose alternating projection runs with margins 0.4 .. 0.05 h and keeps the deepest visible
   point), so a compliant point is used whenever one exists. Where S4 is infeasible (thin parts:
   Schönhardt fixture best 0.18 h, a 10 x 10 x 0.02 slab best 0.002 h) the best-scoring point is
   used rather than failing; refusing would turn S4 into a recovery failure. Step 9 then tries
   to move it, and what remains is counted (`TetStats::n_s4_violations`). Smoothing refuses
   any move that brings a vertex closer (relative to `h_T`) to a triangle than the bound or than
   it already was. Measured Steiner margins: Schönhardt 0.154 h (one point), twisted prisms of 5
   and 10 degrees 0.289 h and 0.263 h (one point each; round 0 needed three at 0.14 h),
   `cube_with_cavity(3)` 0.302 h, `cube_with_cavity(4)` 0.347 h, `two_cubes_shared(3)` 0.41 h,
   `tri_cube(4)` 0.44 h (`tests/a14_s4.rs`, `a13_schonhardt_family`).
   **One definition of `h`.** The ruling does not define it; the crate uses `h_T`, the mean edge
   length of the input triangle `T`, everywhere: the guard (recovery, smoothing, zero-volume
   repair), refinement (`Guard::margin(p) >= 0.3` against *every* input triangle, not only the
   boundary faces of the insertion cavity) and the test measure (`common::min_s4_margin`). The
   one remaining use of the graded size field is the distance of a refinement point to the
   *vertices* of its cavity (`0.3 * size_at(p)`): vertices are interior points as often as
   boundary points and have no triangle to scale by. Round 0 measured `cube_with_cavity(3)` at
   0.267 h_T because refinement checked `0.3 * size_at` against the cavity's boundary faces only.
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
8. **Complete recovery** (`steiner.rs`; Si & Gärtner, IJNME 2011; Si, ACM TOMS 2015). Runs on
   the triangulation the interior-point pass left when that pass is stuck:
   * *Segments.* A missing input edge is recovered by flips (as in step 1); where they are
     stuck, a point is inserted *on* it where it leaves the first tet of its corridor: a 2-6
     split of the crossed face, else an n-2n split of the crossed face's edge the segment
     passes (almost) through, else a split of the simplex around the start vertex that holds a
     point of the segment (exact location: tet, face or edge). Every new tet is validated
     exactly; the first piece is an edge at once. If every point at the start is degenerate
     the edge is split from its other end. The chain of points of each input edge is kept.
   * *Facets.* An input triangle, now a polygon with the points of its edges, is first read
     off the mesh if already present, else recovered by the cavity refill of step 4 when its
     edges are whole. Otherwise every tet edge crossing it (exact test against the triangle;
     in a second pass against an ear triangulation of the polygon through the actual points)
     is removed: by an edge removal creating no new crossing, else by an n-2n split where it
     crosses (each split removes one crossing edge and creates none; rounding next to an
     almost flat ring tet is handled by trying nearby roundings, by removing an interior
     endpoint, or by first flipping the flattest ring tets away). Then the triangle is tiled by
     the mesh faces whose vertices all lie on it; the tiling is found by a backtracking search
     from the boundary pieces (a tet lying flat in the triangle offers two tilings of its
     outline; needle faces along an edge may be used either way) and checked combinatorially.
   * *Invariants.* Nothing given up: no flip removes a constraint edge, cavity refills in this
     mode refuse a cavity holding a constraint edge, and no operation creates a *shortcut*
     (an edge between two points of one split input edge that are not neighbours on it: it
     would lie along the edge past a point rounded off it and leave flat tets on the surface).
   * *Constraint lookup.* The subsegments and the recovered tiles (with their edges) are kept
     in `BTreeMap`s with owner multisets and updated in place: a split replaces one subsegment
     by two, a recovered triangle adds its tiles, a suppression swaps the fan tiles of the
     point. Review round 3 replaced a rebuild of the whole lookup after every recovered
     triangle, split and suppression (quadratic: 40 s for complete recovery on 12 288
     triangles). `TetStats::n_constraint_updates` counts the entries touched, about six per
     input triangle at every size (`a15::complete_recovery_work_is_linear`).
   * *Suppression.* Every temporary point is then moved off the surface, last created first:
     its fans in the surface (one closed fan for a point inside a triangle, two open fans for a
     point on an edge) are re-triangulated without it by ear clipping in the triangle's plane
     (largest smallest angle first, exact 2-D orientations); this cuts its star into a solid
     side and an exterior side. The solid side is refilled without a point if a short gift-wrap
     search finds one, else each side is starred from a point in its kernel next to the old
     position (the kernel point of step 4, the deepest the kernel search finds, and
     `p + delta d`, `d` the bisector into that side, delta from 0.5 h down to 1e-12 h; the
     best-shaped star wins; points left too shallow are handled by step 9). Review round 3
     tried ranking the solid-side candidates by S4 margin before shape (no change on the hard
     relief cases once step 9 runs, but slower and a worse minimum dihedral on
     `relief n12 seed7 d0.995`) and a wider candidate set (all kernel candidates, offsets up to
     1.0 h: it lost `relief n12 seed4 d0.97`, a deeper star blocking a later suppression). One exists in exact
     arithmetic (the old point sees every link face strictly and the new fans are planar); when
     rounding lets a sliver lying flat on the surface hide it, the side cavity absorbs the tets
     behind the blocking faces (never across a constraint face, never enclosing a vertex that
     must stay; up to six rounds, 400 tets) and is starred. Solid-side points are interior
     Steiner points; exterior ones are dropped with the exterior. When all are gone the input
     triangles are faces again, bit-exactly (checked).
   * *Cleanup.* After classification every recovery point whose star can be refilled without
     it (exact gift wrapping) is removed (`Dt::remove_points`), on both paths; the points left
     too close to the surface go to step 9.
   Complete recovery is deterministic (sorted keys and creation order only, no clocks) and
   sequential. `TetStats` reports `n_steiner_recovery`, `n_steiner_refine`,
   `n_boundary_steiner` (temporary points, none in the output) and `n_constraint_updates`.
   A complete-recovery failure (not seen on the test families) falls back to a restart with
   hints and finally to a located `RecoveryFailed`.
   Growing the solid side of a suppressed point further while its best point violates S4 was
   tried in review round 3 and dropped: on `relief n8 seed4 d0.9` a grown side made a later
   suppression impossible (complete recovery failed and the build restarted). After
   classification, where every constraint face is a hull face, cavities can grow freely
   instead (step 9).
9. **Ruling S4 for recovery points** (`relocate.rs`, review round 3). A point of the star of a
   suppressed surface point, or a kernel point of a thin cavity, can sit within a fraction of
   `h` of the surface (before round 3: 0.0003 .. 0.03 h on the hard relief cases, minimum
   dihedral angles down to 0.014 degrees). Each interior point with margin below 0.3 (in
   index order) is removed or moved: for *targets* near it (offsets of 0.35 .. 1 h inward from
   its nearest input triangle and from the hull faces of its star, centroids of the tets
   within three layers; those with margin at least 0.3, nearest first, at most 16) the cavity
   is grown from its star across every interior face that does not see the target strictly
   (at most 100 tets, never enclosing a vertex other than an interior point, which is
   dropped). If the target sees every face of the cavity it is starred from the target (the
   star with the largest smallest dihedral angle over the targets wins); else the cavity is
   gift-wrapped with the target as an optional apex (budget 10 000 + 100 per face; the first
   success, smallest cavity first). A target outside a thin part never becomes an apex but
   still steers the growth, whose refill then needs no point: on the test families this is
   how most of these points disappear. If no target works, the star is grown by whole layers
   (six rounds, 400 tets): a refill without a point or a compliant kernel point ends it,
   otherwise the deepest kernel point that raises the margin without lowering the worst mean
   ratio is used and revisited. Three sweeps per call (a point that failed is retried only
   when its star changed); the three calls of one build share a budget of 80 million units of
   fill work (`Dt::work`; the eleven hard relief cases use at most about 65 million, about 8 s
   on the loaded test host; a point is not started once the budget is used up, which is
   deterministic because the counter is only advanced by sequential code). It runs after
   classification, after refinement (whose points the cavities may then drop) and after
   optimization (rarely needed; the hardest relief case needs it). Exact orientation of
   every new tet, sequential, deterministic. Points it adds after refinement count as
   refinement points in `TetStats`. `TetStats::n_s4_violations` and
   `worst_s4_margin` report what is left over every interior point of the output (thin parts
   where S4 is infeasible, such as the 0.02-thick slab, are counted, never hidden). On the
   eleven hard relief cases every point now meets S4 and recovery adds 0 or 1 points
   (before: 3 .. 13); the smallest dihedral angle is 0.56 .. 5.2 degrees (before 0.014 .. 2.7),
   the worst tets having input vertices only: they span thin walls next to the steepest
   relief, where the sharpest solid wedge is 8 .. 15 degrees and S4 leaves no room for a point.

### Refinement

A priority queue of tets with `R/l_min > radius_edge` or `R > 0.75 h(centroid)`. The key is
`(-badness, sorted vertex tuple)`, never a slot id. The circumcenter is inserted by a
*constrained cavity*: grown from the containing tet through interior faces only, extended until
star-shaped, refused if it lies outside, closer than `0.3 h` to a cavity vertex or to an input
triangle (ruling S4, `h_T` of each input triangle, all of them), or would enclose a vertex. On failure the point moves along the segment
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
* **Spacing, `Dt::respace`** (INT-7, below), the last move of level 2. It is the one operation
  that does not raise a local minimum; it is constrained to leave the mesh minima alone.
* The monotonicity requirement (acceptance 8: the minimum mean ratio, and the minimum dihedral,
  never below the values before optimization) holds in two parts. Flips and smoothing accept
  an operation only if the minimum of the tets it replaces strictly rises, so they cannot lower
  the mesh minimum. `respace` may lower the quality of the tets of the vertex it moves, but only
  down to the *global* minima of mean ratio and dihedral that the mesh had when the pass started
  (and never below `MIN_MEAN_RATIO` / `MIN_DIHEDRAL_DEG`): every accepted move keeps all
  incident tets at or above those two values, and a move changes only its own incident tets, so
  the mesh minima after the pass are at least those before it, in any order of acceptance and
  for any thread count. What may still change the minima afterwards is step 9 (S4 relocation,
  which can trade quality for distance from the surface and runs again after the pass) and
  the flip sweep that follows it; the claim is therefore "the pass does not lower the minima"
  (asserted against the same build without the pass in `int7_r5::respace_never_lowers_the_mesh_minima`)
  and the end-to-end gates are the measured floors of `a08_quality`. A mesh whose minimum
  already lies below the quality floor keeps what it has (the floor applied to a move is
  `max(global minimum, MIN)`, which a below-floor vertex can only meet by improving).

## Ruling INT-7: spacing objective, quality floor, located diagnostics (review round 4)

INT-7 (docs/design/w3/plan.md) supersedes ruling S4. The 0.3 h distance was a proxy for element
quality; the floor now applies to quality itself and spacing is an objective. S4 itself
(`n_s4_violations`, `worst_s4_margin`, and the 0.3 h guard of refinement, smoothing and step 9)
is unchanged in the code and stays a **reported metric**; what changed is what is judged and
reported.

**Normalized spacing.** For an interior Steiner point `p` (every point of the output beyond the
input points), `r(p) = d(p) / h_loc(p)`:

* `d(p)` is the distance to the nearest input triangle (`spacing.rs`, a second `Guard` grid that
  lists each triangle in the cells within `REACH = 1.5` of its own size). A boundary vertex lies
  on a triangle, so the triangles alone give the nearest boundary vertex or triangle.
* `h_loc(p)` is the size field at `p`, built like the refinement size: `hi = h_T + (g - 1) d`
  from the nearest triangle `T` (`h_T` its mean edge length, `g` the gradation), lowered by the
  user size (`Constant` / `Field`), floored at `max(h_T - (g - 1) d, 0)` but never above `hi`.
* `r` is clipped at `cap = REACH / (1 + (g - 1) REACH)` (1.03 for `g = 1.3`): a point with no
  triangle in reach has at least that much. Edge case, disclosed: the search takes the nearest
  triangle *listed in the cell of `p`*; if a coarse triangle is listed and the true nearest one,
  much finer, is not (a triangle is listed only within `1.5 h_T` of itself), `r` is measured
  against the coarse one and is too large, never too small by more than the clip.
* Target `TARGET_R = sqrt(2/3) = 0.8165`: the height of a regular tet over its base face in
  units of the edge length, i.e. the spacing at which the first layer of an equilateral mesh
  sits.

`TetStats::spacing` ([`SpacingStats`]) holds `count`, `mean`, `std_dev` (population), `p01` (the
sorted value at index `floor(0.01 (count - 1))`, i.e. lowest-rank), `min`, `target`, `rms_dev`
(root mean square of `r - target`, the value of the objective) and `cap`. All sums run over the
sorted list: bit-identical for any thread count. An independent implementation in the tests
(`tests/common/r.rs`, bucket grid, brute-force nearest triangle) agrees to 1e-9
(`int7::spacing_stats_match_an_independent_measure`).

**Objective.** Two places minimize the deviation of `r` from the target, both budgeted and
deterministic:

1. Step 9 (`relocate.rs`): the targets of a point are the same nearest-16 candidates as before
   but are tried in order of `|r - target|`, and among refills whose smallest dihedral is at
   least `MIN_DIHEDRAL_DEG` the target with the smaller deviation wins (below the floor the
   larger smallest dihedral wins, as before); among equal-margin kernel candidates of the
   layer-wise fallback with mean ratio at least 0.1 the smaller deviation wins. It still runs
   only on points that violate S4 (the 80 M work budget is unchanged). On the hard relief cases
   it rarely changes the result (most of these points are removed without a replacement); the
   gain comes from 2.
2. `Dt::respace` (`optimize.rs`, after the last smoothing pass, three passes): every interior
   point with `r < TARGET_R` (too close to the surface; points deeper than the target are left
   alone) steps (half, quarter, eighth of the way) towards the point at distance `TARGET_R h_loc` along the
   line from its nearest triangle's closest point through itself. Accepted when no incident
   tet is inverted (exact), the local minimum mean ratio stays at least
   `min(0.99 base, 0.5)` and the local minimum dihedral at least `min(0.97 base, 20 degrees)`
   (a neighbourhood better than 0.5 / 20 degrees may trade the excess for spacing; one below
   keeps what it has), the S4 margin does not shrink, the ratio of mean edge lengths of adjacent tets does not
   exceed `max(its value now, 1.4)` (size grading) and the deviation falls. Round 5: **and the
   incident tets stay at or above the global minima** of mean ratio and dihedral that the mesh
   had when the pass began (clamped from below at `MIN_MEAN_RATIO` / `MIN_DIHEDRAL_DEG`). The
   local relative guards alone let many vertices each take 1 .. 3 % off a neighbourhood that was
   the mesh minimum (round 4: ball `icosphere(1, 4)` h 0.1 min dihedral 23.0 -> 19.3, cube with
   a NaN/0.08 size field 26.1 -> 20.3, gradation-1 ball 26.2 -> 19.8). With the global floor
   the minima are exactly those of the same build without the pass on the four good meshes
   and on the relief cases (`int7_r5::respace_never_lowers_the_mesh_minima`), and 85 .. 98 %
   of the spacing gain (fall of the rms deviation of `r`) is kept on them (report, round 5).
   Colour classes as in `smooth`: bit-identical for any thread count. A first version with a
   looser rule (`min(base, 0.3)` per pass, six passes) cascaded to min dihedral 9.3 and min
   mean ratio 0.30 on the ball and was dropped. The mean of the mean ratio still falls by about
   0.001 .. 0.03 (spacing is bought with average quality, not with the worst tets).

**Quality floor.** `MIN_DIHEDRAL_DEG = 5` (smallest dihedral of a tet) and `MIN_MEAN_RATIO =
0.05` (mean ratio of a tet, the in-crate proxy for cell quality). 5 degrees is the level the
optimizer reaches on non-pathological input (0 violations on the ball `icosphere(1, 2)`,
asserted in `int7`; the ball record has a 19 degree minimum), well above the 0.5 / 1.5 degree test floors of the hard relief
cases. The dual polyhedral cell quality after `mesh.to_poly` is not computable here (sm-poly
owns it); the tet mean ratio is the proxy, and the polyhedral metric belongs to sm-diag.
Every tet below a threshold, per metric, is counted (`n_quality_violations`, exact) and, by
default, every one is located: `TetStats::quality_diagnostics` has exactly
`n_quality_violations` entries, sorted worst first by `value / threshold`, then tet index in the
canonical order, then metric (deterministic; a thin slab, `tri_cube(6)` with z scaled by 0.02,
has 400 located of 400 counted, `int7_r5::diagnostics_are_uncapped_by_default`).
`TetOptions::max_diagnostics` (default 0 = no cap) keeps only the worst that many, for callers
that want a bounded list; the count stays exact. Each entry is a `QualityDiagnostic`:
position (tet centroid), up to six nearest input triangles (indices into `SurfaceInput::tris`
as the caller listed them, i.e. the owners; the pipeline's canonical reordering is undone;
**empty** when the centroid is deeper than the reach, 1.5 triangle sizes, of every input
triangle, which large interior tets of a coarse region can be), metric, measured value, threshold and a
`SuggestedFix`: `RefineSizeNear { h }` (half the local size) when the tet has an interior
vertex, `RefineInputSurface { h }` when it spans input vertices only (frozen: remesh those
triangles finer or smooth the surface there). Violations are never hidden and never an error.

**Performance caps (review round 4).**

* Complete recovery finds the tiles of a temporary point by vertex (`Bnd::by_vertex`, serial
  numbered `subs`) instead of scanning all tiles per point: `suppress` was O(T S).
* The post-classification `Dt::remove_points` is bounded: points are tried in order of star
  size, never above `MAX_REMOVE_STAR = 160` tets, and until `REMOVE_WORK = 30 M` units of fill
  work (the counter of step 9) are spent per call. Points skipped stay (always valid) and are
  counted in `TetStats::n_removal_deferred`. Before the round this step took 72 s of 105 s on
  the 64 x 64 tiled relief cube (stars up to 408 tets).
* `TetStats::n_lookup_rebuilds` counts from-scratch rebuilds of the constraint lookup of
  complete recovery. There is no such code path (the lookup is incremental), so it is 0 by
  construction; the field and the asserts in `a15::complete_recovery_work_is_linear` and
  `int7::tiled_relief_work_per_triangle_is_bounded` are tripwires for someone reintroducing one.
  `steiner::tests::bnd_index_follows_tiles_and_names_owners` checks the incremental lookup
  against a rebuild.

**The tiled-relief cliff (round 5, investigated, not changed).** `tiled_cube(n, 8, 2, 0.9)` at
n = 48 needs 355 temporary boundary points and 5 s; n = 64 needed 2 643 and 28 s (1.8 times the
triangles). Instrumented (counters `n_recovery_restarts`, `n_boundary_steiner_discarded`):

| | n = 48 (36 patches) | n = 64 (64 patches) |
|---|---|---|
| interior-point recovery | fails at attempt 0 | fails at attempts 0, 1, 2 |
| complete recovery | succeeds at attempt 0: 355 points | fails at attempt 0 (451 points, triangle 26383: "not tiled by mesh faces after removing its crossing edges") and attempt 1 (861 points, then triangle 8619), succeeds at attempt 2 with 2 643 |
| restarts / points discarded | 0 / 0 | 2 / 1 312 |
| extra interior points of the restarts | 0 | 3 072, then 6 144 (hints of every failed patch, cap `max(32, n_input / 8)`) |

So the anomaly is not the patches: it is that one hard patch out of 64 defeats complete
recovery, and a failure restarts the *whole* build (all 64 patches) with up to `n_input / 8`
hint points taken from all failed patches of the surface. Two restarts repeat the recovery
three times; the last attempt runs with 6 144 foreign interior points in the Delaunay
triangulation, which make its recovery 3.3 times larger (2 643 points against about 800 per
attempt without them) and 2 times slower (recover stage 14.6 s against 7.3 s with 64 extras).
The number of failing patches grows with the number of patches (the relief of each patch is a
random draw), hence "from about 40 patches on". Fixes tried and **declined**: (a) keeping
only the hints within 8 or 3 edge lengths of the failed triangle, (b) capping the extras at
32 per restart. Every variant changed the outcome of a lottery, not the cause: the same
triangle 26383 fails again at attempts 0 and 1 whatever the extras, and the build only
succeeds when the perturbation happens to cure it (cap 32 with all hints: all three attempts
fail, `RecoveryFailed`; cap 32 within 8 edge lengths: fails; cap 32 within 3 edge lengths:
succeeds with 507 points in 17 s; cap 3 072 within 3: 780 points, 20 s). A policy that passes one case and
fails another by chance is not safe to ship. The real fix is in complete recovery
(`recover_facet`: the rounding case in which no tiling shows up for a triangle), a different
change. Work counters stay in tests (`int7::tiled_relief_work_per_triangle_is_bounded` asserts
0 restarts at the sizes it runs); wall time stays in `record_tiled_relief`.

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

One calibration pass, refreshed in review round 1 from the output of that commit (round 0 had
the table of an earlier commit: the sphere and the torus have since moved because refinement
now applies ruling S4 against every input triangle). Floors frozen at the task's values, with
tighter regression guards (marked "guard") below the measured numbers. Release build, default
options, seeds fixed.

| fixture (default size, optimize = 2) | min dihedral | max dihedral | min mean ratio | mean mean ratio | frac R/l <= 2 at optimize 0 | unrefinable at optimize 0 |
|---|---|---|---|---|---|---|
| `tri_cube(10)` | 27.8 | 135.6 | 0.621 | 0.872 | 1.0000 | 0 of 5101 |
| `icosphere(1, 4)` | 25.1 | 137.9 | 0.558 | 0.871 | 1.0000 | 0 of 27619 |
| `tri_torus(2, 0.7, 48, 24)` | 21.2 | 137.4 | 0.543 | 0.832 | 0.9988 | 0 of 7503 |

Mean ratio without optimization (min): 0.183 / 0.020 / 0.000 (cube / sphere / torus); with level 1:
0.537 / 0.471 / 0.400. Frozen gates (task): min dihedral >= 10, max dihedral <= 165, min mean
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
| complete recovery scaling (`perf::record_hard_patch_scaling`, review round 3; 10-core host at load 15 .. 50): `tri_cube(n)` plain vs with an 8 x 8 hard relief patch (`patch_cube(n, 8, 2, 0.9)`) | n = 16 (3 072 triangles): 0.20 s vs 0.82 s; n = 32 (12 288): 0.85 s vs 1.64 s; n = 48 (27 648): 2.19 s vs 3.46 s (1.6 x). Constraint updates 18 272 / 69 000 / 153 500 (5.9 / 5.6 / 5.6 per triangle). Before round 3: patch n = 16 2.1 s, n = 32 27.5 s (26.1 s of it in the facet phase), n = 48 247 s (orchestrator's measurement) |

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
