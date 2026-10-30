# sm-scheme-pave

The surface `pave` scheme: an unstructured all-quad mesh of a face with fixed boundary nodes.
The face is meshed in its metric-corrected `(u, v)` parameter space: constrained Delaunay
triangulation of the given boundary nodes, rows of frontal points, a perfect matching of the
triangles into convex quads, smoothing. The interval solver only has to supply an even boundary
total.

## Positioning

> Like **Cubit's `pave`** (Blacker-Stephenson paving) and **Gmsh's Frontal-Delaunay for quads
> with Blossom recombination** (Remacle et al. 2013), it meshes a face with any number of loops
> by quadrilaterals that conform to fixed boundary nodes and an even-interval rule, with
> irregular interior nodes where the geometry needs them. It differs in three ways:
> - **(A) It works in a metric-corrected parameter space with exact predicates and a seam-aware
>   boundary.** A global affine map makes the mean first fundamental form of the loop the
>   identity (exact isotropy on planes, cylinders and cones), the residual metric is evaluated
>   per point, and every triangulation decision is an exact `orient2d`/`incircle` of
>   `sm-math`. Boundary `(u, v)` come from `GenCtx::loop_uvs` (pcurves, seam aware), so a
>   periodic face meshes as the unrolled rectangle without any 3-D projection loop.
> - **(B) It is a pure, deterministic function of the plan.** The kernel is sequential, every
>   tie is broken by `total_cmp` plus index, there is no hash map, randomness or clock, so 1
>   thread and N threads give bit-identical meshes.
> - **(C) It is declarative.** The scheme states `VertexRule::Free` and two kinds of interval
>   rows (an even sum over all loops, at least four nodes per loop); soft counts are repaired by
>   the solver before generation, and a hard odd count blames `pave`, which makes the driver fall
>   back (production: `trimesh`).
>
> Compared with classical paving there are no closure or seaming heuristics: robustness comes from
> the Delaunay triangulation plus a matching that either completes or fails with a typed error.
> Quality is a little below Cubit's mature paving where acute corners or coarse unequal
> boundaries leave no perfect matching (see Known limitations); valence cleanup and optimization
> smoothing are deferred to W3b.

## Phases

| Phase | What it does |
|---|---|
| `caps` | `QUAD` only. Never `STRUCTURED`/`BLOCK_STRUCTURED`: a sweep may take a paved source face but must reject it as a linking face. `supports_groups = false`. |
| `applicability` | Score **40** ("unstructured quad", the decision 12 provisional table: map 100, submap 80, pave 40, trimesh 30) for a face with a surface carrier. `NotApplicable`, each with a reason, for: unknown parameter field or out-of-range value (`gradation` outside `(0, 1]`, `smooth_iters` above 10000); no surface carrier; a `DEGENERATE` edge (pole, W3b); embedded edges or vertices (W3b); a periodic surface with any coedge without a pcurve (the plan-time proxy of "`loop_uvs` is `None`": that is exactly its precondition); no loops. |
| `requirements` | `vertex_rule(face, VertexRule::Free)` only. |
| `interval_constraints` | `even(all coedge edges of all loops)` with multiplicity (a seam used twice counts twice, the total is over all loops: Cubit's rule), plus per loop `linear(terms, Ge, 4)` (a polygon needs four nodes). |
| `generate` | below |

## Generation

1. `GenCtx::loops` gives the boundary node keys per loop; `GenCtx::loop_uvs` gives their
   `(u, v)` (one entry per node, seam uses get different `u` for the same key). `None` is a
   `Geometry` failure; there is no private copy of that logic.
2. Callbacks for the kernel: `metric(uv) = [du.du, du.dv, dv.dv]` from `ParametricSurface::jet`,
   `size(uv)` from `cx.size` evaluated at `eval(uv)` (default: the boundary segment lengths).
3. `pave_region` (kernel, below) returns quads over `points`.
4. Boundary nodes enter the patch by `PatchBuilder::refer` (which merges the two seam
   uses of one key); interior points are `NodeKey::new(face, i)` in kernel order, positions from
   `eval_many`, `Param::Surface([u, v])`; quads are `ElemShape::Quad`, no `Patch::grid`.
5. Checks, each a `Quality` failure carrying the four nodes (`GenerateFailure::with_nodes`): a
   quad that repeats a key; a quad whose area normal `(p2 - p0) x (p3 - p1)` disagrees with the
   face normal (the carrier's surface normal at the quad's parameter centroid, flipped by the
   carrier sense: the same value `geom::face_normal` computes at the centroid, without a
   projection per quad); a 3-D scaled Jacobian (minimum corner sine against that normal) of
   0.05 or less.

## The kernel (`pave_region`)

Pure and sequential, in the module `kernel` (`cdt`, `metric`, `front`, `matching`, `smooth`).

1. **Preflight.** Odd total, loops under three nodes, non-finite input, exact
   self-intersection and touching tests over all segments (x-sorted sweep, `orient2d`),
   zero outer area. The outer loop's signed area decides the handedness; a clockwise outer loop is
   reflected (`x -> -x`) inside the frame and the quads keep the winding of the input, so an input
   segment `a -> b` is owned by a quad that lists `a` then `b`, whatever the handedness.
2. **Metric correction.** `w = R A uv`, `A` the upper Cholesky factor of the mean first
   fundamental form over the boundary nodes (no translation: an identity metric is a bit-exact
   identity map), `R` the reflection. The residual metric `M(w) = (A^-1 R)^T G (A^-1 R)` is
   factored `B^T B` at each point; lengths are `|B d|`, frontal directions and circumcircles are
   built in `B`-space.
3. **Constrained Delaunay triangulation.** Super triangle at 8 bounding-box sizes, Bowyer-Watson
   style insertion of the boundary nodes, constraint recovery by edge swaps (Sloan), parity
   flood fill from the super triangle (every constraint toggles in/out, so holes are free),
   removal of the outside, a Lawson pass with exact `incircle`. Boundary nodes are never split.
   Adjacency is `Vec<[u32; 3]>` (SoA), triangles are CCW.
4. **Default size.** Node size = mean metric length of the two adjacent boundary segments,
   interpolated barycentrically over the boundary-node triangulation, limited by `gradation`
   (`h(p) <= h(q) + gradation d(p, q)` over the triangulation edges, Dijkstra order).
5. **Frontal rows.** The boundary loops are the first fronts (interior on the left). Every front
   node proposes an offset point along the inward bisector, at distance `h / cos(half turn)`
   (clamped to 2 h), i.e. at the far corner of a *square built on the front edges*, so pairs
   of triangles of the lattice recombine into squares. A proposal is inserted (Delaunay,
   located by a walk from the proposing node) if it is inside, not on a constraint and at least
   `0.6 h` from every vertex of its cavity. Coincident proposals (the corners of a front) are
   dropped; where a row crowds (a closed front shrinks) adjacent proposals are dropped **in
   pairs**, because a lone dropped node leaves a wedge triangle that no convex pair can absorb;
   a closed front keeps the parity of its node count. A rejected proposal because of another
   front or the boundary ends the chain there (fronts collide, no seaming), the surviving runs
   are the next fronts.
6. **Refinement.** Triangles with circumradius above `0.85 h` get their circumcenter (or
   centroid) if it keeps `0.5 h` from every vertex: this fills whatever the rows leave.
7. **Recombination.** The dual graph of the triangulation restricted to *strictly convex*
   pairs (both reflex-free corners by exact `orient2d`), weight = corner sine times the square
   root of the edge ratio. Greedy by weight in stages (sine floors 0.6, 0.35, 0.15, 1e-4), each
   followed by Edmonds' blossom augmentation (unweighted: only *minimum-weight* optimality is
   deferred); deterministic roots and neighbours. If triangles stay unmatched: **flip
   repair** (flip any convex edge within two triangles of an exposed one, keep it if
   re-augmenting lowers the exposure), then **edge splitting** exactly as the plan says (the
   longest interior edge of each unmatched triangle gets its midpoint, `T` grows by two per
   split, the boundary is untouched), at most `max_rounds` rounds. Then `NoPerfectMatching`.
8. **Smoothing.** Gauss-Seidel smart Laplacian in the `B`-frame over the interior points only
   (`smooth_iters` sweeps): a move is kept iff every incident quad stays strictly convex (exact),
   the worst incident corner sine does not drop and the mean rises. Boundary points are never
   touched.
9. **Output.** Interior points go back to `(u, v)` through `A^-1 R`; boundary points are the
   input, bit for bit. A last exact `orient2d` pass over every quad in the original parameter
   space guards against a numerically singular metric map (`Degenerate`).

### Strategy ladder

Coarse or unequal boundaries (a few nodes against the size, acute corners) can leave a
triangle dual with no perfect matching under the default placement. The kernel therefore
retries a *fixed, ordered* list of placement variants (size scale, row distance, thinning run,
finally "no rows at all"), the first that recombines completely wins; every variant is as
deterministic as the first. `QuadMesh2::rounds` reports the splitting rounds of the winning
attempt and `QuadMesh2::size_scale` the size factor of the winning strategy (above 1 means the
mesh is coarser than the wanted size; the scheme refuses a mesh more than 2x coarser than an
explicit size field, since `Patch` has no diagnostics channel).

**Cut-off.** A failing region is cut off by *counts*, never by the clock: after the first
attempt the ladder may spend `6 x` that attempt's work plus a floor of `10^6` work units
(a unit is one triangle visit of the recombination, including the per-flip re-matching, plus 8 per
vertex; roughly 0.3 us in release), at most 40 attempts, and each later attempt is abandoned
at `4 x` the first attempt's vertices (at least 4000). A small face therefore fails in well
under a second (a thin triangle that used to take 3 s now takes about 0.5 s), a large
one after a few attempts.

### Strip fill

A single loop whose boundary nodes split into four logical sides with equal opposite counts
(`a = c`, `b = d`, so `a + b = n / 2`) is a structured grid; the triangle route cannot
recombine thin ones (a 10 x 0.2 rectangle with 50 x 2 intervals has no perfect matching in the
Delaunay triangles, 10 to 28 s to fail the old ladder). `strip.rs` picks the four corners by
minimising the deviation of the turning angle from a right angle over all `(i, a)` pairs
(O(n^2), nodes <= 8192), fills the interior by transfinite interpolation on index fractions
(boundary nodes untouched) and accepts the grid if every quad is strictly convex by exact
`orient2d` and the worst corner sine is at least `1e-3`. Order of use: a grid with corner sine
>= 0.95 is taken first (an unstructured mesh cannot beat it); otherwise the default placement,
then a grid with sine >= 0.3, then the ladder, and a worse grid only after the whole ladder
failed. A grid more than 1.2x coarser than the wanted size waits behind the ladder (and is
the fallback). Not covered: three-sided regions (a triangle with sides `a, b, c` needs
`a + b - c >= 2` and so on for a three-block decomposition; `10 x 0.5` with 20/1/21 has `a + b = c`
and no all-quad mesh without irregular transition pairs), unequal opposite counts, holes.

### Explicit size field

An explicit `size` is limited by the boundary-derived field (which carries `gradation`): the
interior is never coarser than the boundary spacing grades into. A size *finer* than the boundary
spacing is not graded from below and the kernel cannot refine more than about 1.5x against fixed
boundary nodes. Three outcomes: (1) the ladder finds a mesh (possibly coarser than asked, `size_scale > 1`);
(2) every strategy fails, and `pave_region` re-runs the ladder without the explicit size, i.e. at the
spacing the boundary allows, and reports the measured geometric-mean edge/size ratio as `size_scale`
(the scheme refuses a mesh more than 2.1x coarser than the size field as `Quality`, and the driver
falls back); (3) for a single loop with equal opposite counts the structured strip grid is taken at
once only when it is at most 1.2x coarser than asked, otherwise the ladder runs first and the grid is
the fallback (a 4x3 rectangle with counts 8/6/8/6 at size 0.35 gives 97 quads; at 0.25, where the
ladder fails, the 48-quad grid replaces the request, `size_scale` 2.0).
Absolute bound: the vertex count estimated from the size at the centroid of
every boundary triangle is limited to `2 x 10^6` (`PaveError::TooLarge`, a `Geometry` failure),
and so is any attempt that grows past it.
The production ladder lists 14 strategies ordered by a greedy cover over a 1275-case corpus (`survey_cover`, ignored test), then the other 80 candidates in a fixed order (they recover regions with spatially varying metrics, which the corpus lacks), all under the cut-off.
Surveys and the measured limits are in `W3_REPORT.md`.

## Failure mapping

| Failure | `FailureKind` |
|---|---|
| odd boundary total (`OddBoundary`), loop with fewer than three nodes (`LoopTooSmall`) | `Constraint` |
| `BoundaryIntersects`, `NoPerfectMatching`, `BadMetric`, `BadOrientation`, `NonFinite`, `Degenerate`, kernel guards (`Internal`: vertex budget, ...) | `Geometry` (the driver falls back; `Internal` is deliberately *not* mapped to `FailureKind::Internal`, which aborts the run) |
| `loop_uvs` is `None`, no surface carrier at generation | `Geometry` / `NotApplicable` |
| a quad repeats a key, disagrees with the face normal, has 3-D scaled Jacobian <= 0.05 | `Quality`, with the four node keys |
| unknown or out-of-range parameters | `NotApplicable` |
| a missing earlier-wave node, a face without loops | `Internal` (contract violations) |

The planner never blames a scheme whose choice is a hard intent: for a loop whose hard counts
are infeasible the system is just `Infeasible` without a blame (a soft or auto choice of `pave`
is blamed and falls back).

## Cost and parallelism

`pave_region` on a 100 x 100 square (400 boundary nodes, about 10 000 quads), release build on
Apple silicon macOS: 1.1 s through the triangle route (unequal opposite counts), 0.18 s as a
structured grid (equal counts); the 70c7242 code measured 1.7 s on the same machine and the
earlier figure of 0.12 s came from a different platform (gate: under 2 s, asserted in
`acceptance_11_performance_100_by_100`; the number only binds in release builds, and a run next
to other heavy tests can exceed it). Time through `Driver::mesh` is recorded in the report
only (decision 12: it depends on the W3b bulk commit). Buffers are flat `Vec`s (triangle SoA, CSR
dual graph with at most three neighbours per triangle); point location is a walk from the
neighbouring triangle, never a global scan. The kernel is sequential on purpose (determinism
rule: bit-identical for 1 and N threads); parallelism comes from the driver running faces of
one wave concurrently.

## Quality (measured, see `W3_REPORT.md`)

Planar and analytic fixtures give lattices (scaled Jacobian 1.0 on rectangles and L-shapes,
cylinder lateral face); disks and annuli have irregular nodes at the centre
(minimum 0.5-0.8, mean 0.93-0.97).

## Known limitations

- **Acute corners (< ~90 degrees) and coarse unequal boundaries.** A triangle region fails the
  default placement in nearly all cases, the ladder recovers part of them (surveys in
  `W3_REPORT.md`); rectangles, L and U polygons almost never fail (2 of 450). A failure is typed (`Geometry`) and
  the production fallback `trimesh` takes over. A corner rule (a corner node with a small
  interior angle owns one quad, with the far corner of the rhombus placed explicitly) is the W3b
  fix. The milestone never asks `pave` for a triangle: V2's triangle faces are sweep
  source/target (ruling I3).
- Quality is that of a recombined triangulation plus a Laplacian: irregular nodes where rows
  meet are not cleaned up (valence cleanup is W3b).

## Stretch status

S1 (holes: `rectangle_with_holes`, `annulus(1, 2, 32)`, kernel and scheme) done, S2 (graded
boundary) done, S3 (varying residual metric) measured and reported, not asserted.

## Deferred to W3b

Embedded-member conformance, pole/degenerate-edge faces, blossom *minimum-weight* optimality,
valence cleanup (diagonal collapse, valence-2 removal), Winslow/untangle (smoothing is a
separate crate), anisotropic metric from `SizeField::metric`, intra-face parallelism,
cross-field alignment, auto-selection of `pave`, a shared 2-D CDT crate (the CDT is private and
duplicated with `sm-scheme-tri` for W3a), the acute-corner rule.
