# sm-scheme-tri

Triangle surface schemes for smart-mesh: `trimesh` / `tridelaunay` (constrained Delaunay plus
Ruppert/Chew refinement, one algorithm under two names) and `triadvance` (advancing front, the
fallback of `trimesh`). Layer L3: depends on the contract crates, `sm-math` and nothing else;
the tet mesher reaches these triangles only through `GenCtx::surface_patch(face)`.

## Positioning

Similar to Shewchuk's Triangle (constrained Delaunay + Ruppert/Chew refinement in `-Y` mode, no
Steiner points on boundary segments) and to Gmsh's 2D Delaunay in parameter space
(`meshGFace`), with Cubit's `trimesh` / `tridelaunay` / `triadvance` names; differs in four ways,
giving a face mesher that is conforming by construction, exact, thread-independent and honest
about what it guarantees.

1. **The boundary is a fixed, shared set of `NodeKey`s** from the curve schemes. Neighbouring
   faces and the tet mesh on top conform by construction; this scheme never adds, moves or splits
   a boundary node (decision 12: no boundary Steiner points in W3a).
2. **Every orientation and in-circle decision is an exact `sm-math` predicate** in a
   power-of-two-normalised chart, and the output is bit-identical for any thread count.
3. **Quality is measured in 3-D on the surface** with the metric `J^T M J` of the size field, and
   the face's pcurves give seams and poles (pole fans).
4. **An honest, reported guarantee**: the angle bound holds where the fixed boundary allows it;
   otherwise the driver gets a `Quality` failure with the offending nodes instead of a silent
   bad mesh.

## Guarantee statement (the tri -> tet contract of decision 12)

The boundary is fixed. The Ruppert/Chew angle bound holds only where boundary spacing is
compatible with `h` and input corners are at least 60 degrees. Elsewhere the scheme reports the
achieved angle and either accepts (above the gate `min(fail_angle, 0.9 * smallest input corner
angle)`) or returns `FailureKind::Quality` so the driver can fall back. The documented minimum is
**20 degrees where the fixed boundary allows it (not 25)**; tetmesh calibrates against 20 degree
input and accepts any valid closed surface. The chart is constrained-Delaunay (exact) with
3-D-measured quality. It is exactly Delaunay-and-isometric on plane and cylinder faces and
approximately so on other curved faces.

## Pipeline

```text
GenCtx --loops, loop_uvs--> FaceChart::lift --> Pslg --> Cdt::new --> refine --> optimize --> gate --> Patch
                                        (trimesh)                                              
                                        \-- front::advance (triadvance) -----------------/
```

### Kernel (`cdt`, pure 2-D, no topology)

* **Layout.** Structure of arrays: `Vec<[u32; 3]>` vertices, neighbours (`nbr[t][i]` is across the
  edge opposite vertex `i`) and 3-bit edge codes (`0` ordinary, `1` embedded, `2` boundary),
  always counter-clockwise, slots recycled through a LIFO free list. No per-triangle heap
  allocation; the cavity buffers are reused.
* **Insertion.** Bowyer-Watson in Hilbert order of the coordinates quantized on a fixed `2^-40`
  grid of the (power-of-two enlarged) bounding box. The convex hull is closed with *ghost*
  triangles (vertex `GHOST`), so the hull needs no special cases; a ghost "contains" a point
  strictly beyond its hull edge (or on the open edge). Point location is a visibility walk with
  exact `orient2d` and a scan fallback if it cycles.
* **Segments.** Recovered in the order of the Hilbert ranks of their end points (so the result
  does not depend on the input order) by Sloan flips; a segment that passes exactly through a
  vertex is split there (collinear overlapping segments work); a crossing with a constrained edge
  is `TriError::SelfIntersection`. The segment is constrained **before** the Lawson
  legalization of the new edges, otherwise legalization can flip it away again (a bug the random
  tests found).
* **Inside.** Parity flood fill from the ghosts: a Boundary segment toggles in/out, an Embedded
  one does not, so holes and islands fall out. A conflict means an open loop
  (`TriError::Inconsistent`). Without any Boundary segment everything is kept (the hull).
* **Exactness.** Only `orient2d` / `incircle` decide structure; ties are "no flip". Coordinates
  outside `[2^-100, 2^100]` are refused (`OutOfRange`). Float arithmetic appears in
  circumcenter positions, quality measures (3-D lengths, `sqrt`) and the encroachment filter
  (`dot < -1e-9 |pa||pb|`, where a point on the diametral circle does not encroach); none of
  them can produce an invalid triangulation because every new triangle is re-checked with exact
  `orient2d`.
* **Determinism.** Insertion order, segment order, heap tie-breaks (`(badness bits, Reverse(creation
  id))`), queue order and grid cells are integer keys; there is no `HashMap`/`HashSet` and no
  parallel reduction. The only libm calls that can influence a decision are `sin` for the
  angle threshold and `atan2` in the reported minimum angle; they matter only when a value sits
  exactly on the limit and are platform-stable on one platform.

### Refinement

A triangle is *bad* if its 3-D minimum angle is below `min_angle` or its 3-D circumradius is above
`size_factor * h(centroid) * 0.67` (`0.67 ~ 1.15 / sqrt(3)`). In metric mode (anisotropic size
field) lengths are measured through `A` with `|A e|^2 = e^T M e` and `h = 1`. Bad triangles are
served worst-first from a `BinaryHeap` keyed on `(badness bits, Reverse(creation id))`;
stale entries are detected by the creation id. The circumcenter (of the local surface metric
when the chart is not isometric, `Geometry2::chart_metric`) is inserted by Bowyer-Watson with
constrained edges as walls:

* it is **rejected** (the triangle is left as it is, Triangle's `-Y`) if the walk to it crosses a
  constrained edge or leaves the region, if it lies on a constrained segment, if it encroaches a
  constrained segment of the cavity boundary, or if the cavity is not star-shaped around it;
* a triangle whose circumradius is below `0.02 h` is never split. Without segment splitting this
  is what stops the cascade around an input corner that is too sharp for the angle bound (it
  fired on the pole rings of a sphere before the conformal chart existed);
* rejected triangles are reported as `RefineReport::unfixable` only if they are materially bad:
  the angle is below the limit, or the circumradius exceeds the size limit by more than 6 %.
  A triangle on a boundary segment of length `h` has its circumcenter inside the segment's
  diametral circle whenever `R < h / sqrt(2) = 0.707 h`, so `R` in `(0.67 h, 0.707 h)` cannot be
  split without a Steiner point on the segment; that window is tolerance, not failure. (The
  unit square with 10 intervals per side would otherwise report 12 "unfixable" corner triangles.)

**Ratified: `SIZE_SLACK = 1.06`.** The orchestrator ratified the 6 % size tolerance (reviewed in
the W3a Opus pass, polish round r0). It only decides what is *reported* as `unfixable`; it never
loosens the angle criterion or the insertion rules.

### Optimization

At most 4 Jacobi sweeps of Laplacian smoothing in the chart over the Steiner points only (input
points, in particular boundary, embedded and pole-ring points, never move), in point-id order; a
move is applied only if every incident triangle stays exactly counter-clockwise and the local 3-D
minimum angle (compared through its sine) does not drop. Then flips to a fixed point with a
worklist: exact `incircle` where the chart is isometric, otherwise "sum of the opposite 3-D
angles exceeds pi" evaluated as the sign of `s1 c2 + c1 s2` (no trigonometry). A flip needs an
exactly convex quad.

### Chart (`chart`)

`q = scale * z + c`, with `scale` a power of two and `c` one translation, so all loop
coordinates are plain doubles in `[1, 2]`; the kernel works on exactly those doubles. `z` is

* **linear** (`z = L^T (s (u - u0), v - v0)`, `G = J^T M J = L L^T` at the loop centroid, `M` the
  size metric there or `I / h^2`, `s = -1` for a Reversed carrier so loops stay counter-clockwise)
  for planes, cylinders and anything that is not a surface of revolution: exact where the
  first fundamental form is constant, approximate elsewhere; or
* **conformal** `z = (s (u - u0), psi(v) - psi(v0))`, `psi(v) = int sqrt(G/E) dv` (tabulated,
  cubic Hermite, inverse by bisection) for a curved surface of revolution (orthogonal parameter
  lines, `E` and `G` independent of `u`: sphere, cone, torus), detected numerically. A conformal
  chart preserves angles, so chart-Delaunay triangles and chart circumcenters are the 3-D ones to
  first order. **This was added after measurement**: with the single constant-metric chart of the
  plan the sphere zone met 20 degrees, but the full sphere with 24 seam intervals refined without
  end (the 3-D aspect of chart-isotropic triangles near the poles is `cos(v)` whatever their
  size, so refinement can never fix them). With the conformal chart the same face needs 1105
  points and reaches 29 degrees.

Boundary uv come from `GenCtx::loop_uvs` (pcurves first), with a projection fallback that unwraps
periodic parameters against the previous node; a face without a surface carrier uses its Newell
plane. A loop whose implicit closing segment is much longer than its other steps is
`TriError::OpenLoop`. Seam duplicates are distinct chart points with one `NodeKey`; the patch
collapses them and a triangle that repeats a key is `FailureKind::Geometry`.

### Poles and the pole-fan rationale

A collapsed iso-line (a degenerate coedge, flag `DEGENERATE`) is a whole edge in the chart but a
single node in 3-D. Meshing across it would give triangles with the pole node twice, so the pole is
removed from the chart: a **ring** of `m` chart points replaces the pole node and the pole is joined
to the ring by **`m` explicit fan triangles `(pole, ring_i, ring_{i+1})`** emitted outside the
kernel (`m = 6` for a full revolution, proportional to the span otherwise). The ring lies on
the iso-line through the nodes next to the pole, so the fan triangles are equilateral on a flat
patch. The seam edge `(pole, s1)` must be a mesh edge, hence **`s1`, the seam node next to the
pole, is itself a ring node**; the other `m - 1` ring nodes are face-owned
(`NodeKey::new(face, i)`, `Param::Surface`). This is why the plan's "ring of 6 face-owned nodes"
is 5 face-owned nodes plus the seam node. Ring nodes are input points of the kernel, so
`optimize` never moves them. Result: exactly 6 triangles at each pole, three distinct keys in every
triangle, and the seam stays a conforming edge. Hemispheres and apexes without a collapsed
coedge, and spheres so coarse that the two rings collide, are `Unsupported`. The plan's rule
"`r < 2h` is `Unsupported`" is replaced by this geometric test (ring collision or crossing,
neighbours not on one iso-line, ring not closing): the ring radius is the seam's first interval,
not `h`, so `r < 2h` is not the right threshold. The plan's rule is not evaluated anywhere.

### Patch assembly and failure mapping

Interior nodes are `NodeKey::new(face, i)`, ring nodes first and then the Steiner points in
insertion order, `Param::Surface([u, v])` wrapped into the principal domain; boundary nodes are
`PatchBuilder::refer(key)`; triangles are `ElemShape::Tri`, counter-clockwise in the chart (face
normal oriented). `TriError` maps to the driver: geometry and lift errors to `Geometry`, an
unsupported pole case to `Unsupported`, point budget and quality to `Quality`
(`GenerateFailure::fallback(..).with_nodes(worst triangle)`). The gate is
`min_angle_3d >= min(fail_angle, 0.9 * smallest input corner angle)` and a positive normal against
the face normal for every triangle, **pole-fan triangles included** (measured in 3-D from the
lifted ring and the pole; a fold of the lift is a `Quality` failure).

### Advancing front (`front`, `triadvance`)

The front is the set of directed boundary segments with the unmeshed region on the left (embedded
segments contribute both directions; boundary loops must be oriented, which the chart
guarantees). The shortest front edge (3-D length, ties by point ids) is closed with the **ideal
point** (isosceles apex at distance `h` from both ends), snapped to an existing node within
`0.6 h` when that gives a valid triangle, shrunk toward the edge (0.8, 0.6) if blocked, or
replaced by the best node within `1.5 h` (3-D minimum angle) when it cannot be placed. Every
candidate is checked with exact predicates: counter-clockwise, no front edge crossing, touching or
overlapping its sides (a uniform `BTreeMap` grid of front edges), no other node inside, and the
ideal point keeps `0.45 h` clearance from front edges. Isolated embedded points are inserted into
the finished triangulation. The result is a `Cdt`, so `optimize` applies. The iteration cap is
`20 * (2 * boundary segments + ideal triangle count)`; hitting it (or a front that no edge can
close) is `TriError::Stuck`. No angle guarantee; the gate decides.

## Performance (release, single thread, record only)

A planar square of 117 634 triangles (200 intervals per side): kernel 0.98 s (construction
0.002 s, refinement 0.21 s with 58 418 Steiner points, optimize 0.76 s of which 0.1 s flips),
through the driver 2.0 s including planning, curve schemes and the commit. Target < 3 s, not gated.

## Known gaps

See `W3_REPORT.md`. In short: no off-center insertion (Ungor), no boundary-layer / anisotropic
refinement beyond a constant metric, no parallelism inside one face (the driver parallelises
faces), hemispheres and rim-winding faces need a model-level partition, chart-based refinement is
approximate on non-revolution curved faces with strong anisotropy.
