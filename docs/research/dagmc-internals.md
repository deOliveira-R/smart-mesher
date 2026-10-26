# DAGMC internals: what it actually does (source-verified)

Purpose: give sm-rt honest design claims against DAGMC. Every claim below cites
source (file, function, line numbers at the commit listed) or a paper. Anything not
checked in source or a primary paper is marked **unverified**. Statements marked
**analysis** are my inferences from the verified facts.

Sources read (shallow clones, Oct 2026):

| Repo | Commit | Date |
|---|---|---|
| MOAB (bitbucket.org/fathomteam/moab) | `a5ae4bc75dca` | 2026-10-22 |
| DAGMC (github.com/svalinn/DAGMC) | `3b46ba3c9708` | 2026-09-03 |
| OpenMC (github.com/openmc-dev/openmc) | `c6301133b5c5` | 2026-10-20 |
| double-down (github.com/pshriwise/double-down) | `437ad209eac5` | 2026-08-28 |
| XDG (github.com/xdg-org/xdg) | `83efb306e152` | 2026-10-25 |

Line numbers below are for these commits.

---

## 0. Summary for the impatient

- **The user is right.** `ray_fire(V, …)` traverses only the OBB tree of volume V.
  That tree is built by joining V's *bounding-surface* trees, so DAGMC never touches
  triangles that don't bound the current volume. It does skip most of the model.
- **The weak case is a volume with a huge boundary.** The implicit complement, or an
  explicit coolant/moderator volume around N pins, gets one tree over *all* its
  bounding surfaces, e.g. every pin's outer surface. That is still O(log T) per
  query, with T the number of boundary triangles. The constant is large, though:
  OBB transforms, unordered child traversal, nodes stored as MOAB entity sets.
- **Point location is the real weak spot in practice.** OpenMC never calls
  `find_volume`. Its `Universe::find_cell` loops linearly over every DAGMC cell, each
  doing a bounding-box check and then a `point_in_volume` ray fire. MOAB's
  global-tree `find_volume` exists, but DAGMC's default setup never builds the global
  tree, so without double-down it falls back to a linear loop too.
- **Crossing a surface costs O(1) topology** (`next_vol`: the parent sets of the hit
  surface). The next `ray_fire` then starts again from the new volume's tree root.
  There is no spatial coherence beyond the facet-exclusion "RayHistory".
- **Measured gaps:** native MCNP5 was 9.1× faster than DAG-MCNP5 on a water sphere
  (M&C 2017). In OpenMC with MOAB, CSG was about 5–13× faster in particles/s on FNG
  and ATR. Embree via double-down gives 5.5–6× faster rays and 1.5–2× more
  particles/s. Even with Embree, CSG stays about 3–6× faster (CiSE 2022).
- **Robustness is a real strength.** Smith (2011) ran 10 sealed production models for
  30 CPU-days each and lost zero particles. That came from a watertight sealing
  tool plus a tolerance-free tracking algorithm: Plücker test, previous-facet
  exclusion, and a piercing/glancing check at edges and vertices.
- **Delta tracking:** I found no DAGMC, MOAB, OpenMC or XDG support.
- **Big competitive note:** XDG (DAGMC lineage, Shriwise et al., active in Oct 2026)
  already walks tet and hex **volume elements** face-to-face using Plücker tests on
  element faces. It uses an Embree BVH for point location and is building a GPU
  backend (GPRT/Vulkan). "Walk a volume mesh cell by cell" is therefore not new in
  this community. Our claims must rest on polyhedral cells, exact analytic faces,
  instancing and the cost constant, not on the walking idea itself.

---

## 1. Acceleration structure (MOAB `GeomTopoTool` + `OrientedBoxTreeTool`)

### 1.1 How trees are built

**Surface trees.** `GeomTopoTool::construct_obb_tree(eh)` (`src/GeomTopoTool.cpp`
403–493). For a surface set (dim 2) it collects the surface's triangles and calls
`obbTree->build(tris, root)`. It then tags surface↔root (`set_root_set`, 495–510).

**Volume trees.** For a volume set (dim 3), the same function collects the child
surface sets, builds any missing surface tree, and calls
`obbTree->join_trees(surf_trees, root)` (lines 456–486). A volume tree is therefore a
binary tree **over the volume's surface trees**: each surface tree is a subtree. The
same surface tree is shared by the two volumes on either side of the surface. No
triangle is stored twice.

**`construct_obb_trees(bool make_one_vol = false)`** (`GeomTopoTool.cpp` 552–586,
default argument in `moab/GeomTopoTool.hpp` 165) builds every surface tree and every
volume tree. Only when `make_one_vol` is true does it also join *all* surface trees
into one global tree (`oneVolRootSet`, line 583).

**DAGMC never asks for the global tree.** `DagMC::setup_obbs()`
(`src/dagmc/DagMC.cpp` 193–206) calls `GTT->construct_obb_trees()` with no argument
when built without double-down. **Verified:** a stock DAGMC+MOAB build has no global
surface tree.

**Implicit complement (IC).** `GeomTopoTool::setup_implicit_complement` and
`generate_implicit_complement` (1644–1752) create a volume set. Every surface with
exactly one parent volume becomes a child of the IC, with the IC on the empty side of
the sense tag. Both `GeomQueryTool::initialize()` (674–683) and `DagMC::init_OBBTree()`
(672–692) set up the IC **before** building trees. The IC therefore gets an ordinary
volume tree: the join of every one-parent surface tree. `DagMC::create_graveyard`
(379–585) adds the graveyard box surfaces to the IC and then rebuilds the IC tree
(`remove_bvh(ic, true)`, then `build_bvh(ic)`, lines 565–577).

**Build heuristic for triangles** (`OrientedBoxTreeTool::build_tree`, 204–328):

- Default `Settings`: `max_leaf_entities = 8`, `max_depth = 0` (unlimited),
  `worst_split_ratio = 0.95`, `best_split_ratio = 0.4` (lines 72–74).
- Fit the box (`OrientedBox::compute_from_2d_cells`, `src/OrientedBox.cpp` 309–323).
  Axes are the eigenvectors of the area-weighted covariance of the triangles
  (`covariance_data_from_tris` 267–307, `compute_from_covariance_data` 325–351).
  Extents come from projecting all vertices (`box_from_axes`, 166–227). The
  axes are then sorted so that `col(0)` is the shortest and `col(2)` the longest
  (swap code at 191–210).
- Split with a plane through the box centre, normal to a box axis. Triangles are
  classified by centroid (`split_box` 165–202). Axes are tried longest-first
  (`for axis = 2 .. 0`), stopping at the first split with
  `|nL − nR| / n ≤ best_split_ratio`. A node is split only if the best ratio is below
  0.95. This is a **median-plane/centroid split with no SAH**. Each node is a
  leaf with at most about 8 triangles, unless no acceptable split exists.

**Join heuristic for surfaces** (`build_sets`, 340–467):

- The box is fitted from the summed covariance data plus all vertices of the
  surfaces.
- The split loop is `for (axis = 0; axis < 2; ++axis)`, which tries **only the two
  shortest axes**. It never tries the longest axis (contrast with `build_tree`).
  **Analysis:** this looks like a quirk or bug that worsens upper-tree quality for
  elongated volumes.
- If neither split separates the sets, it alternates assignment arbitrarily
  (437–449).
- A set list of size 1 makes the surface's own tree root the leaf node (382–386).
- The upper tree is therefore about log2(S) deep, S being the number of bounding
  surfaces. The surface trees below it are about log2(T_s/8) deep.

**Storage.** Every tree node is a MOAB **entity set**. Boxes are stored as a tag on the
set (`tagHandle`, `tool->box(node)`). Children come from `get_child_meshsets`, and
leaf triangles from `get_entities_by_handle`, followed by connectivity and coordinate
lookups for each triangle (`RayIntersectSets::leaf`, 989–1034). The OBB stores a
centre, 3 axes and 3 half-lengths in double precision (`OrientedBox.hpp`).
Shriwise et al. (CiSE 2022, §II.A) write: "the use of double-precision values and
added storage of the oriented boxes' local axes in MOAB prevents efficient use of SIMD
operations". They hypothesise that the poor thread scaling of the MOAB kernel on
ATR/ITER comes from "thread contention in MOAB's data structures" (§III.C).

### 1.2 Traversal: `ray_intersect_sets`

`OrientedBoxTreeTool::ray_intersect_sets` (1036–1116) runs `preorder_traverse`
(498–568) with the `RayIntersectSets` operator (886–1034).

- **No near-first ordering.** `preorder_traverse` pushes `children[0]` and then
  `children[1]` onto a LIFO stack (544–550). Child order is fixed by build order, not
  by ray distance. Pruning comes only from (a) `OrientedBox::intersect_ray` failing,
  and (b) the search window shrinking as hits are registered. The registration
  context rewrites the window pointers (see §2.1). Compare `closest_to_location`
  (1118–1256), which *does* push the closer child last so it pops first (1185–1195).
- **Box test** (`OrientedBox::intersect_ray`, `OrientedBox.cpp` 496–616):
  1. A bounding-sphere reject on distance to the line.
  2. A window reject: `|dist_s| − max(nonneg, −neg) > outer radius`.
  3. An inner-sphere accept.
  4. Otherwise transform the ray into box coordinates (3×3 matrix–vector, twice) and
     run a slab/face test.

  The tolerance `reps` (= `numericalPrecision`, default 1e-3) inflates the box.
- **Search window** (`IntersectSearchWindow`, `OrientedBoxTreeTool.hpp` 45–55) is a
  pair of pointers: `first` = nonnegative limit, `second` = negative limit. The box
  test and the triangle test both honour it.
- **Surface context.** When the traversal enters a node that contains a surface set
  (the root of a surface subtree), `visit` records it as `lastSet`. It also calls
  `update_orient`, which turns the surface→volume sense into a desired Plücker
  orientation (946–971; `GQT_IntRegCtxt::update_orient` in `GeomQueryTool.cpp`
  205–241). Each leaf triangle is tested with
  `GeomUtil::plucker_ray_tri_intersect(..., search_win.first, search_win.second, surfTriOrient, &int_type)`.
  Hits go to `int_reg_callback.register_intersection`.

### 1.3 Exactly which triangles does `ray_fire` from inside V traverse?

**Verified:**

- The traversal starts at `get_root(volume)` (`GeomQueryTool::ray_fire`, line 768).
  That root is the join of V's bounding-surface trees.
- Only triangles of surfaces that bound V can be visited. Among those, it visits only
  leaves whose OBB (inflated by 1e-3) the ray hits within the current window.
- With orientation screening (`ray_orientation = 1`, the default exit-only), the
  Plücker test exits early on wrong-facing triangles after 1–2 edge tests.

**Big multiply-connected volume.** Take coolant around hundreds of pins, whether it is
an explicit volume or the IC. Its tree is the join of every bounding surface: all pin
clad outer surfaces, the box walls, and so on. **Analysis:**

- The upper tree is about log2(S) levels over the S surfaces. A ray through a pin
  lattice intersects the OBBs of every pin surface near its path. The upper-level
  OBBs of a lattice are large and overlap little, but the ray still has to descend
  into each candidate pin's subtree until a hit shrinks the window.
- Because traversal is unordered, a far pin can be explored before the near one.
  Early in the traversal the window is large (the physics limit `dist_limit`, or
  `DBL_MAX`), so pruning is weak until a hit is found.
- Cost is still logarithmic in the boundary-triangle count, not linear. The constant
  grows with how many pin OBBs lie along the ray.

Shriwise's CiSE Table I shows the MOAB kernel taking 14.6 µs/ray on a large ITER IC
volume, against 12.6 µs on a sphere (i7-8650u). Large IC volumes are therefore not
catastrophic for the OBB tree.

---

## 2. Queries (`moab/src/GeomQueryTool.cpp`, DAGMC wrappers in `src/dagmc/DagMC.cpp`)

The DAGMC wrappers (`DagMC::ray_fire` 779–788, `point_in_volume` 790–796,
`test_volume_boundary`, `point_in_volume_slow` 808–812, `find_volume` 816–820,
`closest_to_location`, `get_angle` 859–863, `next_vol` 865–868) forward 1:1 to
`GeomQueryTool`. With `-DDOUBLE_DOWN` they forward to
`double_down::RayTracingInterface` instead (`DagMC.hpp` 680–686). Defaults:
`overlap_tolerance = 0`, `numerical_precision = 0.001` (`DagMC.hpp` 78;
`GeomQueryTool.hpp` 122–128).

### 2.1 `ray_fire` (GeomQueryTool.cpp 728–902)

1. `dist_limit` is `user_dist_limit` (the physics limit, i.e. distance to collision)
   if positive, else `DBL_MAX`.
2. **Window:**
   - `neg_ray_len = −numericalPrecision` if `overlapThickness == 0`, else
     `−overlapThickness`.
   - `nonneg_ray_len = max(dist_limit, −neg_ray_len)`.
   - So it always searches a small distance *behind* the origin (1e-3 by default).
3. It builds `GQT_IntRegCtxt` with `tol = numericalPrecision` and `minTolInt = 0`. It
   passes `rootSet = &root`, `geomVol = &volume`, `senseTag`, `desiredOrient = &ray_orientation`
   (default +1 = exiting only), and `prevFacets = history->prev_facets`.
4. It calls `ray_intersect_sets`. Because both window pointers are non-null, the
   context runs in **Mode 1** (`add_mode1_intersection`, 487–523). It keeps the
   closest nonnegative hit and at most one negative hit, and it tightens
   `search_win.first` to the current closest nonnegative hit. That tightening is the
   main pruning mechanism during traversal.
5. **Intersection filtering** (`register_intersection`, 418–463):
   - Reject if the facet is in `prevFacets` (linear `std::find`, 243–246) or in the
     neighbourhood of an already accepted hit (`in_neighborhoods`, 248–259).
   - For an edge or vertex hit (`int_type != INTERIOR`), call
     `tool->get_close_tris(hit_pt, tol, rootSet, ...)`. That is a **second tree
     traversal**, `sphere_intersect_triangles` over the whole volume tree
     (`OrientedBoxTreeTool.cpp` 845–880). It then runs
     `edge_node_piercing_intersect` (274–416) on the triangles sharing that edge or
     vertex. The hit is accepted only if every adjacent triangle normal (with sense)
     has the same sign against the ray, i.e. piercing rather than glancing.
   - The accepted hit's neighbourhood is stored, so the same crossing is not counted
     twice through the adjacent triangle.
6. **Overlap / behind-origin logic** (846–872). If a negative-distance hit exists,
   find the volume on the other side (parents of the surface) and run a full
   `point_in_volume(nx_vol, point, dir, history)`. If the point is inside that
   volume, return the negative hit as the exit with distance clamped to 0. This
   handles both overlaps and "the particle numerically already crossed".
7. If there are no hits, `next_surf = 0`, which the caller treats as lost
   (841–848, 875–882).
8. Otherwise `history->prev_facets.push_back(hit_facet)`.

**`RayHistory`** (`GeomQueryTool.cpp` 685–726) is a `std::vector<EntityHandle>` of
facets already crossed on this straight-line track. Operations:
`reset`, `reset_to_last_intersection` (keep only the last facet, used after a
reflection), `rollback_last_intersection` (used when a "streaming" step ended in a
collision without reaching the surface), `in_history`, `add_entity`.

Usage in callers:

- **DAG-MCNP** (`src/mcnp/mcnp_funcs.cpp` 557–620) resets the history on a new
  particle or a direction change. On a same-direction re-fire it keeps the history,
  rolling back if no surface was visited.
- **OpenMC** (`src/dagmc.cpp` 824–885) resets when `u != last_dir` or
  `on_surface == SURFACE_NONE`. It also resets on any CSG-surface crossing
  (`geometry.cpp` 284–292).

The history is a **topological** replacement for the classic "discard distance"
nudge. Smith (2011) found the discard tolerance was the main source of lost
particles (thesis §3.1).

### 2.2 `next_vol`

`GeomTopoTool::next_vol` (`GeomTopoTool.cpp` 1102–1120) gets the two parent sets of
the surface and returns the other one: O(1). OpenMC uses it directly in
`Particle::cross_surface` (`particle.cpp` 678–700, via `next_cell` in `dagmc.cpp`
1004–1020) instead of neighbour-list searching.

### 2.3 `point_in_volume` (904–1060) and `point_in_box` (1068–1093)

1. **Bounding-box pre-check** against the volume's axis-aligned bounds
   (`GTT->get_bounding_coords`). Return 0 if outside.
2. **Direction:** `uvw` if given, else **`rand()`** for u, v, w (941–951). The same
   applies in `find_volume` (1207–1212). `rand()` is unseeded relative to inputs and
   not thread-safe, so the result is non-deterministic for points on or near a
   boundary.
3. Fire one ray over the volume tree with the window `(1e15, NULL)` (Mode 2):
   - `minTolInt = 1` if there are no overlaps: keep hits within `tol` plus the
     closest beyond.
   - `minTolInt = -1` with overlaps: return all hits to infinity, which the code
     calls "expensive".
4. `boundary_case` (1493–1556) classifies each hit as entering or leaving from the
   facet normal times the sense, dotted with the ray.
   - Without overlaps, the result is decided by the nearest hit.
   - With overlaps, the code counts net crossings. A net count of 0 is "inside" for
     the IC only.
5. No hits means outside.

### 2.4 `point_in_volume_slow` (1135–1171)

A solid-angle sum over every facet of every surface, with sense. Inside if
`|Σ| > 2π`. It is O(T_V) with no tree. The algorithm is Carvalho & Cavalcanti,
Graphics Gems V.

### 2.5 `test_volume_boundary` (1095–1133)

If there is a history, `boundary_case` is run on the last crossed facet. Otherwise
the code finds the closest facet with `closest_to_location` on the volume tree and
runs `boundary_case` on it.

### 2.6 `find_volume` (1173–1285) and `find_volume_slow` (1287–1307)

1. Reject if the point is outside the IC's bounding box.
2. **If there is no global tree** (`get_one_vol_root() == 0`): `find_volume_slow`,
   a linear loop calling `point_in_volume` on every volume until one says inside.
3. **With a global tree:** one ray (random direction if none is given) with window
   `(+DBL_MAX, −DBL_MAX)`. `FindVolumeIntRegCtxt` (45–87) keeps the single nearest
   hit in |distance|, in either direction, and shrinks the window symmetrically. The
   volume is then taken from the surface sense table and the sign of the dot product
   of normal and ray.
4. No hit means `MB_ENTITY_NOT_FOUND`.

Notes:

- The DAGMC wrapper exists only for MOAB ≥ 5.3
  (`#if MOAB_VERSION_MAJOR == 5 && MOAB_VERSION_MINOR > 2`, `DagMC.cpp` 815).
  **The exact date `find_volume` was introduced is unverified** (no git history in
  the shallow clone).
- Because `setup_obbs` never builds the global tree, **stock DAGMC+MOAB
  `find_volume` = linear PIV loop**.
- double-down's `find_volume` (`src/RTI.cpp` 712–780) *does* use a global Embree
  scene (`create_global_scene`, 312–330). Its default direction is the fixed
  `(0.7071, 0.7071, 0)`, so it is deterministic. On a miss it falls back to the
  linear loop.
- **OpenMC does not call `find_volume` at all** (grep over `src/` and `include/`
  finds no hits). See §3.4.

### 2.7 `closest_to_location`

`GeomQueryTool::closest_to_location` (1309–1331) calls
`OrientedBoxTreeTool::closest_to_location` (1118–1256). That is a depth-first branch
and bound with the closer child first and pruning on `dist_sqr > best`. It works over
the volume tree, so it gives the distance to the nearest boundary triangle of V.

### 2.8 `get_angle` → `GeomQueryTool::get_normal` (1440–1484)

The normal of the last facet in the history, or, without a history, the averaged
normals of facets within `numericalPrecision` (found with `closest_to_location` with
tolerance on the surface tree).

### 2.9 The ray–triangle test (`moab/src/GeomUtil.cpp` 148–301)

`plucker_ray_tri_intersect` follows Platis & Theoharis, "Fast Ray-Tetrahedron
Intersection using Plücker Coordinates", JGT 8(4):37–48 (2003), cited in the source
comment at 189–192.

**Watertightness at shared edges.** `plucker_edge_test` (148–174) orders each edge's
endpoints canonically with `first(a, b)`, a lexicographic xyz compare (`first`, 114–146),
before computing the Plücker product, and negates the result if it swapped them. Two
triangles sharing an edge therefore compute the bit-identical edge value with
opposite sign. A ray can't slip between them, and edge and vertex hits are reported
consistently.

**Exact-zero snapping.** Values with `|pip| < 10·DBL_EPSILON` become 0. The zero
pattern of the three edge values gives `int_type` (INTERIOR, EDGE0-2, NODE0-2). That
type feeds the piercing/glancing logic in §2.1.

**Edge test formula.** It uses the edge × vertex *in world coordinates*
(`edge_normal = edge * vertexa`), not coordinates relative to the ray origin.
**Analysis:** the moment terms scale with |x|, so absolute precision degrades far
from the origin. The 10·ε threshold is absolute, not relative.

**Orientation screening.** If `orientation` is set, the test exits after the first
edge value with the wrong sign, which is cheap backface culling.

**Distance.** Computed from the barycentric point using the largest component of
the direction.

**`make_watertight`** (`DAGMC/src/make_watertight/`, `MakeWatertight::make_mesh_watertight`
at `MakeWatertight.cpp` 1783). It seals each surface's skin loops to its bounding
curves by vertex merging (point–point) and point–edge contraction (`seal_arc_pair`
420, `seal_loop` 762, `merge_skin_verts` 1701). After sealing, adjacent surfaces share
vertices on curves, so the canonical-edge Plücker test is watertight *across
surfaces* too. Smith 2011 (thesis ch. 2) describes this. Side effects: sealing raises
the triangle count and creates slivers (ATR: triangles 100 cm × ~0.001 cm), and it can
invert facets when its assumptions are violated (faceting tolerance < feature size,
and > merge tolerance).

---

## 3. Performance work since 2010

### 3.1 Baseline papers

**Wilson, Tautges, Kraftcheck, Smith, Henderson**, "Acceleration techniques for the
direct use of CAD-based geometry in fusion neutronics analysis", *Fusion Eng. Des.*
85 (2010) 1759–1765, doi:10.1016/j.fusengdes.2010.05.030. It describes the OBB-tree
acceleration. Earlier talks claim "within a factor of 2-3 of native MCNP". **The
full text was not read**, so the details beyond the abstract are unverified.

**Smith, B.M.**, *Robust Tracking and Advanced Geometry for Monte Carlo Radiation
Transport*, PhD thesis, UW-Madison, UWFDM-1391, March 2011
(fti.neep.wisc.edu/pdf/fdm1391.pdf). Read:

- Watertight sealing (ch. 2), the robust tracking algorithm (ch. 3), the implicit
  complement (ch. 4) and overlap tolerance.
- Table 3.2, 40° ITER benchmark: the robust algorithm is 2–10% faster than the
  original. With the physics limit it is 16% faster than the original without it.
  Particle rate was 9,766–11,339 particles/min with materials and about 75–81k
  particles/min voided, on a Core2 at 2.66 GHz.
- Table 3.3: 10 sealed production models, 30 CPU-days each, **0 lost particles** with
  the robust algorithm. The original algorithm, extrapolated, lost from tens to about
  31k.
- The *unsealed* 40° ITER model still had a lost fraction of 1.7×10⁻⁵ with the robust
  algorithm.

### 3.2 Signed distance fields (research prototype, not in DAGMC/MOAB source)

**Shriwise, Davis, Jacobson, Wilson**, "Particle Tracking Acceleration via Signed
Distance Fields in DAGMC", M&C 2017, Jeju (kns.org P157S02-08). A journal version is
"Particle tracking acceleration via signed distance fields in direct-accelerated
geometry Monte Carlo", *Nucl. Eng. Technol.* 49(6):1189–1198 (2017). Read the M&C
version.

What the SDF is:

- One SDF per volume: a structured (Cartesian) grid over the volume's AABB with a
  signed distance at each vertex. Interior values are positive (the paper flips the
  usual convention).
- Each grid value comes from `closest_to_location` on the volume's OBB tree. The sign
  comes from the dot product of (closest point − query point) with the facet normal,
  with `point_in_volume` as the fallback when that is ambiguous.
- Stored in MOAB's structured-mesh interface (memory-heavy, called a proof of
  concept).
- Lookup is O(1): compute the voxel index, then trilinear interpolation.
- Error bound used: **ε = √3·h**, the voxel diagonal. That conservatively removes all
  data in voxels that intersect the surface.

How it is used:

- **Next-surface query:** skip `ray_fire` if
  `SDV(p) + SDV(n) > |p − n| + 2ε`, where n is the next collision site. Both
  endpoint spheres must cover the segment.
- It can also precondition point containment and distance-to-boundary queries.

Results:

- Water sphere, R = 10 cm, 5 MeV point source, 100M histories, DAG-MCNP5:
  - native MCNP5: 2.75 min
  - DAG-MCNP5: 25.04 min (**9.13× slower than native**)
  - DAG-MCNP5 with SDF: 7.15 min
  - That is **3.5× faster** than plain DAGMC, still 2.65× slower than native.
  - Utilization (rays avoided / next-surface queries) was 0.97, with identical
    tallies. The grid step in Fig. 3 was 0.5 cm; the step used for Table I is not
    stated explicitly (**unverified**).
- Utilization falls sharply below 0.1 g/cc and goes to zero in void, so the SDF adds
  pure overhead in void and streaming volumes.

Status: the source has no trace of an SDF (grep for "signed distance" or "sdf" over
DAGMC `src/` and MOAB `src/` finds nothing relevant). **It is not in production
DAGMC.**

Thesis: Shriwise, *Geometry query optimizations in CAD-based tessellations for Monte
Carlo radiation transport*, PhD, UW-Madison, 2018
(asset.library.wisc.edu/1711.dl/NNODVI4XHMSOV8F). **Not read in full (unverified).**

### 3.3 double-down / Embree (mixed precision)

**Shriwise, Wilson, Davis, Romano**, "Hardware-Accelerated Ray Tracing of CAD-Based
Geometry for Monte Carlo Radiation Transport", *IEEE Computing in Science &
Engineering* 24(2):52–61 (2022), doi:10.1109/MCSE.2022.3154656. Full text is on OSTI
(purl 1909565). The ANS 2015 and 2018 transactions are earlier versions.

Method:

- Embree builds a single-precision AABB BVH over user-defined primitives, with each
  primitive box dilated.
- Leaf intersection uses the **same MOAB Plücker test in double precision**.
- The single-precision `tfar` is updated so Embree's ordered traversal prunes.
- Dilation in the paper is ε = 5×10⁻³ (§II.D). In the code
  (`RTI.cpp` `createBVH` 181–310) it is
  `bump = 2√3·|diag(AABB_V)|·10^(−FLT_DIG)`, about 3.5×10⁻⁶ × the volume diagonal.
  (The docs formula has √3 without the factor 2.)
- `RTC_SCENE_FLAG_ROBUST`, `RTC_BUILD_QUALITY_HIGH`.
- One Embree scene per volume, with one geometry per surface. Surfaces are
  re-attached per volume, so the BVH is built over the volume's triangles; a surface
  BVH is not shared as an instance. Plus one global scene over all geometries.

Robustness differences from GQT (verified in `double-down/src/MOABRay.cpp` 1–110,
`primitives.cpp`, `RTI.cpp` 940–1060):

- Kept: history (prev-facet) exclusion and orientation culling (`backface_cull` /
  `frontface_cull`).
- **Dropped:** GQT's edge/node piercing-vs-glancing neighbourhood logic.
- The negative-direction (overlap) ray is fired only if `overlap_thickness > 0`.
- `point_in_volume` takes only the nearest hit, with a fixed default direction of
  (0.5, 0.5, 0.5).

Measured, ray tracing only (Table I, 10M random rays from inside, faceting tolerance
1e-4 cm):

| Model | MOAB (µs/ray) | Embree (µs/ray) | Speedup |
|---|---|---|---|
| Sphere | 8.5–12.6 | 1.3–2.0 | **6.0–6.5×** |
| ITER IC volume | 12.4–14.6 | 2.2–2.5 | **5.5–5.9×** |

Measured, OpenMC serial (Table II, particles/s, three Xeon/i7 CPUs). The table layout
in the extracted PDF text is ambiguous; this is my reading:

| Model | CSG | MOAB | Embree |
|---|---|---|---|
| FNG (1M tris, 1,063 surfaces) | 4,221–6,659 | 533–1,073 | 1,077–1,567 |
| ATR (5M tris, 28k surfaces) | 3,139–5,458 | 407–525 | 593–941 |
| ITER B-lite (50M tris, 70k surfaces) | n/a | 1,867–2,885 | 3,747–5,626 |

- The paper's own text: Embree raises particle rates "by factors of 1.5–2×". "The gap
  in performance between CSG and CAD-based geometry for serial simulations remains
  significant."
- Threading: MOAB scales poorly on ATR and ITER. Embree scales like CSG.
- Identical numerical results between MOAB and Embree.

### 3.4 OpenMC's DAGMC integration (`openmc/src/dagmc.cpp`)

**Cell search.**

- `DAGUniverse::find_cell` (570–580) calls `Universe::find_cell` (`universe.cpp`
  40–60). That is a **linear loop over the universe's cells**, calling
  `cell->contains(r, u, surf)` on each.
- `DAGCell::contains` (887–898) calls `point_in_volume(vol, r, result, u)`: an AABB
  pre-check and then one ray fire.
- IC cells are appended last (`cell.cpp` 1240–1275). In a non-root DAGMC universe the
  IC is assigned *without testing* if no other cell matches.
- `UniversePartitioner` is only built when a universe has more than 5 CSG
  `SurfaceZPlane`s (`geometry_aux.cpp` 155–172), so DAGMC universes are never
  partitioned.
- **Cost:** O(N_cells × PIV) per source site, and per entry into a DAGMC universe
  instance from CSG.

**Distance.**

- `DAGCell::distance` (824–885) calls `ray_fire(vol, r, u, hit, dist, &history)`.
- Note that **no `user_dist_limit` is passed**, so OpenMC does not use the physics
  limit. **Analysis:** the window starts at DBL_MAX, so pruning waits for the first
  hit.
- A miss in a non-IC volume, or in the IC of the root universe, means
  `mark_as_lost`.

**Crossing.** `next_cell` uses `next_vol` (`dagmc.cpp` 1004–1020;
`particle.cpp` 678–700), which is O(1).

**Reflection.** `DAGSurface::reflect` (964–972) gets the normal from the last
history facet.

**DAGMC universes inside CSG** (lattices of DAGMC universes):

- Supported: `DAGUniverse` objects are universes and can fill CSG cells and lattices.
- The history is reset on any CSG-surface crossing because "A CSG crossing can move
  the particle into another instance of the same DAGMC universe"
  (`geometry.cpp` 284–292).
- Each instance re-enters through the linear `find_cell`.
- Pure DAGMC has no instancing. OpenMC's CSG lattice is the instancing layer.
- `particle.cpp` 802 comments that "a dagmc model … has exactly one universe" (the
  reflection path).

### 3.5 Lost particles and robustness in practice

- Smith 2011: zero lost particles on sealed models (above).
- OpenMC Discourse threads show lost particles still happen in practice. They usually
  come from geometry defects (non-watertight models, overlaps) or models without a
  graveyard ("No intersection found with DAGMC cell …"). Examples:
  openmc.discourse.group/t/issue-with-lost-particles-from-dagmc-geometry/2663 and
  /t/no-intersection-found-with-dagmc-cell/1992. **Not rates, anecdotal.**
- `check_dagmc_root_univ` warns when there is no graveyard (`dagmc.cpp` 990–1000).

### 3.6 GPU / hardware ray tracing

**Salmon & McIntosh-Smith**, "Exploiting Hardware-Accelerated Ray Tracing for Monte
Carlo Particle Transport with OpenMC", PMBS @ SC19.

- They ported OpenMC to an NVIDIA GPU, getting up to 9.8× (CSG) and 13× (approximate
  triangle mesh) over a 16-core CPU.
- RT cores add 2–20× more, with larger gains on more complex geometry.
- **This is not DAGMC:** they used their own OptiX triangle-mesh geometry, and I see
  no evidence it was merged into OpenMC (**unverified**). Single-precision/robustness
  details: **unverified**.

**XDG** (github.com/xdg-org/xdg, "Accelerated Discretized Geometry", DAGMC lineage,
Shriwise et al.). Verified in source and docs:

- Mesh backends: MOAB and libMesh. Ray-tracing backends: Embree on CPU, GPRT (Vulkan,
  vendor-agnostic) on GPU.
- GPRT uses `double3` vertices and rays with `float3` AABBs, i.e. the same
  mixed-precision idea (`include/xdg/gprt/shared_structs.h`).
- **Volume tracking on tet/hex meshes:**
  - `MeshManager::next_element` (`src/mesh_manager_interface.cpp` 165+) runs Plücker
    tests on each element face, with quads split along a canonical diagonal, skipping
    faces with `u·n < 0`.
  - `walk_elements` (117–152) follows `adjacent_element`.
  - Point→element location uses an Embree BVH over element AABBs with a zero-length
    ray, `rtcOccluded`, and a double-precision barycentric check
    (`docs/methods/element_search.rst`).
  - The docs say volumetric tracking is not yet implemented in GPRT.
- `XDG::find_volume` (`src/xdg.cpp` 115–130) is still a linear loop over volumes doing
  `point_in_volume`.

### 3.7 MOC use

Shriwise, Tramm, Davis, Romano, "Towards CAD-Based Geometry Modeling with the Random
Ray Method", PHYSOR 2020, doi:10.1051/epjconf/202124703023. DAGMC is used in the
random-ray solver; that is ray_fire per segment. **Not read (unverified).**

---

## 4. Delta (Woodcock) tracking

**Found nothing.** Grep for "delta tracking", "woodcock" and "majorant" over DAGMC,
MOAB `src/`, XDG and OpenMC `src/`/`include/`/`docs` finds no geometry-related hit.
OpenMC's only "majorant" is source sampling. A web search turned up no DAGMC
delta-tracking work.

The comparator is **Serpent 2's STL/CAD geometry**. It uses a search mesh that stores
pre-assigned cell information in mesh cells without facets (Serpent user guide,
`check_stl`). Serpent defaults to delta tracking with surface-tracking fallback.
**The interplay with STL was not verified in detail.** References: Leppänen et al.,
*Ann. Nucl. Energy* 176 (2022) 109259; Talamo, Gohar, Leppänen (2018) on adaptive
search.

**Analysis: what DAGMC would need for delta tracking.**

1. Fast global point→volume location at each virtual collision. The candidates are
   `find_volume` on a global tree, which is one global ray fire (double-down has
   this; stock MOAB has it only if `make_one_vol` is set), or the linear loop in
   OpenMC, which is unacceptable.
2. A majorant cross-section per region.
3. Consistent handling of points within ε of facets, since PIV uses random or fixed
   ray directions and has no history.

An SDF or a voxel cell-cache (Serpent-style) would make most lookups O(1). Delta
tracking also removes `ray_fire` entirely, which is where DAGMC's cost sits.

---

## 5. Cost model, weak spots, strengths

### 5.1 Per-step cost model (stock DAGMC + MOAB, as used by OpenMC/DAG-MCNP)

Notation: T_V = triangles bounding the current volume V; S_V = surfaces of V;
N_vol = volumes in the model (or the DAGMC universe); T = all triangles.

| Operation | Cost | Notes |
|---|---|---|
| Distance to boundary (`ray_fire`) | O(log S_V + log(T_V/8)) OBB visits × (sphere test + 3×3 transform + slab test) + leaves × ≤8 Plücker tests | Tree only over V's boundary. Unordered child traversal; pruning only after a hit shrinks the window. Each node visit is MOAB set/tag lookups. A linear scan of `prev_facets` per hit. **+ a sphere tree query** on every edge/vertex hit. **+ a full PIV** on every behind-origin hit. Restarts from V's root on every call: after each collision (new direction) and each crossing. |
| Surface crossing → next volume (`next_vol`) | O(1) | Parents of the surface set. |
| Point in one volume (`point_in_volume`) | AABB test + one ray fire over V's tree | Random (`rand()`) direction if none is given. With overlaps, all hits to infinity. |
| Point location, OpenMC (`find_cell`) | O(N_cells) × PIV | Linear; IC last. Hit on every source site and every DAGMC-universe entry. |
| Point location, MOAB `find_volume` | O(N_vol) × PIV (stock DAGMC: no global tree) or one ray over the global tree (O(log T)) if `make_one_vol` was set | double-down: one global Embree ray. |
| Distance to nearest boundary (`closest_to_location`) | Branch and bound over V's tree | Used by `get_angle` / `test_volume_boundary` without a history. |
| Normal at a hit | O(1) with a history (last facet) | |

Typical constants: 8.5–14.6 µs/ray (MOAB) and 1.3–2.5 µs/ray (Embree) on
sphere/ITER-IC volumes at 1e-4 cm faceting (CiSE 2022, Table I).

### 5.2 Known weak spots (verified unless marked)

1. **Large boundary trees.** IC and coolant volumes have trees over every bounding
   surface. The cost is log, but with a big constant, unordered traversal, and many
   candidate pin subtrees along a ray (analysis).
2. **Faceting count for curved surfaces.** The faceting tolerance must be small to
   conserve volume (≤ 1e-4 cm per CiSE), giving 10³–10⁵ triangles per surface. The
   ITER B-lite model has 50M triangles. Sealing adds more triangles and slivers. A
   cylinder is never exact: geometry error is bounded by the faceting tolerance, not
   zero.
3. **Restart from the root after every crossing or collision.** There is no
   persistent traversal state. `RayHistory` only excludes facets; it doesn't cache
   candidates. OpenMC also doesn't pass the physics limit.
4. **Point location over many volumes.** It is linear in OpenMC and in stock DAGMC
   `find_volume`. That matters for sources, restarts, CSG→DAGMC entry, delta
   tracking and volume calculation.
5. **No instancing in pure DAGMC.** A repeated pin is stored and faceted once per
   copy. Instancing is only available by embedding DAGMC universes in OpenMC CSG
   lattices, and each entry pays a linear `find_cell`.
6. **Memory and threading.** Nodes are MOAB entity sets with double OBBs. The CiSE
   paper attributes poor MOAB thread scaling on large models to data-structure
   contention, and earlier codes needed MPI-only runs because of memory per copy
   (CiSE Fig. 9).
7. **Determinism hazards.** `rand()` in PIV and `find_volume` when no direction is
   given.
8. **Build heuristics.** Median/centroid splits, no SAH. The surface-join split never
   tries the longest axis (`build_sets` axis < 2).
9. **Plücker in world coordinates** with an absolute zero threshold of 10ε (analysis:
   precision degrades far from the origin).
10. **SDF is research-only.** It helps only in dense, collision-dominated volumes and
    is useless in void.

### 5.3 Strengths we must not underestimate

1. **It only searches the current volume's boundary.** Per-step cost is independent
   of total model size; it is a log of the local boundary.
2. **O(1) topological crossing** via the surface→volume sense table. There is no
   neighbour search after a crossing.
3. **Tolerance-free robustness.** The canonical edge ordering makes shared edges
   bit-consistent. The edge/vertex piercing test, previous-facet history and
   `make_watertight` sealing gave zero lost particles in 10 production models ×
   30 CPU-days (Smith 2011). It needs no discard distance.
4. **Overlap tolerance.** `overlap_thickness` plus PIV on the behind-origin hit lets
   it run imperfect CAD.
5. **Implicit complement.** No need to mesh or define void. The void is just the
   one-sided surfaces.
6. **Mature ecosystem and engines.** MCNP5/6, Geant4, FLUKA, OpenMC, Tripoli4 and
   Shift (DAGMC README). Embree via double-down gives 5.5–6× faster rays with
   bit-identical results, and XDG adds GPU (GPRT) and tet/hex element walking.
7. **Orientation culling.** Exit-only Plücker tests reject wrong-facing triangles
   after one edge product.

### 5.4 Implications for sm-rt claims (analysis)

- **Don't claim "DAGMC searches the whole model".** It doesn't.
- **Defensible claims:**
  1. sm-rt has an O(F_cell) step (faces of the current cell) with no tree traversal
     and no restart-from-root. Cell-to-cell adjacency carries the state.
  2. Exact analytic faces: no faceting error and far fewer primitives for curved
     surfaces.
  3. Instancing and lattices native to the ray tracer, instead of linear `find_cell`
     on each universe entry.
  4. Point location that is O(log) or cached, with a hint from the previous cell.
  5. Determinism (no `rand()`).
  6. Flat SoA/CSR layouts rather than database entity sets.
- **The honest counterpoint:** an unstructured volume mesh walk pays per *cell*
  crossed. In a large coolant region DAGMC makes one O(log T) jump to the boundary,
  while a walk crosses many cells. sm-rt needs either coarse cells in homogeneous
  regions (polyhedral agglomeration) or a skip structure. That is the comparison to
  benchmark: segments per track × cost per segment, against one ray_fire per track.
- **XDG already walks tets and hexes** with Plücker tests on faces. Benchmark against
  XDG's `walk_elements` as well as against DAGMC `ray_fire`.

---

## References

- MOAB source: `src/GeomTopoTool.cpp`, `src/OrientedBoxTreeTool.cpp`, `src/OrientedBox.cpp`,
  `src/GeomQueryTool.cpp`, `src/GeomUtil.cpp`, `src/moab/*.hpp` (bitbucket.org/fathomteam/moab @ a5ae4bc).
- DAGMC source: `src/dagmc/DagMC.{cpp,hpp}`, `src/mcnp/mcnp_funcs.cpp`, `src/make_watertight/` (github.com/svalinn/DAGMC @ 3b46ba3).
- OpenMC source: `src/dagmc.cpp`, `src/universe.cpp`, `src/geometry.cpp`, `src/geometry_aux.cpp`, `src/particle.cpp`, `src/cell.cpp` (@ c630113).
- double-down source and docs: `src/RTI.cpp`, `src/MOABRay.cpp`, `src/primitives.cpp`, `docs/methodology/*.rst` (@ 437ad20).
- XDG source and docs: `src/xdg.cpp`, `src/mesh_manager_interface.cpp`, `docs/methods/*.rst` (@ 83efb30).
- N. Platis, T. Theoharis, "Fast Ray-Tetrahedron Intersection using Plücker Coordinates", JGT 8(4):37–48, 2003. doi:10.1080/10867651.2003.10487593
- P.P.H. Wilson et al., "Acceleration techniques for the direct use of CAD-based geometry in fusion neutronics analysis", Fusion Eng. Des. 85:1759–1765, 2010. doi:10.1016/j.fusengdes.2010.05.030 (abstract only)
- B.M. Smith, *Robust Tracking and Advanced Geometry for Monte Carlo Radiation Transport*, PhD thesis, UW-Madison, UWFDM-1391, 2011. https://fti.neep.wisc.edu/fti.neep.wisc.edu/pdf/fdm1391.pdf
- P.C. Shriwise, A. Davis, L.J. Jacobson, P.P.H. Wilson, "Particle Tracking Acceleration via Signed Distance Fields in DAGMC", M&C 2017. https://www.kns.org/files/int_paper/paper/MC2017_2017_2/P157S02-08ShriwiseP.pdf ; journal version NET 49(6):1189–1198, 2017 (https://www.osti.gov/biblio/1425791)
- P.C. Shriwise, PhD thesis, UW-Madison, 2018. https://asset.library.wisc.edu/1711.dl/NNODVI4XHMSOV8F/R/file-4a60b.pdf (not read)
- P.C. Shriwise, P.P.H. Wilson, A. Davis, P.K. Romano, "Hardware-Accelerated Ray Tracing of CAD-Based Geometry for Monte Carlo Radiation Transport", IEEE CiSE 24(2):52–61, 2022. doi:10.1109/MCSE.2022.3154656 ; https://www.osti.gov/pages/servlets/purl/1909565
- J. Salmon, S. McIntosh-Smith, "Exploiting Hardware-Accelerated Ray Tracing for Monte Carlo Particle Transport with OpenMC", PMBS @ SC19. https://www.dcs.warwick.ac.uk/pmbs/pmbs19/PMBS19/abstracts/paper3.html
- P.C. Shriwise, J. Tramm, A. Davis, P.K. Romano, "Towards CAD-Based Geometry Modeling with the Random Ray Method", PHYSOR 2020. doi:10.1051/epjconf/202124703023 (not read)
- J. Leppänen et al., CAD-based geometry type in Serpent 2, Ann. Nucl. Energy 176 (2022) 109259; Serpent user guide `check_stl` (https://serpent.vtt.fi/docs/user_guide/check_stl.html) (details unverified)
