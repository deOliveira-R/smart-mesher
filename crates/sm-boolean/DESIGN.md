# sm-boolean design

## Positioning

Similar to the Parasolid/ACIS-style boolean kernels and to Cubit's `unite`/`subtract`/`imprint`/`merge`/`webcut`
commands, `sm-boolean` differs in working on a *cell complex* (shared, two-sided faces and edges, one
volume per region) instead of on manifold shells with a per-body face list, and in being one fragment
kernel behind every operation. That gives conformal interfaces for free: a merged cap or a kept shared
face is a single `TopoFace` that both neighbouring volumes use, so a mesher sees one set of nodes on it.

## One kernel

Every op (`boolean`, `imprint_bodies`, `webcut`, `merge`) is a composition of the same stages:

1. **Pairs.** Candidate face pairs by AABB, sorted by `(id_a, id_b)`, evaluated with rayon and
   collected in index order (`pieces::candidate_pairs`).
2. **SSI.** `sm_geom::intersect_surfaces` per pair; coplanar/co-surface overlaps become 2D regions.
   Curve-curve intersection is private (`cc.rs`: line/line closed form, others sampled + golden
   section).
3. **Clip.** Each curve is clipped to both faces (`faceclass`, a winding test that reads pole
   pcurves). The result is a list of `Piece`s with a group id and a share mode.
4. **Apply.** Serially in sorted order (`arrange`): vertex registry (BTreeMap grid), `split_edge`,
   then per face a worklist of `Chord/Chain/ClosedLoop/ChordAt/ChainAt` splits and `bridge_loops` for
   holed faces. Pole and seam corners go through `split_degenerate_edge` and a wedge test.
5. **Classify.** Each fragment is `In/Out/OnSame/OnOpposite` against the other operand by
   `locate_point` (signed ray crossing count, eight fixed irrational directions).
6. **Select.** The rule table below keeps or drops fragments; kept ones are welded (vertex/edge
   merge), assembled into shells (`assemble`: union-find over edges, outer vs void by a probe),
   regularized (`join_faces` / `join_edges`) and given pcurves (`pcurves`).

| fragment | unite | intersect | subtract (A - B) |
|---|---|---|---|
| A, out of B | keep | drop | keep |
| A, in B | drop | keep | drop |
| A, on B same | keep once | keep once | drop |
| A, on B opposite | drop | drop (empty) | keep |
| B, out of A | keep | drop | drop |
| B, in A | drop | keep | keep, reversed |

Share modes: **Shared** (booleans, webcut: one edge for both faces) and **Private** (imprint: each
body keeps its own copies, one apply group per body, merge is a separate step).

## Tolerance policy

Nothing moves beyond tolerance. Points within `linear` are the same vertex; `merge` refuses
`tol > linear_max` (`Invalid`); pairs between `linear` and the user `tol` are `near`, never silently
merged. Surface/curve re-pointing in `regularize` only happens when the geometry agrees within `linear`.
Marched curves use a chord tolerance of `100 * linear`. Ambiguous classification (all 8 directions
grazing) is a typed `BooleanError::Ambiguous`.

## Creation order (determinism)

New cells are created in ascending pair order, then curve parameter, then face id. Every map that
feeds output is a `BTreeMap`/`IndexMap` or sorted first. The only parallel stage (pair evaluation)
collects in index order, so 1 thread and N threads are bit-identical. No randomness.

## Op layer

Ops use `topo_txn` + `commit_topology` (atomic: a failing op leaves the state hash unchanged),
`model_txn` for lineage-driven colours/groups (`Model::apply_lineage`), and the `MeshLayer::mesh_of`
guard (a volume with a mesh is refused, "mesh.delete first"). Webcut roles `positive`/`negative` hold
bodies, one body per resulting volume. Interface faces of the model boundary tables are skipped by
`model.merge`.

## Operand order and folding

`unite` and `intersect` take any number of operands. `unite` keeps a running volume and fuses it
with whichever remaining operand touches it (a failed "bodies do not touch" attempt edits nothing,
so the next candidate is tried); `Invalid("bodies do not touch ...")` is returned only when no
remaining operand touches the running volume. The result therefore does not depend on the operand
order for any connected set. `new_faces` accumulates over every fuse.

Candidates that meet the running volume along an edge or at isolated points only (nothing to
imprint, every fragment outside) fail with `TangentContact` before any edit and are skipped the
same way. A line tangency found while arranging (a cylinder resting on a brick face) fails after
edits started and the kernel has no sub-transaction. The ops therefore roll the whole transaction
back and retry with the offending operand fused last (`boolean_ordered`), at most once per
operand, so a bridge operand that covers the tangent line makes the set work in any order. If the
retries fail too, the first `TangentContact` is reported. The library function `boolean` alone
(without the ops) does not retry: it returns the error.

## Contact that is not an overlap

| Contact between two operands | unite | intersect | subtract |
|---|---|---|---|
| far apart | `Invalid("bodies do not touch")` | `EmptyResult` | target unchanged |
| along an edge or at a vertex only (found by a vertex of one lying on the other) | `TangentContact` (union is not a manifold solid) | `EmptyResult` | target unchanged |
| curved surfaces touching at isolated points (SSI tangent points) | `TangentContact` | `TangentContact` | `TangentContact` |
| along a line (cylinder on a plane) | `TangentContact` | `TangentContact` | `TangentContact` |

Edge or vertex contact is detected only when a vertex of one operand lies on a face of the
other; two edges that cross at an interior point of both are reported as "do not touch".

## Known limits

- **Near-tangent contact.** A curved operand whose clearance to another surface is below about
  `1e-2` model units (for example a unit-radius cylinder 1e-5..1e-3 from a brick face, with
  `linear = 1e-6`) is not classified robustly: the assembled volume fails its check and the
  operation is rolled back with `ResultInvalid` (the message names the near-tangent cause).
  Clearances of `1e-2` and above work. Exact line tangency (cylinder on a plane) and point
  tangency of curved surfaces are `TangentContact { entity, detail }` (`OpError::Unsupported`): the message names the
  face and says to perturb the tool beyond the model tolerance.
- **Webcut with a plane on an existing face** does not cross the target: the body is returned in
  one role and the other is empty (documented on `webcut`).

## Performance red flags (acceptable for W3a, to fix in W3b)

- `pieces::candidate_pairs` is a brute-force O(|fa| * |fb|) loop over face pairs with no spatial
  index (use a BVH over face boxes).
- Fragment classification runs `on_face` against every face of the other operand and a ray cast
  per fragment: O(F^2) for F fragments.
- `subtract` with N targets deep-copies the tool N-1 times.
- The unite worklist tries candidates in order, which is O(n^2) pair attempts for n operands in
  the worst case; each failed attempt costs one pair imprint test.

## Deferred to W3b

Mixed-dimension unite (sheets/wires with solids); self-intersecting or non-manifold inputs;
tangential-only contacts (typed `Unsupported`); NURBS faces by SSI marching (M4); extended NURBS
surface as a webcut tool; sheet-tool imprint without separation; projection of curves onto NURBS
faces; tolerant healing of gaps above `linear_max`; partial-overlap merge of curved faces (reported
`overlapping`, never merged); instanced and decomposed volumes (guarded); mesh-level booleans;
virtual topology; promotion of `cc.rs` into sm-geom; multi-edge cut chains between different loops of
a holed face; a per-op COW cost fix for 10^5-face models.

## W4-BOOL-HEAL-A: `model.regularize` and `model.heal_check`

`model.regularize {volumes}` runs the regularizer of the booleans as a standalone op and returns
the surviving joined faces and edges. It differs from the in-boolean call in two ways, which keep
every boolean result (and hash) as it was: it closes a periodic edge whose two arcs meet at `0`
and `2 pi` (the kernel's `join_edges` does not take those for one parameter, so the second arc's
range is moved by one period and put back if the join is still refused), and it rebuilds pcurves
only on the faces a join touched, so a regular import is unchanged. `regularize_volumes` returns
the `RegularizeReport` (joined and refused pairs with the kernel's reason).

`model.heal_check` (`heal_check.rs`) is read-only. Thresholds, the rules per code and what is not
detected (non-planar slivers, sphere/torus/cone blends) are in the module doc; they were tuned on
the W4-REFCORPUS dirty STEP files and the report equals the manifest's `expected` codes for all
eleven STEP entries, including `step-dirty-tol-edge` (`gap_near`, `open_shell`: confirmed).
Findings are sorted by `(code, entity, value)`; the per-entity scans run under rayon with ordered
collection.

## W3b-1: candidate-pair BVH

`pieces::candidate_pairs` queries a BVH (`bvh.rs`, median split, index tie-break) over the second
face list instead of scanning F^2 pairs; output is the same sorted set. `candidate_pair_stats`
reports `candidate_tests` (leaf-level box tests) and `node_visits` (recorded only). Fragment
classification uses `locate::UseIndex`: faces whose sampled box is exact (plane, straight edges) are
found through the BVH, every other face is always a candidate; candidates are visited in use order,
so results equal the unindexed scan. Copies made for a subtract with N targets already share the
surface and curve handles (only cells are copied; see `copy.rs` test).

## W4-BOOL-WEBCUT: native sphere, cone, torus and elliptic webcut tools

`CutTool::Surface` takes a `Sphere`, `Cone` (one nappe), `Torus`, `EllipticCylinder` or
`EllipticCone` besides the plane and cylinder. A bare quadric cannot be capped loop by loop (poles,
two seams, nappes), so `toolbody.rs` closes it into a temporary solid on the user's surface handle
(sphere and torus are closed already; a cone runs from its apex to a plane beyond the target, an
elliptic cylinder is capped on both ends beyond it, `window` overriding the target box). `webcut`
then runs steps 1-5 of the boolean (`boolean::imprint_classify`, shared with `fuse2`) once, and
assembles two volumes from the same fragments: the target inside the tool plus the tool fragments
inside the target (negative piece), the target outside plus the same fragments reversed (positive
piece). The shared fragments are the cut faces: one TopoFace with two uses, `record_generated` on
the tool surface, so the result needs no `merge` afterwards (`merge: false` unmerges them as for a
plane). A tool wholly inside the target gives an inside piece (the ball) and an outside piece with
a cavity; a tool that misses the target, or contains it, returns the target whole on the side it
lies on. The positive side is where the surface normal points (the outside of the solid).

A sphere is rotation symmetric, so `webcut` re-frames it per target (`sphere_frame`): the axis goes
along a cut circle's normal (latitude circles are exact), failing that to a fixed direction with
a margin of 0.5 rad to every circle, because a cut circle near a pole costs accuracy (1e-4 at 3
degrees from the pole) and a loop straddling the seam is the weakest case of the arrangement. The
cut face is stored on the re-framed surface.

Limits: a tool surface that coincides with part of the target boundary is `Unsupported`; spindle and
horn tori do not exist (`Torus::new`); NURBS and implicit tools stay typed refusals. Volumes of
curved pieces come from `volume_moments` quadrature; the property tests (random sphere,
torus and elliptic cylinder frames) hold them to 1e-9 of the analytic value. A cut loop near a
torus or elliptic-cone singularity is not re-framed (only spheres are), so a tilted torus whose
loop passes near its seam crossing is the next place to watch.

Fixed on the way: `geo::face_aabb` took a sphere face's box from its edges alone (the seam and the
degenerate poles), so a plane that missed the seam's box never met the sphere in the candidate-pair
search and no cut was found (every plane not perpendicular to the sphere's axis, in most positions).
A sphere face now uses the whole sphere's box. `arrange::orient_seam_loop` took the tangent at the last sample of a closing
loop from the first sample (a wrap by a period), so a sphere tilted by about 8 to 24 degrees cut
through its centre by a plane failed with an open shell (booleans and webcut alike). The tangent
now comes from the previous sample at the end of the list.

`volume_interior_point` and `face_interior_point` (`locate.rs`) expose the interior probes: a face
interior point, and a point confirmed `In` by `locate_point` by moving face points inward along a
ladder of depths.

`model.webcut_surface` takes the face of any of these surfaces (the whole surface is the tool; a
face with a reversed carrier swaps the sides). `model.regularize` lists the cells of refused pairs
in a `refused` output role.
