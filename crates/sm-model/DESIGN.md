# sm-model (W2 T1, W3b-PRIM)

Primitives, similarity transforms, copy, delete and topology queries, registered by `ModelPlugin`.

## Positioning

Similar to Cubit's `create brick|cylinder|sphere|frustum|prism|pyramid|torus`, `move|rotate|scale|reflect
volume`, `volume ... copy move|rotate ... repeat n`, `move ... to`, `rotate ... about vertex|surface`,
`delete body` and the `list`/`measure` family (and, underneath, to OpenCASCADE's `BRepPrimAPI_*` and
`BRepBuilderAPI_Transform`), but every edit is a *replayable topology transaction* over a generic
topology host (`TopoHost`: the production `TopoStore` or the reference `CellComplex`), with ids
minted by the store, exact lineage and a quantized state hash, and transforms that *keep
identities* instead of copying shapes. That gives an op log in which the same script produces the
same entity ids and the same layer hash on both stores, on any thread count and after a cold
replay, and in which a reflected, rotated or scaled volume is still the same `TopoVolume#n`
(colors, decks, groups and meshes keep pointing at it).

## Surface

| Name | Kind | Notes |
|---|---|---|
| `model.create_brick {x, y?, z?, center?}` | op | `x` is positional; `y`,`z` default to `x`; centred. Roles `body`,`volume`,`faces`,`edges`,`vertices`. |
| `model.create_cylinder {height, radius, center?}` | op | axis z, `z = center.z +- height/2`. Same roles. |
| `model.create_sphere {radius, center?}` | op | `radius` positional. Same roles. |
| `model.create_frustum {height, radius, minor?, top?, center?}` | op | `height` positional; circular or elliptic cone, frustum (also inverted), cylinder twin, apex. Same roles. |
| `model.create_prism {height, sides, radius, minor?, center?}` / `model.create_pyramid {..., top?}` | op | `sides` positional, `3..=1024`; `radius` is the circumradius, vertex 0 on `+x`; all-`Plane` faces. Same roles. |
| `model.create_torus {major, minor, center?}` | op | `major > minor > 0`. Same roles. |
| `model.create_sphere_part {radius, inner?, octants?, center?}` | op | `radius` positional; hemisphere / quarter / octant, hollow. `create_sphere` is unchanged. Same roles. |
| `model.move {entities, delta}` | op | role `volumes` |
| `model.move_to {entities, to, anchor?}` | op | `to` is `[x, y, z]`, `{x?, y?, z?}` or an entity id; `anchor` `centroid` (default) or `bbox_center`; role `volumes` |
| `model.rotate_about {entities, angle_deg, axis? \| vertices? \| face?, origin?}` | op | exactly one form; role `volumes` |
| `model.copy {entities, count?, xform?}` | op | deep copy; roles `volumes`, `bodies` |
| `model.rotate {entities, axis, angle_deg, origin?}` | op | role `volumes` |
| `model.scale {entities, factor, origin?}` | op | uniform; role `volumes` |
| `model.reflect {entities, normal, origin?}` | op | role `volumes` |
| `model.delete {bodies}` | op | `kill_cascade`; retires the cascade |
| `topo.related {kind, within}` | query | sorted, de-duplicated ids; the shape `sm-lang`'s `QueryRelations` calls |
| `model.list {kind}` | query | topology kinds and model records |
| `model.measure {entity}` | query | length / area / volume / centroid / bbox |

`ModelPlugin` registers ops and queries only. The `model` layer comes from
`sm_layer_api::CoreLayers`.

## Primitives

Every primitive builds its cells in one `sm_layer_api::topo_txn` and finishes with
`commit_topology`, which adopts the ids in ascending `(kind, serial)` order, so `Delta.created`
is exactly the new cells and the declared output roles are filled. The brick is
`sm_topo_api::fixtures::brick` with `min = center - extent / 2`: for representable values
(`x = 10`) the vertices are exactly `+-5` and the canonical hash equals the `CellComplex`
fixture's. For non-dyadic values `min + extent` can differ from `center + extent / 2` by one ulp.
The cylinder and sphere are ports of the `fixtures` constructions with an offset (seam vertex on
`+x`, pole edges flagged `DEGENERATE`, pcurves in `(angle, z)` / `(angle, latitude)`).

## Transforms

`model.move|rotate|scale|reflect` share one pipeline (`xform.rs`). The selection is volumes or
bodies (a body stands for its members); the downward closure moves. **Refusals** (`Invalid`, state
untouched): empty selection; a closure cell also used by an unselected non-body cell (moving one of
two merged bricks would tear the shared face); a curve or surface handle used outside the
closure; a meshed volume or face (`mesh.delete first`, checked through `MeshLayer::mesh_of`; a
missing `meshes` layer is tolerated); non-finite or degenerate parameters; `factor <= 0`.

Inside one topology transaction:

1. `set_vertex_point` for the vertices.
2. `replace_curve` / `replace_surface` with `CurveDef::transformed` / `SurfaceDef::transformed`.
   A line's parameter is arc length, so line edges get `range * factor` (`set_carrier`); circle,
   ellipse and NURBS parameters are unchanged under a similarity.
3. Pcurves are mapped through the surface's parameter change: plane `(u,v) -> (s u, s v)`;
   cylinder `(u,v) -> (+-u (+ 2 pi), s v)`; sphere `(u,v) -> (+-u (+ 2 pi), v)` where the sign is
   `-` under a reflection (`frame_zx` makes the angle run backwards); NURBS surfaces unchanged.
   `Line2` is remapped with the edge's parameter factor, `Polyline2` pointwise. Other surface
   kinds with pcurves return `Failed` ("cannot be carried").
4. **Improper maps** (reflect) need the loops reversed while the volume sides stay: the loop-derived
   normal of a mirrored face points inwards. `reverse_face` alone is not enough: it swaps the
   volume sides too, and `end_op` swaps a lone `Right` volume straight back. So per face: take
   the volume uses off (right-hand ones first, so the orientation rule never reverses the face
   behind our back; a guard compares the face row), `reverse_face`, restore the carrier sense to
   `pre-op sense (+) sense change of the image surface` (planes and NURBS flip, surfaces of
   revolution and implicit forms do not), put the uses back with their original sides and shells.
5. `scale_tolerances` for scale.
6. A consistency check evaluates every pcurve on the new surface against the new curve
   (relative `1e-9`), so an uncovered surface/pcurve combination fails the op instead of storing a
   wrong model.
7. `Model::apply_transform` carries decks, cut planes and symmetry.

`commit_topology` finishes with `("volumes", moved volumes)` through `record_role`: the ids
survive and are reported, not created. The `OpId` is the layer's `ops_run + 1`.

## Queries

`topo.related` walks boundary rows down (or coboundary rows up) from the sources, collecting live
cells of the target kind. Same kind returns the sources themselves. `within` is checked against the
topology (`is_alive`), not the entity table, so fixtures injected through
`register_layer_with::<ModelLayer>` work.

`model.measure` uses `face_moments` / `volume_moments` (Gauss quadrature). A *whole sphere* (one loop,
two pole coedges, seam on both sides) keeps the exact closed forms `4 pi r^2`, `4/3 pi r^3` (the
view's quadrature handles pole faces since CR-1; tests compare the two to 1e-11).
Bounding boxes come from edges: exact for lines, circles and ellipses (analytic extremes), 256
samples for NURBS, `centre +- r` for a whole sphere. The box of a partial patch of a curved surface
can be smaller than the true one.

## Determinism and cost

No parallelism, no randomness, no hash-map iteration order (`BTreeMap`/`BTreeSet` throughout).
Cost per transform is O(cells + edges + faces) (the shape-handle sharing check scans every edge
and face once) on top of the layer's copy-on-write clone of the host (O(model), a known W3 item).
The meshed-volume check is O(closure faces x meshes x selection).

## W3b primitives

All new creators use the `create()` guard of `prim.rs` (size at or below the model's linear
tolerance, non-finite or degenerate derived coordinates: `Invalid`, state untouched) and
`SOLID_OUTPUTS`. Frustum, prism and pyramid also refuse sizes above `1e150` (squared edge lengths would overflow); the torus and sphere part refuse overflowing extents and diameters.

**Frustum** (`prim2.rs`). `top == radius` circular is `build_cylinder` (same canonical hash as
`model.create_cylinder`); elliptic twin: `EllipticCylinder` with `Ellipse` edges. A circular cone
or frustum has a `Cone` lateral surface whose frame sits on the *larger* end disc with `z`
pointing away from the other end (`origin` at the bottom for `top < radius`, at the top for the
inverted case), so one layout serves both: loop `[small circle, seam (rev), big circle (rev), seam]`,
`v` in `[-slant, 0]`, `half_angle = atan2(|R - T|, h)`. The apex form is the
`fixtures::cone` construction in its creation order (hash equal to `testkit::topo::cone`). Elliptic:
`EllipticCone` with `a` along x, the top section homothetic (`top * minor / radius`).

**Prism and pyramid.** One builder with all-`Plane` faces; the plane normal is the Newell normal of
the face's own loop vertices (no fitting), so a lateral quad whose corners are coplanar to rounding
gets the exact normal of its loop. Vertices at the quadrant angles are exact (`cos`/`sin` table),
vertex 0 is exactly on `+x`.

**Torus** is `fixtures::torus` translated to `center`.

**Sphere part** (`sphere_part.rs`). The kept patch is a `(u, v)` rectangle of the sphere
frame whose x axis is the patch's start direction (so `u` is in `[0, delta]`, no negative
angles), with `delta` in `{2 pi, pi, pi/2}` and `v` in the full, upper or lower half range.
Pole sides are `DEGENERATE` edges. Caps are planes that share the meridians, the equator arc, the
diameter (half cuts) and the axis (quarter cuts) with the patch and each other; when `delta = pi`
the two end faces are coplanar and merge into one face (an annulus with two loops when hollow).
Hollow: the inner ring is the same construction at the inner radius, its patch is the same sphere
with a `Reversed` carrier and a reversed loop (normal into the cavity). A whole hollow sphere is
`Outer` + `Void` shells; any cut gives a single `Outer` shell with annular caps. The topological
Euler number `V - E + F - (extra loops)` is 2 for every cut hollow part (a plain `V - E + F` gives 3
for a z-cut hollow part: the annulus counts one face for characteristic 0).

## W3b transform forms and copy

`move_to` and `rotate_about` (`xform2.rs`) resolve their parameters from the model (centroid via
`volume_value`, bbox from the faces, vertex points, face normal with the carrier sense and the
use side) and then call `apply_similarity`, so they share the refusals and the pcurve path of
`model.move` / `model.rotate`; `axis: "z"` goes through the very same `rotation_xf` as
`model.rotate`, so the state hashes are equal. `uv_map` now covers `Cone`, `Torus`,
`EllipticCylinder` and `EllipticCone` (angle `u` as for cylinder and sphere; `v` scales for
cones and elliptic cylinders, not for the torus), so every new primitive can be moved, rotated,
scaled and reflected.

Op parameters cannot hold `null` (the op codec refuses it), so the draft's `to: [Option<f64>; 3]`
with `null` for "unchanged" is expressed as the object `{x?, y?, z?}` (an omitted coordinate is
unchanged); `[x, y, z]` and an entity id are accepted too.

`model.copy` (`copy.rs`) snapshots the downward closure once, then per copy `k` creates fresh
vertices, edges, faces and volumes (ascending old ids, so the new ids are deterministic), clones the
curve and surface handles, copies flags, tolerance overrides, pcurves, shell kinds and face-use
sides, calls `record_created`, and runs the existing `transform_cells` with the transform applied `k`
times (`power`: composed matrix, `scale^k`, improper iff reflect and `k` odd). A face shared with an
unselected volume becomes a lone use in the copy (the orientation rule may reverse it at `end_op`).
Not copied: meshes, colors, groups, boundary kinds, decks, decomposition records, embedded members.
