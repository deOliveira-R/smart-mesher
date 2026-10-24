# sm-model (W2 T1)

Primitives, similarity transforms, delete and topology queries, registered by `ModelPlugin`.

## Positioning

Similar to Cubit's `create brick|cylinder|sphere`, `move|rotate|scale|reflect volume`, `delete body`
and the `list`/`measure` family (and, underneath, to OpenCASCADE's `BRepPrimAPI_*` and
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
| `model.move {entities, delta}` | op | role `volumes` |
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

`model.measure` uses `face_moments` / `volume_moments` (Gauss quadrature). The default view refuses
faces on surfaces with poles, so a *whole sphere* (one loop, two pole coedges, seam on both sides)
uses the closed forms `4 pi r^2`, `4/3 pi r^3`; a partial patch of a pole surface is `Failed`.
Bounding boxes come from edges: exact for lines, circles and ellipses (analytic extremes), 256
samples for NURBS, `centre +- r` for a whole sphere. The box of a partial patch of a curved surface
can be smaller than the true one.

## Determinism and cost

No parallelism, no randomness, no hash-map iteration order (`BTreeMap`/`BTreeSet` throughout).
Cost per transform is O(cells + edges + faces) (the shape-handle sharing check scans every edge
and face once) on top of the layer's copy-on-write clone of the host (O(model), a known W3 item).
The meshed-volume check is O(closure faces x meshes x selection).
