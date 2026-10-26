# sm-poly: whole-mesh tet -> polyhedral conversion

## Positioning

Similar to OpenFOAM's `polyDualMesh` and the STAR-CCM+ polyhedral conversion (one cell per
primal vertex, one face per primal edge, tet centroids as dual points, boundary polygons from
fans of boundary triangles), `sm-poly` differs in four ways, giving a dual a transport code and a
ray tracer can use directly:

1. **Features come from the model, not from an angle.** A boundary edge is a feature when its two
   triangles lie on different `TopoFace`s (the `TopoEdge` it lies on comes from the source edge
   table, the model topology or the vertex classes), so curves and corners are preserved
   exactly and a gently curved face is never mistaken for a crease.
2. **Classification is carried, never recomputed.** Every dual vertex, face and cell lies on the
   model entity it came from, so side sets, blocks and `mesh.validate` keep working.
3. **Multi-material aware.** Dual cells are keyed `(vertex, region)`; a vertex on an interface
   gets one cell per volume and the interface polygons of the two cells coincide.
4. **Planar and conservative.** Every emitted face is planar within the default `GeomTol`
   (non-planar polygons are split into planar pieces that both cells share), total volume equals
   the tet volume to rounding, and the output passes `validate` on every input and
   `validate_geometry` on the stage-0 fixtures, perturbed lattices and multi-volume blocks with
   planar interfaces. It does NOT in general: a dual cell at a reflex interface edge can fail
   `GeomSideTet` (see "Risks"). Planar faces give ray tracing exact plane tests.

It is an IR -> IR mesh pass (`MeshRead -> MeshBuild`), not a `VolumeScheme`: a scheme sees one
target's patch, never a finished mesh (`sm-scheme-api` DESIGN, "Transforms").

## The dual rule

A *region* is the set of tets with one cell classification (`TopoVolume`). A triangle is
*region-boundary* if the region has exactly one tet on it (a model boundary face or an interface
between two volumes). For every vertex `v` and every region `R` that has a tet at `v` there is one
dual cell `(v, R)`.

Dual points (the vertices of the output):

| point | where | lies on |
|---|---|---|
| tet centroid `c_t` | vertex mean of the tet | the tet's `TopoVolume` |
| boundary-triangle centroid `fc_f` | vertex mean; one per face, shared by the two regions of an interface | the triangle's `TopoFace` |
| edge midpoint `m_e` | of every region-boundary edge | the `TopoFace` if both triangles lie on it, else the feature `TopoEdge` |
| primal vertex `v` | only where a polygon needs it (feature vertices, per-triangle quads) | the vertex's own class |
| hub `h_e` | midpoint of an interior edge, only for a non-planar edge face in `Planar` mode | the region's `TopoVolume` |

Faces of the cell `(v, R)`:

* **Edge face.** Every primal edge `(v, w)` of the region gives exactly one face shared by the
  cells `(v, R)` and `(w, R)`. For an interior edge it is the closed ring of tet centroids around
  the edge; for a region-boundary edge it is the open path
  `fc_in, c_1 .. c_k, fc_out`, closed by `m_e`:

  ```
        interior edge e = (v, w)          boundary edge e = (v, w), surface below
        c_3 ----- c_2                      c_1 ---- c_2 ---- c_3
       /  (axis      \                    /                     \
      c_4   v -> w    c_1                fc_in ......m_e...... fc_out      (on the surface)
       \             /                      (m_e is on the edge, the polygon closes through it)
        c_5 ----- c_6
  ```

  The ring is walked topologically: a right-handed rotation about `v -> w` leaves a tet through
  the face whose outward loop traverses `v -> w`. The walk starts at the lowest tet (closed ring)
  or at the tet entered through a region-boundary face (open path), so the polygon winds
  counterclockwise about `v -> w`, i.e. its normal points from the lower cell to the higher one.
  The geometric check is a `debug_assert`.
* **Boundary face.** The region-boundary triangles around `v` are linked through edges that are
  not *split edges*. A split edge is a boundary edge whose triangles lie on different
  `TopoFace`s, whose class in the source edge table is a `TopoEdge`, or whose two triangle planes
  differ by more than `1e-9` (relative, fixed). Each connected piece is one polygon: the closed
  ring `m_e1, fc_t1, m_e2, ...` inside a smooth patch, or `v, m_e1, fc_t1, ..., m_ek` for a fan
  that ends at feature edges, so curves and corners are kept exactly. A vertex with exactly one
  split edge in a region is a seam or a dangling curve: `PolyError::UnsupportedFeature`.

```
   smooth patch (closed fan)          fan ended by two feature edges
        m1 --- fc1 --- m2                  v ---- m1
       /  \           /  \                 |  fc1  |
     fc6   \   v     /   fc2               |       fc2 ... m_last
       \    \       /    /                  \  feature curve = v-m1, v-m_last
        m6 -- fc5 -- m3                      (v is a vertex of the polygon)
```

### Counts

For a Kuhn block of `n = (a, b, c)` cells per axis (`tet_block`), `s = ab + bc + ca`:

| quantity | formula | `n = 3` |
|---|---|---|
| dual cells | `(a+1)(b+1)(c+1)` | 64 |
| edge faces (= primal edges) | `a(b+1)(c+1) + b(a+1)(c+1) + c(a+1)(b+1) + ab(c+1) + bc(a+1) + ac(b+1) + abc` | 279 |
| boundary polygons | `2[(a-1)(b-1)+(b-1)(c-1)+(a-1)(c-1)] + 2*4(a+b+c-3) + 3*8` | 96 |
| dual points | `6abc + 4s + 6s + 4(a+b+c-3) + 8` | 464 |

where the edge faces are 144 axis edges + 108 face diagonals (one per lattice face) + 27 body
diagonals. Boundary polygons = 24 face-interior vertices x 1 + 24 edge vertices x 2 + 8 corners x 3. Dual
points = 162 tet centroids + 108 boundary-triangle centroids + 162 boundary-edge midpoints + 24
curve vertices + 8 corners, by class 162 `TopoVolume` + 234 `TopoFace` + 60 `TopoEdge` (36
midpoints + 24 vertices) + 8 `TopoVertex`. `Keep` mode has `279 + 96 = 375` faces. Volumes: the
8 interior cells are 1 each, the 24 face-interior boundary cells 1/2 (the Kuhn lattice is
invariant under translation and under point inversion through the box centre).

### Why interior face centroids are not dual points

The full barycentric subdivision also puts a point at the centroid of every interior triangle;
each edge face is then a fan of `2k` planar triangles. That doubles the vertex count and makes
every edge face a triangle fan. Here the edge face is the single polygon through the `k` tet
centroids. It is generally non-planar, so:

* in `Keep` mode it stays one polygon (geometry follows `smartmesh.fan_vertex_mean.v1`);
* in `Planar` mode (the only mode of the op) a polygon whose planarity measure
  (`GeomTol::planarity` metric) exceeds the default is replaced by hub triangles
  `(h_e, p_i, p_{i+1})`. The hub is the edge midpoint (`m_e` itself for an open path), a new mesh
  vertex only then. Both cells of the face use the same triangles, so every cell stays closed and
  total volume is conserved exactly; two cells can then share several faces. A planar polygon
  stays whole, which is why `Planar` has 627 faces on the 3x3x3 block against 375 in `Keep`
  (`375 - 90 + 342`).
* a non-planar boundary fan is split into one quad `v, m_e1, fc_t, m_e2` per triangle (planar,
  since it lies in the triangle).

Hub splitting keeps the vertex set small, keeps most faces whole, and needs no vertex motion
(which would break exact boundary tiling and classification).

## Implementation

`prep.rs` analyses the source once, in parallel, into position-ordered chunks:

1. regions (sorted `TopoVolume` ids), tet corners and centroids;
2. `6T` `(edge, region, tet)` triples sorted into **slots**, one per (edge, region): the faces
   of the dual;
3. per slot: the ring (closed or open), the split flag and feature class of an open path, the
   planarity decision;
4. dual cells `(region, vertex)` sorted, slot -> cell maps;
5. boundary fans per cell (outward triangle loops rotated to start at the vertex, linked
   counterclockwise).

`polydata.rs` numbers the points (tets, boundary triangles ascending, midpoints by sorted vertex
pair, primal vertices, hubs; only used ones) and records faces and cells serially in
`(region, vertex)` order as plain data (`PolyData`): the lower cell of an edge face gets
`Side::Left`, the higher one `Side::Right`; an interface polygon is found again by its canonical
point cycle so the second region reuses the first one's face. `commit` then feeds the builder
(`add_face`, `add_cell`) after the optional agglomeration pass (`agglom.rs`) has dropped merged
faces and unused points. Cell shapes are inferred by
`finish`: a dual cell with hex topology (all four cells of one tet) comes out as `Hex`; the
block-level element type `POLYHEDRON` belongs to the fem/export layer, not to this crate.

Classification set explicitly: vertices (see table), boundary and interface polygons (their
`TopoFace`), cells (the region). Interior faces are left to `finish`. With an edge table
(`edges`, default: the source has one) the dual edges whose endpoints are both on a model vertex
or edge take the class of their curve end; `finish` derives the rest.

### Determinism

No hash map is iterated. Sorts are total orders on unique keys. Parallel stages write into
per-chunk buffers concatenated in chunk order; reductions use fixed chunks. Output is bit
identical for 1 and N threads (`acc7_determinism`), and the op replays cold without `Diverged`.
The planarity decision of an interface polygon is evaluated on a canonical rotation of the point
cycle, so both regions decide identically.

### Performance (informational, S1)

`tet_block([40,40,40])`, 384 000 tets, release, 4 cores, `lattice::tet_kuhn` source:
`Planar` 0.46 s, `Keep` 0.38 s (68 921 cells, 529 246 / 472 606 faces); 1 thread
(`RAYON_NUM_THREADS=1`) 0.99 s / 0.88 s (noisy shared VM: 0.34-0.46 s seen for the same run).
Peak RSS of the dualization is about 115 MB over the resident source (377 MB peak with the
263 MB source and fixture resident). The serial builder commit is not separated; bulk commit is
W3b. `cargo bench -p sm-poly` times 48k and 384k tets in both modes;
`cargo test --release -p sm-poly --test perf -- --ignored --nocapture` prints the figures above.

## Op, query and the `ambiguous-mesh` consequence

`mesh.to_poly {mesh, keep_source?: bool = true}` converts a whole all-tet mesh entry into a NEW
entry (role `mesh`), copying `selection`, `plan`, `quanta` and `stale`, and records the
succession `source -> [source, new]` (`source -> [new]` with `keep_source: false`) in the delta.
Refusals (`OpError::Invalid`, no state change): not a 3D mesh, a non-tet cell (this includes a
mesh that is already polyhedral), a `NonConformal`/`PartitionCut` face, open rings, an
unresolvable feature edge. `OpError::Unsupported`: a vertex with exactly one feature edge.
`mesh.poly_info {mesh}` returns `PolyStats` (faces per cell, vertices per face, planarity,
volumes, boundary area).

With the default `keep_source: true` the volume has **two** mesh entries. `session.export`
without a mesh named then returns the existing `ambiguous-mesh` error (decision 8), and the
`mesh.run` neighbour guard sees the derived mesh. Scripts and exports must name the mesh
(`mesh: pm.mesh[0]`); `keep_source: false` leaves one.

## Comparing with `polyDualMesh` (integration harness)

`polyDualMesh` keys cells by point and has no regions, and it is not the same dual (it also uses
face centres and a feature angle), so only invariants are compared, on the same tet mesh
(`tetbox`, 10x10x6, volume 600):

1. total volume of both duals equals the tet volume within 1e-9;
2. boundary area per patch (sum of boundary faces per set) equal within the chord error, exact
   for planar patches;
3. `checkMesh`: both `Mesh OK.`, ours with zero non-planar faces, face count and skewness only
   reported;
4. cell counts: ours = number of (vertex, volume) pairs (one volume: the point count of the tet
   mesh), `polyDualMesh` = points.

Cell and face counts are reported, never compared exactly. Docker is the orchestrator's.

## Agglomeration (library option, stretch S2)

`PolyOptions::agglomerate = Some(Agglomerate { min_volume_ratio, max_concavity = 0.1,
max_rounds = 4 })`, off by default and not an op argument. Serial greedy matching on the
`PolyData`: cells below `ratio * median` volume (visited by ascending `(volume, index)`) merge
with the same-region neighbour of largest shared face area (ties lowest index; the first
neighbour that passes is taken) when the shared faces form one disc patch (their boundary is one
simple cycle), the merged cell has positive volume and positive side tetrahedra, and its
concavity `max_{f,v} (x_v - apex_f) . n_f / V^(1/3)` is within `max_concavity`. Each cell merges
once per round; the lower cell index survives. Faces are only removed, so every face stays
planar, interface faces and classes survive, and the volume sum is unchanged.

**Measured limit.** In `Planar` mode the unmerged cells already have concavity 0.14 (median) and
0.235 (max) on `tet_block([4,4,4])`, because hub triangles bulge (0.043 / 0.084 in `Keep`). The
default 0.1 therefore merges nothing there; 0.75 or more admits 20 merges at ratio 0.75 (125 ->
105 cells), nothing merges at or below 0.5. The tests pin both facts rather than relaxing the
default.

## Risks

**Reflex interface edges can produce concave dual cells (`GeomSideTet`), open.** Decision 12 and
the plan foresaw it; the tolerance is not loosened. Repro (pinned by
`known_gap_reflex_interface_edge_gives_side_tet_violations` in `tests/robust.rs`):
`tet_block([4,4,4],[1;3])` with the tets whose centroid has `x<2 && y>2` moved to `TopoVolume` 2,
dualized with the op defaults (`FaceMode::Planar`, `VertexClassResolver`). Cells 12 (vertex
`(2,2,0)`) and 96 (vertex `(2,2,4)`), the two ends of the reflex interface edge, in region 1 fail
`GeomSideTet`. The flagged face is a convex interface kite in the plane `y=2`; the region-1
cell's volume centroid `(2.19, 2.12, 0.27)` lies on the region-2 side of it, so the cell is not
star-shaped about its centroid. It is not a winding or planarity bug (the partner region-2 cells
are not flagged) and the volume is conserved. It depends on the orientation of the notch against
the Kuhn diagonal: notches `{x>2,y>2}`, `{x<2,y<2}`, `{x<2,z<2}`, `{x>2,z>2}` pass; `{x<2,y>2}`
(cells 12, 96), `{x>2,y<2}` (8, 92) and `{y<2,z>2}` (60, 64) fail, as does a 3-concave corner
`{x<2,y<2,z>1}`. The stage-0 `tet_lshape` passes only because its notch (`i>=m && j>=m`) is the
favourable orientation (checked for m = 1..4). Repair needs cell splitting at reflex features
(W3b); until then treat a mesh with a reflex interface or boundary edge as "may fail
`validate_geometry`".

Observation, `Keep` mode only (not reachable from the op): an enclosed cube inclusion (region 2
= `[1,3]^3` in a 4^3 block) gives 12 `GeomSideTet` findings; `Planar` is clean
(`known_gap_keep_mode_enclosed_inclusion`).

## Known gaps (W3b)

Partial dualization of a mixed mesh (needs conformize), circumcentric duals, vertex-moving
planarization, projection of boundary dual points onto the CAD surface (curved bodies deviate by
the chord error), hybrid input, seam edges and a vertex with one split edge, a bulk builder
commit, concavity repair beyond the agglomeration guard (including the reflex-edge `GeomSideTet` risk above), a persistent (state) mark that the poly
mesh is derived from its source (the delta records the succession only; see the report).
