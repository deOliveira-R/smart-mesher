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

## Deferred to W3b

Mixed-dimension unite (sheets/wires with solids); self-intersecting or non-manifold inputs;
tangential-only contacts (typed `Unsupported`); NURBS faces by SSI marching (M4); extended NURBS
surface as a webcut tool; sheet-tool imprint without separation; projection of curves onto NURBS
faces; tolerant healing of gaps above `linear_max`; partial-overlap merge of curved faces (reported
`overlapping`, never merged); instanced and decomposed volumes (guarded); mesh-level booleans;
virtual topology; promotion of `cc.rs` into sm-geom; multi-edge cut chains between different loops of
a holed face; a per-op COW cost fix for 10^5-face models.
