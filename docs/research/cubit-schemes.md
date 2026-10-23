# Cubit scheme research: facts that shape our interfaces

Source: Cubit Help v2026.8 scheme pages (mapping, submap, pave, sweep, tetmesh, trimesh, tridelaunay, triadvance,
polyhedron, thex, qtri, htet, copy, bias/dualbias, curvature, stretch, auto selection, interval matching,
surface vertex types, sizing functions). Boundary-layer page returned 404, so it still needs research.

## Cross-cutting requirements → interface consequences

| Cubit fact | Consequence for `sm-scheme-api` |
|---|---|
| Vertex types End/Side/Corner/Reversal (1/2/3/4 elements, ≈90/180/270/360°), set **per surface** (a vertex can differ per surface); auto = soft, user = hard | `VertexType` enum + `Firmness {Soft, Hard}` stored per (surface, vertex) use, i.e. on Coedge-level / loop-vertex, not on the vertex. Same for volume curve types (sweep). |
| Map needs 4 logical sides (surface) / 6 sides, 8 corners (volume); a side may span several curves; corners may lie mid-curve | Schemes receive **logical-side groupings** of boundary curves; corner-picking is a shared service (`sm-sizing`/`sm-corner`), not per-scheme code. |
| Quad meshing needs an even boundary interval count; map needs opposite sides equal; sweep needs equal layer counts along every source→target path | Each scheme emits `IntervalConstraint`s (linear equalities/inequalities + parity) **before** meshing. A global integer solver (IIA-style) resolves them across the whole model. |
| Interval firmness soft/hard/default; scheme firmness default/soft/hard | `Firmness` is a general concept on all mesh attributes. |
| Auto selection: surfaces from {pave, submap, triprimitive, map}, volumes from {map, submap, sweep}; depends on intervals already set; fuzzy angle tolerance | `applicability(&input) -> Applicability{score, reason}` on every scheme; selector is a separate service. |
| Sweep: source (any surface scheme) → target, linking surfaces must be map/submap; many-to-one/many-to-many; translate/rotate; least-squares transform; smoothing options; source precedence {meshed, merged, specified} | Volume schemes can **impose schemes on child surfaces** (`required_child_schemes()`), and the hierarchy engine must allow a volume scheme to mesh its own surfaces (target). |
| Polyhedron volume scheme auto-sets surfaces; assembly-wide setup required | Same "impose on children" hook, plus a multi-volume (assembly) meshing call. |
| THex: all merged volumes must be converted **in a single call** | `MeshTransform` operates on a set of volumes atomically, with shared-surface consistency. |
| TetMesh: surfaces auto-trimeshed if unset; respect face/edge/node/file constraints; growth factor; optimize levels; boundary recovery | `VolumeMeshInput` carries embedded constraints (respect entities) + size field + growth. |
| TriMesh fallback chain (TriDelaunay → TriAdvance → QTri) | Engine supports **fallback chains** of schemes. |
| Sizing functions: constant, curvature, linear, interval, inverse, super, bias, field-from-file, geometry-adaptive (skeleton) | `SizeField` trait: `size(point) -> f64` (+ optional anisotropic metric `metric(point) -> SymMat3`). |
| Curve schemes: bias (any two of {intervals, factor, first size}), dualbias, multi_bias, curvature (iterative, not periodic), stretch, copy (percent/auto) | `CurveScheme` gets curve length param + endpoint sizes and returns param distribution; must handle periodic curves explicitly. |
| Copy mesh: source→target with vertex/curve correspondences, mirror, morph smoothing | Correspondence maps are an input type; copy is a scheme at every dimension. |
| QTri base scheme: pave (non-periodic) / map (periodic); 2- or 4-split | Conversion schemes may compose a base scheme. |
| HTet 6-tet (no new nodes) vs 28-tet structured | Conversion output may add nodes; ownership must be assigned. |
