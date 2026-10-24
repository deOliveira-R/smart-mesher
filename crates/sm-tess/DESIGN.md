# sm-tess: design

## Positioning

**Similar to** the display tessellators of OpenCASCADE (`BRepMesh_IncrementalMesh`) and Cubit's graphics facets (indexed triangles per B-rep face plus polylines per curve, driven by chord, angle and edge-length tolerances), **differs in** deriving every sample count from a closed form where one exists (a circle gets `n = ceil(span / min(2 acos(1 - chord/r), angle_tol))`, so the sagitta bound is a theorem and not a hope), sharing one polyline per edge between all adjacent faces *by index* (watertight by construction, not by stitching), snapping loops to an integer lattice and triangulating with exact `i128` predicates (no epsilon; a documented fallback only for degenerate input), and numbering vertices in a fixed order so the buffers are **byte-identical for any thread count**, **giving** a display mesh that is a pure function of `(topology state, tolerances)`: it can be cached or diffed by hash, its error is asserted analytically in tests, and it never mutates the model or touches the mesh layer.

## What it computes

`tessellate(view, faces, opts) -> Tessellation`:

| Buffer | Content |
|---|---|
| `positions: Vec<f32>` | `x y z` per vertex; cast from `f64` exactly once |
| `indices: Vec<u32>` | 3 per triangle, counter-clockwise about the face normal (carrier sense applied) |
| `faces: Vec<FaceRange>` | `(face, first_index, index_count)`, ascending by face id |
| `edge_indices`, `edges: Vec<EdgeRange>` | one polyline per edge, edge-forward order, ascending by edge id |

`tessellate_entities` accepts faces, volumes, bodies (expanded to faces) and edges (polyline only). `TessOptions { chord_tol, angle_tol, max_edge }`; `Default` is `0.01`, `0.3 rad`, none.

The query `view.tessellate {entities, chord_tol?, angle_tol?, max_edge?}` runs the same code on `ModelLayer::topo()` and returns the buffers as base64 (little-endian `f32` / `u32`) plus the ranges. A missing `chord_tol` becomes `1e-3` of the diagonal of the scene's vertex and edge-sample box. The query registers through `TessPlugin`, reads only the `model` layer, and writes nothing.

## Pipeline and determinism

1. **Normalise**: sort and deduplicate faces; validate rank and liveness.
2. **Edges (parallel)**: sample every non-degenerate edge of the closure of the faces. Ends are the vertex points (`Carrier::Point`); interior points come from the curve. Output in edge id order.
3. **Number (serial)**: vertices ascending by id, then each edge's interior points in edge id order. Face jobs get read-only CSR views of the edge data.
4. **Faces (parallel)**: each job returns face-local interior points and triangles whose indices are global (`< g_count`) or local (`>= g_count`).
5. **Assemble (serial)**: face-local points are appended in face id order and local indices are remapped by a running offset.

Every parallel stage is `par_iter().map().collect::<Vec<_>>()` (order-preserving) and errors are chosen by position, so no result depends on the scheduler. Threads never mint global indices. Tests: `output_is_identical_for_one_and_many_threads_and_repeats` (testkit `determinism::check`: 1 thread vs 4, repeats, concurrent calls), the same through the query and after a cold replay from the root, and the full suite under `RAYON_NUM_THREADS=1`.

## Sampling rules (and the bounds they give)

| Entity | Rule |
|---|---|
| `Line` edge | 1 segment; `ceil(len / max_edge)` if `max_edge` is set. Brick: 8 vertices, 12 triangles. |
| `Circle` edge | `n = max(1, ceil(span / step))`, `step = min(2 acos(1 - chord/r), angle_tol)`, `ceil(r span / max_edge)`, at least 3 if closed. Uniform parameters. Sagitta `r (1 - cos(theta/2)) <= chord_tol` with `theta = span / n`. |
| other curves | bisection on `[lo, hi]`: split a segment while the chord deviation at 1/4, 1/2, 3/4 exceeds `chord_tol`, the turning angle at the midpoint exceeds `angle_tol / 2`, or the segment is longer than `max_edge` (depth cap 22, 2^20 segments). |
| planar face | ear clipping of the loops projected into the plane frame; boundary points only (exact for planes). |
| parameter-rectangle face | grid (below). |
| any other face | ear clipping in `(u, v)` from the pcurves (or surface projection for non-periodic surfaces); **boundary points only**. |

**Grid faces** are the curved faces whose loop in `(u, v)` is an axis-aligned rectangle (cylinder lateral, cone, sphere, torus and revolved or ruled patches). Rows follow the left and right sides' sample parameters; row `j` has `round(a + (b - a) j / R)` points between the bottom (`a` segments) and top (`b` segments) counts, evaluated with `eval_many` on the surface; the boundary rows are the shared edge polylines. Adjacent rows are joined by a merge ("zipper") triangulation, so unequal counts (cone, frustum) need no special case. A pole row (a degenerate edge) is expanded to the opposite side's count with every point mapped to the one pole vertex; the wedges that become degenerate (two equal indices) are dropped. The cylinder lateral therefore has exactly `2n` triangles and its area error equals the inscribed-prism error, below the plan's bound `2 pi h (r - r cos(theta/2))` (true value about a third of it).

Derived, asserted bounds:
- cylinder: `0 <= 2 pi r h - A <= 2 pi h (r - r cos(theta/2))`, `theta` read from the shared circle polyline;
- sphere: a triangle with vertices on a sphere and circumradius `R_c` has flat area `>= (1 - R_c^2/r^2)` times the area of its central projection (the projection factor is `r^2 d / s^3` with `d <= s <= r`), and the projections tile the sphere, so `A >= 4 pi r^2 (1 - max R_c^2/r^2)`; the test computes `R_c` from the actual triangles. It also asserts the deficit is below `theta_chord^2`.

## Polygon triangulation (`earcut.rs`)

Points are snapped to a `2^30` lattice over the larger bounding-box side; `orient` is exact in `i128`. Holes are joined by the nearest visible vertex of the merged polygon (segment touches no ring edge except at its ends, midpoint inside by an exact parity test). Ears must be strictly convex and contain no reflex vertex (inclusive, ignoring coordinate duplicates). Convex vertices never become reflex when an ear is cut, so the reflex list only shrinks and a convex polygon costs `O(n)`. If a full pass finds no ear, any convex tip is clipped without the containment test, else a flat vertex is dropped; both are last resorts for degenerate input (the fallback can emit overlapping triangles); an instrumented run of the whole test suite never reached it. Collinear boundary vertices (from `max_edge` splits) are kept as vertices of other ears, so no T-junction appears. Property tests triangulate random star polygons with and without a hole (`n - 2` or `n + m` triangles, all counter-clockwise, area equal to the shoelace area, every vertex used).

## Performance

Release build, informal (single run, 4-core container, sphere radius 1): 49,506 triangles in 2.9 ms at `chord = 1e-4`, 493,506 in 27 ms at `1e-5`, 4,932,840 in 0.32 s at `1e-6` (about 15 M triangles/s). A cylinder with 4,442-gon caps takes 1.2 ms. No criterion bench: the task does not name one.

## Known gaps

- **No normals buffer.** Clients flat-shade or compute vertex normals; a per-face normal pass would break index sharing at creases anyway.
- **Curved non-rectangular faces are resolved at the boundary only** (trimmed cylinder patches, NURBS faces with different sampling on opposite edges). Their interior chord error is not controlled; a constrained-Delaunay refinement on the lattice is the planned extension.
- **Grid requires matching left and right sides** (same count, same parameters within `1e-9`); otherwise the face falls back to the boundary-only path.
- **Faces with several loops on a curved surface** use the boundary-only path.
- **Implicit-only surfaces** and faces on periodic surfaces without pcurves are typed errors.
- **Scene box for the default tolerance** uses vertices and nine samples per edge, not the faces, so it can undershoot the true box (a sphere gives its meridian box).
- Nothing is cached in the `DerivedCache`; a query recomputes.
