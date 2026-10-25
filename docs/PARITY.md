# Cubit parity checklist

Source: Coreform Cubit manual v2026.8 (quick reference + section index pages). We match **capabilities**, not syntax.
Each item maps to a crate (see `ARCHITECTURE.md`). Priority: **P0** = first vertical slice, **P1** = core parity, **P2** = full parity.

Gaps still to research (per-scheme manual pages not read yet): exact behavior of each scheme, the virtual-topology command set, healing commands, sizing functions, boundary layers, Sculpt.

## A. Environment & session — `sm-engine`, `sm-lang`, `sm-server`
- [x] P0 Command language (own syntax): entity lists, ranges, `all`, `except`, `in` filters (e.g. "surfaces in volume 2") (W2: sm-lang, tests/entity.rs, run.rs)
- [x] P0 Script files (play), comments, variables (replaces APREPRO) (W2: milestone.rs; "record" = the timeline op log; writing a session back out as a script is P1)
- [x] P0 Headless/batch mode (W2: `smesh` CLI, `serve --stdio`, milestone.rs)
- [ ] P1 Groups: add/remove/equals/xor, unite/intersect/subtract groups
- [ ] P1 Entity naming, persistent IDs, ID renumbering controls
- [ ] P1 Undo/redo; **beyond Cubit:** timeline DAG with fork/checkout
- [ ] P1 Listing/query: list entity, measure distance/angle/area/volume, bounding boxes
- [ ] P1 Python API: query plus op execution (Cubit's API is read-only plus `cmd`)
- [ ] P2 Settings system (warnings, verbosity, stop-on-error, memory limits, parallel meshing toggle)
- [ ] P2 Crash save / autosave (largely free with the op log)

## B. Geometry — `sm-geom`, `sm-topo`, `sm-model`, `sm-cadio`
Primitives (P0 brick, rest P1):
- [x] P0 brick (W2: sm-model, milestone.rs; same hashes on TopoStore and CellComplex)
- [ ] P1 cylinder · frustum/cone (elliptic, top radius) · prism (n sides) · pyramid · sphere (with inner radius, octant options) · torus
Bottom-up (P1):
- [ ] vertex (coords, on curve, intersection) · curve (line, arc, circle, ellipse, spline through/control points, offset, projection) · surface (from curves, planar, net, offset, sweep, skin/loft) · volume (from surfaces, sweep/revolve/extrude surface, thicken sheet)
Booleans (P1):
- [ ] unite · subtract · intersect (with `keep`), regularization, mixed-dimension unite
Transforms (P1):
- [ ] move (delta, to location) · rotate (axis, two vertices, surface normal) · scale · reflect · copy variants
Decomposition, i.e. webcut (P1):
- [ ] plane by vertices / surface / axis plane + offset · tool body · sheet body/surface · extended surface · cylinder · imprint/merge options · section (keep half)
Imprint & merge (P1). This is how conformal multi-volume meshes work:
- [ ] imprint all/bodies/curves on surfaces · merge all/tolerant · overlap detection · merge checks
Virtual topology (P1):
- [ ] composite surfaces/curves · partition surface/curve · collapse curve/angle · remove virtual
Cleanup / healing / defeaturing (P2):
- [ ] small curve/surface detection, sliver removal, blend removal, hole removal, heal (gaps, tolerances), cleanup angle
Import (P1 STEP, STL; P2 rest):
- [ ] STEP · IGES · SAT · STL/facets (as faceted geometry) · MCNP/OpenMC CSG · native project (our op log). Mesh-based geometry: deferred with mesh import.
Export (P1):
- [ ] STEP · IGES · facets/STL
Attributes & metadata (P2): names, colors, assemblies/parts, attribute propagation through ops

## C. Meshing — `sm-mesh`, `sm-sizing`, `sm-scheme-*`, `sm-quality`, `sm-smooth`, `sm-modify`
Process: size → scheme → mesh (bottom-up hierarchy) → quality. Mesh attributes live on geometry entities.
Sizing / intervals:
- [x] P0 interval count, size, auto size (W2: sm-sizing + sm-mesher; end to end in sm-cli tests/p0_rows.rs)
- [ ] P1 hard/soft intervals, interval matching (integer program over all curves) · relative intervals · periodic intervals · vertex sizing · bias/dualbias · sizing functions · auto size function · mesh preview
Curve schemes:
- [x] P0 equal (W2: sm-scheme-curve; p0_rows.rs row 4)
- [ ] P1 bias · dualbias · curvature · stretch · pinpoint · copy
Surface schemes:
- [x] P0 map (W2: sm-scheme-map, inside volume runs; `mesh.run` on faces alone is W3)
- [ ] P1 submap (corner picking) · pave (unstructured quad) · trimesh/tridelaunay (Delaunay, guaranteed quality) · triadvance (advancing front) · tripave · trimap · triprimitive · circle · hole · pentagon · polyhedron · copy · mirror · auto selection
Volume schemes:
- [x] P0 map (W2: sm-scheme-mapvol; milestone.rs row 6)
- [ ] P1 submap · sweep (1-to-1, many-to-1, many-to-many multisweep; rotational) · tetmesh (Delaunay + boundary recovery, quality) · copy · auto selection
- [ ] P2 sphere · tetprimitive · polyhedron · radialmesh (free mesh) · Sculpt (overlay-grid hex, parallel)
Conversion:
- [ ] P1 THex (tet→hex) · QTri (quad→tri, 2/4 split) · HTet (hex→tet) · TQuad (tri→quad)
Element types (via `sm-fem`):
- [x] P0 HEX8 (W2: sm-fem; milestone.rs)
- [ ] P0 QUAD4, TET4 (types exist in sm-fem; need tri/tet and surface-only meshing) · P1 HEX20/27, TET10, TRI6, QUAD8/9, WEDGE6/15, PYRAMID5/13, BAR2/3, SHELL · P2 TETRA14/15, WEDGE16+, cohesive, superelements
- [ ] **beyond Cubit:** native polyhedral volume cells (dual meshes, agglomeration)
Quality (P0 basic, P1 full):
- [x] Verdict-style metrics per element type (scaled Jacobian, aspect ratio, skew, taper, condition number, shape, etc.), thresholds, quality reports, validity (inverted elements)
Smoothing (P1):
- [ ] laplacian · centroid area pull · equipotential · winslow · condition number (optimization) · untangle · randomize · smart/feature-preserving
Modification (P2):
- [ ] refine (local/uniform, templates) · coarsen · pillow · cleanup · intersection removal · node repositioning · collapse edges · align · create/merge elements · tet mesh matching · remesh · scale · skin · cohesive element insertion
Boundary layers (P1): layered hex/wedge inflation on surfaces/curves
Mesh deletion (P1): delete mesh with propagation · free meshes. **Mesh import: deferred by decision** (we import geometry, export meshes).

## D. FE model — `sm-fem`
- [x] P0 blocks (element blocks) with element type (W2: sm-fem; type checked at export, written to `.smsh` in W3)
- [ ] P1 sidesets (surface, wrt volume, forward/reverse/both, sheet) · nodesets · materials · block attributes · global element IDs
- [ ] P2 non-Exodus BCs (displacement, force, pressure, temperature, heat flux, convection, contact) · BC sets · CFD BC codes

## E. Export — `sm-meshio`
- [x] P0 native `.smsh` · VTU (inspect in ParaView) (W1: sm-meshio, round-trip tested)
- [ ] P1 Exodus II (incl. large model / netCDF4), Gmsh msh4, Abaqus (.inp, partial/explicit), DAGMC (h5m, relevant for transport)
- [ ] P2 Nastran, CGNS, I-DEAS UNV, Patran, LS-DYNA, Fluent, Sierra, VRML
- [ ] Export options: by block/sideset/nodeset, 2D/3D dimension, renumbering, everything/overwrite

## F. Beyond-Cubit requirements (ours)
- [x] Timeline: undo/redo, checkout any node, fork (W2: milestone.rs row 2; compare branches is P1)
- [x] Git-native project format (W1: sm-engine)
- [ ] Remote backend + web frontend + headless (W2: protocol server over stdio and headless CLI done; remote transport and web frontend open)
- [x] MCP server for agent control (W2: sm-mcp, MCP 2025-06-18 stdio; milestone.rs)
- [x] Deterministic replay (bit-identical state hashes) (W1: sm-engine strict replay + 1/N-thread CI)
- [ ] Polyhedral-first native format
- [ ] GPU-accelerated smoothing/quality (later)
