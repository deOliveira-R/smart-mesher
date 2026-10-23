# Cubit parity checklist

Source: Coreform Cubit manual v2026.8 (quick reference + section index pages). We match **capabilities**, not syntax.
Each item maps to a crate (see `ARCHITECTURE.md`). Priority: **P0** = first vertical slice, **P1** = core parity, **P2** = full parity.

Gaps still to research (per-scheme manual pages not read yet): exact behavior of each scheme, the virtual-topology command set, healing commands, sizing functions, boundary layers, Sculpt.

## A. Environment & session — `sm-engine`, `sm-lang`, `sm-server`
- [ ] P0 Command language (own syntax): entity lists, ranges, `all`, `except`, `in` filters (e.g. "surfaces in volume 2")
- [ ] P0 Script files (record/play), comments, variables (replaces APREPRO)
- [ ] P0 Headless/batch mode
- [ ] P1 Groups: add/remove/equals/xor, unite/intersect/subtract groups
- [ ] P1 Entity naming, persistent IDs, ID renumbering controls
- [ ] P1 Undo/redo; **beyond Cubit:** timeline DAG with fork/checkout
- [ ] P1 Listing/query: list entity, measure distance/angle/area/volume, bounding boxes
- [ ] P1 Python API: query plus op execution (Cubit's API is read-only plus `cmd`)
- [ ] P2 Settings system (warnings, verbosity, stop-on-error, memory limits, parallel meshing toggle)
- [ ] P2 Crash save / autosave (largely free with the op log)

## B. Geometry — `sm-geom`, `sm-topo`, `sm-model`, `sm-cadio`
Primitives (P0 brick, rest P1):
- [ ] brick · cylinder · frustum/cone (elliptic, top radius) · prism (n sides) · pyramid · sphere (with inner radius, octant options) · torus
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
- [ ] STEP · IGES · STL/facets · mesh-based geometry (import mesh as geometry) · MCNP CSG (relevant for transport!) · native format (our op log)
Export (P1):
- [ ] STEP · IGES · facets/STL
Attributes & metadata (P2): names, colors, assemblies/parts, attribute propagation through ops

## C. Meshing — `sm-mesh`, `sm-sizing`, `sm-scheme-*`, `sm-quality`, `sm-smooth`, `sm-modify`
Process: size → scheme → mesh (bottom-up hierarchy) → quality. Mesh attributes live on geometry entities.
Sizing / intervals:
- [ ] P0 interval count, size, auto size
- [ ] P1 hard/soft intervals, interval matching (integer program over all curves) · relative intervals · periodic intervals · vertex sizing · bias/dualbias · sizing functions · auto size function · mesh preview
Curve schemes:
- [ ] P0 equal · P1 bias · dualbias · curvature · stretch · pinpoint · copy
Surface schemes:
- [ ] P0 map · P1 submap (corner picking) · pave (unstructured quad) · trimesh/tridelaunay (Delaunay, guaranteed quality) · triadvance (advancing front) · tripave · trimap · triprimitive · circle · hole · pentagon · polyhedron · copy · mirror · auto selection
Volume schemes:
- [ ] P0 map · P1 submap · sweep (1-to-1, many-to-1, many-to-many multisweep; rotational) · tetmesh (Delaunay + boundary recovery, quality) · copy · auto selection
- [ ] P2 sphere · tetprimitive · polyhedron · radialmesh (free mesh) · Sculpt (overlay-grid hex, parallel)
Conversion:
- [ ] P1 THex (tet→hex) · QTri (quad→tri, 2/4 split) · HTet (hex→tet) · TQuad (tri→quad)
Element types (via `sm-fem`):
- [ ] P0 HEX8, QUAD4, TET4 · P1 HEX20/27, TET10, TRI6, QUAD8/9, WEDGE6/15, PYRAMID5/13, BAR2/3, SHELL · P2 TETRA14/15, WEDGE16+, cohesive, superelements
- [ ] **beyond Cubit:** native polyhedral volume cells (dual meshes, agglomeration)
Quality (P0 basic, P1 full):
- [ ] Verdict-style metrics per element type (scaled Jacobian, aspect ratio, skew, taper, condition number, shape, etc.), thresholds, quality reports, validity (inverted elements)
Smoothing (P1):
- [ ] laplacian · centroid area pull · equipotential · winslow · condition number (optimization) · untangle · randomize · smart/feature-preserving
Modification (P2):
- [ ] refine (local/uniform, templates) · coarsen · pillow · cleanup · intersection removal · node repositioning · collapse edges · align · create/merge elements · tet mesh matching · remesh · scale · skin · cohesive element insertion
Boundary layers (P1): layered hex/wedge inflation on surfaces/curves
Mesh import/deletion (P1): import mesh (Exodus, ours, others) · delete mesh with propagation · free meshes

## D. FE model — `sm-fem`
- [ ] P0 blocks (element blocks) with element type
- [ ] P1 sidesets (surface, wrt volume, forward/reverse/both, sheet) · nodesets · materials · block attributes · global element IDs
- [ ] P2 non-Exodus BCs (displacement, force, pressure, temperature, heat flux, convection, contact) · BC sets · CFD BC codes

## E. Export — `sm-meshio`
- [ ] P0 native `.smsh` · VTU (inspect in ParaView)
- [ ] P1 Exodus II (incl. large model / netCDF4), Gmsh msh4, Abaqus (.inp, partial/explicit), DAGMC (h5m, relevant for transport)
- [ ] P2 Nastran, CGNS, I-DEAS UNV, Patran, LS-DYNA, Fluent, Sierra, VRML
- [ ] Export options: by block/sideset/nodeset, 2D/3D dimension, renumbering, everything/overwrite

## F. Beyond-Cubit requirements (ours)
- [ ] Timeline: undo/redo, checkout any node, fork, compare branches
- [ ] Git-native project format
- [ ] Remote backend + web frontend + headless
- [ ] MCP server for agent control
- [ ] Deterministic replay (bit-identical state hashes)
- [ ] Polyhedral-first native format
- [ ] GPU-accelerated smoothing/quality (later)
