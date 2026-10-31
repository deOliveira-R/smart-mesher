# Cubit parity checklist

Source: Coreform Cubit manual v2026.8 (quick reference + section index pages). We match **capabilities**, not syntax.
Each item maps to a crate (see `ARCHITECTURE.md`). Priority: **P0** = first vertical slice, **P1** = core parity, **P2** = full parity.

Gaps still to research (per-scheme manual pages not read yet): exact behavior of each scheme, the virtual-topology command set, healing commands, sizing functions, boundary layers, Sculpt.

## A. Environment & session — `sm-engine`, `sm-lang`, `sm-server`
- [x] P0 Command language (own syntax): entity lists, ranges, `all`, `except`, `in` filters (e.g. "surfaces in volume 2") (W2: sm-lang, tests/fix0.rs, fix1.rs, run.rs)
- [x] P0 Script files (play), comments, variables (replaces APREPRO) (W2: milestone.rs)
- [ ] P1 Script record: write a session's op history out as a script file (the timeline already stores every op)
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
- [x] P1 cylinder · sphere (W2: sm-model, tests/primitives.rs)
- [ ] P1 frustum/cone (elliptic, top radius) · prism (n sides) · pyramid · sphere (with inner radius, octant options) · torus
Bottom-up (P1):
- [ ] vertex (coords, on curve, intersection) · curve (line, arc, circle, ellipse, spline through/control points, offset, projection) · surface (from curves, planar, net, offset, sweep, skin/loft) · volume (from surfaces, sweep/revolve/extrude surface, thicken sheet)
Booleans (P1):
- [x] unite · subtract · intersect (with `keep`), regularization (W3a: sm-boolean, tests/boolean_planar.rs, quadric_ops.rs, marched.rs)
- [ ] mixed-dimension unite
Transforms (P1):
- [x] move · rotate · scale · reflect (W2: sm-model, tests/transforms.rs; which axis/vertex/normal forms exist not audited)
- [ ] move (delta, to location) · rotate (axis, two vertices, surface normal) · copy variants
Decomposition, i.e. webcut (P1):
- [x] plane by vertices / surface / axis plane + offset · cylinder (and sphere) · imprint/merge options (`nomerge`) · section (keep half) (W3a: sm-boolean, tests/webcut.rs, ops.rs, quadric_ops.rs)
- [ ] tool body · sheet body/surface · extended surface
Imprint & merge (P1). This is how conformal multi-volume meshes work:
- [x] imprint bodies · merge all/tolerant · merge checks (W3a: sm-boolean, tests/imprint.rs, merge.rs, ops_merge.rs; conformal_mesh.rs shows the shared interface meshes conformally)
- [ ] imprint curves on surfaces · overlap detection (beyond the partial-overlap imprint)
Virtual topology (P1):
- [ ] composite surfaces/curves · partition surface/curve · collapse curve/angle · remove virtual
Cleanup / healing / defeaturing (P2):
- [ ] small curve/surface detection, sliver removal, blend removal, hole removal, heal (gaps, tolerances), cleanup angle
Import (P1 STEP, STL; P2 rest):
- [x] STEP AP203/214/242 B-rep, content-addressed `model.import_step`, assemblies (W3a: sm-cadio, tests/acceptance.rs, roundtrip.rs, assembly_bomb.rs; milestone_w3a.rs path A)
- [ ] IGES · SAT · STL/facets (as faceted geometry) · MCNP/OpenMC CSG · native project (our op log). Mesh-based geometry: deferred with mesh import.
Export (P1):
- [ ] STEP · IGES · facets/STL (STEP writer exists in sm-cadio, tests/roundtrip.rs, but is not an op yet)
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
- [x] P1 submap (W3a: sm-scheme-submap, tests/submap.rs; corner picking not checked, corner types derived from the loop)
- [x] P1 pave (unstructured quad, CDT + matching, any number of loops) (W3a: sm-scheme-pave, tests/driver.rs)
- [x] P1 trimesh/tridelaunay (constrained Delaunay + Ruppert/Chew) · triadvance (advancing front) (W3a: sm-scheme-tri, tests/accept_brick.rs, accept_planar.rs, front.rs)
- [ ] P1 tripave · trimap · triprimitive · circle · hole · pentagon · polyhedron · copy · mirror · auto selection
Volume schemes:
- [x] P0 map (W2: sm-scheme-mapvol; milestone.rs row 6)
- [x] P1 sweep, 1-to-1 only: translate, twist, linking surfaces, rotation about an outside axis; quad->hex, tri->wedge, polygon->prism (W3a: sm-scheme-sweep, tests/bricks.rs, curved.rs; many-to-1, multisweep, projected target, 360 degree revolve open)
- [x] P1 tetmesh (Delaunay + boundary recovery, interior Steiner points only, quality floor) (W3a: sm-scheme-tet, tests/a02_cube.rs, a04_cavity.rs, a08_quality.rs, adapter.rs; hybrid hex/wedge/tet in sm-cli tests/milestone_w3a.rs)
- [ ] P1 volume submap · sweep many-to-1 / many-to-many multisweep · copy · auto selection
- [ ] P2 sphere · tetprimitive · polyhedron · radialmesh (free mesh) · Sculpt (overlay-grid hex, parallel)
Conversion:
- [ ] P1 THex (tet→hex) · QTri (quad→tri, 2/4 split) · HTet (hex→tet) · TQuad (tri→quad)
Element types (via `sm-fem`):
- [x] P0 HEX8 (W2: sm-fem; milestone.rs)
- [x] P0 QUAD4 (W3a: sm-scheme-pave, tests/driver.rs) · TET4 (W3a: sm-scheme-tet, tests/a02_cube.rs) · P1 WEDGE6 (W3a: sm-scheme-sweep, tests/curved.rs)
- [ ] P1 HEX20/27, TET10, TRI6, QUAD8/9, WEDGE15, PYRAMID5/13, BAR2/3, SHELL · P2 TETRA14/15, WEDGE16+, cohesive, superelements
- [x] **beyond Cubit:** native polyhedral volume cells: median dual of a tet mesh (`mesh.to_poly`), agglomeration (W3a: sm-poly, tests/acceptance.rs, agglomerate.rs; milestone_w3a.rs row 12; non-star dual cells are split into wedges, INT-1)
Quality (P0 basic, P1 full):
- [x] Verdict-style metrics per element type (scaled Jacobian, aspect ratio, skew, taper, condition number, shape, etc.), thresholds, quality reports, validity (inverted elements) (W3a: sm-quality, tests/verdict_ref.rs, elements.rs, mesh.rs)
- [x] Finite-volume metrics (non-orthogonality, skewness, cell determinant, face shape) against analytic values (W3a: sm-quality, tests/fv.rs)
- [x] **beyond Cubit:** diagnostics ops `mesh.quality` (histograms, worst cells, smooth suggestion), `mesh.validate`, `mesh.flag_cells` (W3a: sm-diag, tests/quality.rs, validate.rs, flag.rs)
Smoothing (P1):
- [x] laplacian · smart laplacian · centroid area pull · winslow (2-D all-quad only) · condition number / mean-ratio (optimization); classification-constrained and never inverts a cell; `mesh.smooth` op (W3a: sm-smooth, tests/decks.rs, winslow.rs, optimize.rs, never_inverts.rs, op.rs)
- [ ] equipotential · untangle (inverted input is reported, not fixed) · randomize · feature-preserving
Modification (P2):
- [ ] refine (local/uniform, templates) · coarsen · pillow · cleanup · intersection removal · node repositioning · collapse edges · align · create/merge elements · tet mesh matching · remesh · scale · skin · cohesive element insertion
Boundary layers (P1): layered hex/wedge inflation on surfaces/curves
Mesh deletion (P1): delete mesh with propagation · free meshes. **Mesh import: deferred by decision** (we import geometry, export meshes).

## D. FE model — `sm-fem`
- [x] P0 blocks (element blocks) with element type (W2: sm-fem; type checked at export, written to `.smsh` in W3)
- [x] P1 sidesets (surface, wrt volume), several sidesets per face (W3a: sm-fem, tests/ops.rs; milestone_w3a.rs row 9, areas checked)
- [ ] P1 sidesets forward/reverse/both, sheet · nodesets · materials · block attributes · global element IDs
- [ ] P2 non-Exodus BCs (displacement, force, pressure, temperature, heat flux, convection, contact) · BC sets · CFD BC codes

## E. Export — `sm-meshio`
- [x] P0 native `.smsh` · VTU (inspect in ParaView) (W1: sm-meshio, round-trip tested)
- [x] P1 Exodus II: blocks, side sets, node sets, NFACED/NSIDED polyhedra, CDF-1/2/5 (large model), pure-Rust writer, `mesh.exo` (W3a: sm-exodus, tests/exodus_write.rs, sides.rs, netcdf.rs; milestone_w3a.rs row 10; no netCDF-4/HDF5)
- [x] OpenFOAM polyMesh (incl. polyhedral cells, cell zones, patches, split of near-degenerate cells with a cell map) (W3a: sm-meshio, tests/foam.rs, foam_split.rs, foam_parse.rs; milestone_w3a.rs row 11; checkMesh run via tools/conformance, INT-8/INT-9)
- [ ] P1 Exodus netCDF4, Gmsh msh4, Abaqus (.inp, partial/explicit), DAGMC (h5m, relevant for transport)
- [ ] P2 Nastran, CGNS, I-DEAS UNV, Patran, LS-DYNA, Fluent, Sierra, VRML
- [ ] Export options: by block/sideset/nodeset, 2D/3D dimension, renumbering, everything/overwrite

## F. Beyond-Cubit requirements (ours)
- [x] Timeline: undo/redo, checkout any node, fork (W2: milestone.rs row 2; compare branches is P1)
- [x] Git-native project format (W1: sm-engine)
- [ ] Remote backend + web frontend + headless (W2: protocol server over stdio and headless CLI done; remote transport and web frontend open)
- [x] MCP server for agent control (W2: sm-mcp, MCP 2025-06-18 stdio; milestone.rs)
- [x] Deterministic replay (bit-identical state hashes) (W1: sm-engine strict replay + 1/N-thread CI)
- [x] Polyhedral-first native format: `.smsh` and VTU carry POLYHEDRON cells (W3a: sm-meshio, tests/smsh.rs, vtu.rs; milestone_w3a.rs row 12)
- [x] Ray tracing / particle tracking on the mesh: cell-to-cell walker on polyhedral meshes, point location, `.smrt` cache, `rt.info`/`rt.check`/`rt.locate` queries, C/Fortran ABI for transport codes (W3a: sm-rt, tests/locate.rs, cache.rs, chords.rs; sm-rt-capi, tests/c_abi.rs; milestone_w3a.rs row 13)
- [ ] GPU-accelerated smoothing/quality (later)
