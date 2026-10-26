# Gmsh as a polyhedral source or format: findings

Question (user, 2026-10-25): does Gmsh build polyhedral meshes (tets merged into hexes/polyhedra), and is its `.msh` a good export format for polyhedra?

Tested on Gmsh 4.15.2 (pip wheel), unit cube, size 0.25. Scripts: session scratchpad `gmshtest/`.

## Generation

| Path | Result |
|---|---|
| Default 3D (Delaunay) | 1101 tets |
| `Mesh.Recombine3DAll=1` (any level/conformity, per-volume `setRecombine(3,1)`, Algorithm3D 1/10) | Unchanged: tets only. The Yamakawa-Shimada "HEXAHEDRA RECOMBINATOR" code is in the binary but never ran. |
| `Mesh.SubdivisionAlgorithm=2` | 4404 hexes = 4 × 1101: each tet split into 4 hexes, not merged |
| `Mesh.Dual=1` | Mesh unchanged. Help text: "Display the dual mesh obtained by barycentric subdivision", so it only changes the display |
| `Plugin(CutMesh)` level-set cut | The only producer of Polygon (type 34) / Polyhedron (type 35) elements |

Gmsh's manual says its 3D unstructured algorithms produce "tetrahedra, or tetrahedra and pyramids". Polyhedral meshes via the tet dual come from elsewhere (OpenFOAM `polyDualMesh`, STAR-CCM+, Fluent).

## Storage

- A Gmsh polyhedron is a set of sub-tetrahedra, and a polygon a set of sub-triangles. MSH 2.2 line from the test: `35 ... 12 125 126 127 445 125 445 127 300 127 300 445 449`, i.e. 12 nodes = 3 tets. It carries no face list and no shared-face identity.
- MSH 4.1 (the default) silently drops types 34/35: 400 polyhedra became 0 in the file.
- MSH 2.2 writes them in ASCII only ("Unable to write polygons/polyhedra in binary files").
- Reading the MSH 2.2 file back drops the polyhedra (only 901 tets returned).

## Conclusions for smart-mesh

1. `.msh` is not a polyhedral format: it has no faces, cannot be read back, and the current version drops polyhedra. Do not target it for poly output.
2. Gmsh is still the right quality comparator for tet meshes (W3 milestone), and an optional `.msh` 4.1 writer for tet/hex/prism/pyramid is cheap interop.
3. Building polyhedra from tets (dual, or tet agglomeration with planarity/convexity control) is a capability Gmsh lacks. It is ours to build, and a differentiator for the transport code.
4. Face-based polyhedral formats to position against: CGNS `NGON_n`/`NFACE_n` (conceptual start per vision), OpenFOAM `polyMesh`, VTK `VTK_POLYHEDRON`/VTU, Exodus `NFACED`, MED polyhedra. Our `.smsh` poly layout should be face-based CSR (faces → nodes, cells → signed faces) with owner/neighbour, which also suits ray tracing (each face crossing is one plane test).
