# Fluent ASCII `.msh` output, as written by `sm_meshio::fluent`

Interchange format only (ruling D2): the transport code reads `.smsh`. The spec is
`docs/research/fluent-msh.md`; the example files and checker are in `docs/research/fluent-msh/`.

**Limit, stated plainly.** There is no Fluent licence (ruling D10). No file written here has been
read by Fluent. The oracle is OpenFOAM v2506: `fluent3DMeshToFoam` reads it and `checkMesh
-allGeometry -allTopology` finds no orientation, topology or volume error. Where Fluent's own
reading is unverified, section 6 says so and the writer takes the choice that W-OF (OpenFOAM's
`foamMeshToFluent`, which Fluent loads) also makes.

## 1. Sections written, in order

```
(0 "smart-mesh Fluent export")        comments, header, dimension (F3D needs `(2 3)`)
(1 "smart-mesh")
(0 "Dimension:")
(2 3)
(0 "Declarations:")
(10 (0 1 N 0 3))                      declarations: node, cell and face counts, hex
(12 (0 1 M 0))
(13 (0 1 F 0))
(10 (1 1 N 1 3)(                      one node zone, id 1, type 1, one node per line
x y z
))
(12 (id first last 1 type))           one section per cell zone; a body only when mixed
(12 (id first last 1 0)( t t t ))     mixed: one hex type code per cell, 40 per line
(13 (id first last bc ftype)(         interior zone first, then one zone per boundary set
...one face per line...
))
(39 (id type name)())                 every zone, 3-field form, id in decimal
```

All integers in 10/12/13 are lowercase hex; ids in 39 are decimal. No 58/59 hanging-node
sections and no binary variant: polygons are written conformally. Reals are the shortest
round-trip decimal and always carry a `.` (`1.0e-7`, not `1e-7`).

## 2. Orientation

The input's face loop is outward from `left`. Fluent's rule (research 7.3, confirmed by
`checkMesh` on a file and on the same file with every face reversed, which it rejects): the
right-hand normal of the written loop points **toward c0**.

| face | nodes | c0 | c1 |
|---|---|---|---|
| interior | as stored | `right` | `left` |
| boundary | reversed | `left` | 0 |

The tests recompute signed cell volumes from the written text alone (fan about the first node,
c0 negative, c1 positive) and compare them with the analytic volumes to 1e-12 relative; a
reversed file gives negative volumes.

## 3. Cells and zones

- Cells are renumbered by cell zone (ascending source index within a zone) so every zone is one
  contiguous range. `FluentReport::cell_of_fluent` is the map.
- Cell zones come from the mesh's cell sets (blocks, regions, materials after lowering) in set
  order. A cell in two sets goes to the first one (`fluent.cell_zone_overlap`); cells in none go
  to the zone `fluid` (`fluent.default_cell_zone`). Option `cell_zones`: `"all"` (default),
  `"none"` or `{"names": [..]}`.
- Boundary zones come from face and side sets (`PatchSource::Sets`) in set order; faces no set
  claims go to `default_faces`. Type is `wall` (bc 3) unless `boundary_types` maps the set name to
  one of `wall`, `symmetry`, `pressure-inlet`, `pressure-outlet`, `velocity-inlet`,
  `mass-flow-inlet`, `outflow`, `pressure-far-field` (written as bc 3, 7, 4, 5, a, 14, 24, 9 in
  hex, that is decimal 3, 7, 4, 5, 10, 20, 36, 9). Zones left at `wall` are reported (`fluent.boundary_type_default`).
- One `interior` zone (bc 2) holds every interior face, also those between two cell zones.
- Zone ids are unique over node, cell and face zones: node 1, cell zones 2.., interior, then
  boundary zones.
- Names are lowercased to `[a-z][a-z0-9_.-]*` and made unique with `_2`, `_3` (`fluent.name_changed`).
  The Scheme rule wants lowercase and the OpenFOAM lexer is narrower than the manual.
- Element type from the face census of the cell, never from its source shape: 4 triangles = tet
  (2), 6 quads = hex (4), 2 triangles + 3 quads = wedge (6), 4 triangles + 1 quad = pyramid (5);
  anything else, including a hex with a split face, is a polyhedron (7). A zone of one type has
  no body.
- Faces of a zone are all triangles (type 3), all quads (4), or mixed (0, with a count prefix on
  every line).

## 4. Refused or reported

A dimension-2 mesh, inactive (refined parent) cells, non-finite coordinates and unknown sets are
typed errors (the checks of the OpenFOAM exporter, which this writer builds on: faces ordered
`[interior][zone..]`, owner = `left`). High-order cells are written as their linear corners
(`fluent.high_order_dropped`). Non-conformal and partition-cut faces are plain boundary zones.

## 5. Conformance

`crates/sm-meshio/tests/fluent.rs`, with `SM_CONFORMANCE=1` and the image
`opencfd/openfoam-default:2506` (override `SM_CONFORMANCE_IMAGE`): a hex brick, the mixed
transition, the polyhedral honeycomb flower, a hexagonal prism and a truncated octahedron.
Checks: `fluent3DMeshToFoam` converts; `checkMesh` prints no finding other than the two
documented heuristics (small determinant on tiny meshes, concave face-plane test on coplanar
sides); the polyMesh read back has our cell, face and interior-face counts and the same sorted
cell volumes to 1e-12 relative; every cell zone name is in OpenFOAM's `cellZones`; the same file
with every face reversed fails `checkMesh`. The work directory is under the cargo target
directory (Docker Desktop on macOS does not mount `/var/folders`), and the file is passed to
`fluent3DMeshToFoam` by absolute path (a relative name is not found with `-case`).

## 6. Unverified against Fluent itself

Research section 14 items that bear on the writer: the c0/c1 rule for Fluent (item 1, OpenFOAM
agrees), the 3-field 39 header without domain-id (item 13, as W-OF), cell type labels checked
against the face set (item 18, the census rule avoids a mismatch), no limits on polygon size
(item 19), unique ids across node and cell zones (item 20, safe choice), interior faces first
(item 16, as every manual example). Cell zone type is always `fluid`; `solid` and the
non-boundary face types (fan, radiator, porous-jump) are not written.
