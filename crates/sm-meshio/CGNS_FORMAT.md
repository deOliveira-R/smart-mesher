# CGNS export (`sm_meshio::cgns`)

`write_cgns` writes a CGNS 4.x file in its HDF5 encoding: one `Base`, one unstructured `Zone`.
The oracle is the CGNS library (`cgnscheck`, and a C program that links `libcgns` and dumps what
it reads); the file is written with the private pure-Rust `h5` writer, so no HDF5 or CGNS
library is a dependency. Interchange only: the transport code reads `.smsh` (ruling D2).

## Node tree

Every CGNS node is an HDF5 group with four attributes and, when it has data, a child dataset named
` data` (a leading space). The attributes are, in this order: `name` and `label` (fixed ASCII
strings of 33 bytes, null-padded), `type` (3 bytes: `MT` no data, `I4`, `I8`, `R4`, `R8`, `C1`)
and `flags` (`int32[1]` = 1). The root group carries `name = "HDF5 MotherNode"`,
`label = "Root Node of HDF5 File"`, `type = MT` and two datasets, ` format`
(`"IEEE_LITTLE_32"`) and ` hdf5version` (33 bytes). `C1` data are `uint8` arrays without a
terminating NUL.

```
/  ( format) ( hdf5version)
  CGNSLibraryVersion  CGNSLibraryVersion_t  R4 [4.0]
  Base                CGNSBase_t            I4 [3,3]            (cell dim, physical dim)
    Zone              Zone_t                I4|I8 [3,1]         (vertices, cells, 0)
      ZoneType        ZoneType_t            C1 "Unstructured"
      GridCoordinates GridCoordinates_t     MT
        CoordinateX|Y|Z  DataArray_t        R8 [nverts]
      <sections>      Elements_t            I4 [type, 0]        see below
        ElementRange        IndexRange_t    I4|I8 [first,last]  1-based, one id space
        ElementStartOffset  DataArray_t     I4|I8 [n+1]         NGON_n and NFACE_n only
        ElementConnectivity DataArray_t     I4|I8 [..]          1-based vertex or face ids
      ZoneBC          ZoneBC_t              MT
        <bc>          BC_t                  C1 "FamilySpecified"
          PointList     IndexArray_t        I4|I8 [n,1]         element ids of the faces
          GridLocation  GridLocation_t      C1 "FaceCenter"
          FamilyName    FamilyName_t        C1 <bc>
      <block>         ZoneSubRegion_t       I4 [3]              (region dimension)
        GridLocation    GridLocation_t      C1 "CellCenter"
        PointList       IndexArray_t        I4|I8 [n,1]         element ids of the cells
        FamilyName      FamilyName_t        C1 <block>
    <bc>              Family_t              MT
      FamilyBC          FamilyBC_t          C1 BC type word
    <block>           Family_t              MT
```

Children are written in creation order (the order above). The CGNS library iterates groups by
creation order, so every group sets "link creation order tracked and indexed": the link-info
message carries flags 3 and the link messages carry their creation index. The spike
(two-cell file, then a library read) established this is enough; the library needs nothing else
beyond the subset of `h5/mod.rs` plus fixed strings and `f32`.

Index width: `I4` everywhere unless some id or the connectivity length exceeds `i32::MAX`, then
`I8` for the whole file (the zone data, element ranges, offsets, connectivity and point lists).
Element-type nodes are always `I4`.

## Layouts (`layout`)

* `auto` (default): when every cell is a tetrahedron, pyramid, wedge or hexahedron (decided by
  its face census, and a vertex order that closes: a hex with a split face is a polyhedron),
  standard sections; else `ngon`. The reason is a `cgns.layout_ngon` loss item (Info).
* `ngon`: always `NGON_n` and `NFACE_n`.

### Standard sections

Cells are renumbered by block (the cell sets that become sub-regions, in set order) and then by
element type `TETRA_4` (10), `PYRA_5` (12), `PENTA_6` (14), `HEXA_8` (17), source order inside.
One section per (block, type), named `<block>_<TYPE>` (`<TYPE>` for cells in no block). Boundary
faces follow the cells: per BC a `TRI_3` (5) run then a `QUAD_4` (7) run, named
`<bc>_TRI_3` and `<bc>_QUAD_4`, each loop outward from its cell. Element ids are one space from
1: all cell sections, then the boundary-face sections. Interior faces are not elements.

Node order is the CGNS (SIDS) order. The right-hand rule on the first face of each element
points into the element (`TETRA_4` 1-2-3 toward 4, `PYRA_5` 1-2-3-4 toward 5, `PENTA_6` 1-2-3
toward 4-5-6, `HEXA_8` 1-2-3-4 toward 5-6-7-8). The outward faces used by the tests:

| type | outward faces (1-based) |
|------|-------------------------|
| TETRA_4 | 1-3-2, 1-2-4, 2-3-4, 3-1-4 |
| PYRA_5 | 1-4-3-2, 1-2-5, 2-3-5, 3-4-5, 4-1-5 |
| PENTA_6 | 1-3-2, 4-5-6, 1-2-5-4, 2-3-6-5, 3-1-4-6 |
| HEXA_8 | 1-4-3-2, 5-6-7-8, 1-2-6-5, 2-3-7-6, 3-4-8-7, 4-1-5-8 |

### `NGON_n` / `NFACE_n`

`NGON_n` (22) holds every face as `ElementStartOffset` + vertex ids, ids `1..=nf`: interior faces
first (sorted by owner then neighbour), then one run per BC; each loop is outward from the owner
(right-hand normal out of the owner). `NFACE_n` (23) holds the cells, ids `nf+1..=nf+nc`, in
source order; each cell lists its faces as signed face ids: `+f` when the loop's normal points
out of the cell (the cell is the owner), `-f` when it points in (the neighbour). A closed
cell's signed face area vectors sum to zero. BC `PointList`s are face ids (a contiguous run).

## Boundary conditions, blocks, families

* BCs come from the face sets (option `patches`, as for the other exporters); faces in no set
  form the BC `default_faces`. A BC is `BC_t` `FamilySpecified` with a `FamilyName`; the family
  under the base carries `FamilyBC`. The type comes from `boundary_types` (set name to
  `user-defined`, `wall`, `inflow`, `outflow`, `symmetry-plane`, `farfield`; words
  `UserDefined`, `BCWall`, `BCInflow`, `BCOutflow`, `BCSymmetryPlane`, `BCFarfield`). The default
  is `UserDefined` (a face set has no physics), reported as `cgns.boundary_type_default`. A key
  naming no BC is `CgnsError::UnknownBoundary` (a bad request on the wire).
* Blocks come from the cell sets (option `cell_zones`: `all`, `none` or `{"names": [..]}`): a
  `ZoneSubRegion_t` with a `CellCenter` `PointList` of cell element ids and a `Family_t` of the
  same name. A cell in two sets stays in the first (`cgns.cell_zone_overlap`). Cells in no set
  belong to no sub-region.
* One zone, not one zone per block: blocks of one mesh share faces and vertices, and
  `NGON_n`/`NFACE_n` zones would need non-matching interfaces.
* Names: `[A-Za-z0-9_.+-]`, at most 32 bytes, unique over the whole file (BCs, blocks,
  sections, `Zone`); changes are reported (`cgns.name_sanitized` from the mesh builder,
  `cgns.name_changed` for the final spelling).

## Limits and what the library says

* No `DataClass`, units, `FlowSolution`, `GridConnectivity` or periodic data: `cgnscheck`
  reports 0 errors and exactly four recommendation warnings ("dataclass not given" three times,
  "no family name declared for zone"), the same four as for a file the library writes itself.
* The CGNS library version is written as 4.0 (`ElementStartOffset` needs 4.0). The HDF5 file is
  written in the 1.8 subset (superblock 2, object header 2): any HDF5 1.8+ reads it.
* Only 3-D meshes without inactive cells; vertices no face uses are dropped (as for Fluent).
* Vertex ids are `u32` in the mesh builder, so a mesh over 2^32 vertices is out of scope; ids
  over `i32::MAX` select `I8` automatically.
* A reader that wants a zone family finds none (`Zone` has no `FamilyName`): blocks are
  sub-regions.

## Verification

`tests/cgns.rs` rebuilds an independent model from the HDF5 tree (names, labels, type tags,
dtypes and shapes) and checks, on ten fixtures in both layouts: cell volumes from the divergence
theorem equal the analytic volumes to 1e-12 relative with positive sign; every cell closes
(signed area vectors sum to zero); every BC face is an outward loop of some cell; the BCs tile the
boundary and their area is the fixture's; NFACE references are signed correctly; a polyhedron of
five or more faces keeps its face list. Gated by `SM_CONFORMANCE=1` (image
`smesh-dagmc-conformance`, which carries `cgnscheck` and the compiler; rebuild it after
ACR-EXPFMT): `cgnscheck` reports 0 errors, the C reader (`tests/cgns_dump.c`, linked to
`libcgns`) reads the same counts, coordinates, sections, BCs, sub-regions and families we
wrote, and the same analytic checks pass on the library's data; a file whose `NFACE_n` names a
face that does not exist is caught by `cgnscheck`. Bytes are identical across runs and thread
counts. ParaView's reader was not run (no ParaView on the build host).
