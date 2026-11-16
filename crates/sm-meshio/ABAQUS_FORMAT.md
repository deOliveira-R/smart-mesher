# Abaqus export (`sm_meshio::abaqus`)

`write_abaqus` writes an Abaqus input deck (`.inp`) that carries the mesh and its named sets:
nodes, one `*ELEMENT` block per element type, `*ELSET` per cell set (block), `*NSET` per node set
and element-based `*SURFACE` per face or side set. It is model data without a `*PART`; sections,
materials, steps and boundary conditions belong to the analysis, not to the mesh. Output is a pure
function of the mesh and options (same bytes for any thread count). The file is ASCII, `\n` line
ends, no timestamp.

## Layout

```
*HEADING
smart-mesh export
*NODE
1, 0.0, 0.0, 0.0
...
*ELEMENT, TYPE=C3D4
...
*ELEMENT, TYPE=C3D5          (only with pyramids = c3d5)
*ELEMENT, TYPE=C3D6
*ELEMENT, TYPE=C3D8
*ELSET, ELSET=<block>
1, 2, 3, ...                 (16 ids per line, ascending)
*NSET, NSET=<set>
*SURFACE, TYPE=ELEMENT, NAME=<face set>
<element id>, S<n>
```

* Node id = vertex index + 1, in vertex order. Vertices no cell uses are still written.
* Element id = source cell index + 1 (gaps where cells are dropped); decomposition tets follow
  after the last cell id. Blocks of one type list their elements in ascending id.
* Reals are the shortest round-trip form, always with a `.` (`1.0e-7`, never `1e-7`).
* Data lines hold at most 16 values and never end with a comma.

## Element types and node order

The node order is the template order of `sm_mesh_api` (the corner order of `MeshRead::cell_corners`),
which equals the Abaqus order with positive volume. Each row is checked by a test that computes the
signed volume (and the smallest Jacobian determinant) from the written connectivity and compares it
with the analytic value of the unit fixture to 1e-14.

| Cell | Type | Nodes (1-based, positions in the list) | Unit fixture volume |
|---|---|---|---|
| hexahedron | `C3D8` | 1-4 bottom face counter-clockwise seen from above, 5-8 the top face above them | 1 |
| wedge | `C3D6` | 1-3 bottom triangle counter-clockwise seen from above, 4-6 the top triangle above them | 1/2 |
| tetrahedron | `C3D4` | 1-3 counter-clockwise seen from node 4 | 1/6 |
| pyramid | `C3D5` or collapsed `C3D8` | 1-4 base as for the hexahedron, 5 the apex | 1/3 |

### Pyramids (`pyramids`)

Abaqus' own element library has no pyramid we could confirm, and the only reader in the conformance
environment (meshio 5.3.5) does not know `C3D5`. So the default is a typed refusal
(`PyramidsUnsupported`). The two choices:

* `c3d5`: nodes 1-4 the base, node 5 the apex; faces `S1` base, `S2..S5` the sides (table below).
  Loss `abaqus.pyramid_type_unverified` (Info). Use it for a reader that has the type.
* `degenerate_hex`: a `C3D8` with nodes 5 to 8 all at the apex, which Abaqus accepts. The Jacobian
  is singular at the apex corner, so the report says `abaqus.pyramid_degenerate_hex` (Degraded).
  Surface faces use the hexahedron numbers `S1` (base) and `S3..S6` (sides); `S2` has zero area and
  is never used. meshio reads it as a hexahedron.

**Deviation from the draft.** The draft had pyramids follow the polyhedron policy while `C3D5`
was unconfirmed. The writer has its own `pyramids` option instead (default `error`, `c3d5`,
`degenerate_hex`). The default behaves the same (a typed refusal), and `degenerate_hex` is the
conforming choice next to hexahedra. The default for hex-tet transition meshes (W4-HEX-PYR) is
therefore `error`; whether it should change is the orchestrator's or the user's call.

## Face numbering (`*SURFACE`)

Element-based surfaces name a face of an element by `S<n>`. The tables below are the Abaqus face
numbers; the node positions list the nodes of the face. A test writes every face of each unit
fixture as its own set and checks that the face named by `Sn` has the outward unit normal in the
last column to 1e-14.

| Type | `S1` | `S2` | `S3` | `S4` | `S5` | `S6` |
|---|---|---|---|---|---|---|
| `C3D8` nodes | 1-2-3-4 | 5-6-7-8 | 1-2-6-5 | 2-3-7-6 | 3-4-8-7 | 4-1-5-8 |
| `C3D8` normal | -z | +z | -y | +x | +y | -x |
| `C3D6` nodes | 1-2-3 | 4-5-6 | 1-2-5-4 | 2-3-6-5 | 3-1-4-6 | |
| `C3D6` normal (unit wedge) | -z | +z | -y | (1,1,0)/sqrt 2 | -x | |
| `C3D4` nodes | 1-2-3 | 1-4-2 | 2-4-3 | 3-4-1 | | |
| `C3D4` normal (unit tet) | -z | -y | (1,1,1)/sqrt 3 | -x | | |
| `C3D5` nodes | 1-2-3-4 | 1-2-5 | 2-3-5 | 3-4-5 | 4-1-5 | |
| `C3D5` normal (unit pyramid) | -z | (0,-1,1/2)/n | (1,0,1/2)/n | (0,1,1/2)/n | (-1,0,1/2)/n | |

(`n = sqrt 5 / 2`.) A face is matched to its element face by the set of node ids, so the numbering
does not depend on the loop direction stored in the mesh.

## Sets

| Mesh set | Written as |
|---|---|
| cell set | `*ELSET` with the written element ids of its cells, plus (option `nsets`, default on) an `*NSET` of the same name with the nodes of those elements |
| face set | `*SURFACE` with one entry per face, taken from the face's left cell; faces between two cells use the left cell and are counted in `abaqus.surface_interior_face` (Info). Plus an `*NSET` of the face nodes |
| side set | `*SURFACE` with the cell named by each side (`Right` needs a two-cell face); plus the `*NSET` |
| vertex or point set | `*NSET` (members that are no vertex, such as high-order nodes, are counted in `abaqus.set_member_not_exported`) |
| shell-side set | dropped (`abaqus.set_dropped`); a deck is dimension 3 only |

The `*NSET` shares the name of its `*ELSET` or `*SURFACE` (Abaqus keeps one name space per kind).
A set with no exportable member is dropped and reported (`abaqus.set_dropped`), because Abaqus
rejects an empty set.

### Names

Abaqus labels are case-insensitive, at most 80 characters, and a leading underscore is reserved. A
name is mapped to `[A-Za-z][A-Za-z0-9_]*` (any other character becomes `_`; a name that does not
start with a letter gets the prefix `s_`; it is cut to 80 characters). Names must be unique
ignoring case over every set of the file: a clash gets the suffix `_2`, `_3`, ... Every change is
reported as `abaqus.name_changed` (Info, `'old' -> 'new'`).

## Polyhedra (`polyhedra`)

Abaqus has no general polyhedron element. The policy is the shared `PolyPolicy` (also the Gmsh
writer's, re-exported there as `sm_meshio::gmsh::PolyPolicy`):

* `error` (default): `PolyhedraUnsupported { count }`, nothing written.
* `decompose`: each polyhedron becomes `C3D4` tets around a cell centroid, with a face centroid per
  polygon of 4 or more nodes (new nodes after the mesh's own, ids after the vertices). Allowed only
  when every neighbour across a polygonal face is decomposed too (else `NonconformingDecomposition`),
  so the result is conforming. The tets keep the cell's `ELSET` membership; a boundary face of a
  decomposed polyhedron is `S1` of each tet that touches it, so a `*SURFACE` covers the face by
  its fan. Volume is conserved to 1e-12 for planar-faced polyhedra (tested on the truncated
  octahedron, hexagonal prism, split-face cube and the honeycomb dual); the summed surface area equals the analytic boundary area.
  Loss `abaqus.poly_decomposed` (Degraded), `abaqus.nodes_added` (Info).
  The fan is shared with the Gmsh writer (`poly.rs`). A tet with volume <= 0 (a polyhedron that
  is not star-shaped about its centroid, or a flat face) is refused as `InvertedDecomposition`,
  and element ids that would pass `u32::MAX` as `TooLarge`.
* `drop`: polyhedra are left out, ids of the others are unchanged, `abaqus.poly_dropped` (Dropped).
  A mesh with nothing left to write is `Unsupported`, not an empty deck.

## Limits

* Dimension-3 meshes only; inactive (refined parent) cells and non-finite coordinates are refused
  (`Unsupported`).
* High-order cells are written as their linear corner types (`abaqus.high_order_dropped`,
  Degraded); `C3D10`/`C3D20` are not written because the mesh API's high-order node order has not
  been checked against Abaqus.
* No `*MATERIAL`, `*SOLID SECTION`, `*BOUNDARY`, `*STEP`: the elsets are what a section is assigned
  to.
* We do not have Abaqus and had no access to its manual in this task. No licensed reader judged the
  file; the face tables above are the Abaqus numbering as we know it (C3D8 `S1` bottom, `S2` top, then
  the sides in node order; the tet and wedge tables likewise), checked only for self-consistency
  against the outward normals of our own fixtures. A user with Abaqus should confirm one deck per
  type with a `*DSLOAD`/`*DLOAD` on a surface before relying on `S2..S6`. The pyramid type is
  unconfirmed (above).

## Verification

* `tests/abaqus.rs`: signed volume and smallest Jacobian per type from the written connectivity
  against the analytic unit volumes (1e-14); outward normal of every `Sn` for each type; surface
  area equals the analytic skin area (hex block and decomposed honeycomb); sets, names, policies,
  loss codes, typed errors; bytes equal across runs and for 1 and N threads (a mesh of 9,000 cells
  spans several parallel chunks); property test over random hex and tet blocks.
* **meshio oracle** (`tests/abaqus_meshio.py`, run by `meshio_reads_the_decks_with_equal_counts_volumes_and_sets`):
  meshio's Abaqus reader (`meshio==5.3.5`, the version pinned in `tools/conformance/requirements.txt`)
  reads each deck; cell counts per type equal the writer's report, the volume computed from the
  connectivity meshio returns equals the analytic volume to 1e-12, every element set has its size and
  a node set of the same name exists. The test uses `SM_PYTHON` (else `python3`) and prints a
  skip line when `import meshio` fails, so it runs in the conformance venv
  (`SM_PYTHON=target/conformance/venv/bin/python cargo test -p sm-meshio --test abaqus`).
  meshio does not read `*SURFACE` or `C3D5`; surfaces are checked by the parser in the test.
* `sm-server/tests/export_wire.rs` and `sm-py/tests/test_export.py`: `.inp` inference, byte equality
  in process, through the CLI and through MCP, options checked, polyhedra policy, sideset to surface.
