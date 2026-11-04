# OpenFOAM `polyMesh` output (ASCII), as written by `sm_meshio::foam`

This is what `foam::build_polymesh` + `PolyMesh::files` produce and what `foam::parse_polymesh`
accepts. Status: hand-checked against the OpenFOAM v2506 file conventions; **not yet run through
`checkMesh`** (needs Docker; `tools/conformance.sh`, orchestrator-owned). Items whose spelling
OpenFOAM might treat differently are listed in section 7 so a conformance failure points at one
line of this file.

## 1. Why this is a reordering, not a conversion

The mesh IR's orientation rule is OpenFOAM's: a face loop's right-hand normal points from `left`
to `right`, `left` is the lower cell, a one-cell face points outward. So `owner = face_left`,
`neighbour = face_right`, and the stored loop is already outward from the owner. The exporter
only (a) orders the faces, (b) assigns patches, (c) copies cell sets into zones. There is no face
matching, no flipping, no cell renumbering (`renumberMesh` is the user's tool: `cellZones` labels
are the mesh's own cell indices), and polyhedra stay exact. polyMesh stores no cell shapes, so a
true 6-node prism is five faces (2 triangles, 3 quads) and needs no `wedge` handling. `wedge` and
`empty` patch types exist only when the user selects them; they are never inferred.

Inputs that are refused: a dimension-2 mesh (`NotThreeDimensional`; `empty` patches are W3b), a
mesh with inactive refined parents (`InactiveCells`; extract the leaf mesh first), a NaN or
infinite coordinate (`NonFinite`), an interior face with `left >= right` (`Orientation`; never
flipped silently).

## 2. Files

`constant/polyMesh/{points, faces, owner, neighbour, boundary}`, plus `cellZones` / `faceZones`
only when non-empty (`PolyMesh::write_dir` removes stale ones). All ASCII, LF line ends.

Every file starts with the OpenFOAM banner comment and

```
FoamFile
{
    version     2.0;
    format      ascii;
    arch        "LSB;label=32;scalar=64";
    class       <class>;
    note        "nPoints:8  nCells:1  nFaces:6  nInternalFaces:0";   // faces, owner, neighbour only
    location    "constant/polyMesh";
    object      <name>;
}
```

| file | class | body |
|---|---|---|
| `points` | `vectorField` | `N`, `(`, one `(x y z)` per line, `)` |
| `faces` | `faceList` | `N`, `(`, one `k(i0 i1 ...)` per line, `)` |
| `owner` | `labelList` | `N`, `(`, one label per line, `)` |
| `neighbour` | `labelList` | `n_internal` entries, same layout (`0` + `(` + `)` when empty) |
| `boundary` | `polyBoundaryMesh` | patch dictionaries (section 4) |
| `cellZones`, `faceZones` | `regIOobject` | zone dictionaries (section 5) |

Scalars are written with Rust's shortest round-trip text (`{:?}`: `1.0`, `0.1`, `1e-7`, `-0.0`),
so `parse_polymesh(files)` restores every coordinate bit for bit (pinned by a test with
subnormals, `f64::MAX`, `-0.0`). A non-finite number is an error, never written. Formatting runs
in parallel over fixed 4096-line chunks concatenated in order, so the bytes do not depend on the
thread count.

## 3. Ordering

* Cells keep their indices.
* Faces: `[internal][patch 0][patch 1]...`. Internal faces are those with a right cell and kind
  `Interior`, ordered by `(owner, neighbour, source index)`. A mesh in canonical face order
  already has that key non-decreasing and is taken as is; otherwise the faces are sorted
  (`par_sort_unstable_by_key`, the source index makes the key total, so the result is
  deterministic). Two cells sharing two faces tie on `(owner, neighbour)`: the source index
  breaks the tie and `check_polymesh` reports it as `foam.upper_triangular_tie` (warning level:
  checkMesh may want strictly increasing neighbours; no fixture has this).
* Patch faces keep ascending source index inside each patch.
* Loops are copied as stored: smallest vertex first, outward from the owner.
* Points are the mesh vertices in order. A vertex no face references is dropped and the rest
  renumbered in order (`FoamBuild::point_of_foam`; loss `foam.unused_points_dropped`).
  High-order extra points are never written (loss `foam.high_order_dropped`).

## 4. Patches

Patch order in the file: **named patches** (set order, listed order, or ascending model-face
serial), then `defaultFaces`, `nonConformal`, `partitionCut`; empty patches are omitted.

| `PatchSource` | rule |
|---|---|
| `Sets` (default) | every `Faces` or `Sides` set whose members are all `Boundary` faces (`Sides`: `Left` entries only) is a patch named by the set, taken in ascending set id; the first set to claim a face keeps it (rest: loss `foam.patch_overlap`); ineligible sets are loss `foam.set_skipped` |
| `Explicit(specs)` | only the listed sets, in listed order, with the given name and type; a face in two sets is `PatchOverlap{face, a, b}`; an unknown set is `UnknownSet`; a set that is not a boundary face or left-side set is `IneligibleSet` |
| `Classification` | one patch `topo_face_<serial>` per `TopoFace` carrying boundary faces, ascending serial; sets are ignored |

Boundary faces no patch claims go to `default_patch` (default `defaultFaces`, type `patch`; loss
`foam.default_patch`, count). `NonConformal` faces go to a plain `patch` called `nonConformal`
(loss `foam.nonconformal_as_patch`), `PartitionCut` faces to `partitionCut`
(`foam.partition_cut_as_patch`). There is no AMI, cyclic or NCC output.

Decks (`FoamOptions::decks`, the `.smsh` `DeckRecord`s): a patch from the `face_set` of a *mirror*
deck, whose type is still the default `patch`, becomes `symmetryPlane` when all its faces are
coplanar (`max |n.(p - p0)| <= 1e-9 * bbox diagonal`, `n`/`p0` from its first non-degenerate
face); otherwise it stays `patch` with loss `foam.symmetry_not_planar`. An explicit type always
wins. Translation and rotation decks write plain patches (loss `foam.deck_cyclic_not_emitted`).

`boundary` entry:

```
    name
    {
        type            patch;                       // patch | wall | symmetryPlane | symmetry | empty
        inGroups        List<word> 1(wall);          // wall only
        nFaces          4;
        startFace       12;
    }
```

## 5. Zones

* `cellZones`: each non-empty `Cells` set (all, or the named ones per `ZoneSelect`) in set id
  order; labels are cell indices, ascending.
* `faceZones`: opt-in by name (`FoamOptions::face_zones`), never automatic. `Faces` and `Sides`
  sets; labels are foam face indices, ascending; `flipMap[i]` is true iff the entry is a `Right`
  side. A face listed on both sides keeps the `Left` entry (loss `foam.zone_both_sides`).
* Label lists of at most 10 entries are written inline, `cellLabels      List<label> 4(0 1 2 3);`;
  longer ones in OpenFOAM's block layout:

```
    cellLabels      List<label> 
64
(
0
1
...
)
;
```

* `flipMap` is `List<bool>` written with `1` / `0`. The parser also reads `true false on off yes no`.

## 6. Names

Characters outside `[A-Za-z0-9_.+-]` become `_` (loss `foam.name_sanitized`, info). A name that
is empty afterwards is `BadName`; two patches (or two zones of one kind) with the same name
afterwards are `DuplicateName`. A name that starts with a digit, `+`, `-` or `.` gets a leading `_`
(`1e5` -> `_1e5`; OpenFOAM's tokenizer reads the bare spelling as a number; same loss item).
Nothing else is renamed.

## 7. Spellings to confirm in the conformance run (OpenFOAM v2506)

| item | what we write | risk |
|---|---|---|
| `arch` | `"LSB;label=32;scalar=64"` | none expected; readers use it for binary only |
| `note` on faces / owner / neighbour | `nPoints:.. nCells:.. nFaces:.. nInternalFaces:..` (two spaces) | informational; checkMesh does not read it |
| `inGroups` | only for `wall` | `symmetryPlane` patches written without it |
| zone label lists | `List<label> N(...)` inline, block layout when longer | both are OpenFOAM list spellings |
| `flipMap` | `List<bool>` with `1`/`0` | v2506 may prefer `true`/`false` in its own output; both are readable |
| uniform lists `N{v}` | never written; the parser reads them for scalar-like lists, bounded by `N * bytes of one element <= input length`; for `faces` it answers `Unsupported` | OpenFOAM writes them for all-equal lists, not for faces or points |
| `faceCompactList` | not written; parser answers `Unsupported` | newer OpenFOAM writes it for binary `faces` only |
| strictly increasing neighbours | not guaranteed for two cells sharing two faces | fixtures never hit it |
| `symmetryPlane` | exact planarity checked at 1e-9 relative | checkMesh may use a tighter tolerance |

## 8. Parser

`parse_polymesh` / `read_polymesh_dir` read what the writer writes and the usual OpenFOAM
spellings: `//` and `/* */` comments, the `FoamFile` header, `N(...)`, `N\n(\n...\n)`,
`List<T> N(...)`, `N{v}`, nested `k(i ...)` faces, quoted patch names, the `boundary` dictionary,
and both zone files. `format binary` and `faceCompactList` are `Unsupported`; patch types other
than the six above (cyclic, processor, ...) are `Unsupported`. Syntax errors are
`Parse{file, line, detail}`, consistency errors (counts, index ranges, negative labels,
non-finite coordinates, a patch past the faces) are `Malformed`. The parser is iterative and
declared counts never size an allocation: lists grow as tokens are read and the count is compared
at the closing bracket. Panic-free on arbitrary bytes (property-tested, and the huge-count cases are re-run under an
address-space cap in `tests/foam_parse.rs`).

## 9. Checker (`check_polymesh(mesh, tol)`)

| code | meaning |
|---|---|
| `foam.index_range` | array sizes or indices inconsistent; geometry checks are skipped |
| `foam.owner_not_below_neighbour` | an internal face with `owner >= neighbour` |
| `foam.not_upper_triangular` | internal faces not ordered by owner, then neighbour |
| `foam.upper_triangular_tie` | two internal faces with the same `(owner, neighbour)` (warning) |
| `foam.patch_range` | patches do not tile `n_internal..n_faces` |
| `foam.point_unused` | a point no face uses |
| `foam.cell_too_few_faces` | a cell with fewer than 4 faces |
| `foam.cell_unclosed` | `\|sum Sf\| > tol * sum \|Sf\|` |
| `foam.cell_nonpositive_volume` | pyramid-decomposition volume `<= 0` |
| `foam.face_orientation` | `Sf . (fc - cc) <= 0` for the owner |

It mirrors checkMesh's topology part so exporter bugs are found without OpenFOAM. It does not
decide mesh validity; that stays `sm_mesh_api::validate`, which the exporter never calls.

## 10. Loss codes

`foam.default_patch` (info), `foam.name_sanitized` (info), `foam.unused_points_dropped` (info),
`foam.set_skipped`, `foam.patch_overlap`, `foam.nonconformal_as_patch`,
`foam.partition_cut_as_patch`, `foam.symmetry_not_planar`, `foam.deck_cyclic_not_emitted`,
`foam.zone_both_sides`, `foam.high_order_dropped`, `foam.small_det_split`,
`foam.small_det_remaining`, `foam.small_det_unsplit` (all degraded). One item per
`(code, detail)`; the detail names the set or patch.

## 11. Small-determinant split (ruling INT-8)

The internal mesh stays exact; the exporter maps to what `checkMesh` accepts. A cell whose
checkMesh cell determinant (`foam::cell_determinants`, OpenFOAM `primitiveMeshTools::
cellDeterminant`: `|det(sum (Sf/avg)(Sf/avg)^T)| / 8` over the cell's internal faces, `Sf` the face
area vector, `avg` their mean magnitude; a cell with no internal face is 0) is
below `foam::SMALL_DETERMINANT` (0.001) is replaced by one sub-cell per face: a face pyramid
around the cell's vertex centroid (for a tet, the 4-tet centroid split). A tet with three
boundary faces has one internal face and determinant 0, so it is always split. A lone cell (no
internal face) has determinant 0 and is split too. A cube corner cell with 3 internal faces is 1/8,
an interior cube cell 1.

* Cells keep source order; parent `p` becomes consecutive cells. Sub-cell `i` is the pyramid over
  the cell's `i`-th face (ascending foam face order). The centroid is a new last point; the
  triangles (centroid, edge) between sub-cells are new internal faces; internal faces are
  re-sorted upper triangular; patches keep their faces and shift `startFace`.
* cellZones list every sub-cell of a member; patches keep their faces (so sub-cells keep the
  parent's boundary); faceZone labels are renumbered.
* `FoamBuild::face_of_foam` / `point_of_foam` hold `u32::MAX` for added faces and points.
* File `constant/polyMesh/smeshCellMap` (only when a cell was split): a `labelList`, line `i` =
  internal (source mesh) cell of exported cell `i`. `parse_polymesh` / `read_polymesh_dir` restore
  it into `PolyMesh::cell_map`; OpenFOAM ignores the unknown file.
* Report: `foam.small_det_split` (count = split parents; detail gives the sub-cell total),
  `foam.small_det_remaining` (exported cells still below the threshold: slivers), and
  `foam.small_det_unsplit` (flagged cells whose faces are not a closed manifold, or whose centroid face pyramids
  are not all positive in volume; they are written unsplit). Shape counts
  follow the exported cells.
* `FoamOptions::split_small_determinant(false)` disables it. Meshes with no flagged cell (for
  example hex or wedge meshes with three or more internal faces per cell) export byte for byte
  as without the pass.
* 2D cases. OpenFOAM's `checkMesh` evaluates the determinant in the resolved directions only:
  `polyMesh::checkCellDeterminant` (`polyMeshCheck.C`, called from `checkGeometry.C` with
  `geometricD()`) passes `meshD` to `primitiveMeshTools::cellDeterminant`
  (`primitiveMeshTools.C`). With `nDims` resolved directions: 3 (or 0) is the 3D rule above;
  2 sets the knocked-out diagonal entry of the normalised area tensor to 1 before the
  determinant (`areaTensor.zz() = 1` for a z-empty slab), so a cell with two orthogonal internal
  faces is `1/8`; 1 gives every cell 1. `meshD` is `polyMesh::calcDirections`: every component
  of the normalised sum of `cmptMag(Sf)` over `empty` patch faces above 1e-6 is knocked out
  (`solutionD`); `geometricD` additionally knocks out the components of each `wedge` patch's
  `centreNormal` (from the average unit face normal `n`, `sign(n_i) (max(|n_i|, 0.5) - 0.5)`,
  normalised; `wedgePolyPatch::calcGeometry`). Patches without faces are ignored. We follow this
  rule (`foam::geometric_directions`, `foam::cell_determinants`, which read the patch types of
  the polyMesh), so a one-layer hex slab with `empty` front and back, or an axisymmetric wedge
  mesh with `wedge` patches (`FoamPatchType::Empty` / `Wedge`, chosen with
  `PatchSource::Explicit`), is not split and gets no `smeshCellMap`. Without such patches a
  one-layer hex or wedge mesh (for example `hex_block([n, n, 1])`) has rank-2 internal normals,
  checkMesh flags every cell, and it is split as above. Cells with a single internal face
  (a 1-cell-wide slab, two cells in a row) stay flagged in 2D too, as in checkMesh.
  `checkMesh` (opencfd/openfoam-default:2506) on a 4x4x1 empty slab: `Mesh OK`, determinant
  minimum 0.125 average 0.28125; on a 4-cell axisymmetric wedge mesh (2.5 degree half angle,
  prism row on the axis plus a hex row): `Mesh OK`, minimum 0.0511, 2 geometric directions.
* The split can be switched off: `FoamOptions::split_small_determinant(false)`, or
  `sm_fem::ExportRequest::split_small_determinant = false` for `export_files` (default true).
  The `session.export` wire request and the CLI do not carry the switch yet, nor a way to name
  `empty` / `wedge` patches.
