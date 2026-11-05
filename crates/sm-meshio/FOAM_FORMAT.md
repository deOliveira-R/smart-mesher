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
than the six the writer emits (`patch`, `wall`, `symmetryPlane`, `symmetry`, `empty`, `wedge`;
`FoamPatchType`) (cyclic, processor, ...) are `Unsupported`. Syntax errors are
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
`foam.small_det_remaining`, `foam.small_det_unsplit`, `foam.small_det_2d_unsplit`,
`foam.warped_faces_split` (all degraded). One item per
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
  `checkGeometry.C` calls `mesh.checkCellDeterminant(true, &cells)`; the `polyMesh` override
  (`polyMeshCheck.C`) passes `geometricD()` as `meshD` to `primitiveMeshTools::cellDeterminant`
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
  checkMesh flags every cell, and it is split as above. The split never runs on a 2D case
  (fewer than three resolved directions in `foam::geometric_directions`): cells that still
  fall below the threshold there (a cell with a single internal face, such as a corner triangle
  of an extruded tri-prism slab, or a 1-cell-wide slab) are kept unsplit and reported as
  `foam.small_det_2d_unsplit` (count = flagged cells). Both choices fail one checkMesh check, and the
  exporter keeps the exact unsplit cells. Unsplit, checkMesh 2506 (`-allGeometry -allTopology`)
  on a 2x2 tri-prism slab (empty front/back) reports `***Cells with small determinant (< 0.001)
  found, number of cells: 2` and `Failed 1 mesh checks.`. Split, the centroid split of a 2D cell
  adds edges that are not aligned with or perpendicular to the non-empty directions, which the
  same checkMesh fails (`Number of edges not aligned with or perpendicular to non-empty
  directions`; 12 on a 2x2 tri-prism slab with 2 corner triangles, 8 -> 16 cells).
  `checkMesh` (opencfd/openfoam-default:2506) on a 4x4x1 empty slab: `Mesh OK`, determinant
  minimum 0.125 average 0.28125; on a 4-cell axisymmetric wedge mesh (2.5 degree half angle,
  prism row on the axis plus a hex row): `Mesh OK`, minimum 0.0511415134648249, average
  0.178152764506552, 2 geometric directions (pinned in `tests/foam_2d.rs`).
  Degenerate cases differing from OpenFOAM's `polyMesh::calcDirections`: (a) a zero wedge
  vector sum makes OpenFOAM reset `geometricD` to all 1, whereas we keep the empty knockouts;
  (b) with every direction knocked out OpenFOAM's `cellDeterminant` uses the 3D rule, whereas
  `cell_determinants` does too but the split treats the mesh as 2D and skips it.
* The split can be switched off: `FoamOptions::split_small_determinant(false)`, or
  `sm_fem::ExportRequest::split_small_determinant = false` for `export_files` (default true).
* Open follow-ups. (1) A 2D-aware split of flagged 2D cells (split the 2D footprint and extrude
  the pieces, so every new edge lies in-plane or along the empty direction) would pass both
  checkMesh checks; not scheduled. (2) The wire `session.export` request and the CLI carry
  neither the split switch nor a way to name `empty` / `wedge` patches (proposal P5 in
  `docs/design/w3b/plan.md`, pending), and `sm_fem::export_files` never emits them, so through
  sm-fem only the off switch is reachable; the 2D rule is reachable only via `FoamOptions` with
  `PatchSource::Explicit`.

## 12. Warped-face split (W3b FOAM-WARP, opt-in)

`FoamOptions::split_warped_faces(true)` replaces each polygonal face whose planarity deviation
`foam::face_warpage` (largest vertex distance from the plane through the face centre with the
Newell normal, over `sqrt(|Sf|)`) exceeds `warp_tolerance` (default `1e-9`) by the triangles
(edge, apex), the apex being OpenFOAM's face centre `fc`. Properties, all tested in
`tests/foam_warp.rs`:

* Every exported face is planar to 1e-12 (the pieces are triangles).
* The pieces' area vectors sum to the face's `Sf` and the apex lies on the plane
  `Sf . (x - fc) = 0`, so the OpenFOAM pyramid volume `1/3 sum Sf . (fc - x0)` of both adjacent
  cells is conserved to rounding (tested at 1e-12 relative).
* The internal mesh is untouched: cells, cell order, zones and `smeshCellMap` are as without the
  pass; points keep their indices and the apexes are appended (`point_of_foam` is `u32::MAX`);
  pieces are consecutive, patches keep their faces (boundary faces split too), `faceZone`
  labels list every piece. `face_of_foam` of a piece is the source face.
* Pieces of one internal face share (owner, neighbour): `check_polymesh` reports
  `foam.upper_triangular_tie` (a warning) for them.
* Census: a cell with a split face counts as a polyhedron. Report: `FoamReport::warp`
  (`WarpReport`: faces, pieces, points, cells, concave cells before and after) and the loss item
  `foam.warped_faces_split`. `foam::concave_cell_count` is an in-process approximation of
  checkMesh's `Concave cells (using face planes)` (cosine above 1e-5 against the plane of a face
  through its centre), not a replacement for running checkMesh.
* Planar meshes export byte for byte as without the pass; the default options serialize as
  before (the new keys appear only when set).

INT-9 measurement (`concave_counts_are_recorded_before_and_after`): on warped hex blocks the
split turns 0 concave cells into every cell concave (3x3x3, amplitude 0.03: 0 -> 27; amplitude
0.08: 0 -> 27; 4x4x4, 0.05: 0 -> 64). A triangulated warped face is a shallow cone, so the cell
on its inner side sees the apex as a dimple. The pass is therefore off by default; whether a
plane-based concave check accepts the dimples depends on checkMesh's tolerance and was not run
here (no OpenFOAM on this machine). The polydual (1046) and `w3a_poly` (3966) counts need
sm-poly (L3) and are measured by `tools/conformance/check_openfoam.py`, not from this crate.
