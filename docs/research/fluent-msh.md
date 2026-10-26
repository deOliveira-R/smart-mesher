# ANSYS Fluent mesh file format (`.msh`, legacy text/binary): implementation spec

Status: desk research, written 2026-10-25 for the smart-mesh Fluent exporter (and a later importer).
Not yet checked against files written by Fluent itself (no Fluent licence available).
It has been checked against OpenFOAM v2506's readers and writer (see "Empirical check" below).
Everything marked **[unverified]** is collected again in §14. The example files and the checker
script are in `docs/research/fluent-msh/`.

## Empirical check (OpenFOAM v2506 in Docker, 2026-10-25)

| Test | Result |
|---|---|
| §13.1 `two_hex.msh` → `fluent3DMeshToFoam` → `checkMesh -allTopology -allGeometry` | Converts; 2 hexes; no orientation or volume errors |
| §13.2 `prism_poly.msh`, same pipeline | Converts; 1 prism + 1 polyhedron; no orientation or volume errors |
| Both files with every face's node order reversed (normal → c1) | `checkMesh`: "12 faces are incorrectly oriented", negative volumes −1 / −1.5. The test detects a wrong orientation. |
| `foamMeshToFluent` on a 236-prism Gmsh mesh | Output follows this spec: hex header fields, zone 10 written `a` in section 13 and `10` in section 39, 3-field `(39 (id type name)())`, right-hand normal → c0 on every face |
| `foamMeshToFluent` on a polyDualMesh of the elbow tutorial (1060 polyhedra) | Cell type 7, polygonal faces with a count prefix; every cell closes (max residual 4e-14), all volumes positive |

So item 1 of §14 is confirmed as far as OpenFOAM goes: OpenFOAM's reader and writer both use
"right-hand normal → c0", and a reversed file is caught. It is still unconfirmed against Fluent itself.

What the correct files still show in `checkMesh`, and why it is not a file problem:
- "Cells with small determinant": the determinant check uses only internal faces, so every cell of a
  2-cell mesh scores 0. It is a quality heuristic, not validity.
- "Concave cells (using face planes)" on the §13.2 polyhedron: the cell is a convex pentagonal prism,
  but its two coplanar side quads (a 180° T vertex) trip the face-plane test.

Checker caveats (`check.py`): the "normal points toward c0" test uses the vertex-mean cell centre,
which can fall on the wrong side of a face in concave cells. It flagged 2 faces in the elbow dual
whose orientation is in fact consistent (closure and volumes prove it). Its 1e-12 absolute closure
tolerance is also too tight for large coordinates. Our exporter tests should use signed-volume and
closure checks, not centroid side tests.

This is the Fluent/TGrid "legacy case/mesh" format: Scheme-like parenthesized sections. It is
not Gmsh `.msh`, and not the HDF5 "Common Fluids Format" (`.msh.h5` / `.cas.h5`).

---

## 0. Sources and short names

| Tag | Source |
|---|---|
| **M24** | Ansys Fluent 2024 R2 User's Guide, Meshing Mode, Appendix B "Mesh File Format". Pages: [B.1 Guidelines](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_guidelines.html), [B.2 Binary](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_convbinary.html), [B.3 Grid Sections](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_gridsect.html), [B.4 Non-Grid Sections](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_nongridsect.html), [B.5 Examples](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_examples.html) |
| **L24** | Ansys Fluent 2024 R2 User's Guide, Solution Mode, [B.2 "Legacy Case and Data File Formats"](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/flu_ug_app_form_case_data.html) |
| **FN24** | Ansys Fluent 2024 R2 UG [6.1.2 "Face-Node Connectivity in Ansys Fluent"](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/flu_ug_GridTypes.html#flu_ug_sec_face_node_connect) |
| **F12** | ANSYS FLUENT 12.0 User's Guide (2009), Appendix B, ENEA mirror: [B.2](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1463.htm), [B.3.4 Nodes](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1468.htm), [B.3.5 Periodic](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1469.htm), [B.3.6 Cells](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1470.htm), [B.3.7 Faces](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1471.htm), [B.3.8 Face tree](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1472.htm), [B.3.9 Cell tree](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1473.htm), [B.3.10 Face parents](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1474.htm), [B.4.1 Zone](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1476.htm), [B.4.2 Partitions](https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node1477.htm) |
| **UDF12** | ANSYS FLUENT 12.0 UDF Manual, [3.2.4 Face macros (F_AREA)](https://wwwafs.portici.enea.it/project/neptunius/docs/fluent/html/udf/node98.htm) |
| **F3D** | OpenFOAM Foundation `fluent3DMeshToFoam.L` @ [961b538](https://github.com/OpenFOAM/OpenFOAM-dev/blob/961b538704/applications/utilities/mesh/conversion/fluent3DMeshToFoam/fluent3DMeshToFoam.L) (the OpenCFD/ESI copy, mirrored at [olesenm/openfoam@97c8e62](https://github.com/olesenm/openfoam/blob/97c8e62fe4/applications/utilities/mesh/conversion/fluent3DMeshToFoam/fluent3DMeshToFoam.L), has the same c0/c1 handling at L496–498 and L885–892) |
| **F2D** | OpenFOAM Foundation `fluentMeshToFoam.L` @ [961b538](https://github.com/OpenFOAM/OpenFOAM-dev/blob/961b538704/applications/utilities/mesh/conversion/fluentMeshToFoam/fluentMeshToFoam.L) |
| **W-OF** | OpenFOAM Foundation `foamMeshToFluent/fluentFvMesh.C` @ [961b538](https://github.com/OpenFOAM/OpenFOAM-dev/blob/961b538704/applications/utilities/mesh/conversion/foamMeshToFluent/fluentFvMesh.C) |
| **W-ESI** | OpenCFD `fluentFvMesh.C` ([mirror @97c8e62](https://github.com/olesenm/openfoam/blob/97c8e62fe4/applications/utilities/mesh/conversion/foamMeshToFluent/fluentFvMesh.C)); fix commits `bed1305d96` (stray `()` after cell types, OpenFOAM #1872) and `ad2389a37d` (node zone id, [issue #2820](https://gitlab.com/openfoam/core/openfoam/-/issues/2820)) |
| **VTK** | Kitware `vtkFLUENTReader.cxx` @ [d2249a1](https://github.com/Kitware/VTK/blob/d2249a1d5b/IO/Geometry/vtkFLUENTReader.cxx) |
| **MIO** | meshio `src/meshio/ansys/_ansys.py` @ [b2ee998](https://github.com/nschloe/meshio/blob/b2ee99842e/src/meshio/ansys/_ansys.py) |
| **PX** | polyxios `codecs/_fluent.py` @ [616c17f](https://github.com/fury-gl/polyxios/blob/616c17fd8f/polyxios/codecs/_fluent.py) (recent third-party codec, weak source) |

M24 and L24 are the primary sources. F12 is older and differs on several points (§12). The code
sources show what files that tools actually accept look like.

---

## 1. Overall structure (ASCII)

- The file is a sequence of top-level sections. Each one is `(` + decimal section index + body + `)`.
  "Each section is enclosed in parentheses and begins with a decimal integer indicating its type"
  ([M24 B.1](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_guidelines.html)).
- "Header information for lists of items is enclosed in separate sets of parentheses, preceding the
  items, which are in their own parentheses" (M24 B.1). So a typical grid section is:

  ```
  (INDEX (h1 h2 h3 ...)(
  body ...
  ))
  ```
- **Number bases.** The section index is decimal. In the grid sections (10, 12, 13, 18, 58, 59, 61)
  every header field and every integer in the body is **hexadecimal**, with no `0x` prefix. That
  includes `zone-id` and `type`: "The zone-id and type are also in hexadecimal for consistency"
  (M24 B.3.4). Coordinates are ordinary decimal floats. Zone sections 39/45 write `zone-id` in
  **decimal**: "The zone-id is in decimal format. This is in contrast to the use of hexadecimal in the
  grid sections" (M24 B.4.1). The same zone therefore appears as `a` in `(13 (a ...` and as `10` in
  `(39 (10 ...`. W-OF does exactly this: it writes zone `patchi+10` while the stream is in hex mode
  (L107/L171–173) and later in decimal mode (L272/L281–282). The manual's own examples write
  bc-types in hex: `a` = 10 velocity-inlet, `c` = 12 periodic, `1f` = 31 parent, `24` = 36 outflow
  (M24 B.5 Examples 1–3).
- Use lowercase hex. The manual uses it (`2d5`, `3e3`). OpenFOAM uses `strtol(...,16)` and
  `[[:xdigit:]]`, so it accepts both cases (F3D L140, L304).
- Whitespace separates tokens. Line breaks matter for one reader (§11.3, F3D needs one face per line).
- Indices (node, face, cell) start at **1**. "All cells, faces, and nodes have positive indices"
  (M24 B.3.7). A cell index of 0 in a face record means "no cell".

### Section index summary

| Index | Name (xfile.h macro) | Notes / source |
|---|---|---|
| 0 | comment `XF_COMMENT` | `(0 "text")`, allowed anywhere between sections (M24 B.3.1) |
| 1 | header `XF_HEADER` | `(1 "program version")` (M24 B.3.2) |
| 2 | dimensions `XF_DIMENSION` | `(2 ND)`, ND = 2 or 3 (M24 B.3.3) |
| 4 | machine config | byte-order flag for binary (VTK L2888–2902); contents otherwise undocumented **[unverified]** |
| 10 | nodes `XF_NODE` | required |
| 11 | edges `XF_EDGE` | meshing mode only; "In case files written for Fluent, the Edges section will be omitted" (M24 B.3.8) |
| 12 | cells `XF_CELL` | required |
| 13 | faces `XF_FACE` | required |
| 18 | periodic shadow faces `XF_PERIODIC_FACE` | periodic grids only |
| 39 | zone `XF_RP_TV` | zone type and name, plus BC settings |
| 40 | partitions `XF_PARTITION` | F12 B.4.2 |
| 45 | zone (header-only form) | same header as 39, rest ignored (M24 B.4.1) |
| 58 | cell tree `XF_CELL_TREE` | hanging-node adaption only |
| 59 | face tree `XF_FACE_TREE` | hanging-node adaption only |
| 61 | interface face parents `XF_FACE_PARENTS` | non-conformal interfaces only |
| 37, 38, 41, 60, 62, 63, 64 | other case/settings sections | skipped by F3D (L211: `4 37 38 40 41 60 64`) and by VTK (L2678–2700); contents not documented **[unverified]** |
| 20xx / 30xx | binary versions of the above | §2 |

---

## 2. Binary variants

- "For binary files, the header indices ... are preceded by 20 for single-precision binary data, or by
  30 for double-precision binary data (that is, 2010 or 3010 instead of 10). The end of the binary data
  is indicated by End of Binary Section 2010 or End of Binary Section 3010 before the closing parameters
  of the section." ([M24 B.2](https://ansyshelp.ansys.com/public/Views/Secured/corp/v242/en/flu_ug/tgd_user_format_convbinary.html))
- Manual example:

  ```
  (3010 (2 1 2aad 2 3)(
  ...binary...
  )
  End of Binary Section 3010)
  ```
- **The header stays ASCII hex.** Only the body inside the second `(` ... `)` is raw binary.
- **Where the payload starts. The sources conflict.** VTK starts reading at the byte right after
  the body's `(` (`ptr = dstart + 1`, VTK L2975, L3022, L3119, L3249), i.e. no newline. PX
  (L492–520, `_from_some_start`) says the opposite: "Fluent ends the line after the `(` that opens
  a binary block, so the payload begins on the next; a writer that does not begins it at once". PX
  tries both offsets. MIO's writer puts `(\n` before the payload (MIO L413–416). VTK is the more
  established reader, but PX's claim is specific. **This is [unverified]. Settle it with a real
  Fluent binary file before shipping binary.** Until then, emit ASCII. If binary must be written now,
  use no newline (`(` followed directly by the payload), which VTK requires and PX accepts.
- **Where the payload ends.** The payload is followed by `)`, a newline, the literal
  `End of Binary Section NNNN`, and `)` (the manual example; MIO L417–418 writes `\n)` +
  `End of Binary Section 3010)\n`). The payload can contain `)` bytes, so a reader must compute the
  payload length from the header and must not scan for brackets. VTK scans for the
  `End of Binary Section ` text (VTK L911–931, L766–790). That works because the text is unlikely to
  appear inside the payload.
- **Element widths:**

  | Section | 20xx (single) | 30xx (double) | Source |
  |---|---|---|---|
  | 10 nodes | `f32` × ND per node | `f64` × ND per node | VTK L2964–3053, MIO L81–90 |
  | 12 cells (mixed body) | `i32` per cell | `i32` per cell | VTK L3105–3128; comment at L2712: "Should be the same as single precision.. only grabbing ints." |
  | 13 faces | `i32` | `i32` | VTK L3236–3264 (reads 4 bytes for both 2013 and 3013) |
  | 18 / 58 / 59 / 61 | `i32` | `i32` | VTK L3458+, L3571–3596 |

  The 30xx prefix applies to reals only. Fluent's own 3013 connectivity is 32-bit (VTK; PX
  module docstring: "a double-precision section with 32-bit integers, Fluent's own, or the 64-bit
  ones another writer spells"). MIO writes 64-bit integers under 3012 (MIO L435–439), which is
  meshio's own convention. **Whether Fluent accepts int64 connectivity in any binary section is
  [unverified].**
- **Binary face body layout.** One record per face, back to back, with no separators
  (VTK L3249–3270):
  - face-type 0 or 5: `i32 n`, `i32 node[n]`, `i32 c0`, `i32 c1`
  - face-type 2/3/4: `i32 node[k]`, `i32 c0`, `i32 c1` with k = face-type
- **Binary cell body.** Only mixed zones (element-type 0) have one: `i32` element type per cell
  (VTK L3113–3127).
- **Byte order.** The default is little-endian (VTK L120). If a `(4 (...))` section is present, a first
  value of `60` means little-endian and any other value means big-endian (VTK L2888–2902). A writer
  can write little-endian and omit section 4. **Whether Fluent requires section 4 in binary mesh
  files is [unverified].**
- **Exporter recommendation.** Write ASCII first, because F3D cannot read binary at all: its lexer
  only knows `(10`, `(12`, `(13`, ... (F3D L197–211). Binary is an optimization. If it is written,
  use `3010` for nodes and `2012`/`2013` (or `3012`/`3013`, both with int32) for cells and faces.
  Which combination Fluent itself writes is **[unverified]**.

---

## 3. Comment, header, dimension

```
(0 "free text")
(1 "smart-mesh 0.1")
(2 3)
```

- M24 calls 0, 1 and 2 optional. **F3D requires `(2 3)`**: it aborts with "Mesh is not 3D" unless
  dimension == 3 (F3D L844–849). Always write it.
- Comment and header text: F3D lexes quoted text with a narrow character class (F3D L161–188, L281,
  L700). Keep header/comment strings to `[A-Za-z0-9 _.:-]` to be safe. A character the lexer does
  not expect in an unexpected place is fatal: "Do not understand characters" (F3D L748–754).

---

## 4. Nodes (10)

```
(10 (zone-id first-index last-index type ND)(
x1 y1 z1
x2 y2 z2
...
))
```

- **Declaration** (zone-id = 0): it gives the total node count, has no body, and also omits the body
  parentheses. M24 B.3.4: "first-index will be one, last-index will be the total number of nodes in
  hexadecimal, type is zero, ND is omitted". Example `(10 (0 1 2d5 0))`. The manual's own Example 1
  writes `(10 (0 1 8 0 2))`, with ND present. **Write `(10 (0 1 N 0 3))`.** F3D needs at least one
  token after `last-index` (F3D L303) and needs the literal `0 1` prefix (F3D L299).
- **Regular** (zone-id > 0): `first-index` and `last-index` are hex. "The values of last-index in
  each zone must be less than or equal to the value in the declaration section" (M24 B.3.4).
- **type** (M24 B.3.4): 0 = "virtual" nodes (ignored), 1 = no (any) type, 2 = boundary nodes.
  "Nodes of type zero are ignored but types one and two are read and written". F12 says "Type is
  always equal to 1". **Write 1.** The M24 binary example uses type 2.
- **ND** is optional (2 or 3). In a 2D grid each line has only x and y.
- Coordinates: one node per line. Any C float syntax works (`%.17g` or `%.16e`; W-OF uses
  `scientific` with precision 10, W-OF L112–113). F3D accepts integers and floats with optional
  exponent (F3D L166–173, L357–385).
- Several node zones are allowed. Each covers a contiguous range of ids. F3D reads all of them into
  one array indexed by global id (F3D L315–345). **One node zone covering 1..N is the simplest choice.**

---

## 5. Cells (12)

```
(12 (zone-id first-index last-index type element-type))          ; uniform zone: no body
(12 (zone-id first-index last-index type 0)( t1 t2 t3 ... ))     ; mixed zone: body = one type per cell
```

- **Declaration**: `(12 (0 1 N 0))`. "In a declaration section, the type has a value of zero, while
  the element-type is not present" (M24 B.3.6). "This declaration section is required and must precede
  the regular cell sections" (M24 B.3.6). `last-index` = 0 means a surface-only file.
- **element-type** (M24 B.3.6 table):

  | code | shape | nodes | faces |
  |---|---|---|---|
  | 0 | mixed | – | – |
  | 1 | triangle (2D) | 3 | 3 |
  | 2 | tetrahedron | 4 | 4 |
  | 3 | quadrilateral (2D) | 4 | 4 |
  | 4 | hexahedron | 8 | 6 |
  | 5 | pyramid | 5 | 5 |
  | 6 | wedge (prism) | 6 | 5 |
  | 7 | polyhedron | NN | NF |

- **type** (zone type, hex). The sources disagree:
  - M24 B.3.6: "type indicates whether the cell zone is fluid (type = 1) or solid (type = 17).
    A type of zero indicates a dead zone".
  - L24 / F12: "Active zones are represented with type=1, while inactive zones are represented with
    type=32" (inactive = hanging-node parent cells). "A type of zero indicates a dead zone and will be
    skipped".
  - The manual's Example 3 writes the parent zone as `(12 (1 7 7 20 3))`, i.e. hex `20` = 32.
    So these values are hex-encoded, and "17" would be written as `11`. **[unverified for 17]**
  - F3D reads the field but does not use it (F3D L585–588). MIO treats 0 as dead (MIO L114–116).
  - **Write `1`.** Mark fluid or solid through the 39/45 zone type (§8). That was the old meaning, and
    "In the earlier versions ... a distinction was made between solid and fluid zones. This is now
    determined by properties" (L24).
- **Mixed body**: "If a zone is of mixed type (element-type=0), it will have a body that lists the
  element-type of each cell" (M24 B.3.6). Example:

  ```
  (12 (9 1 3d 0 0)(
  1 1 1 3 3 1 1 3 1
  ...
  ))
  ```
  The body is whitespace-separated hex codes, in cell-id order from `first-index` to `last-index`.
  W-ESI writes 25 per line. Line layout is free: F3D skips the body up to the first `)` (L605–610).
- **Uniform (non-mixed) cell zones have no body.** "Regular cell sections have no body" (M24 B.3.6).
  W-OF had a bug that wrote an extra `()` after the mixed body (`)())`, W-OF L269). ESI fixed it in
  `bed1305d96` ("extraneous brackets after Fluent cell types", #1872). Do not copy that.
- **No cell→node or cell→face lists exist.** Cells are defined only by the faces that name them as c0
  or c1 (§10).
- F3D also accepts a 4-field cell header, which it calls "Tgrid syntax problem", without the
  element-type (F3D L544–572).

---

## 6. Faces (13)

```
(13 (zone-id first-index last-index bc-type face-type)(
...one face per line...
))
```

- **Declaration**: `(13 (0 1 N 0))`. "A zone-id of zero indicates a declaration section, which
  provides a count of the total number of faces in the mesh. Such a section omits the bc-type and is
  not followed by a body" (M24 B.3.7). Its 4th field is in practice 0 (M24 Example 1
  `(13 (0 1 a 0))`; W-OF writes `(13 (0 1 N 0 0))`, L93–98). Write `(13 (0 1 N 0))`.
- **face-type** (M24 B.3.7):

  | code | meaning | nodes/face | per-line layout |
  |---|---|---|---|
  | 0 | mixed | varies | `x n0 n1 ... n(x-1) c0 c1` |
  | 2 | linear (2D edge) | 2 | `n0 n1 c0 c1` |
  | 3 | triangular | 3 | `n0 n1 n2 c0 c1` |
  | 4 | quadrilateral | 4 | `n0 n1 n2 n3 c0 c1` |
  | 5 | polygonal | varies | `x n0 n1 ... n(x-1) c0 c1` |

  "If the face zone is of mixed type (face-type = 0) or of polygonal type (face-type = 5), each line of
  the section body will begin with a reference to the number of nodes that make up that particular
  face" (M24 B.3.7, L24). F12 mentions the prefix only for type 0. Code agrees with M24: VTK
  `faceType == 0 || faceType == 5` (L3175, L3252) and F3D `faceGroupElementType == 0 || > 4`
  (L459). Note that MIO's reader treats the leading number of a mixed line as a *type code* (MIO
  L229–241). That is wrong, so do not use MIO as a reference for mixed faces. The count `x` is hex
  like every other integer.
- `c0`, `c1` are hex cell ids. "If a face has a cell only on one side, then either c0 or c1 is zero.
  For files containing only a boundary mesh, both these values are zero" (M24 B.3.7). The orientation
  meaning of c0 and c1 is in §7.
- **bc-type** (M24 B.3.7 table; L24 adds "mass-flow-outlet" under 20). The decimal values below are
  written in **hex** in the header (see the examples in §1):

  | decimal | hex written | meaning (M24/L24) | 39/45 zone-type word(s) |
  |---|---|---|---|
  | 2 | `2` | interior | `interior` |
  | 3 | `3` | wall | `wall` |
  | 4 | `4` | pressure-inlet, inlet-vent, intake-fan | `pressure-inlet` / `inlet-vent` / `intake-fan` |
  | 5 | `5` | pressure-outlet, exhaust-fan, outlet-vent | `pressure-outlet` / `exhaust-fan` / `outlet-vent` |
  | 7 | `7` | symmetry | `symmetry` |
  | 8 | `8` | periodic-shadow | `shadow` (F12 list) |
  | 9 | `9` | pressure-far-field | `pressure-far-field` |
  | 10 | `a` | velocity-inlet | `velocity-inlet` |
  | 12 | `c` | periodic | `periodic` (F12 list) |
  | 14 | `e` | fan, porous-jump, radiator | `fan` / `porous-jump` / `radiator` |
  | 20 | `14` | mass-flow-inlet (L24: also mass-flow-outlet) | `mass-flow-inlet` |
  | 24 | `18` | interface | `interface` |
  | 31 | `1f` | parent (hanging node) | `parent-face` (M24 list) **[unverified mapping]** |
  | 36 | `24` | outflow | `outflow` |
  | 37 | `25` | axis | `axis` (F12 list) |

  The task's list (2, 3, 4, 5, 7, 8, 10, 12, 14, 20, 24, 31, 36, 37) matches, and **9
  pressure-far-field** has to be added. No other codes are documented. Non-conformal intersection
  faces: "A factor of 1000 is added to the bc-type of these sections; for example, 1003 is a wall
  zone" (M24 B.3.7). Whether 1003 is written as hex `3eb` is **[unverified]**.
- The F3D reader **ignores bc-type** (L439–440). It uses the 39/45 zone-type word (§8). A writer must
  still write a correct bc-type for Fluent.
- **Interior faces in an `interior` zone (bc 2) have c0 ≠ 0 and c1 ≠ 0. Boundary faces have exactly
  one of them 0.** An interior face zone may also have type wall/fan/radiator/porous-jump: "Fluent
  allows the wall type to be assigned to face zones both on the inside and on the boundaries"
  (M24 B.4.1). F3D turns two-sided wall zones into baffles (L1349–1361).

---

## 7. Orientation (most important)

### 7.1 What the manuals say (verbatim)

**3D, M24 B.3.7 and L24 B.2.3.7 (2024 R2):**
> "The order of the cell indices is important and is determined by the right-hand rule: if you curl the
> fingers of your right hand in the order of the nodes, your thumb will point toward c0."

**3D, F12 B.3.7 (2009):**
> "The order of the cell indices is important, and is determined by the right-hand rule: if you curl the
> fingers of your right hand in the order of the nodes, your thumb will point toward c1."

**2D, F12 and L24 (identical; the symbols are images in the HTML, called a, b here):**
> "For 2D grids, n2 is omitted. c1 is determined by the cross product of two vectors, a and b. The a
> vector extends from n0 to n1, whereas the b vector has its origin at n0 and points out of the grid
> plane toward the viewer. If you extend your right hand along a and curl your fingers in the direction
> of the angle between a and b, your thumb will point along a × b toward c1."

**Solver-internal convention, UDF12 (`F_AREA`), a different thing:**
> "By convention in ANSYS FLUENT, boundary face area normals always point out of the domain. ... the
> cell out of which a face area normal is pointing is designated as cell C0, while the cell in to
> which a face area normal is pointing is cell c1 ... face area normals always point from cell c0 to
> cell c1."

### 7.2 Resolution: 3D file rule = right-hand normal points **toward c0**

The 2009 and 2024 manuals contradict each other for 3D. The code sources all agree with 2024 (c0):

1. **F3D (reader).** F3D L482–484 (mixed) and L501–503 (uniform):
   `// read neighbour and owner. Neighbour comes first` → `neighbour = c0 - 1; owner = c1 - 1`.
   The face node list is kept as is. In OpenFOAM the right-hand normal of a face points from owner to
   neighbour, so F3D reads the file normal as pointing **c1 → c0, i.e. toward c0**.
   The pre-filter at F3D L865–884 then makes the faces OpenFOAM-legal:
   ```
   if (owner == -1 || (neighbour != -1 && owner > neighbour)) {
       faces[facei].flip();            // skipped with -cubit
       Swap(owner[facei], neighbour[facei]);
   }
   ```
   A boundary face written as `... cell 0` has c1 = 0, so owner = -1. It gets flipped and swapped,
   which gives owner = cell with an outward normal. That is only correct if the file normal pointed
   **into** the cell (toward c0). The `owner > neighbour` branch is only renumbering and does not
   change the geometry.
2. **W-OF / W-ESI (writer).** Interior faces are written as OpenFOAM node order (normal
   owner→neighbour) followed by `nei+1 own+1`, i.e. c0 = neighbour (W-OF L139–154). Boundary faces
   are written with the node order **reversed**, then `cell 0` (W-OF L198–215), with the comment
   *"Note: In Fluent, all boundary faces point inwards, which is opposite from the OpenFOAM
   convention. Turn them around on printout"* (L206–208). Issue #2820 reports that these files "load
   correctly in Fluent and ICEM". That is empirical evidence that Fluent accepts normal-toward-c0
   (assuming Fluent's mesh check flags left-handed faces **[unverified]**).
3. **F2D (reader, old converter).** "boundary faces are oriented such that the owner is zero and the
   face area vector points into the domain. Turn them round" (F2D L1127–1144). F2D also reads
   neighbour = c0, owner = c1 (F2D L489–491, L510–512).
4. **VTK (reader).** `PopulateHexahedronCell` uses a face's nodes in file order as the VTK hex bottom
   face (0,1,2,3) when `face.c0 == cell` and reversed otherwise (VTK L3993–4006). VTK's hex
   convention is that the right-hand normal of (0,1,2,3) points toward the opposite face (4..7), i.e.
   into the cell. Tet (L3884–3895) and pyramid (L4176–4190) follow the same logic. (The wedge routine,
   L4239–4251, uses the same `c0` test, but VTK's wedge base points *away* from the top. That is
   probably a VTK inconsistency and says nothing about Fluent.)
5. **Cubit.** OpenFOAM's `-cubit` option exists for "(incorrect) Cubit files ... (incorrect face
   orientation; hexadecimal zoneIDs)" (F3D L91–92, L814–822). Under `-cubit` the boundary-face flip
   is skipped (L878–881), so Cubit writes boundary faces pointing *outward*, which OpenFOAM considers
   wrong. **Do not use Cubit `.msh` files as validation references.**

Conclusion: the F12 "toward c1" sentence is an error that was corrected in later manuals.
The solver's internal area vector (UDF12, pointing c0 → c1) is the **opposite** of the file's
right-hand normal. Fluent evidently computes it with the opposite winding. Do not mix the two up
when validating.

### 7.3 The rule to implement (3D)

For a face with nodes `n0 n1 ... n(k-1)` and Newell/right-hand normal
`N = Σ (p_i × p_{i+1}) / 2`:

- **N points into c0 and out of c1.**
- Boundary face: write `... cell 0`, so the cell is c0 and c1 = 0. N then points **into the domain**
  (inward). Every manual example uses c1 = 0 for boundary faces. M24 allows c0 = 0 with c1 = cell (N
  pointing outward), and F3D handles it correctly, but **whether Fluent accepts c0 = 0 is
  [unverified]**, so always put the cell in c0.
- Interior face: either assignment works as long as it matches the node order. Any rule is fine, for
  example c0 = the higher-numbered cell. F3D swaps when c1 > c0, but that does not matter.

**Mapping from smart-mesh (face-first, left/right):** let `n_sm` be the right-hand normal of the face's
stored node loop, and suppose smart-mesh's convention is that `n_sm` points from `left` to `right`
(the OpenFOAM owner→neighbour convention; check this against the actual sm contract):
- interior: write nodes in stored order, then `c0 = right`, `c1 = left`.
- boundary, with `n_sm` pointing outward from its single cell `left`: write nodes **reversed**, then
  `c0 = left`, `c1 = 0`.
- If smart-mesh's normal points right → left instead, swap the two rules. Exporter tests must check
  this geometrically: N · (centroid(c0) − face centroid) > 0 for every face (see the check script in
  §13.3).

OpenFOAM mapping summary (the "do they flip?" question):
- **Reader F3D:** owner = c1, neighbour = c0, nodes unchanged. Then every face with c1 = 0 (or
  c1 > c0) is flipped and swapped (L865–884).
- **Writer W-OF:** interior: nodes as stored, `c0 = neighbour`, `c1 = owner` (no flip). Boundary:
  nodes reversed, `c0 = owner`, `c1 = 0` (flip).

### 7.4 2D is the other way round

In 2D the manual's normal is `a × ẑ` (a = n1 − n0, ẑ toward the viewer), and it points toward
**c1**. Put differently, **c0 is on the left of n0→n1** when viewed from +z. Boundary edges then run
counter-clockwise around the domain, and the 2D "normal" `a × ẑ` points **outward** on boundaries,
the opposite of 3D. Checked against M24 Example 1 (nodes: 1(1,0), 2(1,1), 3(2,0), 4(2,1), 5(0,0),
6(3,0), 7(3,1), 8(0,1); cells 1 = [0,1]², 2 = [1,2]×[0,1], 3 = [2,3]×[0,1]):

| face line | a = n1−n0 | a × ẑ | c1 is | check |
|---|---|---|---|---|
| `1 2 1 2` | (0,1) | (+1,0) → x>1 | cell 2 | ✓ |
| `3 4 2 3` | (0,1) | (+1,0) → x>2 | cell 3 | ✓ |
| `5 1 1 0` | (1,0) | (0,−1) → outside | 0 | ✓ |
| `7 4 3 0` | (−1,0) | (0,+1) → outside (y>1) | 0 | ✓ |
| `8 5 1 0` | (0,−1) | (−1,0) → outside (x<0) | 0 | ✓ |
| `6 7 3 0` | (0,1) | (+1,0) → outside (x>3) | 0 | ✓ |

(`a × ẑ` for a = (ax, ay, 0) is (ay, −ax, 0).) F2D relies on this: it extrudes the 2D mesh to −z,
"In order for the owner-neighbour rules to be right, the points given by Fluent need to represent the
FRONT plane of the geometry. Therefore, the extrusion will be in -z direction" (F2D L988–991). The
extruded quad `(n0, n1, n1', n0')` with primes at −z has normal `a × (−ẑ)`, which points toward c0
and so matches the 3D rule.

### 7.5 Consistency hint from Fluent's cell-local face tables (not proof)

FN24 lists each standard cell's faces as, e.g., hex: 1-2-3-4, 1-5-6-2, 2-6-7-3, 4-3-7-8, 1-4-8-5,
5-8-7-6; tet: 4-3-2, 3-4-1, 2-1-4, 3-1-2. Take a positively oriented local numbering (base 1-2-3-4
counter-clockwise seen from the top/apex). Then every listed face has an **inward** right-hand normal.
That is what "normal toward c0" gives if each table is written from the point of view of the cell as
c0. The figures are images that were not inspected, so the local handedness is assumed. Treat this as
weak supporting evidence only.

---

## 8. Zone sections (39 and 45)

```
(39 (zone-id zone-type zone-name domain-id)(
(condition1 . value1)
...
))
(39 (zone-id zone-type zone-name domain-id)())     ; what mesh generators write
(45 (zone-id zone-type zone-name domain-id)())     ; same, but anything after the header is ignored
```

- "Grid generators and preprocessors need only provide the section header and leave the list of
  conditions empty ... The empty parentheses at the end are required" (M24 B.4.1).
- "When only zone-id, zone-type, zone-name, and domain-id are specified, the index 45 may be used ...
  the index 39 must be used if boundary conditions are present" (M24 B.4.1). F12 says 45 "is
  preferred". "In meshing mode, the zone name and type can be extracted from the boundary condition
  section 39 ... or 45, but only section 39 can be written" (M24 B.4.1).
- **zone-id is decimal** here (M24 B.4.1). F3D: `// cell zone-ID not in hexadecimal!!! Inconsistency`
  (L633–642). F2D says the same (L700, L725). Cubit writes it in hex (F3D L636–639). That is one of the
  "Cubit is wrong" cases.
- **domain-id**: present in every manual example (`(39 (1 fluid fuel 1)())`,
  `(39 (8 pressure-inlet pressure-inlet-8 2)())`, `(39 (2 wall wing-skin 3)())`). Its meaning is not
  documented. W-OF omits it (`(39 (1 fluid fluid-1)())`, W-OF L275–303) and those files load in Fluent
  (#2820). F2D has a comment "Fluent manual inconsistency, version 6.1.22" about the 4-field form
  (L722). F3D (`{label}?`, L630) and VTK (L3382–3430) accept 3 or 4 fields. **The semantics of
  domain-id, and whether Fluent wants 1 or omission, are [unverified].** Recommendation: write the
  3-field form `(39 (id type name)())`, which W-OF proves Fluent accepts. Re-test the 4-field form with
  domain-id 1 empirically.
- **One section per zone at most**: "there cannot be more than one zone section for each zone"
  (M24 B.4.1). **Zone sections are not strictly required**: "Although some grid zones may not have
  corresponding sections" (M24 B.4.1). For interop, **declare every zone**:
  - F3D looks up `groupName[cellGroupZoneID]` for every cell zone (L955–961). That is
    `HashTable::operator[]`, which aborts on a missing key (OpenFOAM behaviour **[unverified at
    runtime]**), so cell zones without 39/45 will probably make F3D crash.
  - A face zone without a 39/45 becomes patch `FaceGroup<id>` of type "unknown" if it is a boundary,
    or "Unknown FaceGroup ... not in a zone" if it is interior (F3D L1010–1027). With a face tree
    present, such zones are taken to be hanging-node parents and dropped (L994–1009).
- **zone-type words.** M24 list: degassing, exhaust-fan, fan, fluid, geometry, inlet-vent,
  intake-fan, interface, interior, internal, mass-flow-inlet, outflow, outlet-vent, parent-face,
  porous-jump, pressure-far-field, pressure-inlet, pressure-outlet, radiator, solid, symmetry,
  velocity-inlet, wall, wrapper. The F12 list adds axis, periodic, shadow (written there as
  "exhaust fan", "inlet vent", "intake fan", "outlet vent" with spaces, almost certainly a typesetting
  slip). "cell zones can be assigned only either fluid or solid type"; interior/fan/porous-jump/
  radiator are only for faces inside the domain (M24 B.4.1).
  F3D's patch/faceZone split (L977–993): a zone becomes a **patch** if its first face has c1 = 0 *or*
  its type is wall/fan/radiator/porous-jump. Otherwise a two-sided zone, including the `interior`
  zone, becomes an OpenFOAM **faceZone** with the zone's name. So `int_fluid` in §13 will show up as a
  faceZone, which is expected.
  F3D maps them to OpenFOAM types (L890–919): wall → wall; symmetry/axis → symmetry; periodic/shadow →
  cyclic (paired by shared name prefix, L1034–1101); fan/radiator/porous-jump → mergedCyclic;
  everything else → patch.
- **zone-name** is "a valid Scheme symbol and is written without quotes. ... The first character must
  be a lowercase letter or a special-initial. Each subsequent character must be a lowercase letter, a
  special-initial, a digit, or a special-subsequent. special-initial: `! $ % & * / : < = > ? ~ _ ^`;
  special-subsequent: `. + -`" (M24 B.4.1). **The F3D lexer is narrower**: zone names must match
  `({alpha}|{digit}|[.:-])*` with alpha = `[_[:alpha:]]` (F3D L136, L161, L630). So `/ ! $ % & * < = >
  ? ~ ^ +` break OpenFOAM import.
  **Exporter rule: sanitize names to `[a-z][a-z0-9_.-]*`**, lowercase, unique across all zones (F3D
  makes duplicates unique with `_1`, L95–115). Uppercase is not allowed by the Scheme rule. Whether
  Fluent rejects or lowercases uppercase names is **[unverified]** (W-OF writes OpenFOAM patch names
  unchanged).

---

## 9. Other grid sections

### 9.1 Periodic shadow faces (18)

```
(18 (first-index last-index periodic-zone shadow-zone)(
f00 f01
f10 f11
...
))
```
"These are in hexadecimal format. The indices in the section body (f*) refer to the faces on each of
the periodic boundaries (in hexadecimal), the indices being offsets into the list of faces for the
grid. Note: The first-index and last-index do not refer to face indices; they refer to indices in the
list of periodic pairs" (M24 B.3.5). The periodic zone has bc 12 (`c`), the shadow zone bc 8. M24
Example 2: `(13 (5 9 9 c 2)...)`, `(13 (1 a a 8 2)...)`, `(18 (1 1 5 1)(9 a))`. F3D ignores 18 and
pairs cyclics by name and type instead (L207–213, L1034–1101). Not needed for a first exporter.

### 9.2 Cell tree (58) and face tree (59): hanging nodes

```
(58 (cell-id0 cell-id1 parent-zone-id child-zone-id)(
number-of-kids kid-id-0 kid-id-1 ... kid-id-n
...
))
(59 (face-id0 face-id1 parent-zone-id child-zone-id)(
number-of-kids kid-id-0 kid-id-1 ... kid-id-n
...
))
```
All values are hex. There is one body line per parent, from id0 to id1 (M24 B.3.9/B.3.10). Parent
cells go in an inactive cell zone (type `20` = 32). Parent faces go in zones with bc-type `1f` (31,
"parent"), and they still carry c0/c1 (M24 Example 3: `(13 (8 16 16 1f 2)(9 8 2 7))`,
`(59 (16 16 8 2)(2 7 6))`, `(58 (7 7 1 7)(4 6 5 4 3))`). "You cannot read files that contain [58] in
the meshing mode" (M24 B.3.10). F3D: a 59 section switches on `hangingNodes`, drops face zones that
are not declared, and runs `polyMeshZipUpCells` (L612–616, L994–1009, L1423–1427).
**Exporter recommendation: never write 58/59.** smart-mesh polyhedra can include the hanging node in
the polygon node list of the coarse face, so the face shows up as one polygon (face-type 0/5) and the
coarse cell as type 7. That is the "conformal polyhedral" form Fluent reads without adaption trees.

### 9.3 Interface face parents (61)

```
(61 (face-id0 face-id1)(
parent-id-0 parent-id-1
...
))
```
Hex. Each child (non-conformal intersection) face lists its two parent faces (M24 B.3.11). Not needed.

### 9.4 Edges (11), meshing mode only

`(11 (zone-id first-index last-index type element-type)( v0 v1 ... ))`. type 5 = boundary edge,
6 = interior edge, element-type is ignored, and the section is omitted in files written for the solver
(M24 B.3.8). Do not write it.

### 9.5 Partitions (40)

`(40 (zone-id first-index last-index partition-count)( p1 p2 ... ))`, one partition id per cell
(F12 B.4.2). Do not write it.

---

## 10. Polyhedral cells

- A polyhedral cell is element-type **7**, either as a uniform zone `(12 (z f l 1 7))` or as `7`
  entries in a mixed zone's body.
- **No explicit cell→face or cell→node list exists anywhere in the format.** "Regular cell sections
  have no body" (except the type list of mixed zones) (M24 B.3.6). "For polyhedral cells, there is no
  explicit face and node numbering as with the other cell types" (FN24 6.1.2.7). A cell is the set of
  faces whose c0 or c1 equals its id. All readers rebuild cells that way (F3D L1249–1409 via
  `polyTopoChange`; VTK collects `faceIndices` per cell in L3211–3229 and builds VTK_POLYHEDRON
  through `PopulatePolyhedronCell`, L3774).
- Faces of polyhedral cells are usually polygons, written in face zones of type 0 (mixed) or 5
  (polygonal) with the count prefix.
- Even the standard shapes (2, 4, 5, 6) are stored only as faces. The element-type is a label that
  Fluent presumably checks against the face count. **Exporter rule: write 2/4/5/6 only if the cell has
  exactly the canonical face set (e.g. hex = 6 quads, wedge = 2 tris + 3 quads, pyramid = 1 quad + 4
  tris, tet = 4 tris). Otherwise, including any hex with a split face, write 7.** Whether Fluent
  rejects a type-4 label on a cell with 7 faces is **[unverified]**, but it is reasonable to assume it
  does.
- W-OF writes 7 for every non-standard OpenFOAM cell in a mixed zone (W-OF L252–266). Note that W-OF
  writes all faces as type-0 mixed zones (L136–137, L171–196), even for pure-hex meshes.

---

## 11. Constraints, pitfalls, and exporter rules

### 11.1 Numbering and ranges

- Ids start at 1, separately for nodes, cells, and faces.
- Each regular 10/12/13 section covers one **contiguous** id range `[first-index, last-index]`, so
  **every zone must have contiguous ids**. The exporter has to renumber cells by cell zone and faces
  by face zone.
- The ranges of all regular sections should exactly tile `1..N` of the declaration. F3D sizes arrays
  from the declarations and indexes by global id (L304–306, L415–421). Gaps would leave
  uninitialized faces. Whether Fluent allows gaps is **[unverified]**. Do not leave any.
- `last-index` ≤ declared total (M24 B.3.4).

### 11.2 Ordering

- The declaration sections (zone 0) of 10/12/13 must come before the regular sections ("must precede
  the regular cell sections", M24 B.3.6). Write all three declarations first.
- Otherwise the order of sections is free. The manual examples put the node data **last**, after the
  faces (M24 B.5). The usual order is: header, dimension, declarations, nodes, cells, faces, zones.
- **Interior faces first, then boundary zones.** Both the manual examples and W-OF do this. That
  Fluent *requires* it is **[unverified]**, but it costs nothing.
- Keep each face zone homogeneous: all faces interior (both cells set), or all boundary (c1 = 0). F3D
  classifies a whole zone by its **first** face (`neighbour[start] == -1`, L982).

### 11.3 Text layout needed by OpenFOAM (F3D)

- **One face per line.** F3D reads a whole face record as one token in which "space" is
  `[ \t\f]*`, without newline (L132–134, L176–180, L471–505). A face record split over two lines is
  misread.
- Header tokens: `(13 (` + hex tokens + `)`. The body `(` may follow on the same line or the next one.
- Points: any whitespace layout works, one point per line is simplest.

### 11.4 Zone ids

- **Make zone ids unique across node, cell, and face zones.** W-OF wrote node zone 1 and cell zone 1.
  Fluent loaded the file, but Ansys `tmerge` corrupted the node count. The fix was to move the node
  zone to id 3 ("Ansys uses id 3 by default"; issue #2820, ESI commit `ad2389a37d`). The manual's own
  Example 2 reuses id 1 for both the node zone and a face zone, so Fluent itself seems tolerant, but
  unique ids are the safe choice.
- Cell zone ids and face zone ids share **one** id space. A 39/45 entry is keyed by id alone, and F3D stores cell and face zone types in the same `groupType`/`groupName` maps keyed by id (F3D L72–73, L645–646). Cell zone 2 and face zone 2 would therefore share one 39 entry. Node zone ids should also be distinct (#2820).
- Ids < 10 look the same in hex and decimal. Ids ≥ 10 must be written in hex in 10/12/13 and in decimal
  in 39/45 (§1).
- The maximum zone id is **[unverified]**.

### 11.5 Polygons and polyhedra

- Maximum nodes per polygon face, and maximum faces per polyhedron: **not documented anywhere I
  found [unverified]**. Readers place no limit (F3D: `setSize(count)`; VTK: `resize`).
- Faces must be simple, cyclically ordered polygons. Degenerate faces (repeated node, zero area) are
  not covered by the docs. Do not write them.
- Fluent's mesh check reports left-handed faces and negative volumes (general Fluent knowledge, not
  sourced here **[unverified]**). §13.3 gives an equivalent check to run before writing.

### 11.6 Dimension rules

- 3D: faces have ≥ 3 nodes; cells of type 2/4/5/6/7.
- 2D: faces are edges (face-type 2), cells of type 1/3 (and probably 7 for polygons **[unverified]**),
  points have 2 coordinates (`ND = 2`), and the orientation rule is §7.4. smart-mesh should export 3D
  only.

### 11.7 Hanging nodes

Fluent supports them through adaption trees (58/59 plus parent zones). smart-mesh should write
conformal polygons instead (§9.2).

---

## 12. Version differences

| Topic | Fluent 12.0 (2009, F12) | Fluent 2024 R2 (M24/L24) |
|---|---|---|
| 3D orientation sentence | thumb → **c1** | thumb → **c0** (both appendices) |
| Node declaration | `type` = 1, ND present: `(10 (0 1 2d5 1 2))` (its own examples use `0`) | `type` = 0, ND omitted: `(10 (0 1 2d5 0))` (M24); L24 still has the old text |
| Node `type` | "always 1" | 0 virtual / 1 any / 2 boundary |
| Cell zone `type` | 1 active, 32 inactive, 0 dead | M24: 1 fluid, 17 solid, 0 dead; L24 keeps active/inactive |
| Face-type 5 count prefix | not mentioned | explicit |
| bc 20 | mass-flow-inlet | + mass-flow-outlet (L24) |
| Edge section 11 | absent | meshing mode only |
| 39/45 zone types | includes axis, periodic, shadow | adds degassing, geometry, internal, parent-face, wrapper; drops axis/periodic/shadow from the list |
| 45 vs 39 | "45 is preferred" for header-only | "45 may be used"; meshing mode "only section 39 can be written" |

- **Fluent Meshing `.msh`** (from TGrid). It may contain edges (11), surface-only meshes
  (`(12 (0 1 0 0))`, c0 = c1 = 0), zone types geometry/wrapper/parent-face, and type-2 boundary nodes.
  Sections 2040/2041/3040/3041 and 2063/3063 (and 41/63/64) show up in real files and are skipped by
  VTK (L2726–2731, L2755–2757) and F3D (41/64, L211). **What they contain is not documented
  [unverified].** I found no public description of "(41 ...)" or "2063". A writer does not need them.
- **HDF5 CFF.** Newer Fluent versions write `.msh.h5`/`.cas.h5` by default and call this text/binary
  format "legacy" (the L24 page title). The release where the default changed, and whether the legacy
  `.msh` will stay readable indefinitely, are **[unverified]**. It is still documented and readable
  in 2024 R2.

---

## 13. Worked examples

Geometry for both: the box [0,2]×[0,1]×[0,1] with 12 nodes (ids in hex):

| id | xyz | id | xyz | id | xyz |
|---|---|---|---|---|---|
| 1 | (0,0,0) | 5 | (1,1,0) | 9 | (2,0,1) |
| 2 | (1,0,0) | 6 | (2,1,0) | a | (0,1,1) |
| 3 | (2,0,0) | 7 | (0,0,1) | b | (1,1,1) |
| 4 | (0,1,0) | 8 | (1,0,1) | c | (2,1,1) |

Zone ids: node zone 1, cell zone 2, interior zone 3, wall zone 4 (`walls-left`), wall zone **10**
(`walls-right`, written `a` in the 13 header and `10` in the 39 section, to show the hex/decimal
split). All ids are unique.

### 13.1 Two hexes sharing one face

Cell 1 = [0,1]×[0,1]×[0,1] (centroid (0.5,0.5,0.5)), cell 2 = [1,2]×[0,1]×[0,1] (centroid
(1.5,0.5,0.5)).

```
(0 "smart-mesh example: two hexes sharing one face")
(1 "smart-mesh 0.1")
(0 "Dimension:")
(2 3)
(0 "Declarations: 12 nodes 2 cells 11 faces")
(10 (0 1 c 0 3))
(12 (0 1 2 0))
(13 (0 1 b 0))
(0 "Nodes: zone 1")
(10 (1 1 c 1 3)(
0.0 0.0 0.0
1.0 0.0 0.0
2.0 0.0 0.0
0.0 1.0 0.0
1.0 1.0 0.0
2.0 1.0 0.0
0.0 0.0 1.0
1.0 0.0 1.0
2.0 0.0 1.0
0.0 1.0 1.0
1.0 1.0 1.0
2.0 1.0 1.0
))
(0 "Cells: zone 2 fluid hex")
(12 (2 1 2 1 4))
(0 "Interior faces: zone 3")
(13 (3 1 1 2 4)(
2 5 b 8 2 1
))
(0 "Wall faces of cell 1: zone 4")
(13 (4 2 6 3 4)(
1 4 a 7 1 0
1 7 8 2 1 0
4 5 b a 1 0
1 2 5 4 1 0
7 a b 8 1 0
))
(0 "Wall faces of cell 2: zone 10 written as hex a")
(13 (a 7 b 3 4)(
3 9 c 6 2 0
2 8 9 3 2 0
5 6 c b 2 0
2 3 6 5 2 0
8 b c 9 2 0
))
(0 "Zones")
(39 (2 fluid fluid)())
(39 (3 interior int_fluid)())
(39 (4 wall walls-left)())
(39 (10 wall walls-right)())
```

Line by line: `(12 (2 1 2 1 4))` is zone 2, cells 1..2, type 1 (active/fluid), element-type 4 (hex),
with no body. `(13 (3 1 1 2 4)` is zone 3, face 1..1, bc 2 (interior), face-type 4 (quad).
`(13 (4 2 6 3 4)` is zone 4, faces 2..6, bc 3 (wall), quads. `(13 (a 7 b 3 4)` is zone 10 (hex a),
faces 7..11 (hex b), wall, quads.

**Orientation check by hand.** Rule: N = (p[n1] − p[n0]) × (p[n2] − p[n0]) must point toward c0. That
is exact for these planar convex quads. In each row, e1 = p[n1]−p[n0] and e2 = p[n2]−p[n0].

| # | nodes | c0 c1 | e1 | e2 | N = e1×e2 | face plane | c0 centroid side | ok |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 5 b 8 | 2 1 | (0,1,0) | (0,1,1) | (1,0,0) | x=1 | cell 2 at x=1.5 → +x | ✓ |
| 2 | 1 4 a 7 | 1 0 | (0,1,0) | (0,1,1) | (1,0,0) | x=0 | cell 1 at x=0.5 → +x (inward) | ✓ |
| 3 | 1 7 8 2 | 1 0 | (0,0,1) | (1,0,1) | (0,1,0) | y=0 | y=0.5 → +y | ✓ |
| 4 | 4 5 b a | 1 0 | (1,0,0) | (1,0,1) | (0,−1,0) | y=1 | y=0.5 → −y | ✓ |
| 5 | 1 2 5 4 | 1 0 | (1,0,0) | (1,1,0) | (0,0,1) | z=0 | z=0.5 → +z | ✓ |
| 6 | 7 a b 8 | 1 0 | (0,1,0) | (1,1,0) | (0,0,−1) | z=1 | z=0.5 → −z | ✓ |
| 7 | 3 9 c 6 | 2 0 | (0,0,1) | (0,1,1) | (−1,0,0) | x=2 | cell 2 at x=1.5 → −x | ✓ |
| 8 | 2 8 9 3 | 2 0 | (0,0,1) | (1,0,1) | (0,1,0) | y=0 | +y | ✓ |
| 9 | 5 6 c b | 2 0 | (1,0,0) | (1,0,1) | (0,−1,0) | y=1 | −y | ✓ |
| a | 2 3 6 5 | 2 0 | (1,0,0) | (1,1,0) | (0,0,1) | z=0 | +z | ✓ |
| b | 8 b c 9 | 2 0 | (0,1,0) | (1,1,0) | (0,0,−1) | z=1 | −z | ✓ |

Cross products used: (0,1,0)×(0,1,1) = (1·1−0·1, 0·0−0·1, 0·1−1·0) = (1,0,0);
(0,0,1)×(1,0,1) = (0·1−1·0, 1·1−0·1, 0·0−0·1) = (0,1,0); (1,0,0)×(1,0,1) = (0,−1,0);
(1,0,0)×(1,1,0) = (0,0,1); (0,1,0)×(1,1,0) = (0,0,−1); (0,0,1)×(0,1,1) = (−1,0,0).
All boundary normals point inward, and each cell's faces close (Σ outward area = 0, volume = 1;
script output in §13.3).

How OpenFOAM F3D reads face 1: neighbour = c0−1 = 1, owner = c1−1 = 0, nodes kept. The OpenFOAM normal
(+x) runs from owner (cell 0, left) to neighbour (cell 1, right) ✓. owner < neighbour, so no swap.
Boundary face 2: owner = c1−1 = −1, so it is flipped to `7 a 4 1` and swapped (owner = 0). The normal
is now −x, outward ✓.

### 13.2 Same box as one prism (wedge) plus one polyhedron

Split the box with the vertical plane x + y = 1. Cell 1 is the wedge under the triangle
(0,0),(1,0),(0,1) (nodes 1 2 4 / 7 8 a; volume 0.5). Cell 2 is the pentagonal prism over
(1,0),(2,0),(2,1),(1,1),(0,1) (nodes 2 3 4 5 6 / 8 9 a b c; volume 1.5). Its top and bottom are
pentagons, and its y=1 side is two coplanar quads (node 5/b is a 180° "T" vertex). That makes it a
7-face polyhedron with no standard type, so it is type 7. The counts are the same as in 13.1: 1
interior face and 10 boundary faces (4 + 6).

```
(0 "smart-mesh example: one prism plus one polyhedron")
(1 "smart-mesh 0.1")
(0 "Dimension:")
(2 3)
(10 (0 1 c 0 3))
(12 (0 1 2 0))
(13 (0 1 b 0))
(10 (1 1 c 1 3)(
0.0 0.0 0.0
1.0 0.0 0.0
2.0 0.0 0.0
0.0 1.0 0.0
1.0 1.0 0.0
2.0 1.0 0.0
0.0 0.0 1.0
1.0 0.0 1.0
2.0 0.0 1.0
0.0 1.0 1.0
1.0 1.0 1.0
2.0 1.0 1.0
))
(0 "Cells: zone 2 mixed: cell 1 wedge 6 and cell 2 polyhedron 7")
(12 (2 1 2 1 0)(
6 7
))
(13 (3 1 1 2 4)(
2 4 a 8 2 1
))
(0 "Mixed face zone: count prefix on every line")
(13 (4 2 5 3 0)(
3 1 2 4 1 0
3 7 a 8 1 0
4 1 4 a 7 1 0
4 1 7 8 2 1 0
))
(13 (a 6 b 3 0)(
5 2 3 6 5 4 2 0
5 8 a b c 9 2 0
4 2 8 9 3 2 0
4 3 9 c 6 2 0
4 5 6 c b 2 0
4 4 5 b a 2 0
))
(39 (2 fluid fluid)())
(39 (3 interior int_fluid)())
(39 (4 wall walls-left)())
(39 (10 wall walls-right)())
```

Notes:
- The cell zone is mixed (element-type 0) and its body `6 7` lists the types of cells 1 and 2.
  Splitting it into two cell zones (one `(12 (2 1 1 1 6))` and one `(12 (5 2 2 1 7))`) would also be
  valid, but each extra zone needs its own 39 entry.
- The boundary zones mix triangles and quads (zone 4) or pentagons and quads (zone 10), so they use
  face-type 0. Each line starts with its node count. Face-type 5 (polygonal) would also work with the
  same line layout.
- Writing `6` for cell 1 is safe because its face set (2 tris + 3 quads, counting the shared interior
  quad) is exactly a wedge.

**Orientation check by hand** (c0 centroids: cell 1 ≈ (0.33,0.33,0.5); cell 2 node-average
(1.2,0.6,0.5)):

| # | nodes | c0 c1 | e1 | e2 | N = e1×e2 | face | c0 side | ok |
|---|---|---|---|---|---|---|---|---|
| 1 | 2 4 a 8 | 2 1 | (−1,1,0) | (−1,1,1) | (1,1,0) | x+y=1 | cell 2: x+y=1.8 > 1 → +(1,1) | ✓ |
| 2 | 1 2 4 | 1 0 | (1,0,0) | (0,1,0) | (0,0,1) | z=0 | +z | ✓ |
| 3 | 7 a 8 | 1 0 | (0,1,0) | (1,0,0) | (0,0,−1) | z=1 | −z | ✓ |
| 4 | 1 4 a 7 | 1 0 | (0,1,0) | (0,1,1) | (1,0,0) | x=0 | +x | ✓ |
| 5 | 1 7 8 2 | 1 0 | (0,0,1) | (1,0,1) | (0,1,0) | y=0 | +y | ✓ |
| 6 | 2 3 6 5 4 | 2 0 | (1,0,0) | (1,1,0) | (0,0,1) | z=0 | +z (Newell (0,0,1.5)) | ✓ |
| 7 | 8 a b c 9 | 2 0 | (−1,1,0) | (0,1,0) | (0,0,−1) | z=1 | −z (Newell (0,0,−1.5)) | ✓ |
| 8 | 2 8 9 3 | 2 0 | (0,0,1) | (1,0,1) | (0,1,0) | y=0 | +y | ✓ |
| 9 | 3 9 c 6 | 2 0 | (0,0,1) | (0,1,1) | (−1,0,0) | x=2 | −x | ✓ |
| a | 5 6 c b | 2 0 | (1,0,0) | (1,0,1) | (0,−1,0) | y=1 | −y | ✓ |
| b | 4 5 b a | 2 0 | (1,0,0) | (1,0,1) | (0,−1,0) | y=1 | −y | ✓ |

Row 1: (−1,1,0)×(−1,1,1) = (1·1−0·1, 0·(−1)−(−1)·1, (−1)·1−1·(−1)) = (1,1,0).
Row 7: (−1,1,0)×(0,1,0) = (1·0−0·1, 0·0−(−1)·0, (−1)·1−1·0) = (0,0,−1). For the non-quad polygons the
3-point cross product only gives the direction. The script's Newell normal confirms rows 6–7.

### 13.3 Machine check (appendix)

Both files are in `/private/tmp/claude-501/-Users-rodrigo-git-smart-mesh/c40a3162-2e83-405b-a924-4e8e94047b55/scratchpad/fluent_work/`
(`two_hex.msh`, `prism_poly.msh`, `check.py`). `check.py` parses 10/12/13, computes Newell normals,
and checks N·(centroid(c0) − face centroid) > 0. For each cell it accumulates the outward area
(−N for c0, +N for c1) and the volume Σ(face centroid · outward N)/3. Output:

```
two_hex.msh:    all 11 faces points_toward_c0=True; cell 1 closure=0 volume=1.0; cell 2 closure=0 volume=1.0; ALL OK
prism_poly.msh: all 11 faces points_toward_c0=True; cell 1 closure=0 volume=0.5; cell 2 closure=0 volume=1.5; ALL OK
```
The smart-mesh exporter's tests should run the same three checks on every exported file: normal
toward c0, closure, and positive volume. Neither file has been run through Fluent or
`fluent3DMeshToFoam` yet. That is the second pass.

---

## 14. Open questions / unverified

Orientation
1. 3D rule "right-hand normal → c0". It is backed by the M24/L24 text plus F3D, F2D, W-OF, VTK, and
   reports that W-OF output loads in Fluent. It contradicts the F12 text ("→ c1"). Not tested against
   a file written by Fluent or Fluent Meshing. **Verify: run `checkMesh` after `fluent3DMeshToFoam`
   on §13 files and on a real Fluent-written `.msh`, and read §13 files into Fluent if available.**
2. Whether Fluent accepts boundary faces written with c0 = 0, c1 = cell (outward normal). M24 says
   "either c0 or c1 is zero". Every example uses c1 = 0.
3. Whether the solver's F_AREA (c0→c1) really is the reverse of the file right-hand normal, or the
   UDF manual's "right hand rule ... in order of increasing node number" means something else.
4. The FN24 local-face tables give inward normals only if the (image-only) local numbering is
   positively oriented. Not inspected.

Binary
5. Integer width in 3012/3013/3018/3058/3059/3061. VTK and PX say int32 in Fluent's own files. MIO
   writes int64. Whether Fluent accepts int64 is unknown, as is how Fluent stores > 2³¹ entities in
   legacy binary.
6. Whether Fluent puts a newline between the body `(` and the payload. VTK assumes not. PX says
   Fluent does. Whether Fluent's own reader tolerates both is also unknown.
7. Contents of section `(4 ...)` beyond "first value 60 = little-endian", and whether Fluent requires
   it in binary mesh files.
8. Which binary prefix Fluent itself uses for 12/13 (2012/2013 or 3012/3013) in double-precision
   output.

Header fields and codes
9. Cell zone `type` 17 (solid): decimal 17 written as hex `11`, or literally `17`? Is `1` with the
   zone type set in 39 enough? (W-OF writes `1` and the zone type `fluid`.)
10. Encoding of non-conformal bc-types (1000 + code), e.g. `3eb` for 1003.
11. Mapping of bc 31 "parent" to the 39 word (`parent-face`?). Whether bc 8/12 must use the words
    `shadow`/`periodic`, which M24's list omits.
12. Full list of bc-type codes: only the 16 listed values are documented. Values such as
    degassing/geometry/wrapper have 39 words but no documented numeric bc-type.
13. `domain-id` in 39/45: meaning, whether it is needed, and what value to write (1?).
14. Uppercase and other characters in zone names: the Scheme rule says lowercase. Fluent's behaviour is
    unknown, and OpenFOAM breaks on `/ ! $ % & * < = > ? ~ ^ +`.
15. Maximum zone id.

Structure
16. Whether Fluent requires interior faces before boundary faces, contiguous face ids with no gaps
    across zones, or nodes before faces. The manual examples put nodes last.
17. Whether Fluent requires every zone to have a 39/45 entry. The manual says some may not have one.
    F3D probably aborts on a cell zone without one (not run).
18. Whether Fluent checks element-type labels (2/4/5/6) against the actual face set, and whether `7`
    is accepted inside a mixed zone body (W-OF does it; no Fluent test with polyhedra seen).
19. Maximum nodes per polygonal face and faces per polyhedron: nothing found.
20. Whether node zone ids must differ from cell/face zone ids for Fluent itself, not just tmerge. The
    manual Example 2 reuses id 1.
21. Contents and role of sections 37, 38, 41, 60, 62, 63, 64 and the binary 2040/2041/2063 variants
    seen in Fluent Meshing files.
22. The Fluent version where `.msh.h5` became the default, and the long-term support status of legacy
    `.msh`.
23. Polygonal (type 7) cells and the face-type 5 count prefix in 2D files.
