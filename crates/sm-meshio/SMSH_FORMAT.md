# `.smsh` container format, version 1.0

Native mesh format of smart-mesh. One file holds one mesh in the face-first model of
`sm-mesh-api` (docs/design/ir.md sections 2 and 6), with classification, sets, tags, optional
high-order data, hierarchy maps, deck-transformation metadata and provenance.

Design goals: memory-mappable with zero copies, a reader in about 100 lines of any language
(`readers/smsh.py` is one), no dependency on HDF5, and bit-exact round trips.

Status: **v1.0**. Compatibility rule: readers reject a different **major**; a **minor** bump only
adds optional header fields and arrays, so a v1.0 reader reads any v1.x file and ignores what it
does not know. Writers write exactly the arrays below; readers must look arrays up by name.

## 1. File layout

All integers are little-endian. All offsets are in bytes.

```
0        8 bytes   magic  53 4D 53 48 0D 0A 1A 0A      ("SMSH\r\n\x1a\n")
8        u64       header_len
16       header_len bytes   JSON header, UTF-8, followed by 0x20 padding
16+header_len = D   (D is a multiple of 64)
D + a.offset        array a            (every a.offset is a multiple of 64)
```

* The writer pads the JSON header with spaces (0x20) so that `16 + header_len` is a multiple of
  64. The header therefore always ends in `}` followed by zero or more spaces.
* The **data region** starts at `D`. Every array starts at `D + offset` with `offset % 64 == 0`,
  offsets ascend in file order, gaps between arrays are zero bytes, and the file ends with the
  last array (no trailing bytes). An mmap of the file is 64-byte aligned at every array, so a
  little-endian host can use arrays in place.
* Array bytes are the plain little-endian element values in row-major order. No compression.
* The magic follows the PNG convention: the `\r\n` and `\x1a\n` detect text-mode damage.

A reader must check: magic; `16 + header_len <= file size` and `% 64 == 0`; `format == "smsh"`;
`version.major == 1`; for every array `nbytes == prod(shape) * sizeof(dtype)`, `offset % 64 == 0`
and `offset + nbytes <= file size - D`; array names are unique. `crc32` verification is
optional but cheap (see 3).

A reader that decodes the arrays into a mesh must also range-check every index column before
use: `face_left` below `nc`; `face_right` below `nc` or `0xFFFFFFFF`; `cell_parent` likewise;
`face_vertices`, `cell_corners`, `edge_vertices` below `nv`; `cell_faces` entries `w >> 1` below
`nf`; `cell_shape` and `face_kind` inside their code tables (3.1); set words by `members` kind
against `nv` / `nf` / `nc` / `np` (sides: `w >> 1` against `nf`; shell sides: `w >> 1` against
`nc`); `edge_face_edge` below `ne`; `ho_cell` strictly ascending and below `nc`, `fc_face`
strictly ascending and below `nf`, `ho_nodes` / `fc_nodes` below `np`; tag columns of exactly
the entity count of their target; `n_points >= n_vertices`; level maps and extrusions sized and
ranged for the role of the file's mesh (2.5, 2.6). Violations are a malformed file, not
undefined behaviour. `sm-meshio` does this in `SmshFile::decode`.

## 2. Header (JSON)

A single JSON object. Unknown fields are ignored. Writers emit compact JSON with the field order
below, so output is byte-reproducible; readers must not depend on order.

| field | type | meaning |
|---|---|---|
| `format` | string | always `"smsh"` |
| `version` | `{major, minor}` | `{1, 0}` |
| `endian` | string | always `"little"` |
| `producer` | string | writer id, informational (`"sm-meshio 0.0.0"`) |
| `mesh` | object | identity, counts, hashes (below) |
| `arrays` | list | the array table, in file order |
| `sets` | list | named sets |
| `tags` | list | tag columns |
| `geometry_cache` | object or null | rule and hashes of the cached geometry |
| `level_maps` | list | fine-to-coarse maps involving this mesh |
| `extrusions` | list | extrusion maps involving this mesh |
| `decks` | list | deck transformations from the model |
| `provenance` | object or null | `{timeline_hash}` |

Entity ids appear in text form `Kind#serial` (for example `Mesh#1`, `MeshSet#4`) in the header and
as raw `u64` (`kind << 56 | serial`) in arrays. Hashes (64-bit) appear as 16 lowercase hex digits,
because JSON numbers are not safe for 64 bits in every language.

### 2.1 `mesh`

| field | meaning |
|---|---|
| `id` | mesh id |
| `dim` | `3` (volume mesh) or `2` (surface mesh; faces are two-vertex facets) |
| `geometry_rule` | geometry rule id, `smartmesh.fan_vertex_mean.v1` unless stated otherwise |
| `n_vertices` | vertices (`nv`) |
| `n_points` | points in `x,y,z` (`np >= nv`): vertices, then high-order extra nodes |
| `n_faces` | faces (`nf`; dimension 2: facets) |
| `n_cells` | cells (`nc`) |
| `n_edges` | rows of the edge table (`ne`; 0 = no table) |
| `hashes` | optional `{connectivity, topology, geometry}`: the `sm-mesh-api` content hashes |

### 2.2 `arrays` rows

`{name, dtype, shape, offset, nbytes, crc32}` with `dtype` one of `u8`, `u32`, `u64`, `i64`,
`f64`; `shape` `[n]` or `[n, k]`; `crc32` the CRC-32 (IEEE 802.3, the zlib polynomial; Python
`zlib.crc32`) of the array bytes.

### 2.3 `sets` rows

`{id, name, label, members, role, array}`.

| `members` | variant | `role` | `array` (u32) holds |
|---|---|---|---|
| 0 | vertices | `node_set` | vertex indices, ascending |
| 1 | faces | `face_set` | face indices, ascending |
| 2 | cells | `cell_set` | cell indices, ascending |
| 3 | sides | `side_set` | `face << 1 \| side`, ascending (side 0 = the face's left cell, 1 = right) |
| 4 | shell sides | `shell_side_set` | `cell << 1 \| side` of a dimension-2 mesh (0 = top, 1 = bottom) |
| 5 | points | `point_set` | indices into `x,y,z` (vertices and high-order nodes), ascending |

`name` is unique. `label` is the integer id used by block / side-set / node-set numbering
(Exodus, transport decks), unique per `members` kind when present, else `null`. Cell sets are the
regions / blocks / materials; face and side sets are boundary patches; node sets are vertex sets.
Sets are listed ascending by set id.

### 2.4 `tags` rows

`{name, target, array}`: `target` is `vertex`, `edge`, `face` or `cell`; `array` is `i64` or `f64`
with one value per entity of that kind.

### 2.5 `level_maps` rows (mesh hierarchy)

`{fine, coarse, fine_hash, coarse_hash, coarse_of, face_map}`. `fine` / `coarse` are mesh ids and
`fine_hash` / `coarse_hash` the `connectivity_hash` of each at build time (a consumer must refuse
the map if the mesh it loads hashes differently). `coarse_of` names a `u32[n_fine_cells]` array.
`face_map` is null or the name of a `u32[n_fine_faces]` array (`0xFFFFFFFF` = the fine face lies
inside a coarse cell). The file's mesh is the fine or the coarse mesh of every map it lists.

### 2.6 `extrusions` rows (base mesh x layers)

`{base, extruded, base_hash, extruded_hash, n_layers, z, cell_base, cell_layer}`. `z` names an
`f64[n_layers + 1]` array of strictly monotone levels (the sweep is along +z); `cell_base` and
`cell_layer` name `u32[n_extruded_cells]` arrays giving each extruded cell's base cell and layer.
The file's mesh is the base (dimension 2) or the extruded (dimension 3) mesh.

### 2.7 `decks` rows (deck transformations)

Metadata from the model (ir.md section 5.1) that the consumer needs to reflect or wrap on a deck
boundary: `{name, params, matrix, translation, improper, order, face_set, mate_set}`.
`params` is tagged by `kind`:
`{"kind":"mirror","normal":[x,y,z],"offset":d}` (plane `n.x = d`),
`{"kind":"translation","vector":[x,y,z]}`,
`{"kind":"rotation","axis_point":[...],"axis_dir":[...],"turns":[num,den]}`.
`matrix` is the row-major 3x3 linear part and `translation` the translation of the transform
taking side A to side B (`B = matrix * A + translation`); `improper` is true for a mirror;
`order` is the order of the generated cyclic group (`null` = infinite). `face_set` / `mate_set`
name face or side sets of this file holding sides A and B (`mate_set` is null for a self-paired
mirror). The parameters are the source of truth; the matrix is derived.

**Why `params` is its own schema.** The file format does not embed the serde form of
`sm_model_api::Deck`. That shape is a Rust type owned by the model crate (enum tagging, field
names, `Turns` as a struct, `#[non_exhaustive]` variants) and may change with it, while a `.smsh`
file must stay readable from any language for as long as its major version lives. `params` is
therefore specified here (the three variants above, plain numbers and arrays), and the Rust
side converts with `DeckRecord::from_deck`. A new deck variant needs a minor version bump of
this spec (new `kind`); a writer meeting an unknown model variant returns `Unsupported`.

### 2.8 `provenance`

`{timeline_hash}`: the op-timeline node hash (opaque text, normally hex) the mesh was generated
from, so the mesh can be regenerated or forked.

## 3. Arrays

Notation: `nv, np, nf, nc, ne` as in 2.1. "CSR" = an offsets array of `rows + 1` `u64` starting
at 0 and non-decreasing, plus a flat data array whose length equals the last offset.
Vertex, face and cell indices are dense `u32` (identical to the ascending order of the ids).

### 3.1 Always present

| name | dtype | shape | meaning |
|---|---|---|---|
| `x`, `y`, `z` | f64 | `[np]` | coordinates (SoA); points `0..nv` are the vertices |
| `vertex_ids`, `face_ids`, `cell_ids` | u64 | `[nv]`, `[nf]`, `[nc]` | raw entity ids, ascending |
| `face_offsets` | u64 | `[nf+1]` | CSR offsets of the face loops |
| `face_vertices` | u32 | `[...]` | concatenated face loops (vertex indices) |
| `face_left` | u32 | `[nf]` | left cell of every face |
| `face_right` | u32 | `[nf]` | right cell, `0xFFFFFFFF` = none |
| `face_kind` | u8 | `[nf]` | 0 interior, 1 boundary, 2 non-conformal, 3 partition cut |
| `cell_face_offsets` | u64 | `[nc+1]` | CSR offsets of the signed cell faces |
| `cell_faces` | u32 | `[...]` | `face << 1 \| side` per entry |
| `cell_shape` | u8 | `[nc]` | 0 tet, 1 hex, 2 wedge, 3 pyramid, 4 polyhedron, 5 tri, 6 quad, 7 polygon |
| `cell_corner_offsets` | u64 | `[nc+1]` | CSR offsets of the template corners |
| `cell_corners` | u32 | `[...]` | template-ordered corners; zero-length rows for polyhedra |
| `cell_parent` | u32 | `[nc]` | refinement parent, `0xFFFFFFFF` = root |
| `cell_active` | u8 | `[nc]` | 0 for a parent retained inactive after refinement, else 1 |
| `class_entities` | u64 | `[k]` | sorted unique raw ids of the model / partition entities used below |
| `vertex_class`, `face_class`, `cell_class` | u32 | `[nv]`, `[nf]`, `[nc]` | index into `class_entities` |

**Polyhedra.** A cell is a list of signed faces: `cell_faces` entry `w` names face `w >> 1`
and side `w & 1`. **Orientation rule (binding):** the loop of a face defines its normal by the
right-hand rule; the normal points from the left cell into the right cell; a one-cell face has no
right cell and its normal points outward; `left` is the lower cell index. Side 0 (`+`, CGNS NFACE
sign) means the stored loop is outward for this cell, side 1 means inward. Loops start at their
smallest vertex index (dimension 2: facets are directed `[a, b]`, traversed `a -> b` by the left
cell, never rotated). With this rule OpenFOAM `owner` / `neighbour` and CGNS `ParentElements` /
NFACE are renumberings of these arrays with no flips.

**Template cells** (tet, hex, wedge, pyramid; and every cell of a dimension-2 mesh) also list
their corners in the template order of `sm-mesh-api` (`shape.rs`): tet `0,1,2` base
counter-clockwise from +z with `3` above; hex `0..3` bottom counter-clockwise, `4..7` the top
ring above `0..3`; wedge `0..2` bottom triangle, `3..5` above; pyramid `0..3` base, `4` apex;
dimension 2: the vertex cycle in boundary order. Their faces are also in `face_*` (the face
table is complete for every mesh); corners are an index for fast paths.

**Classification.** Every vertex, face and cell lies on exactly one model (`Topo*`) or
decomposition (`Partition*`) entity; `class_entities[vertex_class[v]]` is that entity's raw id
(`kind << 56 | serial`; kinds: 1 vertex, 2 edge, 3 face, 4 volume, 48..51 partition cell / face /
edge / vertex). Boundary patches, materials and deck lowering derive from this table; sets are
annotations that never replace it.

**Parametric classification.** If any entity of kind `P` (`vertex`, `face`, `cell`, `edge`) has
parameters on its model entity, two more arrays exist: `P_param_kind` (u8 `[n]`: 0 none, 1 curve
`t`, 2 surface `(u, v)`, 3 volume `(u, v, w)`) and `P_param` (f64 `[n, 3]`, unused components 0).

### 3.2 Optional

| present when | arrays |
|---|---|
| the mesh has an edge table (`ne > 0`) | `edge_ids` u64 `[ne]`; `edge_vertices` u32 `[ne, 2]`; `edge_class` (+ `edge_param_kind`, `edge_param`) as above; `edge_face_offsets` u64 `[nf+1]`, `edge_face_edge` u32, `edge_face_forward` u8: CSR of the edges around each face loop, `forward` = the loop traverses the edge from its first to its second vertex |
| any cell has a high-order record | `ho_cell` u32 `[m]` (cell indices, ascending); `ho_degree` u8, `ho_basis` u8 (0 Lagrange equispaced, 1 Lagrange GLL, 2 serendipity) `[m]`; `ho_node_offsets` u64 `[m+1]` and `ho_nodes` u32: point indices in `smartmesh.nodes.v1` order (Gmsh order: corners, edge nodes, quad-face nodes, hex centre) |
| any face is curved | `fc_face`, `fc_degree`, `fc_basis`, `fc_node_offsets`, `fc_nodes`, as above: the loop vertices, then one node per loop edge, then (Lagrange quad) the centre |
| the mesh carries a geometry cache | `gc_face_area` f64 `[nf, 3]`, `gc_face_centroid` `[nf, 3]`, `gc_face_apex` `[nf, 3]`, `gc_cell_volume` `[nc]`, `gc_cell_centroid` `[nc, 3]`; the header's `geometry_cache` gives the rule and the hashes it was computed for. Derived data: ignore it if the hashes do not match the mesh |
| a set exists | `set.<i>` u32 (`i` = position in `sets`) |
| a tag exists | `tag.<i>` i64 or f64 |
| a level map exists | `levelmap.<i>.coarse_of`, optionally `levelmap.<i>.face_map` |
| an extrusion map exists | `extrusion.<i>.z`, `extrusion.<i>.cell_base`, `extrusion.<i>.cell_layer` |

Geometry rule `smartmesh.fan_vertex_mean.v1`: face apex = vertex mean, area vector
`1/2 sum (p_i - apex) x (p_{i+1} - apex)`, cell volume the sum of side tetrahedra of the fan
polyhedron. A consumer that needs only planar faces can use `V = 1/3 sum_f s_f (p_f . A_f)` with
the Newell area vector `A_f` and `s_f = +-1` from the cell side.

## 4. Determinism

The file is a pure function of the mesh and the attached metadata: the writer emits arrays in a
fixed order, fills them with fixed-size parallel chunks, and writes compact JSON with a fixed
field order. Files are byte-identical across runs and thread counts; every scalar goes through explicit
little-endian conversion, so host endianness cannot change the bytes (only little-endian hosts
are tested). Coordinates are stored as exact IEEE bit patterns.

## 5. Versioning and extension

* Add an optional array or header field: minor bump.
* Change the meaning of an existing array, an enum code, or the layout: major bump.
* Unknown `cell_shape` / `face_kind` codes: a reader that needs them must refuse the file;
  new codes only appear with a minor bump when the mesh API grows a shape.
* The schema does not depend on the container: an HDF5 or Zarr mapping can reuse the array
  names, dtypes and shapes verbatim (ARCHITECTURE section 6).

## 6. Reading in a few lines

```python
import json, mmap, struct
m = mmap.mmap(open(path, "rb").fileno(), 0, access=mmap.ACCESS_READ)
(hl,) = struct.unpack_from("<Q", m, 8)
hdr = json.loads(m[16:16 + hl])
start = 16 + hl
a = next(a for a in hdr["arrays"] if a["name"] == "x")
x = memoryview(m)[start + a["offset"]: start + a["offset"] + a["nbytes"]].cast("d")
```

`readers/smsh.py` wraps this with typed access, CRC verification and cell / face helpers.
