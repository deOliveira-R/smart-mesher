# sm-exodus: design

## Positioning

Similar to the Exodus II writer in SEACAS/libexodus (`ex_create`, `ex_put_*`) over netCDF-C, and to
the Exodus writers of meshio and Cubit, but it differs in four ways, giving a writer that is
deterministic, dependency-free and geometrically checked:

1. **Pure Rust, own netCDF encoder.** No libnetcdf, no HDF5. We write only the classic family
   (CDF-1, CDF-2 = 64-bit offset, CDF-5 = 64-bit data) that every Exodus reader still opens;
   libexodus writes netCDF-4/HDF5 by default today. We give no netCDF-4 output (deferred).
2. **Face-first IR in, polyhedra first class.** The writer reads `sm_mesh_api::MeshRead` only. True
   polyhedra (`NFACED`), polygons (`NSIDED`), mixed blocks, classification-derived sets and the
   cell/face orientation come straight from the IR. Template cells keep their exact internal
   corner order (it already equals the Exodus order) and each side is mapped to the Exodus side
   number per element type.
3. **Pure function of mesh and options.** Same bytes across runs and thread counts. libexodus
   embeds the wall-clock time in its QA records; we write a fixed placeholder record.
4. **Side numbering verified by geometry**, not by a round trip through the same table: the tests
   check that the face the Exodus table names is the face the mesh set names, against node lists
   typed from the Exodus manual.

Against meshio the gains are polyhedra, side sets and node sets with names and ids, and no
netCDF4/h5py dependency. Against libexodus: determinism, no native build, and streaming output to
any `Write`.

## Modules

| module | role |
|---|---|
| `netcdf` | `NcFile` model + codec (`layout`, `write`, `to_cdl`, `read`); formats `Classic`/`Offset64`/`Data64` |
| `tables` | `exodus_side`, `local_face`, `exodus_element_type` |
| `options` | `ExodusOptions`, `BlockSelection` |
| `plan` | `plan()`: blocks, element numbering, side/node sets, name lengths (no I/O) |
| `write` | `write_exodus`, `exodus_bytes`, `exodus_nc`; builds the `NcFile` with lazy variables |
| `read` | `read_exodus` -> `ExodusData`: internal decoder for round-trip tests only |

### netCDF codec

Header (all big-endian): magic, `numrecs` (0), `dim_list`, global `att_list`, `var_list`. Widths:
list tags and `nc_type` 4 bytes; `numrecs`, element counts, name lengths, `dim_length`, `ndims`,
dimension ids, attribute counts and `vsize` are 4 bytes (CDF-1/2) or 8 (CDF-5); `begin` is 4 bytes
in CDF-1 and 8 in CDF-2/5. `ABSENT` is tag 0 + count 0. Names, attribute values and non-record
variables are padded to 4 bytes with zeros.

* `layout()` needs no data: header size is computed by serialising the header with zero offsets;
  non-record variables follow in declaration order, record variables (here only `time_whole`, zero
  records) after them, so their `begin` is the end of the file.
* CDF-1 errors `TooLarge` when a `begin`, a `vsize`, a dimension or the file end exceeds `u32::MAX`
  (stricter than the spec, which would allow the last variable to end past 4 GiB).
* CDF-2 clamps `vsize` to `0xFFFF_FFFF` for a variable over 4 GiB and requires it to be the last
  non-record variable; dimensions are 32-bit (a 5 GiB variable is 2-D).
* CDF-5 widens counts, dimension ids and `vsize`; the only format with `NC_INT64`.
* A non-record dimension of length 0 is the record-dimension marker in the header and is rejected;
  consequently `num_node_sets`, `num_side_sets`, `num_qa_rec` and `num_info` (and their variables)
  are omitted when empty, as libexodus does.
* `NcData::Lazy` providers are called once per write, so peak memory is about the largest
  variable. Packing inside a variable is `rayon` over preallocated chunks (order independent).

## Mapping rules

1. Nodes = vertices in index order. `num_dim = 3`, or 2 for a dimension-2 mesh with every z == 0
   (`coordz` omitted). High-order cells -> `Unsupported`; extra points beyond `vertex_count` are
   ignored otherwise. Only active cells are written.
2. Blocks: candidates are the `Cells` sets chosen by `BlockSelection` (default: labelled sets);
   they must be disjoint (`OverlappingBlocks`), labels must be positive and unique (`BadLabel`).
   A one-shape set is one block (`id = label`, name = set name). A mixed set splits by shape: the
   lowest `CellShape::code` keeps label and name, the others get fresh ids (above the largest
   label, ascending by shape code) and `<set>_<ELEMTYPE>`. Cells in no set go to
   `unassigned_<ELEMTYPE>` blocks (fresh ids, ascending shape code). Blocks sorted by id; elements
   numbered block by block, ascending cell index; `elem_num_map[e] = cell + 1`.
3. Template connectivity = `cell_corners + 1` (no permutation).
4. Side sets: `SetMembers::Sides` gives one entry per `(face, side)`; the cell is `face_left` or
   `face_right`; the template face is the one whose vertex cycle equals `face_vertices(f)`
   (`loop_relation`: `Same` for Left, `Reversed` for Right; otherwise `Mesh(InconsistentOrientation)`).
   `Faces` sets (deck face sets) become side sets on the Left cell when `faces_as_sidesets`.
   Entries keep set order; entries on inactive cells are dropped and counted in `skipped_sides`.
   Id = label, else the next unused positive id above the largest label of that family.
5. Node sets: `Vertices` (and `Points` below `vertex_count`); same id rule. Empty sets are not written.
6. Polygons (dimension 2) -> `NSIDED` block; side = 1-based position of the edge in the polygon's
   cycle. Polyhedra -> `NFACED` block over a face block (see below).

## Side tables (ours -> Exodus, 1-based sides)

| shape | our face index -> Exodus side | Exodus side -> nodes (1-based, outward) |
|---|---|---|
| HEX8 | `[0..5] -> [5, 6, 1, 2, 3, 4]` | 1: 1,2,6,5; 2: 2,3,7,6; 3: 3,4,8,7; 4: 1,5,8,4; 5: 1,4,3,2; 6: 5,6,7,8 |
| TET4 | `[0..3] -> [4, 1, 2, 3]` | 1: 1,2,4; 2: 2,3,4; 3: 1,4,3; 4: 1,3,2 |
| WEDGE6 | `[0..4] -> [4, 5, 1, 2, 3]` | 1: 1,2,5,4; 2: 2,3,6,5; 3: 1,4,6,3; 4: 1,3,2; 5: 4,5,6 |
| PYRAMID5 | `[0..4] -> [5, 1, 2, 3, 4]` | 1: 1,2,5; 2: 2,3,5; 3: 3,4,5; 4: 1,5,4; 5: 1,4,3,2 |
| QUAD4, TRI3 | edge `i -> i + 1` | edge `i` runs node `i+1` -> `i+2` (cyclic) |
| NFACED | position in the element's face list (`cell_faces` order) | |
| NSIDED | position of the edge in the polygon cycle | |

HEX8 side 1 is the -y face of the unit cube (nodes 1,2,6,5). The right-hand column is the Exodus
II manual numbering, typed independently in `tests/common/mod.rs` (`manual_side_nodes`); the
tests compare it with the mesh by loop orientation and by centroid.

## Variables written

Global attributes: `api_version` (float 8.11), `version` (float 8.11), `floating_point_word_size`
(int 8), `file_size` (int 1), `maximum_name_length`, `int64_status`, `title`.

Dimensions: `time_step` (unlimited), `len_string` 33, `len_line` 81, `four` 4, `len_name`
(= `maximum_name_length` + 1), `num_dim`, `num_nodes`, `num_elem`, `num_el_blk`, and when
non-empty `num_node_sets`, `num_side_sets`, `num_qa_rec`, `num_info`; per block `num_el_in_blk<k>`,
`num_nod_per_el<k>`; per node set `num_nod_ns<k>`; per side set `num_side_ss<k>`.

Variables: `time_whole(time_step)`, `coordx|y|z(num_nodes)` (z only for `num_dim = 3`),
`coor_names(num_dim, len_name)`, `eb_status`, `eb_prop1` (attr `name = "ID"`), `eb_names`,
`connect<k>(num_el_in_blk<k>, num_nod_per_el<k>)` with attr `elem_type`, `ns_status`, `ns_prop1`
(`name = "ID"`), `ns_names`, `node_ns<k>`, `ss_status`, `ss_prop1` (`name = "ID"`), `ss_names`,
`elem_ss<k>`, `side_ss<k>`, `elem_num_map(num_elem)`, `node_num_map(num_nodes)` (on request),
`qa_records(num_qa_rec, four, len_string)`, `info_records(num_info, len_line)`. Names are char
arrays NUL-padded to `len_name`. Ids and connectivity are `NC_INT`, or `NC_INT64` with
`ExodusOptions::int64` (needs CDF-5). Info lines longer than 80 bytes are split; the provenance
string becomes the info record `smart-mesh provenance: <s>`.

### NSIDED / NFACED (experimental, names from memory of libexodus)

No SEACAS header or source is reachable from the build VM, so these names are recalled, not
checked; `ExodusOptions::polyhedra = true` is the default but the report marks the output
experimental until a SEACAS reader confirms it.

* `NSIDED` block: `connect<k>(num_nod_per_el<k>)` (total nodes, attr `elem_type = "NSIDED"`),
  `ebepecnt<k>(num_el_in_blk<k>)` = nodes per element (attrs `entity_type1 = "NODE"`,
  `entity_type2 = "ELEM"`).
* `NFACED` block: `facconn<k>(num_fac_per_el<k>)` (total face references; attr `elem_type = "NFACED"`; global face numbers,
  1-based), `ebepecnt<k>(num_el_in_blk<k>)` = faces per element (attrs `FACE`, `ELEM`).
* Face block `j` per NFACED block: `num_fa_in_blk<j>`, `num_nod_per_fa<j>`,
  `fbconn<j>(num_nod_per_fa<j>)` (attr `elem_type = "NSIDED"`), `fbepecnt<j>(num_fa_in_blk<j>)` (attrs `NODE`, `FACE`), plus
  `num_fa_blk`, `num_face`, `fa_status`, `fa_prop1` (`name = "ID"`), `fa_names`.
* Names follow `exodusII.h` as recalled (`DIM_NUM_FAC_PER_EL` = `num_fac_per_el%d`,
  `DIM_NUM_FA_IN_FBLK` = `num_fa_in_blk%d`, `VAR_FCONN` = `facconn%d`, `VAR_FBCONN` = `fbconn%d`).
  Still not checked against SEACAS sources (none reachable here); a real libexodus/ParaView
  read-back is the open verification.
* `int64_status` is a libexodus bitmask (maps 1, ids 2, bulk 4, inquiry 8, from memory); with
  `int64 = true` we write 15 (all), since maps, ids and bulk data are all `NC_INT64`.
* Side numbers of polygons and polyhedra are `u32` (position in the face list).
* One outward-oriented face copy per (cell, face), the loop reversed for `Right` entries (the
  smallest vertex stays first); element face order = `cell_faces` order; no shared faces.

## Determinism

`plan` and the writer take only ordered inputs (sets are ordered by id, cells by index) and use
`rayon` only for indexed maps into preallocated buffers. Errors from parallel maps are reported
in index order. Tests: `acc8_*` (two runs; `rayon` pools of 1 and 4 threads; the testkit harness),
and a regression hash of the 20^3 hex block file.

## Performance (record only, decision 12)

`cargo bench -p sm-exodus --bench exodus`, 4 vCPU (`nproc` = 4), release build, `io::sink`:

| mesh | file | time |
|---|---|---|
| 40^3 hex (64,000 cells, 68,921 nodes) | 3,959,424 bytes | 1.68 ms (2.2 GiB/s) |
| 100^3 hex (1,000,000 cells, 1,030,301 nodes) | 60,728,544 bytes | 16.6 ms (3.4 GiB/s) |

(target: under 2 s for 1e6 hexes.)

## Deferred (W3b or later)

netCDF-4/HDF5 output; time steps and results (nodal/element/global variables); high-order types
(HEX20/27, TET10, WEDGE15, PYRAMID13, QUAD8, TRI6) until degree elevation exists; distribution
factors; element attributes; assemblies; edge blocks and edge/face/element sets; shell side sets
(`SetMembers::ShellSides`, SHELL4 / TRISHELL3) and `num_dim = 3` surface meshes; shared-face
NFACED; reading general third-party Exodus files; parallel (Nemesis) files; a vertex-set lowering
in sm-fem; wiring into `session.export` / CLI / MCP (integration step).
