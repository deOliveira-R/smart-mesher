# DAGMC `.h5m` (MOAB HDF5) as written by `sm_meshio::dagmc`

Status: W3b-EXPORT. A faceted B-rep that DAGMC (and OpenMC through it) loads: triangles,
surfaces with a forward and a backward volume, volumes, `mat:` groups. Output is a pure function
of the model and options: no timestamps, no history, children sorted by name, one layout pass.

## 0. Sources and evidence

* **Reference file.** `tools/conformance/fixtures/ref/dagmc_two_volume.h5m` (DAGMC
  `src/dagmc/tests/test_dagmc_impl.h5m`, MOAB 4.9.1, BSD-2-Clause, see `README.md` there) and its
  full `h5dump`. Every layout fact below is read off that dump and off `h5py` (names, datatypes,
  shapes, attribute values, table rows).
* **MOAB headers** (conda-forge `moab` 5.6.0, read as specification, not copied):
  `moab/Types.hpp:121-122` (`MESHSET_SET = 0x2`, `MESHSET_ORDERED = 0x4`),
  `MBTagConventions.hpp` (`GEOM_DIMENSION`, `GLOBAL_ID`, `CATEGORY` / `CATEGORY_TAG_SIZE 32`,
  `NAME` / `NAME_TAG_SIZE 32`), `moab/mhdf_public.h` (the `mhdf_*` reader API, `mhdf_openFile`,
  `mhdf_getFileSummary`). The MOAB `.cpp` sources (`WriteHDF5.cpp`, `ReadHDF5.cpp`, `mhdf.c`,
  `GeomTopoTool.cpp`) were not vendored or copied; where this file states a rule that is not in
  a header (the set-table column semantics, the range bit 8), it is the reference file's
  behaviour, checked by the next point.
* **Independent readers** (`tools/conformance/dagmc/`, Tier B, run 2026-11-04 on
  conda-forge `moab` 5.6.0 / `dagmc` 3.2.4 / HDF5 2.2.0): `mbconvert` reads our file and re-writes it
  with the same triangles, sets and names; libdagmc builds the OBB tree, sees our volumes, surfaces and
  `mat:` names, answers `point_in_volume` for 1000 seeded points and `ray_fire` for 200 seeded rays
  per volume with the analytic result (max distance error 7e-16). `h5py` 3.16 (Tier A) reads every
  object and the same invariants hold for our file and the reference.

## 1. HDF5 subset (`src/h5/`, private, no `sm-*` imports)

| Structure | Written as |
|---|---|
| Superblock | version 2, offsets and lengths 8 bytes, base 0, extension undefined, end-of-file = file length, lookup3 checksum |
| Object header | version 2 (`OHDR`), chunk size width 1/2/4/8 bytes by size, no timestamps (flag bit 5 clear), no creation order, no attribute phase-change values, lookup3 checksum |
| Group | Link Info (fractal heap and name index undefined) + Group Info + one hard Link message per child, children in byte order of the name |
| Dataset | Dataspace v2, Datatype, Data Layout v3 class 1 (contiguous), raw data right behind its header; zero length: undefined address |
| Committed datatype | an object header holding one Datatype message |
| Attribute | message v3, ASCII name, inline datatype and dataspace (scalar: rank 0) |
| Datatypes | `u8`, `i32`, `i64`, `u64`, `f64` (all little endian), opaque of N bytes with an empty tag, `H5T_ARRAY[n]` of `u64`, enum over `u8` |

Anything else is `H5Error::Unsupported`. Checksum function: Jenkins lookup3 (`hashlittle`, initial
value 0; test vectors in `h5/tests.rs`). Addresses: the superblock is 48 bytes, the root group
follows; an object, then its data, then its children recursively, so the layout is the pure
function of the tree.

**Committed vs inline types.** MOAB opens `/tstt/elemtypes` and `/tstt/tags/<tag>/type` by name,
so those are committed datatype objects. Datasets and attributes carry an inline copy of the same
datatype instead of a shared-message reference to it (the reference file shares). The two are
equal as datatypes; `check_h5m.py` compares dtypes, not "is shared".

**No `/tstt/history`.** The reference has a variable-length string dataset (MOAB version and
date), outside the subset (vlen data needs a global heap). `mhdf_openFile` and DAGMC load work
without it (Tier B). It would also be the only byte that depends on time.

## 2. Handles and the tree

Entity handles are 1-based and contiguous across tables, in file order: nodes
`1..=N`, then `Tri3` `N+1..=N+T`, then sets. Each table's first handle is its `start_id` attribute
(`i64`); `/tstt` carries `max_id` (`u64`) = the last handle. Reference: nodes 1, `Edge2` 25, `Tri3`
61, sets 97, `max_id` 181.

```
/tstt                          @max_id u64
  elemtypes                    committed enum u8: Edge 1, Tri 2, Quad 3, Polygon 4, Tet 5, Pyramid 6,
                               Prism 7, Knife 8, Hex 9, Polyhedron 10
  nodes/coordinates            f64 [N,3]   @start_id = 1
  elements/Tri3                @element_type = enum Tri (2)
    connectivity               u64 [T,3]   @start_id = N+1; node handles (1-based)
  sets/list                    i64 [S,4]   @start_id = N+T+1
  sets/contents                u64 [C]     (omitted when empty)
  sets/children                u64 [Ch]    (omitted when empty)
  sets/parents                 u64 [P]     (omitted when empty)
  sets/tags/GLOBAL_ID          i32 [S]     dense tag values
  tags/<TAG>/                  one group per tag, see section 4
```

Not written (reference has them): `Edge2` and the curve and vertex sets (`include_curves` is
deferred), `GEOM_SENSE_N_ENTS` / `GEOM_SENSE_N_SENSES` (curve-to-surface senses), `OBB` (DAGMC
rebuilds it), `GEOMETRY_RESABS`, `EXTRA_NAME*`, `DIRICHLET_SET` / `NEUMANN_SET` / `MATERIAL_SET`
definitions, per-table `tags/GLOBAL_ID` of nodes and elements, `history`. `check_h5m.py` compares
our tree with the reference as "everything we write exists there with the same kind, dtype, rank
and attribute dtypes" and lists the rest as informational.

## 3. The set table

`sets/list` has one row per set, handle `start_id + row`: `[contents_end, children_end,
parents_end, flags]`. Each of the first three is the **cumulative inclusive index of the last
entry** of that set in the `contents`, `children` and `parents` tables (`-1` before anything is
written); an empty set repeats the previous end. `flags`: `2` set semantics (unordered, no
duplicates), `4` ordered, `8` contents are range-encoded `(start, count)` pairs
(`MESHSET_SET`, `MESHSET_ORDERED` from `Types.hpp`; the range bit is the reference's row 0).
We write flags `2` for every set (the reference: surfaces `2`, curves `4`, volumes and groups `2`),
contents as plain handle lists.

Set order, after an optional file set: surfaces by ascending model id, volumes by ascending id,
groups by name.

| Set | contents | children | parents |
|---|---|---|---|
| file set (only with `faceting_tol`) | one range `(1, max_id)`, flags `8` | | |
| surface | its `Tri3` handles, ascending | | the volumes of its sense, forward first |
| volume | | the surfaces that name it in their sense | |
| group `mat:<name>` | its volume handles | | |

The reference surface sets also hold their vertex nodes in `contents`; we do not (DAGMC reads
triangles by type). Volume 177 of the reference is in two material groups (`mat:Graveyard` and
`mat:Steel_comp`): a volume can be in several groups, and ours can too
(`h5m.volume_in_multiple_groups`, Info).

## 4. Tags

`/tstt/tags/<TAG>` holds attributes `class` (`i32`: 1 sparse, 2 dense), the committed `type`, and
for a sparse tag `id_list` (`u64` handles ascending) and `values` (same length, the tag's type).
Dense tags keep their values with the table (`sets/tags/GLOBAL_ID`).

| Tag | class | type | other attributes | on |
|---|---|---|---|---|
| `GLOBAL_ID` | 2 | `i32` | `default` 0, `global` 0 | all sets, dense: 1-based per category (surfaces `1..`, volumes `1..`, groups `1..`; the file set 0) |
| `GEOM_DIMENSION` | 1 | `i32` | `default` -1, `global` -1 | surfaces (2) and volumes (3) |
| `CATEGORY` | 1 | opaque 32 | | `Surface`, `Volume`, `Group` |
| `NAME` | 1 | opaque 32 | | groups: `mat:<set name>` |
| `GEOM_SENSE_2` | 1 | `H5T_ARRAY[2] u64` | `default` `[0,0]`, `is_handle` `i32` 1 | surfaces: `[forward, backward]` volume handles, 0 = none |
| `FACETING_TOL` | 1 | `f64` | | the file set, only with `faceting_tol` |

`GEOM_SENSE_2.is_handle = 1` makes MOAB remap the stored file handles to runtime handles on read.
Opaque values are 32 bytes, NUL padded, with at least one NUL: names of 32 bytes or more (with the
`mat:` prefix) are `DagmcError::NameTooLong`, never truncated. `GLOBAL_ID` of surfaces and
volumes is what OpenMC reports as ids; it is dense and ascending by model serial (so serial gaps
close up).

## 5. Sense

`forward` is the volume the right-hand normal of the surface's triangles points OUT of; the
`backward` volume is the one it points into (none at the model boundary). This is the IR rule
(`face_left` is the forward volume), the rt-api `front`, and DAGMC's `GEOM_SENSE_2[0]`.
**The reference is the oracle**: `check_h5m.py` computes the signed volume of each reference
volume by the divergence theorem over the triangles oriented out of it per `GEOM_SENSE_2` and gets
`1000` (the `[-5,5]` cube) and `2629.809 = 16.9^3 - 13^3` (the shell between the 6.5 and 8.45 boxes,
whose inner surfaces face into the cube), both positive, every edge used twice in opposite
directions. The same check passes for our files.

One surface has one sense. The exporter names the forward volume from the first triangle (the
lowest face of the surface) and reverses the winding of any triangle written the other way round
(an `InconsistentSense` error if a triangle matches neither). So `face_left` may differ between
the faces of one `TopoFace`; the file does not.

## 6. Faceting rules

* Surfaces are the faces classified on a `TopoFace` that bound a cell and separate two different
  volumes or a volume and the outside; faces inside one volume are skipped. A boundary face
  without a `TopoFace` is `Unclassified`; partition-classified entities are `Unsupported`.
* Triangles stay; quads split on the shorter diagonal (equal: the diagonal through the lowest
  vertex index); polygons of five or more vertices fan from a new face-centroid vertex (centroids
  come after the used mesh vertices, in face order). Only vertices used by a triangle are written,
  in ascending mesh index.
* Every volume must close: each edge of its triangles is used exactly twice, in opposite
  directions (`OpenSurface` names the first open edge; more than two uses is `NonManifoldSurface`).
  Nothing is repaired, welded or snapped, and no tolerance enters.
* Graveyard (`DagmcOptions::graveyard`, off by default): two nested axis-aligned boxes. The inner
  box is the bounding box scaled by `scale` (1.1) about its centre, the outer is `thickness_frac`
  (0.05) of the longest inner edge further out; 12 surfaces of 2 triangles, one volume, group
  `mat:Graveyard`. Both boxes' normals point out of the graveyard (into the model at the inner
  box), every surface is `[graveyard, none]`, like the reference. Skipped, with
  `h5m.graveyard_exists`, when the model already has a group named `Graveyard`.
* Groups: each non-empty cell set is `mat:<set name>` holding the `TopoVolume`s of its cells;
  `DagmcOptions` has no prefix or name map (Q6). Face, vertex and side sets are dropped
  (`h5m.sets_dropped`). Volumes in no group: `h5m.volume_without_material`. A set that holds part
  of a volume puts the whole volume in the group: `h5m.set_cuts_volume`.

## 7. Loss codes (stable)

`h5m.high_order_dropped`, `h5m.face_curves_dropped`, `h5m.sets_dropped`, `h5m.set_cuts_volume`,
`h5m.volume_without_material`, `h5m.volume_in_multiple_groups`, `h5m.graveyard_exists`.

## 8. Not covered

Curve (`Edge2`) and vertex sets, reading `.h5m`, CAD names and colours from STEP, density and
temperature metadata on `mat:` groups, `Partition*` classification (an sm-fem lowering step), and
the MOAB `history` dataset.
