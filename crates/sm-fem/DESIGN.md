# sm-fem: blocks, sidesets, deck lowering, export

## Positioning

sm-fem is similar to Cubit's `block` / `sideset` / `export` commands (and to the Exodus
element-block and side-set model), but differs in where the data lives: a block is the *volume
color* and a sideset is a *face color* of the `model` layer, so membership is an exact partition
owned by `sm_model_api::Model` and follows topology lineage for free, and nothing about blocks
is stored in the mesh. Sets are *lowered* onto a copy of the mesh at export time from the
classification of the mesh (cells on volumes, faces on faces). That gives a mesh that can be
re-meshed, deleted or exported repeatedly without block bookkeeping going stale, an export
that is a pure function of `(state, request)`, and deck boundaries that reach the file as named
face sets plus `DeckRecord`s from the same table.

## Pieces

| Piece | Role |
|---|---|
| `FemPlugin` | registers `FemLayer` and `fem.block_add`, `fem.block_remove`, `fem.block_element_type`, `fem.sideset_add` |
| `FemLayer` (`fem`, depends on `model`) | element type per block color; the volume a sideset face is taken `wrt` |
| `lower(model, mesh, fem, alloc)` | blocks, sidesets, decks to `MeshSet`s (+ `DeckRecord`s) on a clone of the mesh |
| `export(state, req, w)` | pick the mesh, `lower`, write `.smsh` (`SmshWriter`) or VTU (`write_vtu`) |
| `ElementType` | `HEX8 TET4 WEDGE6 PYRAMID5 QUAD4 TRI3`, plus higher-order names that are refused |

## Rules

- **Blocks are volume colors.** `fem.block_add {block, volumes?, name?}` finds or creates the
  `Domain::Volume` color with label `block` (named `block_<n>`) inside `model_txn`, colors the
  volumes, and reports the color as role `block`. No topology operation is opened (`ops_run` is
  unchanged). A volume has one color, so moving it out of another block emits a `block-move`
  warning. Every `Domain::Volume` color lowers to a block, however it was created.
- **Sidesets are face colors.** `fem.sideset_add {sideset, faces, wrt?, name?}` is the same for
  `Domain::Face`. `wrt` (a volume that owns every face) is stored in `FemLayer` per face, and follows topology
  lineage (`on_succession`): a split face passes it to its children, a split volume becomes all
  its pieces (the layer has no topology access, so the lowering picks the piece the face borders).
- **Lowering** (`lower`):
  - block `c` becomes `SetMembers::Cells` named like `c`, label `c.label`, found through
    `classified_on(volume).cells` (no per-cell model lookup);
  - sideset `c` becomes `SetMembers::Sides`: for each classified face entity whose
    `Model::live_face_color` is `c` (so a color on a hidden overlay host reaches the mesh faces
    classified on the live `PartitionFace` pieces), the mesh faces classified on it; with `wrt` the side(s) whose cell is classified on that volume, without it a boundary
    face gives its `Left` side and an interior face both sides;
  - deck boundary `b` becomes `SetMembers::Faces` `deck_<b>_a` (side A, or the self-paired
    mirror faces) and `deck_<b>_b`, and `DeckRecord::from_deck(b, deck, a, b)` names them;
  - a `wrt` volume is matched against the cell classification, through `live_cover` when the
    volume is a hidden overlay host (cells on its `PartitionCell` pieces); a `wrt` volume that
    is gone and covered by nothing is `unsupported`, never a silently empty set;
  - deck faces on a hidden host face are found through the pieces that cover it;
  - empty sets are not emitted; colors and boundaries are visited in ascending id order;
  - a declared element type is checked against `cell_shape` of every cell of the block
    (sequential, reporting the first offending cell). An undeclared type is not checked.
  - **The lowering reads the model, not the `fem` layer.** Every volume color, face color and
    deck boundary with cells or faces in the mesh is written, with or without a `fem` layer;
    the layer only adds declared element types and `wrt`. "No sets" therefore holds exactly
    when the model has no colors or decks (`export_without_a_fem_layer_writes_no_sets`); a
    fem-less state that has model colors exports them
    (`fem_less_export_still_lowers_the_models_colors_and_decks`).
- **Set ids** come from a clone of `MeshLayer::alloc()`; nothing is adopted into the entity
  table and the `meshes` layer is never mutated. If there is nothing to add, the stored mesh is
  shared (`Arc`), not copied.
- **Export** with `mesh = None` takes the only mesh; none is `no-mesh`, several is
  `ambiguous-mesh`; a stale mesh is `stale-mesh`. A state without the `fem` layer lowers with
  no declared types; a state without the `model` layer writes the mesh as it is.
  `FemError::code()` gives the stable kebab-case codes.

## Layer digest (version 1)

Element-type count, then `(color id, type code)` ascending; side-preference count, then
`(face id, volume id)` ascending (one pair per candidate volume). Integers only, so exact on every platform. The layer is
empty (and invisible to the state hash) until a type or a `wrt` is recorded, so installing the
plugin does not change the hash of histories that do not use it (tested).
`on_retired` drops entries whose color or face was retired and removes retired volumes from
`wrt` lists (an empty list drops the entry).

## Determinism

Every map is a `BTreeMap`; sets are built from sorted index lists; there is no
parallel code in this crate. The tests compare state, layer and export
hashes at 1 thread, 4 threads, default pool and concurrent runs, and replay cold from the root.

## Limits (W2)

Shell blocks (`faces` in `block_add`) are refused: meshes hold volume cells only. A non-empty
block cannot be removed because `Model` has no way to uncolor a volume (api change request in
`W2_REPORT.md`). Higher-order element types wait for degree elevation. The ops accept `TopoFace` only;
the lowering handles overlay `PartitionFace`/`PartitionCell` classification but no W2 op builds
an overlay, so that path is not tested end to end. Side and face sets exist only in `.smsh`: the
VTU writer (sm-meshio) emits cell sets and a `region` array only.
