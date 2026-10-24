# W2 report: sm-fem (T8)

## Summary

`sm-fem` now implements plan §2 T8: `FemLayer`, `FemPlugin` with `fem.block_add`,
`fem.block_remove`, `fem.block_element_type` and `fem.sideset_add`, `lower` (blocks, sidesets and
deck boundaries to mesh sets and `DeckRecord`s) and `export` (`.smsh` / VTU, pure function of
`(state, request)`). The skeleton entry points keep their signatures. `FemError` is now a
`thiserror` enum with stable `code()` strings. Design and rules: `DESIGN.md`.

Definition of done, all run on this tree: `cargo build --workspace`;
`cargo test -p sm-fem --all-features` and with `RAYON_NUM_THREADS=1` (34 tests); `cargo clippy -p
sm-fem --all-targets --all-features -- -D warnings`; `RUSTDOCFLAGS="-D warnings" cargo doc -p
sm-fem --no-deps`. No benchmark: T8 names none.

## Acceptance bullet to test

All in `crates/sm-fem/tests/`.

| Acceptance bullet | Test |
|---|---|
| Stub-meshed brick plus block 1 gives a `.smsh` with cell set `block_1`, label 1, 1000 cells; `SmshFile::decode` round trip | `acceptance::smsh_has_cell_set_block_1_with_label_1_and_1000_cells` |
| VTU has `set:block_1` all 1 | `acceptance::vtu_has_set_block_1_array_all_ones` |
| Two volumes in two blocks: disjoint, cover all cells | `acceptance::two_volumes_two_blocks_are_disjoint_and_cover_all_cells` |
| TET4 block on hexes is an error | `acceptance::tet4_block_on_hexes_is_an_error_and_hex8_is_fine`, `acceptance::element_check_is_deterministic_on_the_parallel_path` |
| Sideset on +x face: 100 sides, all `Left` | `acceptance::sideset_on_plus_x_face_has_100_left_sides` (also checks x = 10 to 1e-12 and no right cell) |
| `mirror_chamber` deck face set on the mirror plane (1e-12), `DeckRecord` names it | `acceptance::mirror_chamber_deck_face_set_lies_on_the_mirror_plane` |
| Export byte-identical over runs and thread counts | `acceptance::export_is_byte_identical_across_runs_and_thread_counts`, `determinism::state_and_exports_are_identical_across_threads_and_runs` (`sm_testkit::determinism::check`) |
| Export with no `fem` layer works, writes no sets | `acceptance::export_without_a_fem_layer_writes_no_sets` |
| `block_add`: color created via `model_txn`, role `block`, no topology op | `ops::block_add_creates_the_color_and_outputs_it_without_a_topology_op` |
| Moving a volume out of a block warns | `ops::moving_a_volume_out_of_another_block_warns` |
| Higher-order types unsupported | `ops::block_element_type_is_stored_and_validated` |
| `model_txn` retires removed ids | `ops::block_remove_retires_the_color_and_its_element_type` |
| `OUTPUTS`, `x-entity-kind`, positional declared | `ops::ops_declare_outputs_positional_and_entity_kinds` |
| Cold replay from root, no `Diverged`; undo/redo | `determinism::cold_replay_from_the_root_reproduces_the_state_and_the_exports`, `determinism::undo_and_redo_restore_the_fem_layer` |
| Mesh selection rule (decision 8) | `acceptance::mesh_selection_errors_are_typed` |
| Lowering is export-time only (layer untouched, mesh shared when nothing to add) | `acceptance::smsh_has_cell_set_...` (stored mesh has no sets), `acceptance::nothing_to_lower_shares_the_stored_mesh` |

Review fixes (fix0), each with a test unless noted:

- `FemLayer::on_succession`: `side_wrt` follows face splits (children inherit) and volume splits
  (all pieces become candidates; the lowering resolves which piece a face borders).
  `acceptance::sideset_wrt_follows_a_face_and_volume_split` (real webcut), `layer::tests`.
  `side_wrt(face)` now returns `&[EntityId]` (empty = none); `side_prefs()` still yields pairs.
- Sideset and deck lowering go through `live_face_color` / `live_cover`, and `wrt` is matched
  through `live_cover` of a hidden host volume. A `wrt` volume that is gone and covered by
  nothing is `FemError::Unsupported`. **Not tested end to end:** no W2 op builds an overlay in a
  state, so the overlay branches follow the `Model`/`TopologyView` contract only.
- Model colors lower without a `fem` layer by design
  (`acceptance::fem_less_export_still_lowers_the_models_colors_and_decks`);
  `export_without_a_fem_layer_writes_no_sets` holds because that model has no colors. Wording
  fixed in DESIGN and in the rustdoc of `lower`.
- New tests: translation pair deck with `deck_pair_a/_b` and `mate_set` Some
  (`translation_pair_deck_has_two_face_sets_and_a_mate`); sideset `.smsh` round trip checks Left
  sides on x = 10; VTU carries cell sets and `region` only, no side/face sets
  (`vtu_carries_cell_sets_only_not_side_or_face_sets`); a set-name clash is `FemError::Mesh`.
- `references_any` builds one sorted id list; `SidesetAdd` binary-searches a sorted face list.
- `fem.block_remove` and `fem.block_element_type` declare `OUTPUTS = &[]` explicitly.
  `fem.block_element_type` refuses QUAD4/TRI3 with the "shell blocks" message.
- The mirror acceptance test builds the model with an in-test op (`test.mirror_chamber`), not
  `sm_testkit::topo::mirror_chamber()`: that fixture returns a bare `CellComplex` + `Model`, and
  `ModelLayer` has no constructor taking a `Model`, so it cannot be loaded into a `State`.
  The in-test op replays the same construction through `topo_txn`/`commit_topology`.
- sm-meshio has no VTU reader, so VTU checks parse the appended arrays by hand. The `.smsh`
  checks go through `SmshFile::parse/decode`.
- Not done: lowering still clones the mesh when it adds sets (O(mesh) per export, not measured
  at 1e6 cells). Avoiding it needs an overlay `MeshRead` in sm-meshio/sm-mesh-api, outside this
  crate; the identity path stays zero-copy.

Extra coverage: interior-face sidesets with and without `wrt`
(`acceptance::interior_face_sideset_takes_both_sides_or_the_side_of_wrt`), bad input to every op,
retirement cleaning of `FemLayer`, layer digest, empty layer invisible to the state hash.

## Deviations (with reasons)

1. **`FemLayer` also stores `side_wrt` (face to volume).** The plan lists only `element_types`.
   `fem.sideset_add {wrt}` needs a per-face place to keep the side preference and `Model` has none.
2. **`fem.sideset_add` and `fem.block_add` take an optional `name`** (default `sideset_<n>` /
   `block_<n>`). Not in the plan; it is the only way to name a color without another op.
3. **`faces` in `fem.block_add` is accepted by the schema but refused when non-empty.** Face blocks
   (shell elements) need a surface mesh in the `meshes` layer; W2 meshes hold volume cells only.
4. **`fem.block_remove` only removes an empty block.** `Model` has no `uncolor_volumes`, so a
   non-empty block gives an error telling the user to move its volumes first (api change request 1).
5. **Export refuses a stale mesh** (`stale-mesh`). Its classification names retired entities, so
   block membership would silently be empty or wrong. Not in the plan; the skeleton did not check.
6. **Interior faces without `wrt` lower to both sides**; boundary faces to `Left`. The plan
   says only "wrt volume".
7. **Empty sets are not emitted** (a block or sideset with no cell in this mesh, which happens with
   several meshes). Deck records are emitted only for boundaries that have faces in this mesh.
8. **An undeclared element type is not checked.** Only a declared type is compared with the cell
   shapes; mixed-shape blocks with no declaration export fine.
9. **`ExportReport` gained `sets: Vec<SetSummary>`** (it is `#[non_exhaustive]`; the skeleton fields
   are unchanged). `FemError` gained variants (`AmbiguousMesh`, `StaleMesh`, `ElementMismatch`,
   `Unsupported`, `Mesh`) and `code()`.
10. **Dependencies:** added `thiserror`, which the common rule (typed errors) requires. `rayon`
    is **not** on the T8 Deps line and is not a dependency of the library: the first version used
    it for one parallel `position_first` in the element-shape check, which was removed in the
    review fix (a sequential scan costs microseconds even at 1e6 cells). It remains a
    **dev-dependency** only, because the determinism tests build explicit thread pools. Also
    dev-dep `serde_json`. The earlier wording that `rayon` was "on the allowed list" was wrong.
    `Cargo.lock` changed outside the crate by `sm-fem`'s dependency lines only (cargo does this
    automatically; the root `Cargo.toml` is untouched); the orchestrator should take it in the
    merge.
11. The plan's wording for the stub volume scheme in some places is `mapvol`; the stub `mapvol`
    cannot generate. Tests use `sweep` with automatic source/target discovery.

## Known gaps

- No face (shell) blocks, no higher-order element types, no overlapping blocks (all by W2 rulings).
- Sidesets cannot be defined on overlay `PartitionFace` keys through the ops (the ops accept
  `TopoFace` only). The lowering resolves colors inherited from hidden host faces/volumes through
  `live_face_color` and `live_cover`, but this is untested end to end (see fix0 above).
- A block color created by another plugin lowers as a block too; it has no declared element type.
- Lowering clones the mesh when it adds sets (O(mesh) per export); a 10^6-cell timing was not
  measured. The identity path shares the stored mesh.
- Set names come from color and boundary names; a name clash with a set the mesh already has is a
  `mesh` error, not a rename.

## api_change_requests

1. **crate `sm-model-api`:** `Model::uncolor_volumes(&mut self, vols: &[EntityId])` (remove the
   volume color of each volume; ignore uncolored ones). *Why:* `fem.block_remove` cannot delete a
   non-empty block today, and nothing can return a volume to "no block". *Signature:*
   `pub fn uncolor_volumes(&mut self, vols: &[EntityId])`, mirroring `uncolor_faces`.
2. **(optional) crate `sm-op-api`:** an `OpError::Unsupported(String)` variant with code
   `unsupported`, so higher-order element types and shell blocks are distinguishable from invalid
   parameters. Today they are `OpError::Invalid` with "unsupported" in the message.
