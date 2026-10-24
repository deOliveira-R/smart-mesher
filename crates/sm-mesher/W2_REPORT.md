# W2 report: sm-mesher (T7)

Branch: `claude/w2-sm-mesher-w8ijnt` (the session instruction named this branch; the task text
named `claude/w2-sm-mesher`. The session branch was used).

## Summary

`sm-mesher` implements the intent layer, the `mesh.*` ops, the `mesh.info` query, the
`MesherConfig` service, `RayonExec` and the mesh-driver node (`mesh.run`, `mesh.delete`) over
`sm-layer-api` (`insert_mesh`, `retire_mesh`, `MeshLayer`, `ModelLayer`) and the `sm-scheme-api`
`Driver`. It depends on no scheme crate and no sizing crate. All skeleton entry points
(`MeshPlugin`, `MesherConfig` and its builders) keep their signatures.

Additions beyond the plan text (both small, both justified):
* `mesh.default_size {size | function, params}`: the plan lists the `with_size_fields` hook but
  no op that could ever produce a `SizingSpec::Function`; this op sets `defaults.sizing`.
* `FirmnessArg` (`"default" | "soft" | "hard"`), the op-level spelling of firmness, and the
  firmness ladder with a `mesh.firmness_kept` warning (Cubit semantics).

## Acceptance bullet -> test

| Acceptance bullet | Test |
|---|---|
| stub `sweep`, brick 10, size 1 -> 1000 cells, 1331 vertices; `count_of`; `Delta.created` = adopted ranges; `outputs.mesh = [Mesh#1]` | `ops::stub_sweep_brick_10_size_1_gives_1000_cells_and_1331_vertices` |
| failing scheme -> re-plan/fallback recorded in `plan.attempts` | `ops::failing_scheme_replans_and_is_recorded_in_the_plan_attempts` |
| atomic failure on hard infeasible intent | `ops::infeasible_hard_intent_fails_atomically` |
| `mesh.delete` restores live counts | `ops::mesh_delete_restores_the_live_counts` |
| intent digest order-independent; repeated `mesh.size` replaces | `ops::intent_is_independent_of_op_order_and_replaces_repeats`, `intent_layer::tests::digest_depends_on_content_only` |
| undo/redo and cold replay, no `Diverged` | `ops::undo_redo_and_cold_replay_do_not_diverge` |
| 1 thread vs N threads, 20^3 brick (state hash and exact mesh hash) | `ops::state_and_mesh_are_bit_identical_across_threads_for_a_20_cubed_brick` (also run with `RAYON_NUM_THREADS=1`) |
| refused on already meshed volume | `ops::an_already_meshed_volume_is_refused`, `a_neighbour_of_a_meshed_volume_is_refused` |
| `mesh.info` | `ops::mesh_info_reports_counts_hashes_and_volume`, `the_mesh_is_a_valid_unit_lattice` |
| auto size / size-field hooks, `mesh.default_size` | `ops::auto_size_and_size_fields_come_from_the_injected_hooks`, `default_size_applies_where_no_request_reaches` |
| OUTPUTS and `x-entity-kind` declared | `ops::registered_schemas_carry_entity_kinds_and_declared_outputs` |
| intent pruned on retirement | `intent_layer::tests::*`, `ops::intent_requests_on_deleted_entities_are_dropped` |
| input validation, firmness ladder, scoping, missing service | `ops::bad_intent_requests_*`, `firmness_ladder_*`, `the_intent_of_other_volumes_*`, `a_missing_service_*` |
| Benchmark 100^3 | `benches/commit.rs`; numbers in DESIGN.md |

## Deviations

* **mesh.size default firmness** is `default` (a real-valued goal), not Cubit's "soft" count; see
  DESIGN.md. `firmness: "soft"` gives the Cubit reading.
* **Plan acceptance wording "recorded in `MeshEntry.plan.attempts`"**: the driver records the
  failure that caused a re-plan in `plan.rejections` and the passed-over candidate in
  `plan.attempts`; the stored plan is unmodified (so it still equals a replay of
  `plan_with(rejections)`). Tests assert both.
* **Stub volume scheme**: plan says `"sweep"` with `source/target` params; that is what the tests
  use. (`"mapvol"` of the stubs only plans.)
* **Bench** uses the stub schemes, so its generate numbers are those of reference code.

## Known gaps

See DESIGN.md "Known gaps": no `on_succession` for the intent layer, fallbacks without
parameters, whole volumes only, neighbour guard by face adjacency only, no partial success.

## api_change_requests

None.

## Definition of done

* `cargo build --workspace`: ok
* `cargo test -p sm-mesher --all-features`: 4 unit + 19 integration pass; also with `RAYON_NUM_THREADS=1`
* `cargo clippy -p sm-mesher --all-targets --all-features -- -D warnings`: clean
* `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-mesher --no-deps`: clean
* criterion bench run at 100^3 (4-core VM): see DESIGN.md. Stub-scheme caveat applies.
