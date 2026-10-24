# W2 report: sm-mesher (T7)

Branch: `claude/w2-sm-mesher-w8ijnt` (the session instruction named this branch; the task text
named `claude/w2-sm-mesher`. The session branch was used).

## Summary

`sm-mesher` implements the intent layer, the `mesh.*` ops, the `mesh.info` query, the
`MesherConfig` service, `RayonExec` and the mesh-driver node (`mesh.run`, `mesh.delete`) over
`sm-layer-api` (`insert_mesh`, `retire_mesh`, `MeshLayer`, `ModelLayer`) and the `sm-scheme-api`
`Driver`. It depends on no scheme crate and no sizing crate. All skeleton entry points
(`MeshPlugin`, `MesherConfig` and its builders) keep their signatures.

Additions beyond the plan text (both small, both justified; **orchestrator ruling needed on
`mesh.default_size`**: it enters the public catalog (sm-lang, MCP, protocol), so keep it or drop
it until W3):
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
| later `mesh.size` replaces a soft count; cell budget; stale-mesh delete; user `fallbacks`; infeasible request on another volume | `ops::a_later_size_replaces_*`, `an_absurd_size_is_refused_*`, `a_stale_mesh_can_be_deleted_*`, `user_fallbacks_are_tried_*`, `an_infeasible_hard_request_on_another_volume_*` |
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
* **`mesh.run` neighbour guard** (not in the plan): a volume that shares a face with a meshed
  volume outside `targets` is refused (conformity). Previously listed only under Known gaps.
* **Cell budget** (not in the plan): `mesh.run` refuses a plan estimated above
  `MesherConfig::max_cells` (default 20 M cells) with `OpError::Invalid`; the alternative is a
  driver cell-budget ACR. See DESIGN.md.
* **Firmness comparison**: `mesh.size` without explicit firmness is stored as `default` but
  compared at `soft`, so it replaces an earlier soft `mesh.intervals` (Cubit: most recent wins
  at the same level). `hard` is still kept.
* **Bench** uses the stub schemes, so its generate numbers are those of reference code.

## Benchmark (100^3 = 10^6 hexes, stub schemes, 4-core VM, release; full text in DESIGN.md)

| Phase | Time | Cells/s |
|---|---|---|
| plan | 0.11 ms | n/a |
| generate (rayon) | 1.62 s | 0.62 M |
| commit into `MeshBuilder` | 2.74 s | 0.37 M |
| plan + generate + commit (`Driver::mesh`) | 4.72 s | 0.21 M |
| `mesh.run` + `mesh.delete` op pair | 10.6 s | 0.094 M |

Plan section 5 risk 3 is **partly triggered**: the op layer adds about 6 s on top of the driver
(adoption/retirement of 2.4 M ids, quantized digest). No ACR was raised. **Orchestrator decision
needed**: a bulk commit ACR (`MeshBuild::add_template_cells`, would cut the 2.7 s commit) and/or
bulk adopt/retire in `sm-layer-api`, or a profile task before W3. Not blocking W2.

## fix1 (review round 2)

* **Cell budget estimate** now per volume: cube of the geometric mean of the interval counts of
  that volume's edges (exact for a hex brick, `a*b*c`). Slender 1x1x1000 at size 1 gives 1000
  cells, 1x1x100 at 0.1 gives 100000, a 100x100x1 plate at 1 gives 10000, all under the default
  budget; the absurd-size test is kept. Test: `elongated_bricks_are_not_refused_by_the_budget`.
* **Subnormal sizes** are rejected by `mesh.size` and `mesh.default_size` (`is_normal()`), test
  `a_subnormal_size_is_rejected_when_set`.
* **Injected size-field test** now uses a fresh model with no explicit `mesh.size`, so the field
  alone decides the 64 cells.
* **Firmness docs** fixed (code unchanged): any `default`-firmness `mesh.size`, implicit or an
  explicit `"default"`, is compared at `soft` (Cubit: any interval command raises a curve to at
  least soft). Comments, rustdoc and DESIGN.md agree.
* **mesh.run plans twice** (`Driver::plan` for the budget, then `Driver::mesh`). The Driver API
  has no way to reuse a plan; cost is 0.11 ms with the reference solver but would double
  planning with the production IIA solver on large models. **W3 item**: `Driver::mesh_from_plan`
  or a driver-side cell budget (ACR). The `if let Ok(plan)` hides plan errors only until the
  second call, which reports them.
* **Neighbour guard** is a deviation from the plan and blocks Cubit-style sequential meshing of
  adjacent volumes. Acceptable for W2 single-brick milestones. **Orchestrator ruling needed**,
  and **W3 item**: conformal meshing against an existing neighbour mesh.
* **Still open for the orchestrator**: whether `mesh.default_size` enters the public catalog, and
  the bench finding that plan risk 3 is partly triggered (bulk commit / bulk adopt-retire ACR).

## Known gaps

See DESIGN.md "Known gaps": no `on_succession` for the intent layer, fallbacks without
parameters, whole volumes only, neighbour guard by face adjacency only, no partial success.

## api_change_requests

None.

## Definition of done

* `cargo build --workspace`: ok
* `cargo test -p sm-mesher --all-features`: 4 unit + 24 integration pass; also with `RAYON_NUM_THREADS=1`
* `cargo clippy -p sm-mesher --all-targets --all-features -- -D warnings`: clean
* `RUSTDOCFLAGS="-D warnings" cargo doc -p sm-mesher --no-deps`: clean
* sm-quality dev-dependency dropped (unused); serde_json moved to dev-dependencies.
* criterion bench run at 100^3 (4-core VM): see DESIGN.md. Stub-scheme caveat applies.
