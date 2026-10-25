# W2 report: sm-server (T10)

Status: **done** against the pre-merge skeletons. `standard()` wiring is verified only at integration
(see "Known gaps").

## Summary

* `SessionHost<'w>`: `sm_lang::Host` over `&mut Workspace` (apply, query, timeline control, `timeline_head`,
  `session(..)` export/save/hash/status, `load` from the include map then the project directory, relations
  from `topo.related` when the registry has it).
* `Server: sm_protocol::Endpoint`: single-writer dispatcher for every `Request` variant (catalog, apply,
  query, script.run, timeline.*, session.hash/export/save/open). Constructors: `new` (in-memory, skeleton
  signature kept), `from_factory`, `from_workspace`, `open`, `create`, `with_opener`.
* Transports: `InProcess`, `serve_lines` / `serve_stdio` (JSON-lines), `serve_ws` / `serve_ws_listener`
  (feature `ws`). Id-less requests get an `INVALID_REQUEST` reply with `id: null`.
* `standard()` / `try_standard()` / `standard_schemes()` / `distribution(schemes)`; scheme `register` errors
  and missing auto schemes are errors, never discarded.
* Registry-level conventions test (plan §5 risk 6) with a tested checker.
* `DESIGN.md` has the positioning paragraph, the request mapping, the script, export and error contracts.

## Acceptance bullet -> test

| Acceptance bullet | Test (`tests/…`) |
|---|---|
| `script.run` gives the same state hash and node sequence as the equivalent `apply` calls | `session.rs::script_run_equals_the_equivalent_apply_calls`, `batch_through_apply_and_script_agree`, `script_events_carry_op_params_and_delta` |
| Every timeline request maps correctly; fork at an earlier node then checkout back is bit-identical | `session.rs::fork_then_checkout_back_is_bit_identical` (branch at a node, checkout ref/node/empty, undo, redo, log, status), `script_timeline_commands_map_to_the_workspace` |
| `session.hash` at a rev equals `Workspace::state_at(rev).hash()` | `session.rs::session_hash_at_rev_equals_state_at` (state and per-layer hashes) |
| Errors carry the contract `code()` strings | `session.rs::errors_carry_contract_code_strings` (`unknown-op`, `decode`, `unknown-entity`, `unknown-query`, `unknown-ref`, `empty-position`, `no-project`, `no-mesh`, `method-not-found`), `failing_script_reports_script_error_and_keeps_applied_ops`, `session_open_…` |
| A stdio transport round trip runs over an in-memory pipe | `transport.rs::stdio_round_trip_over_an_in_memory_pipe`, `lines_round_trip_over_a_real_os_pipe` |
| `standard()` builds and its catalog lists the skeleton ops | `registry.rs::the_standard_plugin_set_over_stub_schemes_follows_the_conventions` (catalog lists `core.batch`), `standard_follows_the_conventions_once_the_scheme_crates_are_merged`, `standard_panics_exactly_when_try_standard_fails`; **ignored** `standard_lists_the_milestone_ops` for integration |
| Determinism: same request log, same responses, pinned clock and author | `determinism.rs::same_request_log_same_responses_for_1_and_n_threads` (`sm_testkit::determinism::check`, 1 vs 4 rayon threads, concurrent runs), `two_servers_agree_on_every_node_and_state`; also run under `RAYON_NUM_THREADS=1` |
| Cold replay from the root, no `Diverged` | `determinism.rs::cold_replay_from_the_root_does_not_diverge` |
| Path rule for `session.export` | `session.rs::exports_smsh_and_vtu_with_the_path_rule` (project root), `memory_export_resolves_against_the_working_directory` (cwd, absolute) |
| Prompt: id-less request -> `INVALID_REQUEST`, `id: null` | `transport.rs::a_request_without_id_gets_an_invalid_request_reply_with_null_id`, `malformed_lines_get_typed_error_replies_and_the_stream_continues`, `websocket_round_trip` |
| Prompt: every op in `standard()` declares `OUTPUTS`, every `EntityId` property has `x-entity-kind` | `registry.rs` (checker proven by `the_checker_flags_exactly_the_unannotated_id_properties`; run on the demo set, on the standard plugins with stub schemes, and on `standard()`) |
| Export with no mesh named: one mesh, `no-mesh`, `ambiguous-mesh` (decision 8) | `session.rs::export_picks_the_only_mesh_and_rejects_ambiguity`, `errors_carry_contract_code_strings` |
| `Host::session` commands | `session.rs::session_commands_on_the_host`, `save_without_a_project_is_a_typed_host_error` |
| `load` from include map then project dir | `session.rs::includes_come_from_the_request_then_the_project_and_never_escape_it` |

Definition of done run locally: `cargo build --workspace`; `cargo test -p sm-server` (with and without
`--all-features`, and with `RAYON_NUM_THREADS=1`); `cargo clippy -p sm-server --all-targets [--all-features] -- -D warnings`;
`RUSTDOCFLAGS="-D warnings" cargo doc -p sm-server --no-deps`. All clean.

## Deviations (with reasons)

1. **`script.run` failure is an error reply, not an `Ok(ScriptResult)`.** `kind: "script-error"`, with the
   `ScriptResult` shape (plus `rendered`, `error_code`) in `error.data`. Reason: `ScriptResult` documents
   `diagnostics` as "warnings and notes", and CLI exit code 1 / MCP `isError` want a failure signal.
   sm-mcp and sm-cli should read `error.data` for the events of a failed script. Ops applied before the
   failing statement stay applied (one timeline node per statement; a `batch { }` block is one `core.batch` node).
2. ~~Export mesh selection is done in sm-server.~~ Superseded by the polish wave: `sm_fem::export` selects,
   the server only maps its errors (see below).
3. **Branch used.** The task text says `claude/w2-sm-server`; the session's designated branch is
   `claude/w2-sm-server-syo1is`. The same commit is on both.
4. **`Cargo.lock` is committed** (new deps: tungstenite and its tree for the optional `ws` feature,
   `thiserror`/`serde` already present). Expect a trivial lockfile merge with other workers.
5. **New dependencies**, all from the allowed list or well established: `serde`, `thiserror`, optional
   `tungstenite 0.30` (the `ws` feature the plan names); dev: `serde`, `schemars`, `rayon`, `sm-testkit`,
   `sm-mesh-api`, `sm-meshio`, `sm-scheme-api[stubs]`. Base64 is a 20-line local encoder (RFC 4648 vectors
   tested) instead of a crate; SHA-256 is `Hash256::of`.
6. `SessionHost`'s adjacency is a small adapter that owns an `Engine` clone and delegates to
   `sm_lang::QueryRelations` per call. `QueryRelations<'a>` borrows `&Engine`, which cannot live next to a
   `&mut Workspace` without a self-reference.
7. A `null` query input is read as `{}` (the protocol's `Request::Query.input` defaults to `null`).

## Known gaps

* **`standard()` panics today on main.** It is strict (as plan §3 requires), and the skeleton scheme crates
  register nothing, so the auto lists cannot be set. Merge the three scheme crates before anything that calls
  `standard()` (sm-cli's milestone, ignored today). `try_standard()` returns `MissingScheme` instead.
  The tests that touch `standard()` skip (printing why) on `MissingScheme` and become live automatically;
  `registry.rs::standard_lists_the_milestone_ops` is `#[ignore]`d for the integration agent.
* The registry check runs on the demo ops, the standard plugins over the stub schemes and `standard()`
  itself. Ops that write no ids set `TypedOp::NO_OUTPUTS`; every other op must declare `OUTPUTS`
  (`tests/registry.rs` derives both from `OpInfo`, and registration refuses `NO_OUTPUTS` together with
  non-empty `OUTPUTS`).
* `session.*` script statements (`session.export(...)`, `session.hash()`) are parsed by sm-lang T9 and run
  end to end (`session_statements_are_reported_as_session_events`). `SessionCommand::Hash` returns a hex string and `Status` an
  object without null fields (the language has no null).
* No bench: the task names no hot path. Exports are built fully in memory (see DESIGN.md "Known limits").
* The stub-scheme meshing path (`Driver::mesh` through `mesh.run`) is not exercised here: `MeshPlugin` is a
  skeleton, so mesh states in tests come from a demo op that inserts a reference hex block via
  `sm_layer_api::insert_mesh`.

## api_change_requests

None.

## Polish wave

| # | Item | Status |
|---|------|--------|
| 1 | `script_timeline_commands_map_to_the_workspace` could not fail | fixed: sm-lang T9 parses `session.*`, so the first block now requires an Ok run with no diagnostics and checks head and branches (`script_timeline_commands_map_to_the_workspace`) |
| 2 | format vs extension mismatch writes the wrong bytes | fixed: explicit `format` disagreeing with a `.smsh`/`.vtu` extension is `bad-request`; an explicit format on another extension still works (`export_rejects_a_format_that_disagrees_with_the_extension`, protocol and command-language paths) |
| 3 | `provenance` silently dropped for VTU; `Some("")` on empty position | fixed: VTU + provenance is `bad-request` (`provenance_is_rejected_for_vtu_which_cannot_record_it`); empty position gives no provenance (`host::tests::provenance_is_the_node_hash_and_absent_in_the_empty_state`; unreachable through export today, since an empty state has no mesh). Determinism log request 5 changed to smsh + provenance and request 13 (vtu inline) added; the log is test input, no golden changed |
| 4 | named mesh without `meshes` layer is `no-mesh`; `NoMeshLayer` is `export-failed` | fixed: `unknown-mesh` for a named id, `no-mesh` otherwise (`a_named_mesh_in_a_state_without_meshes_is_unknown_not_missing`) |
| 5 | `to_json` turns a serialization failure into `null` | fixed: failure is `SessionError::internal`, which becomes `RpcError::internal` (JSON-RPC -32603, kind `internal`) (`host::tests::a_serialization_failure_is_an_internal_error_not_a_null_result`) |
| 6 | per-request `query_names()` Vec; `other` event without `at` | fixed: hash/export/save use `SessionHost::plain` (no registry scan); `other` carries `at: ""`. Cosmetic, no behaviour test |
| 7 | `serde`/`thiserror` beyond T10's Deps line | not done (needs orchestrator): accept the two deps or amend the plan; both are required (CLAUDE.md style, generic `Serialize` bounds) |
| 8 | mesh selection reimplemented; hard-coded `FemError` codes | fixed: selection delegated to `sm_fem::export`; `From<&FemError>` uses `FemError::code()`; `AmbiguousMesh` keeps `data.meshes` (`export_picks_the_only_mesh_and_rejects_ambiguity`). **Visible code changes:** failures that were `export-failed` now carry the FemError code (`stale-mesh`, `element-mismatch`, `unsupported`, `mesh`, `meshio`); `io` and `no-mesh`/`unknown-mesh`/`ambiguous-mesh` are unchanged |
| 9 | failed `script.run` is an error reply | not done (needs orchestrator): record as a cross-crate decision and tell sm-mcp/sm-cli; already documented on `Request::ScriptRun` and pinned by `failing_script_reports_script_error_and_keeps_applied_ops` |
| 10 | `standard()` skip branches | fixed: scheme crates are merged, `standard_lists_the_milestone_ops` is live; the `MissingScheme` skip is removed so the check fails hard |
| 11 | `NO_OUTPUT_OPS` guess list | obsolete: the list is gone; the checker reads `NO_OUTPUTS` from `OpInfo`, and `standard_follows_the_conventions…` passes against the real ops |
| 12 | `event_json` Debug fallback | fixed: `Event::Session` is mapped explicitly (`session_statements_are_reported_as_session_events`); the unknown-event fallback no longer emits Debug text (`{"event":"other","at":""}`) |
| 13 | no test for the NO_OUTPUTS + OUTPUTS refusal | fixed: `registering_no_outputs_with_declared_outputs_is_refused` |
| 14 | garbled W2_REPORT bullet | fixed: rewritten (Known gaps) |
| 15 | `ScriptRun` doc wording; `diagnostics[0]` indexing | server comment and report fixed; the test now picks the first *error* diagnostic. The doc on `Request::ScriptRun` is in sm-protocol: not done (needs orchestrator) |
| 16 | same garbled bullet | fixed with 14 |
| 17 | protocol doc describes one implementation | not done (needs orchestrator; sm-protocol) |
| 18 | downstream behaviour notes | nothing to do here |
| – | earlier contract requests | confirmed resolved on main; `NO_OUTPUTS` adopted in the registry test, `standard_with` in `reference_core.rs` |

### Not done (needs orchestrator)

* 7: accept `serde` and `thiserror` in `[dependencies]` or amend the plan's Deps line.
* 9: record the `script-error` failure form as a decision and inform sm-mcp and sm-cli.
* 15, 17: `Request::ScriptRun` rustdoc (sm-protocol): say "one `core.batch` node per `batch { }` block", and consider stating the failure form as the contract rather than "as sm-server answers it".

### Changed goldens

None. No state hash, layer hash or export byte changed (export bytes are unchanged for every accepted request).
