# W2 report: sm-mcp (T11)

## Summary

`sm_mcp::run_stdio(&mut dyn Endpoint, BufRead, Write)` (skeleton signature unchanged) is a full
stdio MCP server: `initialize` with version negotiation (2025-06-18, down to 2025-03-26 and
2024-11-05 by omitting newer fields), `ping`, `tools/list`, `tools/call`; notifications are
silent. Tools are generated from `Request::Catalog` (one per op and query via the catalog's
`tool_name`, `core__batch` with the registry batch schema) plus the fixed tools `run_script`,
`session_hash`, `session_export`, `session_status`, `timeline_branch`, `timeline_checkout`,
`timeline_undo`, `timeline_redo`, `timeline_log`, `help`. Results carry `structuredContent` and
compact JSON text; endpoint application errors are `isError` results, bad arguments / unknown
tools / endpoint protocol failures are JSON-RPC errors. Malformed lines get `-32700` with
`id: null` (stage-0b note). Design: `DESIGN.md`.

Dependencies: `sm-protocol`, `sm-op-api`, `serde_json`; optional `sm-lang` (feature `respell`).
Dev: `sm-core`, `serde`, `schemars` (to build a real `Registry` as the mock's catalog).
`Cargo.lock` gained the sm-mcp dependency lines (cargo-generated, no other change).

## Acceptance -> tests (`tests/mcp.rs`)

| Acceptance bullet | Test |
|---|---|
| Scripted session transcript, golden JSON, against the fake endpoint | `golden_session_transcript` (`tests/golden/session.{in,out}.jsonl`, `session.out.respell.jsonl` with the feature; regenerate with `SM_MCP_BLESS=1`) |
| `tools/list` sorted, stable, names <= 64 bytes, `inputSchema.type == object` | `tools_list_sorted_stable_valid`, `tools_cover_catalog_batch_and_fixed`, `hostile_catalog_entries_are_dropped`, `schemas_keep_extensions_and_describe_outputs` |
| Unknown tool / method give the right JSON-RPC error | `unknown_tool_and_method_and_bad_params`, `framing_and_malformed_input`, `invalid_utf8_is_a_parse_error_not_a_crash` |
| Large results stream as one line | `large_results_are_one_line` (>2 MB, exactly one `\n`) |
| Determinism golden | `determinism_same_bytes_across_runs_and_threads` + the golden files (run also under `RAYON_NUM_THREADS=1`; the crate has no threads of its own) |
| Version pin and negotiation | `negotiation_echoes_supported_and_answers_latest_otherwise`, `old_versions_omit_newer_fields` |
| `structuredContent` + text JSON; `isError` vs protocol errors | `op_call_maps_to_apply_and_returns_structured_and_text`, `query_call_maps_to_query_and_wraps_non_objects`, `application_errors_are_iserror_results`, `non_application_endpoint_errors_are_protocol_errors` |
| Fixed tools map to protocol methods, args validated | `fixed_tools_map_to_protocol_requests`, `fixed_tool_arguments_are_validated_before_the_endpoint_is_called`, `fixed_tool_schemas_match_the_protocol_request_fields`, `help_lists_and_describes` |
| `core__batch` | `core_batch_is_applied_as_core_batch` |
| Must NOT keep state beyond the endpoint | `catalog_is_fetched_once_and_fixed_tools_never_need_it` documents the only memo |

## Deviations

1. **`respell` is a cargo feature, off by default.** `sm_lang::respell_json` writes `Face#1`;
   `EntityKind::from_str` accepts only canonical names (`TopoFace`), so a model copying an id from
   the text into a tool call would be rejected. The plan marks the dependency optional; it is
   implemented and tested (`--all-features`) but not on by default.
2. **Non-object results are wrapped** as `{"result": v}` in `structuredContent` (MCP requires an
   object; queries return arrays and numbers, `timeline.log` an array).
3. **`outputSchema` is not published**: the catalog's `output_schema` is `Delta`, but the result is
   `ApplyResult`; publishing it would violate "MUST conform".
4. Tool list is memoized per `run_stdio` call (catalog is immutable per endpoint).
5. `timeline_log.limit` is optional in the tool (default 20) although `Request::Log` requires it.
6. Branch: the task names `claude/w2-sm-mcp`, the session names `claude/sm-mcp-server-ofwqwm`; both
   point at the same final commit.
7. The determinism test does not call `sm_testkit::determinism::check`: sm-testkit is not in T11's
   Deps and this crate has no state hashes to compare, only output bytes, so it compares the
   bytes of repeated runs (including from another thread) plus the pinned golden files.

## Known gaps

- No `notifications/cancelled`/progress handling (synchronous calls).
- No query `at` revision through MCP; no `session.save` / `session.open` tools (not in the T11 list).
- Input arguments of op tools are not JSON-Schema validated here; the endpoint validates them
  (its error comes back as `isError`, or `-32602` if it answers `invalid-params`). How
  `sm-server` reports bad op params is not fixed by the contract; both mappings are covered.
- Real `smesh mcp` end to end is the integration phase (T12); only the mock endpoint is used here.

## api_change_requests

None.

## Polish wave

| # | Item | Status |
|---|---|---|
| 1 | `inputSchema.type == object` forcing branches never ran | fixed: `non_object_input_schemas_are_forced_to_object` (missing `type`, `"string"`, `["object","null"]`, `true`, `false`, for ops and queries). A JSON `null` schema is not reachable: `Schema` (schemars) does not deserialize from `null`, so such a catalog is rejected as unreadable (-32603), which `unknown_tool_with_a_failing_catalog_is_an_internal_error` covers |
| 2 | Missing/non-string `method` with readable id replied `id: null` | fixed: `missing_method_echoes_only_a_valid_id`, `framing_and_malformed_input` (updated: the missing-method case now echoes `1`). A non-string `method` already echoed the id; the code now shares one `id_is_valid` check. Invalid ids (`null`, array, object) still get `null` |
| 3 | Ops-win-over-queries collision untested | fixed: `hostile_catalog_entries_are_dropped` now has a colliding query entry (the op stays, a call goes to `Apply`) |
| 4 | `empty` / 64-hex branch names need the `{"ref":..}` form | fixed: `rev_descriptions_name_the_ref_escape_hatch`, `branches_named_empty_or_hex_need_the_ref_form`. The `rev`/`at` schema is now `anyOf` string / `{ref}` / `{node}` so the documented object forms validate |
| 5 | Golden pins `serverInfo.version` | fixed: the golden test replaces `CARGO_PKG_VERSION` with `<CARGO_PKG_VERSION>` before comparing (and asserts exactly one occurrence). A version bump no longer needs a re-bless |
| 6 | Same as 2 | fixed (see 2) |
| 7 | Unknown tool + failing catalog gives -32603 | documented, not changed: the name cannot be judged unknown without the catalog. `unknown_tool_with_a_failing_catalog_is_an_internal_error` (endpoint error and unreadable payload; fixed tools still work) |
| 8 | Map `invalid-params` op errors to -32602 vs `isError` | not done (needs orchestrator): behaviour decision shared with sm-server (T10). Current: -32602 for `invalid-params` (`non_application_endpoint_errors_are_protocol_errors`), `isError` for application errors |
| 9 | Optional sm-protocol re-exports not adopted by sm-cli / sm-mcp | confirmed: not adopted; sm-mcp still imports from `sm-protocol`/`sm-op-api` directly, which is acceptable |
| 10 | Docs after id-alias ACR | fixed: `Cargo.toml` `respell` comment, `lib.rs` rustdoc, `DESIGN.md`. `respelled_ids_feed_back_as_arguments` already parses with the typed `EntityId` and passes. Engine-backed variant: not done (needs orchestrator: a real-kernel dev-dependency is a Cargo/dependency-graph decision) |
| - | Respell contract request (a)/(b) | resolved on main by option (a) (aliases accepted by `EntityKind::from_str`); `respell` stays off by default so text equals `structuredContent` JSON; flipping the default is an orchestrator call |
| - | Adopt new contract API (`OpError::Unsupported`, `NO_OUTPUTS`, re-exports, ...) | confirmed: nothing in sm-mcp's scope needs it (it passes endpoint results through) |

Changed goldens: `tests/golden/session.out.jsonl` and `session.out.respell.jsonl` (two lines each):
`serverInfo.version` is now the placeholder (item 5), and the `tools/list` line changed because the
`rev`/`at` schema and description changed (item 4). No state hash, layer hash or export byte changed.
Protocol-visible behaviour change: only the `id` of the -32600 reply for a missing `method` (items 2, 6).

api_change_requests: none.
