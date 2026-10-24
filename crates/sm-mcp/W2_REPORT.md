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
6. Branch pushed to both `claude/w2-sm-mcp` and the session branch `claude/sm-mcp-server-ofwqwm`.

## Known gaps

- No `notifications/cancelled`/progress handling (synchronous calls).
- No query `at` revision through MCP; no `session.save` / `session.open` tools (not in the T11 list).
- Input arguments of op tools are not JSON-Schema validated here; the endpoint validates them
  (its error comes back as `isError`, or `-32602` if it answers `invalid-params`). How
  `sm-server` reports bad op params is not fixed by the contract; both mappings are covered.
- Real `smesh mcp` end to end is the integration phase (T12); only the mock endpoint is used here.

## api_change_requests

None.
