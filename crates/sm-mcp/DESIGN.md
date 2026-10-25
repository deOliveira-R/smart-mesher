# sm-mcp: MCP server over the smart-mesh protocol

**Positioning.** Similar to the MCP servers that wrap a CAD/meshing application (Cubit or Gmsh
behind a hand-written tool list), `sm-mcp` differs in that it has no tool list of its own: every
tool is generated from the live op/query catalog of a `dyn Endpoint`, and the only code that is
smart-mesh specific is a handful of fixed tools that mirror protocol methods. That gives an
assistant the exact same ops, schemas, entity-kind annotations and declared output roles that
scripts and the CLI see, with nothing to keep in sync, and gives us one conformance surface
(the protocol) instead of one per front end.

## Layering

`sm-mcp` (L6) depends on `sm-protocol` and `sm-op-api` (catalog types) and `serde_json`. It never
names `sm-server` or `sm-engine`: `run_stdio(&mut dyn Endpoint, BufRead, Write)` is the whole
public surface (plus `McpVersion`). The `respell` feature adds `sm-lang`.

## Wire behavior

- Framing: one compact JSON object per line, flushed per reply; lines are read as bytes so bad
  UTF-8 is a `-32700` reply, not a dead server. Blank lines are skipped, `\r\n` is accepted, EOF
  is a clean shutdown.
- Requests: `id` must be a string or number (MCP forbids `null`). No `id` means notification:
  never answered, whatever the method (`notifications/initialized`, `notifications/cancelled`,
  ...). Responses from the client are ignored. Batch arrays get `-32600` (batching was removed in
  2025-06-18). Unparseable or structurally invalid input is answered with `id: null`, except a request whose `id` is a valid string/number: its -32600 replies (missing or non-string `method`, bad `jsonrpc`) echo the id.
- `initialize`: a requested version in {2025-06-18, 2025-03-26, 2024-11-05} is echoed, anything
  else is answered with 2025-06-18 (the client decides whether to disconnect). The negotiated
  version gates fields: `title` and `structuredContent` need 2025-06-18, tool `annotations` need
  2025-03-26. Requests before `initialize` are served with the latest version (lenient; only
  `ping` is mandatory there).
- `tools/list`: one page, never a `nextCursor`; any non-empty cursor is `-32602`. Sorted by name,
  keys of each tool object sorted, so the bytes are independent of build features.
- `tools/call`: see below.

## Tools

| Source | Tool name | Maps to |
|---|---|---|
| catalog op | catalog `tool_name` (`mesh__run`) | `Request::Apply { op, params: arguments }` |
| `core.batch` | `core__batch`, schema = `batch_schema` | `Apply { op: "core.batch" }` |
| catalog query | catalog `tool_name` | `Request::Query { name, input: arguments }` |
| fixed | `run_script` `session_hash` `session_export` `session_status` `timeline_branch` `timeline_checkout` `timeline_undo` `timeline_redo` `timeline_log` | `ScriptRun`, `Hash`, `Export`, `Status`, `Branch`, `Checkout`, `Undo`, `Redo`, `Log` |
| fixed | `help` | answered locally from the tool list |

Fixed names have no `__`, op/query names always do (registry rule), so they cannot collide; if a
hostile catalog tries, the fixed tool wins, ops win over queries, and names that are not
`[A-Za-z0-9_-]{1,64}` are dropped, so every list is valid for clients. `inputSchema` is the op's
params schema with `x-entity-kind`/`x-positional` kept and `type: object` forced. The
description is the schema description plus the declared output roles (`delta.outputs`).

Fixed-tool arguments are validated by building a `sm_protocol::Envelope` (strict, unknown fields
rejected) before the endpoint is touched. Conveniences: `rev` and `at` are plain strings (branch
name, 64-hex node, `"empty"`; the object forms pass through), `timeline_log.limit` defaults to 20.

## Results and errors

- Success: `structuredContent` = the protocol result (non-object results are wrapped as
  `{"result": v}` because MCP requires an object), text content = the same JSON, compact.
- `RpcError` with code `-32000` (application error: op failure, script failure, no-mesh, ...) is
  a normal result with `isError: true` and `{"error": {code, kind, message, data?}}`, so the model
  can read it and retry.
- `-32602` from the endpoint stays `-32602` (kind in `data`); every other endpoint protocol
  error is `-32603`. Unknown tool, non-object `arguments`, missing `name`: `-32602`.
- No `outputSchema` is published: the apply result is `ApplyResult`, not the `Delta` schema the
  catalog carries, and a wrong `outputSchema` would be a spec violation.

## State

The loop holds the negotiated version and a memo of the tool list. The catalog is fixed when
an endpoint is built, so the memo is a cache of an immutable property, filled at the first
`tools/list`, `help` or op/query call (fixed tools never need it). No project state is kept.

## Determinism

Output is a pure function of (input lines, endpoint replies): no clocks, no randomness, no hash
iteration; maps are sorted. `tests/golden/session.*.jsonl` pin a full transcript byte for byte.

## Known limits

- `notifications/cancelled` and progress are ignored: calls are synchronous.
- Queries cannot be evaluated `at` another revision through MCP (the query input is passed
  through verbatim); use `run_script` or the protocol directly.
- `respell` (off by default) shows `Face#1` in the text; tool arguments accept both spellings (`TopoFace#1` and the alias `Face#1`).
