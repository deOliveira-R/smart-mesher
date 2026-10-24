# sm-server

## Positioning

`sm-server` is **similar to** a language server or a Jupyter kernel in being a long-lived process that
holds one session and answers JSON requests from any front end (CLI, MCP, a browser, Python), and
**similar to** Cubit's journal-driven `cubit.cmd()` bridge in that a text command and a structured
call are two spellings of the same operation. It **differs in** having no business logic and no
mutable state of its own: the whole session is a `sm_engine::Workspace` (a content-addressed op
timeline), every request is dispatched by one single-writer `match`, and the three ways of driving
it (an `apply` request, a script, an MCP tool call) all end in `Workspace::apply_json`, which
re-encodes params canonically. **Giving** the milestone guarantee: the same ops give the same node
hashes and the same state hash whichever front end sent them, and a failed request leaves the
timeline exactly as it was.

## Design

| Piece | Where | Rule |
|---|---|---|
| `Server` | `server.rs` | `sm_protocol::Endpoint`. `call(&mut self)` makes requests run one at a time in arrival order; a project has one `Workspace`, hence one writer. |
| `SessionHost` | `host.rs` | `sm_lang::Host` over `&mut Workspace`, plus the session operations (`export`, `save`, `hash_at`, `status_result`, timeline control) that `Server` also calls, so each exists once. |
| transports | `transport.rs` | Move text to `Endpoint::handle_text` and back. No dispatch, no parsing beyond line splitting. |
| `standard()` | `standard.rs` | Plugin and service assembly. `try_standard()` is the fallible form. |

### Request mapping

| Request | Implementation |
|---|---|
| `catalog` | registry catalog, query catalog and batch schema, as owned mirrors (`OpInfoOwned`, `QueryInfoOwned`) |
| `apply` | `Workspace::apply_json(op, params, version)`; result `ApplyResult{node, state, delta}` |
| `query` | `Workspace::query` / `query_at`; an omitted (`null`) input is read as `{}` |
| `script.run` | `sm_lang::Interpreter` over a `SessionHost` (see below) |
| `timeline.branch` | `Workspace::fork(name, at)`: creates the ref **and** attaches HEAD (workspace.rs `fork_with`); there is no separate checkout |
| `timeline.checkout` / `undo` / `redo` / `log` / `status` | the same-named workspace calls; `status` is a `StatusResult` built from `Status` |
| `session.hash` | state hash and layer hashes at the revision (`state_at` for a revision, the live state otherwise) |
| `session.export` | `SessionHost::export` -> `sm_fem::export` into a buffer, then an atomic file write (below) |
| `session.save` | `Workspace::save` (needs a project: `no-project` otherwise) |
| `session.open` | rebuilds the distribution from the factory given to `Server::from_factory` / `with_opener`, then swaps the workspace |

### Scripts

`script.run` builds an `Interpreter` with `Options::default()` over a `SessionHost` that has the
request's `includes` map. The reply is a `ScriptResult`:

* `output`: the text of `print` (and `help`) statements, one line each.
* `events`: one object per executed statement, `{event: op|batch|query|timeline|print|help|other, at: "file:line:col", ...}`.
  `op` events carry `op`, `version`, `params` (the canonical record) and `delta`; `query` events carry
  `name`, `params`, `result`. Events a newer sm-lang adds appear as `other` with a `debug` rendering,
  so nothing is lost.
* `diagnostics`: `Diagnostic::to_json` for every warning.

A script with an **error diagnostic** (syntax, type, op failure, assert) answers an application error
with `kind: "script-error"`; `error.data` holds the same object as a `ScriptResult` plus `rendered`
(rustc-style text) and `error_code` (the first error's diagnostic code). Ops applied before the failing
statement stay applied: each is already a timeline node, and `data.state`/`data.events` say exactly
what happened. A script that fails to *parse* applies nothing.

### Export

* `mesh` absent: the only mesh is exported; none is `no-mesh`, several is `ambiguous-mesh`
  (`data.meshes` lists them). The server decides this itself so the error contract does not depend on
  the lowering crate's variants.
* **Path rule:** an absolute path is used as is; a relative path resolves against the project root
  when a project is attached, otherwise against the server process's working directory.
* The format is inferred from `.smsh` / `.vtu` when `format` is absent; anything else is `bad-request`.
  Inline-only export (`path` absent, `inline: true`) needs an explicit format.
* Bytes are built in memory (needed for the SHA-256 and the optional base64), then written to
  `<name>.part` and renamed, so a reader never sees a half-written file. A failed export writes nothing.
* `provenance: true` records the current node hash (empty state: no provenance).
* The bytes are a pure function of the state and the request; `ExportResult.sha256` is `Hash256::of(bytes)`.

### Errors

Every failure is `RpcError{code, kind, message, data}` with `kind` = the contract `code()` string
(`EngineError::code`, which forwards `TimelineError`, `RegistryError`, `OpError` codes), or one of the
server's own codes: `script-error`, `no-mesh`, `ambiguous-mesh`, `unknown-mesh`, `bad-request`,
`open-unsupported`, `workspace-not-empty`, `export-failed`, `io`. `stopped-on-warning` carries the
diagnostics in `data`; `not-persisted` carries the recorded node. Protocol-level errors
(`parse-error`, `invalid-request`, `method-not-found`, `invalid-params`) come from `sm-protocol`.

### Transports

* `InProcess<E>`: text in, text out, or typed `Request` through the full serialize/parse/dispatch path.
* `serve_lines` / `serve_stdio`: JSON-lines. One reply line per non-blank request line, flushed per
  reply. A request without `id` (a notification, unsupported by this protocol) gets an `INVALID_REQUEST`
  reply with `id: null`; invalid JSON and invalid UTF-8 get `PARSE_ERROR`; a line over `MAX_LINE_BYTES`
  (64 MiB) is discarded and answered with `INVALID_REQUEST`. Nothing is dropped silently.
* `serve_ws` (feature `ws`, tungstenite): one text frame per message. Connections are served one at a
  time, which keeps the single-writer rule without a lock; binary frames get `INVALID_REQUEST`.

### The standard distribution

`standard()` = `CoreLayers` over `sm_topo::TopoStore`, `ModelPlugin`, `MeshPlugin`, `FemPlugin`,
`TessPlugin`, and the `mesher` service (`MesherConfig` with `IiaSolver`, `AnglePicker`, `size_field`,
`auto_size`) over a scheme registry built from the three scheme crates with the strict auto lists
`curve: [equal]`, `surface: [map]`, `volume: [map]`. Scheme `register` errors are never discarded and
a missing auto scheme is `StandardError::MissingScheme`; `standard()` panics where `try_standard()`
errs. `distribution(schemes)` is the same wiring around any scheme registry (tests pass the stubs).

### Tests

* `tests/session.rs`: script vs `apply` equality (state hash, node sequence, layer hashes), timeline
  mapping, `session.hash` vs `Workspace::state_at`, error codes, export (path rule, formats, inline,
  provenance, mesh selection), `session.open`, includes (and their sandbox), `Host::session`.
* `tests/transport.rs`: JSON-lines over a `Cursor` and over an OS pipe, null-id replies, malformed
  lines, oversized lines, WebSocket.
* `tests/determinism.rs`: the same request log gives byte-identical replies under 1 and N rayon threads
  and concurrent runs (`sm_testkit::determinism::check`, pinned clock and author); cold replay from the
  root with the strict policy gives no `Diverged`.
* `tests/registry.rs`: the registry-level conventions (plan §5 risk 6): every op declares `OUTPUTS` (or
  is in the explicit `NO_OUTPUT_OPS` list) and every `EntityId` property of an op or query carries a
  valid `x-entity-kind`. The checker is tested on synthetic schemas, then run on the demo set, on the
  standard plugins over the stub schemes, and on `standard()` itself.

### Known limits

* Exports are built in memory; a 10^7-cell mesh needs the whole file in RAM. Streaming to the file with
  an incremental hash is the fix if that matters.
* Scripts run with default interpreter limits; `Options` is not configurable per request yet.
* `session.*` script statements depend on sm-lang T9; the host side is implemented and tested through
  `Host::session`.
