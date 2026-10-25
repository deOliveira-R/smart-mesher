# sm-cli: the `smesh` binary

**Positioning.** `smesh` is similar to the Cubit/Trelis journal-driven batch mode (`cubit -batch
script.jou`) and to `git`'s verbs over a content-addressed history: one binary, scripts as the primary
input, every command scriptable. It differs in that it is a thin *client of a wire protocol* rather
than a front end linked to the modeling kernel: each subcommand is exactly one `sm_protocol::Request`
against a `dyn Endpoint`, and the same endpoint serves the MCP loop (`smesh mcp`) and JSON-RPC lines
(`smesh serve`). That gives one artifact in which the CLI, MCP and script paths can be checked against
each other (plan §4: equal state hashes), and a CLI with no modeling, meshing or workspace logic of
its own to get wrong.

## Layering

```
main.rs            wires stdin/stdout/stderr and `connect_standard` into `run`
lib.rs  run()      parse (clap) -> Settings -> connect -> [session.open] -> dispatch -> [session.save]
args.rs            the clap grammar (doc comments are the --help text)
render.rs          human output, `--json` output, error rendering
help.rs            `smesh help [op]` over the catalog
```

`run(args, connect, io)` takes the endpoint *factory* and the three streams as parameters, so tests
drive it with a recording fake endpoint, and the binary passes `connect_standard`
(`sm_server::standard()` + `--author` + `--fixed-time`, wrapped in `Server::new`, i.e. in memory).
`-p dir` is `session.open`, not a different constructor, so the CLI never names a `Workspace`.

## Commands

| Command | Request(s) |
|---|---|
| `new <dir>` | `session.open{create}`, `timeline.status`, `session.save` |
| `run <script> [--print-hash]` | `script.run{text, path, includes}` |
| `apply <op> <json\|->` | `apply{op, params, version?}` |
| `query <name> [json\|-] [--at rev]` | `query{name, input, at?}` |
| `log [-n N] [--from hash]` | `timeline.status` + `timeline.log` (one combined JSON object) |
| `status` | `timeline.status` |
| `hash [--at rev] [--layers]` | `session.hash{at?}` |
| `checkout <rev>` / `fork <name> [--at hash]` / `undo` / `redo` | `timeline.checkout` / `.branch` / `.undo` / `.redo` |
| `export <file> [--mesh id] [--format f] [--provenance]` | `session.export` |
| `serve [--stdio]` | loop of `Endpoint::handle_text`, one reply line per request line |
| `serve --ws addr` | not available in this build (exit 1) |
| `mcp` | `sm_mcp::run_stdio` |
| `help [op]` | `catalog` |

Under a project (`-p`, or one discovered by walking up to a `project.toml`) the session is opened
first, and `session.save` follows every command that can change it (`new`, `run`, `apply`,
`checkout`, `fork`, `undo`, `redo`, `serve`, `mcp`). Read-only commands never save. `--memory` forces
an in-memory session; `-p` and `--memory` conflict.

Revisions (`checkout`, `hash --at`, `query --at`): `empty`, a 64-digit lowercase hex node hash, or a
branch name. A branch literally named `empty` or spelled as 64 hex digits cannot be addressed by name.

Export paths are passed to the server unchanged. The rule (plan T10) is: relative paths resolve
against the project root under `-p`, otherwise against the working directory; absolute paths are
used as is. The format comes from `--format` or the `.smsh`/`.vtu` extension; anything else is a usage
error before any request is sent.

`run` scans the script for `include "path"` lines and sends the files (read next to the script,
nested up to the language's depth of 16) in `script.run.includes`, so in-memory runs resolve includes;
a project run also finds them in the project directory. Unreadable includes are left for the server to
report with the language's own diagnostic.

## Output and exit codes

| Code | Meaning |
|---|---|
| 0 | success (including `--help`/`--version`) |
| 1 | an op, script or server error (`RpcError`), a malformed reply, or I/O on a project |
| 2 | usage: bad flags (clap), unreadable script, malformed JSON argument, bad hash/rev/format, bad `--fixed-time`, unknown name for `help` |

Human mode prints results on stdout and errors as `error[kind]: message` (plus the error's `data`
pretty-printed) on stderr. `--json` prints the server's result value verbatim as one compact line;
a failure prints `{"error": {code, kind, message, data?}}` on stdout with the same exit code. Usage
errors found by clap itself are the same object on stdout when `--json` appears among the arguments,
and text on stderr otherwise. The exceptions to "verbatim" are `log --json`, which
is `{head, position, branches, nodes}` because `timeline.log` returns nodes only, `help --json`
(catalog, or one entry plus `"kind": "op"|"query"`) and `query` (pretty JSON in human mode).

`--print-hash` prints the script's output, then the 64-hex state hash as the last stdout line.

## Determinism

The CLI adds no nondeterminism: no map iteration (only `BTreeMap`/sorted output), no time (the clock
is the server's, pinned by `--fixed-time`), no threads. `--fixed-time` and `--author` configure the
builder before the server exists, so node metadata (part of the node hash) is reproducible. Tests
compare the binary's bytes over repeated runs, `RAYON_NUM_THREADS` 1 vs 3 and concurrent runs.

## Testing

- `tests/commands.rs`: every command against a recording fake endpoint (exact requests, stdout,
  exit codes, `--json` shapes parsed back into the typed `sm_protocol` results).
- `tests/binary.rs`: the real binary and `standard()` server for what is independent of the other W2
  crates (usage, catalog/help, hash, transports, error mapping, determinism).
- `tests/cli_full.rs`: full flows on the real distribution (no longer ignored).
- `tests/p0_rows.rs`: P0 capabilities (interval count, auto size, default size, scheme selection, firmness) through the real binary and MCP on `standard()`.
- `tests/data/{milestone,fork}.smesh`: the plan §4 scripts, for the integration phase's `milestone*.rs`.
