//! MCP server over the smart-mesh protocol (T11).
//!
//! [`run_stdio`] speaks the Model Context Protocol (JSON-RPC 2.0, one compact message per line)
//! on any `BufRead`/`Write` pair and forwards every tool call to a `dyn Endpoint` of
//! `sm-protocol`. It never names `sm-server` or `sm-engine`, and keeps no state beyond the
//! negotiated protocol version and a memo of the endpoint's (immutable) tool list.
//!
//! Pinned version `2025-06-18`; clients asking for `2025-03-26` or `2024-11-05` are answered in
//! that version and newer fields (`structuredContent`, `title`, `annotations`) are left out.
//!
//! Tools:
//! - one per op and query of the catalog, named by the catalog's `tool_name` (`mesh__run`);
//!   arguments are the op params or query input, passed through unchanged;
//! - `core__batch`, whose schema is the registry's batch schema;
//! - fixed tools without `__` in the name (they cannot collide with op tools): `run_script`,
//!   `session_hash`, `session_export`, `session_status`, `timeline_branch`,
//!   `timeline_checkout`, `timeline_undo`, `timeline_redo`, `timeline_log` and `help`.
//!
//! Results: the endpoint's result is `structuredContent` (a non-object result `v` is wrapped as
//! `{"result": v}`) and the same JSON, compact, is the text content. Application errors of the
//! endpoint are normal results with `isError: true` and `{"error": {code, kind, message}}`;
//! malformed arguments, unknown tools and endpoint failures are JSON-RPC errors.
//!
//! Feature `respell` rewrites ids in the *text* content to the command-language spelling
//! (`Face#1`) with `sm_lang::respell_json`. It is off by default. Ids are parsed with
//! `EntityKind::from_str`, which accepts both spellings, so an id copied from the text is
//! accepted back as an argument.

mod server;
mod tools;
mod version;

use std::io::{self, BufRead, Write};

use sm_protocol::Endpoint;

pub use version::McpVersion;

/// Serve MCP (JSON-RPC 2.0, newline-delimited) on `input`/`output` until EOF.
///
/// One reply line per request, in request order, flushed after each. Notifications and blank
/// lines get no reply. A malformed line gets a reply with `id: null` (`-32700` for invalid
/// JSON or UTF-8, `-32600` for a non-object, a batch array or a missing method). Requests run
/// one at a time; `notifications/cancelled` cannot interrupt a running call and is ignored.
///
/// # Errors
/// Only I/O errors of `input` or `output` end the loop early.
pub fn run_stdio(
    endpoint: &mut dyn Endpoint,
    mut input: impl BufRead,
    mut output: impl Write,
) -> io::Result<()> {
    let mut mcp = server::Mcp::new(endpoint);
    let mut line = Vec::new();
    loop {
        line.clear();
        if input.read_until(b'\n', &mut line)? == 0 {
            return Ok(());
        }
        if let Some(reply) = mcp.handle_line(&line) {
            let mut text = serde_json::to_string(&reply).map_err(io::Error::other)?;
            text.push('\n');
            output.write_all(text.as_bytes())?;
            output.flush()?;
        }
    }
}
