//! MCP server over the protocol. Skeleton: see W2 plan §2 task T11 (`docs/design/w2/plan.md`);
//! that task fills in the bodies. It codes against `dyn Endpoint`, never against `sm-server`.

use std::io::{self, BufRead, Write};

use serde_json::{Value, json};
use sm_protocol::Endpoint;

/// Serve MCP (JSON-RPC 2.0, newline-delimited) on `input`/`output` until EOF.
///
/// Skeleton: `initialize` and `ping` are answered minimally, every other request gets
/// `method not found` (-32601), notifications and blank lines are ignored.
pub fn run_stdio(
    _endpoint: &mut dyn Endpoint,
    input: impl BufRead,
    mut output: impl Write,
) -> io::Result<()> {
    for line in input.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let Ok(msg) = serde_json::from_str::<Value>(&line) else {
            continue;
        };
        let Some(id) = msg.get("id").cloned() else {
            continue; // notification
        };
        let reply = match msg.get("method").and_then(Value::as_str) {
            Some("initialize") => json!({"jsonrpc": "2.0", "id": id, "result": {
                "protocolVersion": "2025-06-18",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "smesh", "version": env!("CARGO_PKG_VERSION")},
            }}),
            Some("ping") => json!({"jsonrpc": "2.0", "id": id, "result": {}}),
            _ => json!({"jsonrpc": "2.0", "id": id, "error": {"code": -32601, "message": "method not found"}}),
        };
        writeln!(output, "{reply}")?;
        output.flush()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_protocol::{Request, RpcError};

    struct Fake;
    impl Endpoint for Fake {
        fn call(&mut self, _req: Request) -> Result<Value, RpcError> {
            Ok(Value::Null)
        }
    }

    #[test]
    fn answers_initialize_and_rejects_the_rest() {
        let input = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\"}\n\
                     {\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}\n\
                     {\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"tools/list\"}\n";
        let mut out = Vec::new();
        run_stdio(&mut Fake, input.as_bytes(), &mut out).expect("run");
        let lines: Vec<Value> = String::from_utf8(out)
            .expect("utf8")
            .lines()
            .map(|l| serde_json::from_str(l).expect("json"))
            .collect();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0]["result"]["protocolVersion"], "2025-06-18");
        assert_eq!(lines[1]["error"]["code"], -32601);
    }
}
