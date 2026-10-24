//! Output: `--json` prints the server's value verbatim; the human forms are short and stable.

use serde_json::Value;
use sm_protocol::{ApplyResult, ExportResult, HashResult, ScriptResult, StatusResult};

use crate::{Failure, Io, Res};

fn line(io: &mut Io<'_>, text: &str) {
    // A closed pipe (`smesh log | head`) is not an error worth reporting.
    let _ = writeln!(io.stdout, "{text}");
}

fn compact(v: &Value) -> String {
    serde_json::to_string(v).unwrap_or_default()
}

pub(crate) fn failure(io: &mut Io<'_>, json: bool, f: &Failure) {
    if json {
        let v = serde_json::json!({ "error": f.err });
        line(io, &compact(&v));
        return;
    }
    let _ = writeln!(io.stderr, "error[{}]: {}", f.err.kind, f.err.message);
    if !f.err.data.is_null() {
        let pretty = serde_json::to_string_pretty(&f.err.data).unwrap_or_default();
        let _ = writeln!(io.stderr, "{pretty}");
    }
    if f.exit == 2 {
        let _ = writeln!(io.stderr, "try `smesh --help`");
    }
}

/// A generic JSON value (query results).
pub(crate) fn value(io: &mut Io<'_>, json: bool, v: &Value) {
    if json {
        line(io, &compact(v));
    } else {
        line(io, &serde_json::to_string_pretty(v).unwrap_or_default());
    }
}

pub(crate) fn script(io: &mut Io<'_>, json: bool, r: &ScriptResult, raw: &Value, print_hash: bool) {
    if json {
        line(io, &compact(raw));
        return;
    }
    if !r.output.is_empty() {
        let _ = write!(io.stdout, "{}", r.output);
        if !r.output.ends_with('\n') {
            let _ = writeln!(io.stdout);
        }
    }
    let has_diag = match &r.diagnostics {
        Value::Null => false,
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
        _ => true,
    };
    if has_diag {
        let _ = writeln!(io.stderr, "diagnostics: {}", compact(&r.diagnostics));
    }
    if print_hash {
        line(io, &r.state.to_string());
    }
}

pub(crate) fn apply(io: &mut Io<'_>, json: bool, op: &str, r: &ApplyResult, raw: &Value) {
    if json {
        line(io, &compact(raw));
        return;
    }
    line(io, &format!("{op}: node {} state {}", r.node, r.state));
    for (role, ids) in &r.delta.outputs {
        let ids: Vec<String> = ids.iter().map(ToString::to_string).collect();
        line(io, &format!("  {role}: {}", ids.join(" ")));
    }
    for d in &r.delta.diagnostics {
        let v = serde_json::to_value(d).unwrap_or_default();
        let _ = writeln!(
            io.stderr,
            "{}: {}: {}",
            v["severity"].as_str().unwrap_or("note"),
            v["code"].as_str().unwrap_or(""),
            v["message"].as_str().unwrap_or("")
        );
    }
}

pub(crate) fn hash(io: &mut Io<'_>, json: bool, r: &HashResult, raw: &Value, layers: bool) {
    if json {
        line(io, &compact(raw));
        return;
    }
    line(io, &r.state.to_string());
    if layers {
        for (name, h) in &r.layers {
            line(io, &format!("  {name}: {h}"));
        }
    }
}

pub(crate) fn export(io: &mut Io<'_>, json: bool, r: &ExportResult, raw: &Value) {
    if json {
        line(io, &compact(raw));
        return;
    }
    let path = r.path.as_deref().unwrap_or("<inline>");
    line(
        io,
        &format!("wrote {path} ({} bytes, sha256 {})", r.bytes, r.sha256),
    );
}

fn head_text(raw: &Value) -> String {
    match &raw["head"] {
        Value::String(s) => s.clone(),
        h => match (
            h.get("ref").and_then(Value::as_str),
            h.get("node").and_then(Value::as_str),
        ) {
            (Some(r), _) => r.to_owned(),
            (_, Some(n)) => format!("detached {n}"),
            _ => compact(h),
        },
    }
}

fn status_lines(s: &StatusResult, raw: &Value) -> Vec<String> {
    let pos = s.position.map_or_else(|| "-".to_owned(), |p| p.to_string());
    let mut v = vec![
        format!("head:     {}", head_text(raw)),
        format!("node:     {pos} (depth {})", s.depth),
        format!("state:    {}", s.state),
        format!(
            "nodes:    {}   redo: {}   dirty: {}",
            s.nodes, s.redo, s.dirty
        ),
    ];
    if let Some(p) = &s.project {
        v.push(format!("project:  {p}"));
    }
    for (name, tip) in &s.branches {
        v.push(format!("branch:   {name} {tip}"));
    }
    v
}

pub(crate) fn status(
    io: &mut Io<'_>,
    json: bool,
    s: &StatusResult,
    raw: &Value,
    note: Option<&str>,
) -> Res<()> {
    if json {
        line(io, &compact(raw));
        return Ok(());
    }
    if let Some(n) = note {
        line(io, n);
    }
    for l in status_lines(s, raw) {
        line(io, &l);
    }
    Ok(())
}

/// `log`: nodes newest first, each tagged with the branches that point at it. The JSON form is
/// one object `{head, position, branches, nodes}` because the protocol's `timeline.log` returns
/// the nodes only.
pub(crate) fn log(
    io: &mut Io<'_>,
    json: bool,
    s: &StatusResult,
    status_raw: &Value,
    nodes: &Value,
) -> Res<()> {
    if json {
        let v = serde_json::json!({
            "head": status_raw["head"],
            "position": status_raw["position"],
            "branches": status_raw["branches"],
            "nodes": nodes,
        });
        line(io, &compact(&v));
        return Ok(());
    }
    line(io, &format!("HEAD {}", head_text(status_raw)));
    let empty = Vec::new();
    for n in nodes.as_array().unwrap_or(&empty) {
        let hash = n["node"].as_str().unwrap_or("?");
        let tags: Vec<&str> = s
            .branches
            .iter()
            .filter(|(_, tip)| tip.to_string() == hash)
            .map(|(b, _)| b.as_str())
            .collect();
        let tag = if tags.is_empty() {
            String::new()
        } else {
            format!(" ({})", tags.join(", "))
        };
        let text = format!(
            "{} {}{tag}  {}  {} {}",
            &hash[..hash.len().min(12)],
            n["op"].as_str().unwrap_or(""),
            n["author"].as_str().unwrap_or(""),
            n["time"].as_str().unwrap_or(""),
            n["message"]
                .as_str()
                .filter(|m| Some(*m) != n["op"].as_str())
                .unwrap_or("")
        );
        line(io, text.trim_end());
    }
    Ok(())
}
