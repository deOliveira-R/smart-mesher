//! Output: `--json` prints the server's value verbatim; the human forms are short and stable.

use serde_json::Value;
use sm_protocol::{ApplyResult, ExportResult, HashResult, ScriptRecordResult, ScriptResult, StatusResult};

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
    // Per-op diagnostics (for example a `mesh.auto_tet` fallback), in script order.
    for e in &r.events {
        if matches!(e["event"].as_str(), Some("op" | "batch")) {
            for d in e["delta"]["diagnostics"].as_array().into_iter().flatten() {
                diagnostic(io, d);
            }
        }
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
        diagnostic(io, &serde_json::to_value(d).unwrap_or_default());
    }
}

/// One diagnostic as `severity: code: message` on stderr (shared by `apply` and `script`).
fn diagnostic(io: &mut Io<'_>, v: &Value) {
    let _ = writeln!(
        io.stderr,
        "{}: {}: {}",
        v["severity"].as_str().unwrap_or("note"),
        v["code"].as_str().unwrap_or(""),
        v["message"].as_str().unwrap_or("")
    );
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

pub(crate) fn record(
    io: &mut Io<'_>,
    json: bool,
    r: &ScriptRecordResult,
    raw: &Value,
    output: Option<&std::path::Path>,
) {
    if json {
        line(io, &compact(raw));
        return;
    }
    for n in &r.notes {
        let _ = writeln!(io.stderr, "note: {n}");
    }
    match output {
        Some(p) => line(
            io,
            &format!("wrote {} ({} nodes, {} statements)", p.display(), r.nodes, r.statements),
        ),
        None => {
            let _ = write!(io.stdout, "{}", r.text);
        }
    }
    if let Some(ok) = r.verified {
        let _ = writeln!(
            io.stderr,
            "verify: {}",
            if ok { "replay reproduces the hashes" } else { "REPLAY DIFFERS" }
        );
    }
}

/// Export notes on stderr (stdout is unchanged): every note in text mode, warnings only with
/// `--json`, so `smesh export --json | jq` still shows what the user should act on.
fn export_notes(io: &mut Io<'_>, json: bool, r: &ExportResult) {
    for n in &r.notes {
        if n.is_warning() {
            let _ = writeln!(io.stderr, "warning: {}: {}", n.code, n.detail);
        } else if !json {
            let _ = writeln!(io.stderr, "note: {}: {} (x{})", n.code, n.detail, n.count);
        }
    }
}

pub(crate) fn export(io: &mut Io<'_>, json: bool, r: &ExportResult, raw: &Value) {
    export_notes(io, json, r);
    if json {
        line(io, &compact(raw));
        return;
    }
    let path = r.path.as_deref().unwrap_or("<inline>");
    if r.files.is_empty() {
        line(
            io,
            &format!("wrote {path} ({} bytes, sha256 {})", r.bytes, r.sha256),
        );
        return;
    }
    line(io, &format!("wrote {path} ({} files)", r.files.len()));
    for f in &r.files {
        line(
            io,
            &format!("  {} ({} bytes, sha256 {})", f.name, f.bytes, f.sha256),
        );
    }
}

pub(crate) fn staged(io: &mut Io<'_>, json: bool, file: &std::path::Path, r: &Value) {
    if json {
        line(io, &compact(r));
        return;
    }
    line(
        io,
        &format!(
            "staged {} ({} bytes, sha256 {})",
            file.display(),
            r["bytes"],
            text(&r["sha256"])
        ),
    );
}

/// A scalar as text, or `-` when absent or null.
fn text(v: &Value) -> String {
    match v {
        Value::Null => "-".to_owned(),
        Value::String(t) => t.clone(),
        o => compact(o),
    }
}

fn ids(v: &Value) -> String {
    match v.as_array() {
        Some(a) if !a.is_empty() => a.iter().map(text).collect::<Vec<_>>().join(","),
        _ => "-".to_owned(),
    }
}

fn point(v: &Value) -> String {
    match v.as_array() {
        Some(a) if a.len() == 3 => {
            let c: Vec<String> = a.iter().map(num).collect();
            format!("({})", c.join(", "))
        }
        _ => "-".to_owned(),
    }
}

fn num(v: &Value) -> String {
    v.as_f64()
        .map_or_else(|| "-".to_owned(), |x| format!("{x:.6e}"))
}

/// A mesh entity `{id, index}`: its id, else the dense index.
fn entity(v: &Value) -> String {
    match v {
        Value::Null => "-".to_owned(),
        e => match e["id"].as_str() {
            Some(id) => id.to_owned(),
            None => format!("#{}", e["index"]),
        },
    }
}

pub(crate) fn validate(io: &mut Io<'_>, json: bool, v: &Value) {
    if json {
        line(io, &compact(v));
        return;
    }
    let valid = v["valid"].as_bool().unwrap_or(false);
    let cut = if v["truncated"] == Value::Bool(true) {
        ", list truncated"
    } else {
        ""
    };
    line(
        io,
        &format!(
            "{}: {} ({} violations{cut})",
            text(&v["mesh"]),
            if valid { "valid" } else { "INVALID" },
            v["total"],
        ),
    );
    if v["geometry_checked"] == Value::Bool(false) {
        line(
            io,
            &format!(
                "  geometry not checked: {}",
                text(&v["geometry_skipped_reason"])
            ),
        );
    }
    for x in v["violations"].as_array().into_iter().flatten() {
        line(
            io,
            &format!(
                "  {} {} entity={} owner={} near={} at={}",
                text(&x["severity"]),
                text(&x["code"]),
                entity(&x["entity"]),
                text(&x["owner"]),
                ids(&x["near"]),
                point(&x["location"])
            ),
        );
        if let Some(d) = x["detail"].as_str().filter(|d| !d.is_empty()) {
            line(io, &format!("      {d}"));
        }
    }
}

pub(crate) fn quality(io: &mut Io<'_>, json: bool, v: &Value) {
    if json {
        line(io, &compact(v));
        return;
    }
    line(
        io,
        &format!(
            "{}: verdict {} (profile {}, {} cells, dim {})",
            text(&v["mesh"]),
            text(&v["verdict"]),
            text(&v["profile"]),
            v["cells"],
            v["dim"]
        ),
    );
    let va = &v["validity"];
    line(
        io,
        &format!(
            "  validity: {} valid, {} degenerate, {} inverted",
            va["n_valid"], va["n_degenerate"], va["n_inverted"]
        ),
    );
    for m in v["metrics"].as_array().into_iter().flatten() {
        let sm = &m["summary"];
        line(
            io,
            &format!(
                "  {} [{}]: min {} max {} mean {} n_warn {} n_fail {}",
                text(&m["metric"]),
                text(&m["entity"]),
                num(&sm["min"]),
                num(&sm["max"]),
                num(&sm["mean"]),
                m["n_warn"],
                m["n_fail"]
            ),
        );
        for w in m["worst"].as_array().into_iter().flatten() {
            line(
                io,
                &format!(
                    "      worst {} = {} grade={} owner={} at={}",
                    entity(&w["entity"]),
                    num(&w["value"]),
                    text(&w["grade"]),
                    text(&w["owner"]),
                    point(&w["location"])
                ),
            );
        }
    }
    for k in v["skipped"].as_array().into_iter().flatten() {
        line(io, &format!("  skipped: {}", compact(k)));
    }
    let sug = v["suggestions"].as_array().cloned().unwrap_or_default();
    if !sug.is_empty() {
        line(io, "  suggestions:");
    }
    for x in &sug {
        let avail = if x["available"] == Value::Bool(true) {
            "available"
        } else {
            "unavailable"
        };
        let op = x["op"]
            .as_str()
            .map_or(String::new(), |o| format!(" op={o}"));
        line(
            io,
            &format!(
                "    {} {} ({avail}{op}) targets={}: {}",
                text(&x["severity"]),
                text(&x["kind"]),
                ids(&x["targets"]),
                text(&x["reason"])
            ),
        );
    }
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
            hash.chars().take(12).collect::<String>(),
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
