//! Node-file text emitter.
//!
//! Node files are written by this small emitter instead of `toml::to_string`, which puts
//! strings containing newlines into multi-line literals; a CRLF checkout (git `autocrlf`) would
//! then change the string's content. Here every string is a single-line basic string with
//! escapes, so line-ending conversion cannot alter any value. Reading uses the `toml` crate, and
//! hashes never depend on this text.

use crate::codec::Value;
use crate::hash::Hash256;
use crate::timeline::Node;

/// Render a node as `ops/<hash>.toml` text, or explain why it cannot be rendered.
pub(crate) fn node_to_toml(n: &Node) -> Result<String, String> {
    let params = n.body.op.params.as_object().ok_or("params are not an object")?;
    let mut out = String::new();
    if let Some(p) = n.body.parent {
        out.push_str(&format!("parent = \"{p}\"\n\n"));
    }
    out.push_str(&format!(
        "[op]\nname = {}\nversion = {}\n\n[op.params]\n",
        quote(&n.body.op.name),
        n.body.op.version
    ));
    let mut keys: Vec<&String> = params.keys().collect();
    keys.sort();
    for k in keys {
        out.push_str(&format!("{} = ", key(k)));
        write_value(&params[k], &mut out)?;
        out.push('\n');
    }
    let m = &n.body.meta;
    let check: Hash256 = n.check.state;
    out.push_str(&format!(
        "\n[meta]\nauthor = {}\ntime = {}\nmessage = {}\n\n[check]\nstate = \"{check}\"\n",
        quote(&m.author),
        quote(&m.time),
        quote(&m.message),
    ));
    Ok(out)
}

fn quote(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => o.push_str(&format!("\\u{:04X}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn key(k: &str) -> String {
    if !k.is_empty() && k.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-') {
        k.to_owned()
    } else {
        quote(k)
    }
}

fn write_value(v: &Value, out: &mut String) -> Result<(), String> {
    match v {
        Value::Null => return Err("null is not representable".to_owned()),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                out.push_str(&i.to_string());
            } else if let Some(f) = n.as_f64().filter(|_| n.is_f64()) {
                if !f.is_finite() {
                    return Err("non-finite float".to_owned());
                }
                // `{:?}` is the shortest round-trip form and is valid TOML ("1.0", "1e21", "1.5e-7").
                out.push_str(&format!("{f:?}"));
            } else {
                return Err(format!("integer {n} exceeds the i64 range"));
            }
        }
        Value::String(s) => out.push_str(&quote(s)),
        Value::Array(a) => {
            out.push('[');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                write_value(x, out)?;
            }
            out.push(']');
        }
        Value::Object(m) if m.is_empty() => out.push_str("{}"),
        Value::Object(m) => {
            out.push_str("{ ");
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort();
            for (i, k) in keys.into_iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                out.push_str(&format!("{} = ", key(k)));
                write_value(&m[k], out)?;
            }
            out.push_str(" }");
        }
    }
    Ok(())
}
