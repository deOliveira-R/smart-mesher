//! `smesh help [op]`: the catalog rendered for the terminal.

use clap::CommandFactory;
use serde::Serialize;
use serde_json::Value;
use sm_protocol::CatalogResult;

use crate::{Cli, Failure, Io, Res};

fn put(io: &mut Io<'_>, s: &str) {
    let _ = writeln!(io.stdout, "{s}");
}

fn jstr<T: Serialize>(t: &T) -> String {
    match serde_json::to_value(t) {
        Ok(Value::String(s)) => s,
        Ok(v) => v.to_string(),
        Err(_) => String::new(),
    }
}

fn description(v: &Value) -> &str {
    v.get("description").and_then(Value::as_str).unwrap_or("")
}

fn first_line(v: &Value) -> &str {
    v.get("description")
        .and_then(Value::as_str)
        .and_then(|d| d.lines().next())
        .unwrap_or("")
}

/// Compact type of a property schema: `string`, `array<string>`, `a | b`, or `any`.
fn type_of(p: &Value) -> String {
    match p.get("type") {
        Some(Value::String(t)) if t == "array" => {
            format!(
                "array<{}>",
                p.get("items").map_or_else(|| "any".to_owned(), type_of)
            )
        }
        Some(Value::String(t)) => t.clone(),
        Some(Value::Array(ts)) => ts
            .iter()
            .filter_map(Value::as_str)
            .filter(|t| *t != "null")
            .collect::<Vec<_>>()
            .join(" | "),
        _ => match p
            .get("anyOf")
            .or_else(|| p.get("oneOf"))
            .and_then(Value::as_array)
        {
            Some(alts) => alts
                .iter()
                .map(type_of)
                .filter(|t| t != "null")
                .collect::<Vec<_>>()
                .join(" | "),
            None => p.get("$ref").and_then(Value::as_str).map_or_else(
                || "any".to_owned(),
                |r| r.rsplit('/').next().unwrap_or(r).to_owned(),
            ),
        },
    }
}

fn params_table(schema: &Value) -> Vec<String> {
    let required: Vec<&str> = schema
        .get("required")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    let Some(props) = schema.get("properties").and_then(Value::as_object) else {
        return Vec::new();
    };
    let mut names: Vec<&String> = props.keys().collect();
    names.sort();
    names
        .into_iter()
        .map(|n| {
            let p = &props[n];
            let mut flags = Vec::new();
            if required.contains(&n.as_str()) {
                flags.push("required".to_owned());
            }
            if p.get("x-positional").and_then(Value::as_bool) == Some(true) {
                flags.push("positional".to_owned());
            }
            if let Some(k) = p
                .get("x-entity-kind")
                .or_else(|| p.get("items").and_then(|i| i.get("x-entity-kind")))
                .and_then(Value::as_array)
            {
                flags.push(format!(
                    "ids of {}",
                    k.iter()
                        .filter_map(Value::as_str)
                        .collect::<Vec<_>>()
                        .join("|")
                ));
            }
            let flags = if flags.is_empty() {
                String::new()
            } else {
                format!(" [{}]", flags.join(", "))
            };
            format!("  {n}: {}{flags}  {}", type_of(p), first_line(p))
                .trim_end()
                .to_owned()
        })
        .collect()
}

pub(crate) fn show(
    io: &mut Io<'_>,
    json: bool,
    cat: &CatalogResult,
    raw: &Value,
    op: Option<&str>,
) -> Res<()> {
    let Some(op) = op else {
        if json {
            put(io, &serde_json::to_string(raw).unwrap_or_default());
            return Ok(());
        }
        let help = Cli::command().render_long_help().to_string();
        put(io, help.trim_end());
        put(io, "");
        let mut ops: Vec<_> = cat
            .ops
            .iter()
            .map(|o| {
                (
                    o.name.as_str(),
                    first_line(&serde_json::to_value(&o.schema).unwrap_or_default()).to_owned(),
                )
            })
            .collect();
        ops.sort();
        put(io, "Ops (smesh apply <op> <json>; smesh help <op>):");
        for (n, d) in ops {
            put(io, format!("  {n:<28} {d}").trim_end());
        }
        let mut qs: Vec<_> = cat
            .queries
            .iter()
            .map(|q| {
                (
                    q.name.as_str(),
                    first_line(&serde_json::to_value(&q.schema).unwrap_or_default()).to_owned(),
                )
            })
            .collect();
        qs.sort();
        put(io, "");
        put(io, "Queries (smesh query <name> [json]):");
        for (n, d) in qs {
            put(io, format!("  {n:<28} {d}").trim_end());
        }
        return Ok(());
    };
    let find = |name: &str| name == op || name == op.replace("__", ".");
    if let Some(o) = cat.ops.iter().find(|o| find(&o.name)) {
        let schema = serde_json::to_value(&o.schema).unwrap_or_default();
        if json {
            let mut v = serde_json::to_value(o).unwrap_or_default();
            v["kind"] = "op".into();
            put(io, &serde_json::to_string(&v).unwrap_or_default());
            return Ok(());
        }
        put(
            io,
            &format!(
                "{} (op, version {}, tool {})",
                o.name, o.version, o.tool_name
            ),
        );
        let d = description(&schema);
        if !d.is_empty() {
            put(io, d);
        }
        put(io, "\nParams (JSON object):");
        for l in params_table(&schema) {
            put(io, &l);
        }
        if let Some(p) = &o.positional {
            put(io, &format!("  `{p}` may be given positionally in scripts"));
        }
        if !o.outputs.is_empty() {
            put(io, "\nOutputs (roles):");
            for r in &o.outputs {
                let kinds = r.kinds.iter().map(jstr).collect::<Vec<_>>().join("|");
                put(
                    io,
                    format!(
                        "  {}: {} of {}  {}",
                        r.name,
                        jstr(&r.cardinality),
                        if kinds.is_empty() {
                            "any".into()
                        } else {
                            kinds
                        },
                        r.doc
                    )
                    .trim_end(),
                );
            }
        }
        return Ok(());
    }
    if let Some(q) = cat.queries.iter().find(|q| find(&q.name)) {
        let schema = serde_json::to_value(&q.schema).unwrap_or_default();
        if json {
            let mut v = serde_json::to_value(q).unwrap_or_default();
            v["kind"] = "query".into();
            put(io, &serde_json::to_string(&v).unwrap_or_default());
            return Ok(());
        }
        put(io, &format!("{} (query, tool {})", q.name, q.tool_name));
        let d = description(&schema);
        if !d.is_empty() {
            put(io, d);
        }
        put(io, "\nInput (JSON):");
        for l in params_table(&schema) {
            put(io, &l);
        }
        return Ok(());
    }
    let mut near: Vec<&str> = cat
        .ops
        .iter()
        .map(|o| o.name.as_str())
        .chain(cat.queries.iter().map(|q| q.name.as_str()))
        .filter(|n| n.contains(op) || op.contains(n))
        .collect();
    near.sort_unstable();
    let hint = if near.is_empty() {
        String::new()
    } else {
        format!("; did you mean {}?", near.join(", "))
    };
    Err(Failure::usage(format!("no op or query named `{op}`{hint}")))
}
