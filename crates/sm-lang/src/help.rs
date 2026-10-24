//! Help text generated from the op and query schemas in the registry.

use std::fmt::Write;

use sm_op_api::Registry;

use crate::diag::suggest;
use crate::schema::{ParamInfo, SchemaDoc};

/// One callable (op or query) with its schema, as help sees it.
#[derive(Clone, Debug)]
pub struct Callable {
    /// Registry name.
    pub name: String,
    /// True for a query (read-only, never recorded).
    pub is_query: bool,
    /// Payload version (ops only; queries are 0).
    pub version: u32,
    /// Params schema.
    pub doc: SchemaDoc,
    /// Result schema (queries only): lets entity-id results become entity sets.
    pub out: Option<SchemaDoc>,
    /// Declared output roles (ops only); empty when the op declares none, in which case roles are not checked statically.
    pub outputs: Vec<sm_op_api::OutputRole>,
}

impl Callable {
    /// Description from the schema (doc comment), if any.
    pub fn description(&self) -> Option<String> {
        self.doc.view().description().map(str::to_owned)
    }

    /// Parameters, required first.
    pub fn params(&self) -> Vec<ParamInfo> {
        self.doc.view().params()
    }

    /// `name(a: ty, b?: ty)`.
    pub fn signature(&self) -> String {
        let ps: Vec<String> = self.params().iter().map(|p| format!("{}{}: {}", p.name, if p.required { "" } else { "?" }, p.ty)).collect();
        format!("{}({})", self.name, ps.join(", "))
    }
}

/// Every op and query in registry order (ops, then queries, each sorted by name). Entries whose schema is broken are skipped
/// (the registry reports them in its own catalog).
pub fn callables(registry: &Registry) -> Vec<Callable> {
    let mut out = Vec::new();
    for info in registry.catalog().into_iter().flatten() {
        out.push(Callable { name: info.name, is_query: false, version: info.version, doc: SchemaDoc::new(info.schema.as_value().clone()), out: None, outputs: info.outputs });
    }
    for info in registry.query_catalog().into_iter().flatten() {
        out.push(Callable { name: info.name, is_query: true, version: 0, doc: SchemaDoc::new(info.schema.as_value().clone()), out: Some(SchemaDoc::new(info.output_schema.as_value().clone())), outputs: Vec::new() });
    }
    out
}

fn first_line(s: &str) -> &str {
    s.lines().next().unwrap_or("").trim()
}

/// The `help` overview: every op and query grouped by namespace, one line each.
pub fn overview(registry: &Registry) -> String {
    let all = callables(registry);
    let mut out = String::from("smesh: commands (type `help <name>` for details, `help` for this list)\n");
    for (title, want_query) in [("ops", false), ("queries", true)] {
        let items: Vec<&Callable> = all.iter().filter(|c| c.is_query == want_query).collect();
        if items.is_empty() {
            continue;
        }
        let _ = writeln!(out, "\n{title}:");
        let mut ns = "";
        for c in items {
            let space = c.name.split('.').next().unwrap_or("");
            if space != ns {
                ns = space;
                let _ = writeln!(out, "  [{ns}]");
            }
            let names: Vec<String> = c.params().iter().map(|p| format!("{}{}", p.name, if p.required { "" } else { "?" })).collect();
            let head = format!("{}({})", c.name, names.join(", "));
            match c.description() {
                Some(d) => {
                    let _ = writeln!(out, "    {head:<48} {}", first_line(&d));
                }
                None => {
                    let _ = writeln!(out, "    {head}");
                }
            }
        }
    }
    out.push_str("\nlanguage: `help language` summarizes the syntax.\n");
    out
}

/// A compact syntax summary.
pub fn language_summary() -> &'static str {
    "smesh language summary (full reference: LANGUAGE.md)
  calls       model.create_brick(x: 10, y: 10, z: 10)     named args; ...{json} spreads an object
  variables   let w = 10;  set w = 12;  let b = model.create_brick(x: w, y: w, z: w)   (b.body is its output)
  expressions + - * / % **  == != < <= > >=  and or not  | & (set union/intersection)  - except (difference)
              (`-`/`except` next to `|`/`&`, and `in` before a set operator, need parentheses)
  entities    Volume#3  Volume#1..5  Volume#[1, 3, 5..7]  Volume#[i]  Volume#(n + 1)  all faces  faces in Volume#2
              (Cubit words: surfaces = faces, curves = edges; contains(set, Face#2) tests membership)
  batch       batch { ... }   one atomic node; outputs referenced as $i.role
  control     if c { } else { }   for i in range(0, 3) { }   for i in 1..n { }   include \"file.smesh\"
              (`if c { a } else { b }` is also an expression; `1..n` is the inclusive list 1, 2, ..., n)
  timeline    timeline.branch(\"name\")  timeline.checkout(\"name\")  timeline.undo()  timeline.redo()   (statements, not ops)
              (`timeline.head()` returns {branch, node} and may be used in expressions)
  session     session.export(\"out.vtu\", format: \"vtu\", mesh: Mesh#1)  session.save()   (statements, not ops)
              (`session.hash()` and `session.status()` return values and may be used in expressions)
  other       print e, ...   assert cond, \"msg\"   alias short = model.create_brick   help [name]
  comments    // line   /* block */
"
}

/// Detailed help for one op or query.
pub fn op_help(registry: &Registry, name: &str) -> Result<String, String> {
    if name == "language" {
        return Ok(language_summary().to_owned());
    }
    if let Some(verb) = name.strip_prefix("timeline.") {
        return match verb {
            "branch" => Ok("timeline.branch(name: string)  (timeline command, not an op)\nFork: create a branch at the current position and switch to it.\n".to_owned()),
            "checkout" => Ok("timeline.checkout(ref | node | empty)  (timeline command, not an op)\nMove to a branch (`ref`, or the single positional string), a node by hash (`node`), or the empty state (`empty: true`).\n".to_owned()),
            "undo" => Ok("timeline.undo()  (timeline command, not an op)\nStep back one node; the undone node stays reachable by `checkout`.\n".to_owned()),
            "redo" => Ok("timeline.redo()  (timeline command, not an op)\nStep forward again after an undo.\n".to_owned()),
            "head" => Ok("timeline.head()  (timeline command, not an op)\nReturns the object {branch, node}: the attached branch (absent when detached) and the current node hash (absent in the empty state). May be used in expressions.\n".to_owned()),
            _ => Err(format!("no timeline command `{name}`; timeline commands: branch, checkout, undo, redo, head")),
        };
    }
    if let Some(verb) = name.strip_prefix("session.") {
        return match verb {
            "export" => Ok("session.export(path: string, format?: \"smsh\" | \"vtu\", mesh?: id, provenance?: bool)  (session command, not an op)\nWrite a mesh to a file. `path` may be positional; a relative path is resolved by the host. `format` defaults to the file extension; `mesh` defaults to the only mesh; `provenance` embeds the timeline hash.\n".to_owned()),
            "save" => Ok("session.save()  (session command, not an op)\nSave the project.\n".to_owned()),
            "hash" => Ok("session.hash()  (session command, not an op)\nReturns the state hash. May be used in expressions.\n".to_owned()),
            "status" => Ok("session.status()  (session command, not an op)\nReturns the session status object. May be used in expressions.\n".to_owned()),
            _ => Err(format!("no session command `{name}`; session commands: export, save, hash, status")),
        };
    }
    let all = callables(registry);
    let Some(c) = all.iter().find(|c| c.name == name) else {
        let mut m = format!("no op or query named `{name}`");
        if let Some(s) = suggest(name, all.iter().map(|c| c.name.as_str())) {
            let _ = write!(m, "; did you mean `{s}`?");
        }
        return Err(m);
    };
    let mut out = String::new();
    if c.is_query {
        let _ = writeln!(out, "{}  (query, read-only, not recorded)", c.name);
    } else {
        let _ = writeln!(out, "{}  (op, version {})", c.name, c.version);
    }
    if let Some(d) = c.description() {
        for l in d.lines() {
            let _ = writeln!(out, "  {l}");
        }
    }
    let _ = writeln!(out, "\nusage:\n  {}", c.signature());
    let params = c.params();
    if params.is_empty() {
        out.push_str("\nno parameters\n");
    } else {
        out.push_str("\nparameters:\n");
        let w = params.iter().map(|p| p.name.len()).max().unwrap_or(0);
        let tw = params.iter().map(|p| p.ty.len()).max().unwrap_or(0).min(40);
        for p in &params {
            let req = if p.required { "required" } else { "optional" };
            let mut line = format!("  {:<w$}  {:<tw$}  {req}", p.name, p.ty);
            if let Some(d) = &p.description {
                let _ = write!(line, "  {}", first_line(d));
            }
            if let Some(d) = &p.default {
                let _ = write!(line, "  [default: {d}]");
            }
            let _ = writeln!(out, "{line}");
        }
    }
    if !c.is_query {
        out.push_str("\nresult: created/removed ids and named outputs; `let r = ...` then `r.created`, `r.<role>`.\n");
    }
    Ok(out)
}
