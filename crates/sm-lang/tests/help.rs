//! Help text generated from the demo registry's schemas.

mod common;

use sm_lang::help::{callables, language_summary, op_help, overview};

use common::*;

#[test]
fn overview_groups_ops_and_queries_by_namespace() {
    let r = registry();
    let o = overview(&r);
    for want in ["ops:", "queries:", "[model]", "[intent]", "[mesh]", "[core]", "model.create_brick(x, y, z, name?)", "Create a brick: a body with one volume and six faces.", "model.count(kind?)"] {
        assert!(o.contains(want), "{want:?} missing from:\n{o}");
    }
    // Ops are listed before queries.
    assert!(o.find("ops:").unwrap() < o.find("queries:").unwrap());
    assert_eq!(o, overview(&r), "deterministic");
}

#[test]
fn op_help_comes_from_the_schema() {
    let r = registry();
    let h = op_help(&r, "intent.set_size").unwrap();
    for want in ["intent.set_size  (op, version 1)", "Set the target element size on entities.", "intent.set_size(ids: [id], size: number)", "ids", "required", "Entities that get the size.", "Target size."] {
        assert!(h.contains(want), "{want:?} missing from:\n{h}");
    }
    let m = op_help(&r, "mesh.run").unwrap();
    for want in ["\"map\" | \"pave\"", "merge?: bool", "[default: false]", "order?:", "Element order (1 or 2)."] {
        assert!(m.contains(want), "{want:?} missing from:\n{m}");
    }
    let q = op_help(&r, "model.count").unwrap();
    assert!(q.contains("(query, read-only, not recorded)"), "{q}");
    assert!(!q.contains("result: created/removed"));
    let b = op_help(&r, "core.batch").unwrap();
    assert!(b.contains("core.batch"), "{b}");
}

#[test]
fn unknown_names_suggest_the_closest() {
    let r = registry();
    let e = op_help(&r, "model.create_brik").unwrap_err();
    assert!(e.contains("did you mean `model.create_brick`?"), "{e}");
    assert!(op_help(&r, "zzzzzzzz").unwrap_err().starts_with("no op or query named"));
}

#[test]
fn language_summary_is_reachable() {
    let r = registry();
    assert_eq!(op_help(&r, "language").unwrap(), language_summary());
    assert!(language_summary().contains("faces in Volume#2"));
}

#[test]
fn callables_are_ops_then_queries_each_sorted() {
    let r = registry();
    let c = callables(&r);
    let ops: Vec<&str> = c.iter().filter(|c| !c.is_query).map(|c| c.name.as_str()).collect();
    let qs: Vec<&str> = c.iter().filter(|c| c.is_query).map(|c| c.name.as_str()).collect();
    assert!(ops.windows(2).all(|w| w[0] < w[1]) && qs.windows(2).all(|w| w[0] < w[1]));
    assert!(ops.contains(&"core.batch") && qs == ["model.count", "topo.related"]);
    let brick = c.iter().find(|c| c.name == "model.create_brick").unwrap();
    assert_eq!(brick.signature(), "model.create_brick(x: number, y: number, z: number, name?: string)");
}

#[test]
fn help_statements_emit_help_events() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = sm_lang::Interpreter::new(&mut h, sm_lang::Options::default());
    let rep = it.run_source("h", "help\nhelp mesh.run\nhelp language\nalias m = mesh.run\nhelp m\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let texts: Vec<&str> = rep.events.iter().filter_map(|e| if let sm_lang::Event::Help { text, .. } = e { Some(text.as_str()) } else { None }).collect();
    assert_eq!(texts.len(), 4);
    assert!(texts[0].contains("[model]") && texts[1].contains("\"map\" | \"pave\"") && texts[3] == texts[1]);
    let rep = it.run_source("h2", "help nope.nope");
    assert_eq!(rep.diagnostics[0].code, "unknown-name");
}
