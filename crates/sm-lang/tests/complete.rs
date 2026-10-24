//! Completion at a cursor: statement start, parameter names and values, fields, entity serials.

mod common;

use sm_lang::{CandidateKind, Completion, Interpreter, Options};

use common::*;

/// Complete `src` with the cursor at the `|` marker.
fn at(prelude: &str, src: &str) -> Completion {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    if !prelude.is_empty() {
        let rep = it.run_source("pre", prelude);
        assert!(rep.ok(), "{:?}", rep.diagnostics);
    }
    let cursor = src.find('|').expect("cursor marker");
    let text = src.replace('|', "");
    it.complete(&text, cursor)
}

fn labels(c: &Completion) -> Vec<&str> {
    c.items.iter().map(|i| i.label.as_str()).collect()
}

#[test]
fn statement_start_offers_ops_queries_keywords_and_aliases() {
    let c = at("alias brick = model.create_brick", "|");
    let l = labels(&c);
    for want in ["model.create_brick", "intent.set_size", "mesh.run", "model.count", "let", "batch", "for", "brick"] {
        assert!(l.contains(&want), "{want} missing from {l:?}");
    }
    let q = c.items.iter().find(|i| i.label == "model.count").unwrap();
    assert_eq!(q.kind, CandidateKind::Query);
    assert_eq!(q.insert, "model.count(");
    let op = c.items.iter().find(|i| i.label == "model.create_brick").unwrap();
    assert_eq!(op.detail.as_deref(), Some("Create a brick: a body with one volume and six faces."));
    // Prefix filtering and the replaced range.
    let c = at("", "let a = 1\nmodel.c|");
    assert_eq!(labels(&c), ["model.create_brick", "model.count"], "ops before queries");
    assert!(c.items.iter().all(|i| i.label.starts_with("model.c")));
    assert_eq!((c.start, c.end), (10, 17));
}

#[test]
fn inside_a_call_unused_parameters_come_first_required_before_optional() {
    let c = at("", "model.create_brick(|");
    assert_eq!(labels(&c), ["x", "y", "z", "name"]);
    assert_eq!(c.items[0].insert, "x: ");
    assert_eq!(c.items[0].kind, CandidateKind::Param);
    assert!(c.items[0].detail.as_deref().unwrap().contains("Extent along x."));
    let c = at("", "model.create_brick(x: 1, z: 2, |");
    assert_eq!(labels(&c), ["y", "name"], "given parameters are not offered again");
    let c = at("", "model.create_brick(x: 1, na|");
    assert_eq!(labels(&c), ["name"]);
    // Statement start is not confused by a call on an earlier line.
    let c = at("", "model.create_brick(x: 1, y: 1, z: 1)\n|");
    assert!(labels(&c).contains(&"mesh.run"));
}

#[test]
fn parameter_values_use_the_schema() {
    let c = at("", "mesh.run(scheme: |");
    let l = labels(&c);
    assert!(l.contains(&"\"map\"") && l.contains(&"\"pave\""), "{l:?}");
    assert_eq!(c.items.iter().find(|i| i.label == "\"map\"").unwrap().kind, CandidateKind::Value);
    let c = at("", "mesh.run(targets: [Volume#1], scheme: \"map\", merge: |");
    assert!(labels(&c).contains(&"true") && labels(&c).contains(&"false"));
    // Entity parameters offer kinds and `all <kind>` style starters, but no number builtins.
    let c = at("", "intent.set_size(ids: |");
    let l = labels(&c);
    assert!(l.contains(&"Volume#") && l.contains(&"all") && l.contains(&"faces"), "{l:?}");
    assert!(!l.contains(&"sin"));
    // Variables are offered with their type.
    let c = at("let w = 3", "model.create_brick(x: |");
    let v = c.items.iter().find(|i| i.label == "w").unwrap();
    assert_eq!(v.kind, CandidateKind::Variable);
    assert_eq!(v.detail.as_deref(), Some("int"));
}

#[test]
fn fields_of_op_results_and_objects() {
    let c = at("let b = model.create_brick(x: 1, y: 1, z: 1)", "intent.set_size(ids: b.|");
    assert_eq!(labels(&c), ["body", "created", "faces", "removed", "volume"]);
    let c = at("let o = {alpha: 1, beta: {gamma: 2}}", "print o.|");
    assert_eq!(labels(&c), ["alpha", "beta"]);
    let c = at("let o = {alpha: 1, beta: {gamma: 2}}", "print o.beta.|");
    assert_eq!(labels(&c), ["gamma"]);
    let c = at("", "print nothing.|");
    assert!(c.items.is_empty());
}

#[test]
fn entity_serials_come_from_the_live_state() {
    let pre = "model.create_brick(x: 1, y: 1, z: 1)\nmodel.create_brick(x: 1, y: 1, z: 1)";
    let c = at(pre, "model.pick(Volume#|");
    assert_eq!(labels(&c), ["1", "2"]);
    assert_eq!(c.items[0].detail.as_deref(), Some("TopoVolume#1"));
    let c = at(pre, "model.pick(Body#2|");
    assert_eq!(labels(&c), ["2"]);
    let c = at(pre, "intent.set_size(ids: Face#[1, 3, |");
    assert_eq!(c.items.len(), 12, "all faces of two bricks");
    let c = at(pre, "model.pick(Volume#9|");
    assert!(c.items.is_empty());
    // The list is capped.
    let c = at("for i in range(0, 20) { model.create_brick(x: 1, y: 1, z: 1) }", "model.pick(Face#|");
    assert_eq!(c.items.len(), 50);
}

#[test]
fn keywords_contexts() {
    let c = at("", "let x = all |");
    assert!(labels(&c).contains(&"faces") && labels(&c).contains(&"volumes"));
    assert!(!labels(&c).contains(&"sin"));
    let c = at("", "help |");
    assert!(labels(&c).contains(&"mesh.run"));
    let c = at("let n = 1", "set |");
    assert_eq!(labels(&c), ["n"]);
    let c = at("", "let |");
    assert!(c.items.is_empty(), "a new name is free text");
    // Inside a block the statement start is offered again.
    let c = at("", "batch {\n  |");
    assert!(labels(&c).contains(&"model.create_brick"));
    let c = at("", "for i in range(0, 3) { |");
    assert!(labels(&c).contains(&"print"));
}

#[test]
fn completion_is_total_and_deterministic() {
    // Any prefix of a real script, at any cursor, completes without panicking, and twice gives the same answer.
    let script = "let w = 10\nlet b = model.create_brick(x: w, y: 2 * w, z: [1, 2][0], name: \"s\")\nbatch {\n  intent.set_size(ids: faces in Volume#1 - Face#[1, 3..5], size: 0.5)\n}\n";
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    for cut in 0..=script.len() {
        if !script.is_char_boundary(cut) {
            continue;
        }
        let a = it.complete(script, cut);
        let b = it.complete(script, cut);
        assert_eq!(a, b);
    }
    let c = it.complete("é model.", 100);
    let _ = c;
}
