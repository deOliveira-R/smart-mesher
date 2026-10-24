//! The schema checker against the registry's real schemas, the contract's patterns, `QueryRelations` and aliases.

mod common;

use serde_json::json;
use sm_lang::schema::{PathSeg, SchemaDoc, check, check_in_batch, is_batch_ref, is_id_text};
use sm_lang::{EngineHost, Interpreter, Options, QueryRelations};

use common::*;

fn doc(name: &str) -> SchemaDoc {
    SchemaDoc::new(registry().schema(name).unwrap().as_value().clone())
}

fn issues(name: &str, v: serde_json::Value) -> Vec<(String, &'static str)> {
    let d = doc(name);
    let mut out = Vec::new();
    check(d.view(), &v, true, &mut out);
    out.into_iter()
        .map(|i| {
            let p: Vec<String> = i.path.iter().map(|s| match s { PathSeg::Key(k) => k.clone(), PathSeg::Idx(n) => n.to_string() }).collect();
            (p.join("/"), i.code)
        })
        .collect()
}

#[test]
fn the_contract_patterns_are_pinned() {
    // These are the exact strings the checker reimplements by hand: if the contract changes them, this fails first.
    assert_eq!(sm_lang::schema::ID_PATTERN, r"^[A-Za-z]+#[0-9]+$");
    assert_eq!(sm_op_api::batch::REF_PATTERN, r"^\$[0-9]+\.[^\[\]]+(\[([0-9]+|\*)\])?$");
    assert_eq!(sm_op_api::batch::ESCAPED_LITERAL_PATTERN, r"^\$\$");
    for ok in ["$0.body", "$12.faces[3]", "$1.faces[*]", "$0.a.b", "$0.x y"] {
        assert!(is_batch_ref(ok), "{ok}");
    }
    for bad in ["", "$", "$.body", "$0", "$0.", "$a.body", "$0.body[]", "$0.body[x]", "$0.body[1", "$0.bo]dy", "0.body", "$0.body[*]x", "$$0.body", "$plan.1.body"] {
        assert!(!is_batch_ref(bad), "{bad}");
    }
    assert!(is_id_text("TopoVolume#3") && is_id_text("Volume#0"));
    for bad in ["Volume#", "#3", "Vol1#3", "Volume#-1", "Volume #3", "Volume#3x", ""] {
        assert!(!is_id_text(bad), "{bad}");
    }
}

/// The id pattern is recognized by exact string match, so it is pinned against what `sm-core` really emits: if the
/// contract's schema changes, entity parameters would silently stop being recognized without this test.
#[test]
fn the_id_pattern_is_what_sm_core_emits() {
    let schema = schemars::schema_for!(sm_core::EntityId);
    let v = serde_json::to_value(&schema).unwrap();
    assert_eq!(v["pattern"], sm_lang::schema::ID_PATTERN, "{v}");
    assert_eq!(v["type"], "string");
}

#[test]
fn validates_values_against_op_schemas() {
    assert!(issues("model.create_brick", json!({"x": 1.0, "y": 2.0, "z": 3.0})).is_empty());
    assert!(issues("model.create_brick", json!({"x": 1, "y": 2, "z": 3, "name": "n"})).is_empty());
    assert_eq!(issues("model.create_brick", json!({"x": 1.0})), [("".into(), "missing-property"), ("".into(), "missing-property")]);
    assert_eq!(issues("model.create_brick", json!({"x": "a", "y": 1, "z": 1})), [("x".into(), "type-mismatch")]);
    assert_eq!(issues("model.create_brick", json!({"x": 1, "y": 1, "z": 1, "w": 1})), [("w".into(), "unknown-property")]);
    assert_eq!(issues("mesh.run", json!({"targets": ["TopoVolume#1"], "scheme": "x"})), [("scheme".into(), "enum-violation")]);
    assert_eq!(issues("mesh.run", json!({"targets": ["Volume 1"], "scheme": "map"})), [("targets/0".into(), "pattern-mismatch")]);
    assert_eq!(issues("mesh.run", json!({"targets": [], "scheme": "map", "order": 3})), [("order".into(), "out-of-range")]);
    assert_eq!(issues("mesh.run", json!({"targets": [], "scheme": "map", "order": 1.5})), [("order".into(), "type-mismatch")]);
    assert_eq!(issues("mesh.run", json!({"targets": [], "scheme": "map", "order": 2, "merge": true})), []);
    assert_eq!(issues("intent.set_size", json!({"ids": ["TopoFace#1"], "size": 0.0001})), [("size".into(), "out-of-range")]);
    // Optionals accept absence; null is not part of the params model and is rejected for non-nullable fields.
    assert_eq!(issues("model.create_brick", json!({"x": 1, "y": 1, "z": 1, "name": 5})), [("name".into(), "type-mismatch")]);
}

#[test]
fn batch_references_are_accepted_only_in_batch_mode_and_only_for_ids() {
    let d = doc("intent.set_size");
    let v = json!({"ids": ["$0.faces[*]", "$1.volume"], "size": 1});
    let mut a = Vec::new();
    check(d.view(), &v, true, &mut a);
    assert_eq!(a.len(), 2, "outside a batch references are not ids");
    let mut b = Vec::new();
    check_in_batch(d.view(), &v, true, &mut b);
    assert!(b.is_empty(), "{b:?}");
    let mut c = Vec::new();
    check_in_batch(d.view(), &json!({"ids": ["$0.faces[*]"], "size": "$0.x"}), true, &mut c);
    assert_eq!(c.len(), 1, "a reference is not a number");
}

#[test]
fn the_registry_batch_schema_accepts_what_the_checker_accepts() {
    let schema = SchemaDoc::new(registry().batch_schema().unwrap().as_value().clone());
    let good = json!({"ops": [
        {"name": "model.create_brick", "version": 1, "params": {"x": 1.0, "y": 1.0, "z": 1.0}},
        {"name": "intent.set_size", "version": 1, "params": {"ids": ["$0.faces[*]"], "size": 0.5}},
        {"name": "model.pick", "version": 1, "params": {"id": "$0.body"}},
        {"name": "model.note", "version": 1, "params": {"text": "$$0.body"}},
    ]});
    let mut out = Vec::new();
    check(schema.view(), &good, true, &mut out);
    assert!(out.is_empty(), "{out:?}");
    let bad = json!({"ops": [{"name": "model.pick", "version": 1, "params": {"id": "nonsense"}}]});
    let mut out = Vec::new();
    check(schema.view(), &bad, true, &mut out);
    assert!(!out.is_empty());
}

#[test]
fn query_relations_adapts_a_registered_query() {
    let engine = engine();
    let pre = "model.create_brick(x: 1, y: 1, z: 1)\nmodel.create_brick(x: 1, y: 1, z: 1)\n";
    let script = "let f = faces in Volume#2\nassert count(f) == 6\nassert count(volumes in f) == 1\nassert count(faces in Body#1) == 6\nintent.set_size(ids: f, size: 1)\n";
    let mut results = Vec::new();
    for use_query in [false, true] {
        let mut h = EngineHost::new(&engine, engine.registry().new_state());
        let rel = QueryRelations::new(&engine, "topo.related");
        h = if use_query { h.with_relations(rel) } else { h.with_relations(DemoRelations) };
        let rep = Interpreter::new(&mut h, Options::default()).run_source("q", &format!("{pre}{script}"));
        assert!(rep.ok(), "{:?}", rep.diagnostics);
        results.push(h.into_state().hash());
    }
    assert_eq!(results[0], results[1], "the query adapter and the direct resolver agree");
    // A failing query surfaces as `relation-failed`.
    let mut h = EngineHost::new(&engine, engine.registry().new_state()).with_relations(QueryRelations::new(&engine, "no.such_query"));
    let rep = Interpreter::new(&mut h, Options::default()).run_source("q", &format!("{pre}let f = faces in Volume#1"));
    assert_eq!(rep.diagnostics[0].code, "relation-failed");
}

#[test]
fn aliases_rewrite_names_but_never_shadow_the_registry() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("a", "alias brick = model.create_brick\nalias count_all = model.count\nlet b = brick(x: 1, y: 1, z: 1)\nlet c = count_all()\nassert c.count == 8\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(rep.records()[0].name, "model.create_brick", "records always carry the full registry name");
    let rep = it.run_source("b", "alias model.pick = model.note");
    assert!(rep.diagnostics[0].code == "expected-token" || rep.diagnostics[0].code == "alias-shadows");
    let rep = it.run_source("c", "alias sin = model.note");
    assert_eq!(rep.diagnostics[0].code, "alias-shadows");
}

#[test]
fn results_are_hashed_identically_when_built_by_script_or_by_hand() {
    // A script statement is exactly an `OpRecord` a typed client would build (canonical form).
    let engine = engine();
    let mut h = host(&engine);
    let rep = Interpreter::new(&mut h, Options::default()).run_source("s", "model.create_brick(x: 1, y: 2, z: 3, name: \"n\")");
    let by_hand = sm_op_api::OpRecord::of(&CreateBrick { x: 1.0, y: 2.0, z: 3.0, name: Some("n".into()) }).unwrap();
    assert_eq!(rep.records()[0], &by_hand);
}
