//! `batch { }` lowering to `core.batch`, reference forms, escapes, snapshot semantics.

mod common;

use serde_json::json;
use sm_lang::diag::render_all;
use sm_lang::schema::{SchemaDoc, check};
use sm_lang::{Event, Interpreter, Options, PlanHost};

use common::*;

fn run(src: &str) -> (sm_lang::RunReport, sm_op_api::State, sm_lang::SourceMap) {
    let engine = engine();
    let mut h = host(&engine);
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source("b.smesh", src);
        (rep, it.sources().clone())
    };
    (rep, h.into_state(), map)
}

#[test]
fn batch_lowers_to_one_core_batch_with_references() {
    let (rep, state, map) = run(
        r#"
        batch {
            let p = model.create_brick(x: 1, y: 1, z: 1)
            intent.set_size(ids: p.faces, size: 0.25)
            model.pick(p.body)
            model.pick(p.faces[2])
            intent.set_size(ids: [p.faces[0], p.faces[1]], size: 0.5)
            intent.set_size(ids: [p.faces[*], p.volume], size: 0.75)
            model.note(text: "$0.body")
            model.note(text: "plain")
        }
        model.pick(p.body)
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    let recs = rep.records();
    assert_eq!(recs.len(), 2, "one core.batch node plus the pick after it");
    let b = recs[0];
    assert_eq!(b.name, "core.batch");
    let ops = b.params["ops"].as_array().unwrap();
    assert_eq!(ops.len(), 8);
    for o in ops {
        assert_eq!(o["version"], 1, "every member carries its own version");
    }
    assert_eq!(ops[1]["params"]["ids"], json!(["$0.faces[*]"]), "array parameter: [*] only as an element");
    assert_eq!(ops[2]["params"]["id"], json!("$0.body"), "scalar parameter: bare form");
    assert_eq!(ops[3]["params"]["id"], json!("$0.faces[2]"));
    assert_eq!(ops[4]["params"]["ids"], json!(["$0.faces[0]", "$0.faces[1]"]));
    assert_eq!(ops[5]["params"]["ids"], json!(["$0.faces[*]", "$0.volume[*]"]));
    assert_eq!(ops[6]["params"]["text"], json!("$$0.body"), "a literal starting with $ is escaped");
    assert_eq!(ops[7]["params"]["text"], json!("plain"));
    // The lowered record validates against the registry's own batch schema (with our checker) ...
    let engine = engine();
    let schema = engine.registry().batch_schema().unwrap();
    let mut issues = Vec::new();
    check(SchemaDoc::new(schema.as_value().clone()).view(), &b.params, true, &mut issues);
    assert!(issues.is_empty(), "{issues:?}");
    // ... and applies: the literal `$0.body` came through unescaped, references resolved.
    let m = state.layer::<DemoModel>().unwrap();
    assert_eq!(m.notes[2], "$0.body");
    assert_eq!(m.notes[0], "TopoBody#1");
    assert_eq!(m.notes[1], "TopoFace#3");
    assert!(m.sizes.contains_key(&"TopoVolume#1".parse().unwrap()));
    // After the batch, `p` is concrete again, but only its named outputs are known.
    assert_eq!(m.notes.last().unwrap(), "TopoBody#1");
    let Event::Batch { members, delta, .. } = &rep.events[0] else { panic!() };
    assert_eq!(*members, 8);
    assert!(delta.is_some());
}

#[test]
fn batch_state_equals_sequential_state() {
    let seq = run("let p = model.create_brick(x: 1, y: 2, z: 3)\nintent.set_size(ids: p.faces, size: 0.25)\nmodel.pick(p.body)\n").1;
    let bat = run("batch {\nlet p = model.create_brick(x: 1, y: 2, z: 3)\nintent.set_size(ids: p.faces, size: 0.25)\nmodel.pick(p.body)\n}\n").1;
    assert_eq!(seq.hash(), bat.hash(), "a batch is the same state transition as its members in order");
}

#[test]
fn batch_snapshot_semantics_for_entity_lists() {
    let (rep, state, map) = run(
        r#"
        model.create_brick(x: 1, y: 1, z: 1)
        batch {
            model.create_brick(x: 2, y: 2, z: 2)
            intent.set_size(ids: all faces, size: 1)     // resolved at block entry: only the first brick's faces
        }
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(state.layer::<DemoModel>().unwrap().sizes.len(), 6);
}

#[test]
fn for_loops_unroll_into_batch_indices() {
    let (rep, state, map) = run(
        r#"
        batch {
            for i in range(1, 4) {
                let b = model.create_brick(x: i, y: 1, z: 1)
                model.pick(b.body)
            }
        }
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    let ops = rep.records()[0].params["ops"].as_array().unwrap().clone();
    assert_eq!(ops.len(), 6);
    assert_eq!(ops[1]["params"]["id"], "$0.body");
    assert_eq!(ops[3]["params"]["id"], "$2.body");
    assert_eq!(ops[5]["params"]["id"], "$4.body");
    assert_eq!(state.layer::<DemoModel>().unwrap().notes, ["TopoBody#1", "TopoBody#2", "TopoBody#3"]);
}

fn first_error(src: &str) -> sm_lang::Diagnostic {
    let (rep, _, _) = run(src);
    rep.diagnostics.into_iter().find(|d| d.is_error()).expect("an error")
}

#[test]
fn batch_misuse_is_diagnosed() {
    assert_eq!(first_error("batch { batch { } }").code, "nested-batch");
    assert_eq!(first_error("batch { model.count() }").code, "query-in-batch");
    assert_eq!(first_error("batch { include \"x\" }").code, "include-in-batch");
    // Arithmetic / sets / relations on symbolic values.
    let pre = "batch {\nlet p = model.create_brick(x: 1, y: 1, z: 1)\n";
    assert_eq!(first_error(&format!("{pre}let n = count(p.faces)\n}}")).code, "not-concrete");
    assert_eq!(first_error(&format!("{pre}model.pick(faces in p.volume)\n}}")).code, "not-concrete");
    assert_eq!(first_error(&format!("{pre}let n = p.created\n}}")).code, "not-concrete");
    assert_eq!(first_error(&format!("{pre}model.pick(p.faces[*])\n}}")).code, "type-mismatch");
    assert_eq!(first_error(&format!("{pre}model.note(text: p.body)\n}}")).code, "type-mismatch");
    assert_eq!(first_error(&format!("{pre}model.pick(p)\n}}")).code, "type-mismatch");
    // Explicit ids of entities the batch itself creates cannot exist yet.
    assert_eq!(first_error("batch { model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(Body#1) }").code, "entity-not-found");
    let (rep, _, _) = run("batch { }");
    assert!(rep.ok());
    assert_eq!(rep.diagnostics[0].code, "empty-batch");
}

#[test]
fn a_failing_batch_is_atomic() {
    let (rep, state, _) = run("model.create_brick(x: 1, y: 1, z: 1)\nbatch {\n model.create_brick(x: 2, y: 2, z: 2)\n model.create_brick(x: -1, y: 2, z: 2)\n}\n");
    assert_eq!(rep.diagnostics.last().unwrap().code, "op-failed");
    assert_eq!(state.layer::<DemoModel>().unwrap().bricks.len(), 1, "the whole batch had no effect");
}

#[test]
fn plan_mode_batches_stay_symbolic_and_old_refs_are_stale() {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("p", "batch {\nlet p = model.create_brick(x: 1, y: 1, z: 1)\nmodel.pick(p.body)\n}\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(rep.records()[0].name, "core.batch");
    let rep = it.run_source("q", "batch {\nmodel.pick(p.body)\n}\n");
    assert_eq!(rep.diagnostics[0].code, "stale-reference");
    let rep = it.run_source("r", "model.pick(p.body)\n");
    assert_eq!(rep.diagnostics[0].code, "stale-reference");
}
