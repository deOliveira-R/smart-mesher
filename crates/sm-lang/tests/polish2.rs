//! Polish wave (W2): `x-entity-kind` on batch references and plan placeholders, distinct plan markers for
//! `session.*` / `timeline.head()`, plan-mode `session.export` with unknown arguments, mesh-id string vs literal.

mod common;

use serde_json::json;
use sm_lang::{Diagnostic, EngineHost, Event, Interpreter, Options, PlanHost, RunReport};

use common::*;

fn exec(src: &str) -> RunReport {
    let engine = engine_w2();
    let mut h =
        EngineHost::new(&engine, engine.registry().new_state()).with_relations(DemoRelations);
    Interpreter::new(&mut h, Options::default()).run_source("t.smesh", src)
}

fn plan(src: &str) -> RunReport {
    let engine = engine_w2();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    Interpreter::new(&mut h, Options::default()).run_source("t.smesh", src)
}

fn first_error(rep: &RunReport) -> &Diagnostic {
    rep.diagnostics
        .iter()
        .find(|d| d.is_error())
        .unwrap_or_else(|| panic!("ran without an error: {:?}", rep.diagnostics))
}

#[test]
fn batch_refs_are_kind_checked_against_the_producing_role() {
    for arg in ["c.faces[0]", "c.body"] {
        let rep = exec(&format!(
            "batch {{ let c = w2.make()\nw2.use_volume(vol: {arg}) }}\n"
        ));
        assert_eq!(first_error(&rep).code, "wrong-kind", "{arg}");
    }
    // The matching role still lowers, and arrays are checked too.
    let rep = exec("batch { let c = w2.make()\nw2.use_volume(vol: c.volume) }\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let rep =
        exec("batch { let c = w2.make()\nw2.use_volume(vol: c.volume, many: [c.faces[0]]) }\n");
    assert_eq!(first_error(&rep).code, "wrong-kind");
}

#[test]
fn plan_placeholders_are_kind_checked_against_the_producing_role() {
    for arg in ["c.faces[0]", "c.body"] {
        let rep = plan(&format!("let c = w2.make()\nw2.use_volume(vol: {arg})\n"));
        assert_eq!(first_error(&rep).code, "wrong-kind", "{arg}");
    }
    let rep = plan("let c = w2.make()\nw2.use_volume(vol: c.volume)\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
}

#[test]
fn plan_session_values_do_not_pose_as_an_op_output() {
    let rep = plan("let m = w2.make_mesh()\nlet h = session.hash()\nw2.use_volume(vol: h)\n");
    let r = rep.records();
    assert_eq!(r[1].params["vol"], json!("$plan.1.session.hash"));
    let rep = plan("let t = timeline.head()\nw2.use_volume(vol: t)\n");
    assert_eq!(
        rep.records()[0].params["vol"],
        json!("$plan.0.timeline.head")
    );
}

#[test]
fn plan_export_with_unknown_path_is_skipped_with_a_warning() {
    let src = "let p = session.hash()\nsession.export(path: p)\nsession.export(\"a.vtu\", format: p)\nsession.export(\"a.vtu\", provenance: p)\n";
    let rep = plan(src);
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let exports = rep
        .events
        .iter()
        .filter(|e| {
            matches!(
                e,
                Event::Session {
                    command: sm_lang::SessionCommand::Export { .. },
                    ..
                }
            )
        })
        .count();
    assert_eq!(exports, 0, "unknown arguments cannot be recorded");
    assert_eq!(
        rep.diagnostics
            .iter()
            .filter(|d| d.code == "plan-session")
            .count(),
        1 + 3,
        "hash plus three skipped exports"
    );
}

#[test]
fn a_mesh_id_string_and_literal_agree_on_liveness() {
    let lit = exec("session.export(\"a\", mesh: Mesh#99)\n");
    let st = exec("session.export(\"a\", mesh: \"Mesh#99\")\n");
    assert_eq!(first_error(&lit).code, "entity-not-found");
    assert_eq!(first_error(&st).code, "entity-not-found");
}
