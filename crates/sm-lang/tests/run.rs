//! End-to-end: scripts run through the interpreter against demo ops.

mod common;

use sm_core::{EntityId, EntityKind};
use sm_lang::{Event, Interpreter, Options, PlanHost, Val};
use sm_lang::diag::render_all;

use common::*;

fn run(src: &str) -> (sm_lang::RunReport, sm_op_api::State, sm_lang::SourceMap) {
    let engine = engine();
    let mut h = host(&engine);
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source("t.smesh", src);
        (rep, it.sources().clone())
    };
    (rep, h.into_state(), map)
}

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).unwrap()
}

#[test]
fn a_script_builds_and_meshes_a_brick() {
    let (rep, state, map) = run(
        r#"
        // parametric brick
        let w = 10
        let b = model.create_brick(x: w, y: w, z: 2 * w, name: "slab")
        intent.set_size(ids: b.faces, size: w / 10)
        mesh.run(targets: b.volume, scheme: "map", order: 2)
        print count(b.faces), b.body
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(rep.statements, 5); // let, let, set_size, run, print
    let recs = rep.records();
    assert_eq!(recs.len(), 3);
    assert_eq!(recs[0].name, "model.create_brick");
    assert_eq!(recs[0].version, 1);
    assert_eq!(recs[0].params["z"], 20.0);
    assert_eq!(recs[0].params["x"], 10.0, "ints are written as floats where the schema says number");
    let m = state.layer::<DemoModel>().unwrap();
    assert_eq!(m.sizes.len(), 6);
    assert!(m.sizes.values().all(|s| *s == 1.0));
    assert_eq!(m.meshed.get(&id(EntityKind::TopoVolume, 1)).unwrap(), "Map/false/2");
    let Event::Print { text, .. } = rep.events.last().unwrap() else { panic!() };
    assert_eq!(text, "6 Body#1");
}

#[test]
fn entity_lists_resolve_against_the_current_state() {
    let (rep, state, map) = run(
        r#"
        model.create_brick(x: 1, y: 1, z: 1)
        model.create_brick(x: 2, y: 2, z: 2)
        model.create_brick(x: 3, y: 3, z: 3)
        // faces of volume 2 only, then everything but them
        intent.set_size(ids: faces in Volume#2, size: 0.5)
        intent.set_size(ids: all faces - faces in Volume#2, size: 2.0)
        mesh.run(targets: all volumes except Volume#3, scheme: "pave")
        let n = count(Face#1..6 | Face#[13, 14])
        assert n == 8, "union of a range and a list"
        assert count(faces in (Volume#1 | Volume#3)) == 12
        assert count(volumes in Face#7) == 1
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    let m = state.layer::<DemoModel>().unwrap();
    let f = |n| id(EntityKind::TopoFace, n);
    assert_eq!(m.sizes[&f(7)], 0.5);
    assert_eq!(m.sizes[&f(12)], 0.5);
    assert_eq!(m.sizes[&f(1)], 2.0);
    assert_eq!(m.sizes[&f(18)], 2.0);
    assert_eq!(m.meshed.len(), 2);
    assert!(!m.meshed.contains_key(&id(EntityKind::TopoVolume, 3)));
}

#[test]
fn arguments_normalize_aliases_and_canonicalize_records() {
    let (rep, _, map) = run(
        r#"
        let b = model.create_brick(x: 1, y: 1, z: 1)
        model.pick(Body#1)
        model.pick("Volume#1")
        model.scale_brick(id: b.body, factor: 2)
        model.delete_body(...{ id: "Body#1" })
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    let recs = rep.records();
    assert_eq!(recs[1].params["id"], "TopoBody#1", "ids are written in canonical kind spelling");
    assert_eq!(recs[2].params["id"], "TopoVolume#1");
    assert_eq!(recs[3].params["factor"], 2.0);
    assert_eq!(recs[4].params["id"], "TopoBody#1");
}

#[test]
fn control_flow_and_expressions() {
    let (rep, state, map) = run(
        r#"
        let total = 0
        for i in range(1, 4) {
            set total = total + i
            model.create_brick(x: i, y: 1, z: 1)
        }
        if total == 6 and not (total < 0) { model.note(text: "six") } else { model.note(text: "other") }
        for v in all volumes { intent.set_size(ids: faces in v, size: 1) }
        model.note(text: str(floor(7 / 2)) + "/" + str(7 % 4) + "/" + str(2 ** 10) + "/" + str(max(1, 2.5)))
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    let m = state.layer::<DemoModel>().unwrap();
    assert_eq!(m.bricks.len(), 3);
    assert_eq!(m.notes, ["six", "3/3/1024/2.5"]);
    assert_eq!(m.sizes.len(), 18);
}

#[test]
fn queries_return_values_and_are_never_recorded() {
    let (rep, _, map) = run(
        r#"
        model.create_brick(x: 1, y: 1, z: 1)
        let c = model.count(kind: "TopoFace")
        assert c.count == 6
        model.note(text: str(c.count))
        "#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, &map));
    assert_eq!(rep.records().len(), 2, "the query is not a record");
    assert!(rep.events.iter().any(|e| matches!(e, Event::Query { result: Some(_), .. })));
}

#[test]
fn variables_scoping_and_rebinding() {
    let (rep, _, map) = run("let a = 1\nlet a = 2\n");
    assert!(!rep.ok());
    assert_eq!(rep.diagnostics[0].code, "rebind");
    assert!(rep.diagnostics[0].render(&map).contains("first defined here"));
    let (rep, _, _) = run("for i in range(0, 2) { let t = i }\nprint t\n");
    assert_eq!(rep.diagnostics[0].code, "unknown-variable", "loop bodies have their own scope");
    let (rep, _, _) = run("set x = 1\n");
    assert_eq!(rep.diagnostics[0].code, "unknown-variable");
}

#[test]
fn the_state_persists_between_runs_of_one_interpreter() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    assert!(it.run_source("a", "let b = model.create_brick(x: 1, y: 1, z: 1)").ok());
    let rep = it.run_source("b", "model.pick(b.body)");
    assert!(rep.ok());
    assert!(it.bindings().iter().any(|(n, v)| *n == "b" && matches!(v, Val::Outcome(_))));
}

#[test]
fn includes_run_in_the_current_scope_and_cycles_are_errors() {
    let engine = engine();
    let mut h = sm_lang::EngineHost::new(&engine, engine.registry().new_state()).with_loader(|p| match p {
        "defs.smesh" => Ok("let size = 4\nmodel.create_brick(x: size, y: size, z: size)\n".to_owned()),
        "loop.smesh" => Ok("include \"loop.smesh\"\n".to_owned()),
        "bad.smesh" => Ok("let = 1\n".to_owned()),
        _ => Err(format!("{p}: not found")),
    });
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source("m", "include \"defs.smesh\"\nmodel.note(text: str(size))\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let rep = it.run_source("m2", "include \"loop.smesh\"");
    assert_eq!(rep.diagnostics[0].code, "include-cycle");
    let rep = it.run_source("m3", "include \"nope.smesh\"");
    assert_eq!(rep.diagnostics[0].code, "include-failed");
    let rep = it.run_source("m4", "include \"bad.smesh\"");
    assert_eq!(rep.diagnostics[0].code, "expected-token");
    assert!(it.sources().get(rep.diagnostics[0].span.file).unwrap().name == "bad.smesh", "spans point into the included file");
}

#[test]
fn limits_stop_runaway_loops() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options { max_loop_iterations: 100, ..Options::default() });
    let rep = it.run_source("l", "for i in range(0, 1000) { let x = i }");
    assert_eq!(rep.diagnostics[0].code, "loop-limit");
    let rep = it.run_source("r", "let r = range(0, 100000000)");
    assert_eq!(rep.diagnostics[0].code, "range-too-long");
}

#[test]
fn syntax_errors_prevent_any_execution() {
    let (rep, state, _) = run("model.create_brick(x: 1, y: 1, z: 1)\nlet = 3\n");
    assert!(!rep.ok());
    assert!(rep.events.is_empty());
    assert_eq!(state.layer::<DemoModel>().unwrap().bricks.len(), 0, "nothing ran");
}

#[test]
fn a_failing_op_stops_the_script_and_keeps_earlier_effects() {
    let (rep, state, map) = run("model.create_brick(x: 1, y: 1, z: 1)\nmodel.create_brick(x: -1, y: 1, z: 1)\nmodel.create_brick(x: 5, y: 5, z: 5)\n");
    assert_eq!(rep.diagnostics.len(), 1);
    assert_eq!(rep.diagnostics[0].code, "op-failed");
    assert!(rep.diagnostics[0].render(&map).contains("extents must be positive"));
    assert_eq!(state.layer::<DemoModel>().unwrap().bricks.len(), 1);
}

#[test]
fn plan_mode_checks_without_applying_and_marks_unknowns() {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source(
        "p",
        r#"
        let b = model.create_brick(x: 1, y: 1, z: 1)
        model.pick(Volume#9)
        intent.set_size(ids: b.faces, size: 1)
        let n = model.count()
        "#,
    );
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(rep.records().len(), 3);
    assert!(rep.diagnostics.iter().any(|d| d.code == "plan-unresolved"), "placeholder use is a warning");
    assert!(rep.events.iter().all(|e| !matches!(e, Event::Op { delta: Some(_), .. })));
    let r = rep.records()[2];
    assert_eq!(r.params["ids"][0], "$plan.1.faces", "placeholders can never match REF_PATTERN");
    assert!(!sm_op_api::batch::is_reference("$plan.1.faces"));
    // Arithmetic on an unknown result is an error, not a silent guess.
    let rep = it.run_source("q", "let q2 = model.count()\nlet m = q2.count + 1");
    assert_eq!(rep.diagnostics[0].code, "plan-unresolved");
}
