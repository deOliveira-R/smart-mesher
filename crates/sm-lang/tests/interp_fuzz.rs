//! The interpreter never panics, whatever the script, in execute and plan mode, and is deterministic.

mod common;

use proptest::prelude::*;
use sm_lang::diag::render_all;
use sm_lang::{EngineHost, Interpreter, Options, PlanHost};
use sm_testkit::strategies::config;

use common::*;

const FRAGMENTS: &[&str] = &[
    "let ", "set ", "alias ", "batch ", "{", "}", "(", ")", "[", "]", ",", ":", "=", "...", "\n", "; ", " ", "if ", "else ", "for ", "in ", "print ", "assert ", "include ",
    "help ", "x", "y", "b", "n", "1", "2", "0", "-1", "0.5", "1e300", "\"s\"", "\"$0.body\"", "true", "false", "+", "-", "*", "/", "%", "**", "==", "<", "and ", "or ", "not ",
    "|", "&", "except ", "all ", "faces", "volumes", "bodies", "faces in ", "Volume#1", "Volume#1..9", "Face#[1, 3]", "Body#1", "Face#99", ".body", ".faces", "[0]", "[*]", "[1]",
    "model.create_brick(", "model.pick(", "model.note(", "model.count(", "model.delete_body(", "intent.set_size(", "mesh.run(", "model.scale_brick(", "core.batch(",
    "x: ", "y: ", "z: ", "id: ", "ids: ", "size: ", "text: ", "targets: ", "scheme: ", "order: ", "name: ", "range(0, 3)", "count(", "len(", "sqrt(", "str(", "min(", "ents(", "pi",
    "let b = model.create_brick(x: 1, y: 1, z: 1)\n", "batch {\n let p = model.create_brick(x: 1, y: 2, z: 3)\n model.pick(p.body)\n}\n",
];

fn small() -> Options {
    Options { max_loop_iterations: 200, max_range_len: 200, ..Options::default() }
}

fn run_exec(src: &str) -> String {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, small());
    let rep = it.run_source("f", src);
    for d in &rep.diagnostics {
        let n = src.len() as u32;
        assert!(d.span.hi >= d.span.lo && (d.span.file != 0 || d.span.hi <= n), "bad span {d:?}");
    }
    format!("{}|{}|{}", render_all(&rep.diagnostics, it.sources()), rep.events.len(), h.into_state().hash())
}

fn run_plan(src: &str) -> String {
    let engine = engine();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, small());
    let rep = it.run_source("f", src);
    format!("{}|{}", render_all(&rep.diagnostics, it.sources()), rep.events.len())
}

proptest! {
    #![proptest_config(config(1500))]

    #[test]
    fn token_soup_never_panics_and_is_deterministic(parts in proptest::collection::vec(0..FRAGMENTS.len(), 0..40)) {
        let src: String = parts.iter().map(|&i| FRAGMENTS[i]).collect();
        let a = run_exec(&src);
        prop_assert_eq!(&a, &run_exec(&src));
        let p = run_plan(&src);
        prop_assert_eq!(&p, &run_plan(&src));
    }

    #[test]
    fn arbitrary_text_never_panics(s in ".{0,120}") {
        let _ = run_exec(&s);
        let _ = run_plan(&s);
    }

    #[test]
    fn completion_never_panics_on_soup(parts in proptest::collection::vec(0..FRAGMENTS.len(), 0..30), cut in 0usize..400) {
        let src: String = parts.iter().map(|&i| FRAGMENTS[i]).collect();
        let engine = engine();
        let mut h = EngineHost::new(&engine, engine.registry().new_state()).with_relations(DemoRelations);
        let mut it = Interpreter::new(&mut h, small());
        let c = it.complete(&src, cut);
        prop_assert!(c.start <= c.end && c.end <= src.len());
    }
}

#[test]
fn core_batch_can_be_called_directly_like_any_op() {
    let engine = engine();
    let mut h = host(&engine);
    let mut it = Interpreter::new(&mut h, Options::default());
    let rep = it.run_source(
        "d",
        r#"core.batch(ops: [
            {name: "model.create_brick", version: 1, params: {x: 1.0, y: 1.0, z: 1.0}},
            {name: "model.pick", version: 1, params: {id: "$0.body"}},
        ])"#,
    );
    assert!(rep.ok(), "{}", render_all(&rep.diagnostics, it.sources()));
    assert_eq!(h.current().layer::<DemoModel>().unwrap().notes, ["TopoBody#1"]);
    let rep = Interpreter::new(&mut h, Options::default()).run_source("d2", "batch { core.batch(ops: []) }");
    assert!(!rep.ok());
}
