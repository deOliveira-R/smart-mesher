//! Golden tests for semantic (run-time) diagnostics. Each `tests/golden/run/*.smesh` runs after a fixed prelude (two
//! bricks) through the demo registry; the rendered diagnostics are compared with the `.txt` next to it. File name
//! prefixes pick the host: `plan_` plans without executing, `norel_` has no relation resolver. Regenerate with `UPDATE_GOLDEN=1`.

mod common;

use std::fs;
use std::path::PathBuf;

use sm_lang::diag::render_all;
use sm_lang::{EngineHost, Interpreter, Options, PlanHost};

use common::*;

const PRELUDE: &str = "model.create_brick(x: 1, y: 1, z: 1)\nmodel.create_brick(x: 2, y: 2, z: 2)\n";

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/run")
}

fn render(name: &str, text: &str) -> String {
    let engine = engine();
    let mut base = host(&engine);
    {
        let mut it = Interpreter::new(&mut base, Options::default());
        assert!(it.run_source("prelude", PRELUDE).ok());
    }
    let state = base.current().clone();
    if name.starts_with("plan_") {
        let mut h = PlanHost::new(engine.registry(), state);
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source(name, text);
        render_all(&rep.diagnostics, it.sources())
    } else if name.starts_with("norel_") {
        let mut h = EngineHost::new(&engine, state);
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source(name, text);
        render_all(&rep.diagnostics, it.sources())
    } else {
        let mut h = EngineHost::new(&engine, state).with_relations(DemoRelations);
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source(name, text);
        render_all(&rep.diagnostics, it.sources())
    }
}

#[test]
fn semantic_diagnostics_match_golden_files() {
    let mut inputs: Vec<PathBuf> = fs::read_dir(dir()).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|e| e == "smesh")).collect();
    inputs.sort();
    assert!(inputs.len() >= 30, "golden inputs missing");
    let update = std::env::var_os("UPDATE_GOLDEN").is_some();
    let mut failures = Vec::new();
    for p in inputs {
        let text = fs::read_to_string(&p).unwrap();
        let name = p.file_name().unwrap().to_string_lossy().into_owned();
        let got = render(&name, &text);
        assert!(!got.is_empty(), "{name}: every golden input must produce a diagnostic");
        let exp = p.with_extension("txt");
        if update {
            fs::write(&exp, &got).unwrap();
            continue;
        }
        let want = fs::read_to_string(&exp).unwrap_or_default();
        if want != got {
            failures.push(format!("--- {name}\nexpected:\n{want}\ngot:\n{got}"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
