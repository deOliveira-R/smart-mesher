//! W2 delta (plan T9): `session.*` commands, `x-entity-kind` checking, static role checks, bare `One` roles in batches,
//! kind-aware completion. Golden diagnostics live in `tests/golden/w2` (`UPDATE_GOLDEN=1` regenerates).

mod common;

use std::fs;
use std::path::PathBuf;

use serde_json::{Value as Json, json};
use sm_core::EntityId;
use sm_lang::diag::render_all;
use sm_lang::{CandidateKind, EngineHost, Event, Host, HostError, Interpreter, Options, PlanHost, SessionCommand};
use sm_op_api::{Delta, Engine, OpRecord, Registry, State};

use common::*;

/// Wraps an [`EngineHost`] and records every session command with canned replies.
struct Recording<'a> {
    inner: EngineHost<'a>,
    log: Vec<SessionCommand>,
    fail_save: bool,
}

impl<'a> Recording<'a> {
    fn new(engine: &'a Engine) -> Self {
        Self { inner: EngineHost::new(engine, engine.registry().new_state()).with_relations(DemoRelations), log: Vec::new(), fail_save: false }
    }
}

impl Host for Recording<'_> {
    fn registry(&self) -> &Registry {
        self.inner.registry()
    }
    fn state(&self) -> &State {
        self.inner.state()
    }
    fn relations(&self) -> Option<&dyn sm_lang::Relations> {
        self.inner.relations()
    }
    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError> {
        self.inner.apply(rec)
    }
    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError> {
        self.inner.query(name, params)
    }
    fn session(&mut self, cmd: &SessionCommand) -> Result<Json, HostError> {
        self.log.push(cmd.clone());
        match cmd {
            SessionCommand::Hash => Ok(json!("ab".repeat(32))),
            SessionCommand::Status => Ok(json!({"branch": "main", "nodes": 3})),
            SessionCommand::Save if self.fail_save => Err(HostError { code: "io".into(), message: "disk full".into() }),
            _ => Ok(json!({"ok": true})),
        }
    }
}

fn mesh(n: u64) -> EntityId {
    format!("Mesh#{n}").parse().unwrap()
}

fn run_rec(src: &str) -> (Vec<SessionCommand>, sm_lang::RunReport) {
    let engine = engine_w2();
    let mut h = Recording::new(&engine);
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("t.smesh", src)
    };
    (h.log, rep)
}

fn first_error(src: &str) -> sm_lang::Diagnostic {
    let (_, rep) = run_rec(src);
    rep.diagnostics.into_iter().find(|d| d.is_error()).unwrap_or_else(|| panic!("`{src}` ran without an error"))
}

// ---- session.* reaches the host with exact values ------------------------------------------------

#[test]
fn session_commands_reach_the_host_with_exact_values() {
    let src = "let m = w2.make_mesh()\n\
        session.export(\"a.vtu\")\n\
        session.export(\"b.smsh\", format: \"smsh\", mesh: m.mesh, provenance: true)\n\
        session.export(path: \"c.vtu\", mesh: \"Mesh#1\")\n\
        session.save()\n";
    let (log, rep) = run_rec(src);
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(
        log,
        vec![
            SessionCommand::Export { path: "a.vtu".into(), format: None, mesh: None, provenance: false },
            SessionCommand::Export { path: "b.smsh".into(), format: Some("smsh".into()), mesh: Some(mesh(1)), provenance: true },
            SessionCommand::Export { path: "c.vtu".into(), format: None, mesh: Some(mesh(1)), provenance: false },
            SessionCommand::Save,
        ]
    );
    let events: Vec<_> = rep.events.iter().filter_map(|e| if let Event::Session { command, applied, result, .. } = e { Some((command.verb(), *applied, result.clone())) } else { None }).collect();
    assert_eq!(events.len(), 4);
    assert!(events.iter().all(|(_, applied, res)| *applied && res.as_ref() == Some(&json!({"ok": true}))));
}

#[test]
fn session_hash_and_status_are_expressions() {
    let src = "let h = session.hash()\nprint h\nlet s = session.status()\nprint s.branch, s.nodes\nassert session.hash() == h\nassert len(session.hash()) == 64\n";
    let (log, rep) = run_rec(src);
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert_eq!(log, [SessionCommand::Hash, SessionCommand::Status, SessionCommand::Hash, SessionCommand::Hash]);
    let texts: Vec<_> = rep.events.iter().filter_map(|e| if let Event::Print { text, .. } = e { Some(text.clone()) } else { None }).collect();
    assert_eq!(texts, ["ab".repeat(32), "main 3".to_owned()]);
}

#[test]
fn a_host_error_is_session_failed_and_keeps_the_run_going() {
    let engine = engine_w2();
    let mut h = Recording::new(&engine);
    h.fail_save = true;
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("t.smesh", "session.save()\nprint 1\n")
    };
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!(d.code, "session-failed");
    assert!(d.message.contains("[io]: disk full"), "{}", d.message);
}

#[test]
fn a_host_without_a_session_reports_unsupported() {
    let engine = engine_w2();
    let mut h = EngineHost::new(&engine, engine.registry().new_state());
    let rep = Interpreter::new(&mut h, Options::default()).run_source("t.smesh", "session.save()\n");
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!(d.code, "session-failed");
    assert!(d.message.contains("session-unsupported"), "{}", d.message);
}

#[test]
fn session_in_batch_is_an_error_for_every_verb() {
    for call in ["session.save()", "session.export(\"a.vtu\")", "session.hash()", "session.status()"] {
        let d = first_error(&format!("batch {{\n{call}\n}}\n"));
        assert_eq!(d.code, "session-in-batch", "{call}");
    }
}

#[test]
fn statements_cannot_be_used_as_values_but_queries_can() {
    assert_eq!(first_error("let x = session.save()\n").code, "op-in-expression");
    assert_eq!(first_error("print session.export(\"a.vtu\")\n").code, "op-in-expression");
    let (_, rep) = run_rec("let x = session.hash()\n");
    assert!(rep.ok());
}

#[test]
fn bad_export_arguments_are_diagnosed_at_the_argument() {
    assert_eq!(first_error("session.export(format: \"vtu\")\n").code, "missing-argument");
    assert_eq!(first_error("session.export(3)\n").code, "type-mismatch");
    assert_eq!(first_error("session.export(\"\")\n").code, "type-mismatch");
    let d = first_error("session.export(\"a\", format: \"vut\")\n");
    assert_eq!(d.code, "invalid-argument");
    assert_eq!(d.help.as_deref().map(|h| h.contains("did you mean \"vtu\"")), Some(true), "{:?}", d.help);
    assert_eq!(first_error("session.export(\"a\", provenance: 1)\n").code, "type-mismatch");
    assert_eq!(first_error("model.create_brick(x: 1, y: 1, z: 1)\nsession.export(\"a\", mesh: Volume#1)\n").code, "wrong-kind");
    assert_eq!(first_error("session.export(\"a\", nope: 1)\n").code, "unknown-argument");
    assert_eq!(first_error("session.explode()\n").code, "unknown-name");
    assert_eq!(first_error("session.save(1)\n").code, "positional-ambiguous");
}

#[test]
fn plan_mode_warns_and_never_calls_the_host() {
    let engine = engine_w2();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let rep = Interpreter::new(&mut h, Options::default()).run_source("t.smesh", "let m = w2.make_mesh()\nsession.export(\"a.vtu\", mesh: m.mesh)\nlet h = session.hash()\nsession.save()\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    let warns: Vec<_> = rep.diagnostics.iter().filter(|d| d.code == "plan-session").collect();
    assert_eq!(warns.len(), 3);
    let evs: Vec<_> = rep.events.iter().filter_map(|e| if let Event::Session { command, applied, result, .. } = e { Some((command.clone(), *applied, result.is_some())) } else { None }).collect();
    assert_eq!(evs[0], (SessionCommand::Export { path: "a.vtu".into(), format: None, mesh: None, provenance: false }, false, false));
    assert!(evs.iter().all(|(_, a, r)| !a && !r));
}

// ---- x-entity-kind ---------------------------------------------------------------------------------

#[test]
fn the_annotation_sits_where_the_checker_looks() {
    let r = registry_w2();
    let info = r.catalog().into_iter().flatten().find(|i| i.name == "w2.use_volume").unwrap();
    let schema = info.schema.as_value();
    let props = &schema["properties"];
    // On the property node of a plain id, the Option wrapper and the array node, never needing the item schema.
    assert_eq!(props["vol"]["x-entity-kind"], json!(["TopoVolume"]), "{schema}");
    assert!(props["maybe"].to_string().contains("x-entity-kind"), "{schema}");
    assert!(props["many"].to_string().contains("x-entity-kind"), "{schema}");
}

#[test]
fn a_face_given_for_a_volume_is_wrong_kind_at_the_argument() {
    let engine = engine_w2();
    let mut h = Recording::new(&engine);
    let src = "let b = w2.make()\nw2.use_volume(vol: b.faces[0])\n";
    let (rep, map) = {
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source("t.smesh", src);
        (rep, it.sources().clone())
    };
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!(d.code, "wrong-kind");
    let (lo, hi) = (d.span.lo as usize, d.span.hi as usize);
    assert_eq!(&src[lo..hi], "b.faces[0]", "{}", render_all(&rep.diagnostics, &map));
    assert!(d.message.contains("TopoVolume"), "{}", d.message);
    // Nothing was applied by the failing call.
    assert_eq!(h.state().layer::<DemoModel>().map(|m| m.notes.len()), Some(0));
}

#[test]
fn wrong_kind_covers_options_arrays_strings_and_sets() {
    assert_eq!(first_error("let b = w2.make()\nw2.use_volume(vol: b.volume, maybe: b.faces[0])\n").code, "wrong-kind");
    assert_eq!(first_error("let b = w2.make()\nw2.use_volume(vol: b.volume, many: [b.volume, b.faces[1]])\n").code, "wrong-kind");
    assert_eq!(first_error("let b = w2.make()\nw2.use_volume(vol: b.volume, many: all faces)\n").code, "wrong-kind");
    assert_eq!(first_error("let b = w2.make()\nw2.use_volume(vol: \"TopoFace#1\")\n").code, "wrong-kind");
    // The right kinds pass, including either of several allowed kinds.
    let (_, rep) = run_rec("let b = w2.make()\nw2.use_volume(vol: b.volume, maybe: b.volume, many: [b.volume, b.body])\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
}

#[test]
fn unannotated_parameters_are_unchanged() {
    // `model.pick` takes any entity; the demo registry never annotated it.
    let (_, rep) = run_rec("let b = w2.make()\nmodel.pick(b.faces[0])\nmodel.pick(b.volume)\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
}

// ---- static role checks ----------------------------------------------------------------------------

#[test]
fn a_misspelled_role_gets_did_you_mean_in_batch_and_plan_mode() {
    let d = first_error("batch {\nlet b = w2.make()\nw2.use_volume(vol: b.voluem)\n}\n");
    assert_eq!(d.code, "unknown-field");
    assert_eq!(d.help.as_deref(), Some("did you mean `volume`?"));

    let engine = engine_w2();
    let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
    let rep = Interpreter::new(&mut h, Options::default()).run_source("t.smesh", "let b = w2.make()\nw2.use_volume(vol: b.voluem)\n");
    let d = rep.diagnostics.iter().find(|d| d.is_error()).expect("error");
    assert_eq!((d.code, d.help.as_deref()), ("unknown-field", Some("did you mean `volume`?")));

    // Plan mode still accepts `created`/`removed`; ops that declare no roles stay unchecked.
    let rep = Interpreter::new(&mut h, Options::default()).run_source("t.smesh", "let b = w2.make()\nlet c = model.create_brick(x: 1, y: 1, z: 1)\nprint b.created, c.anything\n");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
}

#[test]
fn a_bare_one_role_in_a_batch_equals_the_top_level_form() {
    let bare = "batch {\nlet b = w2.make()\nw2.use_volume(vol: b.volume)\n}\n";
    let idx = "batch {\nlet b = w2.make()\nw2.use_volume(vol: b.volume[0])\n}\n";
    let top = "let b = w2.make()\nw2.use_volume(vol: b.volume)\n";
    let engine = engine_w2();
    let run = |src: &str| {
        let mut h = EngineHost::new(&engine, engine.registry().new_state());
        let rep = Interpreter::new(&mut h, Options::default()).run_source("t.smesh", src);
        assert!(rep.ok(), "{src}: {:?}", rep.diagnostics);
        let batch = rep.events.iter().find_map(|e| if let Event::Batch { record, .. } = e { Some(record.clone()) } else { None });
        (batch, h.into_state().hash())
    };
    let (bare_rec, bare_hash) = run(bare);
    let (idx_rec, _) = run(idx);
    let (_, top_hash) = run(top);
    assert_eq!(bare_rec.as_ref().map(|r| r.params.to_string()), idx_rec.as_ref().map(|r| r.params.to_string()));
    assert_eq!(bare_rec.unwrap().params, idx_rec.unwrap().params);
    assert_eq!(bare_hash, top_hash);
}

#[test]
fn a_bare_many_role_in_a_batch_is_still_an_error() {
    let d = first_error("batch {\nlet b = w2.make()\nmodel.pick(b.faces)\n}\n");
    assert_eq!(d.code, "expected-one-entity");
}

// ---- completion ---------------------------------------------------------------------------------------

fn complete(src: &str) -> Vec<(CandidateKind, String)> {
    let engine = engine_w2();
    let mut h = EngineHost::new(&engine, engine.registry().new_state());
    let mut it = Interpreter::new(&mut h, Options::default());
    let _ = it.run_source("pre", "let b = w2.make()\nlet k = w2.make_mesh()\n");
    let c = it.complete(src, src.len());
    c.items.into_iter().map(|c| (c.kind, c.label)).collect()
}

#[test]
fn command_names_complete() {
    let items = complete("session.");
    let labels: Vec<_> = items.iter().map(|(_, l)| l.as_str()).collect();
    assert_eq!(labels, ["session.export", "session.hash", "session.save", "session.status"]);
    let labels: Vec<_> = complete("timeline.").into_iter().map(|(_, l)| l).collect();
    assert!(labels.contains(&"timeline.branch".to_owned()) && labels.contains(&"timeline.head".to_owned()), "{labels:?}");
    let params: Vec<_> = complete("session.export(").into_iter().map(|(_, l)| l).collect();
    assert_eq!(params, ["path", "format", "mesh", "provenance"]);
    let fmt: Vec<_> = complete("session.export(\"a\", format: ").into_iter().filter(|(k, _)| *k == CandidateKind::Value).map(|(_, l)| l).collect();
    assert_eq!(fmt, ["\"smsh\"", "\"vtu\""]);
}

#[test]
fn completion_is_kind_aware() {
    let items = complete("w2.use_volume(vol: ");
    let kinds: Vec<_> = items.iter().filter(|(k, _)| *k == CandidateKind::Kind).map(|(_, l)| l.as_str()).collect();
    assert!(kinds.contains(&"Volume#") && kinds.contains(&"volumes"), "{kinds:?}");
    assert!(!kinds.contains(&"Face#") && !kinds.contains(&"faces"), "{kinds:?}");
    // `Mesh#1` is not a volume; the mesh variable `k` holds an op result (kept), entity-set variables of other kinds are dropped.
    let items = complete("let f = Face#1\nw2.use_volume(vol: ");
    assert!(items.iter().all(|(_, l)| l != "f"), "{items:?}");
    let items = complete("session.export(\"a\", mesh: ");
    let kinds: Vec<_> = items.iter().filter(|(k, _)| *k == CandidateKind::Kind).map(|(_, l)| l.as_str()).collect();
    assert_eq!(kinds, ["Mesh#", "meshes"]);
}

// ---- golden diagnostics ---------------------------------------------------------------------------------

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/w2")
}

fn render_golden(name: &str, text: &str) -> String {
    let engine = engine_w2();
    if name.starts_with("plan_") {
        let mut h = PlanHost::new(engine.registry(), engine.registry().new_state());
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source(name, text);
        render_all(&rep.diagnostics, it.sources())
    } else {
        let mut h = EngineHost::new(&engine, engine.registry().new_state());
        let mut it = Interpreter::new(&mut h, Options::default());
        let rep = it.run_source(name, text);
        render_all(&rep.diagnostics, it.sources())
    }
}

#[test]
fn new_diagnostics_match_golden_files() {
    let mut inputs: Vec<PathBuf> = fs::read_dir(golden_dir()).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|e| e == "smesh")).collect();
    inputs.sort();
    assert!(inputs.len() >= 10, "golden inputs missing");
    let update = std::env::var_os("UPDATE_GOLDEN").is_some();
    let mut failures = Vec::new();
    for p in inputs {
        let text = fs::read_to_string(&p).unwrap();
        let name = p.file_name().unwrap().to_string_lossy().into_owned();
        let got = render_golden(&name, &text);
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

// ---- determinism --------------------------------------------------------------------------------------------

#[test]
fn session_scripts_are_deterministic() {
    let src = "let m = w2.make_mesh()\nsession.export(\"a.vtu\", mesh: m.mesh)\nprint session.hash()\nsession.save()\n";
    let digest = || {
        let (log, rep) = run_rec(src);
        format!("{log:?}|{:?}|{:?}", rep.events.iter().map(|e| format!("{e:?}")).collect::<Vec<_>>(), rep.diagnostics)
    };
    assert_eq!(digest(), digest());
}
