//! `Host::export_formats`: a host that adds a format needs no change in sm-lang.

mod common;

use serde_json::{Value as Json, json};
use sm_lang::{EngineHost, Host, HostError, Interpreter, Options, SessionCommand};
use sm_op_api::{Delta, OpRecord, Registry, State};

use common::*;

struct StepHost<'a> {
    inner: EngineHost<'a>,
    exported: Vec<SessionCommand>,
}

impl Host for StepHost<'_> {
    fn registry(&self) -> &Registry {
        self.inner.registry()
    }
    fn state(&self) -> &State {
        self.inner.state()
    }
    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError> {
        self.inner.apply(rec)
    }
    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError> {
        self.inner.query(name, params)
    }
    fn export_formats(&self) -> Vec<String> {
        vec!["smsh".into(), "step".into()]
    }
    fn session(&mut self, cmd: &SessionCommand) -> Result<Json, HostError> {
        self.exported.push(cmd.clone());
        Ok(json!({"ok": true}))
    }
}

fn run(src: &str) -> (Vec<SessionCommand>, sm_lang::RunReport) {
    let e = engine_w2();
    let mut h = StepHost { inner: EngineHost::new(&e, e.registry().new_state()), exported: Vec::new() };
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("t.smesh", src)
    };
    (h.exported, rep)
}

/// Acceptance 11: the host's list decides validation, the did-you-mean and help; the default list is unchanged.
#[test]
fn a_host_adds_an_export_format() {
    let (log, rep) = run("session.export(\"a.step\", format: \"step\")");
    assert!(rep.ok(), "{:?}", rep.diagnostics);
    assert!(matches!(&log[..], [SessionCommand::Export { format: Some(f), .. }] if f == "step"));

    // A format of the default list is rejected, with the host's list in the help.
    let (log, rep) = run("session.export(\"a.vtu\", format: \"vtu\")");
    assert!(log.is_empty());
    let d = rep.diagnostics.iter().find(|d| d.is_error()).unwrap();
    assert_eq!(d.code, "invalid-argument");
    assert_eq!(d.help.as_deref(), Some("formats: \"smsh\", \"step\""));
    // A misspelling gets the did-you-mean from the host's list.
    let (_, rep) = run("session.export(\"a.step\", format: \"stpe\")");
    let d = rep.diagnostics.iter().find(|d| d.is_error()).unwrap();
    assert!(d.help.as_deref().is_some_and(|h| h.contains("did you mean \"step\"")) || d.notes.iter().any(|(_, n)| n.contains("step")), "{d:?}");

    // `help session.export` lists the host's formats.
    let (_, rep) = run("help session.export");
    let text = rep.events.iter().find_map(|e| if let sm_lang::Event::Help { text, .. } = e { Some(text.clone()) } else { None }).unwrap();
    assert!(text.contains("\"smsh\" | \"step\"") && !text.contains("vtu"), "{text}");
}

#[test]
fn the_default_host_keeps_its_four_formats() {
    let e = engine_w2();
    let h = EngineHost::new(&e, e.registry().new_state());
    assert_eq!(h.export_formats(), ["smsh", "vtu", "exodus", "openfoam"]);
    let mut h = h;
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("t.smesh", "help session.export")
    };
    let text = rep.events.iter().find_map(|e| if let sm_lang::Event::Help { text, .. } = e { Some(text.clone()) } else { None }).unwrap();
    assert!(text.starts_with("session.export(path: string, format?: \"smsh\" | \"vtu\" | \"exodus\" | \"openfoam\", mesh?: id, provenance?: bool)"), "{text}");
}
