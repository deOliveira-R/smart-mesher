//! Shared helpers of the script-record tests: a session-backed host, timelines built by hand, record-and-replay checks.
#![allow(dead_code)]

use serde_json::{Value as Json, json};
use sm_lang::{Host, HostError, Interpreter, Options, RecordOptions, Recording, RunReport, SessionCommand, TimelineCommand, TimelineHead, record_script};
use sm_op_api::{Delta, Engine, Hash256, Head, Meta, OpRecord, QuietControl, Registry, Rev, Session, State, Timeline};

use crate::common::{DemoRelations, engine_script};

/// A `Session` as a `Host`: what a real front end does with an interpreter.
pub struct SessionHost {
    pub s: Session,
    rel: DemoRelations,
}

fn he(code: &str, message: String) -> HostError {
    HostError { code: code.to_owned(), message }
}

impl SessionHost {
    pub fn new(engine: &Engine) -> Self {
        Self { s: Session::new(engine.clone()), rel: DemoRelations }
    }
}

impl Host for SessionHost {
    fn registry(&self) -> &Registry {
        self.s.engine().registry()
    }
    fn state(&self) -> &State {
        self.s.state()
    }
    fn relations(&self) -> Option<&dyn sm_lang::Relations> {
        Some(&self.rel)
    }
    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError> {
        let q = QuietControl::new();
        self.s.apply_record(rec.clone(), Meta::new("test", "", ""), &q.control()).map(|(_, d)| d).map_err(|e| he(e.code(), e.to_string()))
    }
    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError> {
        let q = QuietControl::new();
        self.s.engine().run_query(self.s.state(), name, params, &q.control()).map_err(|e| he(e.code(), e.to_string()))
    }
    fn timeline(&mut self, cmd: &TimelineCommand) -> Result<(), HostError> {
        let q = QuietControl::new();
        let r = match cmd {
            TimelineCommand::Branch { name, .. } => self.s.branch(name),
            TimelineCommand::Checkout(rev) => self.s.checkout(rev.clone(), &q.control()),
            TimelineCommand::Undo => self.s.undo(),
            TimelineCommand::Redo => self.s.redo(),
            _ => return Err(he("timeline-unsupported", "unsupported".into())),
        };
        r.map_err(|e| he(e.code(), e.to_string()))
    }
    fn timeline_head(&mut self) -> Result<TimelineHead, HostError> {
        let b = match self.s.head() {
            Head::Ref(r) => Some(r.clone()),
            Head::Detached(_) => None,
        };
        Ok(TimelineHead::new(b, self.s.position().map(|h| h.to_hex())))
    }
    fn session(&mut self, cmd: &SessionCommand) -> Result<Json, HostError> {
        match cmd {
            SessionCommand::Hash => Ok(Json::String(self.s.state().hash().to_hex())),
            SessionCommand::Status => Ok(json!({"nodes": self.s.timeline().len()})),
            _ => Err(he("session-unsupported", "no session".into())),
        }
    }
}

pub fn run_text(engine: &Engine, text: &str) -> (SessionHost, RunReport) {
    let mut h = SessionHost::new(engine);
    let rep = {
        let mut it = Interpreter::new(&mut h, Options::default());
        it.run_source("t.smesh", text)
    };
    (h, rep)
}

pub fn rec(reg: &Registry, name: &str, params: Json) -> OpRecord {
    OpRecord::new(name, reg.op_version(name).unwrap(), params).unwrap()
}

pub fn meta() -> Meta {
    Meta::new("t", "", "")
}

pub fn apply_rec(s: &mut Session, r: OpRecord) -> Hash256 {
    let q = QuietControl::new();
    s.apply_record(r, meta(), &q.control()).unwrap().0
}

pub fn apply(s: &mut Session, name: &str, params: Json) -> Hash256 {
    let r = rec(s.engine().registry(), name, params);
    apply_rec(s, r)
}

pub fn note(s: &mut Session, text: &str) -> Hash256 {
    apply(s, "model.note", json!({ "text": text }))
}

pub fn checkout(s: &mut Session, rev: Rev) {
    let q = QuietControl::new();
    s.checkout(rev, &q.control()).unwrap();
}

/// The records and state hashes on the root path of `tip`, records migrated to the current op versions.
pub fn chain(reg: &Registry, t: &Timeline, tip: Hash256) -> Vec<(OpRecord, Hash256)> {
    t.path_to_root(tip)
        .unwrap()
        .into_iter()
        .map(|h| {
            let n = t.get(h).unwrap();
            let mut r = n.body.op.clone();
            if r.version != reg.op_version(&r.name).unwrap() {
                r = OpRecord::of(reg.decode(&r).unwrap().as_ref()).unwrap();
            }
            (r, n.check.state)
        })
        .collect()
}

/// Record `s`, replay the text on a fresh session and check every recorded ref against the source: same tip state
/// hash and the same record (and state hash) at every node of its root path. Returns the recording and the replay.
pub fn record_and_replay(engine: &Engine, s: &Session, o: &RecordOptions) -> (Recording, SessionHost) {
    let rec = record_script(engine.registry(), s.timeline(), s.head(), o).unwrap_or_else(|e| panic!("record: {e}"));
    let (h, rep) = run_text(engine, &rec.text);
    assert!(rep.diagnostics.is_empty(), "replay diagnostics {:?}\n{}", rep.diagnostics.iter().map(|d| (d.code, d.message.clone())).collect::<Vec<_>>(), rec.text);
    for (name, src_tip) in &rec.refs {
        let rt = *h.s.timeline().refs().get(name).unwrap_or_else(|| panic!("ref {name} missing after replay\n{}", rec.text));
        let a = chain(engine.registry(), s.timeline(), *src_tip);
        let b = chain(engine.registry(), h.s.timeline(), rt);
        assert_eq!(a, b, "ref {name}\n{}", rec.text);
    }
    (rec, h)
}

pub fn eng() -> Engine {
    engine_script()
}
