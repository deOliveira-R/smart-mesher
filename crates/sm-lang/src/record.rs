//! Script record: a timeline as a script that rebuilds it on an empty project.
//!
//! [`record_script`] is a pure function of `(Registry, Timeline, Head, RecordOptions)`: no I/O, no clock, no node
//! hashes, authors, times or messages in the text, so it is byte-stable. [`verify_recording`] runs the text in a fresh
//! session and compares the result with the source timeline.
//!
//! What a script can say: ops (one statement per node, `batch { }` for `core.batch`), `timeline.branch`,
//! `timeline.checkout` and `assert session.hash() == "<hex>"` checks. It never says `undo`, `redo` or a node hash.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value as Json;
use sm_op_api::{Engine, Hash256, Head, Meta, OpRecord, QuietControl, Registry, Session, State, Timeline, TimelineError};

use crate::host::{Host, HostError, SessionCommand, TimelineCommand, TimelineHead};
use crate::interp::{Interpreter, Options};
use crate::parser::{ParseLimits, parse_with};
use crate::print::{PrintError, PrintNote, Printer, Style, string_lit};

/// Which part of the timeline is recorded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum Forks {
    /// The path from the root to the head's node. Replay starts on an unborn `main`, so a head on another branch `r`
    /// ends with `timeline.branch("r")` and `main` also exists at the tip afterwards (documented, Q12).
    #[default]
    Head,
    /// Every ref, as a tree of their root paths. Nodes reachable from no ref (undone work) are not recorded.
    Refs,
}

/// Where `assert session.hash() == "<hex>"` lines go.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum Check {
    /// No asserts.
    None,
    /// After the last statement of every recorded ref tip.
    #[default]
    Tips,
    /// After every statement.
    Every,
}

/// What [`record_script`] does.
///
/// The struct is `#[non_exhaustive]`: build it from [`Default`] and the `with_*` methods.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RecordOptions {
    /// Which nodes (default [`Forks::Head`]).
    pub forks: Forks,
    /// Where the hash checks go (default [`Check::Tips`]).
    pub check: Check,
    /// How ops are written (default [`Style::Readable`]).
    pub style: Style,
    /// Print a stored bare reference (`"$0.body"`) as `[0]` instead of failing with [`PrintError::BareReference`].
    pub normalize_refs: bool,
    /// Write the comment header (default true).
    pub header: bool,
}

impl Default for RecordOptions {
    fn default() -> Self {
        Self { forks: Forks::Head, check: Check::Tips, style: Style::Readable, normalize_refs: false, header: true }
    }
}

impl RecordOptions {
    /// Set [`forks`](Self::forks).
    #[must_use]
    pub fn with_forks(mut self, f: Forks) -> Self {
        self.forks = f;
        self
    }
    /// Set [`check`](Self::check).
    #[must_use]
    pub fn with_check(mut self, c: Check) -> Self {
        self.check = c;
        self
    }
    /// Set [`style`](Self::style).
    #[must_use]
    pub fn with_style(mut self, s: Style) -> Self {
        self.style = s;
        self
    }
    /// Set [`normalize_refs`](Self::normalize_refs).
    #[must_use]
    pub fn with_normalize_refs(mut self, b: bool) -> Self {
        self.normalize_refs = b;
        self
    }
    /// Set [`header`](Self::header).
    #[must_use]
    pub fn with_header(mut self, b: bool) -> Self {
        self.header = b;
        self
    }
}

/// Something the replay project must hold because the script cannot carry it.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum Requirement {
    /// A staged input (`source: { sha256 }` in an op's params), identified by its content hash.
    Input {
        /// Lowercase hex SHA-256 of the file.
        sha256: String,
    },
}

/// Something worth telling the caller about a recording.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RecordNote {
    /// The readable form did not read back to the record, so the statement is written `Raw`.
    RawFallback {
        /// The node.
        node: Hash256,
        /// Why.
        reason: String,
    },
    /// Bare batch references were written as `[0]` (`normalize_refs`).
    Normalized {
        /// The node.
        node: Hash256,
        /// How many references.
        count: usize,
    },
    /// The node was stored with an older op version and is written from its migrated (current) form.
    Migrated {
        /// The node.
        node: Hash256,
        /// Stored version.
        from: u32,
        /// Current version.
        to: u32,
    },
    /// Nodes reachable from no recorded ref (undone work) are not in the script.
    Orphans {
        /// How many.
        count: usize,
    },
    /// A string longer than the language's `Options::max_value_len`; stage the input instead of inlining it.
    LargeValue {
        /// The node.
        node: Hash256,
        /// Its length in bytes.
        bytes: usize,
    },
}

impl std::fmt::Display for RecordNote {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::RawFallback { node, reason } => write!(f, "node {node}: written in raw style: {reason}"),
            Self::Normalized { node, count } => write!(f, "node {node}: {count} bare reference(s) written as `[0]`"),
            Self::Migrated { node, from, to } => write!(f, "node {node}: migrated from op version {from} to {to}"),
            Self::Orphans { count } => write!(f, "{count} node(s) are reachable from no ref and are not recorded"),
            Self::LargeValue { node, bytes } => write!(f, "node {node}: a string of {bytes} bytes is inlined; stage the input (`session.stage`) and pass its sha256 instead"),
        }
    }
}

/// A recorded script.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Recording {
    /// The script text.
    pub text: String,
    /// Timeline nodes written.
    pub nodes: usize,
    /// Top-level statements written (ops, batches, timeline commands and asserts; comments are not statements).
    pub statements: usize,
    /// The source tip of every recorded ref.
    pub refs: BTreeMap<String, Hash256>,
    /// What the replay project must hold.
    pub requires: Vec<Requirement>,
    /// Things to know.
    pub notes: Vec<RecordNote>,
    /// The node each top-level statement belongs to (`None` for a timeline command), in statement order.
    pub statement_nodes: Vec<Option<Hash256>>,
}

/// Why a script could not be recorded or verified.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum RecordError {
    /// A record has no source form.
    #[error("{0}")]
    Print(#[from] PrintError),
    /// The timeline is inconsistent (a ref or head names a missing node).
    #[error("timeline: {0}")]
    Timeline(String),
    /// A stored record could not be migrated to the current op version.
    #[error("node {node}: cannot migrate `{name}` from version {from}: {message}")]
    Migrate {
        /// The node.
        node: Hash256,
        /// The op.
        name: String,
        /// Its stored version.
        from: u32,
        /// Why.
        message: String,
    },
    /// The script is longer than the parser accepts.
    #[error("the script is {bytes} bytes, the parser accepts {limit}")]
    TooLarge {
        /// Script size.
        bytes: usize,
        /// The parser's limit.
        limit: usize,
    },
    /// Running the script failed.
    #[error("statement {statement}{}: [{code}] {message}{}", node_text(*.node), hint_text(.hint))]
    Replay {
        /// 1-based number of the failing top-level statement.
        statement: usize,
        /// The source node that statement records, when it records one.
        node: Option<Hash256>,
        /// Diagnostic code.
        code: String,
        /// Message.
        message: String,
        /// A suggestion.
        hint: Option<String>,
        /// Timeline nodes the replay had created before the failure (the failing statement had no effect).
        nodes_applied: usize,
    },
}

fn node_text(n: Option<Hash256>) -> String {
    n.map(|n| format!(" (node {n})")).unwrap_or_default()
}

fn hint_text(h: &Option<String>) -> String {
    h.as_ref().map(|h| format!(" (hint: {h})")).unwrap_or_default()
}

// ---- recording ----------------------------------------------------------------------------------

struct Em<'a> {
    reg: &'a Registry,
    tl: &'a Timeline,
    o: &'a RecordOptions,
    pr: Printer<'a>,
    body: String,
    stmt_nodes: Vec<Option<Hash256>>,
    notes: Vec<RecordNote>,
    requires: BTreeSet<String>,
    ordinal: usize,
    nodes: usize,
    max_value_len: usize,
}

/// A record at the registry's current version (decoded and re-encoded when it is older).
fn migrated(reg: &Registry, rec: &OpRecord) -> Result<Option<OpRecord>, String> {
    let cur = reg.op_version(&rec.name).map_err(|e| e.to_string())?;
    if rec.version == cur {
        return Ok(None);
    }
    let op = reg.decode(rec).map_err(|e| e.to_string())?;
    OpRecord::of(op.as_ref()).map(Some).map_err(|e| e.to_string())
}

impl<'a> Em<'a> {
    fn new(reg: &'a Registry, tl: &'a Timeline, o: &'a RecordOptions) -> Self {
        Self {
            reg,
            tl,
            o,
            pr: Printer::new(reg),
            body: String::new(),
            stmt_nodes: Vec::new(),
            notes: Vec::new(),
            requires: BTreeSet::new(),
            ordinal: 0,
            nodes: 0,
            max_value_len: Options::default().max_value_len,
        }
    }

    fn op(&mut self, h: Hash256) -> Result<(), RecordError> {
        let node = self.tl.get(h).ok_or_else(|| RecordError::Timeline(format!("unknown node {h}")))?;
        let mut rec = node.body.op.clone();
        match migrated(self.reg, &rec) {
            Ok(None) => {}
            Ok(Some(m)) => {
                self.notes.push(RecordNote::Migrated { node: h, from: rec.version, to: m.version });
                rec = m;
            }
            Err(message) if self.reg.op_version(&rec.name).is_err() => {
                let _ = message;
                return Err(PrintError::UnknownOp { node: Some(h), name: rec.name }.into());
            }
            Err(message) => return Err(RecordError::Migrate { node: h, name: rec.name, from: rec.version, message }),
        }
        self.scan(h, &rec.params);
        let printed = self.pr.print(&rec, self.o.style, self.o.normalize_refs, self.ordinal).map_err(|e| e.at_node(h))?;
        if rec.name == "core.batch" {
            self.ordinal += 1;
        }
        for n in printed.notes {
            self.notes.push(match n {
                PrintNote::RawFallback(reason) => RecordNote::RawFallback { node: h, reason },
                PrintNote::Normalized(count) => RecordNote::Normalized { node: h, count },
            });
        }
        self.body.push_str(&printed.text);
        self.body.push('\n');
        self.stmt_nodes.push(Some(h));
        self.nodes += 1;
        if self.o.check == Check::Every {
            self.assert(h);
        }
        Ok(())
    }

    /// `assert session.hash() == "<state hash after node h>"`.
    fn assert(&mut self, h: Hash256) {
        if let Some(n) = self.tl.get(h) {
            self.body.push_str(&format!("assert session.hash() == \"{}\"\n", n.check.state.to_hex()));
            self.stmt_nodes.push(Some(h));
        }
    }

    fn cmd(&mut self, text: String) {
        self.body.push_str(&text);
        self.body.push('\n');
        self.stmt_nodes.push(None);
    }

    fn branch(&mut self, name: &str) {
        self.cmd(format!("timeline.branch({})", string_lit(name)));
    }

    fn checkout(&mut self, name: &str) {
        self.cmd(format!("timeline.checkout({})", string_lit(name)));
    }

    /// Collect staged inputs and note oversized strings.
    fn scan(&mut self, node: Hash256, j: &Json) {
        match j {
            Json::String(s) if s.len() > self.max_value_len => self.notes.push(RecordNote::LargeValue { node, bytes: s.len() }),
            Json::Array(a) => a.iter().for_each(|x| self.scan(node, x)),
            Json::Object(m) => {
                if let Some(Json::Object(src)) = m.get("source")
                    && let Some(Json::String(h)) = src.get("sha256")
                {
                    self.requires.insert(h.clone());
                }
                m.values().for_each(|x| self.scan(node, x));
            }
            _ => {}
        }
    }
}

/// Record `tl` (as seen from `head`) as a script.
///
/// With [`Forks::Head`] the script is the ops on the path from the root to the head's node. With [`Forks::Refs`] it is
/// every ref's root path as a tree: the script starts detached on the empty state, creates each ref with
/// `timeline.branch` where it is reached, and returns to a fork point through a ref that still sits there. The text is a
/// pure function of the timeline (children are visited in node-hash order), so equal inputs give equal bytes.
///
/// Errors: a record that has no source form ([`PrintError`], naming the node), an inconsistent timeline, or a script
/// longer than the parser's limit.
pub fn record_script(reg: &Registry, tl: &Timeline, head: &Head, o: &RecordOptions) -> Result<Recording, RecordError> {
    let mut em = Em::new(reg, tl, o);
    let mut refs: BTreeMap<String, Hash256> = BTreeMap::new();
    let head_text;
    match o.forks {
        Forks::Head => {
            let (tip, head_ref) = match head {
                Head::Ref(r) => (tl.refs().get(r).copied(), Some(r.as_str())),
                Head::Detached(h) => (*h, None),
            };
            head_text = match head {
                Head::Ref(r) => r.clone(),
                Head::Detached(Some(_)) => "detached".to_owned(),
                Head::Detached(None) => "empty".to_owned(),
            };
            if let Some(tip) = tip {
                let path = tl.path_to_root(tip).map_err(|e| RecordError::Timeline(e.to_string()))?;
                for h in path {
                    em.op(h)?;
                }
                if o.check == Check::Tips {
                    em.assert(tip);
                }
                if let Some(r) = head_ref {
                    refs.insert(r.to_owned(), tip);
                    if r != "main" {
                        em.branch(r);
                    }
                }
            }
        }
        Forks::Refs => {
            head_text = match head {
                Head::Ref(r) => r.clone(),
                Head::Detached(Some(_)) => "detached".to_owned(),
                Head::Detached(None) => "empty".to_owned(),
            };
            refs.clone_from(tl.refs());
            walk_refs(&mut em, head)?;
        }
    }
    let Em { body, stmt_nodes, mut notes, requires, nodes, .. } = em;
    if o.forks == Forks::Refs {
        let reachable = reachable_nodes(tl)?;
        let orphans = tl.nodes().filter(|(h, _)| !reachable.contains(h)).count();
        if orphans > 0 {
            notes.push(RecordNote::Orphans { count: orphans });
        }
    }
    let mut text = String::new();
    if o.header {
        text.push_str("// smart-mesh script record\n");
        let policy = match o.forks {
            Forks::Head => "head",
            Forks::Refs => "refs",
        };
        text.push_str(&format!("// source: head {head_text}, forks {policy}, {} ref tip(s), {nodes} node(s)\n", refs.len()));
        for r in &requires {
            text.push_str(&format!("// requires input sha256:{r}\n"));
        }
        text.push_str("// replay on an empty project\n");
        if !body.is_empty() {
            text.push('\n');
        }
    }
    text.push_str(&body);
    let limit = ParseLimits::default().max_source_bytes;
    if text.len() > limit {
        return Err(RecordError::TooLarge { bytes: text.len(), limit });
    }
    Ok(Recording {
        statements: stmt_nodes.len(),
        text,
        nodes,
        refs,
        requires: requires.into_iter().map(|sha256| Requirement::Input { sha256 }).collect(),
        notes,
        statement_nodes: stmt_nodes,
    })
}

/// Every node on the root path of some ref.
fn reachable_nodes(tl: &Timeline) -> Result<BTreeSet<Hash256>, RecordError> {
    let mut seen = BTreeSet::new();
    for tip in tl.refs().values() {
        let mut cur = Some(*tip);
        while let Some(c) = cur {
            if !seen.insert(c) {
                break;
            }
            cur = tl.get(c).ok_or_else(|| RecordError::Timeline(format!("unknown node {c}")))?.body.parent;
        }
    }
    Ok(seen)
}

/// One node being walked: its children in visiting order with the ref that carries each.
struct Frame {
    kids: Vec<(Hash256, Option<String>)>,
    next: usize,
}

fn walk_refs(em: &mut Em<'_>, head: &Head) -> Result<(), RecordError> {
    let tl = em.tl;
    // minref: the smallest ref name in the subtree of a node (the carrier); own: refs whose tip is the node.
    let mut minref: BTreeMap<Hash256, String> = BTreeMap::new();
    let mut own: BTreeMap<Hash256, Vec<String>> = BTreeMap::new();
    for (name, tip) in tl.refs() {
        own.entry(*tip).or_default().push(name.clone());
        let mut cur = Some(*tip);
        while let Some(c) = cur {
            let node = tl.get(c).ok_or_else(|| RecordError::Timeline(format!("ref `{name}` names the unknown node {c}")))?;
            if minref.contains_key(&c) {
                break;
            }
            minref.insert(c, name.clone());
            cur = node.body.parent;
        }
    }
    let roots: Vec<Hash256> = tl.roots().iter().copied().filter(|r| minref.contains_key(r)).collect();
    if roots.is_empty() {
        return Ok(());
    }
    // Detached at the empty state: replay creates no stray `main`, and `branch` is only legal after the first op.
    em.cmd("timeline.checkout(empty: true)".to_owned());
    let mut att: Option<String> = None;
    let none: Vec<String> = Vec::new();
    for (ri, root) in roots.iter().enumerate() {
        if ri > 0 {
            em.cmd("timeline.checkout(empty: true)".to_owned());
            att = None;
        }
        let mut stack: Vec<Frame> = Vec::new();
        let enter = |em: &mut Em<'_>, att: &mut Option<String>, node: Hash256, att_in: Option<String>| -> Result<Frame, RecordError> {
            em.op(node)?;
            let mine = own.get(&node).unwrap_or(&none);
            if em.o.check == Check::Tips && !mine.is_empty() {
                em.assert(node);
            }
            let kids: Vec<Hash256> = tl.children(node).iter().copied().filter(|k| minref.contains_key(k)).collect();
            // A lone child on a detached walk with nothing to leave behind continues detached; otherwise each child
            // is carried by the smallest ref in its subtree, created here unless it is the one already attached.
            let detached_run = kids.len() == 1 && att_in.is_none() && mine.is_empty();
            let mut plan: Vec<(Hash256, Option<String>)> = kids
                .iter()
                .map(|k| (*k, if detached_run { None } else { minref.get(k).cloned() }))
                .collect();
            for r in mine {
                if att_in.as_ref() != Some(r) {
                    em.branch(r);
                    *att = Some(r.clone());
                }
            }
            for (_, c) in &plan {
                if let Some(c) = c
                    && att_in.as_ref() != Some(c)
                {
                    em.branch(c);
                    *att = Some(c.clone());
                }
            }
            // The child the attached ref continues into goes last, so the ref still sits here for the others.
            plan.sort_by_key(|(_, c)| c.is_some() && *c == att_in);
            Ok(Frame { kids: plan, next: 0 })
        };
        stack.push(enter(em, &mut att, *root, None)?);
        while let Some(top) = stack.last_mut() {
            let Some((kid, carrier)) = top.kids.get(top.next).cloned() else {
                stack.pop();
                continue;
            };
            top.next += 1;
            if let Some(c) = &carrier
                && att.as_ref() != Some(c)
            {
                em.checkout(c);
                att = Some(c.clone());
            }
            stack.push(enter(em, &mut att, kid, carrier)?);
        }
    }
    if let Head::Ref(r) = head
        && tl.refs().contains_key(r)
    {
        em.checkout(r);
    }
    Ok(())
}

// ---- verification --------------------------------------------------------------------------------

/// A fresh [`Session`] as a [`Host`]: the replay target of [`verify_recording`].
struct ReplayHost {
    session: Session,
}

fn host_err(code: &str, message: String) -> HostError {
    HostError { code: code.to_owned(), message }
}

fn tl_err(e: &TimelineError) -> HostError {
    host_err(e.code(), e.to_string())
}

impl Host for ReplayHost {
    fn registry(&self) -> &Registry {
        self.session.engine().registry()
    }

    fn state(&self) -> &State {
        self.session.state()
    }

    fn apply(&mut self, rec: &OpRecord) -> Result<sm_op_api::Delta, HostError> {
        let q = QuietControl::new();
        self.session.apply_record(rec.clone(), Meta::default(), &q.control()).map(|(_, d)| d).map_err(|e| tl_err(&e))
    }

    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError> {
        let q = QuietControl::new();
        self.session.engine().run_query(self.session.state(), name, params, &q.control()).map_err(|e| host_err(e.code(), e.to_string()))
    }

    fn timeline(&mut self, cmd: &TimelineCommand) -> Result<(), HostError> {
        let q = QuietControl::new();
        let r = match cmd {
            TimelineCommand::Branch { name, at: None } => self.session.branch(name),
            TimelineCommand::Branch { name, at: Some(h) } => self.session.create_branch_at(name, *h).and_then(|()| self.session.checkout(sm_op_api::Rev::Ref(name.clone()), &q.control())),
            TimelineCommand::Checkout(rev) => self.session.checkout(rev.clone(), &q.control()),
            TimelineCommand::Undo => self.session.undo(),
            TimelineCommand::Redo => self.session.redo(),
        };
        r.map_err(|e| tl_err(&e))
    }

    fn timeline_head(&mut self) -> Result<TimelineHead, HostError> {
        let branch = match self.session.head() {
            Head::Ref(r) => Some(r.clone()),
            Head::Detached(_) => None,
        };
        Ok(TimelineHead::new(branch, self.session.position().map(|h| h.to_hex())))
    }

    fn session(&mut self, cmd: &SessionCommand) -> Result<Json, HostError> {
        match cmd {
            SessionCommand::Hash => Ok(Json::String(self.session.state().hash().to_hex())),
            _ => Err(host_err("session-unsupported", format!("`session.{}` is not supported by the replay host", cmd.verb()))),
        }
    }
}

/// How one recorded ref came out in the replay.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RefCheck {
    /// The ref exists in the replay and its tip's state hash equals the source tip's.
    pub tip_state_equal: bool,
    /// The records on the replay's root path equal the source's (migrated to the current op versions), node for node.
    pub records_equal: bool,
}

/// The result of [`verify_recording`].
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct VerifyReport {
    /// Top-level statements run.
    pub statements: usize,
    /// Per recorded ref.
    pub refs: BTreeMap<String, RefCheck>,
    /// The ref the replay's HEAD is attached to at the end (`None` when detached).
    pub head: Option<String>,
    /// The source head is attached to an existing ref `r` and the replay is attached to `r` too (true when the source
    /// head is detached: detached heads are not restored).
    pub head_equal: bool,
    /// Every statement ran without a diagnostic, warnings included.
    pub clean: bool,
}

impl VerifyReport {
    /// Every recorded ref came out equal and the head matches.
    pub fn all_equal(&self) -> bool {
        self.head_equal && self.refs.values().all(|r| r.tip_state_equal && r.records_equal)
    }
}

/// Run `rec.text` in a fresh session over `engine` and compare the result with `tl`.
///
/// A replay error names the 1-based statement and the source node it records (`RecordError::Replay`); the failing
/// statement had no effect (`nodes_applied` counts what the replay had by then). On success the report compares every
/// recorded ref's tip state hash and root-path records, and the final head.
pub fn verify_recording(engine: &Engine, tl: &Timeline, head: &Head, rec: &Recording) -> Result<VerifyReport, RecordError> {
    let limits = ParseLimits { max_source_bytes: usize::MAX / 2, ..ParseLimits::default() };
    let parsed = parse_with(0, &rec.text, limits);
    let stmts: Vec<(u32, u32)> = parsed.script.stmts.iter().map(|s| (s.span.lo, s.span.hi)).collect();
    let mut host = ReplayHost { session: Session::new(engine.clone()) };
    let report = {
        let mut it = Interpreter::new(&mut host, Options { parse: limits, ..Options::default() });
        it.run_source("record.smesh", &rec.text)
    };
    if let Some(d) = report.diagnostics.iter().find(|d| d.is_error()) {
        let idx = stmts.iter().position(|(lo, hi)| d.span.lo >= *lo && d.span.lo < (*hi).max(*lo + 1)).unwrap_or(0);
        let node = rec.statement_nodes.get(idx).copied().flatten();
        let mut hint = d.help.clone();
        if d.code == "entity-not-found" {
            hint = Some("the op accepted an id the language considers dead; `style: Raw` spells ids as strings, which the language checks the same way".to_owned());
        }
        return Err(RecordError::Replay {
            statement: idx + 1,
            node,
            code: d.code.to_owned(),
            message: d.message.clone(),
            hint,
            nodes_applied: host.session.timeline().len(),
        });
    }
    let reg = engine.registry();
    let replay = host.session.timeline();
    let chain = |t: &Timeline, tip: Hash256, migrate: bool| -> Option<Vec<OpRecord>> {
        let path = t.path_to_root(tip).ok()?;
        path.iter()
            .map(|h| {
                let r = t.get(*h)?.body.op.clone();
                if migrate { Some(migrated(reg, &r).ok()?.unwrap_or(r)) } else { Some(r) }
            })
            .collect()
    };
    let mut refs = BTreeMap::new();
    for (name, src_tip) in &rec.refs {
        let rep_tip = replay.refs().get(name).copied();
        let tip_state_equal = match (rep_tip.and_then(|h| replay.get(h)), tl.get(*src_tip)) {
            (Some(a), Some(b)) => a.check.state == b.check.state,
            _ => false,
        };
        let records_equal = match (rep_tip, chain(tl, *src_tip, true)) {
            (Some(rt), Some(src)) => chain(replay, rt, false).is_some_and(|c| c == src),
            _ => false,
        };
        refs.insert(name.clone(), RefCheck { tip_state_equal, records_equal });
    }
    let replay_head = match host.session.head() {
        Head::Ref(r) => Some(r.clone()),
        Head::Detached(_) => None,
    };
    let head_equal = match head {
        Head::Ref(r) if tl.refs().contains_key(r) => replay_head.as_deref() == Some(r.as_str()),
        _ => true,
    };
    let clean = report.diagnostics.is_empty();
    Ok(VerifyReport { statements: stmts.len(), refs, head: replay_head, head_equal, clean })
}
