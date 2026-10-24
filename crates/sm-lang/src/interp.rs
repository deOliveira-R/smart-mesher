//! The interpreter: evaluates statements, resolves entity lists against the host state, checks arguments against op
//! schemas, lowers calls to op records (and `batch` blocks to `core.batch`) and executes them through a [`Host`].

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use serde_json::{Map, Value as Json};
use sm_core::{EntityId, EntityKind};
use sm_op_api::{Batch, Delta, Hash256, OpRecord, Rev};

use crate::ast::*;
use crate::diag::{Diagnostic, suggest};
use crate::entity::{EntitySet, all_of, live_range};
use crate::eval::{self, BUILTIN_NAMES, EvalErr};
use crate::help::{self, Callable};
use crate::host::{Host, SessionCommand, TimelineCommand};
use crate::kinds;
use crate::parser::{ParseLimits, parse_with};
use crate::schema::{self, Issue, PathSeg, View};
use crate::span::{SourceMap, Span};
use crate::value::{Outcome, Sel, Val, fmt_num};

/// The `session.*` verbs.
const SESSION_VERBS: [&str; 4] = ["export", "save", "hash", "status"];
/// The `session.*` verbs that have a value and may appear in expressions.
const SESSION_VALUE_VERBS: [&str; 2] = ["hash", "status"];

/// Interpreter limits.
#[derive(Clone, Debug)]
pub struct Options {
    /// Parser limits (also applied to included files).
    pub parse: ParseLimits,
    /// Budget of loop-body executions per run (denial-of-service guard). Only **innermost** executions are charged: an
    /// iteration whose body ran no nested loop iteration costs 1, an iteration that ran inner iterations costs nothing
    /// itself (they paid). So a 1000 x 1000 nested loop costs 1,000,000, not 1,001,000.
    pub max_loop_iterations: u64,
    /// Longest list `range` may build.
    pub max_range_len: u64,
    /// Deepest `include` nesting.
    pub max_include_depth: usize,
    /// Longest string (bytes) or list (elements) a value may have; checked by `+`, `str`, `range` and query results.
    pub max_value_len: usize,
    /// Interactive sessions: let `let` rebind a name that is already bound at the top level (with a `shadowed` warning),
    /// so a resent `let w = 10` is not an error. Inside blocks, and for scripts by default, rebinding stays an error.
    pub allow_top_level_rebind: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self { parse: ParseLimits::default(), max_loop_iterations: 1_000_000, max_range_len: 1_000_000, max_include_depth: 16, max_value_len: 1_000_000, allow_top_level_rebind: false }
    }
}

/// Something that happened while running a script, in order.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum Event {
    /// A top-level op statement (applied unless the host plans only).
    Op {
        /// Statement span.
        span: Span,
        /// The record that was (or would be) applied: the canonical encoded form.
        record: OpRecord,
        /// The op's delta; `None` in plan mode.
        delta: Option<Delta>,
    },
    /// A `batch { }` block, lowered to one `core.batch` record.
    Batch {
        /// Block span.
        span: Span,
        /// The `core.batch` record.
        record: OpRecord,
        /// Number of member ops.
        members: usize,
        /// The batch delta; `None` in plan mode.
        delta: Option<Delta>,
    },
    /// A query statement.
    Query {
        /// Statement span.
        span: Span,
        /// Query name.
        name: String,
        /// Input parameters (wire form: canonical ids such as `TopoFace#1`; see [`respell_json`] for a display copy).
        params: Json,
        /// Result in wire form, as the host returned it; `None` in plan mode. [`respell_json`] gives the source spelling.
        result: Option<Json>,
    },
    /// A timeline-control statement (`timeline.branch(...)` and friends), handled by the host outside the op log.
    Timeline {
        /// Statement span.
        span: Span,
        /// The command.
        command: TimelineCommand,
        /// False in plan mode (nothing was done).
        applied: bool,
    },
    /// A session statement or query (`session.export(...)` and friends), handled by the host outside the op log.
    Session {
        /// Statement span.
        span: Span,
        /// The command.
        command: SessionCommand,
        /// False in plan mode (nothing was done).
        applied: bool,
        /// What the host returned; `None` in plan mode.
        result: Option<Json>,
    },
    /// Output of `print`.
    Print {
        /// Statement span.
        span: Span,
        /// The text.
        text: String,
    },
    /// Output of `help`.
    Help {
        /// Statement span.
        span: Span,
        /// The text.
        text: String,
    },
}

/// What one run produced.
#[derive(Clone, Debug, Default)]
pub struct RunReport {
    /// Events in order.
    pub events: Vec<Event>,
    /// Errors (at most one runtime error, any number of syntax errors) and warnings.
    pub diagnostics: Vec<Diagnostic>,
    /// Number of statements that completed.
    pub statements: usize,
}

impl RunReport {
    /// True if there is no error diagnostic.
    pub fn ok(&self) -> bool {
        !self.diagnostics.iter().any(Diagnostic::is_error)
    }

    /// Every diagnostic as machine-readable JSON (see [`Diagnostic::to_json`]), for an MCP tool result.
    pub fn diagnostics_json(&self, map: &SourceMap) -> Json {
        Json::Array(self.diagnostics.iter().map(|d| d.to_json(map)).collect())
    }

    /// The op and batch records in the order they were applied (or planned).
    pub fn records(&self) -> Vec<&OpRecord> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::Op { record, .. } | Event::Batch { record, .. } => Some(record),
                _ => None,
            })
            .collect()
    }
}

pub(crate) struct Binding {
    pub(crate) val: Val,
    span: Span,
}

struct BatchState {
    id: u64,
    records: Vec<OpRecord>,
    /// Per member, where each argument came from (for pointing a batch failure at the offending argument).
    sites: Vec<Vec<OwnedSite>>,
}

/// An argument's source, owned so it can outlive the call that built it. Only list and object literals keep their
/// expression (to descend into elements); anything else needs just its span.
struct OwnedSite {
    key: String,
    span: Span,
    expr: Option<Expr>,
    direct: bool,
}

impl OwnedSite {
    fn of(s: &Site<'_>) -> Self {
        let keep = matches!(s.expr.kind, ExprKind::List(_) | ExprKind::Obj(_));
        Self { key: s.key.clone(), span: s.expr.span, expr: keep.then(|| s.expr.clone()), direct: s.direct }
    }

    fn locate(&self, rest: &[PathSeg]) -> Span {
        match &self.expr {
            Some(e) if self.direct => locate(e, rest),
            Some(e) => locate_in_spread(e, &self.key, rest),
            None => self.span,
        }
    }
}

type IR<T> = Result<T, ()>;

/// Where one argument of a call came from, so a failure can point at it.
struct Site<'c> {
    key: String,
    expr: &'c Expr,
    /// True when `expr` is the value itself (named or positional); false when it is an object spread into the call.
    direct: bool,
}

/// Evaluates smesh scripts against a [`Host`].
pub struct Interpreter<'h, H: Host + ?Sized> {
    pub(crate) host: &'h mut H,
    opts: Options,
    pub(crate) scopes: Vec<BTreeMap<String, Binding>>,
    pub(crate) aliases: BTreeMap<String, String>,
    alias_spans: BTreeMap<String, Span>,
    map: SourceMap,
    events: Vec<Event>,
    diags: Vec<Diagnostic>,
    batch: Option<BatchState>,
    batch_counter: u64,
    includes: Vec<String>,
    loop_count: u64,
    /// Every loop iteration started so far (charged or not); see `exec_for`.
    iter_events: u64,
    stmt_count: usize,
    plan_n: usize,
    /// Op name of each plan-mode statement ordinal, for static role checks.
    plan_ops: BTreeMap<usize, String>,
    callables: Option<BTreeMap<String, Arc<Callable>>>,
    saw_placeholder: bool,
    /// Spans that already got a `batch-snapshot` warning (an unrolled loop warns once per site).
    snap_warned: BTreeSet<(u32, u32, u32)>,
    /// Spans that already got a block-`let` `shadowed` warning (a loop body warns once per site).
    shadow_warned: BTreeSet<(u32, u32, u32)>,
    /// Set when the call being built contains a plan placeholder (so it cannot be decoded).
    tainted: bool,
}

impl<'h, H: Host + ?Sized> Interpreter<'h, H> {
    /// A new interpreter over `host`.
    pub fn new(host: &'h mut H, opts: Options) -> Self {
        Self {
            host,
            opts,
            scopes: vec![BTreeMap::new()],
            aliases: BTreeMap::new(),
            alias_spans: BTreeMap::new(),
            map: SourceMap::new(),
            events: Vec::new(),
            diags: Vec::new(),
            batch: None,
            batch_counter: 0,
            includes: Vec::new(),
            loop_count: 0,
            iter_events: 0,
            stmt_count: 0,
            plan_n: 0,
            plan_ops: BTreeMap::new(),
            callables: None,
            saw_placeholder: false,
            snap_warned: BTreeSet::new(),
            shadow_warned: BTreeSet::new(),
            tainted: false,
        }
    }

    /// All sources seen so far (render diagnostics against this).
    pub fn sources(&self) -> &SourceMap {
        &self.map
    }

    /// The host.
    pub fn host(&self) -> &H {
        self.host
    }

    /// Visible variable names with their values (innermost binding wins), for completion and inspection.
    pub fn bindings(&self) -> Vec<(&str, &Val)> {
        let mut m: BTreeMap<&str, &Val> = BTreeMap::new();
        for scope in &self.scopes {
            for (k, b) in scope {
                m.insert(k.as_str(), &b.val);
            }
        }
        m.into_iter().collect()
    }

    /// Script-scope aliases (`alias short = full.name`).
    pub fn aliases(&self) -> &BTreeMap<String, String> {
        &self.aliases
    }

    /// Parse and run `text`. Nothing runs if it does not parse.
    pub fn run_source(&mut self, name: &str, text: &str) -> RunReport {
        let file = self.map.add(name, text);
        let out = parse_with(file, text, self.opts.parse);
        self.diags.extend(out.diagnostics);
        if self.diags.iter().any(Diagnostic::is_error) {
            return self.finish();
        }
        self.run_parsed(&out.script)
    }

    /// Register `text` as a source and parse it (so spans resolve in [`sources`](Self::sources)), without running it.
    pub fn parse_source(&mut self, name: &str, text: &str) -> (Script, Vec<Diagnostic>) {
        let file = self.map.add(name, text);
        let out = parse_with(file, text, self.opts.parse);
        (out.script, out.diagnostics)
    }

    /// Run an already parsed script (parsed through [`parse_source`](Self::parse_source) of this interpreter).
    pub fn run_parsed(&mut self, script: &Script) -> RunReport {
        self.batch = None;
        self.loop_count = 0;
        self.scopes.truncate(1);
        let _ = self.exec_stmts(&script.stmts);
        self.batch = None;
        self.scopes.truncate(1);
        self.finish()
    }

    fn finish(&mut self) -> RunReport {
        RunReport { events: std::mem::take(&mut self.events), diagnostics: std::mem::take(&mut self.diags), statements: std::mem::take(&mut self.stmt_count) }
    }

    // ---- diagnostics helpers -------------------------------------------------------------

    fn fail<T>(&mut self, d: Diagnostic) -> IR<T> {
        self.diags.push(d);
        Err(())
    }

    fn eval_fail<T>(&mut self, e: EvalErr, span: Span) -> IR<T> {
        let (code, msg) = e;
        let mut d = Diagnostic::error(code, span, msg).label("here");
        if code == "ambiguous-kind" {
            d = d.help(kinds::UserIdError::AMBIGUOUS_HELP);
        }
        self.fail(d)
    }

    fn warn(&mut self, d: Diagnostic) {
        self.diags.push(d);
    }

    /// A state that cannot arise from any input; reported as a diagnostic instead of panicking.
    fn internal<T>(&mut self, span: Span, what: &str) -> IR<T> {
        self.fail(Diagnostic::error("internal-error", span, format!("internal error: {what}")).label("this is a bug in sm-lang"))
    }

    pub(crate) fn callable_map(&mut self) -> &BTreeMap<String, Arc<Callable>> {
        let host = &*self.host;
        self.callables.get_or_insert_with(|| help::callables(host.registry()).into_iter().map(|c| (c.name.clone(), Arc::new(c))).collect())
    }

    pub(crate) fn lookup_callable(&mut self, name: &str) -> Option<Arc<Callable>> {
        self.callable_map().get(name).cloned()
    }

    // ---- statements ---------------------------------------------------------------------

    fn exec_stmts(&mut self, stmts: &[Stmt]) -> IR<()> {
        for s in stmts {
            self.exec(s)?;
        }
        Ok(())
    }

    fn exec(&mut self, s: &Stmt) -> IR<()> {
        match &s.kind {
            StmtKind::Let { name, value } => {
                let v = self.eval_stmt_value(value)?;
                self.bind(name, v)?;
            }
            StmtKind::Set { name, value } => {
                if !self.try_append(name, value)? {
                    let v = self.eval_stmt_value(value)?;
                    self.rebind(name, v)?;
                }
            }
            StmtKind::Alias { name, target } => self.exec_alias(name, target)?,
            StmtKind::Call(c) => {
                self.exec_call(c, s.span)?;
            }
            StmtKind::Batch(b) => self.exec_batch(b, s.span)?,
            StmtKind::If { cond, then, elifs, els } => self.exec_if(cond, then, elifs, els.as_ref())?,
            StmtKind::For { var, iter, body } => self.exec_for(var, iter, body)?,
            StmtKind::Include { path, path_span } => self.exec_include(path, *path_span)?,
            StmtKind::Print(es) => {
                let mut parts = Vec::new();
                for e in es {
                    let v = self.eval(e)?;
                    parts.push(v.to_string());
                }
                self.events.push(Event::Print { span: s.span, text: parts.join(" ") });
            }
            StmtKind::Assert { cond, msg } => {
                let c = self.eval(cond)?;
                match c {
                    Val::Bool(true) => {}
                    Val::Bool(false) => {
                        let text = match msg {
                            Some(m) => self.eval(m)?.to_string(),
                            None => "assertion failed".to_owned(),
                        };
                        return self.fail(Diagnostic::error("assert-failed", cond.span, text).label("this condition is false"));
                    }
                    other => {
                        let d = Diagnostic::error("type-mismatch", cond.span, format!("`assert` needs a bool, found {}", other.a_type())).label("not a bool");
                        return self.fail(d);
                    }
                }
            }
            StmtKind::Help(name) => {
                let text = match name {
                    None => help::overview(self.host.registry()),
                    Some(p) => {
                        let dotted = p.dotted();
                        let full = self.aliases.get(&dotted).cloned().unwrap_or(dotted);
                        match help::op_help(self.host.registry(), &full) {
                            Ok(t) => t,
                            Err(m) => return self.fail(Diagnostic::error("unknown-name", p.span, m).label("not an op or query")),
                        }
                    }
                };
                self.events.push(Event::Help { span: s.span, text });
            }
            StmtKind::Error => {}
        }
        self.stmt_count += 1;
        Ok(())
    }

    fn bind(&mut self, name: &Ident, val: Val) -> IR<()> {
        let prev = self.scopes.last().and_then(|top| top.get(&name.name)).map(|b| b.span);
        if let Some(prev) = prev {
            if self.opts.allow_top_level_rebind && self.scopes.len() == 1 {
                let d = Diagnostic::warning("shadowed", name.span, format!("`{}` was already defined; its value is replaced", name.name))
                    .label("rebound here")
                    .note(prev, "previously defined here");
                self.warn(d);
            } else {
                let d = Diagnostic::error("rebind", name.span, format!("`{}` is already defined in this scope", name.name))
                    .label("defined again here")
                    .note(prev, "first defined here")
                    .help("use `set` to change its value");
                return self.fail(d);
            }
        }
        // A `let` in a block that hides a binding of an enclosing scope: usually a `set` was meant.
        if prev.is_none() && self.scopes.len() > 1 {
            let outer = self.scopes[..self.scopes.len() - 1].iter().rev().find_map(|sc| sc.get(&name.name)).map(|b| b.span);
            if let Some(outer) = outer.filter(|_| self.shadow_warned.insert((name.span.file, name.span.lo, name.span.hi))) {
                let d = Diagnostic::warning("shadowed", name.span, format!("`{}` shadows a variable of an enclosing scope", name.name))
                    .label("a new variable, not a change to the outer one")
                    .note(outer, "outer binding defined here")
                    .help("use `set` to change the outer variable");
                self.warn(d);
            }
        }
        match self.scopes.last_mut() {
            Some(top) => {
                top.insert(name.name.clone(), Binding { val, span: name.span });
                Ok(())
            }
            None => self.internal(name.span, "no scope"),
        }
    }

    /// `set x = x + a [+ b ...]` on a list or string: append in place instead of copying `x` for every `+`, which made
    /// building a list in a loop quadratic. Returns `Ok(false)` when the statement does not have that shape (the general
    /// path then handles it). The new operands are evaluated and checked before `x` changes, so an error leaves it untouched.
    fn try_append(&mut self, name: &Ident, value: &Expr) -> IR<bool> {
        let (head, rest): (&Expr, Vec<(&Expr, Span)>) = match &value.kind {
            ExprKind::Binary(BinOp::Add, l, r) => (l, vec![(r, value.span)]),
            ExprKind::Chain(BinOp::Add, items) if items.len() >= 2 => (&items[0], items[1..].iter().map(|it| (it, items[0].span.to(it.span))).collect()),
            _ => return Ok(false),
        };
        let ExprKind::Var(n) = &head.kind else { return Ok(false) };
        if *n != name.name {
            return Ok(false);
        }
        let Some(si) = self.scopes.iter().rposition(|sc| sc.contains_key(n)) else { return Ok(false) };
        if !matches!(self.scopes[si][n].val, Val::List(_) | Val::Str(_)) {
            return Ok(false);
        }
        let mut rvs = Vec::with_capacity(rest.len());
        for (e, _) in &rest {
            rvs.push(self.eval(e)?);
        }
        let max = self.opts.max_value_len;
        let mut len = eval::seq_len(&self.scopes[si][n].val);
        for ((_, span), rv) in rest.iter().zip(&rvs) {
            if !eval::can_append(&self.scopes[si][n].val, rv) {
                let acc = self.scopes[si][n].val.clone();
                return match eval::binop_lim(BinOp::Add, &acc, rv, max) {
                    Err(er) => self.eval_fail(er, *span),
                    Ok(_) => self.internal(*span, "append of mismatched values succeeded"),
                };
            }
            len = len.saturating_add(eval::seq_len(rv));
            if len > max {
                let what = if matches!(rv, Val::Str(_)) { "a string" } else { "a list" };
                return self.eval_fail(eval::too_long(what, len, max), *span);
            }
        }
        let Some(b) = self.scopes[si].get_mut(n) else { return Ok(false) };
        for rv in &rvs {
            eval::append_in_place(&mut b.val, rv);
        }
        Ok(true)
    }

    fn rebind(&mut self, name: &Ident, val: Val) -> IR<()> {
        for scope in self.scopes.iter_mut().rev() {
            if let Some(b) = scope.get_mut(&name.name) {
                b.val = val;
                return Ok(());
            }
        }
        let names: Vec<&str> = self.scopes.iter().flat_map(|s| s.keys().map(String::as_str)).collect();
        let mut d = Diagnostic::error("unknown-variable", name.span, format!("`{}` is not defined", name.name)).label("nothing to `set` here");
        if let Some(s) = suggest(&name.name, names.iter().copied()) {
            d = d.help(format!("did you mean `{s}`?"));
        } else {
            d = d.help("define it first with `let`");
        }
        self.fail(d)
    }

    fn exec_alias(&mut self, name: &Ident, target: &Path) -> IR<()> {
        let full = target.dotted();
        if self.lookup_callable(&full).is_none() {
            let mut d = Diagnostic::error("unknown-name", target.span, format!("no op or query named `{full}`")).label("alias target does not exist");
            let names: Vec<String> = self.callable_map().keys().cloned().collect();
            if let Some(s) = suggest(&full, names.iter().map(String::as_str)) {
                d = d.help(format!("did you mean `{s}`?"));
            }
            return self.fail(d);
        }
        if self.lookup_callable(&name.name).is_some() || BUILTIN_NAMES.contains(&name.name.as_str()) {
            let d = Diagnostic::error("alias-shadows", name.span, format!("`{}` is already a registry or builtin name", name.name)).label("choose another alias");
            return self.fail(d);
        }
        // Same rule as `let`: a name is bound once per scope (aliases are script-wide, so once per script).
        if let Some(prev) = self.alias_spans.get(&name.name).copied() {
            if self.opts.allow_top_level_rebind && self.scopes.len() == 1 {
                let d = Diagnostic::warning("shadowed", name.span, format!("alias `{}` was already defined; it now stands for `{full}`", name.name))
                    .label("rebound here")
                    .note(prev, "previously defined here");
                self.warn(d);
            } else {
                let d = Diagnostic::error("rebind", name.span, format!("alias `{}` is already defined", name.name))
                    .label("defined again here")
                    .note(prev, "first defined here")
                    .help("an alias is bound once, like `let`; choose another name");
                return self.fail(d);
            }
        }
        let namespaces: BTreeSet<String> = self.callable_map().keys().filter_map(|k| k.split_once('.').map(|(ns, _)| ns.to_owned())).chain(["timeline".to_owned(), "session".to_owned()]).collect();
        if namespaces.contains(&name.name) {
            let d = Diagnostic::warning("alias-confusing", name.span, format!("`{}` is also the name of a registry namespace (`{}.*`)", name.name, name.name))
                .label("easily mistaken for the namespace")
                .help("choose an alias that is not a namespace name, e.g. `brick` for `model.create_brick`");
            self.warn(d);
        } else if self.is_defined(&name.name) {
            let d = Diagnostic::warning("alias-confusing", name.span, format!("`{}` is also the name of a variable", name.name))
                .label("the alias and the variable share a name")
                .help("a call `name(...)` uses the alias and an expression `name` the variable; choose a distinct alias");
            self.warn(d);
        }
        self.alias_spans.insert(name.name.clone(), name.span);
        self.aliases.insert(name.name.clone(), full);
        Ok(())
    }

    /// Turn the `plan-unresolved` error just reported (if any, after diagnostic `mark`) into a warning. True if it did.
    fn downgrade_plan_error(&mut self, mark: usize, extra: &str) -> bool {
        if self.diags.len() > mark
            && let Some(d) = self.diags.last_mut()
            && d.code == "plan-unresolved"
            && d.is_error()
        {
            d.severity = crate::diag::Severity::Warning;
            d.message.push_str(extra);
            return true;
        }
        false
    }

    /// The value of a condition: `Some(bool)`, or `None` when it depends on a plan placeholder (unknown without executing),
    /// which is reported as a warning so the caller can check every branch instead of failing the plan.
    fn cond_known(&mut self, cond: &Expr) -> IR<Option<bool>> {
        let mark = self.diags.len();
        match self.eval(cond) {
            Ok(Val::Bool(b)) => Ok(Some(b)),
            Ok(Val::Placeholder { .. }) => {
                self.warn(
                    Diagnostic::warning("plan-unresolved", cond.span, "this condition is unknown without executing")
                        .label("every branch is checked once")
                        .help("the real run takes one branch; wrap dependent statements in `batch { }` to lower them without executing"),
                );
                Ok(None)
            }
            Ok(c) => {
                let d = Diagnostic::error("type-mismatch", cond.span, format!("`if` needs a bool, found {}", c.a_type())).label("not a bool");
                self.fail(d)
            }
            Err(()) => {
                if self.downgrade_plan_error(mark, " (so every branch of this `if` is checked instead)") {
                    Ok(None)
                } else {
                    Err(())
                }
            }
        }
    }

    /// Plan mode, unknown condition: run a branch for its checks only; a `plan-unresolved` error inside ends the check as a warning.
    fn check_branch(&mut self, b: &Block) -> IR<()> {
        let mark = self.diags.len();
        match self.exec_block(b) {
            Ok(()) => Ok(()),
            Err(()) => {
                if self.downgrade_plan_error(mark, " (the rest of this branch was not checked)") {
                    Ok(())
                } else {
                    Err(())
                }
            }
        }
    }

    fn exec_if(&mut self, cond: &Expr, then: &Block, elifs: &[(Expr, Block)], els: Option<&Block>) -> IR<()> {
        let mut unknown = false;
        for (c, b) in std::iter::once((cond, then)).chain(elifs.iter().map(|(c, b)| (c, b))) {
            if unknown {
                self.cond_known(c)?;
                self.check_branch(b)?;
                continue;
            }
            match self.cond_known(c)? {
                Some(true) => return self.exec_block(b),
                Some(false) => {}
                None => {
                    unknown = true;
                    self.check_branch(b)?;
                }
            }
        }
        match els {
            Some(b) if unknown => self.check_branch(b),
            Some(b) => self.exec_block(b),
            None => Ok(()),
        }
    }

    fn exec_block(&mut self, b: &Block) -> IR<()> {
        self.scopes.push(BTreeMap::new());
        let r = self.exec_stmts(&b.stmts);
        self.scopes.pop();
        r
    }

    fn exec_for(&mut self, var: &Ident, iter: &Expr, body: &Block) -> IR<()> {
        let v = self.eval(iter)?;
        let items: Vec<Val> = match v {
            Val::List(l) => Arc::try_unwrap(l).unwrap_or_else(|a| (*a).clone()),
            Val::Ents(s) => s.iter().map(|i| Val::Ents(EntitySet::single(*i))).collect(),
            Val::Placeholder { n, role } => return self.exec_for_unknown(var, n, &role, iter.span, body),
            Val::Ref { role, .. } => {
                let d = Diagnostic::error("type-mismatch", iter.span, format!("cannot loop over the batch output `{role}`"))
                    .label("its length is unknown")
                    .help("a batch cannot know how many ids a role holds: pass `x.role` to an array parameter, or move the loop after the batch, where the output is concrete");
                return self.fail(d);
            }
            other => {
                let d = Diagnostic::error("type-mismatch", iter.span, format!("cannot iterate over {}", other.a_type())).label("expected a list or an entity set");
                return self.fail(d);
            }
        };
        for item in items {
            // Budget: only innermost body executions are charged (an iteration during which no nested iteration started).
            // Outer iterations are paid for by the inner ones they contain, and every uncharged iteration has a charged
            // descendant at most `max_depth` levels down, so total work stays bounded by 64 times the budget.
            self.iter_events += 1;
            let before = self.iter_events;
            let mut scope = BTreeMap::new();
            scope.insert(var.name.clone(), Binding { val: item, span: var.span });
            self.scopes.push(scope);
            let r = self.exec_stmts(&body.stmts);
            self.scopes.pop();
            r?;
            if self.iter_events == before {
                self.loop_count += 1;
                if self.loop_count > self.opts.max_loop_iterations {
                    let d = Diagnostic::error("loop-limit", iter.span, format!("more than {} loop-body executions", self.opts.max_loop_iterations))
                        .label("this loop")
                        .help("only the innermost body of nested loops is counted; the budget is `Options::max_loop_iterations`");
                    return self.fail(d);
                }
            }
        }
        Ok(())
    }

    /// Plan mode: the loop's sequence is unknown without executing. The body is checked once with a placeholder
    /// variable; a `plan-unresolved` error inside it ends that check as a warning instead of failing the plan.
    fn exec_for_unknown(&mut self, var: &Ident, n: usize, role: &str, span: Span, body: &Block) -> IR<()> {
        self.warn(
            Diagnostic::warning("plan-unresolved", span, "the sequence of this loop is unknown without executing")
                .label("its body is checked once, with a placeholder")
                .help("the real run repeats the body once per element; wrap dependent statements in `batch { }` to lower them without executing"),
        );
        let mut scope = BTreeMap::new();
        scope.insert(var.name.clone(), Binding { val: Val::Placeholder { n, role: format!("{role}[]") }, span: var.span });
        self.scopes.push(scope);
        let mark = self.diags.len();
        let r = self.exec_stmts(&body.stmts);
        self.scopes.pop();
        if r.is_err() && self.downgrade_plan_error(mark, " (the rest of the loop body was not checked)") {
            return Ok(());
        }
        r
    }

    fn exec_include(&mut self, path: &str, span: Span) -> IR<()> {
        if self.batch.is_some() {
            return self.fail(Diagnostic::error("include-in-batch", span, "`include` is not allowed inside a batch").label("move it out of the block"));
        }
        if self.includes.len() >= self.opts.max_include_depth {
            return self.fail(Diagnostic::error("include-cycle", span, format!("includes nest deeper than {}", self.opts.max_include_depth)).label("too deep"));
        }
        if self.includes.iter().any(|p| p == path) {
            return self.fail(Diagnostic::error("include-cycle", span, format!("{path:?} includes itself")).label("already being included"));
        }
        let text = match self.host.load(path) {
            Ok(t) => t,
            Err(m) => return self.fail(Diagnostic::error("include-failed", span, m).label("cannot load this file")),
        };
        let file = self.map.add(path, &text);
        let out = parse_with(file, &text, self.opts.parse);
        let bad = out.diagnostics.iter().any(Diagnostic::is_error);
        self.diags.extend(out.diagnostics);
        if bad {
            return Err(());
        }
        self.includes.push(path.to_owned());
        let r = self.exec_stmts(&out.script.stmts);
        self.includes.pop();
        r
    }

    fn exec_batch(&mut self, b: &Block, span: Span) -> IR<()> {
        if self.batch.is_some() {
            return self.fail(
                Diagnostic::error("nested-batch", span, "a batch cannot contain another batch")
                    .label("nested here")
                    .help("core.batch rejects nesting; flatten the blocks"),
            );
        }
        self.batch_counter += 1;
        let id = self.batch_counter;
        self.batch = Some(BatchState { id, records: Vec::new(), sites: Vec::new() });
        let r = self.exec_stmts(&b.stmts);
        let Some(state) = self.batch.take() else { return self.internal(span, "batch state missing") };
        r?;
        if state.records.is_empty() {
            self.warn(Diagnostic::warning("empty-batch", span, "the batch emitted no ops").label("nothing to apply"));
            return Ok(());
        }
        let members = state.records.len();
        let batch_rec = Batch { ops: state.records };
        let rec = match OpRecord::of(&batch_rec) {
            Ok(r) => r,
            Err(e) => return self.fail(Diagnostic::error("encode-failed", span, e.to_string()).label("cannot encode the batch")),
        };
        if !self.host.executes() {
            self.events.push(Event::Batch { span, record: rec, members, delta: None });
            return Ok(());
        }
        match self.host.apply(&rec) {
            Ok(delta) => {
                self.export_batch(id, &delta);
                self.events.push(Event::Batch { span, record: rec, members, delta: Some(delta) });
                Ok(())
            }
            Err(e) => {
                let mut d = Diagnostic::error("op-failed", span, format!("batch failed [{}]: {}", e.code, respell(&e.message))).label("this batch was rejected and had no effect");
                // A bad reference (`'$0.voluem[0]'`) points at the argument that wrote it, not at the whole block.
                if let Some(r) = quoted_batch_ref(&e.message)
                    && let Some((sp, key, op)) = batch_site(&r, &batch_rec.ops, &state.sites)
                {
                    d.span = sp;
                    d.label = Some(format!("this reference names an output that `{op}` did not produce (argument `{key}`)"));
                    d.notes.push((span, "in this batch, which was rejected as a whole".to_owned()));
                    d = d.help("check the output name against the op's outputs (`help <op>`)");
                }
                self.fail(d)
            }
        }
    }

    /// Rebind symbolic members of batch `id` to the concrete outputs the batch produced.
    fn export_batch(&mut self, id: u64, delta: &Delta) {
        for scope in &mut self.scopes {
            for b in scope.values_mut() {
                if mentions_batch(&b.val, id) {
                    rebind_val(&mut b.val, id, delta);
                }
            }
        }
    }

    // ---- calls ---------------------------------------------------------------------------

    /// Evaluate the right-hand side of `let`/`set`: an op or query call is allowed here, anything else is an expression.
    fn eval_stmt_value(&mut self, e: &Expr) -> IR<Val> {
        if let ExprKind::Call(c) = &e.kind
            && self.is_registry_call(c)
        {
            if c.path.segs.len() == 2 && c.path.segs[0].name == "timeline" && c.path.segs[1].name != "head" {
                let d = Diagnostic::error("op-in-expression", c.path.span, format!("`{}` is a statement and has no value", c.path.dotted()))
                    .label("a timeline command cannot be bound by `let` or `set`")
                    .help("write it as its own statement, without `let x =`");
                return self.fail(d);
            }
            if c.path.segs.len() == 2 && c.path.segs[0].name == "session" && !SESSION_VALUE_VERBS.contains(&c.path.segs[1].name.as_str()) && SESSION_VERBS.contains(&c.path.segs[1].name.as_str()) {
                let d = Diagnostic::error("op-in-expression", c.path.span, format!("`{}` is a statement and has no value", c.path.dotted()))
                    .label("a session statement cannot be bound by `let` or `set`")
                    .help("write it as its own statement, without `let x =`");
                return self.fail(d);
            }
            return self.exec_call(c, e.span);
        }
        self.eval(e)
    }

    fn is_registry_call(&self, c: &Call) -> bool {
        c.path.segs.len() > 1 || self.aliases.contains_key(&c.path.segs[0].name) || !BUILTIN_NAMES.contains(&c.path.segs[0].name.as_str())
    }

    fn exec_call(&mut self, call: &Call, span: Span) -> IR<Val> {
        let written = call.path.dotted();
        if call.path.segs.len() == 2 && call.path.segs[0].name == "timeline" {
            return self.exec_timeline(call, span);
        }
        if call.path.segs.len() == 2 && call.path.segs[0].name == "session" {
            return self.exec_session(call, span);
        }
        let name = if call.path.segs.len() == 1 { self.aliases.get(&written).cloned().unwrap_or(written.clone()) } else { written.clone() };
        let Some(info) = self.lookup_callable(&name) else {
            let aliases: Vec<String> = self.aliases.keys().cloned().collect();
            let names: Vec<String> = self.callable_map().keys().cloned().chain(aliases).collect();
            let mut d = Diagnostic::error("unknown-name", call.path.span, format!("no op or query named `{written}`")).label("not in the registry");
            if let Some(s) = suggest(&written, names.iter().map(String::as_str)) {
                d = d.help(format!("did you mean `{s}`?"));
            } else if call.path.segs.len() == 1 {
                d = d.help("op and query names are dotted, like `model.create_brick`; `help` lists them");
            } else {
                d = d.help("`help` lists every op and query");
            }
            return self.fail(d);
        };
        self.tainted = false;
        let (params, sites) = self.build_params(call, &info)?;
        let in_batch = self.batch.is_some();
        if info.is_query {
            if in_batch {
                return self.fail(
                    Diagnostic::error("query-in-batch", span, format!("`{name}` is a query; queries are not recorded and cannot be part of a batch"))
                        .label("queries run immediately")
                        .help("run the query before or after the batch"),
                );
            }
            let pj = Json::Object(params);
            if !self.host.executes() {
                self.events.push(Event::Query { span, name, params: pj, result: None });
                self.plan_n += 1;
                return Ok(Val::Outcome(Outcome::Plan { n: self.plan_n }));
            }
            return match self.host.query(&name, &pj) {
                Ok(res) => {
                    let out_view = info.out.as_ref().map(|d| d.view());
                    let val = match json_to_val(&res, out_view, self.opts.max_value_len) {
                        Ok(v) => v,
                        Err(m) => return self.fail(Diagnostic::error("null-value", span, format!("query `{name}` returned a value the language cannot hold: {m}")).label("this query")),
                    };
                    self.events.push(Event::Query { span, name, params: pj, result: Some(res) });
                    Ok(val)
                }
                Err(e) => self.fail(Diagnostic::error("query-failed", span, format!("query `{name}` failed [{}]: {}", e.code, respell(&e.message))).label("this query")),
            };
        }
        let rec = match OpRecord::new(&name, self.host.registry().op_version(&name).unwrap_or(info.version), Json::Object(params)) {
            Ok(r) => r,
            Err(e) => return self.fail(Diagnostic::error("encode-failed", span, e.to_string()).label("cannot encode these arguments")),
        };
        if let Some(b) = &mut self.batch {
            if name == "core.batch" {
                return self.fail(Diagnostic::error("nested-batch", span, "a batch cannot contain another batch").label("nested here"));
            }
            let index = b.records.len();
            let batch = b.id;
            b.records.push(rec);
            b.sites.push(sites.iter().map(OwnedSite::of).collect());
            return Ok(Val::Outcome(Outcome::Batch { batch, index }));
        }
        // Decode gate: the record is known to decode, and the stored form is the canonical re-encoding.
        // (A plan record holding placeholders cannot decode; it is reported as is.)
        let rec = if self.tainted {
            rec
        } else {
            match self.host.registry().decode(&rec) {
                Ok(op) => match OpRecord::of(op.as_ref()) {
                    Ok(c) => c,
                    Err(e) => return self.fail(Diagnostic::error("encode-failed", span, e.to_string()).label("cannot encode this op")),
                },
                Err(e) => return self.fail(Diagnostic::error("decode-failed", span, format!("the registry rejects these arguments [{}]: {e}", e.code())).label("this call")),
            }
        };
        if !self.host.executes() {
            self.events.push(Event::Op { span, record: rec, delta: None });
            self.plan_n += 1;
            self.plan_ops.insert(self.plan_n, name.clone());
            return Ok(Val::Outcome(Outcome::Plan { n: self.plan_n }));
        }
        match self.host.apply(&rec) {
            Ok(delta) => {
                let out = Val::Outcome(Outcome::Concrete {
                    created: EntitySet::from_ordered(delta.created.clone()),
                    removed: EntitySet::from_ordered(delta.removed.clone()),
                    outputs: delta.outputs.iter().map(|(k, v)| (k.clone(), EntitySet::from_ordered(v.clone()))).collect(),
                });
                self.events.push(Event::Op { span, record: rec, delta: Some(delta) });
                Ok(out)
            }
            Err(e) => {
                let msg = e.message.strip_prefix(&format!("op '{name}' failed: ")).unwrap_or(&e.message).to_owned();
                let mut d = Diagnostic::error("op-failed", span, format!("`{name}` failed [{}]: {}", e.code, respell(&msg))).label("this op was rejected and had no effect");
                // When the error names an entity, point at the argument that supplied it.
                if let Some((sp, key)) = localize(&msg, &rec.params, &sites) {
                    d.span = sp;
                    d.label = Some("this argument supplied the entity".to_owned());
                    if let Some(desc) = info.params().iter().find(|p| p.name == key).and_then(|p| p.description.clone()) {
                        d = d.help(format!("parameter `{key}`: {}", desc.lines().next().unwrap_or("").trim()));
                    }
                }
                self.fail(d)
            }
        }
    }

    /// `timeline.branch(name)`, `timeline.checkout(ref | node | empty)`, `timeline.undo()`, `timeline.redo()`.
    fn exec_timeline(&mut self, call: &Call, span: Span) -> IR<Val> {
        const VERBS: [&str; 5] = ["branch", "checkout", "undo", "redo", "head"];
        let verb = call.path.segs[1].name.clone();
        if self.batch.is_some() {
            let d = Diagnostic::error("timeline-in-batch", span, "timeline commands cannot be part of a batch")
                .label("a batch is one timeline node")
                .help("run the command before or after the batch");
            return self.fail(d);
        }
        if !VERBS.contains(&verb.as_str()) {
            let mut d = Diagnostic::error("unknown-name", call.path.segs[1].span, format!("no timeline command `timeline.{verb}`")).label("not a timeline command");
            d = match suggest(&verb, VERBS) {
                Some(s) => d.help(format!("did you mean `timeline.{s}`?")),
                None => d.help("timeline commands: branch(name), checkout(ref | node | empty), undo(), redo(), head()"),
            };
            return self.fail(d);
        }
        if verb == "head" {
            return self.timeline_head_value(call, span);
        }
        let allowed: &[&str] = match verb.as_str() {
            "branch" => &["name", "at"],
            "checkout" => &["ref", "node", "empty"],
            _ => &[],
        };
        let args = self.command_args("timeline", &verb, call, allowed)?;
        let want_str = |this: &mut Self, key: &str, args: &[(String, Val, Span)]| -> IR<Option<String>> {
            match args.iter().find(|(k, ..)| k == key) {
                None => Ok(None),
                Some((_, Val::Str(s), _)) if !s.is_empty() => Ok(Some(s.clone())),
                Some((_, other, sp)) => {
                    let d = Diagnostic::error("type-mismatch", *sp, format!("`{key}` needs a non-empty string, found {}", other.a_type())).label("not a name");
                    this.fail(d)
                }
            }
        };
        let cmd = match verb.as_str() {
            "branch" => match want_str(self, "name", &args)? {
                Some(name) => {
                    let at = match want_str(self, "at", &args)? {
                        Some(h) => Some(self.parse_hash("at", &h, &args)?),
                        None => None,
                    };
                    TimelineCommand::Branch { name, at }
                }
                None => return self.fail(Diagnostic::error("missing-argument", call.args_span, "`timeline.branch` needs the new branch name").label("write `timeline.branch(\"name\")`")),
            },
            "checkout" => {
                let r = want_str(self, "ref", &args)?;
                let n = want_str(self, "node", &args)?;
                let empty = match args.iter().find(|(k, ..)| k == "empty") {
                    Some((_, Val::Bool(true), _)) => true,
                    Some((_, _, sp)) => return self.fail(Diagnostic::error("type-mismatch", *sp, "`empty` must be `true`").label("only `empty: true` is meaningful")),
                    None => false,
                };
                match (r, n, empty) {
                    (Some(r), None, false) => TimelineCommand::Checkout(Rev::Ref(r)),
                    (None, Some(n), false) => TimelineCommand::Checkout(Rev::Node(self.parse_hash("node", &n, &args)?)),
                    (None, None, true) => TimelineCommand::Checkout(Rev::Empty),
                    _ => {
                        let d = Diagnostic::error("missing-argument", call.args_span, "`timeline.checkout` needs exactly one of `ref`, `node`, `empty`")
                            .label("which revision?")
                            .help("`timeline.checkout(\"main\")` (a branch), `timeline.checkout(node: \"<hash>\")`, `timeline.checkout(empty: true)`");
                        return self.fail(d);
                    }
                }
            }
            "undo" => TimelineCommand::Undo,
            _ => TimelineCommand::Redo,
        };
        if !self.host.executes() {
            self.warn(
                Diagnostic::warning("plan-timeline", span, format!("`timeline.{verb}` is not simulated by a plan"))
                    .label("later statements are checked against the state as it is now"),
            );
            self.events.push(Event::Timeline { span, command: cmd, applied: false });
            return Ok(Val::obj(BTreeMap::new()));
        }
        match self.host.timeline(&cmd) {
            Ok(()) => {
                self.events.push(Event::Timeline { span, command: cmd, applied: true });
                Ok(Val::obj(BTreeMap::new()))
            }
            Err(e) => self.fail(Diagnostic::error("timeline-failed", span, format!("`timeline.{verb}` failed [{}]: {}", e.code, e.message)).label("this command had no effect")),
        }
    }

    /// `session.export(path, format?, mesh?, provenance?)`, `session.save()` (statements); `session.hash()`,
    /// `session.status()` (expressions). Dispatched to [`Host::session`]; never recorded as ops.
    fn exec_session(&mut self, call: &Call, span: Span) -> IR<Val> {
        let verb = call.path.segs[1].name.clone();
        if self.batch.is_some() {
            let d = Diagnostic::error("session-in-batch", span, "session commands cannot be part of a batch")
                .label("a batch is one timeline node")
                .help("run the command before or after the batch");
            return self.fail(d);
        }
        if !SESSION_VERBS.contains(&verb.as_str()) {
            let mut d = Diagnostic::error("unknown-name", call.path.segs[1].span, format!("no session command `session.{verb}`")).label("not a session command");
            d = match suggest(&verb, SESSION_VERBS) {
                Some(s) => d.help(format!("did you mean `session.{s}`?")),
                None => d.help("session commands: export(path, format, mesh, provenance), save(), hash(), status()"),
            };
            return self.fail(d);
        }
        let allowed: &[&str] = if verb == "export" { &["path", "format", "mesh", "provenance"] } else { &[] };
        let args = self.command_args("session", &verb, call, allowed)?;
        let get = |k: &str| args.iter().find(|(n, ..)| n == k);
        let cmd = match verb.as_str() {
            "export" => {
                let Some((_, pv, psp)) = get("path") else {
                    return self.fail(Diagnostic::error("missing-argument", call.args_span, "`session.export` needs the destination path").label("write `session.export(\"out.vtu\")`"));
                };
                let path = match pv {
                    Val::Str(s) if !s.is_empty() => s.clone(),
                    other => {
                        return self.fail(Diagnostic::error("type-mismatch", *psp, format!("`path` needs a non-empty string, found {}", other.a_type())).label("not a path"));
                    }
                };
                let format = match get("format") {
                    None => None,
                    Some((_, Val::Str(f), _)) if f == "smsh" || f == "vtu" => Some(f.clone()),
                    Some((_, Val::Str(f), sp)) => {
                        let mut d = Diagnostic::error("invalid-argument", *sp, format!("unknown export format {f:?}")).label("not a format").help("formats: \"smsh\", \"vtu\"");
                        if let Some(sg) = suggest(f, ["smsh", "vtu"]) {
                            d = d.help(format!("did you mean \"{sg}\"?"));
                        }
                        return self.fail(d);
                    }
                    Some((_, other, sp)) => return self.fail(Diagnostic::error("type-mismatch", *sp, format!("`format` needs a string, found {}", other.a_type())).label("not a format")),
                };
                let mesh = match get("mesh") {
                    None => None,
                    Some((_, Val::Ents(e), sp)) => match e.as_slice() {
                        [id] => Some(*id),
                        other => return self.fail(Diagnostic::error("expected-one-entity", *sp, format!("`mesh` needs exactly one entity, found {}", other.len())).label("pick one with `[0]`, or name a single id")),
                    },
                    Some((_, Val::Str(t), sp)) => match kinds::parse_user_entity_text(t) {
                        Ok(id) => Some(id),
                        Err(kinds::UserIdError::AmbiguousKind(m) | kinds::UserIdError::Invalid(m)) => return self.fail(Diagnostic::error("invalid-argument", *sp, m).label("not a mesh id")),
                    },
                    // Plan mode: the id is unknown; the host would pick (recorded as `None`, see LANGUAGE.md 4.7).
                    Some((_, Val::Placeholder { .. }, _)) if !self.host.executes() => {
                        self.saw_placeholder = true;
                        None
                    }
                    Some((_, other, sp)) => return self.fail(Diagnostic::error("type-mismatch", *sp, format!("`mesh` needs a mesh id, found {}", other.a_type())).label("not an entity")),
                };
                if let (Some(id), Some((_, _, sp))) = (mesh, get("mesh"))
                    && id.kind() != EntityKind::Mesh
                {
                    return self.fail(Diagnostic::error("wrong-kind", *sp, format!("`mesh` needs a Mesh, found {}", crate::value::fmt_id(id))).label("not a mesh").help("mesh ids look like `Mesh#1`"));
                }
                let provenance = match get("provenance") {
                    None => false,
                    Some((_, Val::Bool(b), _)) => *b,
                    Some((_, other, sp)) => return self.fail(Diagnostic::error("type-mismatch", *sp, format!("`provenance` needs a bool, found {}", other.a_type())).label("not a bool")),
                };
                SessionCommand::Export { path, format, mesh, provenance }
            }
            "save" => SessionCommand::Save,
            "hash" => SessionCommand::Hash,
            _ => SessionCommand::Status,
        };
        let has_value = SESSION_VALUE_VERBS.contains(&verb.as_str());
        if !self.host.executes() {
            self.warn(Diagnostic::warning("plan-session", span, format!("`session.{verb}` is not simulated by a plan")).label("nothing is written or read"));
            self.events.push(Event::Session { span, command: cmd, applied: false, result: None });
            return Ok(if has_value { Val::Placeholder { n: self.plan_n, role: verb } } else { Val::obj(BTreeMap::new()) });
        }
        match self.host.session(&cmd) {
            Ok(res) => {
                let val = if has_value {
                    match json_to_val(&res, None, self.opts.max_value_len) {
                        Ok(v) => v,
                        Err(m) => return self.fail(Diagnostic::error("null-value", span, format!("`session.{verb}` returned a value the language cannot hold: {m}")).label("this command")),
                    }
                } else {
                    Val::obj(BTreeMap::new())
                };
                self.events.push(Event::Session { span, command: cmd, applied: true, result: Some(res) });
                Ok(val)
            }
            Err(e) => self.fail(Diagnostic::error("session-failed", span, format!("`session.{verb}` failed [{}]: {}", e.code, e.message)).label("this command had no effect")),
        }
    }

    /// Evaluate the arguments of a namespace command (`timeline.*`, `session.*`): named ones must be in `allowed`, one
    /// positional value is taken as `allowed[0]`. Returns `(name, value, span)` in written order.
    fn command_args(&mut self, ns: &str, verb: &str, call: &Call, allowed: &[&str]) -> IR<Vec<(String, Val, Span)>> {
        let mut args: Vec<(String, Val, Span)> = Vec::new();
        let mut pos_seen = false;
        for a in &call.args {
            match a {
                Arg::Named { name, value } => {
                    if !allowed.contains(&name.name.as_str()) {
                        let mut d = Diagnostic::error("unknown-argument", name.span, format!("`{ns}.{verb}` has no parameter `{}`", name.name)).label("unknown");
                        d = if allowed.is_empty() { d.help(format!("`{ns}.{verb}` takes no arguments")) } else { d.help(format!("parameters: {}", allowed.join(", "))) };
                        return self.fail(d);
                    }
                    if args.iter().any(|(k, ..)| *k == name.name) {
                        return self.fail(Diagnostic::error("duplicate-argument", name.span, format!("argument `{}` is given twice", name.name)).label("given again here"));
                    }
                    let v = self.eval(value)?;
                    args.push((name.name.clone(), v, value.span));
                }
                Arg::Pos(e) => {
                    if allowed.is_empty() || pos_seen || args.iter().any(|(k, ..)| k == allowed[0]) {
                        let d = Diagnostic::error("positional-ambiguous", e.span, "cannot tell which parameter this value is for")
                            .label("write `name: value`")
                            .help(format!("`{ns}.{verb}` takes {}", if allowed.is_empty() { "no arguments".to_owned() } else { format!("one string: {}", allowed[0]) }));
                        return self.fail(d);
                    }
                    pos_seen = true;
                    let v = self.eval(e)?;
                    args.push((allowed[0].to_owned(), v, e.span));
                }
                Arg::Spread(e) => return self.fail(Diagnostic::error("bad-arguments", e.span, format!("{ns} commands do not take `...`")).label("spread")),
            }
        }
        Ok(args)
    }

    /// `timeline.head()`: the object `{branch, node}` (a field is absent when it does not exist: detached head, empty state).
    fn timeline_head_value(&mut self, call: &Call, span: Span) -> IR<Val> {
        if let Some(a) = call.args.first() {
            let sp = match a {
                Arg::Named { name, .. } => name.span,
                Arg::Spread(e) | Arg::Pos(e) => e.span,
            };
            return self.fail(Diagnostic::error("unknown-argument", sp, "`timeline.head` takes no arguments").label("unexpected argument"));
        }
        if !self.host.executes() {
            return Ok(Val::Placeholder { n: self.plan_n, role: "head".to_owned() });
        }
        match self.host.timeline_head() {
            Ok(h) => {
                let mut m = BTreeMap::new();
                if let Some(b) = h.branch {
                    m.insert("branch".to_owned(), Val::Str(b));
                }
                if let Some(n) = h.node {
                    m.insert("node".to_owned(), Val::Str(n));
                }
                Ok(Val::obj(m))
            }
            Err(e) => self.fail(Diagnostic::error("timeline-failed", span, format!("`timeline.head` failed [{}]: {}", e.code, e.message)).label("this command failed")),
        }
    }

    /// A node hash given as a string argument: 64 lowercase hex characters, else a diagnostic at the argument.
    fn parse_hash(&mut self, key: &str, text: &str, args: &[(String, Val, Span)]) -> IR<Hash256> {
        match text.parse::<Hash256>() {
            Ok(h) => Ok(h),
            Err(e) => {
                let sp = args.iter().find(|(k, ..)| k == key).map_or(Span::new(0, 0, 0), |(_, _, sp)| *sp);
                let d = Diagnostic::error("invalid-argument", sp, format!("`{key}` is not a node hash: {e}"))
                    .label("not a hash")
                    .help("a node is named by its 64-character lowercase hex hash, as the timeline log prints it");
                self.fail(d)
            }
        }
    }

    fn is_defined(&self, name: &str) -> bool {
        self.scopes.iter().any(|s| s.contains_key(name)) || eval::constant(name).is_some()
    }

    /// Evaluate and convert the arguments, then check the resulting object against the schema.
    fn build_params<'c>(&mut self, call: &'c Call, info: &Callable) -> IR<(Map<String, Json>, Vec<Site<'c>>)> {
        struct Entry<'a> {
            key: String,
            val: Val,
            expr: &'a Expr,
            name_span: Option<Span>,
            direct: bool,
        }
        let view = info.doc.view();
        let mut entries: Vec<Entry<'c>> = Vec::new();
        let mut named_seen: BTreeMap<String, Span> = BTreeMap::new();
        let mut positional: Vec<&Expr> = Vec::new();
        for arg in &call.args {
            match arg {
                Arg::Named { name, value } => {
                    if let Some(prev) = named_seen.get(&name.name) {
                        let d = Diagnostic::error("duplicate-argument", name.span, format!("argument `{}` is given twice", name.name))
                            .label("given again here")
                            .note(*prev, "first given here");
                        return self.fail(d);
                    }
                    named_seen.insert(name.name.clone(), name.span);
                    // An enum value written without quotes: say that enum values are strings.
                    if let ExprKind::Var(vn) = &value.kind
                        && !self.is_defined(vn)
                        && view.property(&name.name).is_some_and(|pv| pv.non_null().choices().iter().any(|c| c.as_str() == Some(vn.as_str())))
                    {
                        let d = Diagnostic::error("unknown-variable", value.span, format!("`{vn}` is not defined"))
                            .label("an enum value, not a variable")
                            .help(format!("enum values are strings: write `{}: \"{vn}\"`", name.name));
                        return self.fail(d);
                    }
                    let v = self.eval(value)?;
                    entries.retain(|e| e.key != name.name);
                    entries.push(Entry { key: name.name.clone(), val: v, expr: value, name_span: Some(name.span), direct: true });
                }
                Arg::Spread(e) => match self.eval(e)? {
                    Val::Obj(m) => {
                        for (k, v) in m.iter() {
                            entries.retain(|x| x.key != *k);
                            entries.push(Entry { key: k.clone(), val: v.clone(), expr: e, name_span: None, direct: false });
                        }
                    }
                    other => {
                        let d = Diagnostic::error("type-mismatch", e.span, format!("`...` spreads an object, found {}", other.a_type())).label("not an object");
                        return self.fail(d);
                    }
                },
                Arg::Pos(e) => positional.push(e),
            }
        }
        if !positional.is_empty() {
            // The positional parameter: the one marked `"x-positional": true` in the schema, else the only required one.
            let marked: Vec<&str> = view
                .properties()
                .map(|m| m.iter().filter(|(_, v)| v.get("x-positional").and_then(Json::as_bool) == Some(true)).map(|(k, _)| k.as_str()).collect())
                .unwrap_or_default();
            let required = view.required();
            let target: Option<&str> = if marked.len() == 1 { Some(marked[0]) } else if required.len() == 1 { Some(required[0]) } else { None };
            let Some(key) = target.filter(|_| positional.len() == 1) else {
                let help = if target.is_some() {
                    "only one positional argument is allowed".to_owned()
                } else {
                    format!("positional arguments need an op with exactly one required (or `x-positional`) parameter; `{}` has {}", info.name, required.len())
                };
                let d = Diagnostic::error("positional-ambiguous", positional[0].span, "cannot tell which parameter this value is for").label("write `name: value`").help(help);
                return self.fail(d);
            };
            let key = key.to_owned();
            if let Some(sp) = named_seen.get(&key) {
                let d = Diagnostic::error("duplicate-argument", positional[0].span, format!("parameter `{key}` is also given by name")).label("given positionally").note(*sp, "given by name here");
                return self.fail(d);
            }
            let v = self.eval(positional[0])?;
            entries.retain(|e| e.key != key);
            entries.push(Entry { key, val: v, expr: positional[0], name_span: None, direct: true });
        }
        // Convert each entry, guided by its parameter schema.
        let mut params = Map::new();
        let mut failed = false;
        for e in &entries {
            let pview = view.property(&e.key);
            self.saw_placeholder = false;
            let mut path = Vec::new();
            match self.conv(&e.val, pview, &mut path) {
                Ok(j) => {
                    params.insert(e.key.clone(), j);
                }
                Err(issue) => {
                    let sp = if e.direct { locate(e.expr, &issue.path) } else { locate_in_spread(e.expr, &e.key, &issue.path) };
                    let mut d = Diagnostic::error(issue_code(issue.code), sp, issue.message).label("here");
                    if issue.code == "ambiguous-kind" {
                        d = d.help(kinds::UserIdError::AMBIGUOUS_HELP);
                    }
                    self.diags.push(d);
                    failed = true;
                }
            }
            if self.saw_placeholder {
                self.tainted = true;
                self.warn(
                    Diagnostic::warning("plan-unresolved", e.expr.span, "this value depends on an op result that is unknown without executing")
                        .label("placeholder `$plan...` emitted")
                        .help("wrap dependent statements in `batch { }` to lower them without executing"),
                );
            }
        }
        if failed {
            return Err(());
        }
        let sites: Vec<Site<'c>> = entries.iter().map(|e| Site { key: e.key.clone(), expr: e.expr, direct: e.direct }).collect();
        // Check the whole object.
        let obj = Json::Object(params);
        let mut issues = Vec::new();
        if self.batch.is_some() {
            schema::check_in_batch(view, &obj, true, &mut issues);
        } else {
            schema::check(view, &obj, true, &mut issues);
        }
        let Json::Object(params) = obj else { return self.internal(call.span, "arguments are not an object") };
        if issues.is_empty() {
            return Ok((params, sites));
        }
        let pinfo = view.params();
        // A misspelled required parameter is reported once, as the unknown argument with its suggestion, not also as missing.
        let suggested: Vec<&str> = issues
            .iter()
            .filter(|i| i.code == "unknown-property")
            .filter_map(|i| match i.path.first() {
                Some(PathSeg::Key(k)) => suggest(k, pinfo.iter().map(|p| p.name.as_str())),
                _ => None,
            })
            .collect();
        issues.retain(|i| i.code != "missing-property" || !i.message.split('`').nth(1).is_some_and(|n| suggested.contains(&n)));
        issues.sort_by_key(|i| i.code != "unknown-property"); // stable: unknown arguments first
        for iss in issues {
            let (span, mut d) = match iss.path.first() {
                Some(PathSeg::Key(k)) => {
                    let ent = entries.iter().find(|e| &e.key == k);
                    let span = match ent {
                        Some(e) if e.name_span.is_some() && iss.code == "unknown-property" => e.name_span.unwrap_or(call.span),
                        Some(e) if e.direct => locate(e.expr, &iss.path[1..]),
                        Some(e) => locate_in_spread(e.expr, k, &iss.path[1..]),
                        None => call.args_span,
                    };
                    (span, Diagnostic::error(issue_code(iss.code), span, iss.message.clone()))
                }
                _ => (call.args_span, Diagnostic::error(issue_code(iss.code), call.args_span, iss.message.clone())),
            };
            d = d.label("here");
            match iss.code {
                "unknown-property" => {
                    if let Some(PathSeg::Key(k)) = iss.path.first() {
                        d.message = format!("`{}` has no parameter `{k}`", info.name);
                        match suggest(k, pinfo.iter().map(|p| p.name.as_str())) {
                            Some(s) => d = d.help(format!("did you mean `{s}`?")),
                            None => {
                                let names: Vec<&str> = pinfo.iter().map(|p| p.name.as_str()).collect();
                                d = d.help(format!("parameters: {}", names.join(", ")));
                            }
                        }
                    }
                }
                "missing-property" => {
                    if let Some(name) = iss.message.split('`').nth(1)
                        && let Some(p) = pinfo.iter().find(|p| p.name == name)
                    {
                        d.message = format!("missing required argument `{name}` for `{}`", info.name);
                        d.label = Some(match &p.description {
                            Some(desc) => format!("`{name}: {}` is required: {}", p.ty, desc.lines().next().unwrap_or("").trim()),
                            None => format!("`{name}: {}` is required", p.ty),
                        });
                    }
                }
                "enum-violation" => {
                    let bad = json_at(&params, &iss.path);
                    let choices: Vec<String> = view_at(view, &iss.path).map(|v| v.non_null().choices().iter().filter_map(|c| c.as_str().map(str::to_owned)).collect()).unwrap_or_default();
                    if let (Some(Json::String(b)), false) = (bad, choices.is_empty()) {
                        match suggest(b, choices.iter().map(String::as_str)) {
                            Some(s) => d = d.help(format!("did you mean \"{s}\"?")),
                            None => d = d.help(format!("allowed values: {}", choices.iter().map(|c| format!("\"{c}\"")).collect::<Vec<_>>().join(", "))),
                        }
                    }
                }
                _ => {}
            }
            let _ = span;
            self.diags.push(d);
        }
        Err(())
    }

    /// Schema-guided conversion of a value to JSON. `Err` carries the path and message of the problem.
    fn conv(&mut self, v: &Val, view: Option<View<'_>>, path: &mut Vec<PathSeg>) -> Result<Json, Issue> {
        let issue = |path: &[PathSeg], code: &'static str, message: String| Issue { path: path.to_vec(), code, message };
        let in_batch = self.batch.is_some();
        let batch_id = self.batch.as_ref().map(|b| b.id);
        let nn = view.map(View::non_null);
        let is_id = nn.is_some_and(View::is_entity_id);
        let id_items = nn.and_then(View::id_array_items);
        // `x-entity-kind`: names this build does not know are ignored (a newer schema must not break an older language).
        let want_kinds: Vec<EntityKind> = if is_id || id_items.is_some() {
            view.and_then(View::entity_kind_names).map(|ns| ns.iter().filter_map(|n| n.parse::<EntityKind>().ok()).collect()).unwrap_or_default()
        } else {
            Vec::new()
        };
        let kind_ok = |id: EntityId, path: &[PathSeg]| -> Result<(), Issue> {
            if want_kinds.is_empty() || want_kinds.contains(&id.kind()) {
                return Ok(());
            }
            let wanted = want_kinds.iter().map(|k| k.to_string()).collect::<Vec<_>>().join(" or ");
            Err(Issue { path: path.to_vec(), code: "wrong-kind", message: format!("expected {wanted}, found {} ({})", crate::value::fmt_id(id), id.kind()) })
        };

        // A reference or placeholder as a single id string.
        let ref_str = |this: &mut Self, v: &Val, array_elem: bool, path: &[PathSeg]| -> Option<Result<Json, Issue>> {
            match v {
                Val::Ref { batch, index, role, sel, base } => {
                    if Some(*batch) != batch_id {
                        let why = if batch_id.is_some() {
                            "this output belongs to a different batch than the one being built".to_owned()
                        } else if !this.host.executes() {
                            "this output belongs to a batch, and a plan does not apply batches, so its ids are unknown".to_owned()
                        } else {
                            format!("the batch has been applied but produced no output `{role}[{}]`", if let Sel::At(k) = sel { k.to_string() } else { "*".to_owned() })
                        };
                        return Some(Err(issue(path, "stale-reference", why)));
                    }
                    let base = if base.is_empty() { "<result>" } else { base.as_str() };
                    let s = match sel {
                        Sel::All if array_elem => format!("${index}.{role}[*]"),
                        Sel::All if this.batch.as_ref().and_then(|b| b.records.get(*index)).map(|r| r.name.clone()).and_then(|op| this.role_cardinality(&op, role)) == Some(sm_op_api::Cardinality::One) => {
                            // A role declared `One` holds exactly one id, so the bare form is that id: `[0]`, by construction
                            // the same bytes as writing `[0]`.
                            format!("${index}.{role}[0]")
                        }
                        Sel::All => {
                            // A batch cannot know how many ids a role holds, and the same statement outside a batch
                            // fails when it holds several; so a bare role is never silently the first id.
                            return Some(Err(issue(
                                path,
                                "expected-one-entity",
                                format!("`{role}` may hold several ids and a batch cannot know how many; write `{base}.{role}[0]` to take one (or `[k]`)"),
                            )));
                        }
                        Sel::At(k) => format!("${index}.{role}[{k}]"),
                        Sel::Star => {
                            if !array_elem {
                                return Some(Err(issue(path, "type-mismatch", "`[*]` selects every output and is only valid as an element of an array parameter".into())));
                            }
                            format!("${index}.{role}[*]")
                        }
                    };
                    Some(Ok(Json::String(s)))
                }
                Val::Placeholder { n, role } => {
                    this.saw_placeholder = true;
                    Some(Ok(Json::String(format!("$plan.{n}.{role}"))))
                }
                Val::Outcome(_) => Some(Err(issue(path, "type-mismatch", "an op result is not a value; use one of its outputs, e.g. `r.body`".into()))),
                _ => None,
            }
        };

        if is_id {
            let one_id = |s: &EntitySet, path: &[PathSeg]| -> Result<Json, Issue> {
                match s.as_slice() {
                    [id] => Ok(Json::String(id.to_string())),
                    other => Err(issue(path, "expected-one-entity", format!("expected exactly one entity, found {} (pick one with `[0]`, or name a single id)", other.len()))),
                }
            };
            return match v {
                Val::Ents(s) => {
                    for id in s.iter() {
                        kind_ok(*id, path)?;
                    }
                    one_id(s, path)
                }
                Val::Str(s) => match kinds::parse_user_entity_text(s) {
                    Ok(id) => {
                        kind_ok(id, path)?;
                        self.string_id(id, path)
                    }
                    Err(kinds::UserIdError::AmbiguousKind(m)) => Err(issue(path, "ambiguous-kind", m)),
                    Err(kinds::UserIdError::Invalid(_)) if nn.is_some_and(|n| !n.alternatives().is_empty()) => Ok(Json::String(self.escape(s))),
                    Err(kinds::UserIdError::Invalid(m)) => Err(issue(path, "pattern-mismatch", m)),
                },
                other => match ref_str(self, other, false, path) {
                    Some(r) => r,
                    None => Err(issue(path, "type-mismatch", format!("expected an entity id, found {}", other.a_type()))),
                },
            };
        }
        if id_items.is_some() {
            let mut out = Vec::new();
            let push_elem = |this: &mut Self, el: &Val, path: &mut Vec<PathSeg>, out: &mut Vec<Json>| -> Result<(), Issue> {
                match el {
                    Val::Ents(s) => {
                        for id in s.iter() {
                            kind_ok(*id, path)?;
                        }
                        out.extend(s.iter().map(|i| Json::String(i.to_string())));
                    }
                    Val::Str(s) => match kinds::parse_user_entity_text(s) {
                        Ok(id) => {
                            kind_ok(id, path)?;
                            out.push(this.string_id(id, path)?)
                        }
                        Err(kinds::UserIdError::AmbiguousKind(m)) => return Err(issue(path, "ambiguous-kind", m)),
                        Err(kinds::UserIdError::Invalid(m)) => return Err(issue(path, "pattern-mismatch", m)),
                    },
                    other => match ref_str(this, other, true, path) {
                        Some(r) => out.push(r?),
                        None => return Err(issue(path, "type-mismatch", format!("expected an entity id, found {}", other.a_type()))),
                    },
                }
                Ok(())
            };
            match v {
                Val::List(items) => {
                    for (i, el) in items.iter().enumerate() {
                        path.push(PathSeg::Idx(i));
                        let r = push_elem(self, el, path, &mut out);
                        path.pop();
                        r?;
                    }
                }
                single => push_elem(self, single, path, &mut out)?,
            }
            return Ok(Json::Array(out));
        }
        match v {
            Val::Bool(b) => Ok(Json::Bool(*b)),
            Val::Int(i) => {
                if nn.is_some_and(View::is_float_only) {
                    if i.unsigned_abs() > (1u64 << 53) {
                        return Err(issue(
                            path,
                            "integer-precision",
                            format!("the integer {i} is not exactly representable as a number (limit 2^53); write it as a float, e.g. `{}`", fmt_num(*i as f64)),
                        ));
                    }
                    serde_json::Number::from_f64(*i as f64).map(Json::Number).ok_or_else(|| issue(path, "non-finite", "not a finite number".into()))
                } else {
                    Ok(Json::from(*i))
                }
            }
            Val::Num(n) => serde_json::Number::from_f64(*n).map(Json::Number).ok_or_else(|| issue(path, "non-finite", "not a finite number".into())),
            Val::Str(s) => Ok(Json::String(if in_batch { self.escape(s) } else { s.clone() })),
            Val::List(items) => {
                let iv = nn.and_then(View::items);
                let mut out = Vec::with_capacity(items.len());
                for (i, el) in items.iter().enumerate() {
                    path.push(PathSeg::Idx(i));
                    let r = self.conv(el, iv, path);
                    path.pop();
                    out.push(r?);
                }
                Ok(Json::Array(out))
            }
            Val::Obj(m) => {
                let mut out = Map::new();
                for (k, el) in m.iter() {
                    let pv = nn.and_then(|n| n.property(k));
                    path.push(PathSeg::Key(k.clone()));
                    let r = self.conv(el, pv, path);
                    path.pop();
                    out.insert(k.clone(), r?);
                }
                Ok(Json::Object(out))
            }
            Val::Ents(s) => Ok(Json::Array(s.iter().map(|i| Json::String(i.to_string())).collect())),
            other => match ref_str(self, other, true, path) {
                Some(Ok(_)) | None => Err(issue(path, "type-mismatch", format!("{} can only be used where an entity id is expected", other.a_type()))),
                Some(Err(e)) => Err(e),
            },
        }
    }

    /// An id named by a string: it must be live, like a bare id (`entity-not-found` otherwise), when the host executes.
    fn string_id(&self, id: EntityId, path: &[PathSeg]) -> Result<Json, Issue> {
        if self.host.executes() && !self.host.state().contains(id) {
            return Err(Issue { path: path.to_vec(), code: "entity-not-found", message: format!("there is no live {} ({})", crate::value::fmt_id(id), self.describe_live(id.kind())) });
        }
        Ok(Json::String(id.to_string()))
    }

    /// Inside a batch, a literal string starting with `$` is written `$$...` so it is never read as a reference.
    fn escape(&self, s: &str) -> String {
        if self.batch.is_some() && s.starts_with('$') { format!("${s}") } else { s.to_owned() }
    }

    // ---- expressions --------------------------------------------------------------------

    fn lookup_var(&mut self, name: &str, span: Span) -> IR<Val> {
        for scope in self.scopes.iter().rev() {
            if let Some(b) = scope.get(name) {
                return Ok(b.val.clone());
            }
        }
        if let Some(c) = eval::constant(name) {
            return Ok(c);
        }
        let names: Vec<&str> = self.scopes.iter().flat_map(|s| s.keys().map(String::as_str)).chain(["pi", "tau"]).collect();
        let mut d = Diagnostic::error("unknown-variable", span, format!("`{name}` is not defined")).label("unknown name");
        if let Some(s) = suggest(name, names.iter().copied()) {
            d = d.help(format!("did you mean `{s}`?"));
        }
        self.fail(d)
    }

    fn eval(&mut self, e: &Expr) -> IR<Val> {
        match &e.kind {
            ExprKind::Int(v) => Ok(Val::Int(*v)),
            ExprKind::Num(v) => Ok(Val::Num(*v)),
            ExprKind::Str(s) => Ok(Val::Str(s.clone())),
            ExprKind::Bool(b) => Ok(Val::Bool(*b)),
            ExprKind::List(items) => Ok(Val::list(items.iter().map(|i| self.eval(i)).collect::<IR<Vec<_>>>()?)),
            ExprKind::Obj(items) => {
                let mut m = BTreeMap::new();
                let mut seen = BTreeSet::new();
                for (k, v) in items {
                    if !seen.insert(k.name.as_str()) {
                        let d = Diagnostic::error("duplicate-key", k.span, format!("key `{}` appears twice in this object", k.name)).label("duplicate");
                        return self.fail(d);
                    }
                    // `key: null` omits the field.
                    if matches!(v.kind, ExprKind::Null) {
                        continue;
                    }
                    let val = self.eval(v)?;
                    m.insert(k.name.clone(), val);
                }
                Ok(Val::obj(m))
            }
            ExprKind::Null => self.internal(e.span, "`null` outside an object literal"),
            ExprKind::Chain(op, items) => self.eval_chain(*op, items),
            ExprKind::If(c, t, f) => match self.cond_known(c)? {
                Some(true) => self.eval(t),
                Some(false) => self.eval(f),
                None => Ok(Val::Placeholder { n: self.plan_n, role: "if".to_owned() }),
            },
            ExprKind::Range(lo, hi) => self.eval_range(lo, hi, e.span),
            ExprKind::Var(n) => self.lookup_var(n, e.span),
            ExprKind::Field(b, f) => {
                let v = self.eval(b)?;
                let mut r = self.field(v, f, e.span)?;
                if let (Val::Ref { base, .. }, ExprKind::Var(n)) = (&mut r, &b.kind) {
                    base.clone_from(n);
                }
                Ok(r)
            }
            ExprKind::Index(b, idx) => {
                let v = self.eval(b)?;
                let i = match idx {
                    Index::All => None,
                    Index::At(x) => Some(self.eval(x)?),
                };
                self.index(v, i, e.span)
            }
            ExprKind::Call(c) => self.eval_builtin_call(c, e.span),
            ExprKind::Unary(op, a) => {
                let v = self.eval(a)?;
                match eval::unop(*op, &v) {
                    Ok(r) => Ok(r),
                    Err(er) => self.eval_fail(er, e.span),
                }
            }
            ExprKind::Binary(op @ (BinOp::And | BinOp::Or), l, r) => {
                let lv = self.eval(l)?;
                if matches!(lv, Val::Outcome(_) | Val::Ref { .. } | Val::Placeholder { .. }) {
                    return self.eval_fail(eval::opaque_err(&lv), l.span);
                }
                let Val::Bool(lb) = lv else {
                    let d = Diagnostic::error("type-mismatch", l.span, format!("`and`/`or` need bools, found {}", lv.a_type())).label("not a bool");
                    return self.fail(d);
                };
                if (*op == BinOp::And && !lb) || (*op == BinOp::Or && lb) {
                    return Ok(Val::Bool(lb));
                }
                let rv = self.eval(r)?;
                if matches!(rv, Val::Outcome(_) | Val::Ref { .. } | Val::Placeholder { .. }) {
                    return self.eval_fail(eval::opaque_err(&rv), r.span);
                }
                match rv {
                    Val::Bool(b) => Ok(Val::Bool(b)),
                    other => {
                        let d = Diagnostic::error("type-mismatch", r.span, format!("`and`/`or` need bools, found {}", other.a_type())).label("not a bool");
                        self.fail(d)
                    }
                }
            }
            ExprKind::Binary(op, l, r) => {
                let lv = self.eval(l)?;
                let rv = self.eval(r)?;
                match eval::binop_lim(*op, &lv, &rv, self.opts.max_value_len) {
                    Ok(v) => Ok(v),
                    Err(er) if matches!(op, BinOp::Eq | BinOp::Ne) && er.0 == "type-mismatch" => {
                        let d = Diagnostic::error(er.0, e.span, er.1).label("different types").help(eval::eq_mismatch_help(&lv, &rv));
                        self.fail(d)
                    }
                    Err(er) => self.eval_fail(er, e.span),
                }
            }
            ExprKind::Entity(en) => self.resolve_entities(en, e.span).map(Val::Ents),
            ExprKind::Error => Err(()),
        }
    }

    /// A flat left-associative chain: `and`/`or` short-circuit; the rest fold left, appending to lists and strings in place.
    fn eval_chain(&mut self, op: BinOp, items: &[Expr]) -> IR<Val> {
        let Some(first) = items.first() else { return self.internal(Span::new(0, 0, 0), "empty operator chain") };
        if matches!(op, BinOp::And | BinOp::Or) {
            for it in items {
                let v = self.eval(it)?;
                if matches!(v, Val::Outcome(_) | Val::Ref { .. } | Val::Placeholder { .. }) {
                    return self.eval_fail(eval::opaque_err(&v), it.span);
                }
                let Val::Bool(b) = v else {
                    let d = Diagnostic::error("type-mismatch", it.span, format!("`and`/`or` need bools, found {}", v.a_type())).label("not a bool");
                    return self.fail(d);
                };
                if (op == BinOp::And && !b) || (op == BinOp::Or && b) {
                    return Ok(Val::Bool(b));
                }
            }
            return Ok(Val::Bool(op == BinOp::And));
        }
        let max = self.opts.max_value_len;
        let mut acc = self.eval(first)?;
        for it in &items[1..] {
            let rv = self.eval(it)?;
            let span = first.span.to(it.span);
            if op == BinOp::Add && eval::can_append(&acc, &rv) {
                let len = eval::seq_len(&acc).saturating_add(eval::seq_len(&rv));
                if len > max {
                    let what = if matches!(rv, Val::Str(_)) { "a string" } else { "a list" };
                    return self.eval_fail(eval::too_long(what, len, max), span);
                }
                eval::append_in_place(&mut acc, &rv);
                continue;
            }
            acc = match eval::binop_lim(op, &acc, &rv, max) {
                Ok(v) => v,
                Err(er) => return self.eval_fail(er, span),
            };
        }
        Ok(acc)
    }

    /// `lo..hi`: the inclusive list of integers (empty when `hi < lo`).
    fn eval_range(&mut self, lo: &Expr, hi: &Expr, span: Span) -> IR<Val> {
        let l = self.eval(lo)?;
        let h = self.eval(hi)?;
        for v in [&l, &h] {
            if matches!(v, Val::Outcome(_) | Val::Ref { .. } | Val::Placeholder { .. }) {
                return self.eval_fail(eval::opaque_err(v), span);
            }
        }
        let (Val::Int(a), Val::Int(b)) = (&l, &h) else {
            let d = Diagnostic::error("type-mismatch", span, format!("a range `a..b` needs integers, found {} and {}", l.a_type(), h.a_type()))
                .label("not integers")
                .help("use `range(a, b)` / `floor(x)` / `int(x)` to get integers");
            return self.fail(d);
        };
        if b < a {
            return Ok(Val::list(Vec::new()));
        }
        let len = (i128::from(*b) - i128::from(*a) + 1) as u128;
        if len > u128::from(self.opts.max_range_len) {
            return self.eval_fail(("range-too-long", format!("range has {len} elements, the limit is {}", self.opts.max_range_len)), span);
        }
        Ok(Val::list((*a..=*b).map(Val::Int).collect()))
    }

    fn eval_builtin_call(&mut self, c: &Call, span: Span) -> IR<Val> {
        let name = c.path.dotted();
        if c.path.segs.len() == 2 && c.path.segs[0].name == "timeline" && c.path.segs[1].name == "head" {
            return self.exec_timeline(c, span);
        }
        if c.path.segs.len() == 2 && c.path.segs[0].name == "timeline" {
            let d = Diagnostic::error("op-in-expression", c.path.span, format!("`{name}` is a statement, not a function"))
                .label("timeline commands cannot be used inside expressions")
                .help("write it as its own statement");
            return self.fail(d);
        }
        if c.path.segs.len() == 2 && c.path.segs[0].name == "session" {
            if SESSION_VALUE_VERBS.contains(&c.path.segs[1].name.as_str()) || !SESSION_VERBS.contains(&c.path.segs[1].name.as_str()) {
                return self.exec_session(c, span);
            }
            let d = Diagnostic::error("op-in-expression", c.path.span, format!("`{name}` is a statement, not a function"))
                .label("this session command cannot be used inside expressions")
                .help("write it as its own statement (`session.hash()` and `session.status()` do have values)");
            return self.fail(d);
        }
        if self.is_registry_call(c) {
            let full = if c.path.segs.len() == 1 { self.aliases.get(&name).cloned().unwrap_or(name.clone()) } else { name.clone() };
            // A query is read-only and deterministic for a given state, so it may appear inside an expression.
            if self.lookup_callable(&full).is_some_and(|i| i.is_query) {
                let saved = (self.tainted, self.saw_placeholder);
                let r = self.exec_call(c, span);
                (self.tainted, self.saw_placeholder) = saved;
                return match r? {
                    Val::Outcome(Outcome::Plan { n }) => Ok(Val::Placeholder { n, role: "result".to_owned() }),
                    v => Ok(v),
                };
            }
            let d = Diagnostic::error("op-in-expression", c.path.span, format!("`{name}` is not a function"))
                .label("ops cannot be used inside expressions (queries can)")
                .help("call it as a statement, or as the whole right-hand side of `let`: `let r = op(...)`");
            return self.fail(d);
        }
        let mut args = Vec::new();
        for a in &c.args {
            match a {
                Arg::Pos(e) => args.push(self.eval(e)?),
                Arg::Named { name, .. } => {
                    return self.fail(Diagnostic::error("bad-arguments", name.span, "builtin functions take positional arguments").label("named argument"));
                }
                Arg::Spread(e) => return self.fail(Diagnostic::error("bad-arguments", e.span, "builtin functions do not take `...`").label("spread")),
            }
        }
        match eval::call_builtin_lim(&name, &args, self.opts.max_range_len, self.opts.max_value_len) {
            Ok(v) => {
                // Ids named by strings must be live, exactly like bare ids (`Volume#7`).
                if name == "ents"
                    && self.host.executes()
                    && let Val::Ents(set) = &v
                    && let Some(dead) = set.iter().find(|i| !self.host.state().contains(**i))
                {
                    let d = Diagnostic::error("entity-not-found", span, format!("there is no live {}", crate::value::fmt_id(*dead)))
                        .label("not in the current state")
                        .help(self.describe_live(dead.kind()));
                    return self.fail(d);
                }
                Ok(v)
            }
            Err(er) => self.eval_fail(er, span),
        }
    }

    fn field(&mut self, v: Val, f: &Ident, span: Span) -> IR<Val> {
        let name = f.name.as_str();
        match v {
            Val::Obj(m) => match m.get(name) {
                Some(x) => Ok(x.clone()),
                None => {
                    let mut d = Diagnostic::error("unknown-field", f.span, format!("the object has no field `{name}`")).label("no such field");
                    match suggest(name, m.keys().map(String::as_str)) {
                        Some(s) => d = d.help(format!("did you mean `{s}`?")),
                        None => d = d.help(format!("fields: {}", m.keys().cloned().collect::<Vec<_>>().join(", "))),
                    }
                    self.fail(d)
                }
            },
            Val::Outcome(Outcome::Concrete { created, removed, outputs }) => match name {
                "created" => Ok(Val::Ents(created)),
                "removed" => Ok(Val::Ents(removed)),
                _ => match outputs.get(name) {
                    Some(ids) => Ok(Val::Ents(ids.clone())),
                    None => self.no_output(name, f.span, outputs.keys().map(String::as_str).chain(["created", "removed"])),
                },
            },
            Val::Outcome(Outcome::Applied { outputs }) => match name {
                "created" | "removed" => self.eval_fail(("not-concrete", format!("`{name}` is not available for an op that ran inside a batch (only its named outputs are)")), span),
                _ => match outputs.get(name) {
                    Some(ids) => Ok(Val::Ents(ids.clone())),
                    None => self.no_output(name, f.span, outputs.keys().map(String::as_str)),
                },
            },
            Val::Outcome(Outcome::Batch { batch, index }) => match name {
                "created" | "removed" => self.eval_fail(("not-concrete", format!("`{name}` is only known after the batch is applied")), span),
                _ => {
                    let op = self.batch.as_ref().filter(|b| b.id == batch).and_then(|b| b.records.get(index)).map(|r| r.name.clone());
                    if let Some(op) = op
                        && let Some(r) = self.role_check(&op, name, f.span, false)
                    {
                        return r;
                    }
                    Ok(Val::Ref { batch, index, role: name.to_owned(), sel: Sel::All, base: String::new() })
                }
            },
            Val::Outcome(Outcome::Plan { n }) => {
                if let Some(op) = self.plan_ops.get(&n).cloned()
                    && let Some(r) = self.role_check(&op, name, f.span, true)
                {
                    return r;
                }
                Ok(Val::Placeholder { n, role: name.to_owned() })
            }
            Val::Placeholder { n, role } => Ok(Val::Placeholder { n, role: format!("{role}.{name}") }),
            other => self.fail(Diagnostic::error("type-mismatch", f.span, format!("{} has no fields", other.a_type())).label("field access")),
        }
    }

    /// Static role check from the op's declared `OUTPUTS`: `Some(Err)` (a diagnostic with did-you-mean) when the op
    /// declares roles and `name` is not one of them. Ops that declare none are unchecked. `created`/`removed` are
    /// accepted when `allow_delta` (plan mode, where the real result has them).
    fn role_check(&mut self, op: &str, name: &str, span: Span, allow_delta: bool) -> Option<IR<Val>> {
        let info = self.lookup_callable(op)?;
        if info.outputs.is_empty() || info.outputs.iter().any(|r| r.name == name) || (allow_delta && (name == "created" || name == "removed")) {
            return None;
        }
        let names: Vec<&str> = info.outputs.iter().map(|r| r.name).collect();
        Some(self.no_output(name, span, names.into_iter()))
    }

    /// Declared cardinality of `role` of op `op`, if the op declares it.
    fn role_cardinality(&mut self, op: &str, role: &str) -> Option<sm_op_api::Cardinality> {
        self.lookup_callable(op)?.outputs.iter().find(|r| r.name == role).map(|r| r.cardinality)
    }

    fn no_output<'a>(&mut self, name: &str, span: Span, roles: impl Iterator<Item = &'a str>) -> IR<Val> {
        let roles: Vec<&str> = roles.collect();
        let mut d = Diagnostic::error("unknown-field", span, format!("the op result has no output `{name}`")).label("no such output");
        match suggest(name, roles.iter().copied()) {
            Some(s) => d = d.help(format!("did you mean `{s}`?")),
            None => d = d.help(format!("available: {}", roles.join(", "))),
        }
        self.fail(d)
    }

    fn index(&mut self, v: Val, i: Option<Val>, span: Span) -> IR<Val> {
        match (v, i) {
            (Val::List(l), Some(Val::Int(k))) => match norm_index(k, l.len()) {
                Some(ix) => Ok(l[ix].clone()),
                None => self.eval_fail(("index-out-of-range", format!("index {k} is out of range for a list of {} elements", l.len())), span),
            },
            (Val::List(l), None) => Ok(Val::List(l)),
            (Val::Ents(s), None) => Ok(Val::Ents(s)),
            (Val::Ents(s), Some(Val::Int(k))) => match norm_index(k, s.len()) {
                Some(ix) => Ok(Val::Ents(EntitySet::single(s.as_slice()[ix]))),
                None => self.eval_fail(("index-out-of-range", format!("index {k} is out of range for {} entities", s.len())), span),
            },
            (Val::Obj(m), Some(Val::Str(k))) => match m.get(&k) {
                Some(x) => Ok(x.clone()),
                None => self.eval_fail(("unknown-field", format!("the object has no key {k:?}")), span),
            },
            (Val::Ref { batch, index, role, sel: Sel::All, base }, Some(Val::Int(k))) if k >= 0 => Ok(Val::Ref { batch, index, role, sel: Sel::At(k as usize), base }),
            (Val::Ref { batch, index, role, sel: Sel::All, base }, None) => Ok(Val::Ref { batch, index, role, sel: Sel::Star, base }),
            (Val::Ref { sel: Sel::All, .. }, Some(Val::Int(k))) => self.eval_fail(
                (
                    "type-mismatch",
                    format!("a batch reference cannot be indexed from the end (`[{k}]`): the reference grammar has no negative index and a batch cannot know how many ids a role holds; write an explicit `[k]` such as `[0]`"),
                ),
                span,
            ),
            (Val::Ref { .. }, _) => self.eval_fail(("type-mismatch", "a batch output can be indexed once, with a non-negative integer or `*`".into()), span),
            (Val::Placeholder { n, role }, _) => Ok(Val::Placeholder { n, role }),
            (other, Some(i)) => self.eval_fail(("type-mismatch", format!("cannot index {} with {}", other.a_type(), i.a_type())), span),
            (other, None) => self.eval_fail(("type-mismatch", format!("`[*]` does not apply to {}", other.a_type())), span),
        }
    }

    // ---- entity lists -------------------------------------------------------------------

    fn describe_live(&self, kind: EntityKind) -> String {
        let n = self.host.state().count_of(kind);
        let sing = kinds::singular_of(kind);
        if n == 0 {
            let note = match kind {
                EntityKind::Surface | EntityKind::Curve => {
                    "; these are geometry carriers (Cubit's surfaces and curves are `faces` and `edges`)"
                }
                _ => "",
            };
            return format!("there are no live {sing} entities{note}");
        }
        let first: Vec<String> = self.host.state().entities_of(kind).take(4).map(crate::value::fmt_id).collect();
        format!("{n} live {sing} entit{}: {}{}", if n == 1 { "y" } else { "ies" }, first.join(", "), if n > 4 { ", ..." } else { "" })
    }

    /// An id serial: an integer expression, at least 0 and within the 56-bit serial space.
    fn eval_serial(&mut self, e: &Expr) -> IR<u64> {
        match self.eval(e)? {
            Val::Int(v) if (0..=EntityId::MAX_SERIAL as i64).contains(&v) => Ok(v as u64),
            Val::Int(v) => {
                let d = Diagnostic::error("bad-id-range", e.span, format!("the serial {v} is outside the id space (0 to 2^56 - 1)")).label("not a valid serial");
                self.fail(d)
            }
            other => {
                let d = Diagnostic::error("type-mismatch", e.span, format!("an id serial must be an integer, found {}", other.a_type())).label("not an integer");
                self.fail(d)
            }
        }
    }

    /// Warn when a block resolves a selection after it already emitted ops: the selection sees the state at block entry.
    fn warn_batch_snapshot(&mut self, span: Span) {
        let emitted = self.batch.as_ref().is_some_and(|b| !b.records.is_empty());
        if emitted && self.snap_warned.insert((span.file, span.lo, span.hi)) {
            self.warn(
                Diagnostic::warning("batch-snapshot", span, "this selection is resolved against the state at the start of the batch")
                    .label("entities created by earlier statements of the block are not included")
                    .help("name new entities through the op result, e.g. `p.body[0]`, or run the selection outside the batch"),
            );
        }
    }

    fn resolve_entities(&mut self, en: &EntityExpr, span: Span) -> IR<EntitySet> {
        match en {
            EntityExpr::Ids { kind, items, .. } => {
                let mut ids: Vec<EntityId> = Vec::new();
                let exec = self.host.executes();
                for it in items {
                    match it {
                        IdItem::One(e) => {
                            let serial = self.eval_serial(e)?;
                            let at = if items.len() == 1 { span } else { e.span };
                            let Some(id) = EntityId::new(*kind, serial) else {
                                return self.fail(Diagnostic::error("entity-not-found", at, "serial too large").label("exceeds the 56-bit serial space"));
                            };
                            if exec && !self.host.state().contains(id) {
                                let mut d = Diagnostic::error("entity-not-found", at, format!("there is no live {}", crate::value::fmt_id(id))).label("not in the current state");
                                d = if self.batch.is_some() {
                                    d.help("a batch resolves ids against the state at its start, so ids created inside the block cannot be named; use the op result instead, e.g. `p.body[0]`")
                                } else {
                                    d.help(self.describe_live(*kind))
                                };
                                return self.fail(d);
                            }
                            ids.push(id);
                        }
                        IdItem::Range { lo, hi, step, span: rs } => {
                            self.warn_batch_snapshot(*rs);
                            let (l, h) = (self.eval_serial(lo)?, self.eval_serial(hi)?);
                            let st = match step {
                                Some(s) => self.eval_serial(s)?,
                                None => 1,
                            };
                            if h < l || st < 1 {
                                let msg = if st < 1 { "range step must be at least 1".to_owned() } else { format!("range start {l} is greater than its end {h}") };
                                return self.fail(Diagnostic::error("bad-id-range", *rs, msg).label("invalid range").help("ranges are inclusive and ascending: `Volume#1..5`"));
                            }
                            let set = live_range(self.host.state(), *kind, l, h, st);
                            if set.is_empty() {
                                let h = self.describe_live(*kind);
                                self.warn(Diagnostic::warning("empty-selection", *rs, "the range selects no live entity").label("empty").help(h));
                            }
                            ids.extend_from_slice(set.as_slice());
                        }
                    }
                }
                Ok(EntitySet::from_vec(ids))
            }
            EntityExpr::All { kind, .. } => {
                self.warn_batch_snapshot(span);
                let set = all_of(self.host.state(), *kind);
                if set.is_empty() {
                    let d = Diagnostic::warning("empty-selection", span, format!("there are no live {} entities", kinds::singular_of(*kind))).label("empty");
                    let d = if matches!(kind, EntityKind::Surface | EntityKind::Curve) {
                        d.help("geometry carriers (`geom_surfaces`, `geom_curves`) are not the Cubit surfaces and curves; those are `faces` and `edges`")
                    } else {
                        d
                    };
                    self.warn(d);
                }
                Ok(set)
            }
            EntityExpr::In { kind, within, .. } => {
                let w = self.eval(within)?;
                let set = match w {
                    Val::Ents(s) => s.to_sorted(),
                    other => {
                        let er = match &other {
                            Val::Ref { .. } | Val::Outcome(_) | Val::Placeholder { .. } => eval::opaque_err(&other),
                            _ => ("type-mismatch", format!("`in` needs an entity set on its right, found {}", other.a_type())),
                        };
                        return self.eval_fail(er, within.span);
                    }
                };
                self.warn_batch_snapshot(span);
                let Some(rel) = self.host.relations() else {
                    let d = Diagnostic::error("no-relations", span, "this host cannot resolve relations between entities")
                        .label("`in` needs topology")
                        .help("the host must provide a `Relations` resolver (see `LANGUAGE.md` §5.3)");
                    return self.fail(d);
                };
                match rel.related(self.host.state(), *kind, set.as_slice()) {
                    Ok(v) => {
                        let st = self.host.state();
                        let out: Vec<EntityId> = v.into_iter().filter(|i| i.kind() == *kind && st.contains(*i)).collect();
                        let out = EntitySet::from_vec(out);
                        if out.is_empty() {
                            self.warn(Diagnostic::warning("empty-selection", span, "no related entities").label("empty"));
                        }
                        Ok(out)
                    }
                    Err(m) => self.fail(Diagnostic::error("relation-failed", span, format!("relation lookup failed: {m}")).label("here")),
                }
            }
        }
    }
}

/// True if `v` is, or holds in a list or object, a symbolic value of batch `id`.
fn mentions_batch(v: &Val, id: u64) -> bool {
    match v {
        Val::Outcome(Outcome::Batch { batch, .. }) | Val::Ref { batch, .. } => *batch == id,
        Val::List(l) => l.iter().any(|x| mentions_batch(x, id)),
        Val::Obj(m) => m.values().any(|x| mentions_batch(x, id)),
        _ => false,
    }
}

/// Replace the symbolic values of batch `id` inside `v` (also nested in lists and objects) by what the batch produced.
/// A selection past the end of a role stays a reference; using it reports that the batch has no such output.
fn rebind_val(v: &mut Val, id: u64, delta: &Delta) {
    match v {
        Val::Outcome(Outcome::Batch { batch, index }) if *batch == id => {
            let prefix = format!("{index}.");
            let outputs = delta.outputs.iter().filter_map(|(k, o)| k.strip_prefix(&prefix).map(|r| (r.to_owned(), EntitySet::from_ordered(o.clone())))).collect();
            *v = Val::Outcome(Outcome::Applied { outputs });
        }
        Val::Ref { batch, index, role, sel, .. } if *batch == id => {
            if let Some(ids) = delta.outputs.get(&format!("{index}.{role}")) {
                let picked = match sel {
                    Sel::All | Sel::Star => Some(EntitySet::from_ordered(ids.clone())),
                    Sel::At(k) => ids.get(*k).map(|i| EntitySet::single(*i)),
                };
                if let Some(p) = picked {
                    *v = Val::Ents(p);
                }
            }
        }
        Val::List(l) => {
            for x in Arc::make_mut(l).iter_mut() {
                rebind_val(x, id, delta);
            }
        }
        Val::Obj(m) => {
            for x in Arc::make_mut(m).values_mut() {
                rebind_val(x, id, delta);
            }
        }
        _ => {}
    }
}

/// Rewrite canonical ids in a host message (`TopoFace#1`, `Surface#7`) into the spelling scripts use (`Face#1`,
/// `GeomSurface#7`), so an id copied out of an error can be pasted into a script.
fn respell(msg: &str) -> String {
    let b = msg.as_bytes();
    let mut out = String::with_capacity(msg.len());
    let mut last = 0;
    let mut i = 0;
    while i < b.len() {
        if b[i] != b'#' {
            i += 1;
            continue;
        }
        let mut lo = i;
        while lo > last && b[lo - 1].is_ascii_alphabetic() {
            lo -= 1;
        }
        let mut hi = i + 1;
        while hi < b.len() && b[hi].is_ascii_digit() {
            hi += 1;
        }
        if lo < i
            && hi > i + 1
            && let Ok(id) = msg[lo..hi].parse::<EntityId>()
        {
            out.push_str(&msg[last..lo]);
            out.push_str(&crate::value::fmt_id(id));
            last = hi;
        }
        i = hi.max(i + 1);
    }
    out.push_str(&msg[last..]);
    out
}

/// A copy of `json` with canonical entity ids written the way scripts spell them (`TopoFace#1` becomes `Face#1`, the
/// geometry carrier `Surface#7` becomes `GeomSurface#7`), in every string, at any depth. Op error messages get the same
/// treatment. [`Event`] records, query parameters and query results stay in wire form (they are what the host applied,
/// stored and hashed); this is for showing them to a person or a model next to the script's own spelling.
pub fn respell_json(json: &Json) -> Json {
    match json {
        Json::String(s) => Json::String(respell(s)),
        Json::Array(a) => Json::Array(a.iter().map(respell_json).collect()),
        Json::Object(m) => Json::Object(m.iter().map(|(k, v)| (k.clone(), respell_json(v))).collect()),
        other => other.clone(),
    }
}

/// The batch reference an error message quotes (`batch reference '$0.voluem[0]': no such output`).
fn quoted_batch_ref(msg: &str) -> Option<String> {
    let lo = msg.find("'$")? + 1;
    let len = msg[lo..].find('\'')?;
    Some(msg[lo..lo + len].to_owned())
}

/// The member argument that wrote reference string `r`: its span, parameter name and the op that holds it.
fn batch_site<'r>(r: &str, ops: &'r [OpRecord], sites: &[Vec<OwnedSite>]) -> Option<(Span, String, &'r str)> {
    for (op, member) in ops.iter().zip(sites) {
        let mut path = Vec::new();
        if !find_str(&op.params, r, &mut path) {
            continue;
        }
        let Some(PathSeg::Key(k)) = path.first() else { continue };
        let Some(site) = member.iter().find(|s| &s.key == k) else { continue };
        return Some((site.locate(&path[1..]), k.clone(), op.name.as_str()));
    }
    None
}

/// Entity ids (`Kind#serial`) written in a message.
fn id_texts(msg: &str) -> Vec<String> {
    let b = msg.as_bytes();
    let mut out = Vec::new();
    for (i, &c) in b.iter().enumerate() {
        if c != b'#' {
            continue;
        }
        let mut lo = i;
        while lo > 0 && b[lo - 1].is_ascii_alphabetic() {
            lo -= 1;
        }
        let mut hi = i + 1;
        while hi < b.len() && b[hi].is_ascii_digit() {
            hi += 1;
        }
        if lo < i && hi > i + 1 {
            out.push(msg[lo..hi].to_owned());
        }
    }
    out
}

fn find_str(j: &Json, want: &str, path: &mut Vec<PathSeg>) -> bool {
    match j {
        Json::String(s) => s == want,
        Json::Array(a) => {
            for (i, x) in a.iter().enumerate() {
                path.push(PathSeg::Idx(i));
                if find_str(x, want, path) {
                    return true;
                }
                path.pop();
            }
            false
        }
        Json::Object(m) => {
            for (k, x) in m {
                path.push(PathSeg::Key(k.clone()));
                if find_str(x, want, path) {
                    return true;
                }
                path.pop();
            }
            false
        }
        _ => false,
    }
}

/// The argument (and the element inside it) that supplied an entity named by an op error: its span and parameter name.
fn localize(msg: &str, params: &Json, sites: &[Site<'_>]) -> Option<(Span, String)> {
    for cand in id_texts(msg) {
        let mut path = Vec::new();
        if !find_str(params, &cand, &mut path) {
            continue;
        }
        let Some(PathSeg::Key(k)) = path.first() else { continue };
        let Some(site) = sites.iter().find(|s| &s.key == k) else { continue };
        let sp = if site.direct { locate(site.expr, &path[1..]) } else { locate_in_spread(site.expr, k, &path[1..]) };
        return Some((sp, k.clone()));
    }
    None
}

fn json_at<'j>(params: &'j Map<String, Json>, path: &[PathSeg]) -> Option<&'j Json> {
    let Some(PathSeg::Key(k)) = path.first() else { return None };
    let mut cur = params.get(k)?;
    for seg in &path[1..] {
        cur = match (seg, cur) {
            (PathSeg::Key(k), Json::Object(m)) => m.get(k)?,
            (PathSeg::Idx(i), Json::Array(a)) => a.get(*i)?,
            _ => return None,
        };
    }
    Some(cur)
}

fn view_at<'a>(view: View<'a>, path: &[PathSeg]) -> Option<View<'a>> {
    let mut cur = view;
    for seg in path {
        cur = match seg {
            PathSeg::Key(k) => cur.non_null().property(k)?,
            PathSeg::Idx(_) => cur.non_null().items()?,
        };
    }
    Some(cur)
}

/// A query result as a language value. Where the result schema says "entity id" or "array of entity ids" (the
/// `EntityId` pattern), the strings become entity sets, so `bc.sideset(faces: geom.find_faces(...))` works.
fn json_to_val(v: &Json, view: Option<View<'_>>, max_len: usize) -> Result<Val, String> {
    if let Some(vw) = view {
        let nn = vw.non_null();
        if nn.is_entity_id()
            && let Json::String(s) = v
            && let Ok(id) = kinds::parse_entity_text(s)
        {
            return Ok(Val::Ents(EntitySet::single(id)));
        }
        if nn.id_array_items().is_some()
            && let Json::Array(a) = v
        {
            if a.len() > max_len {
                return Err(format!("a list of {} ids exceeds the limit of {max_len}", a.len()));
            }
            let ids: Option<Vec<EntityId>> = a.iter().map(|x| x.as_str().and_then(|s| kinds::parse_entity_text(s).ok())).collect();
            if let Some(ids) = ids {
                return Ok(Val::Ents(EntitySet::from_ordered(ids)));
            }
        }
    }
    match v {
        Json::Array(a) => {
            if a.len() > max_len {
                return Err(format!("a list of {} elements exceeds the limit of {max_len}", a.len()));
            }
            let iv = view.and_then(|w| w.non_null().items());
            Ok(Val::list(a.iter().map(|x| json_to_val(x, iv, max_len)).collect::<Result<_, _>>()?))
        }
        Json::Object(m) => Ok(Val::obj(
            m.iter()
                .map(|(k, x)| Ok((k.clone(), json_to_val(x, view.and_then(|w| w.non_null().property(k)), max_len)?)))
                .collect::<Result<_, String>>()?,
        )),
        other => Val::from_json_limited(other, max_len),
    }
}

fn norm_index(k: i64, len: usize) -> Option<usize> {
    let ix = if k < 0 { i64::try_from(len).ok()? + k } else { k };
    usize::try_from(ix).ok().filter(|i| *i < len)
}

fn issue_code(c: &'static str) -> &'static str {
    match c {
        "unknown-property" => "unknown-argument",
        "missing-property" => "missing-argument",
        "enum-violation" | "out-of-range" | "pattern-mismatch" => "invalid-argument",
        other => other,
    }
}

/// Span of the sub-expression a value path points to, descending into list and object literals.
pub fn locate(e: &Expr, path: &[PathSeg]) -> Span {
    match (&e.kind, path.first()) {
        (ExprKind::List(items), Some(PathSeg::Idx(i))) => items.get(*i).map_or(e.span, |x| locate(x, &path[1..])),
        (ExprKind::Obj(items), Some(PathSeg::Key(k))) => items.iter().find(|(kk, _)| kk.name == *k).map_or(e.span, |(_, x)| locate(x, &path[1..])),
        _ => e.span,
    }
}

fn locate_in_spread(e: &Expr, key: &str, rest: &[PathSeg]) -> Span {
    if let ExprKind::Obj(items) = &e.kind
        && let Some((_, x)) = items.iter().find(|(k, _)| k.name == key)
    {
        return locate(x, rest);
    }
    e.span
}
