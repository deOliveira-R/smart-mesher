//! [`Server`]: the single-writer protocol endpoint over one [`Workspace`].

use std::path::{Path, PathBuf};
use std::sync::Arc;

use serde_json::{Value, json};
use sm_engine::{ApplyOptions, EngineError, ProjectConfig, Workspace, WorkspaceBuilder};
use sm_lang::diag::render_all;
use sm_lang::{Event, Interpreter, Options, SourceMap};
use sm_op_api::{OpInfoOwned, QueryInfoOwned, Rev};
use sm_protocol::{
    ApplyResult, CatalogResult, Endpoint, HashResult, PROTOCOL_VERSION, Request, RpcError, ScriptResult,
};

use crate::error::{ServerError, SessionError};
use crate::host::{ExportParams, SessionHost, resolve_path, status_result};

/// Rebuilds the distribution (plugins and services) for `session.open`.
type Opener = Arc<dyn Fn() -> WorkspaceBuilder + Send + Sync>;

/// A single-writer protocol server over one workspace.
///
/// `call` takes `&mut self`, so requests run one at a time in arrival order; a project has
/// exactly one writer because it has exactly one [`Workspace`]. Transports (see
/// [`transport`](crate::transport)) only move text; every method is dispatched here.
pub struct Server {
    ws: Workspace,
    opener: Option<Opener>,
}

impl Server {
    /// A server over an in-memory workspace built from `builder`. `session.open` is not
    /// available (the builder is consumed); use [`from_factory`](Self::from_factory) for that.
    pub fn new(builder: WorkspaceBuilder) -> Result<Self, ServerError> {
        Ok(Self { ws: builder.memory()?, opener: None })
    }

    /// A server over an in-memory workspace from `factory`, which `session.open` calls again to
    /// build the workspace of the opened project (same plugins, services, clock and author).
    pub fn from_factory(factory: impl Fn() -> WorkspaceBuilder + Send + Sync + 'static) -> Result<Self, ServerError> {
        let ws = factory().memory()?;
        Ok(Self { ws, opener: Some(Arc::new(factory)) })
    }

    /// A server over an existing workspace (for example one opened with `smesh -p dir`).
    pub fn from_workspace(ws: Workspace) -> Self {
        Self { ws, opener: None }
    }

    /// A server over the project at `dir`, opened (and replayed strictly) with `builder`.
    pub fn open(builder: WorkspaceBuilder, dir: impl AsRef<Path>) -> Result<Self, ServerError> {
        Ok(Self::from_workspace(builder.open(dir)?))
    }

    /// A server over a new project at `dir` (absent or empty directory) named `name`.
    pub fn create(builder: WorkspaceBuilder, dir: impl AsRef<Path>, name: &str) -> Result<Self, ServerError> {
        Ok(Self::from_workspace(builder.create(dir, ProjectConfig::new(name))?))
    }

    /// Let `session.open` rebuild the distribution with `factory`.
    #[must_use]
    pub fn with_opener(mut self, factory: impl Fn() -> WorkspaceBuilder + Send + Sync + 'static) -> Self {
        self.opener = Some(Arc::new(factory));
        self
    }

    /// The workspace (read-only: all mutation goes through [`Endpoint::call`]).
    pub fn workspace(&self) -> &Workspace {
        &self.ws
    }

    /// Give the workspace back (to close it, for example).
    pub fn into_workspace(self) -> Workspace {
        self.ws
    }

    fn host(&mut self) -> SessionHost<'_> {
        SessionHost::new(&mut self.ws)
    }

    fn catalog(&self) -> Result<Value, RpcError> {
        let reg = self.ws.registry();
        let ops = reg.catalog().into_iter().collect::<Result<Vec<_>, _>>().map_err(internal)?;
        let queries = reg.query_catalog().into_iter().collect::<Result<Vec<_>, _>>().map_err(internal)?;
        let batch_schema = to_value(&reg.batch_schema().map_err(internal)?)?;
        let result = CatalogResult {
            protocol: PROTOCOL_VERSION,
            ops: ops.iter().map(OpInfoOwned::from).collect(),
            queries: queries.iter().map(QueryInfoOwned::from).collect(),
            batch_schema,
        };
        to_value(&result)
    }

    fn run_script(&mut self, text: &str, path: Option<&str>, includes: std::collections::BTreeMap<String, String>) -> Result<Value, RpcError> {
        let name = path.unwrap_or("<script>");
        let mut host = SessionHost::new(&mut self.ws).with_includes(includes);
        let (events, diagnostics, rendered, output, failed) = {
            let mut interp = Interpreter::new(&mut host, Options::default());
            let report = interp.run_source(name, text);
            let map = interp.sources();
            let events: Vec<Value> = report.events.iter().map(|e| event_json(e, map)).collect();
            let output = report
                .events
                .iter()
                .filter_map(|e| match e {
                    Event::Print { text, .. } | Event::Help { text, .. } => Some(format!("{text}\n")),
                    _ => None,
                })
                .collect::<String>();
            let rendered = render_all(&report.diagnostics, map);
            (events, report.diagnostics_json(map), rendered, output, report.diagnostics.iter().find(|d| d.is_error()).map(|d| (d.code.to_string(), d.message.clone())))
        };
        let state = self.ws.state_hash();
        let result = ScriptResult::new(state, output, events, diagnostics);
        match failed {
            None => to_value(&result),
            Some((code, message)) => {
                // Ops applied before the failing statement stay applied (each is a timeline node).
                let mut data = to_value(&result)?;
                data["rendered"] = Value::String(rendered);
                data["error_code"] = Value::String(code);
                Err(RpcError::application("script-error", message).with_data(data))
            }
        }
    }

    fn open_project(&mut self, path: &str, create: bool) -> Result<Value, SessionError> {
        let Some(opener) = self.opener.clone() else {
            return Err(SessionError::new("open-unsupported", "this server was built without a distribution factory, so it cannot open projects"));
        };
        let dir = resolve_path(self.ws.root(), path)?;
        if let Some(root) = self.ws.root()
            && same_dir(root, &dir)
        {
            return Ok(to_json(&status_result(&self.ws)));
        }
        if self.ws.root().is_some() {
            self.ws.save()?;
        } else if !self.ws.timeline().is_empty() {
            return Err(SessionError::new(
                "workspace-not-empty",
                format!("opening a project would discard the in-memory session ({} nodes); `session.save` it elsewhere first", self.ws.timeline().len()),
            ));
        }
        let new = match opener().open(&dir) {
            Err(EngineError::NotAProject(_)) if create => {
                let name = dir.file_name().map_or_else(|| "project".to_owned(), |n| n.to_string_lossy().into_owned());
                opener().create(&dir, ProjectConfig::new(name))?
            }
            other => other?,
        };
        self.ws = new;
        Ok(to_json(&status_result(&self.ws)))
    }

    fn dispatch(&mut self, req: Request) -> Result<Value, SessionError> {
        match req {
            Request::Apply { op, params, version } => {
                let opts = ApplyOptions { version, ..ApplyOptions::default() };
                let r = self.ws.apply_json(&op, params, &opts)?;
                Ok(to_json(&ApplyResult::new(r.node, r.state, r.delta)))
            }
            Request::Query { name, input, at } => {
                // An omitted input is `null` on the wire; queries take an object.
                let input = if input.is_null() { Value::Object(Default::default()) } else { input };
                match at {
                    None => Ok(self.ws.query(&name, &input)?),
                    Some(rev) => Ok(self.ws.query_at(&rev, &name, &input)?),
                }
            }
            Request::Log { from, limit } => Ok(to_json(&self.ws.log(from, limit)?)),
            Request::Status => Ok(to_json(&status_result(&self.ws))),
            Request::Checkout { rev } => {
                self.ws.checkout(rev)?;
                Ok(to_json(&status_result(&self.ws)))
            }
            Request::Branch { name, at } => {
                self.ws.fork(&name, at)?;
                Ok(to_json(&status_result(&self.ws)))
            }
            Request::Undo => {
                self.ws.undo()?;
                Ok(to_json(&status_result(&self.ws)))
            }
            Request::Redo => {
                self.ws.redo()?;
                Ok(to_json(&status_result(&self.ws)))
            }
            Request::Hash { at } => {
                let (state, layers) = self.host().hash_at(at.as_ref())?;
                Ok(to_json(&HashResult::new(state, layers)))
            }
            Request::Export { path, format, mesh, provenance, inline } => {
                let r = self.host().export(&ExportParams { path, format, mesh, provenance, inline })?;
                Ok(to_json(&r))
            }
            Request::Save => {
                self.host().save()?;
                Ok(to_json(&status_result(&self.ws)))
            }
            Request::Open { path, create } => self.open_project(&path, create),
            other => Err(SessionError::new("method-not-implemented", format!("`{}` is not implemented", other.method()))),
        }
    }
}

impl Endpoint for Server {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        match req {
            Request::Catalog => self.catalog(),
            Request::ScriptRun { text, path, includes } => self.run_script(&text, path.as_deref(), includes),
            other => self.dispatch(other).map_err(RpcError::from),
        }
    }
}

fn internal(e: impl std::fmt::Display) -> RpcError {
    RpcError::internal(e.to_string())
}

fn to_value<T: serde::Serialize>(v: &T) -> Result<Value, RpcError> {
    serde_json::to_value(v).map_err(internal)
}

fn to_json<T: serde::Serialize>(v: &T) -> Value {
    serde_json::to_value(v).unwrap_or(Value::Null)
}

fn same_dir(a: &Path, b: &Path) -> bool {
    let canon = |p: &Path| -> PathBuf { p.canonicalize().unwrap_or_else(|_| p.to_owned()) };
    canon(a) == canon(b)
}

/// The JSON of one script event. Shape (owned by this crate): `event` is `op`, `batch`, `query`,
/// `timeline`, `print`, `help` or `other`; `at` is `file:line:col` of the statement.
fn event_json(e: &Event, map: &SourceMap) -> Value {
    let at = |span: sm_lang::Span| -> String {
        match map.get(span.file) {
            Some(src) => {
                let (l, c) = src.line_col(span.lo);
                format!("{}:{l}:{c}", src.name)
            }
            None => String::new(),
        }
    };
    match e {
        Event::Op { span, record, delta } => json!({
            "event": "op", "at": at(*span), "op": record.name, "version": record.version,
            "params": record.params, "delta": delta,
        }),
        Event::Batch { span, record, members, delta } => json!({
            "event": "batch", "at": at(*span), "op": record.name, "version": record.version,
            "params": record.params, "members": members, "delta": delta,
        }),
        Event::Query { span, name, params, result } => json!({
            "event": "query", "at": at(*span), "name": name, "params": params, "result": result,
        }),
        Event::Timeline { span, command, applied } => json!({
            "event": "timeline", "at": at(*span), "command": command.verb(), "applied": applied,
            "detail": timeline_detail(command),
        }),
        Event::Print { span, text } => json!({ "event": "print", "at": at(*span), "text": text }),
        Event::Help { span, text } => json!({ "event": "help", "at": at(*span), "text": text }),
        // Events added by newer sm-lang versions keep their information.
        other => json!({ "event": "other", "debug": format!("{other:?}") }),
    }
}

fn timeline_detail(c: &sm_lang::TimelineCommand) -> Value {
    use sm_lang::TimelineCommand as T;
    match c {
        T::Branch { name, at } => json!({ "name": name, "at": at }),
        T::Checkout(rev) => json!({ "rev": rev as &Rev }),
        _ => Value::Null,
    }
}
