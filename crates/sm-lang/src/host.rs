//! The execution backend of the interpreter: where ops are applied, queries run and includes are loaded.

use serde_json::Value as Json;
use sm_core::{EntityId, EntityKind};
use sm_op_api::{Delta, Engine, Hash256, OpRecord, QuietControl, Registry, Rev, State};

use crate::entity::Relations;

/// A failure reported by the host: the contract's stable error code plus its message.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("[{code}] {message}")]
pub struct HostError {
    /// Stable kebab-case code (`ApplyError::code`, `RegistryError::code`, ...).
    pub code: String,
    /// Human message.
    pub message: String,
}

/// A timeline-control command (`timeline.branch(...)`, `timeline.checkout(...)`, `timeline.undo()`, `timeline.redo()`).
///
/// These are statements of the language but **not** registry ops: a fork or an undo is not itself a timeline node,
/// so it cannot be recorded as one. A host handles them outside [`Host::apply`] (a session maps them to
/// `Session::branch`, `checkout`, `undo`, `redo`). See `LANGUAGE.md` section 4.6.
///
/// The enum is `#[non_exhaustive]`: new verbs (`tag`, `log`, `reset`, ...) will be added, so hosts need a wildcard arm.
/// The revision type is the contract's [`Rev`]; the interpreter parses a node hash before the host sees it.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum TimelineCommand {
    /// Create a branch and attach to it (fork), at the current position or at an earlier node.
    Branch {
        /// The new branch name.
        name: String,
        /// Fork at this node instead of the current position (`Workspace::fork(.., at)`).
        at: Option<Hash256>,
    },
    /// Move to a revision.
    Checkout(Rev),
    /// Step back one node.
    Undo,
    /// Step forward again after an undo.
    Redo,
}

impl TimelineCommand {
    /// The statement verb: `branch`, `checkout`, `undo` or `redo`.
    pub fn verb(&self) -> &'static str {
        match self {
            Self::Branch { .. } => "branch",
            Self::Checkout(_) => "checkout",
            Self::Undo => "undo",
            Self::Redo => "redo",
        }
    }
}

/// What the interpreter needs from its environment.
///
/// Production hosts adapt a timeline `Session` (one node per applied record). sm-lang itself never
/// builds a `Meta` (author/time are not its business), so [`EngineHost`] applies through
/// `Engine::apply_record`, the documented replay path, and [`PlanHost`] applies nothing.
pub trait Host {
    /// The op/query registry.
    fn registry(&self) -> &Registry;
    /// The current state (entity lists resolve against it).
    fn state(&self) -> &State;
    /// Adjacency for `kind in X` filters, if the host has any.
    fn relations(&self) -> Option<&dyn Relations> {
        None
    }
    /// False for a host that does not execute (plan mode): op outputs are unknown and existence of explicit ids is not checked.
    fn executes(&self) -> bool {
        true
    }
    /// Apply one record (an op or a `core.batch`) and return its delta.
    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError>;
    /// Run a registered query.
    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError>;
    /// Execute a timeline-control command. Hosts without a timeline keep the default, which reports `timeline-unsupported`.
    /// Never called when [`executes`](Self::executes) is false.
    fn timeline(&mut self, cmd: &TimelineCommand) -> Result<(), HostError> {
        Err(HostError { code: "timeline-unsupported".into(), message: format!("this host has no timeline: `timeline.{}` is unavailable", cmd.verb()) })
    }
    /// Text of an included file. The language performs no I/O itself.
    fn load(&mut self, path: &str) -> Result<String, String> {
        Err(format!("this host cannot load {path:?}: includes are not available"))
    }
}

type Loader<'a> = Box<dyn Fn(&str) -> Result<String, String> + 'a>;

/// A plain `Engine` + `State` host: applies records with `Engine::apply_record` and keeps a log.
pub struct EngineHost<'a> {
    engine: &'a Engine,
    state: State,
    relations: Option<Box<dyn Relations + 'a>>,
    loader: Option<Loader<'a>>,
    applied: Vec<OpRecord>,
}

impl<'a> EngineHost<'a> {
    /// Start from `state`.
    pub fn new(engine: &'a Engine, state: State) -> Self {
        Self { engine, state, relations: None, loader: None, applied: Vec::new() }
    }

    /// Provide adjacency for `kind in X`.
    #[must_use]
    pub fn with_relations(mut self, r: impl Relations + 'a) -> Self {
        self.relations = Some(Box::new(r));
        self
    }

    /// Provide include-file loading.
    #[must_use]
    pub fn with_loader(mut self, f: impl Fn(&str) -> Result<String, String> + 'a) -> Self {
        self.loader = Some(Box::new(f));
        self
    }

    /// The current state.
    pub fn current(&self) -> &State {
        &self.state
    }

    /// Take the final state.
    pub fn into_state(self) -> State {
        self.state
    }

    /// Every record applied so far, in order.
    pub fn applied(&self) -> &[OpRecord] {
        &self.applied
    }
}

impl Host for EngineHost<'_> {
    fn registry(&self) -> &Registry {
        self.engine.registry()
    }

    fn state(&self) -> &State {
        &self.state
    }

    fn relations(&self) -> Option<&dyn Relations> {
        self.relations.as_deref()
    }

    fn apply(&mut self, rec: &OpRecord) -> Result<Delta, HostError> {
        let q = QuietControl::new();
        match self.engine.apply_record(&mut self.state, rec, &q.control()) {
            Ok(d) => {
                self.applied.push(rec.clone());
                Ok(d)
            }
            Err(e) => Err(HostError { code: e.code().to_owned(), message: e.to_string() }),
        }
    }

    fn query(&mut self, name: &str, params: &Json) -> Result<Json, HostError> {
        let q = QuietControl::new();
        self.engine.run_query(&self.state, name, params, &q.control()).map_err(|e| HostError { code: e.code().to_owned(), message: e.to_string() })
    }

    fn load(&mut self, path: &str) -> Result<String, String> {
        match &self.loader {
            Some(f) => f(path),
            None => Err(format!("no loader configured for include {path:?}")),
        }
    }
}

/// A host that applies nothing: scripts are parsed, resolved and schema-checked, op records are collected in the report,
/// and results that depend on execution stay unknown (`LANGUAGE.md` §4.3).
pub struct PlanHost<'a> {
    registry: &'a Registry,
    state: State,
    relations: Option<&'a dyn Relations>,
}

impl<'a> PlanHost<'a> {
    /// Plan against `state` (entity lists resolve against it).
    pub fn new(registry: &'a Registry, state: State) -> Self {
        Self { registry, state, relations: None }
    }

    /// Provide adjacency for `kind in X`.
    #[must_use]
    pub fn with_relations(mut self, r: &'a dyn Relations) -> Self {
        self.relations = Some(r);
        self
    }
}

impl Host for PlanHost<'_> {
    fn registry(&self) -> &Registry {
        self.registry
    }

    fn state(&self) -> &State {
        &self.state
    }

    fn relations(&self) -> Option<&dyn Relations> {
        self.relations
    }

    fn executes(&self) -> bool {
        false
    }

    fn apply(&mut self, _: &OpRecord) -> Result<Delta, HostError> {
        Err(HostError { code: "plan-host".into(), message: "a plan host does not apply records".into() })
    }

    fn query(&mut self, _: &str, _: &Json) -> Result<Json, HostError> {
        Err(HostError { code: "plan-host".into(), message: "a plan host does not run queries".into() })
    }
}

/// Adapts a registered query to [`Relations`]: input `{ "kind": "<Kind>", "within": ["<id>", ...] }`, output `["<id>", ...]`.
/// The production topology plugin is expected to register such a query (see `W1_REPORT.md`).
pub struct QueryRelations<'a> {
    engine: &'a Engine,
    query: String,
}

impl<'a> QueryRelations<'a> {
    /// Use the registered query `query` as the adjacency source.
    pub fn new(engine: &'a Engine, query: impl Into<String>) -> Self {
        Self { engine, query: query.into() }
    }
}

impl Relations for QueryRelations<'_> {
    fn related(&self, state: &State, target: EntityKind, within: &[EntityId]) -> Result<Vec<EntityId>, String> {
        let input = serde_json::json!({
            "kind": target.name(),
            "within": within.iter().map(|i| Json::String(i.to_string())).collect::<Vec<_>>(),
        });
        let q = QuietControl::new();
        let out = self.engine.run_query(state, &self.query, &input, &q.control()).map_err(|e| e.to_string())?;
        let arr = out.as_array().ok_or_else(|| format!("query `{}` must return an array of ids", self.query))?;
        arr.iter()
            .map(|v| v.as_str().ok_or_else(|| "non-string id in query result".to_owned()).and_then(|s| s.parse::<EntityId>().map_err(|e| e.to_string())))
            .collect()
    }
}
