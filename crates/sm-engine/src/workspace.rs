//! [`Workspace`]: the engine facade. A [`Session`] bound to an optional project directory, with
//! plugins, autosave, branch management, a diagnostics policy and a JSON surface for servers.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Arc;

use serde::{Deserialize, Serialize};
use sm_op_api::{
    Control, Delta, DerivedCache, Engine, Hash256, Head, Meta, Op, OpRecord, Plugin, PrepareBase,
    Prepared, QuietControl, Registry, ReplayPolicy, Rev, Services, Session, State, Timeline,
    TimelineError, TypedOp, Value, valid_ref_name,
};

use crate::cache;
use crate::error::{EngineError, Result};
use crate::fs::{self, Layout, WriteCtx};
use crate::fsio::{self, Durability, FaultInjector, ProjectLock};
use crate::policy::{Clock, DiagnosticPolicy, SystemClock};
use crate::project::ProjectConfig;

/// Per-call options of the `apply` family. Deserializable, so a server can take them straight
/// from a request.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ApplyOptions {
    /// Node message (default: the op name).
    pub message: Option<String>,
    /// Node author (default: the workspace author).
    pub author: Option<String>,
    /// Node time, RFC 3339 (default: the workspace clock).
    pub time: Option<String>,
    /// Payload version the params were written with (default: the registry's current version).
    pub version: Option<u32>,
    /// Record the op even if the policy would stop it on a warning.
    pub accept_warnings: bool,
}

/// What an applied op did.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ApplyReport {
    /// The new node.
    pub node: Hash256,
    /// Quantized state hash after the op.
    pub state: Hash256,
    /// What the op changed. `delta.diagnostics` is filtered by the policy.
    pub delta: Delta,
    /// Diagnostics hidden by verbosity or suppression.
    pub hidden_diagnostics: usize,
}

/// One entry of [`Workspace::log`].
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct LogEntry {
    /// Node hash.
    pub node: Hash256,
    /// Parent node.
    pub parent: Option<Hash256>,
    /// Distance from the root (roots are 1).
    pub depth: usize,
    /// Op name.
    pub op: String,
    /// Author.
    pub author: String,
    /// Time.
    pub time: String,
    /// Message.
    pub message: String,
}

/// Snapshot of where the workspace stands.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Status {
    /// HEAD.
    pub head: Head,
    /// Current node (`None` = empty state).
    pub position: Option<Hash256>,
    /// Depth of the current node (0 when empty).
    pub depth: usize,
    /// Quantized hash of the current state.
    pub state: Hash256,
    /// Nodes in the timeline.
    pub nodes: usize,
    /// Branches and their tips.
    pub branches: BTreeMap<String, Hash256>,
    /// Nodes `redo` can step through.
    pub redo: usize,
    /// True when something is not on disk yet.
    pub dirty: bool,
    /// Project name, if a project is attached.
    pub project: Option<String>,
}

/// What `open` did, for logs and front ends.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct OpenReport {
    /// Notes about ignored files, damaged cache entries and similar; none is fatal.
    pub warnings: Vec<String>,
    /// The node whose persisted snapshot seeded the replay, if one was usable.
    pub seeded_from: Option<Hash256>,
    /// Nodes re-baselined while opening (under [`ReplayPolicy::Rebaseline`]).
    pub rebaselined: Vec<Hash256>,
}

struct Settings {
    policy: DiagnosticPolicy,
    clock: Arc<dyn Clock>,
    author: String,
    durability: Durability,
    autosave: bool,
    snapshot_every: Option<usize>,
    snapshot_keep: usize,
    reflog_limit: usize,
    read_only: bool,
    fault: Option<Arc<FaultInjector>>,
}

impl Settings {
    fn ctx(&self) -> WriteCtx<'_> {
        WriteCtx {
            durability: self.durability,
            fault: self.fault.as_deref(),
        }
    }
}

struct Project {
    layout: Layout,
    config: ProjectConfig,
    _lock: Option<ProjectLock>,
    /// Nodes to (re)write: recorded since the last save, or re-baselined.
    dirty: BTreeSet<Hash256>,
    refs_on_disk: BTreeMap<String, Hash256>,
    head_on_disk: Option<Head>,
    redo_on_disk: Vec<Hash256>,
    reflog_persisted: usize,
}

/// Builds a [`Workspace`]: register plugins and services, choose policies, then
/// [`memory`](Self::memory), [`create`](Self::create) or [`open`](Self::open).
pub struct WorkspaceBuilder {
    plugins: Vec<Box<dyn Plugin>>,
    services: Services,
    cache: Option<Arc<DerivedCache>>,
    replay: ReplayPolicy,
    s: Settings,
}

impl Default for WorkspaceBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl WorkspaceBuilder {
    /// Defaults: autosave on, full durability, strict replay, warnings reported, system clock,
    /// author `"smart-mesh"`, no automatic snapshots, 10 000 reflog entries kept.
    pub fn new() -> Self {
        Self {
            plugins: Vec::new(),
            services: Services::new(),
            cache: None,
            replay: ReplayPolicy::Strict,
            s: Settings {
                policy: DiagnosticPolicy::default(),
                clock: Arc::new(SystemClock),
                author: "smart-mesh".to_owned(),
                durability: Durability::Full,
                autosave: true,
                snapshot_every: None,
                snapshot_keep: 4,
                reflog_limit: 10_000,
                read_only: false,
                fault: None,
            },
        }
    }
    /// Register a plugin (layers, ops, queries). Installed in the order given; the result does
    /// not depend on that order.
    #[must_use]
    pub fn plugin(mut self, p: impl Plugin + 'static) -> Self {
        self.plugins.push(Box::new(p));
        self
    }
    /// Services injected into ops and queries (geometry kernel, mesher).
    #[must_use]
    pub fn services(mut self, s: Services) -> Self {
        self.services = s;
        self
    }
    /// Use this derived cache (default: a fresh 256 MiB cache). Pass `DerivedCache::disabled()`
    /// to prove that cache hits never change results.
    #[must_use]
    pub fn derived_cache(mut self, c: Arc<DerivedCache>) -> Self {
        self.cache = Some(c);
        self
    }
    /// Diagnostics policy.
    #[must_use]
    pub fn policy(mut self, p: DiagnosticPolicy) -> Self {
        self.s.policy = p;
        self
    }
    /// Clock for node times.
    #[must_use]
    pub fn clock(mut self, c: impl Clock + 'static) -> Self {
        self.s.clock = Arc::new(c);
        self
    }
    /// Default node author.
    #[must_use]
    pub fn author(mut self, a: impl Into<String>) -> Self {
        self.s.author = a.into();
        self
    }
    /// `fsync` behavior.
    #[must_use]
    pub fn durability(mut self, d: Durability) -> Self {
        self.s.durability = d;
        self
    }
    /// Save after every mutating call (default `true`). When off, call
    /// [`Workspace::save`].
    #[must_use]
    pub fn autosave(mut self, on: bool) -> Self {
        self.s.autosave = on;
        self
    }
    /// Write a state snapshot to `.cache/snapshots` whenever the depth of a newly recorded node
    /// is a multiple of `n`. Layers without the snapshot hook are skipped with a warning.
    #[must_use]
    pub fn snapshot_every(mut self, n: Option<usize>) -> Self {
        self.s.snapshot_every = n.map(|n| n.max(1));
        self
    }
    /// How many snapshots to keep on disk (the deepest ones; default 4).
    #[must_use]
    pub fn keep_snapshots(mut self, n: usize) -> Self {
        self.s.snapshot_keep = n;
        self
    }
    /// What replay does when a recorded result hash differs (default strict).
    #[must_use]
    pub fn replay_policy(mut self, p: ReplayPolicy) -> Self {
        self.replay = p;
        self
    }
    /// Newest reflog entries kept across restarts.
    #[must_use]
    pub fn reflog_limit(mut self, n: usize) -> Self {
        self.s.reflog_limit = n;
        self
    }
    /// Open without taking the lock and without writing anything. Mutating calls fail with
    /// [`EngineError::ReadOnly`]; checkout and queries work.
    #[must_use]
    pub fn read_only(mut self, on: bool) -> Self {
        self.s.read_only = on;
        self
    }
    /// Install a crash simulator (tests).
    #[must_use]
    pub fn fault(mut self, f: Arc<FaultInjector>) -> Self {
        self.s.fault = Some(f);
        self
    }

    fn engine(&mut self) -> Result<Engine> {
        let mut reg = Registry::new();
        for p in &self.plugins {
            reg.install(p.as_ref())?;
        }
        let cache = self.cache.clone().unwrap_or_default();
        Ok(Engine::new(
            Arc::new(reg),
            std::mem::take(&mut self.services),
            cache,
        )?)
    }

    fn finish(self, session: Session, project: Option<Project>, report: OpenReport) -> Workspace {
        Workspace {
            session,
            s: self.s,
            project,
            warnings: report.warnings.clone(),
            report,
        }
    }

    /// A workspace with no project directory (nothing is saved until
    /// [`Workspace::save_as`]).
    pub fn memory(mut self) -> Result<Workspace> {
        let session = Session::new(self.engine()?);
        Ok(self.finish(session, None, OpenReport::default()))
    }

    /// Create a new project at `path` (an absent or empty directory) and bind a fresh workspace
    /// to it.
    pub fn create(self, path: impl AsRef<Path>, config: ProjectConfig) -> Result<Workspace> {
        let mut ws = self.memory()?;
        ws.save_as(path, config)?;
        Ok(ws)
    }

    /// Open the project at `path` and replay to its HEAD, with no progress and no cancellation.
    pub fn open(self, path: impl AsRef<Path>) -> Result<Workspace> {
        let q = QuietControl::new();
        self.open_with(path, &q.control())
    }

    /// Open the project at `path`: read `project.toml`, take the write lock (unless read-only),
    /// sweep temp files left by a killed writer, load and verify the timeline, restore HEAD,
    /// reflog and redo stack from `.cache/`, seed the replay with the deepest usable persisted
    /// snapshot on the path to HEAD, and replay.
    pub fn open_with(mut self, path: impl AsRef<Path>, ctl: &Control<'_>) -> Result<Workspace> {
        let layout = Layout::new(path.as_ref());
        let text = match std::fs::read_to_string(layout.project_toml()) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                return Err(EngineError::NotAProject(layout.root().to_owned()));
            }
            Err(e) => return Err(EngineError::io(layout.project_toml(), e)),
        };
        let config = ProjectConfig::from_toml(&text)?;
        let ro = self.s.read_only;
        let lock = if ro {
            None
        } else {
            Some(ProjectLock::acquire(layout.root(), &layout.cache_dir())?)
        };
        let mut report = OpenReport::default();
        if !ro {
            for d in [layout.timeline_dir(), layout.cache_dir()] {
                fsio::sweep_tmp(&d).map_err(|e| EngineError::io(&d, e))?;
            }
        }
        let (timeline, loaded) = fs::load_timeline(&layout)?;
        report.warnings.extend(
            loaded
                .ignored
                .iter()
                .map(|r| format!("ignored file timeline/{r}")),
        );

        let head_on_disk = cache::read_head(&layout);
        let redo = cache::read_redo(&layout);
        // An unborn branch is a legitimate HEAD in an empty project and after undoing a branch
        // to its root (the top of the redo stack is then a root node). Otherwise a HEAD naming a
        // branch that no longer exists (deleted by a pull, say) must not open as an empty state.
        let undone_to_root = redo
            .last()
            .and_then(|h| timeline.get(*h))
            .is_some_and(|n| n.body.parent.is_none());
        let head = match &head_on_disk {
            Some(Head::Ref(n))
                if timeline.check_ref_name(n).is_ok()
                    && (timeline.refs().contains_key(n)
                        || timeline.refs().is_empty()
                        || undone_to_root) =>
            {
                Head::Ref(n.clone())
            }
            Some(Head::Detached(Some(h))) if timeline.get(*h).is_some() => Head::Detached(Some(*h)),
            Some(Head::Detached(None)) => Head::Detached(None),
            Some(_) => {
                report
                    .warnings
                    .push("HEAD names an unknown branch or node; using the default".into());
                timeline.default_head()
            }
            None => timeline.default_head(),
        };

        let engine = self.engine()?;
        let ctx_owned = Settings::ctx(&self.s);
        let (reflog, rewrite) = cache::read_reflog(&layout, self.s.reflog_limit);
        if rewrite && !ro {
            cache::rewrite_reflog(&layout, &reflog, ctx_owned)?;
        }
        let reflog_persisted = reflog.len();

        let target = match &head {
            Head::Ref(n) => timeline.refs().get(n).copied(),
            Head::Detached(h) => *h,
        };
        let mut seeds = Vec::new();
        if let Some(t) = target {
            let mut path_to_root = timeline.path_to_root(t)?;
            path_to_root.reverse(); // nearest first
            // One directory listing instead of a failed open per ancestor.
            let have: BTreeSet<Hash256> = cache::snapshot_nodes(&layout).into_iter().collect();
            for h in path_to_root.into_iter().filter(|h| have.contains(h)) {
                match cache::read_snapshot(&layout, h) {
                    Ok(None) => continue,
                    Ok(Some(f)) if !versions_current(engine.registry(), &f.versions) => {
                        // Not damaged, just written by other code: ignore, keep the file.
                        report.warnings.push(format!(
                            "snapshot of {h} was written with other op or layer versions; ignored"
                        ));
                        continue;
                    }
                    Ok(Some(f)) => match engine.registry().restore(&f.snap) {
                        // A seed must agree with itself and with the node's recorded check.
                        Ok(state)
                            if state.hash() == f.state
                                && timeline.get(h).is_some_and(|n| n.check.state == f.state) =>
                        {
                            seeds.push((h, state));
                            report.seeded_from = Some(h);
                            break;
                        }
                        Ok(_) => report.warnings.push(format!(
                            "snapshot of {h} does not match the node's recorded state; ignored"
                        )),
                        Err(_) => report
                            .warnings
                            .push(format!("snapshot of {h} does not restore; ignored")),
                    },
                    Err(e) => report
                        .warnings
                        .push(format!("snapshot of {h} unreadable ({e}); ignored")),
                }
                if !ro {
                    let _ = cache::remove_snapshot(&layout, h, ctx_owned);
                }
            }
        }
        let refs_on_disk = timeline.refs().clone();
        let mut session = if self.replay == ReplayPolicy::Strict || target.is_none() {
            Session::open_with(engine, timeline, head.clone(), seeds, reflog, ctl)?
        } else {
            // `Session::open_with` replays under the strict policy, so open at the empty state,
            // choose the policy, then check out HEAD (one extra reflog entry).
            let mut s =
                Session::open_with(engine, timeline, Head::Detached(None), seeds, reflog, ctl)?;
            s.set_replay_policy(self.replay);
            let rev = match &head {
                Head::Ref(n) => Rev::Ref(n.clone()),
                Head::Detached(Some(h)) => Rev::Node(*h),
                Head::Detached(None) => Rev::Empty,
            };
            s.checkout(rev, ctl)?;
            s
        };
        let rebaselined = session.take_rebaselined();
        for w in session.take_warnings() {
            report.warnings.push(format!("replay: {w}"));
        }
        report.rebaselined = rebaselined.clone();
        let redo_on_disk = redo.clone();
        if !redo.is_empty() && session.set_redo_stack(redo).is_err() {
            report
                .warnings
                .push("redo stack does not match HEAD; dropped".into());
        }
        let project = Project {
            layout,
            config,
            _lock: lock,
            dirty: rebaselined.into_iter().collect(),
            refs_on_disk,
            head_on_disk,
            redo_on_disk,
            reflog_persisted,
        };
        let mut ws = self.finish(session, Some(project), report);
        if ws.s.autosave && !ro {
            ws.persist().map_err(|e| EngineError::NotPersisted {
                node: None,
                source: Box::new(e),
            })?;
        }
        Ok(ws)
    }
}

/// An engine session over a timeline, optionally backed by a project directory.
///
/// All mutating calls take `&mut self`; wrap the workspace in a mutex to share it. Methods that
/// replay (`checkout`, `undo`, `query_at`, …) have `_with` variants taking a [`Control`].
pub struct Workspace {
    session: Session,
    s: Settings,
    project: Option<Project>,
    warnings: Vec<String>,
    report: OpenReport,
}

impl Workspace {
    /// Start building a workspace.
    pub fn builder() -> WorkspaceBuilder {
        WorkspaceBuilder::new()
    }

    /// The underlying session (read-only; mutate through the workspace so that saving stays
    /// in step).
    pub fn session(&self) -> &Session {
        &self.session
    }
    /// The current state.
    pub fn state(&self) -> &State {
        self.session.state()
    }
    /// The timeline.
    pub fn timeline(&self) -> &Timeline {
        self.session.timeline()
    }
    /// The registry (op, query and layer catalogs).
    pub fn registry(&self) -> &Arc<Registry> {
        self.session.engine().registry()
    }
    /// The project config, if a project is attached.
    pub fn config(&self) -> Option<&ProjectConfig> {
        self.project.as_ref().map(|p| &p.config)
    }
    /// The project directory, if attached.
    pub fn root(&self) -> Option<&Path> {
        self.project.as_ref().map(|p| p.layout.root())
    }
    /// What `open` found.
    pub fn open_report(&self) -> &OpenReport {
        &self.report
    }
    /// Notes accumulated since the last call (ignored files, skipped snapshots, tolerated
    /// replay divergences).
    pub fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }
    /// The diagnostics policy.
    pub fn policy(&self) -> &DiagnosticPolicy {
        &self.s.policy
    }
    /// Replace the diagnostics policy.
    pub fn set_policy(&mut self, p: DiagnosticPolicy) {
        self.s.policy = p;
    }
    /// Turn autosave on or off.
    pub fn set_autosave(&mut self, on: bool) {
        self.s.autosave = on;
    }
    /// Quantized hash of the current state.
    pub fn state_hash(&self) -> Hash256 {
        self.session.state().hash()
    }

    fn writable(&self) -> Result<()> {
        if self.s.read_only {
            Err(EngineError::ReadOnly)
        } else {
            Ok(())
        }
    }

    fn note_replay(&mut self) {
        let re = self.session.take_rebaselined();
        if let Some(p) = self.project.as_mut() {
            p.dirty.extend(re);
        }
        for w in self.session.take_warnings() {
            self.warnings.push(format!("replay: {w}"));
        }
    }

    // ------------------------------------------------------------------ persistence

    /// True when something recorded or moved is not on disk yet (always false without a project).
    pub fn dirty(&self) -> bool {
        let Some(p) = self.project.as_ref().filter(|_| !self.s.read_only) else {
            return false;
        };
        !p.dirty.is_empty()
            || p.refs_on_disk != *self.session.timeline().refs()
            || p.head_on_disk.as_ref() != Some(self.session.head())
            || p.redo_on_disk != self.session.redo_stack()
            || p.reflog_persisted < self.session.reflog().len()
    }

    /// Write whatever is not on disk yet, in crash-safe order: node files, then ref files
    /// (deleted refs last), then HEAD, redo stack and reflog in `.cache/`. Each file is replaced
    /// atomically, so a crash at any point leaves a project that opens (see
    /// [`FaultInjector`]). On failure what already succeeded stays recorded as saved, so a retry
    /// continues where it stopped.
    pub fn save(&mut self) -> Result<()> {
        self.writable()?;
        if self.project.is_none() {
            return Err(EngineError::NoProject);
        }
        self.persist()
    }

    fn persist(&mut self) -> Result<()> {
        if self.s.read_only {
            return Ok(());
        }
        self.note_replay();
        let ctx = self.s.ctx();
        let Some(p) = self.project.as_mut() else {
            return Ok(());
        };
        let tl = self.session.timeline();
        let list: Vec<Hash256> = p.dirty.iter().copied().collect();
        let mut done = Vec::new();
        let r = fs::write_nodes(&p.layout, tl, &list, ctx, |h| done.push(h));
        for h in done {
            p.dirty.remove(&h);
        }
        r?;
        let head = self.session.head();
        let (layout, head_on_disk) = (&p.layout, &mut p.head_on_disk);
        fs::sync_refs(layout, tl, &mut p.refs_on_disk, ctx, &mut || {
            if head_on_disk.as_ref() != Some(head) {
                cache::write_head(layout, head, ctx)?;
                *head_on_disk = Some(head.clone());
            }
            Ok(())
        })?;
        if p.redo_on_disk != self.session.redo_stack() {
            cache::write_redo(&p.layout, self.session.redo_stack(), ctx)?;
            p.redo_on_disk = self.session.redo_stack().to_vec();
        }
        let log = self.session.reflog();
        if p.reflog_persisted < log.len() {
            cache::append_reflog(&p.layout, &log[p.reflog_persisted..])?;
            p.reflog_persisted = log.len();
        }
        Ok(())
    }

    fn autosave(&mut self, node: Option<Hash256>) -> Result<()> {
        if self.s.autosave && self.project.is_some() {
            self.persist().map_err(|e| EngineError::NotPersisted {
                node,
                source: Box::new(e),
            })?;
        }
        Ok(())
    }

    /// Bind an in-memory workspace to a new project directory and write everything. The
    /// directory must be absent or empty. Also writes `project.toml` and a `.gitignore` that
    /// ignores `.cache/`.
    ///
    /// On failure, everything written is removed again (the directory is deleted if it did not
    /// exist before, emptied otherwise), so the same path can be retried. The cleanup is safe
    /// because the directory was verified empty and is locked by this call.
    pub fn save_as(&mut self, path: impl AsRef<Path>, config: ProjectConfig) -> Result<()> {
        self.writable()?;
        if self.project.is_some() {
            return Err(EngineError::AlreadyExists(path.as_ref().to_owned()));
        }
        let layout = Layout::new(path.as_ref());
        let root = layout.root();
        let mut existed = true;
        let non_empty = match std::fs::read_dir(root) {
            Ok(mut rd) => rd.next().is_some(),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                existed = false;
                false
            }
            Err(e) => return Err(EngineError::io(root, e)),
        };
        if non_empty {
            return Err(EngineError::AlreadyExists(root.to_owned()));
        }
        let toml = config.to_toml()?;
        let lock = ProjectLock::acquire(root, &layout.cache_dir())?;
        // `project.toml` last: a half-created project is not a project.
        let dirty: BTreeSet<Hash256> = self.session.timeline().nodes().map(|(h, _)| h).collect();
        self.project = Some(Project {
            layout: layout.clone(),
            config,
            _lock: Some(lock),
            dirty,
            refs_on_disk: BTreeMap::new(),
            head_on_disk: None,
            redo_on_disk: Vec::new(),
            reflog_persisted: 0,
        });
        let r = self
            .s
            .ctx()
            .write(&layout.gitignore(), b"/.cache/\n")
            .and_then(|()| self.persist())
            .and_then(|()| self.s.ctx().write(&layout.project_toml(), toml.as_bytes()));
        if r.is_err() {
            // Unbind (and unlock), then undo our writes so a retry finds an empty directory.
            self.project = None;
            if let Ok(rd) = std::fs::read_dir(root) {
                for e in rd.flatten() {
                    let p = e.path();
                    let _ = if p.is_dir() {
                        std::fs::remove_dir_all(&p)
                    } else {
                        std::fs::remove_file(&p)
                    };
                }
            }
            if !existed {
                let _ = std::fs::remove_dir(root);
            }
        }
        r
    }

    /// Replace the project config (name, units, tolerances, metadata) and save it.
    pub fn set_config(&mut self, config: ProjectConfig) -> Result<()> {
        self.writable()?;
        let ctx = self.s.ctx();
        let p = self.project.as_mut().ok_or(EngineError::NoProject)?;
        ctx.write(&p.layout.project_toml(), config.to_toml()?.as_bytes())?;
        p.config = config;
        Ok(())
    }

    /// Write a persisted snapshot of the current state to `.cache/snapshots` so the next `open`
    /// can skip replaying up to here. Returns `false` (and notes a warning) when a layer has no
    /// snapshot support or the position is the empty state; prunes old snapshots.
    pub fn snapshot_now(&mut self) -> Result<bool> {
        self.writable()?;
        let ctx = self.s.ctx();
        let keep = self.s.snapshot_keep;
        let Some(p) = self.project.as_ref() else {
            return Err(EngineError::NoProject);
        };
        let Some(node) = self.session.position() else {
            return Ok(false);
        };
        let state = self.session.state();
        let recorded = self.session.timeline().get(node).map(|n| n.check.state);
        if recorded != Some(state.hash()) {
            self.warnings.push(format!(
                "no snapshot at {node}: state does not match the node's recorded state"
            ));
            return Ok(false);
        }
        let snap = match state.snapshot() {
            Ok(s) => s,
            Err(e) => {
                self.warnings.push(format!("no snapshot at {node}: {e}"));
                return Ok(false);
            }
        };
        let versions = snapshot_versions(
            self.session.engine().registry(),
            self.session.timeline(),
            node,
            &snap,
        )?;
        cache::write_snapshot(&p.layout, node, state.hash(), &versions, &snap, ctx)?;
        cache::prune_snapshots(&p.layout, self.session.timeline(), keep, Some(node), ctx)?;
        Ok(true)
    }

    /// Final save. The project lock is released when the workspace is dropped.
    pub fn close(mut self) -> Result<()> {
        if self.project.is_some() && !self.s.read_only {
            self.persist()?;
        }
        Ok(())
    }

    // ------------------------------------------------------------------ ops

    fn meta(&self, op: &str, o: &ApplyOptions) -> Meta {
        Meta::new(
            o.author.clone().unwrap_or_else(|| self.s.author.clone()),
            o.time.clone().unwrap_or_else(|| self.s.clock.now()),
            o.message.clone().unwrap_or_else(|| op.to_owned()),
        )
    }

    fn commit(
        &mut self,
        prepared: sm_op_api::Prepared,
        op: &str,
        o: &ApplyOptions,
    ) -> Result<ApplyReport> {
        let blockers = self.s.policy.blockers(&prepared.delta().diagnostics);
        if !blockers.is_empty() && !o.accept_warnings {
            return Err(EngineError::StoppedOnWarning {
                diagnostics: blockers,
            });
        }
        let meta = self.meta(op, o);
        let (node, mut delta) = self.session.commit(prepared, meta)?;
        let (shown, hidden) = self.s.policy.report(&delta.diagnostics);
        delta.diagnostics = shown;
        if let Some(p) = self.project.as_mut() {
            p.dirty.insert(node);
        }
        self.autosave(Some(node))?;
        if let Some(n) = self.s.snapshot_every
            && self.project.is_some()
            && self
                .session
                .timeline()
                .depth(node)
                .is_some_and(|d| d % n == 0)
        {
            // The op is recorded and saved; a snapshot is only an accelerator.
            if let Err(e) = self.snapshot_now() {
                self.warnings
                    .push(format!("snapshot at {node} failed: {e}"));
            }
        }
        Ok(ApplyReport {
            node,
            state: self.session.state().hash(),
            delta,
            hidden_diagnostics: hidden,
        })
    }

    /// Apply a typed op and record it.
    pub fn apply<T: TypedOp>(&mut self, op: &T, o: &ApplyOptions) -> Result<ApplyReport> {
        self.apply_dyn(op, o, &QuietControl::new().control())
    }

    /// Apply a decoded op and record it.
    pub fn apply_dyn(
        &mut self,
        op: &dyn Op,
        o: &ApplyOptions,
        ctl: &Control<'_>,
    ) -> Result<ApplyReport> {
        self.writable()?;
        let prepared = self.session.prepare(op, ctl)?;
        self.commit(prepared, op.name(), o)
    }

    /// Apply an op given by name and JSON params (the server/MCP/Python entry point). Params are
    /// canonicalized (nulls dropped, `-0.0` folded), migrated to the current version and
    /// re-encoded before hashing, so the same op is the same node whichever client sent it.
    pub fn apply_json(&mut self, op: &str, params: Value, o: &ApplyOptions) -> Result<ApplyReport> {
        self.apply_json_with(op, params, o, &QuietControl::new().control())
    }

    /// [`apply_json`](Self::apply_json) with progress and cancellation.
    pub fn apply_json_with(
        &mut self,
        op: &str,
        params: Value,
        o: &ApplyOptions,
        ctl: &Control<'_>,
    ) -> Result<ApplyReport> {
        self.writable()?;
        let version = match o.version {
            Some(v) => v,
            None => self.registry().op_version(op)?,
        };
        let rec = OpRecord::new(op, version, params).map_err(TimelineError::from)?;
        let prepared = self.session.prepare_base().prepare_record(rec, ctl)?;
        self.commit(prepared, op, o)
    }

    /// An owned copy of the current state and position, to run an op on a worker thread while
    /// the workspace stays usable: `base.prepare(&op, &ctl)` there, then
    /// [`commit_prepared`](Self::commit_prepared) here.
    pub fn prepare_base(&self) -> PrepareBase {
        self.session.prepare_base()
    }

    /// Record an op prepared with [`prepare_base`](Self::prepare_base), subject to the same
    /// policy, autosave and snapshot rules as [`apply`](Self::apply). Fails with
    /// `stale-base` if the position moved since the base was taken; nothing is recorded then.
    pub fn commit_prepared(&mut self, p: Prepared, o: &ApplyOptions) -> Result<ApplyReport> {
        self.writable()?;
        let name = p.record().name.clone();
        self.commit(p, &name, o)
    }

    /// Run a registered query against the current state. Nothing is recorded.
    pub fn query(&self, name: &str, input: &Value) -> Result<Value> {
        self.query_with(name, input, &QuietControl::new().control())
    }

    /// [`query`](Self::query) with progress and cancellation.
    pub fn query_with(&self, name: &str, input: &Value, ctl: &Control<'_>) -> Result<Value> {
        Ok(self
            .session
            .engine()
            .run_query(self.session.state(), name, input, ctl)?)
    }

    /// Run a query against the state at `rev` without moving HEAD.
    pub fn query_at(&mut self, rev: &Rev, name: &str, input: &Value) -> Result<Value> {
        let state = self.state_at(rev)?;
        let q = QuietControl::new();
        Ok(self
            .session
            .engine()
            .run_query(&state, name, input, &q.control())?)
    }

    /// The state at `rev`, reconstructed by replay (cached states and snapshots speed it up).
    pub fn state_at(&mut self, rev: &Rev) -> Result<State> {
        let node = self.resolve(rev)?;
        let q = QuietControl::new();
        let s = self.session.state_at(node, &q.control());
        self.note_replay();
        Ok(s?)
    }

    fn resolve(&self, rev: &Rev) -> Result<Option<Hash256>> {
        match rev {
            Rev::Empty => Ok(None),
            Rev::Node(h) => self
                .timeline()
                .get(*h)
                .map(|_| Some(*h))
                .ok_or_else(|| TimelineError::UnknownNode(*h).into()),
            Rev::Ref(n) => self
                .timeline()
                .refs()
                .get(n)
                .map(|h| Some(*h))
                .ok_or_else(|| TimelineError::UnknownRef(n.clone()).into()),
            _ => Err(EngineError::BadRequest("unsupported revision".into())),
        }
    }

    // ------------------------------------------------------------------ navigation

    /// Check out a branch, a node (detached) or the empty state.
    pub fn checkout(&mut self, rev: Rev) -> Result<()> {
        self.checkout_with(rev, &QuietControl::new().control())
    }

    /// [`checkout`](Self::checkout) with progress and cancellation. On error the workspace is
    /// unchanged.
    pub fn checkout_with(&mut self, rev: Rev, ctl: &Control<'_>) -> Result<()> {
        let r = self.session.checkout(rev, ctl);
        self.note_replay();
        r?;
        self.autosave(None)
    }

    /// A branch HEAD is attached to but that has no tip (undone to its root) must not be created
    /// by name: the session would keep its empty position while HEAD names a born branch.
    fn guard_unborn_head(&self, name: &str) -> Result<()> {
        match self.session.head() {
            Head::Ref(n) if n == name && !self.timeline().refs().contains_key(n) => {
                Err(EngineError::BadRequest(format!(
                    "'{name}' is the branch HEAD is on and it has no tip; redo or check out another revision first"
                )))
            }
            _ => Ok(()),
        }
    }

    /// Create branch `name` at `at` (default: the current node) without moving HEAD.
    pub fn create_branch(&mut self, name: &str, at: Option<Hash256>) -> Result<()> {
        self.writable()?;
        self.guard_unborn_head(name)?;
        let at = at
            .or(self.session.position())
            .ok_or(TimelineError::EmptyPosition)?;
        self.session.create_branch_at(name, at)?;
        self.autosave(None)
    }

    /// Fork: create branch `name` at `at` (default: the current node) and attach HEAD to it, so
    /// the next op extends the new branch and leaves the old one untouched.
    pub fn fork(&mut self, name: &str, at: Option<Hash256>) -> Result<()> {
        self.fork_with(name, at, &QuietControl::new().control())
    }

    /// [`fork`](Self::fork) with progress and cancellation. If replay to `at` fails, the new
    /// branch is removed again.
    pub fn fork_with(&mut self, name: &str, at: Option<Hash256>, ctl: &Control<'_>) -> Result<()> {
        self.writable()?;
        self.guard_unborn_head(name)?;
        let at = at
            .or(self.session.position())
            .ok_or(TimelineError::EmptyPosition)?;
        self.session.create_branch_at(name, at)?;
        let r = self.session.checkout(Rev::Ref(name.to_owned()), ctl);
        self.note_replay();
        if let Err(e) = r {
            let _ = self.session.delete_branch(name);
            return Err(e.into());
        }
        self.autosave(None)
    }

    /// Delete a branch (not the one HEAD is on). Its nodes stay in the timeline and on disk.
    pub fn delete_branch(&mut self, name: &str) -> Result<()> {
        self.writable()?;
        self.session.delete_branch(name)?;
        self.autosave(None)
    }

    /// Rename a branch; HEAD follows.
    pub fn rename_branch(&mut self, from: &str, to: &str) -> Result<()> {
        self.writable()?;
        self.guard_unborn_head(to)?;
        self.session.rename_branch(from, to)?;
        self.autosave(None)
    }

    /// Move a branch to `to` (git reset).
    pub fn reset_branch(&mut self, name: &str, to: Hash256) -> Result<()> {
        self.writable()?;
        let q = QuietControl::new();
        let r = self.session.reset_branch(name, to, &q.control());
        self.note_replay();
        r?;
        self.autosave(None)
    }

    /// Undo the last op on the attached branch (moves the branch to the parent; the node stays).
    pub fn undo(&mut self) -> Result<()> {
        self.writable()?;
        let r = self.session.undo();
        self.note_replay();
        r?;
        self.autosave(None)
    }

    /// Redo the most recently undone node. The redo stack survives a restart.
    pub fn redo(&mut self) -> Result<()> {
        self.writable()?;
        let r = self.session.redo();
        self.note_replay();
        r?;
        self.autosave(None)
    }

    // ------------------------------------------------------------------ introspection

    /// Where the workspace stands.
    pub fn status(&self) -> Status {
        let tl = self.session.timeline();
        Status {
            head: self.session.head().clone(),
            position: self.session.position(),
            depth: self
                .session
                .position()
                .and_then(|h| tl.depth(h))
                .unwrap_or(0),
            state: self.session.state().hash(),
            nodes: tl.len(),
            branches: tl.refs().clone(),
            redo: self.session.redo_stack().len(),
            dirty: self.dirty(),
            project: self.project.as_ref().map(|p| p.config.name.clone()),
        }
    }

    /// The history from `from` (default: the current node) back to the root, newest first, at
    /// most `limit` entries.
    pub fn log(&self, from: Option<Hash256>, limit: usize) -> Result<Vec<LogEntry>> {
        let tl = self.session.timeline();
        let mut cur = from.or(self.session.position());
        let mut out = Vec::new();
        while let Some(h) = cur {
            if out.len() >= limit {
                break;
            }
            let n = tl.get(h).ok_or(TimelineError::UnknownNode(h))?;
            out.push(LogEntry {
                node: h,
                parent: n.body.parent,
                depth: tl.depth(h).unwrap_or(0),
                op: n.body.op.name.clone(),
                author: n.body.meta.author.clone(),
                time: n.body.meta.time.clone(),
                message: n.body.meta.message.clone(),
            });
            cur = n.body.parent;
        }
        Ok(out)
    }

    /// Op and query catalogs as JSON (`{"ops": [...], "queries": [...]}`); an entry whose schema
    /// failed is `{"error": "<code>"}`.
    pub fn catalog_json(&self) -> Value {
        fn entry<T: Serialize>(r: std::result::Result<T, sm_op_api::RegistryError>) -> Value {
            match r {
                Ok(v) => serde_json::to_value(v)
                    .unwrap_or_else(|_| serde_json::json!({ "error": "serialize" })),
                Err(e) => serde_json::json!({ "error": e.code() }),
            }
        }
        let reg = self.registry();
        let ops: Vec<Value> = reg.catalog().into_iter().map(entry).collect();
        let queries: Vec<Value> = reg.query_catalog().into_iter().map(entry).collect();
        serde_json::json!({ "ops": ops, "queries": queries })
    }

    /// True if `name` would be accepted as a new branch name by the timeline.
    pub fn branch_name_ok(&self, name: &str) -> bool {
        valid_ref_name(name) && self.timeline().check_ref_name(name).is_ok()
    }

    /// Replay the node `h` from the root with no state caches (validation and tests).
    pub fn state_from_root(&self, h: Hash256) -> Result<State> {
        let q = QuietControl::new();
        Ok(self.session.state_at_from_root(h, &q.control())?)
    }
}

/// The implementation versions a snapshot at `node` depends on: `layer:<name>` for each layer in
/// `snap` and `op:<name>` for each op on the path from the root to `node` (nested batch ops
/// included), as the current registry reports them.
fn snapshot_versions(
    registry: &sm_op_api::Registry,
    tl: &Timeline,
    node: Hash256,
    snap: &sm_op_api::StateSnapshot,
) -> Result<BTreeMap<String, u32>> {
    let mut v = BTreeMap::new();
    for name in snap.layers.keys() {
        if let Some(ver) = registry.layer_digest_version(name) {
            v.insert(format!("layer:{name}"), ver);
        }
    }
    for h in tl.path_to_root(node)? {
        let Some(n) = tl.get(h) else { continue };
        let names = std::iter::once(n.body.op.name.clone()).chain(
            n.check
                .versions
                .keys()
                .filter_map(|k| k.strip_prefix("op:").map(str::to_owned)),
        );
        for name in names {
            if let Some(ver) = registry.op_impl_version(&name) {
                v.insert(format!("op:{name}"), ver);
            }
        }
    }
    Ok(v)
}

/// Whether every recorded version still equals the registry's (unknown names count as changed).
fn versions_current(registry: &sm_op_api::Registry, recorded: &BTreeMap<String, u32>) -> bool {
    recorded.iter().all(|(k, ver)| {
        let now = match k.split_once(':') {
            Some(("op", n)) => registry.op_impl_version(n),
            Some(("layer", n)) => registry.layer_digest_version(n),
            _ => None,
        };
        now == Some(*ver)
    })
}
