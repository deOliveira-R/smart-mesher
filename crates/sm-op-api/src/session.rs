//! [`Session`]: engine + timeline + HEAD + current state + caches. The reference implementation
//! of "apply an op, record it, check out any node, branch, undo".

use std::collections::{BTreeMap, VecDeque};

use serde::{Deserialize, Serialize};

use crate::cache::DerivedCache;
use crate::engine::Engine;
use crate::error::{ApplyError, TimelineError};
use crate::hash::Hash256;
use crate::op::{Delta, Op, OpRecord};
use crate::progress::{Control, QuietControl, Scaled};
use crate::state::State;
use crate::timeline::{Check, Head, Meta, Node, NodeBody, Timeline};
use std::sync::Arc;

/// A revision to check out.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Rev {
    /// The empty state, before any op (detached).
    Empty,
    /// A node by hash (detached).
    Node(Hash256),
    /// A branch (attached).
    Ref(String),
}

/// What replay does when a node's recorded result hash differs from the recomputed one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ReplayPolicy {
    /// Fail with [`TimelineError::Diverged`]. Determinism is a binding rule, so this is the default.
    #[default]
    Strict,
    /// Keep going and collect the divergence in [`Session::take_warnings`].
    Warn,
}

/// One movement of a ref or of HEAD, for the local reflog persisted by `sm-engine` in `.cache/`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReflogEntry {
    /// The branch that moved, or `None` when HEAD itself moved while detached.
    pub ref_name: Option<String>,
    /// Position before.
    pub from: Option<Hash256>,
    /// Position after.
    pub to: Option<Hash256>,
    /// What happened: `apply`, `checkout`, `branch`, `reset`, `undo`, `redo`.
    pub action: String,
}

/// An op applied to a cloned state but not yet recorded; produced off-thread by
/// [`PrepareBase::prepare`] and recorded with [`Session::commit`].
pub struct Prepared {
    engine: Engine,
    base: Option<Hash256>,
    record: OpRecord,
    state: State,
    delta: Delta,
}

impl Prepared {
    /// The position this op was prepared against.
    pub fn base(&self) -> Option<Hash256> {
        self.base
    }
    /// The stored form of the op.
    pub fn record(&self) -> &OpRecord {
        &self.record
    }
    /// What the op changed.
    pub fn delta(&self) -> &Delta {
        &self.delta
    }
    /// The resulting state.
    pub fn state(&self) -> &State {
        &self.state
    }
}

/// An owned copy of what is needed to run an op off-thread: the engine, the state and the
/// position. Take it with [`Session::prepare_base`], run [`prepare`](Self::prepare) on a worker
/// while the session stays usable, then [`Session::commit`] (which fails with
/// [`TimelineError::StaleBase`] if the session moved meanwhile).
#[derive(Clone)]
pub struct PrepareBase {
    engine: Engine,
    state: State,
    at: Option<Hash256>,
}

impl PrepareBase {
    /// Apply `op` (encoded and decoded first, so the live path is the replay path).
    pub fn prepare(&self, op: &dyn Op, ctl: &Control<'_>) -> Result<Prepared, TimelineError> {
        let record = OpRecord::of(op).map_err(|source| ApplyError::Encode {
            name: op.name().to_owned(),
            source,
        })?;
        // Apply the decoded record, so the live path is the replay path. An op such as
        // `core.batch` may hand back a more canonical stored form.
        let mut state = self.state.clone();
        let (delta, canonical) = self
            .engine
            .apply_record_canonical(&mut state, &record, ctl)?;
        let record = canonical.unwrap_or(record);
        Ok(Prepared {
            engine: self.engine.clone(),
            base: self.at,
            record,
            state,
            delta,
        })
    }

    /// Apply a stored-form op (remote clients, scripts). The record is decoded (migrated if
    /// older) and re-encoded canonically, so the same op is the same node whichever client
    /// produced it; fields the op does not know are dropped.
    pub fn prepare_record(
        &self,
        record: OpRecord,
        ctl: &Control<'_>,
    ) -> Result<Prepared, TimelineError> {
        let op = self
            .engine
            .registry()
            .decode(&record)
            .map_err(ApplyError::from)?;
        self.prepare(op.as_ref(), ctl)
    }
}

/// A working session over one timeline.
pub struct Session {
    engine: Engine,
    timeline: Timeline,
    head: Head,
    state: State,
    at: Option<Hash256>,
    snapshots: BTreeMap<Hash256, State>,
    snapshot_order: VecDeque<Hash256>,
    recent: VecDeque<(Hash256, State)>,
    snapshot_every: usize,
    max_snapshots: usize,
    recent_cap: usize,
    redo: Vec<Hash256>,
    reflog: Vec<ReflogEntry>,
    policy: ReplayPolicy,
    warnings: Vec<TimelineError>,
}

struct Replayed {
    state: State,
    snaps: Vec<(Hash256, State)>,
}

impl Session {
    /// New session on an empty timeline, HEAD on an unborn `main`.
    pub fn new(engine: Engine) -> Self {
        let state = engine.registry().new_state();
        Self {
            engine,
            timeline: Timeline::new(),
            head: Head::Ref("main".to_owned()),
            state,
            at: None,
            snapshots: BTreeMap::new(),
            snapshot_order: VecDeque::new(),
            recent: VecDeque::new(),
            snapshot_every: 16,
            max_snapshots: 64,
            recent_cap: 8,
            redo: Vec::new(),
            reflog: Vec::new(),
            policy: ReplayPolicy::Strict,
            warnings: Vec::new(),
        }
    }

    /// Resume over an existing timeline (e.g. loaded by `sm-engine`) with the HEAD recorded for
    /// this checkout, replaying to the head's node.
    pub fn open(
        engine: Engine,
        timeline: Timeline,
        head: Head,
        ctl: &Control<'_>,
    ) -> Result<Self, TimelineError> {
        let target = Self::resolve_head(&timeline, &head)?;
        let mut s = Self::new(engine);
        s.timeline = timeline;
        s.state = s.state_at(target, ctl)?;
        s.at = target;
        s.head = head;
        Ok(s)
    }

    fn resolve_head(t: &Timeline, head: &Head) -> Result<Option<Hash256>, TimelineError> {
        match head {
            Head::Ref(n) => {
                t.check_ref_name(n)?;
                Ok(t.refs().get(n).copied())
            }
            Head::Detached(Some(h)) => t
                .get(*h)
                .map(|_| Some(*h))
                .ok_or(TimelineError::UnknownNode(*h)),
            Head::Detached(None) => Ok(None),
        }
    }

    /// Tune the caches: keep a state every `snapshot_every` levels of depth (at most 64 are
    /// retained, oldest dropped first) and the last `recent_states` positions left behind.
    pub fn set_cache_policy(&mut self, snapshot_every: usize, recent_states: usize) {
        self.snapshot_every = snapshot_every.max(1);
        self.recent_cap = recent_states;
        self.recent.truncate(recent_states);
    }

    /// Choose what replay does on a result-hash mismatch (default: strict).
    pub fn set_replay_policy(&mut self, p: ReplayPolicy) {
        self.policy = p;
    }

    /// Divergences tolerated under [`ReplayPolicy::Warn`] since the last call.
    pub fn take_warnings(&mut self) -> Vec<TimelineError> {
        std::mem::take(&mut self.warnings)
    }

    /// The engine.
    pub fn engine(&self) -> &Engine {
        &self.engine
    }

    /// The derived cache shared by every state of this session.
    pub fn derived(&self) -> &Arc<DerivedCache> {
        self.engine.cache()
    }

    /// The timeline.
    pub fn timeline(&self) -> &Timeline {
        &self.timeline
    }

    /// HEAD (per-checkout state; `sm-engine` stores it in `.cache/HEAD`).
    pub fn head(&self) -> &Head {
        &self.head
    }

    /// The current state.
    pub fn state(&self) -> &State {
        &self.state
    }

    /// The node the state corresponds to (`None` = empty state).
    pub fn position(&self) -> Option<Hash256> {
        self.at
    }

    /// Movements of refs and HEAD in this session, oldest first.
    pub fn reflog(&self) -> &[ReflogEntry] {
        &self.reflog
    }

    /// Number of cached depth-interval states.
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }

    fn log(&mut self, action: &str, from: Option<Hash256>) {
        let ref_name = match &self.head {
            Head::Ref(n) => Some(n.clone()),
            Head::Detached(_) => None,
        };
        self.reflog.push(ReflogEntry {
            ref_name,
            from,
            to: self.at,
            action: action.to_owned(),
        });
    }

    /// Copy what is needed to prepare an op on another thread.
    pub fn prepare_base(&self) -> PrepareBase {
        PrepareBase {
            engine: self.engine.clone(),
            state: self.state.clone(),
            at: self.at,
        }
    }

    /// Apply `op` to a clone of the current state without recording it.
    pub fn prepare(&self, op: &dyn Op, ctl: &Control<'_>) -> Result<Prepared, TimelineError> {
        self.prepare_base().prepare(op, ctl)
    }

    /// Record a prepared op as a new node and advance the attached branch (or detached HEAD).
    /// Fails with [`TimelineError::StaleBase`] if the position moved since it was prepared.
    /// Nothing is recorded on failure.
    pub fn commit(&mut self, p: Prepared, meta: Meta) -> Result<(Hash256, Delta), TimelineError> {
        if !p.engine.same_context(&self.engine) {
            return Err(TimelineError::ForeignPrepared);
        }
        if p.base != self.at {
            return Err(TimelineError::StaleBase {
                expected: p.base,
                found: self.at,
            });
        }
        if let Head::Ref(name) = &self.head {
            self.timeline.check_ref_name(name)?;
        }
        let check = Check {
            state: p.state.hash(),
        };
        let h = self.timeline.insert(Node {
            body: NodeBody {
                parent: self.at,
                op: p.record,
                meta,
            },
            check,
        })?;
        let from = self.at;
        match self.head.clone() {
            Head::Ref(name) => self.timeline.set_ref(&name, h)?,
            Head::Detached(_) => self.head = Head::Detached(Some(h)),
        }
        let prev = std::mem::replace(&mut self.state, p.state);
        if let Some(f) = from {
            self.remember(f, prev);
        }
        self.at = Some(h);
        self.redo.clear();
        self.log("apply", from);
        self.maybe_snapshot(h);
        Ok((h, p.delta))
    }

    /// Apply an op at the current position and record it (prepare + commit).
    pub fn apply(
        &mut self,
        op: &dyn Op,
        meta: Meta,
        ctl: &Control<'_>,
    ) -> Result<(Hash256, Delta), TimelineError> {
        let p = self.prepare(op, ctl)?;
        self.commit(p, meta)
    }

    /// Apply a stored-form op (remote clients, scripts) and record it.
    pub fn apply_record(
        &mut self,
        rec: OpRecord,
        meta: Meta,
        ctl: &Control<'_>,
    ) -> Result<(Hash256, Delta), TimelineError> {
        let p = self.prepare_base().prepare_record(rec, ctl)?;
        self.commit(p, meta)
    }

    /// Install `new` as the current state, remembering the one left behind under `left`.
    fn swap_state(&mut self, left: Option<Hash256>, new: State) {
        let old = std::mem::replace(&mut self.state, new);
        if let Some(h) = left {
            self.remember(h, old);
        }
    }

    fn remember(&mut self, h: Hash256, s: State) {
        if self.recent_cap == 0 {
            return;
        }
        self.recent.retain(|(k, _)| *k != h);
        self.recent.push_back((h, s));
        while self.recent.len() > self.recent_cap {
            self.recent.pop_front();
        }
    }

    fn maybe_snapshot(&mut self, h: Hash256) {
        let depth = self.timeline.depth(h).unwrap_or(0);
        if depth > 0 && depth.is_multiple_of(self.snapshot_every) {
            self.store_snapshot(h, self.state.clone());
        }
    }

    fn store_snapshot(&mut self, h: Hash256, s: State) {
        if self.snapshots.insert(h, s).is_none() {
            self.snapshot_order.push_back(h);
            while self.snapshot_order.len() > self.max_snapshots {
                if let Some(old) = self.snapshot_order.pop_front() {
                    self.snapshots.remove(&old);
                }
            }
        }
    }

    fn cached_state(&self, h: Hash256) -> Option<&State> {
        if self.at == Some(h) {
            return Some(&self.state);
        }
        self.recent
            .iter()
            .find(|(k, _)| *k == h)
            .map(|(_, s)| s)
            .or_else(|| self.snapshots.get(&h))
    }

    /// Replay `nodes` (a chain, each the child of the previous) onto `state`, verifying every
    /// recorded result hash.
    fn replay(
        &self,
        mut state: State,
        nodes: &[Hash256],
        base_depth: usize,
        ctl: &Control<'_>,
        keep_snaps: bool,
        warnings: &mut Vec<TimelineError>,
    ) -> Result<Replayed, TimelineError> {
        let n = nodes.len();
        let mut snaps = Vec::new();
        for (i, h) in nodes.iter().enumerate() {
            if ctl.cancel.is_cancelled() {
                return Err(TimelineError::Cancelled);
            }
            let node = self
                .timeline
                .get(*h)
                .ok_or(TimelineError::UnknownNode(*h))?;
            ctl.progress.report(i as f32 / n.max(1) as f32, "replay");
            let scaled = Scaled {
                inner: ctl.progress,
                lo: i as f32 / n as f32,
                hi: (i + 1) as f32 / n as f32,
            };
            let sub = Control::new(&scaled, ctl.cancel);
            self.engine
                .apply_record(&mut state, &node.body.op, &sub)
                .map_err(|e| {
                    if e.is_cancelled() {
                        TimelineError::Cancelled
                    } else {
                        TimelineError::Apply(e)
                    }
                })?;
            let actual = state.hash();
            if actual != node.check.state {
                let err = TimelineError::Diverged {
                    node: *h,
                    recorded: node.check.state,
                    actual,
                };
                match self.policy {
                    ReplayPolicy::Strict => return Err(err),
                    ReplayPolicy::Warn => warnings.push(err),
                }
            }
            if keep_snaps && (base_depth + i + 1).is_multiple_of(self.snapshot_every) {
                snaps.push((*h, state.clone()));
            }
        }
        ctl.progress.report(1.0, "replay");
        Ok(Replayed { state, snaps })
    }

    /// Reconstruct the state at `target`, replaying from the nearest cached state (current
    /// position, recent positions, depth snapshots) or the root, verifying each node's recorded
    /// result hash. Reports progress and honours cancellation.
    pub fn state_at(
        &mut self,
        target: Option<Hash256>,
        ctl: &Control<'_>,
    ) -> Result<State, TimelineError> {
        let Some(target) = target else {
            return Ok(self.engine.registry().new_state());
        };
        if let Some(s) = self.cached_state(target) {
            return Ok(s.clone());
        }
        let path = self.timeline.path_to_root(target)?;
        let start = path
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, h)| self.cached_state(*h).map(|s| (i, s.clone())));
        let (state, from) = match start {
            Some((i, s)) => (s, i + 1),
            None => (self.engine.registry().new_state(), 0),
        };
        let mut warnings = Vec::new();
        let r = self.replay(state, &path[from..], from, ctl, true, &mut warnings)?;
        self.warnings.extend(warnings);
        for (h, s) in r.snaps {
            self.store_snapshot(h, s);
        }
        Ok(r.state)
    }

    /// Reconstruct the state at `h` by replaying every node from the root, using no cache. For
    /// validating snapshots and for determinism tests.
    pub fn state_at_from_root(
        &self,
        h: Hash256,
        ctl: &Control<'_>,
    ) -> Result<State, TimelineError> {
        let path = self.timeline.path_to_root(h)?;
        let mut warnings = Vec::new();
        Ok(self
            .replay(
                self.engine.registry().new_state(),
                &path,
                0,
                ctl,
                false,
                &mut warnings,
            )?
            .state)
    }

    /// Check out a revision. On error the session is unchanged.
    pub fn checkout(&mut self, rev: Rev, ctl: &Control<'_>) -> Result<(), TimelineError> {
        let (head, node) = match rev {
            Rev::Empty => (Head::Detached(None), None),
            Rev::Node(h) => {
                self.timeline.get(h).ok_or(TimelineError::UnknownNode(h))?;
                (Head::Detached(Some(h)), Some(h))
            }
            Rev::Ref(name) => {
                let h = *self
                    .timeline
                    .refs()
                    .get(&name)
                    .ok_or_else(|| TimelineError::UnknownRef(name.clone()))?;
                (Head::Ref(name), Some(h))
            }
        };
        let state = self.state_at(node, ctl)?;
        let from = self.at;
        self.swap_state(from, state);
        self.head = head;
        self.at = node;
        self.redo.clear();
        self.log("checkout", from);
        Ok(())
    }

    /// Create a branch at the current position and attach HEAD to it.
    pub fn branch(&mut self, name: &str) -> Result<(), TimelineError> {
        let at = self.at.ok_or(TimelineError::EmptyPosition)?;
        self.timeline.create_ref(name, at)?;
        self.head = Head::Ref(name.to_owned());
        self.redo.clear();
        self.log("branch", Some(at));
        Ok(())
    }

    /// Move an existing branch to `to` (git reset). If HEAD is attached to it, the session
    /// moves too. Nodes the branch no longer reaches stay in the timeline and remain
    /// checkout-able by hash.
    pub fn reset_branch(
        &mut self,
        name: &str,
        to: Hash256,
        ctl: &Control<'_>,
    ) -> Result<(), TimelineError> {
        if !self.timeline.refs().contains_key(name) {
            return Err(TimelineError::UnknownRef(name.to_owned()));
        }
        self.timeline
            .get(to)
            .ok_or(TimelineError::UnknownNode(to))?;
        let attached = self.head == Head::Ref(name.to_owned());
        let state = if attached {
            Some(self.state_at(Some(to), ctl)?)
        } else {
            None
        };
        let prev_tip = self.timeline.refs().get(name).copied();
        let at = self.at;
        self.timeline.set_ref(name, to)?;
        if let Some(s) = state {
            self.swap_state(at, s);
            self.at = Some(to);
        }
        self.redo.clear();
        let entry = ReflogEntry {
            ref_name: Some(name.to_owned()),
            from: prev_tip,
            to: Some(to),
            action: "reset".to_owned(),
        };
        self.reflog.push(entry);
        Ok(())
    }

    /// Undo the last op by moving the attached branch to the current node's parent (git reset).
    /// The undone node stays in the timeline, so it can still be checked out or branched from;
    /// [`redo`](Self::redo) moves the branch back. Undoing a root deletes the ref (the branch
    /// becomes unborn). Needs HEAD attached.
    ///
    /// Parent states come from the recent-state cache when possible; otherwise they are
    /// replayed without progress reporting (use [`checkout`](Self::checkout) for cancellable
    /// navigation).
    pub fn undo(&mut self) -> Result<(), TimelineError> {
        let Head::Ref(name) = self.head.clone() else {
            return Err(TimelineError::NotAttached);
        };
        let tip = self.at.ok_or(TimelineError::NothingToUndo)?;
        let parent = self
            .timeline
            .get(tip)
            .ok_or(TimelineError::UnknownNode(tip))?
            .body
            .parent;
        let q = QuietControl::new();
        let state = self.state_at(parent, &q.control())?;
        match parent {
            Some(p) => self.timeline.set_ref(&name, p)?,
            None => {
                self.timeline.delete_ref(&name)?;
            }
        }
        self.swap_state(Some(tip), state);
        self.at = parent;
        self.redo.push(tip);
        self.log("undo", Some(tip));
        Ok(())
    }

    /// Re-apply the most recently undone node by moving the branch back to it.
    pub fn redo(&mut self) -> Result<(), TimelineError> {
        let Head::Ref(name) = self.head.clone() else {
            return Err(TimelineError::NotAttached);
        };
        let target = *self.redo.last().ok_or(TimelineError::NothingToRedo)?;
        let node = self
            .timeline
            .get(target)
            .ok_or(TimelineError::UnknownNode(target))?;
        if node.body.parent != self.at {
            self.redo.clear();
            return Err(TimelineError::NothingToRedo);
        }
        let q = QuietControl::new();
        let state = self.state_at(Some(target), &q.control())?;
        self.timeline.set_ref(&name, target)?;
        let from = self.at;
        self.swap_state(from, state);
        self.at = Some(target);
        self.redo.pop();
        self.log("redo", from);
        Ok(())
    }
}
