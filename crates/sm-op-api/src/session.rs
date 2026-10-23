//! [`Session`]: engine + timeline + current state + snapshots. The reference implementation of
//! "apply an op, record it, check out any node, branch".

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::engine::Engine;
use crate::error::{ApplyError, TimelineError};
use crate::hash::Hash256;
use crate::op::{Delta, Op, OpRecord};
use crate::progress::{NoProgress, Progress};
use crate::registry::Registry;
use crate::state::State;
use crate::timeline::{Head, Meta, Node, Timeline};

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

/// A working session over one timeline.
pub struct Session {
    engine: Engine,
    timeline: Timeline,
    state: State,
    at: Option<Hash256>,
    snapshots: BTreeMap<Hash256, State>,
    snapshot_every: usize,
}

impl Session {
    /// New session on an empty timeline.
    pub fn new(registry: Arc<Registry>) -> Self {
        let state = registry.new_state();
        Self {
            engine: Engine::new(registry),
            timeline: Timeline::new(),
            state,
            at: None,
            snapshots: BTreeMap::new(),
            snapshot_every: 16,
        }
    }

    /// Resume over an existing timeline (e.g. loaded from disk), checked out at HEAD.
    pub fn with_timeline(registry: Arc<Registry>, timeline: Timeline) -> Result<Self, TimelineError> {
        let mut s = Self::new(registry);
        let head = timeline.head_node();
        s.timeline = timeline;
        if let Some(h) = head {
            s.state = s.state_at(Some(h))?;
            s.at = Some(h);
        }
        Ok(s)
    }

    /// Cache a snapshot every `n` nodes of depth (minimum 1).
    pub fn set_snapshot_interval(&mut self, n: usize) {
        self.snapshot_every = n.max(1);
    }

    /// The engine.
    pub fn engine(&self) -> &Engine {
        &self.engine
    }

    /// The timeline.
    pub fn timeline(&self) -> &Timeline {
        &self.timeline
    }

    /// The current state.
    pub fn state(&self) -> &State {
        &self.state
    }

    /// The node the state corresponds to (`None` = empty state).
    pub fn position(&self) -> Option<Hash256> {
        self.at
    }

    /// Number of cached snapshots.
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }

    /// Apply an op at the current position and record it. Without progress or cancellation.
    pub fn apply(&mut self, op: &dyn Op, meta: Meta) -> Result<(Hash256, Delta), TimelineError> {
        self.apply_with(op, meta, &NoProgress)
    }

    /// Apply with a progress/cancellation sink. The op is encoded and decoded back before it
    /// runs, so the live path is exactly the replay path. On any failure nothing is recorded
    /// and the state, position and refs are unchanged.
    pub fn apply_with(&mut self, op: &dyn Op, meta: Meta, progress: &dyn Progress) -> Result<(Hash256, Delta), TimelineError> {
        let rec = OpRecord::of(op)
            .map_err(|source| ApplyError::Encode { name: op.name().to_owned(), source })?;
        self.apply_record(rec, meta, progress)
    }

    /// Apply a stored-form op (used by remote clients and scripts).
    pub fn apply_record(&mut self, rec: OpRecord, meta: Meta, progress: &dyn Progress) -> Result<(Hash256, Delta), TimelineError> {
        let mut next = self.state.clone();
        let delta = self.engine.apply_record(&mut next, &rec, progress)?;
        let node = Node { parent: self.at, op: rec, meta, state: next.hash() };
        let h = self.timeline.insert(node)?;
        // Commit: nothing below can fail except ref bookkeeping on names we already validated.
        match self.timeline.head().clone() {
            Head::Ref(name) => self.timeline.set_ref(&name, h)?,
            Head::Detached(_) => self.timeline.set_head(Head::Detached(Some(h)))?,
        }
        self.state = next;
        self.at = Some(h);
        self.maybe_snapshot(h)?;
        Ok((h, delta))
    }

    fn maybe_snapshot(&mut self, h: Hash256) -> Result<(), TimelineError> {
        let depth = self.timeline.path_to_root(h)?.len();
        if depth.is_multiple_of(self.snapshot_every) {
            self.snapshots.insert(h, self.state.clone());
        }
        Ok(())
    }

    /// Reconstruct the state at `target` by replaying from the nearest cached snapshot (or
    /// the root), verifying each node's recorded state hash.
    pub fn state_at(&mut self, target: Option<Hash256>) -> Result<State, TimelineError> {
        let Some(target) = target else { return Ok(self.engine.registry().new_state()) };
        let path = self.timeline.path_to_root(target)?;
        let start = path.iter().rposition(|h| self.snapshots.contains_key(h));
        let (mut state, from) = match start {
            Some(i) => (self.snapshots[&path[i]].clone(), i + 1),
            None => (self.engine.registry().new_state(), 0),
        };
        for (i, h) in path.iter().enumerate().skip(from) {
            let node = self.timeline.get(*h).ok_or(TimelineError::UnknownNode(*h))?.clone();
            self.engine.apply_record(&mut state, &node.op, &NoProgress)?;
            let actual = state.hash();
            if actual != node.state {
                return Err(TimelineError::Diverged { node: *h, recorded: node.state, actual });
            }
            if (i + 1).is_multiple_of(self.snapshot_every) {
                self.snapshots.insert(*h, state.clone());
            }
        }
        Ok(state)
    }

    /// Check out a revision. On error the session is unchanged.
    pub fn checkout(&mut self, rev: Rev) -> Result<(), TimelineError> {
        let (head, node) = match rev {
            Rev::Empty => (Head::Detached(None), None),
            Rev::Node(h) => (Head::Detached(Some(h)), Some(h)),
            Rev::Ref(name) => {
                let h = *self.timeline.refs().get(&name).ok_or_else(|| TimelineError::UnknownRef(name.clone()))?;
                (Head::Ref(name), Some(h))
            }
        };
        let state = self.state_at(node)?;
        self.timeline.set_head(head)?;
        self.state = state;
        self.at = node;
        Ok(())
    }

    /// Create a branch at the current position and attach HEAD to it.
    pub fn branch(&mut self, name: &str) -> Result<(), TimelineError> {
        let at = self.at.ok_or(TimelineError::EmptyPosition)?;
        self.timeline.create_ref(name, at)?;
        self.timeline.set_head(Head::Ref(name.to_owned()))
    }
}
