//! Reference in-memory engine: current state, timeline, periodic snapshots, checkout and fork.

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::ctx::{Control, Delta, NoControl};
use crate::derived::DerivedCache;
use crate::error::OpError;
use crate::hash::Hash256;
use crate::layer::{OpOf, Stack, op_version};
use crate::state::State;
use crate::timeline::{Head, Meta, NODE_FORMAT, NodeBody, Timeline, TimelineError};

/// Session tuning.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionConfig {
    /// Keep a state snapshot at every node whose depth is a multiple of this (0 disables).
    pub snapshot_every: u64,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self { snapshot_every: 16 }
    }
}

/// Errors from session operations.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SessionError {
    /// The op failed; nothing changed.
    #[error(transparent)]
    Op(#[from] OpError),
    /// A timeline operation failed; nothing changed.
    #[error(transparent)]
    Timeline(#[from] TimelineError),
    /// A stored op failed while replaying, which means non-determinism or corruption.
    #[error("replay of node {node} failed: {source}")]
    Replay {
        /// The node that failed.
        node: Hash256,
        /// The underlying error.
        source: OpError,
    },
    /// A stored node's op schema version differs from the running layer's version.
    #[error("node {node} has op version {found}, this build expects {expected}")]
    OpVersion {
        /// The node.
        node: Hash256,
        /// Version found in the node.
        found: u32,
        /// Version this build understands.
        expected: u32,
    },
}

/// Result of a successful [`Session::apply`].
#[derive(Clone, Debug, PartialEq)]
pub struct Applied {
    /// Hash of the new timeline node.
    pub node: Hash256,
    /// What the op changed.
    pub delta: Delta,
}

/// The engine: applies ops, records them in the timeline, and navigates history.
pub struct Session<S: Stack> {
    timeline: Timeline<OpOf<S>>,
    snapshots: BTreeMap<Hash256, State<S>>,
    config: SessionConfig,
    current: State<S>,
    derived: Arc<DerivedCache>,
}

impl<S: Stack> Session<S> {
    /// A new session with an empty state and timeline.
    pub fn new(config: SessionConfig) -> Self {
        let derived = Arc::new(DerivedCache::default());
        Self {
            timeline: Timeline::new(),
            snapshots: BTreeMap::new(),
            config,
            current: State::with_derived(derived.clone()),
            derived,
        }
    }

    /// Build a session from a loaded timeline, replaying to HEAD. Ops are checked against the
    /// running layers' op versions.
    pub fn from_timeline(
        timeline: Timeline<OpOf<S>>,
        config: SessionConfig,
    ) -> Result<Self, SessionError> {
        let derived = Arc::new(DerivedCache::default());
        let current = State::with_derived(derived.clone());
        let mut s = Self {
            timeline,
            snapshots: BTreeMap::new(),
            config,
            current,
            derived,
        };
        if let Some(head) = s.timeline.head_commit() {
            s.current = s.replay_filling_snapshots(&head)?;
        }
        Ok(s)
    }

    /// The current state (the state at HEAD).
    pub fn state(&self) -> &State<S> {
        &self.current
    }
    /// The timeline.
    pub fn timeline(&self) -> &Timeline<OpOf<S>> {
        &self.timeline
    }
    /// Number of stored snapshots.
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }
    /// Whether a snapshot is stored for `node`.
    pub fn has_snapshot(&self, node: &Hash256) -> bool {
        self.snapshots.contains_key(node)
    }

    /// Apply `op` at HEAD with no progress hook. See [`Session::apply_with`].
    pub fn apply(&mut self, op: OpOf<S>, meta: Meta) -> Result<Applied, SessionError> {
        self.apply_with(op, meta, &NoControl)
    }

    /// Apply `op` at HEAD, append a node, advance the branch (or detached HEAD).
    ///
    /// Transactional: the op runs on a copy of the state; on any error (including cancellation)
    /// neither the state, the timeline nor any ref has changed.
    pub fn apply_with(
        &mut self,
        op: OpOf<S>,
        meta: Meta,
        control: &dyn Control,
    ) -> Result<Applied, SessionError> {
        let (next, delta) = self.current.apply(&op, control)?;
        let body = NodeBody {
            format: NODE_FORMAT,
            parent: self.timeline.head_commit(),
            op_version: op_version::<S>(&op),
            meta,
            op,
        };
        let node = self.timeline.insert(body)?;
        match self.timeline.head().clone() {
            Head::Branch(name) => self.timeline.set_branch(&name, node)?,
            Head::Detached(_) => self.timeline.set_head(Head::Detached(node))?,
        }
        self.maybe_snapshot(&node, &next);
        self.current = next;
        Ok(Applied { node, delta })
    }

    /// Check out a revision (`HEAD`, branch name, hash or prefix). A branch name puts HEAD on the
    /// branch; anything else detaches HEAD. The current state becomes the state at that node.
    pub fn checkout(&mut self, rev: &str) -> Result<(), SessionError> {
        let node = self.timeline.resolve(rev)?;
        let state = self.state_at(&node)?;
        let head = if self.timeline.branch(rev) == Some(node) {
            Head::Branch(rev.to_string())
        } else {
            Head::Detached(node)
        };
        self.timeline.set_head(head)?;
        self.current = state;
        Ok(())
    }

    /// Create a branch at the current HEAD commit and switch to it. On an unborn timeline the
    /// branch is unborn too (HEAD just names it).
    pub fn branch(&mut self, name: &str) -> Result<(), SessionError> {
        match self.timeline.head_commit() {
            Some(at) => {
                self.timeline.create_branch(name, at)?;
                self.timeline.set_head(Head::Branch(name.to_string()))?;
            }
            None => self.timeline.set_head(Head::Branch(name.to_string()))?,
        }
        Ok(())
    }

    /// The state at `node`: replay from the nearest ancestor snapshot (or the root).
    pub fn state_at(&self, node: &Hash256) -> Result<State<S>, SessionError> {
        let (base, ops) = self.plan(node, true)?;
        self.replay(base, &ops)
    }

    /// The state at `node`, always replayed from the root (ignores snapshots). Used to validate
    /// snapshots and determinism.
    pub fn state_at_from_root(&self, node: &Hash256) -> Result<State<S>, SessionError> {
        let (base, ops) = self.plan(node, false)?;
        self.replay(base, &ops)
    }

    fn plan(
        &self,
        node: &Hash256,
        use_snapshots: bool,
    ) -> Result<(State<S>, Vec<Hash256>), SessionError> {
        let path = self.timeline.path_to(node)?;
        let start = if use_snapshots {
            path.iter().rposition(|h| self.snapshots.contains_key(h))
        } else {
            None
        };
        Ok(match start {
            Some(i) => (self.snapshots[&path[i]].clone(), path[i + 1..].to_vec()),
            None => (State::with_derived(self.derived.clone()), path),
        })
    }

    fn replay(&self, mut state: State<S>, nodes: &[Hash256]) -> Result<State<S>, SessionError> {
        for h in nodes {
            state = self.step(&state, h)?;
        }
        Ok(state)
    }

    fn step(&self, state: &State<S>, h: &Hash256) -> Result<State<S>, SessionError> {
        let n = self
            .timeline
            .node(h)
            .ok_or(TimelineError::UnknownNode(*h))?;
        let expected = op_version::<S>(&n.body().op);
        if n.body().op_version != expected {
            return Err(SessionError::OpVersion {
                node: *h,
                found: n.body().op_version,
                expected,
            });
        }
        state
            .apply(&n.body().op, &NoControl)
            .map(|(s, _)| s)
            .map_err(|source| SessionError::Replay { node: *h, source })
    }

    fn replay_filling_snapshots(&mut self, head: &Hash256) -> Result<State<S>, SessionError> {
        let path = self.timeline.path_to(head)?;
        let mut state = State::with_derived(self.derived.clone());
        for h in &path {
            state = self.step(&state, h)?;
            self.maybe_snapshot(h, &state);
        }
        Ok(state)
    }

    fn maybe_snapshot(&mut self, node: &Hash256, state: &State<S>) {
        let every = self.config.snapshot_every;
        if every > 0 && self.timeline.depth(node).is_some_and(|d| d % every == 0) {
            self.snapshots.insert(*node, state.clone());
        }
    }
}
