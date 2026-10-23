//! The timeline: a DAG of content-addressed [`Node`]s, a [`TimelineStore`] contract with a
//! reference [`MemStore`] that holds the exact on-disk file layout, and the [`Workspace`]
//! that applies ops, forks and checks out by walking deltas.

use crate::error::TimelineError;
use crate::hash::Hash;
use crate::op::{ApplyOptions, Op, OpRecord, Registry};
use crate::state::{Delta, State};
use crate::toml;
use crate::value::{Table, Value, from_value, to_value};
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use std::collections::BTreeMap;
use std::sync::Arc;

/// On-disk format version of node files.
pub const NODE_FORMAT: i64 = 1;

/// Free-form metadata of a node. It is part of the node hash (like a git commit), but not
/// of the state hash. The api never reads a clock: `time` is whatever the caller supplies
/// (empty is fine), so replays and tests stay deterministic.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct NodeMeta {
    /// Who or what issued the op (user, `mcp`, `python`, ...).
    pub author: String,
    /// Caller-supplied timestamp text (RFC 3339 recommended).
    pub time: String,
    /// Optional note.
    pub message: Option<String>,
}

/// One timeline node: an op applied on top of `parent`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Node {
    /// Parent node; `None` for a root.
    pub parent: Option<Hash>,
    /// The op.
    pub op: OpRecord,
    /// Metadata.
    pub meta: NodeMeta,
}

impl Node {
    fn to_table(&self) -> Result<Table, TimelineError> {
        let Value::Table(mut t) = to_value(self)? else {
            return Err(TimelineError::Corrupt("node is not a table".into()));
        };
        t.insert("format".into(), Value::Int(NODE_FORMAT));
        Ok(t)
    }

    /// Canonical TOML text. The node hash is computed over exactly this text.
    pub fn to_toml(&self) -> Result<String, TimelineError> {
        Ok(toml::to_string(&self.to_table()?))
    }

    /// Parse node TOML.
    pub fn from_toml(s: &str) -> Result<Self, TimelineError> {
        let mut t = toml::from_str(s)?;
        match t.remove("format") {
            Some(Value::Int(NODE_FORMAT)) => {}
            other => {
                return Err(TimelineError::Corrupt(format!(
                    "unsupported node format {other:?}"
                )));
            }
        }
        Ok(from_value(&Value::Table(t))?)
    }

    /// Content hash (the node's identity).
    pub fn hash(&self) -> Result<Hash, TimelineError> {
        let text = self.to_toml()?;
        let mut bytes = b"smesh-node-v1\n".to_vec();
        bytes.extend_from_slice(text.as_bytes());
        Ok(Hash::of(&bytes))
    }
}

/// What HEAD points at.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Head {
    /// A named branch (`refs/heads/<name>`); new nodes advance it.
    Branch(String),
    /// A bare node (detached); new nodes are reachable only through HEAD until named.
    Detached(Hash),
}

/// Where to move to.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Target {
    /// A node by hash (detaches HEAD).
    Node(Hash),
    /// A branch by name (attaches HEAD).
    Branch(String),
}

/// Persistence contract for a timeline. Nodes are immutable and content addressed; refs and
/// HEAD are the only mutable pieces. Deltas are a cache: a store may drop or corrupt them
/// and the workspace recovers by re-running ops.
pub trait TimelineStore {
    /// Store a node under its hash (idempotent).
    fn put_node(&mut self, hash: Hash, node: &Node) -> Result<(), TimelineError>;
    /// Load and verify a node.
    fn node(&self, hash: &Hash) -> Result<Option<Node>, TimelineError>;
    /// Store the delta of a node.
    fn put_delta(&mut self, node: Hash, delta: &Delta) -> Result<(), TimelineError>;
    /// Load the delta of a node, `None` when absent.
    fn delta(&self, node: &Hash) -> Result<Option<Delta>, TimelineError>;
    /// Point a branch at a node.
    fn set_branch(&mut self, name: &str, node: Hash) -> Result<(), TimelineError>;
    /// Resolve a branch.
    fn branch(&self, name: &str) -> Result<Option<Hash>, TimelineError>;
    /// All branches, sorted by name.
    fn branches(&self) -> Result<Vec<(String, Hash)>, TimelineError>;
    /// Persist HEAD.
    fn set_head(&mut self, head: &Head) -> Result<(), TimelineError>;
    /// Read HEAD (default: unborn `main`).
    fn head(&self) -> Result<Head, TimelineError>;
}

/// Reference store keeping the exact project file layout in memory as `path -> text`:
///
/// ```text
/// timeline/ops/<hash>.toml      node (parent, op, meta)
/// timeline/deltas/<hash>.toml   reversible delta of that node (derived, optional)
/// timeline/refs/heads/<name>    node hash
/// timeline/HEAD                 `ref: heads/<name>` or a node hash
/// ```
///
/// A disk-backed store only has to write the same files.
#[derive(Clone, Debug, Default)]
pub struct MemStore {
    files: BTreeMap<String, String>,
}

impl MemStore {
    /// Empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Rebuild a store from a previously exported file set.
    pub fn from_files(files: BTreeMap<String, String>) -> Self {
        MemStore { files }
    }

    /// All files (what would be written to `<project>.smesh/`).
    pub fn files(&self) -> &BTreeMap<String, String> {
        &self.files
    }

    /// Mutable file access, for tooling and tests that simulate edits or corruption.
    pub fn files_mut(&mut self) -> &mut BTreeMap<String, String> {
        &mut self.files
    }

    fn node_path(h: &Hash) -> String {
        format!("timeline/ops/{h}.toml")
    }
    fn delta_path(h: &Hash) -> String {
        format!("timeline/deltas/{h}.toml")
    }
}

impl TimelineStore for MemStore {
    fn put_node(&mut self, hash: Hash, node: &Node) -> Result<(), TimelineError> {
        self.files.insert(Self::node_path(&hash), node.to_toml()?);
        Ok(())
    }

    fn node(&self, hash: &Hash) -> Result<Option<Node>, TimelineError> {
        let Some(text) = self.files.get(&Self::node_path(hash)) else {
            return Ok(None);
        };
        let node = Node::from_toml(text)?;
        if node.hash()? != *hash {
            return Err(TimelineError::Corrupt(format!(
                "node file {hash} does not match its hash"
            )));
        }
        Ok(Some(node))
    }

    fn put_delta(&mut self, node: Hash, delta: &Delta) -> Result<(), TimelineError> {
        let text = delta.to_toml()?;
        self.files.insert(Self::delta_path(&node), text);
        Ok(())
    }

    fn delta(&self, node: &Hash) -> Result<Option<Delta>, TimelineError> {
        match self.files.get(&Self::delta_path(node)) {
            None => Ok(None),
            Some(t) => Delta::from_toml(t)
                .map(Some)
                .map_err(|e| TimelineError::Corrupt(e.to_string())),
        }
    }

    fn set_branch(&mut self, name: &str, node: Hash) -> Result<(), TimelineError> {
        self.files
            .insert(format!("timeline/refs/heads/{name}"), format!("{node}\n"));
        Ok(())
    }

    fn branch(&self, name: &str) -> Result<Option<Hash>, TimelineError> {
        match self.files.get(&format!("timeline/refs/heads/{name}")) {
            None => Ok(None),
            Some(t) => Hash::from_hex(t.trim())
                .map(Some)
                .map_err(|e| TimelineError::Corrupt(e.to_string())),
        }
    }

    fn branches(&self) -> Result<Vec<(String, Hash)>, TimelineError> {
        const P: &str = "timeline/refs/heads/";
        let mut out = Vec::new();
        for (path, text) in &self.files {
            if let Some(name) = path.strip_prefix(P) {
                let h = Hash::from_hex(text.trim())
                    .map_err(|e| TimelineError::Corrupt(e.to_string()))?;
                out.push((name.to_owned(), h));
            }
        }
        Ok(out)
    }

    fn set_head(&mut self, head: &Head) -> Result<(), TimelineError> {
        let text = match head {
            Head::Branch(b) => format!("ref: heads/{b}\n"),
            Head::Detached(h) => format!("{h}\n"),
        };
        self.files.insert("timeline/HEAD".into(), text);
        Ok(())
    }

    fn head(&self) -> Result<Head, TimelineError> {
        match self.files.get("timeline/HEAD") {
            None => Ok(Head::Branch("main".into())),
            Some(t) => {
                let t = t.trim();
                match t.strip_prefix("ref: heads/") {
                    Some(b) => Ok(Head::Branch(b.to_owned())),
                    None => Hash::from_hex(t)
                        .map(Head::Detached)
                        .map_err(|e| TimelineError::Corrupt(e.to_string())),
                }
            }
        }
    }
}

/// Outcome of [`Workspace::apply`].
#[derive(Clone, Debug)]
pub struct ApplyReport {
    /// The new node.
    pub node: Hash,
    /// Ids allocated by the op, in allocation order; later ops may reference them.
    pub created: Vec<EntityId>,
    /// State hash after the op.
    pub state_hash: Hash,
}

fn valid_branch_name(n: &str) -> bool {
    !n.is_empty()
        && !n.contains("..")
        && !n.starts_with('/')
        && !n.ends_with('/')
        && n.bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-' | b'/'))
}

/// A live session over one timeline: the current [`State`], where it sits in the DAG, and
/// the store. All state changes go through [`Workspace::apply`]; moving around goes through
/// [`Workspace::checkout`].
///
/// Invariant: `state()` is always exactly the state of node `at()` (or the empty state when
/// `at()` is `None`).
pub struct Workspace<S: TimelineStore> {
    registry: Arc<Registry>,
    store: S,
    state: State,
    at: Option<Hash>,
    head: Head,
}

impl<S: TimelineStore> Workspace<S> {
    /// Open a store: resolves HEAD and brings the state there, using stored deltas where
    /// they verify and replaying ops otherwise.
    pub fn open(registry: Arc<Registry>, store: S) -> Result<Self, TimelineError> {
        let head = store.head()?;
        let target = match &head {
            Head::Branch(b) => store.branch(b)?,
            Head::Detached(h) => Some(*h),
        };
        let state = registry.new_state();
        let mut ws = Workspace {
            registry,
            store,
            state,
            at: None,
            head,
        };
        ws.goto(target)?;
        Ok(ws)
    }

    /// Current state.
    pub fn state(&self) -> &State {
        &self.state
    }

    /// Hash of the current state.
    pub fn state_hash(&self) -> Hash {
        self.state.hash()
    }

    /// Current node (`None` before the first op).
    pub fn at(&self) -> Option<Hash> {
        self.at
    }

    /// What HEAD points at.
    pub fn head(&self) -> &Head {
        &self.head
    }

    /// The store (to export files).
    pub fn store(&self) -> &S {
        &self.store
    }

    /// The registry.
    pub fn registry(&self) -> &Arc<Registry> {
        &self.registry
    }

    /// Load a node.
    pub fn node(&self, h: &Hash) -> Result<Node, TimelineError> {
        self.store.node(h)?.ok_or(TimelineError::UnknownNode(*h))
    }

    /// Hashes from the root to `h` inclusive.
    pub fn path_to(&self, h: &Hash) -> Result<Vec<Hash>, TimelineError> {
        let mut path = vec![*h];
        let mut cur = self.node(h)?;
        while let Some(p) = cur.parent {
            path.push(p);
            cur = self.node(&p)?;
        }
        path.reverse();
        Ok(path)
    }

    /// Typed convenience over [`Workspace::apply_record`] with default options.
    pub fn apply<O: Op>(&mut self, op: &O, meta: NodeMeta) -> Result<ApplyReport, TimelineError> {
        let rec = OpRecord::new(op)?;
        self.apply_record(rec, meta, &ApplyOptions::default())
    }

    /// Apply an op on top of the current node, append it to the timeline and advance HEAD.
    ///
    /// Transactional: if the op fails (or is cancelled), the state, the timeline and the
    /// refs are exactly as before the call.
    pub fn apply_record(
        &mut self,
        record: OpRecord,
        meta: NodeMeta,
        opts: &ApplyOptions<'_>,
    ) -> Result<ApplyReport, TimelineError> {
        let node = Node {
            parent: self.at,
            op: record,
            meta,
        };
        let hash = node.hash()?;
        let applied = self.registry.apply(&mut self.state, &node.op, opts)?;
        if let Err(e) = self.persist(&node, hash, &applied.delta) {
            // Undo the in-memory change; the store write failed.
            self.state.revert_delta(&applied.delta)?;
            return Err(e);
        }
        self.at = Some(hash);
        Ok(ApplyReport {
            node: hash,
            created: applied.created,
            state_hash: applied.delta.state_after,
        })
    }

    fn persist(&mut self, node: &Node, hash: Hash, delta: &Delta) -> Result<(), TimelineError> {
        self.store.put_node(hash, node)?;
        self.store.put_delta(hash, delta)?;
        match &self.head {
            Head::Branch(b) => {
                self.store.set_branch(b, hash)?;
                self.store.set_head(&self.head)?;
            }
            Head::Detached(_) => {
                let h = Head::Detached(hash);
                self.store.set_head(&h)?;
                self.head = h;
            }
        }
        Ok(())
    }

    /// Create a branch at the current node and attach HEAD to it (`git checkout -b`).
    pub fn create_branch(&mut self, name: &str) -> Result<(), TimelineError> {
        if !valid_branch_name(name) {
            return Err(TimelineError::Corrupt(format!(
                "invalid branch name `{name}`"
            )));
        }
        let at = self
            .at
            .ok_or_else(|| TimelineError::Corrupt("cannot branch from an empty timeline".into()))?;
        if self.store.branch(name)?.is_some() {
            return Err(TimelineError::RefExists(name.to_owned()));
        }
        self.store.set_branch(name, at)?;
        let h = Head::Branch(name.to_owned());
        self.store.set_head(&h)?;
        self.head = h;
        Ok(())
    }

    /// Move to a node or branch. Walks the DAG through the common ancestor, *reverting*
    /// deltas going up and *applying* deltas going down; no op code runs when the deltas
    /// are present and verify against the recorded state hash. Any missing, corrupt or
    /// mismatching delta triggers a full op replay from the root instead (and regenerates
    /// the deltas), so ops stay the source of truth.
    pub fn checkout(&mut self, target: &Target) -> Result<(), TimelineError> {
        let (goal, head) = match target {
            Target::Node(h) => {
                self.node(h)?;
                (Some(*h), Head::Detached(*h))
            }
            Target::Branch(b) => {
                let h = self
                    .store
                    .branch(b)?
                    .ok_or_else(|| TimelineError::UnknownRef(b.clone()))?;
                (Some(h), Head::Branch(b.clone()))
            }
        };
        self.goto(goal)?;
        self.store.set_head(&head)?;
        self.head = head;
        Ok(())
    }

    fn goto(&mut self, goal: Option<Hash>) -> Result<(), TimelineError> {
        let goal_path = match goal {
            Some(h) => self.path_to(&h)?,
            None => Vec::new(),
        };
        let cur_path = match self.at {
            Some(h) => self.path_to(&h)?,
            None => Vec::new(),
        };
        if goal_path == cur_path {
            return Ok(());
        }
        let common = goal_path
            .iter()
            .zip(&cur_path)
            .take_while(|(a, b)| a == b)
            .count();
        match self.walk(&cur_path, &goal_path, common) {
            Some(()) => Ok(()),
            None => self.rebuild(&goal_path),
        }
    }

    /// Fast path along deltas. `None` means "fall back to replay"; `self` stays consistent
    /// (state matches `at`) after every step, success or not.
    fn walk(&mut self, cur: &[Hash], goal: &[Hash], common: usize) -> Option<()> {
        for i in (common..cur.len()).rev() {
            let d = self.store.delta(&cur[i]).ok().flatten()?;
            self.state.revert_delta(&d).ok()?;
            self.at = if i == 0 { None } else { Some(cur[i - 1]) };
        }
        let mut expected = None;
        for h in &goal[common..] {
            let d = self.store.delta(h).ok().flatten()?;
            self.state.apply_delta(&d).ok()?;
            self.at = Some(*h);
            expected = Some(d.state_after);
        }
        // One end-to-end integrity check instead of one per step.
        let want = match (expected, goal.last()) {
            (Some(e), _) => e,
            (None, Some(last)) => self.store.delta(last).ok().flatten()?.state_after,
            (None, None) => self.registry.new_state().hash(),
        };
        (self.state.hash() == want).then_some(())
    }

    /// Slow path: replay ops from the empty state, refreshing the deltas.
    fn rebuild(&mut self, goal: &[Hash]) -> Result<(), TimelineError> {
        let mut state = self.registry.new_state();
        for h in goal {
            let node = self.node(h)?;
            let applied = self
                .registry
                .apply(&mut state, &node.op, &ApplyOptions::default())?;
            self.store.put_delta(*h, &applied.delta)?;
        }
        self.state = state;
        self.at = goal.last().copied();
        Ok(())
    }

    /// Replay the path to the current node from the empty state using ops only and return
    /// the resulting state (does not touch the workspace). For verification and tests.
    pub fn replay_from_root(&self) -> Result<State, TimelineError> {
        let mut state = self.registry.new_state();
        if let Some(at) = self.at {
            for h in self.path_to(&at)? {
                let node = self.node(&h)?;
                self.registry
                    .apply(&mut state, &node.op, &ApplyOptions::default())?;
            }
        }
        Ok(state)
    }
}
