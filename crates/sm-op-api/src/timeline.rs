//! The timeline DAG: content-addressed nodes, git-like refs and HEAD, and the on-disk layout.
//!
//! A [`Node`] is `(parent hash, op, metadata)`; its id is the hash of its canonical encoding, so
//! appending never conflicts and branches coexist. This module is pure data: reading and writing
//! files is left to the caller (see [`Timeline::to_files`] / [`Timeline::from_files`]), which
//! passes in the text codec (TOML in `sm-engine`).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::hash::{CanonError, Hash256, content_hash};

/// Current node file format version.
pub const NODE_FORMAT: u32 = 1;

/// Free-form metadata of a node. All fields are part of the node hash. `time` is supplied by the
/// caller (seconds since the Unix epoch, or 0); this crate never reads a clock.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
pub struct Meta {
    /// Who made the change (user, agent or script name).
    pub author: String,
    /// Optional note.
    #[serde(default)]
    pub message: String,
    /// Caller-supplied timestamp.
    #[serde(default)]
    pub time: u64,
}

/// The hashed content of a node. Field order matters for readable TOML: scalars first, tables last.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeBody<O> {
    /// Node file format version ([`NODE_FORMAT`]).
    pub format: u32,
    /// Parent node hash; absent for a root node.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<Hash256>,
    /// Op schema version of the op's layer (`Layer::OP_VERSION` when the node was written).
    pub op_version: u32,
    /// Node metadata.
    pub meta: Meta,
    /// The operation.
    pub op: O,
}

/// A node together with its content hash.
#[derive(Clone, Debug, PartialEq)]
pub struct Node<O> {
    hash: Hash256,
    body: NodeBody<O>,
}

impl<O: Serialize> Node<O> {
    /// Compute the hash of `body` and wrap it.
    pub fn new(body: NodeBody<O>) -> Result<Self, CanonError> {
        let hash = content_hash("sm-op-api/node/v1", &body)?;
        Ok(Self { hash, body })
    }
}

impl<O> Node<O> {
    /// The content hash (the node's id).
    pub fn hash(&self) -> Hash256 {
        self.hash
    }
    /// The hashed content.
    pub fn body(&self) -> &NodeBody<O> {
        &self.body
    }
    /// The parent, if any.
    pub fn parent(&self) -> Option<Hash256> {
        self.body.parent
    }
}

/// Where HEAD points.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Head {
    /// On a branch (which may not exist yet if nothing was committed).
    Branch(String),
    /// Detached at a specific node.
    Detached(Hash256),
}

/// Errors from timeline operations.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TimelineError {
    /// A node names a parent that is not in the timeline.
    #[error("parent {0} not found")]
    MissingParent(Hash256),
    /// No node with this hash.
    #[error("node {0} not found")]
    UnknownNode(Hash256),
    /// A revision string matched nothing.
    #[error("cannot resolve revision `{0}`")]
    UnknownRevision(String),
    /// A hash prefix matched several nodes.
    #[error("revision `{0}` is ambiguous")]
    AmbiguousRevision(String),
    /// A branch name is not allowed (`[A-Za-z0-9._/-]`, no `..`, no leading or trailing `/`).
    #[error("invalid branch name `{0}`")]
    InvalidBranchName(String),
    /// A branch with this name already exists.
    #[error("branch `{0}` already exists")]
    BranchExists(String),
    /// The node format version is not supported.
    #[error("unsupported node format {0}")]
    UnsupportedFormat(u32),
    /// A stored file is inconsistent (hash mismatch, bad ref, undecodable text).
    #[error("corrupt timeline: {0}")]
    Corrupt(String),
    /// Hashing failed.
    #[error(transparent)]
    Canon(#[from] CanonError),
}

/// Whether `name` is a valid branch name.
pub fn valid_branch_name(name: &str) -> bool {
    !name.is_empty()
        && !name.contains("..")
        && !name.starts_with('/')
        && !name.ends_with('/')
        && !name.contains("//")
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-' | b'/'))
}

/// Path of a node file relative to the project root: `timeline/ops/<hash>.toml`.
pub fn node_path(hash: &Hash256) -> String {
    format!("timeline/ops/{hash}.toml")
}

/// Path of a branch ref file: `timeline/refs/heads/<name>`.
pub fn ref_path(name: &str) -> String {
    format!("timeline/refs/heads/{name}")
}

/// Path of the HEAD file.
pub const HEAD_PATH: &str = "timeline/HEAD";

/// In-memory timeline: nodes, branch refs and HEAD. Nodes are immutable and never removed.
#[derive(Clone, Debug)]
pub struct Timeline<O> {
    nodes: BTreeMap<Hash256, Node<O>>,
    depth: BTreeMap<Hash256, u64>,
    refs: BTreeMap<String, Hash256>,
    head: Head,
}

impl<O> Default for Timeline<O> {
    fn default() -> Self {
        Self {
            nodes: BTreeMap::new(),
            depth: BTreeMap::new(),
            refs: BTreeMap::new(),
            head: Head::Branch("main".into()),
        }
    }
}

impl<O: Serialize> Timeline<O> {
    /// An empty timeline with HEAD on the unborn branch `main`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a node (idempotent if it already exists). The parent must already be present.
    /// Refs and HEAD are not moved. Returns the node hash.
    pub fn insert(&mut self, body: NodeBody<O>) -> Result<Hash256, TimelineError> {
        if body.format != NODE_FORMAT {
            return Err(TimelineError::UnsupportedFormat(body.format));
        }
        let depth = match body.parent {
            Some(p) => self.depth.get(&p).ok_or(TimelineError::MissingParent(p))? + 1,
            None => 1,
        };
        let node = Node::new(body)?;
        let hash = node.hash();
        self.depth.insert(hash, depth);
        self.nodes.entry(hash).or_insert(node);
        Ok(hash)
    }
}

impl<O> Timeline<O> {
    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }
    /// Whether there are no nodes.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }
    /// Look up a node.
    pub fn node(&self, hash: &Hash256) -> Option<&Node<O>> {
        self.nodes.get(hash)
    }
    /// Distance from the root, counting the node itself (a root has depth 1).
    pub fn depth(&self, hash: &Hash256) -> Option<u64> {
        self.depth.get(hash).copied()
    }
    /// All node hashes in hash order.
    pub fn hashes(&self) -> impl Iterator<Item = &Hash256> {
        self.nodes.keys()
    }
    /// Children of `hash` in hash order.
    pub fn children(&self, hash: &Hash256) -> Vec<Hash256> {
        self.nodes
            .values()
            .filter(|n| n.parent().as_ref() == Some(hash))
            .map(Node::hash)
            .collect()
    }
    /// The chain root..=`hash`, root first.
    pub fn path_to(&self, hash: &Hash256) -> Result<Vec<Hash256>, TimelineError> {
        let mut out = Vec::new();
        let mut cur = Some(*hash);
        while let Some(h) = cur {
            let n = self.nodes.get(&h).ok_or(TimelineError::UnknownNode(h))?;
            out.push(h);
            cur = n.parent();
        }
        out.reverse();
        Ok(out)
    }

    /// Branch refs in name order.
    pub fn branches(&self) -> impl Iterator<Item = (&str, &Hash256)> {
        self.refs.iter().map(|(k, v)| (k.as_str(), v))
    }
    /// The tip of a branch.
    pub fn branch(&self, name: &str) -> Option<Hash256> {
        self.refs.get(name).copied()
    }
    /// Create or move a branch to `at`.
    pub fn set_branch(&mut self, name: &str, at: Hash256) -> Result<(), TimelineError> {
        if !valid_branch_name(name) {
            return Err(TimelineError::InvalidBranchName(name.into()));
        }
        if !self.nodes.contains_key(&at) {
            return Err(TimelineError::UnknownNode(at));
        }
        self.refs.insert(name.into(), at);
        Ok(())
    }
    /// Create a branch at `at`; fails if it exists.
    pub fn create_branch(&mut self, name: &str, at: Hash256) -> Result<(), TimelineError> {
        if self.refs.contains_key(name) {
            return Err(TimelineError::BranchExists(name.into()));
        }
        self.set_branch(name, at)
    }
    /// Where HEAD points.
    pub fn head(&self) -> &Head {
        &self.head
    }
    /// Point HEAD at a branch (possibly unborn) or a node.
    pub fn set_head(&mut self, head: Head) -> Result<(), TimelineError> {
        match &head {
            Head::Branch(n) if !valid_branch_name(n) => {
                return Err(TimelineError::InvalidBranchName(n.clone()));
            }
            Head::Detached(h) if !self.nodes.contains_key(h) => {
                return Err(TimelineError::UnknownNode(*h));
            }
            _ => {}
        }
        self.head = head;
        Ok(())
    }
    /// The node HEAD currently resolves to (`None` on an unborn branch).
    pub fn head_commit(&self) -> Option<Hash256> {
        match &self.head {
            Head::Branch(n) => self.refs.get(n).copied(),
            Head::Detached(h) => Some(*h),
        }
    }

    /// Resolve `HEAD`, a branch name, a full hash or a unique hash prefix (at least 4 hex digits).
    pub fn resolve(&self, rev: &str) -> Result<Hash256, TimelineError> {
        let unknown = || TimelineError::UnknownRevision(rev.into());
        if rev == "HEAD" {
            return self.head_commit().ok_or_else(unknown);
        }
        if let Some(h) = self.refs.get(rev) {
            return Ok(*h);
        }
        if let Ok(h) = rev.parse::<Hash256>() {
            return if self.nodes.contains_key(&h) {
                Ok(h)
            } else {
                Err(unknown())
            };
        }
        if rev.len() >= 4
            && rev
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            let mut hits = self.nodes.keys().filter(|h| h.to_hex().starts_with(rev));
            return match (hits.next(), hits.next()) {
                (Some(h), None) => Ok(*h),
                (Some(_), Some(_)) => Err(TimelineError::AmbiguousRevision(rev.into())),
                _ => Err(unknown()),
            };
        }
        Err(unknown())
    }

    /// Render HEAD as file text.
    pub fn head_text(&self) -> String {
        match &self.head {
            Head::Branch(n) => format!("ref: refs/heads/{n}\n"),
            Head::Detached(h) => format!("{h}\n"),
        }
    }
}

impl<O: Serialize> Timeline<O> {
    /// Render the whole timeline as `(relative path, text)` files in a deterministic order:
    /// one `timeline/ops/<hash>.toml` per node (text produced by `encode`), one file per branch,
    /// and `timeline/HEAD`.
    pub fn to_files(
        &self,
        encode: impl Fn(&NodeBody<O>) -> Result<String, String>,
    ) -> Result<Vec<(String, String)>, TimelineError> {
        let mut files = Vec::new();
        for (h, n) in &self.nodes {
            let text = encode(n.body()).map_err(TimelineError::Corrupt)?;
            files.push((node_path(h), text));
        }
        for (name, h) in &self.refs {
            files.push((ref_path(name), format!("{h}\n")));
        }
        files.push((HEAD_PATH.to_string(), self.head_text()));
        Ok(files)
    }

    /// Rebuild a timeline from files produced by [`Timeline::to_files`] (any order). Node files
    /// are decoded with `decode`, and every node's recomputed hash must match its file name.
    pub fn from_files(
        files: impl IntoIterator<Item = (String, String)>,
        decode: impl Fn(&str) -> Result<NodeBody<O>, String>,
    ) -> Result<Self, TimelineError> {
        let mut pending: Vec<(Hash256, NodeBody<O>)> = Vec::new();
        let mut refs = Vec::new();
        let mut head_text = None;
        for (path, text) in files {
            if let Some(rest) = path.strip_prefix("timeline/ops/") {
                let stem = rest
                    .strip_suffix(".toml")
                    .ok_or_else(|| TimelineError::Corrupt(format!("bad node file name {path}")))?;
                let want: Hash256 = stem
                    .parse()
                    .map_err(|_| TimelineError::Corrupt(format!("bad node file name {path}")))?;
                pending.push((want, decode(&text).map_err(TimelineError::Corrupt)?));
            } else if let Some(name) = path.strip_prefix("timeline/refs/heads/") {
                let h: Hash256 = text
                    .trim()
                    .parse()
                    .map_err(|_| TimelineError::Corrupt(format!("bad ref {path}")))?;
                refs.push((name.to_string(), h));
            } else if path == HEAD_PATH {
                head_text = Some(text);
            }
        }
        // Insert parents before children; each pass must make progress.
        let mut tl = Self::new();
        while !pending.is_empty() {
            let before = pending.len();
            let mut rest = Vec::new();
            for (want, body) in pending {
                let ready = body.parent.is_none_or(|p| tl.nodes.contains_key(&p));
                if !ready {
                    rest.push((want, body));
                    continue;
                }
                let got = tl.insert(body)?;
                if got != want {
                    return Err(TimelineError::Corrupt(format!(
                        "node {want} hashes to {got}"
                    )));
                }
            }
            if rest.len() == before {
                return Err(TimelineError::Corrupt("node with missing parent".into()));
            }
            pending = rest;
        }
        for (name, h) in refs {
            tl.set_branch(&name, h)?;
        }
        if let Some(t) = head_text {
            let t = t.trim();
            let head = match t.strip_prefix("ref: refs/heads/") {
                Some(n) => Head::Branch(n.to_string()),
                None => Head::Detached(
                    t.parse()
                        .map_err(|_| TimelineError::Corrupt("bad HEAD".into()))?,
                ),
            };
            tl.set_head(head)?;
        }
        Ok(tl)
    }
}
