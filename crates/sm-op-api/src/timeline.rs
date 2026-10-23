//! The timeline DAG: content-addressed nodes, refs and HEAD, like git.

use std::collections::BTreeMap;

use crate::error::TimelineError;
use crate::hash::Hash256;
use crate::op::OpRecord;
use crate::toml::{from_toml, to_toml};
use crate::value::{CodecError, Value, from_value, to_value};

/// Free-form node metadata. Part of the hash, so callers supply `time` explicitly (the engine
/// never reads a clock).
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Meta {
    /// Who issued the op (user, script, agent).
    pub author: String,
    /// Caller-supplied timestamp (e.g. Unix seconds); `0` when irrelevant.
    pub time: u64,
    /// Optional human message.
    pub message: String,
}

/// One timeline node: `{parent, op, meta, resulting state hash}`.
///
/// The node hash is SHA-256 of its canonical TOML text, so it commits to the whole history
/// through `parent`. Recording the resulting state hash lets replay detect nondeterminism.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Node {
    /// Parent node, `None` for a root (applied to the empty state).
    pub parent: Option<Hash256>,
    /// The op, in stored form.
    pub op: OpRecord,
    /// Metadata.
    pub meta: Meta,
    /// `State::hash()` after applying `op` to the parent's state.
    pub state: Hash256,
}

impl Node {
    /// Canonical TOML text (the file content).
    pub fn to_toml(&self) -> Result<String, CodecError> {
        match to_value(self)? {
            Value::Table(t) => Ok(to_toml(&t)),
            _ => Err(CodecError::new("node is not a table")),
        }
    }

    /// Parse canonical TOML text.
    pub fn from_toml(text: &str) -> Result<Self, CodecError> {
        from_value(&Value::Table(from_toml(text)?))
    }

    /// Content hash of the canonical text.
    pub fn hash(&self) -> Result<Hash256, CodecError> {
        Ok(Hash256::of(self.to_toml()?.as_bytes()))
    }
}

/// Where HEAD points.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Head {
    /// Attached to a branch; new nodes advance it.
    Ref(String),
    /// Detached at a node (or at the empty root state); new nodes do not move any ref.
    Detached(Option<Hash256>),
}

/// Whether `name` is an acceptable ref name (also a safe relative path).
pub fn valid_ref_name(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with('/')
        && !name.ends_with('/')
        && name.split('/').all(|p| !p.is_empty() && p != "." && p != "..")
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-' | b'/'))
}

/// In-memory timeline store (the reference implementation; `fs` persists it).
#[derive(Clone, Debug)]
pub struct Timeline {
    nodes: BTreeMap<Hash256, Node>,
    refs: BTreeMap<String, Hash256>,
    head: Head,
}

impl Default for Timeline {
    fn default() -> Self {
        Self { nodes: BTreeMap::new(), refs: BTreeMap::new(), head: Head::Ref("main".to_owned()) }
    }
}

impl Timeline {
    /// Empty timeline with HEAD on an unborn `main`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a node whose parent (if any) is already present. Idempotent. Returns its hash.
    pub fn insert(&mut self, node: Node) -> Result<Hash256, TimelineError> {
        if let Some(p) = node.parent
            && !self.nodes.contains_key(&p)
        {
            return Err(TimelineError::UnknownParent(p));
        }
        let h = node.hash()?;
        self.nodes.entry(h).or_insert(node);
        Ok(h)
    }

    /// Look up a node.
    pub fn get(&self, h: Hash256) -> Option<&Node> {
        self.nodes.get(&h)
    }

    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// True when there are no nodes.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// All nodes in hash order.
    pub fn nodes(&self) -> impl Iterator<Item = (Hash256, &Node)> {
        self.nodes.iter().map(|(h, n)| (*h, n))
    }

    /// Hashes from the root down to `h` inclusive.
    pub fn path_to_root(&self, h: Hash256) -> Result<Vec<Hash256>, TimelineError> {
        let mut out = Vec::new();
        let mut cur = Some(h);
        while let Some(c) = cur {
            let n = self.nodes.get(&c).ok_or(TimelineError::UnknownNode(c))?;
            out.push(c);
            cur = n.parent;
        }
        out.reverse();
        Ok(out)
    }

    /// Children of a node (or the roots for `None`), in hash order.
    pub fn children(&self, h: Option<Hash256>) -> Vec<Hash256> {
        self.nodes.iter().filter(|(_, n)| n.parent == h).map(|(k, _)| *k).collect()
    }

    /// All refs, sorted by name.
    pub fn refs(&self) -> &BTreeMap<String, Hash256> {
        &self.refs
    }

    /// Create a new ref; errors if it exists.
    pub fn create_ref(&mut self, name: &str, target: Hash256) -> Result<(), TimelineError> {
        if self.refs.contains_key(name) {
            return Err(TimelineError::RefExists(name.to_owned()));
        }
        self.set_ref(name, target)
    }

    /// Create or move a ref.
    pub fn set_ref(&mut self, name: &str, target: Hash256) -> Result<(), TimelineError> {
        if !valid_ref_name(name) {
            return Err(TimelineError::BadRefName(name.to_owned()));
        }
        if !self.nodes.contains_key(&target) {
            return Err(TimelineError::UnknownNode(target));
        }
        self.refs.insert(name.to_owned(), target);
        Ok(())
    }

    /// Where HEAD points.
    pub fn head(&self) -> &Head {
        &self.head
    }

    /// Move HEAD. A `Ref` head must have a valid name; it may be unborn.
    pub fn set_head(&mut self, head: Head) -> Result<(), TimelineError> {
        match &head {
            Head::Ref(n) if !valid_ref_name(n) => return Err(TimelineError::BadRefName(n.clone())),
            Head::Detached(Some(h)) if !self.nodes.contains_key(h) => {
                return Err(TimelineError::UnknownNode(*h));
            }
            _ => {}
        }
        self.head = head;
        Ok(())
    }

    /// The node HEAD currently resolves to (`None` for the empty root state or unborn branch).
    pub fn head_node(&self) -> Option<Hash256> {
        match &self.head {
            Head::Ref(n) => self.refs.get(n).copied(),
            Head::Detached(h) => *h,
        }
    }
}
