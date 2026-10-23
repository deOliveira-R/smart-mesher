//! The timeline DAG: content-addressed nodes and branch refs, like git. Pure data: the exact
//! on-disk layout is exposed as a path-to-text map ([`Timeline::to_files`]); reading and writing
//! files belongs to `sm-engine`.
//!
//! `HEAD` is deliberately not part of the timeline: it is per-checkout state (see [`Head`]) and
//! lives in the gitignored `.cache/` so concurrent branches never conflict on it.

use std::collections::{BTreeMap, VecDeque};

use serde::{Deserialize, Serialize};

use crate::codec::{CodecError, canonicalize, canonical_bytes};
use crate::error::TimelineError;
use crate::hash::Hash256;
use crate::op::OpRecord;

/// Domain tag of node hashes. Bump only with a format version.
pub const NODE_DOMAIN: &str = "smart-mesh/node/v1";

/// Free-form node metadata. Part of the node hash, so callers supply `time` explicitly (the
/// engine never reads a clock).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Meta {
    /// Who issued the op (user, script, agent).
    pub author: String,
    /// Caller-supplied timestamp text (e.g. RFC 3339); empty when irrelevant.
    pub time: String,
    /// Optional human message.
    pub message: String,
}

/// The hashed part of a node: where it hangs, what it does, who did it.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeBody {
    /// Parent node, `None` for a root (applied to the empty state).
    pub parent: Option<Hash256>,
    /// The op, in stored form.
    pub op: OpRecord,
    /// Metadata.
    pub meta: Meta,
}

impl NodeBody {
    /// The node's identity: `SHA-256(domain ‖ canonical_bytes(body))`. It commits to the whole
    /// history through `parent` and does not depend on text formatting or on the op's result.
    pub fn hash(&self) -> Result<Hash256, CodecError> {
        use serde_json::json;
        // Hash the canonical form so a hand-built record hashes like the stored one.
        let mut params = self.op.params.clone();
        canonicalize(&mut params)?;
        let mut body = json!({
            "op": { "name": self.op.name, "version": self.op.version, "params": params },
            "meta": { "author": self.meta.author, "time": self.meta.time, "message": self.meta.message },
        });
        if let (Some(p), Some(m)) = (self.parent, body.as_object_mut()) {
            m.insert("parent".to_owned(), json!(p.to_hex()));
        }
        Ok(Hash256::of_domain(NODE_DOMAIN, &canonical_bytes(&body)))
    }
}

/// The recorded result of a node: not hashed, so the same op on another platform yields the
/// same node even if float bits differ; replay compares it and reports `Diverged` on mismatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Check {
    /// `State::hash()` (quantized, platform independent) after applying the op.
    pub state: Hash256,
}

/// One timeline node: hashed [`NodeBody`] plus the unhashed result [`Check`].
#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    /// The hashed part.
    pub body: NodeBody,
    /// The recorded result hash.
    pub check: Check,
}

/// On-disk shape of a node file (`ops/<hash>.toml`), read with the `toml` crate.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct NodeFile {
    #[serde(default)]
    parent: Option<Hash256>,
    op: OpRecord,
    meta: Meta,
    check: Check,
}

impl From<NodeFile> for Node {
    fn from(f: NodeFile) -> Self {
        Self { body: NodeBody { parent: f.parent, op: f.op, meta: f.meta }, check: f.check }
    }
}

impl Node {
    /// Content hash of the body (the node id).
    pub fn hash(&self) -> Result<Hash256, CodecError> {
        self.body.hash()
    }
}

/// Where HEAD points.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Head {
    /// Attached to a branch; new nodes advance it. The branch may be unborn.
    Ref(String),
    /// Detached at a node (or at the empty root state); new nodes do not move any ref.
    Detached(Option<Hash256>),
}

/// Whether `name` is an acceptable ref name (also a safe relative path).
pub fn valid_ref_name(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with('/')
        && !name.ends_with('/')
        && name.split('/').all(|p| !p.is_empty() && p != "." && p != ".." && !p.ends_with(".lock"))
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-' | b'/'))
}

#[derive(Clone, Debug)]
struct Entry {
    node: Node,
    depth: usize,
}

/// In-memory timeline store: nodes, a children index, and branch refs.
#[derive(Clone, Debug, Default)]
pub struct Timeline {
    nodes: BTreeMap<Hash256, Entry>,
    children: BTreeMap<Hash256, Vec<Hash256>>,
    roots: Vec<Hash256>,
    refs: BTreeMap<String, Hash256>,
}

const OPS_DIR: &str = "ops/";
const HEADS_DIR: &str = "refs/heads/";

impl Timeline {
    /// An empty timeline.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert a node whose parent (if any) is already present. Returns its hash. Idempotent for
    /// an identical node; a node with the same body but a different recorded result is a
    /// [`Diverged`](TimelineError::Diverged) error (the op is not deterministic).
    pub fn insert(&mut self, mut node: Node) -> Result<Hash256, TimelineError> {
        let depth = match node.body.parent {
            Some(p) => self.nodes.get(&p).ok_or(TimelineError::UnknownParent(p))?.depth + 1,
            None => 1,
        };
        canonicalize(&mut node.body.op.params)?;
        let h = node.body.hash()?;
        if let Some(old) = self.nodes.get(&h) {
            if old.node.check != node.check {
                return Err(TimelineError::Diverged { node: h, recorded: old.node.check.state, actual: node.check.state });
            }
            return Ok(h);
        }
        let list = match node.body.parent {
            Some(p) => self.children.entry(p).or_default(),
            None => &mut self.roots,
        };
        let at = list.binary_search(&h).unwrap_or_else(|i| i);
        list.insert(at, h);
        self.nodes.insert(h, Entry { node, depth });
        Ok(h)
    }

    /// Look up a node.
    pub fn get(&self, h: Hash256) -> Option<&Node> {
        self.nodes.get(&h).map(|e| &e.node)
    }

    /// Number of ancestors including itself (a root has depth 1).
    pub fn depth(&self, h: Hash256) -> Option<usize> {
        self.nodes.get(&h).map(|e| e.depth)
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
        self.nodes.iter().map(|(h, e)| (*h, &e.node))
    }

    /// Hashes from the root down to `h` inclusive.
    pub fn path_to_root(&self, h: Hash256) -> Result<Vec<Hash256>, TimelineError> {
        let mut out = Vec::new();
        let mut cur = Some(h);
        while let Some(c) = cur {
            let e = self.nodes.get(&c).ok_or(TimelineError::UnknownNode(c))?;
            out.push(c);
            cur = e.node.body.parent;
        }
        out.reverse();
        Ok(out)
    }

    /// Children of a node, in hash order.
    pub fn children(&self, h: Hash256) -> &[Hash256] {
        self.children.get(&h).map_or(&[], Vec::as_slice)
    }

    /// Root nodes (no parent), in hash order.
    pub fn roots(&self) -> &[Hash256] {
        &self.roots
    }

    /// All branch refs, sorted by name.
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

    /// Create or move a ref. Names must be valid and must not collide with another ref as a
    /// file versus a directory (`a` and `a/b`).
    pub fn set_ref(&mut self, name: &str, target: Hash256) -> Result<(), TimelineError> {
        self.check_ref_name(name)?;
        if !self.nodes.contains_key(&target) {
            return Err(TimelineError::UnknownNode(target));
        }
        self.refs.insert(name.to_owned(), target);
        Ok(())
    }

    /// Whether a ref of this name could be created or moved: valid, and not colliding with
    /// another ref as a file versus a directory (`a` and `a/b`).
    pub fn check_ref_name(&self, name: &str) -> Result<(), TimelineError> {
        if !valid_ref_name(name) {
            return Err(TimelineError::BadRefName(name.to_owned()));
        }
        if !self.refs.contains_key(name) {
            let as_dir = format!("{name}/");
            if self.refs.keys().any(|r| r.starts_with(&as_dir) || name.starts_with(&format!("{r}/"))) {
                return Err(TimelineError::BadRefName(name.to_owned()));
            }
        }
        Ok(())
    }

    /// Delete a ref, returning where it pointed.
    pub fn delete_ref(&mut self, name: &str) -> Result<Hash256, TimelineError> {
        self.refs.remove(name).ok_or_else(|| TimelineError::UnknownRef(name.to_owned()))
    }

    /// The HEAD to use when none was recorded: `main` if it exists, else the first branch by
    /// name, else an unborn `main`. Never a silent empty detached state.
    pub fn default_head(&self) -> Head {
        if self.refs.contains_key("main") {
            return Head::Ref("main".to_owned());
        }
        match self.refs.keys().next() {
            Some(n) => Head::Ref(n.clone()),
            None => Head::Ref("main".to_owned()),
        }
    }

    /// The committed layout as path to text: `ops/<hash>.toml` per node and `refs/heads/<name>`
    /// per branch. There is no HEAD file. Node files are content-addressed and immutable, and
    /// each branch owns exactly one ref file, so independent branches touch disjoint files.
    pub fn to_files(&self) -> Result<BTreeMap<String, String>, TimelineError> {
        let mut out = BTreeMap::new();
        for (h, e) in &self.nodes {
            let text = crate::text::node_to_toml(&e.node).map_err(|e| TimelineError::Format(format!("encode node {h}: {e}")))?;
            out.insert(format!("{OPS_DIR}{h}.toml"), text);
        }
        for (name, h) in &self.refs {
            out.insert(format!("{HEADS_DIR}{name}"), format!("{h}\n"));
        }
        Ok(out)
    }

    /// Parse the layout written by [`to_files`](Self::to_files). Every node's hash is
    /// recomputed from its parsed content and must equal its file name; parents must exist;
    /// refs must point at known nodes. Files outside `ops/` and `refs/heads/` are ignored.
    pub fn from_files(files: &BTreeMap<String, String>) -> Result<Self, TimelineError> {
        let mut parsed: BTreeMap<Hash256, Node> = BTreeMap::new();
        for (path, text) in files {
            let Some(file) = path.strip_prefix(OPS_DIR) else { continue };
            let expected: Hash256 = file
                .strip_suffix(".toml")
                .and_then(|s| s.parse().ok())
                .ok_or_else(|| TimelineError::Format(format!("bad node file name '{path}'")))?;
            let node: Node = toml::from_str::<NodeFile>(text).map_err(|e| TimelineError::Format(format!("{path}: {e}")))?.into();
            let mut canon = node.clone();
            canonicalize(&mut canon.body.op.params)?;
            let actual = canon.body.hash()?;
            if actual != expected {
                return Err(TimelineError::HashMismatch { expected, actual });
            }
            parsed.insert(expected, canon);
        }
        // Insert parents before children, in a deterministic order.
        let mut waiting: BTreeMap<Hash256, Vec<Hash256>> = BTreeMap::new();
        let mut ready: VecDeque<Hash256> = VecDeque::new();
        for (h, n) in &parsed {
            match n.body.parent {
                Some(p) if parsed.contains_key(&p) => waiting.entry(p).or_default().push(*h),
                Some(p) => return Err(TimelineError::UnknownParent(p)),
                None => ready.push_back(*h),
            }
        }
        let mut t = Timeline::new();
        while let Some(h) = ready.pop_front() {
            if let Some(n) = parsed.remove(&h) {
                t.insert(n)?;
            }
            if let Some(kids) = waiting.remove(&h) {
                ready.extend(kids);
            }
        }
        if let Some((h, _)) = parsed.iter().next() {
            return Err(TimelineError::UnknownNode(*h)); // unreachable from any root: a cycle
        }
        for (path, text) in files {
            let Some(name) = path.strip_prefix(HEADS_DIR) else { continue };
            let h: Hash256 = text
                .trim()
                .parse()
                .map_err(|_| TimelineError::Format(format!("bad ref file '{path}'")))?;
            t.set_ref(name, h)?;
        }
        Ok(t)
    }
}
