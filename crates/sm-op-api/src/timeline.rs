//! The timeline DAG: content-addressed nodes and branch refs, like git. Pure data: the exact
//! on-disk layout is exposed as a path-to-text map ([`Timeline::to_files`]); reading and writing
//! files belongs to `sm-engine`.
//!
//! `HEAD` is deliberately not part of the timeline: it is per-checkout state (see [`Head`]) and
//! lives in the gitignored `.cache/` so concurrent branches never conflict on it.

use std::collections::{BTreeMap, VecDeque};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::codec::{CodecError, canonical_bytes, canonicalize};
use crate::error::TimelineError;
use crate::hash::Hash256;
use crate::op::{OpRecord, TypedOp};

/// Domain tag of node hashes. Bump only with a format version.
pub const NODE_DOMAIN: &str = "smart-mesh/node/v1";

/// Free-form node metadata. Part of the node hash, so callers supply `time` explicitly (the
/// engine never reads a clock). It may gain fields: build it with [`Meta::new`]. A field added
/// later enters [`NodeBody::hash`] only when it differs from its default, so existing node
/// hashes stay stable (see DESIGN.md).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub struct Meta {
    /// Who issued the op (user, script, agent).
    pub author: String,
    /// Caller-supplied timestamp text (e.g. RFC 3339); empty when irrelevant.
    pub time: String,
    /// Optional human message.
    pub message: String,
}

impl Meta {
    /// Metadata from author, caller-supplied time text and message.
    pub fn new(author: impl Into<String>, time: impl Into<String>, message: impl Into<String>) -> Self {
        Self { author: author.into(), time: time.into(), message: message.into() }
    }
}

/// The hashed part of a node: where it hangs, what it does, who did it. Serializes with the
/// same field names as the node file (`parent` omitted for a root).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct NodeBody {
    /// Parent node, `None` for a root (applied to the empty state).
    #[serde(default, skip_serializing_if = "Option::is_none")]
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
        // Freeze rule (DESIGN.md): a `Meta` field added later is inserted into "meta" only when
        // it differs from its default, so existing node hashes stay stable.
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
/// It may gain fields: build it with [`Check::new`], [`Check::of`] or [`Check::of_with`].
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub struct Check {
    /// `State::hash()` (quantized, platform independent) after applying the op.
    pub state: Hash256,
    /// Quantized hash of every non-empty authoritative layer after the op
    /// (`State::layer_hashes`), so a divergence names the layers that differ. Not part of the
    /// node hash; may be empty (then a divergence reports no layer names).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub layers: BTreeMap<String, Hash256>,
    /// Implementation provenance: `layer:<name>` to the layer's `LayerType::DIGEST_VERSION`
    /// (for every layer in [`layers`](Self::layers)) and `op:<name>` to the op's
    /// `TypedOp::IMPL_VERSION` (the node's op and, for `core.batch`, every nested op), plus
    /// `service:<name>` for each versioned injected service ([`Check::of_with`]). Not
    /// part of the node hash. When replay mismatches and one of these differs from the current
    /// registry, the error is [`Upgraded`](TimelineError::Upgraded) rather than `Diverged`.
    /// May be empty (older nodes, hand-built checks).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub versions: BTreeMap<String, u32>,
}

impl Check {
    /// A check carrying only the state hash (no per-layer hashes, no versions).
    pub fn new(state: Hash256) -> Self {
        Self { state, ..Self::default() }
    }
    /// The check of a state after applying `op`: its hash, its per-layer hashes and the
    /// implementation versions the engine's registry reports for those layers and for `op`
    /// (and the ops nested in a `core.batch`). Names the registry does not know are skipped.
    pub fn of(registry: &crate::Registry, state: &crate::state::State, op: &OpRecord) -> Self {
        let layers = state.layer_hashes();
        let mut versions = BTreeMap::new();
        for name in layers.keys() {
            if let Some(v) = registry.layer_digest_version(name) {
                versions.insert(format!("layer:{name}"), v);
            }
        }
        let mut ops = vec![op.name.as_str()];
        if op.name == crate::batch::Batch::NAME
            && let Some(list) = op.params.get("ops").and_then(|v| v.as_array())
        {
            ops.extend(
                list.iter()
                    .filter_map(|r| r.get("name").and_then(|n| n.as_str())),
            );
        }
        for name in ops {
            if let Some(v) = registry.op_impl_version(name) {
                versions.insert(format!("op:{name}"), v);
            }
        }
        Self {
            state: state.hash(),
            layers,
            versions,
        }
    }

    /// Like [`of`](Self::of), also recording every versioned service of `services`
    /// (`Services::with_versioned`) as `service:<name>` -> version. This is what `Session`
    /// records, so a replay mismatch after an injected service changed version reports
    /// [`Upgraded`](TimelineError::Upgraded). Nodes recorded without these keys are unaffected
    /// (`changed_versions` ignores keys present on one side only).
    pub fn of_with(
        registry: &crate::Registry,
        services: &crate::Services,
        state: &crate::state::State,
        op: &OpRecord,
    ) -> Self {
        let mut c = Self::of(registry, state, op);
        for (name, v) in services.versions() {
            c.versions.insert(format!("service:{name}"), v);
        }
        c
    }

    /// Keys whose recorded version (in `self`) differs from the version in `current`, sorted.
    /// Only keys present on both sides count: a key on one side only means the content
    /// differs (a layer became non-empty), not that an implementation changed.
    pub fn changed_versions(&self, current: &Check) -> Vec<String> {
        self.versions
            .iter()
            .filter(|(k, v)| current.versions.get(*k).is_some_and(|c| c != *v))
            .map(|(k, _)| k.clone())
            .collect()
    }

    /// The error for a replay whose result `actual` does not match this recorded check of
    /// node `node`: [`Upgraded`](TimelineError::Upgraded) when a recorded implementation
    /// version differs from the current one, else [`Diverged`](TimelineError::Diverged).
    pub fn mismatch(&self, node: Hash256, actual: &Check) -> TimelineError {
        let changed = self.changed_versions(actual);
        if changed.is_empty() {
            TimelineError::Diverged {
                node,
                recorded: self.state,
                actual: actual.state,
                layers: self.differing_layers(actual),
            }
        } else {
            TimelineError::Upgraded { node, changed }
        }
    }

    /// Names of layers whose hashes differ between `self` (recorded) and `actual`, sorted.
    /// Empty when either side carries no layer hashes.
    pub fn differing_layers(&self, actual: &Check) -> Vec<String> {
        if self.layers.is_empty() || actual.layers.is_empty() {
            return Vec::new();
        }
        let mut out: Vec<String> = self
            .layers
            .iter()
            .filter(|(n, h)| actual.layers.get(*n) != Some(h))
            .map(|(n, _)| n.clone())
            .collect();
        out.extend(
            actual
                .layers
                .keys()
                .filter(|n| !self.layers.contains_key(*n))
                .cloned(),
        );
        out.sort();
        out
    }
}

/// One timeline node: hashed [`NodeBody`] plus the unhashed result [`Check`]. Serializes with
/// the same shape as the node file: `parent`, `op`, `meta`, `check`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Node {
    /// The hashed part.
    #[serde(flatten)]
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
        Self {
            body: NodeBody {
                parent: f.parent,
                op: f.op,
                meta: f.meta,
            },
            check: f.check,
        }
    }
}

impl Node {
    /// Content hash of the body (the node id).
    pub fn hash(&self) -> Result<Hash256, CodecError> {
        self.body.hash()
    }
}

/// Where HEAD points. Serializes like [`Rev`](crate::Rev): `{"ref": "main"}`,
/// `{"node": "<hash>"}` (detached) or `"empty"` (detached at the empty state).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(from = "crate::session::Rev", into = "crate::session::Rev")]
pub enum Head {
    /// Attached to a branch; new nodes advance it. The branch may be unborn.
    Ref(String),
    /// Detached at a node (or at the empty root state); new nodes do not move any ref.
    Detached(Option<Hash256>),
}

impl From<crate::session::Rev> for Head {
    fn from(r: crate::session::Rev) -> Self {
        use crate::session::Rev;
        match r {
            Rev::Empty => Head::Detached(None),
            Rev::Node(h) => Head::Detached(Some(h)),
            Rev::Ref(n) => Head::Ref(n),
        }
    }
}

impl From<Head> for crate::session::Rev {
    fn from(h: Head) -> Self {
        use crate::session::Rev;
        match h {
            Head::Detached(None) => Rev::Empty,
            Head::Detached(Some(h)) => Rev::Node(h),
            Head::Ref(n) => Rev::Ref(n),
        }
    }
}

impl JsonSchema for Head {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "Head".into()
    }
    fn json_schema(g: &mut schemars::SchemaGenerator) -> schemars::Schema {
        <crate::session::Rev as JsonSchema>::json_schema(g)
    }
}

/// Whether `name` is an acceptable ref name (also a safe, portable relative path).
///
/// Segments are separated by `/` and use only ASCII letters, digits, `.`, `_` and `-`. A segment
/// must not be empty, `.` or `..`, end in `.lock` or `.`, or have a stem (the part before its
/// first `.`) that is a Windows reserved device name, case-insensitively: `CON`, `PRN`, `AUX`,
/// `NUL`, `COM1`-`COM9`, `LPT1`-`LPT9`. Mixed case is allowed; case-insensitive collisions
/// between refs are rejected by [`Timeline::check_ref_name`].
pub fn valid_ref_name(name: &str) -> bool {
    !name.is_empty()
        && !name.starts_with('/')
        && !name.ends_with('/')
        && name.split('/').all(valid_segment)
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-' | b'/'))
}

fn valid_segment(p: &str) -> bool {
    if p.is_empty() || p == "." || p == ".." || p.ends_with(".lock") || p.ends_with('.') {
        return false;
    }
    let stem = p.split('.').next().unwrap_or(p).to_ascii_uppercase();
    let reserved = matches!(stem.as_str(), "CON" | "PRN" | "AUX" | "NUL")
        || (stem.len() == 4
            && (stem.starts_with("COM") || stem.starts_with("LPT"))
            && matches!(stem.as_bytes()[3], b'1'..=b'9'));
    !reserved
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
            Some(p) => {
                self.nodes
                    .get(&p)
                    .ok_or(TimelineError::UnknownParent(p))?
                    .depth
                    + 1
            }
            None => 1,
        };
        canonicalize(&mut node.body.op.params)?;
        let h = node.body.hash()?;
        if let Some(old) = self.nodes.get(&h) {
            if old.node.check.state != node.check.state {
                return Err(TimelineError::Diverged {
                    node: h,
                    recorded: old.node.check.state,
                    actual: node.check.state,
                    layers: old.node.check.differing_layers(&node.check),
                });
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

    /// Replace the recorded result of node `h` (re-baselining after an implementation
    /// upgrade). The body, and so the node hash, file name and every descendant, stay
    /// unchanged; only the node file's `[check]` text changes, so a storage backend must
    /// rewrite that one file ([`node_file`](Self::node_file)).
    pub fn replace_check(&mut self, h: Hash256, check: Check) -> Result<(), TimelineError> {
        let e = self.nodes.get_mut(&h).ok_or(TimelineError::UnknownNode(h))?;
        e.node.check = check;
        Ok(())
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

    /// Whether a ref of this name could be created or moved: valid ([`valid_ref_name`]), and,
    /// for a new name, not colliding with an existing ref on a case-insensitive file system:
    /// its ASCII-lowercased form must differ from every existing ref's lowercased form (`try`
    /// and `Try`), and must not clash with one as a file versus a directory after lowercasing
    /// (`a` and `a/b`, `Feat` and `feat/one`).
    pub fn check_ref_name(&self, name: &str) -> Result<(), TimelineError> {
        if !valid_ref_name(name) {
            return Err(TimelineError::BadRefName(name.to_owned()));
        }
        if !self.refs.contains_key(name) {
            let lower = name.to_ascii_lowercase();
            let nested = |long: &str, short: &str| {
                long.len() > short.len()
                    && long.starts_with(short)
                    && long.as_bytes()[short.len()] == b'/'
            };
            let clash = self.refs.keys().any(|r| {
                let r = r.to_ascii_lowercase();
                r == lower || nested(&r, &lower) || nested(&lower, &r)
            });
            if clash {
                return Err(TimelineError::BadRefName(name.to_owned()));
            }
        }
        Ok(())
    }

    /// Delete a ref, returning where it pointed.
    pub fn delete_ref(&mut self, name: &str) -> Result<Hash256, TimelineError> {
        self.refs
            .remove(name)
            .ok_or_else(|| TimelineError::UnknownRef(name.to_owned()))
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
    /// per branch. There is no HEAD file. Node files are content-addressed (immutable but for `replace_check`), and
    /// each branch owns exactly one ref file, so independent branches touch disjoint files.
    pub fn to_files(&self) -> Result<BTreeMap<String, String>, TimelineError> {
        let mut out = BTreeMap::new();
        for h in self.nodes.keys() {
            let (path, text) = self.node_file(*h)?;
            out.insert(path, text);
        }
        for name in self.refs.keys() {
            if let Some((path, text)) = self.ref_file(name) {
                out.insert(path, text);
            }
        }
        Ok(out)
    }

    /// Path and text of one node file, exactly as [`to_files`](Self::to_files) writes it (for
    /// incremental saves: node files are immutable except after `replace_check`, so only new or re-baselined ones need writing).
    pub fn node_file(&self, h: Hash256) -> Result<(String, String), TimelineError> {
        let e = self.nodes.get(&h).ok_or(TimelineError::UnknownNode(h))?;
        let text = crate::text::node_to_toml(&e.node)
            .map_err(|e| TimelineError::Format(format!("encode node {h}: {e}")))?;
        Ok((format!("{OPS_DIR}{h}.toml"), text))
    }

    /// Path and text of one ref file, exactly as [`to_files`](Self::to_files) writes it, or
    /// `None` if no such branch exists.
    pub fn ref_file(&self, name: &str) -> Option<(String, String)> {
        self.refs
            .get(name)
            .map(|h| (format!("{HEADS_DIR}{name}"), format!("{h}\n")))
    }

    /// Parse the layout written by [`to_files`](Self::to_files). Every node's hash is
    /// recomputed from its parsed content and must equal its file name; parents must exist;
    /// refs must point at known nodes. Files outside `ops/` and `refs/heads/` are ignored.
    pub fn from_files(files: &BTreeMap<String, String>) -> Result<Self, TimelineError> {
        let mut parsed: BTreeMap<Hash256, Node> = BTreeMap::new();
        for (path, text) in files {
            let Some(file) = path.strip_prefix(OPS_DIR) else {
                continue;
            };
            let expected: Hash256 = file
                .strip_suffix(".toml")
                .and_then(|s| s.parse().ok())
                .ok_or_else(|| TimelineError::Format(format!("bad node file name '{path}'")))?;
            let node: Node = toml::from_str::<NodeFile>(text)
                .map_err(|e| TimelineError::Format(format!("{path}: {e}")))?
                .into();
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
            let Some(name) = path.strip_prefix(HEADS_DIR) else {
                continue;
            };
            let h: Hash256 = text
                .trim()
                .parse()
                .map_err(|_| TimelineError::Format(format!("bad ref file '{path}'")))?;
            t.set_ref(name, h)?;
        }
        Ok(t)
    }
}
