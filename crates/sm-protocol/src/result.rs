use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sm_op_api::{Delta, Hash256, Head, OpInfoOwned, QueryInfoOwned};

/// Result of `apply`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ApplyResult {
    /// The new timeline node.
    pub node: Hash256,
    /// State hash after the op.
    pub state: Hash256,
    /// What the op changed.
    pub delta: Delta,
}

impl ApplyResult {
    /// Build a result.
    pub fn new(node: Hash256, state: Hash256, delta: Delta) -> Self {
        Self { node, state, delta }
    }
}

/// Result of `session.hash`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct HashResult {
    /// Quantized hash of the whole state.
    pub state: Hash256,
    /// Per-layer digests by layer name.
    pub layers: BTreeMap<String, Hash256>,
}

impl HashResult {
    /// Build a result.
    pub fn new(state: Hash256, layers: BTreeMap<String, Hash256>) -> Self {
        Self { state, layers }
    }
}

/// One file of a multi-file export (for example an OpenFOAM `polyMesh` directory).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ExportedFile {
    /// File name relative to the export path.
    pub name: String,
    /// Size in bytes.
    pub bytes: u64,
    /// SHA-256 of the file's bytes.
    pub sha256: Hash256,
}

impl ExportedFile {
    /// A file entry.
    pub fn new(name: impl Into<String>, bytes: u64, sha256: Hash256) -> Self {
        Self {
            name: name.into(),
            bytes,
            sha256,
        }
    }
}

/// Result of `session.export`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ExportResult {
    /// File written, if any.
    pub path: Option<String>,
    /// Format produced.
    pub format: crate::ExportFormat,
    /// Size in bytes.
    pub bytes: u64,
    /// SHA-256 of the exported bytes.
    pub sha256: Hash256,
    /// The bytes, base64-encoded, when `inline` was requested.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base64: Option<String>,
    /// The files written, for formats that produce several (empty for single-file formats,
    /// where `bytes` and `sha256` describe the one file).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub files: Vec<ExportedFile>,
}

impl ExportResult {
    /// Build a result without inline bytes; add them with [`with_base64`](Self::with_base64).
    pub fn new(
        path: Option<String>,
        format: crate::ExportFormat,
        bytes: u64,
        sha256: Hash256,
    ) -> Self {
        Self {
            path,
            format,
            bytes,
            sha256,
            base64: None,
            files: Vec::new(),
        }
    }

    /// Attach the per-file listing of a multi-file export.
    #[must_use]
    pub fn with_files(mut self, files: Vec<ExportedFile>) -> Self {
        self.files = files;
        self
    }

    /// Attach the base64-encoded bytes (for `inline` exports).
    #[must_use]
    pub fn with_base64(mut self, base64: impl Into<String>) -> Self {
        self.base64 = Some(base64.into());
        self
    }
}

/// Result of `script.run` on success.
///
/// On failure the server answers with an error instead (`kind` `"script-error"`), whose `data`
/// has this same shape plus `rendered` and `error_code`; see
/// [`Request::ScriptRun`](crate::Request::ScriptRun) for the exact form and for what stays applied.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct ScriptResult {
    /// State hash after the script.
    pub state: Hash256,
    /// Text the script printed.
    pub output: String,
    /// One entry per executed command, in order (shape owned by the server).
    pub events: Vec<Value>,
    /// Warnings and notes (shape owned by the server).
    pub diagnostics: Value,
}

impl ScriptResult {
    /// Build a result.
    pub fn new(
        state: Hash256,
        output: impl Into<String>,
        events: Vec<Value>,
        diagnostics: Value,
    ) -> Self {
        Self {
            state,
            output: output.into(),
            events,
            diagnostics,
        }
    }
}

/// Snapshot of where the workspace stands. Mirrors `sm_engine::Status` field for field, built
/// from contract types so this crate need not depend on `sm-engine`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct StatusResult {
    /// HEAD (serializes like a `Rev`).
    pub head: Head,
    /// Current node (`null` = empty state).
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

impl StatusResult {
    /// A status at `head` with state hash `state`; every other field starts empty/zero/false and
    /// is set with the `with_*` methods.
    pub fn new(head: Head, state: Hash256) -> Self {
        Self {
            head,
            position: None,
            depth: 0,
            state,
            nodes: 0,
            branches: BTreeMap::new(),
            redo: 0,
            dirty: false,
            project: None,
        }
    }
    /// Set the current node and its depth.
    #[must_use]
    pub fn with_position(mut self, position: Option<Hash256>, depth: usize) -> Self {
        self.position = position;
        self.depth = depth;
        self
    }
    /// Set the node count and the branch tips.
    #[must_use]
    pub fn with_branches(mut self, nodes: usize, branches: BTreeMap<String, Hash256>) -> Self {
        self.nodes = nodes;
        self.branches = branches;
        self
    }
    /// Set the number of nodes `redo` can step through.
    #[must_use]
    pub fn with_redo(mut self, redo: usize) -> Self {
        self.redo = redo;
        self
    }
    /// Set the dirty flag.
    #[must_use]
    pub fn with_dirty(mut self, dirty: bool) -> Self {
        self.dirty = dirty;
        self
    }
    /// Set the project name.
    #[must_use]
    pub fn with_project(mut self, project: Option<String>) -> Self {
        self.project = project;
        self
    }
}

/// Result of `catalog`. The op and query entries are the owned mirrors
/// ([`OpInfoOwned`], [`QueryInfoOwned`]) of `sm_op_api::OpInfo` / `QueryInfo`: they serialize to
/// exactly the same JSON (pinned by `golden/catalog.json`, `fixtures::CATALOG`) and also
/// deserialize, so remote clients (`sm-mcp`, `sm-cli`) read the reply typed. Entries carry
/// `tool_name`, the MCP-safe name; clients never recompute it.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CatalogResult {
    /// [`PROTOCOL_VERSION`](crate::PROTOCOL_VERSION) of the endpoint.
    pub protocol: u32,
    /// Registered ops.
    pub ops: Vec<OpInfoOwned>,
    /// Registered queries.
    pub queries: Vec<QueryInfoOwned>,
    /// Schema of the `core.batch` op.
    pub batch_schema: Value,
}
